mod config;
mod drive;
mod drive_report;
mod drive_view;
mod placement;
mod view;

use std::{
    collections::BTreeMap,
    io::{self, BufRead, IsTerminal, Write},
    path::{Path, PathBuf},
    process::ExitCode,
};

use clap::{Arg, ArgAction, Command, value_parser};
use vehicle_assets::ror::{
    AssetKind, ImportWarning, TerrainImport, VehicleImport, discover_entries, load_terrain,
    load_vehicle,
};

use crate::config::Config;

fn main() -> ExitCode {
    let args = Command::new("vehicle_drive")
        .about("Select terrain and vehicle definitions from Rigs of Rods ZIP archives")
        .arg(
            Arg::new("config")
                .long("config")
                .value_name("PATH")
                .value_parser(value_parser!(PathBuf))
                .default_value("config.toml")
                .help("TOML file with terrain and vehicle archive paths"),
        )
        .arg(
            Arg::new("view-terrain")
                .long("view-terrain")
                .action(ArgAction::SetTrue)
                .help("Open the selected terrain in a 3D viewer"),
        )
        .arg(
            Arg::new("view-vehicle")
                .long("view-vehicle")
                .action(ArgAction::SetTrue)
                .conflicts_with("view-terrain")
                .help("Inspect the selected vehicle on the selected terrain"),
        )
        .arg(
            Arg::new("drive")
                .long("drive")
                .action(ArgAction::SetTrue)
                .conflicts_with_all(["view-terrain", "view-vehicle", "drive-report"])
                .help("Drive the selected vehicle on imported collision geometry"),
        )
        .arg(
            Arg::new("drive-report")
                .long("drive-report")
                .action(ArgAction::SetTrue)
                .conflicts_with_all(["view-terrain", "view-vehicle"])
                .help("Run the imported driving checks without opening a window"),
        )
        .get_matches();
    let path = args
        .get_one::<PathBuf>("config")
        .expect("config has a default");

    let report = args.get_flag("drive-report");
    if args.get_flag("drive") || report {
        return match start_drive(path, report) {
            Ok(true) => ExitCode::SUCCESS,
            Ok(false) => ExitCode::FAILURE,
            Err(error) => {
                eprintln!("{error}");
                ExitCode::FAILURE
            }
        };
    }

    let result = {
        let stdin = io::stdin();
        let stderr = io::stderr();
        let mut input = (stdin.is_terminal() && stderr.is_terminal()).then(|| stdin.lock());
        let mut diagnostics = stderr.lock();
        let show_vehicle = args.get_flag("view-vehicle");
        if show_vehicle || args.get_flag("view-terrain") {
            Config::read(path).and_then(|config| {
                let (terrain, _, vehicle) = import_scene(
                    path,
                    &config,
                    show_vehicle,
                    input.as_mut(),
                    &mut diagnostics,
                )?;
                let vehicle = vehicle
                    .map(|import| {
                        let translation =
                            placement::park(&terrain, &import.vehicle).map_err(|error| {
                                format!(
                                    "cannot park {} in {}: {error}",
                                    import.vehicle.name, terrain.terrain.name
                                )
                            })?;
                        Ok::<_, String>(view::ParkedVehicle {
                            import,
                            translation,
                        })
                    })
                    .transpose()?;
                Ok(Some(view::Scene { terrain, vehicle }))
            })
        } else {
            run(
                path,
                input.as_mut(),
                &mut io::stdout().lock(),
                &mut diagnostics,
            )
            .map(|()| None)
        }
    };
    match result {
        Ok(Some(scene)) => {
            print_import_summary(&scene.terrain);
            if let Some(vehicle) = &scene.vehicle {
                print_vehicle_summary(&vehicle.import);
            }
            if view::run(scene) {
                ExitCode::SUCCESS
            } else {
                ExitCode::FAILURE
            }
        }
        Ok(None) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}

fn import_scene(
    path: &Path,
    config: &Config,
    show_vehicle: bool,
    mut input: Option<&mut impl BufRead>,
    diagnostics: &mut impl Write,
) -> Result<(TerrainImport, String, Option<VehicleImport>), String> {
    let vehicle_archive =
        if show_vehicle {
            Some(
                config.assets.vehicle.as_deref().ok_or_else(|| {
                    format!("config {} is missing [assets].vehicle", path.display())
                })?,
            )
        } else {
            None
        };
    let archive = &config.assets.terrain;
    let selected = choose_asset(
        archive,
        AssetKind::Terrain,
        "terrain",
        config.assets.terrain_entry.as_deref(),
        input.as_deref_mut(),
        diagnostics,
    )?;
    let terrain = load_terrain(archive, &selected).map_err(|error| error.to_string())?;
    let vehicle = if let Some(archive) = vehicle_archive {
        let selected = choose_asset(
            archive,
            AssetKind::Vehicle,
            "vehicle",
            config.assets.vehicle_entry.as_deref(),
            input,
            diagnostics,
        )?;
        Some(load_vehicle(archive, &selected).map_err(|error| error.to_string())?)
    } else {
        None
    };
    Ok((terrain, selected, vehicle))
}

fn start_drive(path: &Path, report: bool) -> Result<bool, String> {
    let mut config = Config::read(path)?;
    let ramp = if report {
        config
            .drive
            .take()
            .and_then(|drive| drive.report_ramp)
            .map(config::RampApproach::validate)
            .transpose()
            .map_err(|error| format!("config {}: {error}", path.display()))?
    } else {
        None
    };
    let (terrain, terrain_entry, vehicle) = {
        let stdin = io::stdin();
        let stderr = io::stderr();
        let mut input = (stdin.is_terminal() && stderr.is_terminal()).then(|| stdin.lock());
        import_scene(path, &config, true, input.as_mut(), &mut stderr.lock())?
    };
    let vehicle = vehicle.expect("drive imports a vehicle");
    print_import_summary(&terrain);
    print_vehicle_summary(&vehicle);
    let setup = drive::DriveSetup::new(ramp, &terrain, &vehicle)
        .map_err(|error| format!("config {}: {terrain_entry}: {error}", path.display()))?;
    setup.print_setup(&vehicle.entry, &terrain_entry);
    let session = drive::DriveSession::new(setup)?;
    if report {
        drive_report::run(session)?;
        Ok(true)
    } else {
        Ok(drive_view::run(session, terrain, vehicle))
    }
}

fn choose_asset(
    archive: &Path,
    kind: AssetKind,
    field: &str,
    requested: Option<&str>,
    input: Option<&mut impl BufRead>,
    diagnostics: &mut impl Write,
) -> Result<String, String> {
    let entries =
        discover_entries(archive, kind).map_err(|error| format!("[assets].{field}: {error}"))?;
    select_entry(&entries, requested, field, archive, input, diagnostics).map(str::to_owned)
}

fn print_vehicle_summary(import: &VehicleImport) {
    let vehicle = &import.vehicle;
    println!("Vehicle: {} ({})", vehicle.name, import.entry);
    println!(
        "Structure: {} nodes, {} beams, {} wheels",
        vehicle.nodes.len(),
        vehicle.edges.len(),
        vehicle.wheels.len()
    );
    for (index, wheel) in vehicle.wheels.iter().enumerate() {
        let position = wheel.centre;
        println!(
            "Wheel {}: centre ({:.3}, {:.3}, {:.3}) m, radius {:.3} m, rim {:.3} m, width {:.3} m",
            index + 1,
            position.x,
            position.y,
            position.z,
            wheel.outer_radius,
            wheel.rim_radius.unwrap_or_default(),
            wheel.width,
        );
    }
    let _ = io::stdout().flush();
    print_warnings(&import.warnings);
}

fn print_import_summary(import: &TerrainImport) {
    let terrain = &import.terrain;
    println!("Terrain: {}", terrain.name);
    if let Some(grid) = &terrain.heightfield {
        println!(
            "Heightfield: {} x {} samples",
            grid.samples_x, grid.samples_z
        );
    }
    let collision = terrain
        .objects
        .iter()
        .filter(|object| object.collision_mesh.is_some())
        .count();
    println!(
        "Objects: {} placements, {collision} with collision meshes",
        terrain.objects.len()
    );
    let _ = io::stdout().flush();

    print_warnings(&import.warnings);
}

fn print_warnings(diagnostics: &[ImportWarning]) {
    let mut warnings: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
    for warning in diagnostics {
        warnings
            .entry(&warning.message)
            .or_default()
            .push(&warning.entry);
    }
    for (message, entries) in warnings {
        eprintln!("Warning: {message} ({})", entries.join(", "));
    }
}

fn run(
    path: &Path,
    mut input: Option<&mut impl BufRead>,
    output: &mut impl Write,
    diagnostics: &mut impl Write,
) -> Result<(), String> {
    let config = Config::read(path)?;
    let assets =
        [
            (
                AssetKind::Terrain,
                "terrain",
                &config.assets.terrain,
                config.assets.terrain_entry.as_deref(),
            ),
            (
                AssetKind::Vehicle,
                "vehicle",
                config.assets.vehicle.as_ref().ok_or_else(|| {
                    format!("config {} is missing [assets].vehicle", path.display())
                })?,
                config.assets.vehicle_entry.as_deref(),
            ),
        ];
    let mut selections = Vec::with_capacity(assets.len());

    for (kind, field, archive, requested) in assets {
        let selected = choose_asset(
            archive,
            kind,
            field,
            requested,
            input.as_deref_mut(),
            diagnostics,
        )?;
        selections.push((field, archive, selected));
    }

    for (field, archive, selected) in selections {
        writeln!(output, "{field}: {} -> {selected:?}", archive.display())
            .map_err(|error| format!("cannot write selection summary: {error}"))?;
    }
    Ok(())
}

fn select_entry<'a>(
    entries: &'a [String],
    requested: Option<&str>,
    field: &str,
    archive: &Path,
    input: Option<&mut impl BufRead>,
    diagnostics: &mut impl Write,
) -> Result<&'a str, String> {
    if let Some(requested) = requested {
        return entries.iter().find(|entry| entry.as_str() == requested).map(String::as_str)
            .ok_or_else(|| {
                let available = if entries.is_empty() {
                    "no eligible definitions".into()
                } else {
                    format!("available entries: {entries:?}")
                };
                format!("[assets].{field}_entry {requested:?} is not an eligible entry in {} ({available})", archive.display())
            });
    }

    match entries {
        [] => {
            return Err(format!(
                "[assets].{field}: no eligible {field} definitions in {}",
                archive.display()
            ));
        }
        [only] => return Ok(only),
        _ => {}
    }

    let Some(input) = input else {
        return Err(format!(
            "[assets].{field}: multiple definitions in {}. Set {field}_entry in [assets] to one of {entries:?}",
            archive.display()
        ));
    };

    choose_from_menu(entries, field, archive, input, diagnostics)
        .map_err(|error| {
            format!(
                "[assets].{field}: cannot select an entry in {}: {error}",
                archive.display()
            )
        })?
        .ok_or_else(|| {
            format!(
                "[assets].{field}: input closed. Set {field}_entry in [assets] to select directly"
            )
        })
}

fn choose_from_menu<'a>(
    entries: &'a [String],
    field: &str,
    archive: &Path,
    input: &mut impl BufRead,
    diagnostics: &mut impl Write,
) -> io::Result<Option<&'a str>> {
    writeln!(diagnostics, "Select {field} from {}", archive.display())?;
    for (index, entry) in entries.iter().enumerate() {
        writeln!(diagnostics, "  {}. {entry:?}", index + 1)?;
    }

    let mut line = String::new();
    loop {
        write!(diagnostics, "Enter a number (1-{}): ", entries.len())?;
        diagnostics.flush()?;
        line.clear();
        if input.read_line(&mut line)? == 0 {
            return Ok(None);
        }
        if let Ok(number) = line.trim().parse::<usize>()
            && (1..=entries.len()).contains(&number)
        {
            return Ok(Some(&entries[number - 1]));
        }
        writeln!(diagnostics, "Choose one of the listed numbers.")?;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn menu_retries_and_returns_the_same_entry_as_explicit_config() {
        let entries = ["accord.truck".to_owned(), "accordb.truck".to_owned()];
        let archive = Path::new("accord23.zip");
        let mut diagnostics = Vec::new();

        let interactive = select_entry(
            &entries,
            None,
            "vehicle",
            archive,
            Some(&mut &b"0\ncar\n3\n1\n"[..]),
            &mut diagnostics,
        )
        .unwrap();
        let explicit = select_entry(
            &entries,
            Some("accord.truck"),
            "vehicle",
            archive,
            None::<&mut &[u8]>,
            &mut Vec::new(),
        )
        .unwrap();
        assert_eq!(interactive, explicit);
        let menu = String::from_utf8(diagnostics).unwrap();
        assert!(menu.contains("1. \"accord.truck\""));
        assert!(menu.contains("2. \"accordb.truck\""));
        assert_eq!(menu.matches("Choose one of the listed numbers.").count(), 3);
    }

    #[test]
    fn closed_input_exits_with_the_config_key() {
        let entries = ["accord.truck".to_owned(), "accordb.truck".to_owned()];
        let error = select_entry(
            &entries,
            None,
            "vehicle",
            Path::new("accord23.zip"),
            Some(&mut &b""[..]),
            &mut Vec::new(),
        )
        .unwrap_err();
        assert!(error.contains("input closed"));
        assert!(error.contains("vehicle_entry"));
    }
}

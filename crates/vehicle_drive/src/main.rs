mod config;
mod crash_replay;
mod drive_view;
mod view;

use std::{collections::BTreeMap, path::PathBuf, process::ExitCode};

use clap::{Arg, ArgAction, Command, value_parser};
use vehicle_assets::ror::{ImportWarning, TerrainImport, VehicleImport, reference};
use vehicle_scene::{
    drive::{AlignmentMode, DriveSetup},
    placement,
};

fn main() -> ExitCode {
    let args = Command::new("vehicle_drive")
        .about("Drive or inspect the bundled reference assets or configured Rigs of Rods archives")
        .arg(
            Arg::new("config")
                .long("config")
                .value_name("PATH")
                .value_parser(value_parser!(PathBuf))
                .help("Read terrain and vehicle ZIP paths for manual driving or inspection"),
        )
        .arg(
            Arg::new("drive")
                .long("drive")
                .action(ArgAction::SetTrue)
                .conflicts_with_all(["view-terrain", "view-vehicle", "crash-replay"])
                .help("Drive the selected car (default)"),
        )
        .arg(
            Arg::new("benchmark")
                .long("benchmark")
                .action(ArgAction::SetTrue)
                .conflicts_with_all(["view-terrain", "view-vehicle", "crash-replay", "config"])
                .help("Measure scripted driving and a frontal crash in the window"),
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
                .help("Inspect the selected car parked on the terrain"),
        )
        .arg(
            Arg::new("crash-replay")
                .long("crash-replay")
                .action(ArgAction::SetTrue)
                .conflicts_with_all(["view-terrain", "view-vehicle", "config"])
                .help("Inspect the textured reference car's recorded crash"),
        )
        .get_matches();

    match run(
        args.get_one::<PathBuf>("config").map(PathBuf::as_path),
        args.get_flag("view-terrain"),
        args.get_flag("view-vehicle"),
        args.get_flag("crash-replay"),
        args.get_flag("benchmark"),
    ) {
        Ok(true) => ExitCode::SUCCESS,
        Ok(false) => ExitCode::FAILURE,
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}

fn run(
    config_path: Option<&std::path::Path>,
    view_terrain: bool,
    view_vehicle: bool,
    crash_replay: bool,
    benchmark: bool,
) -> Result<bool, String> {
    let config = config_path.map(config::Config::read).transpose()?;
    let terrain = match &config {
        Some(config) => config.terrain()?,
        None => reference::terrain().map_err(|error| error.to_string())?,
    };
    print_import_summary(&terrain);
    if view_terrain {
        return Ok(view::run(view::Scene {
            terrain,
            vehicle: None,
        }));
    }

    let vehicle = match &config {
        Some(config) => config.vehicle()?,
        None => reference::vehicle().map_err(|error| error.to_string())?,
    };
    print_vehicle_summary(&vehicle);
    if view_vehicle {
        let translation = placement::park(&terrain, &vehicle.vehicle)?;
        let vehicle = Some(view::ParkedVehicle {
            import: vehicle,
            translation,
        });
        return Ok(view::run(view::Scene { terrain, vehicle }));
    }

    let setup = DriveSetup::new(&terrain, &vehicle, AlignmentMode::Authored)?;
    if crash_replay {
        return crash_replay::run(setup, terrain, vehicle);
    }
    let mode = if config.is_some() {
        drive_view::DriveMode::Configured
    } else if benchmark {
        drive_view::DriveMode::Benchmark
    } else {
        drive_view::DriveMode::Reference
    };
    drive_view::run(setup, terrain, vehicle, mode)
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

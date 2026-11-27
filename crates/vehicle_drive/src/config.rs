use std::{
    fs,
    io::{self, BufRead, IsTerminal, Write},
    path::{Path, PathBuf},
};

use serde::Deserialize;
use vehicle_assets::ror::{
    AssetKind, TerrainImport, VehicleImport, discover_entries, load_terrain, load_vehicle,
};

pub(crate) struct Config {
    path: PathBuf,
    assets: Assets,
}

#[derive(Deserialize)]
struct ConfigFile {
    assets: Assets,
}

#[derive(Deserialize)]
struct Assets {
    terrain: PathBuf,
    terrain_entry: Option<String>,
    vehicle: Option<PathBuf>,
    vehicle_entry: Option<String>,
}

impl Config {
    pub(crate) fn read(path: &Path) -> Result<Self, String> {
        let text = fs::read_to_string(path)
            .map_err(|error| format!("cannot read config {}: {error}", path.display()))?;
        let ConfigFile { mut assets } = toml::from_str(&text)
            .map_err(|error| format!("cannot parse config {}: {error}", path.display()))?;
        let directory = path.parent().unwrap_or_else(|| Path::new("."));
        for (key, archive) in std::iter::once(("terrain", &mut assets.terrain))
            .chain(assets.vehicle.as_mut().map(|path| ("vehicle", path)))
        {
            if archive.as_os_str().is_empty() {
                return Err(format!(
                    "config {}: [assets].{key} needs a ZIP path",
                    path.display()
                ));
            }
            if archive.is_relative() {
                *archive = directory.join(&archive);
            }
        }
        Ok(Self {
            path: path.to_owned(),
            assets,
        })
    }

    pub(crate) fn terrain(&self) -> Result<TerrainImport, String> {
        let archive = &self.assets.terrain;
        let entry = select(
            archive,
            AssetKind::Terrain,
            "terrain",
            self.assets.terrain_entry.as_deref(),
        )?;
        println!("Terrain archive: {} ({entry})", archive.display());
        load_terrain(archive, &entry).map_err(|error| error.to_string())
    }

    pub(crate) fn vehicle(&self) -> Result<VehicleImport, String> {
        let archive =
            self.assets.vehicle.as_deref().ok_or_else(|| {
                format!("config {} is missing [assets].vehicle", self.path.display())
            })?;
        let entry = select(
            archive,
            AssetKind::Vehicle,
            "vehicle",
            self.assets.vehicle_entry.as_deref(),
        )?;
        println!("Vehicle archive: {} ({entry})", archive.display());
        load_vehicle(archive, &entry).map_err(|error| error.to_string())
    }
}

fn select(
    path: &Path,
    kind: AssetKind,
    key: &str,
    requested: Option<&str>,
) -> Result<String, String> {
    let entries =
        discover_entries(path, kind).map_err(|error| format!("[assets].{key}: {error}"))?;
    let name = path.display();
    if entries.is_empty() {
        return Err(format!("[assets].{key}: no eligible definitions in {name}"));
    }
    if let Some(entry) = requested {
        return if entries.iter().any(|candidate| candidate == entry) {
            Ok(entry.to_owned())
        } else {
            Err(format!(
                "[assets].{key}_entry {entry:?} is not in {name}. Available entries: {entries:?}"
            ))
        };
    }
    if let [entry] = entries.as_slice() {
        return Ok(entry.clone());
    }
    let stdin = io::stdin();
    let stderr = io::stderr();
    let help = format!("Set [assets].{key}_entry to one of {entries:?}");
    if !stdin.is_terminal() || !stderr.is_terminal() {
        return Err(format!(
            "[assets].{key}: multiple definitions in {name}. {help}"
        ));
    }
    choose_entry(&entries, &mut stdin.lock(), &mut stderr.lock())
        .map_err(|error| format!("[assets].{key}: cannot select an entry: {error}"))?
        .map(str::to_owned)
        .ok_or_else(|| format!("[assets].{key}: input closed. {help}"))
}

fn choose_entry<'a>(
    entries: &'a [String],
    input: &mut impl BufRead,
    output: &mut impl Write,
) -> io::Result<Option<&'a str>> {
    for (index, entry) in entries.iter().enumerate() {
        writeln!(output, "  {}. {entry}", index + 1)?;
    }
    let mut line = String::new();
    loop {
        write!(output, "Select an entry (1–{}): ", entries.len())?;
        output.flush()?;
        line.clear();
        if input.read_line(&mut line)? == 0 {
            return Ok(None);
        }
        if let Ok(index) = line.trim().parse::<usize>()
            && let Some(entry) = index.checked_sub(1).and_then(|i| entries.get(i))
        {
            return Ok(Some(entry));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resolves_paths_and_selects_entries_from_the_project_archives() {
        let directory = tempfile::tempdir().unwrap();
        let path = directory.path().join("test.toml");
        fs::write(
            directory.path().join("track.zip"),
            include_bytes!("../../vehicle_assets/reference/track.zip"),
        )
        .unwrap();
        fs::write(
            directory.path().join("car.zip"),
            include_bytes!("../../vehicle_assets/reference/car.zip"),
        )
        .unwrap();

        let terrain = "[assets]\nterrain = 'track.zip'\n";
        fs::write(&path, terrain).unwrap();
        let config = Config::read(&path).unwrap();
        assert_eq!(
            config.terrain().unwrap().terrain.name,
            "Reference Test Track"
        );
        assert!(config.vehicle().unwrap_err().contains("[assets].vehicle"));

        let both = format!("{terrain}vehicle = 'car.zip'\n");
        let displaced = format!("{both}terrain_entry = 'missing.terrn2'\n");
        fs::write(&path, displaced).unwrap();
        assert!(
            Config::read(&path)
                .unwrap()
                .terrain()
                .unwrap_err()
                .contains("track.terrn2")
        );
        fs::write(
            &path,
            format!(
                "{both}vehicle_entry = 'car-wide.truck'\n[drive.report_ramp]\nobject = 'unused'\n"
            ),
        )
        .unwrap();
        assert_eq!(
            Config::read(&path).unwrap().vehicle().unwrap().entry,
            "car-wide.truck"
        );
    }

    #[test]
    fn menu_retries_invalid_choices_and_handles_closed_input() {
        let entries = ["car-wide.truck".into(), "car.truck".into()];
        let mut output = Vec::new();
        assert_eq!(
            choose_entry(&entries, &mut &b"0\nword\n2\n"[..], &mut output).unwrap(),
            Some("car.truck")
        );
        assert_eq!(
            choose_entry(&entries, &mut &b""[..], &mut output).unwrap(),
            None
        );
    }
}

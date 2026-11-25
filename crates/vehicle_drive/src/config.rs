use std::{
    fs, io,
    path::{Path, PathBuf},
};

use serde::Deserialize;

#[derive(Deserialize)]
pub(crate) struct Config {
    pub(crate) assets: Assets,
}

#[derive(Deserialize)]
pub(crate) struct Assets {
    pub(crate) terrain: PathBuf,
    pub(crate) vehicle: PathBuf,
    pub(crate) terrain_entry: Option<String>,
    pub(crate) vehicle_entry: Option<String>,
}

impl Config {
    pub(crate) fn read(path: &Path) -> Result<Self, String> {
        let text = fs::read_to_string(path).map_err(|error| {
            let mut message = format!("cannot read config {}: {error}", path.display());
            if error.kind() == io::ErrorKind::NotFound {
                message.push_str(". Copy config.example.toml and set the paths in [assets]");
            }
            message
        })?;
        let mut config: Self = toml::from_str(&text)
            .map_err(|error| format!("cannot parse config {}: {error}", path.display()))?;
        let dir = path.parent().unwrap_or_else(|| Path::new("."));
        for (field, archive) in [
            ("terrain", &mut config.assets.terrain),
            ("vehicle", &mut config.assets.vehicle),
        ] {
            if archive.as_os_str().is_empty() {
                return Err(format!(
                    "config {}: [assets].{field} must be a ZIP path",
                    path.display()
                ));
            }
            if archive.is_relative() {
                *archive = dir.join(&archive);
            }
        }
        Ok(config)
    }
}

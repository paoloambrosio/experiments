use std::{
    fs, io,
    path::{Path, PathBuf},
};

use serde::Deserialize;

#[derive(Deserialize)]
pub(crate) struct Config {
    pub(crate) assets: Assets,
    pub(crate) drive: Option<DriveConfig>,
}

#[derive(Deserialize)]
pub(crate) struct Assets {
    pub(crate) terrain: PathBuf,
    pub(crate) vehicle: Option<PathBuf>,
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
        let archives = std::iter::once(("terrain", &mut config.assets.terrain)).chain(
            config
                .assets
                .vehicle
                .as_mut()
                .map(|vehicle| ("vehicle", vehicle)),
        );
        for (field, archive) in archives {
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

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct DriveConfig {
    pub report_ramp: Option<RampApproach>,
}

#[derive(Clone, Deserialize, Debug)]
#[serde(deny_unknown_fields)]
pub(crate) struct RampApproach {
    pub object: String,
    pub start_m: [f32; 3],
    pub heading_deg: f32,
}

impl RampApproach {
    pub(crate) fn validate(self) -> Result<Self, String> {
        if self.object.trim().is_empty() {
            return Err("invalid [drive.report_ramp].object".into());
        }
        if !self.start_m.into_iter().all(f32::is_finite) {
            return Err("invalid [drive.report_ramp].start_m".into());
        }
        if !self.heading_deg.is_finite() {
            return Err("invalid [drive.report_ramp].heading_deg".into());
        }
        Ok(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drive_accepts_assets_only_and_an_optional_ramp() {
        let example = include_str!("../../../config.example.toml");
        let config = toml::from_str::<Config>(example).unwrap();
        assert!(config.drive.is_none());
        let ramp = format!(
            "{example}\n[drive.report_ramp]\nobject = 'Ramp'\nstart_m = [1.0, 2.0, 3.0]\nheading_deg = 0.0\n"
        );
        let config = toml::from_str::<Config>(&ramp).unwrap();
        assert_eq!(
            config
                .drive
                .unwrap()
                .report_ramp
                .unwrap()
                .validate()
                .unwrap()
                .object,
            "Ramp"
        );
    }
}

//! Project-authored RoR assets embedded for driving and reproducible experiments.

use std::path::Path;

use super::{ImportError, TerrainImport, VehicleImport, terrain, vehicle};

const TRACK: &[u8] = include_bytes!("../../reference/track.zip");
const CAR: &[u8] = include_bytes!("../../reference/car.zip");

/// Imports the committed track, including its ramp and collision wall.
pub fn terrain() -> Result<TerrainImport, ImportError> {
    terrain::load_embedded(Path::new("track.zip"), TRACK, "track.terrn2")
}

/// Imports the base car with its authored alignment and deformable structure.
pub fn vehicle() -> Result<VehicleImport, ImportError> {
    vehicle::load_embedded(Path::new("car.zip"), CAR, "car.truck")
}

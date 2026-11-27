use vehicle_assets::ror::reference;
use vehicle_scene::drive::{AlignmentMode, DriveSession, DriveSetup};

use crate::{crash_report, drive_report};

pub(crate) fn drive() -> Result<(), Box<dyn Error>> {
    let terrain = reference::terrain()?;
    let vehicle = reference::vehicle()?;
    let setup = DriveSetup::new(&terrain, &vehicle, AlignmentMode::Authored)?;
    drive_report::run(DriveSession::new(setup)?)?;
    Ok(())
}

pub(crate) fn alignment() -> Result<(), Box<dyn Error>> {
    let terrain = reference::terrain()?;
    let vehicle = reference::vehicle()?;
    let upright = DriveSetup::new(&terrain, &vehicle, AlignmentMode::Upright)?;
    let authored = DriveSetup::new(&terrain, &vehicle, AlignmentMode::Authored)?;
    drive_report::compare_alignment(upright, authored)?;
    Ok(())
}

pub(crate) fn crash() -> Result<(), Box<dyn Error>> {
    let terrain = reference::terrain()?;
    let vehicle = reference::vehicle()?;
    let setup = DriveSetup::new(&terrain, &vehicle, AlignmentMode::Authored)?;
    crash_report::run(&setup, &vehicle.vehicle, "wall")?;
    Ok(())
}
use std::error::Error;

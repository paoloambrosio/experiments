use bevy::math::DVec3;
use vehicle_physics::{
    elastic_tyre::{ElasticTyre, ElasticTyreConfig, ElasticTyreError},
    mounted_tyre::{FlatFloor, MountedStep, MountedTyre, RimMotion},
    tread_reinforcement::TreadReinforcementConfig,
};

use crate::{drop_experiment, elastic_experiment::Experiment};

pub const G: f64 = 9.81;
pub const NOMINAL_LOAD_N: f64 = 250.0 * G;
pub const RATES: [u32; 2] = [3840, 7680];
pub const FRICTION: f64 = 0.8;
pub const PRESSURE_PA: f64 = 150_000.0;
pub const BARE_RIM_KG: f64 = 9.0;
pub const UPRIGHT_KG: f32 = 3.0;
pub const BEAD_RADIUS_M: f64 = 0.20;

pub fn floor(light_car: bool) -> FlatFloor {
    let floor = if light_car {
        FlatFloor::new(0.0, 500_000.0, 150.0).unwrap()
    } else {
        drop_experiment::bench_floor()
    };
    floor.with_friction(FRICTION, 0.1).unwrap()
}

pub fn preparation(rate: u32, light_car: bool) -> Experiment {
    if !light_car {
        return drop_experiment::prepare_wheel(rate, 30_000.0);
    }
    let mut material = ElasticTyreConfig::default();
    material.circumferential.stiffness *= 2.5;
    material.cross_section.stiffness *= 4.0;
    material.diagonal.stiffness *= 4.0;
    material.circumferential.damping *= 3.0;
    material.cross_section.damping *= 3.0;
    material.diagonal.damping *= 3.0;
    let mut reinforcement = TreadReinforcementConfig::default();
    reinforcement.hoop_stiffness_multiplier = 16.0;
    reinforcement.crown.stiffness *= 4.0;
    Experiment::reinforced(rate, PRESSURE_PA, material, reinforcement)
}

pub fn prepare(rate: u32, light_car: bool) -> Result<ElasticTyre, Box<dyn std::error::Error>> {
    let mut experiment = preparation(rate, light_car);
    while !experiment.is_ready() {
        experiment.prepare_step()?;
    }
    Ok(experiment.tyre)
}

pub fn unloaded_radius(tyre: &ElasticTyre) -> f64 {
    -tyre
        .positions()
        .iter()
        .map(|p| p.y)
        .fold(f64::INFINITY, f64::min)
}

/// Static load fixture. The tyre resolves bead forces while the fixture guides the rim
/// along Y and keeps its orientation fixed.
pub struct LoadWheel {
    pub tyre: MountedTyre,
    pub last: MountedStep,
    pub radius_m: f64,
    pub target_load_n: f64,
    mass_kg: f64,
}

impl LoadWheel {
    pub fn new(tyre: ElasticTyre, light_car: bool) -> Result<Self, ElasticTyreError> {
        let radius_m = unloaded_radius(&tyre);
        let motion = RimMotion {
            centre: DVec3::Y * radius_m,
            ..Default::default()
        };
        let tyre = MountedTyre::new(tyre, motion, Some(floor(light_car)))?;
        let (mass_kg, _) = tyre.rim_mass_properties(drop_experiment::RIM_MASS_KG)?;
        Ok(Self {
            tyre,
            last: MountedStep::default(),
            radius_m,
            target_load_n: 20.0 * G,
            mass_kg,
        })
    }

    pub fn step(&mut self, dt: f64) -> Result<(), ElasticTyreError> {
        let mut rim = self.tyre.rim();
        self.last = self.tyre.step(dt, DVec3::NEG_Y * G)?;
        let free_node_mass_kg = 20.0 - self.mass_kg;
        let downward_n = self.target_load_n - free_node_mass_kg * G;
        rim.linear_velocity.y += (self.last.rim_force.y - downward_n) / self.mass_kg * dt;
        rim.centre.y += rim.linear_velocity.y * dt;
        self.tyre.sync_rim(rim)
    }

    pub fn static_reading(&self) -> StaticReading {
        let height_m = self.tyre.rim().centre.y;
        let penetration_m = self.tyre.floor_penetration_m();
        let patch = self.tyre.footprint();
        StaticReading {
            radius_m: self.radius_m,
            height_m,
            deflection_m: self.radius_m - height_m - penetration_m,
            penetration_m,
            clearance_m: height_m - BEAD_RADIUS_M,
            patch_length_m: patch.as_ref().map_or(0.0, |patch| patch.length_m),
            patch_width_m: patch.as_ref().map_or(0.0, |patch| patch.width_m),
            pressure_pa: self.tyre.chamber_reading().unwrap().gauge_pressure_pa,
        }
    }
}

pub struct StaticReading {
    pub radius_m: f64,
    pub height_m: f64,
    pub deflection_m: f64,
    pub penetration_m: f64,
    pub clearance_m: f64,
    pub patch_length_m: f64,
    pub patch_width_m: f64,
    pub pressure_pa: f64,
}

pub fn settle_load(wheel: &mut LoadWheel, rate: u32, target: f64) -> Result<f64, String> {
    let dt = 1.0 / f64::from(rate);
    let initial = wheel.target_load_n;
    let mut quiet = 0;
    let mut sum = 0.0;
    for step in 1..=rate * 10 {
        wheel.target_load_n = initial + (target - initial) * f64::from(step.min(rate)) * dt;
        wheel
            .step(dt)
            .map_err(|error| format!("{target:.0} N: {error}"))?;
        if step >= rate
            && wheel.tyre.rim().linear_velocity.y.abs() < 0.005
            && (wheel.last.floor_force_n - target).abs() < 0.02 * target
        {
            quiet += 1;
            sum += wheel.last.floor_force_n;
            if quiet == rate / 2 {
                return Ok(sum / f64::from(quiet));
            }
        } else {
            quiet = 0;
            sum = 0.0;
        }
    }
    Err(format!(
        "{target:.0} N: did not settle (height {:.4} m, support {:.0} N)",
        wheel.tyre.rim().centre.y,
        wheel.last.floor_force_n
    ))
}

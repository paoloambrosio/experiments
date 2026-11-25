use bevy::math::{DQuat, DVec3};
use vehicle_physics::{
    elastic_tyre::{ElasticTyre, ElasticTyreError},
    mounted_tyre::{FlatFloor, MountedStep, MountedTyre, RimMotion},
    reference_car,
};

use crate::drop_experiment;

pub use reference_car::{
    BARE_RIM_KG, BEAD_RADIUS_M, FRICTION, G, NOMINAL_LOAD_N, UPRIGHT_KG, unloaded_radius,
};
pub const RATES: [u32; 2] = [3840, 7680];
pub fn floor(light_car: bool) -> FlatFloor {
    if light_car {
        reference_car::road()
    } else {
        drop_experiment::bench_floor()
            .with_friction(FRICTION, 0.1)
            .unwrap()
    }
}
pub fn prepare(rate: u32, light_car: bool) -> Result<ElasticTyre, Box<dyn std::error::Error>> {
    if light_car {
        let mut reference = reference_car::ReferenceTyre::new(rate);
        while !reference.advance()? {}
        return Ok(reference.tyre);
    }
    let mut experiment = drop_experiment::prepare_wheel(rate, 30_000.0);
    while !experiment.is_ready() {
        experiment.prepare_step()?;
    }
    Ok(experiment.tyre)
}

/// The fixture guides the rim along Y and allows spin about X.
#[derive(Clone)]
pub struct LoadWheel {
    pub tyre: MountedTyre,
    pub last: MountedStep,
    pub radius_m: f64,
    pub target_load_n: f64,
    guided_mass_kg: f64,
    spin_inertia_kg_m2: f64,
}

impl LoadWheel {
    pub fn new(tyre: ElasticTyre, light_car: bool) -> Result<Self, ElasticTyreError> {
        Self::mount(tyre, light_car, drop_experiment::RIM_MASS_KG, 0.0)
    }

    pub fn rolling(tyre: ElasticTyre) -> Result<Self, ElasticTyreError> {
        Self::mount(tyre, true, BARE_RIM_KG, f64::from(UPRIGHT_KG))
    }

    fn mount(
        tyre: ElasticTyre,
        light_car: bool,
        bare_rim_kg: f64,
        hub_kg: f64,
    ) -> Result<Self, ElasticTyreError> {
        let radius_m = unloaded_radius(&tyre);
        let motion = RimMotion {
            centre: DVec3::Y * radius_m,
            ..Default::default()
        };
        let tyre = MountedTyre::new(tyre, motion, Some(floor(light_car)))?;
        let (rim_kg, inertia) = tyre.rim_mass_properties(bare_rim_kg)?;
        Ok(Self {
            tyre,
            last: MountedStep::default(),
            radius_m,
            target_load_n: 20.0 * G,
            guided_mass_kg: rim_kg + hub_kg,
            spin_inertia_kg_m2: inertia.x,
        })
    }

    pub fn step(&mut self, dt: f64) -> Result<(), ElasticTyreError> {
        self.step_with_spin(dt, Some(0.0))
    }

    /// Prescribe a spin in rad/s or let the bead torque accelerate the rim.
    pub fn step_with_spin(&mut self, dt: f64, spin: Option<f64>) -> Result<(), ElasticTyreError> {
        let mut rim = self.tyre.rim();
        if let Some(spin) = spin {
            rim.angular_velocity.x = spin;
            self.tyre.sync_rim(rim)?;
        }
        self.last = self.tyre.step(dt, DVec3::NEG_Y * G)?;
        let free_node_mass_kg = 20.0 - self.guided_mass_kg;
        let downward_n = self.target_load_n - free_node_mass_kg * G;
        rim.linear_velocity.y += (self.last.rim_force.y - downward_n) / self.guided_mass_kg * dt;
        rim.centre.y += rim.linear_velocity.y * dt;
        if spin.is_none() {
            rim.angular_velocity.x += self.last.rim_torque.x / self.spin_inertia_kg_m2 * dt;
        }
        if rim.angular_velocity.x != 0.0 {
            rim.rotation =
                (DQuat::from_rotation_x(rim.angular_velocity.x * dt) * rim.rotation).normalize();
        }
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
    let mut settling = LoadSettling::new(wheel, rate, target);
    loop {
        if let Some(support) = settling.advance(wheel)? {
            return Ok(support);
        }
    }
}

pub struct LoadSettling {
    initial: f64,
    target: f64,
    rate: u32,
    step: u32,
    quiet: u32,
    sum: f64,
}

impl LoadSettling {
    pub fn new(wheel: &LoadWheel, rate: u32, target: f64) -> Self {
        Self {
            initial: wheel.target_load_n,
            target,
            rate,
            step: 0,
            quiet: 0,
            sum: 0.0,
        }
    }

    pub fn advance(&mut self, wheel: &mut LoadWheel) -> Result<Option<f64>, String> {
        self.step += 1;
        let dt = 1.0 / f64::from(self.rate);
        let target = self.target;
        wheel.target_load_n =
            self.initial + (target - self.initial) * f64::from(self.step.min(self.rate)) * dt;
        wheel
            .step(dt)
            .map_err(|error| format!("{target:.0} N: {error}"))?;
        if self.step >= self.rate
            && wheel.tyre.rim().linear_velocity.y.abs() < 0.005
            && (wheel.last.floor_force_n - target).abs() < 0.02 * target
        {
            self.quiet += 1;
            self.sum += wheel.last.floor_force_n;
            if self.quiet == self.rate / 2 {
                return Ok(Some(self.sum / f64::from(self.quiet)));
            }
        } else {
            self.quiet = 0;
            self.sum = 0.0;
        }
        if self.step >= self.rate * 10 {
            return Err(format!(
                "{target:.0} N: did not settle (height {:.4} m, support {:.0} N)",
                wheel.tyre.rim().centre.y,
                wheel.last.floor_force_n
            ));
        }
        Ok(None)
    }
}

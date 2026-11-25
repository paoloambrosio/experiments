use std::{error::Error, time::Duration};

use avian3d::prelude::*;
use bevy::{math::DVec3, prelude::*, time::TimeUpdateStrategy};
use vehicle_physics::{
    avian_suspension::{AvianSuspensionPlugin, CornerConfig, WheelCorner},
    avian_tyre::{AvianTyre, AvianTyrePlugin},
    elastic_tyre::ElasticTyre,
    kerb::Kerb,
    mounted_tyre::{MountedTyre, RimMotion},
};

use crate::{
    elastic_experiment::TICKS_PER_SECOND,
    loaded_tyre::{
        self, BARE_RIM_KG, BEAD_RADIUS_M, G, LoadWheel, NOMINAL_LOAD_N, RATES, UPRIGHT_KG,
    },
};

use vehicle_physics::reference_car::spawn_chassis;
pub use vehicle_physics::reference_car::{SAG_M, SUSPENSION};
pub const BODY_MOUNT_DROP_M: f32 = 0.28;
pub const STEP_Z_M: f64 = 3.5;
pub const STEP_RISE_M: f64 = 0.02;
pub const SETTLE_TICKS: u32 = 4 * TICKS_PER_SECOND;
pub const CORNER_TICKS: u32 = 10 * TICKS_PER_SECOND;

pub(crate) fn physics_app(rate: u32) -> App {
    let mut app = App::new();
    app.add_plugins((
        MinimalPlugins,
        TransformPlugin,
        PhysicsPlugins::default(),
        AvianTyrePlugin,
        AvianSuspensionPlugin,
    ))
    .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
        1.0 / f64::from(TICKS_PER_SECOND),
    )))
    .insert_resource(Time::<Fixed>::from_hz(f64::from(TICKS_PER_SECOND)))
    .insert_resource(SubstepCount(rate / TICKS_PER_SECOND))
    .insert_resource(Gravity(Vec3::NEG_Y * G as f32));
    app.finish();
    app.cleanup();
    app.update();
    app
}

fn attach(
    world: &mut World,
    chassis: Entity,
    tyre: &ElasticTyre,
    mount: Vec3,
    step: bool,
) -> Result<WheelCorner, Box<dyn Error>> {
    let centre = world.get::<Position>(chassis).unwrap().0 + mount
        - Vec3::Y * (SUSPENSION.free_length_m - SAG_M as f32);
    let motion = RimMotion {
        centre: centre.as_dvec3(),
        ..Default::default()
    };
    let mut mounted = MountedTyre::new(tyre.clone(), motion, Some(loaded_tyre::floor(true)))?;
    if step {
        mounted = mounted.with_kerb(Kerb::new(STEP_Z_M, STEP_RISE_M, 0.008)?);
    }
    Ok(WheelCorner::spawn(
        world,
        chassis,
        mounted,
        CornerConfig {
            mount,
            suspension: SUSPENSION,
            upright_mass_kg: UPRIGHT_KG,
            bare_rim_mass_kg: BARE_RIM_KG,
        },
    )?)
}

#[derive(Clone, Copy, Default)]
pub struct CornerSample {
    pub time: f64,
    pub position: DVec3,
    pub body_y: f64,
    pub compression: f64,
    pub support: f64,
    pub kerb_force: f64,
    pub clearance: f64,
    hub_error: f64,
}

pub struct CornerRun {
    pub app: App,
    pub wheel: WheelCorner,
    pub chassis: Entity,
    pub tick: u32,
    pub step: bool,
}

impl CornerRun {
    pub fn new(rate: u32, tyre: &ElasticTyre, step: bool) -> Result<Self, Box<dyn Error>> {
        let mut app = physics_app(rate);
        let world = app.world_mut();
        let body_y = loaded_tyre::unloaded_radius(tyre) as f32 + SUSPENSION.free_length_m
            - SAG_M as f32
            + BODY_MOUNT_DROP_M;
        let chassis = spawn_chassis(world, 230.0, Vec3::Y * body_y, Vec3::new(0.45, 0.16, 0.45));
        world.entity_mut(chassis).insert(
            LockedAxes::new()
                .lock_translation_x()
                .lock_rotation_x()
                .lock_rotation_y()
                .lock_rotation_z(),
        );
        let wheel = attach(world, chassis, tyre, Vec3::NEG_Y * BODY_MOUNT_DROP_M, step)?;
        Ok(Self {
            app,
            wheel,
            chassis,
            tick: 0,
            step,
        })
    }

    pub fn advance(&mut self) -> Result<CornerSample, Box<dyn Error>> {
        let speed = 2.0
            * (self.tick.saturating_sub(SETTLE_TICKS) as f32 / TICKS_PER_SECOND as f32).min(1.0);
        self.app
            .world_mut()
            .get_mut::<LinearVelocity>(self.chassis)
            .unwrap()
            .z = speed;
        self.app.update();
        self.tick += 1;
        self.sample()
    }

    fn sample(&self) -> Result<CornerSample, Box<dyn Error>> {
        let world = self.app.world();
        let wheel = world.get::<AvianTyre>(self.wheel.rim).unwrap();
        if let Some(error) = wheel.error() {
            return Err(error.into());
        }
        let position = wheel.tyre().rim().centre;
        let body_y = f64::from(world.get::<Position>(self.chassis).unwrap().y);
        let hub = world.get::<Position>(self.wheel.hub).unwrap().0.as_dvec3();
        let road_y = if self.step && position.z + BEAD_RADIUS_M >= STEP_Z_M {
            STEP_RISE_M
        } else {
            0.0
        };
        let road = wheel.last_step();
        if !position.is_finite() || !body_y.is_finite() || !hub.is_finite() {
            return Err("nonfinite corner pose".into());
        }
        Ok(CornerSample {
            time: f64::from(self.tick) / f64::from(TICKS_PER_SECOND),
            position,
            body_y,
            compression: f64::from(SUSPENSION.free_length_m + BODY_MOUNT_DROP_M) - (body_y - hub.y),
            support: road.floor_force_n + road.kerb_force.y,
            kerb_force: road.kerb_normal_force_n,
            clearance: position.y - road_y - BEAD_RADIUS_M,
            hub_error: position.distance(hub),
        })
    }
}

pub fn report(samples: bool) -> Result<(), Box<dyn Error>> {
    let original = loaded_tyre::prepare(RATES[1], false)?;
    let mut wheel = LoadWheel::new(original, false)?;
    for load in [20.0 * G, 0.5 * NOMINAL_LOAD_N, NOMINAL_LOAD_N] {
        let support = loaded_tyre::settle_load(&mut wheel, RATES[1], load)?;
        print_static("Original", &wheel, support);
    }
    if samples {
        println!(
            "corner,case,rate,time_s,z_m,axle_y_m,body_y_m,compression_m,support_n,kerb_normal_n"
        );
    }
    let mut reference = None;
    for rate in RATES {
        let tyre = loaded_tyre::prepare(rate, true)
            .map_err(|error| format!("{rate} Hz inflation: {error}"))?;
        let mut wheel = LoadWheel::new(tyre.clone(), true)?;
        let support = loaded_tyre::settle_load(&mut wheel, rate, NOMINAL_LOAD_N)?;
        print_static(&format!("Light car {rate} Hz"), &wheel, support);
        let static_reading = wheel.static_reading();
        if !(0.010..=0.035).contains(&static_reading.deflection_m)
            || static_reading.penetration_m > 0.003
            || static_reading.clearance_m <= 0.0
        {
            return Err(format!("{rate} Hz light-car static load targets failed").into());
        }
        let flat = run_case(rate, &tyre, false, samples)?;
        let step = run_case(rate, &tyre, true, samples)?;
        if ((step.0 - flat.0) - STEP_RISE_M).abs() > 0.003 {
            return Err(format!("{rate} Hz corner did not settle at the top of the step").into());
        }
        if let Some((height, peak)) = reference {
            let delta_height: f64 = height - static_reading.height_m;
            let delta_peak: f64 = peak - step.1;
            if delta_height.abs() > 0.002 || delta_peak.abs() > 0.005 {
                return Err(format!(
                    "{rate} Hz static height or peak travel changed beyond tolerance"
                )
                .into());
            }
        }
        reference = Some((static_reading.height_m, step.1));
    }
    println!("Static tyre load, suspension sag, step recovery, and timestep checks passed.");
    Ok(())
}

fn print_static(label: &str, wheel: &LoadWheel, support: f64) {
    let s = wheel.static_reading();
    println!(
        "{label}: load {:.1} N, support {support:.1} N, radius {:.5} m, height {:.5} m, deflection {:.5} m, penetration {:.5} m, rim clearance {:.5} m, patch {:.3} x {:.3} m, {:.2} kPa",
        wheel.target_load_n,
        s.radius_m,
        s.height_m,
        s.deflection_m,
        s.penetration_m,
        s.clearance_m,
        s.patch_length_m,
        s.patch_width_m,
        s.pressure_pa / 1000.0
    );
}

fn run_case(
    rate: u32,
    tyre: &ElasticTyre,
    step: bool,
    samples: bool,
) -> Result<(f64, f64), Box<dyn Error>> {
    let name = if step { "step" } else { "flat" };
    let mut run = CornerRun::new(rate, tyre, step)?;
    let mut static_sums = [0.0; 2];
    let mut final_sums = [0.0; 4];
    let mut peak = 0.0_f64;
    let mut min_clearance = f64::INFINITY;
    let mut hub_error = 0.0_f64;
    let mut hit = false;
    for tick in 1..=CORNER_TICKS {
        let s = run
            .advance()
            .map_err(|error| format!("{rate} Hz {name} at tick {tick}: {error}"))?;
        hub_error = hub_error.max(s.hub_error);
        if samples {
            println!(
                "corner,{name},{rate},{:.6},{:.6},{:.6},{:.6},{:.6},{:.3},{:.3}",
                s.time,
                s.position.z,
                s.position.y,
                s.body_y,
                s.compression,
                s.support,
                s.kerb_force
            );
        }
        if (SETTLE_TICKS - 59..=SETTLE_TICKS).contains(&tick) {
            static_sums[0] += s.support / 60.0;
            static_sums[1] += s.compression / 60.0;
        }
        if tick > SETTLE_TICKS {
            peak = peak.max((s.compression - SAG_M).abs());
            min_clearance = min_clearance.min(s.clearance);
            hit |= s.kerb_force > 0.0;
        }
        if tick > CORNER_TICKS - 60 {
            final_sums[0] += s.position.y / 60.0;
            final_sums[1] += s.support / 60.0;
            final_sums[2] += s.compression / 60.0;
            let body_speed = run
                .app
                .world()
                .get::<LinearVelocity>(run.chassis)
                .unwrap()
                .y;
            final_sums[3] += f64::from(body_speed.abs()) / 60.0;
        }
    }
    println!(
        "{rate} Hz {name}: static support {:.1} N, sag {:.5} m, final height {:.5} m, support {:.1} N, sag {:.5} m, peak travel {:.5} m, clearance {:.5} m",
        static_sums[0],
        static_sums[1],
        final_sums[0],
        final_sums[1],
        final_sums[2],
        peak,
        min_clearance
    );
    if (static_sums[0] - NOMINAL_LOAD_N).abs() > 0.02 * NOMINAL_LOAD_N
        || (static_sums[1] - SAG_M).abs() > 0.003
        || (final_sums[1] - NOMINAL_LOAD_N).abs() > 0.02 * NOMINAL_LOAD_N
        || (final_sums[2] - SAG_M).abs() > 0.003
        || final_sums[3] > 0.02
        || min_clearance <= 0.0
        || hub_error > 0.001
        || hit != step
    {
        return Err(
            format!("{rate} Hz {name} support, sag, clearance, or settling check failed").into(),
        );
    }
    Ok((final_sums[0], peak))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_jointed_corner_carries_the_sprung_mass_across_a_step() {
        let rate = RATES[0];
        let tyre = loaded_tyre::prepare(rate, true).unwrap();
        let (height, travel) = run_case(rate, &tyre, true, false).unwrap();
        assert!(height > BEAD_RADIUS_M + STEP_RISE_M);
        assert!(travel > 0.001);
    }
}

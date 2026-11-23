use std::collections::VecDeque;

use avian3d::prelude::*;
use bevy::{math::DVec3, prelude::*};
use vehicle_physics::{
    avian_tyre::{AvianTyre, AvianTyrePlugin, TyreSubstepComplete},
    elastic_tyre::{ElasticTyreConfig, ElasticTyreError},
    mounted_tyre::{FlatFloor, MountedTyre, RimMotion},
};

use crate::elastic_experiment::{Experiment, PRESETS_PA, TICKS_PER_SECOND};

pub const CLEARANCE_M: f64 = 0.10;
pub const GRAVITY_MPS2: f32 = 9.81;
const RIM_MASS_KG: f64 = 12.0;
pub const WHEEL_WEIGHT_N: f64 = 196.2;
pub const TIMEOUT_S: u32 = 10;
const FLOOR_STIFFNESS: f64 = 35_000.0;
const FLOOR_DAMPING: f64 = 150.0;
const IMPACT_BEAM_DAMPING_SCALE: f64 = 3.0;

pub fn install(app: &mut App, preset: usize, rate: u32, auto_release: bool) {
    app.add_plugins((PhysicsPlugins::default(), AvianTyrePlugin))
        .insert_resource(Time::<Fixed>::from_hz(f64::from(TICKS_PER_SECOND)))
        .insert_resource(SubstepCount(rate / TICKS_PER_SECOND))
        .insert_resource(Gravity(Vec3::NEG_Y * GRAVITY_MPS2))
        .insert_resource(DropBench::new(preset, rate, auto_release))
        .add_systems(FixedUpdate, update_bench)
        .add_systems(SubstepSchedule, record_substep.after(TyreSubstepComplete));
}

#[derive(Resource)]
pub struct DropBench {
    pub preset: usize,
    pub preparation: Option<Experiment>,
    pub baseline: Option<MountedTyre>,
    pub wheel: Option<Entity>,
    pub run: DropRun,
    pub error: Option<String>,
    rate: u32,
    auto_release: bool,
}

impl DropBench {
    fn new(preset: usize, rate: u32, auto_release: bool) -> Self {
        let mut elastic_config = ElasticTyreConfig::default();
        elastic_config.circumferential.damping *= IMPACT_BEAM_DAMPING_SCALE;
        elastic_config.cross_section.damping *= IMPACT_BEAM_DAMPING_SCALE;
        elastic_config.diagonal.damping *= IMPACT_BEAM_DAMPING_SCALE;
        Self {
            preset,
            preparation: Some(Experiment::radial_with_config(
                rate,
                PRESETS_PA[preset],
                elastic_config,
            )),
            baseline: None,
            wheel: None,
            run: DropRun::default(),
            error: None,
            rate,
            auto_release,
        }
    }

    pub fn select(&mut self, preset: usize, commands: &mut Commands) {
        self.reset(commands);
        *self = Self::new(preset, self.rate, self.auto_release);
    }

    pub fn reset(&mut self, commands: &mut Commands) {
        if let Some(wheel) = self.wheel.take() {
            commands.entity(wheel).despawn();
        }
        self.run = DropRun::default();
        self.error = None;
    }

    pub fn release(&mut self, commands: &mut Commands) {
        if let Some(baseline) = self.baseline.clone() {
            self.reset(commands);
            self.run.trace.push((0.0, baseline.rim().centre.y));
            match AvianTyre::rim_bundle(baseline, RIM_MASS_KG) {
                Ok(bundle) => self.wheel = Some(commands.spawn(bundle).id()),
                Err(error) => self.error = Some(error.to_string()),
            }
        }
    }
}

fn update_bench(
    mut commands: Commands,
    mut bench: ResMut<DropBench>,
    enabled: Query<(), Without<RigidBodyDisabled>>,
) {
    if let Some(wheel) = bench.wheel
        && (bench.run.settled.is_some() || bench.error.is_some())
        && enabled.contains(wheel)
    {
        commands.entity(wheel).insert(RigidBodyDisabled);
    }
    if bench.error.is_some() {
        return;
    }
    let Some(preparation) = &mut bench.preparation else {
        return;
    };
    for _ in 0..preparation.substeps_per_tick() {
        if preparation.is_ready() {
            break;
        }
        if let Err(error) = preparation.prepare_step() {
            bench.error = Some(error.to_string());
            return;
        }
    }
    if preparation.is_ready() {
        let preparation = bench.preparation.take().expect("pressure prepared");
        match mount(preparation) {
            Ok(baseline) => {
                bench.baseline = Some(baseline);
                if bench.auto_release {
                    bench.release(&mut commands);
                }
            }
            Err(error) => bench.error = Some(error.to_string()),
        }
    }
}

fn mount(preparation: Experiment) -> Result<MountedTyre, ElasticTyreError> {
    let floor = FlatFloor::new(0.0, FLOOR_STIFFNESS, FLOOR_DAMPING)?;
    let lowest = preparation.tyre.positions()[preparation.segments..4 * preparation.segments]
        .iter()
        .map(|position| position.y)
        .fold(f64::INFINITY, f64::min);
    MountedTyre::new(
        preparation.tyre,
        RimMotion {
            centre: DVec3::Y * (CLEARANCE_M - lowest),
            ..default()
        },
        Some(floor),
    )
}

#[derive(Default)]
pub struct DropRun {
    steps: u32,
    pub contact_time_s: Option<f64>,
    pub max_penetration_m: f64,
    pub rebound_m: Option<f64>,
    compressed_height_m: Option<f64>,
    rebound_peak_m: f64,
    slow_steps: usize,
    forces: VecDeque<f64>,
    force_sum: f64,
    pub last_floor_force_n: f64,
    pub settled: Option<SettledDrop>,
    pub trace: Vec<(f64, f64)>,
}

pub struct SettledDrop {
    pub time_s: f64,
    pub axle_height_m: f64,
    pub patch_length_m: f64,
    pub patch_width_m: f64,
    pub mean_floor_force_n: f64,
}

impl DropRun {
    fn record(&mut self, tyre: &MountedTyre, floor_force_n: f64, rate: u32) {
        self.steps += 1;
        let time_s = f64::from(self.steps) / f64::from(rate);
        let rim = tyre.rim();
        let penetration_m = tyre.floor_penetration_m();
        self.last_floor_force_n = floor_force_n;
        self.max_penetration_m = self.max_penetration_m.max(penetration_m);
        if penetration_m > 0.0 && self.contact_time_s.is_none() {
            self.contact_time_s = Some(time_s);
        }
        if self.contact_time_s.is_some() && self.rebound_m.is_none() {
            if let Some(compressed_height_m) = self.compressed_height_m {
                self.rebound_peak_m = self.rebound_peak_m.max(rim.centre.y);
                if rim.linear_velocity.y < 0.0 {
                    self.rebound_m = Some(self.rebound_peak_m - compressed_height_m);
                }
            } else if rim.linear_velocity.y >= 0.0 {
                let compressed_height_m = rim.centre.y - rim.linear_velocity.y / f64::from(rate);
                self.compressed_height_m = Some(compressed_height_m);
                self.rebound_peak_m = rim.centre.y;
            }
        }
        let end_of_tick = self.steps.is_multiple_of(rate / TICKS_PER_SECOND);
        if end_of_tick {
            self.trace.push((time_s, rim.centre.y));
        }
        let window = rate as usize / 2;
        self.forces.push_back(floor_force_n);
        self.force_sum += floor_force_n;
        if self.forces.len() > window {
            self.force_sum -= self.forces.pop_front().expect("force window is full");
        }
        if rim.linear_velocity.y.abs() < 0.01 {
            self.slow_steps += 1;
        } else {
            self.slow_steps = 0;
        }
        let mean_force_n = self.force_sum / window as f64;
        if end_of_tick
            && self.slow_steps >= window
            && (mean_force_n - WHEEL_WEIGHT_N).abs() <= 0.02 * WHEEL_WEIGHT_N
            && let Some(patch) = tyre.footprint()
        {
            self.settled = Some(SettledDrop {
                time_s,
                axle_height_m: rim.centre.y,
                patch_length_m: patch.length_m,
                patch_width_m: patch.width_m,
                mean_floor_force_n: mean_force_n,
            });
        }
    }
}

fn record_substep(mut bench: ResMut<DropBench>, wheels: Query<&AvianTyre>) {
    if bench.run.settled.is_some() || bench.error.is_some() {
        return;
    }
    let Some(wheel) = bench.wheel.and_then(|entity| wheels.get(entity).ok()) else {
        return;
    };
    if let Some(error) = wheel.error() {
        bench.error = Some(error.to_string());
        return;
    }
    let rate = bench.rate;
    bench
        .run
        .record(wheel.tyre(), wheel.last_step().floor_force_n, rate);
    if bench.run.steps >= TIMEOUT_S * rate && bench.run.settled.is_none() {
        let rim = wheel.tyre().rim();
        let mean_force_n = bench.run.force_sum / bench.run.forces.len() as f64;
        bench.error = Some(format!(
            "Wheel did not settle within {TIMEOUT_S} s\nAxle speed {:+.3} m/s, mean floor force {:.1} N",
            rim.linear_velocity.y, mean_force_n,
        ));
    }
}

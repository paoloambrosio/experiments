use avian3d::prelude::*;
use bevy::{math::DVec3, prelude::*};
use vehicle_physics::{
    avian_tyre::{AvianTyre, AvianTyrePlugin, TyreSubstepComplete},
    beam_damage::{BeamDamage, BeamDamageConfig},
    elastic_tyre::{ElasticTyre, ElasticTyreError},
    kerb::Kerb,
    mounted_tyre::{MountedTyre, RimMotion},
};

use crate::{
    drop_experiment::{self, GRAVITY_MPS2, RIM_MASS_KG},
    elastic_experiment::{Experiment, TICKS_PER_SECOND},
    rolling_experiment::{FRICTION, PRESSURE_PA},
};

pub const SPEEDS_MPS: [f64; 3] = [2.0, 4.0, 6.0];
pub const KERB_RISE_M: f64 = 0.06;
pub const KERB_RADIUS_M: f64 = 0.01;
pub const LAUNCH_Z_M: f64 = -0.85;
const ROLLING_RADIUS_M: f64 = 0.33551;
const GRIP_TRANSITION_MPS: f64 = 0.1;
const CAPTURE_AFTER_CONTACT_S: f64 = 0.45;
const APPROACH_TIMEOUT_S: u32 = 2;
const UNLOAD_TIMEOUT_S: u32 = 8;
const SETTLE_SPEED_MPS: f64 = 0.0005;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Inflating,
    Ready,
    Coasting,
    Unloading,
    Complete,
    Failed,
}

#[derive(Clone, Copy, Default)]
pub struct Sample {
    pub time_s: f64,
    pub forward_speed_mps: f64,
    pub kerb_force_n: f64,
    pub inward_m: f64,
    pub yielded: usize,
    pub broken: usize,
}

#[derive(Default)]
pub struct ImpactRun {
    pub last: Sample,
    pub samples: Vec<Sample>,
    pub contact: Option<(f64, f64)>,
    pub peak_inward_m: f64,
    pub residual_m: Option<f64>,
    pub speed_mps: f64,
    pub control: bool,
    steps: u32,
}

#[derive(Resource)]
pub struct ImpactBench {
    pub phase: Phase,
    pub preparation: Option<Experiment>,
    pub baseline: Option<ElasticTyre>,
    pub wheel: Option<Entity>,
    pub unloaded: Option<ElasticTyre>,
    pub run: ImpactRun,
    pub error: Option<String>,
    pending: Option<Action>,
    rate: u32,
    unload_steps: u32,
    slow_steps: u32,
    zero_forces: Vec<DVec3>,
}

enum Action {
    Launch { speed_mps: f64, control: bool },
    Reset,
}

impl ImpactBench {
    fn new(rate: u32) -> Self {
        Self {
            phase: Phase::Inflating,
            preparation: Some(drop_experiment::prepare_wheel(rate, PRESSURE_PA)),
            baseline: None,
            wheel: None,
            unloaded: None,
            run: ImpactRun::default(),
            error: None,
            pending: None,
            rate,
            unload_steps: 0,
            slow_steps: 0,
            zero_forces: Vec::new(),
        }
    }

    pub fn launch(&mut self, speed_mps: f64, control: bool) {
        if self.baseline.is_some() {
            self.pending = Some(Action::Launch { speed_mps, control });
        }
    }

    pub fn reset(&mut self) {
        self.pending = Some(Action::Reset);
    }

    pub fn is_at(&self, phase: Phase) -> bool {
        self.phase == phase && self.pending.is_none()
    }

    fn despawn_wheel(&mut self, commands: &mut Commands) {
        if let Some(wheel) = self.wheel.take() {
            commands.entity(wheel).despawn();
        }
    }

    fn fail(&mut self, error: impl ToString) {
        self.error = Some(error.to_string());
        self.phase = Phase::Failed;
    }

    fn mount(&self, speed_mps: f64, control: bool) -> Result<MountedTyre, ElasticTyreError> {
        let tyre = self.baseline.as_ref().expect("prepared tyre").clone();
        let floor = drop_experiment::bench_floor().with_friction(FRICTION, GRIP_TRANSITION_MPS)?;
        let rim = RimMotion {
            centre: DVec3::new(0.0, launch_height_m(&tyre), LAUNCH_Z_M),
            linear_velocity: DVec3::Z * speed_mps,
            angular_velocity: DVec3::X * (speed_mps / ROLLING_RADIUS_M),
            ..default()
        };
        let mounted = MountedTyre::new(tyre, rim, Some(floor))?;
        if control {
            Ok(mounted)
        } else {
            Ok(mounted.with_kerb(Kerb::new(0.0, KERB_RISE_M, KERB_RADIUS_M)?))
        }
    }
}

pub fn launch_height_m(tyre: &ElasticTyre) -> f64 {
    -tyre
        .positions()
        .iter()
        .map(|point| point.y)
        .fold(f64::INFINITY, f64::min)
}

fn material() -> BeamDamageConfig {
    BeamDamageConfig::new(0.10, 0.09, 0.35, 0.12).expect("impact material settings")
}

pub fn install(app: &mut App, rate: u32) {
    app.add_plugins((PhysicsPlugins::default(), AvianTyrePlugin))
        .insert_resource(Time::<Fixed>::from_hz(f64::from(TICKS_PER_SECOND)))
        .insert_resource(SubstepCount(rate / TICKS_PER_SECOND))
        .insert_resource(Gravity(Vec3::NEG_Y * GRAVITY_MPS2))
        .insert_resource(ImpactBench::new(rate))
        .add_systems(FixedUpdate, update_bench)
        .add_systems(SubstepSchedule, record_coast.after(TyreSubstepComplete));
}

fn update_bench(mut commands: Commands, mut bench: ResMut<ImpactBench>) {
    if let Some(action) = bench.pending.take() {
        bench.despawn_wheel(&mut commands);
        if bench.baseline.is_none() {
            *bench = ImpactBench::new(bench.rate);
            return;
        }
        bench.unloaded = None;
        bench.unload_steps = 0;
        bench.slow_steps = 0;
        bench.error = None;
        bench.run = ImpactRun::default();
        bench.phase = Phase::Ready;
        if let Action::Launch { speed_mps, control } = action {
            let bundle = bench
                .mount(speed_mps, control)
                .and_then(|tyre| AvianTyre::rim_bundle(tyre, RIM_MASS_KG));
            match bundle {
                Ok(bundle) => {
                    bench.wheel = Some(
                        commands
                            .spawn((
                                bundle,
                                LockedAxes::new()
                                    .lock_translation_x()
                                    .lock_rotation_y()
                                    .lock_rotation_z(),
                            ))
                            .id(),
                    );
                    bench.run.speed_mps = speed_mps;
                    bench.run.control = control;
                    bench.run.last.forward_speed_mps = speed_mps;
                    let initial = bench.run.last;
                    bench.run.samples.push(initial);
                    bench.phase = Phase::Coasting;
                }
                Err(error) => bench.fail(error),
            }
        }
        return;
    }

    match bench.phase {
        Phase::Inflating => {
            let preparation = bench.preparation.as_mut().expect("preparing tyre");
            for _ in 0..preparation.substeps_per_tick() {
                if preparation.is_ready() {
                    break;
                }
                if let Err(error) = preparation.prepare_step() {
                    bench.fail(error);
                    return;
                }
            }
            if preparation.is_ready() {
                let tyre = bench.preparation.take().expect("prepared tyre").tyre;
                bench.zero_forces = vec![DVec3::ZERO; tyre.positions().len()];
                bench.baseline = Some(tyre.with_beam_damage(material()));
                bench.phase = Phase::Ready;
            }
        }
        Phase::Unloading => {
            bench.despawn_wheel(&mut commands);
            if let Err(error) = relax(&mut bench) {
                bench.fail(error);
            }
        }
        Phase::Failed => bench.despawn_wheel(&mut commands),
        _ => {}
    }
}

fn record_coast(mut bench: ResMut<ImpactBench>, wheels: Query<&AvianTyre>) {
    if bench.phase != Phase::Coasting {
        return;
    }
    let Some(wheel) = bench.wheel.and_then(|entity| wheels.get(entity).ok()) else {
        bench.fail("coasting wheel missing");
        return;
    };
    if let Some(error) = wheel.error() {
        bench.fail(error);
        return;
    }
    let tyre = wheel.tyre();
    let rim = tyre.rim();
    let step = wheel.last_step();
    let dt = 1.0 / f64::from(bench.rate);
    let time_s = f64::from(bench.run.steps) * dt;
    if bench.run.contact.is_none() && step.kerb_normal_force_n > 0.0 {
        bench.run.contact = Some((time_s, bench.run.last.forward_speed_mps));
    }

    let baseline = bench.baseline.as_ref().expect("prepared tyre");
    let inverse = rim.rotation.inverse();
    let inward_m = baseline
        .positions()
        .iter()
        .zip(tyre.positions())
        .map(|(&start, &world)| {
            let radial = start.with_x(0.0).normalize();
            (start - inverse * (world - rim.centre)).dot(radial)
        })
        .fold(0.0, f64::max);
    let (yielded, broken) = damage_counts(tyre.beam_damage().expect("enabled damage"));
    bench.run.steps += 1;
    let sample = Sample {
        time_s: time_s + dt,
        forward_speed_mps: rim.linear_velocity.z,
        kerb_force_n: step.kerb_normal_force_n,
        inward_m,
        yielded,
        broken,
    };
    bench.run.last = sample;
    if bench.run.control || bench.run.contact.is_some() {
        bench.run.peak_inward_m = bench.run.peak_inward_m.max(inward_m);
    }
    if bench
        .run
        .steps
        .is_multiple_of(bench.rate / TICKS_PER_SECOND)
    {
        bench.run.samples.push(sample);
    }
    let capture = bench.run.contact.map_or_else(
        || bench.run.control && sample.time_s >= 0.85,
        |(contact_s, _)| sample.time_s - contact_s >= CAPTURE_AFTER_CONTACT_S,
    );
    if capture {
        match tyre.clone().into_fixed_bead_tyre() {
            Ok(unloaded) => {
                bench.unloaded = Some(unloaded);
                bench.phase = Phase::Unloading;
            }
            Err(error) => bench.fail(error),
        }
    } else if bench.run.contact.is_none() && bench.run.steps >= APPROACH_TIMEOUT_S * bench.rate {
        bench.fail("wheel did not reach the kerb");
    }
}

fn relax(bench: &mut ImpactBench) -> Result<(), Box<dyn std::error::Error>> {
    let tyre = bench.unloaded.as_mut().expect("captured tyre");
    let dt = 1.0 / f64::from(bench.rate);
    for _ in 0..bench.rate / TICKS_PER_SECOND {
        tyre.step(dt, &bench.zero_forces)?;
        bench.unload_steps += 1;
        if tyre
            .velocities()
            .iter()
            .all(|v| v.length_squared() < SETTLE_SPEED_MPS.powi(2))
        {
            bench.slow_steps += 1;
        } else {
            bench.slow_steps = 0;
        }
        if bench.slow_steps >= bench.rate / 4 {
            let baseline = bench.baseline.as_ref().expect("prepared tyre");
            bench.run.residual_m = Some(
                baseline
                    .positions()
                    .iter()
                    .zip(tyre.positions())
                    .map(|(before, after)| before.distance(*after))
                    .fold(0.0, f64::max),
            );
            bench.phase = Phase::Complete;
            return Ok(());
        }
    }
    if bench.unload_steps >= UNLOAD_TIMEOUT_S * bench.rate {
        return Err("unloaded tyre did not settle within 8 seconds".into());
    }
    Ok(())
}

pub fn damage_counts(states: &[BeamDamage]) -> (usize, usize) {
    states.iter().fold((0, 0), |(yielded, broken), state| {
        (
            yielded + usize::from(state.yielded()),
            broken + usize::from(state.broken()),
        )
    })
}

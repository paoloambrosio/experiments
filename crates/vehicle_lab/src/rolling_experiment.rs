use std::f64::consts::TAU;

use avian3d::prelude::*;
use bevy::{math::DVec3, prelude::*};
use vehicle_physics::{
    avian_tyre::{AvianTyre, AvianTyrePlugin, TyreSubstepComplete},
    mounted_tyre::{MountedStep, MountedTyre, RimMotion},
};

use crate::{
    drop_experiment::{self, DropRun, GRAVITY_MPS2, RIM_MASS_KG, WHEEL_WEIGHT_N},
    elastic_experiment::{Experiment, TICKS_PER_SECOND},
};

pub const PRESSURE_PA: f64 = 30_000.0;
pub const BELT_SPEED_MPS: f64 = 2.0;
pub const FRICTION: f64 = 0.8;
const TRANSITION_SPEED_MPS: f64 = 0.1;
pub const ROLL_TIMEOUT_S: u32 = 12;

#[derive(Clone, Copy, Debug, Default)]
pub enum LoadCase {
    #[default]
    Free,
    Drive,
    Brake,
    Lateral,
}

impl LoadCase {
    pub const ALL: [Self; 4] = [Self::Free, Self::Drive, Self::Brake, Self::Lateral];

    pub fn name(self) -> &'static str {
        match self {
            Self::Free => "Free rolling",
            Self::Drive => "Drive",
            Self::Brake => "Braking",
            Self::Lateral => "Lateral load",
        }
    }

    pub fn torque_nm(self) -> f32 {
        match self {
            Self::Drive => 10.0,
            Self::Brake => -10.0,
            _ => 0.0,
        }
    }

    pub fn lateral_load_n(self) -> f32 {
        match self {
            Self::Lateral => 30.0,
            _ => 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    Inflating,
    Settling,
    StartBelt,
    FreeRolling,
    Ready,
    Running,
    Complete,
    Failed,
}

pub fn install(app: &mut App, rate: u32) {
    app.add_plugins((PhysicsPlugins::default(), AvianTyrePlugin))
        .insert_resource(Time::<Fixed>::from_hz(f64::from(TICKS_PER_SECOND)))
        .insert_resource(SubstepCount(rate / TICKS_PER_SECOND))
        .insert_resource(Gravity(Vec3::NEG_Y * GRAVITY_MPS2))
        .insert_resource(RollingBench::new(rate))
        .add_systems(FixedUpdate, update_bench)
        .add_systems(SubstepSchedule, record_substep.after(TyreSubstepComplete));
}

#[derive(Resource)]
pub struct RollingBench {
    pub phase: Phase,
    pub preparation: Option<Experiment>,
    pub baseline: Option<MountedTyre>,
    pub wheel: Option<Entity>,
    pub radius_m: Option<f64>,
    pub case: LoadCase,
    pub latest: Sample,
    pub mean: Option<Sample>,
    pub trace: Vec<(f64, Sample)>,
    pub error: Option<String>,
    drop: DropRun,
    turns: RollingTurns,
    rate: u32,
    steps: u32,
    belt_steps: u32,
    action: Option<Action>,
}

enum Action {
    Replay(LoadCase),
    Reset,
}

impl RollingBench {
    fn new(rate: u32) -> Self {
        Self {
            phase: Phase::Inflating,
            preparation: Some(drop_experiment::prepare_wheel(rate, PRESSURE_PA)),
            baseline: None,
            wheel: None,
            radius_m: None,
            case: LoadCase::Free,
            latest: Sample::default(),
            mean: None,
            trace: Vec::new(),
            error: None,
            drop: DropRun::default(),
            turns: RollingTurns::default(),
            rate,
            steps: 0,
            belt_steps: 0,
            action: None,
        }
    }

    pub fn replay(&mut self, case: LoadCase) {
        if self.baseline.is_some() {
            self.action = Some(Action::Replay(case));
        }
    }

    pub fn reset(&mut self) {
        self.action = Some(Action::Reset);
    }

    pub fn is_at(&self, phase: Phase) -> bool {
        self.phase == phase && self.action.is_none()
    }

    pub fn belt_distance_m(&self) -> f64 {
        f64::from(self.belt_steps) / f64::from(self.rate) * BELT_SPEED_MPS
    }

    fn fail(&mut self, error: impl ToString) {
        self.phase = Phase::Failed;
        self.error = Some(error.to_string());
    }

    fn clear_run(&mut self) {
        self.steps = 0;
        self.belt_steps = 0;
        self.turns = RollingTurns::default();
        self.latest = Sample::default();
        self.mean = None;
        self.trace.clear();
        self.error = None;
    }

    fn despawn(&mut self, commands: &mut Commands) {
        if let Some(wheel) = self.wheel.take() {
            commands.entity(wheel).despawn();
        }
    }

    fn spawn(&mut self, commands: &mut Commands, tyre: MountedTyre, case: LoadCase) {
        match AvianTyre::rim_bundle(tyre, RIM_MASS_KG) {
            Ok(wheel) => {
                self.wheel = Some(
                    commands
                        .spawn((
                            wheel,
                            LockedAxes::new()
                                .lock_translation_z()
                                .lock_rotation_y()
                                .lock_rotation_z(),
                            ConstantTorque(Vec3::X * case.torque_nm()),
                            ConstantForce(Vec3::X * case.lateral_load_n()),
                        ))
                        .id(),
                );
            }
            Err(error) => self.fail(error),
        }
    }
}

fn update_bench(
    mut commands: Commands,
    mut bench: ResMut<RollingBench>,
    mut running_wheel: Query<&mut AvianTyre, Without<RigidBodyDisabled>>,
) {
    match bench.action.take() {
        Some(Action::Replay(case)) => {
            let Some(baseline) = bench.baseline.clone() else {
                return;
            };
            bench.despawn(&mut commands);
            bench.clear_run();
            bench.phase = Phase::Running;
            bench.case = case;
            bench.spawn(&mut commands, baseline, case);
            return;
        }
        Some(Action::Reset) => {
            bench.despawn(&mut commands);
            if bench.baseline.is_some() {
                bench.clear_run();
                bench.phase = Phase::Ready;
            } else {
                *bench = RollingBench::new(bench.rate);
            }
            return;
        }
        None => {}
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
                let preparation = bench.preparation.take().expect("prepared tyre");
                match drop_experiment::mount(preparation) {
                    Ok(tyre) => {
                        bench.phase = Phase::Settling;
                        bench.spawn(&mut commands, tyre, LoadCase::Free);
                    }
                    Err(error) => bench.fail(error),
                }
            }
        }
        Phase::StartBelt => {
            let Some(mut wheel) = bench
                .wheel
                .and_then(|entity| running_wheel.get_mut(entity).ok())
            else {
                bench.fail("settled wheel missing");
                return;
            };
            let floor = drop_experiment::bench_floor()
                .with_friction(FRICTION, TRANSITION_SPEED_MPS)
                .and_then(|floor| floor.with_surface_velocity(-DVec3::Z * BELT_SPEED_MPS))
                .expect("rolling belt settings");
            wheel.set_floor(Some(floor));
            bench.clear_run();
            bench.phase = Phase::FreeRolling;
        }
        Phase::Ready => bench.despawn(&mut commands),
        Phase::Complete | Phase::Failed => {
            if let Some(wheel) = bench.wheel
                && running_wheel.contains(wheel)
            {
                commands.entity(wheel).insert(RigidBodyDisabled);
            }
        }
        _ => {}
    }
}

fn record_substep(mut bench: ResMut<RollingBench>, wheels: Query<&AvianTyre>) {
    if !matches!(
        bench.phase,
        Phase::Settling | Phase::FreeRolling | Phase::Running
    ) {
        return;
    }
    let Some(wheel) = bench.wheel.and_then(|entity| wheels.get(entity).ok()) else {
        bench.fail("active rolling wheel missing");
        return;
    };
    if let Some(error) = wheel.error() {
        bench.fail(error);
        return;
    }

    bench.steps += 1;
    let rate = bench.rate;
    let tyre = wheel.tyre();
    let step = wheel.last_step();
    if bench.phase == Phase::Settling {
        bench.drop.record(tyre, step.floor_force_n, rate);
        if bench.drop.settled.is_some() {
            bench.phase = Phase::StartBelt;
        } else if bench.steps >= drop_experiment::TIMEOUT_S * rate {
            bench.fail("wheel did not settle on the stationary belt");
        }
        return;
    }

    bench.belt_steps += 1;
    let sample = Sample::from_step(tyre.rim(), step);
    let running_case = bench.phase == Phase::Running;
    bench.latest = sample;
    let time_s = f64::from(bench.steps) / f64::from(rate);
    if running_case && bench.steps.is_multiple_of(rate / TICKS_PER_SECOND) {
        bench.trace.push((time_s, sample));
    }
    if time_s > 1.0
        && let Some(mean) = bench
            .turns
            .record(sample, 1.0 / f64::from(rate), steady_rolling)
    {
        if running_case {
            bench.mean = Some(mean);
            bench.phase = Phase::Complete;
        } else {
            bench.radius_m = Some(BELT_SPEED_MPS / mean.spin_rad_s);
            bench.baseline = Some(tyre.clone());
            bench.phase = Phase::Ready;
        }
    } else if bench.steps >= ROLL_TIMEOUT_S * rate {
        let case = if running_case {
            bench.case.name()
        } else {
            "Belt start"
        };
        bench.fail(format!(
            "{case} did not reach steady rolling within {ROLL_TIMEOUT_S} s"
        ));
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Sample {
    pub spin_rad_s: f64,
    pub slip_angle_rad: f64,
    pub floor_force_n: DVec3,
    pub axle_force_n: DVec3,
}

impl Sample {
    fn from_step(rim: RimMotion, step: MountedStep) -> Self {
        let relative_speed_z = rim.linear_velocity.z + BELT_SPEED_MPS;
        Self {
            spin_rad_s: rim.angular_velocity.x,
            slip_angle_rad: rim.linear_velocity.x.atan2(relative_speed_z),
            floor_force_n: step.floor_tangential_force.with_y(step.floor_force_n),
            axle_force_n: step.rim_force,
        }
    }

    pub fn slip_ratio(self, radius_m: f64) -> f64 {
        self.spin_rad_s * radius_m / BELT_SPEED_MPS - 1.0
    }
}

#[derive(Default)]
struct TurnIntegral {
    seconds: f64,
    sum: Sample,
}

impl TurnIntegral {
    fn add(&mut self, sample: Sample, seconds: f64) {
        self.seconds += seconds;
        self.sum.spin_rad_s += sample.spin_rad_s * seconds;
        self.sum.slip_angle_rad += sample.slip_angle_rad * seconds;
        self.sum.floor_force_n += sample.floor_force_n * seconds;
        self.sum.axle_force_n += sample.axle_force_n * seconds;
    }

    fn mean(&self) -> Sample {
        Sample {
            spin_rad_s: self.sum.spin_rad_s / self.seconds,
            slip_angle_rad: self.sum.slip_angle_rad / self.seconds,
            floor_force_n: self.sum.floor_force_n / self.seconds,
            axle_force_n: self.sum.axle_force_n / self.seconds,
        }
    }
}

#[derive(Default)]
pub(crate) struct RollingTurns {
    angle_rad: f64,
    turn: TurnIntegral,
    previous: Option<TurnIntegral>,
}

impl RollingTurns {
    pub(crate) fn record(
        &mut self,
        sample: Sample,
        dt: f64,
        steady: impl FnOnce(Sample, Sample) -> bool,
    ) -> Option<Sample> {
        let speed = sample.spin_rad_s;
        if speed <= 0.0 {
            *self = Self::default();
            return None;
        }
        let remaining_s = (TAU - self.angle_rad) / speed;
        if dt < remaining_s {
            self.turn.add(sample, dt);
            self.angle_rad += speed * dt;
            return None;
        }
        // Split the boundary substep between the two revolutions.
        self.turn.add(sample, remaining_s);
        let current = std::mem::take(&mut self.turn);
        self.turn.add(sample, dt - remaining_s);
        self.angle_rad = speed * (dt - remaining_s);
        let mean = current.mean();
        let previous = self.previous.replace(current)?;
        if steady(mean, previous.mean()) {
            let mut combined = previous;
            let current = self.previous.as_ref().expect("current turn");
            combined.add(mean, current.seconds);
            Some(combined.mean())
        } else {
            None
        }
    }
}

fn steady_rolling(mean: Sample, prior_mean: Sample) -> bool {
    [mean, prior_mean]
        .iter()
        .all(|sample| (sample.floor_force_n.y - WHEEL_WEIGHT_N).abs() <= 0.02 * WHEEL_WEIGHT_N)
        && (mean.spin_rad_s - prior_mean.spin_rad_s).abs() <= 0.005 * mean.spin_rad_s
        && (mean.slip_angle_rad - prior_mean.slip_angle_rad)
            .abs()
            .to_degrees()
            <= 0.10
        && (mean.axle_force_n - prior_mean.axle_force_n).length() <= 1.5
}

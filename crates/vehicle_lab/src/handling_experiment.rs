use std::error::Error;

use avian3d::{dynamics::solver::schedule::SolverSystems, prelude::*};
use bevy::{math::DVec3, prelude::*};
use vehicle_physics::{
    avian_suspension::{WheelControl, WheelCorner, ackermann_steer},
    avian_tyre::{AvianTyre, TyreSubstepComplete},
    elastic_tyre::ElasticTyre,
    reference_car::ReferenceCar,
};

use crate::{
    corner_experiment::{SETTLE_TICKS, SUSPENSION, physics_app},
    elastic_experiment::TICKS_PER_SECOND as HZ,
    loaded_tyre::BEAD_RADIUS_M,
};

pub use vehicle_physics::reference_car::{
    BODY_SIZE, COM_HEIGHT, MOUNTS, TRACK, WHEEL_NAMES, WHEELBASE,
};
const DT: f64 = 1.0 / HZ as f64;
const DEADLINE: u32 = 30 * HZ;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Brake {
    Moderate,
    Limit,
}

impl Brake {
    fn torque(self) -> f32 {
        match self {
            Self::Moderate => 250.0,
            Self::Limit => 1200.0,
        }
    }
}

#[derive(Clone, Copy)]
pub struct Case {
    pub id: &'static str,
    pub label: &'static str,
    pub speed: f64,
    pub turn: f32,
    pub brake: Option<Brake>,
}

pub const CASES: [Case; 9] = [
    Case {
        id: "stop-4",
        label: "Straight / 4 m/s / Moderate brake",
        speed: 4.0,
        turn: 0.0,
        brake: Some(Brake::Moderate),
    },
    Case {
        id: "stop-8",
        label: "Straight / 8 m/s / Moderate brake",
        speed: 8.0,
        turn: 0.0,
        brake: Some(Brake::Moderate),
    },
    Case {
        id: "limit-stop-8",
        label: "Straight / 8 m/s / Limit brake",
        speed: 8.0,
        turn: 0.0,
        brake: Some(Brake::Limit),
    },
    Case {
        id: "left-4",
        label: "Left turn / 4 m/s",
        speed: 4.0,
        turn: 1.0,
        brake: None,
    },
    Case {
        id: "right-4",
        label: "Right turn / 4 m/s",
        speed: 4.0,
        turn: -1.0,
        brake: None,
    },
    Case {
        id: "left-8",
        label: "Left turn / 8 m/s",
        speed: 8.0,
        turn: 1.0,
        brake: None,
    },
    Case {
        id: "right-8",
        label: "Right turn / 8 m/s",
        speed: 8.0,
        turn: -1.0,
        brake: None,
    },
    Case {
        id: "brake-left-8",
        label: "Left turn / 8 m/s / Moderate brake",
        speed: 8.0,
        turn: 1.0,
        brake: Some(Brake::Moderate),
    },
    Case {
        id: "limit-left-8",
        label: "Left turn / 8 m/s / Limit brake",
        speed: 8.0,
        turn: 1.0,
        brake: Some(Brake::Limit),
    },
];

#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub enum Phase {
    #[default]
    Settle,
    Launch,
    Turn,
    Brake,
    Done,
}

impl Phase {
    pub fn label(self) -> &'static str {
        match self {
            Self::Settle => "Settling",
            Self::Launch => "Launching",
            Self::Turn => "Turning",
            Self::Brake => "Braking",
            Self::Done => "Complete",
        }
    }
}

#[derive(Component, Default)]
struct TickRoad {
    sum: DVec3,
    count: u32,
    min_clearance: f64,
}

fn reset_road(mut wheels: Query<&mut TickRoad>) {
    for mut road in &mut wheels {
        *road = TickRoad {
            min_clearance: f64::INFINITY,
            ..default()
        };
    }
}

fn gather_road(mut wheels: Query<(&AvianTyre, &mut TickRoad)>) {
    for (tyre, mut road) in &mut wheels {
        let step = tyre.last_step();
        road.sum += step.floor_tangential_force + DVec3::Y * step.floor_force_n;
        road.count += 1;
        let rim = tyre.tyre().rim();
        road.min_clearance = road.min_clearance.min(rim.centre.y - BEAD_RADIUS_M);
    }
}

#[derive(Clone, Copy, Default)]
pub struct WheelSample {
    pub road: DVec3,
    pub sag: f64,
    pub clearance: f64,
    pub steer: f32,
    pub spin: f64,
    pub forward_speed: f64,
    pub locked: bool,
}

#[derive(Clone, Copy, Default)]
pub struct Sample {
    pub time: f64,
    pub phase: Phase,
    pub position: Vec3,
    pub velocity: Vec3,
    pub rotation: Quat,
    pub speed: f64,
    pub yaw_rate: f64,
    pub roll: f64,
    pub lateral_accel: f64,
    pub wheels: [WheelSample; 4],
}

#[derive(Clone, Copy, Default)]
struct Window {
    count: u32,
    speed_ok: bool,
    yaw_sum: f64,
    lateral_sum: f64,
    roll_sum: f64,
    distance: f64,
    heading: f64,
    support: [f64; 4],
    sag: [f64; 4],
    height: f64,
    max_drift: f64,
}

impl Window {
    fn push(&mut self, sample: Sample, target: f64, distance: f64, heading: f64) {
        let speed_ok = (sample.speed - target).abs() <= target * 0.02;
        self.speed_ok = (self.count == 0 || self.speed_ok) && speed_ok;
        self.count += 1;
        self.distance += distance;
        self.heading += heading;
        self.height += f64::from(sample.position.y);
        self.max_drift = self.max_drift.max(f64::from(sample.position.x.abs()));
        self.yaw_sum += sample.yaw_rate;
        self.lateral_sum += sample.lateral_accel;
        self.roll_sum += sample.roll;
        for (i, wheel) in sample.wheels.iter().enumerate() {
            self.support[i] += wheel.road.y;
            self.sag[i] += wheel.sag;
        }
    }

    fn mean(self, sum: f64) -> f64 {
        sum / f64::from(self.count)
    }

    fn loads(self) -> [f64; 4] {
        self.support.map(|n| self.mean(n))
    }
}

#[derive(Default)]
pub struct Results {
    pub static_loads: [f64; 4],
    pub static_sag: [f64; 4],
    pub static_height: f64,
    pub launch_time: f64,
    pub drift: f64,
    pub radius: f64,
    pub yaw: f64,
    pub lateral_accel: f64,
    pub roll: f64,
    pub turn_loads: [f64; 4],
    pub brake_entry_speed: f64,
    pub stop_distance: f64,
    pub stop_time: f64,
    pub brake_loads: [f64; 4],
    pub brake_curvature: f64,
    pub brake_yaw: f64,
    pub lock_time: [Option<f64>; 4],
    pub min_clearance: f64,
}

pub struct HandlingRun {
    pub app: App,
    pub chassis: Entity,
    pub wheels: [WheelCorner; 4],
    pub case: Case,
    pub phase: Phase,
    pub sample: Option<Sample>,
    pub results: Results,
    tick: u32,
    phase_tick: u32,
    integral: f64,
    window: Window,
    previous_turn: Option<Window>,
    below_stop: u32,
    lock_ticks: [u32; 4],
    brake_approach: Window,
    brake_curve_done: bool,
}

impl HandlingRun {
    pub fn new(rate: u32, tyre: &ElasticTyre, case: Case) -> Result<Self, Box<dyn Error>> {
        let mut app = physics_app(rate);
        app.add_systems(PhysicsSchedule, reset_road.before(SolverSystems::Substep))
            .add_systems(SubstepSchedule, gather_road.after(TyreSubstepComplete));
        let world = app.world_mut();
        let ReferenceCar { chassis, wheels } = ReferenceCar::spawn(world, tyre)?;
        for wheel in &wheels {
            world.entity_mut(wheel.rim).insert(TickRoad::default());
        }
        Ok(Self {
            app,
            chassis,
            wheels,
            case,
            phase: Phase::Settle,
            sample: None,
            tick: 0,
            phase_tick: 0,
            integral: 0.0,
            window: Window::default(),
            previous_turn: None,
            below_stop: 0,
            lock_ticks: [0; 4],
            brake_approach: Window::default(),
            brake_curve_done: false,
            results: Results {
                min_clearance: f64::INFINITY,
                ..default()
            },
        })
    }

    pub fn advance(&mut self) -> Result<Sample, Box<dyn Error>> {
        if self.phase == Phase::Done {
            return Ok(self.sample.unwrap());
        }
        self.set_controls();
        self.app.update();
        self.tick += 1;
        self.phase_tick += 1;
        let mut sample = self.read_sample()?;
        let (distance, heading) = self.sample.map_or((0.0, 0.0), |last| {
            let delta = (sample.position - last.position).with_y(0.0);
            let from = last.velocity.with_y(0.0);
            let to = sample.velocity.with_y(0.0);
            let heading = if from.length_squared() > 0.01 && to.length_squared() > 0.01 {
                f64::from(from.cross(to).y.atan2(from.dot(to)))
            } else {
                0.0
            };
            (f64::from(delta.length()), heading)
        });

        self.window.push(sample, self.case.speed, distance, heading);
        match self.phase {
            Phase::Settle if self.phase_tick == SETTLE_TICKS => {
                self.results.static_loads = self.window.loads();
                self.results.static_sag = self.window.sag.map(|x| self.window.mean(x));
                self.results.static_height = self.window.mean(self.window.height);
                self.transition(Phase::Launch);
            }
            Phase::Settle if self.phase_tick == SETTLE_TICKS - HZ => {
                self.window = Window::default()
            }
            Phase::Launch => {
                self.results.drift = self.results.drift.max(f64::from(sample.position.x.abs()));
                if !self.window.speed_ok {
                    self.window = Window::default();
                } else if self.window.count == HZ {
                    self.results.launch_time = f64::from(self.phase_tick - HZ + 1) * DT;
                    if self.case.turn == 0.0 {
                        self.begin_braking(sample);
                    } else {
                        self.transition(Phase::Turn);
                    }
                }
            }
            Phase::Turn if self.phase_tick == HZ / 2 => self.window = Window::default(),
            Phase::Turn if self.window.count == HZ => {
                let yaw = self.window.mean(self.window.yaw_sum);
                let steady = self.window.speed_ok
                    && yaw * f64::from(self.case.turn) > 0.01
                    && self.previous_turn.is_some_and(|prev| {
                        (yaw - prev.mean(prev.yaw_sum)).abs() <= yaw.abs() * 0.03
                    });
                if steady {
                    self.results.radius = self.window.distance / self.window.heading.abs();
                    self.results.yaw = yaw;
                    self.results.lateral_accel = self.window.mean(self.window.lateral_sum);
                    self.results.roll = self.window.mean(self.window.roll_sum);
                    self.results.turn_loads = self.window.loads();
                    if self.case.brake.is_some() {
                        self.begin_braking(sample);
                    } else {
                        self.transition(Phase::Done);
                    }
                } else {
                    self.previous_turn = self.window.speed_ok.then_some(self.window);
                    self.window = Window::default();
                }
            }
            Phase::Brake => self.measure_stop(&mut sample, distance, heading),
            _ => {}
        }
        if self.tick >= DEADLINE && self.phase != Phase::Done {
            return Err(format!(
                "{} timed out at {:.2} m/s",
                self.phase.label(),
                sample.speed
            )
            .into());
        }
        self.sample = Some(sample);
        Ok(sample)
    }

    fn transition(&mut self, phase: Phase) {
        self.phase = phase;
        self.phase_tick = 0;
        self.window = Window::default();
    }

    fn begin_braking(&mut self, sample: Sample) {
        self.results.brake_entry_speed = sample.speed;
        self.transition(Phase::Brake);
    }

    fn set_controls(&mut self) {
        let steer = if matches!(self.phase, Phase::Turn | Phase::Brake) {
            self.case.turn
                * 6.0_f32.to_radians()
                * if self.phase == Phase::Brake {
                    1.0
                } else {
                    (self.phase_tick as f32 / (HZ / 2) as f32).min(1.0)
                }
        } else {
            0.0
        };
        let [left, right] = ackermann_steer(WHEELBASE, TRACK, steer).unwrap();
        let mut drive = 0.0;
        let mut brake = 0.0;
        match self.phase {
            Phase::Launch | Phase::Turn => {
                let speed = self.sample.map_or(0.0, |s| s.speed);
                let error = self.case.speed - speed;
                let proportional = 220.0 * error;
                // Accumulate near the target to avoid winding up during launch.
                if error < 0.25 {
                    self.integral = (self.integral + 42.0 * error * DT).clamp(0.0, 100.0);
                }
                drive = (proportional + self.integral).clamp(0.0, 400.0) as f32;
            }
            Phase::Brake => brake = self.case.brake.unwrap().torque(),
            _ => {}
        }
        for (i, (wheel, steer_rad)) in self.wheels.iter().zip([left, right, 0.0, 0.0]).enumerate() {
            *self
                .app
                .world_mut()
                .get_mut::<WheelControl>(wheel.spin_joint)
                .unwrap() = WheelControl {
                steer_rad,
                drive_nm: if i >= 2 { drive } else { 0.0 },
                brake_nm: brake,
            };
        }
    }

    fn read_sample(&mut self) -> Result<Sample, Box<dyn Error>> {
        let world = self.app.world();
        let position = world.get::<Position>(self.chassis).unwrap().0;
        let rotation = world.get::<Rotation>(self.chassis).unwrap().0;
        let velocity = world.get::<LinearVelocity>(self.chassis).unwrap().0;
        let angular = world.get::<AngularVelocity>(self.chassis).unwrap().0;
        if !position.is_finite()
            || !rotation.is_finite()
            || !velocity.is_finite()
            || !angular.is_finite()
        {
            return Err("nonfinite chassis motion".into());
        }
        let lateral = rotation * Vec3::X;
        let mut readings = [WheelSample::default(); 4];
        for (i, wheel) in self.wheels.iter().enumerate() {
            let tyre = world.get::<AvianTyre>(wheel.rim).unwrap();
            if let Some(error) = tyre.error() {
                return Err(format!("{} tyre: {error}", WHEEL_NAMES[i]).into());
            }
            let rim = tyre.tyre().rim();
            let hub_position = world.get::<Position>(wheel.hub).unwrap().0;
            let hub_spin = world
                .get::<AngularVelocity>(wheel.hub)
                .unwrap()
                .0
                .as_dvec3();
            let steer = world
                .get::<WheelControl>(wheel.spin_joint)
                .unwrap()
                .steer_rad;
            let steered = rotation * Quat::from_rotation_y(steer);
            let axis = (steered * Vec3::X).as_dvec3();
            let forward = (steered * Vec3::Z)
                .with_y(0.0)
                .normalize_or_zero()
                .as_dvec3();
            let road = world.get::<TickRoad>(wheel.rim).unwrap();
            let clearance = road.min_clearance;
            if clearance <= 0.0
                || !clearance.is_finite()
                || road.count == 0
                || !road.sum.is_finite()
                || !hub_position.is_finite()
                || !hub_spin.is_finite()
            {
                return Err(format!(
                    "{} invalid road, rim clearance, or hub state",
                    WHEEL_NAMES[i]
                )
                .into());
            }
            self.results.min_clearance = self.results.min_clearance.min(clearance);
            readings[i] = WheelSample {
                road: road.sum / f64::from(road.count),
                sag: f64::from(
                    SUSPENSION.free_length_m
                        - (position + rotation * MOUNTS[i] - hub_position).dot(rotation * Vec3::Y),
                ),
                clearance,
                steer,
                spin: (rim.angular_velocity - hub_spin).dot(axis),
                forward_speed: rim.linear_velocity.with_y(0.0).dot(forward),
                locked: false,
            };
        }
        Ok(Sample {
            time: f64::from(self.tick) * DT,
            phase: self.phase,
            position,
            velocity,
            rotation,
            speed: f64::from(velocity.with_y(0.0).length()),
            yaw_rate: f64::from(angular.y),
            roll: f64::from(lateral.y.clamp(-1.0, 1.0).asin()),
            lateral_accel: self.sample.map_or(0.0, |last| {
                f64::from((velocity - last.velocity).dot(lateral.with_y(0.0).normalize_or_zero()))
                    / DT
            }),
            wheels: readings,
        })
    }

    fn measure_stop(&mut self, sample: &mut Sample, distance: f64, heading: f64) {
        // Milestone 009 nominal-load rolling radius at 8 m/s.
        const RADIUS: f64 = 0.33726;
        for (i, wheel) in sample.wheels.iter_mut().enumerate() {
            if wheel.forward_speed.abs() > 1.0
                && wheel.spin.abs() * RADIUS < 0.1 * wheel.forward_speed.abs()
            {
                self.lock_ticks[i] += 1;
            } else {
                self.lock_ticks[i] = 0;
            }
            wheel.locked = self.lock_ticks[i] >= HZ / 10;
            if wheel.locked && self.results.lock_time[i].is_none() {
                self.results.lock_time[i] =
                    Some(f64::from(self.phase_tick - self.lock_ticks[i] + 1) * DT);
            }
        }

        if !self.brake_curve_done {
            let last_speed = self.sample.unwrap().speed;
            let cutoff = self.case.speed * 0.5;
            if sample.speed <= cutoff {
                let fraction =
                    ((last_speed - cutoff) / (last_speed - sample.speed)).clamp(0.0, 1.0);
                let total_time = (f64::from(self.brake_approach.count) + fraction) * DT;
                self.results.brake_curvature = (self.brake_approach.heading + heading * fraction)
                    / (self.brake_approach.distance + distance * fraction);
                self.results.brake_yaw =
                    (self.brake_approach.yaw_sum + sample.yaw_rate * fraction) * DT / total_time;
                self.results.brake_loads = std::array::from_fn(|i| {
                    (self.brake_approach.support[i] + sample.wheels[i].road.y * fraction) * DT
                        / total_time
                });
                self.brake_curve_done = true;
            } else {
                self.brake_approach
                    .push(*sample, self.case.speed, distance, heading);
            }
        }

        if sample.speed < 0.1 {
            if self.below_stop == 0 {
                self.results.stop_distance = self.window.distance;
                self.results.stop_time = f64::from(self.phase_tick) * DT;
            }
            self.below_stop += 1;
            if self.below_stop == HZ / 4 {
                self.transition(Phase::Done);
            }
        } else {
            self.below_stop = 0;
        }
    }
}

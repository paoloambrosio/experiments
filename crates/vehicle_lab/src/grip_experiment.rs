use std::{error::Error, fmt};

use bevy::math::{DVec3, Vec3Swizzles};
use vehicle_physics::elastic_tyre::ElasticTyre;

use crate::{
    loaded_tyre::{self, BEAD_RADIUS_M, FRICTION, LoadSettling, LoadWheel, NOMINAL_LOAD_N},
    rolling_experiment::{ROLL_TIMEOUT_S, RollingTurns, Sample},
};

pub const LOADS_N: [f64; 3] = [0.5 * NOMINAL_LOAD_N, NOMINAL_LOAD_N, 1.5 * NOMINAL_LOAD_N];
pub const SPEEDS_MPS: [f64; 2] = [2.0, 8.0];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SlipCase {
    pub percent: i8,
    pub angle_deg: i8,
}

impl SlipCase {
    const fn new(percent: i8, angle_deg: i8) -> Self {
        Self { percent, angle_deg }
    }

    pub const BENCHMARK: Self = Self::new(10, 5);
    pub const ALL: [Self; 19] = [
        Self::new(0, 0),
        Self::new(2, 0),
        Self::new(5, 0),
        Self::new(10, 0),
        Self::new(20, 0),
        Self::new(-2, 0),
        Self::new(-5, 0),
        Self::new(-10, 0),
        Self::new(-20, 0),
        Self::new(0, 2),
        Self::new(0, 5),
        Self::new(0, 10),
        Self::new(0, -2),
        Self::new(0, -5),
        Self::new(0, -10),
        Self::BENCHMARK,
        Self::new(10, -5),
        Self::new(-10, 5),
        Self::new(-10, -5),
    ];

    pub fn ratio(self) -> f64 {
        f64::from(self.percent) / 100.0
    }

    pub fn belt(self, speed_mps: f64) -> DVec3 {
        let (sin, cos) = f64::from(self.angle_deg).to_radians().sin_cos();
        -speed_mps * DVec3::new(sin, 0.0, cos)
    }

    pub fn commanded_spin(self, belt: DVec3, radius_m: f64) -> f64 {
        (1.0 + self.ratio()) * -belt.z / radius_m
    }
}

impl fmt::Display for SlipCase {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        let family = match (self.percent, self.angle_deg) {
            (0, 0) => "Straight",
            (0, _) => "Lateral",
            (_, 0) if self.percent > 0 => "Drive",
            (_, 0) => "Brake",
            _ => "Combined",
        };
        write!(
            formatter,
            "{family} {:+}% / {:+} deg",
            self.percent, self.angle_deg
        )
    }
}

pub fn slip_ratio(spin_rad_s: f64, belt: DVec3, radius_m: f64) -> f64 {
    spin_rad_s * radius_m / -belt.z - 1.0
}

pub struct Calibration {
    pub wheel: LoadWheel,
    pub radius_m: f64,
    pub mean: Sample,
}

impl Calibration {
    pub fn replay(&self, rate: u32, speed_mps: f64, case: SlipCase) -> GripRun {
        GripRun {
            wheel: self.wheel.clone(),
            pass: BeltPass::new(rate, speed_mps, Some((case, self.radius_m))),
        }
    }
}

pub struct Preparation {
    pub wheel: LoadWheel,
    settling: Option<LoadSettling>,
    pass: BeltPass,
}

impl Preparation {
    pub fn new(
        tyre: ElasticTyre,
        rate: u32,
        load_n: f64,
        speed_mps: f64,
    ) -> Result<Self, Box<dyn Error>> {
        let wheel = LoadWheel::rolling(tyre)?;
        let settling = Some(LoadSettling::new(&wheel, rate, load_n));
        Ok(Self {
            wheel,
            settling,
            pass: BeltPass::new(rate, speed_mps, None),
        })
    }

    pub fn advance(&mut self) -> Result<Option<Calibration>, String> {
        if let Some(settling) = &mut self.settling {
            let settled = settling.advance(&mut self.wheel)?.is_some();
            self.pass.read_wheel(&self.wheel)?;
            if settled {
                self.settling = None;
            }
            return Ok(None);
        }
        let Some(mean) = self.pass.advance(&mut self.wheel)? else {
            return Ok(None);
        };
        let radius_m = self.pass.speed_mps / mean.spin_rad_s;
        validate_force_balance(mean, self.wheel.target_load_n)?;
        if !radius_m.is_finite() || radius_m <= 0.0 {
            return Err(format!("invalid effective rolling radius {radius_m:.6} m"));
        }
        Ok(Some(Calibration {
            wheel: self.wheel.clone(),
            radius_m,
            mean,
        }))
    }

    pub fn is_loading(&self) -> bool {
        self.settling.is_some()
    }

    pub fn belt_velocity(&self) -> DVec3 {
        self.pass.belt_velocity
    }
}

pub struct GripRun {
    pub wheel: LoadWheel,
    pass: BeltPass,
}

impl GripRun {
    pub fn advance(&mut self) -> Result<Option<Sample>, String> {
        self.pass.advance(&mut self.wheel)
    }

    pub fn step_count(&self) -> u32 {
        self.pass.steps
    }

    pub fn belt_velocity(&self) -> DVec3 {
        self.pass.belt_velocity
    }

    pub fn latest(&self) -> Sample {
        self.pass.latest
    }
}

struct BeltPass {
    rate: u32,
    speed_mps: f64,
    target: Option<(SlipCase, f64)>,
    steps: u32,
    start_spin: f64,
    belt_velocity: DVec3,
    latest: Sample,
    turns: RollingTurns,
    last_unsteady: Option<String>,
}

impl BeltPass {
    fn new(rate: u32, speed_mps: f64, target: Option<(SlipCase, f64)>) -> Self {
        Self {
            rate,
            speed_mps,
            target,
            steps: 0,
            start_spin: 0.0,
            belt_velocity: DVec3::ZERO,
            latest: Sample::default(),
            turns: RollingTurns::default(),
            last_unsteady: None,
        }
    }

    fn advance(&mut self, wheel: &mut LoadWheel) -> Result<Option<Sample>, String> {
        if self.steps == 0 {
            self.start_spin = wheel.tyre.rim().angular_velocity.x;
        }
        self.steps += 1;
        let dt = 1.0 / f64::from(self.rate);
        let fraction = (f64::from(self.steps) * 2.0 * dt).min(1.0);
        let straight = -self.speed_mps * DVec3::Z;
        let spin = if let Some((case, radius)) = self.target {
            let end = case.belt(self.speed_mps);
            self.belt_velocity = straight.lerp(end, fraction);
            let end_spin = case.commanded_spin(end, radius);
            Some(self.start_spin + fraction * (end_spin - self.start_spin))
        } else {
            self.belt_velocity = straight * fraction;
            None
        };
        let floor = loaded_tyre::floor(true)
            .with_surface_velocity(self.belt_velocity)
            .map_err(|error| error.to_string())?;
        wheel.tyre.set_floor(Some(floor));
        wheel
            .step_with_spin(dt, spin)
            .map_err(|error| error.to_string())?;
        self.read_wheel(wheel)?;

        let load = wheel.target_load_n;
        let free = self.target.is_none();
        if self.steps > self.rate + self.rate / 2
            && let Some(mean) = self.turns.record(self.latest, dt, |mean, prior| {
                let support_error = (mean.floor_force_n.y - load).abs()
                    .max((prior.floor_force_n.y - load).abs());
                let road_change = (mean.floor_force_n - prior.floor_force_n).xz().length();
                let axle_change = (mean.axle_force_n - prior.axle_force_n).xz().length();
                let spin_change = (mean.spin_rad_s - prior.spin_rad_s).abs();
                self.last_unsteady = if support_error > 0.02 * load {
                    Some(format!("turn support missed target by {support_error:.1} N"))
                } else if road_change > 0.01 * load || axle_change > 0.01 * load {
                    Some(format!("road and axle force means changed by {road_change:.1} and {axle_change:.1} N between turns"))
                } else if free && spin_change > 0.005 * mean.spin_rad_s {
                    Some(format!("spin changed by {spin_change:.3} rad/s between turns"))
                } else {
                    None
                };
                self.last_unsteady.is_none()
            })
        {
            return Ok(Some(mean));
        }
        if self.steps >= ROLL_TIMEOUT_S * self.rate {
            return Err(format!(
                "did not settle within {ROLL_TIMEOUT_S} s ({}, latest support {:.0} N, spin {:.3} rad/s)",
                self.last_unsteady
                    .as_deref()
                    .unwrap_or("fewer than two complete turns"),
                self.latest.floor_force_n.y,
                self.latest.spin_rad_s,
            ));
        }
        Ok(None)
    }

    fn read_wheel(&mut self, wheel: &LoadWheel) -> Result<(), String> {
        let step = wheel.last;
        let rim = wheel.tyre.rim();
        let sample = Sample {
            spin_rad_s: rim.angular_velocity.x,
            slip_angle_rad: (-self.belt_velocity.x).atan2(-self.belt_velocity.z),
            floor_force_n: step.floor_tangential_force.with_y(step.floor_force_n),
            axle_force_n: step.rim_force,
        };
        if !sample.spin_rad_s.is_finite()
            || !sample.floor_force_n.is_finite()
            || !sample.axle_force_n.is_finite()
            || !step.rim_torque.is_finite()
        {
            return Err("nonfinite force, torque or spin".into());
        }
        if rim.centre.y <= BEAD_RADIUS_M {
            return Err(format!(
                "rim clearance {:.4} m",
                rim.centre.y - BEAD_RADIUS_M
            ));
        }
        let limit = FRICTION * step.floor_force_n;
        let magnitude = step.floor_tangential_force.length();
        if magnitude > limit + 64.0 * f64::EPSILON * limit.max(1.0) {
            return Err(format!(
                "horizontal road force {magnitude:.3} N exceeds friction limit {limit:.3} N"
            ));
        }
        self.latest = sample;
        Ok(())
    }
}

pub fn validate_force_balance(mean: Sample, load_n: f64) -> Result<(), String> {
    let support = mean.floor_force_n.y;
    let difference = (mean.floor_force_n - mean.axle_force_n).xz().length();
    if (support - load_n).abs() > 0.02 * load_n {
        return Err(format!(
            "mean support {support:.1} N differs from {load_n:.1} N by more than 2%"
        ));
    }
    if difference > 0.02 * load_n {
        return Err(format!(
            "mean horizontal road and axle forces differ by {difference:.1} N"
        ));
    }
    Ok(())
}

pub fn validate_case(
    mean: Sample,
    calibration: &Calibration,
    case: SlipCase,
) -> Result<(), String> {
    validate_force_balance(mean, calibration.wheel.target_load_n)?;
    for (name, force, reference) in [
        ("road", mean.floor_force_n, calibration.mean.floor_force_n),
        ("axle", mean.axle_force_n, calibration.mean.axle_force_n),
    ] {
        if case.percent != 0 && (force.z - reference.z) * f64::from(case.percent) <= 0.0 {
            return Err(format!(
                "{name} longitudinal force {:.1} N disagrees with signed slip (reference {:.1} N)",
                force.z, reference.z
            ));
        }
        if case.angle_deg != 0 && force.x * f64::from(case.angle_deg) >= 0.0 {
            return Err(format!(
                "{name} lateral force {:.1} N does not oppose lateral slip",
                force.x
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::loaded_tyre::RATES;

    #[test]
    fn case_matrix_contains_all_nineteen_unique_points() {
        let actual = SlipCase::ALL.map(|case| (case.percent, case.angle_deg));
        let mut expected = vec![(0, 0)];
        expected.extend([-20, -10, -5, -2, 2, 5, 10, 20].map(|slip| (slip, 0)));
        expected.extend([-10, -5, -2, 2, 5, 10].map(|angle| (0, angle)));
        for slip in [-10, 10] {
            expected.extend([-5, 5].map(|angle| (slip, angle)));
        }
        assert_eq!(actual.len(), expected.len());
        assert!(expected.iter().all(|point| actual.contains(point)));
    }

    #[test]
    fn slip_reconstruction_uses_the_longitudinal_part_of_belt_velocity() {
        for case in SlipCase::ALL {
            let belt = case.belt(8.0);
            let radius = 0.325;
            let spin = case.commanded_spin(belt, radius);
            assert!((slip_ratio(spin, belt, radius) - case.ratio()).abs() < 1e-12);
            assert!(
                ((-belt.x).atan2(-belt.z).to_degrees() - f64::from(case.angle_deg)).abs() < 1e-12
            );
        }
    }

    #[test]
    fn replay_from_a_saved_calibration_repeats_the_same_motion_and_forces() {
        let rate = RATES[0];
        let tyre = loaded_tyre::prepare(rate, true).unwrap();
        let mut prep = Preparation::new(tyre, rate, NOMINAL_LOAD_N, 8.0).unwrap();
        let calibration = loop {
            if let Some(calibration) = prep.advance().unwrap() {
                break calibration;
            }
        };
        let run = || {
            let mut replay = calibration.replay(rate, 8.0, SlipCase::BENCHMARK);
            let mean = loop {
                if let Some(mean) = replay.advance().unwrap() {
                    break mean;
                }
            };
            validate_case(mean, &calibration, SlipCase::BENCHMARK).unwrap();
            (
                replay.step_count(),
                replay.wheel.tyre.rim(),
                mean.floor_force_n,
                mean.axle_force_n,
            )
        };
        assert_eq!(run(), run());
    }
}

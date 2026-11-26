use std::{error::Error, ops::RangeInclusive};

use bevy::math::DVec3;
use vehicle_physics::{
    beam_damage::BeamDamageConfig,
    chassis::{BeamCoefficients, Chassis, ChassisBeamSpec, ChassisNode, WallFace},
};

const RATE: usize = 3840;
const DT: f64 = 1.0 / RATE as f64;
const WALL_Z_M: f64 = 1.0;
const WALL_Y_M: RangeInclusive<f64> = -1.0..=2.0;
const QUIET_STEPS: usize = RATE / 4;
const QUIET_RANGE_M: f64 = 0.00025;
const TIMEOUT_STEPS: usize = 8 * RATE;
const NOSE_BEAMS: RangeInclusive<usize> = 18..=33;

#[derive(Clone, Copy)]
enum Impact {
    Light,
    Hard,
    Left,
    Right,
}

impl Impact {
    fn name(self) -> &'static str {
        match self {
            Self::Light => "Light frontal",
            Self::Hard => "Hard frontal",
            Self::Left => "Left offset",
            Self::Right => "Right offset (mirror)",
        }
    }

    fn speed_mps(self) -> f64 {
        match self {
            Self::Light => 1.0,
            _ => 8.0,
        }
    }

    fn wall_x_m(self) -> RangeInclusive<f64> {
        match self {
            Self::Light | Self::Hard => -3.0..=3.0,
            Self::Left => 0.4..=1.4,
            Self::Right => -1.4..=-0.4,
        }
    }
}

pub fn report() -> Result<(), Box<dyn Error>> {
    println!("Soft-body chassis | 12 nodes | 50 beams | 1,080 kg | {RATE} substeps/s");
    let light = replay(Impact::Light)?;
    let hard = replay(Impact::Hard)?;
    let left = replay(Impact::Left)?;
    let right = replay(Impact::Right)?;

    if light.rebound_mps <= 0.0
        || light.peak_m[0].min(light.peak_m[1]) <= 0.001
        || light.yielded != 0
        || light.residual_m[..2]
            .iter()
            .any(|crush| crush.abs() >= 0.001)
    {
        return Err("light strike failed to rebound and recover elastically".into());
    }
    if hard.yielded_nose == 0
        || hard.residual_m[0].min(hard.residual_m[1]) < 0.010
        || hard.residual_m[2].abs().max(hard.residual_m[3].abs())
            >= hard.residual_m[0].min(hard.residual_m[1])
    {
        return Err(
            "hard strike failed to leave more permanent crush in the nose than the cabin".into(),
        );
    }
    if left.residual_m[0] - left.residual_m[1] < 0.005
        || right.residual_m[1] - right.residual_m[0] < 0.005
        || !left.passed_beside
        || !right.passed_beside
        || left.release_yaw_rad.abs() < 0.02
        || (left.release_yaw_rad + right.release_yaw_rad).abs() > 1e-6
        || (left.residual_m[0] - right.residual_m[1]).abs() > 1e-6
        || (left.residual_m[1] - right.residual_m[0]).abs() > 1e-6
    {
        return Err("offset strikes failed to mirror crush and cabin yaw".into());
    }
    println!(
        "Elastic recovery, permanent crush, mirrored yaw, momentum balance, and reset replay passed."
    );
    Ok(())
}

fn replay(impact: Impact) -> Result<CrashResult, Box<dyn Error>> {
    let wall = WallFace::new(WALL_Z_M, impact.wall_x_m(), WALL_Y_M)?;
    let mut chassis = fixture(impact.speed_mps())?;
    let result = run(&mut chassis, &wall, impact.wall_x_m())
        .map_err(|error| format!("{}: {error}", impact.name()))?;
    print_result(impact, result);
    chassis.reset();
    let repeated = run(&mut chassis, &wall, impact.wall_x_m())?;
    if repeated != result {
        return Err(format!("{} measurements changed after reset", impact.name()).into());
    }
    Ok(result)
}

fn fixture(speed_mps: f64) -> Result<Chassis, Box<dyn Error>> {
    // Front to rear. Each section is left lower, left upper, right lower, right upper.
    let sections = [(0.85, 40.0), (0.0, 100.0), (-1.40, 130.0)];
    let nodes = sections
        .into_iter()
        .flat_map(|(z, mass_kg)| {
            [(0.75, 0.30), (0.75, 0.85), (-0.75, 0.30), (-0.75, 0.85)].map(|(x, y)| ChassisNode {
                mass_kg,
                position_m: DVec3::new(x, y, z),
                velocity_mps: DVec3::Z * speed_mps,
            })
        })
        .collect();
    let nose = BeamCoefficients {
        stiffness: 330_000.0,
        damping: 3_500.0,
    };
    let cabin = BeamCoefficients {
        stiffness: 2_600_000.0,
        damping: 10_400.0,
    };
    let nose_yield = BeamDamageConfig::yield_only(0.08, 0.06)?;
    let cabin_yield = BeamDamageConfig::yield_only(0.08, 0.08)?;
    let mut beams = Vec::with_capacity(50);
    for section in [0, 4, 8] {
        for corner in section..section + 4 {
            for other in corner + 1..section + 4 {
                beams.push(ChassisBeamSpec {
                    endpoints: [corner, other],
                    coefficients: cabin,
                    material: Some(cabin_yield),
                });
            }
        }
    }
    for (front, coefficients, material) in [(0, nose, nose_yield), (4, cabin, cabin_yield)] {
        for corner in front..front + 4 {
            for other in front + 4..front + 8 {
                beams.push(ChassisBeamSpec {
                    endpoints: [corner, other],
                    coefficients,
                    material: Some(material),
                });
            }
        }
    }
    Ok(Chassis::new(nodes, beams)?)
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct CrashResult {
    approach_mps: f64,
    rebound_mps: f64,
    // Left and right nose, then left and right cabin.
    peak_m: [f64; 4],
    residual_m: [f64; 4],
    release_yaw_rad: f64,
    impulse_z_ns: f64,
    momentum_error_ns: f64,
    max_penetration_m: f64,
    max_trial_depth_m: f64,
    yielded: usize,
    yielded_nose: usize,
    passed_beside: bool,
}

fn run(
    chassis: &mut Chassis,
    wall: &WallFace,
    wall_x: RangeInclusive<f64>,
) -> Result<CrashResult, Box<dyn Error>> {
    let initial_lengths = lengths(chassis.nodes());
    let mass_kg = chassis.nodes().iter().map(|node| node.mass_kg).sum::<f64>();
    let initial_momentum = momentum(chassis.nodes());
    let mut peak_m = [0.0_f64; 4];
    let mut impulse_z_ns = 0.0;
    let mut release_yaw_rad = 0.0;
    let mut max_penetration_m = 0.0_f64;
    let mut max_trial_depth_m = 0.0_f64;
    let mut passed_beside = false;
    let mut quiet = QuietWindow::default();

    for _ in 0..TIMEOUT_STEPS {
        let before = std::array::from_fn::<_, 12, _>(|i| chassis.nodes()[i].position_m);
        let reaction = chassis.step(DT, Some(wall))?;
        if reaction.struck_nodes > 0 {
            impulse_z_ns += reaction.impulse_z_ns;
            release_yaw_rad = yaw(chassis.nodes());
        }
        max_trial_depth_m = max_trial_depth_m.max(reaction.max_trial_depth_m);

        // Independently check committed positions. A node passing beyond the
        // plane is allowed when its crossing point lies beyond a wall edge.
        for (old, new) in before.into_iter().zip(chassis.nodes()) {
            let end = new.position_m;
            if old.z <= WALL_Z_M && end.z > WALL_Z_M {
                let crossing = old.lerp(end, (WALL_Z_M - old.z) / (end.z - old.z));
                if wall_x.contains(&crossing.x) && WALL_Y_M.contains(&crossing.y) {
                    max_penetration_m = max_penetration_m.max(end.z - WALL_Z_M);
                } else {
                    passed_beside = true;
                }
            }
        }

        let current = lengths(chassis.nodes());
        let crush = std::array::from_fn(|i| initial_lengths[i] - current[i]);
        for (peak, shortening) in peak_m.iter_mut().zip(crush) {
            *peak = peak.max(shortening);
        }
        let still_on_wall = chassis.nodes().iter().any(|node| {
            let p = node.position_m;
            (p.z - WALL_Z_M).abs() < 1e-8 && wall_x.contains(&p.x) && WALL_Y_M.contains(&p.y)
        });
        if reaction.struck_nodes > 0 || still_on_wall {
            quiet = QuietWindow::default();
            continue;
        }
        if impulse_z_ns == 0.0 {
            continue;
        }
        if let Some(residual_m) = quiet.observe(crush) {
            let final_momentum = momentum(chassis.nodes());
            let imbalance = final_momentum - initial_momentum - DVec3::Z * impulse_z_ns;
            let momentum_error_ns = imbalance.length();
            if momentum_error_ns > 1e-7 * initial_momentum.length() {
                return Err(format!(
                    "wall impulse disagrees with momentum change by {momentum_error_ns:.6} N·s"
                )
                .into());
            }
            if max_penetration_m > 1e-9 {
                return Err(
                    format!("a node passed through the wall by {max_penetration_m:.6} m").into(),
                );
            }
            return Ok(CrashResult {
                approach_mps: initial_momentum.z / mass_kg,
                rebound_mps: -final_momentum.z / mass_kg,
                peak_m,
                residual_m,
                release_yaw_rad,
                impulse_z_ns,
                momentum_error_ns,
                max_penetration_m,
                max_trial_depth_m,
                yielded: chassis.beam_damage().filter(|beam| beam.yielded()).count(),
                yielded_nose: chassis
                    .beam_damage()
                    .enumerate()
                    .filter(|(index, beam)| NOSE_BEAMS.contains(index) && beam.yielded())
                    .count(),
                passed_beside,
            });
        }
    }
    Err("the chassis did not separate and settle within eight seconds".into())
}

fn momentum(nodes: &[ChassisNode]) -> DVec3 {
    nodes
        .iter()
        .map(|node| node.mass_kg * node.velocity_mps)
        .sum()
}

fn lengths(nodes: &[ChassisNode]) -> [f64; 4] {
    [0, 2, 4, 6].map(|i| {
        let distance = |a: usize, b: usize| nodes[a].position_m.distance(nodes[b].position_m);
        0.5 * (distance(i, i + 4) + distance(i + 1, i + 5))
    })
}

fn yaw(nodes: &[ChassisNode]) -> f64 {
    let centre = |first: usize| {
        nodes[first..first + 4]
            .iter()
            .map(|node| node.position_m)
            .sum::<DVec3>()
            * 0.25
    };
    let forward = centre(4) - centre(8);
    forward.x.atan2(forward.z)
}

struct QuietWindow {
    count: usize,
    min_m: [f64; 4],
    max_m: [f64; 4],
    sum_m: [f64; 4],
}

impl Default for QuietWindow {
    fn default() -> Self {
        Self {
            count: 0,
            min_m: [f64::INFINITY; 4],
            max_m: [f64::NEG_INFINITY; 4],
            sum_m: [0.0; 4],
        }
    }
}

impl QuietWindow {
    fn observe(&mut self, crush: [f64; 4]) -> Option<[f64; 4]> {
        for (i, shortening) in crush.into_iter().enumerate() {
            self.min_m[i] = self.min_m[i].min(shortening);
            self.max_m[i] = self.max_m[i].max(shortening);
            self.sum_m[i] += shortening;
        }
        self.count += 1;
        if self.count != QUIET_STEPS {
            return None;
        }
        let stable = (0..4).all(|i| self.max_m[i] - self.min_m[i] <= QUIET_RANGE_M);
        let average = self.sum_m.map(|sum| sum / self.count as f64);
        *self = Self::default();
        stable.then_some(average)
    }
}

fn print_result(impact: Impact, result: CrashResult) {
    let peak = result.peak_m.map(|m| m * 1_000.0);
    let residual = result.residual_m.map(|m| m * 1_000.0);
    println!(
        "{} | approach {:.3} m/s | rebound {:.3} m/s | release yaw {:+.3}°",
        impact.name(),
        result.approach_mps,
        result.rebound_mps,
        result.release_yaw_rad.to_degrees()
    );
    println!(
        "  Shortening [left, right] in mm | nose peak {:.2?}, residual {:.2?} | cabin peak {:.2?}, residual {:.2?}",
        &peak[..2],
        &residual[..2],
        &peak[2..],
        &residual[2..],
    );
    println!(
        "  Yielded beams {} ({} in nose) | wall impulse {:+.3} N·s along Z | momentum error {:.2e} N·s",
        result.yielded, result.yielded_nose, result.impulse_z_ns, result.momentum_error_ns,
    );
    println!(
        "  Wall penetration {:.4} mm | largest corrected trial depth {:.2} mm",
        result.max_penetration_m * 1_000.0,
        result.max_trial_depth_m * 1_000.0,
    );
}

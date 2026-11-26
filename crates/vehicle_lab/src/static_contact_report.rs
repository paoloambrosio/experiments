use std::{error::Error, sync::Arc};

use bevy::math::{DQuat, DVec3};
use vehicle_physics::{
    chassis::{Chassis, ChassisCollision, ChassisNode},
    elastic_tyre::{ElasticTyre, ElasticTyreConfig},
    mounted_tyre::{ContactParameters, MountedTyre, RimMotion},
    static_collision::{ContactFeature, StaticCollisionMesh, StaticContact},
    triangle_road::TriangleRoad,
    tyre::{TyreConfig, TyreTopology},
};

use crate::chassis_crash_report;

const RATE: usize = 3840;
const DT: f64 = 1.0 / RATE as f64;
const BUMPER: [[usize; 3]; 2] = [[0, 1, 3], [0, 3, 2]];
const FACE: [[usize; 3]; 2] = [[0, 1, 2], [0, 2, 3]];
const POST_FRONT: f64 = 1.0;
const POST_HALF_WIDTH: f64 = 0.06;
const POST_TOP: f64 = 0.65;
const POST_BOTTOM: f64 = 0.0;

pub fn report() -> Result<(), Box<dyn Error>> {
    let mut chassis = chassis_crash_report::fixture(8.0)?;
    let surface = ChassisCollision::new(&chassis, (0..12).collect(), BUMPER.to_vec())?;
    let centred = post(0.0)?;
    let strike = strike_post(&mut chassis, &surface, &centred)?;
    chassis.reset();
    if strike != strike_post(&mut chassis, &surface, &centred)? {
        return Err("centred post changed after reset".into());
    }
    if !strike.first.is_some_and(|hit| {
        matches!(
            hit.feature,
            ContactFeature::Panel(_) | ContactFeature::PanelEdge { .. }
        )
    }) || !strike.nodes_cleared
        || !strike.released
        || strike.impulse_ns.z >= -1.0
        || strike.nose_shortening_m < 0.005
        || strike.nose_range_m > 0.00025
        || strike.penetration_m > 1e-5
        || strike.momentum_error_ns > 1e-7
    {
        return Err(format!("centred post failed: {strike:?}").into());
    }
    chassis.reset();
    let miss = strike_post(&mut chassis, &surface, &post(0.90)?)?;
    if miss.first.is_some() || miss.impulse_ns != DVec3::ZERO {
        return Err(format!("offset post should miss: {miss:?}").into());
    }
    let first = strike.first.unwrap();
    println!(
        "Centred post | first {:?} on triangle {} at {:.3?} m | impulse {:.3?} N·s",
        first.feature,
        first.triangle,
        first.position_m.to_array(),
        strike.impulse_ns.to_array()
    );
    println!(
        "  Nose shortening {:.3} mm | final quarter-second variation {:.3} mm | penetration {:.6} mm | momentum error {:.2e} N·s",
        strike.nose_shortening_m * 1000.0,
        strike.nose_range_m * 1000.0,
        strike.penetration_m * 1000.0,
        strike.momentum_error_ns
    );
    println!("Offset post | zero impulse");
    wheel_first()?;
    Ok(())
}

fn post(centre_x: f64) -> Result<StaticCollisionMesh, Box<dyn Error>> {
    let xl = centre_x - POST_HALF_WIDTH;
    let xr = centre_x + POST_HALF_WIDTH;
    let vertices = [POST_FRONT, POST_FRONT + 0.2]
        .into_iter()
        .flat_map(|z| {
            [
                (xl, POST_BOTTOM),
                (xr, POST_BOTTOM),
                (xr, POST_TOP),
                (xl, POST_TOP),
            ]
            .map(|(x, y)| DVec3::new(x, y, z))
        })
        .collect::<Vec<_>>();
    Ok(StaticCollisionMesh::new(
        &vertices,
        &[
            [0, 1, 2],
            [0, 2, 3],
            [4, 6, 5],
            [4, 7, 6],
            [0, 4, 5],
            [0, 5, 1],
            [1, 5, 6],
            [1, 6, 2],
            [2, 6, 7],
            [2, 7, 3],
            [3, 7, 4],
            [3, 4, 0],
        ],
    )?)
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct PostResult {
    first: Option<StaticContact>,
    nodes_cleared: bool,
    released: bool,
    impulse_ns: DVec3,
    penetration_m: f64,
    nose_shortening_m: f64,
    nose_range_m: f64,
    momentum_error_ns: f64,
}

fn strike_post(
    chassis: &mut Chassis,
    surface: &ChassisCollision,
    world: &StaticCollisionMesh,
) -> Result<PostResult, Box<dyn Error>> {
    let momentum = |nodes: &[ChassisNode]| -> DVec3 {
        nodes
            .iter()
            .map(|node| node.mass_kg * node.velocity_mps)
            .sum()
    };
    let length = |nodes: &[ChassisNode]| {
        (0..4)
            .map(|i| nodes[i].position_m.distance(nodes[i + 4].position_m))
            .sum::<f64>()
            / 4.0
    };
    let initial_length = length(chassis.nodes());
    let initial_p = momentum(chassis.nodes());
    let mut first = None;
    let mut nodes_cleared = true;
    let mut impulse_ns = DVec3::ZERO;
    let mut penetration_m = 0.0_f64;
    let mut last_contact = None;
    let mut low = f64::INFINITY;
    let mut high = f64::NEG_INFINITY;
    let mut sum = 0.0;
    let duration = 3 * RATE;
    let quiet_steps = RATE / 4;
    for step in 0..duration {
        let reaction = chassis
            .step_with_static_world(DT, surface, world)
            .map_err(|error| format!("post substep {step}: {error}"))?;
        let nodes = chassis.nodes();
        if let Some(contact) = reaction.first {
            first.get_or_insert(contact);
            last_contact = Some(step);
            nodes_cleared &= nodes[..4]
                .iter()
                .all(|node| node.position_m.x.abs() > POST_HALF_WIDTH)
                && !matches!(contact.feature, ContactFeature::Node(_));
        }
        impulse_ns += reaction.impulse_ns;
        if first.is_some() {
            penetration_m = penetration_m.max(bumper_depth(nodes));
        }
        if step >= duration - quiet_steps {
            let crush = initial_length - length(nodes);
            low = low.min(crush);
            high = high.max(crush);
            sum += crush;
        }
    }
    Ok(PostResult {
        first,
        nodes_cleared,
        impulse_ns,
        penetration_m,
        released: last_contact.is_some_and(|step| step < duration - quiet_steps),
        nose_shortening_m: sum / quiet_steps as f64,
        nose_range_m: high - low,
        momentum_error_ns: (momentum(chassis.nodes()) - initial_p - impulse_ns).length(),
    })
}

// Clip the committed bumper to the front face's footprint, independently of
// the solver's reported points and impulses.
fn bumper_depth(nodes: &[ChassisNode]) -> f64 {
    BUMPER
        .into_iter()
        .map(|panel| {
            let mut polygon = panel.map(|i| nodes[i].position_m).to_vec();
            for (axis, offset, sign) in [
                (0, -POST_HALF_WIDTH, 1.0),
                (0, POST_HALF_WIDTH, -1.0),
                (1, POST_BOTTOM, 1.0),
                (1, POST_TOP, -1.0),
            ] {
                let old = std::mem::take(&mut polygon);
                for (&a, &b) in old.iter().zip(old.iter().cycle().skip(1)) {
                    let da = sign * (a[axis] - offset);
                    let db = sign * (b[axis] - offset);
                    if da >= 0.0 {
                        polygon.push(a);
                    }
                    if (da >= 0.0) != (db >= 0.0) {
                        polygon.push(a.lerp(b, da / (da - db)));
                    }
                }
            }
            polygon
                .into_iter()
                .map(|p| p.z - POST_FRONT)
                .fold(0.0, f64::max)
        })
        .fold(0.0, f64::max)
}

fn wheel_first() -> Result<(), Box<dyn Error>> {
    const BARE_RIM_KG: f64 = 16.0;
    let floor = [
        DVec3::new(-2.0, 0.0, -2.0),
        DVec3::new(2.0, 0.0, -2.0),
        DVec3::new(2.0, 0.0, 2.0),
        DVec3::new(-2.0, 0.0, 2.0),
    ];
    let road = Arc::new(TriangleRoad::new(&floor, &FACE, 0.05)?);
    let road_contact = ContactParameters::new(80_000.0, 90.0)?.with_friction(0.8, 0.1)?;
    let gravity = -9.81 * DVec3::Y;
    let carcass = ElasticTyre::new(
        TyreTopology::generate(&TyreConfig::default())?,
        ElasticTyreConfig::default(),
    )?;
    let parked = RimMotion {
        centre: DVec3::Y * 0.31,
        ..Default::default()
    };
    let mut loaded = MountedTyre::new(carcass, parked, None)?;
    loaded.set_triangle_road(road.clone(), road_contact);
    for _ in 0..2 * RATE {
        loaded.step(DT, gravity)?;
    }
    let wall_x = loaded.positions().iter().map(|p| p.x).fold(0.0, f64::max) + 0.0015;
    let wall = [
        DVec3::new(wall_x, 0.12, -0.4),
        DVec3::new(wall_x, 0.12, 0.4),
        DVec3::new(wall_x, 0.65, 0.4),
        DVec3::new(wall_x, 0.65, -0.4),
    ];
    let mesh = Arc::new(StaticCollisionMesh::new(
        &[floor, wall].concat(),
        &[FACE[0], FACE[1], [4, 5, 6], [4, 6, 7]],
    )?);
    let mut rim = RimMotion {
        linear_velocity: DVec3::new(1.5, 0.0, 0.2),
        ..parked
    };
    let mut wheel = MountedTyre::new(loaded.into_fixed_bead_tyre()?, rim, None)?;
    wheel.set_triangle_road(road, road_contact);
    let mut control = wheel.clone();
    let mut floor_only = wheel.clone();
    floor_only.set_obstacles(
        Arc::new(StaticCollisionMesh::new(&floor, &FACE)?),
        BARE_RIM_KG,
    )?;
    wheel.set_obstacles(mesh.clone(), BARE_RIM_KG)?;
    let (mass, inertia) = wheel.rim_mass_properties(BARE_RIM_KG)?;
    let mut body = Chassis::new(
        vec![ChassisNode {
            mass_kg: 100.0,
            position_m: DVec3::new(0.08, 0.4, 0.0),
            velocity_mps: rim.linear_velocity,
        }],
        vec![],
    )?;
    let body_surface = ChassisCollision::new(&body, vec![0], vec![])?;
    let mut first = None;
    let mut body_first = None;
    let mut impulse = DVec3::ZERO;
    let mut rim_impulse = DVec3::ZERO;
    let mut rim_torque_impulse = DVec3::ZERO;
    let mut penetration = 0.0_f64;
    for step in 0..96 {
        let expected = control.step(DT, gravity)?;
        let floor_step = floor_only.step(DT, gravity)?;
        let observed = wheel
            .step(DT, gravity)
            .map_err(|error| format!("wheel substep {step}: {error}"))?;
        let ground = wheel.triangle_reaction();
        let ground_baseline = control.triangle_reaction();
        if floor_only.obstacle_reaction().contacts != 0
            || floor_only.positions() != control.positions()
            || floor_only.triangle_reaction().normal_force != ground_baseline.normal_force
            || floor_only.triangle_reaction().tangential_force != ground_baseline.tangential_force
            || floor_step.rim_force != expected.rim_force
        {
            return Err("a duplicate floor face changed road load or grip".into());
        }
        let contact = wheel.obstacle_reaction();
        if let Some(hit) = contact.first {
            first.get_or_insert((step, hit, ground.normal_force.y));
        }
        if body
            .step_with_static_world(DT, &body_surface, &mesh)?
            .first
            .is_some()
        {
            body_first.get_or_insert(step);
        }
        impulse += contact.impulse_ns;
        rim_impulse += observed.rim_force * DT;
        rim_torque_impulse += observed.rim_torque * DT;
        penetration = penetration.max(contact.max_residual_depth_m);
        rim.linear_velocity += (observed.rim_force / mass).with_y(0.0) * DT;
        rim.angular_velocity +=
            rim.rotation * ((rim.rotation.conjugate() * observed.rim_torque) / inertia) * DT;
        rim.centre += rim.linear_velocity * DT;
        rim.rotation =
            (DQuat::from_scaled_axis(rim.angular_velocity * DT) * rim.rotation).normalize();
        wheel.sync_rim(rim)?;
        if body_first.is_some() {
            break;
        }
    }
    let (step, hit, load) = first.ok_or("exposed tyre missed the wall")?;
    let body_step = body_first.ok_or("chassis fixture missed the wall")?;
    if step >= body_step
        || load <= 0.0
        || impulse.x > -1.0
        || rim_impulse.x >= -0.01
        || !rim_torque_impulse.is_finite()
        || penetration > 0.002
    {
        return Err(format!(
            "wheel-first check failed at {step} / {body_step}, load {load:.3} N, +             impulse {impulse:.3?}, rim {rim_impulse:.3?}, depth {penetration:.6}"
        ).into());
    }
    println!(
        "Wheel first | {:?} on triangle {} at {:.3} ms | chassis at {:.3} ms | floor load {:.1} N",
        hit.feature,
        hit.triangle,
        step as f64 * DT * 1000.0,
        body_step as f64 * DT * 1000.0,
        load
    );
    println!(
        "  Obstacle impulse {:.3?} N·s | rim {:.3?} N·s, {:.3?} N·m·s | residual contact depth {:.3} mm",
        impulse.to_array(),
        rim_impulse.to_array(),
        rim_torque_impulse.to_array(),
        penetration * 1000.0
    );
    Ok(())
}

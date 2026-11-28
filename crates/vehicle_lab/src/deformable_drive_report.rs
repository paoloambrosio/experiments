use std::{error::Error, ops::Range, sync::Arc, time::Instant};

use avian3d::prelude::SubstepSchedule;
use bevy::{math::DVec3, prelude::*};
use vehicle_assets::ror::reference;
use vehicle_physics::{
    avian_tyre::{AvianTyre, TyreSubstepComplete},
    deformable_vehicle::DeformableBody,
    static_collision::StaticReaction,
};
use vehicle_scene::{
    body_skin::BodySkin,
    crash::{CrashSetup, DeformableSession},
    drive::{
        AlignmentMode, BENCHMARK_TICKS, DriveSetup, DriverInput, TICKS_PER_SECOND, benchmark_input,
    },
    placement,
};

mod dense_idle;

const THROTTLE: DriverInput = DriverInput {
    throttle: 1.0,
    brake: 0.0,
    steer: 0.0,
};

pub(crate) fn report() -> Result<(), Box<dyn Error>> {
    let terrain = reference::terrain()?;
    dense_idle::run(&terrain)?;
    let vehicle = reference::vehicle()?;
    let asset = &vehicle.vehicle;
    let drive = DriveSetup::new(&terrain, &vehicle, AlignmentMode::Authored)?;
    let CrashSetup {
        approach,
        vehicle: setup,
    } = CrashSetup::new(&drive, asset, "wall")?;
    let rate = setup.substeps * TICKS_PER_SECOND;
    let skin = BodySkin::new(asset, &setup.imported.source)?;
    let mut session = setup.start(drive.start, |app| {
        app.add_systems(SubstepSchedule, trace_wall.after(TyreSubstepComplete));
    })?;
    session.reset()?;
    println!("Playable deformable drive | {rate} substeps/s");
    ordinary(&mut session, &skin, drive.weight_n())?;
    ramp(&mut session, &drive)?;

    session.reset()?;
    let fresh = session.frame();
    let commands = DriverInput {
        throttle: 1.0,
        steer: 0.25,
        ..default()
    };
    for _ in 0..TICKS_PER_SECOND {
        session.step(commands)?;
    }
    let fresh_trace = session.frame();

    side_strike(&mut session, &drive, approach, rate)?;
    session.reset()?;
    let reset = session.frame();
    if reset.nodes_m != fresh.nodes_m
        || !same_rims(&fresh.rims, &reset.rims, 1e-9)
        || session.velocity_mps() != DVec3::ZERO
        || session
            .body()
            .chassis()
            .beam_damage()
            .any(|beam| beam.permanent_length_change_m() != 0.0)
    {
        return Err("reset failed to restore the fresh assembly".into());
    }
    for _ in 0..TICKS_PER_SECOND {
        session.step(commands)?;
    }
    let repeat = session.frame();
    if !fresh_trace
        .nodes_m
        .iter()
        .zip(&repeat.nodes_m)
        .all(|(a, b)| a.abs_diff_eq(*b, 1e-6))
        || !same_rims(&fresh_trace.rims, &repeat.rims, 1e-6)
    {
        return Err("driving after reset diverged from the fresh run".into());
    }
    println!("Reset restored the start and matched a fresh one-second drive");
    crate::crash_report::run(&drive, asset, "wall")?;
    Ok(())
}

fn same_rims(
    a: &[vehicle_physics::mounted_tyre::RimMotion; 4],
    b: &[vehicle_physics::mounted_tyre::RimMotion; 4],
    tolerance: f64,
) -> bool {
    a.iter().zip(b).all(|(a, b)| {
        a.centre.abs_diff_eq(b.centre, tolerance)
            && a.linear_velocity.abs_diff_eq(b.linear_velocity, tolerance)
            && a.angular_velocity
                .abs_diff_eq(b.angular_velocity, tolerance)
            && a.rotation.dot(b.rotation).abs() >= 1.0 - tolerance
    })
}

fn ordinary(session: &mut DeformableSession, skin: &BodySkin, weight: f64) -> Result<(), String> {
    let mut physics = Vec::new();
    let mut skinning = Vec::new();
    let mut load = [0.0; 4];
    let mut before = Vec3::Z;
    let mut entry_speed = 0.0;
    let mut turn = 0.0;
    for tick in 0..BENCHMARK_TICKS {
        let start = Instant::now();
        session.step(benchmark_input(tick))?;
        if tick == 120
            && !session
                .wheel_controls()
                .iter()
                .any(|wheel| wheel.drive_nm > 0.0)
        {
            return Err("throttle missed its first physics tick".into());
        }
        if tick == 250
            && !session.wheel_controls()[..2]
                .iter()
                .all(|wheel| wheel.steer_rad > 0.0)
        {
            return Err("steering missed its first physics tick".into());
        }
        if tick == 310
            && !session
                .wheel_controls()
                .iter()
                .all(|wheel| wheel.brake_nm > 0.0 && wheel.drive_nm == 0.0)
        {
            return Err("brake priority missed its first physics tick".into());
        }
        physics.push(start.elapsed().as_secs_f64());
        let start = Instant::now();
        let shape = skin.sample(&session.frame().nodes_m)?;
        skinning.push(start.elapsed().as_secs_f64());
        if (90..120).contains(&tick) {
            for (mean, reaction) in load.iter_mut().zip(session.reactions()) {
                *mean += (reaction.normal_force.y + reaction.tangential_force.y) / 30.0;
            }
        }
        let heading = shape.pose.rotation * Vec3::Z;
        if tick == 249 {
            before = heading;
            if session.velocity_mps().z < 3.0 {
                return Err(format!(
                    "weak straight-line acceleration: {:?}",
                    session.velocity_mps()
                ));
            }
        }
        if tick == 309 {
            turn = before
                .cross(heading)
                .y
                .atan2(before.dot(heading))
                .to_degrees();
            entry_speed = speed(session);
        }
    }
    print_timing("Ordinary driving physics", &mut physics);
    print_timing("Ordinary driving skin sampling", &mut skinning);
    let stopped = speed(session);
    if load.iter().any(|force| *force < 0.02 * weight)
        || (load.iter().sum::<f64>() - weight).abs() > 0.1 * weight
        || turn < 2.0
        || stopped > 0.15
        || entry_speed < 2.0
    {
        return Err(format!(
            "ordinary driving failed: loads {load:.1?} N / {weight:.1} N, turn {turn:.2}°, brake from {entry_speed:.3} to {stopped:.3} m/s"
        ));
    }
    println!(
        "Ordinary driving | loads {load:.1?} N | turn {turn:.2}° | brake from {entry_speed:.3} to {stopped:.3} m/s"
    );
    Ok(())
}

fn speed(session: &DeformableSession) -> f64 {
    session.velocity_mps().with_y(0.0).length()
}

fn ramp(session: &mut DeformableSession, drive: &DriveSetup) -> Result<(), String> {
    let (_, faces) = drive
        .collision
        .objects
        .iter()
        .find(|(name, _)| name == "ramp")
        .ok_or("missing ramp collision placement")?;
    let ramp = &drive.collision.triangles[faces.clone()];
    let pose = drive.pose(Vec3::new(16.0, 3.0, 12.0), Quat::IDENTITY)?;
    session.spawn_at(pose, DVec3::ZERO, None)?;
    for _ in 0..2 * TICKS_PER_SECOND {
        session.step(default())?;
    }
    let rest = session.frame().rims.map(|rim| rim.centre.y);
    let mut rise = [0.0_f64; 4];
    for _ in 0..10 * TICKS_PER_SECOND {
        session.step(if speed(session) < 6.0 {
            THROTTLE
        } else {
            default()
        })?;
        let wheels = session.frame().rims;
        for (i, reaction) in session.reactions().into_iter().enumerate() {
            let point = wheels[i].centre.as_vec3();
            if let Some(height) = placement::support_below(ramp, point)
                && placement::support_below(&drive.collision.triangles, point) == Some(height)
                && reaction.normal_force.y > 0.0
            {
                rise[i] = rise[i].max(wheels[i].centre.y - rest[i]);
            }
        }
        if rise.iter().all(|dy| *dy > 0.05)
            && wheels
                .iter()
                .all(|rim| placement::support_below(ramp, rim.centre.as_vec3()).is_none())
        {
            println!("Ramp | loaded wheel rises {rise:.3?} m");
            return Ok(());
        }
    }
    Err(format!(
        "ramp traversal failed: loaded wheel rises {rise:.3?} m"
    ))
}

fn side_strike(
    session: &mut DeformableSession,
    drive: &DriveSetup,
    approach: vehicle_scene::crash::Approach,
    rate: u32,
) -> Result<(), String> {
    let rotation = approach.pose.rotation * Quat::from_rotation_y(std::f32::consts::FRAC_PI_2);
    // A sideways approach scrubs off speed quickly. Start closer than the frontal launch.
    let start = approach.pose.origin_m + 2.6 * approach.forward;
    let pose = drive.pose(start.with_y(approach.point.y), rotation)?;
    let forward = approach.forward.as_dvec3();
    let velocity = 5.0 * forward;
    let (body, rims) = session.spawn_at(pose, velocity, None)?;
    let target = session.obstacles.target.clone();
    let source = session.obstacles.source.clone();
    let tyre_source = session.obstacles.tyre_source.clone();
    let faces = drive.collision.triangles[target.clone()].to_vec();
    session.world_mut().insert_resource(WallTrace {
        body,
        rims,
        target,
        source,
        tyre_source,
        faces,
        forward,
        point: approach.point.as_dvec3(),
        steps: 0,
        chassis_hit: None,
        wheel_hits: [None; 4],
        transferred_ns: 0.0,
        penetration_m: 0.0,
    });
    let mut timings = Vec::new();
    for _ in 0..2 * TICKS_PER_SECOND {
        let start = Instant::now();
        session.step(default())?;
        timings.push(start.elapsed().as_secs_f64());
    }
    print_timing("Wheel-first impact physics", &mut timings);
    let trace = session.world_mut().remove_resource::<WallTrace>().unwrap();
    let hit = trace
        .wheel_hits
        .iter()
        .flatten()
        .min()
        .copied()
        .ok_or("lateral approach missed wall")?;
    println!(
        "Wheel-first wall strike | chassis {:?} | wheels {:?} | transferred {:.3} N·s | tyre penetration {:.3} mm",
        trace.chassis_hit,
        trace.wheel_hits,
        trace.transferred_ns,
        trace.penetration_m * 1000.0
    );
    if trace.steps != 2 * rate
        || trace.chassis_hit.is_some_and(|step| step <= hit)
        || trace.transferred_ns <= 0.05
        || trace.penetration_m > 0.002
    {
        return Err("wheel-first contact, mount transfer, or wall penetration check failed".into());
    }
    Ok(())
}

#[derive(Resource)]
struct WallTrace {
    body: Entity,
    rims: [Entity; 4],
    target: Range<usize>,
    source: Arc<[usize]>,
    tyre_source: Arc<[usize]>,
    faces: Vec<[Vec3; 3]>,
    forward: DVec3,
    point: DVec3,
    steps: u32,
    chassis_hit: Option<u32>,
    wheel_hits: [Option<u32>; 4],
    transferred_ns: f64,
    penetration_m: f64,
}

fn trace_wall(
    mut trace: Option<ResMut<WallTrace>>,
    bodies: Query<&DeformableBody>,
    tyres: Query<&AvianTyre>,
) {
    let Some(trace) = trace.as_deref_mut() else {
        return;
    };
    let body = bodies.get(trace.body).unwrap();
    trace.steps += 1;
    let hit = |reaction: StaticReaction, source: &[usize]| {
        reaction
            .first
            .is_some_and(|contact| trace.target.contains(&source[contact.triangle]))
    };
    if hit(body.obstacle_reaction(), &trace.source) {
        trace.chassis_hit.get_or_insert(trace.steps);
    }
    let first_wheel_hit = trace.wheel_hits.iter().flatten().min().copied();
    for (i, rim) in trace.rims.into_iter().enumerate() {
        let tyre = tyres.get(rim).unwrap().tyre();
        if hit(tyre.obstacle_reaction(), &trace.tyre_source) {
            trace.wheel_hits[i].get_or_insert(trace.steps);
        }
        if trace.wheel_hits[i].is_some()
            && trace.chassis_hit.is_none()
            && first_wheel_hit.is_none_or(|step| trace.steps - step <= 60)
        {
            let mount = body.mount_reactions()[i].impulse_ns;
            trace.transferred_ns = trace.transferred_ns.max(-mount.dot(trace.forward));
        }
        for &node in tyre.positions() {
            let depth = (node - trace.point).dot(trace.forward);
            if depth > trace.penetration_m
                && trace.faces.iter().any(|face| {
                    // Use horizontal distance along the wall and world height for this vertical plane.
                    let project = |v: Vec3| {
                        Vec3::new(v.dot(Vec3::Y.cross(trace.forward.as_vec3())), 0.0, v.y)
                    };
                    placement::height_at(face.map(project), project(node.as_vec3())).is_some()
                })
            {
                trace.penetration_m = depth;
            }
        }
    }
}

fn print_timing(label: &str, samples: &mut [f64]) {
    samples.sort_by(f64::total_cmp);
    let percentile = |p: usize| 1000.0 * samples[(samples.len() * p).div_ceil(100) - 1];
    let multiplier =
        samples.len() as f64 / f64::from(TICKS_PER_SECOND) / samples.iter().sum::<f64>();
    println!(
        "{label} | median {:.3} ms | p95 {:.3} ms | worst {:.3} ms | {multiplier:.2}× real time",
        percentile(50),
        percentile(95),
        percentile(100)
    );
}

use bevy::prelude::*;
use vehicle_physics::vehicle_assembly::WHEEL_NAMES;

use vehicle_scene::{
    drive::{DriveSession, DriveSetup, DriverInput, TICKS_PER_SECOND},
    placement,
};

const RAMP: &str = "ramp";
const DRIVE: DriverInput = DriverInput {
    throttle: 1.0,
    brake: 0.0,
    steer: 0.0,
};

pub(crate) fn run(mut run: DriveSession) -> Result<(), String> {
    let mut clearance = settle(&mut run)?.clearance_m;
    let forward = run.setup.start.rotation * Vec3::Z;
    let speed = |run: &DriveSession| run.chassis().2.with_y(0.0).length();
    let tick_seconds = 1.0 / TICKS_PER_SECOND as f32;
    let mut acceleration_ticks = 0;
    while speed(&run) < 4.0 {
        if acceleration_ticks >= 8 * TICKS_PER_SECOND {
            return Err("the car did not accelerate to 4 m/s in 8 s".into());
        }
        run.step(DRIVE)?;
        clearance = clearance.min(run.clearance_m());
        acceleration_ticks += 1;
    }
    if run.chassis().2.dot(forward) < 3.8 {
        return Err("the car did not accelerate along its starting heading".into());
    }
    let heading_before = run.chassis().1 * Vec3::Z;
    for _ in 0..TICKS_PER_SECOND {
        run.step(DriverInput {
            throttle: 0.25,
            steer: 0.35,
            ..Default::default()
        })?;
        clearance = clearance.min(run.clearance_m());
    }
    let heading_after = run.chassis().1 * Vec3::Z;
    let turn = heading_before
        .cross(heading_after)
        .y
        .atan2(heading_before.dot(heading_after));
    if turn.to_degrees() < 2.0 {
        return Err(format!(
            "heading change {:.2}° missed the commanded turn",
            turn.to_degrees()
        ));
    }
    let entry_speed = speed(&run);
    let mut distance = 0.0;
    let mut previous = run.chassis().0;
    for _ in 0..(5 * TICKS_PER_SECOND) {
        run.step(DriverInput {
            brake: 1.0,
            ..Default::default()
        })?;
        clearance = clearance.min(run.clearance_m());
        let position = run.chassis().0;
        distance += (position - previous).with_y(0.0).length();
        previous = position;
        if speed(&run) < 0.1 {
            break;
        }
    }
    if speed(&run) >= 0.1 || clearance <= 0.0 {
        return Err("braking or rim clearance check failed".into());
    }
    println!(
        "Drive check: 4 m/s in {:.2} s, turn {:.2}°, brake from {entry_speed:.2} m/s in {distance:.2} m to {:.3} m/s, minimum clearance {:.3} m",
        acceleration_ticks as f32 * tick_seconds,
        turn.to_degrees(),
        speed(&run),
        clearance
    );

    let mut matching = run
        .setup
        .collision
        .objects
        .iter()
        .filter(|(name, _)| *name == RAMP);
    let (_, faces) = matching
        .next()
        .ok_or_else(|| format!("no collision placement named {}", RAMP))?;
    if matching.next().is_some() {
        return Err(format!("ambiguous collision placement {}", RAMP));
    }
    let ramp = run.setup.collision.triangles[faces.clone()].to_vec();
    let pose = run
        .setup
        .pose(Vec3::new(16.0, 3.0, 12.0), Quat::IDENTITY)
        .map_err(|error| format!("{} approach: {error}", RAMP))?;
    run.reset(pose)?;
    check_ramp(&mut run, &ramp, pose.origin_m, &RAMP, 0.0_f32)?;
    println!("Imported drive checks passed.");
    Ok(())
}

struct Settled {
    clearance_m: f32,
    load_centres_m: [Option<f64>; 4],
}

fn settle(run: &mut DriveSession) -> Result<Settled, String> {
    let mut mean = [0.0; 4];
    let mut normal = [0.0; 4];
    let mut moment = [0.0; 4];
    let mut clearance = f32::INFINITY;
    for tick in 0..(5 * TICKS_PER_SECOND) {
        run.step(DriverInput {
            brake: 1.0,
            ..Default::default()
        })?;
        clearance = clearance.min(run.clearance_m());
        if tick >= 4 * TICKS_PER_SECOND {
            for (i, reaction) in run.reactions().into_iter().enumerate() {
                let load = reaction.normal_force.y;
                mean[i] += (load + reaction.tangential_force.y) / f64::from(TICKS_PER_SECOND);
                if let Some(centre) = reaction.lateral_load_centre_m {
                    normal[i] += load;
                    moment[i] += load * centre;
                }
            }
        }
    }
    let weight = run.setup.weight_n();
    let support: f64 = mean.iter().sum();
    if mean.into_iter().any(|force| force <= 0.0)
        || (support - weight).abs() > 0.02 * weight
        || clearance <= 0.0
        || run.chassis().2.length() > 0.1
    {
        return Err(format!(
            "settling check failed: support {mean:.1?} N, weight {weight:.1} N, rim clearance {clearance:.3} m, speed {:.3} m/s",
            run.chassis().2.length()
        ));
    }
    println!(
        "Settled: support {mean:.1?} N, total {support:.1} / {weight:.1} N, rim clearance {clearance:.3} m"
    );
    Ok(Settled {
        clearance_m: clearance,
        load_centres_m: std::array::from_fn(|i| (normal[i] > 0.0).then(|| moment[i] / normal[i])),
    })
}

pub(crate) fn compare_alignment(upright: DriveSetup, authored: DriveSetup) -> Result<(), String> {
    let (upright_centres, upright_scrub) = sample_alignment("Upright", upright)?;
    let alignments = authored.alignments();
    let (authored_centres, authored_scrub) = sample_alignment("Authored", authored)?;
    println!(
        "Load offset is positive toward vehicle left. Positive camber leans outward and positive toe points inward."
    );
    println!(
        "        Authored camber/toe           Load centre (mm)       Mean |lateral force| (N)"
    );
    println!(
        "Wheel      camber°      toe°        upright    authored           upright    authored"
    );

    for (i, name) in WHEEL_NAMES.into_iter().enumerate() {
        let side = if i % 2 == 0 { 1.0 } else { -1.0 };
        let axle = alignments[i].rotation() * Vec3::X;
        let camber = (-side * axle.y.atan2(axle.x.hypot(axle.z))).to_degrees();
        let toe = (side * axle.z.atan2(axle.x)).to_degrees();
        let centre_mm = |offset: Option<f64>| {
            offset.map_or_else(|| "unloaded".to_owned(), |m| format!("{:+.2}", 1000.0 * m))
        };
        println!(
            "{name:5} {camber:12.3} {toe:9.3} {:>13} {:>11} {:17.2} {:11.2}",
            centre_mm(upright_centres[i]),
            centre_mm(authored_centres[i]),
            upright_scrub[i],
            authored_scrub[i],
        );
    }
    Ok(())
}

fn sample_alignment(
    label: &str,
    setup: DriveSetup,
) -> Result<([Option<f64>; 4], [f64; 4]), String> {
    const MIN_SPEED_MPS: f32 = 2.0;
    const MAX_SPEED_MPS: f32 = 4.0;
    const LEVEL_TOLERANCE_M: f32 = 0.005;

    let lateral = setup.start.rotation * Vec3::X;
    let forward = setup.start.rotation * Vec3::Z;
    let mut run = DriveSession::new(setup)?;
    let road_heights = |run: &DriveSession| {
        run.rims().map(|rim| {
            placement::support_below(&run.setup.collision.triangles, rim.centre.as_vec3())
        })
    };
    let error = || format!("{label} alignment report requires a level start and straight approach");
    let floor = road_heights(&run)[0].ok_or_else(error)?;
    let check_level = |run: &DriveSession| {
        road_heights(run)
            .into_iter()
            .all(|height| height.is_some_and(|height| (height - floor).abs() <= LEVEL_TOLERANCE_M))
    };
    if !check_level(&run) {
        return Err(error());
    }

    println!("{label} alignment");
    let settled = settle(&mut run)?;
    let mut forces = [0.0; 4];
    let mut count = 0;
    for _ in 0..8 * TICKS_PER_SECOND {
        run.step(DRIVE)?;
        if !check_level(&run) {
            return Err(error());
        }
        let speed = run.chassis().2.dot(forward);
        if speed > MAX_SPEED_MPS {
            if count == 0 {
                return Err(format!(
                    "{label} alignment skipped the rolling speed window"
                ));
            }
            println!("Rolling at {MIN_SPEED_MPS:.0}–{MAX_SPEED_MPS:.0} m/s, {count} samples");
            return Ok((
                settled.load_centres_m,
                forces.map(|force| force / f64::from(count)),
            ));
        }
        if speed >= MIN_SPEED_MPS {
            for (force, reaction) in forces.iter_mut().zip(run.reactions()) {
                *force += reaction.tangential_force.dot(lateral.as_dvec3()).abs();
            }
            count += 1;
        }
    }
    Err(format!(
        "{label} alignment did not reach {MAX_SPEED_MPS} m/s in 8 s"
    ))
}

fn bounds(triangles: &[[Vec3; 3]]) -> (Vec3, Vec3) {
    triangles.iter().flatten().fold(
        (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY)),
        |(min, max), &point| (min.min(point), max.max(point)),
    )
}

fn check_ramp(
    run: &mut DriveSession,
    ramp: &[[Vec3; 3]],
    origin: Vec3,
    name: &str,
    heading_deg: f32,
) -> Result<(), String> {
    let mut clearance = settle(run)?.clearance_m;
    let base = run.rims().map(|wheel| wheel.centre.y);
    let mut on_ramp = [false; 4];
    let mut rise = [0.0_f64; 4];
    let forward = Quat::from_rotation_y(heading_deg.to_radians()) * Vec3::Z;
    let mut last_contact = [f32::NEG_INFINITY; 4];
    for _ in 0..(12 * TICKS_PER_SECOND) {
        let throttle = if run.chassis().2.with_y(0.0).length() < 8.0 {
            1.0
        } else {
            0.0
        };
        run.step(DriverInput {
            throttle,
            ..Default::default()
        })?;
        clearance = clearance.min(run.clearance_m());
        let mut over_ramp = [false; 4];
        let support = run.support();
        for (i, wheel) in run.rims().iter().enumerate() {
            let point = wheel.centre.as_vec3();
            if let Some(height) = placement::support_below(ramp, point) {
                over_ramp[i] = true;
                // Require actual load on the named slope, with the selected road surface
                // matching its height at the wheel rather than a different stacked level.
                if support[i] > 0.0
                    && placement::support_below(&run.setup.collision.triangles, point)
                        == Some(height)
                {
                    on_ramp[i] = true;
                    last_contact[i] = (point - origin).dot(forward);
                    rise[i] = rise[i].max(wheel.centre.y - base[i]);
                }
            }
        }
        let passed = run
            .rims()
            .into_iter()
            .zip(last_contact)
            .all(|(rim, last)| (rim.centre.as_vec3() - origin).dot(forward) > last);
        if passed
            && on_ramp.into_iter().all(|on| on)
            && rise.into_iter().all(|dy| dy > 0.05)
            && over_ramp.into_iter().all(|over| !over)
        {
            println!(
                "Ramp {name}: origin {:.3?} m, heading {heading_deg:.1}°, wheel rises {rise:.3?} m, minimum clearance {clearance:.3} m",
                origin.to_array()
            );
            return if clearance > 0.0 {
                Ok(())
            } else {
                Err(format!("nonpositive rim clearance crossing {name}"))
            };
        }
    }
    let position = run.chassis().0;
    let (min, max) = bounds(ramp);
    Err(format!(
        "failed to traverse {name}: wheel contacts {on_ramp:?}, rises {rise:.3?} m, last chassis {position:.3?}, collision bounds {min:.3?} .. {max:.3?}"
    ))
}

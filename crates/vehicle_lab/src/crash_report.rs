use std::{ops::Range, sync::Arc};

use avian3d::prelude::SubstepSchedule;
use bevy::{math::DVec3, prelude::*};
use vehicle_assets::VehicleAsset;
use vehicle_physics::{
    avian_tyre::{AvianTyre, TyreSubstepComplete},
    chassis::ChassisNode,
    deformable_vehicle::{DeformableBody, StructureSpec},
    static_collision::StaticContact,
    vehicle_assembly::{VehicleRoad, VehicleSpec, WHEEL_NAMES},
};
use vehicle_scene::{
    body_skin::BodySkin,
    crash::{
        CRASH_TICKS, CrashFrame, CrashSession, CrashSetup, ImportedStructure, Obstacles, SPEED_MPS,
    },
    drive::{self, DriveSetup, TICKS_PER_SECOND},
};

const MIN_VISIBLE_CRUSH_M: f32 = 0.10;
const MIN_RETAINED_CRUSH_FRACTION: f32 = 0.80;
const MAX_REBOUND_SPEED_FRACTION: f64 = 0.10;

pub(crate) fn run(setup: &DriveSetup, asset: &VehicleAsset, target: &str) -> Result<(), String> {
    let crash = CrashSetup::new(setup, asset, target)?;
    let ImportedStructure {
        structure,
        specification,
        source,
        counts,
        mounts,
    } = &crash.imported;
    let rate = crash.substeps * TICKS_PER_SECOND;
    println!(
        "Crash structure | {} sprung nodes | {} beams | {} body panels | {rate} substeps/s",
        structure.nodes.len(),
        structure.beams.len(),
        structure.collision_panels.len()
    );
    for (i, mount) in mounts.iter().enumerate() {
        println!(
            "{} | mount nodes {mount:?} | {} rigid upright links and {} links represented by the suspension",
            WHEEL_NAMES[i], counts.upright_internal[i], counts.suspension_links[i]
        );
    }
    println!(
        "Unavailable source beams {:?} | unavailable source panels {:?} | links across uprights {} | upright contact nodes represented by wheel collision {}",
        counts.unavailable_beams,
        counts.unavailable_panels,
        counts.cross_upright,
        counts.upright_contact_nodes
    );
    println!(
        "Yield-only material | support extension limits omitted {} | ground exclusions approximated by static body contact {}",
        counts.support_extension_limits, counts.ground_exclusions
    );
    let approach = crash.approach;
    let target_range = crash.obstacles.target.clone();
    let triangle_source = crash.obstacles.source.clone();
    println!(
        "{target} | approach face {} | point {:.3?} m | travel direction {:.4?} | launch origin {:.3?} m",
        approach.triangle - target_range.start,
        approach.point.to_array(),
        approach.forward.to_array(),
        approach.pose.origin_m.to_array()
    );
    println!(
        "Matched comparison ends at {:.3} s within supported terrain",
        f64::from(approach.control_ticks) / f64::from(TICKS_PER_SECOND)
    );
    let gauge = NoseGauge::new(structure, *specification)?;
    let skin = BodySkin::new(asset, source)?;
    let source = source.clone();
    let control = Obstacles::new(&setup.collision, Some(&target_range), &target_range)?;
    let control_road = drive::road_from_triangles(
        setup
            .collision
            .triangles
            .iter()
            .copied()
            .enumerate()
            .filter(|(i, _)| !target_range.contains(i))
            .map(|(_, face)| face),
    )
    .map_err(|error| format!("control road: {error}"))?;
    let session = crash.start(|app| {
        app.add_systems(SubstepSchedule, measure.after(TyreSubstepComplete));
    })?;
    let mut bench = Bench {
        session,
        gauge,
        forward: approach.forward.as_dvec3(),
    };
    bench.spawn(DVec3::ZERO, None)?;
    let loads = bench.settle(setup.weight_n())?;
    println!("Settled floor loads {:.1?} N", loads);

    let velocity = approach.velocity();
    bench.spawn(velocity, Some((&control_road, &control)))?;
    let clear = bench
        .crash(approach.control_ticks, approach.control_ticks)
        .map_err(|error| format!("control: {error}"))?;
    bench.spawn(velocity, None)?;
    let impact = bench
        .crash(CRASH_TICKS, approach.control_ticks)
        .map_err(|error| format!("impact: {error}"))?;
    check_wall_penetration(
        asset,
        &skin,
        &impact.frames,
        setup.collision.triangles[approach.triangle],
        approach.forward,
    )?;
    let (step, hit) = impact
        .first_body
        .ok_or("no chassis contact with the selected target")?;
    if let (Some(first), Some(first_hit), Some(corner)) = (
        impact.first_target,
        impact.first_target_contact,
        impact.first_target_wheel,
    ) {
        println!(
            "First target contact at {:.4} s | {} tyre {:?} | {target} face {}",
            first as f64 / f64::from(rate),
            WHEEL_NAMES[corner],
            first_hit.feature,
            triangle_source[first_hit.triangle] - target_range.start
        );
    }
    let (edge, change) = impact
        .comparison_damage
        .iter()
        .zip(&clear.damage)
        .enumerate()
        .map(|(i, (change, baseline))| {
            (i, change.abs().min(impact.damage[i].abs()) - baseline.abs())
        })
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .ok_or("no simulated beams")?;
    let yielded = impact
        .damage
        .iter()
        .filter(|change| change.abs() > 1e-9)
        .count();
    println!(
        "First body contact at {:.4} s | {} | {target} face {} | incoming {:.3} m/s | wall impulse {:.2?} N·s",
        step as f64 / f64::from(rate),
        source.feature(hit.feature),
        triangle_source[hit.triangle] - target_range.start,
        impact.incoming_mps,
        impact.wall_impulse.to_array()
    );
    println!(
        "Peak chassis rebound speed {:.3} m/s",
        impact.peak_rebound_mps
    );
    println!(
        "Nose shortening peak {:.3} mm | residual {:.3} mm | matched shortening {:.3} mm (control {:.3} mm)",
        1000.0 * impact.peak_crush,
        1000.0 * impact.residual_crush,
        1000.0 * impact.comparison_crush,
        1000.0 * clear.residual_crush
    );
    let (largest, largest_change) = impact
        .damage
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.abs().total_cmp(&b.1.abs()))
        .ok_or("no simulated beams")?;
    println!(
        "{yielded} yielded beams | largest change {:.3} mm on source edge {} | difference from control up to {:.3} mm on source edge {}",
        1000.0 * largest_change,
        source.beams[largest],
        1000.0 * change,
        source.beams[edge]
    );
    if clear.first_body.is_some()
        || clear.first_other.is_some()
        || impact.first_other.is_some_and(|other| other < step)
        || !(0.85 * SPEED_MPS..=1.15 * SPEED_MPS).contains(&impact.incoming_mps)
        || impact.wall_impulse.dot(approach.forward.as_dvec3()) >= -1.0
        || change < 0.001
        || impact.last_target + (rate / 4) as usize > impact.steps
    {
        return Err(format!(
            "crash acceptance failed for {target}: approach speed {:.3} m/s, non-target contact at {:?} (control {:?}), last target contact at step {} of {}",
            impact.incoming_mps,
            impact.first_other,
            clear.first_other,
            impact.last_target,
            impact.steps,
        ));
    }
    if impact.peak_rebound_mps > MAX_REBOUND_SPEED_FRACTION * impact.incoming_mps {
        return Err("peak chassis rebound exceeded 10% of its speed at first body contact".into());
    }
    measure_rendered_nose(asset, &skin, &clear, &impact)?;
    bench.spawn(velocity, None)?;
    let replay = bench
        .crash(CRASH_TICKS, approach.control_ticks)
        .map_err(|error| format!("replay: {error}"))?;
    if !impact.matches(&replay) {
        return Err("reset and replay changed target contact or damage".into());
    }
    println!(
        "Crash passed | last target contact at {:.4} s | reset replay matched",
        impact.last_target as f64 / f64::from(rate)
    );
    Ok(())
}

fn check_wall_penetration(
    asset: &VehicleAsset,
    skin: &BodySkin,
    frames: &[CrashFrame],
    wall: [Vec3; 3],
    forward: Vec3,
) -> Result<(), String> {
    // This frontal strike stays within the bounds of the bundled planar wall.
    let [a, b, c] = wall.map(Vec3::as_dvec3);
    let normal = (b - a).cross(c - a).normalize();
    let toward_wall = normal * normal.dot(forward.as_dvec3()).signum();
    let depth = |point: DVec3| (point - a).dot(toward_wall);
    let mut peak_depth_m = 0.0_f64;
    for (tick, frame) in frames.iter().enumerate() {
        let body = skin.sample(&frame.nodes_m)?;
        let rendered = skin
            .meshes
            .iter()
            .zip(&body.vertices)
            .flat_map(|(mesh, vertices)| {
                let instance = &asset.objects[mesh.object];
                vertices.iter().map(|&point| {
                    (body.pose.origin_m + body.pose.rotation * instance.transform_point(point))
                        .as_dvec3()
                })
            });
        let frame_depth_m = frame
            .nodes_m
            .iter()
            .copied()
            .chain(rendered)
            .map(depth)
            .fold(0.0_f64, f64::max);
        // Allow for conversion to f32 world coordinates in the rendered pose.
        if frame_depth_m > 0.0001 {
            return Err(format!(
                "wall penetration at frame {tick}: {:.3} mm",
                1000.0 * frame_depth_m
            ));
        }
        peak_depth_m = peak_depth_m.max(frame_depth_m);
    }
    println!(
        "Peak chassis and rendered body wall penetration {:.3} mm",
        1000.0 * peak_depth_m
    );
    Ok(())
}

fn measure_rendered_nose(
    asset: &VehicleAsset,
    skin: &BodySkin,
    clear: &CrashResult,
    impact: &CrashResult,
) -> Result<(), String> {
    // Select the front bumper surface from the authored geometry in vehicle space.
    let mut vertices = Vec::new();
    for (mesh_index, mesh) in skin.meshes.iter().enumerate() {
        let object = &asset.objects[mesh.object];
        let geometry = &asset.meshes[object.visual_mesh.unwrap().0];
        vertices.extend(
            geometry
                .vertices
                .iter()
                .enumerate()
                .map(|(vertex, &point)| (mesh_index, vertex, object.transform_point(point).z)),
        );
    }
    let nose = vertices
        .iter()
        .map(|v| v.2)
        .reduce(f32::max)
        .ok_or("no rendered body vertices")?;
    vertices.retain(|v| v.2 >= nose - 0.05);
    let shortening = |frame: &CrashFrame| -> Result<f32, String> {
        // The body shape uses the cabin placement frame and removes rigid motion.
        let shape = skin.sample(&frame.nodes_m)?;
        let change: f32 = vertices
            .iter()
            .map(|&(mesh, vertex, rest_z)| {
                let object = &asset.objects[skin.meshes[mesh].object];
                rest_z - object.transform_point(shape.vertices[mesh][vertex]).z
            })
            .sum();
        Ok(change / vertices.len() as f32)
    };
    let matched_tick = clear.frames.len() - 1;
    let control = shortening(&clear.frames[matched_tick])?;
    let changes = impact
        .frames
        .iter()
        .map(shortening)
        .collect::<Result<Vec<_>, _>>()?;
    let matched = changes[matched_tick];
    let (peak_tick, peak) = changes
        .iter()
        .copied()
        .enumerate()
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .unwrap();
    let residual = *changes.last().unwrap();
    let quarter_second = TICKS_PER_SECOND as usize / 4;
    let unloaded_min = changes[changes.len() - 1 - quarter_second..]
        .iter()
        .copied()
        .reduce(f32::min)
        .unwrap();
    println!(
        "Rendered nose shortening | peak {:.2} mm at frame {peak_tick} | matched {:.2} mm (control {:.2} mm) | unloaded minimum {:.2} mm | final {:.2} mm",
        1000.0 * peak,
        1000.0 * matched,
        1000.0 * control,
        1000.0 * unloaded_min,
        1000.0 * residual,
    );
    if matched - control < MIN_VISIBLE_CRUSH_M {
        return Err("rendered nose did not shorten at least 100 mm more than the control at the matched tick".into());
    }
    if unloaded_min < MIN_VISIBLE_CRUSH_M || unloaded_min < MIN_RETAINED_CRUSH_FRACTION * peak {
        return Err("rendered nose did not retain at least 100 mm and 80% of peak shortening throughout the final quarter-second clear of the wall".into());
    }
    Ok(())
}

#[derive(Clone)]
struct NoseGauge {
    nose: Vec<usize>,
    cabin: Vec<usize>,
    length_m: f64,
}

impl NoseGauge {
    fn new(structure: &StructureSpec, specification: VehicleSpec) -> Result<Self, String> {
        let wheels = specification.wheels.map(Option::unwrap);
        let front_axle = f64::from(wheels[0].centre_m.z);
        let rear_axle = f64::from(wheels[2].centre_m.z);
        let span = front_axle - rear_axle;
        let body: std::collections::BTreeSet<_> = structure
            .collision_panels
            .iter()
            .flatten()
            .copied()
            .collect();
        let front = body
            .iter()
            .map(|&i| structure.nodes[i].position_m.z)
            .fold(f64::NEG_INFINITY, f64::max);
        let nose = body
            .iter()
            .copied()
            .filter(|&i| structure.nodes[i].position_m.z >= front.max(front_axle) - 0.15 * span)
            .collect::<Vec<_>>();
        let cabin = body
            .into_iter()
            .filter(|&i| {
                (rear_axle + 0.25 * span..=rear_axle + 0.75 * span)
                    .contains(&structure.nodes[i].position_m.z)
            })
            .collect::<Vec<_>>();
        if nose.is_empty() || cabin.is_empty() {
            return Err(
                "could not identify front and cabin collision nodes for the nose gauge".into(),
            );
        }
        let mut gauge = Self {
            nose,
            cabin,
            length_m: 0.0,
        };
        gauge.length_m = gauge.distance(&structure.nodes);
        Ok(gauge)
    }

    fn distance(&self, nodes: &[ChassisNode]) -> f64 {
        let average = |ids: &[usize]| {
            ids.iter().map(|&i| nodes[i].position_m).sum::<DVec3>() / ids.len() as f64
        };
        average(&self.nose).distance(average(&self.cabin))
    }
}

struct Bench {
    session: CrashSession,
    gauge: NoseGauge,
    forward: DVec3,
}

impl Bench {
    fn spawn(
        &mut self,
        velocity: DVec3,
        control: Option<(&VehicleRoad, &Obstacles)>,
    ) -> Result<(), String> {
        let (chassis, rims) = self.session.spawn(velocity, control)?;
        let obstacles = control.map_or(&self.session.obstacles, |(_, obstacles)| obstacles);
        let trace = Trace {
            chassis,
            rims,
            source: obstacles.source.clone(),
            target: obstacles.target.clone(),
            gauge: self.gauge.clone(),
            forward: self.forward,
            velocity,
            load: [0.0; 4],
            result: CrashResult::default(),
        };
        self.session.world_mut().insert_resource(trace);
        Ok(())
    }

    fn advance(&mut self) -> Result<(), String> {
        self.session.advance().map_err(|error| {
            let trace = self.session.world().resource::<Trace>();
            format!(
                "substep {} after last committed state: {error}",
                trace.result.steps + 1
            )
        })
    }

    fn settle(&mut self, weight: f64) -> Result<[f64; 4], String> {
        let sample_ticks = TICKS_PER_SECOND / 4;
        let mut average = [0.0; 4];
        for tick in 0..2 * TICKS_PER_SECOND {
            self.advance()?;
            let trace = self.session.world().resource::<Trace>();
            if let Some(contact) = trace.result.first_target_contact {
                return Err(format!(
                    "stationary settling touched target at step {:?}: {contact:?}, source triangle {}",
                    trace.result.first_target, trace.source[contact.triangle]
                ));
            }
            if tick >= 2 * TICKS_PER_SECOND - sample_ticks {
                for (mean, load) in average
                    .iter_mut()
                    .zip(self.session.world().resource::<Trace>().load)
                {
                    *mean += load / f64::from(sample_ticks);
                }
            }
        }
        let trace = self.session.world().resource::<Trace>();
        if average
            .iter()
            .any(|load| !load.is_finite() || *load < 0.02 * weight)
            || (average.iter().sum::<f64>() - weight).abs() > 0.1 * weight
            || trace.velocity.length() > 0.25
            || trace.result.first_target.is_some()
        {
            return Err(format!(
                "settling did not support four wheels near the vehicle weight of {weight:.1} N: loads {average:.1?} N, chassis speed {:.3} m/s, first target hit {:?}",
                trace.velocity.length(),
                trace.result.first_target
            ));
        }
        Ok(average)
    }

    fn crash(&mut self, ticks: u32, compare_at: u32) -> Result<CrashResult, String> {
        let mut comparison_damage = Vec::new();
        let mut comparison_crush = 0.0;
        let mut frames = Vec::with_capacity(ticks as usize + 1);
        frames.push(self.session.frame());
        for tick in 1..=ticks {
            self.advance()?;
            frames.push(self.session.frame());
            if tick == compare_at {
                let world = self.session.world();
                let trace = world.resource::<Trace>();
                let body = world.get::<DeformableBody>(trace.chassis).unwrap();
                comparison_damage = body
                    .chassis()
                    .beam_damage()
                    .map(|beam| beam.permanent_length_change_m())
                    .collect();
                comparison_crush =
                    trace.gauge.length_m - trace.gauge.distance(body.chassis().nodes());
            }
        }
        let world = self.session.world_mut();
        let trace = world.remove_resource::<Trace>().unwrap();
        let mut result = trace.result;
        let body = world.get::<DeformableBody>(trace.chassis).unwrap();
        result.residual_crush = trace.gauge.length_m - trace.gauge.distance(body.chassis().nodes());
        result.damage = body
            .chassis()
            .beam_damage()
            .map(|beam| beam.permanent_length_change_m())
            .collect();
        result.comparison_damage = comparison_damage;
        result.comparison_crush = comparison_crush;
        result.frames = frames;
        Ok(result)
    }
}

#[derive(Default)]
struct CrashResult {
    frames: Vec<CrashFrame>,
    steps: usize,
    first_target: Option<usize>,
    first_target_contact: Option<StaticContact>,
    first_target_wheel: Option<usize>,
    first_body: Option<(usize, StaticContact)>,
    first_other: Option<usize>,
    last_target: usize,
    wall_impulse: DVec3,
    incoming_mps: f64,
    peak_rebound_mps: f64,
    peak_crush: f64,
    residual_crush: f64,
    comparison_crush: f64,
    damage: Vec<f64>,
    comparison_damage: Vec<f64>,
}

impl CrashResult {
    fn matches(&self, replay: &Self) -> bool {
        self.first_target == replay.first_target
            && self
                .first_target_contact
                .map(|hit| (hit.triangle, hit.feature))
                == replay
                    .first_target_contact
                    .map(|hit| (hit.triangle, hit.feature))
            && self.first_target_wheel == replay.first_target_wheel
            && self
                .first_body
                .map(|(step, hit)| (step, hit.triangle, hit.feature))
                == replay
                    .first_body
                    .map(|(step, hit)| (step, hit.triangle, hit.feature))
            && self.last_target == replay.last_target
            && self.wall_impulse.abs_diff_eq(replay.wall_impulse, 1e-4)
            && (self.incoming_mps - replay.incoming_mps).abs() < 1e-6
            && (self.peak_rebound_mps - replay.peak_rebound_mps).abs() < 1e-6
            && (self.peak_crush - replay.peak_crush).abs() < 1e-6
            && (self.residual_crush - replay.residual_crush).abs() < 1e-6
            && (self.comparison_crush - replay.comparison_crush).abs() < 1e-6
            && self.comparison_damage.len() == replay.comparison_damage.len()
            && self.damage.len() == replay.damage.len()
            && self
                .damage
                .iter()
                .chain(&self.comparison_damage)
                .zip(replay.damage.iter().chain(&replay.comparison_damage))
                .all(|(a, b)| (a - b).abs() < 1e-6)
    }
}

#[derive(Resource)]
struct Trace {
    chassis: Entity,
    rims: [Entity; 4],
    source: Arc<[usize]>,
    target: Range<usize>,
    gauge: NoseGauge,
    forward: DVec3,
    velocity: DVec3,
    load: [f64; 4],
    result: CrashResult,
}

fn measure(
    mut trace: Option<ResMut<Trace>>,
    bodies: Query<&DeformableBody>,
    tyres: Query<&AvianTyre>,
) {
    let Some(trace) = trace.as_deref_mut() else {
        return;
    };
    let Ok(body) = bodies.get(trace.chassis) else {
        return;
    };
    if body.error().is_some() {
        return;
    }
    let result = &mut trace.result;
    result.steps += 1;
    let step = result.steps;
    let mut measure_contact = |reaction: vehicle_physics::static_collision::StaticReaction,
                               wheel: Option<usize>| {
        let Some(hit) = reaction.first else {
            return None;
        };
        if trace.target.contains(&trace.source[hit.triangle]) {
            if result.first_target.is_none() {
                result.first_target = Some(step);
                result.first_target_contact = Some(hit);
                result.first_target_wheel = wheel;
            }
            result.last_target = step;
            result.wall_impulse += reaction.impulse_ns;
            Some(hit)
        } else {
            result.first_other.get_or_insert(step);
            None
        }
    };
    let first_body_hit = measure_contact(body.obstacle_reaction(), None);
    for (i, rim) in trace.rims.into_iter().enumerate() {
        let tyre = tyres.get(rim).unwrap().tyre();
        trace.load[i] = tyre.triangle_reaction().normal_force.y;
        measure_contact(tyre.obstacle_reaction(), Some(i));
    }
    if let Some(hit) = first_body_hit
        && result.first_body.is_none()
    {
        result.first_body = Some((step, hit));
        result.incoming_mps = trace.velocity.dot(trace.forward);
    }
    let nodes = body.chassis().nodes();
    trace.velocity = nodes
        .iter()
        .map(|node| node.mass_kg * node.velocity_mps)
        .sum::<DVec3>()
        / nodes.iter().map(|node| node.mass_kg).sum::<f64>();
    if result.first_body.is_some() {
        result.peak_rebound_mps = result
            .peak_rebound_mps
            .max(-trace.velocity.dot(trace.forward));
    }
    result.peak_crush = result
        .peak_crush
        .max(trace.gauge.length_m - trace.gauge.distance(nodes));
}

#[cfg(test)]
mod tests {
    use super::*;
    use vehicle_assets::ror::reference;
    use vehicle_physics::chassis::ChassisBeamKind;
    use vehicle_scene::drive::AlignmentMode;

    #[test]
    fn imported_structure_maps_shuffled_uprights_and_excludes_the_wall_from_control() {
        let vehicle = reference::vehicle().unwrap();
        let terrain = reference::terrain().unwrap();
        let setup = DriveSetup::new(&terrain, &vehicle, AlignmentMode::Authored).unwrap();
        assert_eq!(setup.wheel_visuals().map(|(i, _)| i), [1, 3, 2, 0]);
        let assembly = vehicle_scene::crash::assemble(
            &vehicle.vehicle,
            setup.wheel_visuals(),
            setup.specification(),
        )
        .unwrap();
        assert_eq!(assembly.mounts.map(|nodes| nodes[0]), [17, 23, 20, 14]);
        assert_eq!(assembly.counts.upright_internal, [1; 4]);
        assert_eq!(assembly.counts.suspension_links, [1; 4]);
        assert!(
            assembly
                .structure
                .beams
                .iter()
                .any(|beam| beam.kind == ChassisBeamKind::CompressionSupport)
        );

        let mut wheels = setup.wheel_visuals();
        wheels[0].1.attachment = None;
        let error = vehicle_scene::crash::assemble(&vehicle.vehicle, wheels, setup.specification())
            .err()
            .unwrap();
        assert!(error.contains("FL: no unique authored shock attachment"));

        let (_, target) = setup
            .collision
            .objects
            .iter()
            .find(|(name, _)| name == "wall")
            .unwrap();
        let full = Obstacles::new(&setup.collision, None, target).unwrap();
        let control = Obstacles::new(&setup.collision, Some(target), target).unwrap();
        assert_eq!(
            full.source.iter().filter(|i| target.contains(i)).count(),
            target.len()
        );
        assert!(control.source.iter().all(|i| !target.contains(i)));
    }
}

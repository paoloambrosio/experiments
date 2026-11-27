use std::{collections::BTreeMap, ops::Range, sync::Arc};

use avian3d::prelude::SubstepSchedule;
use bevy::{math::DVec3, prelude::*};
use vehicle_assets::VehicleAsset;
use vehicle_physics::{
    avian_tyre::{AvianTyre, TyreSubstepComplete},
    chassis::ChassisNode,
    deformable_vehicle::{
        DeformableBody, DeformableVehicle, DeformableVehiclePlugin, PreparedDeformableVehicle,
        StructureSpec,
    },
    static_collision::{StaticCollisionMesh, StaticContact},
    vehicle_assembly::{PreparedVehicle, VehiclePose, VehicleRoad, VehicleSpec, WHEEL_NAMES},
};

use crate::crash_structure::{self, ImportedStructure};
use vehicle_scene::{
    drive::{self, DriveSetup, TICKS_PER_SECOND},
    placement::{self, CollisionGeometry},
};

const SPEED_MPS: f64 = 8.0;
const MIN_SUBSTEPS: u32 = 64;
const APPROACH_GAP_M: f32 = 2.0;
const CRASH_TICKS: u32 = 2 * TICKS_PER_SECOND;

pub(crate) fn run(setup: &DriveSetup, asset: &VehicleAsset, target: &str) -> Result<(), String> {
    let target_range = match setup
        .collision
        .objects
        .iter()
        .filter(|(name, _)| name == target)
        .collect::<Vec<_>>()
        .as_slice()
    {
        [(_, range)] if !range.is_empty() => range.clone(),
        [] => {
            return Err(format!(
                "crash target {target:?} is not a placed object with collision geometry"
            ));
        }
        _ => {
            return Err(format!(
                "crash target {target:?} must identify one nonempty placement"
            ));
        }
    };
    let ImportedStructure {
        structure,
        specification,
        source,
        counts,
        mounts,
    } = crash_structure::assemble(asset, setup.wheel_visuals(), setup.specification())?;
    let substeps = crash_structure::substeps(&structure, TICKS_PER_SECOND, MIN_SUBSTEPS)?;
    let rate = substeps * TICKS_PER_SECOND;
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
    let prepared_tyres = specification
        .prepare(rate)
        .map_err(|error| error.to_string())?
        .with_wheel_alignments(setup.alignments());
    let approach = approach(
        &setup.collision,
        &target_range,
        &structure,
        specification,
        &prepared_tyres,
    )?;
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
    let gauge = NoseGauge::new(&structure, specification)?;
    let vehicle = PreparedDeformableVehicle::new(specification, structure, setup.alignments())
        .map_err(|error| error.to_string())?;
    let world = Obstacles::new(&setup.collision, None, &target_range)?;
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
    let mut bench = Bench::new(
        substeps,
        &vehicle,
        &prepared_tyres,
        gauge,
        approach.forward.as_dvec3(),
    );
    bench.spawn(approach.pose, DVec3::ZERO, setup.road(), &world)?;
    let loads = bench.settle(setup.weight_n())?;
    println!("Settled floor loads {:.1?} N", loads);

    let velocity = SPEED_MPS * approach.forward.as_dvec3();
    bench.spawn(approach.pose, velocity, &control_road, &control)?;
    let clear = bench
        .crash(approach.control_ticks, approach.control_ticks)
        .map_err(|error| format!("control: {error}"))?;
    bench.spawn(approach.pose, velocity, setup.road(), &world)?;
    let impact = bench
        .crash(CRASH_TICKS, approach.control_ticks)
        .map_err(|error| format!("impact: {error}"))?;
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
            world.source[first_hit.triangle] - target_range.start
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
        world.source[hit.triangle] - target_range.start,
        impact.incoming_mps,
        impact.wall_impulse.to_array()
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
    bench.spawn(approach.pose, velocity, setup.road(), &world)?;
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

struct Obstacles {
    mesh: Arc<StaticCollisionMesh>,
    source: Arc<[usize]>,
    target: Range<usize>,
}

impl Obstacles {
    fn new(
        terrain: &CollisionGeometry,
        excluded: Option<&Range<usize>>,
        target: &Range<usize>,
    ) -> Result<Self, String> {
        let mut indices = BTreeMap::new();
        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        let mut source = Vec::new();
        for (i, &triangle) in terrain.triangles.iter().enumerate() {
            if excluded.is_some_and(|range| range.contains(&i)) {
                continue;
            }
            let [a, b, c] = triangle.map(Vec3::as_dvec3);
            if (b - a).cross(c - a).length_squared() == 0.0 {
                continue;
            }
            triangles.push([a, b, c].map(|point| {
                let key = point
                    .to_array()
                    .map(|v| if v == 0.0 { 0 } else { v.to_bits() });
                *indices.entry(key).or_insert_with(|| {
                    let index = vertices.len();
                    vertices.push(point);
                    index
                })
            }));
            source.push(i);
        }
        let mesh = StaticCollisionMesh::new(&vertices, &triangles)
            .map_err(|error| format!("static terrain collision: {error}"))?;
        Ok(Self {
            mesh: Arc::new(mesh),
            source: source.into(),
            target: target.clone(),
        })
    }
}

struct Approach {
    pose: VehiclePose,
    point: Vec3,
    forward: Vec3,
    triangle: usize,
    control_ticks: u32,
}

fn approach(
    terrain: &CollisionGeometry,
    target: &Range<usize>,
    structure: &StructureSpec,
    specification: VehicleSpec,
    tyres: &PreparedVehicle,
) -> Result<Approach, String> {
    let wheels = specification.wheels.map(Option::unwrap);
    let front = structure
        .nodes
        .iter()
        .map(|node| node.position_m.z as f32)
        .fold(f32::NEG_INFINITY, f32::max);
    let probes: Vec<_> = structure
        .collision_panels
        .iter()
        .flatten()
        .copied()
        .chain(structure.collision_nodes.iter().copied())
        .collect::<std::collections::BTreeSet<_>>()
        .into_iter()
        .map(|i| structure.nodes[i].position_m.as_vec3())
        .collect();
    let mut faces: Vec<_> = target
        .clone()
        .filter_map(|i| {
            let [a, b, c] = terrain.triangles[i];
            let cross = (b - a).cross(c - a);
            let normal = cross.try_normalize()?;
            (normal.y.abs() <= 0.15).then_some((
                cross.length_squared(),
                i,
                (a + b + c) / 3.0,
                normal,
            ))
        })
        .collect();
    faces.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
    for (_, triangle, point, normal) in faces {
        let mut best = None;
        let mut best_clearance = 0;
        for direction in [normal, -normal] {
            let forward = direction.with_y(0.0).normalize();
            let rotation = Quat::from_rotation_y(forward.x.atan2(forward.z));
            let launch = point - forward * (front + APPROACH_GAP_M);
            let Some(ground) = placement::support_below(&terrain.triangles, launch.with_y(point.y))
            else {
                continue;
            };
            let Ok(pose) = drive::spawn_pose(
                &terrain.triangles,
                launch.with_y(ground + 0.35),
                rotation,
                wheels,
                tyres,
            ) else {
                continue;
            };
            let mut intersects_target = false;
            let mut blocked = false;
            let mut first_target = f32::INFINITY;
            let mut control_clearance: f32 = 2.0;
            for local in &probes {
                let start = pose.origin_m + rotation * *local;
                let reaches_wall = segment_hits(start, forward, terrain.triangles[triangle])
                    .is_some_and(|distance| {
                        (APPROACH_GAP_M - 0.2..=APPROACH_GAP_M + 1.5).contains(&distance)
                    });
                intersects_target |= reaches_wall;
                for (i, face) in terrain.triangles.iter().enumerate() {
                    let Some(distance) = segment_hits(start, forward, *face) else {
                        continue;
                    };
                    if target.contains(&i) {
                        first_target = first_target.min(distance);
                        continue;
                    }
                    blocked |= distance <= APPROACH_GAP_M + 0.3;
                    control_clearance = control_clearance.min(distance / SPEED_MPS as f32);
                }
            }
            if !intersects_target || blocked || first_target < APPROACH_GAP_M - 0.15 {
                continue;
            }
            let support_until = (0..=CRASH_TICKS)
                .find(|&tick| {
                    // Probe a tread radius ahead of the wheel centre so the
                    // control stays clear of the edge of a finite heightfield.
                    let travel = tick as f32 * SPEED_MPS as f32 / TICKS_PER_SECOND as f32;
                    let centre = pose.origin_m + forward * travel;
                    wheels.iter().any(|wheel| {
                        let sample = centre
                            + rotation * wheel.centre_m
                            + forward * wheel.tyre.geometry.tread_radius;
                        !placement::support_below(&terrain.triangles, sample)
                            .is_some_and(|y| (y - ground).abs() <= 0.15)
                    })
                })
                .map_or(CRASH_TICKS, |tick| tick.saturating_sub(1));
            let control_ticks =
                support_until.min((control_clearance * TICKS_PER_SECOND as f32).floor() as u32);
            let minimum_ticks = ((f64::from(first_target) / SPEED_MPS + 0.20)
                * f64::from(TICKS_PER_SECOND))
            .ceil() as u32;
            if control_ticks >= minimum_ticks && control_ticks > best_clearance {
                best = Some(Approach {
                    pose,
                    point,
                    forward,
                    triangle,
                    control_ticks,
                });
                best_clearance = control_ticks;
            }
        }
        if let Some(approach) = best {
            return Ok(approach);
        }
    }
    Err("no vertical target face has a clear frontal approach and supported control lane at body height".into())
}

fn segment_hits(start: Vec3, forward: Vec3, [a, b, c]: [Vec3; 3]) -> Option<f32> {
    let u = b - a;
    let v = c - a;
    let cross = forward.cross(v);
    let det = u.dot(cross);
    if det.abs() <= 1e-7 {
        return None;
    }
    let offset = start - a;
    let bary_u = offset.dot(cross) / det;
    let q = offset.cross(u);
    let bary_v = forward.dot(q) / det;
    let t = v.dot(q) / det;
    (bary_u >= -1e-5 && bary_v >= -1e-5 && bary_u + bary_v <= 1.0 + 1e-5 && t > 0.0).then_some(t)
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

struct Bench<'a> {
    app: App,
    active: Option<DeformableVehicle>,
    prepared: &'a PreparedDeformableVehicle,
    tyres: &'a PreparedVehicle,
    gauge: NoseGauge,
    forward: DVec3,
}

impl<'a> Bench<'a> {
    fn new(
        substeps: u32,
        prepared: &'a PreparedDeformableVehicle,
        tyres: &'a PreparedVehicle,
        gauge: NoseGauge,
        forward: DVec3,
    ) -> Self {
        let app = drive::physics_app(substeps, |app| {
            app.add_plugins(DeformableVehiclePlugin)
                .add_systems(SubstepSchedule, measure.after(TyreSubstepComplete));
        });
        Self {
            app,
            active: None,
            prepared,
            tyres,
            gauge,
            forward,
        }
    }

    fn spawn(
        &mut self,
        pose: VehiclePose,
        velocity: DVec3,
        road: &VehicleRoad,
        obstacles: &Obstacles,
    ) -> Result<(), String> {
        let world = self.app.world_mut();
        if let Some(previous) = self.active.take() {
            previous.despawn(world);
        }
        let vehicle = self
            .prepared
            .spawn(
                world,
                self.tyres.tyres(),
                pose,
                velocity,
                road,
                Some(obstacles.mesh.clone()),
            )
            .map_err(|error| error.to_string())?;
        world.insert_resource(Trace {
            chassis: vehicle.body,
            rims: vehicle.wheels.map(|wheel| wheel.rim),
            source: obstacles.source.clone(),
            target: obstacles.target.clone(),
            gauge: self.gauge.clone(),
            forward: self.forward,
            velocity,
            load: [0.0; 4],
            result: CrashResult::default(),
        });
        self.active = Some(vehicle);
        Ok(())
    }

    fn advance(&mut self) -> Result<(), String> {
        self.app.update();
        let world = self.app.world();
        let trace = world.resource::<Trace>();
        let body = world.get::<DeformableBody>(trace.chassis).unwrap();
        if let Some(error) = body.error() {
            return Err(format!(
                "substep {} after last committed state: {error}",
                trace.result.steps + 1
            ));
        }
        Ok(())
    }

    fn settle(&mut self, weight: f64) -> Result<[f64; 4], String> {
        let sample_ticks = TICKS_PER_SECOND / 4;
        let mut average = [0.0; 4];
        for tick in 0..2 * TICKS_PER_SECOND {
            self.advance()?;
            let trace = self.app.world().resource::<Trace>();
            if let Some(contact) = trace.result.first_target_contact {
                return Err(format!(
                    "stationary settling touched target at step {:?}: {contact:?}, source triangle {}",
                    trace.result.first_target, trace.source[contact.triangle]
                ));
            }
            if tick >= 2 * TICKS_PER_SECOND - sample_ticks {
                for (mean, load) in average
                    .iter_mut()
                    .zip(self.app.world().resource::<Trace>().load)
                {
                    *mean += load / f64::from(sample_ticks);
                }
            }
        }
        let trace = self.app.world().resource::<Trace>();
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
        for tick in 1..=ticks {
            self.advance()?;
            if tick == compare_at {
                let world = self.app.world();
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
        let world = self.app.world_mut();
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
        Ok(result)
    }
}

#[derive(Default)]
struct CrashResult {
    steps: usize,
    first_target: Option<usize>,
    first_target_contact: Option<StaticContact>,
    first_target_wheel: Option<usize>,
    first_body: Option<(usize, StaticContact)>,
    first_other: Option<usize>,
    last_target: usize,
    wall_impulse: DVec3,
    incoming_mps: f64,
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
        let assembly = crash_structure::assemble(
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
        let error = crash_structure::assemble(&vehicle.vehicle, wheels, setup.specification())
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

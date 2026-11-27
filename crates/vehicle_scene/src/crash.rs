//! Shared setup and capture for the imported vehicle wall strike.

use std::{collections::BTreeMap, ops::Range, sync::Arc};

use bevy::{math::DVec3, prelude::*};
use vehicle_assets::VehicleAsset;
use vehicle_physics::{
    avian_tyre::AvianTyre,
    deformable_vehicle::{
        DeformableBody, DeformableVehicle, DeformableVehiclePlugin, PreparedDeformableVehicle,
        StructureSpec,
    },
    mounted_tyre::RimMotion,
    static_collision::StaticCollisionMesh,
    vehicle_assembly::{
        PreparedVehicle, VehiclePose, VehicleRoad, VehicleSpec, WHEEL_NAMES, WheelAlignment,
    },
};

pub use crate::crash_structure::{ImportedStructure, SourceIndices, assemble};
use crate::{
    crash_structure,
    drive::{self, DriveSetup, TICKS_PER_SECOND},
    placement::{self, CollisionGeometry},
};

pub const SPEED_MPS: f64 = 8.0;
pub const CRASH_TICKS: u32 = 2 * TICKS_PER_SECOND;
const MIN_SUBSTEPS: u32 = 64;
const APPROACH_GAP_M: f32 = 2.0;

pub struct CrashSetup {
    pub imported: ImportedStructure,
    pub substeps: u32,
    pub approach: Approach,
    pub obstacles: Obstacles,
    tyres: PreparedVehicle,
    road: VehicleRoad,
    alignments: [WheelAlignment; 4],
}

impl CrashSetup {
    pub fn new(setup: &DriveSetup, asset: &VehicleAsset, target: &str) -> Result<Self, String> {
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
        let imported = assemble(asset, setup.wheel_visuals(), setup.specification())?;
        let substeps =
            crash_structure::substeps(&imported.structure, TICKS_PER_SECOND, MIN_SUBSTEPS)?;
        let alignments = setup.alignments();
        let tyres = imported
            .specification
            .prepare(substeps * TICKS_PER_SECOND)
            .map_err(|error| error.to_string())?
            .with_wheel_alignments(alignments);
        let approach = approach(
            &setup.collision,
            &target_range,
            &imported.structure,
            imported.specification,
            &tyres,
        )?;
        let obstacles = Obstacles::new(&setup.collision, None, &target_range)?;
        Ok(Self {
            imported,
            substeps,
            approach,
            obstacles,
            tyres,
            road: setup.road().clone(),
            alignments,
        })
    }

    pub fn start(self, configure: impl FnOnce(&mut App)) -> Result<CrashSession, String> {
        let vehicle = PreparedDeformableVehicle::new(
            self.imported.specification,
            self.imported.structure,
            self.alignments,
        )
        .map_err(|error| error.to_string())?;
        let app = drive::physics_app(self.substeps, |app| {
            app.add_plugins(DeformableVehiclePlugin);
            configure(app);
        });
        Ok(CrashSession {
            app,
            vehicle,
            tyres: self.tyres,
            active: None,
            road: self.road,
            approach: self.approach,
            obstacles: self.obstacles,
        })
    }

    pub fn record(self) -> Result<Vec<CrashFrame>, String> {
        let mut session = self.start(|_| {})?;
        session.spawn(session.approach.velocity(), None)?;
        let mut frames = Vec::with_capacity(CRASH_TICKS as usize + 1);
        frames.push(session.frame());
        for tick in 1..=CRASH_TICKS {
            session
                .advance()
                .map_err(|error| format!("crash replay tick {tick}: {error}"))?;
            frames.push(session.frame());
        }
        Ok(frames)
    }
}

/// Committed state at one 60 Hz tick. Positions are in world metres.
/// Chassis nodes follow the order in `SourceIndices::nodes`. Rim order is FL, FR, RL, RR.
pub struct CrashFrame {
    pub nodes_m: Vec<DVec3>,
    pub rims: [RimMotion; 4],
}

pub struct CrashSession {
    app: App,
    vehicle: PreparedDeformableVehicle,
    tyres: PreparedVehicle,
    active: Option<DeformableVehicle>,
    road: VehicleRoad,
    approach: Approach,
    pub obstacles: Obstacles,
}

impl CrashSession {
    pub fn spawn(
        &mut self,
        velocity_mps: DVec3,
        control: Option<(&VehicleRoad, &Obstacles)>,
    ) -> Result<(Entity, [Entity; 4]), String> {
        let world = self.app.world_mut();
        if let Some(previous) = self.active.take() {
            previous.despawn(world);
        }
        let (road, obstacles) = control.unwrap_or((&self.road, &self.obstacles));
        let vehicle = self
            .vehicle
            .spawn(
                world,
                self.tyres.tyres(),
                self.approach.pose,
                velocity_mps,
                road,
                Some(obstacles.mesh.clone()),
            )
            .map_err(|error| error.to_string())?;
        let entities = (vehicle.body, vehicle.wheels.map(|wheel| wheel.rim));
        self.active = Some(vehicle);
        Ok(entities)
    }

    pub fn world(&self) -> &World {
        self.app.world()
    }

    pub fn world_mut(&mut self) -> &mut World {
        self.app.world_mut()
    }

    pub fn body(&self) -> &DeformableBody {
        let vehicle = self.active.as_ref().expect("spawn the crash vehicle first");
        self.world().get::<DeformableBody>(vehicle.body).unwrap()
    }

    pub fn advance(&mut self) -> Result<(), String> {
        self.app.update();
        if let Some(error) = self.body().error() {
            return Err(error.to_string());
        }
        let vehicle = self.active.as_ref().expect("spawn the crash vehicle first");
        for (wheel, name) in vehicle.wheels.iter().zip(WHEEL_NAMES) {
            if let Some(error) = self.world().get::<AvianTyre>(wheel.rim).unwrap().error() {
                return Err(format!("{name} tyre simulation: {error}"));
            }
        }
        Ok(())
    }

    pub fn frame(&self) -> CrashFrame {
        let vehicle = self.active.as_ref().expect("spawn the crash vehicle first");
        CrashFrame {
            nodes_m: self
                .body()
                .chassis()
                .nodes()
                .iter()
                .map(|node| node.position_m)
                .collect(),
            rims: vehicle.wheels.map(|wheel| {
                self.world()
                    .get::<AvianTyre>(wheel.rim)
                    .unwrap()
                    .tyre()
                    .rim()
            }),
        }
    }
}

impl Approach {
    pub fn velocity(&self) -> DVec3 {
        SPEED_MPS * self.forward.as_dvec3()
    }
}

pub struct Obstacles {
    mesh: Arc<StaticCollisionMesh>,
    pub source: Arc<[usize]>,
    pub target: Range<usize>,
}

impl Obstacles {
    pub fn new(
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

#[derive(Clone, Copy)]
pub struct Approach {
    pub pose: VehiclePose,
    pub point: Vec3,
    pub forward: Vec3,
    pub triangle: usize,
    pub control_ticks: u32,
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

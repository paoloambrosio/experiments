use std::{sync::Arc, time::Duration};

use avian3d::prelude::*;
use bevy::{math::DVec3, prelude::*, time::TimeUpdateStrategy};
use vehicle_assets::{
    WheelGeometry, WheelPhysics, WheelSide,
    ror::{TerrainImport, VehicleImport},
};
use vehicle_physics::{
    avian_suspension::{AvianSuspensionPlugin, Suspension, WheelControl},
    avian_tyre::{AvianTyre, AvianTyrePlugin},
    mounted_tyre::{ContactParameters, RimMotion},
    reference_car::{self, G},
    triangle_road::TriangleRoad,
    vehicle_assembly::{
        ChassisSpec, PreparedVehicle, VehicleAssembly, VehiclePose, VehicleRoad, VehicleSpec,
        WHEEL_NAMES, WheelSpec,
    },
};

use crate::{
    config::RampApproach,
    placement::{self, CollisionGeometry},
};

pub(crate) const TICKS_PER_SECOND: u32 = 60;
const SUBSTEPS: u32 = 64;
const SPAWN_CLEARANCE_M: f32 = 0.010;
const TYRE_MASS_FRACTION: f64 = 0.22;
const TYRE_GAUGE_PRESSURE_PA: f64 = 150_000.0;
const SHOULDER_HEIGHT_FRACTION: f32 = 0.90;
const BEAD_WIDTH_FRACTION: f32 = 0.85;
const MAX_STEER_DEG: f32 = 28.0;
const ROAD_STIFFNESS_N_PER_M: f64 = 500_000.0;
const ROAD_DAMPING_NS_PER_M: f64 = 150.0;
const ROAD_FRICTION: f64 = 0.8;
const ROAD_TRANSITION_SPEED_MPS: f64 = 0.1;
const ROAD_CONTACT_DEPTH_M: f64 = 0.05;

#[derive(Clone, Copy)]
struct WheelInput {
    geometry_index: usize,
    data: WheelPhysics,
    spec: WheelSpec,
    drive_nm: f32,
    brake_nm: f32,
}

pub(crate) struct DriveSetup {
    report_ramp: Option<RampApproach>,
    specification: VehicleSpec,
    wheels: [WheelInput; 4],
    prepared: PreparedVehicle,
    pub collision: CollisionGeometry,
    road: VehicleRoad,
    pub start: VehiclePose,
}

impl DriveSetup {
    pub(crate) fn new(
        report_ramp: Option<RampApproach>,
        terrain: &TerrainImport,
        vehicle: &VehicleImport,
    ) -> Result<Self, String> {
        let asset = &vehicle.vehicle;
        let chassis = asset.physics.chassis.ok_or_else(|| {
            format!("{}: missing rigid chassis properties from the authored nodes, beams, globals, and body mesh", vehicle.entry)
        })?;
        let wheels = resolve_wheels(&asset.wheels, &asset.physics.wheels)
            .map_err(|error| format!("{}: {error}", vehicle.entry))?;
        let specification = VehicleSpec {
            chassis: ChassisSpec {
                mass_kg: chassis.mass_kg,
                size_m: chassis.size_m,
                centre_of_mass_m: chassis.centre_of_mass_m,
            },
            wheels: wheels.map(|wheel| Some(wheel.spec)),
        };
        let prepared = specification
            .prepare(TICKS_PER_SECOND * SUBSTEPS)
            .map_err(|error| format!("cannot prepare physics for {}: {error}", vehicle.entry))?;
        if prepared
            .geometry()
            .front_steer(MAX_STEER_DEG.to_radians())
            .is_none()
        {
            return Err(
                "invalid default steering limit for imported wheelbase and front track".into(),
            );
        }
        let collision = placement::collision_geometry(&terrain.terrain);
        let (vertices, triangles): (Vec<_>, Vec<_>) = collision
            .triangles
            .iter()
            .enumerate()
            .map(|(i, face)| {
                (
                    face.map(|point| point.as_dvec3()),
                    [3 * i, 3 * i + 1, 3 * i + 2],
                )
            })
            .unzip();
        let vertices: Vec<DVec3> = vertices.into_iter().flatten().collect();
        let road = Arc::new(
            TriangleRoad::new(&vertices, &triangles, ROAD_CONTACT_DEPTH_M)
                .map_err(|error| format!("{} collision geometry: {error}", terrain.terrain.name))?,
        );
        let contact = ContactParameters::new(ROAD_STIFFNESS_N_PER_M, ROAD_DAMPING_NS_PER_M)
            .and_then(|parameters| {
                parameters.with_friction(ROAD_FRICTION, ROAD_TRANSITION_SPEED_MPS)
            })
            .map_err(|error| format!("invalid road contact defaults: {error}"))?;
        let start = spawn_pose(
            &collision.triangles,
            placement::start_hint(terrain)?,
            Quat::IDENTITY,
            specification.wheels.map(Option::unwrap),
            prepared.unloaded_radii_m(),
        )
        .map_err(|error| format!("{} terrain start: {error}", terrain.terrain.name))?;
        Ok(Self {
            report_ramp,
            specification,
            wheels,
            prepared,
            collision,
            road: VehicleRoad::Triangles { road, contact },
            start,
        })
    }

    pub(crate) fn pose(&self, hint: Vec3, rotation: Quat) -> Result<VehiclePose, String> {
        spawn_pose(
            &self.collision.triangles,
            hint,
            rotation,
            self.specification.wheels.map(Option::unwrap),
            self.prepared.unloaded_radii_m(),
        )
    }

    pub(crate) fn centre_of_mass(&self) -> Vec3 {
        self.specification.chassis.centre_of_mass_m
    }

    pub(crate) fn wheel_visuals(&self) -> [(usize, WheelPhysics); 4] {
        self.wheels.map(|wheel| (wheel.geometry_index, wheel.data))
    }

    pub(crate) fn weight_n(&self) -> f64 {
        let total = self
            .wheels
            .iter()
            .map(|wheel| {
                wheel.spec.bare_rim_mass_kg
                    + wheel.spec.tyre.material.total_mass
                    + f64::from(wheel.spec.upright_mass_kg)
            })
            .sum::<f64>();
        (f64::from(self.specification.chassis.mass_kg) + total) * G
    }

    pub(crate) fn report_ramp(&self) -> Option<&RampApproach> {
        self.report_ramp.as_ref()
    }

    pub(crate) fn print_setup(&self, entry: &str, terrain_entry: &str) {
        let geometry = self.prepared.geometry();
        let radii = self.prepared.unloaded_radii_m();
        println!("Drive sources: {} and {}", entry, terrain_entry);
        let chassis = self.specification.chassis;
        println!(
            "Derived chassis: {:.2} kg, body bounds {:.3?} m, centre of mass {:.3?} m",
            chassis.mass_kg,
            chassis.size_m.to_array(),
            chassis.centre_of_mass_m.to_array()
        );
        println!(
            "Tyre and steering defaults: {TYRE_GAUGE_PRESSURE_PA:.0} Pa gauge, tyre mass fraction {TYRE_MASS_FRACTION:.2}, shoulder height fraction {SHOULDER_HEIGHT_FRACTION:.2}, bead width fraction {BEAD_WIDTH_FRACTION:.2}, front steer {MAX_STEER_DEG:.1}°"
        );
        println!(
            "Road defaults: {ROAD_STIFFNESS_N_PER_M:.0} N/m, {ROAD_DAMPING_NS_PER_M:.0} N·s/m, friction {ROAD_FRICTION:.2}, transition {ROAD_TRANSITION_SPEED_MPS:.2} m/s, depth {ROAD_CONTACT_DEPTH_M:.2} m"
        );
        println!(
            "Derived geometry: wheelbase {:.3} m, front/rear tracks {:.3}/{:.3} m, weight {:.1} N",
            geometry.wheelbase_m,
            geometry.front_track_m,
            geometry.rear_track_m,
            self.weight_n()
        );
        for (index, (wheel, input)) in self
            .specification
            .wheels
            .map(Option::unwrap)
            .into_iter()
            .zip(self.wheels)
            .enumerate()
        {
            let geometry = wheel.tyre.geometry;
            let mount = wheel.centre_m + Vec3::Y * wheel.initial_length_m - self.centre_of_mass();
            println!(
                "{} archive: centre {:.3?} m, outer/rim radii {:.3}/{:.3} m, width {:.3} m, wheel {:.1} kg, service brake {:.1} N·m",
                WHEEL_NAMES[index],
                wheel.centre_m.to_array(),
                geometry.tread_radius,
                geometry.bead_radius,
                geometry.shoulder_width,
                wheel.bare_rim_mass_kg + wheel.tyre.material.total_mass,
                input.brake_nm,
            );
            let suspension = wheel.suspension;
            println!(
                "{} derived: upright {:.2} kg, fixed first-gear torque {:.1} N·m, vertical spring {:.1} N/m, damper {:.1} N·s/m, range {:.3}–{:.3} m, free {:.3} m, initial {:.3} m",
                WHEEL_NAMES[index],
                wheel.upright_mass_kg,
                input.drive_nm,
                suspension.stiffness_n_per_m,
                suspension.damping_ns_per_m,
                suspension.min_length_m,
                suspension.max_length_m,
                suspension.free_length_m,
                wheel.initial_length_m,
            );
            println!(
                "{} derived: rim/tyre {:.2}/{:.2} kg, shoulder radius {:.3} m, bead width {:.3} m, inflated radius {:.4} m, mount {:.3?} m",
                WHEEL_NAMES[index],
                wheel.bare_rim_mass_kg,
                wheel.tyre.material.total_mass,
                geometry.shoulder_radius,
                geometry.bead_width,
                radii[index],
                mount.to_array()
            );
        }
        let tyre = self.specification.wheels[0].unwrap().tyre;
        println!(
            "Reference tyre material: {} segments, ambient {:.0} Pa, mass in kg and spring/damper coefficients in N/m and N·s/m. Beams {:?}, reinforcement {:?}",
            tyre.geometry.segments, tyre.ambient_pressure_pa, tyre.material, tyre.reinforcement
        );
        println!(
            "Solver: {TICKS_PER_SECOND} ticks/s, {SUBSTEPS} substeps per tick, gravity {G} m/s², initial tread clearance {SPAWN_CLEARANCE_M} m"
        );
        println!(
            "Derived spawn: {:?} m, {} collision triangles from imported terrain",
            self.start.origin_m.to_array(),
            self.collision.triangles.len()
        );
    }
}

fn resolve_wheels(
    geometry: &[WheelGeometry],
    details: &[WheelPhysics],
) -> Result<[WheelInput; 4], String> {
    if geometry.len() != 4 || details.len() != 4 {
        return Err("driving requires four meshwheels2 wheels".into());
    }
    let mut ordered = [0, 1, 2, 3];
    ordered.sort_by(|&a, &b| geometry[b].centre.z.total_cmp(&geometry[a].centre.z));
    if geometry[ordered[1]].centre.z - geometry[ordered[2]].centre.z < 0.1 {
        return Err("cannot distinguish front and rear wheel pairs from their centres".into());
    }
    for axle in ordered.chunks_exact_mut(2) {
        if details[axle[0]].side == WheelSide::Right {
            axle.swap(0, 1);
        }
        let [left, right] = [axle[0], axle[1]];
        if details[left].side != WheelSide::Left
            || details[right].side != WheelSide::Right
            || geometry[left].centre.x <= geometry[right].centre.x
        {
            return Err("each axle needs a distinct left and right wheel in lateral order".into());
        }
    }
    let [fl, fr, rl, rr] = std::array::from_fn(|position| {
        let index = ordered[position];
        build_wheel(index, &geometry[index], details[index])
            .map_err(|error| format!("{}: {error}", WHEEL_NAMES[position]))
    });
    Ok([fl?, fr?, rl?, rr?])
}

fn build_wheel(
    geometry_index: usize,
    geometry: &WheelGeometry,
    data: WheelPhysics,
) -> Result<WheelInput, &'static str> {
    let mass = data.mass_kg.ok_or("missing positive authored wheel mass")?;
    let upright_mass_kg = data
        .upright_mass_kg
        .ok_or("missing mass at the authored axle nodes")?;
    let suspension = data
        .suspension
        .ok_or("missing a supported shock attached to the wheel")?;
    let drive_nm = data
        .drive_torque_nm
        .ok_or("missing supported first-gear torque")?;
    let brake_nm = data
        .brake_torque_nm
        .ok_or("missing supported service-brake torque")?;
    let bead = geometry.rim_radius.ok_or("missing authored rim radius")?;
    let mut tyre = reference_car::tyre_spec();
    tyre.geometry.bead_radius = bead;
    tyre.geometry.tread_radius = geometry.outer_radius;
    tyre.geometry.shoulder_radius =
        bead + (geometry.outer_radius - bead) * SHOULDER_HEIGHT_FRACTION;
    tyre.geometry.shoulder_width = geometry.width;
    tyre.geometry.bead_width = geometry.width * BEAD_WIDTH_FRACTION;
    tyre.material.total_mass = f64::from(mass) * TYRE_MASS_FRACTION;
    tyre.gauge_pressure_pa = TYRE_GAUGE_PRESSURE_PA;
    Ok(WheelInput {
        geometry_index,
        data,
        drive_nm,
        brake_nm,
        spec: WheelSpec {
            centre_m: geometry.centre,
            initial_length_m: suspension.initial_length_m,
            suspension: Suspension {
                stiffness_n_per_m: suspension.stiffness_n_per_m,
                damping_ns_per_m: suspension.damping_ns_per_m,
                free_length_m: suspension.free_length_m,
                min_length_m: suspension.min_length_m,
                max_length_m: suspension.max_length_m,
            },
            upright_mass_kg,
            bare_rim_mass_kg: f64::from(mass) - tyre.material.total_mass,
            tyre,
        },
    })
}

fn spawn_pose(
    triangles: &[[Vec3; 3]],
    hint: Vec3,
    rotation: Quat,
    wheels: [WheelSpec; 4],
    radii: [f64; 4],
) -> Result<VehiclePose, String> {
    let mut lift = f32::NEG_INFINITY;
    for (i, wheel) in wheels.into_iter().enumerate() {
        for side in [-0.5, 0.0, 0.5] {
            let point = wheel.centre_m + Vec3::X * side * wheel.tyre.geometry.shoulder_width;
            let sample = rotation * point + hint.with_y(0.0);
            let support =
                placement::support_below(triangles, sample.with_y(hint.y)).ok_or_else(|| {
                    format!(
                        "no support beneath {} wheel at ({:.3}, {:.3}) m",
                        WHEEL_NAMES[i], sample.x, sample.z
                    )
                })?;
            lift = lift.max(support + radii[i] as f32 - sample.y);
        }
    }
    Ok(VehiclePose {
        origin_m: hint.with_y(lift + SPAWN_CLEARANCE_M),
        rotation,
    })
}

#[derive(Clone, Copy, Default)]
pub(crate) struct DriverInput {
    pub throttle: f32,
    pub brake: f32,
    pub steer: f32,
}

pub(crate) struct DriveSession {
    pub setup: DriveSetup,
    app: App,
    car: VehicleAssembly,
    commands: DriverInput,
}

impl DriveSession {
    pub(crate) fn new(setup: DriveSetup) -> Result<Self, String> {
        let mut app = App::new();
        app.add_plugins((
            MinimalPlugins,
            TransformPlugin,
            PhysicsPlugins::default(),
            AvianTyrePlugin,
            AvianSuspensionPlugin,
        ))
        .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
            1.0 / f64::from(TICKS_PER_SECOND),
        )))
        .insert_resource(Time::<Fixed>::from_hz(f64::from(TICKS_PER_SECOND)))
        .insert_resource(SubstepCount(SUBSTEPS))
        .insert_resource(Gravity(Vec3::NEG_Y * G as f32));
        app.finish();
        app.cleanup();
        app.update();
        let car = setup
            .prepared
            .spawn(app.world_mut(), setup.start, &setup.road)
            .map_err(|error| error.to_string())?;
        Ok(Self {
            setup,
            app,
            car,
            commands: DriverInput::default(),
        })
    }

    pub(crate) fn reset(&mut self, pose: VehiclePose) -> Result<(), String> {
        let world = self.app.world_mut();
        let replacement = self
            .setup
            .prepared
            .spawn(world, pose, &self.setup.road)
            .map_err(|error| error.to_string())?;
        std::mem::replace(&mut self.car, replacement).despawn(world);
        self.commands = DriverInput::default();
        Ok(())
    }

    pub(crate) fn step(&mut self, input: DriverInput) -> Result<(), String> {
        let approach = |old: f32, target: f32, per_second: f32| {
            let step = per_second / TICKS_PER_SECOND as f32;
            old + (target - old).clamp(-step, step)
        };
        self.commands.throttle = approach(self.commands.throttle, input.throttle, 2.0);
        self.commands.brake = approach(self.commands.brake, input.brake, 5.0);
        self.commands.steer = approach(self.commands.steer, input.steer, 1.8);
        let [left, right] = self
            .car
            .geometry
            .front_steer(self.commands.steer * MAX_STEER_DEG.to_radians())
            .expect("validated steering limit");
        let world = self.app.world_mut();
        for (i, wheel) in self.car.wheels.iter().enumerate() {
            let input_wheel = self.setup.wheels[i];
            *world
                .get_mut::<WheelControl>(wheel.spin_joint)
                .expect("spawned wheel") = WheelControl {
                steer_rad: [left, right, 0.0, 0.0][i],
                drive_nm: if input.brake > 0.0 || self.commands.brake > 0.0 {
                    0.0
                } else {
                    self.commands.throttle * input_wheel.drive_nm
                },
                brake_nm: self.commands.brake * input_wheel.brake_nm,
            };
        }
        self.app.update();
        for (i, wheel) in self.car.wheels.iter().enumerate() {
            if let Some(error) = self
                .app
                .world()
                .get::<AvianTyre>(wheel.rim)
                .expect("spawned wheel")
                .error()
            {
                return Err(format!("{} tyre simulation: {error}", WHEEL_NAMES[i]));
            }
        }
        Ok(())
    }

    pub(crate) fn chassis(&self) -> (Vec3, Quat, Vec3) {
        let world = self.app.world();
        let id = self.car.chassis;
        (
            world.get::<Position>(id).unwrap().0,
            world.get::<Rotation>(id).unwrap().0,
            world.get::<LinearVelocity>(id).unwrap().0,
        )
    }

    pub(crate) fn rims(&self) -> [RimMotion; 4] {
        self.car.wheels.map(|wheel| {
            self.app
                .world()
                .get::<AvianTyre>(wheel.rim)
                .unwrap()
                .tyre()
                .rim()
        })
    }

    pub(crate) fn support(&self) -> [f64; 4] {
        self.car.wheels.map(|wheel| {
            let reaction = self
                .app
                .world()
                .get::<AvianTyre>(wheel.rim)
                .unwrap()
                .tyre()
                .triangle_reaction();
            reaction.normal_force.y + reaction.tangential_force.y
        })
    }

    pub(crate) fn clearance_m(&self) -> f32 {
        self.rims()
            .into_iter()
            .zip(self.specification_wheels())
            .map(|(rim, wheel)| {
                let centre = rim.centre.as_vec3();
                placement::support_below(&self.setup.collision.triangles, centre)
                    .map_or(f32::NEG_INFINITY, |height| {
                        centre.y - height - wheel.tyre.geometry.bead_radius
                    })
            })
            .fold(f32::INFINITY, f32::min)
    }

    fn specification_wheels(&self) -> [WheelSpec; 4] {
        self.setup.specification.wheels.map(Option::unwrap)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vehicle_assets::{
        ChassisPhysics, Heightfield, MeshAppearance, MeshId, MeshInstance, TerrainAsset,
        TriangleMesh, VehicleAsset, VehiclePhysics, VerticalSuspension,
    };

    fn fixture_vehicle() -> VehicleImport {
        // Deliberately shuffle the two axles and the left/right order.
        let points = [(-0.8, -1.4), (0.7, 1.25), (0.8, -1.4), (-0.7, 1.25)];
        let wheels = points
            .map(|(x, z)| WheelGeometry {
                centre: Vec3::new(x, if z > 0.0 { 0.27 } else { 0.20 }, z),
                axle: Vec3::X,
                width: 0.205,
                outer_radius: 0.31,
                rim_radius: Some(0.19),
            })
            .to_vec();
        let wheel_details = points
            .into_iter()
            .enumerate()
            .map(|(i, (x, z))| WheelPhysics {
                side: if x > 0.0 {
                    WheelSide::Left
                } else {
                    WheelSide::Right
                },
                mass_kg: Some(37.0 + i as f32),
                upright_mass_kg: Some(3.0),
                suspension: Some(VerticalSuspension {
                    stiffness_n_per_m: 32000.0,
                    damping_ns_per_m: 2200.0,
                    free_length_m: 0.26,
                    min_length_m: 0.06,
                    max_length_m: 0.33,
                    initial_length_m: 0.17,
                }),
                drive_torque_nm: Some(if z > 0.0 { 340.0 } else { 0.0 }),
                brake_torque_nm: Some(480.0),
                rim_object: i * 2,
                tyre_object: i * 2 + 1,
            })
            .collect();
        VehicleImport {
            vehicle: VehicleAsset {
                name: "fixture".into(),
                meshes: Vec::new(),
                objects: Vec::new(),
                nodes: Vec::new(),
                edges: Vec::new(),
                wheels,
                physics: VehiclePhysics {
                    chassis: Some(ChassisPhysics {
                        mass_kg: 1080.0,
                        size_m: Vec3::new(1.68, 0.50, 4.46),
                        centre_of_mass_m: Vec3::new(0.0, 0.45, 0.22),
                    }),
                    wheels: wheel_details,
                },
            },
            appearance: MeshAppearance::default(),
            warnings: Vec::new(),
            entry: "fixture.truck".into(),
        }
    }

    fn fixture_terrain() -> TerrainImport {
        let square = TriangleMesh {
            vertices: vec![
                -Vec3::X - Vec3::Z,
                -Vec3::X + Vec3::Z,
                Vec3::X + Vec3::Z,
                Vec3::X - Vec3::Z,
            ],
            triangles: vec![[0, 1, 2], [0, 2, 3]],
            normals: None,
            uvs: None,
        };
        let instance = |name: &str, y| MeshInstance {
            name: name.into(),
            translation: Vec3::new(10.0, y, 20.0),
            rotation: Quat::from_rotation_y(0.3),
            scale: Vec3::new(-5.0, 1.0, 4.0),
            visual_mesh: None,
            collision_mesh: Some(MeshId(0)),
        };
        TerrainImport {
            terrain: TerrainAsset {
                name: "fixture terrain".into(),
                heightfield: Some(Heightfield {
                    origin: Vec3::ZERO,
                    samples_x: 2,
                    samples_z: 2,
                    spacing_x: 50.0,
                    spacing_z: 50.0,
                    heights: vec![0.0; 4],
                }),
                meshes: vec![square],
                objects: vec![instance("floor", 2.0), instance("deck", 8.0)],
            },
            start_position: Some(Vec3::new(10.0, 5.0, 20.0)),
            appearance: MeshAppearance::default(),
            warnings: Vec::new(),
        }
    }

    #[test]
    fn shuffled_wheels_share_transformed_road_and_reset_cleanly() {
        let mut vehicle = fixture_vehicle();
        let terrain = fixture_terrain();
        let setup = DriveSetup::new(None, &terrain, &vehicle).unwrap();
        assert_eq!(setup.wheel_visuals().map(|(i, _)| i), [1, 3, 2, 0]);
        for (i, spec) in setup
            .specification
            .wheels
            .map(Option::unwrap)
            .into_iter()
            .enumerate()
        {
            let sum = spec.bare_rim_mass_kg + spec.tyre.material.total_mass;
            assert_eq!(sum, f64::from(setup.wheels[i].data.mass_kg.unwrap()));
        }
        let floor_triangle = setup.collision.triangles[2];
        assert!(floor_triangle[0].abs_diff_eq(
            terrain.terrain.objects[0].transform_point(terrain.terrain.meshes[0].vertices[0]),
            1e-6
        ));
        assert!(
            (2.10..2.20).contains(&setup.start.origin_m.y),
            "spawn beneath deck: {:?}",
            setup.start.origin_m
        );
        assert!(
            setup
                .pose(Vec3::new(-10.0, 5.0, 0.0), Quat::IDENTITY)
                .unwrap_err()
                .contains("FL")
        );
        let mut run = DriveSession::new(setup).unwrap();
        let count = run.app.world().entities().count_spawned();
        let original = run.chassis().0;
        for _ in 0..3 {
            for _ in 0..15 {
                run.step(DriverInput {
                    throttle: 1.0,
                    steer: 0.5,
                    ..Default::default()
                })
                .unwrap();
            }
            for (i, corner) in run.car.wheels.iter().enumerate() {
                let input = run
                    .app
                    .world()
                    .get::<WheelControl>(corner.spin_joint)
                    .unwrap();
                assert_eq!(input.drive_nm > 0.0, i < 2);
                assert_eq!(input.steer_rad > 0.0, i < 2);
            }
            run.reset(run.setup.start).unwrap();
            assert_eq!(run.app.world().entities().count_spawned(), count);
            assert_eq!(run.chassis().0, original);
            assert_eq!(run.chassis().2, Vec3::ZERO);
            assert!(
                run.rims()
                    .iter()
                    .all(|rim| rim.angular_velocity == DVec3::ZERO)
            );
        }
        vehicle.vehicle.physics.wheels[0].side = WheelSide::Left;
        assert!(resolve_wheels(&vehicle.vehicle.wheels, &vehicle.vehicle.physics.wheels).is_err());
    }
}

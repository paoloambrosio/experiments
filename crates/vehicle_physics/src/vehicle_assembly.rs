//! Assemble a four-wheel vehicle using the Avian suspension adapter.
//! Vehicle-local +Y is up, +Z forward, and +X left. Inputs use SI units.

use std::{error::Error, fmt, sync::Arc};

use avian3d::prelude::*;
use bevy::{math::DVec3, prelude::*};

use crate::{
    avian_suspension::{CornerConfig, CornerError, Suspension, WheelCorner, ackermann_steer},
    elastic_tyre::ElasticTyre,
    inflation::{TyrePreparationError, unloaded_radius},
    mounted_tyre::{ContactParameters, FlatFloor, MountedTyre, RimMotion},
    triangle_road::TriangleRoad,
};

pub use crate::{avian_suspension::WheelAlignment, inflation::TyreSpec};

pub const WHEEL_NAMES: [&str; 4] = ["FL", "FR", "RL", "RR"];

#[derive(Clone, Copy, Debug)]
pub struct ChassisSpec {
    /// Sprung mass, excluding all four wheel assemblies.
    pub mass_kg: f32,
    /// Uniform-box inertia dimensions in local X, Y, Z. The chassis is collider-free.
    pub size_m: Vec3,
    /// Centre of mass in metres relative to the vehicle origin.
    pub centre_of_mass_m: Vec3,
}

#[derive(Clone, Copy, Debug)]
pub struct WheelSpec {
    /// Centre in vehicle-local metres at the specified initial suspension length.
    pub centre_m: Vec3,
    /// Distance down from the suspension mount to the wheel centre at spawn.
    pub initial_length_m: f32,
    pub suspension: Suspension,
    pub upright_mass_kg: f32,
    pub bare_rim_mass_kg: f64,
    pub tyre: TyreSpec,
}

impl WheelSpec {
    pub(crate) fn corner_config(self, centre_of_mass_m: Vec3) -> CornerConfig {
        CornerConfig {
            mount: self.centre_m + Vec3::Y * self.initial_length_m - centre_of_mass_m,
            suspension: self.suspension,
            upright_mass_kg: self.upright_mass_kg,
            bare_rim_mass_kg: self.bare_rim_mass_kg,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct VehicleSpec {
    pub chassis: ChassisSpec,
    /// Front left, front right, rear left, rear right. None denotes a missing input.
    pub wheels: [Option<WheelSpec>; 4],
}

#[derive(Clone, Copy, Debug, Default)]
pub struct VehiclePose {
    /// Vehicle origin in world metres.
    pub origin_m: Vec3,
    /// Unit rotation from vehicle coordinates to world coordinates.
    pub rotation: Quat,
}

#[derive(Clone, Debug)]
pub enum VehicleRoad {
    Flat(FlatFloor),
    Triangles {
        road: Arc<TriangleRoad>,
        contact: ContactParameters,
    },
}

#[derive(Clone, Copy, Debug)]
pub struct VehicleGeometry {
    pub wheelbase_m: f32,
    pub front_track_m: f32,
    pub rear_track_m: f32,
}

impl VehicleGeometry {
    /// Front wheel angles in FL, FR order for the given equivalent steer in radians.
    pub fn front_steer(self, steer_rad: f32) -> Option<[f32; 2]> {
        ackermann_steer(self.wheelbase_m, self.front_track_m, steer_rad)
    }
}

#[derive(Debug)]
pub enum AssemblyError {
    MissingWheel(&'static str),
    InvalidChassis(&'static str),
    InvalidWheel {
        wheel: &'static str,
        field: &'static str,
    },
    InvalidPose(&'static str),
    Tyre {
        wheel: &'static str,
        source: TyrePreparationError,
    },
    Corner {
        wheel: &'static str,
        source: CornerError,
    },
}

impl fmt::Display for AssemblyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingWheel(wheel) => write!(f, "missing {wheel} wheel specification"),
            Self::InvalidChassis(field) => write!(f, "invalid chassis.{field}"),
            Self::InvalidWheel { wheel, field } => write!(f, "invalid {wheel} wheel {field}"),
            Self::InvalidPose(field) => write!(f, "invalid vehicle pose {field}"),
            Self::Tyre { wheel, source } => write!(f, "{wheel} wheel {source}"),
            Self::Corner { wheel, source } => write!(f, "{wheel} wheel: {source}"),
        }
    }
}

impl Error for AssemblyError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Tyre { source, .. } => Some(source),
            Self::Corner { source, .. } => Some(source),
            _ => None,
        }
    }
}

struct VehicleLayout {
    chassis: ChassisSpec,
    inertia: Vec3,
    wheels: [WheelSpec; 4],
    corners: [CornerConfig; 4],
    wheel_alignments: [WheelAlignment; 4],
    geometry: VehicleGeometry,
}

/// A validated vehicle and its sealed tyres, reusable across spawns.
pub struct PreparedVehicle {
    layout: VehicleLayout,
    tyres: [Arc<ElasticTyre>; 4],
}

impl VehicleSpec {
    pub(crate) fn validated_geometry(self) -> Result<VehicleGeometry, AssemblyError> {
        self.layout().map(|layout| layout.geometry)
    }

    /// Validate the layout and prepare each distinct tyre once at the given substeps per second.
    pub fn prepare(self, rate: u32) -> Result<PreparedVehicle, AssemblyError> {
        let layout = self.layout()?;
        let mut tyres = Vec::with_capacity(4);
        for (i, wheel) in layout.wheels.iter().enumerate() {
            let tyre = if let Some(previous) = layout.wheels[..i]
                .iter()
                .position(|other| other.tyre == wheel.tyre)
            {
                Arc::clone(&tyres[previous])
            } else {
                Arc::new(
                    wheel
                        .tyre
                        .prepare(rate)
                        .map_err(|source| AssemblyError::Tyre {
                            wheel: WHEEL_NAMES[i],
                            source,
                        })?,
                )
            };
            tyres.push(tyre);
        }
        let [a, b, c, d] = tyres.try_into().ok().expect("four wheel specifications");
        Ok(PreparedVehicle {
            layout,
            tyres: [a, b, c, d],
        })
    }

    // Compatibility path for callers that already own a prepared reference tyre.
    pub(crate) fn spawn_with_tyres(
        self,
        world: &mut World,
        tyres: [&ElasticTyre; 4],
        pose: VehiclePose,
        road: &VehicleRoad,
    ) -> Result<VehicleAssembly, AssemblyError> {
        self.layout()?.spawn(world, tyres, pose, road)
    }

    fn layout(self) -> Result<VehicleLayout, AssemblyError> {
        let chassis = self.chassis;
        if !chassis.mass_kg.is_finite() || chassis.mass_kg <= 0.0 {
            return Err(AssemblyError::InvalidChassis("mass_kg"));
        }
        if !chassis.size_m.is_finite() || chassis.size_m.min_element() <= 0.0 {
            return Err(AssemblyError::InvalidChassis("size_m"));
        }
        if !chassis.centre_of_mass_m.is_finite() {
            return Err(AssemblyError::InvalidChassis("centre_of_mass_m"));
        }
        let inertia = box_inertia(chassis.mass_kg, chassis.size_m);
        if !inertia.is_finite() || inertia.min_element() <= 0.0 {
            return Err(AssemblyError::InvalidChassis(
                "inertia from mass_kg and size_m",
            ));
        }

        let [fl, fr, rl, rr] = self.wheels;
        let wheels = [
            fl.ok_or(AssemblyError::MissingWheel("FL"))?,
            fr.ok_or(AssemblyError::MissingWheel("FR"))?,
            rl.ok_or(AssemblyError::MissingWheel("RL"))?,
            rr.ok_or(AssemblyError::MissingWheel("RR"))?,
        ];
        let corners = wheels.map(|wheel| wheel.corner_config(chassis.centre_of_mass_m));
        for (i, (wheel, corner)) in wheels.iter().zip(corners).enumerate() {
            let invalid = |field| AssemblyError::InvalidWheel {
                wheel: WHEEL_NAMES[i],
                field,
            };
            if !wheel.centre_m.is_finite() {
                return Err(invalid("centre_m"));
            }
            if !wheel.bare_rim_mass_kg.is_finite() || wheel.bare_rim_mass_kg <= 0.0 {
                return Err(invalid("bare_rim_mass_kg"));
            }
            let limits = wheel.suspension.min_length_m..=wheel.suspension.max_length_m;
            if !wheel.initial_length_m.is_finite() {
                return Err(invalid("initial_length_m"));
            }
            if let Some(field) = corner.invalid_field() {
                return Err(invalid(field));
            }
            if !limits.contains(&wheel.initial_length_m) {
                return Err(invalid("initial_length_m outside suspension travel"));
            }
        }
        let [fl, fr, rl, rr] = wheels.map(|wheel| wheel.centre_m);
        let geometry = VehicleGeometry {
            front_track_m: fl.x - fr.x,
            rear_track_m: rl.x - rr.x,
            wheelbase_m: (fl.z * 0.5 + fr.z * 0.5) - (rl.z * 0.5 + rr.z * 0.5),
        };
        for (dimension, wheel, field) in [
            (geometry.front_track_m, "FL", "centre_m relative to FR"),
            (geometry.rear_track_m, "RL", "centre_m relative to RR"),
            (
                geometry.wheelbase_m,
                "FL/FR",
                "centre_m relative to the rear axle",
            ),
        ] {
            if !dimension.is_finite() || dimension <= 0.0 {
                return Err(AssemblyError::InvalidWheel { wheel, field });
            }
        }
        Ok(VehicleLayout {
            chassis,
            inertia,
            wheels,
            corners,
            wheel_alignments: [WheelAlignment::default(); 4],
            geometry,
        })
    }
}

impl PreparedVehicle {
    /// Set chassis-local rest camber and toe in FL, FR, RL, RR order. Preparation
    /// defaults to upright wheels, and alignment does not change the inflated tyres.
    pub fn with_wheel_alignments(mut self, alignments: [WheelAlignment; 4]) -> Self {
        self.layout.wheel_alignments = alignments;
        self
    }

    pub fn wheel_alignments(&self) -> [WheelAlignment; 4] {
        self.layout.wheel_alignments
    }

    pub fn geometry(&self) -> VehicleGeometry {
        self.layout.geometry
    }

    /// Unloaded radii along tyre-local -Y after inflation, in FL, FR, RL, RR order.
    pub fn unloaded_radii_m(&self) -> [f64; 4] {
        std::array::from_fn(|i| unloaded_radius(&self.tyres[i]))
    }

    /// Inflated tread nodes in tyre-local metres, in FL, FR, RL, RR order.
    pub fn unloaded_tread_positions_m(&self) -> [&[DVec3]; 4] {
        self.tyres.each_ref().map(|tyre| tyre.tread_positions_m())
    }

    /// Spawn four independent tyres and their chassis at an explicit world pose.
    pub fn spawn(
        &self,
        world: &mut World,
        pose: VehiclePose,
        road: &VehicleRoad,
    ) -> Result<VehicleAssembly, AssemblyError> {
        self.layout
            .spawn(world, self.tyres.each_ref().map(Arc::as_ref), pose, road)
    }
}

impl VehicleLayout {
    fn spawn(
        &self,
        world: &mut World,
        tyres: [&ElasticTyre; 4],
        pose: VehiclePose,
        road: &VehicleRoad,
    ) -> Result<VehicleAssembly, AssemblyError> {
        if !pose.origin_m.is_finite() {
            return Err(AssemblyError::InvalidPose("origin_m"));
        }
        if !pose.rotation.is_normalized() {
            return Err(AssemblyError::InvalidPose("rotation"));
        }
        let world_point = |point| pose.origin_m + pose.rotation * point;
        let centre = world_point(self.chassis.centre_of_mass_m);
        if !centre.is_finite() {
            return Err(AssemblyError::InvalidPose(
                "chassis centre in world coordinates",
            ));
        }
        let [fl, fr, rl, rr] = std::array::from_fn(|i| {
            let error = |source| AssemblyError::Corner {
                wheel: WHEEL_NAMES[i],
                source,
            };
            let motion = RimMotion {
                centre: world_point(self.wheels[i].centre_m).as_dvec3(),
                rotation: (pose.rotation * self.wheel_alignments[i].rotation())
                    .as_dquat()
                    .normalize(),
                ..Default::default()
            };
            let floor = match road {
                VehicleRoad::Flat(floor) => Some(*floor),
                VehicleRoad::Triangles { .. } => None,
            };
            let mut tyre = MountedTyre::new(tyres[i].clone(), motion, floor)
                .map_err(CornerError::Tyre)
                .map_err(error)?;
            if let VehicleRoad::Triangles { road, contact } = road {
                tyre.set_triangle_road(Arc::clone(road), *contact);
            }
            WheelCorner::prepare(tyre, self.corners[i]).map_err(error)
        });
        let corners = [fl?, fr?, rl?, rr?];
        let chassis = spawn_chassis_body(
            world,
            self.chassis.mass_kg,
            centre,
            pose.rotation,
            self.inertia,
        );
        let mut alignments = self.wheel_alignments.into_iter();
        let wheels = corners.map(|corner| {
            let alignment = alignments.next().unwrap();
            corner.spawn(world, chassis, pose.rotation, alignment)
        });
        Ok(VehicleAssembly {
            chassis,
            wheels,
            geometry: self.geometry,
        })
    }
}

pub(crate) fn box_inertia(mass_kg: f32, size_m: Vec3) -> Vec3 {
    let squared = size_m * size_m;
    (Vec3::splat(squared.element_sum()) - squared) * (mass_kg / 12.0)
}

pub(crate) fn spawn_chassis_body(
    world: &mut World,
    mass_kg: f32,
    centre: Vec3,
    rotation: Quat,
    inertia: Vec3,
) -> Entity {
    world
        .spawn((
            RigidBody::Dynamic,
            SleepingDisabled,
            Position(centre),
            Rotation(rotation),
            Mass(mass_kg),
            AngularInertia::new(inertia),
        ))
        .id()
}

/// Handles to a spawned chassis and its four corners.
pub struct VehicleAssembly {
    pub chassis: Entity,
    pub wheels: [WheelCorner; 4],
    pub geometry: VehicleGeometry,
}

impl VehicleAssembly {
    /// Remove the four corners, their joints, and the chassis.
    pub fn despawn(self, world: &mut World) {
        despawn(world, self.chassis, self.wheels);
    }
}

pub(crate) fn despawn(world: &mut World, chassis: Entity, wheels: [WheelCorner; 4]) {
    for wheel in wheels {
        for entity in [
            wheel.spin_joint,
            wheel.suspension_joint,
            wheel.rim,
            wheel.hub,
        ] {
            world.despawn(entity);
        }
    }
    world.despawn(chassis);
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::*;
    use crate::{
        avian_suspension::{AvianSuspensionPlugin, WheelControl},
        avian_tyre::AvianTyrePlugin,
        reference_car,
    };
    use bevy::time::TimeUpdateStrategy;

    #[test]
    fn wheel_alignment_builds_a_basis_from_a_unit_leftward_axle() {
        assert_eq!(WheelAlignment::default().rotation(), Quat::IDENTITY);
        for invalid in [Vec3::ZERO, Vec3::NAN, Vec3::Y, Vec3::NEG_X, 2.0 * Vec3::X] {
            assert!(WheelAlignment::from_axle(invalid).is_none(), "{invalid}");
        }
        for side in [-1.0, 1.0] {
            let axle = Vec3::new(1.0, side * 0.08, side * 0.03).normalize();
            let rotation = WheelAlignment::from_axle(axle).unwrap().rotation();
            assert!((rotation * Vec3::X).abs_diff_eq(axle, 1e-6));
            let projected_up = (Vec3::Y - axle.y * axle).normalize();
            assert!((rotation * Vec3::Y).abs_diff_eq(projected_up, 1e-6));
        }
    }

    #[test]
    fn aligned_wheel_steers_and_spins_on_a_vertical_slider() {
        let spec = reference_car::specification();
        let axle = Vec3::new(1.0, -0.08, 0.04).normalize();
        let wheel = spec.wheels[0].unwrap();
        let mut alignments = [WheelAlignment::default(); 4];
        alignments[0] = WheelAlignment::from_axle(axle).unwrap();
        let initial_length = wheel.initial_length_m;
        let mount = wheel.centre_m + Vec3::Y * initial_length;
        let pose = VehiclePose {
            origin_m: Vec3::Y * 2.0,
            rotation: Quat::from_rotation_y(0.45) * Quat::from_rotation_z(0.12),
        };
        let mut app = App::new();
        app.add_plugins((
            MinimalPlugins,
            TransformPlugin,
            PhysicsPlugins::default(),
            AvianTyrePlugin,
            AvianSuspensionPlugin,
        ))
        .insert_resource(Gravity(Vec3::NEG_Y * 9.81))
        .insert_resource(Time::<Fixed>::from_hz(60.0))
        .insert_resource(SubstepCount(64))
        .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
            1.0 / 60.0,
        )));
        app.finish();
        app.cleanup();
        app.update();
        let car = spec
            .prepare(3840)
            .unwrap()
            .with_wheel_alignments(alignments)
            .spawn(
                app.world_mut(),
                pose,
                &VehicleRoad::Flat(FlatFloor::new(-20.0, 100_000.0, 150.0).unwrap()),
            )
            .unwrap();
        app.world_mut()
            .entity_mut(car.chassis)
            .insert(RigidBody::Static);
        let corner = car.wheels[0];

        let check_axes = |app: &App, steer: f32| {
            let world = app.world();
            let chassis = world.get::<Rotation>(car.chassis).unwrap().0;
            let hub = world.get::<Rotation>(corner.hub).unwrap().0;
            let rim = world.get::<Rotation>(corner.rim).unwrap().0;
            let joint = world.get::<RevoluteJoint>(corner.spin_joint).unwrap();
            let expected = chassis * (Quat::from_rotation_y(steer) * axle);
            assert!((hub * Vec3::Y).abs_diff_eq(chassis * Vec3::Y, 2e-3));
            assert!((hub * joint.local_hinge_axis1().unwrap()).abs_diff_eq(expected, 2e-3));
            assert!(
                (rim * Vec3::X).abs_diff_eq(expected, 2e-3),
                "steer {steer}: rim axis {:?}, expected {expected:?}",
                rim * Vec3::X
            );
            expected
        };
        check_axes(&app, 0.0);
        let command = |app: &mut App, steer_rad, drive_nm, brake_nm| {
            *app.world_mut()
                .get_mut::<WheelControl>(corner.spin_joint)
                .unwrap() = WheelControl {
                steer_rad,
                drive_nm,
                brake_nm,
            };
        };
        let steer = 0.35;
        for tick in 1..=15 {
            command(&mut app, steer * tick as f32 / 15.0, 0.0, 0.0);
            app.update();
        }
        for _ in 0..12 {
            app.update();
        }
        check_axes(&app, steer);

        command(&mut app, steer, 20.0, 0.0);
        for _ in 0..12 {
            app.update();
        }
        let axis = check_axes(&app, steer);
        let world = app.world();
        let spin = world.get::<AngularVelocity>(corner.rim).unwrap().0
            - world.get::<AngularVelocity>(corner.hub).unwrap().0;
        assert!(spin.dot(axis) > 1.0, "{spin:?}");
        assert!(spin.cross(axis).length() < 0.1, "{spin:?}");
        let hub_centre = world.get::<Position>(corner.hub).unwrap().0;
        let travel = pose.rotation.inverse() * (hub_centre - pose.origin_m) - mount;
        assert!(travel.with_y(0.0).length() < 0.002, "{travel:?}");
        assert!(-travel.y > initial_length + 0.01, "{travel:?}");

        for tick in (0..15).rev() {
            command(&mut app, steer * tick as f32 / 15.0, 0.0, 500.0);
            app.update();
        }
        for _ in 0..12 {
            app.update();
        }
        let axis = check_axes(&app, 0.0);
        let stopped = app
            .world()
            .get::<AngularVelocity>(corner.rim)
            .unwrap()
            .0
            .dot(axis);
        assert!(stopped.abs() < 0.1, "{stopped}");
    }
}

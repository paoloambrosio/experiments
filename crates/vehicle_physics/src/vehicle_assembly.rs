//! Assemble an upright four-wheel vehicle using the Avian suspension adapter.
//! Vehicle-local +Y is up, +Z forward, and +X left. Inputs use SI units.

use std::{error::Error, fmt, sync::Arc};

use avian3d::prelude::*;
use bevy::prelude::*;

use crate::{
    avian_suspension::{CornerConfig, CornerError, Suspension, WheelCorner, ackermann_steer},
    elastic_tyre::ElasticTyre,
    inflation::{TyrePreparationError, unloaded_radius},
    mounted_tyre::{ContactParameters, FlatFloor, MountedTyre, RimMotion},
    triangle_road::TriangleRoad,
};

pub use crate::inflation::TyreSpec;

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
    fn corner_config(self, centre_of_mass_m: Vec3) -> CornerConfig {
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
    geometry: VehicleGeometry,
}

/// A validated vehicle and its sealed tyres, reusable across spawns.
pub struct PreparedVehicle {
    layout: VehicleLayout,
    tyres: [Arc<ElasticTyre>; 4],
}

impl VehicleSpec {
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
            geometry,
        })
    }
}

impl PreparedVehicle {
    pub fn geometry(&self) -> VehicleGeometry {
        self.layout.geometry
    }

    /// Unloaded radii along tyre-local -Y after inflation, in FL, FR, RL, RR order.
    pub fn unloaded_radii_m(&self) -> [f64; 4] {
        std::array::from_fn(|i| unloaded_radius(&self.tyres[i]))
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
                rotation: pose.rotation.as_dquat().normalize(),
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
        let wheels = corners.map(|corner| corner.spawn(world, chassis));
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

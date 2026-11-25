//! The 1,000 kg reference car. World +Y is up, +Z forward and +X left.
//! Lengths are metres, masses kilograms and times seconds.
use crate::{
    avian_suspension::{CornerConfig, CornerError, Suspension, WheelCorner},
    elastic_tyre::{ElasticTyre, ElasticTyreConfig},
    inflation::Inflation,
    mounted_tyre::{FlatFloor, MountedTyre, RimMotion},
    pressure::PressureConfig,
    tread_reinforcement::TreadReinforcementConfig,
    tyre::{TyreConfig, TyreTopology},
};
use avian3d::prelude::*;
use bevy::{math::DVec3, prelude::*};
pub const G: f64 = 9.81;
pub const NOMINAL_LOAD_N: f64 = 250.0 * G;
pub const FRICTION: f64 = 0.8;
pub const PRESSURE_PA: f64 = 150_000.0;
pub const BARE_RIM_KG: f64 = 9.0;
pub const UPRIGHT_KG: f32 = 3.0;
pub const BEAD_RADIUS_M: f64 = 0.20;

pub const SUSPENSION: Suspension = Suspension {
    stiffness_n_per_m: 30_000.0,
    damping_ns_per_m: 2_000.0,
    free_length_m: 0.25,
    min_length_m: 0.06,
    max_length_m: 0.29,
};
pub const SAG_M: f64 = 230.0 * G / 30_000.0;

pub const BODY_SIZE: Vec3 = Vec3::new(1.5, 0.5, 3.6);
pub const COM_HEIGHT: f32 = 0.50;
pub const WHEELBASE: f32 = 2.5;
pub const TRACK: f32 = 1.5;
pub const WHEEL_NAMES: [&str; 4] = ["FL", "FR", "RL", "RR"];
// The 008 nominal-load static fixture puts the axle at about 326.14 mm.
const MOUNT_Y: f32 = 0.32614 + SUSPENSION.free_length_m - SAG_M as f32 - COM_HEIGHT;

pub const MOUNTS: [Vec3; 4] = [
    Vec3::new(TRACK / 2.0, MOUNT_Y, WHEELBASE / 2.0),
    Vec3::new(-TRACK / 2.0, MOUNT_Y, WHEELBASE / 2.0),
    Vec3::new(TRACK / 2.0, MOUNT_Y, -WHEELBASE / 2.0),
    Vec3::new(-TRACK / 2.0, MOUNT_Y, -WHEELBASE / 2.0),
];

pub fn road() -> FlatFloor {
    FlatFloor::new(0.0, 500_000.0, 150.0)
        .unwrap()
        .with_friction(FRICTION, 0.1)
        .unwrap()
}
fn tyre_material() -> (ElasticTyreConfig, TreadReinforcementConfig) {
    let mut material = ElasticTyreConfig::default();
    material.circumferential.stiffness *= 2.5;
    material.cross_section.stiffness *= 4.0;
    material.diagonal.stiffness *= 4.0;
    material.circumferential.damping *= 3.0;
    material.cross_section.damping *= 3.0;
    material.diagonal.damping *= 3.0;
    let mut reinforcement = TreadReinforcementConfig::default();
    reinforcement.hoop_stiffness_multiplier = 16.0;
    reinforcement.crown.stiffness *= 4.0;
    (material, reinforcement)
}
/// Inflate once per solver rate, then reuse the prepared shape for all four wheels and resets.
pub struct ReferenceTyre {
    pub tyre: ElasticTyre,
    fill: Inflation,
    forces: Vec<DVec3>,
    sealed: bool,
}
impl ReferenceTyre {
    pub fn new(rate: u32) -> Self {
        let (material, reinforcement) = tyre_material();
        let topology = TyreTopology::generate(&TyreConfig::default()).unwrap();
        let forces = vec![DVec3::ZERO; topology.nodes().len()];
        let tyre = ElasticTyre::new_inflated(topology, material, PressureConfig::default())
            .unwrap()
            .with_tread_reinforcement(reinforcement)
            .unwrap();
        Self {
            tyre,
            fill: Inflation::new(rate),
            forces,
            sealed: false,
        }
    }
    /// Advance one inflation substep. Returns true once the tyre is ready to mount.
    pub fn advance(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
        if !self.sealed {
            self.sealed = self
                .fill
                .advance(&mut self.tyre, PRESSURE_PA, &self.forces)?;
        }
        Ok(self.sealed)
    }
    pub fn is_ready(&self) -> bool {
        self.sealed
    }
    pub fn seconds(&self) -> f64 {
        self.fill.seconds()
    }
}
pub fn unloaded_radius(tyre: &ElasticTyre) -> f64 {
    -tyre
        .positions()
        .iter()
        .map(|p| p.y)
        .fold(f64::INFINITY, f64::min)
}

/// Spawn an awake rigid chassis with uniform-box inertia. Size is (width, height, length).
pub fn spawn_chassis(world: &mut World, mass_kg: f32, centre: Vec3, size: Vec3) -> Entity {
    let squared = size * size;
    let inertia = (Vec3::splat(squared.element_sum()) - squared) * (mass_kg / 12.0);
    world
        .spawn((
            RigidBody::Dynamic,
            SleepingDisabled,
            Position(centre),
            Rotation(Quat::IDENTITY),
            Mass(mass_kg),
            AngularInertia::new(inertia),
        ))
        .id()
}

/// Owner of the reference chassis and four suspension assemblies.
#[derive(Resource)]
pub struct ReferenceCar {
    pub chassis: Entity,
    /// Front left, front right, rear left, rear right.
    pub wheels: [WheelCorner; 4],
}
impl ReferenceCar {
    /// Mount four copies of a prepared tyre above the stationary road at the origin.
    pub fn spawn(world: &mut World, tyre: &ElasticTyre) -> Result<Self, CornerError> {
        let height =
            unloaded_radius(tyre) as f32 + SUSPENSION.free_length_m - SAG_M as f32 - MOUNT_Y;
        let mounted = MOUNTS.map(|mount| {
            let centre =
                Vec3::Y * height + mount - Vec3::Y * (SUSPENSION.free_length_m - SAG_M as f32);
            MountedTyre::new(
                tyre.clone(),
                RimMotion {
                    centre: centre.as_dvec3(),
                    ..default()
                },
                Some(road()),
            )
        });
        let [a, b, c, d] = mounted.map(|wheel| wheel.map_err(CornerError::Tyre));
        let mounted = [a?, b?, c?, d?];
        let chassis = spawn_chassis(world, 920.0, Vec3::Y * height, BODY_SIZE);
        let mut wheels: Vec<WheelCorner> = Vec::with_capacity(4);
        for (mount, mounted) in MOUNTS.into_iter().zip(mounted) {
            match WheelCorner::spawn(
                world,
                chassis,
                mounted,
                CornerConfig {
                    mount,
                    suspension: SUSPENSION,
                    upright_mass_kg: UPRIGHT_KG,
                    bare_rim_mass_kg: BARE_RIM_KG,
                },
            ) {
                Ok(wheel) => wheels.push(wheel),
                Err(error) => {
                    for wheel in wheels {
                        remove_corner(world, wheel);
                    }
                    world.despawn(chassis);
                    return Err(error);
                }
            }
        }
        Ok(Self {
            chassis,
            wheels: wheels.try_into().unwrap(),
        })
    }
    /// Remove this assembly, including its joints. Caller-owned visuals can be parented to the bodies.
    pub fn despawn(self, world: &mut World) {
        for wheel in self.wheels {
            remove_corner(world, wheel);
        }
        world.despawn(self.chassis);
    }
}
fn remove_corner(world: &mut World, wheel: WheelCorner) {
    for entity in [
        wheel.spin_joint,
        wheel.suspension_joint,
        wheel.rim,
        wheel.hub,
    ] {
        world.despawn(entity);
    }
}

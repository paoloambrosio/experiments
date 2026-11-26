//! The 1,000 kg reference car. World +Y is up, +Z forward and +X left.
//! Lengths are metres, masses kilograms and times seconds.
use crate::{
    avian_suspension::{CornerError, Suspension, WheelCorner},
    elastic_tyre::{ElasticTyre, ElasticTyreConfig},
    inflation::{TyreInflation, TyreSpec},
    mounted_tyre::FlatFloor,
    pressure::PressureConfig,
    tread_reinforcement::TreadReinforcementConfig,
    tyre::TyreConfig,
    vehicle_assembly::{
        self, AssemblyError, ChassisSpec, VehiclePose, VehicleRoad, VehicleSpec, WheelSpec,
    },
};
use bevy::prelude::*;

pub use crate::{inflation::unloaded_radius, vehicle_assembly::WHEEL_NAMES};
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
/// The light-car tyre used in the reference handling and grip measurements.
pub fn tyre_spec() -> TyreSpec {
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
    TyreSpec {
        geometry: TyreConfig::default(),
        material,
        reinforcement,
        ambient_pressure_pa: PressureConfig::default().ambient_pressure_pa,
        gauge_pressure_pa: PRESSURE_PA,
    }
}

/// Reference geometry with the vehicle origin at the chassis centre of mass.
pub fn specification() -> VehicleSpec {
    let initial_length_m = SUSPENSION.free_length_m - SAG_M as f32;
    VehicleSpec {
        chassis: ChassisSpec {
            mass_kg: 920.0,
            size_m: BODY_SIZE,
            centre_of_mass_m: Vec3::ZERO,
        },
        wheels: MOUNTS.map(|mount| {
            Some(WheelSpec {
                centre_m: mount - Vec3::Y * initial_length_m,
                initial_length_m,
                suspension: SUSPENSION,
                upright_mass_kg: UPRIGHT_KG,
                bare_rim_mass_kg: BARE_RIM_KG,
                tyre: tyre_spec(),
            })
        }),
    }
}
/// Inflate once per solver rate, then reuse the prepared shape for all four wheels and resets.
pub struct ReferenceTyre {
    pub tyre: ElasticTyre,
    fill: TyreInflation,
}
impl ReferenceTyre {
    pub fn new(rate: u32) -> Self {
        let (tyre, fill) = tyre_spec()
            .begin(rate)
            .expect("valid reference tyre and inflation rate");
        Self { tyre, fill }
    }
    /// Advance one inflation substep. Returns true once the tyre is ready to mount.
    pub fn advance(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
        Ok(self.fill.advance(&mut self.tyre)?)
    }
    pub fn is_ready(&self) -> bool {
        self.fill.is_ready()
    }
    pub fn seconds(&self) -> f64 {
        self.fill.seconds()
    }
}

/// Spawn an awake rigid chassis with uniform-box inertia. Size is (width, height, length).
pub fn spawn_chassis(world: &mut World, mass_kg: f32, centre: Vec3, size: Vec3) -> Entity {
    vehicle_assembly::spawn_chassis_body(
        world,
        mass_kg,
        centre,
        Quat::IDENTITY,
        vehicle_assembly::box_inertia(mass_kg, size),
    )
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
        let pose = VehiclePose {
            origin_m: Vec3::Y * height,
            rotation: Quat::IDENTITY,
        };
        let assembly = specification()
            .spawn_with_tyres(world, [tyre; 4], pose, &VehicleRoad::Flat(road()))
            .map_err(|error| match error {
                AssemblyError::Corner { source, .. } => source,
                AssemblyError::InvalidPose(_) => {
                    CornerError::Tyre(crate::elastic_tyre::ElasticTyreError::InvalidRimMotion)
                }
                _ => CornerError::InvalidConfiguration,
            })?;
        Ok(Self {
            chassis: assembly.chassis,
            wheels: assembly.wheels,
        })
    }

    /// Remove this assembly, including its joints. Caller-owned visuals can be parented to the bodies.
    pub fn despawn(self, world: &mut World) {
        vehicle_assembly::despawn(world, self.chassis, self.wheels);
    }
}

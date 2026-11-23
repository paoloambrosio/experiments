//! Upright suspension for an Avian rigid chassis with deformable tyres.

use std::{error::Error, fmt};

use avian3d::{
    dynamics::{
        integrator::IntegrationSystems,
        solver::solver_body::{SolverBodies, SolverBodyIndex},
    },
    prelude::*,
};
use bevy::prelude::*;

use crate::{
    avian_tyre::{AvianTyre, TyreWrenchApplied},
    elastic_tyre::ElasticTyreError,
    mounted_tyre::MountedTyre,
};

/// Chassis-local vertical suspension, with length measured down from the mount.
#[derive(Clone, Copy, Debug)]
pub struct Suspension {
    /// Linear spring stiffness in N/m.
    pub stiffness_n_per_m: f32,
    /// Damper coefficient in N·s/m.
    pub damping_ns_per_m: f32,
    /// Unloaded length in metres.
    pub free_length_m: f32,
    /// Fully compressed length in metres.
    pub min_length_m: f32,
    /// Fully extended length in metres.
    pub max_length_m: f32,
}

/// Corner entities. Vehicle-local axes are +Y up and +Z forward, with spin about wheel-local +X.
/// The chassis mount and hub travel on the same chassis-local Y axis.
#[derive(Clone, Copy, Debug)]
pub struct WheelCorner {
    pub rim: Entity,
    pub hub: Entity,
    pub suspension_joint: Entity,
    pub spin_joint: Entity,
}

/// Mount location in chassis-local metres and unsprung rigid-body properties in SI units.
#[derive(Clone, Copy, Debug)]
pub struct CornerConfig {
    pub mount: Vec3,
    pub suspension: Suspension,
    /// Hub mass. The adapter uses an isotropic inertia of 0.02 kg·m² per kg of hub mass.
    pub upright_mass_kg: f32,
    pub bare_rim_mass_kg: f64,
}

#[derive(Debug)]
pub enum CornerError {
    MissingChassis,
    InvalidConfiguration,
    Tyre(ElasticTyreError),
}

impl fmt::Display for CornerError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingChassis => {
                formatter.write_str("chassis must be an existing Avian rigid body")
            }
            Self::InvalidConfiguration => {
                formatter.write_str("invalid suspension travel, coefficients, mount, or hub mass")
            }
            Self::Tyre(error) => write!(formatter, "cannot mount tyre: {error}"),
        }
    }
}

impl Error for CornerError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Tyre(error) => Some(error),
            _ => None,
        }
    }
}

/// Apply suspension impulses on Avian's substep clock. Install after PhysicsPlugins.
pub struct AvianSuspensionPlugin;

impl Plugin for AvianSuspensionPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            SubstepSchedule,
            apply_suspension
                .after(TyreWrenchApplied)
                .before(IntegrationSystems::Velocity),
        );
    }
}

impl WheelCorner {
    /// Connect an already positioned tyre to the chassis. The rim starts aligned with the
    /// chassis, with its centre below the mount and inside the suspension travel limits.
    pub fn spawn(
        world: &mut World,
        chassis: Entity,
        tyre: MountedTyre,
        config: CornerConfig,
    ) -> Result<Self, CornerError> {
        if world.get::<RigidBody>(chassis).is_none() {
            return Err(CornerError::MissingChassis);
        }
        let suspension = config.suspension;
        if !config.mount.is_finite()
            || !config.upright_mass_kg.is_finite()
            || config.upright_mass_kg <= 0.0
            || !suspension.stiffness_n_per_m.is_finite()
            || suspension.stiffness_n_per_m < 0.0
            || !suspension.damping_ns_per_m.is_finite()
            || suspension.damping_ns_per_m < 0.0
            || !suspension.max_length_m.is_finite()
            || suspension.min_length_m < 0.0
            || !(suspension.min_length_m..=suspension.max_length_m)
                .contains(&suspension.free_length_m)
            || suspension.min_length_m == suspension.max_length_m
        {
            return Err(CornerError::InvalidConfiguration);
        }
        let motion = tyre.rim();
        let rim_bundle =
            AvianTyre::rim_bundle(tyre, config.bare_rim_mass_kg).map_err(CornerError::Tyre)?;
        let rim = world.spawn(rim_bundle).id();
        let hub = world
            .spawn((
                RigidBody::Dynamic,
                SleepingDisabled,
                Position(motion.centre.as_vec3()),
                Rotation(motion.rotation.as_quat()),
                LinearVelocity(motion.linear_velocity.as_vec3()),
                Mass(config.upright_mass_kg),
                AngularInertia::new(Vec3::splat(config.upright_mass_kg * 0.02)),
            ))
            .id();
        let suspension_joint = world
            .spawn((
                PrismaticJoint::new(chassis, hub)
                    .with_local_anchor1(config.mount)
                    .with_local_anchor2(Vec3::ZERO)
                    .with_slider_axis(Vec3::NEG_Y)
                    .with_limits(suspension.min_length_m, suspension.max_length_m),
                SpringDamper {
                    mount: config.mount,
                    suspension,
                },
            ))
            .id();
        let spin_joint = world
            .spawn(RevoluteJoint::new(hub, rim).with_hinge_axis(Vec3::X))
            .id();
        Ok(Self {
            rim,
            hub,
            suspension_joint,
            spin_joint,
        })
    }
}

#[derive(Component)]
struct SpringDamper {
    mount: Vec3,
    suspension: Suspension,
}

fn apply_suspension(
    time: Res<Time>,
    springs: Query<(&PrismaticJoint, &SpringDamper), Without<JointDisabled>>,
    bodies: Query<(
        &SolverBodyIndex,
        &Position,
        &Rotation,
        &ComputedCenterOfMass,
        &ComputedAngularInertia,
    )>,
    mut solvers: ResMut<SolverBodies>,
) {
    for (joint, spring) in &springs {
        let Ok(
            [
                (body_index, body_pos, body_rot, body_com, angular_inertia),
                (hub_index, hub_pos, _, _, _),
            ],
        ) = bodies.get_many([joint.body1, joint.body2])
        else {
            continue;
        };
        let (Some(body), Some(hub), Some(mut body_inertia), Some(hub_inertia)) = (
            solvers.get(*body_index),
            solvers.get(*hub_index),
            solvers.get_inertia(*body_index).cloned(),
            solvers.get_inertia(*hub_index),
        ) else {
            continue;
        };
        let rotation = body.delta_rotation * body_rot.0;
        let lever = rotation * (spring.mount - body_com.0);
        let axis = rotation * Vec3::Y;
        let offset = (body_pos.0 - hub_pos.0)
            + (body.delta_position - hub.delta_position)
            + body_rot.0 * body_com.0
            + lever;
        let speed = body.linear_velocity + body.angular_velocity.cross(lever) - hub.linear_velocity;
        let settings = spring.suspension;
        let force = settings.stiffness_n_per_m * (settings.free_length_m - offset.dot(axis))
            - settings.damping_ns_per_m * speed.dot(axis);
        let impulse = axis * (force * time.delta_secs());
        if !impulse.is_finite() {
            continue;
        }
        body_inertia.update_effective_inv_angular_inertia(angular_inertia, rotation);
        let hub_delta_v = -hub_inertia.effective_inv_mass() * impulse;
        let body = solvers.get_mut(*body_index).unwrap();
        body.linear_velocity += body_inertia.effective_inv_mass() * impulse;
        body.angular_velocity +=
            body_inertia.effective_inv_angular_inertia() * lever.cross(impulse);
        solvers.get_mut(*hub_index).unwrap().linear_velocity += hub_delta_v;
    }
}

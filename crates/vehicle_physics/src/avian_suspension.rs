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

/// Commands on [WheelCorner::spin_joint]. Positive steer turns +Z towards +X about the
/// hub's local +Y. Positive drive spins the rim about its steered +X axis.
/// Angles are in radians and torques in N·m. Brake capacity is nonnegative.
#[derive(Component, Clone, Copy, Debug, Default)]
pub struct WheelControl {
    pub steer_rad: f32,
    pub drive_nm: f32,
    pub brake_nm: f32,
}

/// Front wheel angles in the order +X, -X for chassis-local +Z forward and +Y up.
/// Positive equivalent steer turns towards +X. Wheelbase and track are positive lengths
/// in metres. Returns `None` for invalid geometry or a turn centre at or inside the track.
pub fn ackermann_steer(wheelbase_m: f32, track_m: f32, steer_rad: f32) -> Option<[f32; 2]> {
    if !wheelbase_m.is_finite()
        || wheelbase_m <= 0.0
        || !track_m.is_finite()
        || track_m <= 0.0
        || !(steer_rad.abs() < std::f32::consts::FRAC_PI_2)
    {
        return None;
    }
    let tangent = steer_rad.tan();
    let offset = 0.5 * track_m * tangent;
    if offset.abs() >= wheelbase_m {
        return None;
    }
    Some([wheelbase_m - offset, wheelbase_m + offset].map(|run| (wheelbase_m * tangent).atan2(run)))
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
            PhysicsSchedule,
            steer_wheels.before(PhysicsStepSystems::First),
        )
        .add_systems(
            SubstepSchedule,
            (apply_suspension, apply_wheel_torque)
                .chain()
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
            .spawn((
                RevoluteJoint::new(hub, rim).with_hinge_axis(Vec3::X),
                WheelControl::default(),
                WheelActuator {
                    chassis,
                    suspension_joint,
                },
            ))
            .id();
        Ok(Self {
            rim,
            hub,
            suspension_joint,
            spin_joint,
        })
    }
}

fn steer_wheels(mut wheels: Query<(&WheelControl, &mut RevoluteJoint), Changed<WheelControl>>) {
    for (control, mut joint) in &mut wheels {
        if control.steer_rad.is_finite() {
            let basis = Quat::from_rotation_y(control.steer_rad);
            if joint.local_basis1() != Some(basis) {
                joint.frame1.basis = JointBasis::Local(basis);
            }
        }
    }
}

#[derive(Component)]
struct WheelActuator {
    chassis: Entity,
    suspension_joint: Entity,
}

fn apply_wheel_torque(
    time: Res<Time>,
    wheels: Query<(&RevoluteJoint, &WheelControl, &WheelActuator), Without<JointDisabled>>,
    sliders: Query<(), (With<PrismaticJoint>, Without<JointDisabled>)>,
    bodies: Query<(Option<&SolverBodyIndex>, &Rotation, &ComputedAngularInertia)>,
    mut solvers: ResMut<SolverBodies>,
) {
    for (joint, control, actuator) in &wheels {
        if (control.drive_nm == 0.0 && control.brake_nm <= 0.0)
            || !control.drive_nm.is_finite()
            || !control.brake_nm.is_finite()
        {
            continue;
        }
        let Ok(
            [
                (Some(hub_index), hub_rotation, hub_inertia),
                (Some(rim_index), rim_rotation, rim_inertia),
                (chassis_index, chassis_rotation, chassis_inertia),
            ],
        ) = bodies.get_many([joint.body1, joint.body2, actuator.chassis])
        else {
            continue;
        };
        let (Some(hub), Some(rim), Some(mut hub_mass), Some(mut rim_mass), Some(local_axis)) = (
            solvers.get(*hub_index),
            solvers.get(*rim_index),
            solvers.get_inertia(*hub_index).cloned(),
            solvers.get_inertia(*rim_index).cloned(),
            joint.local_hinge_axis1(),
        ) else {
            continue;
        };
        let hub_orientation = hub.delta_rotation * hub_rotation.0;
        let axis = hub_orientation * local_axis;
        hub_mass.update_effective_inv_angular_inertia(hub_inertia, hub_orientation);
        rim_mass
            .update_effective_inv_angular_inertia(rim_inertia, rim.delta_rotation * rim_rotation.0);
        let hub_response = hub_mass.effective_inv_angular_inertia() * axis;
        let rim_response = rim_mass.effective_inv_angular_inertia() * axis;
        // The slider locks hub rotation to the chassis. Use the chassis alone as
        // an upper estimate of its angular response after constraint projection.
        let chassis_response = chassis_index
            .and_then(|index| Some((solvers.get(*index)?, solvers.get_inertia(*index)?.clone())))
            .map_or(Vec3::ZERO, |(chassis, mut mass)| {
                mass.update_effective_inv_angular_inertia(
                    chassis_inertia,
                    chassis.delta_rotation * chassis_rotation.0,
                );
                mass.effective_inv_angular_inertia() * axis
            });
        let carrier_response = if sliders.contains(actuator.suspension_joint) {
            chassis_response
        } else {
            hub_response
        };
        let inverse_inertia = axis.dot(rim_response + carrier_response);
        if inverse_inertia <= 0.0 {
            continue;
        }
        let spin = (rim.angular_velocity - hub.angular_velocity).dot(axis);
        let impulse = wheel_impulse(spin, inverse_inertia, *control, time.delta_secs());
        if impulse.is_finite() {
            solvers.get_mut(*hub_index).unwrap().angular_velocity -= hub_response * impulse;
            solvers.get_mut(*rim_index).unwrap().angular_velocity += rim_response * impulse;
        }
    }
}

fn wheel_impulse(spin: f32, inverse_inertia: f32, control: WheelControl, dt: f32) -> f32 {
    let drive = control.drive_nm * dt;
    let brake = control.brake_nm.max(0.0) * dt;
    let cancel_spin = -(spin / inverse_inertia + drive);
    drive + cancel_spin.clamp(-brake, brake)
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

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::time::TimeUpdateStrategy;
    use std::time::Duration;

    #[test]
    fn ackermann_angles_mirror_and_aim_at_one_turn_centre() {
        let steer = 6.0_f32.to_radians();
        let [inner, outer] = ackermann_steer(2.5, 1.5, steer).unwrap();
        assert_eq!(ackermann_steer(2.5, 1.5, -steer), Some([-outer, -inner]));
        assert!(inner > steer && steer > outer);
        assert!((2.5 / outer.tan() - 2.5 / inner.tan() - 1.5).abs() < 1e-5);
        assert_eq!(ackermann_steer(2.5, 1.5, 0.0), Some([0.0; 2]));
        assert!(ackermann_steer(2.5, 1.5, 1.5).is_none());
    }

    #[test]
    fn brake_cancels_relative_spin_and_holds_against_drive() {
        let brake = WheelControl {
            brake_nm: 100.0,
            ..default()
        };
        for spin in [-4.0, 0.0, 4.0] {
            assert_eq!(spin + 5.0 * wheel_impulse(spin, 5.0, brake, 0.01), 0.0);
        }
        assert_eq!(wheel_impulse(10.0, 5.0, brake, 0.01), -1.0);
        let drive_and_brake = WheelControl {
            drive_nm: 60.0,
            ..brake
        };
        assert_eq!(wheel_impulse(0.0, 5.0, drive_and_brake, 0.01), 0.0);
    }

    #[test]
    fn the_drive_impulse_spins_the_rim_and_reacts_through_the_hub() {
        let mut app = App::new();
        app.add_plugins((
            MinimalPlugins,
            TransformPlugin,
            PhysicsPlugins::default(),
            AvianSuspensionPlugin,
        ))
        .insert_resource(Gravity(Vec3::ZERO))
        .insert_resource(Time::<Fixed>::from_hz(60.0))
        .insert_resource(SubstepCount(16))
        .insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_secs_f64(
            1.0 / 60.0,
        )));
        app.finish();
        app.cleanup();
        app.update();

        let world = app.world_mut();
        let [chassis, hub, rim] = [3.0, 2.0, 1.0].map(|inertia| {
            world
                .spawn((
                    RigidBody::Dynamic,
                    SleepingDisabled,
                    Mass(1.0),
                    AngularInertia::new(Vec3::splat(inertia)),
                    Transform::IDENTITY,
                ))
                .id()
        });
        let suspension_joint = world
            .spawn(PrismaticJoint::new(chassis, hub).with_slider_axis(Vec3::NEG_Y))
            .id();
        world.spawn((
            RevoluteJoint::new(hub, rim).with_hinge_axis(Vec3::X),
            WheelActuator {
                chassis,
                suspension_joint,
            },
            WheelControl {
                drive_nm: 12.0,
                ..default()
            },
        ));
        app.update();

        let [body_spin, hub_spin, rim_spin] =
            [chassis, hub, rim].map(|e| app.world().get::<AngularVelocity>(e).unwrap().x);
        assert!((rim_spin - 0.2).abs() < 0.002, "{rim_spin}");
        assert!(body_spin < 0.0);
        assert!(
            (hub_spin - body_spin).abs() < 0.002,
            "{hub_spin}, {body_spin}"
        );
        assert!((3.0 * body_spin + 2.0 * hub_spin + rim_spin).abs() < 0.002);
    }
}

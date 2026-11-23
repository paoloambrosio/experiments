//! Couple a mounted tyre to an Avian dynamic body on the solver's substep clock.

use avian3d::{
    dynamics::{
        integrator::IntegrationSystems,
        solver::{
            schedule::SubstepSolverSystems,
            solver_body::{SolverBodies, SolverBody, SolverBodyIndex},
        },
    },
    math::ToRealPrecision,
    prelude::*,
};
use bevy::{math::DVec3, prelude::*};

use crate::{
    elastic_tyre::ElasticTyreError,
    mounted_tyre::{MountedStep, MountedTyre, RimMotion},
};

/// Insert with [Self::rim_bundle] on the same entity. The rim centre is the centre of mass.
#[derive(Component)]
pub struct AvianTyre {
    tyre: MountedTyre,
    last_step: MountedStep,
    error: Option<ElasticTyreError>,
}

impl AvianTyre {
    /// An awake, collider-free dynamic rim with mass and inertia for the attached beads.
    /// Use a floor on the mounted tyre to resolve tread contact.
    pub fn rim_bundle(
        tyre: MountedTyre,
        bare_rim_mass_kg: f64,
    ) -> Result<impl Bundle, ElasticTyreError> {
        let (mass, inertia) = tyre.rim_mass_properties(bare_rim_mass_kg)?;
        let (mass, inertia) = (mass as f32, inertia.as_vec3());
        if !mass.is_finite() || mass <= 0.0 || !inertia.is_finite() || inertia.min_element() <= 0.0
        {
            return Err(ElasticTyreError::InvalidMass);
        }
        let rim = tyre.rim();
        let position = rim.centre.real();
        let linear_velocity = rim.linear_velocity.as_vec3();
        let angular_velocity = rim.angular_velocity.as_vec3();
        if !position.is_finite() || !linear_velocity.is_finite() || !angular_velocity.is_finite() {
            return Err(ElasticTyreError::InvalidRimMotion);
        }
        Ok((
            RigidBody::Dynamic,
            SleepingDisabled,
            Mass(mass),
            AngularInertia::new(inertia),
            Position(position),
            Rotation(rim.rotation.as_quat()),
            LinearVelocity(linear_velocity),
            AngularVelocity(angular_velocity),
            Self {
                tyre,
                last_step: MountedStep::default(),
                error: None,
            },
        ))
    }

    pub fn tyre(&self) -> &MountedTyre {
        &self.tyre
    }

    pub fn last_step(&self) -> MountedStep {
        self.last_step
    }

    pub fn error(&self) -> Option<ElasticTyreError> {
        self.error
    }
}

/// Order measurements after the beads have been synchronized to the end of the substep.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TyreSubstepComplete;

/// Install after Avian's physics plugins. Share Avian's gravity and substep size with the tyre.
pub struct AvianTyrePlugin;

impl Plugin for AvianTyrePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            SubstepSchedule,
            (
                apply_tyre_wrench.before(IntegrationSystems::Velocity),
                sync_beads
                    .after(SubstepSolverSystems::Damping)
                    .in_set(TyreSubstepComplete),
            ),
        );
    }
}

fn rim_motion(position: &Position, rotation: &Rotation, solver: &SolverBody) -> RimMotion {
    RimMotion {
        centre: DVec3::from(position.0) + solver.delta_position.as_dvec3(),
        rotation: (solver.delta_rotation * rotation.0).as_dquat().normalize(),
        linear_velocity: solver.linear_velocity.as_dvec3(),
        angular_velocity: solver.angular_velocity.as_dvec3(),
    }
}

fn apply_tyre_wrench(
    time: Res<Time>,
    gravity: Res<Gravity>,
    mut solvers: ResMut<SolverBodies>,
    mut tyres: Query<(
        &mut AvianTyre,
        &SolverBodyIndex,
        &Position,
        &Rotation,
        &ComputedAngularInertia,
        Option<&GravityScale>,
    )>,
) {
    let dt = time.delta_secs_f64();
    for (mut wheel, &index, position, rotation, angular_inertia, gravity_scale) in &mut tyres {
        if wheel.error.is_some() {
            continue;
        }
        let Some(mut inertia) = solvers.get_inertia(index).cloned() else {
            continue;
        };
        let Some(solver) = solvers.get_mut(index) else {
            continue;
        };
        let rim = rim_motion(position, rotation, solver);
        let gravity = gravity.0.as_dvec3() * f64::from(gravity_scale.map_or(1.0, |scale| scale.0));
        let step = wheel
            .tyre
            .sync_rim(rim)
            .and_then(|()| wheel.tyre.step(dt, gravity));
        match step {
            Ok(step) => {
                // Avian's usual force path caches the same increment for all substeps.
                // The bead wrench changes each substep, so apply its incremental impulse here.
                inertia
                    .update_effective_inv_angular_inertia(angular_inertia, rim.rotation.as_quat());
                let linear_velocity = solver.linear_velocity
                    + inertia.effective_inv_mass() * (step.rim_force * dt).as_vec3();
                let angular_velocity = solver.angular_velocity
                    + inertia.effective_inv_angular_inertia() * (step.rim_torque * dt).as_vec3();
                if !linear_velocity.is_finite() || !angular_velocity.is_finite() {
                    wheel.error = Some(ElasticTyreError::NonFiniteStep);
                    continue;
                }
                solver.linear_velocity = linear_velocity;
                solver.angular_velocity = angular_velocity;
                wheel.last_step = step;
            }
            Err(error) => wheel.error = Some(error),
        }
    }
}

fn sync_beads(
    solvers: Res<SolverBodies>,
    mut tyres: Query<(&mut AvianTyre, &SolverBodyIndex, &Position, &Rotation)>,
) {
    for (mut wheel, &index, position, rotation) in &mut tyres {
        if wheel.error.is_none()
            && let Some(solver) = solvers.get(index)
            && let Err(error) = wheel.tyre.sync_rim(rim_motion(position, rotation, solver))
        {
            wheel.error = Some(error);
        }
    }
}

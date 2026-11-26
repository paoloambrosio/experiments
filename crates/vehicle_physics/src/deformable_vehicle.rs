//! Couple a node-and-beam chassis to four Avian wheel assemblies.
//! Vehicle axes are +Y up, +Z forward and +X left. Inputs use SI units.

use std::{error::Error, fmt, sync::Arc};

use avian3d::{
    dynamics::{
        integrator::IntegrationSystems,
        solver::{
            schedule::SubstepSolverSystems,
            solver_body::{SolverBodies, SolverBody, SolverBodyIndex},
        },
    },
    prelude::*,
};
use bevy::{
    math::{DMat3, DQuat, DVec3},
    prelude::*,
};

use crate::{
    avian_suspension::{
        CornerError, RigidSuspensionApplied, Suspension, WheelControl, WheelCorner, wheel_impulse,
    },
    avian_tyre::{AvianTyre, TyreSubstepComplete, TyreWrenchApplied, rim_motion},
    chassis::{Chassis, ChassisBeamSpec, ChassisCollision, ChassisError, ChassisNode},
    elastic_tyre::{ElasticTyre, ElasticTyreError},
    mounted_tyre::{MountedTyre, RimMotion, RimObstacleSweep},
    static_collision::{StaticCollisionMesh, StaticReaction},
    structural_mount::{Mount, Patch, outer, point_inertia},
    vehicle_assembly::{
        AssemblyError, VehicleGeometry, VehiclePose, VehicleRoad, VehicleSpec, WHEEL_NAMES,
        WheelAlignment,
    },
};

pub use crate::structural_mount::SuspensionMount;

/// Structural rest geometry in vehicle-local coordinates.
pub struct StructureSpec {
    pub nodes: Vec<ChassisNode>,
    pub beams: Vec<ChassisBeamSpec>,
    pub collision_nodes: Vec<usize>,
    pub collision_panels: Vec<[usize; 3]>,
    /// Front left, front right, rear left, rear right.
    pub mounts: [SuspensionMount; 4],
}

#[derive(Debug)]
pub enum DeformableError {
    Assembly(String),
    Chassis(ChassisError),
    InvalidMount(usize),
    InvalidMassProperties,
    Wheel(usize, CornerError),
    Tyre(usize, ElasticTyreError),
    InvalidWheelState(usize),
}

impl From<ChassisError> for DeformableError {
    fn from(error: ChassisError) -> Self {
        Self::Chassis(error)
    }
}

impl fmt::Display for DeformableError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Assembly(error) => f.write_str(error),
            Self::Chassis(error) => error.fmt(f),
            Self::InvalidMount(i) => write!(f, "invalid or collapsed {} mount", WHEEL_NAMES[*i]),
            Self::InvalidMassProperties => f.write_str(
                "structure mass or centre of mass differs from the vehicle specification",
            ),
            Self::Wheel(i, error) => write!(f, "{} corner: {error}", WHEEL_NAMES[*i]),
            Self::Tyre(i, error) => write!(f, "{} tyre: {error}", WHEEL_NAMES[*i]),
            Self::InvalidWheelState(i) => write!(f, "invalid {} solver state", WHEEL_NAMES[*i]),
        }
    }
}

impl Error for DeformableError {}

/// Contact and reaction samples from the last committed Avian substep.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MountReaction {
    /// Impulse on the chassis in world N·s.
    pub impulse_ns: DVec3,
    /// Angular impulse on the chassis about the upright centre in world N·m·s.
    pub moment_nms: DVec3,
    pub length_m: f64,
}

/// Reusable, validated rest state. Respawn from this value to reset the full assembly.
pub struct PreparedDeformableVehicle {
    structure: StructureSpec,
    mounts: [Mount; 4],
    vehicle: VehicleSpec,
    alignments: [WheelAlignment; 4],
    geometry: VehicleGeometry,
    pub inertia_kg_m2: DMat3,
}

impl PreparedDeformableVehicle {
    pub fn new(
        vehicle: VehicleSpec,
        structure: StructureSpec,
        alignments: [WheelAlignment; 4],
    ) -> Result<Self, DeformableError> {
        let geometry = vehicle
            .validated_geometry()
            .map_err(|e| DeformableError::Assembly(e.to_string()))?;
        let chassis = Chassis::new(structure.nodes.clone(), structure.beams.clone())?;
        ChassisCollision::new(
            &chassis,
            structure.collision_nodes.clone(),
            structure.collision_panels.clone(),
        )?;
        let mass = structure.nodes.iter().map(|n| n.mass_kg).sum::<f64>();
        let centre = structure
            .nodes
            .iter()
            .map(|n| n.position_m * (n.mass_kg / mass))
            .sum::<DVec3>();
        if (mass - f64::from(vehicle.chassis.mass_kg)).abs() > 1e-4
            || !centre.abs_diff_eq(vehicle.chassis.centre_of_mass_m.as_dvec3(), 1e-5)
        {
            return Err(DeformableError::InvalidMassProperties);
        }
        let [fl, fr, rl, rr] = std::array::from_fn(|i| {
            let mount = Mount::new(structure.mounts[i], &structure.nodes)
                .ok_or(DeformableError::InvalidMount(i))?;
            let patch = mount
                .patch(&structure.nodes)
                .ok_or(DeformableError::InvalidMount(i))?;
            let wheel = vehicle.wheels[i].expect("validated layout");
            let expected = wheel.centre_m + Vec3::Y * wheel.initial_length_m;
            if !patch.anchor.abs_diff_eq(expected.as_dvec3(), 1e-5) {
                return Err(DeformableError::InvalidMount(i));
            }
            Ok::<_, DeformableError>(mount)
        });
        let inertia_kg_m2 = point_inertia(
            structure
                .nodes
                .iter()
                .map(|n| (n.mass_kg, n.position_m - centre)),
        );
        Ok(Self {
            structure,
            vehicle,
            alignments,
            geometry,
            mounts: [fl?, fr?, rl?, rr?],
            inertia_kg_m2,
        })
    }

    pub fn spawn(
        &self,
        world: &mut World,
        tyres: [&ElasticTyre; 4],
        pose: VehiclePose,
        launch_velocity_mps: DVec3,
        road: &VehicleRoad,
        obstacles: Option<Arc<StaticCollisionMesh>>,
    ) -> Result<DeformableVehicle, DeformableError> {
        if !pose.origin_m.is_finite()
            || !pose.rotation.is_normalized()
            || !launch_velocity_mps.is_finite()
        {
            return Err(DeformableError::Assembly(
                AssemblyError::InvalidPose("deformable vehicle").to_string(),
            ));
        }
        let rotation = pose.rotation.as_dquat().normalize();
        let point = |local: DVec3| pose.origin_m.as_dvec3() + rotation * local;
        let nodes = self
            .structure
            .nodes
            .iter()
            .map(|&node| ChassisNode {
                position_m: point(node.position_m),
                velocity_mps: launch_velocity_mps + rotation * node.velocity_mps,
                ..node
            })
            .collect();
        let chassis = Chassis::new(nodes, self.structure.beams.clone())?;
        let collision = ChassisCollision::new(
            &chassis,
            self.structure.collision_nodes.clone(),
            self.structure.collision_panels.clone(),
        )?;
        let [fl, fr, rl, rr] = std::array::from_fn(|i| {
            self.mounts[i]
                .patch(chassis.nodes())
                .ok_or(DeformableError::InvalidMount(i))?;
            let spec = self.vehicle.wheels[i].expect("validated layout");
            let rim = RimMotion {
                centre: point(spec.centre_m.as_dvec3()),
                rotation: (rotation * self.alignments[i].rotation().as_dquat()).normalize(),
                linear_velocity: launch_velocity_mps,
                ..Default::default()
            };
            let floor = match road {
                VehicleRoad::Flat(floor) => Some(*floor),
                VehicleRoad::Triangles { .. } => None,
            };
            let tyre_error = |error| DeformableError::Tyre(i, error);
            let mut tyre = MountedTyre::new(tyres[i].clone(), rim, floor).map_err(tyre_error)?;
            if let VehicleRoad::Triangles { road, contact } = road {
                tyre.set_triangle_road(road.clone(), *contact);
            }
            if let Some(obstacles) = &obstacles {
                tyre.set_obstacles(obstacles.clone(), spec.bare_rim_mass_kg)
                    .map_err(tyre_error)?;
            }
            let (rim_mass, rim_inertia) = tyre
                .rim_mass_properties(spec.bare_rim_mass_kg)
                .map_err(tyre_error)?;
            let corner = WheelCorner::prepare(
                tyre,
                spec.corner_config(self.vehicle.chassis.centre_of_mass_m),
            )
            .map_err(|error| DeformableError::Wheel(i, error))?;
            Ok::<_, DeformableError>((
                corner,
                CornerMass {
                    total: rim_mass + f64::from(spec.upright_mass_kg),
                    hub_inertia: f64::from(spec.upright_mass_kg * 0.02),
                    rim_inertia,
                },
                spec.suspension,
            ))
        });
        let prepared = [fl?, fr?, rl?, rr?];
        let mut index = 0;
        let corners = prepared.map(|(corner, mass, suspension)| {
            let i = index;
            index += 1;
            let (hub, rim, spin_joint) =
                corner.spawn_pair(world, pose.rotation, self.alignments[i]);
            let suspension_joint = world
                .spawn(StructuralSuspension(self.structure.mounts[i]))
                .id();
            Corner {
                wheel: WheelCorner {
                    hub,
                    rim,
                    spin_joint,
                    suspension_joint,
                },
                mount: self.mounts[i],
                mass,
                suspension,
            }
        });
        let wheels = corners.map(|corner| corner.wheel);
        let body = world
            .spawn(DeformableBody {
                chassis,
                collision,
                obstacles,
                corners,
                snapshots: None,
                last_mounts: [MountReaction::default(); 4],
                last_contact: StaticReaction::default(),
                error: None,
            })
            .id();
        Ok(DeformableVehicle {
            body,
            wheels,
            geometry: self.geometry,
        })
    }
}

/// The structural mount definition associated with a corner handle.
#[derive(Component)]
pub struct StructuralSuspension(pub SuspensionMount);

pub struct DeformableVehicle {
    pub body: Entity,
    pub wheels: [WheelCorner; 4],
    pub geometry: VehicleGeometry,
}

impl DeformableVehicle {
    pub fn despawn(self, world: &mut World) {
        crate::vehicle_assembly::despawn(world, self.body, self.wheels);
    }
}

#[derive(Clone, Copy)]
struct CornerMass {
    total: f64,
    hub_inertia: f64,
    rim_inertia: DVec3,
}

#[derive(Clone, Copy)]
struct Corner {
    wheel: WheelCorner,
    mount: Mount,
    mass: CornerMass,
    suspension: Suspension,
}

struct Snapshot {
    hub: SolverBody,
    rim: SolverBody,
    tyre: AvianTyre,
}

#[derive(Component)]
pub struct DeformableBody {
    chassis: Chassis,
    collision: ChassisCollision,
    obstacles: Option<Arc<StaticCollisionMesh>>,
    corners: [Corner; 4],
    snapshots: Option<[Snapshot; 4]>,
    last_mounts: [MountReaction; 4],
    last_contact: StaticReaction,
    error: Option<DeformableError>,
}

impl DeformableBody {
    pub fn chassis(&self) -> &Chassis {
        &self.chassis
    }
    pub fn error(&self) -> Option<&DeformableError> {
        self.error.as_ref()
    }
    pub fn mount_reactions(&self) -> &[MountReaction; 4] {
        &self.last_mounts
    }
    pub fn obstacle_reaction(&self) -> StaticReaction {
        self.last_contact
    }
    /// Upright mount origin and orientation in the committed world state.
    pub fn mount_frame(&self, i: usize) -> Option<(DVec3, DQuat)> {
        let patch = self.corners.get(i)?.mount.patch(self.chassis.nodes())?;
        Some((patch.anchor, patch.rotation))
    }
}

/// Install alongside AvianTyrePlugin and AvianSuspensionPlugin.
pub struct DeformableVehiclePlugin;

impl Plugin for DeformableVehiclePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            SubstepSchedule,
            (
                snapshot.before(TyreWrenchApplied),
                apply_controls
                    .after(TyreWrenchApplied)
                    .after(RigidSuspensionApplied)
                    .before(IntegrationSystems::Velocity),
                couple
                    .after(SubstepSolverSystems::Damping)
                    .before(TyreSubstepComplete),
            ),
        );
    }
}

type BodyQuery<'w, 's> = Query<
    'w,
    's,
    (
        &'static SolverBodyIndex,
        &'static Position,
        &'static Rotation,
    ),
>;

fn snapshot(
    mut vehicles: Query<&mut DeformableBody>,
    wheels: BodyQuery,
    tyres: Query<&AvianTyre>,
    solvers: Res<SolverBodies>,
) {
    for mut vehicle in &mut vehicles {
        let [fl, fr, rl, rr] = vehicle.corners.map(|corner| {
            let [(hub, _, _), (rim, _, _)] =
                wheels.get_many([corner.wheel.hub, corner.wheel.rim]).ok()?;
            Some(Snapshot {
                hub: solvers.get(*hub)?.clone(),
                rim: solvers.get(*rim)?.clone(),
                tyre: tyres.get(corner.wheel.rim).ok()?.clone(),
            })
        });
        vehicle.snapshots = match (fl, fr, rl, rr) {
            (Some(fl), Some(fr), Some(rl), Some(rr)) => Some([fl, fr, rl, rr]),
            _ => None,
        };
    }
}

fn apply_controls(
    time: Res<Time>,
    vehicles: Query<&DeformableBody>,
    controls: Query<(&WheelControl, &RevoluteJoint), Without<JointDisabled>>,
    wheels: BodyQuery,
    mut solvers: ResMut<SolverBodies>,
) {
    for vehicle in &vehicles {
        if vehicle.error.is_some() || vehicle.snapshots.is_none() {
            continue;
        }
        for corner in vehicle.corners {
            let Ok((&control, joint)) = controls.get(corner.wheel.spin_joint) else {
                continue;
            };
            if (control.drive_nm == 0.0 && control.brake_nm <= 0.0)
                || !control.drive_nm.is_finite()
                || !control.brake_nm.is_finite()
            {
                continue;
            }
            let Ok([(hub_id, _, hub_rotation), (rim_id, _, rim_rotation)]) =
                wheels.get_many([corner.wheel.hub, corner.wheel.rim])
            else {
                continue;
            };
            let (Some(hub), Some(rim), Some(local_axis)) = (
                solvers.get(*hub_id),
                solvers.get(*rim_id),
                joint.local_hinge_axis1(),
            ) else {
                continue;
            };
            let axis = ((hub.delta_rotation * hub_rotation.0) * local_axis)
                .as_dvec3()
                .normalize();
            let spin = (rim.angular_velocity - hub.angular_velocity)
                .as_dvec3()
                .dot(axis);
            let q = (rim.delta_rotation * rim_rotation.0).as_dquat().normalize();
            let rim_response = q * ((q.conjugate() * axis) / corner.mass.rim_inertia);
            let chassis_response = corner
                .mount
                .patch(vehicle.chassis.nodes())
                .map_or(0.0, |patch| patch.inverse_angular_mass(axis));
            let inverse_inertia =
                axis.dot(rim_response) + chassis_response.min(corner.mass.hub_inertia.recip());
            let impulse = f64::from(wheel_impulse(
                spin as f32,
                inverse_inertia as f32,
                control,
                time.delta_secs(),
            ));
            if impulse.is_finite() {
                solvers.get_mut(*hub_id).unwrap().angular_velocity -=
                    (axis * (impulse / corner.mass.hub_inertia)).as_vec3();
                solvers.get_mut(*rim_id).unwrap().angular_velocity +=
                    (rim_response * impulse).as_vec3();
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn triangle() -> Vec<ChassisNode> {
        [DVec3::NEG_X, DVec3::X, DVec3::Y]
            .map(|position_m| ChassisNode {
                mass_kg: 2.0,
                position_m,
                velocity_mps: DVec3::ZERO,
            })
            .to_vec()
    }

    #[test]
    fn the_wheel_and_mount_exchange_equal_impulse_and_angular_impulse() {
        let mut nodes = triangle();
        let mount = Mount::new(
            SuspensionMount {
                nodes: [0, 1, 2],
                offset_m: DVec3::ZERO,
            },
            &nodes,
        )
        .unwrap();
        let patch = mount.patch(&nodes).unwrap();
        let mass = CornerMass {
            total: 20.0,
            hub_inertia: 0.06,
            rim_inertia: DVec3::new(0.5, 0.25, 0.25),
        };
        let motion = RimMotion {
            centre: DVec3::new(1.0, -0.3, 0.2),
            rotation: DQuat::from_rotation_y(0.3),
            ..Default::default()
        };
        let mut wheel = WheelTrial::new(SolverBody::DUMMY, SolverBody::DUMMY, mass, motion, motion);
        wheel.impulse(
            &mut nodes,
            &patch,
            DVec3::new(3.0, -1.0, 4.0),
            DVec3::new(0.1, 1.4, 0.3),
        );
        let (hub, rim) = wheel.finish(1.0 / 3840.0).unwrap();
        let nodal_impulse: DVec3 = nodes.iter().map(|n| n.mass_kg * n.velocity_mps).sum();
        let nodal_moment: DVec3 = nodes
            .iter()
            .map(|n| (n.position_m - motion.centre).cross(n.mass_kg * n.velocity_mps))
            .sum();
        let q = motion.rotation;
        let angular = mass.hub_inertia * hub.angular_velocity.as_dvec3()
            + q * (mass.rim_inertia * (q.conjugate() * rim.angular_velocity.as_dvec3()));
        assert!(nodal_impulse.abs_diff_eq(-mass.total * hub.linear_velocity.as_dvec3(), 1e-6));
        assert!(nodal_moment.abs_diff_eq(-angular, 1e-6));
    }

    #[test]
    fn invalid_mount_indices_and_collinearity_are_rejected() {
        let nodes = triangle();
        for indices in [[0, 1, 99], [0, 1, 1]] {
            assert!(
                Mount::new(
                    SuspensionMount {
                        nodes: indices,
                        offset_m: DVec3::ZERO
                    },
                    &nodes
                )
                .is_none()
            );
        }
    }

    #[test]
    fn a_collapsing_trial_mount_does_not_commit_chassis_motion() {
        let mut initial = triangle();
        initial[2].velocity_mps = DVec3::NEG_Y;
        let mut chassis = Chassis::new(initial.clone(), vec![]).unwrap();
        let mount = Mount::new(
            SuspensionMount {
                nodes: [0, 1, 2],
                offset_m: DVec3::ZERO,
            },
            &initial,
        )
        .unwrap();
        chassis.stage_loaded_step(1.0, DVec3::ZERO).unwrap();
        let result = chassis.complete_loaded_step(1.0, None, |trial| {
            mount
                .patch(trial)
                .map(|_| ())
                .ok_or(DeformableError::InvalidMount(2))
        });
        assert!(matches!(result, Err(DeformableError::InvalidMount(2))));
        assert_eq!(chassis.nodes(), initial);
    }
}

fn couple(
    time: Res<Time>,
    gravity: Res<Gravity>,
    mut vehicles: Query<&mut DeformableBody>,
    wheels: BodyQuery,
    mut tyres: Query<&mut AvianTyre>,
    mut solvers: ResMut<SolverBodies>,
) {
    for mut vehicle in &mut vehicles {
        let Some(snapshots) = vehicle.snapshots.take() else {
            continue;
        };
        if vehicle.error.is_none() {
            if let Err(error) = vehicle.advance(
                time.delta_secs_f64(),
                gravity.0.as_dvec3(),
                &snapshots,
                &wheels,
                &mut tyres,
                &mut solvers,
            ) {
                vehicle.error = Some(error);
            }
        }
        if vehicle.error.is_some() {
            for (corner, previous) in vehicle.corners.iter().zip(snapshots) {
                if let Ok([(hub, _, _), (rim, _, _)]) =
                    wheels.get_many([corner.wheel.hub, corner.wheel.rim])
                {
                    *solvers.get_mut(*hub).unwrap() = previous.hub;
                    *solvers.get_mut(*rim).unwrap() = previous.rim;
                    *tyres.get_mut(corner.wheel.rim).unwrap() = previous.tyre;
                }
            }
        }
    }
}

impl DeformableBody {
    fn advance(
        &mut self,
        dt: f64,
        gravity: DVec3,
        snapshots: &[Snapshot; 4],
        wheels: &BodyQuery,
        tyres: &mut Query<&mut AvianTyre>,
        solvers: &mut SolverBodies,
    ) -> Result<(), DeformableError> {
        let [fl, fr, rl, rr] = std::array::from_fn(|i| {
            let corner = self.corners[i];
            let [(hub_id, hub_pos, hub_rot), (rim_id, rim_pos, rim_rot)] = wheels
                .get_many([corner.wheel.hub, corner.wheel.rim])
                .map_err(|_| DeformableError::InvalidWheelState(i))?;
            let tyre = tyres
                .get(corner.wheel.rim)
                .map_err(|_| DeformableError::InvalidWheelState(i))?;
            if let Some(error) = tyre.error() {
                return Err(DeformableError::Tyre(i, error));
            }
            let hub = solvers
                .get(*hub_id)
                .ok_or(DeformableError::InvalidWheelState(i))?
                .clone();
            let rim = solvers
                .get(*rim_id)
                .ok_or(DeformableError::InvalidWheelState(i))?
                .clone();
            let previous = rim_motion(hub_pos, hub_rot, &snapshots[i].hub);
            let motion = rim_motion(rim_pos, rim_rot, &rim);
            Ok::<_, DeformableError>((
                WheelTrial::new(hub, rim, corner.mass, previous, motion),
                *hub_id,
                *rim_id,
                rim_pos,
                rim_rot,
            ))
        });
        let mut trials = [fl?, fr?, rl?, rr?];
        let nodes = self.chassis.stage_loaded_step(dt, gravity)?;
        let mut reactions = [MountReaction::default(); 4];
        let [a, b, c, d] = std::array::from_fn(|i| {
            self.corners[i]
                .mount
                .patch(nodes)
                .ok_or(DeformableError::InvalidMount(i))
        });
        let patches = [a?, b?, c?, d?];
        for i in 0..4 {
            let (wheel, ..) = &mut trials[i];
            wheel.spring(nodes, &patches[i], self.corners[i].suspension, dt);
        }
        for _ in 0..4 {
            for i in 0..4 {
                let (wheel, ..) = &mut trials[i];
                wheel.constrain(nodes, &patches[i], self.corners[i].suspension, dt);
            }
        }
        let mut final_bodies = [const { (SolverBody::DUMMY, SolverBody::DUMMY) }; 4];
        for i in 0..4 {
            let (wheel, _, _, rim_pos, rim_rot) = &mut trials[i];
            let mut tyre = tyres
                .get_mut(self.corners[i].wheel.rim)
                .map_err(|_| DeformableError::InvalidWheelState(i))?;
            let mut free_impulses = Vec::new();
            if self.obstacles.is_some() {
                free_impulses.resize(tyre.tyre().positions().len(), DVec3::ZERO);
            }
            // Mount and hinge impulses change the bead trajectory after the
            // initial tyre solve. Check the resulting path before committing.
            for pass in 0..=8 {
                let states = wheel
                    .finish(dt)
                    .ok_or(DeformableError::InvalidWheelState(i))?;
                let rim = rim_motion(rim_pos, rim_rot, &states.1);
                if !free_impulses.is_empty()
                    && let Some((linear, angular)) = tyre
                        .sweep_coupled_rim(
                            RimObstacleSweep {
                                previous: snapshots[i].tyre.tyre(),
                                rim,
                                dt,
                                inverse_mass: wheel.inverse_mass,
                                inverse_inertia: wheel.rim_inverse_inertia(),
                            },
                            &mut free_impulses,
                        )
                        .map_err(|error| DeformableError::Tyre(i, error))?
                {
                    if pass == 8 {
                        return Err(DeformableError::Tyre(i, ElasticTyreError::ContactLimit));
                    }
                    wheel.apply_obstacle_impulse(linear, angular);
                    continue;
                }
                tyre.sync_motion(rim, &free_impulses, dt)
                    .map_err(|error| DeformableError::Tyre(i, error))?;
                final_bodies[i] = states;
                break;
            }
            reactions[i] = wheel.reaction;
        }
        let contact = self.chassis.complete_loaded_step(
            dt,
            self.obstacles
                .as_deref()
                .map(|world| (&self.collision, world)),
            |nodes| {
                for (i, corner) in self.corners.iter().enumerate() {
                    let patch = corner
                        .mount
                        .patch(nodes)
                        .ok_or(DeformableError::InvalidMount(i))?;
                    let (_, _, _, rim_pos, rim_rot) = trials[i];
                    let motion = rim_motion(rim_pos, rim_rot, &final_bodies[i].1);
                    reactions[i].length_m =
                        (patch.anchor - motion.centre).dot(patch.rotation * DVec3::Y);
                }
                Ok::<_, DeformableError>(())
            },
        )?;
        for ((_, hub, rim, _, _), (hub_state, rim_state)) in trials.into_iter().zip(final_bodies) {
            *solvers.get_mut(hub).unwrap() = hub_state;
            *solvers.get_mut(rim).unwrap() = rim_state;
        }
        self.last_mounts = reactions;
        self.last_contact = contact;
        Ok(())
    }
}

struct WheelTrial {
    hub: SolverBody,
    rim: SolverBody,
    previous: RimMotion,
    inverse_mass: f64,
    inverse_inertia: DMat3,
    axis: DVec3,
    spin_momentum_axis: DVec3,
    inverse_spin_inertia: f64,
    dv: DVec3,
    dw: DVec3,
    obstacle_spin_delta: f64,
    reaction: MountReaction,
}

impl WheelTrial {
    fn new(
        hub: SolverBody,
        rim: SolverBody,
        mass: CornerMass,
        previous: RimMotion,
        rim_motion: RimMotion,
    ) -> Self {
        let r = DMat3::from_quat(rim_motion.rotation);
        let inertia = r * DMat3::from_diagonal(mass.rim_inertia) * r.transpose();
        let axis = rim_motion.rotation * DVec3::X;
        let spin_momentum_axis = inertia * axis;
        let inverse_spin_inertia = axis.dot(spin_momentum_axis).recip();
        // Eliminate free rim spin from the coupled angular mass matrix.
        let inverse_inertia = (DMat3::from_diagonal(DVec3::splat(mass.hub_inertia)) + inertia
            - outer(spin_momentum_axis) * inverse_spin_inertia)
            .inverse();
        Self {
            hub,
            rim,
            previous,
            inverse_inertia,
            axis,
            spin_momentum_axis,
            inverse_spin_inertia,
            inverse_mass: mass.total.recip(),
            dv: DVec3::ZERO,
            dw: DVec3::ZERO,
            obstacle_spin_delta: 0.0,
            reaction: MountReaction::default(),
        }
    }

    fn rim_inverse_inertia(&self) -> DMat3 {
        // The hinge couples rotation perpendicular to the axle. Torque along
        // the axle also changes the rim's free spin.
        let spin_projection = DMat3::IDENTITY
            - DMat3::from_cols(
                self.axis * self.spin_momentum_axis.x,
                self.axis * self.spin_momentum_axis.y,
                self.axis * self.spin_momentum_axis.z,
            ) * self.inverse_spin_inertia;
        spin_projection * self.inverse_inertia * spin_projection.transpose()
            + outer(self.axis) * self.inverse_spin_inertia
    }

    fn apply_obstacle_impulse(&mut self, linear: DVec3, angular: DVec3) {
        self.dv += self.inverse_mass * linear;
        let spin_delta = self.axis.dot(angular) * self.inverse_spin_inertia;
        self.dw += self.inverse_inertia * (angular - self.spin_momentum_axis * spin_delta);
        self.obstacle_spin_delta += spin_delta;
    }

    fn impulse(&mut self, nodes: &mut [ChassisNode], patch: &Patch, j: DVec3, angular: DVec3) {
        self.dv += self.inverse_mass * j;
        self.dw += self.inverse_inertia * angular;
        self.reaction.impulse_ns -= j;
        self.reaction.moment_nms -= angular;
        patch.apply(nodes, self.previous.centre, -j, -angular);
    }

    fn relative_velocity(&self, nodes: &[ChassisNode], patch: &Patch) -> DVec3 {
        self.hub.linear_velocity.as_dvec3() + self.dv - patch.motion(nodes, self.previous.centre).0
    }

    fn inverse_linear_mass(&self, patch: &Patch, axis: DVec3) -> f64 {
        self.inverse_mass + patch.inverse_mass(self.previous.centre, axis)
    }

    fn spring(&mut self, nodes: &mut [ChassisNode], patch: &Patch, settings: Suspension, dt: f64) {
        let up = patch.rotation * DVec3::Y;
        let length = (patch.anchor - self.previous.centre).dot(up);
        let rate = f64::from(settings.stiffness_n_per_m);
        let damping = f64::from(settings.damping_ns_per_m) + rate * dt;
        let speed = self.relative_velocity(nodes, patch).dot(up);
        let impulse = -(rate * (f64::from(settings.free_length_m) - length) + damping * speed) * dt
            / (1.0 + damping * dt * self.inverse_linear_mass(patch, up));
        self.impulse(nodes, patch, up * impulse, DVec3::ZERO);
    }

    fn constrain(
        &mut self,
        nodes: &mut [ChassisNode],
        patch: &Patch,
        settings: Suspension,
        dt: f64,
    ) {
        let bias = 0.2 / dt;
        let offset = self.previous.centre - patch.anchor;
        for local_axis in [DVec3::X, DVec3::Z] {
            let axis = patch.rotation * local_axis;
            let speed = self.relative_velocity(nodes, patch).dot(axis);
            let target = -bias * offset.dot(axis);
            self.impulse(
                nodes,
                patch,
                axis * ((target - speed) / self.inverse_linear_mass(patch, axis)),
                DVec3::ZERO,
            );
        }
        let axis = patch.rotation * DVec3::Y;
        let length = -offset.dot(axis);
        let speed = self.relative_velocity(nodes, patch).dot(axis);
        let target = speed.clamp(
            (length - f64::from(settings.max_length_m)) / dt,
            (length - f64::from(settings.min_length_m)) / dt,
        );
        self.impulse(
            nodes,
            patch,
            axis * ((target - speed) / self.inverse_linear_mass(patch, axis)),
            DVec3::ZERO,
        );
        let orientation_error =
            (patch.rotation * self.previous.rotation.conjugate()).to_scaled_axis();
        for local_axis in [DVec3::X, DVec3::Y, DVec3::Z] {
            let axis = patch.rotation * local_axis;
            let relative_spin = self.hub.angular_velocity.as_dvec3() + self.dw
                - patch.motion(nodes, self.previous.centre).1;
            let inverse_inertia =
                axis.dot(self.inverse_inertia * axis) + patch.inverse_angular_mass(axis);
            let angular =
                axis * ((orientation_error * bias - relative_spin).dot(axis) / inverse_inertia);
            self.impulse(nodes, patch, DVec3::ZERO, angular);
        }
    }

    fn finish(&self, dt: f64) -> Option<(SolverBody, SolverBody)> {
        let rim_dw = self.dw
            + self.axis
                * (self.obstacle_spin_delta
                    - self.spin_momentum_axis.dot(self.dw) * self.inverse_spin_inertia);
        let update = |mut body: SolverBody, dw: DVec3| {
            body.linear_velocity += self.dv.as_vec3();
            body.delta_position += (self.dv * dt).as_vec3();
            body.angular_velocity += dw.as_vec3();
            body.delta_rotation =
                (Quat::from_scaled_axis((dw * dt).as_vec3()) * body.delta_rotation).normalize();
            (body.linear_velocity.is_finite()
                && body.delta_position.is_finite()
                && body.angular_velocity.is_finite()
                && body.delta_rotation.is_finite())
            .then_some(body)
        };
        Some((
            update(self.hub.clone(), self.dw)?,
            update(self.rim.clone(), rim_dw)?,
        ))
    }
}

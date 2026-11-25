//! World-space tyre dynamics with a prescribed rigid rim and optional road contact.

use std::sync::Arc;

use bevy_math::{DQuat, DVec3};

use crate::{
    beam_damage::BeamDamage,
    elastic_tyre::{ElasticTyre, ElasticTyreError},
    kerb::Kerb,
    pressure::{ChamberReading, triangulate},
    puncture::{PunctureReading, TornPanel},
    triangle_road::TriangleRoad,
    tyre::TyreTopology,
};

/// Rim motion in world coordinates, in m, m/s, and rad/s. The rim's local axle is +X.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RimMotion {
    pub centre: DVec3,
    /// Unit rotation from rim-local to world coordinates.
    pub rotation: DQuat,
    pub linear_velocity: DVec3,
    pub angular_velocity: DVec3,
}

impl RimMotion {
    fn validate(self) -> Result<(), ElasticTyreError> {
        if self.centre.is_finite()
            && self.rotation.is_normalized()
            && self.linear_velocity.is_finite()
            && self.angular_velocity.is_finite()
        {
            Ok(())
        } else {
            Err(ElasticTyreError::InvalidRimMotion)
        }
    }

    fn point(self, local_position: DVec3, local_velocity: DVec3) -> (DVec3, DVec3) {
        let offset = self.rotation * local_position;
        (
            self.centre + offset,
            self.linear_velocity
                + self.angular_velocity.cross(offset)
                + self.rotation * local_velocity,
        )
    }
}

/// Per-node road stiffness in N/m, damping in N·s/m, and optional friction.
#[derive(Debug, Clone, Copy)]
pub struct ContactParameters {
    stiffness_n_per_m: f64,
    damping_ns_per_m: f64,
    friction: Option<(f64, f64)>,
}

impl ContactParameters {
    pub fn new(stiffness_n_per_m: f64, damping_ns_per_m: f64) -> Result<Self, ElasticTyreError> {
        if !stiffness_n_per_m.is_finite()
            || stiffness_n_per_m < 0.0
            || !damping_ns_per_m.is_finite()
            || damping_ns_per_m < 0.0
        {
            return Err(ElasticTyreError::InvalidFloor);
        }
        Ok(Self {
            stiffness_n_per_m,
            damping_ns_per_m,
            friction: None,
        })
    }

    /// Enable regularized Coulomb friction with a dimensionless coefficient and a
    /// positive transition speed in m/s. A zero coefficient gives normal-only contact.
    pub fn with_friction(
        mut self,
        coefficient: f64,
        transition_speed_mps: f64,
    ) -> Result<Self, ElasticTyreError> {
        if !coefficient.is_finite()
            || coefficient < 0.0
            || !transition_speed_mps.is_finite()
            || transition_speed_mps <= 0.0
        {
            return Err(ElasticTyreError::InvalidFloor);
        }
        self.friction = (coefficient > 0.0).then_some((coefficient, transition_speed_mps));
        Ok(self)
    }

    fn normal_force_at_depth(self, depth: f64, normal_velocity: f64) -> f64 {
        if depth > 0.0 {
            (self.stiffness_n_per_m * depth - self.damping_ns_per_m * normal_velocity).max(0.0)
        } else {
            0.0
        }
    }

    fn friction_force(self, normal_n: f64, slip: DVec3, dt_over_mass: f64) -> DVec3 {
        let Some((coefficient, transition_speed)) = self.friction else {
            return DVec3::ZERO;
        };
        let speed = slip.x.hypot(slip.y).hypot(slip.z);
        if normal_n == 0.0 || speed == 0.0 {
            return DVec3::ZERO;
        }
        let limit = coefficient * normal_n;
        if !speed.is_finite() || !limit.is_finite() || !dt_over_mass.is_finite() {
            return DVec3::splat(f64::NAN);
        }

        // Solve the local backward-Euler friction update. A force of this magnitude
        // leaves speed - dt_over_mass * force of slip after this substep.
        let mut low = 0.0;
        let mut high = limit.min(speed / dt_over_mass);
        for _ in 0..32 {
            let force = low + 0.5 * (high - low);
            let remaining_slip = (speed - dt_over_mass * force).max(0.0);
            if force < limit * (remaining_slip / transition_speed).tanh() {
                low = force;
            } else {
                high = force;
            }
        }
        -(slip / speed) * (low + 0.5 * (high - low))
    }
}

/// +Y-facing horizontal floor. Stiffness and damping are per tread node.
#[derive(Debug, Clone, Copy)]
pub struct FlatFloor {
    height_m: f64,
    contact: ContactParameters,
    surface_velocity: DVec3,
}

impl FlatFloor {
    pub fn new(
        height_m: f64,
        stiffness_n_per_m: f64,
        damping_ns_per_m: f64,
    ) -> Result<Self, ElasticTyreError> {
        if !height_m.is_finite() {
            return Err(ElasticTyreError::InvalidFloor);
        }
        Ok(Self {
            height_m,
            contact: ContactParameters::new(stiffness_n_per_m, damping_ns_per_m)?,
            surface_velocity: DVec3::ZERO,
        })
    }

    /// Enable regularized Coulomb friction with a dimensionless coefficient and a
    /// positive transition speed in m/s. A zero coefficient gives normal-only contact.
    pub fn with_friction(
        mut self,
        coefficient: f64,
        transition_speed_mps: f64,
    ) -> Result<Self, ElasticTyreError> {
        self.contact = self
            .contact
            .with_friction(coefficient, transition_speed_mps)?;
        Ok(self)
    }

    /// Extract the same stationary contact parameters for use on another surface.
    pub fn contact_parameters(self) -> ContactParameters {
        self.contact
    }

    /// Set the world-space velocity of the surface in m/s. The Y component must be zero.
    pub fn with_surface_velocity(mut self, velocity: DVec3) -> Result<Self, ElasticTyreError> {
        if !velocity.is_finite() || velocity.y != 0.0 {
            return Err(ElasticTyreError::InvalidFloor);
        }
        self.surface_velocity = velocity;
        Ok(self)
    }

    pub fn height_m(self) -> f64 {
        self.height_m
    }

    /// Upward force magnitude in newtons. A rising node can lose support while still below the plane.
    pub fn normal_force_n(self, position: DVec3, velocity: DVec3) -> f64 {
        let depth = self.height_m - position.y;
        self.contact.normal_force_at_depth(depth, velocity.y)
    }

    fn tangential_force(
        self,
        normal_n: f64,
        predicted_velocity: DVec3,
        dt_over_mass: f64,
    ) -> DVec3 {
        self.contact.friction_force(
            normal_n,
            (predicted_velocity - self.surface_velocity).with_y(0.0),
            dt_over_mass,
        )
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct MountedStep {
    /// Force in world newtons transferred from the tyre to the rim.
    pub rim_force: DVec3,
    /// Torque in world N·m about the rim centre.
    pub rim_torque: DVec3,
    /// Sum of upward normal forces from the flat approach at the start of the step, in newtons.
    pub floor_force_n: f64,
    /// Tangential force from the flat approach, in world newtons with zero Y.
    pub floor_tangential_force: DVec3,
    /// Normal and friction reaction from the kerb on the tyre, in world newtons.
    pub kerb_force: DVec3,
    /// Sum of kerb normal force magnitudes in newtons. Positive during supported contact.
    pub kerb_normal_force_n: f64,
}

#[derive(Debug, Clone)]
pub struct Footprint {
    /// Clipped tread polygons projected onto the floor in world coordinates.
    pub polygons: Vec<Vec<DVec3>>,
    /// Span along world Z (the upright drop bench's rolling direction), in metres.
    pub length_m: f64,
    /// Span along world X (the upright drop bench's axle), in metres.
    pub width_m: f64,
}

/// Triangle road reaction at the start of the tyre's last successful substep.
#[derive(Debug, Clone, Copy, Default)]
pub struct TriangleReaction {
    /// Sum of normal forces in world newtons.
    pub normal_force: DVec3,
    /// Sum of friction forces in world newtons.
    pub tangential_force: DVec3,
    /// Maximum penetration below a selected face, measured along its normal in metres.
    pub penetration_m: f64,
}

/// Reuses the elastic carcass in world space. The rigid rim includes both bead rings' mass.
#[derive(Debug, Clone)]
pub struct MountedTyre {
    tyre: ElasticTyre,
    rim: RimMotion,
    floor: Option<FlatFloor>,
    kerb: Option<Kerb>,
    triangle_road: Option<(Arc<TriangleRoad>, ContactParameters)>,
    triangle_reaction: TriangleReaction,
    tread_nodes: std::ops::Range<usize>,
    tread_triangles: Vec<[usize; 3]>,
    external_forces: Vec<DVec3>,
}

impl MountedTyre {
    /// Mount a tyre from a fixed-bead experiment, preserving its deformation and gas charge.
    /// World node velocities include the rigid point velocity and the rotated local velocities.
    pub fn new(
        mut tyre: ElasticTyre,
        rim: RimMotion,
        floor: Option<FlatFloor>,
    ) -> Result<Self, ElasticTyreError> {
        rim.validate()?;
        let node_count = tyre.positions().len();
        let segments = node_count / 5;
        let tread_nodes = segments..4 * segments;
        let tread_triangles = triangulate(tyre.topology())
            .into_iter()
            .filter(|face| face.nodes.iter().all(|index| tread_nodes.contains(index)))
            .map(|face| face.nodes)
            .collect();
        for index in 0..node_count {
            let (position, velocity) = rim.point(tyre.positions[index], tyre.velocities[index]);
            if !position.is_finite() || !velocity.is_finite() {
                return Err(ElasticTyreError::NonFiniteStep);
            }
            tyre.next_positions[index] = position;
            tyre.next_velocities[index] = velocity;
        }
        tyre.commit_next()?;
        Ok(Self {
            tyre,
            rim,
            floor,
            kerb: None,
            triangle_road: None,
            triangle_reaction: TriangleReaction::default(),
            tread_nodes,
            tread_triangles,
            external_forces: vec![DVec3::ZERO; node_count],
        })
    }

    pub fn rim(&self) -> RimMotion {
        self.rim
    }

    pub fn topology(&self) -> &TyreTopology {
        self.tyre.topology()
    }

    /// World positions in metres, in topology order.
    pub fn positions(&self) -> &[DVec3] {
        self.tyre.positions()
    }

    /// World velocities in metres per second, in topology order.
    pub fn velocities(&self) -> &[DVec3] {
        self.tyre.velocities()
    }

    pub fn chamber_reading(&self) -> Option<ChamberReading> {
        self.tyre.chamber_reading()
    }

    /// Chamber air mass in kg, using the shared isothermal air temperature.
    pub fn chamber_air_mass_kg(&self) -> Option<f64> {
        self.tyre.chamber_air_mass_kg()
    }

    /// Airflow and total effective opening, when the puncture model is enabled.
    pub fn puncture_reading(&self) -> Option<PunctureReading> {
        self.tyre.puncture_reading()
    }

    /// Sidewall openings in side and segment order, with corners in topology node order.
    pub fn torn_panels(&self) -> &[TornPanel] {
        self.tyre.torn_panels()
    }

    /// Axial material state in topology beam order, when damage is enabled.
    pub fn beam_damage(&self) -> Option<&[BeamDamage]> {
        self.tyre.beam_damage()
    }

    /// Add a stationary kerb using the floor's stiffness, damping, and friction settings.
    /// The kerb participates in contact whenever a floor is installed.
    pub fn with_kerb(mut self, kerb: Kerb) -> Self {
        self.kerb = Some(kerb);
        self
    }

    /// Hold the beads in a fixed local frame, preserving deformation, gas charge, and
    /// beam damage. Node velocities are expressed relative to the rim's rigid motion.
    pub fn into_fixed_bead_tyre(mut self) -> Result<ElasticTyre, ElasticTyreError> {
        let rim = self.rim;
        let inverse_rotation = rim.rotation.inverse();
        let tyre = &mut self.tyre;
        for index in 0..tyre.positions.len() {
            let offset = tyre.positions[index] - rim.centre;
            let position = inverse_rotation * offset;
            let velocity = inverse_rotation
                * (tyre.velocities[index]
                    - rim.linear_velocity
                    - rim.angular_velocity.cross(offset));
            if !position.is_finite() || !velocity.is_finite() {
                return Err(ElasticTyreError::NonFiniteStep);
            }
            tyre.next_positions[index] = position;
            tyre.next_velocities[index] = velocity;
        }
        for &index in &tyre.bead_nodes {
            tyre.next_positions[index] = tyre.rest_positions[index];
            tyre.next_velocities[index] = DVec3::ZERO;
        }
        tyre.commit_next()?;
        Ok(self.tyre)
    }

    /// Select the analytic floor and optional kerb, or disable road contact with None.
    pub fn set_floor(&mut self, floor: Option<FlatFloor>) {
        self.floor = floor;
        self.triangle_road = None;
        self.triangle_reaction = TriangleReaction::default();
    }

    /// Select a stationary triangle road for subsequent steps, replacing the analytic floor.
    pub fn set_triangle_road(&mut self, road: Arc<TriangleRoad>, contact: ContactParameters) {
        self.floor = None;
        self.triangle_road = Some((road, contact));
        self.triangle_reaction = TriangleReaction::default();
    }

    pub fn triangle_reaction(&self) -> TriangleReaction {
        self.triangle_reaction
    }

    /// Total rim mass (kg) and local principal inertia (kg·m²) including both bead rings.
    /// Model the bare rim as a thin hoop at the bead radius and centred between the rings.
    pub fn rim_mass_properties(
        &self,
        bare_rim_mass_kg: f64,
    ) -> Result<(f64, DVec3), ElasticTyreError> {
        let bead_mass = self.tyre.node_mass * self.tyre.bead_nodes.len() as f64;
        let bead = self.tyre.rest_positions[self.tyre.bead_nodes[0]];
        let radius_squared = bead.y * bead.y + bead.z * bead.z;
        let mass = bare_rim_mass_kg + bead_mass;
        let axial_inertia = mass * radius_squared;
        let transverse_inertia = 0.5 * axial_inertia + bead_mass * bead.x * bead.x;
        let inertia = DVec3::new(axial_inertia, transverse_inertia, transverse_inertia);
        if !bare_rim_mass_kg.is_finite()
            || bare_rim_mass_kg <= 0.0
            || !mass.is_finite()
            || !inertia.is_finite()
        {
            return Err(ElasticTyreError::InvalidMass);
        }
        Ok((mass, inertia))
    }

    /// Update the prescribed bead positions and point velocities from the rigid rim.
    /// Free nodes keep their world-space motion. A failed update preserves the prior state.
    pub fn sync_rim(&mut self, rim: RimMotion) -> Result<(), ElasticTyreError> {
        if rim == self.rim {
            return Ok(());
        }
        rim.validate()?;
        let tyre = &mut self.tyre;
        tyre.next_positions.copy_from_slice(&tyre.positions);
        tyre.next_velocities.copy_from_slice(&tyre.velocities);
        for &index in &tyre.bead_nodes {
            let (position, velocity) = rim.point(tyre.rest_positions[index], DVec3::ZERO);
            if !position.is_finite() || !velocity.is_finite() {
                return Err(ElasticTyreError::NonFiniteStep);
            }
            tyre.next_positions[index] = position;
            tyre.next_velocities[index] = velocity;
        }
        tyre.commit_next()?;
        self.rim = rim;
        Ok(())
    }

    /// Advance the free nodes with gravity (world m/s²) and road contact.
    /// The caller applies the returned wrench and gravity to the rim, advances it on
    /// the same clock, then calls [Self::sync_rim] with its new motion.
    pub fn step(
        &mut self,
        dt_seconds: f64,
        gravity: DVec3,
    ) -> Result<MountedStep, ElasticTyreError> {
        let mut floor_force_n = 0.0;
        let mut floor_tangential_force = DVec3::ZERO;
        let mut kerb_force = DVec3::ZERO;
        let mut kerb_normal_force_n = 0.0;
        self.external_forces[self.tread_nodes.clone()].fill(gravity * self.tyre.node_mass);
        let dt_over_mass = dt_seconds / self.tyre.node_mass;
        let floor = self.floor;
        let kerb = self.kerb;
        let triangle_road = self.triangle_road.as_ref();
        let mut triangle_reaction = TriangleReaction::default();
        let tread_nodes = self.tread_nodes.clone();
        let (rim_force, rim_torque) = self.tyre.advance_with_contact(
            dt_seconds,
            &self.external_forces,
            Some(self.rim.centre),
            |positions, velocities, forces| {
                if let Some((road, contact)) = triangle_road {
                    for index in tread_nodes {
                        let Some(hit) = road.contact(positions[index]) else {
                            continue;
                        };
                        let depth = hit.depth();
                        let axis = hit.normal;
                        let normal =
                            contact.normal_force_at_depth(depth, velocities[index].dot(axis));
                        let predicted = velocities[index] + forces[index] * dt_over_mass;
                        let slip = predicted - axis * predicted.dot(axis);
                        let normal_force = axis * normal;
                        let tangent = contact.friction_force(normal, slip, dt_over_mass);
                        forces[index] += normal_force + tangent;
                        triangle_reaction.normal_force += normal_force;
                        triangle_reaction.tangential_force += tangent;
                        triangle_reaction.penetration_m =
                            triangle_reaction.penetration_m.max(depth);
                    }
                    if !triangle_reaction.normal_force.is_finite()
                        || !triangle_reaction.tangential_force.is_finite()
                    {
                        return Err(ElasticTyreError::NonFiniteStep);
                    }
                    return Ok(());
                }
                let Some(floor) = floor else { return Ok(()) };
                for index in tread_nodes {
                    let predicted_velocity = velocities[index] + forces[index] * dt_over_mass;
                    if let Some((depth, axis)) =
                        kerb.and_then(|kerb| kerb.contact(floor.height_m(), positions[index]))
                    {
                        let normal = floor
                            .contact
                            .normal_force_at_depth(depth, velocities[index].dot(axis));
                        let slip = predicted_velocity - axis * predicted_velocity.dot(axis);
                        let reaction = axis * normal
                            + floor.contact.friction_force(normal, slip, dt_over_mass);
                        forces[index] += reaction;
                        kerb_normal_force_n += normal;
                        kerb_force += reaction;
                        continue;
                    }
                    let normal = floor.normal_force_n(positions[index], velocities[index]);
                    let tangential =
                        floor.tangential_force(normal, predicted_velocity, dt_over_mass);
                    floor_force_n += normal;
                    floor_tangential_force += tangential;
                    forces[index] += DVec3::Y * normal + tangential;
                }
                if !floor_force_n.is_finite()
                    || !floor_tangential_force.is_finite()
                    || !kerb_force.is_finite()
                    || !kerb_normal_force_n.is_finite()
                {
                    return Err(ElasticTyreError::NonFiniteStep);
                }
                Ok(())
            },
        )?;
        self.triangle_reaction = triangle_reaction;
        Ok(MountedStep {
            rim_force,
            rim_torque,
            floor_force_n,
            floor_tangential_force,
            kerb_force,
            kerb_normal_force_n,
        })
    }

    pub fn active_contacts(&self) -> impl Iterator<Item = usize> + '_ {
        self.tread_nodes.clone().filter(|&index| {
            let position = self.positions()[index];
            let velocity = self.velocities()[index];
            if let Some((road, contact)) = &self.triangle_road {
                return road.contact(position).is_some_and(|hit| {
                    contact.normal_force_at_depth(hit.depth(), velocity.dot(hit.normal)) > 0.0
                });
            }
            self.floor.is_some_and(|floor| {
                if let Some((depth, axis)) = self
                    .kerb
                    .and_then(|kerb| kerb.contact(floor.height_m(), position))
                {
                    floor
                        .contact
                        .normal_force_at_depth(depth, velocity.dot(axis))
                        > 0.0
                } else {
                    floor.normal_force_n(position, velocity) > 0.0
                }
            })
        })
    }

    pub fn floor_penetration_m(&self) -> f64 {
        self.floor.map_or(0.0, |floor| {
            self.tread_nodes
                .clone()
                .map(|index| floor.height_m - self.positions()[index].y)
                .fold(0.0, f64::max)
        })
    }

    /// Estimate the geometric intersection of the tread with the flat approach plane.
    pub fn footprint(&self) -> Option<Footprint> {
        let floor = self.floor?;
        let mut polygons = Vec::new();
        let mut min = DVec3::splat(f64::INFINITY);
        let mut max = DVec3::splat(f64::NEG_INFINITY);
        for face in &self.tread_triangles {
            let vertices = face.map(|index| self.positions()[index]);
            if vertices.iter().all(|point| point.y >= floor.height_m) {
                continue;
            }
            let mut polygon = Vec::with_capacity(4);
            for [a, b] in [[0, 1], [1, 2], [2, 0]].map(|edge| edge.map(|i| vertices[i])) {
                let a_inside = a.y < floor.height_m;
                let b_inside = b.y < floor.height_m;
                if a_inside {
                    polygon.push(DVec3::new(a.x, floor.height_m, a.z));
                }
                if a_inside != b_inside {
                    let point = a.lerp(b, (floor.height_m - a.y) / (b.y - a.y));
                    polygon.push(DVec3::new(point.x, floor.height_m, point.z));
                }
            }
            for &point in &polygon {
                min = min.min(point);
                max = max.max(point);
            }
            polygons.push(polygon);
        }
        if polygons.is_empty() {
            return None;
        }
        Some(Footprint {
            polygons,
            length_m: max.z - min.z,
            width_m: max.x - min.x,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{elastic_tyre::ElasticTyreConfig, tyre::TyreConfig};

    fn rest_tyre() -> ElasticTyre {
        ElasticTyre::new(
            TyreTopology::generate(&TyreConfig::default()).unwrap(),
            ElasticTyreConfig::default(),
        )
        .unwrap()
    }

    #[test]
    fn moving_rim_prescribes_both_bead_rings_only() {
        let mut mounted = MountedTyre::new(rest_tyre(), RimMotion::default(), None).unwrap();
        let positions = mounted.positions().to_vec();
        let rim = RimMotion {
            centre: DVec3::new(0.01, 0.01, -0.02),
            rotation: DQuat::from_rotation_x(0.1),
            linear_velocity: DVec3::new(0.3, -0.4, 0.5),
            angular_velocity: DVec3::X * 1.2,
        };
        mounted.sync_rim(rim).unwrap();
        for (index, rest) in positions.into_iter().enumerate() {
            let (position, velocity) = if mounted.tyre.bead_nodes.contains(&index) {
                let offset = rim.rotation * rest;
                (
                    rim.centre + offset,
                    rim.linear_velocity + rim.angular_velocity.cross(offset),
                )
            } else {
                (rest, DVec3::ZERO)
            };
            assert_eq!(mounted.positions()[index], position);
            assert_eq!(mounted.velocities()[index], velocity);
        }
        let positions = mounted.positions().to_vec();
        let invalid = RimMotion {
            rotation: DQuat::from_xyzw(0.0, 0.0, 0.0, 0.0),
            ..rim
        };
        assert_eq!(
            mounted.sync_rim(invalid),
            Err(ElasticTyreError::InvalidRimMotion)
        );
        assert_eq!(mounted.positions(), positions);
    }

    #[test]
    fn off_centre_bead_force_returns_its_wrench_about_the_rim_centre() {
        let rim = RimMotion {
            centre: DVec3::new(1.0, 2.0, 3.0),
            ..Default::default()
        };
        let mut mounted = MountedTyre::new(rest_tyre(), rim, None).unwrap();
        let index = mounted.tyre.bead_nodes[0];
        let force = DVec3::Z * 10.0;
        let offset = mounted.positions()[index] - rim.centre;
        let mut forces = vec![DVec3::ZERO; mounted.positions().len()];
        forces[index] = force;
        let (rim_force, rim_torque) = mounted
            .tyre
            .advance(1.0 / 3840.0, &forces, Some(rim.centre))
            .unwrap();
        assert!((rim_force - force).length() < 1.0e-9);
        assert!((rim_torque - offset.cross(force)).length() < 1.0e-9);
    }

    #[test]
    fn floor_contact_is_upward_and_releases_without_tension() {
        let floor = FlatFloor::new(0.0, 80_000.0, 90.0).unwrap();
        let contact = DVec3::NEG_Y * 0.001;
        assert_eq!(floor.normal_force_n(contact, DVec3::ZERO), 80.0);
        assert_eq!(
            floor.normal_force_n(contact, DVec3::X * 2.0 + DVec3::Z),
            80.0
        );
        assert_eq!(floor.normal_force_n(contact, DVec3::Y), 0.0);
        assert_eq!(floor.normal_force_n(DVec3::Y, DVec3::NEG_Y), 0.0);

        let rim = RimMotion {
            centre: DVec3::Y * 0.325,
            ..Default::default()
        };
        let mut mounted = MountedTyre::new(rest_tyre(), rim, Some(floor)).unwrap();
        let patch = mounted
            .footprint()
            .expect("centre tread intersects the plane");
        assert!(patch.length_m > 0.0);
        assert!(patch.width_m > 0.0 && patch.width_m < 0.2);
        assert!(patch.polygons.iter().flatten().all(|point| point.y == 0.0));
        let result = mounted.step(1.0 / 3840.0, DVec3::ZERO).unwrap();
        assert!(result.floor_force_n > 0.0);
        assert_eq!(result.floor_tangential_force, DVec3::ZERO);
        assert!(
            mounted
                .velocities()
                .iter()
                .all(|v| v.x.abs() < 1.0e-12 && v.z.abs() < 1.0e-12)
        );
    }

    #[test]
    fn moving_floor_friction_opposes_relative_slip_with_a_shared_limit() {
        let belt_velocity = DVec3::NEG_Z * 2.0;
        let floor = FlatFloor::new(0.0, 35_000.0, 150.0).unwrap();
        assert!(floor.with_friction(-1.0, 0.1).is_err());
        assert!(floor.with_friction(0.8, 0.0).is_err());
        assert!(floor.with_surface_velocity(DVec3::Y).is_err());
        assert!(floor.with_surface_velocity(DVec3::splat(f64::NAN)).is_err());
        let floor = floor
            .with_friction(0.8, 0.1)
            .unwrap()
            .with_surface_velocity(belt_velocity)
            .unwrap();
        assert_eq!(floor.tangential_force(0.0, DVec3::ZERO, 0.0), DVec3::ZERO);
        assert_eq!(
            floor.tangential_force(100.0, belt_velocity + DVec3::Y, 0.0),
            DVec3::ZERO
        );
        let force = floor.tangential_force(100.0, belt_velocity + DVec3::X * 0.1, 0.0);
        assert!((force.x + 80.0 * 1.0_f64.tanh()).abs() < 1.0e-8);
        let slip = DVec3::new(3.0, 0.0, -4.0);
        let force = floor.tangential_force(100.0, belt_velocity + slip, 0.0);
        assert!(force.dot(slip) < 0.0);
        assert!(force.cross(slip).length() < 1.0e-10);
        assert!((force.length() - 80.0).abs() < 1.0e-8);
        let force = floor.tangential_force(100.0, belt_velocity + DVec3::X * 0.01, 0.005);
        let remaining_slip = 0.01 + force.x * 0.005;
        assert!(remaining_slip > 0.0 && remaining_slip < 0.01);
        assert!((force.x + 80.0 * (remaining_slip / 0.1).tanh()).abs() < 1.0e-8);

        let rim = RimMotion {
            centre: DVec3::Y * 0.325,
            ..Default::default()
        };
        let mut mounted = MountedTyre::new(rest_tyre(), rim, Some(floor)).unwrap();
        let dt = 1.0 / 3840.0;
        let step = mounted.step(dt, DVec3::ZERO).unwrap();
        let expected =
            floor.tangential_force(step.floor_force_n, DVec3::ZERO, dt / mounted.tyre.node_mass);
        assert!(expected.z < 0.0);
        assert!((step.floor_tangential_force - expected).length() < 1.0e-10);
        assert!(mounted.velocities().iter().any(|velocity| velocity.z < 0.0));

        // The penetrated centre tread is moving up fast enough to lose support.
        let rim = RimMotion {
            linear_velocity: DVec3::Y * 2.0,
            ..rim
        };
        let mut mounted = MountedTyre::new(rest_tyre(), rim, Some(floor)).unwrap();
        assert!(mounted.floor_penetration_m() > 0.0);
        let step = mounted.step(1.0 / 3840.0, DVec3::ZERO).unwrap();
        assert_eq!(step.floor_force_n, 0.0);
        assert_eq!(step.floor_tangential_force, DVec3::ZERO);
    }

    #[test]
    fn triangle_tread_contact_reuses_the_floor_law_once_at_a_seam() {
        let floor = FlatFloor::new(0.0, 80_000.0, 90.0)
            .unwrap()
            .with_friction(0.8, 0.1)
            .unwrap();
        let rim = RimMotion {
            centre: DVec3::Y * 0.325,
            linear_velocity: DVec3::Z,
            ..Default::default()
        };
        let vertices = [
            DVec3::new(-1.0, 0.0, -1.0),
            DVec3::new(1.0, 0.0, -1.0),
            DVec3::new(1.0, 0.0, 1.0),
            DVec3::new(-1.0, 0.0, 1.0),
        ];
        let road = Arc::new(TriangleRoad::new(&vertices, &[[0, 1, 2], [0, 3, 2]], 0.05).unwrap());
        let mut analytic = MountedTyre::new(rest_tyre(), rim, Some(floor)).unwrap();
        let mut mesh = analytic.clone();
        mesh.set_triangle_road(road, floor.contact_parameters());
        let active: Vec<_> = analytic.active_contacts().collect();
        assert!(!active.is_empty());
        assert_eq!(mesh.active_contacts().collect::<Vec<_>>(), active);
        let expected_penetration = analytic.floor_penetration_m();
        let dt = 1.0 / 3_840.0;
        let expected = analytic.step(dt, DVec3::ZERO).unwrap();
        let step = mesh.step(dt, DVec3::ZERO).unwrap();
        let actual = mesh.triangle_reaction();
        assert_eq!(step.floor_force_n, 0.0);
        assert!(
            actual
                .normal_force
                .abs_diff_eq(DVec3::Y * expected.floor_force_n, 1e-9)
        );
        assert!(
            actual
                .tangential_force
                .abs_diff_eq(expected.floor_tangential_force, 1e-9)
        );
        assert!((actual.penetration_m - expected_penetration).abs() < 1e-12);
        mesh.set_floor(Some(floor));
        assert_eq!(mesh.triangle_reaction().normal_force, DVec3::ZERO);
        assert!(mesh.step(dt, DVec3::ZERO).unwrap().floor_force_n > 0.0);
    }

    #[test]
    fn triangle_contact_follows_the_ramp_normal_and_releases() {
        let vertices = [
            DVec3::new(-1.0, -0.1, -1.0),
            DVec3::new(1.0, -0.1, -1.0),
            DVec3::new(1.0, 0.1, 1.0),
            DVec3::new(-1.0, 0.1, 1.0),
        ];
        let road = Arc::new(TriangleRoad::new(&vertices, &[[0, 1, 2], [0, 2, 3]], 0.05).unwrap());
        let contact = ContactParameters::new(80_000.0, 90.0)
            .unwrap()
            .with_friction(0.8, 0.1)
            .unwrap();
        let uphill = DVec3::new(0.0, 0.1, 1.0).normalize();
        let normal = DVec3::new(0.0, 1.0, -0.1).normalize();
        let rim = RimMotion {
            centre: DVec3::Y * 0.325,
            linear_velocity: uphill,
            ..Default::default()
        };
        let mut tyre = MountedTyre::new(rest_tyre(), rim, None).unwrap();
        tyre.set_triangle_road(road.clone(), contact);
        tyre.step(1.0 / 3_840.0, DVec3::ZERO).unwrap();
        let reaction = tyre.triangle_reaction();
        assert!(reaction.normal_force.dot(normal) > 0.0);
        assert!(reaction.normal_force.cross(normal).length() < 1e-9);
        assert!(reaction.tangential_force.dot(uphill) < 0.0);
        assert!(reaction.tangential_force.dot(normal).abs() < 1e-9);
        assert!(reaction.tangential_force.length() <= 0.8 * reaction.normal_force.length());

        let rim = RimMotion {
            linear_velocity: normal * 20.0,
            ..rim
        };
        let mut tyre = MountedTyre::new(rest_tyre(), rim, None).unwrap();
        tyre.set_triangle_road(road, contact);
        assert_eq!(tyre.active_contacts().count(), 0);
        tyre.step(1.0 / 3_840.0, DVec3::ZERO).unwrap();
        let reaction = tyre.triangle_reaction();
        assert!(reaction.penetration_m > 0.0);
        assert_eq!(reaction.normal_force, DVec3::ZERO);
        assert_eq!(reaction.tangential_force, DVec3::ZERO);
    }
}

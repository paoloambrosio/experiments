//! World-space tyre dynamics with a prescribed rigid rim and optional flat floor.

use bevy_math::{DQuat, DVec3};

use crate::{
    elastic_tyre::{ElasticTyre, ElasticTyreError},
    pressure::{ChamberReading, triangulate},
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

/// +Y-facing horizontal floor. Stiffness and damping are per tread node.
#[derive(Debug, Clone, Copy)]
pub struct FlatFloor {
    height_m: f64,
    stiffness_n_per_m: f64,
    damping_ns_per_m: f64,
}

impl FlatFloor {
    pub fn new(
        height_m: f64,
        stiffness_n_per_m: f64,
        damping_ns_per_m: f64,
    ) -> Result<Self, ElasticTyreError> {
        if !height_m.is_finite()
            || !stiffness_n_per_m.is_finite()
            || stiffness_n_per_m < 0.0
            || !damping_ns_per_m.is_finite()
            || damping_ns_per_m < 0.0
        {
            return Err(ElasticTyreError::InvalidFloor);
        }
        Ok(Self {
            height_m,
            stiffness_n_per_m,
            damping_ns_per_m,
        })
    }

    pub fn height_m(self) -> f64 {
        self.height_m
    }

    /// Upward force magnitude in newtons. A rising node can lose support while still below the plane.
    pub fn normal_force_n(self, position: DVec3, velocity: DVec3) -> f64 {
        let depth = self.height_m - position.y;
        if depth > 0.0 {
            (self.stiffness_n_per_m * depth - self.damping_ns_per_m * velocity.y).max(0.0)
        } else {
            0.0
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct MountedStep {
    /// Force in world newtons transferred from the tyre to the rim.
    pub rim_force: DVec3,
    /// Torque in world N·m about the rim centre.
    pub rim_torque: DVec3,
    /// Sum of upward normal forces at the start of the step, in newtons.
    pub floor_force_n: f64,
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

/// Reuses the elastic carcass in world space. The rigid rim includes both bead rings' mass.
#[derive(Debug, Clone)]
pub struct MountedTyre {
    tyre: ElasticTyre,
    rim: RimMotion,
    floor: Option<FlatFloor>,
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

    /// Advance the free nodes with gravity (world m/s²) and normal contact.
    /// The caller applies the returned wrench and gravity to the rim, advances it on
    /// the same clock, then calls [Self::sync_rim] with its new motion.
    pub fn step(
        &mut self,
        dt_seconds: f64,
        gravity: DVec3,
    ) -> Result<MountedStep, ElasticTyreError> {
        let mut floor_force_n = 0.0;
        self.external_forces.fill(DVec3::ZERO);
        for index in self.tread_nodes.clone() {
            let normal = self.floor.map_or(0.0, |floor| {
                floor.normal_force_n(self.tyre.positions[index], self.tyre.velocities[index])
            });
            floor_force_n += normal;
            self.external_forces[index] = gravity * self.tyre.node_mass + DVec3::Y * normal;
        }
        if !floor_force_n.is_finite() {
            return Err(ElasticTyreError::NonFiniteStep);
        }
        let (rim_force, rim_torque) =
            self.tyre
                .advance(dt_seconds, &self.external_forces, Some(self.rim.centre))?;
        Ok(MountedStep {
            rim_force,
            rim_torque,
            floor_force_n,
        })
    }

    pub fn active_contacts(&self) -> impl Iterator<Item = usize> + '_ {
        self.tread_nodes.clone().filter(|&index| {
            self.floor.is_some_and(|floor| {
                floor.normal_force_n(self.positions()[index], self.velocities()[index]) > 0.0
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

    /// Estimate the geometric intersection of the tread with the floor.
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
        assert!(
            mounted
                .velocities()
                .iter()
                .all(|v| v.x.abs() < 1.0e-12 && v.z.abs() < 1.0e-12)
        );
    }
}

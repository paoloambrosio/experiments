//! Axial springs and dampers on a tyre carcass with its bead rings fixed.

use std::{error::Error, fmt};

use bevy_math::DVec3;

use crate::tyre::{BeamFamily, RingRole, TyreTopology};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BeamCoefficients {
    /// Axial spring stiffness in N/m.
    pub stiffness: f64,
    /// Axial damping in N·s/m.
    pub damping: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ElasticTyreConfig {
    /// Mass in kg, divided equally among all nodes, including the pinned nodes.
    pub total_mass: f64,
    pub circumferential: BeamCoefficients,
    pub cross_section: BeamCoefficients,
    pub diagonal: BeamCoefficients,
}

impl Default for ElasticTyreConfig {
    fn default() -> Self {
        Self {
            total_mass: 8.0,
            circumferential: BeamCoefficients {
                stiffness: 12_000.0,
                damping: 4.0,
            },
            cross_section: BeamCoefficients {
                stiffness: 20_000.0,
                damping: 6.0,
            },
            diagonal: BeamCoefficients {
                stiffness: 8_000.0,
                damping: 3.0,
            },
        }
    }
}

impl ElasticTyreConfig {
    fn coefficients(self, family: BeamFamily) -> BeamCoefficients {
        match family {
            BeamFamily::Circumferential => self.circumferential,
            BeamFamily::CrossSection => self.cross_section,
            BeamFamily::Diagonal => self.diagonal,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ElasticTyreError {
    InvalidMass,
    InvalidCoefficients(BeamFamily),
    InvalidTimestep,
    InvalidForceCount { expected: usize, actual: usize },
    NonFiniteExternalForce(usize),
    CollapsedBeam(usize),
    NonFiniteStep,
}

impl fmt::Display for ElasticTyreError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidMass => f.write_str("node mass must be positive and finite"),
            Self::InvalidCoefficients(family) => {
                write!(
                    f,
                    "{family:?} stiffness and damping must be finite and nonnegative"
                )
            }
            Self::InvalidTimestep => f.write_str("timestep must be positive and finite"),
            Self::InvalidForceCount { expected, actual } => {
                write!(f, "expected {expected} external forces, received {actual}")
            }
            Self::NonFiniteExternalForce(index) => {
                write!(f, "external force at node {index} is not finite")
            }
            Self::CollapsedBeam(index) => write!(f, "beam {index} collapsed to zero length"),
            Self::NonFiniteStep => {
                f.write_str("step produced non-finite geometry, force, or motion")
            }
        }
    }
}

impl Error for ElasticTyreError {}

#[derive(Debug)]
struct ElasticBeam {
    endpoints: [usize; 2],
    rest_length: f64,
    coefficients: BeamCoefficients,
}

impl ElasticBeam {
    fn axial_force_on_start(&self, offset: DVec3, length: f64, relative_velocity: DVec3) -> DVec3 {
        let axis = offset / length;
        let spring = self.coefficients.stiffness * (length - self.rest_length);
        let damping = self.coefficients.damping * relative_velocity.dot(axis);
        axis * (spring + damping)
    }
}

/// A deformable tyre with fixed bead nodes and caller-supplied external forces.
///
/// Positions use tyre-local metres, with the axle along X and segment zero on
/// +Y. Velocities are in m/s and all node arrays follow topology order.
#[derive(Debug)]
pub struct ElasticTyre {
    topology: TyreTopology,
    rest_positions: Vec<DVec3>,
    positions: Vec<DVec3>,
    velocities: Vec<DVec3>,
    beams: Vec<ElasticBeam>,
    free_nodes: Vec<usize>,
    node_mass: f64,
    forces: Vec<DVec3>,
    next_positions: Vec<DVec3>,
    next_velocities: Vec<DVec3>,
}

impl ElasticTyre {
    pub fn new(
        topology: TyreTopology,
        config: ElasticTyreConfig,
    ) -> Result<Self, ElasticTyreError> {
        use ElasticTyreError::*;

        let node_count = topology.nodes().len();
        let node_mass = config.total_mass / node_count as f64;
        if !node_mass.is_finite() || node_mass <= 0.0 {
            return Err(InvalidMass);
        }
        for family in [
            BeamFamily::Circumferential,
            BeamFamily::CrossSection,
            BeamFamily::Diagonal,
        ] {
            let coefficients = config.coefficients(family);
            if [coefficients.stiffness, coefficients.damping]
                .into_iter()
                .any(|value| !value.is_finite() || value < 0.0)
            {
                return Err(InvalidCoefficients(family));
            }
        }

        let rest_positions: Vec<_> = topology
            .nodes()
            .iter()
            .map(|node| node.rest_position.as_dvec3())
            .collect();
        let beams = topology
            .beams()
            .iter()
            .map(|beam| {
                let [start, end] = beam.endpoints;
                ElasticBeam {
                    endpoints: beam.endpoints,
                    // Match the integrator's precision so the unloaded rest shape has zero spring force.
                    rest_length: (rest_positions[end] - rest_positions[start]).length(),
                    coefficients: config.coefficients(beam.family),
                }
            })
            .collect();
        let free_nodes = topology
            .nodes()
            .iter()
            .enumerate()
            .filter(|(_, node)| !matches!(node.ring, RingRole::LeftBead | RingRole::RightBead))
            .map(|(index, _)| index)
            .collect();

        Ok(Self {
            topology,
            positions: rest_positions.clone(),
            next_positions: rest_positions.clone(),
            rest_positions,
            velocities: vec![DVec3::ZERO; node_count],
            next_velocities: vec![DVec3::ZERO; node_count],
            forces: vec![DVec3::ZERO; node_count],
            beams,
            free_nodes,
            node_mass,
        })
    }

    pub fn topology(&self) -> &TyreTopology {
        &self.topology
    }

    /// Current node positions in tyre-local metres.
    pub fn positions(&self) -> &[DVec3] {
        &self.positions
    }

    /// Current node velocities in tyre-local m/s.
    pub fn velocities(&self) -> &[DVec3] {
        &self.velocities
    }

    /// Restore all nodes to their rest positions with zero velocity.
    pub fn reset(&mut self) {
        self.positions.copy_from_slice(&self.rest_positions);
        self.velocities.fill(DVec3::ZERO);
    }

    /// Advance one semi-implicit Euler step by a positive, finite number of seconds.
    ///
    /// Supply one tyre-local force vector in newtons per node. Forces on the
    /// prescribed bead nodes have no effect on their motion. The caller chooses
    /// a timestep small enough for the configured beam coefficients and mass.
    /// An error leaves the published positions and velocities at the previous step.
    pub fn step(
        &mut self,
        dt_seconds: f64,
        external_forces: &[DVec3],
    ) -> Result<(), ElasticTyreError> {
        use ElasticTyreError::*;

        if !dt_seconds.is_finite() || dt_seconds <= 0.0 {
            return Err(InvalidTimestep);
        }
        if external_forces.len() != self.forces.len() {
            return Err(InvalidForceCount {
                expected: self.forces.len(),
                actual: external_forces.len(),
            });
        }
        if let Some(index) = external_forces.iter().position(|force| !force.is_finite()) {
            return Err(NonFiniteExternalForce(index));
        }
        self.forces.copy_from_slice(external_forces);

        for beam in &self.beams {
            let [start, end] = beam.endpoints;
            let offset = self.positions[end] - self.positions[start];
            // The initial and every committed geometry have had their lengths checked.
            let force = beam.axial_force_on_start(
                offset,
                offset.length(),
                self.velocities[end] - self.velocities[start],
            );
            self.forces[start] += force;
            self.forces[end] -= force;
        }
        if self.forces.iter().any(|force| !force.is_finite()) {
            return Err(NonFiniteStep);
        }
        // Only free nodes are updated in either buffer. The bead slots retain rest state.
        for &index in &self.free_nodes {
            let velocity =
                self.velocities[index] + (self.forces[index] / self.node_mass) * dt_seconds;
            let position = self.positions[index] + velocity * dt_seconds;
            if !velocity.is_finite() || !position.is_finite() {
                return Err(NonFiniteStep);
            }
            self.next_velocities[index] = velocity;
            self.next_positions[index] = position;
        }
        for (index, beam) in self.beams.iter().enumerate() {
            let [start, end] = beam.endpoints;
            let length_squared =
                self.next_positions[start].distance_squared(self.next_positions[end]);
            if length_squared == 0.0 {
                return Err(CollapsedBeam(index));
            }
            if !length_squared.is_finite() {
                return Err(NonFiniteStep);
            }
        }

        std::mem::swap(&mut self.positions, &mut self.next_positions);
        std::mem::swap(&mut self.velocities, &mut self.next_velocities);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tyre::TyreConfig;

    fn default_tyre() -> ElasticTyre {
        ElasticTyre::new(
            TyreTopology::generate(&TyreConfig::default()).unwrap(),
            ElasticTyreConfig::default(),
        )
        .unwrap()
    }

    #[test]
    fn springs_pull_when_stretched_and_push_when_compressed() {
        let beam = ElasticBeam {
            endpoints: [0, 1],
            rest_length: 1.0,
            coefficients: BeamCoefficients {
                stiffness: 10.0,
                damping: 3.0,
            },
        };
        assert_eq!(
            beam.axial_force_on_start(DVec3::Y * 1.25, 1.25, DVec3::ZERO),
            DVec3::Y * 2.5,
        );
        assert_eq!(
            beam.axial_force_on_start(DVec3::Y * 0.75, 0.75, DVec3::ZERO),
            -DVec3::Y * 2.5,
        );
    }

    #[test]
    fn damping_opposes_axial_relative_motion() {
        let beam = ElasticBeam {
            endpoints: [0, 1],
            rest_length: 1.0,
            coefficients: BeamCoefficients {
                stiffness: 10.0,
                damping: 3.0,
            },
        };
        for relative_speed in [-2.0, 2.0] {
            let relative_velocity = DVec3::new(4.0, relative_speed, 8.0);
            assert_eq!(
                beam.axial_force_on_start(DVec3::Y, 1.0, relative_velocity),
                DVec3::Y * (3.0 * relative_speed),
            );
        }
        assert_eq!(
            beam.axial_force_on_start(DVec3::Y, 1.0, DVec3::X),
            DVec3::ZERO,
        );
    }

    #[test]
    fn accumulates_equal_and_opposite_endpoint_forces() {
        let mut tyre = default_tyre();
        let [a, b] = [tyre.free_nodes[0], tyre.free_nodes[1]];
        tyre.beams.retain(|beam| beam.endpoints == [a, b]);
        let offset = tyre.positions[b] - tyre.positions[a];
        tyre.positions[b] += offset * 0.25;
        let forces = vec![DVec3::ZERO; tyre.positions().len()];
        tyre.step(0.001, &forces).unwrap();
        assert!(tyre.forces[a].length() > 0.0);
        assert_eq!(tyre.forces[a], -tyre.forces[b]);
    }

    #[test]
    fn unloaded_rest_is_an_equilibrium() {
        let mut tyre = default_tyre();
        let forces = vec![DVec3::ZERO; tyre.positions().len()];
        for _ in 0..32 {
            tyre.step(1.0 / 960.0, &forces).unwrap();
        }
        assert_eq!(tyre.positions(), tyre.rest_positions);
        assert_eq!(tyre.velocities(), forces);
    }

    #[test]
    fn beads_stay_fixed_under_load_and_reset_repeats_the_motion() {
        let mut tyre = default_tyre();
        let patch = tyre
            .topology()
            .nodes()
            .iter()
            .position(|node| node.ring == RingRole::TreadCentre)
            .unwrap();
        let mut forces = vec![DVec3::ZERO; tyre.positions().len()];
        forces[0] = DVec3::X;
        forces[patch] = -DVec3::Y;
        let apply_load = |tyre: &mut ElasticTyre| {
            for _ in 0..64 {
                tyre.step(1.0 / 960.0, &forces).unwrap();
            }
        };
        apply_load(&mut tyre);
        assert!(tyre.positions()[patch].y < tyre.rest_positions[patch].y);
        for (index, node) in tyre.topology().nodes().iter().enumerate() {
            if matches!(node.ring, RingRole::LeftBead | RingRole::RightBead) {
                assert_eq!(tyre.positions()[index], tyre.rest_positions[index]);
                assert_eq!(tyre.velocities()[index], DVec3::ZERO);
            }
        }
        let positions = tyre.positions().to_vec();
        let velocities = tyre.velocities().to_vec();
        tyre.reset();
        apply_load(&mut tyre);
        assert_eq!(tyre.positions(), positions);
        assert_eq!(tyre.velocities(), velocities);
    }

    #[test]
    fn rejects_invalid_inputs_and_keeps_the_previous_state_on_failure() {
        let mut tyre = default_tyre();
        let patch = tyre.free_nodes[0];
        let mut forces = vec![DVec3::ZERO; tyre.positions().len()];
        use ElasticTyreError::*;
        assert_eq!(tyre.step(f64::NAN, &forces), Err(InvalidTimestep));
        assert!(matches!(
            tyre.step(0.001, &[]),
            Err(InvalidForceCount { .. })
        ));
        forces[patch] = DVec3::splat(f64::INFINITY);
        assert_eq!(
            tyre.step(0.001, &forces),
            Err(NonFiniteExternalForce(patch))
        );
        forces[patch] = -DVec3::Y;
        tyre.step(0.001, &forces).unwrap();
        let positions = tyre.positions().to_vec();
        let velocities = tyre.velocities().to_vec();
        forces[tyre.free_nodes[1]] = DVec3::splat(f64::MAX);
        assert_eq!(tyre.step(1.0, &forces), Err(NonFiniteStep));
        assert_eq!(tyre.positions(), positions);
        assert_eq!(tyre.velocities(), velocities);
        forces.fill(DVec3::ZERO);
        tyre.step(0.001, &forces).unwrap();
    }

    #[test]
    fn rejects_a_step_that_collapses_a_beam() {
        let topology = TyreTopology::generate(&TyreConfig::default()).unwrap();
        let config = ElasticTyreConfig {
            total_mass: topology.nodes().len() as f64,
            ..ElasticTyreConfig::default()
        };
        let mut tyre = ElasticTyre::new(topology, config).unwrap();
        let (index, beam) = tyre
            .beams
            .iter()
            .enumerate()
            .find(|(_, beam)| beam.endpoints[1] == tyre.free_nodes[0])
            .unwrap();
        let [a, b] = beam.endpoints;
        let mut forces = vec![DVec3::ZERO; tyre.positions().len()];
        forces[b] = tyre.positions()[a] - tyre.positions()[b];
        assert_eq!(
            tyre.step(1.0, &forces),
            Err(ElasticTyreError::CollapsedBeam(index))
        );
        assert_eq!(tyre.positions(), tyre.rest_positions);
        assert!(
            tyre.velocities()
                .iter()
                .all(|&velocity| velocity == DVec3::ZERO)
        );
    }

    #[test]
    fn rejects_invalid_mass_and_beam_coefficients() {
        let topology = TyreTopology::generate(&TyreConfig::default()).unwrap();
        let mut config = ElasticTyreConfig {
            total_mass: 0.0,
            ..ElasticTyreConfig::default()
        };
        assert!(matches!(
            ElasticTyre::new(topology.clone(), config),
            Err(ElasticTyreError::InvalidMass)
        ));
        config.total_mass = 8.0;
        config.diagonal.damping = -1.0;
        assert!(matches!(
            ElasticTyre::new(topology, config),
            Err(ElasticTyreError::InvalidCoefficients(BeamFamily::Diagonal)),
        ));
    }
}

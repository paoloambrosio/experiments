//! Axial springs and dampers on a tyre carcass with its bead rings fixed.

use std::{error::Error, fmt};

use bevy_math::DVec3;

use crate::{
    beam_damage::{BeamDamage, BeamDamageConfig},
    pressure::{ChamberReading, PressureChamber, PressureConfig, PressureTriangle},
    tread_reinforcement::{CrownSpring, TreadReinforcementConfig},
    tyre::{BeamFamily, RingRole, TyreTopology},
};

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
    VentedTyre,
    InvalidPressure,
    InvalidPressureTriangle(usize),
    InvalidChamberVolume,
    InvalidTreadReinforcement,
    InvalidRimMotion,
    InvalidFloor,
    InvalidKerb,
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
            Self::VentedTyre => f.write_str("a vented tyre has no sealed gas charge"),
            Self::InvalidPressure => f.write_str(
                "ambient pressure, absolute pressure, and gas charge must be positive and finite",
            ),
            Self::InvalidPressureTriangle(index) => {
                write!(f, "pressure triangle {index} has zero or non-finite area")
            }
            Self::InvalidChamberVolume => {
                f.write_str("the signed chamber volume must be positive and finite")
            }
            Self::InvalidTreadReinforcement => f.write_str(
                "tread multipliers must be at least one and effective coefficients must be finite and nonnegative",
            ),
            Self::InvalidRimMotion => f.write_str("rim motion must be finite with a unit rotation"),
            Self::InvalidFloor => f.write_str("floor settings must be finite, with nonnegative coefficients, a positive friction transition speed, and horizontal surface velocity"),
            Self::InvalidKerb => f.write_str("kerb dimensions must be finite, with a positive edge radius smaller than the rise"),
        }
    }
}

impl Error for ElasticTyreError {}

#[derive(Debug, Clone, Copy)]
struct ElasticBeam {
    endpoints: [usize; 2],
    rest_length: f64,
    coefficients: BeamCoefficients,
    stiffness_multiplier: f64,
}

impl ElasticBeam {
    fn axial_force_on_start(&self, offset: DVec3, length: f64, relative_velocity: DVec3) -> DVec3 {
        let axis = offset / length;
        let spring =
            self.coefficients.stiffness * self.stiffness_multiplier * (length - self.rest_length);
        let damping = self.coefficients.damping * relative_velocity.dot(axis);
        axis * (spring + damping)
    }
}

/// A deformable tyre with fixed bead nodes and caller-supplied external forces.
///
/// Positions use tyre-local metres, with the axle along X and segment zero on
/// +Y. Velocities are in m/s and all node arrays follow topology order.
#[derive(Debug, Clone)]
pub struct ElasticTyre {
    topology: TyreTopology,
    pub(crate) rest_positions: Vec<DVec3>,
    pub(crate) positions: Vec<DVec3>,
    pub(crate) velocities: Vec<DVec3>,
    beams: Vec<ElasticBeam>,
    beam_damage: Option<(BeamDamageConfig, Vec<BeamDamage>)>,
    next_damage: Vec<BeamDamage>,
    crown_springs: Vec<CrownSpring>,
    crown_coefficients: BeamCoefficients,
    free_nodes: Vec<usize>,
    pub(crate) bead_nodes: Vec<usize>,
    pub(crate) node_mass: f64,
    chamber: Option<PressureChamber>,
    forces: Vec<DVec3>,
    pub(crate) next_positions: Vec<DVec3>,
    pub(crate) next_velocities: Vec<DVec3>,
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
                    stiffness_multiplier: 1.0,
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
        let bead_nodes = topology
            .nodes()
            .iter()
            .enumerate()
            .filter(|(_, node)| matches!(node.ring, RingRole::LeftBead | RingRole::RightBead))
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
            beam_damage: None,
            next_damage: Vec::new(),
            crown_springs: Vec::new(),
            crown_coefficients: BeamCoefficients {
                stiffness: 0.0,
                damping: 0.0,
            },
            free_nodes,
            bead_nodes,
            node_mass,
            chamber: None,
        })
    }

    /// Construct a tyre with a sealed chamber at the given initial gauge pressure in Pa.
    pub fn new_inflated(
        topology: TyreTopology,
        config: ElasticTyreConfig,
        pressure: PressureConfig,
    ) -> Result<Self, ElasticTyreError> {
        let mut tyre = Self::new(topology, config)?;
        tyre.chamber = Some(PressureChamber::new(
            &tyre.topology,
            &tyre.positions,
            pressure,
        )?);
        Ok(tyre)
    }

    /// Configure the tread band while keeping the current node positions and gas charge.
    ///
    /// The original mesh supplies the crown spring's rest geometry. Applying another
    /// configuration replaces the multipliers and coefficients. Stiffer beams can
    /// require a smaller timestep.
    pub fn with_tread_reinforcement(
        mut self,
        config: TreadReinforcementConfig,
    ) -> Result<Self, ElasticTyreError> {
        if [
            config.hoop_stiffness_multiplier,
            config.brace_stiffness_multiplier,
        ]
        .into_iter()
        .any(|scale| !scale.is_finite() || scale < 1.0)
            || [config.crown.stiffness, config.crown.damping]
                .into_iter()
                .any(|coefficient| !coefficient.is_finite() || coefficient < 0.0)
        {
            return Err(ElasticTyreError::InvalidTreadReinforcement);
        }
        for (beam, topology_beam) in self.beams.iter_mut().zip(self.topology.beams()) {
            let in_tread = beam.endpoints.iter().all(|&index| {
                matches!(
                    self.topology.nodes()[index].ring,
                    RingRole::LeftShoulder | RingRole::TreadCentre | RingRole::RightShoulder,
                )
            });
            let multiplier = match (in_tread, topology_beam.family) {
                (true, BeamFamily::Circumferential) => config.hoop_stiffness_multiplier,
                (true, BeamFamily::CrossSection | BeamFamily::Diagonal) => {
                    config.brace_stiffness_multiplier
                }
                (false, _) => 1.0,
            };
            if !(beam.coefficients.stiffness * multiplier).is_finite() {
                return Err(ElasticTyreError::InvalidTreadReinforcement);
            }
            beam.stiffness_multiplier = multiplier;
        }
        // The five-ring topology stores every ring in the same angular order.
        let segments = self.topology.nodes().len() / 5;
        self.crown_coefficients = config.crown;
        self.crown_springs = (0..segments)
            .map(|segment| {
                let nodes = [
                    segments + segment,
                    2 * segments + segment,
                    3 * segments + segment,
                ];
                CrownSpring::new(nodes, &self.rest_positions)
            })
            .collect();
        Ok(self)
    }

    /// Change the sealed gas charge to obtain this gauge pressure at the current volume.
    /// Subsequent steps conserve the new charge.
    pub fn set_gauge_pressure(&mut self, gauge_pressure_pa: f64) -> Result<(), ElasticTyreError> {
        self.chamber
            .as_mut()
            .ok_or(ElasticTyreError::VentedTyre)?
            .set_gauge_pressure(gauge_pressure_pa)
    }

    /// Pressure and volume of the sealed chamber, when present.
    pub fn chamber_reading(&self) -> Option<ChamberReading> {
        self.chamber.as_ref().map(PressureChamber::reading)
    }

    /// Outward-facing pressure triangles indexed by node. Empty for a vented tyre.
    pub fn pressure_triangles(&self) -> &[PressureTriangle] {
        self.chamber
            .as_ref()
            .map_or(&[], |chamber| chamber.triangles.as_slice())
    }

    pub fn topology(&self) -> &TyreTopology {
        &self.topology
    }

    /// Enable axial damage on the topology beams. Reconfiguring preserves existing damage.
    pub fn with_beam_damage(mut self, config: BeamDamageConfig) -> Self {
        if let Some((material, _)) = &mut self.beam_damage {
            *material = config;
        } else {
            let states: Vec<_> = self
                .beams
                .iter()
                .map(|beam| BeamDamage::new(beam.rest_length).expect("validated topology beam"))
                .collect();
            self.next_damage = states.clone();
            self.beam_damage = Some((config, states));
        }
        self
    }

    /// Axial material state in topology beam order, when damage is enabled.
    pub fn beam_damage(&self) -> Option<&[BeamDamage]> {
        self.beam_damage
            .as_ref()
            .map(|(_, states)| states.as_slice())
    }

    /// Current node positions in tyre-local metres.
    pub fn positions(&self) -> &[DVec3] {
        &self.positions
    }

    /// Current node velocities in tyre-local m/s.
    pub fn velocities(&self) -> &[DVec3] {
        &self.velocities
    }

    /// Restore the rest positions, zero velocities, initial gas charge, and undamaged beams.
    pub fn reset(&mut self) {
        self.positions.copy_from_slice(&self.rest_positions);
        self.velocities.fill(DVec3::ZERO);
        if let Some((_, states)) = &mut self.beam_damage {
            states.iter_mut().for_each(BeamDamage::reset);
        }
        if let Some(chamber) = &mut self.chamber {
            chamber.reset();
        }
    }

    /// Advance one semi-implicit Euler step by a positive, finite number of seconds.
    ///
    /// Supply one tyre-local force vector in newtons per node. Forces on the
    /// prescribed bead nodes have no effect on their motion. The caller chooses
    /// a timestep small enough for the configured beam coefficients and mass.
    /// An error preserves the published motion, chamber state, and beam damage.
    pub fn step(
        &mut self,
        dt_seconds: f64,
        external_forces: &[DVec3],
    ) -> Result<(), ElasticTyreError> {
        self.advance(dt_seconds, external_forces, None).map(|_| ())
    }

    pub(crate) fn advance(
        &mut self,
        dt_seconds: f64,
        external_forces: &[DVec3],
        rim_centre: Option<DVec3>,
    ) -> Result<(DVec3, DVec3), ElasticTyreError> {
        self.advance_with_contact(dt_seconds, external_forces, rim_centre, |_, _, _| Ok(()))
    }

    pub(crate) fn advance_with_contact(
        &mut self,
        dt_seconds: f64,
        external_forces: &[DVec3],
        rim_centre: Option<DVec3>,
        apply_contact: impl FnOnce(&[DVec3], &[DVec3], &mut [DVec3]) -> Result<(), ElasticTyreError>,
    ) -> Result<(DVec3, DVec3), ElasticTyreError> {
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

        let damage = self.beam_damage.as_ref().map(|(_, states)| states);
        for (index, beam) in self.beams.iter().enumerate() {
            let state = damage.map(|states| states[index]);
            if state.is_some_and(BeamDamage::broken) {
                continue;
            }
            let [start, end] = beam.endpoints;
            let offset = self.positions[end] - self.positions[start];
            // Committed geometry guarantees a positive length for every intact beam.
            let mut force_beam = *beam;
            if let Some(state) = state {
                force_beam.rest_length = state.stress_free_length_m();
            }
            let force = force_beam.axial_force_on_start(
                offset,
                offset.length(),
                self.velocities[end] - self.velocities[start],
            );
            self.forces[start] += force;
            self.forces[end] -= force;
        }
        for spring in &self.crown_springs {
            spring.add_forces(
                self.crown_coefficients,
                &self.positions,
                &self.velocities,
                &mut self.forces,
            );
        }
        if let Some(chamber) = &self.chamber {
            chamber.add_forces(&self.positions, &mut self.forces);
        }
        apply_contact(&self.positions, &self.velocities, &mut self.forces)?;
        if self.forces.iter().any(|force| !force.is_finite()) {
            return Err(NonFiniteStep);
        }
        let mut force_on_rim = DVec3::ZERO;
        let mut torque_on_rim = DVec3::ZERO;
        if let Some(centre) = rim_centre {
            for &index in &self.bead_nodes {
                let force = self.forces[index];
                force_on_rim += force;
                torque_on_rim += (self.positions[index] - centre).cross(force);
            }
            if !force_on_rim.is_finite() || !torque_on_rim.is_finite() {
                return Err(NonFiniteStep);
            }
        }
        // Carry the prescribed rigid-body state into the next integration buffer.
        for &index in &self.bead_nodes {
            self.next_positions[index] = self.positions[index];
            self.next_velocities[index] = self.velocities[index];
        }
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
        self.commit_next()?;
        Ok((force_on_rim, torque_on_rim))
    }

    pub(crate) fn commit_next(&mut self) -> Result<(), ElasticTyreError> {
        use ElasticTyreError::*;

        for (index, beam) in self.beams.iter().enumerate() {
            let [start, end] = beam.endpoints;
            let length_squared =
                self.next_positions[start].distance_squared(self.next_positions[end]);
            if !length_squared.is_finite() {
                return Err(NonFiniteStep);
            }
            let broken = if let Some((config, states)) = &self.beam_damage {
                let state = states[index]
                    .at_length(length_squared.sqrt(), *config)
                    .ok_or(NonFiniteStep)?;
                self.next_damage[index] = state;
                state.broken()
            } else {
                false
            };
            if length_squared == 0.0 && !broken {
                return Err(CollapsedBeam(index));
            }
        }

        if let Some(chamber) = &mut self.chamber {
            chamber.current = chamber.at_positions(&self.next_positions)?;
        }
        if let Some((_, states)) = &mut self.beam_damage {
            std::mem::swap(states, &mut self.next_damage);
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

    #[test]
    fn broken_beam_applies_no_spring_or_damping_and_reset_restores_it() {
        let config = BeamDamageConfig::new(0.02, 0.02, 0.10, 0.10).unwrap();
        let mut tyre = default_tyre().with_beam_damage(config);
        let node = tyre.free_nodes[0];
        let beam_index = tyre
            .beams
            .iter()
            .position(|beam| beam.endpoints.contains(&node))
            .unwrap();
        let target = tyre.beams[beam_index].endpoints;
        let other = *target.iter().find(|&&index| index != node).unwrap();
        let direction = (tyre.positions[node] - tyre.positions[other]).normalize();
        // Isolate the selected beam so its force can be checked without other beams.
        for (index, beam) in tyre.beams.iter_mut().enumerate() {
            if index != beam_index {
                beam.coefficients = BeamCoefficients {
                    stiffness: 0.0,
                    damping: 0.0,
                };
            }
        }
        tyre.next_positions.copy_from_slice(&tyre.positions);
        tyre.next_positions[node] += direction * tyre.beams[beam_index].rest_length * 0.20;
        tyre.next_velocities[node] = direction;
        tyre.commit_next().unwrap();
        assert!(tyre.beam_damage().unwrap()[beam_index].broken());
        let clone = tyre.clone();
        assert_eq!(tyre.beam_damage(), clone.beam_damage());
        let forces = vec![DVec3::ZERO; tyre.positions.len()];
        tyre.step(1.0 / 3840.0, &forces).unwrap();
        assert_eq!(tyre.velocities[node], direction);
        tyre.reset();
        assert!(!tyre.beam_damage().unwrap()[beam_index].broken());
        assert_eq!(
            tyre.beam_damage().unwrap()[beam_index].permanent_length_change_m(),
            0.0
        );
        assert!(clone.beam_damage().unwrap()[beam_index].broken());
    }

    #[test]
    fn invalid_chamber_candidate_does_not_commit_staged_damage() {
        let topology = TyreTopology::generate(&TyreConfig::default()).unwrap();
        let mut tyre = ElasticTyre::new_inflated(
            topology,
            ElasticTyreConfig::default(),
            PressureConfig::default(),
        )
        .unwrap()
        .with_beam_damage(BeamDamageConfig::new(0.02, 0.02, 0.06, 0.06).unwrap());
        let before = tyre.clone();
        for (next, point) in tyre.next_positions.iter_mut().zip(&tyre.positions) {
            *next = point.with_x(-point.x);
        }
        tyre.next_positions[tyre.free_nodes[0]].y += 0.04;
        assert_eq!(
            tyre.commit_next(),
            Err(ElasticTyreError::InvalidChamberVolume)
        );
        assert!(tyre.next_damage.iter().any(|state| state.broken()));
        assert_eq!(tyre.beam_damage(), before.beam_damage());
        assert_eq!(tyre.positions(), before.positions());
        assert_eq!(tyre.velocities(), before.velocities());
        assert_eq!(tyre.chamber_reading(), before.chamber_reading());
    }

    #[test]
    fn reinforcement_is_applied_only_to_the_tread_band() {
        let mut tyre = default_tyre()
            .with_tread_reinforcement(TreadReinforcementConfig::default())
            .unwrap();
        let mut hoop_count = 0;
        let mut brace_count = 0;
        for (beam, topology_beam) in tyre.beams.iter().zip(tyre.topology.beams()) {
            let rings = beam
                .endpoints
                .map(|index| tyre.topology.nodes()[index].ring);
            if rings.contains(&RingRole::LeftBead) || rings.contains(&RingRole::RightBead) {
                assert_eq!(beam.stiffness_multiplier, 1.0);
            } else if topology_beam.family == BeamFamily::Circumferential {
                assert_eq!(beam.stiffness_multiplier, 20.0);
                hoop_count += 1;
            } else {
                assert_eq!(beam.stiffness_multiplier, 2.0);
                brace_count += 1;
            }
        }
        let segments = TyreConfig::default().segments;
        assert_eq!(hoop_count, 3 * segments);
        assert_eq!(brace_count, 6 * segments);
        assert_eq!(tyre.crown_springs.len(), segments);
        for spring in &tyre.crown_springs {
            assert_eq!(
                spring.nodes.map(|index| tyre.topology.nodes()[index].ring),
                [
                    RingRole::LeftShoulder,
                    RingRole::TreadCentre,
                    RingRole::RightShoulder
                ],
            );
        }
        let unloaded = vec![DVec3::ZERO; tyre.positions.len()];
        tyre.step(1.0 / 3840.0, &unloaded).unwrap();
        assert_eq!(tyre.positions, tyre.rest_positions);
        assert_eq!(tyre.velocities, unloaded);
    }

    #[test]
    fn inflated_tyre_keeps_beads_fixed_and_reset_restores_initial_charge() {
        let config = PressureConfig {
            initial_gauge_pressure_pa: 10_000.0,
            ..PressureConfig::default()
        };
        let mut tyre = ElasticTyre::new_inflated(
            TyreTopology::generate(&TyreConfig::default()).unwrap(),
            ElasticTyreConfig::default(),
            config,
        )
        .unwrap();
        let initial = tyre.chamber_reading().unwrap();
        tyre.set_gauge_pressure(15_000.0).unwrap();
        let forces = vec![DVec3::ZERO; tyre.positions().len()];
        for _ in 0..120 {
            tyre.step(1.0 / 960.0, &forces).unwrap();
        }
        assert!(tyre.chamber_reading().unwrap().volume_m3 > initial.volume_m3);
        for (index, node) in tyre.topology().nodes().iter().enumerate() {
            if matches!(node.ring, RingRole::LeftBead | RingRole::RightBead) {
                assert_eq!(tyre.positions()[index], node.rest_position.as_dvec3());
                assert_eq!(tyre.velocities()[index], DVec3::ZERO);
            }
        }
        tyre.reset();
        assert_eq!(tyre.chamber_reading(), Some(initial));
        assert_eq!(tyre.positions(), tyre.rest_positions);
    }

    #[test]
    fn invalid_pressure_changes_and_invalid_candidate_meshes_roll_back() {
        let topology = TyreTopology::generate(&TyreConfig::default()).unwrap();
        let config = ElasticTyreConfig {
            total_mass: topology.nodes().len() as f64,
            ..ElasticTyreConfig::default()
        };
        let mut tyre =
            ElasticTyre::new_inflated(topology, config, PressureConfig::default()).unwrap();
        let initial = tyre.chamber_reading();
        assert_eq!(
            tyre.set_gauge_pressure(f64::NAN),
            Err(ElasticTyreError::InvalidPressure)
        );
        assert_eq!(
            tyre.set_gauge_pressure(-101_325.0),
            Err(ElasticTyreError::InvalidPressure)
        );
        assert_eq!(tyre.chamber_reading(), initial);

        // Flat triangle with three distinct vertices and all beam lengths nonzero.
        let [a, b, c] = tyre.pressure_triangles()[0].nodes;
        let midpoint = (tyre.positions()[a] + tyre.positions()[b]) * 0.5;
        let mut forces = vec![DVec3::ZERO; tyre.positions().len()];
        forces[c] = midpoint - tyre.positions()[c];
        assert_eq!(
            tyre.step(1.0, &forces),
            Err(ElasticTyreError::InvalidPressureTriangle(0))
        );
        assert_eq!(tyre.positions(), tyre.rest_positions);
        assert!(
            tyre.velocities()
                .iter()
                .all(|velocity| *velocity == DVec3::ZERO)
        );
        assert_eq!(tyre.chamber_reading(), initial);
        forces.fill(DVec3::ZERO);
        tyre.step(1.0 / 960.0, &forces).unwrap();
        assert_eq!(tyre.chamber_reading(), initial);
    }

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
            stiffness_multiplier: 1.0,
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
            stiffness_multiplier: 1.0,
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

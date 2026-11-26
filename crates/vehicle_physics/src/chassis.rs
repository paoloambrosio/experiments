//! Free node-and-beam structures in world space. Positions are in metres, with +Y
//! up, +Z forward at spawn and +X to vehicle left. The crash wall faces −Z.

use std::{error::Error, fmt, ops::RangeInclusive};

use bevy_math::DVec3;

use crate::beam_damage::{BeamDamage, BeamDamageConfig};
pub use crate::elastic_tyre::BeamCoefficients;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChassisNode {
    pub mass_kg: f64,
    pub position_m: DVec3,
    pub velocity_mps: DVec3,
}

#[derive(Debug, Clone, Copy)]
pub struct ChassisBeamSpec {
    /// Indices into the chassis node array.
    pub endpoints: [usize; 2],
    pub coefficients: BeamCoefficients,
    /// Optional axial yielding and fracture limits.
    pub material: Option<BeamDamageConfig>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct WallFace {
    z_m: f64,
    x_m: RangeInclusive<f64>,
    y_m: RangeInclusive<f64>,
}

impl WallFace {
    /// A finite, frictionless face at fixed world Z. Contact acts from the lower-Z side.
    pub fn new(
        z_m: f64,
        x_m: RangeInclusive<f64>,
        y_m: RangeInclusive<f64>,
    ) -> Result<Self, ChassisError> {
        let valid_span = |span: &RangeInclusive<f64>| {
            span.start().is_finite() && span.end().is_finite() && span.start() < span.end()
        };
        if !z_m.is_finite() || !valid_span(&x_m) || !valid_span(&y_m) {
            return Err(ChassisError::InvalidWall);
        }
        Ok(Self { z_m, x_m, y_m })
    }

    fn contact(&self, previous: DVec3, travel: DVec3, node: &mut ChassisNode) -> Option<f64> {
        if travel.z <= 0.0 || previous.z > self.z_m || node.position_m.z < self.z_m {
            return None;
        }
        let fraction = (self.z_m - previous.z) / travel.z;
        let crossing = previous + fraction * travel;
        if !self.x_m.contains(&crossing.x) || !self.y_m.contains(&crossing.y) {
            return None;
        }
        let trial_depth_m = node.position_m.z - self.z_m;
        node.position_m.z = self.z_m;
        node.velocity_mps.z = 0.0;
        Some(trial_depth_m)
    }
}

/// Contact from a successful chassis step.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct WallReaction {
    /// Sum of wall impulses on the chassis along world +Z, in N·s.
    pub impulse_z_ns: f64,
    pub struck_nodes: usize,
    /// Largest distance an unconstrained node would have passed through the face.
    pub max_trial_depth_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChassisError {
    NoNodes,
    InvalidNode(usize),
    InvalidBeam(usize),
    InvalidWall,
    InvalidTimestep,
    CollapsedBeam(usize),
    NonFiniteStep,
}

impl fmt::Display for ChassisError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Self::NoNodes => f.write_str("the chassis needs at least one node"),
            Self::InvalidNode(index) => write!(f, "invalid motion or mass at chassis node {index}"),
            Self::InvalidBeam(index) => write!(
                f,
                "invalid endpoints, length, or coefficients at chassis beam {index}"
            ),
            Self::InvalidWall => {
                f.write_str("the wall needs finite coordinates and positive width and height")
            }
            Self::InvalidTimestep => f.write_str("the timestep must be positive and finite"),
            Self::CollapsedBeam(index) => {
                write!(f, "chassis beam {index} collapsed to zero length")
            }
            Self::NonFiniteStep => {
                f.write_str("the chassis step produced non-finite motion, geometry, or impulse")
            }
        }
    }
}

impl Error for ChassisError {}

#[derive(Debug, Clone, Copy)]
struct Beam {
    spec: ChassisBeamSpec,
    damage: BeamDamage,
}

/// A free node-and-beam chassis with an initial geometry and launch state for reset.
#[derive(Debug, Clone)]
pub struct Chassis {
    initial: Vec<ChassisNode>,
    nodes: Vec<ChassisNode>,
    beams: Vec<Beam>,
    forces: Vec<DVec3>,
    next_nodes: Vec<ChassisNode>,
    next_damage: Vec<BeamDamage>,
}

impl Chassis {
    pub fn new(nodes: Vec<ChassisNode>, specs: Vec<ChassisBeamSpec>) -> Result<Self, ChassisError> {
        if nodes.is_empty() {
            return Err(ChassisError::NoNodes);
        }
        for (index, node) in nodes.iter().enumerate() {
            if !node.mass_kg.is_finite()
                || node.mass_kg <= 0.0
                || !node.position_m.is_finite()
                || !node.velocity_mps.is_finite()
            {
                return Err(ChassisError::InvalidNode(index));
            }
        }
        let beams: Vec<_> = specs
            .into_iter()
            .enumerate()
            .map(|(index, spec)| {
                let [a, b] = spec.endpoints;
                let invalid = ChassisError::InvalidBeam(index);
                let a = nodes.get(a).ok_or(invalid)?;
                let b = nodes.get(b).ok_or(invalid)?;
                let length = a.position_m.distance(b.position_m);
                let damage = BeamDamage::new(length).ok_or(invalid)?;
                let coefficients = spec.coefficients;
                if [coefficients.stiffness, coefficients.damping]
                    .into_iter()
                    .any(|value| !value.is_finite() || value < 0.0)
                {
                    return Err(invalid);
                }
                Ok(Beam { spec, damage })
            })
            .collect::<Result<_, _>>()?;
        Ok(Self {
            initial: nodes.clone(),
            next_nodes: nodes.clone(),
            forces: vec![DVec3::ZERO; nodes.len()],
            next_damage: beams.iter().map(|beam| beam.damage).collect(),
            nodes,
            beams,
        })
    }

    /// Node states in constructor order.
    pub fn nodes(&self) -> &[ChassisNode] {
        &self.nodes
    }

    /// Beam material states in constructor order.
    pub fn beam_damage(&self) -> impl ExactSizeIterator<Item = BeamDamage> + '_ {
        self.beams.iter().map(|beam| beam.damage)
    }

    /// Restore the initial geometry, launch velocities, and stress-free lengths.
    pub fn reset(&mut self) {
        self.nodes.copy_from_slice(&self.initial);
        for beam in &mut self.beams {
            beam.damage.reset();
        }
    }

    /// Advance by one semi-implicit substep under internal beam forces and an optional
    /// wall reaction. On error the previously committed node and beam states remain valid.
    pub fn step(
        &mut self,
        dt_seconds: f64,
        wall: Option<&WallFace>,
    ) -> Result<WallReaction, ChassisError> {
        if !dt_seconds.is_finite() || dt_seconds <= 0.0 {
            return Err(ChassisError::InvalidTimestep);
        }
        self.forces.fill(DVec3::ZERO);
        for (index, beam) in self.beams.iter().enumerate() {
            if beam.damage.broken() {
                continue;
            }
            let [a, b] = beam.spec.endpoints;
            let offset = self.nodes[b].position_m - self.nodes[a].position_m;
            let length = checked_length(offset, index)?;
            let axis = offset / length;
            let speed = (self.nodes[b].velocity_mps - self.nodes[a].velocity_mps).dot(axis);
            let coefficients = beam.spec.coefficients;
            let spring = coefficients.stiffness * (length - beam.damage.stress_free_length_m());
            let force = axis * (spring + coefficients.damping * speed);
            self.forces[a] += force;
            self.forces[b] -= force;
        }

        let mut reaction = WallReaction::default();
        for ((next, previous), force) in self
            .next_nodes
            .iter_mut()
            .zip(&self.nodes)
            .zip(&self.forces)
        {
            let velocity = previous.velocity_mps + (*force / previous.mass_kg) * dt_seconds;
            let travel = velocity * dt_seconds;
            let position = previous.position_m + travel;
            if !velocity.is_finite() || !travel.is_finite() || !position.is_finite() {
                return Err(ChassisError::NonFiniteStep);
            }
            *next = ChassisNode {
                mass_kg: previous.mass_kg,
                position_m: position,
                velocity_mps: velocity,
            };
            if let Some(depth) =
                wall.and_then(|face| face.contact(previous.position_m, travel, next))
            {
                reaction.impulse_z_ns -= previous.mass_kg * velocity.z;
                reaction.struck_nodes += 1;
                reaction.max_trial_depth_m = reaction.max_trial_depth_m.max(depth);
            }
        }
        if !reaction.impulse_z_ns.is_finite() || !reaction.max_trial_depth_m.is_finite() {
            return Err(ChassisError::NonFiniteStep);
        }

        for (index, (beam, next)) in self.beams.iter().zip(&mut self.next_damage).enumerate() {
            *next = beam.damage;
            if beam.damage.broken() {
                continue;
            }
            let [a, b] = beam.spec.endpoints;
            let offset = self.next_nodes[b].position_m - self.next_nodes[a].position_m;
            let length = checked_length(offset, index)?;
            if let Some(material) = beam.spec.material {
                *next = beam
                    .damage
                    .at_length(length, material)
                    .ok_or(ChassisError::NonFiniteStep)?;
            }
        }

        std::mem::swap(&mut self.nodes, &mut self.next_nodes);
        for (beam, &next) in self.beams.iter_mut().zip(&self.next_damage) {
            beam.damage = next;
        }
        Ok(reaction)
    }
}

fn checked_length(offset: DVec3, beam: usize) -> Result<f64, ChassisError> {
    let length = offset.length();
    if length == 0.0 {
        return Err(ChassisError::CollapsedBeam(beam));
    }
    if !length.is_finite() {
        return Err(ChassisError::NonFiniteStep);
    }
    Ok(length)
}

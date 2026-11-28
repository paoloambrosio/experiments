//! Linearized backward Euler for the axial chassis springs and dampers.
//! Suspension impulses and swept contacts still use the resulting trial
//! velocities before committing node positions and material damage.

use bevy_math::{DMat3, DVec3};

use super::{Beam, ChassisBeamKind, ChassisError, ChassisNode, checked_length};

#[derive(Debug, Clone)]
struct Edge {
    endpoints: [usize; 2],
    axis: DVec3,
    // dt * k * extension (N·s) and dt * (c + dt * k) (kg).
    spring_impulse: f64,
    coefficient: f64,
    unilateral: bool,
    active: bool,
}

impl Edge {
    fn apply(&self, values: &[DVec3]) -> DVec3 {
        let [a, b] = self.endpoints;
        self.axis * (self.coefficient * (values[a] - values[b]).dot(self.axis))
    }

    fn should_activate(&self, velocities: &[DVec3]) -> bool {
        self.spring_impulse < self.apply(velocities).dot(self.axis)
    }
}

#[derive(Debug, Clone)]
pub(super) struct BeamSolver {
    edges: Vec<Edge>,
    velocities: Vec<DVec3>,
    velocity_delta: Vec<DVec3>,
    rhs: Vec<DVec3>,
    inverse_blocks: Vec<DMat3>,
    residual: Vec<DVec3>,
    direction: Vec<DVec3>,
    product: Vec<DVec3>,
}

impl BeamSolver {
    pub(super) fn new(nodes: usize, beams: usize) -> Self {
        Self {
            edges: Vec::with_capacity(beams),
            velocities: vec![DVec3::ZERO; nodes],
            velocity_delta: vec![DVec3::ZERO; nodes],
            rhs: vec![DVec3::ZERO; nodes],
            inverse_blocks: vec![DMat3::ZERO; nodes],
            residual: vec![DVec3::ZERO; nodes],
            direction: vec![DVec3::ZERO; nodes],
            product: vec![DVec3::ZERO; nodes],
        }
    }

    pub(super) fn reset(&mut self) {
        self.velocity_delta.fill(DVec3::ZERO);
    }

    pub(super) fn predict(
        &mut self,
        nodes: &[ChassisNode],
        beams: &[Beam],
        next: &mut [ChassisNode],
        dt: f64,
        gravity: DVec3,
    ) -> Result<(), ChassisError> {
        if !dt.is_finite() || dt <= 0.0 {
            return Err(ChassisError::InvalidTimestep);
        }
        for (velocity, node) in self.velocities.iter_mut().zip(nodes) {
            *velocity = node.velocity_mps;
        }
        self.edges.clear();
        for (index, beam) in beams.iter().enumerate() {
            if beam.damage.broken() {
                continue;
            }
            let [a, b] = beam.spec.endpoints;
            let delta = nodes[b].position_m - nodes[a].position_m;
            let length = checked_length(delta, index)?;
            let extension = length - beam.damage.stress_free_length_m();
            let unilateral = beam.spec.kind == ChassisBeamKind::CompressionSupport;
            if unilateral && extension > 0.0 {
                continue;
            }
            let material = beam.spec.coefficients;
            let mut edge = Edge {
                endpoints: [a, b],
                axis: delta / length,
                spring_impulse: dt * material.stiffness * extension,
                coefficient: dt * (material.damping + dt * material.stiffness),
                unilateral,
                active: true,
            };
            edge.active = !unilateral || edge.should_activate(&self.velocities);
            self.edges.push(edge);
        }

        // A compression support may switch as the solved velocity changes.
        // Rebuild the active set until no support would pull in tension.
        for _ in 0..8 {
            self.solve(nodes, dt * gravity)?;
            for ((velocity, node), &delta) in self
                .velocities
                .iter_mut()
                .zip(nodes)
                .zip(&self.velocity_delta)
            {
                *velocity = node.velocity_mps + delta;
            }
            let mut changed = false;
            for edge in self.edges.iter_mut().filter(|edge| edge.unilateral) {
                let active = edge.should_activate(&self.velocities);
                changed |= active != edge.active;
                edge.active = active;
            }
            if !changed {
                for ((trial, node), &velocity) in next.iter_mut().zip(nodes).zip(&self.velocities) {
                    *trial = ChassisNode {
                        velocity_mps: velocity,
                        ..*node
                    };
                }
                return Ok(());
            }
        }
        Err(ChassisError::BeamSolveDidNotConverge)
    }

    fn solve(&mut self, nodes: &[ChassisNode], gravity_delta: DVec3) -> Result<(), ChassisError> {
        for (i, node) in nodes.iter().enumerate() {
            self.rhs[i] = node.mass_kg * gravity_delta;
            self.inverse_blocks[i] = DMat3::from_diagonal(DVec3::splat(node.mass_kg));
        }
        for edge in self.edges.iter().filter(|edge| edge.active) {
            let [a, b] = edge.endpoints;
            let old_relative_speed = (nodes[b].velocity_mps - nodes[a].velocity_mps).dot(edge.axis);
            let impulse = (edge.spring_impulse + edge.coefficient * old_relative_speed) * edge.axis;
            self.rhs[a] += impulse;
            self.rhs[b] -= impulse;
            let axis = edge.axis;
            let scaled = edge.coefficient * axis;
            let block = DMat3::from_cols(axis.x * scaled, axis.y * scaled, axis.z * scaled);
            self.inverse_blocks[a] += block;
            self.inverse_blocks[b] += block;
        }
        // Keep the off-axis terms of each node's diagonal block. A beam
        // accelerating several coordinate axes is still one axial mode.
        for block in &mut self.inverse_blocks {
            *block = block.inverse();
        }
        // Solve for the velocity increment. The last increment is usually a
        // close guess, even while the vehicle has a large rigid velocity.
        apply_matrix(nodes, &self.edges, &self.velocity_delta, &mut self.product);
        let mut cold_error = 0.0;
        for i in 0..nodes.len() {
            self.residual[i] = self.rhs[i] - self.product[i];
            self.direction[i] = self.inverse_blocks[i] * self.residual[i];
            cold_error += self.rhs[i].dot(self.inverse_blocks[i] * self.rhs[i]);
        }
        let mut error = dot(&self.residual, &self.direction);
        let tolerance = (cold_error * 1e-10).max(1e-16);
        // Preconditioned conjugate gradient. The positive node masses and
        // nonnegative axis coefficients make the system positive definite.
        for _ in 0..64 {
            if !error.is_finite() {
                return Err(ChassisError::NonFiniteStep);
            }
            if error <= tolerance {
                return Ok(());
            }
            apply_matrix(nodes, &self.edges, &self.direction, &mut self.product);
            let alpha = error / dot(&self.direction, &self.product);
            if !alpha.is_finite() {
                return Err(ChassisError::NonFiniteStep);
            }
            let mut next_error = 0.0;
            for i in 0..nodes.len() {
                self.velocity_delta[i] += alpha * self.direction[i];
                self.residual[i] -= alpha * self.product[i];
                self.product[i] = self.inverse_blocks[i] * self.residual[i];
                next_error += self.residual[i].dot(self.product[i]);
            }
            if next_error <= tolerance {
                return Ok(());
            }
            let beta = next_error / error;
            for (direction, &residual) in self.direction.iter_mut().zip(&self.product) {
                *direction = residual + beta * *direction;
            }
            error = next_error;
        }
        Err(ChassisError::BeamSolveDidNotConverge)
    }
}

fn apply_matrix(nodes: &[ChassisNode], edges: &[Edge], values: &[DVec3], result: &mut [DVec3]) {
    for ((result, &value), node) in result.iter_mut().zip(values).zip(nodes) {
        *result = node.mass_kg * value;
    }
    for edge in edges.iter().filter(|edge| edge.active) {
        let [a, b] = edge.endpoints;
        let response = edge.apply(values);
        result[a] += response;
        result[b] -= response;
    }
}

fn dot(a: &[DVec3], b: &[DVec3]) -> f64 {
    a.iter().zip(b).map(|(a, b)| a.dot(*b)).sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chassis::{BeamCoefficients, Chassis, ChassisBeamSpec};

    #[test]
    fn stiff_beams_preserve_momentum_and_supports_release_in_extension() {
        let dt = 1.0 / 3840.0;
        let coefficients = BeamCoefficients {
            stiffness: 5e8,
            damping: 1e6,
        };
        let factor = 1.0
            + dt * (coefficients.damping + dt * coefficients.stiffness)
                * (0.5_f64.recip() + 200.0_f64.recip());
        for (kind, incoming, expected) in [
            (ChassisBeamKind::Bilateral, 1.0, 1.0 / factor),
            (ChassisBeamKind::CompressionSupport, -1.0, -1.0 / factor),
            (ChassisBeamKind::CompressionSupport, 1.0, 1.0),
        ] {
            let nodes = vec![
                ChassisNode {
                    mass_kg: 200.0,
                    position_m: DVec3::ZERO,
                    velocity_mps: DVec3::ZERO,
                },
                ChassisNode {
                    mass_kg: 0.5,
                    position_m: DVec3::X,
                    velocity_mps: incoming * DVec3::X,
                },
            ];
            let mut chassis = Chassis::new(
                nodes,
                vec![ChassisBeamSpec {
                    endpoints: [0, 1],
                    coefficients,
                    kind,
                    material: None,
                }],
            )
            .unwrap();
            chassis.use_implicit_beams();
            let trial = chassis.stage_loaded_step(dt, DVec3::ZERO).unwrap();
            let relative = (trial[1].velocity_mps - trial[0].velocity_mps).x;
            let momentum = trial
                .iter()
                .map(|node| node.mass_kg * node.velocity_mps.x)
                .sum::<f64>();
            assert!(
                (relative - expected).abs() < 1e-10,
                "{kind:?}: {relative} vs {expected}"
            );
            assert!((momentum - 0.5 * incoming).abs() < 1e-10);
        }
    }
}

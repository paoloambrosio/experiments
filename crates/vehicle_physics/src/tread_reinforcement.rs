//! Reinforcement across the tread-centre and shoulder rings.

use bevy_math::DVec3;

use crate::elastic_tyre::BeamCoefficients;

/// Additional stiffness for the crown of a node-and-beam tyre.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TreadReinforcementConfig {
    /// Stiffness multiplier for circumferential beams on the three tread rings.
    pub hoop_stiffness_multiplier: f64,
    /// Stiffness multiplier for beams joining the tread centre to either shoulder.
    pub brace_stiffness_multiplier: f64,
    /// Spring and damper coefficients for the centre-to-shoulder-midpoint distance.
    pub crown: BeamCoefficients,
}

impl Default for TreadReinforcementConfig {
    fn default() -> Self {
        Self {
            hoop_stiffness_multiplier: 20.0,
            brace_stiffness_multiplier: 2.0,
            crown: BeamCoefficients {
                stiffness: 30_000.0,
                damping: 5.0,
            },
        }
    }
}

#[derive(Debug, Clone)]
pub(crate) struct CrownSpring {
    pub(crate) nodes: [usize; 3],
    rest_height: f64,
}

impl CrownSpring {
    pub(crate) fn new(nodes: [usize; 3], rest_positions: &[DVec3]) -> Self {
        Self {
            nodes,
            rest_height: softened_length(midpoint_offset(nodes, rest_positions)),
        }
    }

    pub(crate) fn add_forces(
        &self,
        coefficients: BeamCoefficients,
        positions: &[DVec3],
        velocities: &[DVec3],
        forces: &mut [DVec3],
    ) {
        let offset = midpoint_offset(self.nodes, positions);
        let height = softened_length(offset);
        // The gradient of the softened distance goes to zero at a flat crown.
        let gradient = offset / height;
        let height_rate = midpoint_offset(self.nodes, velocities).dot(gradient);
        let force = -gradient
            * (coefficients.stiffness * (height - self.rest_height)
                + coefficients.damping * height_rate);
        let [left, centre, right] = self.nodes;
        forces[centre] += force;
        forces[left] -= force * 0.5;
        forces[right] -= force * 0.5;
    }
}

fn midpoint_offset([left, centre, right]: [usize; 3], vectors: &[DVec3]) -> DVec3 {
    vectors[centre] - (vectors[left] * 0.5 + vectors[right] * 0.5)
}

fn softened_length(offset: DVec3) -> f64 {
    // Smooth the distance over one micrometre to avoid a singular force direction.
    (offset.length_squared() + 1e-12).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crown_spring_is_balanced_and_zero_at_rest() {
        let rest = [-DVec3::X, DVec3::Y * 0.1, DVec3::X];
        let spring = CrownSpring::new([0, 1, 2], &rest);
        let coefficients = TreadReinforcementConfig::default().crown;
        let mut forces = [DVec3::ZERO; 3];
        spring.add_forces(coefficients, &rest, &[DVec3::ZERO; 3], &mut forces);
        assert_eq!(forces, [DVec3::ZERO; 3]);

        let deformed = [
            -DVec3::X,
            DVec3::new(0.05, 0.18, 0.12),
            DVec3::new(1.0, -0.1, 0.1),
        ];
        let velocities = [DVec3::Y, DVec3::X, DVec3::ZERO];
        spring.add_forces(coefficients, &deformed, &velocities, &mut forces);
        assert!(forces[1].length() > 1.0);
        assert!(forces.into_iter().sum::<DVec3>().length() < 1e-10);
        let torque: DVec3 = deformed
            .into_iter()
            .zip(forces)
            .map(|(position, force)| position.cross(force))
            .sum();
        assert!(torque.length() < 1e-10);
    }

    #[test]
    fn crown_spring_stays_finite_when_the_tread_is_flat() {
        let flat = [-DVec3::X, DVec3::ZERO, DVec3::X];
        let rounded = [-DVec3::X, DVec3::Y * 0.1, DVec3::X];
        let coefficients = TreadReinforcementConfig::default().crown;
        for rest in [flat, rounded] {
            let spring = CrownSpring::new([0, 1, 2], &rest);
            let mut forces = [DVec3::ZERO; 3];
            spring.add_forces(coefficients, &flat, &[DVec3::ZERO; 3], &mut forces);
            assert!(forces.into_iter().all(DVec3::is_finite));
        }
    }
}

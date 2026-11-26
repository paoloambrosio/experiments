use bevy::math::{DMat3, DQuat, DVec3};

use crate::chassis::ChassisNode;

/// Three structural nodes and an offset from the first node in rest vehicle axes.
/// Positions are in metres, with +Y up, +Z forward, and +X left at rest.
#[derive(Clone, Copy, Debug)]
pub struct SuspensionMount {
    pub nodes: [usize; 3],
    pub offset_m: DVec3,
}

#[derive(Clone, Copy)]
pub(crate) struct Mount {
    pub spec: SuspensionMount,
    rest_inverse: DQuat,
}

impl Mount {
    pub fn new(spec: SuspensionMount, nodes: &[ChassisNode]) -> Option<Self> {
        if !spec.offset_m.is_finite() {
            return None;
        }
        let [Some(a), Some(b), Some(c)] = spec.nodes.map(|i| nodes.get(i)) else {
            return None;
        };
        let triangle = [*a, *b, *c];
        let mount = Self {
            spec,
            rest_inverse: frame(triangle)?.conjugate(),
        };
        mount.patch(nodes)?;
        Some(mount)
    }

    pub fn patch(self, nodes: &[ChassisNode]) -> Option<Patch> {
        let triangle = self.spec.nodes.map(|i| nodes[i]);
        let rotation = (frame(triangle)? * self.rest_inverse).normalize();
        let anchor = triangle[0].position_m + rotation * self.spec.offset_m;
        let mass: f64 = triangle.iter().map(|n| n.mass_kg).sum();
        let centre = triangle
            .iter()
            .map(|n| n.position_m * (n.mass_kg / mass))
            .sum();
        let levers = triangle.map(|n| n.position_m - centre);
        let inertia = point_inertia(triangle.iter().zip(levers).map(|(n, r)| (n.mass_kg, r)));
        let inverse_inertia = inertia.inverse();
        if !anchor.is_finite() || !inverse_inertia.is_finite() {
            return None;
        }
        Some(Patch {
            anchor,
            rotation,
            centre,
            inverse_inertia,
            levers,
            indices: self.spec.nodes,
            mass,
            masses: triangle.map(|n| n.mass_kg),
        })
    }
}

fn frame([a, b, c]: [ChassisNode; 3]) -> Option<DQuat> {
    let x = (b.position_m - a.position_m).try_normalize()?;
    let other = c.position_m - a.position_m;
    let plane = x.cross(other);
    if plane.length() <= 1e-6 * other.length() {
        return None;
    }
    let z = plane.try_normalize()?;
    let y = z.cross(x);
    Some(DQuat::from_mat3(&DMat3::from_cols(x, y, z)).normalize())
}

pub(crate) fn point_inertia(points: impl IntoIterator<Item = (f64, DVec3)>) -> DMat3 {
    points
        .into_iter()
        .map(|(m, r)| m * (DMat3::from_diagonal(DVec3::splat(r.length_squared())) - outer(r)))
        .fold(DMat3::ZERO, |total, inertia| total + inertia)
}

pub(crate) fn outer(v: DVec3) -> DMat3 {
    DMat3::from_cols(v * v.x, v * v.y, v * v.z)
}

pub(crate) struct Patch {
    pub anchor: DVec3,
    pub rotation: DQuat,
    centre: DVec3,
    inverse_inertia: DMat3,
    levers: [DVec3; 3],
    indices: [usize; 3],
    mass: f64,
    masses: [f64; 3],
}

impl Patch {
    pub fn motion(&self, nodes: &[ChassisNode], point: DVec3) -> (DVec3, DVec3) {
        let mut momentum = DVec3::ZERO;
        let mut angular_momentum = DVec3::ZERO;
        for i in 0..3 {
            let p = self.masses[i] * nodes[self.indices[i]].velocity_mps;
            momentum += p;
            angular_momentum += self.levers[i].cross(p);
        }
        let omega = self.inverse_inertia * angular_momentum;
        (
            momentum / self.mass + omega.cross(point - self.centre),
            omega,
        )
    }

    pub fn inverse_mass(&self, point: DVec3, axis: DVec3) -> f64 {
        let angular_axis = (point - self.centre).cross(axis);
        self.mass.recip() + angular_axis.dot(self.inverse_inertia * angular_axis)
    }

    pub fn inverse_angular_mass(&self, axis: DVec3) -> f64 {
        axis.dot(self.inverse_inertia * axis)
    }

    /// Apply a wrench expressed about the same world point used by the caller.
    pub fn apply(&self, nodes: &mut [ChassisNode], point: DVec3, impulse: DVec3, moment: DVec3) {
        let spin = self.inverse_inertia * (moment + (point - self.centre).cross(impulse));
        for i in 0..3 {
            nodes[self.indices[i]].velocity_mps += impulse / self.mass + spin.cross(self.levers[i]);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nodal_impulses_reproduce_the_reaction_about_the_upright() {
        let mut nodes = [DVec3::X, DVec3::Y, DVec3::Z].map(|position_m| ChassisNode {
            mass_kg: 3.0 + position_m.y,
            position_m,
            velocity_mps: DVec3::ZERO,
        });
        let mount = Mount::new(
            SuspensionMount {
                nodes: [0, 1, 2],
                offset_m: DVec3::NEG_Y,
            },
            &nodes,
        )
        .unwrap();
        let patch = mount.patch(&nodes).unwrap();
        let point = DVec3::new(2.0, -0.7, -0.3);
        let impulse = DVec3::new(-3.0, 6.0, 5.0);
        let moment = DVec3::new(1.0, 2.0, -4.0);
        patch.apply(&mut nodes, point, -impulse, -moment);
        let actual_impulse = nodes
            .iter()
            .map(|n| n.mass_kg * n.velocity_mps)
            .sum::<DVec3>();
        let actual_moment = nodes
            .iter()
            .map(|n| (n.position_m - point).cross(n.mass_kg * n.velocity_mps))
            .sum::<DVec3>();
        assert!(actual_impulse.abs_diff_eq(-impulse, 1e-12));
        assert!(actual_moment.abs_diff_eq(-moment, 1e-12));
    }
}

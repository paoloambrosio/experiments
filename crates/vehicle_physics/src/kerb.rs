//! Straight, full-width step with a rounded leading edge.

use bevy_math::DVec3;

use crate::elastic_tyre::ElasticTyreError;

/// Cross-section extruded along world X. Approach from negative Z with world +Y up.
/// The top extends in +Z and rises above the supplied flat floor height.
#[derive(Debug, Clone, Copy)]
pub struct Kerb {
    leading_z_m: f64,
    rise_m: f64,
    radius_m: f64,
}

impl Kerb {
    pub fn new(leading_z_m: f64, rise_m: f64, radius_m: f64) -> Result<Self, ElasticTyreError> {
        if !leading_z_m.is_finite()
            || !rise_m.is_finite()
            || !radius_m.is_finite()
            || radius_m <= 0.0
            || radius_m >= rise_m
        {
            return Err(ElasticTyreError::InvalidKerb);
        }
        Ok(Self {
            leading_z_m,
            rise_m,
            radius_m,
        })
    }

    /// Depth in metres and outward unit normal for a point inside the step on its
    /// positive-Z side. Use flat-floor contact on the approach side.
    pub fn contact(self, floor_y_m: f64, position: DVec3) -> Option<(f64, DVec3)> {
        let z = position.z - self.leading_z_m;
        let y = position.y - floor_y_m;
        if z <= 0.0 || y >= self.rise_m {
            return None;
        }

        let shoulder_y = self.rise_m - self.radius_m;
        if z < self.radius_m && y > shoulder_y {
            let radial = DVec3::new(0.0, y - shoulder_y, z - self.radius_m);
            let radius = radial.length();
            let depth = self.radius_m - radius;
            return (depth > 0.0).then(|| (depth, radial / radius));
        }

        // Below the rounded edge, compare the exposed face (or toe) with the top.
        // Use one closest boundary so overlapping planes cannot double the force.
        // For points below the floor, the toe is the nearest point on the face.
        let face_offset = DVec3::new(0.0, y - y.clamp(0.0, shoulder_y), z);
        let face_depth = face_offset.length();
        let top_depth = self.rise_m - y;
        if z >= self.radius_m && top_depth <= face_depth {
            Some((top_depth, DVec3::Y))
        } else {
            Some((face_depth, -face_offset / face_depth))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn leads_with_one_face_and_rounds_continuously_to_the_top() {
        let kerb = Kerb::new(0.0, 0.06, 0.01).unwrap();
        assert!(kerb.contact(0.0, DVec3::new(0.0, 0.025, -0.001)).is_none());
        assert!(kerb.contact(0.0, DVec3::new(0.0, 0.059, 0.001)).is_none());
        for (point, expected) in [
            (DVec3::new(0.0, 0.032, 0.001), DVec3::NEG_Z),
            (DVec3::new(0.0, 0.05, 0.001), DVec3::NEG_Z),
            (
                DVec3::new(
                    0.0,
                    0.05 + 0.009 / 2.0_f64.sqrt(),
                    0.01 - 0.009 / 2.0_f64.sqrt(),
                ),
                (DVec3::Y - DVec3::Z).normalize(),
            ),
            (DVec3::new(0.0, 0.059, 0.01), DVec3::Y),
            (DVec3::new(0.0, 0.059, 0.02), DVec3::Y),
        ] {
            let (depth, normal) = kerb.contact(0.0, point).unwrap();
            assert!((depth - 0.001).abs() < 1e-12);
            assert!((normal - expected).length() < 1e-12);
        }
    }
}

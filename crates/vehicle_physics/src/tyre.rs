//! Generate the rest geometry of a five-ring node-and-beam tyre.

use std::{error::Error, f64::consts::TAU, fmt};

use bevy_math::Vec3;

/// Cross-section dimensions in metres and the number of nodes per ring.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TyreConfig {
    pub segments: usize,
    pub bead_radius: f32,
    pub shoulder_radius: f32,
    pub tread_radius: f32,
    pub bead_width: f32,
    pub shoulder_width: f32,
}

impl Default for TyreConfig {
    fn default() -> Self {
        Self {
            segments: 32,
            bead_radius: 0.20,
            shoulder_radius: 0.32,
            tread_radius: 0.33,
            bead_width: 0.17,
            shoulder_width: 0.20,
        }
    }
}

/// The cross-section ring a node belongs to, ordered from negative to positive X.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RingRole {
    LeftBead,
    LeftShoulder,
    TreadCentre,
    RightShoulder,
    RightBead,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TyreNode {
    /// Rest position in tyre-local metres. The axle lies on X.
    pub rest_position: Vec3,
    pub ring: RingRole,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BeamFamily {
    /// Joins consecutive nodes in one ring.
    Circumferential,
    /// Joins adjacent rings at the same angular segment.
    CrossSection,
    /// Joins adjacent rings at consecutive angular segments.
    Diagonal,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TyreBeam {
    /// An unordered pair of indices into [`TyreTopology::nodes`].
    pub endpoints: [usize; 2],
    pub family: BeamFamily,
    pub rest_length: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TyreConfigError {
    TooFewSegments,
    InvalidRadii,
    InvalidWidths,
    UnrepresentableGeometry,
    CapacityExceeded,
}

impl fmt::Display for TyreConfigError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(match self {
            Self::TooFewSegments => "a tyre needs at least three circumferential segments",
            Self::InvalidRadii => "radii must be finite and satisfy 0 < bead < shoulder <= tread",
            Self::InvalidWidths => {
                "widths must be finite and satisfy 0 < bead width <= shoulder width"
            }
            Self::UnrepresentableGeometry => {
                "tyre dimensions and resolution must produce positive, finite beam lengths"
            }
            Self::CapacityExceeded => "the tyre has too many nodes or beams to allocate",
        })
    }
}

impl Error for TyreConfigError {}

/// Nodes and beams at rest. Nodes are stored ring by ring.
#[derive(Debug, Clone, PartialEq)]
pub struct TyreTopology {
    nodes: Vec<TyreNode>,
    beams: Vec<TyreBeam>,
}

impl TyreTopology {
    pub fn generate(config: &TyreConfig) -> Result<Self, TyreConfigError> {
        use TyreConfigError::*;

        let segments = config.segments;
        if segments < 3 {
            return Err(TooFewSegments);
        }
        if !(0.0 < config.bead_radius
            && config.bead_radius < config.shoulder_radius
            && config.shoulder_radius <= config.tread_radius
            && config.tread_radius.is_finite())
        {
            return Err(InvalidRadii);
        }
        if !(0.0 < config.bead_width
            && config.bead_width <= config.shoulder_width
            && config.shoulder_width.is_finite())
        {
            return Err(InvalidWidths);
        }

        let bead_offset = config.bead_width / 2.0;
        let shoulder_offset = config.shoulder_width / 2.0;
        if bead_offset == 0.0 {
            return Err(UnrepresentableGeometry);
        }
        let rings = [
            (RingRole::LeftBead, -bead_offset, config.bead_radius),
            (
                RingRole::LeftShoulder,
                -shoulder_offset,
                config.shoulder_radius,
            ),
            (RingRole::TreadCentre, 0.0, config.tread_radius),
            (
                RingRole::RightShoulder,
                shoulder_offset,
                config.shoulder_radius,
            ),
            (RingRole::RightBead, bead_offset, config.bead_radius),
        ];

        let node_count = segments.checked_mul(rings.len()).ok_or(CapacityExceeded)?;
        let beam_count = segments
            .checked_mul(rings.len() + 3 * (rings.len() - 1))
            .ok_or(CapacityExceeded)?;
        let mut nodes = Vec::new();
        let mut beams = Vec::new();
        nodes
            .try_reserve_exact(node_count)
            .map_err(|_| CapacityExceeded)?;
        beams
            .try_reserve_exact(beam_count)
            .map_err(|_| CapacityExceeded)?;

        for (ring, x, radius) in rings {
            for segment in 0..segments {
                let angle = TAU * segment as f64 / segments as f64;
                let (sin, cos) = angle.sin_cos();
                let radius = f64::from(radius);
                nodes.push(TyreNode {
                    rest_position: Vec3::new(x, (radius * cos) as f32, (radius * sin) as f32),
                    ring,
                });
            }
        }

        let mut add_beam = |a: usize, b: usize, family| {
            let start = nodes[a].rest_position.as_dvec3();
            let end = nodes[b].rest_position.as_dvec3();
            let rest_length = start.distance(end) as f32;
            if rest_length <= 0.0 || !rest_length.is_finite() {
                return Err(UnrepresentableGeometry);
            }
            beams.push(TyreBeam {
                endpoints: [a, b],
                family,
                rest_length,
            });
            Ok(())
        };

        for ring in 0..rings.len() {
            let ring_start = ring * segments;
            for segment in 0..segments {
                // The modulo joins the last segment back to the first without a duplicate node.
                let next = (segment + 1) % segments;
                let a = ring_start + segment;
                let b = ring_start + next;
                add_beam(a, b, BeamFamily::Circumferential)?;

                if ring + 1 < rings.len() {
                    let c = a + segments;
                    let d = b + segments;
                    add_beam(a, c, BeamFamily::CrossSection)?;
                    add_beam(a, d, BeamFamily::Diagonal)?;
                    add_beam(b, c, BeamFamily::Diagonal)?;
                }
            }
        }

        Ok(Self { nodes, beams })
    }

    pub fn nodes(&self) -> &[TyreNode] {
        &self.nodes
    }

    pub fn beams(&self) -> &[TyreBeam] {
        &self.beams
    }
}

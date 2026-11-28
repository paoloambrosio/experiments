//! Finite, static road support for tread contact in a +Y-up world.

use std::{error::Error, fmt};

use bevy_math::DVec3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriangleRoadError {
    InvalidDepth,
    NonFiniteVertex(usize),
    InvalidTriangle(usize),
    NoDrivingFaces,
}

impl fmt::Display for TriangleRoadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Self::InvalidDepth => f.write_str("road contact depth must be positive and finite"),
            Self::NonFiniteVertex(index) => write!(f, "road vertex {index} is not finite"),
            Self::InvalidTriangle(index) => {
                write!(f, "road triangle {index} has an invalid index or area")
            }
            Self::NoDrivingFaces => f.write_str("road has no faces with a horizontal projection"),
        }
    }
}

impl Error for TriangleRoadError {}

/// An immutable, indexed road that can be shared between mounted tyres.
#[derive(Debug)]
pub struct TriangleRoad {
    faces: FaceTree,
    max_vertical_depth_m: f64,
}

impl TriangleRoad {
    /// Build from world-space vertices in metres and zero-based triangle indices.
    /// Either winding is accepted. Vertical faces are omitted from tread support.
    /// The depth bound limits how far above a tread node a road may support it.
    /// Choose a bound wider than the expected penetration and smaller than gaps
    /// between stacked road levels.
    pub fn new(
        vertices: &[DVec3],
        triangles: &[[usize; 3]],
        max_vertical_depth_m: f64,
    ) -> Result<Self, TriangleRoadError> {
        if !max_vertical_depth_m.is_finite() || max_vertical_depth_m <= 0.0 {
            return Err(TriangleRoadError::InvalidDepth);
        }
        if let Some(index) = vertices.iter().position(|vertex| !vertex.is_finite()) {
            return Err(TriangleRoadError::NonFiniteVertex(index));
        }
        let mut faces = Vec::new();
        for (index, &[a, b, c]) in triangles.iter().enumerate() {
            let (Some(&a), Some(&b), Some(&c)) =
                (vertices.get(a), vertices.get(b), vertices.get(c))
            else {
                return Err(TriangleRoadError::InvalidTriangle(index));
            };
            if let Some(face) = Face::new(index, a, b, c)? {
                faces.push(face);
            }
        }
        if faces.is_empty() {
            return Err(TriangleRoadError::NoDrivingFaces);
        }
        Ok(Self {
            faces: FaceTree::build(faces),
            max_vertical_depth_m,
        })
    }

    pub(crate) fn contact(&self, point: DVec3) -> Option<SurfaceHit> {
        let mut hit = None;
        self.faces.find(point, self.max_vertical_depth_m, &mut hit);
        hit
    }

    pub(crate) fn tread_patch(&self, points: &[DVec3]) -> TreadPatch<'_> {
        let (min, max) = points.iter().fold(
            (DVec3::splat(f64::INFINITY), DVec3::splat(f64::NEG_INFINITY)),
            |(min, max), &point| (min.min(point), max.max(point)),
        );
        let envelope = Bounds {
            min,
            max: max + self.max_vertical_depth_m * DVec3::Y,
        };
        let mut patch = TreadPatch {
            road: self,
            faces: [None; 16],
            len: 0,
            max_y: f64::NEG_INFINITY,
            complete: true,
        };
        patch.complete = self.faces.collect_patch(envelope, &mut patch);
        patch
    }
}

pub(crate) struct TreadPatch<'a> {
    road: &'a TriangleRoad,
    faces: [Option<&'a Face>; 16],
    len: usize,
    max_y: f64,
    complete: bool,
}

impl TreadPatch<'_> {
    pub(crate) fn contact(&self, point: DVec3) -> Option<SurfaceHit> {
        if !self.complete {
            return self.road.contact(point);
        }
        if point.y >= self.max_y {
            return None;
        }
        let mut hit = None;
        for face in self.faces[..self.len].iter().flatten() {
            face.consider(point, self.road.max_vertical_depth_m, &mut hit);
        }
        hit
    }
}

// Allow round-off at shared edges when computing barycentric coordinates.
const SEAM_EPSILON: f64 = 32.0 * f64::EPSILON;

#[derive(Clone, Copy, Debug)]
struct Bounds {
    min: DVec3,
    max: DVec3,
}

impl Bounds {
    fn union(self, other: Self) -> Self {
        Self {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    fn intersects_column(self, point: DVec3, depth: f64) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.z >= self.min.z
            && point.z <= self.max.z
            && point.y < self.max.y
            && point.y + depth >= self.min.y
    }

    fn overlaps(self, other: Self) -> bool {
        self.min.cmple(other.max).all() && self.max.cmpge(other.min).all()
    }
}

#[derive(Debug)]
struct Face {
    index: usize,
    origin: DVec3,
    ab: DVec3,
    ac: DVec3,
    normal: DVec3,
    inverse_determinant: f64,
    bounds: Bounds,
}

impl Face {
    fn new(index: usize, a: DVec3, b: DVec3, c: DVec3) -> Result<Option<Self>, TriangleRoadError> {
        let ab = b - a;
        let ac = c - a;
        let area = ab.cross(ac);
        let length = area.length();
        if !length.is_finite() || length == 0.0 {
            return Err(TriangleRoadError::InvalidTriangle(index));
        }
        let normal = area / length;
        if normal.y.abs() <= SEAM_EPSILON {
            return Ok(None);
        }
        let inverse_determinant = (-area.y).recip();
        if !inverse_determinant.is_finite() {
            return Err(TriangleRoadError::InvalidTriangle(index));
        }
        let min = a.min(b).min(c);
        let max = a.max(b).max(c);
        let padding = ((max - min) * SEAM_EPSILON).with_y(0.0);
        Ok(Some(Self {
            index,
            origin: a,
            ab,
            ac,
            normal: if normal.y < 0.0 { -normal } else { normal },
            inverse_determinant,
            bounds: Bounds {
                min: min - padding,
                max: max + padding,
            },
        }))
    }

    fn hit(&self, point: DVec3, max_depth: f64) -> Option<SurfaceHit> {
        if !self.bounds.intersects_column(point, max_depth) {
            return None;
        }
        let offset = point - self.origin;
        let u = (offset.x * self.ac.z - offset.z * self.ac.x) * self.inverse_determinant;
        let v = (self.ab.x * offset.z - self.ab.z * offset.x) * self.inverse_determinant;
        if u < -SEAM_EPSILON || v < -SEAM_EPSILON || u + v > 1.0 + SEAM_EPSILON {
            return None;
        }
        let vertical_depth = u * self.ab.y + v * self.ac.y - offset.y;
        (vertical_depth > 0.0 && vertical_depth <= max_depth).then_some(SurfaceHit {
            index: self.index,
            vertical_depth,
            normal: self.normal,
        })
    }

    fn consider(&self, point: DVec3, max_depth: f64, hit: &mut Option<SurfaceHit>) {
        if let Some(candidate) = self.hit(point, max_depth)
            && hit.as_ref().is_none_or(|best| {
                // Take the first road above the node, breaking ties by source index.
                candidate
                    .vertical_depth
                    .total_cmp(&best.vertical_depth)
                    .then(candidate.index.cmp(&best.index))
                    .is_lt()
            })
        {
            *hit = Some(candidate);
        }
    }
}

#[derive(Debug)]
pub(crate) struct SurfaceHit {
    index: usize,
    vertical_depth: f64,
    pub normal: DVec3,
}

impl SurfaceHit {
    pub fn depth(&self) -> f64 {
        self.vertical_depth * self.normal.y
    }
}

#[derive(Debug)]
struct FaceTree {
    bounds: Bounds,
    contents: FaceTreeContents,
}

#[derive(Debug)]
enum FaceTreeContents {
    Leaf(Vec<Face>),
    Branch(Box<[FaceTree; 2]>),
}

impl FaceTree {
    fn build(mut faces: Vec<Face>) -> Self {
        let bounds = faces
            .iter()
            .map(|face| face.bounds)
            .reduce(Bounds::union)
            .unwrap();
        let contents = if faces.len() <= 8 {
            FaceTreeContents::Leaf(faces)
        } else {
            let span = bounds.max - bounds.min;
            let axis = if span.x >= span.y.max(span.z) {
                0
            } else if span.y >= span.z {
                1
            } else {
                2
            };
            let middle = faces.len() / 2;
            let centre = |face: &Face| 0.5 * face.bounds.min[axis] + 0.5 * face.bounds.max[axis];
            faces.select_nth_unstable_by(middle, |a, b| {
                centre(a).total_cmp(&centre(b)).then(a.index.cmp(&b.index))
            });
            let right = faces.split_off(middle);
            FaceTreeContents::Branch(Box::new([Self::build(faces), Self::build(right)]))
        };
        Self { bounds, contents }
    }

    fn find(&self, point: DVec3, max_depth: f64, hit: &mut Option<SurfaceHit>) {
        if !self.bounds.intersects_column(point, max_depth) {
            return;
        }
        match &self.contents {
            FaceTreeContents::Leaf(faces) => {
                for face in faces {
                    face.consider(point, max_depth, hit);
                }
            }
            FaceTreeContents::Branch(children) => {
                for child in children.iter() {
                    child.find(point, max_depth, hit);
                }
            }
        }
    }

    fn collect_patch<'a>(&'a self, envelope: Bounds, patch: &mut TreadPatch<'a>) -> bool {
        if !self.bounds.overlaps(envelope) {
            return true;
        }
        match &self.contents {
            FaceTreeContents::Leaf(faces) => {
                for face in faces.iter().filter(|face| face.bounds.overlaps(envelope)) {
                    let Some(slot) = patch.faces.get_mut(patch.len) else {
                        return false;
                    };
                    *slot = Some(face);
                    patch.len += 1;
                    patch.max_y = patch.max_y.max(face.bounds.max.y);
                }
                true
            }
            FaceTreeContents::Branch(children) => children
                .iter()
                .all(|child| child.collect_patch(envelope, patch)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn batched_tread_lookup_matches_single_node_lookup_across_seams_and_layers() {
        let points = [
            DVec3::new(0.5, -0.01, 0.5),
            DVec3::new(0.25, 0.02, 0.75),
            DVec3::new(0.75, 0.99, 0.25),
            DVec3::new(1.1, -0.01, 0.25),
        ];
        for count in [2, 20] {
            let mut vertices = Vec::new();
            let mut triangles = Vec::new();
            for index in 0..count {
                let first = vertices.len();
                let origin = if index % 2 == 0 {
                    DVec3::ZERO
                } else {
                    DVec3::Y
                };
                vertices.extend([
                    origin,
                    origin + DVec3::X,
                    origin + DVec3::X + DVec3::Z,
                    origin + DVec3::Z,
                ]);
                triangles.extend([[first, first + 1, first + 2], [first, first + 3, first + 2]]);
            }
            let road = TriangleRoad::new(&vertices, &triangles, 0.05).unwrap();
            let patch = road.tread_patch(&points);
            assert_eq!(patch.complete, count == 2);
            for point in points {
                let key = |hit: SurfaceHit| (hit.index, hit.vertical_depth, hit.normal);
                assert_eq!(patch.contact(point).map(key), road.contact(point).map(key));
            }
        }
    }

    #[test]
    fn road_faces_are_bounded_and_accept_both_windings_at_seams() {
        let vertices = [DVec3::ZERO, DVec3::X, DVec3::X + DVec3::Z, DVec3::Z];
        // The two faces have opposite winding.
        let road = TriangleRoad::new(&vertices, &[[0, 1, 2], [0, 3, 2]], 0.05).unwrap();
        for (x, z, index) in [
            (0.75, 0.25, 0),
            (0.25, 0.75, 1),
            (0.5, 0.5, 0),
            (0.0, 0.0, 0),
            (1.0, 0.5, 0),
        ] {
            let hit = road.contact(DVec3::new(x, -0.001, z)).unwrap();
            assert_eq!(hit.index, index);
            assert_eq!(hit.normal, DVec3::Y);
            assert_eq!(hit.depth(), 0.001);
        }
        assert!(road.contact(DVec3::new(1.001, -0.001, 0.5)).is_none());
        assert!(road.contact(DVec3::new(0.5, 0.0, 0.5)).is_none());
        assert!(road.contact(DVec3::new(0.5, -0.051, 0.5)).is_none());
    }

    #[test]
    fn indexed_search_selects_the_local_road_level() {
        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        for (x, y) in [
            (0.0, 1.0),
            (0.0, 0.0),
            (10.0, 0.0),
            (20.0, 0.0),
            (30.0, 0.0),
        ] {
            let origin = DVec3::new(x, y, 0.0);
            let i = vertices.len();
            vertices.extend([
                origin,
                origin + DVec3::X,
                origin + DVec3::Z,
                origin + DVec3::X + DVec3::Z,
            ]);
            triangles.extend([[i, i + 1, i + 2], [i + 1, i + 3, i + 2]]);
        }
        let road = TriangleRoad::new(&vertices, &triangles, 0.05).unwrap();
        assert_eq!(road.contact(DVec3::new(0.2, -0.001, 0.2)).unwrap().index, 2);
        assert_eq!(road.contact(DVec3::new(0.2, 0.999, 0.2)).unwrap().index, 0);
        assert!(road.contact(DVec3::new(0.2, 0.01, 0.2)).is_none());
        assert_eq!(
            road.contact(DVec3::new(30.2, -0.001, 0.2)).unwrap().index,
            8
        );
    }

    #[test]
    fn invalid_geometry_reports_its_source_index_and_walls_are_omitted() {
        let vertices = [DVec3::ZERO, DVec3::X, DVec3::Y, DVec3::Z];
        assert!(matches!(
            TriangleRoad::new(&vertices, &[[0, 1, 3], [0, 4, 3]], 0.05),
            Err(TriangleRoadError::InvalidTriangle(1))
        ));
        assert!(matches!(
            TriangleRoad::new(&vertices, &[[0, 0, 3]], 0.05),
            Err(TriangleRoadError::InvalidTriangle(0))
        ));
        assert!(matches!(
            TriangleRoad::new(&vertices, &[[0, 1, 2]], 0.05),
            Err(TriangleRoadError::NoDrivingFaces)
        ));
        assert!(TriangleRoad::new(&vertices, &[[0, 1, 2], [0, 1, 3]], 0.05).is_ok());
        assert!(matches!(
            TriangleRoad::new(&[DVec3::NAN], &[], 0.05),
            Err(TriangleRoadError::NonFiniteVertex(0))
        ));
    }
}

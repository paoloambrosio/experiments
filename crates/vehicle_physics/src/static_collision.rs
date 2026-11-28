//! Swept contact with finite triangles in world space, with distances in metres.

use std::{collections::BTreeMap, error::Error, fmt};

use bevy_math::DVec3;

// A distance tolerance for closed edges and a smaller minimum approach per sweep.
pub(crate) const CONTACT_GAP_M: f64 = 1e-8;
const EDGE_EPS: f64 = 1e-10;
const NODE_PLANE_EPS_M: f64 = 1e-10;
const MIN_APPROACH_M: f64 = 1e-12;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StaticCollisionError {
    EmptyMesh,
    InvalidVertex(usize),
    InvalidTriangle(usize),
}

impl fmt::Display for StaticCollisionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyMesh => f.write_str("the static collision mesh needs at least one triangle"),
            Self::InvalidVertex(i) => write!(f, "non-finite static collision vertex {i}"),
            Self::InvalidTriangle(i) => write!(f, "invalid indices or area at static triangle {i}"),
        }
    }
}

impl Error for StaticCollisionError {}

/// Immutable acceleration structure for two-sided static collision triangles.
#[derive(Debug)]
pub struct StaticCollisionMesh {
    tree: BoundsTree,
}

#[derive(Clone, Copy)]
pub(crate) enum SweepMode {
    AdvanceToContact,
    FullStep,
}

impl StaticCollisionMesh {
    /// Build from world-space vertices and zero-based triangle indices.
    /// Triangle indices in contact reactions refer to this input order.
    pub fn new(vertices: &[DVec3], triangles: &[[usize; 3]]) -> Result<Self, StaticCollisionError> {
        if triangles.is_empty() {
            return Err(StaticCollisionError::EmptyMesh);
        }
        if let Some(i) = vertices.iter().position(|v| !v.is_finite()) {
            return Err(StaticCollisionError::InvalidVertex(i));
        }
        let mut faces = Vec::with_capacity(triangles.len());
        let mut edges = BTreeMap::<_, Vec<(usize, usize)>>::new();
        for (index, &ids) in triangles.iter().enumerate() {
            let [Some(&a), Some(&b), Some(&c)] = ids.map(|i| vertices.get(i)) else {
                return Err(StaticCollisionError::InvalidTriangle(index));
            };
            let points = [a, b, c];
            let normal = face_normal(points).ok_or(StaticCollisionError::InvalidTriangle(index))?;
            faces.push(Face {
                points,
                normal,
                bounds: Bounds::of(points),
                index,
                edges: [false; 3],
            });
            for edge in 0..3 {
                let [mut a, mut b] = [ids[edge], ids[(edge + 1) % 3]];
                if a > b {
                    std::mem::swap(&mut a, &mut b);
                }
                edges.entry([a, b]).or_default().push((index, edge));
            }
        }
        for owners in edges.values() {
            let (face, edge) = owners[0];
            // A shared coplanar diagonal is neither a boundary nor a crease.
            if owners.len() == 1
                || owners
                    .iter()
                    .any(|&(i, _)| faces[i].normal.dot(faces[face].normal).abs() < 1.0 - EDGE_EPS)
            {
                faces[face].edges[edge] = true;
            }
        }
        Ok(Self {
            tree: BoundsTree::build(faces),
        })
    }

    pub(crate) fn first_hit(
        &self,
        positions: &[DVec3],
        travel: &[DVec3],
        nodes: &[usize],
        panels: &[[usize; 3]],
        mode: SweepMode,
        allow: impl Fn(&Hit) -> bool,
    ) -> Option<Hit> {
        // Advancing to a later hit can leave the earlier feature beyond the
        // plane. A full-step solver can prefer features within a time tie
        // because it sweeps again from the original positions.
        let time_tolerance = match mode {
            SweepMode::AdvanceToContact => 0.0,
            SweepMode::FullStep => EDGE_EPS,
        };
        let mut best = None::<Hit>;
        let mut consider = |hit: Hit| {
            if !allow(&hit) {
                return;
            }
            let key = |hit: &Hit| (hit.contact.feature, hit.contact.triangle);
            if best.as_ref().is_none_or(|old| {
                hit.time < old.time - time_tolerance
                    || ((hit.time - old.time).abs() <= time_tolerance && key(&hit) < key(old))
            }) {
                best = Some(hit);
            }
        };
        for &node in nodes {
            let start = positions[node];
            let delta = travel[node];
            self.tree
                .visit(Bounds::of([start, start + delta]), &mut |face| {
                    let speed = delta.dot(face.normal);
                    if speed.abs() <= MIN_APPROACH_M {
                        return;
                    }
                    let distance = (face.points[0] - start).dot(face.normal);
                    let t = distance / speed;
                    // A previous hit can advance a simultaneous contact a
                    // rounding error past the plane. Resolve it at time zero.
                    let recover_roundoff = matches!(mode, SweepMode::AdvanceToContact)
                        && t < 0.0
                        && distance.abs() <= NODE_PLANE_EPS_M;
                    if !recover_roundoff && !(0.0..=1.0).contains(&t) {
                        return;
                    }
                    let (t, point) = if recover_roundoff {
                        (0.0, start + distance * face.normal)
                    } else {
                        (t, start + t * delta)
                    };
                    if barycentric(face.points, point, face.normal).is_some() {
                        consider(Hit::new(
                            t,
                            face.index,
                            ContactFeature::Node(node),
                            point,
                            -face.normal * speed.signum(),
                            [node; 3],
                            [1.0, 0.0, 0.0],
                        ));
                    }
                });
        }
        for (panel, &ids) in panels.iter().enumerate() {
            let start = ids.map(|i| positions[i]);
            let delta = ids.map(|i| travel[i]);
            let end = std::array::from_fn::<_, 3, _>(|i| start[i] + delta[i]);
            let bounds = Bounds::of(start.into_iter().chain(end));
            self.tree.visit(bounds, &mut |face| {
                sweep_panel(start, delta, ids, panel, bounds, face, &mut consider);
            });
        }
        best
    }

    pub(crate) fn overlaps_bounds(&self, min: DVec3, max: DVec3) -> bool {
        self.tree.has_overlap(Bounds { min, max })
    }
}

/// Feature on the moving body that received an obstacle reaction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ContactFeature {
    Node(usize),
    Panel(usize),
    PanelEdge { panel: usize, edge: usize },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StaticContact {
    pub triangle: usize,
    pub feature: ContactFeature,
    pub position_m: DVec3,
    /// Unit normal directed against the incident motion.
    pub normal: DVec3,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct StaticReaction {
    pub first: Option<StaticContact>,
    /// Net impulse on the moving body, in world N·s.
    pub impulse_ns: DVec3,
    pub contacts: usize,
    /// Largest remaining normal overlap at the resolved contact points, in metres.
    pub max_residual_depth_m: f64,
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct Hit {
    pub contact: StaticContact,
    pub time: f64,
    pub nodes: [usize; 3],
    pub weights: [f64; 3],
}

impl Hit {
    fn new(
        time: f64,
        triangle: usize,
        feature: ContactFeature,
        position_m: DVec3,
        normal: DVec3,
        nodes: [usize; 3],
        weights: [f64; 3],
    ) -> Self {
        Self {
            contact: StaticContact {
                triangle,
                feature,
                position_m,
                normal,
            },
            time,
            nodes,
            weights,
        }
    }

    pub fn weights(self) -> impl Iterator<Item = (usize, f64)> {
        self.nodes
            .into_iter()
            .zip(self.weights)
            .filter(|&(_, w)| w > 0.0)
    }

    pub fn interpolate(self, values: &[DVec3]) -> DVec3 {
        self.weights().map(|(i, w)| values[i] * w).sum()
    }
}

fn sweep_panel(
    start: [DVec3; 3],
    delta: [DVec3; 3],
    ids: [usize; 3],
    panel: usize,
    bounds: Bounds,
    face: &Face,
    mut consider: impl FnMut(Hit),
) {
    let a = start[0];
    let da = delta[0];
    let u = start[1] - a;
    let v = start[2] - a;
    let du = delta[1] - da;
    let dv = delta[2] - da;
    let n0 = u.cross(v);
    let n1 = du.cross(v) + u.cross(dv);
    let n2 = du.cross(dv);
    for point in face.points {
        // Barycentric coordinates may reach EDGE_EPS beyond the panel.
        if !bounds
            .with_feature_tolerance()
            .overlaps(Bounds::of([point]))
        {
            continue;
        }
        let r = a - point;
        for t in roots([
            r.dot(n0),
            r.dot(n1) + da.dot(n0),
            r.dot(n2) + da.dot(n1),
            da.dot(n2),
        ]) {
            let moving = std::array::from_fn(|i| start[i] + t * delta[i]);
            let Some(normal) = face_normal(moving) else {
                continue;
            };
            let Some(weights) = barycentric(moving, point, normal) else {
                continue;
            };
            let velocity: DVec3 = delta.into_iter().zip(weights).map(|(d, w)| d * w).sum();
            let speed = velocity.dot(normal);
            if speed.abs() > MIN_APPROACH_M {
                consider(Hit::new(
                    t,
                    face.index,
                    ContactFeature::Panel(panel),
                    point,
                    -normal * speed.signum(),
                    ids,
                    weights,
                ));
            }
        }
    }
    for (edge, enabled) in face.edges.into_iter().enumerate() {
        if !enabled {
            continue;
        }
        let p = face.points[edge];
        let q = face.points[(edge + 1) % 3];
        if !bounds
            .with_feature_tolerance()
            .overlaps(Bounds::of([p, q]).with_feature_tolerance())
        {
            continue;
        }
        let fixed = q - p;
        for i in 0..3 {
            let j = (i + 1) % 3;
            let r = start[i] - p;
            let u = start[j] - start[i];
            let du = delta[j] - delta[i];
            let n0 = u.cross(fixed);
            let n1 = du.cross(fixed);
            for t in roots([
                r.dot(n0),
                r.dot(n1) + delta[i].dot(n0),
                delta[i].dot(n1),
                0.0,
            ]) {
                let moving = u + t * du;
                let offset = r + t * delta[i];
                let cross = moving.cross(fixed);
                let denom = cross.length_squared();
                if denom <= EDGE_EPS * EDGE_EPS * moving.length_squared() * fixed.length_squared() {
                    continue;
                }
                let s = -offset.cross(fixed).dot(cross) / denom;
                let q = -offset.cross(moving).dot(cross) / denom;
                if !(-EDGE_EPS..=1.0 + EDGE_EPS).contains(&s)
                    || !(-EDGE_EPS..=1.0 + EDGE_EPS).contains(&q)
                {
                    continue;
                }
                let s = s.clamp(0.0, 1.0);
                let normal = cross / denom.sqrt();
                let speed = delta[i].lerp(delta[j], s).dot(normal);
                if speed.abs() > MIN_APPROACH_M {
                    let mut weights = [0.0; 3];
                    weights[i] = 1.0 - s;
                    weights[j] = s;
                    consider(Hit::new(
                        t,
                        face.index,
                        ContactFeature::PanelEdge { panel, edge: i },
                        p + fixed * q.clamp(0.0, 1.0),
                        -normal * speed.signum(),
                        ids,
                        weights,
                    ));
                }
            }
        }
    }
}

pub(crate) fn face_normal([a, b, c]: [DVec3; 3]) -> Option<DVec3> {
    (b - a).cross(c - a).try_normalize()
}

fn barycentric([a, b, c]: [DVec3; 3], point: DVec3, normal: DVec3) -> Option<[f64; 3]> {
    let u = b - a;
    let v = c - a;
    let r = point - a;
    let area = u.cross(v).dot(normal);
    let s = r.cross(v).dot(normal) / area;
    let t = u.cross(r).dot(normal) / area;
    let weights = [1.0 - s - t, s, t];
    if weights.into_iter().all(|w| w >= -EDGE_EPS) {
        let clamped = weights.map(|w| w.max(0.0));
        let sum: f64 = clamped.iter().sum();
        Some(clamped.map(|w| w / sum))
    } else {
        None
    }
}

// Split a cubic at derivative roots into monotone intervals, then bisect sign
// changes. Callers reject tangencies using the normal approach at each root.
fn roots([a, b, c, d]: [f64; 4]) -> impl Iterator<Item = f64> {
    let scale = a.abs().max(b.abs()).max(c.abs()).max(d.abs());
    let mut found = [None; 6];
    if scale == 0.0 || !scale.is_finite() {
        return found.into_iter().flatten();
    }
    let [a, b, c, d] = [a, b, c, d].map(|x| x / scale);
    if d.abs() <= f64::EPSILON && c.abs() <= f64::EPSILON {
        let t = -a / b;
        found[0] = (0.0..=1.0).contains(&t).then_some(t);
        return found.into_iter().flatten();
    }
    let mut cuts = [0.0, 1.0, 1.0, 1.0];
    let discriminant = (2.0 * c).powi(2) - 12.0 * d * b;
    if d.abs() > f64::EPSILON && discriminant >= 0.0 {
        let q = -0.5 * (2.0 * c + discriminant.sqrt().copysign(c));
        cuts[1] = (q / (3.0 * d)).clamp(0.0, 1.0);
        if q != 0.0 {
            cuts[2] = (b / q).clamp(0.0, 1.0);
        }
    } else if d.abs() <= f64::EPSILON {
        cuts[1] = (-b / (2.0 * c)).clamp(0.0, 1.0);
    }
    cuts.sort_by(f64::total_cmp);
    let eval = |t| ((d * t + c) * t + b) * t + a;
    let mut count = 0;
    for [mut lo, mut hi] in [[cuts[0], cuts[1]], [cuts[1], cuts[2]], [cuts[2], cuts[3]]] {
        if lo == hi {
            continue;
        }
        let low = eval(lo);
        let high = eval(hi);
        if low == 0.0 {
            found[count] = Some(lo);
            count += 1;
        }
        if high == 0.0 && hi == 1.0 {
            found[count] = Some(hi);
            count += 1;
        } else if low != 0.0 && high != 0.0 && low.is_sign_positive() != high.is_sign_positive() {
            for _ in 0..48 {
                let mid = 0.5 * (lo + hi);
                if eval(mid).is_sign_positive() == low.is_sign_positive() {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            found[count] = Some(0.5 * (lo + hi));
            count += 1;
        }
    }
    found.into_iter().flatten()
}

#[derive(Debug, Clone, Copy)]
struct Bounds {
    min: DVec3,
    max: DVec3,
}

impl Bounds {
    fn of(points: impl IntoIterator<Item = DVec3>) -> Self {
        points.into_iter().fold(
            Self {
                min: DVec3::splat(f64::INFINITY),
                max: DVec3::splat(f64::NEG_INFINITY),
            },
            |bounds, point| Self {
                min: bounds.min.min(point),
                max: bounds.max.max(point),
            },
        )
    }

    fn overlaps(self, other: Self) -> bool {
        let pad = DVec3::splat(CONTACT_GAP_M);
        (self.min - pad).cmple(other.max).all() && (self.max + pad).cmpge(other.min).all()
    }

    fn with_feature_tolerance(self) -> Self {
        let pad = (self.max - self.min) * (2.0 * EDGE_EPS);
        Self {
            min: self.min - pad,
            max: self.max + pad,
        }
    }
}

#[derive(Debug)]
struct Face {
    points: [DVec3; 3],
    normal: DVec3,
    bounds: Bounds,
    index: usize,
    edges: [bool; 3],
}

#[derive(Debug)]
struct BoundsTree {
    bounds: Bounds,
    contents: Contents,
}

#[derive(Debug)]
enum Contents {
    Leaf(Vec<Face>),
    Branch(Box<[BoundsTree; 2]>),
}

impl BoundsTree {
    fn build(mut faces: Vec<Face>) -> Self {
        let bounds = Bounds::of(
            faces
                .iter()
                .flat_map(|face| [face.bounds.min, face.bounds.max]),
        );
        let contents = if faces.len() <= 8 {
            Contents::Leaf(faces)
        } else {
            let span = bounds.max - bounds.min;
            let axis = if span.x >= span.y.max(span.z) {
                0
            } else if span.y >= span.z {
                1
            } else {
                2
            };
            let pivot = faces.len() / 2;
            let center = |face: &Face| 0.5 * face.bounds.min[axis] + 0.5 * face.bounds.max[axis];
            faces.select_nth_unstable_by(pivot, |a, b| {
                center(a).total_cmp(&center(b)).then(a.index.cmp(&b.index))
            });
            let right = faces.split_off(pivot);
            Contents::Branch(Box::new([Self::build(faces), Self::build(right)]))
        };
        Self { bounds, contents }
    }

    fn visit(&self, bounds: Bounds, f: &mut impl FnMut(&Face)) {
        if !self.bounds.overlaps(bounds) {
            return;
        }
        match &self.contents {
            Contents::Leaf(faces) => {
                for face in faces.iter().filter(|face| face.bounds.overlaps(bounds)) {
                    f(face);
                }
            }
            Contents::Branch(children) => {
                for child in children.iter() {
                    child.visit(bounds, f);
                }
            }
        }
    }

    fn has_overlap(&self, bounds: Bounds) -> bool {
        self.bounds.overlaps(bounds)
            && match &self.contents {
                Contents::Leaf(faces) => faces.iter().any(|face| face.bounds.overlaps(bounds)),
                Contents::Branch(children) => {
                    children.iter().any(|child| child.has_overlap(bounds))
                }
            }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn swept_bounds_reject_the_empty_space_between_distant_obstacles() {
        let mut vertices = Vec::new();
        let mut triangles = Vec::new();
        for x in [-12.0, -11.0, -10.0, -9.0, -8.0, 8.0, 9.0, 10.0, 11.0, 12.0] {
            let first = vertices.len();
            vertices.extend([
                DVec3::new(x, 0.0, 0.0),
                DVec3::new(x, 2.0, 0.0),
                DVec3::new(x, 0.0, 3.0),
            ]);
            triangles.push([first, first + 1, first + 2]);
        }
        let mesh = StaticCollisionMesh::new(&vertices, &triangles).unwrap();
        assert!(!mesh.overlaps_bounds(DVec3::new(-1.0, 0.1, 0.1), DVec3::new(1.0, 0.4, 1.0)));
        assert!(mesh.overlaps_bounds(DVec3::new(7.9, 0.1, 0.1), DVec3::new(8.1, 0.4, 1.0)));
    }
}

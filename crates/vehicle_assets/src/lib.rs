//! In-memory geometry and supported physics properties shared by asset importers and their consumers.
//!
//! Positions and dimensions are in metres. The asset frame has +Y up,
//! +Z forward and +X to vehicle left. Converters resolve source coordinates
//! and references before handing these types to an application.

mod material;
pub mod ror;
mod validation;

use bevy_math::{Quat, Vec2, Vec3};

pub use material::{
    DiffuseMaterial, DiffuseTexture, MaterialId, MaterialSection, MeshAppearance, TextureId,
};
pub use validation::{AssetValidationError, ValidationErrorKind};

/// Index into the owning asset's mesh library.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct MeshId(pub usize);

/// Index into the owning vehicle's structural nodes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct NodeId(pub usize);

#[derive(Clone, Debug)]
pub struct TerrainAsset {
    pub name: String,
    pub heightfield: Option<Heightfield>,
    pub meshes: Vec<TriangleMesh>,
    pub objects: Vec<MeshInstance>,
}

/// Grid with X changing fastest and successive rows along +Z.
///
/// Heights are relative to the origin's Y coordinate. Each axis has at
/// least two samples, and N samples span N - 1 intervals.
#[derive(Clone, Debug)]
pub struct Heightfield {
    pub origin: Vec3,
    pub samples_x: usize,
    pub samples_z: usize,
    pub spacing_x: f32,
    pub spacing_z: f32,
    pub heights: Vec<f32>,
}

impl Heightfield {
    /// Returns the sample's position in terrain space.
    ///
    /// Returns None for an out-of-bounds coordinate or missing sample.
    pub fn sample_position(&self, x: usize, z: usize) -> Option<Vec3> {
        if x >= self.samples_x || z >= self.samples_z {
            return None;
        }

        let index = z.checked_mul(self.samples_x)?.checked_add(x)?;
        let height = *self.heights.get(index)?;
        Some(self.origin + Vec3::new(x as f32 * self.spacing_x, height, z as f32 * self.spacing_z))
    }
}

/// Mesh-local geometry. Triangles are counterclockwise when seen from the
/// front face. Optional attributes follow the vertex order.
#[derive(Clone, Debug)]
pub struct TriangleMesh {
    pub vertices: Vec<Vec3>,
    pub triangles: Vec<[u32; 3]>,
    pub normals: Option<Vec<Vec3>>,
    pub uvs: Option<Vec<Vec2>>,
}

/// A placement shared by the visual and collision mesh roles.
///
/// Scale acts on local axes before rotation and translation. Negative scale
/// components reflect those axes, so an odd number reverses triangle winding.
#[derive(Clone, Debug)]
pub struct MeshInstance {
    pub name: String,
    pub translation: Vec3,
    pub rotation: Quat,
    pub scale: Vec3,
    pub visual_mesh: Option<MeshId>,
    pub collision_mesh: Option<MeshId>,
}

impl MeshInstance {
    /// Transforms a mesh-local point into terrain or vehicle space.
    pub fn transform_point(&self, point: Vec3) -> Vec3 {
        self.translation + self.rotation * (self.scale * point)
    }
}

#[derive(Clone, Debug)]
pub struct VehicleAsset {
    pub name: String,
    pub meshes: Vec<TriangleMesh>,
    pub objects: Vec<MeshInstance>,
    pub nodes: Vec<Vec3>,
    /// Authored structural edges for showing the vehicle outline.
    pub edges: Vec<[NodeId; 2]>,
    pub wheels: Vec<WheelGeometry>,
    pub physics: VehiclePhysics,
}

/// Vehicle-local wheel geometry with a unit axle direction toward vehicle left.
#[derive(Clone, Debug)]
pub struct WheelGeometry {
    pub centre: Vec3,
    pub axle: Vec3,
    pub outer_radius: f32,
    pub width: f32,
    pub rim_radius: Option<f32>,
}

#[derive(Clone, Debug, Default)]
pub struct VehiclePhysics {
    /// Rigid approximation derived from the authored structure and body geometry.
    pub chassis: Option<ChassisPhysics>,
    /// In the same order as `VehicleAsset::wheels` when supplied by the importer.
    pub wheels: Vec<WheelPhysics>,
    pub structure: Option<VehicleStructure>,
}

/// Structural properties of the vehicle in its authored rest pose.
#[derive(Clone, Debug, Default)]
pub struct VehicleStructure {
    /// One entry per `VehicleAsset::nodes` position, in the same order.
    pub nodes: Vec<StructuralNode>,
    /// One entry per `VehicleAsset::edges` pair, in the same order.
    /// A `None` slot preserves the edge geometry when its physics is unavailable.
    pub beams: Vec<Option<StructuralBeam>>,
    /// Collision panels indexed by their nodes, in authored winding order.
    pub collision_triangles: Vec<[NodeId; 3]>,
    /// Explicitly authored nodes for contact with other vehicles' panels.
    pub contact_nodes: Vec<NodeId>,
    pub deformable_meshes: Vec<DeformableMeshBinding>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct StructuralNode {
    /// Per-node mass, including any mass allocated to an upright.
    pub mass_kg: Option<f32>,
    /// Index into `VehicleAsset::wheels`. A node with no upright belongs to the chassis.
    pub upright: Option<usize>,
    pub excludes_ground_contact: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StructuralBeam {
    pub kind: StructuralBeamKind,
    pub stiffness_n_per_m: f32,
    pub damping_ns_per_m: f32,
    pub deformation_threshold_n: f32,
    pub breaking_threshold_n: f32,
    /// Dimensionless coefficient from the source material, in 0..=1.
    pub plasticity: f32,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum StructuralBeamKind {
    Bilateral,
    /// A support whose elastic force opposes compression.
    CompressionSupport {
        /// Authored limit on elongation as a multiple of the rest length.
        extension_limit_ratio: Option<f32>,
    },
}

/// Node references for one deformable instance of a visual mesh.
#[derive(Clone, Debug)]
pub struct DeformableMeshBinding {
    /// Index into `VehicleAsset::objects`.
    pub object: usize,
    /// Origin, local X reference, and local plane reference, in that order.
    pub placement_nodes: [NodeId; 3],
    pub influence_nodes: Vec<NodeId>,
}

#[derive(Clone, Copy, Debug)]
pub struct ChassisPhysics {
    pub mass_kg: f32,
    pub size_m: Vec3,
    pub centre_of_mass_m: Vec3,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WheelSide {
    Left,
    Right,
}

#[derive(Clone, Copy, Debug)]
pub struct WheelPhysics {
    pub side: WheelSide,
    /// Combined mass of the rim and tyre, in kg.
    pub mass_kg: Option<f32>,
    /// Mass assigned to the upright, excluded from chassis and wheel mass.
    pub upright_mass_kg: Option<f32>,
    pub suspension: Option<VerticalSuspension>,
    /// Signed first-gear torque under full throttle, in N·m. Zero for a free wheel.
    pub drive_torque_nm: Option<f32>,
    /// Full service-brake capacity in N·m. Zero for an unbraked wheel.
    pub brake_torque_nm: Option<f32>,
    /// Indices into `VehicleAsset::objects`.
    pub rim_object: usize,
    pub tyre_object: usize,
}

/// Linearized suspension along vehicle-local Y. Length increases as the wheel drops.
#[derive(Clone, Copy, Debug)]
pub struct VerticalSuspension {
    pub stiffness_n_per_m: f32,
    pub damping_ns_per_m: f32,
    pub initial_length_m: f32,
    pub free_length_m: f32,
    pub min_length_m: f32,
    pub max_length_m: f32,
}

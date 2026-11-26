use std::fmt;

use crate::{
    Heightfield, MeshInstance, TerrainAsset, TriangleMesh, VehicleAsset, VehicleStructure,
    WheelGeometry,
};

/// The first invalid field found, with asset-local indices in the path.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AssetValidationError {
    pub field: String,
    pub kind: ValidationErrorKind,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ValidationErrorKind {
    NonFinite,
    NonPositive,
    OutOfRange,
    ZeroScale,
    NotUnitLength,
    TooFewSamples,
    SampleCountOverflow,
    CountMismatch { expected: usize, actual: usize },
    IndexOutOfBounds { index: usize, len: usize },
    MissingMeshRole,
    RimLargerThanWheel,
}

impl fmt::Display for AssetValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.field, self.kind)
    }
}

impl std::error::Error for AssetValidationError {}

impl fmt::Display for ValidationErrorKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFinite => f.write_str("must be finite"),
            Self::NonPositive => f.write_str("must be positive"),
            Self::OutOfRange => f.write_str("value is out of range"),
            Self::ZeroScale => f.write_str("scale components must be nonzero"),
            Self::NotUnitLength => f.write_str("must be unit length"),
            Self::TooFewSamples => f.write_str("must contain at least two samples along this axis"),
            Self::SampleCountOverflow => f.write_str("grid sample count overflows usize"),
            Self::CountMismatch { expected, actual } => {
                write!(f, "expected {expected} elements, found {actual}")
            }
            Self::IndexOutOfBounds { index, len } => {
                write!(f, "index {index} is out of bounds for {len} elements")
            }
            Self::MissingMeshRole => f.write_str("requires a visual or collision mesh"),
            Self::RimLargerThanWheel => f.write_str("rim radius exceeds the outer radius"),
        }
    }
}

use ValidationErrorKind as Kind;

type Validation = Result<(), AssetValidationError>;

impl TerrainAsset {
    /// Checks geometry, sample counts and asset-local references.
    pub fn validate(&self) -> Result<(), AssetValidationError> {
        if let Some(heightfield) = &self.heightfield {
            validate_heightfield(heightfield)?;
        }
        validate_objects(&self.meshes, &self.objects)
    }
}

impl VehicleAsset {
    /// Checks geometry, wheel dimensions, structural properties and asset-local references.
    pub fn validate(&self) -> Result<(), AssetValidationError> {
        validate_objects(&self.meshes, &self.objects)?;

        for (index, node) in self.nodes.iter().enumerate() {
            if !node.is_finite() {
                return invalid(format!("nodes[{index}]"), Kind::NonFinite);
            }
        }
        for (edge, nodes) in self.edges.iter().enumerate() {
            for (endpoint, node) in nodes.iter().enumerate() {
                check_index(node.0, self.nodes.len(), || {
                    format!("edges[{edge}][{endpoint}]")
                })?;
            }
        }
        for (index, wheel) in self.wheels.iter().enumerate() {
            validate_wheel(wheel, index)?;
        }
        if let Some(structure) = &self.physics.structure {
            validate_structure(self, structure)?;
        }
        Ok(())
    }
}

fn validate_structure(vehicle: &VehicleAsset, structure: &VehicleStructure) -> Validation {
    check_count(structure.nodes.len(), vehicle.nodes.len(), || {
        "physics.structure.nodes".into()
    })?;
    check_count(structure.beams.len(), vehicle.edges.len(), || {
        "physics.structure.beams".into()
    })?;
    for (index, node) in structure.nodes.iter().enumerate() {
        if let Some(mass) = node.mass_kg {
            check_range(mass, 0.0..=f32::MAX, || {
                format!("physics.structure.nodes[{index}].mass_kg")
            })?;
        }
        if let Some(owner) = node.upright {
            check_index(owner, vehicle.wheels.len(), || {
                format!("physics.structure.nodes[{index}].upright")
            })?;
        }
    }
    for (index, beam) in structure.beams.iter().enumerate() {
        let Some(beam) = beam else { continue };
        for (field, value) in [
            ("stiffness_n_per_m", beam.stiffness_n_per_m),
            ("damping_ns_per_m", beam.damping_ns_per_m),
            ("deformation_threshold_n", beam.deformation_threshold_n),
            ("breaking_threshold_n", beam.breaking_threshold_n),
        ] {
            check_range(value, 0.0..=f32::MAX, || {
                format!("physics.structure.beams[{index}].{field}")
            })?;
        }
        check_range(beam.plasticity, 0.0..=1.0, || {
            format!("physics.structure.beams[{index}].plasticity")
        })?;
        if let crate::StructuralBeamKind::CompressionSupport {
            extension_limit_ratio: Some(limit),
        } = beam.kind
        {
            check_dimension(limit, || {
                format!("physics.structure.beams[{index}].kind.extension_limit_ratio")
            })?;
        }
    }
    for (index, triangle) in structure.collision_triangles.iter().enumerate() {
        for (corner, node) in triangle.iter().enumerate() {
            check_index(node.0, vehicle.nodes.len(), || {
                format!("physics.structure.collision_triangles[{index}][{corner}]")
            })?;
        }
    }
    for (index, node) in structure.contact_nodes.iter().enumerate() {
        check_index(node.0, vehicle.nodes.len(), || {
            format!("physics.structure.contact_nodes[{index}]")
        })?;
    }
    for (index, binding) in structure.deformable_meshes.iter().enumerate() {
        let field = || format!("physics.structure.deformable_meshes[{index}]");
        check_index(binding.object, vehicle.objects.len(), || {
            format!("{}.object", field())
        })?;
        if vehicle.objects[binding.object].visual_mesh.is_none() {
            return invalid(format!("{}.object", field()), Kind::MissingMeshRole);
        }
        for (role, nodes) in [
            ("placement_nodes", binding.placement_nodes.as_slice()),
            ("influence_nodes", binding.influence_nodes.as_slice()),
        ] {
            for (offset, node) in nodes.iter().enumerate() {
                check_index(node.0, vehicle.nodes.len(), || {
                    format!("{}.{role}[{offset}]", field())
                })?;
            }
        }
    }
    Ok(())
}

fn check_range(
    value: f32,
    range: std::ops::RangeInclusive<f32>,
    field: impl FnOnce() -> String,
) -> Validation {
    if !value.is_finite() {
        invalid(field(), Kind::NonFinite)
    } else if !range.contains(&value) {
        invalid(field(), Kind::OutOfRange)
    } else {
        Ok(())
    }
}

fn validate_heightfield(grid: &Heightfield) -> Validation {
    if !grid.origin.is_finite() {
        return invalid("heightfield.origin", Kind::NonFinite);
    }
    for (count, field) in [
        (grid.samples_x, "heightfield.samples_x"),
        (grid.samples_z, "heightfield.samples_z"),
    ] {
        if count < 2 {
            return invalid(field, Kind::TooFewSamples);
        }
    }
    check_dimension(grid.spacing_x, || "heightfield.spacing_x".into())?;
    check_dimension(grid.spacing_z, || "heightfield.spacing_z".into())?;

    let Some(expected) = grid.samples_x.checked_mul(grid.samples_z) else {
        return invalid("heightfield.heights", Kind::SampleCountOverflow);
    };
    check_count(grid.heights.len(), expected, || {
        "heightfield.heights".into()
    })?;

    if !(grid.origin.x + (grid.samples_x - 1) as f32 * grid.spacing_x).is_finite() {
        return invalid("heightfield.spacing_x", Kind::NonFinite);
    }
    if !(grid.origin.z + (grid.samples_z - 1) as f32 * grid.spacing_z).is_finite() {
        return invalid("heightfield.spacing_z", Kind::NonFinite);
    }
    for (index, height) in grid.heights.iter().enumerate() {
        if !(grid.origin.y + height).is_finite() {
            return invalid(format!("heightfield.heights[{index}]"), Kind::NonFinite);
        }
    }
    Ok(())
}

fn validate_objects(meshes: &[TriangleMesh], objects: &[MeshInstance]) -> Validation {
    for (index, mesh) in meshes.iter().enumerate() {
        validate_mesh(mesh, index)?;
    }
    for (index, object) in objects.iter().enumerate() {
        if !object.translation.is_finite() {
            return invalid(format!("objects[{index}].translation"), Kind::NonFinite);
        }
        check_unit_length(object.rotation.length_squared(), || {
            format!("objects[{index}].rotation")
        })?;
        if !object.scale.is_finite() {
            return invalid(format!("objects[{index}].scale"), Kind::NonFinite);
        }
        if object.scale.to_array().contains(&0.0) {
            return invalid(format!("objects[{index}].scale"), Kind::ZeroScale);
        }

        if object.visual_mesh.is_none() && object.collision_mesh.is_none() {
            return invalid(format!("objects[{index}]"), Kind::MissingMeshRole);
        }
        for (role, mesh) in [
            ("visual_mesh", object.visual_mesh),
            ("collision_mesh", object.collision_mesh),
        ] {
            if let Some(mesh) = mesh {
                check_index(mesh.0, meshes.len(), || format!("objects[{index}].{role}"))?;
            }
        }
    }
    Ok(())
}

fn validate_mesh(mesh: &TriangleMesh, index: usize) -> Validation {
    for (vertex, position) in mesh.vertices.iter().enumerate() {
        if !position.is_finite() {
            return invalid(
                format!("meshes[{index}].vertices[{vertex}]"),
                Kind::NonFinite,
            );
        }
    }
    for (triangle, vertices) in mesh.triangles.iter().enumerate() {
        for (corner, vertex) in vertices.iter().enumerate() {
            check_index(*vertex as usize, mesh.vertices.len(), || {
                format!("meshes[{index}].triangles[{triangle}][{corner}]")
            })?;
        }
    }
    if let Some(normals) = &mesh.normals {
        check_count(normals.len(), mesh.vertices.len(), || {
            format!("meshes[{index}].normals")
        })?;
        for (vertex, normal) in normals.iter().enumerate() {
            if !normal.is_finite() {
                return invalid(
                    format!("meshes[{index}].normals[{vertex}]"),
                    Kind::NonFinite,
                );
            }
        }
    }
    if let Some(uvs) = &mesh.uvs {
        check_count(uvs.len(), mesh.vertices.len(), || {
            format!("meshes[{index}].uvs")
        })?;
        for (vertex, uv) in uvs.iter().enumerate() {
            if !uv.is_finite() {
                return invalid(format!("meshes[{index}].uvs[{vertex}]"), Kind::NonFinite);
            }
        }
    }
    Ok(())
}

fn validate_wheel(wheel: &WheelGeometry, index: usize) -> Validation {
    if !wheel.centre.is_finite() {
        return invalid(format!("wheels[{index}].centre"), Kind::NonFinite);
    }
    check_unit_length(wheel.axle.length_squared(), || {
        format!("wheels[{index}].axle")
    })?;
    check_dimension(wheel.outer_radius, || {
        format!("wheels[{index}].outer_radius")
    })?;
    check_dimension(wheel.width, || format!("wheels[{index}].width"))?;
    if let Some(rim_radius) = wheel.rim_radius {
        check_dimension(rim_radius, || format!("wheels[{index}].rim_radius"))?;
        if rim_radius > wheel.outer_radius {
            return invalid(
                format!("wheels[{index}].rim_radius"),
                Kind::RimLargerThanWheel,
            );
        }
    }
    Ok(())
}

fn check_index(index: usize, len: usize, field: impl FnOnce() -> String) -> Validation {
    if index >= len {
        return invalid(field(), Kind::IndexOutOfBounds { index, len });
    }
    Ok(())
}

fn check_count(actual: usize, expected: usize, field: impl FnOnce() -> String) -> Validation {
    if actual != expected {
        return invalid(field(), Kind::CountMismatch { expected, actual });
    }
    Ok(())
}

fn check_dimension(value: f32, field: impl FnOnce() -> String) -> Validation {
    if !value.is_finite() {
        return invalid(field(), Kind::NonFinite);
    }
    if value <= 0.0 {
        return invalid(field(), Kind::NonPositive);
    }
    Ok(())
}

fn check_unit_length(length_squared: f32, field: impl FnOnce() -> String) -> Validation {
    if !length_squared.is_finite() {
        return invalid(field(), Kind::NonFinite);
    }
    if (length_squared - 1.0).abs() > 1e-4 {
        return invalid(field(), Kind::NotUnitLength);
    }
    Ok(())
}

fn invalid(field: impl Into<String>, kind: ValidationErrorKind) -> Validation {
    Err(AssetValidationError {
        field: field.into(),
        kind,
    })
}

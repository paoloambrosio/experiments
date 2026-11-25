//! Read the base static geometry of legacy OGRE binary meshes.

use std::{
    collections::{BTreeMap, HashMap},
    ops::Range,
};

use bevy_math::{Vec2, Vec3};

use super::archive::{Archive, Import};
use crate::{MeshId, TriangleMesh};

#[derive(Default)]
pub(super) struct MeshCache {
    pub geometry: Vec<TriangleMesh>,
    pub bindings: Vec<(String, Vec<(Range<usize>, String)>)>,
    loaded: HashMap<String, MeshId>,
}

impl MeshCache {
    pub fn load(&mut self, archive: &mut Archive, from: &str, reference: &str) -> Import<MeshId> {
        let path = archive.reference(from, reference)?;
        if let Some(id) = self.loaded.get(&path) {
            return Ok(*id);
        }
        let mesh = read(&archive.read(&path)?)
            .map_err(|error| archive.error(&path, format!("referenced by {from}: {error}")))?;
        let id = self.push(&path, mesh);
        self.loaded.insert(path, id);
        Ok(id)
    }

    pub fn push(&mut self, entry: &str, mesh: Mesh) -> MeshId {
        let id = MeshId(self.geometry.len());
        self.geometry.push(mesh.geometry);
        self.bindings.push((entry.to_owned(), mesh.sections));
        id
    }
}

pub(super) struct Mesh {
    pub geometry: TriangleMesh,
    pub sections: Vec<(Range<usize>, String)>,
}

type Parse<T> = Result<T, String>;

const MESH: u16 = 0x3000;
const SUBMESH: u16 = 0x4000;
const OPERATION: u16 = 0x4010;
const GEOMETRY: u16 = 0x5000;
const DECLARATION: u16 = 0x5100;
const ELEMENT: u16 = 0x5110;
const BUFFER: u16 = 0x5200;
const BUFFER_DATA: u16 = 0x5210;
const BOUNDS: u16 = 0x9000;

pub(super) fn read(bytes: &[u8]) -> Parse<Mesh> {
    let mut input = Reader(bytes);
    if input.u16()? != 0x1000 {
        return Err("expected little-endian OGRE mesh header".into());
    }
    let version = input.line()?;
    if !matches!(
        version,
        "[MeshSerializer_v1.30]" | "[MeshSerializer_v1.40]" | "[MeshSerializer_v1.41]"
    ) {
        return Err(format!("unsupported OGRE mesh serializer {version:?}"));
    }
    // Some legacy exporters miscount the sizes of parent chunks. Follow the
    // known records and bound each index and vertex buffer by its actual size.
    input.expect_chunk(MESH)?;
    if input.boolean()? {
        return Err("animated OGRE mesh is outside the static geometry subset".into());
    }

    let shared = if input.at(GEOMETRY) {
        Some(read_vertices(&mut input)?)
    } else {
        None
    };

    let mut geometry = TriangleMesh {
        vertices: Vec::new(),
        triangles: Vec::new(),
        normals: Some(Vec::new()),
        uvs: Some(Vec::new()),
    };
    let mut sections = Vec::new();
    let mut shared_offset = None;
    while input.at(SUBMESH) {
        input.expect_chunk(SUBMESH)?;
        let material = input.line()?.to_owned();
        let uses_shared = input.boolean()?;
        let index_count = input.u32()? as usize;
        if index_count == 0 || !index_count.is_multiple_of(3) {
            return Err("submesh index count must describe complete triangles".into());
        }
        let index_width = if input.boolean()? { 4 } else { 2 };
        let indices = input.take_array(index_count, index_width)?;
        let local;
        let vertices = if uses_shared {
            shared
                .as_ref()
                .ok_or("submesh refers to missing shared geometry")?
        } else {
            local = read_vertices(&mut input)?;
            &local
        };
        let offset = if uses_shared {
            if let Some(offset) = shared_offset {
                offset
            } else {
                let offset = append_vertices(&mut geometry, vertices)?;
                shared_offset = Some(offset);
                offset
            }
        } else {
            append_vertices(&mut geometry, vertices)?
        };

        if input.at(OPERATION) {
            input.expect_chunk(OPERATION)?;
            if input.u16()? != 4 {
                return Err("OGRE submesh operation is outside the triangle-list subset".into());
            }
        }

        let start = geometry.triangles.len();
        let mut indices = Reader(indices);
        for _ in 0..index_count / 3 {
            let mut triangle = [0; 3];
            for vertex in &mut triangle {
                let index = if index_width == 4 {
                    indices.u32()?
                } else {
                    u32::from(indices.u16()?)
                };
                if index as usize >= vertices.positions.len() {
                    return Err(format!("submesh index {index} exceeds its vertex count"));
                }
                *vertex = offset.checked_add(index).ok_or("mesh index overflow")?;
            }
            geometry.triangles.push(triangle);
        }
        sections.push((start..geometry.triangles.len(), material));
    }
    if sections.is_empty() {
        return Err("OGRE mesh has no triangle submeshes".into());
    }

    // Bounds follow base geometry. Edge lists and names after bounds are
    // renderer metadata. The asset computes its own bounds from vertices.
    if !input.0.is_empty() {
        input.expect_chunk(BOUNDS)?;
        input.take(28)?;
    }
    Ok(Mesh { geometry, sections })
}

fn append_vertices(mesh: &mut TriangleMesh, vertices: &Vertices) -> Parse<u32> {
    let offset = u32::try_from(mesh.vertices.len()).map_err(|_| "mesh vertex count exceeds u32")?;
    mesh.vertices.extend_from_slice(&vertices.positions);
    if let (Some(target), Some(normals)) = (&mut mesh.normals, &vertices.normals) {
        target.extend_from_slice(normals);
    } else {
        mesh.normals = None;
    }
    if let Some(uvs) = &mut mesh.uvs {
        if let Some(source) = &vertices.uvs {
            uvs.extend_from_slice(source);
        } else {
            uvs.resize(mesh.vertices.len(), Vec2::ZERO);
        }
    }
    Ok(offset)
}

struct Vertices {
    positions: Vec<Vec3>,
    normals: Option<Vec<Vec3>>,
    uvs: Option<Vec<Vec2>>,
}

fn read_vertices(input: &mut Reader<'_>) -> Parse<Vertices> {
    input.expect_chunk(GEOMETRY)?;
    let count = input.u32()? as usize;
    input.expect_chunk(DECLARATION)?;
    let mut elements = BTreeMap::new();
    while input.at(ELEMENT) {
        input.expect_chunk(ELEMENT)?;
        let source = input.u16()?;
        let format = input.u16()?;
        let semantic = input.u16()?;
        let offset = input.u16()? as usize;
        let index = input.u16()?;
        if index == 0
            && matches!(semantic, 1 | 4 | 7)
            && elements
                .insert(semantic, (source, format, offset))
                .is_some()
        {
            return Err("duplicate OGRE vertex attribute".into());
        }
    }

    let mut buffers = BTreeMap::new();
    while input.at(BUFFER) {
        input.expect_chunk(BUFFER)?;
        let source = input.u16()?;
        let stride = input.u16()? as usize;
        input.expect_chunk(BUFFER_DATA)?;
        let data = input.take_array(count, stride)?;
        if stride == 0 || buffers.insert(source, (stride, data)).is_some() {
            return Err("invalid OGRE vertex buffer binding".into());
        }
    }

    let attribute = |semantic, width| -> Parse<Option<_>> {
        let Some(&(source, format, offset)) = elements.get(&semantic) else {
            return Ok(None);
        };
        let &(stride, data) = buffers
            .get(&source)
            .ok_or("missing OGRE vertex buffer binding")?;
        if (format as usize + 1) * 4 != width || offset + width > stride {
            return Err(format!(
                "unsupported OGRE vertex layout for attribute {semantic}"
            ));
        }
        Ok(Some(data.chunks_exact(stride).map(move |vertex| {
            Reader(&vertex[offset..offset + width])
        })))
    };
    let positions = attribute(1, 12)?
        .ok_or("OGRE mesh has no positions")?
        .map(|mut r| r.vec3())
        .collect::<Parse<_>>()?;
    let normals = attribute(4, 12)?
        .map(|rows| rows.map(|mut r| r.vec3()).collect::<Parse<_>>())
        .transpose()?;
    let uvs = attribute(7, 8)?
        .map(|rows| {
            rows.map(|mut r| Ok(Vec2::new(r.float()?, r.float()?)))
                .collect::<Parse<_>>()
        })
        .transpose()?;
    Ok(Vertices {
        positions,
        normals,
        uvs,
    })
}

struct Reader<'a>(&'a [u8]);

impl<'a> Reader<'a> {
    fn take(&mut self, count: usize) -> Parse<&'a [u8]> {
        let (bytes, remaining) = self
            .0
            .split_at_checked(count)
            .ok_or("truncated OGRE mesh data")?;
        self.0 = remaining;
        Ok(bytes)
    }

    fn take_array(&mut self, count: usize, stride: usize) -> Parse<&'a [u8]> {
        self.take(
            count
                .checked_mul(stride)
                .ok_or("OGRE buffer size overflow")?,
        )
    }

    fn u16(&mut self) -> Parse<u16> {
        Ok(u16::from_le_bytes(self.take(2)?.try_into().unwrap()))
    }

    fn u32(&mut self) -> Parse<u32> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }

    fn float(&mut self) -> Parse<f32> {
        let value = f32::from_bits(self.u32()?);
        if !value.is_finite() {
            return Err("non-finite OGRE vertex attribute".into());
        }
        Ok(value)
    }

    fn vec3(&mut self) -> Parse<Vec3> {
        Ok(Vec3::new(self.float()?, self.float()?, self.float()?))
    }

    fn boolean(&mut self) -> Parse<bool> {
        match self.take(1)?[0] {
            0 => Ok(false),
            1 => Ok(true),
            _ => Err("invalid OGRE boolean".into()),
        }
    }

    fn line(&mut self) -> Parse<&'a str> {
        let length = self
            .0
            .iter()
            .position(|byte| *byte == b'\n')
            .ok_or("unterminated OGRE string")?;
        let bytes = self.take(length + 1)?;
        std::str::from_utf8(&bytes[..length]).map_err(|error| error.to_string())
    }

    fn at(&self, chunk: u16) -> bool {
        self.0.starts_with(&chunk.to_le_bytes())
    }

    fn expect_chunk(&mut self, chunk: u16) -> Parse<()> {
        let actual = self.u16()?;
        if actual != chunk || self.u32()? < 6 {
            return Err(format!(
                "expected OGRE chunk {chunk:04x}, found {actual:04x}"
            ));
        }
        Ok(())
    }
}

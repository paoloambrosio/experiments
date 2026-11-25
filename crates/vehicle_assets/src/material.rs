use std::{collections::HashMap, ops::Range};

use crate::MeshId;

/// Appearance for an asset's indexed triangle meshes.
#[derive(Clone, Debug, Default)]
pub struct MeshAppearance {
    pub sections: HashMap<MeshId, Vec<MaterialSection>>,
    pub materials: Vec<DiffuseMaterial>,
    pub textures: Vec<DiffuseTexture>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct MaterialId(pub usize);

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TextureId(pub usize);

/// A contiguous range in the owning mesh's triangle list.
#[derive(Clone, Debug)]
pub struct MaterialSection {
    pub triangles: Range<usize>,
    pub material: MaterialId,
}

#[derive(Clone, Debug)]
pub struct DiffuseMaterial {
    pub rgba: [f32; 4],
    pub base_texture: Option<TextureId>,
    pub double_sided: bool,
    /// Blend the diffuse image's alpha channel, for surfaces such as glass.
    pub transparent: bool,
}

/// Decoded sRGB image in top-to-bottom RGBA8 pixel order.
#[derive(Clone, Debug)]
pub struct DiffuseTexture {
    pub width: u32,
    pub height: u32,
    pub rgba8: Vec<u8>,
}

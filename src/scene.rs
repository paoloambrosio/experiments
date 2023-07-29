use bevy::prelude::{Mat4, Vec3, Vec2};

pub struct Scene {
  pub textures: Vec<Texture>,
  pub materials: Vec<Material>,
  pub root_node: Node
}

// bevy::render::texture::Image
pub struct Texture {
  pub type_id: i32,
  pub name: String,
  pub content: Vec<u8>
}

// bevy::pbr::StandardMaterial
pub struct Material {
  pub name: String,
  pub shader_name: String,
  pub shaders: Vec<Shader>,
  pub textures: Vec<MaterialTexture>
}

pub struct Shader {
  pub name: String,
  pub value: f32
}

pub struct MaterialTexture {
  pub sample_name: String,
  pub sample_slot: i32,
  pub texture_name: String
}

// bevy::gltf::GltfNode ???
// - bevy::render::mesh::Mesh
pub enum Node {
  DummyNode {
    name: String,
    transformation: Mat4,
    children: Vec<Node>
  },
  StaticMeshNode {
    name: String,
    vertices: Vec<Vertex>,
    indices: Vec<u16>,
    material_id: i32,
    children: Vec<Node>
  }
}

impl Node {

  pub fn flatten(&self) -> Vec<Node> {
    match self {
      Node::DummyNode { name, transformation, children } => {
        children.iter().flat_map(|n| n.flatten()).collect()
      }
      Node::StaticMeshNode { name, vertices, indices, material_id, children } => {
        //&vec![self; 1]
        unimplemented!()
      }
    }
  }
}

pub struct Vertex {
  pub position: Vec3,
  pub normal: Vec3,
  pub texture: Vec2,
  pub tangents: Vec3
}

use bevy::math::{Mat4, Vec2, Vec3};
use byteorder::{ReadBytesExt, LittleEndian};
use std::fs::File;
use std::io::{BufReader, Read, Seek, SeekFrom};

fn main() {
    let mut input = BufReader::new(
        File::open("abarth500_D.kn5").unwrap()
    );

    let scene = load_kn5_scene(&mut input).unwrap();
    print_scene(scene);
}

struct Scene {
  textures: Vec<Texture>,
  materials: Vec<Material>,
  root_node: Node
}

struct Texture {
  type_id: i32,
  name: String,
  content: Vec<u8>
}

struct Material {
  name: String,
  shader_name: String,
  shaders: Vec<Shader>,
  textures: Vec<MaterialTexture>
}

struct Shader {
  name: String,
  value: f32
}

struct MaterialTexture {
  sample_name: String,
  sample_slot: i32,
  texture_name: String
}

enum Node {
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

struct Vertex {
  position: Vec3,
  normal: Vec3,
  texture: Vec2,
  tangents: Vec3
}

type LoadError = ();

fn to_load_error<T>(_t: T) {}

fn load_kn5_scene<R: Read + Seek>(input: &mut R) -> Result<Scene, LoadError> {
  verify_bytes(input, "sc6969".as_bytes())?;
  let version = read_i32(input)?;
  println!("version: {}", version);
  if version > 5 {
    input.seek(SeekFrom::Current(4)).map_err(to_load_error)?;
  }
  let textures = load_kn5_textures(input)?;
  let materials = load_kn5_materials(input, version)?;
  let root_node = load_kn5_node(input)?;
  Ok(Scene { textures, materials, root_node })
}

fn load_kn5_textures<R: Read + Seek>(input: &mut R) -> Result<Vec<Texture>, LoadError> {
  let textures_len = read_i32(input)?;
  let mut textures = Vec::with_capacity(textures_len as usize);
  for _ in 0..textures_len {
      let type_id = read_i32(input)?;
      let name = read_string(input)?;
      let content = read_binary(input)?;
      textures.push(Texture{type_id, name, content});
  }
  Ok(textures)
}

fn load_kn5_materials<R: Read + Seek>(input: &mut R, version: i32) -> Result<Vec<Material>, LoadError> {
  let materials_len = read_i32(input)?;
  let mut materials = Vec::with_capacity(materials_len as usize);
  for _ in 0..materials_len {
      let name = read_string(input)?;
      let shader_name = read_string(input)?;
      input.seek(SeekFrom::Current(2)).map_err(to_load_error)?;
      if version > 4 {
        input.seek(SeekFrom::Current(4)).map_err(to_load_error)?;
      }
      let shaders_len = read_i32(input)?;
      let mut shaders = Vec::with_capacity(shaders_len as usize);
      for _ in 0..shaders_len {
          let name = read_string(input)?;
          let value = read_f32(input)?;
          input.seek(SeekFrom::Current(36)).map_err(to_load_error)?;
          shaders.push(Shader{name, value});
      }
      let textures_len = read_i32(input)?;
      let mut textures = Vec::with_capacity(textures_len as usize);
      for _ in 0..textures_len {
          let sample_name = read_string(input)?;
          let sample_slot = read_i32(input)?;
          let texture_name = read_string(input)?;
          textures.push(MaterialTexture{sample_name, sample_slot, texture_name});
      }
      materials.push(Material { name, shader_name, shaders, textures })
  }
  Ok(materials)
}

fn load_kn5_node<R: Read + Seek>(input: &mut R) -> Result<Node, LoadError> {
  let type_id = read_i32(input)?;
  let name = read_string(input)?;
  let children_len = read_i32(input)?;
  input.seek(SeekFrom::Current(1)).map_err(to_load_error)?;
  match type_id {
    1 => {
      let transformation = read_mat4(input)?;
      let mut children = Vec::with_capacity(children_len as usize);
      for _ in 0..children_len {
        children.push(load_kn5_node(input)?)
      }
      Ok(Node::DummyNode { name, transformation, children })
    }
    2 => {
        input.seek(SeekFrom::Current(3)).map_err(to_load_error)?;
        let vertices_len = read_i32(input)?;
        let mut vertices = Vec::with_capacity(vertices_len as usize);
        for _ in 0..vertices_len {
          let position = read_vec3(input)?;
          let normal = read_vec3(input)?;
          let texture = read_vec2(input)?;
          let tangents = read_vec3(input)?;
          vertices.push(Vertex{position, normal, texture, tangents});
        }
        let indices_len = read_i32(input)?;
        let mut indices = Vec::with_capacity(indices_len as usize);
        for _ in 0..indices_len {
          let index = read_u16(input)?;
          indices.push(index);
        }
        let material_id = read_i32(input)?;
        input.seek(SeekFrom::Current(29)).map_err(to_load_error)?;
        let mut children = Vec::with_capacity(children_len as usize);
        for _ in 0..children_len {
          children.push(load_kn5_node(input)?)
        }
        Ok(Node::StaticMeshNode { name, vertices, indices, material_id, children })
    }
    3 => {
      Err(())
    }
    _ => Err(())
  }
}



fn verify_bytes<R: Read>(input: &mut R, expected: &[u8]) -> Result<(), LoadError> {
    let mut buf = vec![0; expected.len()];
    input.read_exact(&mut buf).map_err(to_load_error)?;
    if buf != expected {
        Err(())
    } else {
      Ok(())
    }
}

fn read_i32<R: Read>(input: &mut R) -> Result<i32, LoadError> {
    input.read_i32::<LittleEndian>().map_err(to_load_error)
}

fn read_u16<R: Read>(input: &mut R) -> Result<u16, LoadError> {
  input.read_u16::<LittleEndian>().map_err(to_load_error)
}

fn read_f32<R: Read>(input: &mut R) -> Result<f32, LoadError> {
    input.read_f32::<LittleEndian>().map_err(to_load_error)
}

fn read_string<R: Read + Seek>(input: &mut R) -> Result<String, LoadError> {
    let size = read_i32(input)?;
    let mut buf = vec![0; size as usize];
    input.read_exact(&mut buf).map_err(to_load_error)?;
    String::from_utf8(buf).map_err(to_load_error)
}

fn read_binary<R: Read + Seek>(input: &mut R) -> Result<Vec<u8>, LoadError> {
    let size = read_i32(input)?;
    let mut buf = vec![0; size as usize];
    input.read_exact(&mut buf).map_err(to_load_error)?;
    Ok(buf)
}

fn read_mat4<R: Read>(input: &mut R) -> Result<Mat4, LoadError> {
  let mat = [
    [read_f32(input)?, read_f32(input)?, read_f32(input)?, read_f32(input)?],
    [read_f32(input)?, read_f32(input)?, read_f32(input)?, read_f32(input)?],
    [read_f32(input)?, read_f32(input)?, read_f32(input)?, read_f32(input)?],
    [read_f32(input)?, read_f32(input)?, read_f32(input)?, read_f32(input)?],
  ];
  Ok(Mat4::from_cols_array_2d(&mat))
}

fn read_vec2<R: Read>(input: &mut R) -> Result<Vec2, LoadError> {
  Ok(Vec2::new(read_f32(input)?, read_f32(input)?))
}

fn read_vec3<R: Read>(input: &mut R) -> Result<Vec3, LoadError> {
  Ok(Vec3::new(read_f32(input)?, read_f32(input)?, read_f32(input)?))
}



fn print_scene(scene: Scene) {
  println!("{} textures:", scene.textures.len());
  for t in scene.textures {
    println!("- {} {}", t.type_id, t.name)
  }

  println!("{} materials:", scene.materials.len());
  for m in scene.materials {
    println!("- {} {}", m.name, m.shader_name);
    println!("  {} shaders", m.shaders.len());
    for s in m.shaders {
      println!("  - {} {}", s.name, s.value);
    }
    for t in m.textures {
      println!("  - {} {} {}", t.sample_name, t.sample_slot, t.texture_name);
    }
  }

  println!("nodes:");
  print_node(scene.root_node, "");
}

fn print_node(node: Node, prefix: &str) {
  match node {
    Node::DummyNode{ name, transformation, children } => {
      println!("{}- dummy {}", prefix, name);
      let nested_prefix = format!("{}{}", "  ", prefix);
      for c in children {
        print_node(c, nested_prefix.as_str());
      }
    }
    Node::StaticMeshNode { name, vertices, indices, material_id, children } => {
      println!("{}- static mesh {}", prefix, name);
      println!("{}  {} vertices", prefix, vertices.len());
      println!("{}  {} indices", prefix, indices.len());
      println!("{}  material_id {}", prefix, material_id);
      let nested_prefix = format!("{}{}", "  ", prefix);
      for c in children {
        print_node(c, nested_prefix.as_str());
      }
    }
  }
}

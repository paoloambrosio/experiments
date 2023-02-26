use std::{io::{BufReader, Read, Seek, SeekFrom}, fs::File};

use bevy::prelude::{Mat4, Vec2, Vec3};
use byteorder::{LittleEndian, ReadBytesExt};

use crate::scene::*;

type LoadError = ();

fn to_load_error<T>(_t: T) {}

pub fn load_kn5_file(path: &str) -> Result<Scene, LoadError> {
  let mut input = BufReader::new(
    File::open(path).map_err(to_load_error)?
  );
  load_kn5_scene(&mut input)
}

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

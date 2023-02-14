use bevy::math::{Mat4, Vec2, Vec3};
use byteorder::{ReadBytesExt, LittleEndian};
use core::panic;
use std::fs::File;
use std::io::{BufReader, Read, Seek, SeekFrom};

fn main() {
    let mut input = BufReader::new(
        File::open("abarth500_D.kn5").unwrap()
    );
    
    // header
    verify_bytes(&mut input, "sc6969".as_bytes());
    let version = read_i32(&mut input);
    println!("version: {}", version);
    if version > 5 {
        input.seek_relative(4).unwrap()
    }

    print_textures(&mut input);
    print_materials(&mut input, version);
    print_nodes(&mut input);
}

fn print_textures<R: Read + Seek>(input: &mut R) {
  let textures_len = read_i32(input);
  println!("{} textures:", textures_len);
  for _ in 0..textures_len {
      let type_id = read_i32(input);
      let name = read_string(input);
      let _content = read_binary(input);
      println!("- {} {}", type_id, name)
  }
}

fn print_materials<R: Read + Seek>(input: &mut R, version: i32) {
  let materials_len = read_i32(input);
  println!("{} materials:", materials_len);
  for _ in 0..materials_len {
      let name = read_string(input);
      let shader = read_string(input);
      println!("- {} {}", name, shader);
      input.seek(SeekFrom::Current(2)).unwrap();
      if version > 4 {
        input.seek(SeekFrom::Current(4)).unwrap();
      }
      let shaders_len = read_i32(input);
      println!("  {} shaders", shaders_len);
      for _ in 0..shaders_len {
          let name = read_string(input);
          let value = read_f32(input);
          println!("  - {} {}", name, value);
          input.seek(SeekFrom::Current(36)).unwrap();
      }
      let textures_len = read_i32(input);
      println!("  {} textures", textures_len);
      for _ in 0..textures_len {
          let sample_name = read_string(input);
          let sample_slot = read_i32(input);
          let texture_name = read_string(input);
          println!("  - {} {} {}", sample_name, sample_slot, texture_name);
      }
  }
}

fn print_nodes<R: Read + Seek>(input: &mut R) {
  println!("nodes:");
  print_node(input);
}

fn print_node<R: Read + Seek>(input: &mut R) {
  let type_id = read_i32(input);
  let name = read_string(input);
  let children_len = read_i32(input);
  input.seek(SeekFrom::Current(1)).unwrap();
  match type_id {
      1 => {
          println!("  - dummy {}", name);
          let _transformation = read_mat4(input);
      }
      2 => {
          println!("  - mesh {}", name);
          input.seek(SeekFrom::Current(3)).unwrap();
          let vertices_len = read_i32(input);
          println!("  {} vertices", vertices_len);
          for _ in 0..vertices_len {
            let _position = read_vec3(input);
            let _normal = read_vec3(input);
            let _texture = read_vec2(input);
            let _tangents = read_vec3(input);
          }
          let indices_len = read_i32(input);
          println!("  {} indices", indices_len);
          for _ in 0..indices_len {
            let index = read_u16(input);
            println!("  - {}", index);
          }
          let material_id = read_i32(input);
          println!("  material {}", material_id);
          input.seek(SeekFrom::Current(29)).unwrap();
      }
      3 => {
          println!("  - animated mesh {}", name);
          unimplemented!();
      }
      _ => panic!("Unknown node type")
  }
  println!("{} children:", children_len);
  for _ in 0..children_len {
    print_node(input);
  }
}

fn verify_bytes<R: Read>(input: &mut R, expected: &[u8]) {
    let mut buf = vec![0; expected.len()];
    input.read_exact(&mut buf).unwrap();
    if buf != expected {
        panic!("Bytes read do not match expectation")
    }
}

fn read_i32<R: Read>(input: &mut R) -> i32 {
    input.read_i32::<LittleEndian>().unwrap()
}

fn read_u16<R: Read>(input: &mut R) -> u16 {
  input.read_u16::<LittleEndian>().unwrap()
}

fn read_f32<R: Read>(input: &mut R) -> f32 {
    input.read_f32::<LittleEndian>().unwrap()
}

fn read_string<R: Read + Seek>(input: &mut R) -> String {
    let size = read_i32(input);
    let mut buf = vec![0; size as usize];
    input.read_exact(&mut buf).unwrap();
    String::from_utf8(buf).unwrap()
}

fn read_binary<R: Read + Seek>(input: &mut R) -> Vec<u8> {
    let size = read_i32(input);
    let mut buf = vec![0; size as usize];
    input.read_exact(&mut buf).unwrap();
    buf
}

fn read_mat4<R: Read>(input: &mut R) -> Mat4 {
  let mat = [
    [read_f32(input), read_f32(input), read_f32(input), read_f32(input)],
    [read_f32(input), read_f32(input), read_f32(input), read_f32(input)],
    [read_f32(input), read_f32(input), read_f32(input), read_f32(input)],
    [read_f32(input), read_f32(input), read_f32(input), read_f32(input)],
  ];
  Mat4::from_cols_array_2d(&mat)
}

fn read_vec2<R: Read>(input: &mut R) -> Vec2 {
  Vec2::new(read_f32(input), read_f32(input))
}

fn read_vec3<R: Read>(input: &mut R) -> Vec3 {
  Vec3::new(read_f32(input), read_f32(input), read_f32(input))
}
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

    // textures
    let textures_len = read_i32(&mut input);
    println!("{} textures:", textures_len);
    for _ in 0..textures_len {
        let type_id = read_i32(&mut input);
        let name = read_string(&mut input);
        let _content = read_binary(&mut input);
        println!("- {} {}", type_id, name)
    }

    // materials
    let materials_len = read_i32(&mut input);
    println!("{} materials:", materials_len);
    for _ in 0..materials_len {
        let name = read_string(&mut input);
        let shader = read_string(&mut input);
        println!("- {} {}", name, shader);
        input.seek_relative(2).unwrap();
        if version > 4 {
            input.seek_relative(4).unwrap()
        }
        let shaders_len = read_i32(&mut input);
        println!("  {} shaders", shaders_len);
        for _ in 0..shaders_len {
            let name = read_string(&mut input);
            let value = read_f32(&mut input);
            println!("  - {} {}", name, value);
            input.seek_relative(36).unwrap();
        }
        let textures_len = read_i32(&mut input);
        println!("  {} textures", textures_len);
        for _ in 0..textures_len {
            let sample_name = read_string(&mut input);
            let sample_slot = read_i32(&mut input);
            let texture_name = read_string(&mut input);
            println!("  - {} {} {}", sample_name, sample_slot, texture_name);
        }
    }

    // nodes
    println!("nodes:");
    let type_id = read_i32(&mut input);
    let name = read_string(&mut input);
    let children_len = read_i32(&mut input);
    input.seek_relative(1).unwrap();
    match type_id {
        1 => {
            println!("  - dummy {}", name);
            input.seek_relative(4*4*4).unwrap() // TODO
        }
        2 => {
            println!("  - mesh {}", name);
            input.seek_relative(3).unwrap();
            let vertices_len = read_i32(&mut input);
            println!("  {} vertices", vertices_len);
            for _ in 0..vertices_len {
                input.seek_relative(4*3).unwrap(); // TODO
                input.seek_relative(4*3).unwrap(); // TODO
                input.seek_relative(4*2).unwrap(); // TODO
                input.seek_relative(4*3).unwrap(); // TODO
            }
        }
        3 => {
            println!("  - animated mesh {}", name);
            unimplemented!();
        }
        _ => panic!("Unknown node type")
    }
    println!("{} children:", children_len);
    for _ in 0..children_len {
        unimplemented!(); // TODO this needs to be a recursive call
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
    input.seek(SeekFrom::Current(size.into())).unwrap(); // TODO
    Vec::new()
}
use crate::scene::{Scene, Node};

pub fn print_scene(scene: Scene) {
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
    Node::DummyNode { name, transformation, children } => {
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

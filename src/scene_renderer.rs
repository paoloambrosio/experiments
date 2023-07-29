use crate::scene::{Scene, Node};
use bevy::prelude::*;
use bevy::render::mesh::{self, PrimitiveTopology};

pub fn to_mesh(scene: Scene) -> Mesh {
  node_mesh(scene.root_node, None)
}

fn node_mesh(node: Node, parent: Option<Mesh>) -> Mesh {
  let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);

  // Positions of the vertices
  // See https://bevy-cheatbook.github.io/features/coords.html
  mesh.insert_attribute(
      Mesh::ATTRIBUTE_POSITION,
      vec![[0., 0., 0.], [1., 2., 1.], [2., 0., 0.]],
  );

  // In this example, normals and UVs don't matter,
  // so we just use the same value for all of them
  mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0., 1., 0.]; 3]);
  mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, vec![[0., 0.]; 3]);

  // A triangle using vertices 0, 2, and 1.
  // Note: order matters. [0, 1, 2] will be flipped upside down, and you won't see it from behind!
  mesh.set_indices(Some(mesh::Indices::U32(vec![0, 2, 1])));

  mesh
}

// fn print_node_transform(node: Node, prefix: &str, base_transformation: Mat4) {
//   match node {
//     Node::DummyNode { name, transformation, children } => {
//       println!("{}- dummy {}", prefix, name);
      
//       let nested_prefix = format!("{}{}", "  ", prefix);
//       for c in children {
//         print_node_transform(c, nested_prefix.as_str(), base_transformation.mul_mat4(&transformation));
//       }
//     }
//     Node::StaticMeshNode { name, vertices, indices, material_id, children } => {
//       println!("{}- static mesh {}", prefix, name);
//       println!("{}  {} vertices", prefix, vertices.len());
//       println!("{}  {} indices", prefix, indices.len());
//       println!("{}  material_id {}", prefix, material_id);
//       let nested_prefix = format!("{}{}", "  ", prefix);
//       for c in children {
//         print_node(c, nested_prefix.as_str());
//       }
//     }
//   }
// }

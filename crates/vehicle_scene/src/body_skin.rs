//! Vertex deformation from imported placement and influence nodes.

use bevy::{
    math::{DMat3, DQuat, DVec3},
    prelude::*,
};
use vehicle_assets::{DeformableMeshBinding, MeshInstance, NodeId, VehicleAsset};
use vehicle_physics::vehicle_assembly::VehiclePose;

use crate::crash::SourceIndices;

#[derive(Clone, Copy)]
struct Frame {
    origin: DVec3,
    rotation: DQuat,
}

impl Frame {
    fn new([origin, x, plane]: [DVec3; 3]) -> Option<Self> {
        let x = (x - origin).try_normalize()?;
        let y = (plane - origin).cross(x).try_normalize()?;
        let rotation = DQuat::from_mat3(&DMat3::from_cols(x, y, x.cross(y))).normalize();
        Some(Self { origin, rotation })
    }

    fn to_local(self, point: DVec3) -> DVec3 {
        self.rotation.inverse() * (point - self.origin)
    }

    fn to_world(self, point: DVec3) -> DVec3 {
        self.origin + self.rotation * point
    }
}

struct Vertex {
    rest: DVec3,
    weights: Vec<(usize, f64)>,
}

pub struct MeshSkin {
    pub object: usize,
    instance: MeshInstance,
    placement: [usize; 3],
    placement_ids: [NodeId; 3],
    rest_frame: Frame,
    influences: Vec<(usize, DVec3)>,
    vertices: Vec<Vertex>,
}

impl MeshSkin {
    fn new(
        asset: &VehicleAsset,
        binding: &DeformableMeshBinding,
        source: &SourceIndices,
    ) -> Result<Self, String> {
        let instance = asset.objects.get(binding.object).ok_or_else(|| {
            format!(
                "deformable binding references missing object {}",
                binding.object
            )
        })?;
        let label = format!("{} (object {})", instance.name, binding.object);
        let mesh = instance
            .visual_mesh
            .and_then(|id| asset.meshes.get(id.0))
            .ok_or_else(|| format!("{label}: missing visual mesh"))?;
        let resolve = |id: NodeId| {
            let index = source.nodes.iter().position(|&node| node == id.0);
            index
                .zip(asset.nodes.get(id.0).map(|point| point.as_dvec3()))
                .ok_or_else(|| {
                    format!(
                        "{label}: source node {} has no simulated chassis node",
                        id.0
                    )
                })
        };
        let [a, b, c] = binding.placement_nodes.map(resolve);
        let [(a, pa), (b, pb), (c, pc)] = [a?, b?, c?];
        let rest_frame = Frame::new([pa, pb, pc]).ok_or_else(|| {
            format!(
                "{label}: degenerate placement nodes {:?}",
                binding.placement_nodes
            )
        })?;
        let influences = binding
            .influence_nodes
            .iter()
            .map(|&id| resolve(id).map(|(index, point)| (index, rest_frame.to_local(point))))
            .collect::<Result<Vec<_>, _>>()?;
        if influences.is_empty() {
            return Err(format!("{label}: missing influence nodes"));
        }
        let vertices = mesh
            .vertices
            .iter()
            .map(|&point| {
                let rest = rest_frame.to_local(instance.transform_point(point).as_dvec3());
                let mut distances: Vec<_> = influences
                    .iter()
                    .enumerate()
                    .map(|(i, &(_, node))| (i, rest.distance_squared(node)))
                    .collect();
                distances.sort_by(|a, b| a.1.total_cmp(&b.1));
                // Imported positions and transforms are f32. Snap coincidences within 1 µm.
                let (nearest, distance) = distances[0];
                let weights = if distance <= 1e-12 {
                    vec![(nearest, 1.0)]
                } else {
                    distances.truncate(4);
                    let total = distances.iter().map(|&(_, d)| d.recip()).sum::<f64>();
                    distances
                        .into_iter()
                        .map(|(i, d)| (i, d.recip() / total))
                        .collect()
                };
                Vertex { rest, weights }
            })
            .collect();
        Ok(Self {
            object: binding.object,
            instance: instance.clone(),
            placement: [a, b, c],
            placement_ids: binding.placement_nodes,
            rest_frame,
            influences,
            vertices,
        })
    }

    fn frame(&self, nodes_m: &[DVec3]) -> Result<Frame, String> {
        let [a, b, c] = self.placement.map(|i| nodes_m[i]);
        Frame::new([a, b, c]).ok_or_else(|| {
            format!(
                "{} (object {}): degenerate placement nodes {:?}",
                self.instance.name, self.object, self.placement_ids
            )
        })
    }

    fn vertices(&self, nodes_m: &[DVec3], frame: Frame, body: Frame) -> Vec<Vec3> {
        let displacements: Vec<_> = self
            .influences
            .iter()
            .map(|&(i, rest)| frame.to_local(nodes_m[i]) - rest)
            .collect();
        let rotation = self.instance.rotation.as_dquat().normalize().inverse();
        let translation = self.instance.translation.as_dvec3();
        let scale = self.instance.scale.as_dvec3();
        self.vertices
            .iter()
            .map(|vertex| {
                let offset: DVec3 = vertex
                    .weights
                    .iter()
                    .map(|&(i, weight)| weight * displacements[i])
                    .sum();
                let point = body.to_local(frame.to_world(vertex.rest + offset));
                (rotation * (point - translation) / scale).as_vec3()
            })
            .collect()
    }
}

/// Bindings in authored object order. All referenced source nodes must be simulated chassis nodes.
pub struct BodySkin {
    pub meshes: Vec<MeshSkin>,
    node_count: usize,
}

/// Body transform in world metres and mesh-local vertices for each binding in order.
pub struct BodyShape {
    pub pose: VehiclePose,
    pub vertices: Vec<Vec<Vec3>>,
}

impl BodySkin {
    pub fn new(asset: &VehicleAsset, source: &SourceIndices) -> Result<Self, String> {
        let structure = asset
            .physics
            .structure
            .as_ref()
            .ok_or("missing authored chassis structure")?;
        let meshes = structure
            .deformable_meshes
            .iter()
            .map(|binding| MeshSkin::new(asset, binding, source))
            .collect::<Result<Vec<_>, _>>()?;
        if meshes.is_empty() {
            return Err("vehicle has no deformable mesh bindings".into());
        }
        Ok(Self {
            meshes,
            node_count: source.nodes.len(),
        })
    }

    pub fn sample(&self, nodes_m: &[DVec3]) -> Result<BodyShape, String> {
        if nodes_m.len() != self.node_count || nodes_m.iter().any(|p| !p.is_finite()) {
            return Err("invalid recorded chassis node positions".into());
        }
        let frames = self
            .meshes
            .iter()
            .map(|mesh| mesh.frame(nodes_m))
            .collect::<Result<Vec<_>, _>>()?;
        // The reference body's first placement is anchored behind its crush zone.
        let rest = self.meshes[0].rest_frame;
        let current = frames[0];
        let rotation = (current.rotation * rest.rotation.inverse()).normalize();
        let body = Frame {
            origin: current.origin - rotation * rest.origin,
            rotation,
        };
        Ok(BodyShape {
            pose: VehiclePose {
                origin_m: body.origin.as_vec3(),
                rotation: body.rotation.as_quat(),
            },
            vertices: self
                .meshes
                .iter()
                .zip(frames)
                .map(|(mesh, frame)| mesh.vertices(nodes_m, frame, body))
                .collect(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vehicle_assets::ror::reference;

    fn fixture() -> (VehicleAsset, SourceIndices) {
        let asset = reference::vehicle().unwrap().vehicle;
        let source = SourceIndices {
            nodes: asset
                .physics
                .structure
                .as_ref()
                .unwrap()
                .nodes
                .iter()
                .enumerate()
                .filter_map(|(i, node)| node.upright.is_none().then_some(i))
                .collect(),
            beams: vec![],
            panels: vec![],
        };
        (asset, source)
    }

    #[test]
    fn rest_shape_survives_rigid_motion() {
        let (asset, source) = fixture();
        let skin = BodySkin::new(&asset, &source).unwrap();
        let instance = &asset.objects[skin.meshes[0].object];
        let vertices = &asset.meshes[instance.visual_mesh.unwrap().0].vertices;
        for (rotation, translation) in [
            (DQuat::IDENTITY, DVec3::ZERO),
            (
                DQuat::from_rotation_y(0.7) * DQuat::from_rotation_x(0.4),
                DVec3::new(57.0, 0.8, 24.0),
            ),
        ] {
            let nodes: Vec<_> = source
                .nodes
                .iter()
                .map(|&i| translation + rotation * asset.nodes[i].as_dvec3())
                .collect();
            let shape = skin.sample(&nodes).unwrap();
            assert!(shape.pose.origin_m.abs_diff_eq(translation.as_vec3(), 1e-6));
            assert!(shape.pose.rotation.abs_diff_eq(rotation.as_quat(), 1e-6));
            for (actual, expected) in shape.vertices[0].iter().zip(vertices) {
                assert!(actual.abs_diff_eq(*expected, 1e-6));
            }
        }
    }

    #[test]
    fn a_nose_vertex_follows_its_node_without_moving_another_instance() {
        let (mut asset, source) = fixture();
        let structure = asset.physics.structure.as_mut().unwrap();
        let first = &structure.deformable_meshes[0];
        let object = first.object;
        let instance = asset.objects[object].clone();
        let mesh_id = instance.visual_mesh.unwrap();
        let second = asset.objects.len();
        asset.objects.push(MeshInstance {
            translation: instance.translation + Vec3::X,
            rotation: Quat::from_rotation_y(0.3) * instance.rotation,
            scale: Vec3::new(-1.0, 1.0, 1.0),
            ..instance
        });
        structure.deformable_meshes.push(DeformableMeshBinding {
            object: second,
            placement_nodes: first.placement_nodes,
            influence_nodes: vec![NodeId(0)],
        });
        let skin = BodySkin::new(&asset, &source).unwrap();
        let mut nodes: Vec<_> = source
            .nodes
            .iter()
            .map(|&i| asset.nodes[i].as_dvec3())
            .collect();
        let node = source.nodes.iter().position(|i| *i == 8).unwrap();
        let displacement = DVec3::new(0.025, 0.0, -0.05);
        nodes[node] += displacement;

        let vertex = asset.meshes[mesh_id.0]
            .vertices
            .iter()
            .position(|&point| {
                asset.objects[object]
                    .transform_point(point)
                    .abs_diff_eq(asset.nodes[8], 1e-6)
            })
            .unwrap();
        let shape = skin.sample(&nodes).unwrap();
        let world_vertex = |mesh: usize, instance: usize| {
            asset.objects[instance].transform_point(shape.vertices[mesh][vertex])
        };
        assert!(world_vertex(0, object).abs_diff_eq(nodes[node].as_vec3(), 1e-6));
        let second_rest =
            asset.objects[second].transform_point(asset.meshes[mesh_id.0].vertices[vertex]);
        assert!(world_vertex(skin.meshes.len() - 1, second).abs_diff_eq(second_rest, 1e-6));
    }

    #[test]
    fn reference_nose_crush_ends_at_the_bulkhead() {
        let (asset, source) = fixture();
        let skin = BodySkin::new(&asset, &source).unwrap();
        let rest: Vec<_> = source
            .nodes
            .iter()
            .map(|&i| asset.nodes[i].as_dvec3())
            .collect();
        let mut crushed = rest.clone();
        for (point, &id) in crushed.iter_mut().zip(&source.nodes) {
            if (8..12).contains(&id) {
                point.z -= 0.1;
            }
        }
        let initial = skin.sample(&rest).unwrap();
        let impact = skin.sample(&crushed).unwrap();
        let body = &asset.objects[skin.meshes[0].object];
        let bulkhead_z = asset.nodes[4].z;
        for (&original, &moved) in initial.vertices[0].iter().zip(&impact.vertices[0]) {
            let original = body.transform_point(original);
            let moved = body.transform_point(moved);
            let expected = if original.z > bulkhead_z + 1e-6 {
                original - 0.1 * Vec3::Z
            } else {
                original
            };
            assert!(
                moved.abs_diff_eq(expected, 1e-6),
                "{original:?} moved to {moved:?}"
            );
        }
        assert_eq!(initial.vertices[1], impact.vertices[1]);
    }
}

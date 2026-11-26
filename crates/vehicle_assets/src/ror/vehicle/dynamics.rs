use super::{Record, WheelRecord, number, resolve_nodes};
use crate::{
    ChassisPhysics, NodeId, StructuralNode, VehicleAsset, VehiclePhysics, VerticalSuspension,
    WheelAttachment,
};
use bevy_math::Vec3;
use std::collections::HashMap;

pub(super) fn convert(
    records: &[Record<'_>],
    vehicle: &VehicleAsset,
    nodes: &HashMap<u32, NodeId>,
    mut wheels: Vec<WheelRecord>,
    body_objects: &[usize],
    structural_nodes: &mut [StructuralNode],
) -> VehiclePhysics {
    let shocks: Vec<_> = records
        .iter()
        .copied()
        .filter(|record| record.section == "shocks")
        .filter_map(|record| Shock::parse(record, nodes, &vehicle.nodes))
        .collect();
    let mut upright_nodes = Vec::with_capacity(wheels.len());
    for (wheel, geometry) in wheels.iter_mut().zip(&vehicle.wheels) {
        let [a, b] = wheel.axle_nodes.map(|id| vehicle.nodes[id.0]);
        let axle = b - a;
        let mut matches = shocks.iter().filter(|shock| {
            let lower = vehicle.nodes[shock.lower.0];
            let fraction = ((lower - a).dot(axle) / axle.length_squared()).clamp(0.0, 1.0);
            lower.distance(a + fraction * axle) <= 0.25 * geometry.width
        });
        let mut attached = wheel.axle_nodes.to_vec();
        if let Some(shock) = matches.next()
            && matches.next().is_none()
        {
            wheel.physics.suspension = shock.suspension();
            if wheel.physics.suspension.is_some() {
                wheel.physics.attachment = Some(WheelAttachment {
                    axle_nodes: wheel.axle_nodes,
                    upper_node: if shock.lower == shock.ends[0] {
                        shock.ends[1]
                    } else {
                        shock.ends[0]
                    },
                    lower_node: shock.lower,
                });
            }
            if !attached.contains(&shock.lower) {
                attached.push(shock.lower);
            }
        }
        upright_nodes.push(attached);
    }

    let chassis = node_masses(records, vehicle, &shocks).and_then(|masses| {
        let mut owners = vec![None; masses.len()];
        for (index, nodes) in upright_nodes.iter().enumerate() {
            for id in nodes {
                if owners[id.0].replace(index).is_some() {
                    return None;
                }
            }
        }
        for ((node, &mass), owner) in structural_nodes.iter_mut().zip(&masses).zip(owners) {
            node.mass_kg = Some(mass);
            node.upright = owner;
        }
        for (wheel, nodes) in wheels.iter_mut().zip(upright_nodes) {
            let upright_mass: f32 = nodes.iter().map(|id| masses[id.0]).sum();
            wheel.physics.upright_mass_kg = (upright_mass > 0.0).then_some(upright_mass);
        }
        let size_m = body_size(vehicle, body_objects)?;
        let mut mass_kg = 0.0;
        let mut moment = Vec3::ZERO;
        for ((&point, mass), node) in vehicle.nodes.iter().zip(masses).zip(structural_nodes) {
            if node.upright.is_none() {
                mass_kg += mass;
                moment += point * mass;
            }
        }
        let centre_of_mass_m = moment / mass_kg;
        (mass_kg > 0.0 && mass_kg.is_finite() && centre_of_mass_m.is_finite()).then_some(
            ChassisPhysics {
                mass_kg,
                size_m,
                centre_of_mass_m,
            },
        )
    });

    let driven = wheels
        .iter()
        .filter(|wheel| matches!(wheel.drive, Some(1 | 2)))
        .count();
    let launch_torque = fields_in(records, "engine").and_then(|fields| {
        let engine = number(fields.get(2)?).ok()?;
        let final_drive = number(fields.get(3)?).ok()?;
        let first_gear = number(fields.get(6)?).ok()?;
        let torque = engine * final_drive * first_gear / driven as f32;
        (engine > 0.0 && final_drive > 0.0 && first_gear > 0.0 && torque.is_finite())
            .then_some(torque)
    });
    let brake_torque = fields_in(records, "brakes")
        .and_then(|fields| number(fields.first()?).ok())
        .filter(|&value| value >= 0.0);
    let wheels = wheels
        .into_iter()
        .map(|mut wheel| {
            wheel.physics.drive_torque_nm = match wheel.drive {
                Some(0) => Some(0.0),
                Some(1) => launch_torque,
                Some(2) => launch_torque.map(|torque| -torque),
                _ => None,
            };
            wheel.physics.brake_torque_nm = match wheel.brake {
                Some(0) => Some(0.0),
                Some(1..=4) => brake_torque,
                _ => None,
            };
            wheel.physics
        })
        .collect();
    VehiclePhysics {
        chassis,
        wheels,
        structure: None,
    }
}

fn fields_in<'a>(records: &[Record<'a>], section: &str) -> Option<Vec<&'a str>> {
    records
        .iter()
        .rev()
        .filter(|record| record.section == section)
        .map(|record| record.fields())
        .find(|fields| fields.first().is_some_and(|value| number(value).is_ok()))
}

fn body_size(vehicle: &VehicleAsset, body_objects: &[usize]) -> Option<Vec3> {
    let mut min = Vec3::splat(f32::INFINITY);
    let mut max = Vec3::splat(f32::NEG_INFINITY);
    for &index in body_objects {
        let object = &vehicle.objects[index];
        let mesh = &vehicle.meshes[object.visual_mesh?.0];
        for &vertex in &mesh.vertices {
            let point = object.transform_point(vertex);
            min = min.min(point);
            max = max.max(point);
        }
    }
    let size = max - min;
    (size.is_finite() && size.min_element() > 0.0).then_some(size)
}

fn node_masses(
    records: &[Record<'_>],
    vehicle: &VehicleAsset,
    shocks: &[Shock],
) -> Option<Vec<f32>> {
    let globals = fields_in(records, "globals")?;
    let dry = number(globals.first()?).ok()?;
    let payload = number(globals.get(1)?).ok()?;
    if dry < 0.0 || payload < 0.0 {
        return None;
    }
    let minimass = fields_in(records, "minimass");
    let minimum = match &minimass {
        Some(fields) => number(fields.first()?).ok()?,
        None => 0.0,
    };

    // RoR assigns half of each structural beam's mass to each authored endpoint.
    let mut lengths = vec![0.0; vehicle.nodes.len()];
    let beams = vehicle.edges.iter().map(|&edge| {
        let [a, b] = edge.map(|id| vehicle.nodes[id.0]);
        (edge, a.distance(b))
    });
    for ([a, b], length) in beams.chain(shocks.iter().map(|shock| (shock.ends, shock.free))) {
        lengths[a.0] += length * 0.5;
        lengths[b.0] += length * 0.5;
    }
    let total_length = lengths.iter().sum::<f32>();
    if dry > 0.0 && !(total_length.is_finite() && total_length > 0.0) {
        return None;
    }
    let density = if dry > 0.0 { dry / total_length } else { 0.0 };

    let mut loads = Vec::with_capacity(lengths.len());
    let mut default_weight = None;
    let mut default_loaded = false;
    for &record in records {
        match record.section {
            "set_node_defaults" => {
                let fields = record.fields();
                default_weight = fields
                    .get(1)
                    .and_then(|value| number(value).ok())
                    .filter(|value| *value >= 0.0);
                default_loaded = fields.get(5).is_some_and(|value| value.contains('l'));
            }
            "nodes" => {
                let fields = record.fields();
                let loaded =
                    default_loaded || fields.get(4).is_some_and(|value| value.contains('l'));
                let weight = fields
                    .get(5)
                    .and_then(|value| number(value).ok())
                    .filter(|w| *w >= 0.0);
                loads.push((loaded, weight.or(default_weight)));
            }
            _ => {}
        }
    }

    let sharing = loads
        .iter()
        .filter(|&&(loaded, weight)| loaded && weight.is_none())
        .count();
    let share = if sharing > 0 {
        payload / sharing as f32
    } else {
        0.0
    };
    let masses: Vec<_> = lengths
        .into_iter()
        .zip(loads)
        .map(|(length, (loaded, weight))| {
            let load = if loaded { weight.unwrap_or(share) } else { 0.0 };
            (length * density + load).max(minimum)
        })
        .collect();
    masses.iter().all(|mass| mass.is_finite()).then_some(masses)
}

struct Shock {
    ends: [NodeId; 2],
    lower: NodeId,
    length: f32,
    vertical: f32,
    free: f32,
    short: f32,
    long: f32,
    stiffness: f32,
    damping: f32,
}

impl Shock {
    fn parse(record: Record<'_>, nodes: &HashMap<u32, NodeId>, points: &[Vec3]) -> Option<Self> {
        let fields = record.fields();
        let [first, second, spring, damper, short, long, rest @ ..] = fields.as_slice() else {
            return None;
        };
        let ends = resolve_nodes(nodes, [first, second]).ok()?;
        let [a, b] = ends.map(|id| points[id.0]);
        let length = a.distance(b);
        let vertical = (a.y - b.y).abs();
        let precompression = rest.first().map_or(Ok(1.0), |value| number(value)).ok()?;
        let flags = rest.get(1).copied().unwrap_or("");
        if flags.chars().any(|flag| !matches!(flag, 'i' | 'm' | 'n')) {
            return None;
        }
        let stiffness = number(spring).ok()?;
        let damping = number(damper).ok()?;
        let short = number(short).ok()?;
        let long = number(long).ok()?;
        if length <= 0.0
            || precompression <= 0.0
            || stiffness <= 0.0
            || damping < 0.0
            || short < 0.0
            || long < 0.0
        {
            return None;
        }
        let bound_scale = if flags.contains('m') {
            length.recip()
        } else {
            1.0
        };
        Some(Self {
            ends,
            lower: if a.y < b.y { ends[0] } else { ends[1] },
            length,
            vertical,
            free: length * precompression,
            short: short * bound_scale,
            long: long * bound_scale,
            stiffness,
            damping,
        })
    }

    fn suspension(&self) -> Option<VerticalSuspension> {
        let ratio = self.vertical / self.length;
        if ratio < 0.1 {
            return None;
        }
        // First-order projection about the authored pose. Shock displacement is
        // ratio times slider displacement, and shock force projects by ratio too.
        let free = self.vertical + (self.free - self.length) / ratio;
        let min = (free - self.free * self.short / ratio).max(0.0);
        let max = free + self.free * self.long / ratio;
        (max.is_finite() && max > min && (min..=max).contains(&self.vertical)).then_some(
            VerticalSuspension {
                stiffness_n_per_m: self.stiffness * ratio * ratio,
                damping_ns_per_m: self.damping * ratio * ratio,
                initial_length_m: self.vertical,
                free_length_m: free,
                min_length_m: min,
                max_length_m: max,
            },
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MeshId, MeshInstance, TriangleMesh, WheelGeometry, WheelPhysics, WheelSide};
    use bevy_math::Quat;

    #[test]
    fn conserves_beam_and_node_mass_when_assigning_an_upright() {
        let (.., records) = super::super::parse_truck(
            "Fixture\nglobals\n20,90\nnodes\n0,0,0,0,l 4\n1,0,0,0,l\n2,0,0,0\n3,0,0,0\nshocks\n1,2,1000,100,0.25,0.25,1.1,i\n",
        ).unwrap();
        let vehicle = VehicleAsset {
            name: "Fixture".into(),
            nodes: vec![Vec3::ZERO, Vec3::X, Vec3::new(1.0, 1.0, 0.0), Vec3::Z],
            edges: vec![[NodeId(0), NodeId(1)], [NodeId(0), NodeId(3)]],
            meshes: vec![TriangleMesh {
                vertices: vec![Vec3::ZERO, Vec3::new(2.0, 1.0, 4.0)],
                triangles: Vec::new(),
                normals: None,
                uvs: None,
            }],
            objects: vec![MeshInstance {
                name: "body".into(),
                translation: Vec3::Z,
                rotation: Quat::IDENTITY,
                scale: Vec3::ONE,
                visual_mesh: Some(MeshId(0)),
                collision_mesh: None,
            }],
            wheels: vec![WheelGeometry {
                centre: Vec3::X * 0.5,
                axle: Vec3::X,
                outer_radius: 0.3,
                width: 0.2,
                rim_radius: Some(0.2),
            }],
            physics: VehiclePhysics::default(),
        };
        let wheel = WheelRecord {
            physics: WheelPhysics {
                side: WheelSide::Left,
                mass_kg: Some(5.0),
                upright_mass_kg: None,
                attachment: None,
                suspension: None,
                drive_torque_nm: None,
                brake_torque_nm: None,
                rim_object: 1,
                tyre_object: 2,
            },
            axle_nodes: [NodeId(0), NodeId(1)],
            drive: Some(0),
            brake: Some(0),
        };
        let nodes = (0..4).map(|i| (i, NodeId(i as usize))).collect();
        let mut structural_nodes = vec![StructuralNode::default(); vehicle.nodes.len()];
        let physics = convert(
            &records,
            &vehicle,
            &nodes,
            vec![wheel],
            &[0],
            &mut structural_nodes,
        );
        assert_eq!(
            structural_nodes
                .iter()
                .map(|node| node.upright)
                .collect::<Vec<_>>(),
            [Some(0), Some(0), None, None],
        );
        let chassis = physics.chassis.unwrap();
        let wheel = physics.wheels[0];
        let mass = chassis.mass_kg + wheel.upright_mass_kg.unwrap() + wheel.mass_kg.unwrap();
        assert!((mass - (20.0 + 90.0 + 4.0 + 5.0)).abs() < 1e-5);
        assert_eq!(chassis.size_m, Vec3::new(2.0, 1.0, 4.0));
        assert!(
            chassis
                .centre_of_mass_m
                .abs_diff_eq((Vec3::new(1.0, 1.0, 0.0) * 1.1 + Vec3::Z) / 2.1, 1e-5,)
        );
        let suspension = wheel.suspension.unwrap();
        assert_eq!(
            wheel.attachment,
            Some(WheelAttachment {
                axle_nodes: [NodeId(0), NodeId(1)],
                upper_node: NodeId(2),
                lower_node: NodeId(1),
            }),
        );
        assert_eq!(suspension.stiffness_n_per_m, 1000.0);
        assert!((suspension.free_length_m - 1.1).abs() < 1e-6);
        assert!((suspension.min_length_m - 0.825).abs() < 1e-6);
        assert!((suspension.max_length_m - 1.375).abs() < 1e-6);
    }
}

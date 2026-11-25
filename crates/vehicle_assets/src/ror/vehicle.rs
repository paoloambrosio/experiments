use std::{
    collections::{BTreeSet, HashMap},
    path::Path,
};

use bevy_math::{Mat3, Quat, Vec2, Vec3};

use crate::{
    MeshAppearance, MeshId, MeshInstance, NodeId, TriangleMesh, VehicleAsset, WheelGeometry,
};

use super::{
    archive::{Archive, ImportError, ImportWarning, warn},
    materials::Materials,
    ogre::{Mesh, MeshCache},
};

#[derive(Debug)]
pub struct VehicleImport {
    pub vehicle: VehicleAsset,
    pub appearance: MeshAppearance,
    pub warnings: Vec<ImportWarning>,
    pub entry: String,
}

#[derive(Clone, Copy)]
struct Record<'a> {
    section: &'a str,
    line: usize,
    value: &'a str,
}

impl<'a> Record<'a> {
    fn fields(self) -> Vec<&'a str> {
        self.value
            .split(|c: char| c == ',' || c.is_whitespace())
            .filter(|field| !field.is_empty())
            .collect()
    }
}

/// Loads the authored rest pose of a RoR vehicle from its ZIP archive.
pub fn load_vehicle(path: &Path, entry: &str) -> Result<VehicleImport, ImportError> {
    let (mut archive, entry) = Archive::select(path, entry)?;
    let text = archive.text(&entry)?;
    let mut warnings = Vec::new();
    let (title, records) = parse_truck(&text).map_err(|error| archive.error(&entry, error))?;
    let mut vehicle = VehicleAsset {
        name: title.to_owned(),
        meshes: Vec::new(),
        objects: Vec::new(),
        nodes: Vec::new(),
        edges: Vec::new(),
        wheels: Vec::new(),
    };
    let mut nodes = HashMap::new();
    let mut meshes = MeshCache::default();
    let mut materials = Materials::load(&mut archive, &mut warnings);
    let error = |archive: &Archive, record: Record<'_>, detail: String| {
        archive.error(
            &entry,
            format!("{} line {}: {detail}", record.section, record.line),
        )
    };

    for &record in records.iter().filter(|record| record.section == "nodes") {
        let fields = record.fields();
        let result = (|| {
            let [id, x, y, z, ..] = fields.as_slice() else {
                return Err("expected node ID and three coordinates".into());
            };
            let id = node_number(id)?;
            if nodes.insert(id, NodeId(vehicle.nodes.len())).is_some() {
                return Err(format!("duplicate node {id}"));
            }
            let point = vec3([x, y, z])?;
            // RoR's truck frame has -X forward and +Z left.
            vehicle.nodes.push(Vec3::new(point.z, point.y, -point.x));
            Ok(())
        })();
        result.map_err(|detail| error(&archive, record, detail))?;
    }
    if nodes.is_empty() {
        return Err(archive.error(&entry, "missing authored nodes"));
    }

    let mut deferred = BTreeSet::new();
    for &record in &records {
        let fields = record.fields();
        let result = match record.section {
            "nodes" => continue,
            "beams" => {
                if let [first, second, ..] = fields.as_slice() {
                    let edge = resolve_nodes(&nodes, [first, second])
                        .map_err(|detail| error(&archive, record, detail))?;
                    vehicle.edges.push(edge);
                    Ok(())
                } else {
                    Err("expected two node IDs".into())
                }
            }
            "meshwheels2" => add_wheel(
                &mut archive,
                &entry,
                &mut meshes,
                &mut vehicle,
                &nodes,
                &fields,
            ),
            "flexbodies" | "props" => {
                if fields
                    .first()
                    .is_some_and(|field| matches!(*field, "forset" | "forvert" | "add_animation"))
                {
                    deferred.insert(if record.section == "flexbodies" {
                        "flexbody deformation"
                    } else {
                        "prop animation"
                    });
                    continue;
                }
                add_attachment(
                    &mut archive,
                    &entry,
                    &mut meshes,
                    &mut vehicle,
                    &nodes,
                    record.section,
                    &fields,
                )
            }
            "managedmaterials" => {
                if let [name, effect, diffuse, extra @ ..] = fields.as_slice() {
                    match *effect {
                        "mesh_standard"
                        | "mesh_transparent"
                        | "flexmesh_standard"
                        | "flexmesh_transparent" => {
                            materials.define_managed(
                                &entry,
                                name,
                                (*diffuse != "-").then_some(*diffuse),
                                effect.ends_with("_transparent"),
                            );
                            if extra.iter().any(|texture| *texture != "-") {
                                deferred.insert("specular and damage textures");
                            }
                        }
                        _ => {
                            warn(
                                &mut warnings,
                                &entry,
                                format!("managed material {name} has unsupported effect {effect}"),
                            );
                        }
                    }
                    Ok(())
                } else {
                    Err("expected material name, effect, and diffuse texture".into())
                }
            }
            section => {
                deferred.insert(section);
                continue;
            }
        };
        result.map_err(|detail| error(&archive, record, detail))?;
    }
    if !deferred.is_empty() {
        warn(
            &mut warnings,
            &entry,
            format!(
                "static preview skips {}",
                deferred.into_iter().collect::<Vec<_>>().join(", ")
            ),
        );
    }

    vehicle.meshes = meshes.geometry;
    vehicle
        .validate()
        .map_err(|error| archive.error(&entry, error.to_string()))?;
    for id in vehicle
        .objects
        .iter()
        .filter_map(|object| object.visual_mesh)
    {
        let (path, sections) = &meshes.bindings[id.0];
        materials.bind(&mut archive, id, path, sections, &mut warnings);
    }
    warnings.sort();
    warnings.dedup();
    Ok(VehicleImport {
        vehicle,
        appearance: materials.appearance,
        warnings,
        entry,
    })
}

fn parse_truck(text: &str) -> Result<(&str, Vec<Record<'_>>), String> {
    let mut lines = text
        .trim_start_matches('\u{feff}')
        .lines()
        .enumerate()
        .filter_map(|(line, value)| {
            let value = value.split(';').next()?.split("//").next()?.trim();
            (!value.is_empty()).then_some((line + 1, value))
        });
    let (_, title) = lines.next().ok_or("missing truck title")?;
    let mut section = "header";
    let mut records = Vec::new();
    for (line, value) in lines {
        if value == "end" {
            break;
        }
        if value.starts_with("set_") || value.starts_with("detacher_group ") {
            continue;
        }
        if value.starts_with("extcamera ") {
            records.push(Record {
                section: "extcamera",
                line,
                value,
            });
            continue;
        }
        if value.starts_with(|ch: char| ch.is_ascii_alphabetic())
            && value
                .bytes()
                .all(|byte| byte.is_ascii_alphanumeric() || byte == b'_')
        {
            section = value;
        } else {
            records.push(Record {
                section,
                line,
                value,
            });
        }
    }
    Ok((title, records))
}

fn number(value: &str) -> Result<f32, String> {
    value
        .parse::<f32>()
        .ok()
        .filter(|value| value.is_finite())
        .ok_or_else(|| format!("invalid number {value:?}"))
}

fn vec3(fields: [&str; 3]) -> Result<Vec3, String> {
    let [x, y, z] = fields;
    Ok(Vec3::new(number(x)?, number(y)?, number(z)?))
}

fn node_number(value: &str) -> Result<u32, String> {
    value
        .parse::<u32>()
        .map_err(|_| format!("invalid node ID {value:?}"))
}

fn resolve_nodes<const N: usize>(
    nodes: &HashMap<u32, NodeId>,
    fields: [&str; N],
) -> Result<[NodeId; N], String> {
    let mut resolved = [NodeId(0); N];
    for (field, output) in fields.into_iter().zip(&mut resolved) {
        let id = node_number(field)?;
        *output = *nodes.get(&id).ok_or_else(|| format!("unknown node {id}"))?;
    }
    Ok(resolved)
}

fn attach(visual_mesh: MeshId, name: &str, translation: Vec3, rotation: Quat) -> MeshInstance {
    MeshInstance {
        name: name.to_owned(),
        translation,
        rotation,
        scale: Vec3::ONE,
        visual_mesh: Some(visual_mesh),
        collision_mesh: None,
    }
}

fn add_attachment(
    archive: &mut Archive,
    entry: &str,
    meshes: &mut MeshCache,
    vehicle: &mut VehicleAsset,
    nodes: &HashMap<u32, NodeId>,
    section: &str,
    fields: &[&str],
) -> Result<(), String> {
    let [reference, x, y, tx, ty, tz, rx, ry, rz, mesh, extra @ ..] = fields else {
        return Err("expected reference nodes, offset, rotation, and mesh".into());
    };
    let [reference, x, y] = resolve_nodes(nodes, [reference, x, y])?.map(|id| vehicle.nodes[id.0]);
    let dx = x - reference;
    let dy = y - reference;
    let normal = dy
        .cross(dx)
        .try_normalize()
        .ok_or("attachment nodes are collinear")?;
    let local_x = dx.normalize();
    let frame = Quat::from_mat3(&Mat3::from_cols(local_x, normal, local_x.cross(normal)));
    let offset = vec3([tx, ty, tz])?;
    let euler = vec3([rx, ry, rz])? * (std::f32::consts::PI / 180.0);
    let translation = reference + offset.x * dx + offset.y * dy + offset.z * normal;
    let rotation = frame
        * Quat::from_rotation_z(euler.z)
        * Quat::from_rotation_y(euler.y)
        * Quat::from_rotation_x(euler.x);
    let id = meshes
        .load(archive, entry, mesh)
        .map_err(|error| error.to_string())?;
    vehicle
        .objects
        .push(attach(id, mesh, translation, rotation));

    if section == "props"
        && let [steering, rest @ ..] = extra
        && steering.to_ascii_lowercase().ends_with(".mesh")
    {
        let offset = match rest {
            [x, y, z, ..] => vec3([x, y, z])?,
            [] => Vec3::ZERO,
            _ => return Err("expected three steering wheel offset coordinates".into()),
        };
        let id = meshes
            .load(archive, entry, steering)
            .map_err(|error| error.to_string())?;
        let tilt = Quat::from_rotation_x(-59.0_f32.to_radians());
        vehicle.objects.push(attach(
            id,
            steering,
            translation + rotation * offset,
            rotation * tilt,
        ));
    }
    Ok(())
}

fn add_wheel(
    archive: &mut Archive,
    entry: &str,
    meshes: &mut MeshCache,
    vehicle: &mut VehicleAsset,
    nodes: &HashMap<u32, NodeId>,
    fields: &[&str],
) -> Result<(), String> {
    let [
        radius,
        rim,
        width,
        rays,
        first,
        second,
        _,
        _,
        _,
        _,
        _,
        _,
        _,
        side,
        mesh,
        tread,
        ..,
    ] = fields
    else {
        return Err(
            "expected meshwheels2 dimensions, axle nodes, side, rim mesh, and tyre material".into(),
        );
    };
    let [outer_radius, rim, width] = [number(radius)?, number(rim)?, number(width)?];
    if rim <= 0.0 || rim >= outer_radius || width <= 0.0 {
        return Err("expected positive tyre width and radii with the rim inside the tyre".into());
    }
    let rays = rays.parse::<u32>().map_err(|_| "invalid tyre ray count")?;
    if !(3..=1024).contains(&rays) {
        return Err("tyre ray count must be in 3..=1024".into());
    }
    let [first, second] = resolve_nodes(nodes, [first, second])?.map(|id| vehicle.nodes[id.0]);
    let direction = second - first;
    if direction.x == 0.0 {
        return Err("wheel axle has no lateral span".into());
    }
    let axle = direction.normalize() * direction.x.signum();
    let centre = (first + second) * 0.5;
    let out = match *side {
        "l" | "L" => axle,
        "r" | "R" => -axle,
        _ => return Err(format!("invalid wheel side {side:?}")),
    };
    let up = (Vec3::Y - out * out.y).normalize();
    let rim_rotation = Quat::from_mat3(&Mat3::from_cols(out, up, out.cross(up)));
    let rim_id = meshes
        .load(archive, entry, mesh)
        .map_err(|error| error.to_string())?;
    vehicle
        .objects
        .push(attach(rim_id, mesh, centre, rim_rotation));

    let rotation = Quat::from_mat3(&Mat3::from_cols(axle, up, axle.cross(up)));
    let tyre = tyre_mesh(outer_radius, rim, width, rays * 4, tread);
    let id = meshes.push(entry, tyre);
    vehicle.objects.push(attach(id, tread, centre, rotation));
    vehicle.wheels.push(WheelGeometry {
        centre,
        axle,
        outer_radius,
        width,
        rim_radius: Some(rim),
    });
    Ok(())
}

fn tyre_mesh(radius: f32, rim: f32, width: f32, segments: u32, material: &str) -> Mesh {
    let shoulder = rim + (radius - rim) * 0.9;
    let profile = [
        (-0.5, rim, 0.0),
        (-0.5, shoulder, 0.23),
        (-0.38, radius, 0.27),
        (0.38, radius, 0.73),
        (0.5, shoulder, 0.77),
        (0.5, rim, 1.0),
    ];
    let mut vertices = Vec::new();
    let mut uvs = Vec::new();
    let mut triangles = Vec::new();
    let rows = profile.len() as u32;
    for segment in 0..=segments {
        let u = segment as f32 / segments as f32;
        let angle = std::f32::consts::TAU * u;
        let (sin, cos) = angle.sin_cos();
        for &(x, r, v) in &profile {
            vertices.push(Vec3::new(x * width, r * cos, r * sin));
            uvs.push(Vec2::new(u, v));
        }
    }
    for segment in 0..segments {
        for row in 0..rows - 1 {
            let a = segment * rows + row;
            let b = a + rows;
            triangles.extend([[a, b, a + 1], [a + 1, b, b + 1]]);
        }
    }
    let sections = vec![(0..triangles.len(), material.to_owned())];
    Mesh {
        geometry: TriangleMesh {
            vertices,
            triangles,
            normals: None,
            uvs: Some(uvs),
        },
        sections,
    }
}

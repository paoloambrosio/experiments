use std::{
    collections::{BTreeSet, HashMap},
    path::Path,
};

use bevy_math::{Mat3, Quat, Vec2, Vec3};

use crate::{
    DeformableMeshBinding, MeshAppearance, MeshId, MeshInstance, NodeId, StructuralNode,
    TriangleMesh, VehicleAsset, VehicleStructure, WheelGeometry, WheelPhysics, WheelSide,
};

use super::{
    archive::{Archive, Import, ImportError, ImportWarning, warn},
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

mod dynamics;
mod structure;

struct WheelRecord {
    physics: WheelPhysics,
    axle_nodes: [NodeId; 2],
    drive: Option<u8>,
    brake: Option<u8>,
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

    fn describe(self, detail: &str) -> String {
        format!("{} line {}: {detail}", self.section, self.line)
    }
}

/// Loads the authored rest pose of a RoR vehicle from its ZIP archive.
pub fn load_vehicle(path: &Path, entry: &str) -> Result<VehicleImport, ImportError> {
    let (archive, entry) = Archive::select(path, entry)?;
    import(archive, entry)
}

pub(super) fn load_embedded(
    path: &Path,
    bytes: &'static [u8],
    entry: &str,
) -> Import<VehicleImport> {
    let (archive, entry) = Archive::select_embedded(path, bytes, entry)?;
    import(archive, entry)
}

fn import(mut archive: Archive, entry: String) -> Import<VehicleImport> {
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
        physics: Default::default(),
    };
    let mut nodes = HashMap::new();
    let mut wheel_details = Vec::new();
    let mut body_objects = Vec::new();
    let mut meshes = MeshCache::default();
    let mut materials = Materials::load(&mut archive, &mut warnings);
    let error = |archive: &Archive, record: Record<'_>, detail: String| {
        archive.error(&entry, record.describe(&detail))
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
    let mut structure = VehicleStructure::default();
    let mut beam_defaults = structure::BeamDefaults::default();
    let mut advanced_deformation = false;
    let mut excludes_ground_contact = false;
    let mut vertex_overrides = BTreeSet::new();
    for &record in &records {
        let fields = record.fields();
        let result = match record.section {
            "minimass" | "globals" | "shocks" | "engine" | "brakes" => continue,
            "set_node_defaults" => {
                if let Some(flags) = fields.get(5) {
                    excludes_ground_contact = flags.contains('c');
                }
                Ok(())
            }
            "nodes" => {
                structure.nodes.push(StructuralNode {
                    excludes_ground_contact: excludes_ground_contact
                        || fields.get(4).is_some_and(|flags| flags.contains('c')),
                    ..Default::default()
                });
                Ok(())
            }
            "enable_advanced_deformation" => {
                advanced_deformation = true;
                Ok(())
            }
            "set_beam_defaults" => {
                if fields.len() > 8 {
                    warn(
                        &mut warnings,
                        &entry,
                        record.describe("ignoring extra fields after beam defaults"),
                    );
                }
                beam_defaults.read(&fields[1..], advanced_deformation)
            }
            "set_beam_defaults_scale" => beam_defaults.read_scale(&fields[1..]),
            "beams" => {
                if let [first, second, options @ ..] = fields.as_slice() {
                    let edge = resolve_nodes(&nodes, [first, second])
                        .map_err(|detail| error(&archive, record, detail))?;
                    vehicle.edges.push(edge);
                    let beam = beam_defaults
                        .beam(options)
                        .map_err(|detail| error(&archive, record, detail))?;
                    structure.beams.push(match beam {
                        Ok(beam) => Some(beam),
                        Err(detail) => {
                            warn(&mut warnings, &entry, record.describe(&detail));
                            None
                        }
                    });
                    Ok(())
                } else {
                    Err("expected two node IDs".into())
                }
            }
            "commands" | "commands2" => {
                let required = if record.section == "commands2" { 8 } else { 7 };
                let passive_link = (|| {
                    let [a, b, ..] = fields.as_slice() else {
                        return Err("expected command beam endpoints".into());
                    };
                    if fields.len() < required {
                        return Err("incomplete command beam record".into());
                    }
                    let edge = resolve_nodes(&nodes, [a, b])?;
                    for value in &fields[2..required] {
                        number(value)?;
                    }
                    let flags = fields.get(required).copied().unwrap_or("");
                    Ok::<_, String>((edge, beam_defaults.passive_command(flags)?))
                })();
                match passive_link {
                    Ok((edge, beam)) => {
                        vehicle.edges.push(edge);
                        structure.beams.push(match beam {
                            Ok(beam) => {
                                deferred.insert("command beam actuation");
                                Some(beam)
                            }
                            Err(detail) => {
                                warn(&mut warnings, &entry, record.describe(&detail));
                                None
                            }
                        });
                    }
                    Err(detail) => warn(&mut warnings, &entry, record.describe(&detail)),
                }
                Ok(())
            }
            "cab" => {
                if let [first, second, third, options @ ..] = fields.as_slice()
                    && options.len() <= 1
                {
                    let triangle = resolve_nodes(&nodes, [first, second, third])
                        .map_err(|detail| error(&archive, record, detail))?;
                    let flags = options.first().copied().unwrap_or("");
                    if flags
                        .chars()
                        .any(|flag| matches!(flag, 'c' | 'p' | 'u' | 'D' | 'F' | 'S'))
                    {
                        structure.collision_triangles.push(triangle);
                    }
                    if flags
                        .chars()
                        .any(|flag| !matches!(flag, 'n' | 'c' | 'p' | 'u'))
                    {
                        warn(
                            &mut warnings,
                            &entry,
                            record.describe(&format!("unsupported cab options {flags:?}")),
                        );
                    }
                    Ok(())
                } else {
                    Err("expected three node IDs and optional cab flags".into())
                }
            }
            "contacters" => {
                if let [node] = fields.as_slice() {
                    resolve_nodes(&nodes, [node]).map(|[id]| structure.contact_nodes.push(id))
                } else {
                    Err("expected one contact node ID".into())
                }
            }
            "forset" => {
                if let Some(binding) = structure.deformable_meshes.last_mut() {
                    structure::influence_nodes(&fields[1..], &nodes).map(|influences| {
                        for node in influences {
                            if !binding.influence_nodes.contains(&node) {
                                binding.influence_nodes.push(node);
                            }
                        }
                    })
                } else {
                    Err("expected a preceding flexbody".into())
                }
            }
            "forvert" => {
                if let Some(binding) = structure.deformable_meshes.last() {
                    vertex_overrides.insert(binding.object);
                }
                warn(
                    &mut warnings,
                    &entry,
                    record.describe("unsupported per-vertex flexbody binding"),
                );
                Ok(())
            }
            "meshwheels2" => add_wheel(
                &mut archive,
                &entry,
                &mut meshes,
                &mut vehicle,
                &nodes,
                &fields,
            )
            .map(|wheel| wheel_details.push(wheel)),
            "flexbodies" | "props" => {
                if fields.first() == Some(&"add_animation") {
                    deferred.insert("prop animation");
                    continue;
                }
                let object = vehicle.objects.len();
                add_attachment(
                    &mut archive,
                    &entry,
                    &mut meshes,
                    &mut vehicle,
                    &nodes,
                    record.section,
                    &fields,
                )
                .map(|placement_nodes| {
                    if record.section == "flexbodies" {
                        body_objects.push(object);
                        structure.deformable_meshes.push(DeformableMeshBinding {
                            object,
                            placement_nodes,
                            influence_nodes: Vec::new(),
                        });
                    }
                })
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
                "vehicle import skips {}",
                deferred.into_iter().collect::<Vec<_>>().join(", ")
            ),
        );
    }

    vehicle.meshes = meshes.geometry;
    structure.deformable_meshes.retain(|binding| {
        !binding.influence_nodes.is_empty() && !vertex_overrides.contains(&binding.object)
    });
    vehicle.physics = dynamics::convert(
        &records,
        &vehicle,
        &nodes,
        wheel_details,
        &body_objects,
        &mut structure.nodes,
    );
    if structure.beams.iter().any(Option::is_some)
        || !structure.collision_triangles.is_empty()
        || !structure.contact_nodes.is_empty()
        || !structure.deformable_meshes.is_empty()
        || structure
            .nodes
            .iter()
            .any(|node| node.mass_kg.is_some() || node.excludes_ground_contact)
    {
        vehicle.physics.structure = Some(structure);
    }
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
        let directive = value
            .split(|ch: char| ch == ',' || ch.is_whitespace())
            .next();
        if let Some(
            directive @ ("set_node_defaults"
            | "set_beam_defaults"
            | "set_beam_defaults_scale"
            | "enable_advanced_deformation"
            | "forset"
            | "forvert"),
        ) = directive
        {
            records.push(Record {
                section: directive,
                line,
                value,
            });
            continue;
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
) -> Result<[NodeId; 3], String> {
    let [reference, x, y, tx, ty, tz, rx, ry, rz, mesh, extra @ ..] = fields else {
        return Err("expected reference nodes, offset, rotation, and mesh".into());
    };
    let placement_nodes = resolve_nodes(nodes, [reference, x, y])?;
    let [reference, x, y] = placement_nodes.map(|id| vehicle.nodes[id.0]);
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
    Ok(placement_nodes)
}

fn add_wheel(
    archive: &mut Archive,
    entry: &str,
    meshes: &mut MeshCache,
    vehicle: &mut VehicleAsset,
    nodes: &HashMap<u32, NodeId>,
    fields: &[&str],
) -> Result<WheelRecord, String> {
    let [
        radius,
        rim,
        width,
        rays,
        first,
        second,
        _,
        brake,
        drive,
        _,
        mass,
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
    let axle_nodes = resolve_nodes(nodes, [first, second])?;
    let [first, second] = axle_nodes.map(|id| vehicle.nodes[id.0]);
    let direction = second - first;
    if direction.x == 0.0 {
        return Err("wheel axle has no lateral span".into());
    }
    let axle = direction.normalize() * direction.x.signum();
    let centre = (first + second) * 0.5;
    let side = match *side {
        "l" | "L" => WheelSide::Left,
        "r" | "R" => WheelSide::Right,
        _ => return Err(format!("invalid wheel side {side:?}")),
    };
    let mode = |value: &str| {
        let mode = number(value).ok()?;
        ((0.0..=255.0).contains(&mode) && mode.fract() == 0.0).then_some(mode as u8)
    };
    let wheel = WheelRecord {
        physics: WheelPhysics {
            attachment: None,
            side,
            mass_kg: number(mass).ok().filter(|mass| *mass > 0.0),
            upright_mass_kg: None,
            suspension: None,
            drive_torque_nm: None,
            brake_torque_nm: None,
            rim_object: vehicle.objects.len(),
            tyre_object: vehicle.objects.len() + 1,
        },
        axle_nodes,
        drive: mode(drive),
        brake: mode(brake),
    };
    let out = match side {
        WheelSide::Left => axle,
        WheelSide::Right => -axle,
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
    Ok(wheel)
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

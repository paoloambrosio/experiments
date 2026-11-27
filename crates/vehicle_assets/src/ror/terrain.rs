use std::{
    collections::{BTreeMap, HashMap},
    path::Path,
};

use bevy_math::{Quat, Vec3};

use super::{
    archive::{Archive, Import, ImportError, ImportWarning, warn},
    materials::{Materials, read_image},
    ogre,
};
use crate::{Heightfield, MeshAppearance, MeshInstance, TerrainAsset};

#[derive(Debug)]
pub struct TerrainImport {
    pub terrain: TerrainAsset,
    pub start_position: Option<Vec3>,
    pub appearance: MeshAppearance,
    pub warnings: Vec<ImportWarning>,
}

/// Loads one static terrain and its resources directly from a RoR archive.
pub fn load_terrain(path: &Path, entry: &str) -> Result<TerrainImport, ImportError> {
    let (mut archive, definition) = Archive::select(path, entry)?;
    import(&mut archive, &definition)
}

pub(super) fn load_embedded(
    path: &Path,
    bytes: &'static [u8],
    entry: &str,
) -> Import<TerrainImport> {
    let (mut archive, definition) = Archive::select_embedded(path, bytes, entry)?;
    import(&mut archive, &definition)
}

fn import(archive: &mut Archive, entry: &str) -> Import<TerrainImport> {
    let mut warnings = Vec::new();
    let text = archive.text(entry)?;
    let ini = parse_ini(&text);
    let general = |key: &str| {
        ini.get("general").and_then(|properties| {
            properties
                .iter()
                .find(|(k, _)| k.eq_ignore_ascii_case(key))
                .map(|(_, value)| *value)
        })
    };
    let start_position = general("StartPosition")
        .map(|value| {
            let fields: Vec<_> = value
                .split(|c: char| c.is_whitespace() || c == ',')
                .filter(|field| !field.is_empty())
                .collect();
            match fields.as_slice() {
                [x, y, z] => numbers([x, y, z]),
                _ => Err("expected three coordinates".into()),
            }
            .map_err(|error| archive.error(entry, format!("StartPosition: {error}")))
        })
        .transpose()?;
    let geometry =
        general("GeometryConfig").ok_or_else(|| archive.error(entry, "missing GeometryConfig"))?;
    let height_entry = archive.reference(entry, geometry)?;
    let heightfield = read_heightfield(archive, &height_entry, &mut warnings)?;
    if general("CaelumConfigFile").is_some() {
        warn(
            &mut warnings,
            entry,
            "authored sky and lighting are skipped",
        );
    }
    if general("WaterLine").is_some() {
        warn(&mut warnings, entry, "authored water is skipped");
    }
    if ini
        .get("scripts")
        .is_some_and(|scripts| !scripts.is_empty())
    {
        warn(&mut warnings, entry, "terrain scripts are skipped");
    }

    let mut terrain = TerrainAsset {
        name: general("Name").unwrap_or(entry).to_owned(),
        heightfield: Some(heightfield),
        meshes: Vec::new(),
        objects: Vec::new(),
    };
    let mut meshes = ogre::MeshCache::default();
    let mut odefs = HashMap::new();
    for (list, _) in ini.get("objects").into_iter().flatten() {
        let list = archive.reference(entry, list)?;
        for (line, value) in archive.text(&list)?.lines().enumerate() {
            let value = content(value);
            if value.is_empty() {
                continue;
            }
            if value.eq_ignore_ascii_case("end") {
                break;
            }
            let fields: Vec<_> = value.splitn(7, ',').map(str::trim).collect();
            let [x, y, z, rx, ry, rz, name] = fields.as_slice() else {
                return Err(archive.error(
                    &list,
                    format!("line {}: expected a static object placement", line + 1),
                ));
            };
            let name = name.split_whitespace().next().ok_or_else(|| {
                archive.error(&list, format!("line {}: missing object name", line + 1))
            })?;
            let translation = numbers([x, y, z])
                .map_err(|error| archive.error(&list, format!("line {}: {error}", line + 1)))?;
            let rotation = numbers([rx, ry, rz])
                .map_err(|error| archive.error(&list, format!("line {}: {error}", line + 1)))?;
            let definition = archive.reference(&list, &format!("{name}.odef"))?;
            if !odefs.contains_key(&definition) {
                let object = read_object(archive, &definition, &mut warnings)?;
                odefs.insert(definition.clone(), object);
            }
            let object = &odefs[&definition];
            let rotation = Quat::from_rotation_x(rotation.x.to_radians())
                * Quat::from_rotation_y(rotation.y.to_radians())
                * Quat::from_rotation_z(rotation.z.to_radians());
            let rotation = if object.standard {
                rotation
            } else {
                rotation * Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2)
            };
            let mut import_mesh = |reference: &str| meshes.load(archive, &definition, reference);
            let visual_mesh = object.visual.as_deref().map(&mut import_mesh).transpose()?;
            let mut collision = object.collision.iter();
            let collision_mesh = collision
                .next()
                .map(|reference| import_mesh(reference))
                .transpose()?;
            if visual_mesh.is_some() || collision_mesh.is_some() {
                terrain.objects.push(MeshInstance {
                    name: name.to_owned(),
                    translation,
                    rotation,
                    scale: object.scale,
                    visual_mesh,
                    collision_mesh,
                });
            }
            // Each additional collision mesh shares the authored transform.
            for reference in collision {
                terrain.objects.push(MeshInstance {
                    name: name.to_owned(),
                    translation,
                    rotation,
                    scale: object.scale,
                    visual_mesh: None,
                    collision_mesh: Some(import_mesh(reference)?),
                });
            }
        }
    }

    terrain.meshes = meshes.geometry;
    terrain
        .validate()
        .map_err(|error| archive.error(entry, error.to_string()))?;
    let mut materials = Materials::load(archive, &mut warnings);
    for id in terrain
        .objects
        .iter()
        .filter_map(|instance| instance.visual_mesh)
    {
        let (path, sections) = &meshes.bindings[id.0];
        materials.bind(archive, id, path, sections, &mut warnings);
    }
    warnings.sort();
    warnings.dedup();
    Ok(TerrainImport {
        terrain,
        start_position,
        appearance: materials.appearance,
        warnings,
    })
}

fn read_heightfield(
    archive: &mut Archive,
    entry: &str,
    warnings: &mut Vec<ImportWarning>,
) -> Import<Heightfield> {
    let text = archive.text(entry)?;
    let ini = parse_ini(&text);
    let get = |key: &str| {
        ini.values()
            .flatten()
            .find(|(k, _)| k.eq_ignore_ascii_case(key))
            .map(|(_, value)| *value)
    };
    if ["Pages_X", "Pages_Y"]
        .iter()
        .any(|key| get(key).is_some_and(|value| value != "0"))
    {
        return Err(archive.error(entry, "only single-page terrain heightfields are supported"));
    }
    let extent = |key: &str| {
        get(key)
            .and_then(|value| value.parse::<f32>().ok())
            .filter(|value| value.is_finite() && *value > 0.0)
            .ok_or_else(|| archive.error(entry, format!("{key} must be a positive finite number")))
    };
    let (width, depth, amplitude) = (
        extent("WorldSizeX")?,
        extent("WorldSizeZ")?,
        extent("WorldSizeY")?,
    );
    let image = get("HeightmapImage.0.0")
        .ok_or_else(|| archive.error(entry, "missing HeightmapImage.0.0"))?;
    let image = archive.reference(entry, image)?;
    let heights = read_image(&archive.read(&image)?)
        .map_err(|error| archive.error(&image, error))?
        .to_rgb32f();
    let (samples_x, samples_z) = heights.dimensions();
    if samples_x < 2 || samples_z < 2 {
        return Err(archive.error(
            &image,
            "heightfield needs at least two samples along each axis",
        ));
    }
    if get("Layers.count").is_some_and(|value| value != "0") {
        warn(warnings, entry, "heightfield texture layers are skipped");
    }

    // RoR uses the image's red channel. Increasing image row corresponds
    // to increasing world Z for the XZ-aligned terrain.
    Ok(Heightfield {
        origin: Vec3::ZERO,
        samples_x: samples_x as usize,
        samples_z: samples_z as usize,
        spacing_x: width / (samples_x - 1) as f32,
        spacing_z: depth / (samples_z - 1) as f32,
        heights: heights
            .pixels()
            .map(|pixel| pixel.0[0] * amplitude)
            .collect(),
    })
}

struct Object {
    visual: Option<String>,
    collision: Vec<String>,
    scale: Vec3,
    standard: bool,
}

fn read_object(
    archive: &mut Archive,
    entry: &str,
    warnings: &mut Vec<ImportWarning>,
) -> Import<Object> {
    let text = archive.text(entry)?;
    let mut lines = text.lines().map(content).filter(|line| !line.is_empty());
    let visual = lines
        .next()
        .ok_or_else(|| archive.error(entry, "missing visual mesh declaration"))?;
    let scale: Vec<_> = lines
        .next()
        .ok_or_else(|| archive.error(entry, "missing object scale"))?
        .split(',')
        .map(str::trim)
        .collect();
    let [x, y, z] = scale.as_slice() else {
        return Err(archive.error(entry, "expected three object scale components"));
    };
    let scale = numbers([x, y, z]).map_err(|error| archive.error(entry, error))?;
    let mut object = Object {
        visual: (visual != "none").then(|| visual.to_owned()),
        collision: Vec::new(),
        scale,
        standard: false,
    };
    let mut collision = None;
    let mut inside_mesh = false;
    let mut inside_box = false;
    for line in lines {
        let (command, value) = line.split_once(char::is_whitespace).unwrap_or((line, ""));
        if inside_box && command != "endbox" {
            continue;
        }
        match command {
            "standard" => object.standard = true,
            "beginmesh" if !inside_mesh => inside_mesh = true,
            "mesh" if inside_mesh && !value.trim().is_empty() => {
                collision = Some(value.trim().to_owned())
            }
            "endmesh" if inside_mesh => {
                inside_mesh = false;
                object.collision.push(collision.take().ok_or_else(|| {
                    archive.error(entry, "mesh block missing its collision mesh")
                })?);
            }
            "beginbox" => {
                inside_box = true;
                warn(warnings, entry, "collision box commands are skipped");
            }
            "endbox" => inside_box = false,
            "stdfriction" | "usefriction" => {
                warn(warnings, entry, "surface friction is reserved for physics")
            }
            "end" => break,
            command => warn(
                warnings,
                entry,
                format!("object command {command} is skipped"),
            ),
        }
    }
    if inside_mesh || inside_box {
        return Err(archive.error(entry, "unterminated object collision block"));
    }
    if object.visual.is_none() && object.collision.is_empty() {
        warn(warnings, entry, "object has no supported static geometry");
    }
    Ok(object)
}

fn numbers(parts: [&str; 3]) -> Result<Vec3, String> {
    let (x, y, z) = (number(parts[0])?, number(parts[1])?, number(parts[2])?);
    Ok(Vec3::new(x, y, z))
}

fn number(value: &str) -> Result<f32, String> {
    value
        .parse::<f32>()
        .ok()
        .filter(|value| value.is_finite())
        .ok_or_else(|| format!("invalid number {value:?}"))
}

fn content(line: &str) -> &str {
    let line = line.split("//").next().unwrap_or(line).trim();
    if line.starts_with(['#', ';']) {
        ""
    } else {
        line
    }
}

fn parse_ini(input: &str) -> BTreeMap<String, Vec<(&str, &str)>> {
    let mut sections = BTreeMap::<_, Vec<_>>::new();
    let mut name = String::new();
    for line in input
        .trim_start_matches('\u{feff}')
        .lines()
        .map(content)
        .filter(|line| !line.is_empty())
    {
        if let Some(section) = line
            .strip_prefix('[')
            .and_then(|line| line.strip_suffix(']'))
        {
            name = section.trim().to_ascii_lowercase();
        } else {
            let (key, value) = line.split_once('=').unwrap_or((line, ""));
            sections
                .entry(name.clone())
                .or_default()
                .push((key.trim(), value.trim()));
        }
    }
    sections
}

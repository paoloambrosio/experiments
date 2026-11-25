use std::{collections::HashMap, ops::Range};

use crate::{
    DiffuseMaterial, DiffuseTexture, MaterialId, MaterialSection, MeshAppearance, MeshId, TextureId,
};

use super::{
    archive::{Archive, ImportWarning, warn},
    dds,
};

const FALLBACK: [f32; 4] = [0.85, 0.25, 0.65, 1.0];

#[derive(Default)]
pub(super) struct Materials {
    pub appearance: MeshAppearance,
    definitions: HashMap<String, Definition>,
    bound: HashMap<String, MaterialId>,
    decoded: HashMap<String, TextureId>,
}

struct Definition {
    entry: String,
    rgba: [f32; 4],
    double_sided: bool,
    transparent: bool,
    base: Option<String>,
    base_uses_uv0: bool,
    warnings: Vec<String>,
}

impl Materials {
    pub(super) fn load(archive: &mut Archive, warnings: &mut Vec<ImportWarning>) -> Self {
        let mut entries: Vec<_> = archive
            .names()
            .iter()
            .filter(|name| {
                name.rsplit_once('.')
                    .is_some_and(|(_, extension)| extension.eq_ignore_ascii_case("material"))
            })
            .cloned()
            .collect();
        entries.sort_unstable();
        let mut materials = Self::default();
        for entry in entries {
            match archive.text(&entry) {
                Ok(text) => materials.read_definitions(&entry, &text),
                Err(error) => warn(
                    warnings,
                    &entry,
                    format!("material script skipped: {error}"),
                ),
            }
        }
        materials
    }

    pub(super) fn define_managed(
        &mut self,
        entry: &str,
        name: &str,
        base: Option<&str>,
        transparent: bool,
    ) {
        self.definitions.insert(
            name.to_owned(),
            Definition {
                entry: entry.to_owned(),
                rgba: [1.0; 4],
                double_sided: transparent,
                transparent,
                base: base.map(str::to_owned),
                base_uses_uv0: true,
                warnings: Vec::new(),
            },
        );
    }

    fn read_definitions(&mut self, entry: &str, text: &str) {
        let mut blocks = Vec::new();
        let mut pending = "";
        let mut active = None;
        let mut techniques = 0;
        let mut passes = 0;
        let mut units = 0;

        // Statements are line-based within brace-delimited material blocks.
        let text = text
            .lines()
            .map(|line| line.split("//").next().unwrap_or(line))
            .collect::<Vec<_>>()
            .join("\n");
        let text = text.replace('{', "\n{\n").replace('}', "\n}\n");
        for statement in text.lines().map(str::trim).filter(|line| !line.is_empty()) {
            match statement {
                "{" => {
                    blocks.push(pending);
                    pending = "";
                    continue;
                }
                "}" => {
                    if blocks.pop() == Some("material") {
                        active = None;
                    }
                    pending = "";
                    continue;
                }
                _ => {}
            }
            let (command, args) = statement
                .split_once(char::is_whitespace)
                .unwrap_or((statement, ""));
            let args = args.trim();
            pending = command;
            if command == "material" {
                let name = args
                    .split(':')
                    .next()
                    .unwrap_or(args)
                    .trim()
                    .trim_matches('"');
                active = Some(name);
                self.definitions
                    .entry(name.to_owned())
                    .or_insert_with(|| Definition {
                        entry: entry.to_owned(),
                        rgba: [1.0; 4],
                        double_sided: false,
                        transparent: false,
                        base: None,
                        base_uses_uv0: true,
                        warnings: Vec::new(),
                    });
                techniques = 0;
                passes = 0;
                units = 0;
                continue;
            }
            let Some(definition) = active.and_then(|name| self.definitions.get_mut(name)) else {
                continue;
            };
            if definition.entry != entry {
                continue;
            }
            if command == "technique" {
                techniques += 1;
            }
            if techniques > 1 {
                continue;
            }
            if command == "pass" {
                passes += 1;
                if passes == 2 {
                    definition
                        .warnings
                        .push("additional material passes are skipped".into());
                }
            }
            if passes != 1 {
                continue;
            }
            match (blocks.last().copied(), command) {
                (Some("pass"), "texture_unit") => units += 1,
                (Some("pass"), "diffuse") => {
                    let values: Vec<_> = args
                        .split_whitespace()
                        .filter_map(|v| v.parse::<f32>().ok())
                        .collect();
                    if let [r, g, b, a] = values.as_slice()
                        && values.iter().all(|v| v.is_finite())
                    {
                        definition.rgba = [*r, *g, *b, *a];
                    } else if let [r, g, b] = values.as_slice()
                        && values.iter().all(|v| v.is_finite())
                    {
                        definition.rgba = [*r, *g, *b, 1.0];
                    }
                }
                (Some("pass"), "cull_hardware") => definition.double_sided = args == "none",
                (Some("texture_unit"), "texture") => {
                    if let Some(path) = args.split_whitespace().next() {
                        let path = path.trim_matches('"');
                        if units == 1 {
                            definition.base = Some(path.to_owned());
                        } else {
                            definition
                                .warnings
                                .push(format!("secondary texture {path} is skipped"));
                        }
                    }
                }
                (Some("texture_unit"), "tex_coord_set") if units == 1 => {
                    definition.base_uses_uv0 = args == "0";
                    if !definition.base_uses_uv0 {
                        definition.warnings.push(format!(
                            "base texture coordinates on UV set {args} are skipped"
                        ));
                    }
                }
                _ => {}
            }
        }
    }

    pub(super) fn bind(
        &mut self,
        archive: &mut Archive,
        mesh: MeshId,
        mesh_entry: &str,
        sections: &[(Range<usize>, String)],
        warnings: &mut Vec<ImportWarning>,
    ) {
        if self.appearance.sections.contains_key(&mesh) {
            return;
        }
        let sections = sections
            .iter()
            .map(|(triangles, name)| MaterialSection {
                triangles: triangles.clone(),
                material: self.material(archive, mesh_entry, name, warnings),
            })
            .collect();
        self.appearance.sections.insert(mesh, sections);
    }

    fn material(
        &mut self,
        archive: &mut Archive,
        mesh_entry: &str,
        name: &str,
        warnings: &mut Vec<ImportWarning>,
    ) -> MaterialId {
        if let Some(id) = self.bound.get(name) {
            return *id;
        }
        let mut material = DiffuseMaterial {
            rgba: FALLBACK,
            base_texture: None,
            double_sided: false,
            transparent: false,
        };
        if let Some(definition) = self.definitions.get(name) {
            material.rgba = definition.rgba;
            material.double_sided = definition.double_sided;
            material.transparent = definition.transparent;
            for message in &definition.warnings {
                warn(warnings, &definition.entry, message.clone());
            }
            if definition.base_uses_uv0
                && let Some(base) = &definition.base
            {
                let resolved = archive.resolve(&definition.entry, base);
                match resolved.and_then(|entry| {
                    load_texture(
                        archive,
                        &entry,
                        &mut self.decoded,
                        &mut self.appearance.textures,
                    )
                }) {
                    Ok(id) => material.base_texture = Some(id),
                    Err(detail) => {
                        material.rgba = FALLBACK;
                        warn(
                            warnings,
                            &definition.entry,
                            format!("base texture {base} uses fallback: {detail}"),
                        );
                    }
                }
            }
        } else if name == "renderdash" {
            material.rgba = [0.14, 0.15, 0.16, 1.0];
            warn(
                warnings,
                mesh_entry,
                "live dashboard texture is displayed as a flat material",
            );
        } else {
            warn(
                warnings,
                mesh_entry,
                format!("material {name} uses fallback: missing declaration"),
            );
        }

        let id = MaterialId(self.appearance.materials.len());
        self.appearance.materials.push(material);
        self.bound.insert(name.to_owned(), id);
        id
    }
}

fn load_texture(
    archive: &mut Archive,
    entry: &str,
    decoded: &mut HashMap<String, TextureId>,
    textures: &mut Vec<DiffuseTexture>,
) -> Result<TextureId, String> {
    if let Some(id) = decoded.get(entry) {
        return Ok(*id);
    }
    let bytes = archive.read(entry).map_err(|error| error.to_string())?;
    let image = read_image(&bytes)?.into_rgba8();
    let (width, height) = image.dimensions();
    let id = TextureId(textures.len());
    textures.push(DiffuseTexture {
        width,
        height,
        rgba8: image.into_raw(),
    });
    decoded.insert(entry.to_owned(), id);
    Ok(id)
}

pub(super) fn read_image(bytes: &[u8]) -> Result<image::DynamicImage, String> {
    if let Some(image) = dds::decode_rgb(bytes)? {
        return Ok(image::DynamicImage::ImageRgba8(image));
    }
    image::load_from_memory(bytes).map_err(|error| format!("image decode failed: {error}"))
}

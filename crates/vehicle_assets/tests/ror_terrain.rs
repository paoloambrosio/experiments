use std::{fs::File, io::Write, path::Path};

use bevy_math::Vec3;
use image::{ImageBuffer, ImageEncoder, Rgb, codecs::png::PngEncoder};
use vehicle_assets::ror::load_terrain;
use zip::{CompressionMethod, ZipWriter, write::SimpleFileOptions};

fn chunk(output: &mut Vec<u8>, id: u16, payload: &[u8]) {
    output.extend_from_slice(&id.to_le_bytes());
    output.extend_from_slice(&((payload.len() + 6) as u32).to_le_bytes());
    output.extend_from_slice(payload);
}

fn triangle_mesh(version: &str, materials: &[&str], height: f32) -> Vec<u8> {
    let mut mesh = vec![0];
    for (section, material) in materials.iter().enumerate() {
        let mut submesh = material.as_bytes().to_vec();
        submesh.extend_from_slice(&[b'\n', 0]);
        submesh.extend_from_slice(&3u32.to_le_bytes());
        let wide = section > 0;
        submesh.push(u8::from(wide));
        for index in [0u32, 1, 2] {
            let bytes = index.to_le_bytes();
            submesh.extend_from_slice(&bytes[..if wide { 4 } else { 2 }]);
        }

        let mut geometry = 3u32.to_le_bytes().to_vec();
        let mut declaration = Vec::new();
        for (ty, semantic, offset) in [(2u16, 1u16, 0u16), (2, 4, 12), (1, 7, 24)] {
            let element: Vec<_> = [0, ty, semantic, offset, 0]
                .into_iter()
                .flat_map(u16::to_le_bytes)
                .collect();
            chunk(&mut declaration, 0x5110, &element);
        }
        chunk(&mut geometry, 0x5100, &declaration);
        let mut buffer = vec![0, 0, 32, 0];
        let mut vertices = Vec::new();
        for [x, y, z] in [
            [1.0f32, 0.0, height],
            [2.0, 0.0, height],
            [1.0, 1.0, height],
        ] {
            let values = [x, y, z, 0.0, 0.0, 1.0, x - 1.0, y];
            vertices.extend(values.into_iter().flat_map(f32::to_le_bytes));
        }
        chunk(&mut buffer, 0x5210, &vertices);
        chunk(&mut geometry, 0x5200, &buffer);
        chunk(&mut submesh, 0x5000, &geometry);
        chunk(&mut submesh, 0x4010, &4u16.to_le_bytes());
        chunk(&mut mesh, 0x4000, &submesh);
    }
    let mut bytes = 0x1000u16.to_le_bytes().to_vec();
    bytes.extend_from_slice(format!("[MeshSerializer_v{version}]\n").as_bytes());
    chunk(&mut bytes, 0x3000, &mesh);
    bytes
}

fn write_zip(path: &Path, bad_mesh: bool, ambiguous: bool) {
    let img = ImageBuffer::from_fn(3, 2, |x, z| Rgb([(x * 30 + z * 90) as u8, 255, 0]));
    let mut png = Vec::new();
    PngEncoder::new(&mut png)
        .write_image(img.as_raw(), 3, 2, image::ExtendedColorType::Rgb8)
        .unwrap();

    let mut visual = triangle_mesh("1.41", &["floor", "paint"], 1.0);
    if bad_mesh {
        visual.truncate(visual.len() - 16);
    }
    let collision = triangle_mesh("1.40", &["collision"], 0.5);
    let mut archive = ZipWriter::new(File::create(path).unwrap());
    let options = SimpleFileOptions::default().compression_method(CompressionMethod::Deflated);
    let mut entry = |name: &str, payload: &[u8]| {
        archive.start_file(name, options).unwrap();
        archive.write_all(payload).unwrap();
    };
    entry("pack/Test.terrn2", b"[General]\nName = Test track\nGeometryConfig = geometry/Sim.otc\n[Objects]\nlayout.tobj=\n");
    entry("pack/geometry/sim.otc", b"Pages_X=0\nPages_Y=0\nWorldSizeX=12\nWorldSizeZ=9\nWorldSizeY=255\nHeightmapImage.0.0=Field.png\n");
    entry("pack/geometry/field.png", &png);
    if ambiguous {
        entry("pack/geometry/FIELD.PNG", &png);
    }
    entry(
        "pack/layout.tobj",
        b"10,2,20,0,90,0,objects/ramp\n2,0,1,0,0,0,objects/marker\n",
    );
    entry(
        "pack/objects/ramp.odef",
        b"ramp.mesh\n2,3,4\nbeginmesh\nmesh contact.mesh\nendmesh\nend\n",
    );
    entry(
        "pack/objects/marker.odef",
        b"ramp.mesh\n1,1,1\nstandard\nend\n",
    );
    entry("pack/objects/ramp.mesh", &visual);
    entry("pack/objects/contact.mesh", &collision);
    entry("pack/objects/base.png", &png);
    entry("pack/objects/track.material", b"material floor\n{\n technique\n {\n  pass\n  {\n   cull_hardware none\n   diffuse 0.2 0.3 0.4 1\n   texture_unit\n   {\n    texture base.png\n   }\n   texture_unit\n   {\n    texture shading.png\n   }\n  }\n }\n}\nmaterial paint\n{\n technique\n {\n  pass\n  {\n   texture_unit\n   {\n    texture missing.png\n   }\n  }\n }\n}\n");
    archive.finish().unwrap();
}

#[test]
fn imports_compressed_terrain_with_distinct_collision_and_visual_geometry() {
    let dir = tempfile::tempdir().unwrap();
    let zip = dir.path().join("terrain.zip");
    write_zip(&zip, false, false);

    let imported = load_terrain(&zip, "pack/Test.terrn2").unwrap();
    let terrain = imported.terrain;
    assert_eq!(terrain.name, "Test track");
    let grid = terrain.heightfield.unwrap();
    assert_eq!((grid.samples_x, grid.samples_z), (3, 2));
    assert!(
        grid.sample_position(2, 1)
            .unwrap()
            .abs_diff_eq(Vec3::new(12.0, 150.0, 9.0), 0.001)
    );
    assert!(
        grid.sample_position(1, 0)
            .unwrap()
            .abs_diff_eq(Vec3::new(6.0, 30.0, 0.0), 0.001)
    );

    assert_eq!(terrain.meshes.len(), 2);
    let ramp = &terrain.objects[0];
    let id = ramp.visual_mesh.unwrap();
    let geometry = &terrain.meshes[id.0];
    assert_eq!(geometry.triangles, [[0, 1, 2], [3, 4, 5]]);
    assert_eq!(geometry.normals.as_ref().unwrap()[0], Vec3::Z);
    assert!(
        ramp.transform_point(geometry.vertices[0])
            .abs_diff_eq(Vec3::new(10.0, 6.0, 18.0), 0.001)
    );
    let contact = &terrain.meshes[ramp.collision_mesh.unwrap().0];
    assert!(
        ramp.transform_point(contact.vertices[0])
            .abs_diff_eq(Vec3::new(10.0, 4.0, 18.0), 0.001)
    );

    let marker = &terrain.objects[1];
    assert_eq!(marker.visual_mesh, Some(id));
    assert_eq!(marker.collision_mesh, None);
    assert_eq!(
        marker.transform_point(geometry.vertices[0]),
        Vec3::new(3.0, 0.0, 2.0)
    );

    let sections = &imported.appearance.sections[&id];
    assert_eq!(sections[0].triangles, 0..1);
    assert_eq!(sections[1].triangles, 1..2);
    let floor = &imported.appearance.materials[sections[0].material.0];
    assert!(floor.double_sided);
    assert_eq!(floor.rgba, [0.2, 0.3, 0.4, 1.0]);
    assert!(floor.base_texture.is_some());
    assert!(
        imported.appearance.materials[sections[1].material.0]
            .base_texture
            .is_none()
    );
    assert!(
        imported
            .warnings
            .iter()
            .any(|warning| warning.message.contains("missing.png"))
    );
    assert!(
        imported
            .warnings
            .iter()
            .any(|warning| warning.message.contains("secondary texture shading.png"))
    );
}

#[test]
fn reports_a_reference_with_multiple_case_insensitive_matches() {
    let dir = tempfile::tempdir().unwrap();
    let zip = dir.path().join("terrain.zip");
    write_zip(&zip, false, true);

    let error = load_terrain(&zip, "pack/Test.terrn2")
        .unwrap_err()
        .to_string();
    assert!(error.contains("pack/geometry/sim.otc"), "{error}");
    assert!(
        error.contains("ambiguous archive reference \"Field.png\""),
        "{error}"
    );
}

#[test]
fn reports_truncated_geometry_with_the_referring_definition() {
    let dir = tempfile::tempdir().unwrap();
    let zip = dir.path().join("terrain.zip");
    write_zip(&zip, true, false);

    let error = load_terrain(&zip, "pack/Test.terrn2")
        .unwrap_err()
        .to_string();
    assert!(error.contains("pack/objects/ramp.mesh"), "{error}");
    assert!(
        error.contains("referenced by pack/objects/ramp.odef"),
        "{error}"
    );
    assert!(error.contains("truncated OGRE mesh"), "{error}");
}

#[test]
#[ignore = "requires the local CM 0.5.zip archive"]
fn local_facility_matches_its_authored_layout() {
    let archive = std::env::var_os("ROR_CM_ARCHIVE").expect("set ROR_CM_ARCHIVE to CM 0.5.zip");
    let imported = load_terrain(Path::new(&archive), "CM.terrn2").unwrap();
    let terrain = &imported.terrain;
    assert_eq!(terrain.name, "N-Labs Testing Facility 0.5");
    let grid = terrain.heightfield.as_ref().unwrap();
    assert_eq!((grid.samples_x, grid.samples_z), (5, 5));
    assert_eq!((grid.spacing_x, grid.spacing_z), (128.0, 128.0));
    assert_eq!(terrain.objects.len(), 16);
    assert_eq!(terrain.meshes.len(), 18);
    assert_eq!(
        terrain
            .objects
            .iter()
            .filter(|instance| instance.collision_mesh.is_some())
            .count(),
        15
    );

    let instance = |name| {
        terrain
            .objects
            .iter()
            .find(|instance| instance.name == name)
            .unwrap()
    };
    let floor = instance("CMFloor");
    let visual = floor.visual_mesh.unwrap();
    let collision = floor.collision_mesh.unwrap();
    assert_ne!(visual, collision);
    let bounds = |id: vehicle_assets::MeshId| {
        terrain.meshes[id.0]
            .vertices
            .iter()
            .map(|&point| floor.transform_point(point))
            .fold(
                (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY)),
                |(min, max), point| (min.min(point), max.max(point)),
            )
    };
    let (visual_min, visual_max) = bounds(visual);
    let (collision_min, collision_max) = bounds(collision);
    assert!((visual_max.y - visual_min.y).abs() < 0.01);
    assert!(
        visual_min.abs_diff_eq(collision_min, 0.2),
        "{visual_min:?} {collision_min:?}"
    );
    assert!(
        visual_max.abs_diff_eq(collision_max, 0.2),
        "{visual_max:?} {collision_max:?}"
    );
    for name in ["CMRamp1", "CMRamp2", "CMRamp3"] {
        let ramp = instance(name);
        assert_eq!(ramp.visual_mesh, ramp.collision_mesh);
        assert!(ramp.visual_mesh.is_some());
    }
    assert!(instance("CMRuler").collision_mesh.is_none());
    let material = imported.appearance.sections[&visual][0].material;
    let base = imported.appearance.materials[material.0]
        .base_texture
        .unwrap();
    let base = &imported.appearance.textures[base.0];
    assert_eq!((base.width, base.height), (512, 512));
    assert!(
        imported
            .warnings
            .iter()
            .any(|warning| warning.message.contains("CMFloor_TEXT_AO.png"))
    );
}

use std::{fs::File, io::Write, path::Path};

use bevy_math::Vec3;
use image::{ImageBuffer, ImageEncoder, Rgb, codecs::png::PngEncoder};
use vehicle_assets::ror::load_terrain;
use zip::{CompressionMethod, ZipWriter, write::SimpleFileOptions};

mod common;

use common::triangle_mesh;

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
fn reference_track_preserves_the_level_start_and_authored_collision_placements() {
    let zip = Path::new(env!("CARGO_MANIFEST_DIR")).join("reference/track.zip");
    let imported = load_terrain(&zip, "track.terrn2").unwrap();
    let terrain = &imported.terrain;
    assert_eq!(imported.start_position, Some(Vec3::new(34.0, 3.0, 10.0)));
    let grid = terrain.heightfield.as_ref().unwrap();
    assert_eq!((grid.samples_x, grid.samples_z), (2, 2));
    assert_eq!((grid.spacing_x, grid.spacing_z), (90.0, 130.0));
    assert!(grid.heights.iter().all(|&h| h == 0.0));

    let [ramp, wall, line] = terrain.objects.as_slice() else {
        panic!("expected the ramp, wall, and start line");
    };
    assert_eq!(ramp.name, "ramp");
    assert_eq!(ramp.visual_mesh, ramp.collision_mesh);
    assert!(ramp.collision_mesh.is_some());
    assert_eq!(wall.name, "wall");
    let visual = wall.visual_mesh.unwrap();
    let collision = wall.collision_mesh.unwrap();
    assert_ne!(visual, collision);
    assert_eq!(
        terrain.meshes[visual.0].vertices,
        terrain.meshes[collision.0].vertices
    );
    assert!(
        wall.transform_point(Vec3::new(4.0, 2.5, 0.0))
            .abs_diff_eq(Vec3::new(65.9611, 2.5, 59.4433), 1e-4)
    );
    assert_eq!(line.name, "start-line");
    assert!(line.collision_mesh.is_none());
    let [section] = imported.appearance.sections[&visual].as_slice() else {
        panic!("expected one wall material");
    };
    let texture = imported.appearance.materials[section.material.0]
        .base_texture
        .unwrap();
    let texture = &imported.appearance.textures[texture.0];
    assert_eq!((texture.width, texture.height), (2, 2));
    assert!(imported.warnings.is_empty(), "{:?}", imported.warnings);
}

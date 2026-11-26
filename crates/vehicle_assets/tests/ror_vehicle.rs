mod common;

use std::{fs::File, io::Write, path::Path};

use bevy_math::Vec3;
use vehicle_assets::ror::load_vehicle;
use zip::{CompressionMethod, ZipWriter, write::SimpleFileOptions};

fn dds(pixel: &[u8]) -> Vec<u8> {
    let mut bytes = vec![0; 128];
    bytes[..4].copy_from_slice(b"DDS ");
    let alpha = pixel.len() == 4;
    for (offset, value) in [
        (4, 124u32),
        (8, 0x100f),
        (12, 1),
        (16, 1),
        (20, pixel.len() as u32),
        (76, 32),
        (80, if alpha { 0x41 } else { 0x40 }),
        (88, pixel.len() as u32 * 8),
        (92, 0x00ff0000),
        (96, 0x0000ff00),
        (100, 0x000000ff),
        (104, if alpha { 0xff000000 } else { 0 }),
        (108, 0x1000),
    ] {
        bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }
    bytes.extend_from_slice(pixel);
    bytes
}

fn write_fixture(path: &Path, bad_reference: bool) {
    let truck = "Fixture
nodes
10,0,0,0
20,0,0,2
30,-1,0,0
80,-1.2,0.4,0.7
70,-1.2,0.4,0.9
beams
10,30
meshwheels2
0.32,0.18,0.2,12,70,80,9999,1,1,30,2,1000,100,l,rim.mesh,tread
flexbodies
10,20,30,0.5,0.25,0.3,0,0,90,body.mesh
forset 10,20,30
managedmaterials
paint flexmesh_standard body.dds
glass flexmesh_transparent glass.dds
tread mesh_standard absent.dds
end
";
    let truck = if bad_reference {
        truck.replace("10,30\n", "10,999\n")
    } else {
        truck.to_owned()
    };
    let mut zip = ZipWriter::new(File::create(path).unwrap());
    let opts = SimpleFileOptions::default().compression_method(CompressionMethod::Deflated);
    for (name, data) in [
        ("pack/fixture.truck", truck.into_bytes()),
        (
            "pack/Body.mesh",
            common::triangle_mesh("1.30", &["paint", "glass"], 0.0),
        ),
        (
            "pack/rim.mesh",
            common::triangle_mesh("1.30", &["paint"], 0.0),
        ),
        ("pack/body.dds", dds(&[0x44, 0x33, 0x22])),
        ("pack/glass.dds", dds(&[0x11, 0x22, 0x33, 0x50])),
    ] {
        zip.start_file(name, opts).unwrap();
        zip.write_all(&data).unwrap();
    }
    zip.finish().unwrap();
}

#[test]
fn imports_the_rest_pose_wheel_basis_and_dds_materials() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("fixture.zip");
    write_fixture(&path, false);

    let imported = load_vehicle(&path, "pack/fixture.truck").unwrap();
    let vehicle = &imported.vehicle;
    assert_eq!(vehicle.nodes.len(), 5);
    assert_eq!(vehicle.nodes[2], Vec3::Z);
    let [a, b] = vehicle.edges[0];
    assert_eq!((a.0, b.0), (0, 2));
    let wheel = &vehicle.wheels[0];
    assert!(wheel.centre.abs_diff_eq(Vec3::new(0.8, 0.4, 1.2), 1e-6));
    assert_eq!(wheel.axle, Vec3::X);
    assert_eq!(wheel.width, 0.2);
    assert_eq!(wheel.rim_radius, Some(0.18));
    let detail = vehicle.physics.wheels[0];
    assert_eq!(detail.side, vehicle_assets::WheelSide::Left);
    assert_eq!(detail.mass_kg, Some(2.0));
    assert!(detail.drive_torque_nm.is_none());
    assert!(detail.brake_torque_nm.is_none());
    assert!(detail.suspension.is_none());
    assert!(vehicle.physics.chassis.is_none());
    assert_eq!(vehicle.objects[detail.rim_object].name, "rim.mesh");
    assert_eq!(vehicle.objects[detail.tyre_object].name, "tread");

    let body = vehicle
        .objects
        .iter()
        .find(|object| object.name == "body.mesh")
        .unwrap();
    assert!(
        body.translation
            .abs_diff_eq(Vec3::new(1.0, 0.3, 0.25), 1e-6)
    );
    assert!((body.rotation * Vec3::X).abs_diff_eq(Vec3::Y, 1e-6));
    let sections = &imported.appearance.sections[&body.visual_mesh.unwrap()];
    let material = |section: usize| &imported.appearance.materials[sections[section].material.0];
    let pixels =
        |section| &imported.appearance.textures[material(section).base_texture.unwrap().0].rgba8;
    assert_eq!(pixels(0), &[0x22, 0x33, 0x44, 255]);
    assert_eq!(pixels(1), &[0x33, 0x22, 0x11, 0x50]);
    assert!(material(1).transparent);
    assert!(
        imported
            .warnings
            .iter()
            .any(|warning| warning.message.contains("absent.dds"))
    );
}

#[test]
fn reports_missing_beam_node_with_file_section_and_line() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("fixture.zip");
    write_fixture(&path, true);
    let error = load_vehicle(&path, "pack/fixture.truck")
        .unwrap_err()
        .to_string();
    assert!(
        error.contains("fixture.truck: beams line 9: unknown node 999"),
        "{error}"
    );
}

#[test]
#[ignore = "requires the local accord23.zip archive"]
fn local_accord_variants_keep_their_authored_geometry_and_textures() {
    let archive =
        std::env::var_os("ROR_ACCORD_ARCHIVE").expect("set ROR_ACCORD_ARCHIVE to accord23.zip");
    let values = [
        ("accord.truck", 0.310, 0.190, 0.205, 0.205),
        ("accordb.truck", 0.315, 0.241, 0.245, 0.245),
        ("accordc.truck", 0.320, 0.241, 0.225, 0.225),
        ("accordd.truck", 0.315, 0.200, 0.225, 0.245),
    ];
    let approx = |observed: f32, expected: f32| {
        assert!(
            (observed - expected).abs() < 1e-4,
            "expected {expected}, got {observed}"
        )
    };
    for (entry, radius, rim, front_width, rear_width) in values {
        let imported = load_vehicle(Path::new(&archive), entry).unwrap();
        let vehicle = &imported.vehicle;
        let [rf, lf, rr, lr] = vehicle.wheels.as_slice() else {
            panic!("{entry} did not import four wheels");
        };
        for wheel in &vehicle.wheels {
            assert!(wheel.axle.x > 0.99);
            approx(wheel.outer_radius, radius);
            approx(wheel.rim_radius.unwrap(), rim);
        }
        for wheel in [rf, lf] {
            approx(wheel.width, front_width);
        }
        for wheel in [rr, lr] {
            approx(wheel.width, rear_width);
        }
        if entry == "accord.truck" {
            assert_eq!(vehicle.nodes.len(), 281);
            assert_eq!(vehicle.edges.len(), 1767);
            approx(lf.centre.x - rf.centre.x, 1.375);
            approx(lr.centre.x - rr.centre.x, 1.395);
            approx(lf.centre.z - lr.centre.z, 2.600);
            let physics = &vehicle.physics;
            let chassis = physics.chassis.expect("rigid chassis properties");
            assert!(chassis.size_m.min_element() > 0.0);
            assert!(chassis.centre_of_mass_m.x.abs() < 0.03);
            assert_eq!(physics.wheels.len(), 4);
            for (i, detail) in physics.wheels.iter().enumerate() {
                approx(detail.mass_kg.unwrap(), 39.0);
                let [spring, damper, rise, run] = if i < 2 {
                    [38000.0, 2650.0, 0.38, 0.3575_f32.hypot(0.05)]
                } else {
                    [31000.0, 2050.0, 0.444, 0.27]
                };
                let ratio_sq = rise * rise / (rise * rise + run * run);
                let suspension = detail.suspension.expect("wheel shock");
                assert!((suspension.stiffness_n_per_m - spring * ratio_sq).abs() < 0.01);
                assert!((suspension.damping_ns_per_m - damper * ratio_sq).abs() < 0.01);
                approx(suspension.initial_length_m, rise);
                assert!(suspension.free_length_m > suspension.initial_length_m);
                assert!(detail.upright_mass_kg.unwrap() > 18.0);
                approx(detail.brake_torque_nm.unwrap(), 1800.0);
                approx(
                    detail.drive_torque_nm.unwrap(),
                    if i < 2 {
                        170.0 * 4.066 * 3.181 / 2.0
                    } else {
                        0.0
                    },
                );
            }
        }
        let rim_objects = vehicle.objects.iter().filter(|object| {
            object.name.starts_with("accordwheel") && object.name.ends_with(".mesh")
        });
        for (rim, wheel) in rim_objects.zip(&vehicle.wheels) {
            assert!((rim.rotation * Vec3::X).x * wheel.centre.x > 0.0);
        }
        assert!(
            vehicle
                .objects
                .iter()
                .any(|object| object.name == "accordsteer.mesh")
        );
        assert!(
            imported
                .appearance
                .materials
                .iter()
                .any(|m| m.transparent && m.base_texture.is_some())
        );
        assert!(
            !imported
                .warnings
                .iter()
                .any(|w| w.message.contains("uses fallback")),
            "{:?}",
            imported.warnings
        );
    }
}

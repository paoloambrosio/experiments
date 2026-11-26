mod common;

use std::{fmt::Write as _, fs::File, io::Write, path::Path};

use bevy_math::Vec3;
use vehicle_assets::{NodeId, StructuralBeamKind, ror::load_vehicle};
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
    write_archive(path, &truck);
}

fn write_archive(path: &Path, truck: &str) {
    let mut zip = ZipWriter::new(File::create(path).unwrap());
    let opts = SimpleFileOptions::default().compression_method(CompressionMethod::Deflated);
    for (name, data) in [
        ("pack/fixture.truck", truck.as_bytes().to_vec()),
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
fn imports_shock_attachments_in_shuffled_wheel_order() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("attachments.zip");
    let corners = [(-0.7, -1.2), (0.7, 1.2), (0.7, -1.2), (-0.7, 1.2)];
    let mut truck = String::from("Attachments\nglobals\n600,0\nminimass\n2\nnodes\n");
    for (i, &(x, z)) in corners.iter().enumerate() {
        let n = 3 * i;
        writeln!(truck, "{n},{},{},{}", -z, 0.3, x - 0.1).unwrap();
        writeln!(truck, "{},{},{},{}", n + 1, -z, 0.3, x + 0.1).unwrap();
        writeln!(truck, "{},{},{},{}", n + 2, -z, 0.7, x).unwrap();
    }
    truck.push_str("beams\n2,5\n2,8,s\n5,8\n2,11\n5,11\n8,11\nshocks\n");
    for i in 0..4 {
        writeln!(truck, "{},{},32000,2000,0.2,0.2,1.1,i", 3 * i + 2, 3 * i).unwrap();
    }
    truck.push_str("meshwheels2\n");
    for (i, &(x, _)) in corners.iter().enumerate() {
        let (side, outer, inner) = if x > 0.0 {
            ('l', 3 * i + 1, 3 * i)
        } else {
            ('r', 3 * i, 3 * i + 1)
        };
        writeln!(
            truck,
            "0.32,0.18,0.2,12,{outer},{inner},9999,1,1,2,30,1000,100,{side},rim.mesh,tread"
        )
        .unwrap();
    }
    truck.push_str(
        "submesh\ncab\n2,5,8,n\nflexbodies\n2,5,8,0,0,0,0,0,0,body.mesh\nforset 2,5,8,11\nend\n",
    );
    write_archive(&path, &truck);
    let asset = load_vehicle(&path, "pack/fixture.truck").unwrap().vehicle;
    let structure = asset.physics.structure.unwrap();
    for (i, (wheel, &(x, z))) in asset.wheels.iter().zip(&corners).enumerate() {
        assert!(wheel.centre.abs_diff_eq(Vec3::new(x, 0.3, z), 1e-6));
        let attachment = asset.physics.wheels[i].attachment.unwrap();
        assert_eq!(attachment.upper_node, NodeId(3 * i + 2));
        assert_eq!(attachment.lower_node, NodeId(3 * i));
        assert_eq!(structure.nodes[attachment.upper_node.0].upright, None);
        for axle in attachment.axle_nodes {
            assert_eq!(structure.nodes[axle.0].upright, Some(i));
            assert!(structure.nodes[axle.0].mass_kg.unwrap() > 0.0);
        }
    }
    assert!(matches!(
        structure.beams[1].unwrap().kind,
        StructuralBeamKind::CompressionSupport { .. }
    ));
}

#[test]
fn imports_structural_properties_in_authored_order() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("structure.zip");
    write_archive(
        &path,
        "Structure
globals
0,8
minimass
2
nodes
10,0,0,0,l 4
set_node_defaults -1,-1,-1,-1,c
20,0,0,2
set_node_defaults -1,-1,-1,-1,n
30,-1,0,0,l
31,-1,1,0,c
32,-1,0,1
beams
10,20
set_beam_defaults 2000,80,120,8000,0.05,tracks/beam,0.25
10,30,i
enable_advanced_deformation
set_beam_defaults_scale 0.5,0.25,0.5,0.75
20,30,i
set_beam_defaults 3200
20,31,is 2.5
set_beam_defaults_scale 1,1,1,1
set_beam_defaults -1,-1,-1,-1,0.05,tracks/beam,-1
30,31,s
31,32,r
submesh
cab
10,30,20,u
10,31,30,c
10,32,31,p
10,20,31,n
10,31,32,b
contacters
31
flexbodies
10,20,30,0,0,0,0,0,0,body.mesh
forset 30-32,20,31
10,30,20,0,0,0,0,0,0,body.mesh
forset 10,20
end
",
    );
    let imported = load_vehicle(&path, "pack/fixture.truck").unwrap();
    let vehicle = &imported.vehicle;
    let structure = vehicle.physics.structure.as_ref().unwrap();
    let beam_values = |i: usize| {
        let beam = structure.beams[i].unwrap();
        [
            beam.stiffness_n_per_m,
            beam.damping_ns_per_m,
            beam.deformation_threshold_n,
            beam.breaking_threshold_n,
            beam.plasticity,
        ]
    };
    assert_eq!(beam_values(0), [9e6, 12e3, 400e3, 1e6, 0.0]);
    assert_eq!(beam_values(1), [2000.0, 80.0, 400e3, 8000.0, 0.25]);
    assert_eq!(beam_values(2), [1000.0, 20.0, 200e3, 6000.0, 0.25]);
    assert_eq!(beam_values(3), [1600.0, 20.0, 60.0, 6000.0, 0.25]);
    assert_eq!(beam_values(4), beam_values(0));
    assert_eq!(
        structure.beams[3].unwrap().kind,
        StructuralBeamKind::CompressionSupport {
            extension_limit_ratio: Some(2.5),
        },
    );
    assert_eq!(
        structure.beams[4].unwrap().kind,
        StructuralBeamKind::CompressionSupport {
            extension_limit_ratio: None,
        },
    );
    assert!(structure.beams[5].is_none());
    assert_eq!(vehicle.edges[5], [NodeId(3), NodeId(4)]);
    assert!(
        imported
            .warnings
            .iter()
            .any(|w| w.entry == "pack/fixture.truck"
                && w.message.contains("beams line")
                && w.message.contains("unsupported beam options"))
    );
    assert_eq!(
        structure
            .nodes
            .iter()
            .map(|node| node.mass_kg.unwrap())
            .collect::<Vec<_>>(),
        [4.0, 2.0, 8.0, 2.0, 2.0],
    );
    assert_eq!(
        structure
            .nodes
            .iter()
            .map(|node| node.excludes_ground_contact)
            .collect::<Vec<_>>(),
        [false, true, false, true, false],
    );
    assert_eq!(structure.contact_nodes, [NodeId(3)]);
    assert_eq!(
        structure.collision_triangles,
        [
            [NodeId(0), NodeId(2), NodeId(1)],
            [NodeId(0), NodeId(3), NodeId(2)],
            [NodeId(0), NodeId(4), NodeId(3)]
        ],
    );
    let [first, second] = structure.deformable_meshes.as_slice() else {
        panic!("expected two mesh instance bindings");
    };
    assert_eq!((first.object, second.object), (0, 1));
    assert_eq!(
        vehicle.objects[0].visual_mesh,
        vehicle.objects[1].visual_mesh
    );
    assert_eq!(first.placement_nodes, [NodeId(0), NodeId(1), NodeId(2)]);
    assert_eq!(second.placement_nodes, [NodeId(0), NodeId(2), NodeId(1)]);
    assert_eq!(
        first.influence_nodes,
        [NodeId(2), NodeId(3), NodeId(4), NodeId(1)]
    );
    assert_eq!(second.influence_nodes, [NodeId(0), NodeId(1)]);
}

#[test]
fn reports_invalid_collision_node_with_file_section_and_line() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("bad-panel.zip");
    write_archive(&path, "Fixture\nnodes\n0,0,0,0\ncab\n0,0,999,c\nend\n");
    let error = load_vehicle(&path, "pack/fixture.truck")
        .unwrap_err()
        .to_string();
    assert!(
        error.contains("fixture.truck: cab line 5: unknown node 999"),
        "{error}"
    );
}

#[test]
fn loads_a_geometry_only_vehicle() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("geometry.zip");
    write_archive(&path, "Fixture\nnodes\n3,0,0,0\nend\n");
    let vehicle = load_vehicle(&path, "pack/fixture.truck").unwrap().vehicle;
    assert_eq!(vehicle.nodes, [Vec3::ZERO]);
    assert!(vehicle.physics.structure.is_none());
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
        let structure = vehicle
            .physics
            .structure
            .as_ref()
            .expect("authored structure");
        assert!(structure.nodes.iter().all(|node| node.mass_kg.is_some()));
        assert!(
            structure.beams.iter().all(Option::is_some),
            "{entry}: {:?}",
            imported.warnings
        );
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
            assert_eq!(structure.collision_triangles.len(), 269);
            assert_eq!(structure.contact_nodes.len(), 85);
            assert_eq!(structure.deformable_meshes.len(), 13);
            assert_eq!(
                structure
                    .beams
                    .iter()
                    .flatten()
                    .filter(|beam| matches!(
                        beam.kind,
                        StructuralBeamKind::CompressionSupport { .. },
                    ))
                    .count(),
                54,
            );
            for (index, stiffness, damping, deform, breaking) in [
                (0, 2_500_000.0, 800.0, 8000.0, 25_000_000.0),
                (140, 3_000_000.0, 950.0, 40_000.0, 57_500.0),
            ] {
                let beam = structure.beams[index].unwrap();
                approx(beam.stiffness_n_per_m, stiffness * 0.80);
                approx(beam.damping_ns_per_m, damping * 0.13);
                approx(beam.deformation_threshold_n, deform * 0.56);
                approx(beam.breaking_threshold_n, breaking * 0.61);
                approx(beam.plasticity, 0.1);
            }
            let mut sprung_mass = 0.0;
            let mut sprung_moment = Vec3::ZERO;
            let mut upright_masses = [0.0; 4];
            for (node, &point) in structure.nodes.iter().zip(&vehicle.nodes) {
                let mass = node.mass_kg.unwrap();
                if let Some(wheel) = node.upright {
                    upright_masses[wheel] += mass;
                } else {
                    sprung_mass += mass;
                    sprung_moment += point * mass;
                }
            }
            approx(sprung_mass, chassis.mass_kg);
            assert!((sprung_moment / sprung_mass).abs_diff_eq(chassis.centre_of_mass_m, 1e-5));
            assert!((sprung_mass - 866.36).abs() < 0.01);
            let node_mass: f32 = structure
                .nodes
                .iter()
                .map(|node| node.mass_kg.unwrap())
                .sum();
            let wheel_mass: f32 = physics
                .wheels
                .iter()
                .map(|wheel| wheel.mass_kg.unwrap())
                .sum();
            approx(wheel_mass, 156.0);
            let total_mass = chassis.mass_kg + upright_masses.into_iter().sum::<f32>() + wheel_mass;
            assert!((total_mass - (node_mass + wheel_mass)).abs() < 0.002);
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
                let attachment = detail.attachment.expect("authored shock nodes");
                assert_eq!(structure.nodes[attachment.upper_node.0].upright, None);
                for node in [
                    attachment.lower_node,
                    attachment.axle_nodes[0],
                    attachment.axle_nodes[1],
                ] {
                    assert_eq!(structure.nodes[node.0].upright, Some(i));
                }
                assert!((suspension.stiffness_n_per_m - spring * ratio_sq).abs() < 0.01);
                assert!((suspension.damping_ns_per_m - damper * ratio_sq).abs() < 0.01);
                approx(suspension.initial_length_m, rise);
                assert!(suspension.free_length_m > suspension.initial_length_m);
                assert!(detail.upright_mass_kg.unwrap() > 18.0);
                approx(detail.upright_mass_kg.unwrap(), upright_masses[i]);
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

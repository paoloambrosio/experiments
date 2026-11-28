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
fn idle_command_beams_retain_a_panel_link_with_its_authored_material() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("panel.zip");
    write_archive(
        &path,
        "+Hinged panel
nodes
0,0,0,0
1,0,0.8,0
2,-0.7,0.4,0
3,-0.4,0.3,-0.2
beams
0,2,i
1,2,i
commands2
set_beam_defaults 400000,600,400000,1000000
set_beam_defaults_scale 0.5,0.25,1,1
2,3,0.2,0.3,1,2,1,2,ip Panel
commands
1,3,0.2,1,2,3,4,i Trim
0,3,0.2,1,2,5,6,r Cable
end
",
    );
    let imported = load_vehicle(&path, "pack/fixture.truck").unwrap();
    let asset = &imported.vehicle;
    let structure = asset.physics.structure.as_ref().unwrap();
    assert_eq!(
        asset.edges[2..],
        [
            [NodeId(2), NodeId(3)],
            [NodeId(1), NodeId(3)],
            [NodeId(0), NodeId(3)]
        ]
    );
    for beam in &structure.beams[2..4] {
        let beam = beam.unwrap();
        assert_eq!(beam.kind, StructuralBeamKind::Bilateral);
        assert_eq!(beam.stiffness_n_per_m, 200000.0);
        assert_eq!(beam.damping_ns_per_m, 150.0);
    }
    assert!(structure.beams[4].is_none());
    assert!(
        imported
            .warnings
            .iter()
            .any(|warning| warning.message.contains("tension-only command beam"))
    );
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
fn reference_car_variants_keep_their_authored_geometry_and_textures() {
    let zip = Path::new(env!("CARGO_MANIFEST_DIR")).join("reference/car.zip");
    for (entry, radius, rim, front, rear) in [
        ("car.truck", 0.31, 0.19, 0.205, 0.205),
        ("car-wide.truck", 0.33, 0.20, 0.225, 0.245),
    ] {
        let imported = load_vehicle(&zip, entry).unwrap();
        let vehicle = &imported.vehicle;
        let physics = &vehicle.physics;
        let structure = physics.structure.as_ref().unwrap();
        let chassis = physics.chassis.unwrap();
        assert_eq!(
            chassis.mass_kg,
            8.0 * 78.0 + 4.0 * 14.0 + 4.0 * 12.0 + 2.0 * 10.0
        );
        let sprung_moment: Vec3 = structure
            .nodes
            .iter()
            .zip(&vehicle.nodes)
            .filter(|(node, _)| node.upright.is_none())
            .map(|(node, point)| node.mass_kg.unwrap() * *point)
            .sum();
        assert!((sprung_moment / chassis.mass_kg).abs_diff_eq(chassis.centre_of_mass_m, 1e-6));
        assert!(
            chassis
                .size_m
                .abs_diff_eq(Vec3::new(1.36, 0.83, 3.82), 1e-5)
        );
        assert_eq!(structure.nodes.len(), 26);
        assert_eq!(structure.collision_triangles.len(), 4);
        assert_eq!(
            structure.contact_nodes,
            [NodeId(8), NodeId(9), NodeId(10), NodeId(11)]
        );
        let support = structure
            .beams
            .iter()
            .map(|beam| beam.unwrap())
            .find(|beam| matches!(beam.kind, StructuralBeamKind::CompressionSupport { .. }))
            .unwrap();
        assert_eq!(support.stiffness_n_per_m, 500000.0);
        assert_eq!(support.damping_ns_per_m, 2500.0);
        assert_eq!(support.deformation_threshold_n, 5500.0);
        assert_eq!(support.breaking_threshold_n, 500000.0);
        let [body, cabin] = structure.deformable_meshes.as_slice() else {
            panic!("expected bindings for the shell and cabin");
        };
        assert_eq!(vehicle.objects[body.object].name, "body.mesh");
        assert_eq!(vehicle.objects[cabin.object].name, "cabin.mesh");
        assert_eq!(body.placement_nodes, [NodeId(0), NodeId(1), NodeId(4)]);
        assert_eq!(cabin.placement_nodes, body.placement_nodes);
        assert_eq!(
            body.influence_nodes,
            (0..12).map(NodeId).collect::<Vec<_>>()
        );
        assert_eq!(cabin.influence_nodes, [NodeId(0)]);
        assert_eq!(vehicle.wheels.len(), 4);
        for (i, (wheel, detail)) in vehicle.wheels.iter().zip(&physics.wheels).enumerate() {
            let base = 12 + 3 * i;
            let is_front = i == 1 || i == 3;
            assert_eq!(wheel.outer_radius, radius);
            assert_eq!(wheel.rim_radius, Some(rim));
            assert_eq!(wheel.width, if is_front { front } else { rear });
            assert!(wheel.axle.x > 0.99);
            assert!(wheel.axle.y.abs() > 0.01);
            assert_eq!(wheel.axle.z.abs() > 0.01, is_front);
            let attachment = detail.attachment.unwrap();
            assert_eq!(attachment.axle_nodes, [NodeId(base + 1), NodeId(base)]);
            assert_eq!(attachment.upper_node, NodeId(base + 2));
            assert_eq!(attachment.lower_node, NodeId(base));
            assert_eq!(structure.nodes[base + 2].upright, None);
            for id in attachment.axle_nodes {
                assert_eq!(structure.nodes[id.0].upright, Some(i));
            }
            assert_eq!(detail.upright_mass_kg, Some(6.0));
            assert_eq!(detail.mass_kg, Some(32.0));
            assert_eq!(detail.brake_torque_nm, Some(550.0));
            assert_eq!(detail.drive_torque_nm.unwrap() > 0.0, is_front);
            let suspension = detail.suspension.unwrap();
            assert_eq!(suspension.stiffness_n_per_m, 32000.0);
            assert!(suspension.free_length_m > suspension.initial_length_m);
            let rim_object = &vehicle.objects[detail.rim_object];
            assert!((rim_object.rotation * Vec3::X).x * wheel.centre.x > 0.0);
        }
        assert!(
            imported
                .appearance
                .materials
                .iter()
                .any(|m| m.transparent && m.base_texture.is_some())
        );
        assert!(imported.warnings.is_empty(), "{:?}", imported.warnings);
    }
}

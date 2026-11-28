use std::time::Instant;

use bevy::prelude::*;
use vehicle_assets::{
    DeformableMeshBinding, MeshId, MeshInstance, NodeId, StructuralBeam, StructuralBeamKind,
    StructuralNode, TriangleMesh,
    ror::{TerrainImport, VehicleImport, reference},
};
use vehicle_scene::{
    body_skin::BodySkin,
    crash::DeformableSetup,
    drive::{AlignmentMode, DriveSetup, DriverInput, TICKS_PER_SECOND},
};

use super::{ordinary, print_timing, same_rims};

fn fixture(columns: u32, rows: u32) -> Result<VehicleImport, String> {
    let mut import = reference::vehicle().map_err(|error| error.to_string())?;
    let asset = &mut import.vehicle;
    let structure = asset.physics.structure.as_mut().unwrap();
    let mount_hardpoints = [0, 1, 2, 3, 4, 5, 6, 7].map(NodeId);
    let first = asset.nodes.len();
    // A lightly loaded deck with stiff connections to the main frame. A broad
    // mesh uses a narrow placement triangle and distributed influence nodes.
    let locations = [
        Vec3::new(-0.3, 0.56, 0.25),
        Vec3::new(0.3, 0.56, 0.25),
        Vec3::new(0.0, 0.56, 0.251),
    ]
    .into_iter()
    .chain((0..rows).flat_map(|row| {
        (0..columns).map(move |column| {
            Vec3::new(
                -0.55 + 1.05 * column as f32 / (columns - 1) as f32,
                0.57,
                -1.45 + 1.90 * row as f32 / (rows - 1) as f32,
            )
        })
    }));
    let beam = StructuralBeam {
        kind: StructuralBeamKind::Bilateral,
        stiffness_n_per_m: 450_000.0,
        damping_ns_per_m: 800.0,
        deformation_threshold_n: 40_000.0,
        breaking_threshold_n: 100_000.0,
        plasticity: 0.1,
    };
    for point in locations {
        let node = NodeId(asset.nodes.len());
        asset.nodes.push(point);
        structure.nodes.push(StructuralNode {
            mass_kg: Some(0.5),
            ..default()
        });
        for anchor in mount_hardpoints {
            asset.edges.push([anchor, node]);
            structure.beams.push(Some(beam));
        }
    }
    asset.physics.chassis.as_mut().unwrap().mass_kg += (asset.nodes.len() - first) as f32 * 0.5;
    let object = asset.objects.len();
    let mut second_panel = asset.objects[structure.deformable_meshes[1].object].clone();
    second_panel.name = "Deck trim".into();
    asset.objects.push(second_panel);
    structure.deformable_meshes.insert(
        0,
        DeformableMeshBinding {
            object,
            placement_nodes: [first, first + 1, first + 2].map(NodeId),
            influence_nodes: (first..asset.nodes.len()).map(NodeId).collect(),
        },
    );
    Ok(import)
}

pub(super) fn run(terrain: &TerrainImport) -> Result<(), String> {
    check(terrain, fixture(8, 6)?)?;
    let mut scattered = TerrainImport {
        terrain: terrain.terrain.clone(),
        start_position: terrain.start_position,
        appearance: terrain.appearance.clone(),
        warnings: vec![],
    };
    let mesh = MeshId(scattered.terrain.meshes.len());
    scattered.terrain.meshes.push(TriangleMesh {
        vertices: vec![
            Vec3::ZERO,
            3.0 * Vec3::Y,
            3.0 * Vec3::Y + 4.0 * Vec3::Z,
            4.0 * Vec3::Z,
        ],
        triangles: vec![[0, 1, 2], [0, 2, 3]],
        normals: None,
        uvs: None,
    });
    let start = scattered.start_position.unwrap_or(Vec3::ZERO);
    for column in [-96, -48, -24, -12, 12, 24, 48, 96] {
        for row in 0..32 {
            scattered.terrain.objects.push(MeshInstance {
                name: format!("Scattered barrier {column} / {row}"),
                translation: Vec3::new(start.x + column as f32, 0.0, start.z + 6.0 * row as f32),
                rotation: Quat::IDENTITY,
                scale: Vec3::ONE,
                visual_mesh: None,
                collision_mesh: Some(mesh),
            });
        }
    }
    println!("Dense drive with scattered obstacles");
    check(&scattered, fixture(17, 13)?)
}

fn check(terrain: &TerrainImport, vehicle: VehicleImport) -> Result<(), String> {
    let asset = &vehicle.vehicle;
    let drive = DriveSetup::new(terrain, &vehicle, AlignmentMode::Authored)?;
    let setup = DeformableSetup::for_drive(&drive, asset)?;
    let rate = setup.substeps * TICKS_PER_SECOND;
    println!(
        "Dense idle | {} sprung nodes | {} beams | {} substeps/s",
        setup.imported.structure.nodes.len(),
        setup.imported.structure.beams.len(),
        rate
    );
    let skin = BodySkin::new(asset, &setup.imported.source)?;
    let mut session = setup.start(drive.start, |_| {})?;
    session.reset()?;
    let initial = session.frame();
    let rest = skin.sample(&initial.nodes_m)?;
    let world_vertices = |shape: &vehicle_scene::body_skin::BodyShape| {
        skin.meshes
            .iter()
            .zip(&shape.vertices)
            .flat_map(|(mesh, vertices)| {
                let instance = &asset.objects[mesh.object];
                vertices.iter().map(|&point| {
                    shape.pose.origin_m + shape.pose.rotation * instance.transform_point(point)
                })
            })
            .collect::<Vec<_>>()
    };
    let rest_vertices = world_vertices(&rest);
    let mut physics = Vec::new();
    let mut skinning = Vec::new();
    let mut node_motion = 0.0_f64;
    let mut vertex_motion = 0.0_f32;
    let mut damage = 0.0_f64;
    for _ in 0..TICKS_PER_SECOND {
        let start = Instant::now();
        session.step(DriverInput::default())?;
        physics.push(start.elapsed().as_secs_f64());
        let frame = session.frame();
        let start = Instant::now();
        let shape = skin.sample(&frame.nodes_m)?;
        skinning.push(start.elapsed().as_secs_f64());
        for (point, original) in frame.nodes_m.iter().zip(&initial.nodes_m) {
            node_motion = node_motion.max(point.distance(*original));
        }
        for (point, original) in world_vertices(&shape).iter().zip(&rest_vertices) {
            vertex_motion = vertex_motion.max(point.distance(*original));
        }
        for beam in session.body().chassis().beam_damage() {
            damage = damage.max(beam.permanent_length_change_m().abs());
        }
    }
    print_timing("Dense idle physics", &mut physics);
    print_timing("Dense idle skin sampling", &mut skinning);
    println!(
        "Dense idle | peak node motion {node_motion:.4} m | peak vertex motion {vertex_motion:.4} m | permanent length change {damage:.6} m"
    );
    if damage > 1e-6
        || node_motion > 0.08
        || session.velocity_mps().length() > 0.15
        || vertex_motion as f64 > node_motion + 0.01
    {
        return Err("dense chassis failed to settle without yield or excess mesh motion".into());
    }
    session.reset()?;
    println!("Dense ordinary driving");
    let start = Instant::now();
    ordinary(&mut session, &skin, drive.weight_n())?;
    let elapsed = start.elapsed().as_secs_f64();
    let ticks = vehicle_scene::drive::BENCHMARK_TICKS;
    println!(
        "Dense ordinary driving | {ticks} ticks in {elapsed:.3} s | {:.2} ticks/s including skin sampling and checks",
        f64::from(ticks) / elapsed
    );
    session.reset()?;
    let reset = session.frame();
    if reset.nodes_m != initial.nodes_m
        || !same_rims(&initial.rims, &reset.rims, 1e-9)
        || session.velocity_mps() != bevy::math::DVec3::ZERO
    {
        return Err("dense drive reset did not restore the spawn".into());
    }
    Ok(())
}

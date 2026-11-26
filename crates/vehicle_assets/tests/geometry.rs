use std::f32::consts::FRAC_PI_2;

use bevy_math::{Quat, Vec2, Vec3};
use vehicle_assets::{
    Heightfield, MeshId, MeshInstance, NodeId, TerrainAsset, TriangleMesh, ValidationErrorKind,
    VehicleAsset, WheelGeometry,
};

fn terrain() -> TerrainAsset {
    let visual = TriangleMesh {
        vertices: vec![
            Vec3::ZERO,
            Vec3::new(0.0, 1.0, 1.0),
            Vec3::new(2.0, 0.0, 1.0),
        ],
        triangles: vec![[0, 1, 2]],
        normals: Some(vec![Vec3::Y; 3]),
        uvs: Some(vec![Vec2::ZERO, Vec2::Y, Vec2::ONE]),
    };
    let collision = TriangleMesh {
        vertices: vec![
            Vec3::ZERO,
            Vec3::new(0.0, 0.75, 1.0),
            Vec3::new(2.0, 0.0, 0.5),
        ],
        triangles: vec![[0, 1, 2]],
        normals: None,
        uvs: None,
    };
    TerrainAsset {
        name: "Test facility".into(),
        heightfield: Some(Heightfield {
            origin: Vec3::new(10.0, 3.0, -4.0),
            samples_x: 3,
            samples_z: 2,
            spacing_x: 2.0,
            spacing_z: 4.0,
            heights: vec![0.0, 0.5, 1.0, 2.0, 3.0, 4.0],
        }),
        meshes: vec![visual, collision],
        objects: vec![
            MeshInstance {
                name: "Rotated ramp".into(),
                translation: Vec3::new(10.0, 2.0, 20.0),
                rotation: Quat::from_rotation_y(FRAC_PI_2),
                scale: Vec3::new(2.0, 3.0, 4.0),
                visual_mesh: Some(MeshId(0)),
                collision_mesh: Some(MeshId(1)),
            },
            MeshInstance {
                name: "Second ramp".into(),
                translation: Vec3::new(20.0, 2.0, 10.0),
                rotation: Quat::IDENTITY,
                scale: Vec3::ONE,
                visual_mesh: Some(MeshId(0)),
                collision_mesh: Some(MeshId(0)),
            },
        ],
    }
}

fn vehicle() -> VehicleAsset {
    let nodes = vec![
        Vec3::new(0.75, 0.34, 1.5),
        Vec3::new(-0.75, 0.34, 1.5),
        Vec3::new(0.75, 0.34, -1.0),
        Vec3::new(-0.75, 0.34, -1.0),
    ];
    let wheels = nodes
        .iter()
        .map(|&centre| WheelGeometry {
            centre,
            axle: Vec3::X,
            outer_radius: 0.34,
            width: 0.22,
            rim_radius: Some(0.20),
        })
        .collect();
    VehicleAsset {
        physics: Default::default(),
        name: "Test car".into(),
        meshes: Vec::new(),
        objects: Vec::new(),
        nodes,
        edges: vec![
            [NodeId(0), NodeId(1)],
            [NodeId(1), NodeId(3)],
            [NodeId(3), NodeId(2)],
            [NodeId(2), NodeId(0)],
        ],
        wheels,
    }
}

#[test]
fn terrain_positions_use_grid_spacing_and_local_mesh_origins() {
    let terrain = terrain();
    terrain.validate().unwrap();

    let grid = terrain.heightfield.as_ref().unwrap();
    assert_eq!(grid.sample_position(0, 0), Some(Vec3::new(10.0, 3.0, -4.0)));
    assert_eq!(grid.sample_position(1, 0), Some(Vec3::new(12.0, 3.5, -4.0)));
    assert_eq!(grid.sample_position(0, 1), Some(Vec3::new(10.0, 5.0, 0.0)));
    assert_eq!(grid.sample_position(2, 1), Some(Vec3::new(14.0, 7.0, 0.0)));
    assert_eq!(grid.sample_position(3, 0), None);
    assert_eq!(grid.sample_position(0, 2), None);

    let ramp = &terrain.objects[0];
    let visual = &terrain.meshes[ramp.visual_mesh.unwrap().0];
    let collision = &terrain.meshes[ramp.collision_mesh.unwrap().0];
    assert_eq!(ramp.transform_point(visual.vertices[0]), ramp.translation);
    assert!(
        ramp.transform_point(visual.vertices[2])
            .abs_diff_eq(Vec3::new(14.0, 2.0, 16.0), 1e-5)
    );
    assert!(
        ramp.transform_point(collision.vertices[2])
            .abs_diff_eq(Vec3::new(12.0, 2.0, 16.0), 1e-5)
    );
    let second_ramp = &terrain.objects[1];
    assert_eq!(second_ramp.visual_mesh, second_ramp.collision_mesh);
    assert_eq!(
        second_ramp.transform_point(visual.vertices[2]),
        Vec3::new(22.0, 2.0, 11.0)
    );
}

#[test]
fn vehicle_positions_describe_track_wheelbase_and_outline() {
    let vehicle = vehicle();
    vehicle.validate().unwrap();

    let [front_left, front_right, rear_left, rear_right] = vehicle.wheels.as_slice() else {
        panic!("fixture must have four wheels");
    };
    assert_eq!(front_left.centre - front_right.centre, 1.5 * Vec3::X);
    assert_eq!(rear_left.centre - rear_right.centre, 1.5 * Vec3::X);
    assert_eq!(front_left.centre - rear_left.centre, 2.5 * Vec3::Z);
    assert_eq!(front_right.centre - rear_right.centre, 2.5 * Vec3::Z);
    for wheel in &vehicle.wheels {
        assert_eq!(wheel.axle, Vec3::X);
        assert_eq!(wheel.outer_radius, 0.34);
        assert_eq!(wheel.width, 0.22);
        assert_eq!(wheel.rim_radius, Some(0.20));
    }

    let [start, end] = vehicle.edges[0];
    assert_eq!(vehicle.nodes[start.0], front_left.centre);
    assert_eq!(vehicle.nodes[end.0], front_right.centre);
}

#[test]
fn mesh_and_node_errors_identify_the_bad_reference() {
    let mut terrain = terrain();
    terrain.objects[0].collision_mesh = Some(MeshId(9));
    let error = terrain.validate().unwrap_err();
    assert_eq!(error.field, "objects[0].collision_mesh");
    assert_eq!(
        error.kind,
        ValidationErrorKind::IndexOutOfBounds { index: 9, len: 2 }
    );

    terrain.objects[0].collision_mesh = Some(MeshId(1));
    terrain.meshes[1].triangles[0][2] = 3;
    let error = terrain.validate().unwrap_err();
    assert_eq!(error.field, "meshes[1].triangles[0][2]");
    assert_eq!(
        error.kind,
        ValidationErrorKind::IndexOutOfBounds { index: 3, len: 3 }
    );

    let mut vehicle = vehicle();
    vehicle.edges[2][1] = NodeId(4);
    let error = vehicle.validate().unwrap_err();
    assert_eq!(error.field, "edges[2][1]");
    assert_eq!(
        error.kind,
        ValidationErrorKind::IndexOutOfBounds { index: 4, len: 4 }
    );
}

#[test]
fn optional_mesh_roles_and_attribute_counts_are_checked() {
    let mut terrain = terrain();
    terrain.objects[0].visual_mesh = None;
    terrain.objects[1].collision_mesh = None;
    terrain.validate().unwrap();

    terrain.objects[1].visual_mesh = None;
    let error = terrain.validate().unwrap_err();
    assert_eq!(error.field, "objects[1]");
    assert_eq!(error.kind, ValidationErrorKind::MissingMeshRole);

    terrain.objects[1].visual_mesh = Some(MeshId(0));
    terrain.meshes[0].uvs.as_mut().unwrap().pop();
    let error = terrain.validate().unwrap_err();
    assert_eq!(error.field, "meshes[0].uvs");
    assert_eq!(
        error.kind,
        ValidationErrorKind::CountMismatch {
            expected: 3,
            actual: 2
        }
    );
}

#[test]
fn grid_size_and_height_counts_are_checked() {
    let mut terrain = terrain();
    terrain.heightfield.as_mut().unwrap().heights.pop();
    let error = terrain.validate().unwrap_err();
    assert_eq!(error.field, "heightfield.heights");
    assert_eq!(
        error.kind,
        ValidationErrorKind::CountMismatch {
            expected: 6,
            actual: 5
        }
    );

    terrain.heightfield.as_mut().unwrap().samples_x = usize::MAX;
    let error = terrain.validate().unwrap_err();
    assert_eq!(error.field, "heightfield.heights");
    assert_eq!(error.kind, ValidationErrorKind::SampleCountOverflow);
}

#[test]
fn wheel_dimensions_and_orientation_are_checked() {
    let mut vehicle = vehicle();
    vehicle.wheels[2].rim_radius = None;
    vehicle.validate().unwrap();

    vehicle.wheels[2].width = 0.0;
    let error = vehicle.validate().unwrap_err();
    assert_eq!(error.field, "wheels[2].width");
    assert_eq!(error.kind, ValidationErrorKind::NonPositive);

    vehicle.wheels[2].width = 0.22;
    vehicle.wheels[2].axle = 2.0 * Vec3::X;
    let error = vehicle.validate().unwrap_err();
    assert_eq!(error.field, "wheels[2].axle");
    assert_eq!(error.kind, ValidationErrorKind::NotUnitLength);

    vehicle.wheels[2].axle = Vec3::X;
    vehicle.wheels[2].rim_radius = Some(0.35);
    let error = vehicle.validate().unwrap_err();
    assert_eq!(error.field, "wheels[2].rim_radius");
    assert_eq!(error.kind, ValidationErrorKind::RimLargerThanWheel);
}

#[test]
fn mesh_transforms_and_coordinates_are_checked() {
    let mut terrain = terrain();
    terrain.objects[0].rotation = Quat::from_xyzw(0.0, 0.0, 0.0, 2.0);
    let error = terrain.validate().unwrap_err();
    assert_eq!(error.field, "objects[0].rotation");
    assert_eq!(error.kind, ValidationErrorKind::NotUnitLength);

    terrain.objects[0].rotation = Quat::IDENTITY;
    terrain.objects[0].scale = Vec3::new(-1.0, 1.0, 1.0);
    terrain.validate().unwrap();

    terrain.meshes[1].vertices[2].x = f32::NAN;
    let error = terrain.validate().unwrap_err();
    assert_eq!(error.field, "meshes[1].vertices[2]");
    assert_eq!(error.kind, ValidationErrorKind::NonFinite);
}

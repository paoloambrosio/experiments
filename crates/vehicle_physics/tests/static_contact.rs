use bevy_math::DVec3;
use vehicle_physics::{
    chassis::{
        BeamCoefficients, Chassis, ChassisBeamSpec, ChassisCollision, ChassisError, ChassisNode,
    },
    static_collision::{ContactFeature, StaticCollisionError, StaticCollisionMesh},
};

fn node(position_m: DVec3, velocity_mps: DVec3, mass_kg: f64) -> ChassisNode {
    ChassisNode {
        position_m,
        velocity_mps,
        mass_kg,
    }
}

#[test]
fn fast_oblique_and_overhead_strikes_preserve_tangential_motion_across_a_seam() {
    for normal in [DVec3::new(1.0, 0.0, 2.0).normalize(), -DVec3::Y] {
        let tangent = normal.cross(DVec3::Z).normalize();
        let vertical = normal.cross(tangent);
        let vertices = [
            -tangent - vertical,
            tangent - vertical,
            tangent + vertical,
            -tangent + vertical,
        ];
        for triangles in [[[0, 1, 2], [0, 2, 3]], [[2, 1, 0], [3, 2, 0]]] {
            let mesh = StaticCollisionMesh::new(&vertices, &triangles).unwrap();
            for crossing in [DVec3::ZERO, 0.4 * tangent] {
                for side in [1.0, -1.0] {
                    let incoming = -side * 200.0 * normal + 0.5 * tangent;
                    let start = crossing - incoming * 0.01;
                    let mut chassis =
                        Chassis::new(vec![node(start, incoming, 3.0)], vec![]).unwrap();
                    let surface = ChassisCollision::new(&chassis, vec![0], vec![]).unwrap();
                    let reaction = chassis
                        .step_with_static_world(0.02, &surface, &mesh)
                        .unwrap();
                    assert_eq!(reaction.contacts, 1);
                    assert_eq!(reaction.first.unwrap().feature, ContactFeature::Node(0));
                    assert!(reaction.impulse_ns.abs_diff_eq(side * normal * 600.0, 1e-9));
                    assert!(
                        chassis.nodes()[0]
                            .velocity_mps
                            .abs_diff_eq(tangent * 0.5, 1e-10)
                    );
                    assert!(side * chassis.nodes()[0].position_m.dot(normal) >= 0.0);
                    assert!(reaction.max_residual_depth_m < 1e-8);
                    // The solver can release from the surface on the following substep.
                    assert_eq!(
                        chassis
                            .step_with_static_world(0.02, &surface, &mesh)
                            .unwrap()
                            .contacts,
                        0
                    );
                }
            }
        }
    }
}

#[test]
fn nearly_simultaneous_nodes_stop_at_the_wall_in_time_order() {
    let wall_z = 64.0;
    let world = StaticCollisionMesh::new(
        &[
            DVec3::new(-2.0, -2.0, wall_z),
            DVec3::new(2.0, -2.0, wall_z),
            DVec3::new(0.0, 2.0, wall_z),
        ],
        &[[0, 1, 2]],
    )
    .unwrap();
    let mut chassis = Chassis::new(
        vec![
            node(DVec3::new(-0.5, 0.0, wall_z - 0.5 - 5e-11), DVec3::Z, 2.0),
            node(DVec3::new(0.5, 0.0, wall_z - 0.5), DVec3::Z, 2.0),
        ],
        vec![],
    )
    .unwrap();
    let surface = ChassisCollision::new(&chassis, vec![0, 1], vec![]).unwrap();
    let reaction = chassis
        .step_with_static_world(1.0, &surface, &world)
        .unwrap();
    assert!(
        chassis
            .nodes()
            .iter()
            .all(|node| node.position_m.z < wall_z)
    );
    assert_eq!(reaction.first.unwrap().feature, ContactFeature::Node(1));
    assert_eq!(reaction.contacts, 2);
    assert_eq!(reaction.impulse_ns, -4.0 * DVec3::Z);
}

#[test]
fn simultaneous_nodes_stop_on_an_oblique_wall_despite_roundoff() {
    let wall = [
        DVec3::new(60.0, 0.0, 60.0),
        DVec3::new(68.0, 0.0, 59.0),
        DVec3::new(60.0, 3.0, 60.0),
    ];
    let normal = (wall[1] - wall[0]).cross(wall[2] - wall[0]).normalize();
    let world = StaticCollisionMesh::new(&wall, &[[0, 1, 2]]).unwrap();
    let incoming = DVec3::new(0.01, 0.0, 0.1);
    let mut chassis = Chassis::new(
        [0.5, 1.0]
            .map(|y| node(DVec3::new(63.001, y, 59.6), incoming, 2.0))
            .to_vec(),
        vec![],
    )
    .unwrap();
    let surface = ChassisCollision::new(&chassis, vec![0, 1], vec![]).unwrap();
    let reaction = chassis
        .step_with_static_world(1.0, &surface, &world)
        .unwrap();
    assert_eq!(reaction.contacts, 2);
    for node in chassis.nodes() {
        assert!((node.position_m - wall[0]).dot(normal) < 0.0);
        assert!(
            node.velocity_mps
                .abs_diff_eq(incoming.reject_from_normalized(normal), 1e-12)
        );
    }
}

#[test]
fn edge_crossing_catches_two_panels_with_mutually_exterior_vertices() {
    let quad = [[0, 1, 2], [0, 2, 3]];
    let world = StaticCollisionMesh::new(
        &[
            DVec3::new(-0.1, -2.0, 0.0),
            DVec3::new(0.1, -2.0, 0.0),
            DVec3::new(0.1, 2.0, 0.0),
            DVec3::new(-0.1, 2.0, 0.0),
        ],
        &quad,
    )
    .unwrap();
    let mut chassis = Chassis::new(
        [(-2.0, -0.1), (2.0, -0.1), (2.0, 0.1), (-2.0, 0.1)]
            .into_iter()
            .map(|(x, y)| node(DVec3::new(x, y, -1.0), DVec3::Z, 2.0))
            .collect(),
        vec![],
    )
    .unwrap();
    let surface = ChassisCollision::new(&chassis, vec![], quad.to_vec()).unwrap();
    let hit = chassis
        .step_with_static_world(1.0, &surface, &world)
        .unwrap();
    assert!(matches!(
        hit.first.unwrap().feature,
        ContactFeature::PanelEdge { .. }
    ));
    assert!(hit.impulse_ns.z < -3.0);
    let momentum: DVec3 = chassis
        .nodes()
        .iter()
        .map(|n| n.mass_kg * n.velocity_mps)
        .sum();
    assert!(momentum.abs_diff_eq(8.0 * DVec3::Z + hit.impulse_ns, 1e-12));
    let after = chassis.nodes().to_vec();
    chassis.reset();
    assert_eq!(
        chassis
            .step_with_static_world(1.0, &surface, &world)
            .unwrap(),
        hit
    );
    assert_eq!(chassis.nodes(), after);
}

#[test]
fn invalid_geometry_and_a_failed_step_preserve_committed_state() {
    assert_eq!(
        StaticCollisionMesh::new(&[DVec3::ZERO, DVec3::X, DVec3::Y], &[[0, 1, 1]]).unwrap_err(),
        StaticCollisionError::InvalidTriangle(0),
    );
    let mesh = StaticCollisionMesh::new(&[DVec3::ZERO, DVec3::X, DVec3::Y], &[[0, 1, 2]]).unwrap();
    let mut chassis = Chassis::new(
        vec![
            node(DVec3::Z, DVec3::Z, 1.0),
            node(2.0 * DVec3::Z, DVec3::ZERO, 1.0),
        ],
        vec![ChassisBeamSpec {
            kind: Default::default(),
            endpoints: [0, 1],
            coefficients: BeamCoefficients {
                stiffness: 10.0,
                damping: 0.0,
            },
            material: None,
        }],
    )
    .unwrap();
    assert_eq!(
        ChassisCollision::new(&chassis, vec![], vec![[0, 0, 1]]).unwrap_err(),
        ChassisError::InvalidCollisionPanel(0),
    );
    let surface = ChassisCollision::new(&chassis, vec![0, 1], vec![]).unwrap();
    let before = chassis.nodes().to_vec();
    let beams = chassis.beam_damage().collect::<Vec<_>>();
    assert_eq!(
        chassis.step_with_static_world(1.0, &surface, &mesh),
        Err(ChassisError::CollapsedBeam(0)),
    );
    assert_eq!(chassis.nodes(), before);
    assert_eq!(chassis.beam_damage().collect::<Vec<_>>(), beams);
    assert!(
        chassis
            .step_with_static_world(0.25, &surface, &mesh)
            .is_ok()
    );
}

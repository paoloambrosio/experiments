use bevy_math::DVec3;
use vehicle_physics::{
    beam_damage::BeamDamageConfig,
    chassis::{
        BeamCoefficients, Chassis, ChassisBeamKind, ChassisBeamSpec, ChassisError, ChassisNode,
        WallFace,
    },
};

const DT: f64 = 1.0 / 3840.0;

fn node(x_m: f64, speed_mps: f64) -> ChassisNode {
    ChassisNode {
        mass_kg: 2.0,
        position_m: x_m * DVec3::X,
        velocity_mps: speed_mps * DVec3::X,
    }
}

fn beam(endpoints: [usize; 2], material: Option<BeamDamageConfig>) -> ChassisBeamSpec {
    ChassisBeamSpec {
        kind: Default::default(),
        endpoints,
        coefficients: BeamCoefficients {
            stiffness: 400.0,
            damping: 0.0,
        },
        material,
    }
}

fn momentum(chassis: &Chassis) -> DVec3 {
    chassis
        .nodes()
        .iter()
        .map(|n| n.mass_kg * n.velocity_mps)
        .sum()
}

fn length(chassis: &Chassis) -> f64 {
    chassis.nodes()[0]
        .position_m
        .distance(chassis.nodes()[1].position_m)
}

#[test]
fn compression_support_goes_slack_in_extension_and_yields_under_compression() {
    let support = ChassisBeamSpec {
        kind: ChassisBeamKind::CompressionSupport,
        coefficients: BeamCoefficients {
            stiffness: 400.0,
            damping: 8.0,
        },
        ..beam(
            [0, 1],
            Some(BeamDamageConfig::yield_only(0.03, 0.03).unwrap()),
        )
    };
    let mut extending = Chassis::new(vec![node(0.0, 0.0), node(1.0, 2.0)], vec![support]).unwrap();
    let mut closing = Chassis::new(vec![node(0.0, 2.0), node(1.0, 0.0)], vec![support]).unwrap();
    let mut forward = 0.0_f64;
    for _ in 0..768 {
        extending.step(DT, None).unwrap();
        closing.step(DT, None).unwrap();
        forward = forward.max(closing.nodes()[1].velocity_mps.x);
    }
    assert!((length(&extending) - 1.4).abs() < 1e-10);
    assert!(!extending.beam_damage().next().unwrap().yielded());
    assert!(forward > 1.0);
    assert!(
        closing
            .beam_damage()
            .next()
            .unwrap()
            .permanent_length_change_m()
            < -0.001
    );
    assert!(momentum(&closing).abs_diff_eq(DVec3::X * 4.0, 1e-10));
}

#[test]
fn elastic_bay_recovers_at_constant_momentum() {
    let mut spring = beam(
        [0, 1],
        Some(BeamDamageConfig::yield_only(0.10, 0.05).unwrap()),
    );
    spring.coefficients.damping = 12.0;
    let mut chassis = Chassis::new(
        vec![
            node(0.0, 0.8),
            ChassisNode {
                mass_kg: 3.0,
                ..node(1.0, 0.2)
            },
        ],
        vec![spring],
    )
    .unwrap();
    let initial = momentum(&chassis);
    let mut shortest = f64::INFINITY;
    for _ in 0..11_520 {
        chassis.step(DT, None).unwrap();
        shortest = shortest.min(length(&chassis));
        assert!(momentum(&chassis).abs_diff_eq(initial, 1e-10));
    }
    assert!(shortest < 0.98, "{shortest}");
    assert!((length(&chassis) - 1.0).abs() < 1e-6);
    assert!(!chassis.beam_damage().next().unwrap().yielded());
}

#[test]
fn yielded_bay_keeps_its_shortening_and_reset_repeats_the_motion() {
    let mut spring = beam(
        [0, 1],
        Some(BeamDamageConfig::yield_only(0.5, 0.04).unwrap()),
    );
    spring.coefficients.damping = 8.0;
    let nodes = vec![node(0.0, 2.4), node(1.0, 0.0)];
    let mut chassis = Chassis::new(nodes.clone(), vec![spring]).unwrap();
    let run = |chassis: &mut Chassis| {
        for _ in 0..11_520 {
            chassis.step(DT, None).unwrap();
        }
        let damage = chassis.beam_damage().next().unwrap();
        assert!(damage.yielded() && !damage.broken());
        assert!(length(chassis) < 0.95, "length {}", length(chassis));
        assert!((length(chassis) - damage.stress_free_length_m()).abs() < 1e-5);
        (chassis.nodes().to_vec(), damage)
    };
    let first = run(&mut chassis);
    chassis.reset();
    assert_eq!(chassis.nodes(), nodes);
    let restored = chassis.beam_damage().next().unwrap();
    assert_eq!(restored.stress_free_length_m(), 1.0);
    assert!(!restored.yielded());
    assert_eq!(run(&mut chassis), first);
}

#[test]
fn wall_tests_the_crossing_and_preserves_tangential_motion() {
    let wall = WallFace::new(0.0, -0.3..=0.3, -0.4..=0.4).unwrap();
    let hit = ChassisNode {
        mass_kg: 2.0,
        position_m: DVec3::new(-1.0, 0.0, -1.0),
        velocity_mps: DVec3::new(4.0, 0.0, 4.0),
    };
    let beside = ChassisNode {
        position_m: DVec3::new(-1.3, 0.0, -1.0),
        velocity_mps: DVec3::new(3.0, 0.0, 4.0),
        ..hit
    };
    let above = ChassisNode {
        position_m: hit.position_m.with_y(0.6),
        ..hit
    };
    let mut chassis = Chassis::new(vec![hit, beside, above], Vec::new()).unwrap();
    let reaction = chassis.step(0.5, Some(&wall)).unwrap();
    assert_eq!(reaction.struck_nodes, 1);
    assert_eq!(reaction.impulse_z_ns, -8.0);
    assert_eq!(reaction.max_trial_depth_m, 1.0);
    assert_eq!(chassis.nodes()[0].position_m, DVec3::X);
    assert_eq!(chassis.nodes()[0].velocity_mps, 4.0 * DVec3::X);
    assert_eq!(chassis.nodes()[1].position_m.z, 1.0);
    assert_eq!(chassis.nodes()[2].position_m.z, 1.0);

    let pressing = ChassisNode {
        position_m: DVec3::ZERO,
        velocity_mps: DVec3::Z,
        ..hit
    };
    let mut chassis = Chassis::new(vec![pressing], Vec::new()).unwrap();
    let reaction = chassis.step(DT, Some(&wall)).unwrap();
    assert_eq!(reaction.impulse_z_ns, -2.0);
    assert_eq!(chassis.nodes()[0].position_m, DVec3::ZERO);
    assert_eq!(chassis.nodes()[0].velocity_mps, DVec3::ZERO);
}

#[test]
fn invalid_configuration_and_failed_substeps_preserve_state() {
    let invalid_mass = ChassisNode {
        mass_kg: f64::NAN,
        ..node(0.0, 0.0)
    };
    assert!(matches!(
        Chassis::new(vec![invalid_mass], vec![]),
        Err(ChassisError::InvalidNode(0))
    ));
    assert!(matches!(
        Chassis::new(vec![node(0.0, 0.0)], vec![beam([0, 1], None)]),
        Err(ChassisError::InvalidBeam(0))
    ));
    assert!(matches!(
        Chassis::new(vec![node(0.0, 0.0)], vec![beam([0, 0], None)]),
        Err(ChassisError::InvalidBeam(0))
    ));
    assert!(WallFace::new(0.0, -1.0..=1.0, 0.0..=0.0).is_err());
    assert!(BeamDamageConfig::yield_only(0.0, 0.06).is_err());

    let initial = vec![
        node(0.0, 0.0),
        node(1.0, -0.4),
        node(3.0, 0.0),
        node(4.0, -1.0),
    ];
    let material = Some(BeamDamageConfig::yield_only(0.05, 0.05).unwrap());
    let mut chassis = Chassis::new(
        initial.clone(),
        vec![beam([0, 1], material), beam([2, 3], material)],
    )
    .unwrap();
    let undamaged: Vec<_> = chassis.beam_damage().collect();
    assert_eq!(
        chassis.step(f64::NAN, None),
        Err(ChassisError::InvalidTimestep)
    );
    assert_eq!(chassis.step(1.0, None), Err(ChassisError::CollapsedBeam(1)));
    assert_eq!(chassis.nodes(), initial);
    assert_eq!(chassis.beam_damage().collect::<Vec<_>>(), undamaged);
    chassis.step(DT, None).unwrap();
    assert!((chassis.nodes()[3].position_m.x - (4.0 - DT)).abs() < 1e-12);
}

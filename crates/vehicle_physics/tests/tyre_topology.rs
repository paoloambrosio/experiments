use std::collections::HashSet;

use vehicle_physics::tyre::{BeamFamily, RingRole, TyreConfig, TyreConfigError, TyreTopology};

#[test]
fn generates_the_default_tyre() {
    let config = TyreConfig::default();
    let topology = check_topology(&config);
    assert_eq!(topology.nodes().len(), 160);
    assert_eq!(topology.beams().len(), 544);
}

#[test]
fn three_segments_form_closed_rings_with_unique_beams() {
    check_topology(&TyreConfig {
        segments: 3,
        ..TyreConfig::default()
    });
}

fn check_topology(config: &TyreConfig) -> TyreTopology {
    let topology = TyreTopology::generate(config).unwrap();
    let n = config.segments;
    let nodes = topology.nodes();
    let beams = topology.beams();
    assert_eq!(nodes.len(), 5 * n);
    assert_eq!(beams.len(), 17 * n);

    let profile = [
        (
            RingRole::LeftBead,
            -config.bead_width / 2.0,
            config.bead_radius,
        ),
        (
            RingRole::LeftShoulder,
            -config.shoulder_width / 2.0,
            config.shoulder_radius,
        ),
        (RingRole::TreadCentre, 0.0, config.tread_radius),
        (
            RingRole::RightShoulder,
            config.shoulder_width / 2.0,
            config.shoulder_radius,
        ),
        (
            RingRole::RightBead,
            config.bead_width / 2.0,
            config.bead_radius,
        ),
    ];
    for (ring, (role, x, radius)) in profile.into_iter().enumerate() {
        for segment in 0..n {
            let node = nodes[ring * n + segment];
            let p = node.rest_position;
            assert_eq!(node.ring, role);
            assert_eq!(p.x, x);
            assert!((p.y.hypot(p.z) - radius).abs() < 1e-6);
            let mirrored = nodes[(4 - ring) * n + segment].rest_position;
            assert_eq!((p.x, p.y, p.z), (-mirrored.x, mirrored.y, mirrored.z));
        }
    }

    let mut endpoints = HashSet::new();
    let mut family_counts = [0; 3];
    for beam in beams {
        let [a, b] = beam.endpoints;
        assert!(a < nodes.len() && b < nodes.len() && a != b);
        assert!(endpoints.insert((a.min(b), a.max(b))));
        let distance = nodes[a].rest_position.distance(nodes[b].rest_position);
        assert!(beam.rest_length.is_finite() && beam.rest_length > 0.0);
        assert!((beam.rest_length - distance).abs() < 1e-6);

        let ring_distance = (a / n).abs_diff(b / n);
        let segment_distance = (a % n).abs_diff(b % n);
        let consecutive = segment_distance == 1 || segment_distance == n - 1;
        match beam.family {
            BeamFamily::Circumferential => {
                assert_eq!(ring_distance, 0);
                assert!(consecutive);
                family_counts[0] += 1;
            }
            BeamFamily::CrossSection => {
                assert_eq!(ring_distance, 1);
                assert_eq!(segment_distance, 0);
                family_counts[1] += 1;
            }
            BeamFamily::Diagonal => {
                assert_eq!(ring_distance, 1);
                assert!(consecutive);
                family_counts[2] += 1;
            }
        }
    }
    assert_eq!(family_counts, [5 * n, 4 * n, 8 * n]);
    for ring in 0..5 {
        let first = ring * n;
        let last = first + n - 1;
        assert!(endpoints.contains(&(first, last)));
        if ring < 4 {
            assert!(endpoints.contains(&(last, first + n)));
            assert!(endpoints.contains(&(first, last + n)));
        }
    }
    topology
}

#[test]
fn rejects_invalid_geometry() {
    let default = TyreConfig::default();
    let cases = [
        (
            TyreConfig {
                segments: 2,
                ..default
            },
            TyreConfigError::TooFewSegments,
        ),
        (
            TyreConfig {
                bead_radius: 0.0,
                ..default
            },
            TyreConfigError::InvalidRadii,
        ),
        (
            TyreConfig {
                shoulder_radius: default.bead_radius,
                ..default
            },
            TyreConfigError::InvalidRadii,
        ),
        (
            TyreConfig {
                tread_radius: 0.30,
                ..default
            },
            TyreConfigError::InvalidRadii,
        ),
        (
            TyreConfig {
                tread_radius: f32::INFINITY,
                ..default
            },
            TyreConfigError::InvalidRadii,
        ),
        (
            TyreConfig {
                shoulder_width: 0.10,
                ..default
            },
            TyreConfigError::InvalidWidths,
        ),
        (
            TyreConfig {
                bead_width: f32::NAN,
                ..default
            },
            TyreConfigError::InvalidWidths,
        ),
        (
            TyreConfig {
                bead_width: f32::from_bits(1),
                ..default
            },
            TyreConfigError::UnrepresentableGeometry,
        ),
        (
            TyreConfig {
                segments: 3,
                tread_radius: f32::MAX,
                ..default
            },
            TyreConfigError::UnrepresentableGeometry,
        ),
        (
            TyreConfig {
                segments: usize::MAX,
                ..default
            },
            TyreConfigError::CapacityExceeded,
        ),
    ];
    for (config, expected) in cases {
        assert_eq!(TyreTopology::generate(&config), Err(expected));
    }
}

#[test]
fn accepts_a_flat_tread_and_equal_bead_and_shoulder_widths() {
    let default = TyreConfig::default();
    check_topology(&TyreConfig {
        tread_radius: default.shoulder_radius,
        bead_width: default.shoulder_width,
        ..default
    });
}

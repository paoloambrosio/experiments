use std::f32::consts::FRAC_PI_2;

use bevy::{input::InputSystems, prelude::*};
use vehicle_physics::avian_tyre::AvianTyre;

use crate::{
    BEAM_STYLES, OrbitCamera,
    drop_experiment::CLEARANCE_M,
    elastic_experiment::RADIAL_SUBSTEPS_PER_SECOND,
    orbit_camera,
    performance::{self, Benchmark, WindowOptions},
    rolling_experiment::{self, BELT_SPEED_MPS, LoadCase, Phase, RollingBench},
};

const LONGITUDINAL: Color = Color::srgb(0.40, 0.76, 1.0);
const LATERAL: Color = Color::srgb(1.0, 0.48, 0.38);
const BELT: Color = Color::srgb(0.22, 0.32, 0.44);

#[derive(Component)]
struct RimMesh;

#[derive(Component)]
struct RollingReadout;

#[derive(Resource, Default)]
struct Selection(LoadCase);

pub fn run(window: WindowOptions) -> AppExit {
    let mut app = App::new();
    app.insert_resource(ClearColor(Color::srgb(0.047, 0.063, 0.091)))
        .init_resource::<Selection>()
        .add_plugins(DefaultPlugins.set(window.plugin("Vehicle Lab | Rolling and Grip")))
        .add_systems(Startup, setup)
        .add_systems(PreUpdate, input.after(InputSystems))
        .add_systems(Update, (draw, orbit_camera).chain());
    rolling_experiment::install(&mut app, RADIAL_SUBSTEPS_PER_SECOND);
    if window.benchmark {
        app.insert_resource(Selection(LoadCase::Drive));
        Benchmark::install(&mut app, "Rolling with 10 N m drive torque");
        app.add_systems(Last, benchmark_rolling.before(performance::finish));
    }
    app.run()
}

fn benchmark_rolling(mut bench: ResMut<RollingBench>, mut benchmark: ResMut<Benchmark>) {
    if benchmark.observe(
        bench.is_at(Phase::Ready),
        bench.is_at(Phase::Complete),
        bench.error.as_deref(),
    ) {
        bench.replay(LoadCase::Drive);
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut gizmos: ResMut<Assets<GizmoAsset>>,
) {
    let orbit = OrbitCamera {
        focus: Vec3::Y * 0.32,
        ..default()
    };
    commands.spawn((Camera3d::default(), orbit.transform(), orbit));
    commands.spawn((
        RimMesh,
        Mesh3d(meshes.add(Cylinder::new(0.197, 0.165))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.32, 0.39, 0.48),
            metallic: 0.25,
            ..default()
        })),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 12_000.0,
            ..default()
        },
        Transform::from_xyz(1.0, 2.0, 1.4).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn(Gizmo {
        handle: gizmos.add(GizmoAsset::new()),
        ..default()
    });
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: px(20),
                top: px(20),
                width: px(288),
                padding: UiRect::all(px(16)),
                flex_direction: FlexDirection::Column,
                row_gap: px(14),
                ..default()
            },
            BackgroundColor(Color::srgb(0.085, 0.112, 0.155)),
        ))
        .with_children(|panel| {
            panel.spawn((Text::new("Rolling and grip"), TextFont::from_font_size(23.0)));
            panel.spawn((RollingReadout, Text::default(), TextFont::from_font_size(16.0)));
            panel.spawn((Text::new("Blue   Longitudinal force"), TextFont::from_font_size(14.0), TextColor(LONGITUDINAL)));
            panel.spawn((Text::new("Coral  Lateral force"), TextFont::from_font_size(14.0), TextColor(LATERAL)));
            panel.spawn((
                Text::new("1  Free     2  Drive\n3  Brake    4  Lateral\nEnter   Run / replay\nBackspace   Reset\nLeft drag   Orbit\nScroll   Zoom\nR   Reset view"),
                TextFont::from_font_size(14.0),
                TextColor(Color::srgb(0.77, 0.83, 0.91)),
            ));
        });
}

fn input(
    keys: Res<ButtonInput<KeyCode>>,
    mut selection: ResMut<Selection>,
    mut bench: ResMut<RollingBench>,
) {
    for (key, case) in [
        KeyCode::Digit1,
        KeyCode::Digit2,
        KeyCode::Digit3,
        KeyCode::Digit4,
    ]
    .into_iter()
    .zip(LoadCase::ALL)
    {
        if keys.just_pressed(key) {
            selection.0 = case;
        }
    }
    if keys.just_pressed(KeyCode::Backspace) {
        bench.reset();
    } else if keys.just_pressed(KeyCode::Enter) {
        bench.replay(selection.0);
    }
}

fn draw(
    bench: Res<RollingBench>,
    selection: Res<Selection>,
    wheels: Query<&AvianTyre>,
    gizmo: Single<&Gizmo>,
    mut assets: ResMut<Assets<GizmoAsset>>,
    mut camera: Single<&mut OrbitCamera>,
    mut rim: Single<&mut Transform, With<RimMesh>>,
    mut readout: Single<&mut Text, With<RollingReadout>>,
) {
    if !bench.is_changed() && !selection.is_changed() {
        return;
    }
    let mut lines = assets.get_mut(&gizmo.handle).expect("rolling gizmo");
    lines.clear();
    draw_belt(&mut lines, bench.belt_distance_m() as f32);

    let mounted = bench
        .wheel
        .and_then(|entity| wheels.get(entity).ok())
        .map(AvianTyre::tyre)
        .or(bench.baseline.as_ref());
    let preview = bench.preparation.as_ref().map(|preparation| {
        let tyre = &preparation.tyre;
        let lowest = tyre
            .positions()
            .iter()
            .map(|p| p.y)
            .fold(f64::INFINITY, f64::min);
        (tyre, Vec3::Y * (CLEARANCE_M - lowest) as f32)
    });
    if let Some(tyre) = mounted {
        let motion = tyre.rim();
        rim.translation = motion.centre.as_vec3();
        rim.rotation = motion.rotation.as_quat() * Quat::from_rotation_z(-FRAC_PI_2);
        for index in tyre.active_contacts() {
            let point = tyre.positions()[index].as_vec3();
            lines.line(point, point.with_y(0.006), LATERAL);
        }
    } else if let Some((_, offset)) = preview {
        rim.translation = offset;
        rim.rotation = Quat::from_rotation_z(-FRAC_PI_2);
    }
    if let Some((topology, positions, offset)) = mounted
        .map(|tyre| (tyre.topology(), tyre.positions(), Vec3::ZERO))
        .or_else(|| preview.map(|(tyre, offset)| (tyre.topology(), tyre.positions(), offset)))
    {
        for beam in topology.beams() {
            let [a, b] = beam
                .endpoints
                .map(|index| positions[index].as_vec3() + offset);
            let style = BEAM_STYLES
                .iter()
                .find(|style| style.family == beam.family)
                .unwrap();
            lines.line(a, b, style.colour);
        }
    }
    camera.focus.x = rim.translation.x;

    let phase = match bench.phase {
        Phase::Inflating => "Inflating to 30 kPa gauge",
        Phase::Settling | Phase::StartBelt => "Settling on the stationary belt",
        Phase::FreeRolling => "Finding free-rolling speed",
        Phase::Ready => "Ready to replay",
        Phase::Running => "Running",
        Phase::Complete => "Run complete (2-turn mean)",
        Phase::Failed => "Simulation stopped",
    };
    let mut text = format!("{phase}\nSelected   {}", selection.0.name());
    if let Some(error) = &bench.error {
        text.push_str(&format!("\n{error}"));
    }
    if matches!(bench.phase, Phase::Running | Phase::Complete) {
        let sample = bench.mean.unwrap_or(bench.latest);
        let slip = sample.slip_ratio(bench.radius_m.expect("calibrated wheel"));
        text.push_str(&format!(
            "\nCase   {}\nBelt   -{BELT_SPEED_MPS:.1} m/s along Z\nRim torque   {:+.0} N m\nLateral load   {:+.0} N\nSpin   {:.2} rad/s\nSlip   {:+.2}%\nSlip angle   {:+.2} deg\nNormal load   {:.1} N\nAxle Z   {:+.1} N\nAxle X   {:+.1} N",
            bench.case.name(),
            bench.case.torque_nm(),
            bench.case.lateral_load_n(),
            sample.spin_rad_s,
            100.0 * slip,
            sample.slip_angle_rad.to_degrees(),
            sample.floor_force_n.y,
            sample.axle_force_n.z,
            sample.axle_force_n.x,
        ));
        let origin = rim.translation + Vec3::Y * 0.40;
        let force = sample.axle_force_n.as_vec3() * 0.008;
        draw_arrow(&mut lines, origin, Vec3::Z * force.z, LONGITUDINAL);
        draw_arrow(&mut lines, origin, Vec3::X * force.x, LATERAL);
    }
    readout.0 = text;
}

fn draw_belt(lines: &mut GizmoAsset, distance: f32) {
    for x in [-0.65, 0.65] {
        lines.line(Vec3::new(x, 0.0, -1.2), Vec3::new(x, 0.0, 1.2), BELT);
    }
    for index in -7..=8 {
        let z = index as f32 * 0.15 - distance.rem_euclid(0.15);
        lines.line(Vec3::new(-0.65, 0.0, z), Vec3::new(0.65, 0.0, z), BELT);
    }
    draw_arrow(lines, Vec3::new(0.7, 0.0, 0.4), Vec3::NEG_Z * 0.32, BELT);
}

fn draw_arrow(lines: &mut GizmoAsset, origin: Vec3, vector: Vec3, colour: Color) {
    let length = vector.length();
    if length < 0.005 {
        return;
    }
    let tip = origin + vector;
    let head = (length * 0.3).min(0.04) * vector / length;
    let width = Vec3::Y.cross(head) * 0.5;
    lines.line(origin, tip, colour);
    lines.line(tip, tip - head + width, colour);
    lines.line(tip, tip - head - width, colour);
}

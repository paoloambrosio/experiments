use std::f32::consts::FRAC_PI_2;
use vehicle_physics::reference_car::ReferenceTyre;

use bevy::{
    asset::RenderAssetUsages, ecs::change_detection::NonSendMut, input::InputSystems,
    mesh::PrimitiveTopology, prelude::*,
};
use vehicle_physics::avian_tyre::AvianTyre;

use crate::{
    BEAM_STYLES, OrbitCamera,
    corner_experiment::{
        BODY_MOUNT_DROP_M, CORNER_TICKS, CornerRoad, CornerRun, CornerSample, SETTLE_TICKS,
        STEP_RISE_M, STEP_Z_M, SUSPENSION,
    },
    elastic_experiment::TICKS_PER_SECOND,
    loaded_tyre::{self, BEAD_RADIUS_M, RATES},
    orbit_camera,
    performance::{self, Benchmark, WindowOptions},
};

#[derive(Default)]
enum Action {
    #[default]
    None,
    Start,
    Reset,
}

struct Replay {
    experiment: ReferenceTyre,
    simulation: Option<CornerRun>,
    sample: Option<CornerSample>,
    selected_road: CornerRoad,
    running: bool,
    action: Action,
    error: Option<String>,
}

#[derive(Component)]
struct Readout;

#[derive(Component)]
struct Rim;

#[derive(Component)]
struct Body;

#[derive(Component)]
struct StepTop;

#[derive(Component)]
struct AnalyticFloor;

#[derive(Component)]
struct Ramp;

pub fn run(window: WindowOptions) -> AppExit {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(window.plugin("Vehicle Lab | Loaded Wheel and Suspension")))
        .insert_resource(ClearColor(Color::srgb(0.047, 0.063, 0.091)))
        .insert_resource(Time::<Fixed>::from_hz(f64::from(TICKS_PER_SECOND)))
        .add_systems(Startup, setup)
        .add_systems(PreUpdate, input.after(InputSystems))
        .add_systems(FixedUpdate, advance)
        .add_systems(Update, (draw, orbit_camera).chain());
    app.world_mut().insert_non_send(Replay {
        experiment: ReferenceTyre::new(RATES[0]),
        simulation: None,
        sample: None,
        selected_road: CornerRoad::Step,
        running: false,
        action: Action::None,
        error: None,
    });
    if window.benchmark {
        Benchmark::install(&mut app, "Loaded corner crossing the 20 mm step");
        app.add_systems(Last, benchmark_corner.before(performance::finish));
    }
    app.run()
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut gizmos: ResMut<Assets<GizmoAsset>>,
) {
    let orbit = OrbitCamera {
        focus: Vec3::Y * 0.39,
        distance: 2.05,
        ..default()
    };
    commands.spawn((Camera3d::default(), orbit.transform(), orbit));
    commands.spawn((
        DirectionalLight {
            illuminance: 10_000.0,
            ..default()
        },
        Transform::from_xyz(1.0, 2.0, 1.4).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn(Gizmo {
        handle: gizmos.add(GizmoAsset::new()),
        ..default()
    });
    commands.spawn((
        Rim,
        Mesh3d(meshes.add(Cylinder::new(BEAD_RADIUS_M as f32, 0.17))),
        MeshMaterial3d(materials.add(Color::srgb(0.26, 0.33, 0.42))),
    ));
    commands.spawn((
        Body,
        Mesh3d(meshes.add(Cuboid::new(0.45, 0.16, 0.45))),
        MeshMaterial3d(materials.add(Color::srgb(0.15, 0.35, 0.59))),
    ));
    commands.spawn((
        AnalyticFloor,
        Mesh3d(meshes.add(Cuboid::new(1.4, 0.035, 16.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.12, 0.17, 0.23))),
        Transform::from_xyz(0.0, -0.0175, 5.0),
    ));
    commands.spawn((
        StepTop,
        Mesh3d(meshes.add(Cuboid::new(1.4, STEP_RISE_M as f32, 12.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.22, 0.29, 0.37))),
        Transform::from_xyz(0.0, STEP_RISE_M as f32 * 0.5, STEP_Z_M as f32 + 6.0),
    ));
    let (vertices, triangles) = CornerRoad::Ramp.geometry();
    let mut ramp = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD,
    );
    ramp.insert_attribute(
        Mesh::ATTRIBUTE_POSITION,
        triangles
            .iter()
            .flatten()
            .map(|&index| vertices[index].as_vec3().to_array())
            .collect::<Vec<_>>(),
    );
    ramp.compute_flat_normals();
    commands.spawn((
        Ramp,
        Mesh3d(meshes.add(ramp)),
        MeshMaterial3d(materials.add(Color::srgb(0.19, 0.27, 0.35))),
        Visibility::Hidden,
    ));
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: px(20),
                top: px(20),
                width: px(308),
                padding: UiRect::all(px(16)),
                flex_direction: FlexDirection::Column,
                row_gap: px(12),
                ..default()
            },
            BackgroundColor(Color::srgb(0.085, 0.112, 0.155)),
        ))
        .with_children(|panel| {
            panel.spawn((Text::new("Loaded suspension"), TextFont::from_font_size(23.0)));
            panel.spawn((Readout, Text::default(), TextFont::from_font_size(16.0)));
            panel.spawn((
                Text::new("1  Flat    2  Step    3  Ramp\nEnter   Run / replay\nBackspace   Reset\nLeft drag   Orbit\nScroll   Zoom\nR   Reset view"),
                TextFont::from_font_size(14.0),
                TextColor(Color::srgb(0.77, 0.83, 0.91)),
            ));
        });
}

fn input(keys: Res<ButtonInput<KeyCode>>, mut replay: NonSendMut<Replay>) {
    if keys.just_pressed(KeyCode::Digit1) {
        replay.selected_road = CornerRoad::Flat;
    } else if keys.just_pressed(KeyCode::Digit2) {
        replay.selected_road = CornerRoad::Step;
    } else if keys.just_pressed(KeyCode::Digit3) {
        replay.selected_road = CornerRoad::Ramp;
    }
    if keys.just_pressed(KeyCode::Backspace) {
        replay.action = Action::Reset;
    } else if keys.just_pressed(KeyCode::Enter) {
        replay.action = Action::Start;
    }
}

fn advance(mut replay: NonSendMut<Replay>) {
    match std::mem::take(&mut replay.action) {
        Action::Reset => {
            if !replay.experiment.is_ready() {
                replay.experiment = ReferenceTyre::new(RATES[0]);
            }
            replay.simulation = None;
            replay.sample = None;
            replay.running = false;
            replay.error = None;
        }
        Action::Start if replay.experiment.is_ready() => {
            replay.sample = None;
            replay.error = None;
            match CornerRun::new(RATES[0], &replay.experiment.tyre, replay.selected_road) {
                Ok(simulation) => {
                    replay.simulation = Some(simulation);
                    replay.running = true;
                }
                Err(error) => replay.error = Some(error.to_string()),
            }
        }
        _ => {}
    }
    if replay.error.is_some() {
        return;
    }
    if !replay.experiment.is_ready() {
        for _ in 0..(RATES[0] / TICKS_PER_SECOND) {
            if let Err(error) = replay.experiment.advance() {
                replay.error = Some(error.to_string());
                break;
            }
            if replay.experiment.is_ready() {
                break;
            }
        }
    } else if replay.running {
        let simulation = replay.simulation.as_mut().unwrap();
        match simulation.advance() {
            Ok(sample) => {
                let running = simulation.tick < CORNER_TICKS;
                replay.sample = Some(sample);
                replay.running = running;
            }
            Err(error) => {
                replay.error = Some(error.to_string());
                replay.running = false;
            }
        }
    }
}

fn benchmark_corner(mut replay: NonSendMut<Replay>, mut benchmark: ResMut<Benchmark>) {
    if benchmark.observe(
        replay.experiment.is_ready(),
        replay.sample.is_some() && !replay.running,
        replay.error.as_deref(),
    ) {
        replay.action = Action::Start;
    }
}

fn draw(
    replay: bevy::ecs::change_detection::NonSend<Replay>,
    mut camera: Single<&mut OrbitCamera>,
    mut readout: Single<&mut Text, With<Readout>>,
    mut rim: Single<&mut Transform, (With<Rim>, Without<Body>)>,
    mut body: Single<&mut Transform, (With<Body>, Without<Rim>)>,
    mut road_meshes: Query<
        (&mut Visibility, Has<StepTop>, Has<Ramp>),
        Or<(With<StepTop>, With<AnalyticFloor>, With<Ramp>)>,
    >,
    gizmo: Single<&Gizmo>,
    mut assets: ResMut<Assets<GizmoAsset>>,
) {
    let mut lines = assets.get_mut(&gizmo.handle).unwrap();
    lines.clear();
    let run = replay.simulation.as_ref();
    let road = run.map_or(replay.selected_road, |run| run.road);
    for (mut visibility, is_step, is_ramp) in &mut road_meshes {
        let visible = if is_step {
            road == CornerRoad::Step
        } else if is_ramp {
            road == CornerRoad::Ramp
        } else {
            road != CornerRoad::Ramp
        };
        *visibility = if visible {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
    let tyre = &replay.experiment.tyre;
    let (topology, positions, offset) = if let Some(run) = run {
        let wheel = run
            .app
            .world()
            .get::<AvianTyre>(run.wheel.rim)
            .unwrap()
            .tyre();
        let motion = wheel.rim();
        rim.translation = motion.centre.as_vec3();
        rim.rotation = motion.rotation.as_quat() * Quat::from_rotation_z(-FRAC_PI_2);
        body.translation = rim.translation.with_y(replay.sample.map_or(
            rim.translation.y + SUSPENSION.free_length_m + BODY_MOUNT_DROP_M,
            |s| s.body_y as f32,
        ));
        (wheel.topology(), wheel.positions(), Vec3::ZERO)
    } else {
        rim.translation = Vec3::Y * loaded_tyre::unloaded_radius(tyre) as f32;
        rim.rotation = Quat::from_rotation_z(-FRAC_PI_2);
        body.translation =
            rim.translation + Vec3::Y * (SUSPENSION.free_length_m + BODY_MOUNT_DROP_M);
        (tyre.topology(), tyre.positions(), rim.translation)
    };
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
    let outside = Vec3::X * 0.215;
    let lower = rim.translation + outside;
    let upper = body.translation + outside - Vec3::Y * BODY_MOUNT_DROP_M;
    let spring_colour = Color::srgb(1.0, 0.74, 0.30);
    lines.line(
        upper,
        body.translation + outside - Vec3::Y * 0.075,
        spring_colour,
    );
    for segment in 0..8 {
        let point = |i: i32| {
            lower.lerp(upper, i as f32 / 8.0) + Vec3::Z * if i % 2 == 0 { -0.023 } else { 0.023 }
        };
        lines.line(point(segment), point(segment + 1), spring_colour);
    }
    let road_line = Color::srgb(0.53, 0.62, 0.72);
    if road == CornerRoad::Ramp {
        let (vertices, triangles) = road.geometry();
        for [a, b, c] in triangles {
            let [a, b, c] = [a, b, c].map(|index| vertices[index].as_vec3() + Vec3::Y * 0.001);
            lines.line(a, b, road_line);
            lines.line(b, c, road_line);
            lines.line(c, a, road_line);
        }
    } else {
        for index in -4..=22 {
            let z = index as f32 * 0.5;
            let y = road.height(f64::from(z)) as f32 + 0.001;
            for x in [-0.52, 0.52] {
                lines.line(Vec3::new(x, y, z), Vec3::new(x, y, z + 0.20), road_line);
            }
        }
    }
    camera.focus.z = rim.translation.z;
    let phase = if replay.error.is_some() {
        "Simulation stopped"
    } else if let Some(run) = run {
        if !replay.running {
            "Replay complete"
        } else if run.tick <= SETTLE_TICKS {
            "Settling under load"
        } else {
            "Towing along +Z"
        }
    } else if replay.experiment.is_ready() {
        "Ready to replay"
    } else {
        "Inflating to 150 kPa gauge"
    };
    let mut text = format!("{phase}\nSelected   {}", replay.selected_road.name());
    if let Some(sample) = replay.sample {
        text.push_str(&format!(
            "\n{}   {:.2} s\nAxle height   {:.1} mm\nBody height   {:.1} mm\nSpring compression   {:.1} mm\nRoad support   {:.0} N",
            road.name(), sample.time, sample.position.y * 1000.0, sample.body_y * 1000.0,
            sample.compression * 1000.0, sample.support,
        ));
    }
    if let Some(error) = &replay.error {
        text.push_str(&format!("\n{error}"));
    }
    readout.0 = text;
}

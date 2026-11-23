use std::f32::consts::FRAC_PI_2;

use bevy::{
    ecs::change_detection::{NonSend, NonSendMut},
    input::InputSystems,
    prelude::*,
};
use vehicle_physics::avian_tyre::AvianTyre;

use crate::{
    BEAM_STYLES, OrbitCamera, OrbitHome,
    elastic_experiment::{Experiment, TICKS_PER_SECOND},
    handling_experiment::{
        BODY_SIZE, CASES, COM_HEIGHT, HandlingRun, MOUNTS, Phase, Sample, WHEEL_NAMES,
    },
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
    preparation: Experiment,
    run: Option<HandlingRun>,
    selected: usize,
    path: Vec<(Vec3, Phase)>,
    action: Action,
    error: Option<String>,
}

#[derive(Component)]
struct Readout;

#[derive(Component)]
struct Body;

#[derive(Component)]
struct Rim(usize);

pub fn run(window: WindowOptions) -> AppExit {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(window.plugin("Vehicle Lab | Whole-Car Handling")))
        .insert_resource(ClearColor(Color::srgb(0.047, 0.063, 0.091)))
        .insert_resource(Time::<Fixed>::from_hz(f64::from(TICKS_PER_SECOND)))
        .add_systems(Startup, setup)
        .add_systems(PreUpdate, input.after(InputSystems))
        .add_systems(FixedUpdate, advance)
        .add_systems(Update, (draw, orbit_camera).chain());
    app.world_mut().insert_non_send(Replay {
        preparation: loaded_tyre::preparation(RATES[0], true),
        run: None,
        selected: if window.benchmark { CASES.len() - 1 } else { 0 },
        path: Vec::new(),
        action: Action::None,
        error: None,
    });
    if window.benchmark {
        Benchmark::install(
            &mut app,
            "Four live tyres | 8 m/s limit braking in a left turn",
        );
        app.add_systems(Last, benchmark.before(performance::finish));
    }
    app.run()
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut gizmos: ResMut<Assets<GizmoAsset>>,
) {
    let home = OrbitHome {
        pitch: 0.52,
        distance: 7.2,
        min_distance: 3.5,
        max_distance: 32.0,
    };
    let orbit = OrbitCamera {
        focus: Vec3::Y * COM_HEIGHT,
        pitch: home.pitch,
        distance: home.distance,
        ..default()
    };
    commands.spawn((Camera3d::default(), orbit.transform(), orbit, home));
    commands.spawn((
        DirectionalLight {
            illuminance: 10_000.0,
            ..default()
        },
        Transform::from_xyz(1.0, 2.0, -1.4).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn(Gizmo {
        handle: gizmos.add(GizmoAsset::new()),
        ..default()
    });
    commands
        .spawn((
            Body,
            Mesh3d(meshes.add(Cuboid::from_size(BODY_SIZE.with_x(1.14)))),
            MeshMaterial3d(materials.add(Color::srgb(0.19, 0.34, 0.53))),
        ))
        .with_children(|body| {
            body.spawn((
                Mesh3d(meshes.add(Cuboid::new(1.08, 0.06, 0.16))),
                MeshMaterial3d(materials.add(Color::srgb(0.91, 0.83, 0.54))),
                Transform::from_xyz(0.0, BODY_SIZE.y / 2.0, 1.54),
            ));
        });
    let rim = meshes.add(Cylinder::new(BEAD_RADIUS_M as f32, 0.15));
    let rim_colour = materials.add(Color::srgb(0.34, 0.43, 0.53));
    for index in 0..4 {
        commands.spawn((
            Rim(index),
            Mesh3d(rim.clone()),
            MeshMaterial3d(rim_colour.clone()),
        ));
    }
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(800.0, 0.05, 800.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.10, 0.135, 0.18))),
        Transform::from_xyz(0.0, -0.025, 0.0),
    ));
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: px(20), top: px(20), width: px(332),
                padding: UiRect::all(px(16)),
                flex_direction: FlexDirection::Column, row_gap: px(10), ..default()
            },
            BackgroundColor(Color::srgb(0.085, 0.112, 0.155)),
        ))
        .with_children(|panel| {
            panel.spawn((Text::new("Whole-car handling"), TextFont::from_font_size(23.0)));
            panel.spawn((Readout, Text::default(), TextFont::from_font_size(16.0)));
            panel.spawn((
                Text::new("1-9 / Left / Right   Select case\nEnter   Run / replay\nBackspace   Reset\nLeft drag   Orbit     Scroll   Zoom\nR   Reset camera"),
                TextFont::from_font_size(14.0),
                TextColor(Color::srgb(0.77, 0.83, 0.91)),
            ));
        });
}

fn input(keys: Res<ButtonInput<KeyCode>>, mut replay: NonSendMut<Replay>) {
    let digits = [
        KeyCode::Digit1,
        KeyCode::Digit2,
        KeyCode::Digit3,
        KeyCode::Digit4,
        KeyCode::Digit5,
        KeyCode::Digit6,
        KeyCode::Digit7,
        KeyCode::Digit8,
        KeyCode::Digit9,
    ];
    if let Some(index) = digits.into_iter().position(|key| keys.just_pressed(key)) {
        replay.selected = index;
    } else if keys.just_pressed(KeyCode::ArrowLeft) {
        replay.selected = (replay.selected + CASES.len() - 1) % CASES.len();
    } else if keys.just_pressed(KeyCode::ArrowRight) {
        replay.selected = (replay.selected + 1) % CASES.len();
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
            if !replay.preparation.is_ready() {
                replay.preparation = loaded_tyre::preparation(RATES[0], true);
            }
            replay.run = None;
            replay.path.clear();
            replay.error = None;
        }
        Action::Start if replay.preparation.is_ready() => {
            replay.path.clear();
            replay.error = None;
            match HandlingRun::new(RATES[0], &replay.preparation.tyre, CASES[replay.selected]) {
                Ok(run) => replay.run = Some(run),
                Err(error) => {
                    replay.run = None;
                    replay.error = Some(error.to_string());
                }
            }
        }
        _ => {}
    }
    if replay.error.is_some() {
        return;
    }
    if !replay.preparation.is_ready() {
        for _ in 0..replay.preparation.substeps_per_tick() {
            if let Err(error) = replay.preparation.prepare_step() {
                replay.error = Some(error.to_string());
                break;
            }
            if replay.preparation.is_ready() {
                break;
            }
        }
    } else if let Some(run) = &mut replay.run
        && run.phase != Phase::Done
    {
        match run.advance() {
            Ok(sample) if sample.phase != Phase::Settle => {
                replay
                    .path
                    .push((sample.position.with_y(0.012), sample.phase));
            }
            Ok(_) => {}
            Err(error) => replay.error = Some(error.to_string()),
        }
    }
}

fn benchmark(mut replay: NonSendMut<Replay>, mut bench: ResMut<Benchmark>) {
    if bench.observe(
        replay.preparation.is_ready(),
        replay
            .run
            .as_ref()
            .is_some_and(|run| run.phase == Phase::Done),
        replay.error.as_deref(),
    ) {
        replay.action = Action::Start;
    }
}

fn draw(
    replay: NonSend<Replay>,
    mut camera: Single<&mut OrbitCamera>,
    mut readout: Single<&mut Text, With<Readout>>,
    mut body: Single<&mut Transform, With<Body>>,
    mut rims: Query<(&Rim, &mut Transform), Without<Body>>,
    gizmo: Single<&Gizmo>,
    mut assets: ResMut<Assets<GizmoAsset>>,
) {
    let mut lines = assets.get_mut(&gizmo.handle).unwrap();
    lines.clear();
    let run = replay.run.as_ref();
    let sample = run.and_then(|r| r.sample);
    body.translation = sample.map_or(Vec3::Y * COM_HEIGHT, |s| s.position);
    body.rotation = sample.map_or(Quat::IDENTITY, |s| s.rotation);
    camera.focus = body.translation;
    draw_grid(&mut lines, body.translation);
    for pair in replay.path.windows(2) {
        let [(from, _), (to, phase)] = [pair[0], pair[1]];
        let color = if phase == Phase::Brake {
            Color::srgb(1.0, 0.55, 0.23)
        } else {
            Color::srgb(0.45, 0.85, 1.0)
        };
        lines.line(from, to, color);
    }

    let tyre = &replay.preparation.tyre;
    for (Rim(index), mut transform) in &mut rims {
        let preview = MOUNTS[*index].with_y(loaded_tyre::unloaded_radius(tyre) as f32);
        let (positions, offset, centre, rotation) = if let Some(run) = run {
            let mounted = run
                .app
                .world()
                .get::<AvianTyre>(run.wheels[*index].rim)
                .unwrap()
                .tyre();
            let rim = mounted.rim();
            (
                mounted.positions(),
                Vec3::ZERO,
                rim.centre.as_vec3(),
                rim.rotation.as_quat(),
            )
        } else {
            (tyre.positions(), preview, preview, Quat::IDENTITY)
        };
        transform.translation = centre;
        transform.rotation = rotation * Quat::from_rotation_z(-FRAC_PI_2);
        for beam in tyre.topology().beams() {
            let [a, b] = beam.endpoints.map(|i| positions[i].as_vec3() + offset);
            let style = BEAM_STYLES
                .iter()
                .find(|style| style.family == beam.family)
                .unwrap();
            lines.line(a, b, style.colour);
        }
        if let Some(Sample { wheels, .. }) = sample {
            let wheel = wheels[*index];
            let side = MOUNTS[*index].x.signum();
            let base = centre.with_y(0.02) + body.rotation * Vec3::X * (side * 0.23);
            let color = if wheel.locked {
                Color::srgb(1.0, 0.39, 0.23)
            } else {
                Color::srgb(1.0, 0.86, 0.37)
            };
            lines.arrow(base, base + Vec3::Y * (wheel.road.y as f32 / 3500.0), color);
        }
    }

    let mut text = format!(
        "Selected {} / {}\n{}",
        replay.selected + 1,
        CASES.len(),
        CASES[replay.selected].label
    );
    if let Some(error) = &replay.error {
        text.push_str(&format!("\n{error}"));
    } else if let (Some(run), Some(s)) = (run, sample) {
        let phase = if run.phase == Phase::Done {
            run.phase
        } else {
            s.phase
        };
        text.push_str(&format!(
            "\n\n{} / {}\n{:.2} s    {:.2} m/s\nYaw rate  {:+.1} deg/s",
            phase.label(),
            run.case.id,
            s.time,
            s.speed,
            s.yaw_rate.to_degrees(),
        ));
        for (name, wheel) in WHEEL_NAMES.into_iter().zip(s.wheels) {
            let lock = if wheel.locked { "   LOCKED" } else { "" };
            text.push_str(&format!("\n{name}    {:>4.0} N{lock}", wheel.road.y));
        }
        if run.phase == Phase::Done {
            if run.results.radius > 0.0 {
                text.push_str(&format!("\nTurn radius   {:.2} m", run.results.radius));
            }
            if run.case.brake.is_some() {
                text.push_str(&format!(
                    "\nStopping distance   {:.2} m",
                    run.results.stop_distance
                ));
                for (name, time) in WHEEL_NAMES.into_iter().zip(run.results.lock_time) {
                    if let Some(time) = time {
                        text.push_str(&format!("\n{name} locked at {time:.2} s of braking"));
                    }
                }
            }
        }
    } else if replay.preparation.is_ready() {
        text.push_str("\n\nReady to replay");
    } else {
        text.push_str("\n\nPreparing sealed tyres");
    }
    readout.0 = text;
}

fn draw_grid(lines: &mut GizmoAsset, position: Vec3) {
    let base_x = (position.x / 5.0).floor() * 5.0;
    let base_z = (position.z / 5.0).floor() * 5.0;
    let color = Color::srgb(0.25, 0.32, 0.40);
    for i in -12..=12 {
        let x = base_x + i as f32 * 5.0;
        let z = base_z + i as f32 * 5.0;
        lines.line(
            Vec3::new(x, 0.002, base_z - 60.0),
            Vec3::new(x, 0.002, base_z + 60.0),
            color,
        );
        lines.line(
            Vec3::new(base_x - 60.0, 0.002, z),
            Vec3::new(base_x + 60.0, 0.002, z),
            color,
        );
    }
}

mod elastic_experiment;

use std::{f32::consts::TAU, process::ExitCode};

use bevy::{
    camera::visibility::RenderLayers,
    input::{
        InputSystems,
        mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll, MouseScrollUnit},
    },
    prelude::*,
    window::WindowResolution,
};
use elastic_experiment::{Experiment, LAB_SUBSTEPS_PER_SECOND, LOAD_NEWTONS, TICKS_PER_SECOND};
use vehicle_physics::{elastic_tyre::ElasticTyreError, tyre::BeamFamily};

#[derive(Resource)]
struct LabTyre {
    experiment: Experiment,
    mode: Mode,
}

enum Mode {
    Idle,
    Replay,
    Manual { pressed: bool },
    Failed(ElasticTyreError),
}

#[derive(Component)]
struct NodeIndex(usize);

#[derive(Component)]
struct ExperimentReadout;

#[derive(Component)]
struct OrbitCamera {
    yaw: f32,
    pitch: f32,
    distance: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            yaw: 0.85,
            pitch: 0.33,
            distance: 1.35,
        }
    }
}

impl OrbitCamera {
    fn transform(&self) -> Transform {
        let rotation = Quat::from_rotation_y(self.yaw) * Quat::from_rotation_x(-self.pitch);
        Transform::from_translation(rotation * Vec3::Z * self.distance).with_rotation(rotation)
    }
}

#[derive(Clone, Copy, Component)]
struct BeamStyle {
    family: BeamFamily,
    key: KeyCode,
    label: &'static str,
    colour: Color,
}

impl BeamStyle {
    fn legend_label(self, visible: bool) -> String {
        let status = if visible { "On" } else { "Off" };
        format!("{:<20} [{status}]", self.label)
    }
}

const BEAM_STYLES: [BeamStyle; 3] = [
    BeamStyle {
        family: BeamFamily::Circumferential,
        key: KeyCode::Digit1,
        label: "1   Circumferential",
        colour: Color::srgb(0.25, 0.87, 0.75),
    },
    BeamStyle {
        family: BeamFamily::CrossSection,
        key: KeyCode::Digit2,
        label: "2   Cross-section",
        colour: Color::srgb(1.0, 0.71, 0.32),
    },
    BeamStyle {
        family: BeamFamily::Diagonal,
        key: KeyCode::Digit3,
        label: "3   Diagonal",
        colour: Color::srgb(0.57, 0.47, 0.80),
    },
];

fn main() -> ExitCode {
    if std::env::args_os()
        .skip(1)
        .any(|arg| arg == "--elastic-report")
    {
        return match elastic_experiment::report() {
            Ok(()) => ExitCode::SUCCESS,
            Err(error) => {
                eprintln!("Elastic carcass report failed: {error}");
                ExitCode::FAILURE
            }
        };
    }

    App::new()
        .insert_resource(LabTyre {
            experiment: Experiment::new(LAB_SUBSTEPS_PER_SECOND),
            mode: Mode::Idle,
        })
        .insert_resource(Time::<Fixed>::from_hz(f64::from(TICKS_PER_SECOND)))
        .insert_resource(ClearColor(Color::srgb(0.047, 0.063, 0.091)))
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Vehicle Lab | Elastic Carcass".into(),
                resolution: WindowResolution::new(1120, 720),
                ..default()
            }),
            ..default()
        }))
        .add_systems(Startup, (setup_scene, setup_panel))
        .add_systems(PreUpdate, experiment_input.after(InputSystems))
        .add_systems(FixedUpdate, advance_experiment)
        .add_systems(Update, (orbit_camera, toggle_beams, draw_experiment))
        .run();
    ExitCode::SUCCESS
}

fn setup_scene(
    mut commands: Commands,
    tyre: Res<LabTyre>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut gizmos: ResMut<Assets<GizmoAsset>>,
) {
    let orbit = OrbitCamera::default();
    commands.spawn((Camera3d::default(), orbit.transform(), orbit));

    let node_mesh = meshes.add(Sphere::new(0.004).mesh().ico(1).unwrap());
    let node_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.9, 0.94, 1.0),
        unlit: true,
        ..default()
    });
    let patch_material = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.45, 0.63),
        unlit: true,
        ..default()
    });
    let experiment = &tyre.experiment;
    let mut rest_markers = GizmoAsset::new();
    for (index, node) in experiment.tyre.topology().nodes().iter().enumerate() {
        let segment = index % experiment.segments;
        let seam = segment == 0 || segment == experiment.segments - 1;
        let patch_node = experiment.patch.contains(&index);
        let mut size = if seam { 1.8 } else { 1.0 };
        let material = if patch_node {
            size *= 0.65;
            let marker_colour = Color::srgb(0.78, 0.81, 0.87);
            for axis in [Vec3::X, Vec3::Z] {
                let arm = axis * 0.006;
                rest_markers.line(
                    node.rest_position - arm,
                    node.rest_position + arm,
                    marker_colour,
                );
            }
            patch_material.clone()
        } else {
            node_material.clone()
        };
        commands.spawn((
            NodeIndex(index),
            Mesh3d(node_mesh.clone()),
            MeshMaterial3d(material),
            Transform::from_translation(node.rest_position).with_scale(Vec3::splat(size)),
        ));
    }

    commands.spawn(Gizmo {
        handle: gizmos.add(rest_markers),
        ..default()
    });
    for style in BEAM_STYLES {
        commands.spawn((
            Gizmo {
                handle: gizmos.add(GizmoAsset::new()),
                line_config: GizmoLineConfig {
                    width: if style.family == BeamFamily::Diagonal {
                        1.0
                    } else {
                        1.75
                    },
                    ..default()
                },
                ..default()
            },
            style,
            RenderLayers::default(),
        ));
    }
}

fn setup_panel(mut commands: Commands, tyre: Res<LabTyre>) {
    let topology = tyre.experiment.tyre.topology();
    let panel = Node {
        position_type: PositionType::Absolute,
        left: px(20),
        top: px(20),
        width: px(268),
        padding: UiRect::all(px(16)),
        flex_direction: FlexDirection::Column,
        row_gap: px(10),
        ..default()
    };
    commands
        .spawn((panel, BackgroundColor(Color::srgb(0.085, 0.112, 0.155))))
        .with_children(|panel| {
            panel.spawn((Text::new("Elastic carcass"), TextFont::from_font_size(23.0)));
            panel.spawn((
                Text::new(format!(
                    "{} nodes  /  {} beams",
                    topology.nodes().len(),
                    topology.beams().len()
                )),
                TextFont::from_font_size(15.0),
                TextColor(Color::srgb(0.77, 0.83, 0.91)),
            ));
            panel.spawn((ExperimentReadout, Text::default(), TextFont::from_font_size(15.0)));
            for style in BEAM_STYLES {
                panel.spawn((
                    Text::new(style.legend_label(true)),
                    TextFont::from_font_size(15.0),
                    TextColor(style.colour),
                    style,
                ));
            }
            panel.spawn((
                Text::new("Pink nodes mark the load patch\nCrosses mark its rest positions"),
                TextFont::from_font_size(14.0),
                TextColor(Color::srgb(0.77, 0.83, 0.91)),
            ));
            panel.spawn((
                Text::new("Enter   Replay\nHold Space   Press tread\nBackspace   Reset experiment\nLeft drag   Orbit\nScroll   Zoom\nR   Reset view"),
                TextFont::from_font_size(14.0),
                TextColor(Color::srgb(0.77, 0.83, 0.91)),
            ));
        });
}

fn experiment_input(keys: Res<ButtonInput<KeyCode>>, mut tyre: ResMut<LabTyre>) {
    if keys.just_pressed(KeyCode::Backspace) {
        tyre.experiment.reset();
        tyre.mode = Mode::Idle;
    } else if keys.just_pressed(KeyCode::Enter) {
        tyre.experiment.reset();
        tyre.mode = Mode::Replay;
    } else if let Mode::Manual { pressed } = tyre.mode {
        let now_pressed = keys.pressed(KeyCode::Space);
        if pressed != now_pressed {
            tyre.mode = Mode::Manual {
                pressed: now_pressed,
            };
        }
    } else if keys.just_pressed(KeyCode::Space) {
        tyre.experiment.reset();
        tyre.mode = Mode::Manual {
            pressed: keys.pressed(KeyCode::Space),
        };
    }
}

fn advance_experiment(mut tyre: ResMut<LabTyre>) {
    for _ in 0..LAB_SUBSTEPS_PER_SECOND / TICKS_PER_SECOND {
        let load = match tyre.mode {
            Mode::Replay if !tyre.experiment.replay_finished() => tyre.experiment.replay_load(),
            Mode::Manual { pressed } => pressed,
            _ => return,
        };
        if let Err(error) = tyre.experiment.advance(load) {
            tyre.mode = Mode::Failed(error);
            return;
        }
    }
}

fn draw_experiment(
    tyre: Res<LabTyre>,
    mut nodes: Query<(&NodeIndex, &mut Transform)>,
    beam_gizmos: Query<(&Gizmo, &BeamStyle)>,
    mut assets: ResMut<Assets<GizmoAsset>>,
    mut readout: Single<&mut Text, With<ExperimentReadout>>,
) {
    if !tyre.is_changed() {
        return;
    }
    let experiment = &tyre.experiment;
    let positions = experiment.tyre.positions();
    for (index, mut transform) in &mut nodes {
        transform.translation = positions[index.0].as_vec3();
    }
    for (gizmo, style) in &beam_gizmos {
        let mut lines = assets.get_mut(&gizmo.handle).expect("lab beam gizmo");
        lines.clear();
        for beam in experiment
            .tyre
            .topology()
            .beams()
            .iter()
            .filter(|beam| beam.family == style.family)
        {
            let [a, b] = beam.endpoints.map(|index| positions[index].as_vec3());
            lines.line(a, b, style.colour);
        }
    }
    let (phase, loaded) = match tyre.mode {
        Mode::Idle => ("Ready", false),
        Mode::Replay if experiment.replay_load() => ("Loading", true),
        Mode::Replay if experiment.replay_finished() => ("Replay complete", false),
        Mode::Replay => ("Rebound", false),
        Mode::Manual { pressed: true } => ("Manual press", true),
        Mode::Manual { pressed: false } => ("Manual release", false),
        Mode::Failed(error) => {
            readout.0 = format!("Simulation stopped\n{error}\nPress Backspace to reset");
            return;
        }
    };
    let load = if loaded { LOAD_NEWTONS } else { 0.0 };
    readout.0 = format!(
        "{phase}   {:.2} s\nLoad   {load:.1} N\nDeflection   {:+.2} mm",
        experiment.elapsed_seconds(),
        experiment.deflection_mm()
    );
}

fn orbit_camera(
    camera: Single<(&mut OrbitCamera, &mut Transform)>,
    mouse: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    keys: Res<ButtonInput<KeyCode>>,
) {
    let (mut orbit, mut transform) = camera.into_inner();
    if keys.just_pressed(KeyCode::KeyR) {
        *orbit = OrbitCamera::default();
    } else {
        if mouse.pressed(MouseButton::Left) {
            orbit.yaw = (orbit.yaw - motion.delta.x * 0.006).rem_euclid(TAU);
            orbit.pitch = (orbit.pitch + motion.delta.y * 0.006).clamp(-1.5, 1.5);
        }
        let scroll_scale = match scroll.unit {
            MouseScrollUnit::Line => 0.14,
            MouseScrollUnit::Pixel => 0.003,
        };
        orbit.distance = (orbit.distance * (-scroll.delta.y * scroll_scale).exp()).clamp(0.65, 3.5);
    }
    *transform = orbit.transform();
}

fn toggle_beams(
    keys: Res<ButtonInput<KeyCode>>,
    mut beams: Query<(&BeamStyle, &mut RenderLayers), With<Gizmo>>,
    mut legend: Query<(&BeamStyle, &mut Text, &mut TextColor)>,
) {
    for (style, mut layers) in &mut beams {
        if !keys.just_pressed(style.key) {
            continue;
        }
        // Retained gizmos are filtered by the camera's render layers.
        let show = *layers == RenderLayers::none();
        *layers = if show {
            RenderLayers::default()
        } else {
            RenderLayers::none()
        };
        for (row, mut text, mut colour) in &mut legend {
            if row.family == style.family {
                text.0 = row.legend_label(show);
                colour.0 = if show {
                    row.colour
                } else {
                    Color::srgb(0.62, 0.67, 0.74)
                };
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn space_interrupts_replay_and_loads_only_while_held() {
        let mut app = App::new();
        app.insert_resource(LabTyre {
            experiment: Experiment::new(LAB_SUBSTEPS_PER_SECOND),
            mode: Mode::Replay,
        })
        .init_resource::<ButtonInput<KeyCode>>()
        .add_systems(PreUpdate, experiment_input)
        .add_systems(FixedUpdate, advance_experiment);
        let world = app.world_mut();
        world.run_schedule(FixedUpdate);
        world
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::Space);
        world.run_schedule(PreUpdate);

        let tyre = world.resource::<LabTyre>();
        assert!(matches!(tyre.mode, Mode::Manual { pressed: true }));
        assert_eq!(tyre.experiment.elapsed_seconds(), 0.0);
        assert_eq!(tyre.experiment.deflection_mm(), 0.0);
        for _ in 0..30 {
            world.run_schedule(FixedUpdate);
        }
        assert!(world.resource::<LabTyre>().experiment.deflection_mm() > 3.0);

        world
            .resource_mut::<ButtonInput<KeyCode>>()
            .release(KeyCode::Space);
        world.run_schedule(PreUpdate);
        assert!(matches!(
            world.resource::<LabTyre>().mode,
            Mode::Manual { pressed: false }
        ));
        let mut crossed_rest = false;
        for _ in 0..90 {
            world.run_schedule(FixedUpdate);
            crossed_rest |= world.resource::<LabTyre>().experiment.deflection_mm() < 0.0;
        }
        assert!(crossed_rest);

        world
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::Backspace);
        world.run_schedule(PreUpdate);
        world.run_schedule(FixedUpdate);
        let tyre = world.resource::<LabTyre>();
        assert!(matches!(tyre.mode, Mode::Idle));
        assert_eq!(tyre.experiment.elapsed_seconds(), 0.0);
        assert_eq!(tyre.experiment.deflection_mm(), 0.0);
    }
}

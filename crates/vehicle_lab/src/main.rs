use std::f32::consts::TAU;

use bevy::{
    camera::visibility::RenderLayers,
    input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll, MouseScrollUnit},
    prelude::*,
    window::WindowResolution,
};
use vehicle_physics::tyre::{BeamFamily, TyreConfig, TyreTopology};

#[derive(Resource)]
struct LabTyre {
    topology: TyreTopology,
    segments: usize,
}

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

fn main() {
    let config = TyreConfig::default();
    let topology = TyreTopology::generate(&config).expect("default tyre must be valid");
    App::new()
        .insert_resource(LabTyre {
            topology,
            segments: config.segments,
        })
        .insert_resource(ClearColor(Color::srgb(0.047, 0.063, 0.091)))
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Vehicle Lab | Tyre Topology".into(),
                resolution: WindowResolution::new(1120, 720),
                ..default()
            }),
            ..default()
        }))
        .add_systems(Startup, (setup_scene, setup_panel))
        .add_systems(Update, (orbit_camera, toggle_beams))
        .run();
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
    for (index, node) in tyre.topology.nodes().iter().enumerate() {
        let segment = index % tyre.segments;
        let size = if segment == 0 || segment == tyre.segments - 1 {
            1.8
        } else {
            1.0
        };
        commands.spawn((
            Mesh3d(node_mesh.clone()),
            MeshMaterial3d(node_material.clone()),
            Transform::from_translation(node.rest_position).with_scale(Vec3::splat(size)),
        ));
    }

    for style in BEAM_STYLES {
        let mut lines = GizmoAsset::new();
        for beam in tyre
            .topology
            .beams()
            .iter()
            .filter(|beam| beam.family == style.family)
        {
            let [a, b] = beam
                .endpoints
                .map(|index| tyre.topology.nodes()[index].rest_position);
            lines.line(a, b, style.colour);
        }
        commands.spawn((
            Gizmo {
                handle: gizmos.add(lines),
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
            panel.spawn((Text::new("Tyre topology"), TextFont::from_font_size(23.0)));
            panel.spawn((
                Text::new(format!(
                    "{} nodes  /  {} beams",
                    tyre.topology.nodes().len(),
                    tyre.topology.beams().len()
                )),
                TextFont::from_font_size(15.0),
                TextColor(Color::srgb(0.77, 0.83, 0.91)),
            ));
            for style in BEAM_STYLES {
                panel.spawn((
                    Text::new(style.legend_label(true)),
                    TextFont::from_font_size(15.0),
                    TextColor(style.colour),
                    style,
                ));
            }
            panel.spawn((
                Text::new("Larger dots mark the seam"),
                TextFont::from_font_size(14.0),
                TextColor(Color::srgb(0.77, 0.83, 0.91)),
            ));
            panel.spawn((
                Text::new("Left drag   Orbit\nScroll   Zoom\nR   Reset view"),
                TextFont::from_font_size(14.0),
                TextColor(Color::srgb(0.77, 0.83, 0.91)),
            ));
        });
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

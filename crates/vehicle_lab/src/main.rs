mod assembly_report;
mod chassis_crash_report;
mod corner_experiment;
mod corner_lab;
mod coupled_chassis_report;
mod crash_report;
mod deformable_drive_report;
mod drive_report;
mod drop_experiment;
mod drop_lab;
mod drop_report;
mod elastic_experiment;
mod grip_experiment;
mod grip_lab;
mod grip_report;
mod handling_experiment;
mod handling_lab;
mod handling_report;
mod impact_experiment;
mod impact_lab;
mod impact_report;
mod imported_reports;
mod inflation_report;
mod loaded_tyre;
mod performance;
mod puncture_experiment;
mod puncture_report;
mod radial_report;
mod rolling_experiment;
mod rolling_lab;
mod rolling_report;
mod static_contact_report;
mod surface_report;

use std::{collections::BTreeMap, f32::consts::TAU};

use bevy::{
    camera::visibility::RenderLayers,
    input::{
        InputSystems,
        mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll, MouseScrollUnit},
    },
    prelude::*,
};
use elastic_experiment::{
    Experiment, LOAD_NEWTONS, PRESETS_PA, RADIAL_SUBSTEPS_PER_SECOND, TICKS_PER_SECOND,
};
use performance::{Benchmark, WindowOptions};
use vehicle_physics::tyre::BeamFamily;

#[derive(Resource)]
struct LabTyre {
    experiment: Experiment,
    mode: Mode,
    preset: usize,
    show_pressure_surface: bool,
}

impl LabTyre {
    fn select_preset(&mut self, preset: usize) {
        self.preset = preset;
        self.experiment = Experiment::radial(RADIAL_SUBSTEPS_PER_SECOND, PRESETS_PA[preset]);
        self.mode = if self.experiment.is_ready() {
            Mode::Idle
        } else {
            Mode::Preparing
        };
    }
}

enum Mode {
    Idle,
    Preparing,
    Replay,
    Manual { pressed: bool },
    Failed(String),
}

#[derive(Component)]
struct NodeIndex(usize);

#[derive(Component)]
struct ExperimentReadout;

#[derive(Component)]
enum Overlay {
    PatchReference,
    PressureSurface(BTreeMap<[usize; 2], bool>),
}

#[derive(Component)]
struct OrbitCamera {
    yaw: f32,
    pitch: f32,
    distance: f32,
    focus: Vec3,
}

#[derive(Component)]
struct OrbitHome {
    pitch: f32,
    distance: f32,
    min_distance: f32,
    max_distance: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            yaw: 0.85,
            pitch: 0.33,
            distance: 1.35,
            focus: Vec3::ZERO,
        }
    }
}

impl OrbitCamera {
    fn transform(&self) -> Transform {
        let rotation = Quat::from_rotation_y(self.yaw) * Quat::from_rotation_x(-self.pitch);
        Transform::from_translation(self.focus + rotation * Vec3::Z * self.distance)
            .with_rotation(rotation)
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

fn main() -> AppExit {
    let mut inflation_lab = false;
    let mut rolling_lab = false;
    let mut impact_lab = false;
    let mut puncture_lab = false;
    let mut corner_lab = false;
    let mut grip_lab = false;
    let mut handling_lab = false;
    let mut window = WindowOptions::default();
    for argument in std::env::args_os().skip(1) {
        let result = if argument == "--drive-report" {
            imported_reports::drive()
        } else if argument == "--deformable-drive-report" {
            deformable_drive_report::report()
        } else if argument == "--alignment-report" {
            imported_reports::alignment()
        } else if argument == "--crash-report" {
            imported_reports::crash()
        } else if argument == "--assembly-report" {
            assembly_report::report()
        } else if argument == "--chassis-crash-report" {
            chassis_crash_report::report()
        } else if argument == "--coupled-chassis-report" {
            coupled_chassis_report::report()
        } else if argument == "--static-contact-report" {
            static_contact_report::report()
        } else if argument == "--handling-report" {
            handling_report::report(std::env::args_os().any(|argument| argument == "--samples"))
        } else if argument == "--grip-report" {
            grip_report::report(std::env::args_os().any(|argument| argument == "--samples"))
        } else if argument == "--surface-report" {
            surface_report::report(std::env::args_os().any(|argument| argument == "--samples"))
        } else if argument == "--corner-report" {
            corner_experiment::report(std::env::args_os().any(|argument| argument == "--samples"))
        } else if argument == "--elastic-report" {
            elastic_experiment::report()
        } else if argument == "--inflation-report" {
            inflation_report::report()
        } else if argument == "--radial-report" {
            radial_report::report()
        } else if argument == "--drop-report" {
            drop_report::report(std::env::args_os().any(|argument| argument == "--samples"))
        } else if argument == "--rolling-report" {
            rolling_report::report(std::env::args_os().any(|argument| argument == "--samples"))
        } else if argument == "--impact-report" {
            impact_report::report(std::env::args_os().any(|argument| argument == "--samples"))
        } else if argument == "--puncture-report" {
            puncture_report::report(std::env::args_os().any(|argument| argument == "--samples"))
        } else {
            inflation_lab |= argument == "--inflation-lab";
            rolling_lab |= argument == "--rolling-lab";
            impact_lab |= argument == "--impact-lab";
            puncture_lab |= argument == "--puncture-lab";
            corner_lab |= argument == "--corner-lab";
            grip_lab |= argument == "--grip-lab";
            handling_lab |= argument == "--handling-lab";
            window.no_vsync |= argument == "--no-vsync";
            window.benchmark |= argument == "--benchmark";
            continue;
        };
        return match result {
            Ok(()) => AppExit::Success,
            Err(error) => {
                eprintln!("Vehicle lab report failed: {error}");
                AppExit::error()
            }
        };
    }

    if handling_lab {
        handling_lab::run(window)
    } else if grip_lab {
        grip_lab::run(window)
    } else if corner_lab {
        corner_lab::run(window)
    } else if puncture_lab {
        impact_lab::run_puncture(window)
    } else if impact_lab {
        impact_lab::run(window)
    } else if rolling_lab {
        rolling_lab::run(window)
    } else if inflation_lab {
        run_inflation_lab(window)
    } else {
        drop_lab::run(window)
    }
}

fn run_inflation_lab(window: WindowOptions) -> AppExit {
    let mut tyre = LabTyre {
        experiment: Experiment::radial(RADIAL_SUBSTEPS_PER_SECOND, PRESETS_PA[0]),
        mode: Mode::Idle,
        preset: 0,
        show_pressure_surface: false,
    };
    let mut app = App::new();
    if window.benchmark {
        tyre.select_preset(performance::BENCHMARK_PRESSURE_PRESET);
        Benchmark::install(&mut app, "Inflation at 30 kPa and 3 N tread press");
        app.add_systems(Last, benchmark_inflation.before(performance::finish));
    }
    app.insert_resource(tyre)
        .insert_resource(Time::<Fixed>::from_hz(f64::from(TICKS_PER_SECOND)))
        .insert_resource(ClearColor(Color::srgb(0.047, 0.063, 0.091)))
        .add_plugins(DefaultPlugins.set(window.plugin("Vehicle Lab | Tyre Inflation")))
        .add_systems(Startup, (setup_scene, setup_panel))
        .add_systems(PreUpdate, experiment_input.after(InputSystems))
        .add_systems(FixedUpdate, advance_experiment)
        .add_systems(Update, (orbit_camera, toggle_beams, draw_experiment));
    app.run()
}

fn benchmark_inflation(mut tyre: ResMut<LabTyre>, mut benchmark: ResMut<Benchmark>) {
    let error = match &tyre.mode {
        Mode::Failed(error) => Some(error.as_str()),
        _ => None,
    };
    if benchmark.observe(
        tyre.experiment.is_ready(),
        tyre.experiment.replay_finished(),
        error,
    ) {
        tyre.experiment.reset();
        tyre.mode = Mode::Replay;
    }
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
    for (index, node) in experiment.tyre.topology().nodes().iter().enumerate() {
        let segment = index % experiment.segments;
        let seam = segment == 0 || segment == experiment.segments - 1;
        let patch_node = experiment.patch.contains(&index);
        let mut size = if seam { 1.8 } else { 1.0 };
        let material = if patch_node {
            size *= 0.65;
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

    let mut pressure_edges = BTreeMap::new();
    for face in experiment.tyre.pressure_triangles() {
        let [a, b, c] = face.nodes;
        for mut edge in [[a, b], [b, c], [c, a]] {
            edge.sort_unstable();
            // Give the shared bead edge the same colour as the rim closure.
            *pressure_edges.entry(edge).or_insert(false) |= face.rim_closure;
        }
    }
    for overlay in [
        Overlay::PatchReference,
        Overlay::PressureSurface(pressure_edges),
    ] {
        commands.spawn((
            Gizmo {
                handle: gizmos.add(GizmoAsset::new()),
                ..default()
            },
            overlay,
        ));
    }
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
        width: px(290),
        padding: UiRect::all(px(16)),
        flex_direction: FlexDirection::Column,
        row_gap: px(10),
        ..default()
    };
    commands
        .spawn((panel, BackgroundColor(Color::srgb(0.085, 0.112, 0.155))))
        .with_children(|panel| {
            panel.spawn((Text::new("Tyre inflation"), TextFont::from_font_size(23.0)));
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
                Text::new("Pink nodes mark the load patch\nCrosses mark the baseline"),
                TextFont::from_font_size(14.0),
                TextColor(Color::srgb(0.77, 0.83, 0.91)),
            ));
            panel.spawn((
                Text::new("P   Next pressure preset\nEnter   Replay\nHold Space   Press tread\nBackspace   Reset experiment\nLeft drag   Orbit\nScroll   Zoom\nR   Reset view"),
                TextFont::from_font_size(14.0),
                TextColor(Color::srgb(0.77, 0.83, 0.91)),
            ));
        });
}

fn experiment_input(keys: Res<ButtonInput<KeyCode>>, mut tyre: ResMut<LabTyre>) {
    if keys.just_pressed(KeyCode::Digit4) {
        tyre.show_pressure_surface = !tyre.show_pressure_surface;
    }
    if keys.just_pressed(KeyCode::KeyP) {
        let next = (tyre.preset + 1) % PRESETS_PA.len();
        tyre.select_preset(next);
        return;
    }
    if !tyre.experiment.is_ready() {
        if keys.just_pressed(KeyCode::Backspace) {
            let preset = tyre.preset;
            tyre.select_preset(preset);
        }
        return;
    }
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
    for _ in 0..tyre.experiment.substeps_per_tick() {
        if matches!(tyre.mode, Mode::Preparing) {
            if let Err(error) = tyre.experiment.prepare_step() {
                tyre.mode = Mode::Failed(error.to_string());
                return;
            }
            if tyre.experiment.is_ready() {
                tyre.mode = Mode::Idle;
                return;
            }
            continue;
        }
        let load = match tyre.mode {
            Mode::Replay if !tyre.experiment.replay_finished() => tyre.experiment.replay_load(),
            Mode::Manual { pressed } => pressed,
            _ => return,
        };
        if let Err(error) = tyre.experiment.advance(load) {
            tyre.mode = Mode::Failed(error.to_string());
            return;
        }
    }
}

fn draw_experiment(
    tyre: Res<LabTyre>,
    mut nodes: Query<(&NodeIndex, &mut Transform)>,
    beam_gizmos: Query<(&Gizmo, &BeamStyle)>,
    overlays: Query<(&Gizmo, &Overlay)>,
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
    for (gizmo, overlay) in &overlays {
        let mut lines = assets.get_mut(&gizmo.handle).expect("lab overlay gizmo");
        lines.clear();
        match overlay {
            Overlay::PatchReference => {
                if let Some(references) = experiment.patch_reference_positions() {
                    for position in references.map(|position| position.as_vec3()) {
                        for axis in [Vec3::X, Vec3::Z] {
                            let arm = axis * 0.006;
                            lines.line(
                                position - arm,
                                position + arm,
                                Color::srgb(0.78, 0.81, 0.87),
                            );
                        }
                    }
                }
            }
            Overlay::PressureSurface(edges) if tyre.show_pressure_surface => {
                for (&edge, &rim) in edges {
                    let [a, b] = edge.map(|index| positions[index].as_vec3());
                    let colour = if rim {
                        Color::srgb(1.0, 0.53, 0.32)
                    } else {
                        Color::srgb(0.46, 0.70, 0.99)
                    };
                    lines.line(a, b, colour);
                }
            }
            Overlay::PressureSurface(_) => {}
        }
    }
    let chamber = experiment
        .tyre
        .chamber_reading()
        .expect("lab uses a sealed tyre");
    let surface = if tyre.show_pressure_surface {
        "On"
    } else {
        "Off"
    };
    let pressures = format!(
        "Target   {:.0} kPa gauge\nGauge   {:+.3} kPa\nVolume   {:.3} L\n4   Chamber [{surface}]",
        PRESETS_PA[tyre.preset] / 1000.0,
        chamber.gauge_pressure_pa / 1000.0,
        chamber.volume_m3 * 1000.0,
    );
    let (phase, loaded) = match tyre.mode {
        Mode::Idle => ("Sealed / ready", false),
        Mode::Preparing => {
            let time = experiment.preparation_seconds();
            let phase = if time < 1.0 { "Filling" } else { "Settling" };
            readout.0 = format!("{phase}   {time:.2} s\n{pressures}");
            return;
        }
        Mode::Replay if experiment.replay_load() => ("Loading", true),
        Mode::Replay if experiment.replay_finished() => ("Replay complete", false),
        Mode::Replay => ("Rebound", false),
        Mode::Manual { pressed: true } => ("Manual press", true),
        Mode::Manual { pressed: false } => ("Manual release", false),
        Mode::Failed(ref error) => {
            readout.0 = format!("Simulation stopped\n{error}\n{pressures}");
            return;
        }
    };
    let load = if loaded { LOAD_NEWTONS } else { 0.0 };
    readout.0 = format!(
        "{phase}   {:.2} s\n{pressures}\nLoad   {load:.1} N\nDeflection   {:+.3} mm\nPeak press   {:.3} mm",
        experiment.elapsed_seconds(),
        experiment.deflection_mm(),
        experiment.peak_inward_mm(),
    );
}

fn orbit_camera(
    camera: Single<(&mut OrbitCamera, &mut Transform, Option<&OrbitHome>)>,
    mouse: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    keys: Res<ButtonInput<KeyCode>>,
) {
    let (mut orbit, mut transform, home) = camera.into_inner();
    if keys.just_pressed(KeyCode::KeyR) {
        *orbit = OrbitCamera {
            focus: orbit.focus,
            ..default()
        };
        if let Some(home) = home {
            orbit.pitch = home.pitch;
            orbit.distance = home.distance;
        }
    } else {
        if mouse.pressed(MouseButton::Left) {
            orbit.yaw = (orbit.yaw - motion.delta.x * 0.006).rem_euclid(TAU);
            orbit.pitch = (orbit.pitch + motion.delta.y * 0.006).clamp(-1.5, 1.5);
        }
        let scroll_scale = match scroll.unit {
            MouseScrollUnit::Line => 0.14,
            MouseScrollUnit::Pixel => 0.003,
        };
        let (near, far) = home.map_or((0.65, 3.5), |home| (home.min_distance, home.max_distance));
        orbit.distance = (orbit.distance * (-scroll.delta.y * scroll_scale).exp()).clamp(near, far);
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
    use crate::elastic_experiment::REFERENCE_SUBSTEPS_PER_SECOND;

    #[test]
    fn changing_pressure_mid_fill_starts_at_zero_for_the_new_target() {
        let mut tyre = LabTyre {
            experiment: Experiment::radial(RADIAL_SUBSTEPS_PER_SECOND, 0.0),
            mode: Mode::Idle,
            preset: 0,
            show_pressure_surface: false,
        };
        assert!(tyre.experiment.is_ready());
        tyre.select_preset(1);
        assert!(matches!(tyre.mode, Mode::Preparing));
        for _ in 0..120 {
            tyre.experiment.prepare_step().unwrap();
        }
        assert!(
            tyre.experiment
                .tyre
                .chamber_reading()
                .unwrap()
                .gauge_pressure_pa
                > 0.0
        );
        tyre.select_preset(2);
        assert!(!tyre.experiment.is_ready());
        assert_eq!(tyre.experiment.preparation_seconds(), 0.0);
        assert_eq!(
            tyre.experiment
                .tyre
                .chamber_reading()
                .unwrap()
                .gauge_pressure_pa,
            0.0
        );
        tyre.select_preset(0);
        assert!(matches!(tyre.mode, Mode::Idle));
        assert!(tyre.experiment.is_ready());
    }

    #[test]
    fn space_interrupts_replay_and_loads_only_while_held() {
        let mut app = App::new();
        app.insert_resource(LabTyre {
            experiment: Experiment::new(REFERENCE_SUBSTEPS_PER_SECOND),
            mode: Mode::Replay,
            preset: 0,
            show_pressure_surface: false,
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
        assert!(world.resource::<LabTyre>().experiment.peak_inward_mm() > 4.0);

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
        assert_eq!(tyre.experiment.peak_inward_mm(), 0.0);
    }
}

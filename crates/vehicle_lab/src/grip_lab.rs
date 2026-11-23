use std::f32::consts::FRAC_PI_2;

use bevy::{input::InputSystems, prelude::*};

use crate::{
    BEAM_STYLES, OrbitCamera,
    elastic_experiment::{Experiment, TICKS_PER_SECOND},
    grip_experiment::{self, Calibration, GripRun, LOADS_N, Preparation, SPEEDS_MPS, SlipCase},
    loaded_tyre::{self, BEAD_RADIUS_M, RATES},
    orbit_camera,
    performance::{self, Benchmark, WindowOptions},
    rolling_experiment::Sample,
    rolling_lab::draw_arrow,
};

const RATE: u32 = RATES[0];
const BELT_COLOR: Color = Color::srgb(0.62, 0.72, 0.84);
const ROAD_COLOR: Color = Color::srgb(0.38, 0.81, 1.0);
const AXLE_COLOR: Color = Color::srgb(1.0, 0.71, 0.35);

#[derive(Resource)]
struct GripLab {
    inflation: Experiment,
    preparation: Option<Preparation>,
    calibration: Option<Calibration>,
    replay: Option<(SlipCase, GripRun)>,
    mean: Option<Sample>,
    load_index: usize,
    speed_index: usize,
    case_index: usize,
    belt_travel: Vec3,
    error: Option<String>,
}

impl GripLab {
    fn new(benchmark: bool) -> Self {
        Self {
            inflation: loaded_tyre::preparation(RATE, true),
            preparation: None,
            calibration: None,
            replay: None,
            mean: None,
            load_index: 1,
            speed_index: usize::from(benchmark),
            case_index: if benchmark {
                SlipCase::ALL
                    .iter()
                    .position(|case| *case == SlipCase::BENCHMARK)
                    .unwrap()
            } else {
                0
            },
            belt_travel: Vec3::ZERO,
            error: None,
        }
    }

    fn load_n(&self) -> f64 {
        LOADS_N[self.load_index]
    }

    fn speed_mps(&self) -> f64 {
        SPEEDS_MPS[self.speed_index]
    }

    fn selected_case(&self) -> SlipCase {
        SlipCase::ALL[self.case_index]
    }

    fn reset(&mut self) {
        if self.calibration.is_none() {
            self.preparation = None;
            if !self.inflation.is_ready() {
                self.inflation = loaded_tyre::preparation(RATE, true);
            }
        }
        self.replay = None;
        self.mean = None;
        self.belt_travel = Vec3::ZERO;
        self.error = None;
    }

    fn start(&mut self) {
        if let Some(calibration) = &self.calibration {
            let case = self.selected_case();
            self.replay = Some((case, calibration.replay(RATE, self.speed_mps(), case)));
            self.mean = None;
            self.error = None;
        }
    }

    fn advance_tick(&mut self) -> Result<(), String> {
        for _ in 0..RATE / TICKS_PER_SECOND {
            if !self.inflation.is_ready() {
                self.inflation
                    .prepare_step()
                    .map_err(|error| error.to_string())?;
            } else if self.calibration.is_none() {
                if self.preparation.is_none() {
                    self.preparation = Some(
                        Preparation::new(
                            self.inflation.tyre.clone(),
                            RATE,
                            self.load_n(),
                            self.speed_mps(),
                        )
                        .map_err(|error| error.to_string())?,
                    );
                }
                let preparation = self.preparation.as_mut().unwrap();
                self.calibration = preparation.advance()?;
                self.belt_travel += preparation.belt_velocity().as_vec3() / RATE as f32;
                if self.calibration.is_some() {
                    self.preparation = None;
                    break;
                }
            } else if let Some((case, replay)) = &mut self.replay
                && self.mean.is_none()
            {
                self.mean = replay
                    .advance()
                    .map_err(|error| format!("{case}\n{error}"))?;
                self.belt_travel += replay.belt_velocity().as_vec3() / RATE as f32;
                if let Some(mean) = self.mean {
                    grip_experiment::validate_case(mean, self.calibration.as_ref().unwrap(), *case)
                        .map_err(|error| format!("{case}\n{error}"))?;
                    break;
                }
            } else {
                break;
            }
        }
        Ok(())
    }
}

#[derive(Component)]
struct Readout;

#[derive(Component)]
struct Rim;

pub fn run(window: WindowOptions) -> AppExit {
    let mut app = App::new();
    app.add_plugins(DefaultPlugins.set(window.plugin("Vehicle Lab | Loaded Tyre Grip")))
        .insert_resource(ClearColor(Color::srgb(0.047, 0.063, 0.091)))
        .insert_resource(Time::<Fixed>::from_hz(f64::from(TICKS_PER_SECOND)))
        .insert_resource(GripLab::new(window.benchmark))
        .add_systems(Startup, setup)
        .add_systems(PreUpdate, input.after(InputSystems))
        .add_systems(FixedUpdate, advance)
        .add_systems(Update, (draw, orbit_camera).chain());
    if window.benchmark {
        Benchmark::install(
            &mut app,
            "Loaded grip at 2452.5 N, 8 m/s, +10% slip and +5°",
        );
        app.add_systems(Last, benchmark_grip.before(performance::finish));
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
        focus: Vec3::Y * 0.25,
        distance: 1.65,
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
        Mesh3d(meshes.add(Cuboid::new(1.20, 0.035, 2.4))),
        MeshMaterial3d(materials.add(Color::srgb(0.12, 0.17, 0.23))),
        Transform::from_xyz(0.0, -0.0175, 0.0),
    ));
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            left: px(20),
            top: px(20),
                width: px(336),
            padding: UiRect::all(px(16)),
            flex_direction: FlexDirection::Column,
            row_gap: px(10),
            ..default()
        },
        BackgroundColor(Color::srgb(0.085, 0.112, 0.155)),
    )).with_children(|panel| {
        panel.spawn((Text::new("Loaded tyre grip"), TextFont::from_font_size(23.0)));
        panel.spawn((Readout, Text::default(), TextFont::from_font_size(15.0)));
        for (label, color) in [("Road force", ROAD_COLOR), ("Axle force", AXLE_COLOR), ("Belt direction", BELT_COLOR)] {
            panel.spawn((Text::new(label), TextFont::from_font_size(14.0), TextColor(color)));
        }
        panel.spawn((
            Text::new("1/2/3  Load    V  Belt speed\nLeft/Right  Select slip case\nEnter  Replay    Backspace  Reset\nDrag  Orbit    Scroll  Zoom\nR  Reset view"),
            TextFont::from_font_size(13.0),
            TextColor(Color::srgb(0.77, 0.83, 0.91)),
        ));
    });
}

fn input(keys: Res<ButtonInput<KeyCode>>, mut lab: ResMut<GripLab>) {
    let load = [KeyCode::Digit1, KeyCode::Digit2, KeyCode::Digit3]
        .iter()
        .position(|key| keys.just_pressed(*key))
        .unwrap_or(lab.load_index);
    let speed = if keys.just_pressed(KeyCode::KeyV) {
        1 - lab.speed_index
    } else {
        lab.speed_index
    };
    if load != lab.load_index || speed != lab.speed_index {
        lab.load_index = load;
        lab.speed_index = speed;
        lab.calibration = None;
        lab.reset();
    }
    let count = SlipCase::ALL.len();
    if keys.just_pressed(KeyCode::ArrowLeft) {
        lab.case_index = (lab.case_index + count - 1) % count;
    } else if keys.just_pressed(KeyCode::ArrowRight) {
        lab.case_index = (lab.case_index + 1) % count;
    }
    if keys.just_pressed(KeyCode::Backspace) {
        lab.reset();
    } else if keys.just_pressed(KeyCode::Enter) {
        lab.start();
    }
}

fn advance(mut lab: ResMut<GripLab>) {
    if lab.error.is_none()
        && let Err(error) = lab.advance_tick()
    {
        lab.error = Some(error);
    }
}

fn benchmark_grip(mut lab: ResMut<GripLab>, mut benchmark: ResMut<Benchmark>) {
    if benchmark.observe(
        lab.calibration.is_some(),
        lab.mean.is_some(),
        lab.error.as_deref(),
    ) {
        lab.start();
    }
}

fn draw(
    lab: Res<GripLab>,
    mut readout: Single<&mut Text, With<Readout>>,
    mut rim: Single<&mut Transform, With<Rim>>,
    gizmo: Single<&Gizmo>,
    mut assets: ResMut<Assets<GizmoAsset>>,
) {
    let mut lines = assets.get_mut(&gizmo.handle).unwrap();
    lines.clear();
    let (wheel, belt) = if let Some((_, replay)) = &lab.replay {
        (Some(&replay.wheel), replay.belt_velocity())
    } else if let Some(calibration) = &lab.calibration {
        (
            Some(&calibration.wheel),
            -lab.speed_mps() * Vec3::Z.as_dvec3(),
        )
    } else if let Some(preparation) = &lab.preparation {
        (Some(&preparation.wheel), preparation.belt_velocity())
    } else {
        (None, Default::default())
    };
    let preview = &lab.inflation.tyre;
    let (topology, positions, offset) = if let Some(wheel) = wheel {
        let motion = wheel.tyre.rim();
        rim.translation = motion.centre.as_vec3();
        rim.rotation = motion.rotation.as_quat() * Quat::from_rotation_z(-FRAC_PI_2);
        for index in wheel.tyre.active_contacts() {
            let point = wheel.tyre.positions()[index].as_vec3();
            lines.line(point, point.with_y(0.009), ROAD_COLOR);
        }
        (wheel.tyre.topology(), wheel.tyre.positions(), Vec3::ZERO)
    } else {
        rim.translation = Vec3::Y * loaded_tyre::unloaded_radius(preview) as f32;
        rim.rotation = Quat::from_rotation_z(-FRAC_PI_2);
        (preview.topology(), preview.positions(), rim.translation)
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
    for row in -6..=6 {
        let z = (row as f32 * 0.18 + lab.belt_travel.z + 1.17).rem_euclid(2.34) - 1.17;
        for column in -2..=2 {
            let x = (column as f32 * 0.22 + lab.belt_travel.x + 0.55).rem_euclid(1.1) - 0.55;
            lines.line(
                Vec3::new(x - 0.034, 0.002, z),
                Vec3::new(x + 0.034, 0.002, z),
                BELT_COLOR,
            );
        }
    }
    draw_arrow(
        &mut lines,
        Vec3::new(0.52, 0.025, 0.65),
        belt.as_vec3().normalize_or_zero() * 0.37,
        BELT_COLOR,
    );

    let status = if lab.error.is_some() {
        "Experiment stopped"
    } else if lab.mean.is_some() {
        "Final mean over two turns"
    } else if lab.replay.is_some() {
        "Measuring slip"
    } else if lab.calibration.is_some() {
        "Ready to replay"
    } else if let Some(preparation) = &lab.preparation {
        if preparation.is_loading() {
            "Settling axle load"
        } else {
            "Calibrating free rolling"
        }
    } else {
        "Inflating to 150 kPa gauge"
    };
    let mut text = format!(
        "{status}\n{:.2} N at {:.0} m/s\nSelected  {}",
        lab.load_n(),
        lab.speed_mps(),
        lab.selected_case()
    );
    if let Some(calibration) = &lab.calibration {
        text.push_str(&format!(
            "\nRolling radius  {:.2} mm",
            1000.0 * calibration.radius_m
        ));
        if let Some((case, replay)) = &lab.replay {
            let sample = lab.mean.unwrap_or_else(|| replay.latest());
            if belt.z < 0.0 {
                let slip = 100.0
                    * grip_experiment::slip_ratio(sample.spin_rad_s, belt, calibration.radius_m);
                let scale = 0.40 / lab.load_n() as f32;
                for (origin, force, color) in [
                    (Vec3::Y * 0.018, sample.floor_force_n, ROAD_COLOR),
                    (
                        rim.translation + Vec3::X * 0.11,
                        sample.axle_force_n,
                        AXLE_COLOR,
                    ),
                ] {
                    draw_arrow(
                        &mut lines,
                        origin,
                        force.as_vec3().with_y(0.0) * scale,
                        color,
                    );
                }
                text.push_str(&format!(
                    "\n\nRun  {case}\nMeasured  {slip:+.2}% / {:+.2} deg\nSpin  {:.2} rad/s\nSupport  {:.0} N\nRoad Z/X  {:+.0} / {:+.0} N\nAxle Z/X  {:+.0} / {:+.0} N",
                    sample.slip_angle_rad.to_degrees(), sample.spin_rad_s, sample.floor_force_n.y,
                    sample.floor_force_n.z, sample.floor_force_n.x, sample.axle_force_n.z, sample.axle_force_n.x,
                ));
            }
        }
    }
    if let Some(error) = &lab.error {
        text.push_str(&format!("\n{error}"));
    }
    readout.0 = text;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arrow_controls_cycle_through_every_case_in_both_directions() {
        let mut app = App::new();
        app.init_resource::<ButtonInput<KeyCode>>()
            .insert_resource(GripLab::new(false))
            .add_systems(Update, input);
        let count = SlipCase::ALL.len();
        for (arrow, step) in [(KeyCode::ArrowRight, 1), (KeyCode::ArrowLeft, count - 1)] {
            let mut index = 0;
            for _ in SlipCase::ALL {
                let mut keys = app.world_mut().resource_mut::<ButtonInput<KeyCode>>();
                keys.reset_all();
                keys.press(arrow);
                app.update();
                index = (index + step) % count;
                assert_eq!(
                    app.world().resource::<GripLab>().selected_case(),
                    SlipCase::ALL[index]
                );
            }
        }
    }
}

use std::f32::consts::FRAC_PI_2;

use bevy::{input::InputSystems, math::DVec3, prelude::*};
use vehicle_physics::{avian_tyre::AvianTyre, tyre::BeamFamily};

use crate::{
    OrbitCamera,
    drop_experiment::{self, CLEARANCE_M, DropBench},
    elastic_experiment::{PRESETS_PA, RADIAL_SUBSTEPS_PER_SECOND},
    orbit_camera,
    performance::{self, Benchmark, WindowOptions},
};

#[derive(Component)]
struct RimMesh;

#[derive(Component)]
struct DropLines;

#[derive(Component)]
struct DropReadout;

const CONTACT: Color = Color::srgb(1.0, 0.48, 0.38);

pub fn run(window: WindowOptions) -> AppExit {
    let mut app = App::new();
    app.insert_resource(ClearColor(Color::srgb(0.047, 0.063, 0.091)))
        .add_plugins(DefaultPlugins.set(window.plugin("Vehicle Lab | Wheel Drop")))
        .add_systems(Startup, setup)
        .add_systems(PreUpdate, drop_input.after(InputSystems))
        .add_systems(Update, (orbit_camera, draw_drop));
    let mut preset = 0;
    if window.benchmark {
        preset = performance::BENCHMARK_PRESSURE_PRESET;
        Benchmark::install(&mut app, "Wheel drop at 30 kPa");
        app.add_systems(Last, benchmark_drop.before(performance::finish));
    }
    drop_experiment::install(&mut app, preset, RADIAL_SUBSTEPS_PER_SECOND, false);
    app.run()
}

fn benchmark_drop(
    mut bench: ResMut<DropBench>,
    mut benchmark: ResMut<Benchmark>,
    mut commands: Commands,
) {
    if benchmark.observe(
        bench.baseline.is_some(),
        bench.run.settled.is_some(),
        bench.error.as_deref(),
    ) {
        bench.release(&mut commands);
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
    let mut floor = GizmoAsset::new();
    for index in -8..=8 {
        let offset = index as f32 * 0.10;
        let colour = Color::srgb(0.20, 0.27, 0.35);
        floor.line(
            Vec3::new(offset, 0.0, -0.8),
            Vec3::new(offset, 0.0, 0.8),
            colour,
        );
        floor.line(
            Vec3::new(-0.8, 0.0, offset),
            Vec3::new(0.8, 0.0, offset),
            colour,
        );
    }
    commands.spawn(Gizmo {
        handle: gizmos.add(floor),
        ..default()
    });
    commands.spawn((
        DropLines,
        Gizmo {
            handle: gizmos.add(GizmoAsset::new()),
            ..default()
        },
    ));
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: px(20),
                top: px(20),
                width: px(276),
                padding: UiRect::all(px(16)),
                flex_direction: FlexDirection::Column,
                row_gap: px(14),
                ..default()
            },
            BackgroundColor(Color::srgb(0.085, 0.112, 0.155)),
        ))
        .with_children(|panel| {
            panel.spawn((Text::new("Wheel drop"), TextFont::from_font_size(23.0)));
            panel.spawn((DropReadout, Text::default(), TextFont::from_font_size(16.0)));
            panel.spawn((Text::new("Pink marks floor contact"), TextFont::from_font_size(14.0), TextColor(CONTACT)));
            panel.spawn((
                Text::new("Enter   Drop / replay\nP   Next pressure preset\nBackspace   Reset drop\nLeft drag   Orbit\nScroll   Zoom\nR   Reset view"),
                TextFont::from_font_size(14.0),
                TextColor(Color::srgb(0.77, 0.83, 0.91)),
            ));
        });
}

fn drop_input(
    keys: Res<ButtonInput<KeyCode>>,
    mut bench: ResMut<DropBench>,
    mut commands: Commands,
) {
    if keys.just_pressed(KeyCode::KeyP) {
        let next = (bench.preset + 1) % PRESETS_PA.len();
        bench.select(next, &mut commands);
    } else if keys.just_pressed(KeyCode::Backspace) {
        if bench.baseline.is_some() {
            bench.reset(&mut commands);
        } else {
            let preset = bench.preset;
            bench.select(preset, &mut commands);
        }
    } else if keys.just_pressed(KeyCode::Enter) {
        bench.release(&mut commands);
    }
}

fn draw_drop(
    bench: Res<DropBench>,
    wheels: Query<&AvianTyre>,
    gizmo: Single<&Gizmo, With<DropLines>>,
    mut assets: ResMut<Assets<GizmoAsset>>,
    mut rim: Single<&mut Transform, With<RimMesh>>,
    mut readout: Single<&mut Text, With<DropReadout>>,
) {
    if !bench.is_changed() {
        return;
    }
    let mounted = bench
        .wheel
        .and_then(|entity| wheels.get(entity).ok())
        .map(AvianTyre::tyre)
        .or(bench.baseline.as_ref());
    let (topology, positions, centre, rotation, offset, gauge) = if let Some(tyre) = mounted {
        let rim = tyre.rim();
        (
            tyre.topology(),
            tyre.positions(),
            rim.centre,
            rim.rotation.as_quat(),
            DVec3::ZERO,
            tyre.chamber_reading(),
        )
    } else if let Some(preparation) = &bench.preparation {
        let tyre = &preparation.tyre;
        let bottom = tyre
            .positions()
            .iter()
            .map(|point| point.y)
            .fold(f64::INFINITY, f64::min);
        let centre = DVec3::Y * (CLEARANCE_M - bottom);
        (
            tyre.topology(),
            tyre.positions(),
            centre,
            Quat::IDENTITY,
            centre,
            tyre.chamber_reading(),
        )
    } else {
        return;
    };
    rim.translation = centre.as_vec3();
    rim.rotation = rotation * Quat::from_rotation_z(-FRAC_PI_2);
    let mut lines = assets.get_mut(&gizmo.handle).expect("wheel gizmo");
    lines.clear();
    for beam in topology.beams() {
        let [a, b] = beam
            .endpoints
            .map(|index| (positions[index] + offset).as_vec3());
        let colour = match beam.family {
            BeamFamily::Circumferential => Color::srgb(0.25, 0.87, 0.75),
            BeamFamily::CrossSection => Color::srgb(1.0, 0.71, 0.32),
            BeamFamily::Diagonal => Color::srgb(0.43, 0.37, 0.61),
        };
        lines.line(a, b, colour);
    }
    if let Some(tyre) = mounted {
        for index in tyre.active_contacts() {
            let point = tyre.positions()[index].as_vec3();
            let projection = Vec3::new(point.x, 0.001, point.z);
            for axis in [Vec3::X, Vec3::Z] {
                lines.line(
                    projection - axis * 0.008,
                    projection + axis * 0.008,
                    CONTACT,
                );
            }
            lines.line(point, projection, CONTACT);
        }
        if let Some(patch) = tyre.footprint() {
            for polygon in patch.polygons {
                for (&a, &b) in polygon.iter().zip(polygon.iter().cycle().skip(1)) {
                    let lift = Vec3::Y * 0.001;
                    lines.line(a.as_vec3() + lift, b.as_vec3() + lift, CONTACT);
                }
            }
        }
    }
    let phase = if let Some(error) = &bench.error {
        format!("Simulation stopped\n{error}")
    } else if let Some(settled) = &bench.run.settled {
        format!("Settled   {:.2} s", settled.time_s)
    } else if bench.preparation.is_some() {
        "Preparing pressure".into()
    } else if bench.wheel.is_none() {
        "Ready   100 mm clearance".into()
    } else if bench.run.contact_time_s.is_some() {
        "Rebound and settling".into()
    } else {
        "Falling".into()
    };
    let gauge_kpa = gauge.map_or(0.0, |chamber| chamber.gauge_pressure_pa / 1000.0);
    let target_kpa = PRESETS_PA[bench.preset] / 1000.0;
    let mut display = format!(
        "{phase}\nPreset   {target_kpa:.0} kPa gauge\nLive gauge   {gauge_kpa:.2} kPa\nAxle height   {:.1} mm",
        centre.y * 1000.0,
    );
    if let Some(time) = bench.run.contact_time_s {
        display.push_str(&format!("\nFirst contact   {:.1} ms", time * 1000.0));
    }
    if let Some(rebound) = bench.run.rebound_m {
        display.push_str(&format!("\nFirst rebound   {:.1} mm", rebound * 1000.0));
    }
    if let Some(settled) = &bench.run.settled {
        display.push_str(&format!(
            "\nPatch length   {:.1} mm\nPatch width   {:.1} mm\nMean support   {:.1} N",
            settled.patch_length_m * 1000.0,
            settled.patch_width_m * 1000.0,
            settled.mean_floor_force_n,
        ));
    } else if bench.wheel.is_some() {
        display.push_str(&format!(
            "\nFloor force   {:.1} N",
            bench.run.last_floor_force_n
        ));
    }
    readout.0 = display;
}

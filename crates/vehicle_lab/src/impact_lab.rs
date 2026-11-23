use std::f32::consts::{FRAC_PI_2, TAU};

use bevy::{input::InputSystems, prelude::*, window::WindowResolution};
use vehicle_physics::{avian_tyre::AvianTyre, tyre::RingRole};

use crate::{
    OrbitCamera,
    elastic_experiment::RADIAL_SUBSTEPS_PER_SECOND,
    impact_experiment::{
        self, ImpactBench, KERB_RADIUS_M, KERB_RISE_M, LAUNCH_Z_M, Phase, SPEEDS_MPS,
        damage_counts, launch_height_m,
    },
    orbit_camera,
};

const INTACT: Color = Color::srgb(0.30, 0.66, 0.65);
const YIELDED: Color = Color::srgb(1.0, 0.71, 0.26);
const BROKEN: Color = Color::srgb(1.0, 0.32, 0.35);
const ORIGINAL: Color = Color::srgb(0.78, 0.84, 0.94);
const ROAD: Color = Color::srgb(0.26, 0.38, 0.51);

#[derive(Component)]
struct RimMesh;

#[derive(Component)]
struct ImpactReadout;

#[derive(Resource, Default)]
struct SelectedSpeed(usize);

pub fn run() {
    let mut app = App::new();
    app.insert_resource(ClearColor(Color::srgb(0.047, 0.063, 0.091)))
        .init_resource::<SelectedSpeed>()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Vehicle Lab | Kerb Impact".into(),
                resolution: WindowResolution::new(1120, 720),
                ..default()
            }),
            ..default()
        }))
        .add_systems(Startup, setup)
        .add_systems(PreUpdate, input.after(InputSystems))
        .add_systems(Update, (draw, orbit_camera).chain());
    impact_experiment::install(&mut app, RADIAL_SUBSTEPS_PER_SECOND);
    app.run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut gizmos: ResMut<Assets<GizmoAsset>>,
) {
    let orbit = OrbitCamera {
        focus: Vec3::new(0.0, 0.27, -0.45),
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
                width: px(272),
                padding: UiRect::all(px(16)),
                flex_direction: FlexDirection::Column,
                row_gap: px(12),
                ..default()
            },
            BackgroundColor(Color::srgb(0.085, 0.112, 0.155)),
        ))
        .with_children(|panel| {
            panel.spawn((Text::new("Kerb impact"), TextFont::from_font_size(23.0)));
            panel.spawn((ImpactReadout, Text::default(), TextFont::from_font_size(16.0)));
            for (label, colour) in [
                ("Gold   Yielded beam", YIELDED),
                ("Red    Broken beam", BROKEN),
                ("White  Original tread", ORIGINAL),
            ] {
                panel.spawn((Text::new(label), TextFont::from_font_size(14.0), TextColor(colour)));
            }
            let [slow, medium, fast] = SPEEDS_MPS;
            panel.spawn((
                Text::new(format!(
                    "1  {slow}   2  {medium}   3  {fast} m/s\nEnter   Strike / replay\nBackspace   Reset\nLeft drag   Orbit\nScroll   Zoom\nR   Reset view"
                )),
                TextFont::from_font_size(14.0),
                TextColor(Color::srgb(0.77, 0.83, 0.91)),
            ));
        });
}

fn input(
    keys: Res<ButtonInput<KeyCode>>,
    mut selected: ResMut<SelectedSpeed>,
    mut bench: ResMut<ImpactBench>,
) {
    for (index, key) in [KeyCode::Digit1, KeyCode::Digit2, KeyCode::Digit3]
        .into_iter()
        .enumerate()
    {
        if keys.just_pressed(key) {
            selected.0 = index;
        }
    }
    if keys.just_pressed(KeyCode::Backspace) {
        bench.reset();
    } else if keys.just_pressed(KeyCode::Enter) {
        bench.launch(SPEEDS_MPS[selected.0], false);
    }
}

fn draw(
    bench: Res<ImpactBench>,
    selected: Res<SelectedSpeed>,
    wheels: Query<&AvianTyre>,
    gizmo: Single<&Gizmo>,
    mut assets: ResMut<Assets<GizmoAsset>>,
    mut camera: Single<&mut OrbitCamera>,
    mut rim: Single<&mut Transform, With<RimMesh>>,
    mut readout: Single<&mut Text, With<ImpactReadout>>,
) {
    if !bench.is_changed() && !selected.is_changed() {
        return;
    }
    let mut lines = assets.get_mut(&gizmo.handle).expect("impact gizmo");
    lines.clear();
    draw_road(&mut lines);
    let wheel = bench
        .wheel
        .and_then(|wheel| wheels.get(wheel).ok())
        .map(AvianTyre::tyre);
    let local_tyre = bench
        .unloaded
        .as_ref()
        .or(bench.baseline.as_ref())
        .or_else(|| {
            bench
                .preparation
                .as_ref()
                .map(|preparation| &preparation.tyre)
        });
    let (mesh, centre, rotation, offset) =
        if let Some(tyre) = wheel.filter(|_| bench.unloaded.is_none()) {
            let rim = tyre.rim();
            (
                Some((tyre.topology(), tyre.positions(), tyre.beam_damage())),
                rim.centre.as_vec3(),
                rim.rotation.as_quat(),
                Vec3::ZERO,
            )
        } else if let Some(tyre) = local_tyre {
            let offset = if bench.unloaded.is_some() {
                Vec3::new(0.0, 0.49, 0.12)
            } else {
                Vec3::new(0.0, launch_height_m(tyre) as f32, LAUNCH_Z_M as f32)
            };
            (
                Some((tyre.topology(), tyre.positions(), tyre.beam_damage())),
                offset,
                Quat::IDENTITY,
                offset,
            )
        } else {
            (None, Vec3::ZERO, Quat::IDENTITY, Vec3::ZERO)
        };
    rim.translation = centre;
    rim.rotation = rotation * Quat::from_rotation_z(-FRAC_PI_2);
    camera.focus = Vec3::new(0.0, 0.33, centre.z + 0.26);

    if let Some((topology, positions, states)) = mesh {
        for (index, beam) in topology.beams().iter().enumerate() {
            let [a, b] = beam
                .endpoints
                .map(|node| positions[node].as_vec3() + offset);
            let state = states.map(|states| states[index]);
            if state.is_some_and(|state| state.broken()) {
                lines.line(a, a.lerp(b, 0.35), BROKEN);
                lines.line(a.lerp(b, 0.65), b, BROKEN);
            } else {
                let colour = if state.is_some_and(|state| state.yielded()) {
                    YIELDED
                } else {
                    INTACT
                };
                lines.line(a, b, colour);
            }
        }
    }
    if bench.unloaded.is_some()
        && let Some(baseline) = &bench.baseline
    {
        for beam in baseline.topology().beams() {
            let [i, j] = beam.endpoints;
            if [i, j]
                .iter()
                .all(|&node| baseline.topology().nodes()[node].ring == RingRole::TreadCentre)
            {
                let [a, b] = [i, j].map(|node| baseline.positions()[node].as_vec3() + offset);
                lines.line(a.lerp(b, 0.15), a.lerp(b, 0.85), ORIGINAL);
            }
        }
    }

    let phase = match bench.phase {
        Phase::Inflating => "Inflating to 30 kPa",
        Phase::Ready => "Ready",
        Phase::Coasting if bench.run.contact.is_none() => "Approaching kerb",
        Phase::Coasting => "Kerb strike",
        Phase::Unloading => "Relaxing on fixed rim",
        Phase::Complete => "Unloaded comparison",
        Phase::Failed => "Simulation stopped",
    };
    let mut text = format!("{phase}\nSelected  {:.1} m/s", SPEEDS_MPS[selected.0]);
    if let Some(error) = &bench.error {
        text.push_str(&format!("\n{error}"));
    }
    if matches!(
        bench.phase,
        Phase::Coasting | Phase::Unloading | Phase::Complete
    ) {
        let last = bench.run.last;
        let (yielded, broken) = bench
            .unloaded
            .as_ref()
            .and_then(|tyre| tyre.beam_damage())
            .map_or((last.yielded, last.broken), damage_counts);
        text.push_str(&format!(
            "\nLaunch  {:.1} m/s\nPeak inward  {:.1} mm\nYielded  {yielded}\nBroken   {broken}",
            bench.run.speed_mps,
            bench.run.peak_inward_m * 1000.0,
        ));
        if let Some((_, speed)) = bench.run.contact {
            text.push_str(&format!("\nStrike speed  {speed:.2} m/s"));
        }
        if let Some(residual) = bench.run.residual_m {
            text.push_str(&format!("\nPermanent set  {:.2} mm", residual * 1000.0));
        }
    }
    readout.0 = text;
}

fn draw_road(lines: &mut GizmoAsset) {
    let height = KERB_RISE_M as f32;
    let radius = KERB_RADIUS_M as f32;
    for index in -4..=4 {
        let x = index as f32 * 0.14;
        let mut previous = Vec3::new(x, height - radius, 0.0);
        lines.line(Vec3::new(x, 0.0, -1.6), Vec3::new(x, 0.0, 0.0), ROAD);
        lines.line(Vec3::new(x, 0.0, 0.0), previous, ROAD);
        for arc in 1..=8 {
            let theta = (arc as f32) * TAU / 32.0;
            let point = Vec3::new(
                x,
                height - radius + radius * theta.sin(),
                radius * (1.0 - theta.cos()),
            );
            lines.line(previous, point, ROAD);
            previous = point;
        }
        lines.line(previous, Vec3::new(x, height, 2.5), ROAD);
    }
    for index in -8..=12 {
        let z = index as f32 * 0.20;
        let y = if z > 0.0 { height } else { 0.0 };
        lines.line(Vec3::new(-0.56, y, z), Vec3::new(0.56, y, z), ROAD);
    }
    lines.line(
        Vec3::new(-0.56, height, radius),
        Vec3::new(0.56, height, radius),
        ROAD,
    );
}

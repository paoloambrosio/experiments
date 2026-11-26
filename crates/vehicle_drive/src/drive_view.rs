use bevy::{
    ecs::change_detection::{NonSend, NonSendMut},
    gizmos::config::GizmoConfigStore,
    prelude::*,
};
use vehicle_assets::ror::{TerrainImport, VehicleImport};

use crate::{
    drive::{DriveSession, DriverInput, TICKS_PER_SECOND},
    view::{self, MeshAssets},
};

#[derive(Resource)]
struct DriveScene {
    terrain: TerrainImport,
    vehicle: VehicleImport,
}

#[derive(Resource)]
struct VisualBodies {
    chassis: Entity,
    wheels: [Entity; 4],
}

#[derive(Resource, Default)]
struct DriveStatus {
    error: Option<String>,
    reset_requested: bool,
    camera_reset: bool,
}

#[derive(Component)]
struct SpeedReadout;

pub(crate) fn run(session: DriveSession, terrain: TerrainImport, vehicle: VehicleImport) -> bool {
    let title = format!("{} | {}", vehicle.vehicle.name, terrain.terrain.name);
    let mut app = App::new();
    app.insert_non_send(session);
    app.insert_resource(DriveScene { terrain, vehicle })
        .init_resource::<DriveStatus>()
        .insert_resource(ClearColor(Color::srgb(0.37, 0.49, 0.60)))
        .insert_resource(Time::<Fixed>::from_hz(f64::from(TICKS_PER_SECOND)))
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window { title, ..default() }),
            ..default()
        }))
        .add_systems(Startup, setup)
        .add_systems(PreUpdate, request_reset.after(bevy::input::InputSystems))
        .add_systems(FixedUpdate, advance)
        .add_systems(Update, (draw, view::collision_overlay));
    app.run().is_success()
}

fn setup(
    mut commands: Commands,
    scene: Res<DriveScene>,
    session: NonSend<DriveSession>,
    mut assets: MeshAssets,
    mut gizmo_config: ResMut<GizmoConfigStore>,
) {
    view::setup_terrain(
        &mut commands,
        &scene.terrain,
        &mut assets,
        &mut gizmo_config,
        true,
    );
    let chassis = commands
        .spawn((Name::new("Body"), Transform::default(), Visibility::Visible))
        .id();
    let wheels: [Entity; 4] = vehicle_physics::vehicle_assembly::WHEEL_NAMES.map(|name| {
        commands
            .spawn((Name::new(name), Transform::default(), Visibility::Visible))
            .id()
    });
    let vehicle = &scene.vehicle.vehicle;
    let mut instances = vehicle.objects.clone();
    let mut parents = vec![chassis; instances.len()];
    for (wheel, (index, detail)) in wheels.into_iter().zip(session.setup.wheel_visuals()) {
        let inverse = instances[detail.tyre_object].rotation.inverse();
        for i in [detail.rim_object, detail.tyre_object] {
            let instance = &mut instances[i];
            instance.translation = inverse * (instance.translation - vehicle.wheels[index].centre);
            instance.rotation = inverse * instance.rotation;
            parents[i] = wheel;
        }
    }
    view::spawn_meshes(
        &mut commands,
        &mut assets,
        &scene.vehicle.appearance,
        &vehicle.meshes,
        instances.iter().zip(parents.into_iter().map(Some)),
    );
    commands.insert_resource(VisualBodies { chassis, wheels });
    let (centre, rotation, _) = session.chassis();
    commands.spawn((Camera3d::default(), chase_pose(centre, rotation)));
    commands
        .spawn((
            Node {
                position_type: PositionType::Absolute,
                left: px(20),
                bottom: px(20),
                padding: UiRect::all(px(14)),
                flex_direction: FlexDirection::Column,
                row_gap: px(4),
                ..default()
            },
            BackgroundColor(Color::srgba(0.055, 0.075, 0.10, 0.9)),
        ))
        .with_children(|panel| {
            panel.spawn((
                SpeedReadout,
                Text::new("0 km/h"),
                TextFont::from_font_size(28.0),
            ));
            panel.spawn((
                Text::new("W  Throttle   S  Brake   A/D  Steer   R  Reset   C  Collision"),
                TextFont::from_font_size(15.0),
            ));
        });
    commands.remove_resource::<DriveScene>();
}

fn request_reset(keys: Res<ButtonInput<KeyCode>>, mut status: ResMut<DriveStatus>) {
    status.reset_requested |= keys.just_pressed(KeyCode::KeyR);
}

fn advance(
    keys: Res<ButtonInput<KeyCode>>,
    mut session: NonSendMut<DriveSession>,
    mut status: ResMut<DriveStatus>,
) {
    if std::mem::take(&mut status.reset_requested) {
        let start = session.setup.start;
        status.error = session.reset(start).err();
        status.camera_reset = true;
        return;
    }
    if status.error.is_none() {
        let pressed = |key| f32::from(u8::from(keys.pressed(key)));
        status.error = session
            .step(DriverInput {
                throttle: pressed(KeyCode::KeyW),
                brake: pressed(KeyCode::KeyS),
                steer: pressed(KeyCode::KeyA) - pressed(KeyCode::KeyD),
            })
            .err();
    }
}

fn chase_pose(centre: Vec3, rotation: Quat) -> Transform {
    let forward = (rotation * Vec3::Z)
        .with_y(0.0)
        .try_normalize()
        .unwrap_or(Vec3::Z);
    let camera = centre - forward * 5.7 + Vec3::Y * 2.5;
    Transform::from_translation(camera).looking_at(centre + forward * 1.0, Vec3::Y)
}

fn draw(
    session: NonSend<DriveSession>,
    visuals: Res<VisualBodies>,
    mut status: ResMut<DriveStatus>,
    time: Res<Time>,
    mut bodies: Query<&mut Transform, Without<Camera3d>>,
    mut camera: Single<&mut Transform, With<Camera3d>>,
    mut readout: Single<&mut Text, With<SpeedReadout>>,
) {
    let (centre, rotation, velocity) = session.chassis();
    let mut chassis = bodies.get_mut(visuals.chassis).unwrap();
    *chassis = Transform::from_translation(centre - rotation * session.setup.centre_of_mass())
        .with_rotation(rotation);
    for (entity, rim) in visuals.wheels.into_iter().zip(session.rims()) {
        *bodies.get_mut(entity).unwrap() =
            Transform::from_translation(rim.centre.as_vec3()).with_rotation(rim.rotation.as_quat());
    }
    let target = chase_pose(centre, rotation);
    if std::mem::take(&mut status.camera_reset) {
        **camera = target;
    } else {
        let weight = 1.0 - (-10.0 * time.delta_secs()).exp();
        camera.translation = camera.translation.lerp(target.translation, weight);
        camera.rotation = camera.rotation.slerp(target.rotation, weight);
    }
    let speed = velocity.with_y(0.0).length() * 3.6;
    readout.0 = match &status.error {
        Some(error) => format!("{speed:.0} km/h\n{error}"),
        None => format!("{speed:.0} km/h"),
    };
}

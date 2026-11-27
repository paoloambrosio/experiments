use bevy::{camera::primitives::Aabb, gizmos::config::GizmoConfigStore, prelude::*};
use vehicle_assets::ror::{TerrainImport, VehicleImport};
use vehicle_physics::vehicle_assembly::{VehiclePose, WHEEL_NAMES};
use vehicle_scene::{
    body_skin::{BodyShape, BodySkin},
    crash::CrashSetup,
    drive::{DriveSetup, TICKS_PER_SECOND},
};

use crate::{
    drive_view::wheel_instances,
    view::{self, MeshAssets, MutableSection, OrbitCamera},
};

#[derive(Resource)]
struct Scene {
    terrain: TerrainImport,
    vehicle: VehicleImport,
    drive: DriveSetup,
}

struct Frame {
    body: BodyShape,
    wheels: [Transform; 4],
}

#[derive(Resource)]
struct Replay {
    objects: Vec<usize>,
    frames: Vec<Frame>,
}

impl Replay {
    fn record(drive: &DriveSetup, vehicle: &VehicleImport) -> Result<Self, String> {
        let asset = &vehicle.vehicle;
        let crash = CrashSetup::new(drive, asset, "wall")?;
        let skin = BodySkin::new(asset, &crash.imported.source)?;
        let objects: Vec<_> = skin.meshes.iter().map(|mesh| mesh.object).collect();
        let frames = crash
            .record()?
            .into_iter()
            .enumerate()
            .map(|(tick, frame)| {
                let mut body = skin
                    .sample(&frame.nodes_m)
                    .map_err(|error| format!("crash replay tick {tick}: {error}"))?;
                if tick == 0 {
                    // Reuse the authored positions exactly at launch, including after reset.
                    for (&object, vertices) in objects.iter().zip(&mut body.vertices) {
                        let id = asset.objects[object].visual_mesh.unwrap();
                        vertices.clone_from(&asset.meshes[id.0].vertices);
                    }
                }
                Ok(Frame {
                    body,
                    wheels: frame.rims.map(|rim| {
                        Transform::from_translation(rim.centre.as_vec3())
                            .with_rotation(rim.rotation.as_quat())
                    }),
                })
            })
            .collect::<Result<Vec<_>, String>>()?;
        Ok(Self { objects, frames })
    }
}

#[derive(Resource, Default)]
struct Playback {
    frame: usize,
    playing: bool,
    fraction: f64,
}

impl Playback {
    fn update(&mut self, keys: &ButtonInput<KeyCode>, dt: f64, last: usize) {
        if keys.just_pressed(KeyCode::KeyR) {
            *self = Self::default();
            return;
        }
        let left = keys.just_pressed(KeyCode::ArrowLeft);
        let right = keys.just_pressed(KeyCode::ArrowRight);
        if left || right {
            self.frame = (self.frame + usize::from(right))
                .saturating_sub(usize::from(left))
                .min(last);
            self.playing = false;
            self.fraction = 0.0;
        } else if keys.just_pressed(KeyCode::Space) {
            self.playing = !self.playing && self.frame < last;
        } else if self.playing {
            self.fraction += dt * f64::from(TICKS_PER_SECOND);
            let ticks = self.fraction as usize;
            self.fraction -= ticks as f64;
            self.frame = self.frame.saturating_add(ticks).min(last);
            self.playing = self.frame < last;
        }
    }
}

#[derive(Resource)]
struct Visuals {
    chassis: Entity,
    wheels: [Entity; 4],
    sections: Vec<Vec<MutableSection>>,
    shown_frame: usize,
}

#[derive(Component)]
struct Readout;

pub(crate) fn run(
    drive: DriveSetup,
    terrain: TerrainImport,
    vehicle: VehicleImport,
) -> Result<bool, String> {
    println!("Recording the reference wall strike…");
    let replay = Replay::record(&drive, &vehicle)?;
    let title = format!("{} | Crash Replay", vehicle.vehicle.name);
    let mut app = App::new();
    app.insert_resource(Scene {
        terrain,
        vehicle,
        drive,
    })
    .insert_resource(replay)
    .init_resource::<Playback>()
    .insert_resource(ClearColor(Color::srgb(0.37, 0.49, 0.60)))
    .add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window { title, ..default() }),
        ..default()
    }))
    .add_systems(Startup, setup)
    .add_systems(Update, (update_playback, draw, view::orbit_camera).chain());
    Ok(app.run().is_success())
}

fn setup(
    mut commands: Commands,
    scene: Res<Scene>,
    replay: Res<Replay>,
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
    let frame = &replay.frames[0];
    let pose = frame.body.pose;
    let chassis = commands
        .spawn((Name::new("Body"), body_transform(pose), Visibility::Visible))
        .id();
    let wheels = std::array::from_fn(|index| {
        commands
            .spawn((
                Name::new(WHEEL_NAMES[index]),
                frame.wheels[index],
                Visibility::Visible,
            ))
            .id()
    });
    let vehicle = &scene.vehicle.vehicle;
    let (instances, parents) =
        wheel_instances(vehicle, scene.drive.wheel_visuals(), chassis, wheels);
    let mut sections = view::spawn_mesh_instances(
        &mut commands,
        &mut assets,
        &scene.vehicle.appearance,
        &vehicle.meshes,
        instances
            .iter()
            .zip(parents)
            .enumerate()
            .map(|(index, (instance, parent))| {
                (instance, Some(parent), replay.objects.contains(&index))
            }),
    );
    commands.insert_resource(Visuals {
        chassis,
        wheels,
        sections: replay
            .objects
            .iter()
            .map(|object| sections.remove(object).unwrap())
            .collect(),
        shown_frame: 0,
    });

    let target = camera_target(pose);
    // Keep the default eye behind the nose so the nearby wall cannot hide it.
    let home = Transform::from_translation(target + pose.rotation * Vec3::new(3.8, 2.0, 0.2))
        .looking_at(target, Vec3::Y);
    commands.spawn((
        Camera3d::default(),
        home,
        OrbitCamera {
            home,
            target,
            minimum_distance: 0.75,
            maximum_distance: 20.0,
        },
    ));
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
            panel.spawn((Readout, Text::default(), TextFont::from_font_size(23.0)));
            panel.spawn((
                Text::new("Space  Play/Pause   Left/Right  Frame   R  Reset"),
                TextFont::from_font_size(15.0),
            ));
            panel.spawn((
                Text::new("Right-drag  Orbit   Scroll  Zoom   Home  Restore camera"),
                TextFont::from_font_size(15.0),
            ));
        });
    commands.remove_resource::<Scene>();
}

fn body_transform(pose: VehiclePose) -> Transform {
    Transform::from_translation(pose.origin_m).with_rotation(pose.rotation)
}

fn camera_target(pose: VehiclePose) -> Vec3 {
    pose.origin_m + pose.rotation * Vec3::new(0.0, 0.57, 1.05)
}

fn update_playback(
    replay: Res<Replay>,
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut playback: ResMut<Playback>,
) {
    playback.update(&keys, time.delta_secs_f64(), replay.frames.len() - 1);
}

fn draw(
    replay: Res<Replay>,
    playback: Res<Playback>,
    mut visuals: ResMut<Visuals>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut bounds: Query<&mut Aabb>,
    mut bodies: Query<&mut Transform, Without<Camera3d>>,
    mut camera: Single<(&mut Transform, &mut OrbitCamera), With<Camera3d>>,
    mut readout: Single<&mut Text, With<Readout>>,
) {
    let index = playback.frame;
    let last = replay.frames.len() - 1;
    let state = if playback.playing {
        "Playing"
    } else {
        "Paused"
    };
    let label = format!(
        "{:.3} s  |  Frame {index}/{last}  |  {state}",
        index as f64 / f64::from(TICKS_PER_SECOND)
    );
    if readout.0 != label {
        readout.0 = label;
    }
    if visuals.shown_frame == index {
        return;
    }
    visuals.shown_frame = index;
    let frame = &replay.frames[index];
    let pose = frame.body.pose;
    *bodies.get_mut(visuals.chassis).unwrap() = body_transform(pose);
    for (entity, transform) in visuals.wheels.into_iter().zip(frame.wheels) {
        *bodies.get_mut(entity).unwrap() = transform;
    }
    for (sections, vertices) in visuals.sections.iter().zip(&frame.body.vertices) {
        for section in sections {
            let mut mesh = meshes.get_mut(&section.mesh).unwrap();
            *bounds.get_mut(section.entity).unwrap() =
                section.update(&mut mesh, vertices, index == 0);
        }
    }
    let (transform, orbit) = &mut *camera;
    orbit.follow(transform, camera_target(pose));
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    use bevy::gizmos::{AppGizmoBuilder, config::DefaultGizmoConfigGroup};
    use vehicle_assets::ror::reference;
    use vehicle_scene::{crash::CRASH_TICKS, drive::AlignmentMode};

    #[test]
    fn replay_steps_plays_and_restores_rendered_body_and_wheels() {
        let terrain = reference::terrain().unwrap();
        let vehicle = reference::vehicle().unwrap();
        let drive = DriveSetup::new(&terrain, &vehicle, AlignmentMode::Authored).unwrap();
        let replay = Replay::record(&drive, &vehicle).unwrap();
        assert_eq!(replay.frames.len(), CRASH_TICKS as usize + 1);

        let mut app = App::new();
        app.insert_resource(Scene {
            terrain,
            vehicle,
            drive,
        })
        .insert_resource(replay)
        .init_gizmo_group::<DefaultGizmoConfigGroup>()
        .init_resource::<Assets<Mesh>>()
        .init_resource::<Assets<StandardMaterial>>()
        .init_resource::<Assets<Image>>()
        .init_resource::<Time>()
        .init_resource::<ButtonInput<KeyCode>>()
        .init_resource::<Playback>()
        .add_systems(Startup, setup)
        .add_systems(Update, (update_playback, draw).chain());
        let world = app.world_mut();
        world.run_schedule(Startup);
        let snapshot = |world: &World| {
            let visuals = world.resource::<Visuals>();
            let meshes = world.resource::<Assets<Mesh>>();
            let vertices: Vec<_> = visuals
                .sections
                .iter()
                .flatten()
                .map(|section| {
                    meshes
                        .get(&section.mesh)
                        .unwrap()
                        .attribute(Mesh::ATTRIBUTE_POSITION)
                        .unwrap()
                        .as_float3()
                        .unwrap()
                        .to_vec()
                })
                .collect();
            let wheels = visuals
                .wheels
                .map(|wheel| *world.get::<Transform>(wheel).unwrap());
            (vertices, wheels)
        };
        let tap = |world: &mut World, key| {
            let mut keys = world.resource_mut::<ButtonInput<KeyCode>>();
            keys.reset_all();
            keys.press(key);
            world.run_schedule(Update);
            world.resource_mut::<ButtonInput<KeyCode>>().reset_all();
        };

        let launch = snapshot(world);
        assert!(!world.resource::<Playback>().playing);
        tap(world, KeyCode::ArrowLeft);
        assert_eq!(world.resource::<Playback>().frame, 0);
        tap(world, KeyCode::ArrowRight);
        assert_eq!(world.resource::<Playback>().frame, 1);
        tap(world, KeyCode::ArrowLeft);
        assert_eq!(snapshot(world), launch);

        tap(world, KeyCode::Space);
        world
            .resource_mut::<Time>()
            .advance_by(Duration::from_secs(3));
        world.run_schedule(Update);
        let end = world.resource::<Playback>();
        assert_eq!(end.frame, CRASH_TICKS as usize);
        assert!(!end.playing);
        let (body, wheels) = snapshot(world);
        assert_ne!(body[0], launch.0[0]);
        // The authored cabin follows the chassis frame as one piece, through
        // both peak impact and final rebound.
        let replay = world.resource::<Replay>();
        let cabin = &replay.frames[0].body.vertices[1];
        for frame in &replay.frames {
            assert!(
                frame.body.vertices[1]
                    .iter()
                    .zip(cabin)
                    .all(|(current, original)| current.abs_diff_eq(*original, 1e-6))
            );
        }
        for (current, original) in body[1..].iter().zip(&launch.0[1..]) {
            assert!(current.iter().zip(original).all(|(current, original)| {
                Vec3::from(*current).abs_diff_eq(Vec3::from(*original), 1e-6)
            }));
        }
        assert!(
            wheels
                .iter()
                .zip(&launch.1)
                .all(|(current, original)| current != original)
        );
        tap(world, KeyCode::ArrowRight);
        assert_eq!(world.resource::<Playback>().frame, CRASH_TICKS as usize);

        tap(world, KeyCode::KeyR);
        assert_eq!(world.resource::<Playback>().frame, 0);
        assert!(!world.resource::<Playback>().playing);
        assert_eq!(snapshot(world), launch);
    }
}

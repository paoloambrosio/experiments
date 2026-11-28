use std::time::{Duration, Instant};

use bevy::{
    camera::primitives::Aabb,
    ecs::change_detection::{NonSend, NonSendMut},
    gizmos::config::GizmoConfigStore,
    prelude::*,
};
use vehicle_assets::{
    VehicleAsset,
    ror::{TerrainImport, VehicleImport},
};
use vehicle_scene::{
    body_skin::BodySkin,
    crash::{Approach, CRASH_TICKS, CrashSetup, DeformableSession, DeformableSetup},
    drive::{
        BENCHMARK_TICKS, DriveSession, DriveSetup, DriverInput, TICKS_PER_SECOND, benchmark_input,
    },
};

use crate::view::{self, MeshAssets, MutableSection};

#[derive(Resource)]
struct DriveScene {
    terrain: TerrainImport,
    vehicle: VehicleImport,
    wheel_visuals: [(usize, vehicle_assets::WheelPhysics); 4],
    camera_anchor: Vec3,
}

#[derive(Resource)]
struct VisualBodies {
    chassis: Entity,
    wheels: [Entity; 4],
    sections: Vec<Vec<MutableSection>>,
    camera_anchor: Vec3,
}

#[derive(Resource)]
struct LiveSkin(BodySkin);

enum InteractiveDrive {
    Deformable(DeformableSession),
    Rigid(DriveSession),
}

impl InteractiveDrive {
    fn reset(&mut self) -> Result<(), String> {
        match self {
            Self::Deformable(session) => session.reset(),
            Self::Rigid(session) => session.reset(session.setup.start),
        }
    }

    fn step(&mut self, input: DriverInput) -> Result<(), String> {
        match self {
            Self::Deformable(session) => session.step(input),
            Self::Rigid(session) => session.step(input),
        }
    }

    fn velocity(&self) -> Vec3 {
        match self {
            Self::Deformable(session) => session.velocity_mps().as_vec3(),
            Self::Rigid(session) => session.chassis().2,
        }
    }
}

#[derive(Resource, Default)]
struct DriveStatus {
    error: Option<String>,
    reset_requested: bool,
    camera_reset: bool,
    shape_dirty: bool,
    rest_shape: bool,
}

#[derive(Component)]
struct SpeedReadout;

#[derive(Resource)]
struct Benchmark {
    warmup: Option<Instant>,
    approach: Approach,
    ordinary_passed: Option<bool>,
    timings: FrameTimings,
}

#[derive(Default)]
struct FrameTimings {
    started: Option<Instant>,
    ticks: u32,
    physics_ms: Vec<f64>,
    mesh_ms: Vec<f64>,
    input_sample: Option<Instant>,
    response_ms: Vec<f64>,
    ticks_at_deadline: Option<u32>,
    max_lag_ticks: u32,
}

impl FrameTimings {
    fn measure_frame(&mut self, limit: u32) -> Option<f64> {
        let elapsed = self.started?.elapsed().as_secs_f64();
        if let Some(sample) = self.input_sample.take() {
            self.response_ms
                .push(sample.elapsed().as_secs_f64() * 1000.0);
        }
        let expected = (elapsed * f64::from(TICKS_PER_SECOND)).floor() as u32;
        self.max_lag_ticks = self
            .max_lag_ticks
            .max(expected.min(limit).saturating_sub(self.ticks));
        Some(elapsed)
    }

    fn print_phase(&mut self, label: &str, limit: u32, elapsed: f64) {
        let ticks = self.ticks_at_deadline.unwrap_or(self.ticks);
        println!(
            "{label} | {ticks}/{limit} ticks by {} s | {:.2} s measured | max frame-boundary lag {} ticks",
            limit / TICKS_PER_SECOND,
            elapsed,
            self.max_lag_ticks,
        );
        Self::print("Physics tick", &mut self.physics_ms);
        Self::print("Live mesh update", &mut self.mesh_ms);
        Self::print(
            "Input sample to updated render state",
            &mut self.response_ms,
        );
    }

    fn print(label: &str, samples: &mut [f64]) {
        if samples.is_empty() {
            return;
        }
        samples.sort_by(f64::total_cmp);
        let percentile = |p: usize| samples[(samples.len() * p).div_ceil(100) - 1];
        println!(
            "{label} | median {:.3} ms | p95 {:.3} ms | worst {:.3} ms",
            percentile(50),
            percentile(95),
            percentile(100)
        );
    }
}

pub(crate) enum DriveMode {
    Reference,
    Benchmark,
    Configured,
}

pub(crate) fn run(
    drive: DriveSetup,
    terrain: TerrainImport,
    vehicle: VehicleImport,
    mode: DriveMode,
) -> Result<bool, String> {
    let mut app = App::new();
    let wheel_visuals = drive.wheel_visuals();
    let camera_anchor = drive.centre_of_mass();
    let entry = &vehicle.entry;
    let terrain_name = &terrain.terrain.name;
    let benchmark = matches!(mode, DriveMode::Benchmark);
    let fixed_time = Time::<Fixed>::from_hz(f64::from(TICKS_PER_SECOND));
    if !benchmark {
        // A slow physics tick must not lead to a long chain of catch-up
        // ticks before the app can read input and render again.
        app.insert_resource(Time::<Virtual>::from_max_delta(2 * fixed_time.timestep()));
    }
    let (session, skin) = if matches!(mode, DriveMode::Configured) {
        start_configured(drive, &vehicle.vehicle, entry, terrain_name)?
    } else {
        let deformable = if benchmark {
            let CrashSetup { approach, vehicle } =
                CrashSetup::new(&drive, &vehicle.vehicle, "wall")?;
            app.insert_resource(Benchmark {
                warmup: None,
                approach,
                ordinary_passed: None,
                timings: FrameTimings::default(),
            });
            println!("Window benchmark | 2 s warm-up, 10 s driving, then a frontal strike");
            vehicle
        } else {
            DeformableSetup::new(&drive, &vehicle.vehicle, "wall")?
        };
        let substeps = deformable.substeps;
        let (session, skin) = start_deformable(&drive, &vehicle.vehicle, deformable)?;
        drive.print_setup(entry, terrain_name, substeps);
        (InteractiveDrive::Deformable(session), Some(skin))
    };
    let title = format!("{} | {}", vehicle.vehicle.name, terrain.terrain.name);
    app.insert_non_send(session);
    if let Some(skin) = skin {
        app.insert_resource(LiveSkin(skin));
    }
    app.insert_resource(DriveScene {
        terrain,
        vehicle,
        wheel_visuals,
        camera_anchor,
    })
    .insert_resource(DriveStatus {
        shape_dirty: true,
        rest_shape: true,
        camera_reset: true,
        ..default()
    })
    .insert_resource(ClearColor(Color::srgb(0.37, 0.49, 0.60)))
    .insert_resource(fixed_time)
    .add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title,
            present_mode: if benchmark {
                bevy::window::PresentMode::AutoNoVsync
            } else {
                default()
            },
            ..default()
        }),
        ..default()
    }))
    .add_systems(Startup, setup)
    .add_systems(PreUpdate, request_reset.after(bevy::input::InputSystems))
    .add_systems(FixedUpdate, advance)
    .add_systems(Update, (draw, view::collision_overlay))
    .add_systems(Last, finish_benchmark);
    Ok(app.run().is_success())
}

fn start_deformable(
    drive: &DriveSetup,
    asset: &VehicleAsset,
    deformable: DeformableSetup,
) -> Result<(DeformableSession, BodySkin), String> {
    let skin = BodySkin::new(asset, &deformable.imported.source)?;
    let mut session = deformable.start(drive.start, |_| {})?;
    session.reset()?;
    skin.sample(&session.frame().nodes_m)?;
    Ok((session, skin))
}

fn start_configured(
    drive: DriveSetup,
    asset: &VehicleAsset,
    entry: &str,
    terrain_name: &str,
) -> Result<(InteractiveDrive, Option<BodySkin>), String> {
    let deformable = DeformableSetup::for_drive(&drive, asset).and_then(|setup| {
        let substeps = setup.substeps;
        let (mut session, skin) = start_deformable(&drive, asset, setup)?;
        // Check that the first loaded tick and its skin can commit before
        // opening the window, then start the player at the authored pose.
        session.step(DriverInput::default())?;
        skin.sample(&session.frame().nodes_m)?;
        session.reset()?;
        Ok::<_, String>((session, skin, substeps))
    });
    match deformable {
        Ok((session, skin, substeps)) => {
            println!("Chassis: deformable");
            drive.print_setup(entry, terrain_name, substeps);
            return Ok((InteractiveDrive::Deformable(session), Some(skin)));
        }
        Err(error) => eprintln!("Using a rigid chassis for {entry}: {error}"),
    }
    println!("Chassis: rigid with deformable tyres");
    drive.print_setup(entry, terrain_name, DriveSession::SUBSTEPS);
    Ok((InteractiveDrive::Rigid(DriveSession::new(drive)?), None))
}

fn setup(
    mut commands: Commands,
    scene: Res<DriveScene>,
    skin: Option<Res<LiveSkin>>,
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
    let (instances, parents) = wheel_instances(vehicle, scene.wheel_visuals, chassis, wheels);
    let bindings = skin
        .as_ref()
        .map_or(&[][..], |skin| skin.0.meshes.as_slice());
    let mut sections = view::spawn_mesh_instances(
        &mut commands,
        &mut assets,
        &scene.vehicle.appearance,
        &vehicle.meshes,
        instances
            .iter()
            .zip(parents)
            .enumerate()
            .map(|(i, (instance, parent))| {
                (
                    instance,
                    Some(parent),
                    bindings.iter().any(|mesh| mesh.object == i),
                )
            }),
    );
    commands.insert_resource(VisualBodies {
        chassis,
        wheels,
        camera_anchor: scene.camera_anchor,
        sections: bindings
            .iter()
            .map(|mesh| sections.remove(&mesh.object).unwrap_or_default())
            .collect(),
    });
    commands.spawn(Camera3d::default());
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

pub(crate) fn wheel_instances(
    vehicle: &vehicle_assets::VehicleAsset,
    wheel_visuals: [(usize, vehicle_assets::WheelPhysics); 4],
    chassis: Entity,
    wheels: [Entity; 4],
) -> (Vec<vehicle_assets::MeshInstance>, Vec<Entity>) {
    let mut instances = vehicle.objects.clone();
    let mut parents = vec![chassis; instances.len()];
    for (wheel, (index, detail)) in wheels.into_iter().zip(wheel_visuals) {
        let inverse = instances[detail.tyre_object].rotation.inverse();
        for i in [detail.rim_object, detail.tyre_object] {
            let instance = &mut instances[i];
            instance.translation = inverse * (instance.translation - vehicle.wheels[index].centre);
            instance.rotation = inverse * instance.rotation;
            parents[i] = wheel;
        }
    }
    (instances, parents)
}

fn request_reset(keys: Res<ButtonInput<KeyCode>>, mut status: ResMut<DriveStatus>) {
    status.reset_requested |= keys.just_pressed(KeyCode::KeyR);
}

fn advance(
    keys: Res<ButtonInput<KeyCode>>,
    mut session: NonSendMut<InteractiveDrive>,
    mut status: ResMut<DriveStatus>,
    benchmark: Option<ResMut<Benchmark>>,
) {
    if std::mem::take(&mut status.reset_requested) {
        status.error = session.reset().err();
        status.camera_reset = true;
        status.shape_dirty = status.error.is_none();
        status.rest_shape = true;
        return;
    }
    if status.error.is_none() {
        let pressed = |key| f32::from(u8::from(keys.pressed(key)));
        let mut benchmark = benchmark;
        let start = Instant::now();
        let input = if let Some(bench) = benchmark.as_deref_mut() {
            let impact = bench.ordinary_passed.is_some();
            let limit = if impact { CRASH_TICKS } else { BENCHMARK_TICKS };
            if bench
                .warmup
                .is_none_or(|warmup| warmup.elapsed() < Duration::from_secs(2))
                || bench.timings.ticks >= limit
            {
                return;
            }
            if impact && bench.timings.started.is_none() {
                let approach = bench.approach;
                let InteractiveDrive::Deformable(session) = &mut *session else {
                    unreachable!("the benchmark uses the bundled deformable car");
                };
                if let Err(error) = session.spawn_at(approach.pose, approach.velocity(), None) {
                    status.error = Some(error);
                    return;
                }
                status.camera_reset = true;
            }
            bench.timings.started.get_or_insert(start);
            let tick = bench.timings.ticks;
            if impact {
                // The frontal strike reaches the wall during tick 16.
                if tick == 17 {
                    bench.timings.input_sample = Some(start);
                }
                DriverInput {
                    brake: f32::from(u8::from(tick >= 17)),
                    ..default()
                }
            } else {
                if matches!(tick, 120 | 250 | 310) {
                    bench.timings.input_sample = Some(start);
                }
                benchmark_input(tick)
            }
        } else {
            DriverInput {
                throttle: pressed(KeyCode::KeyW),
                brake: pressed(KeyCode::KeyS),
                steer: pressed(KeyCode::KeyA) - pressed(KeyCode::KeyD),
            }
        };
        status.error = session.step(input).err();
        if let Some(bench) = benchmark.as_deref_mut() {
            let elapsed = start.elapsed().as_secs_f64() * 1000.0;
            bench.timings.physics_ms.push(elapsed);
            let seconds = if bench.ordinary_passed.is_some() {
                2
            } else {
                10
            };
            if bench.timings.started.unwrap().elapsed() >= Duration::from_secs(seconds) {
                bench
                    .timings
                    .ticks_at_deadline
                    .get_or_insert(bench.timings.ticks);
            }
            bench.timings.ticks += 1;
        }
        status.shape_dirty |= status.error.is_none();
        status.rest_shape = false;
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
    session: NonSend<InteractiveDrive>,
    skin: Option<Res<LiveSkin>>,
    visuals: Res<VisualBodies>,
    mut status: ResMut<DriveStatus>,
    time: Res<Time>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut bounds: Query<&mut Aabb>,
    mut bodies: Query<&mut Transform, Without<Camera3d>>,
    mut camera: Single<&mut Transform, With<Camera3d>>,
    mut readout: Single<&mut Text, With<SpeedReadout>>,
    benchmark: Option<ResMut<Benchmark>>,
) {
    let start = Instant::now();
    let shape_dirty = status.shape_dirty;
    if std::mem::take(&mut status.shape_dirty) {
        let pose = match &*session {
            InteractiveDrive::Rigid(session) => {
                let (centre, rotation, _) = session.chassis();
                Some((
                    centre - rotation * visuals.camera_anchor,
                    rotation,
                    session.rims(),
                ))
            }
            InteractiveDrive::Deformable(session) => {
                let frame = session.frame();
                match skin.as_ref().unwrap().0.sample(&frame.nodes_m) {
                    Ok(shape) => {
                        for (sections, vertices) in visuals.sections.iter().zip(&shape.vertices) {
                            for section in sections {
                                let mut mesh = meshes.get_mut(&section.mesh).unwrap();
                                *bounds.get_mut(section.entity).unwrap() =
                                    section.update(&mut mesh, vertices, status.rest_shape);
                            }
                        }
                        Some((shape.pose.origin_m, shape.pose.rotation, frame.rims))
                    }
                    Err(error) => {
                        status.error = Some(error);
                        None
                    }
                }
            }
        };
        if let Some((origin, rotation, rims)) = pose {
            *bodies.get_mut(visuals.chassis).unwrap() =
                Transform::from_translation(origin).with_rotation(rotation);
            for (entity, rim) in visuals.wheels.into_iter().zip(rims) {
                *bodies.get_mut(entity).unwrap() =
                    Transform::from_translation(rim.centre.as_vec3())
                        .with_rotation(rim.rotation.as_quat());
            }
        }
    }
    let chassis = bodies.get(visuals.chassis).unwrap();
    let centre = chassis.translation + chassis.rotation * visuals.camera_anchor;
    let target = chase_pose(centre, chassis.rotation);
    if std::mem::take(&mut status.camera_reset) {
        **camera = target;
    } else {
        let weight = 1.0 - (-10.0 * time.delta_secs()).exp();
        camera.translation = camera.translation.lerp(target.translation, weight);
        camera.rotation = camera.rotation.slerp(target.rotation, weight);
    }
    let speed = session.velocity().with_y(0.0).length() * 3.6;
    let label = match &status.error {
        Some(error) => format!("{speed:.0} km/h\n{error}"),
        None => format!("{speed:.0} km/h"),
    };
    if readout.0 != label {
        readout.0 = label;
    }
    if let Some(mut benchmark) = benchmark
        && benchmark.timings.started.is_some()
        && shape_dirty
    {
        benchmark
            .timings
            .mesh_ms
            .push(start.elapsed().as_secs_f64() * 1000.0);
    }
}

fn finish_benchmark(
    benchmark: Option<ResMut<Benchmark>>,
    status: Res<DriveStatus>,
    mut exit: MessageWriter<AppExit>,
) {
    let Some(mut benchmark) = benchmark else {
        return;
    };
    if let Some(error) = &status.error {
        eprintln!("Window benchmark: {error}");
        exit.write(AppExit::error());
        return;
    }
    benchmark.warmup.get_or_insert_with(Instant::now);
    if let Some(ordinary_passed) = benchmark.ordinary_passed {
        if let Some(elapsed) = benchmark.timings.measure_frame(CRASH_TICKS)
            && benchmark.timings.ticks == CRASH_TICKS
        {
            benchmark
                .timings
                .print_phase("Frontal impact", CRASH_TICKS, elapsed);
            exit.write(
                if ordinary_passed && benchmark.timings.response_ms.len() == 1 {
                    AppExit::Success
                } else {
                    AppExit::error()
                },
            );
        }
    } else if let Some(elapsed) = benchmark.timings.measure_frame(BENCHMARK_TICKS)
        && elapsed >= f64::from(BENCHMARK_TICKS / TICKS_PER_SECOND)
    {
        let timings = &mut benchmark.timings;
        timings.print_phase("Ordinary driving", BENCHMARK_TICKS, elapsed);
        let ticks = timings.ticks_at_deadline.unwrap_or(timings.ticks);
        benchmark.ordinary_passed =
            Some(ticks >= BENCHMARK_TICKS - 1 && timings.response_ms.len() == 3);
        benchmark.timings = FrameTimings::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::gizmos::{AppGizmoBuilder, config::DefaultGizmoConfigGroup};
    use vehicle_assets::ror::reference;
    use vehicle_scene::drive::AlignmentMode;

    fn deformable(world: &World) -> &DeformableSession {
        match world.non_send::<InteractiveDrive>() {
            InteractiveDrive::Deformable(session) => session,
            _ => panic!("expected a deformable test session"),
        }
    }

    #[test]
    fn configured_drive_defaults_to_deformation_without_a_named_wall_and_keeps_a_rigid_fallback() {
        let mut terrain = reference::terrain().unwrap();
        terrain
            .terrain
            .objects
            .retain(|object| object.name != "wall");
        let mut vehicle = reference::vehicle().unwrap();
        for expected_deformable in [true, false] {
            let setup = DriveSetup::new(&terrain, &vehicle, AlignmentMode::Authored).unwrap();
            let (mut session, skin) = start_configured(
                setup,
                &vehicle.vehicle,
                &vehicle.entry,
                &terrain.terrain.name,
            )
            .unwrap();
            assert_eq!(skin.is_some(), expected_deformable);
            assert_eq!(
                matches!(session, InteractiveDrive::Deformable(_)),
                expected_deformable
            );
            session
                .step(DriverInput {
                    throttle: 1.0,
                    ..default()
                })
                .unwrap();
            session.reset().unwrap();
            assert_eq!(session.velocity(), Vec3::ZERO);
            vehicle.vehicle.physics.structure = None;
        }
    }

    #[test]
    fn driving_keys_move_and_reset_the_bundled_car() {
        let terrain = reference::terrain().unwrap();
        let vehicle = reference::vehicle().unwrap();
        let setup = DriveSetup::new(&terrain, &vehicle, AlignmentMode::Authored).unwrap();
        let CrashSetup {
            vehicle: deformable,
            approach,
        } = CrashSetup::new(&setup, &vehicle.vehicle, "wall").unwrap();
        let skin = BodySkin::new(&vehicle.vehicle, &deformable.imported.source).unwrap();
        let mut session = deformable.start(setup.start, |_| {}).unwrap();
        session.reset().unwrap();
        let mut app = App::new();
        app.insert_non_send(InteractiveDrive::Deformable(session));
        app.insert_resource(DriveScene {
            terrain,
            vehicle,
            wheel_visuals: setup.wheel_visuals(),
            camera_anchor: setup.centre_of_mass(),
        })
        .insert_resource(LiveSkin(skin))
        .insert_resource(DriveStatus {
            shape_dirty: true,
            rest_shape: true,
            camera_reset: true,
            ..default()
        })
        .init_resource::<ButtonInput<KeyCode>>()
        .init_resource::<Assets<Mesh>>()
        .init_resource::<Assets<StandardMaterial>>()
        .init_resource::<Assets<Image>>()
        .init_resource::<Time>()
        .init_gizmo_group::<DefaultGizmoConfigGroup>()
        .add_systems(Startup, super::setup)
        .add_systems(PreUpdate, request_reset)
        .add_systems(FixedUpdate, advance)
        .add_systems(Update, draw);
        let world = app.world_mut();
        world.run_schedule(Startup);
        world.run_schedule(Update);
        let start = self::deformable(world).frame().nodes_m;
        let snapshot = |world: &mut World| {
            let visuals = world.resource::<VisualBodies>();
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
            let camera = *world
                .query_filtered::<&Transform, With<Camera3d>>()
                .single(world)
                .unwrap();
            (vertices, wheels, camera)
        };
        let original = snapshot(world);

        world
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::KeyW);
        world.run_schedule(FixedUpdate);
        assert!(
            self::deformable(world)
                .wheel_controls()
                .iter()
                .any(|wheel| wheel.drive_nm > 0.0)
        );
        for _ in 1..TICKS_PER_SECOND {
            world.run_schedule(FixedUpdate);
        }
        let session = self::deformable(world);
        assert!(session.frame().nodes_m[0].z - start[0].z > 0.5);
        assert!(session.velocity_mps().z > 1.0);

        world
            .resource_mut::<ButtonInput<KeyCode>>()
            .release(KeyCode::KeyW);
        {
            let mut drive = world.non_send_mut::<InteractiveDrive>();
            let InteractiveDrive::Deformable(session) = &mut *drive else {
                unreachable!()
            };
            session
                .spawn_at(approach.pose, approach.velocity(), None)
                .unwrap();
        }
        for _ in 0..CRASH_TICKS {
            world.run_schedule(FixedUpdate);
        }
        world.run_schedule(Update);
        assert!(world.resource::<DriveStatus>().error.is_none());
        let crushed = snapshot(world);
        assert!(
            crushed.0[0]
                .iter()
                .zip(&original.0[0])
                .any(|(crushed, original)| original[2] - crushed[2] > 0.10)
        );
        assert_ne!(crushed.1, original.1);

        // Reset also clears the pause caused by a simulation or skinning error.
        world.resource_mut::<DriveStatus>().error = Some("test failure".into());
        world
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::KeyR);
        world.run_schedule(PreUpdate);
        world.run_schedule(FixedUpdate);
        let session = self::deformable(world);
        assert_eq!(session.frame().nodes_m, start);
        assert_eq!(session.velocity_mps(), bevy::math::DVec3::ZERO);
        let status = world.resource::<DriveStatus>();
        assert!(status.error.is_none());
        assert!(status.camera_reset);
        world.run_schedule(Update);
        assert_eq!(snapshot(world), original);
    }
}

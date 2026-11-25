use std::collections::{HashMap, HashSet};

use bevy::{
    asset::RenderAssetUsages,
    color::palettes::css::AQUAMARINE,
    ecs::system::SystemParam,
    gizmos::config::{DefaultGizmoConfigGroup, GizmoConfigStore},
    image::{ImageAddressMode, ImageSampler, ImageSamplerDescriptor},
    input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll, MouseScrollUnit},
    mesh::{Indices, PrimitiveTopology},
    prelude::*,
    render::render_resource::{Extent3d, Face, TextureDimension, TextureFormat},
};
use vehicle_assets::{
    DiffuseTexture, MeshAppearance, MeshId, MeshInstance, TerrainAsset, TriangleMesh,
    WheelGeometry,
    ror::{TerrainImport, VehicleImport},
};

use crate::placement::triangulate;

#[derive(Resource)]
pub(crate) struct Scene {
    pub terrain: TerrainImport,
    pub vehicle: Option<ParkedVehicle>,
}

pub(crate) struct ParkedVehicle {
    pub import: VehicleImport,
    pub translation: Vec3,
}

#[derive(SystemParam)]
struct MeshAssets<'w> {
    meshes: ResMut<'w, Assets<Mesh>>,
    materials: ResMut<'w, Assets<StandardMaterial>>,
    images: ResMut<'w, Assets<Image>>,
}

#[derive(Resource)]
struct CollisionOverlay {
    edges: Vec<[Vec3; 2]>,
    visible: bool,
}

#[derive(Component)]
struct FlyCamera {
    overview: Transform,
    speed: f32,
}

pub(crate) fn run(scene: Scene) -> bool {
    let title = if let Some(vehicle) = &scene.vehicle {
        format!(
            "{} | {}",
            vehicle.import.vehicle.name, scene.terrain.terrain.name
        )
    } else {
        format!("{} | Terrain Viewer", scene.terrain.terrain.name)
    };
    let inspect = scene.vehicle.is_some();
    let mut app = App::new();
    app.insert_resource(scene)
        .insert_resource(ClearColor(Color::srgb(0.37, 0.49, 0.60)))
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window { title, ..default() }),
            ..default()
        }))
        .add_systems(Startup, setup)
        .add_systems(Update, collision_overlay);
    if inspect {
        app.add_systems(Update, (orbit_camera, structure_overlay));
    } else {
        app.add_systems(Update, fly_camera);
    }
    app.run().is_success()
}

fn setup(
    mut commands: Commands,
    source: Res<Scene>,
    mut assets: MeshAssets,
    mut gizmo_config: ResMut<GizmoConfigStore>,
) {
    let TerrainImport {
        terrain,
        appearance,
        ..
    } = &source.terrain;
    spawn_meshes(
        &mut commands,
        &mut assets,
        appearance,
        &terrain.meshes,
        &terrain.objects,
        None,
    );
    if let Some(grid) = &terrain.heightfield {
        let terrain_mesh = triangulate(grid);
        commands.spawn((
            Name::new("Heightfield"),
            Mesh3d(
                assets
                    .meshes
                    .add(to_bevy_mesh(&terrain_mesh, &terrain_mesh.triangles, false)),
            ),
            MeshMaterial3d(assets.materials.add(Color::srgb(0.35, 0.39, 0.34))),
        ));
    }

    if let Some(parked) = &source.vehicle {
        setup_vehicle(&mut commands, parked, &mut assets);
    } else {
        let (min, max) = scene_bounds(terrain);
        let centre = (max + min) * 0.5;
        let extent = (max - min).max_element().max(10.0);
        let overview = Transform::from_translation(centre + extent * Vec3::new(0.86, 0.93, 0.95))
            .looking_at(centre + extent * Vec3::new(0.03, 0.0, 0.03), Vec3::Y);
        commands.spawn((
            Camera3d::default(),
            overview,
            FlyCamera {
                overview,
                speed: extent / 8.0,
            },
        ));
    }
    commands.insert_resource(GlobalAmbientLight {
        brightness: 700.0,
        ..default()
    });
    commands.spawn((
        DirectionalLight {
            illuminance: 9300.0,
            shadow_maps_enabled: source.vehicle.is_some(),
            ..default()
        },
        Transform::from_xyz(1.0, 2.0, 1.5).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    gizmo_config
        .config_mut::<DefaultGizmoConfigGroup>()
        .0
        .depth_bias = -0.001;
    commands.insert_resource(CollisionOverlay {
        edges: edges(terrain),
        visible: false,
    });
    commands.remove_resource::<Scene>();
}

fn spawn_meshes(
    commands: &mut Commands,
    assets: &mut MeshAssets,
    appearance: &MeshAppearance,
    geometry: &[TriangleMesh],
    instances: &[MeshInstance],
    parent: Option<Entity>,
) {
    let MeshAssets {
        meshes,
        materials,
        images,
    } = assets;
    let textures: Vec<_> = appearance
        .textures
        .iter()
        .map(|texture| images.add(texture_image(texture)))
        .collect();
    let surface_materials: Vec<_> = appearance
        .materials
        .iter()
        .map(|material| {
            let [r, g, b, a] = material.rgba;
            materials.add(StandardMaterial {
                base_color: Color::srgba(r, g, b, a),
                base_color_texture: material.base_texture.map(|id| textures[id.0].clone()),
                cull_mode: (!material.double_sided).then_some(Face::Back),
                double_sided: material.double_sided,
                perceptual_roughness: 0.9,
                alpha_mode: if material.transparent {
                    AlphaMode::Blend
                } else {
                    AlphaMode::Opaque
                },
                ..default()
            })
        })
        .collect();

    let mut rendered = HashMap::new();
    for instance in instances {
        let Some(id) = instance.visual_mesh else {
            continue;
        };
        let reflected = instance
            .scale
            .to_array()
            .into_iter()
            .filter(|v| v.is_sign_negative())
            .count()
            % 2
            == 1;
        let sections = rendered.entry((id, reflected)).or_insert_with(|| {
            let mesh = &geometry[id.0];
            appearance.sections[&id]
                .iter()
                .map(|section| {
                    (
                        meshes.add(to_bevy_mesh(
                            mesh,
                            &mesh.triangles[section.triangles.clone()],
                            reflected,
                        )),
                        surface_materials[section.material.0].clone(),
                    )
                })
                .collect::<Vec<_>>()
        });
        for (mesh, material) in sections {
            let mut entity = commands.spawn((
                Name::new(instance.name.clone()),
                Mesh3d(mesh.clone()),
                MeshMaterial3d(material.clone()),
                Transform {
                    translation: instance.translation,
                    rotation: instance.rotation,
                    scale: instance.scale,
                },
            ));
            if let Some(parent) = parent {
                entity.insert(ChildOf(parent));
            }
        }
    }
}

#[derive(Component)]
struct OrbitCamera {
    home: Transform,
    target: Vec3,
    minimum_distance: f32,
    maximum_distance: f32,
}

#[derive(Resource)]
struct Inspection {
    root: Entity,
    translation: Vec3,
    nodes: Vec<Vec3>,
    edges: Vec<[vehicle_assets::NodeId; 2]>,
    wheels: Vec<WheelGeometry>,
    nose: Vec3,
    structural: bool,
}

fn setup_vehicle(commands: &mut Commands, parked: &ParkedVehicle, assets: &mut MeshAssets) {
    let vehicle = &parked.import.vehicle;
    let root = commands
        .spawn((
            Name::new(vehicle.name.clone()),
            Transform::from_translation(parked.translation),
            Visibility::Visible,
        ))
        .id();
    spawn_meshes(
        commands,
        assets,
        &parked.import.appearance,
        &vehicle.meshes,
        &vehicle.objects,
        Some(root),
    );

    let (mut min, mut max) = (Vec3::splat(f32::INFINITY), Vec3::splat(f32::NEG_INFINITY));
    let visuals = vehicle
        .objects
        .iter()
        .filter_map(|instance| Some((instance, &vehicle.meshes[instance.visual_mesh?.0])));
    for point in visuals.flat_map(|(instance, mesh)| {
        mesh.vertices
            .iter()
            .map(move |&point| instance.transform_point(point))
    }) {
        min = min.min(point);
        max = max.max(point);
    }
    if !min.is_finite() {
        for &point in &vehicle.nodes {
            min = min.min(point);
            max = max.max(point);
        }
    }
    let centre = (max + min) * 0.5;
    let extent = (max - min).max_element().max(1.0);
    let target = parked.translation + centre;
    let home = Transform::from_translation(target + extent * Vec3::new(0.88, 0.49, 0.98))
        .looking_at(target, Vec3::Y);
    commands.spawn((
        Camera3d::default(),
        home,
        OrbitCamera {
            home,
            target,
            minimum_distance: extent * 0.45,
            maximum_distance: extent * 15.0,
        },
    ));
    commands.insert_resource(Inspection {
        root,
        translation: parked.translation,
        nodes: vehicle.nodes.clone(),
        edges: vehicle.edges.clone(),
        wheels: vehicle.wheels.clone(),
        nose: Vec3::new(centre.x, centre.y + 0.1, max.z),
        structural: false,
    });
}

fn orbit_camera(
    mut camera: Single<(&mut Transform, &OrbitCamera)>,
    keys: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
) {
    let (transform, orbit) = &mut *camera;
    if keys.just_pressed(KeyCode::Home) {
        **transform = orbit.home;
        return;
    }
    let delta = if mouse.pressed(MouseButton::Right) {
        motion.delta
    } else {
        Vec2::ZERO
    };
    let zoom = match scroll.unit {
        MouseScrollUnit::Line => scroll.delta.y * 0.14,
        MouseScrollUnit::Pixel => scroll.delta.y * 0.002,
    };
    if delta == Vec2::ZERO && zoom == 0.0 {
        return;
    }
    let distance = (transform.translation.distance(orbit.target) * (-zoom).exp())
        .clamp(orbit.minimum_distance, orbit.maximum_distance);
    let (yaw, pitch, _) = transform.rotation.to_euler(EulerRot::YXZ);
    let rotation = Quat::from_euler(
        EulerRot::YXZ,
        yaw - delta.x * 0.004,
        (pitch - delta.y * 0.004).clamp(-1.4, -0.04),
        0.0,
    );
    transform.translation = orbit.target + rotation * Vec3::Z * distance;
    transform.rotation = rotation;
}

fn structure_overlay(
    mut gizmos: Gizmos,
    keys: Res<ButtonInput<KeyCode>>,
    mut inspection: ResMut<Inspection>,
    mut visibilities: Query<&mut Visibility>,
) {
    if keys.just_pressed(KeyCode::KeyB) {
        inspection.structural = !inspection.structural;
        if let Ok(mut visibility) = visibilities.get_mut(inspection.root) {
            *visibility = if inspection.structural {
                Visibility::Hidden
            } else {
                Visibility::Visible
            };
        }
    }
    if !inspection.structural {
        return;
    }

    let origin = inspection.translation;
    for &[a, b] in &inspection.edges {
        gizmos.line(
            origin + inspection.nodes[a.0],
            origin + inspection.nodes[b.0],
            Color::srgb(0.22, 0.57, 0.72),
        );
    }
    for &node in &inspection.nodes {
        let centre = origin + node;
        for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
            gizmos.line(
                centre - 0.012 * axis,
                centre + 0.012 * axis,
                Color::srgb(0.97, 0.76, 0.30),
            );
        }
    }
    for wheel in &inspection.wheels {
        let centre = origin + wheel.centre;
        let up = (Vec3::Y - wheel.axle * wheel.axle.y).normalize_or_zero();
        let forward = wheel.axle.cross(up);
        for side in [-0.5, 0.5] {
            let face = centre + wheel.axle * (side * wheel.width);
            let points = (0..32).map(|index| {
                let phase = index as f32 * std::f32::consts::TAU / 32.0;
                let (sin, cos) = phase.sin_cos();
                face + wheel.outer_radius * (cos * up + sin * forward)
            });
            gizmos.lineloop(points, Color::srgb(0.40, 0.90, 1.0));
        }
        let span = wheel.width * 0.9;
        gizmos.arrow(
            centre - wheel.axle * span,
            centre + wheel.axle * span,
            Color::srgb(1.0, 0.54, 0.25),
        );
    }
    let nose = origin + inspection.nose;
    gizmos.arrow(nose, nose + Vec3::Z * 0.75, Color::srgb(1.0, 0.54, 0.25));
}

fn texture_image(texture: &DiffuseTexture) -> Image {
    use ::image::{
        ImageBuffer, Rgba,
        imageops::{FilterType, resize},
    };

    let size = Extent3d {
        width: texture.width,
        height: texture.height,
        depth_or_array_layers: 1,
    };
    let mut image = Image::new_uninit(
        size,
        TextureDimension::D2,
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::RENDER_WORLD,
    );

    let mut level =
        ImageBuffer::<Rgba<u8>, _>::from_raw(texture.width, texture.height, texture.rgba8.clone())
            .expect("the importer builds RGBA8 textures");
    let mut data = level.as_raw().clone();
    while level.width() > 1 || level.height() > 1 {
        level = resize(
            &level,
            (level.width() / 2).max(1),
            (level.height() / 2).max(1),
            FilterType::Triangle,
        );
        data.extend_from_slice(level.as_raw());
        image.texture_descriptor.mip_level_count += 1;
    }
    image.data = Some(data);
    image.sampler = ImageSampler::Descriptor(ImageSamplerDescriptor {
        address_mode_u: ImageAddressMode::Repeat,
        address_mode_v: ImageAddressMode::Repeat,
        anisotropy_clamp: 8,
        ..ImageSamplerDescriptor::linear()
    });
    image
}

fn to_bevy_mesh(mesh: &TriangleMesh, triangles: &[[u32; 3]], reflected: bool) -> Mesh {
    let mut render = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::RENDER_WORLD,
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, mesh.vertices.clone());
    if let Some(uvs) = &mesh.uvs {
        render.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs.clone());
    }
    let indices = triangles
        .iter()
        .flat_map(|&[a, b, c]| if reflected { [a, c, b] } else { [a, b, c] })
        .collect();
    render.insert_indices(Indices::U32(indices));
    if let Some(normals) = &mesh.normals {
        render.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals.clone());
        render
    } else {
        let mut render = render.with_computed_normals();
        if reflected
            && let Some(bevy::mesh::VertexAttributeValues::Float32x3(normals)) =
                render.attribute_mut(Mesh::ATTRIBUTE_NORMAL)
        {
            for normal in normals {
                *normal = normal.map(|v| -v);
            }
        }
        render
    }
}

fn scene_bounds(terrain: &TerrainAsset) -> (Vec3, Vec3) {
    let mut min = Vec3::splat(f32::INFINITY);
    let mut max = Vec3::splat(f32::NEG_INFINITY);
    let mut include = |point: Vec3| {
        min = min.min(point);
        max = max.max(point);
    };
    if let Some(grid) = &terrain.heightfield {
        for z in 0..grid.samples_z {
            for x in 0..grid.samples_x {
                include(grid.sample_position(x, z).expect("validated heightfield"));
            }
        }
    }
    for instance in &terrain.objects {
        for id in [instance.visual_mesh, instance.collision_mesh]
            .into_iter()
            .flatten()
        {
            for &vertex in &terrain.meshes[id.0].vertices {
                include(instance.transform_point(vertex));
            }
        }
    }
    (min, max)
}

fn edges(terrain: &TerrainAsset) -> Vec<[Vec3; 2]> {
    let mut cache: HashMap<MeshId, Vec<[u32; 2]>> = HashMap::new();
    let mut edges = Vec::new();
    for instance in &terrain.objects {
        let Some(id) = instance.collision_mesh else {
            continue;
        };
        let mesh = &terrain.meshes[id.0];
        let pairs = cache.entry(id).or_insert_with(|| {
            let mut seen = HashSet::new();
            mesh.triangles
                .iter()
                .flat_map(|&[a, b, c]| [[a, b], [b, c], [c, a]])
                .filter(|&[a, b]| seen.insert([a.min(b), a.max(b)]))
                .collect()
        });
        edges.extend(
            pairs
                .iter()
                .map(|pair| pair.map(|id| instance.transform_point(mesh.vertices[id as usize]))),
        );
    }
    edges
}

fn collision_overlay(
    mut gizmos: Gizmos,
    keys: Res<ButtonInput<KeyCode>>,
    mut overlay: ResMut<CollisionOverlay>,
) {
    if keys.just_pressed(KeyCode::KeyC) {
        overlay.visible = !overlay.visible;
    }
    if overlay.visible {
        for &[start, end] in &overlay.edges {
            gizmos.line(start, end, AQUAMARINE);
        }
    }
}

fn fly_camera(
    mut camera: Single<(&mut Transform, &FlyCamera)>,
    keys: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    time: Res<Time>,
) {
    let (transform, fly) = &mut *camera;
    if keys.just_pressed(KeyCode::Home) {
        **transform = fly.overview;
    }
    if mouse.pressed(MouseButton::Right) && motion.delta != Vec2::ZERO {
        let (yaw, pitch, _) = transform.rotation.to_euler(EulerRot::YXZ);
        let yaw = yaw - motion.delta.x * 0.004;
        let pitch = (pitch - motion.delta.y * 0.004).clamp(-1.55, 1.55);
        transform.rotation = Quat::from_euler(EulerRot::YXZ, yaw, pitch, 0.0);
    }
    let axis = |positive, negative| {
        (i32::from(keys.pressed(positive)) - i32::from(keys.pressed(negative))) as f32
    };
    let direction = *transform.forward() * axis(KeyCode::KeyW, KeyCode::KeyS)
        + *transform.right() * axis(KeyCode::KeyD, KeyCode::KeyA)
        + Vec3::Y * axis(KeyCode::KeyE, KeyCode::KeyQ);
    transform.translation += direction.normalize_or_zero() * fly.speed * time.delta_secs();
}

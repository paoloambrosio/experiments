use std::collections::{HashMap, HashSet};

use bevy::{
    asset::RenderAssetUsages,
    color::palettes::css::AQUAMARINE,
    gizmos::config::{DefaultGizmoConfigGroup, GizmoConfigStore},
    image::{ImageAddressMode, ImageSampler, ImageSamplerDescriptor},
    input::mouse::AccumulatedMouseMotion,
    mesh::{Indices, PrimitiveTopology},
    prelude::*,
    render::render_resource::{Extent3d, Face, TextureDimension, TextureFormat},
};
use vehicle_assets::{
    DiffuseTexture, Heightfield, MeshId, TerrainAsset, TriangleMesh, ror::TerrainImport,
};

#[derive(Resource)]
struct Source(TerrainImport);

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

pub fn run(terrain: TerrainImport) -> bool {
    let title = terrain.terrain.name.clone();
    App::new()
        .insert_resource(Source(terrain))
        .insert_resource(ClearColor(Color::srgb(0.37, 0.49, 0.60)))
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: format!("{title} | Terrain Viewer"),
                ..default()
            }),
            ..default()
        }))
        .add_systems(Startup, setup)
        .add_systems(Update, (fly_camera, collision_overlay))
        .run()
        .is_success()
}

fn setup(
    mut commands: Commands,
    source: Res<Source>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
    mut gizmo_config: ResMut<GizmoConfigStore>,
) {
    let TerrainImport {
        terrain,
        appearance,
        ..
    } = &source.0;
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
                ..default()
            })
        })
        .collect();

    let mut rendered = HashMap::new();
    for instance in &terrain.objects {
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
            let mesh = &terrain.meshes[id.0];
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
            commands.spawn((
                Name::new(instance.name.clone()),
                Mesh3d(mesh.clone()),
                MeshMaterial3d(material.clone()),
                Transform {
                    translation: instance.translation,
                    rotation: instance.rotation,
                    scale: instance.scale,
                },
            ));
        }
    }
    if let Some(grid) = &terrain.heightfield {
        let terrain_mesh = triangulate(grid);
        commands.spawn((
            Name::new("Heightfield"),
            Mesh3d(meshes.add(to_bevy_mesh(&terrain_mesh, &terrain_mesh.triangles, false))),
            MeshMaterial3d(materials.add(Color::srgb(0.35, 0.39, 0.34))),
        ));
    }

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
    commands.insert_resource(GlobalAmbientLight {
        brightness: 700.0,
        ..default()
    });
    commands.spawn((
        DirectionalLight {
            illuminance: 9300.0,
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
    commands.remove_resource::<Source>();
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

fn triangulate(grid: &Heightfield) -> TriangleMesh {
    let vertices = (0..grid.samples_z)
        .flat_map(|z| {
            (0..grid.samples_x)
                .map(move |x| grid.sample_position(x, z).expect("validated heightfield"))
        })
        .collect();
    let mut triangles = Vec::new();
    for z in 0..grid.samples_z - 1 {
        for x in 0..grid.samples_x - 1 {
            let a = (z * grid.samples_x + x) as u32;
            let b = a + 1;
            let c = a + grid.samples_x as u32;
            let d = c + 1;
            // OGRE alternates the diagonal on consecutive terrain rows.
            if z.is_multiple_of(2) {
                triangles.extend([[a, c, d], [a, d, b]]);
            } else {
                triangles.extend([[a, c, b], [b, c, d]]);
            }
        }
    }
    TriangleMesh {
        vertices,
        triangles,
        normals: None,
        uvs: None,
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

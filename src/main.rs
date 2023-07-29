use bevy::prelude::*;

mod scene;
mod kn5_loader;
mod scene_printer;
mod scene_renderer;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        //.add_systems(Startup, print_scene)
        .add_systems(Startup, add_kn5_model)
        .run();

}

fn print_scene() {
    let scene = kn5_loader::load_kn5_file("abarth500_D.kn5").unwrap();
    scene_printer::print_scene(scene);
}

fn add_kn5_model(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let scene = kn5_loader::load_kn5_file("abarth500_D.kn5").unwrap();
    let mesh = scene_renderer::to_mesh(scene);

    commands.spawn(PbrBundle {
        mesh: meshes.add(mesh),
        material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
        ..default()
    });

    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });

    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(-2.0, 2.5, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });
}
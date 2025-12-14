use bevy::prelude::*;

// Main function for both desktop and Android
pub fn run() {
    App::new()
        // añade los plugins de realidad virtual
        .add_plugins(bevy_mod_openxr::add_xr_plugins(DefaultPlugins))
        .add_systems(Startup, setup_scene)
        .run();
}

// Android entry point
#[bevy_main]
fn main() {
    run();
}

/// Creates a simple scene with a cube and a camera
fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Spawn a single cube
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.8, 0.7, 0.6))),
        Transform::from_xyz(0.0, 0.5, 0.0),
    ));

    // Spawn a single light source
    commands.spawn((
        PointLight {
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0),
    ));
}
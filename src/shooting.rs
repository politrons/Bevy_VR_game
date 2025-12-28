use bevy::prelude::*;
use bevy_xr_utils::actions::XRUtilsActionState;

use crate::controller::{RightController, RightControllerAim};
use crate::player::ShootAction;

/// Visual assets for bullets (shared mesh/material).
#[derive(Resource, Clone)]
pub struct BulletAssets {
    pub mesh: Handle<Mesh>,
    pub material: Handle<StandardMaterial>,
}

/// Tracks trigger edge detection so we spawn one bullet per press.
#[derive(Resource, Default)]
pub struct BulletFireState {
    pub prev_pressed: bool,
}

/// Simple projectile component that moves forward and despawns after a distance.
#[derive(Component)]
pub struct Bullet {
    pub velocity: Vec3,
    pub remaining_m: f32,
}

const BULLET_RADIUS_M: f32 = 0.05;
const BULLET_SPEED_MPS: f32 = 18.0;
const BULLET_MAX_DISTANCE_M: f32 = 40.0;
const BULLET_SPAWN_OFFSET_M: f32 = 0.12;
const BULLET_PITCH_DOWN_DEG: f32 = 6.0;
const TRIGGER_FIRE_THRESHOLD: f32 = 0.6;

/// Creates the shared bullet mesh/material.
pub(crate) fn setup_bullet_assets(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mesh = meshes.add(Sphere::new(BULLET_RADIUS_M));
    let material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.98, 0.92, 0.20),
        emissive: Color::srgb(0.40, 0.35, 0.06).into(),
        unlit: true,
        ..default()
    });

    commands.insert_resource(BulletAssets { mesh, material });
}

/// Spawns a bullet from the right controller when the trigger is pressed.
pub(crate) fn spawn_bullets(
    mut commands: Commands,
    actions: Query<&XRUtilsActionState, With<ShootAction>>,
    right_aim: Query<&GlobalTransform, With<RightControllerAim>>,
    right_controller: Query<&GlobalTransform, With<RightController>>,
    assets: Res<BulletAssets>,
    mut fire_state: ResMut<BulletFireState>,
) {
    let pressed = actions
        .iter()
        .find_map(|s| match s {
            XRUtilsActionState::Float(v) => Some(v.current_state > TRIGGER_FIRE_THRESHOLD),
            XRUtilsActionState::Bool(b) => Some(b.current_state),
            _ => None,
        })
        .unwrap_or(false);

    let pressed_edge = pressed && !fire_state.prev_pressed;
    fire_state.prev_pressed = pressed;
    if !pressed_edge {
        return;
    }

    let right_gt = match right_aim.single() {
        Ok(gt) => gt,
        Err(_) => match right_controller.single() {
            Ok(gt) => gt,
            Err(_) => return,
        },
    };
    let right_t = right_gt.compute_transform();
    let forward = (right_t.rotation * Vec3::NEG_Z).normalize_or_zero();
    if forward.length_squared() <= f32::EPSILON {
        return;
    }
    let right_axis = (right_t.rotation * Vec3::X).normalize_or_zero();
    let pitch = Quat::from_axis_angle(right_axis, -BULLET_PITCH_DOWN_DEG.to_radians());
    let dir = (pitch * forward).normalize_or_zero();

    let spawn_pos = right_t.translation + dir * BULLET_SPAWN_OFFSET_M;

    commands.spawn((
        Bullet {
            velocity: dir * BULLET_SPEED_MPS,
            remaining_m: BULLET_MAX_DISTANCE_M,
        },
        Mesh3d(assets.mesh.clone()),
        MeshMaterial3d(assets.material.clone()),
        Transform::from_translation(spawn_pos),
        Name::new("Bullet"),
    ));
}

/// Moves bullets forward and despawns them once they travel far enough.
pub(crate) fn move_bullets(
    mut commands: Commands,
    time: Res<Time>,
    mut bullets: Query<(Entity, &mut Transform, &mut Bullet)>,
) {
    let dt = time.delta_secs();
    for (entity, mut transform, mut bullet) in &mut bullets {
        let step = bullet.velocity * dt;
        transform.translation += step;
        bullet.remaining_m -= step.length();
        if bullet.remaining_m <= 0.0 {
            commands.entity(entity).despawn();
        }
    }
}

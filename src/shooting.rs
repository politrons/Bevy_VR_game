use bevy::prelude::*;
use bevy_xr_utils::actions::XRUtilsActionState;

use crate::controller::{RightController, RightControllerAim};
use crate::player::ShootAction;
use crate::ramp::{MonsterHitbox, MonsterMode, MonsterState};

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

const BULLET_RADIUS_M: f32 = 0.0245;
const BULLET_LENGTH_M: f32 = 0.14;
const BULLET_SPEED_MPS: f32 = 18.0;
const BULLET_MAX_DISTANCE_M: f32 = 40.0;
const BULLET_SPAWN_OFFSET_M: f32 = 0.20;
const BULLET_PITCH_DOWN_DEG: f32 = 32.0;
const BULLET_YAW_LEFT_DEG: f32 = 35.0;
const TRIGGER_FIRE_THRESHOLD: f32 = 0.6;

/// Creates the shared bullet mesh/material.
pub(crate) fn setup_bullet_assets(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mesh = meshes.add(Capsule3d::new(BULLET_RADIUS_M, BULLET_LENGTH_M));
    let material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.95, 0.10, 0.10),
        emissive: Color::srgb(0.35, 0.05, 0.05).into(),
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
    let up_axis = (right_t.rotation * Vec3::Y).normalize_or_zero();
    let yaw = Quat::from_axis_angle(up_axis, BULLET_YAW_LEFT_DEG.to_radians());
    let pitch = Quat::from_axis_angle(right_axis, -BULLET_PITCH_DOWN_DEG.to_radians());
    let dir = (pitch * yaw * forward).normalize_or_zero();

    let spawn_pos = right_t.translation + dir * (BULLET_SPAWN_OFFSET_M + BULLET_LENGTH_M * 0.5);

    let rotation = Quat::from_rotation_arc(Vec3::Y, dir);
    commands.spawn((
        Bullet {
            velocity: dir * BULLET_SPEED_MPS,
            remaining_m: BULLET_MAX_DISTANCE_M,
        },
        Mesh3d(assets.mesh.clone()),
        MeshMaterial3d(assets.material.clone()),
        Transform {
            translation: spawn_pos,
            rotation,
            ..default()
        },
        Name::new("Bullet"),
    ));
}

/// Moves bullets forward and despawns them once they travel far enough.
pub(crate) fn move_bullets(
    mut commands: Commands,
    time: Res<Time>,
    mut bullets: Query<(Entity, &mut Transform, &mut Bullet)>,
    mut monsters: Query<(Entity, &GlobalTransform, &mut MonsterState, &MonsterHitbox), Without<Bullet>>,
) {
    let dt = time.delta_secs();
    for (entity, mut transform, mut bullet) in &mut bullets {
        let step = bullet.velocity * dt;
        let start = transform.translation;
        let end = start + step;

        let mut hit_entity = None;
        let mut hit_t = f32::INFINITY;
        for (monster_entity, monster_transform, monster_state, hitbox) in &mut monsters {
            if monster_state.mode == MonsterMode::Dead {
                continue;
            }
            let center = monster_transform.translation();
            let expanded = hitbox.half_extents + Vec3::splat(BULLET_RADIUS_M);
            let min = center - expanded;
            let max = center + expanded;
            if let Some(t) = segment_aabb_entry(start, end, min, max) {
                if t < hit_t {
                    hit_t = t;
                    hit_entity = Some(monster_entity);
                }
            }
        }

        if let Some(monster_entity) = hit_entity {
            transform.translation = start + (end - start) * hit_t;
            if let Ok((_, _, mut monster_state, _)) = monsters.get_mut(monster_entity) {
                monster_state.mode = MonsterMode::Dead;
                monster_state.despawn_timer.reset();
                monster_state.just_switched = true;
            }
            commands.entity(entity).despawn();
            continue;
        }

        transform.translation = end;
        bullet.remaining_m -= step.length();
        if bullet.remaining_m <= 0.0 {
            commands.entity(entity).despawn();
        }
    }
}

fn segment_aabb_entry(start: Vec3, end: Vec3, min: Vec3, max: Vec3) -> Option<f32> {
    let dir = end - start;
    let mut tmin: f32 = 0.0;
    let mut tmax: f32 = 1.0;
    let axes = [
        (start.x, dir.x, min.x, max.x),
        (start.y, dir.y, min.y, max.y),
        (start.z, dir.z, min.z, max.z),
    ];
    for (s, d, min_a, max_a) in axes {
        if d.abs() < 1.0e-6 {
            if s < min_a || s > max_a {
                return None;
            }
            continue;
        }
        let inv = 1.0 / d;
        let mut t1 = (min_a - s) * inv;
        let mut t2 = (max_a - s) * inv;
        if t1 > t2 {
            std::mem::swap(&mut t1, &mut t2);
        }
        tmin = tmin.max(t1);
        tmax = tmax.min(t2);
        if tmin > tmax {
            return None;
        }
    }
    if tmax < 0.0 || tmin > 1.0 {
        return None;
    }
    Some(tmin.max(0.0))
}

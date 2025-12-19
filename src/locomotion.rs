use bevy::prelude::*;
use bevy_mod_openxr::resources::OxrViews;
use bevy_mod_xr::session::XrTrackingRoot;
use bevy_xr_utils::actions::XRUtilsActionState;

use crate::scene::FloorParams;

// -------------------------
// Locomotion components/resources
// -------------------------

#[derive(Component)]
pub(crate) struct MoveAction;

#[derive(Component)]
pub(crate) struct TurnAction;

#[derive(Component)]
pub(crate) struct JumpAction;

#[derive(Component, Debug, Clone, Copy)]
pub(crate) struct MovingPlatform {
    pub(crate) half_extents: Vec2, // x and z half sizes
    pub(crate) thickness: f32,
    pub(crate) z_min: f32,
    pub(crate) z_max: f32,
    pub(crate) speed_mps: f32,
    pub(crate) dir: f32, // +1 or -1 along Z
}

#[derive(Resource, Debug, Clone, Copy)]
pub(crate) struct LocomotionSettings {
    pub(crate) move_speed_mps: f32,
    pub(crate) turn_speed_rad_s: f32,
    pub(crate) stick_deadzone: f32,
    pub(crate) jump_velocity_mps: f32,
    pub(crate) gravity_mps2: f32,
}

impl Default for LocomotionSettings {
    fn default() -> Self {
        Self {
            move_speed_mps: 2.0,
            turn_speed_rad_s: 120.0_f32.to_radians(),
            stick_deadzone: 0.15,
            jump_velocity_mps: 3.5,
            gravity_mps2: -9.81,
        }
    }
}

#[derive(Resource, Debug, Default)]
pub(crate) struct PlayerKinematics {
    pub(crate) vertical_velocity: f32,
}

pub(crate) fn handle_locomotion(
    move_query: Query<&XRUtilsActionState, With<MoveAction>>,
    turn_query: Query<&XRUtilsActionState, With<TurnAction>>,
    jump_query: Query<&XRUtilsActionState, With<JumpAction>>,
    mut xr_root: Query<&mut Transform, (With<XrTrackingRoot>, Without<MovingPlatform>)>,
    time: Res<Time>,
    views: Res<OxrViews>,
    settings: Res<LocomotionSettings>,
    road: Res<FloorParams>,
    platforms: Query<(&Transform, &MovingPlatform), Without<XrTrackingRoot>>,
    mut kin: ResMut<PlayerKinematics>,
) {
    let Ok(mut root) = xr_root.single_mut() else {
        return;
    };

    let dt = time.delta_secs();

    let move_xy = move_query
        .iter()
        .find_map(|s| match s {
            XRUtilsActionState::Vector(v) => Some(v.current_state),
            _ => None,
        })
        .unwrap_or([0.0, 0.0]);

    let turn_xy = turn_query
        .iter()
        .find_map(|s| match s {
            XRUtilsActionState::Vector(v) => Some(v.current_state),
            _ => None,
        })
        .unwrap_or([0.0, 0.0]);

    let jump_pressed_edge = jump_query
        .iter()
        .find_map(|s| match s {
            XRUtilsActionState::Bool(b) => Some(b.current_state && b.changed_since_last_sync),
            _ => None,
        })
        .unwrap_or(false);

    let Some((head_yaw, pivot_local)) = head_yaw_and_pivot(&views) else {
        return;
    };

    let turn_x = apply_deadzone(turn_xy[0], settings.stick_deadzone);
    if turn_x.abs() > 0.0 {
        let delta_yaw = -turn_x * settings.turn_speed_rad_s * dt;

        let old_rot = root.rotation;
        let delta_rot = Quat::from_rotation_y(delta_yaw);
        let new_rot = delta_rot * old_rot;

        let world_pivot_before = old_rot.mul_vec3(pivot_local);
        let world_pivot_after = new_rot.mul_vec3(pivot_local);
        root.translation += world_pivot_before - world_pivot_after;

        root.rotation = new_rot;
    }

    let move_x = apply_deadzone(move_xy[0], settings.stick_deadzone);
    let move_y = apply_deadzone(move_xy[1], settings.stick_deadzone);
    let move_input = Vec3::new(move_x, 0.0, -move_y);

    if move_input.length_squared() > 0.0 {
        let world_yaw = root.rotation * head_yaw;
        root.translation += world_yaw.mul_vec3(move_input) * settings.move_speed_mps * dt;
    }

    let (ground_y, ground_vel) = ground_y_and_velocity(root.translation, &road, &platforms);
    let grounded = (root.translation.y - ground_y).abs() <= 0.05;

    root.translation += ground_vel * dt;

    if jump_pressed_edge && grounded {
        kin.vertical_velocity = settings.jump_velocity_mps;
    }

    if !grounded || kin.vertical_velocity > 0.0 {
        kin.vertical_velocity += settings.gravity_mps2 * dt;
        root.translation.y += kin.vertical_velocity * dt;

        if root.translation.y <= ground_y {
            root.translation.y = ground_y;
            kin.vertical_velocity = 0.0;
        }
    }
}

pub(crate) fn move_platforms(mut q: Query<(&mut Transform, &mut MovingPlatform)>, time: Res<Time>) {
    let dt = time.delta_secs();

    for (mut t, mut p) in &mut q {
        t.translation.z += p.dir * p.speed_mps * dt;

        if t.translation.z > p.z_max {
            t.translation.z = p.z_max;
            p.dir = -1.0;
        } else if t.translation.z < p.z_min {
            t.translation.z = p.z_min;
            p.dir = 1.0;
        }
    }
}

fn ground_y_and_velocity(
    pos: Vec3,
    road: &FloorParams,
    platforms: &Query<(&Transform, &MovingPlatform), Without<XrTrackingRoot>>,
) -> (f32, Vec3) {
    let mut best_y = road.ground_y_at(pos);
    let mut best_vel = Vec3::ZERO;

    for (t, p) in platforms.iter() {
        let center = t.translation;
        let top_y = center.y + p.thickness * 0.5;

        let dx = (pos.x - center.x).abs();
        let dz = (pos.z - center.z).abs();
        if dx <= p.half_extents.x && dz <= p.half_extents.y {
            if top_y > best_y {
                best_y = top_y;
                best_vel = Vec3::new(0.0, 0.0, p.dir * p.speed_mps);
            }
        }
    }

    (best_y, best_vel)
}

fn head_yaw_and_pivot(views: &OxrViews) -> Option<(Quat, Vec3)> {
    if views.is_empty() {
        return None;
    }

    let q = views[0].pose.orientation;
    let q = Quat::from_xyzw(q.x, q.y, q.z, q.w);
    let (y, _, _) = q.to_euler(EulerRot::YXZ);
    let yaw = Quat::from_rotation_y(y);

    let mut sum = Vec3::ZERO;
    let mut n = 0.0;
    for v in views.iter().take(2) {
        sum += Vec3::new(v.pose.position.x, v.pose.position.y, v.pose.position.z);
        n += 1.0;
    }

    let avg = if n > 0.0 { sum / n } else { Vec3::ZERO };
    let pivot = Vec3::new(avg.x, 0.0, avg.z);

    Some((yaw, pivot))
}

fn apply_deadzone(v: f32, deadzone: f32) -> f32 {
    if v.abs() < deadzone { 0.0 } else { v }
}
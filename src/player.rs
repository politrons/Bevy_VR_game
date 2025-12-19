use bevy::prelude::*;
use bevy_mod_openxr::resources::OxrViews;
use bevy_mod_xr::session::XrTrackingRoot;
use bevy_xr_utils::actions::XRUtilsActionState;

use crate::ramp::MovingRamp;
use crate::scene::{FloorParams, PlayerSpawn};

// -------------------------
// XR input marker components
// -------------------------

/// Marker component attached to the XR action entity that provides the left-stick (move) input.
///
/// The system reads [`XRUtilsActionState::Vector`] from this entity.
#[derive(Component)]
pub struct MoveAction;

/// Marker component attached to the XR action entity that provides the right-stick (turn) input.
///
/// The system reads [`XRUtilsActionState::Vector`] from this entity.
#[derive(Component)]
pub struct TurnAction;

/// Marker component attached to the XR action entity that provides the jump button input.
///
/// The system reads [`XRUtilsActionState::Bool`] and uses edge-detection to trigger a jump.
#[derive(Component)]
pub struct JumpAction;

// -------------------------
// Locomotion resources
// -------------------------

/// Tunable locomotion parameters (speed, turn rate, deadzone, jump impulse, gravity).
///
/// These are applied in [`handle_player`] every frame.
#[derive(Resource, Debug, Clone, Copy)]
pub struct PlayerSettings {
    pub move_speed_mps: f32,
    pub turn_speed_rad_s: f32,
    pub stick_deadzone: f32,
    pub jump_velocity_mps: f32,
    pub gravity_mps2: f32,
}

impl Default for PlayerSettings {
    fn default() -> Self {
        Self {
            // Faster so longer gaps are jumpable.
            move_speed_mps: 3.2,
            turn_speed_rad_s: 120.0_f32.to_radians(),
            stick_deadzone: 0.15,
            // Higher jump so vertical steps feel fair.
            jump_velocity_mps: 4.6,
            gravity_mps2: -9.81,
        }
    }
}

/// Minimal player physics state kept across frames.
///
/// Currently only tracks vertical velocity for jump + gravity integration.
#[derive(Resource, Debug, Default)]
pub struct PlayerKinematics {
    pub vertical_velocity: f32,
}

/// Tracks whether the player has already stepped onto a ramp.
///
/// Once the player has been on a ramp, touching the main floor again is considered a failure
/// and will reset the player to the spawn point.
#[derive(Resource, Debug, Default)]
pub struct PlayerProgress {
    pub has_touched_ramp: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GroundKind {
    Road,
    Ramp,
}

// -------------------------
// Locomotion system
// -------------------------

/// Main player locomotion system.
///
/// - Reads XR input (move/turn/jump)
/// - Turns around the current head pivot (prevents orbiting)
/// - Moves in the direction the user is looking
/// - Applies jump + gravity
/// - Computes ground height from the road and from ramps
/// - Carries the player by the underlying ramp velocity when grounded
/// - If the player has touched a ramp and later touches the road, resets the run
pub fn handle_player(
    move_query: Query<&XRUtilsActionState, With<MoveAction>>,
    turn_query: Query<&XRUtilsActionState, With<TurnAction>>,
    jump_query: Query<&XRUtilsActionState, With<JumpAction>>,
    mut xr_root: Query<&mut Transform, (With<XrTrackingRoot>, Without<MovingRamp>)>,
    time: Res<Time>,
    views: Option<Res<OxrViews>>,
    settings: Res<PlayerSettings>,
    spawn: Option<Res<PlayerSpawn>>,
    mut progress: ResMut<PlayerProgress>,
    road: Option<Res<FloorParams>>,
    ramps: Query<(&Transform, &MovingRamp), Without<XrTrackingRoot>>,
    mut kin: ResMut<PlayerKinematics>,
) {
    let Ok(mut root) = xr_root.single_mut() else {
        return;
    };

    let Some(views) = views else {
        return;
    };
    let Some(spawn) = spawn else {
        return;
    };
    let Some(road) = road else {
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
            // We treat a "changed" true as an edge-trigger (button down).
            XRUtilsActionState::Bool(b) => Some(b.current_state && b.changed_since_last_sync),
            _ => None,
        })
        .unwrap_or(false);

    let Some((head_yaw, pivot_local)) = head_yaw_and_pivot(&views) else {
        return;
    };

    // Turn around the head pivot (prevents "orbiting" around world origin).
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

    // Move in the direction the head is facing.
    let move_x = apply_deadzone(move_xy[0], settings.stick_deadzone);
    let move_y = apply_deadzone(move_xy[1], settings.stick_deadzone);
    let move_input = Vec3::new(move_x, 0.0, -move_y);

    if move_input.length_squared() > 0.0 {
        let world_yaw = root.rotation * head_yaw;
        root.translation += world_yaw.mul_vec3(move_input) * settings.move_speed_mps * dt;
    }

    // Grounding (road vs ramps).
    let (ground_y, ground_vel, kind) = ground_y_and_velocity(root.translation, &road, &ramps);
    let grounded = (root.translation.y - ground_y).abs() <= 0.05;

    // Carry the player by the motion of the surface they are standing on.
    root.translation += ground_vel * dt;

    // Track progress: once we step onto any ramp, landing on the road again resets the run.
    if kind == GroundKind::Ramp {
        progress.has_touched_ramp = true;
    }

    if grounded && kind == GroundKind::Road && progress.has_touched_ramp {
        // Failure condition: fell off ramps and touched the main floor again.
        root.translation = spawn.pos;
        root.rotation = spawn.rot;
        kin.vertical_velocity = 0.0;
        progress.has_touched_ramp = false;
        return;
    }

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

fn ground_y_and_velocity(
    pos: Vec3,
    road: &FloorParams,
    ramps: &Query<(&Transform, &MovingRamp), Without<XrTrackingRoot>>,
) -> (f32, Vec3, GroundKind) {
    let mut best_y = road.ground_y_at(pos);
    let mut best_vel = Vec3::ZERO;
    let mut best_kind = GroundKind::Road;

    for (t, r) in ramps.iter() {
        let center = t.translation;

        // These fields must exist in `MovingRamp` (defined in `ramp.rs`).
        let top_y = center.y + r.thickness * 0.5;

        let dx = (pos.x - center.x).abs();
        let dz = (pos.z - center.z).abs();
        if dx <= r.half_extents.x && dz <= r.half_extents.y {
            if top_y > best_y {
                best_y = top_y;
                best_vel = r.current_vel;
                best_kind = GroundKind::Ramp;
            }
        }
    }

    (best_y, best_vel, best_kind)
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
    if v.abs() < deadzone {
        0.0
    } else {
        v
    }
}
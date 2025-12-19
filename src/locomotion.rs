use bevy::prelude::*;
use bevy::ecs::schedule::common_conditions::resource_exists;
use bevy_mod_openxr::resources::OxrViews;
use bevy_mod_xr::session::XrTrackingRoot;
use bevy_xr_utils::actions::XRUtilsActionState;

use crate::scene::{FloorParams, PlayerSpawn};

// -------------------------
// Locomotion components/resources
// -------------------------

/// Marker component attached to the XR action entity that provides the left-stick (move) input.
///
/// The system reads [`XRUtilsActionState::Vector`] from this entity.
#[derive(Component)]
pub(crate) struct MoveAction;

/// Marker component attached to the XR action entity that provides the right-stick (turn) input.
///
/// The system reads [`XRUtilsActionState::Vector`] from this entity.
#[derive(Component)]
pub(crate) struct TurnAction;

/// Marker component attached to the XR action entity that provides the jump button input.
///
/// The system reads [`XRUtilsActionState::Bool`] and uses edge-detection to trigger a jump.
#[derive(Component)]
pub(crate) struct JumpAction;

/// Back-and-forth moving platform (legacy / optional).
#[derive(Component, Debug, Clone, Copy)]
pub(crate) struct MovingPlatform {
    pub(crate) half_extents: Vec2, // x and z half sizes
    pub(crate) thickness: f32,
    pub(crate) z_min: f32,
    pub(crate) z_max: f32,
    pub(crate) speed_mps: f32,
    pub(crate) dir: f32, // +1 or -1 along Z
}

/// A moving ramp that travels forward (Z) and also oscillates in X (left/right) and Y (up/down).
///
/// - Z movement provides the "conveyor" progression: ramps advance away from the player.
/// - X/Y oscillation makes the path feel dynamic.
/// - Ramps are despawned once they go past their configured end bound.
#[derive(Component, Debug, Clone, Copy)]
pub(crate) struct MovingRamp {
    /// Half-size footprint in X/Z used for simple standing checks.
    pub(crate) half_extents: Vec2,
    /// Visual thickness of the ramp mesh.
    pub(crate) thickness: f32,

    /// Base (center) position around which the ramp oscillates in X/Y.
    ///
    /// Note: `base_center.z` is treated as the *starting* Z, but the ramp will advance
    /// over time along Z.
    pub(crate) base_center: Vec3,

    /// Oscillation amplitude in X (meters).
    pub(crate) x_amp: f32,
    /// Oscillation amplitude in Y (meters).
    pub(crate) y_amp: f32,
    /// Oscillation frequency in X (radians/sec).
    pub(crate) x_freq: f32,
    /// Oscillation frequency in Y (radians/sec).
    pub(crate) y_freq: f32,
    /// Phase offset to desynchronize different ramps.
    pub(crate) phase: f32,

    /// Z travel bounds. The ramp will despawn once it goes past the end it is moving toward.
    pub(crate) z_min: f32,
    pub(crate) z_max: f32,
    /// Z speed in meters/second.
    pub(crate) z_speed_mps: f32,
    /// Z direction: `+1.0` or `-1.0`.
    pub(crate) z_dir: f32,

    /// Current instantaneous world velocity of the ramp (used to carry the player when grounded).
    pub(crate) current_vel: Vec3,
}

/// Tracks whether the player has already stepped onto a ramp.
///
/// Once the player has been on a ramp, touching the main floor again is considered a failure
/// and will reset the player to the start.
#[derive(Resource, Debug, Default)]
pub(crate) struct PlayerProgress {
    pub(crate) has_touched_ramp: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GroundKind {
    Road,
    Platform,
    Ramp,
}

/// Tunable locomotion parameters (speed, turn rate, deadzone, jump impulse, gravity).
///
/// These are applied in [`handle_locomotion`] every frame.
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

/// Minimal player physics state kept across frames.
///
/// Currently only tracks vertical velocity for jump + gravity integration.
#[derive(Resource, Debug, Default)]
pub(crate) struct PlayerKinematics {
    pub(crate) vertical_velocity: f32,
}

/// Main player locomotion system.
///
/// - Reads XR input (move/turn/jump)
/// - Applies smooth turning around the current head pivot
/// - Moves in the direction the user is looking
/// - Applies jump + gravity
/// - Computes ground height from the road and from moving surfaces (platforms/ramps)
/// - Carries the player by the underlying moving surface velocity when standing on it
/// - If the player falls off the ramps and lands back on the road, resets to the spawn point
pub(crate) fn handle_locomotion(
    move_query: Query<&XRUtilsActionState, With<MoveAction>>,
    turn_query: Query<&XRUtilsActionState, With<TurnAction>>,
    jump_query: Query<&XRUtilsActionState, With<JumpAction>>,
    mut xr_root: Query<&mut Transform, (With<XrTrackingRoot>, Without<MovingPlatform>)>,
    time: Res<Time>,
    views: Option<Res<OxrViews>>,
    settings: Res<LocomotionSettings>,
    spawn: Option<Res<PlayerSpawn>>,
    mut progress: ResMut<PlayerProgress>,
    road: Option<Res<FloorParams>>,
    platforms: Query<(&Transform, &MovingPlatform), Without<XrTrackingRoot>>,
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

    let (ground_y, ground_vel, kind) =
        ground_y_and_velocity(root.translation, &road, &platforms, &ramps);
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

/// Moves simple rectangular platforms along Z and despawns them when they reach the end.
///
/// This is useful for "conveyor" style gameplay: platforms do not bounce back, they disappear.
pub(crate) fn move_platforms(
    mut commands: Commands,
    mut q: Query<(Entity, &mut Transform, &MovingPlatform)>,
    time: Res<Time>,
) {
    let dt = time.delta_secs();

    for (e, mut t, p) in &mut q {
        t.translation.z += p.dir * p.speed_mps * dt;

        let past_end = (p.dir >= 0.0 && t.translation.z >= p.z_max)
            || (p.dir < 0.0 && t.translation.z <= p.z_min);

        if past_end {
            // Once a platform reaches the end of its travel, remove it from the world.
            commands.entity(e).despawn();
        }
    }
}

/// Updates all [`MovingRamp`] entities by advancing them in Z and oscillating them in X/Y over time.
///
/// Also computes and stores the current instantaneous velocity in the component so the
/// locomotion system can carry the player when standing on a ramp.
///
/// Ramps do **not** bounce back: once they reach the end of the floor in the direction
/// they are moving, they are despawned.
pub(crate) fn move_ramps(
    mut commands: Commands,
    mut q: Query<(Entity, &mut Transform, &mut MovingRamp)>,
    time: Res<Time>,
) {
    let t = time.elapsed_secs();
    let dt = time.delta_secs();

    for (e, mut tr, mut r) in &mut q {
        // Advance in Z.
        tr.translation.z += r.z_dir * r.z_speed_mps * dt;

        let past_end = (r.z_dir >= 0.0 && tr.translation.z >= r.z_max)
            || (r.z_dir < 0.0 && tr.translation.z <= r.z_min);

        if past_end {
            commands.entity(e).despawn();
            continue;
        }

        // Oscillate in X/Y around the base position.
        let x = r.base_center.x + r.x_amp * (t * r.x_freq + r.phase).sin();
        let y = r.base_center.y + r.y_amp * (t * r.y_freq + r.phase * 1.37).sin();
        let z = tr.translation.z; // keep the advanced Z

        // Instantaneous velocity: derivative of the sine waves + constant Z velocity.
        let vx = r.x_amp * r.x_freq * (t * r.x_freq + r.phase).cos();
        let vy = r.y_amp * r.y_freq * (t * r.y_freq + r.phase * 1.37).cos();
        let vz = r.z_dir * r.z_speed_mps;
        r.current_vel = Vec3::new(vx, vy, vz);

        tr.translation = Vec3::new(x, y, z);
    }
}

fn ground_y_and_velocity(
    pos: Vec3,
    road: &FloorParams,
    platforms: &Query<(&Transform, &MovingPlatform), Without<XrTrackingRoot>>,
    ramps: &Query<(&Transform, &MovingRamp), Without<XrTrackingRoot>>,
) -> (f32, Vec3, GroundKind) {
    let mut best_y = road.ground_y_at(pos);
    let mut best_vel = Vec3::ZERO;
    let mut best_kind = GroundKind::Road;

    for (t, p) in platforms.iter() {
        let center = t.translation;
        let top_y = center.y + p.thickness * 0.5;

        let dx = (pos.x - center.x).abs();
        let dz = (pos.z - center.z).abs();
        if dx <= p.half_extents.x && dz <= p.half_extents.y {
            if top_y > best_y {
                best_y = top_y;
                best_vel = Vec3::new(0.0, 0.0, p.dir * p.speed_mps);
                best_kind = GroundKind::Platform;
            }
        }
    }

    for (t, r) in ramps.iter() {
        let center = t.translation;
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
    if v.abs() < deadzone { 0.0 } else { v }
}
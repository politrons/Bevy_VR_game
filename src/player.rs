use bevy::prelude::*;
use bevy_mod_openxr::resources::OxrViews;
use bevy_mod_xr::session::XrTrackingRoot;
use bevy_xr_utils::actions::XRUtilsActionState;


use crate::ramp::MovingRamp;
use crate::scene::{FloorParams, PlayerSpawn};

// -------------------------
// Ramp sliding tuning constants
// -------------------------

/// Extra acceleration multiplier applied from gravity when the player is standing on an inclined ramp.
///
/// - `1.0` means the pure physical projection.
/// - `1.30` means ramps feel ~30% more slippery (faster downhill, harder uphill).
const RAMP_SLIDE_ACCEL_MULT: f32 = 1.30;
/// Additional multiplier for UP ramps (positive slope) to make climbs more slippery.
const RAMP_SLIDE_UP_MULT: f32 = 2.0;

/// If the absolute ramp slope (dy/dz) is below this threshold, we treat the ramp as flat.
/// This prevents applying the "slippery" multiplier on nearly-flat surfaces.
const RAMP_SLOPE_EPS: f32 = 1.0e-4;

/// Per-second drag applied to ramp sliding velocity.
///
/// Higher = more friction, lower = more sliding.
///
/// We keep this slightly lower to make inclined ramps ~30% more slippery.
const RAMP_SLIDE_DRAG_PER_SEC: f32 = 0.46;

/// Small velocity threshold to snap ramp sliding velocity to zero (reduces tiny jitter).
const RAMP_SLIDE_VEL_EPS: f32 = 0.03;

// -------------------------
// Push tuning constants
// -------------------------

/// Speed multiplier applied while a push boost is active.
const PUSH_SPEED_MULT: f32 = 2.0;

/// Duration (seconds) of the push boost once triggered.
const PUSH_DURATION_S: f32 = 0.30;
/// Cooldown (seconds) before another push boost can trigger.
const PUSH_COOLDOWN_S: f32 = 1.0;


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

/// Marker component attached to the XR action entity that provides the sprint (grip) input.
///
/// The system reads [`XRUtilsActionState::Bool`] from this entity and, when pressed,
/// increases the movement speed by a fixed multiplier.
#[derive(Component)]
pub struct SprintAction;

/// Marker component attached to the XR action entity that provides the jump button input.
///
/// The system reads [`XRUtilsActionState::Bool`] and uses edge-detection to trigger a jump.
#[derive(Component)]
pub struct JumpAction;

/// Marker component attached to the XR action entity that provides the push (speed boost) input.
///
/// The system reads [`XRUtilsActionState::Bool`] and uses rising-edge detection to trigger a short boost.
#[derive(Component)]
pub struct PushAction;




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
            jump_velocity_mps: 5.98,
            gravity_mps2: -9.81,
        }
    }
}

/// Minimal player physics state kept across frames.
///
/// Currently tracks vertical velocity for jump + gravity integration,
/// and the previous frame's Y position for true landing detection.
#[derive(Resource, Debug, Default)]
pub struct PlayerKinematics {
    pub vertical_velocity: f32,
    /// Previous frame Y position of the tracking root. Used to detect true landings.
    pub prev_y: f32,
    /// Velocity along the ramp direction (world Z) used to simulate sliding on inclined ramps.
    pub ramp_velocity_z: f32,
    /// Last frame's jump button state. Used for robust edge-detection on all runtimes.
    pub prev_jump_pressed: bool,
    /// Last frame's push button state. Used for robust edge-detection on all runtimes.
    pub prev_push_pressed: bool,
    /// Remaining time (seconds) for the active push boost.
    pub push_time_left_s: f32,
    /// Remaining cooldown (seconds) before another push boost can trigger.
    pub push_cooldown_s: f32,
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
    push_query: Query<&XRUtilsActionState, With<PushAction>>,
    sprint_query: Query<&XRUtilsActionState, With<SprintAction>>,
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

    // Track previous frame Y for landing detection.
    let prev_y = kin.prev_y;
    kin.prev_y = root.translation.y;

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

    // We do our own rising-edge detection. Some runtimes can reset
    // `changed_since_last_sync` before gameplay systems run.
    let jump_pressed_now = jump_query
        .iter()
        .find_map(|s| match s {
            XRUtilsActionState::Bool(b) => Some(b.current_state),
            _ => None,
        })
        .unwrap_or(false);

    let jump_pressed_edge = jump_pressed_now && !kin.prev_jump_pressed;
    kin.prev_jump_pressed = jump_pressed_now;

    // We do our own rising-edge detection for push as well.
    let push_pressed_now = push_query
        .iter()
        .find_map(|s| match s {
            XRUtilsActionState::Bool(b) => Some(b.current_state),
            _ => None,
        })
        .unwrap_or(false);

    let push_pressed_edge = push_pressed_now && !kin.prev_push_pressed;
    kin.prev_push_pressed = push_pressed_now;

    // Start a short speed boost on push if the cooldown has expired.
    if push_pressed_edge && kin.push_cooldown_s <= 0.0 {
        kin.push_time_left_s = PUSH_DURATION_S;
        kin.push_cooldown_s = PUSH_COOLDOWN_S;
    }

    // Count down boost timer.
    if kin.push_time_left_s > 0.0 {
        kin.push_time_left_s = (kin.push_time_left_s - dt).max(0.0);
    }
    // Count down cooldown timer.
    if kin.push_cooldown_s > 0.0 {
        kin.push_cooldown_s = (kin.push_cooldown_s - dt).max(0.0);
    }

    let sprint_pressed = sprint_query
        .iter()
        .find_map(|s| match s {
            XRUtilsActionState::Bool(b) => Some(b.current_state),
            _ => None,
        })
        .unwrap_or(false);

    // Sprint is a sustained speed multiplier while the grip is held.
    let sprint_mul = if sprint_pressed { 1.3 } else { 1.0 };

    // Push is a short boost (edge-triggered) that temporarily increases movement speed.
    let push_mul = if kin.push_time_left_s > 0.0 { PUSH_SPEED_MULT } else { 1.0 };

    // Combine multipliers.
    let speed_mul = sprint_mul * push_mul;

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

    // Move input in the direction the head is facing.
    // IMPORTANT: we compute the desired direction now, but we apply it later once we know
    // whether we're on the road or on an inclined ramp.
    let move_x = apply_deadzone(move_xy[0], settings.stick_deadzone);
    let move_y = apply_deadzone(move_xy[1], settings.stick_deadzone);
    let move_input = Vec3::new(move_x, 0.0, -move_y);

    let desired_world_dir = if move_input.length_squared() > 0.0 {
        let world_yaw = root.rotation * head_yaw;
        world_yaw.mul_vec3(move_input).normalize_or_zero()
    } else {
        Vec3::ZERO
    };

    // Grounding (road vs ramps).
    // NOTE: We will compute ground twice:
    // - First, to know what surface we're currently on (for ramp sliding / surface carry).
    // - Second, after we move in X/Z, to resolve Y correctly at the *new* position.
    //   This avoids jitter where we move along Z but still snap Y to the old surface.
    let (_ground_y_before, ground_vel_before, kind_before, slope_dy_dz_before) =
        ground_y_and_velocity(root.translation, &road, &ramps);

    // Carry the player by the motion of the surface they are standing on.
    root.translation += ground_vel_before * dt;

    // Apply movement *after* we know the ground type.
    // - On the road: standard planar movement.
    // - On ramps: we simulate gravity along the ramp direction (world Z) so the player slides.
    //             Climbing requires forward stick input; descending ramps speed you up.
    if desired_world_dir.length_squared() > 0.0 {
        match kind_before {
            GroundKind::Road => {
                root.translation += desired_world_dir * (settings.move_speed_mps * speed_mul) * dt;
                // Leaving ramps: decay any leftover ramp velocity quickly.
                kin.ramp_velocity_z *= 0.6;
            }
            GroundKind::Ramp => {
                // Allow strafing freely in X.
                root.translation.x += desired_world_dir.x * (settings.move_speed_mps * speed_mul) * dt;

                // Player-controlled movement along Z (forward/back).
                let input_z = desired_world_dir.z;
                let input_z_speed = input_z * (settings.move_speed_mps * speed_mul);

                let m = slope_dy_dz_before;
                let slope_denom = 1.0 + m * m;

                // Gravity component along the ramp direction projected onto world Z.
                // Using slope dy/dz: a_z = g * (dy/dz) / (1 + (dy/dz)^2)
                // This points downhill automatically because gravity is negative.
                let mut a_z = if slope_denom > 1e-6 {
                    settings.gravity_mps2 * (m / slope_denom)
                } else {
                    0.0
                };

                // Make *inclined* ramps feel more slippery (30% more sliding).
                // We explicitly avoid applying it on nearly-flat ramps.
                if m.abs() > RAMP_SLOPE_EPS {
                    a_z *= RAMP_SLIDE_ACCEL_MULT;
                    if m > 0.0 {
                        a_z *= RAMP_SLIDE_UP_MULT;
                    }
                }

                // Apply drag in a frame-rate stable way.
                kin.ramp_velocity_z = (kin.ramp_velocity_z + a_z * dt) * (-RAMP_SLIDE_DRAG_PER_SEC * dt).exp();
                if kin.ramp_velocity_z.abs() < RAMP_SLIDE_VEL_EPS {
                    kin.ramp_velocity_z = 0.0;
                }

                // Total Z speed = input + slide.
                let z_total_speed = input_z_speed + kin.ramp_velocity_z;
                root.translation.z += z_total_speed * dt;
            }
        }
    } else {
        // No input: still slide on ramps, stand still on the road.
        if kind_before == GroundKind::Ramp {
            let m = slope_dy_dz_before;
            let slope_denom = 1.0 + m * m;

            // Gravity-driven sliding when the player is standing on a ramp without input.
            let mut a_z = if slope_denom > 1e-6 {
                settings.gravity_mps2 * (m / slope_denom)
            } else {
                0.0
            };

            // Make inclined ramps more slippery.
            if m.abs() > RAMP_SLOPE_EPS {
                a_z *= RAMP_SLIDE_ACCEL_MULT;
                if m > 0.0 {
                    a_z *= RAMP_SLIDE_UP_MULT;
                }
            }

            kin.ramp_velocity_z = (kin.ramp_velocity_z + a_z * dt) * (-RAMP_SLIDE_DRAG_PER_SEC * dt).exp();
            if kin.ramp_velocity_z.abs() < RAMP_SLIDE_VEL_EPS {
                kin.ramp_velocity_z = 0.0;
            }
            root.translation.z += kin.ramp_velocity_z * dt;
        } else {
            // On the road, ramp velocity should die out.
            kin.ramp_velocity_z *= 0.6;
        }
    }

    // Recompute ground at the *new* X/Z.
    let (ground_y, _ground_vel, kind, _slope_dy_dz) =
        ground_y_and_velocity(root.translation, &road, &ramps);

    // We only consider the player grounded when they are close to the surface *and*
    // they are not moving upward. This avoids counting side-swipes as a "landing".
    let snap_eps = 0.06;
    let mut grounded = (root.translation.y - ground_y).abs() <= snap_eps && kin.vertical_velocity <= 0.05;

    // Detect a true landing: coming from above and crossing onto the surface while descending.
    let landed_this_frame = kin.vertical_velocity <= 0.05
        && prev_y > ground_y + 0.08
        && root.translation.y <= ground_y + snap_eps;

    // Track progress: only mark ramp progress on a real landing.
    if kind == GroundKind::Ramp && landed_this_frame {
        progress.has_touched_ramp = true;
    }

    if grounded && kind == GroundKind::Road && progress.has_touched_ramp {
        // Failure condition: fell off ramps and touched the main floor again.
        root.translation = spawn.pos;
        root.rotation = spawn.rot;
        kin.vertical_velocity = 0.0;
        kin.prev_y = spawn.pos.y;
        kin.ramp_velocity_z = 0.0;
        // Reset button-edge state so the next press is detected correctly after respawn.
        kin.prev_jump_pressed = false;
        kin.prev_push_pressed = false;
        kin.push_time_left_s = 0.0;
        kin.push_cooldown_s = 0.0;
        progress.has_touched_ramp = false;
        return;
    }

    // IMPORTANT: if we jump this frame, we must treat the player as airborne immediately.
    // Otherwise the "grounded" branch below would snap Y to the surface and reset
    // `vertical_velocity` back to 0 in the same frame (making jumps appear broken).
    let jumped_this_frame = jump_pressed_edge && grounded;
    if jumped_this_frame {
        kin.vertical_velocity = settings.jump_velocity_mps;
        // Jumping cancels ramp sliding so we don't carry weird lateral speed into the air.
        kin.ramp_velocity_z = 0.0;
        grounded = false;
    }

    // Vertical integration + snap-to-surface.
    // If grounded, keep Y glued to the surface.
    // If airborne (or rising), apply gravity and clamp when we hit the surface.
    if grounded {
        root.translation.y = ground_y;
        kin.vertical_velocity = 0.0;
        kin.prev_y = ground_y;
        // Keep button-edge state consistent while grounded.
        // (No-op here, but explicit so future changes don't accidentally reset it.)
    } else {
        kin.vertical_velocity += settings.gravity_mps2 * dt;
        root.translation.y += kin.vertical_velocity * dt;

        if root.translation.y <= ground_y {
            root.translation.y = ground_y;
            kin.vertical_velocity = 0.0;
            kin.prev_y = ground_y;
        }
    }
}

/// Returns the surface Y, surface velocity, ground kind, and slope dy/dz under the player.
fn ground_y_and_velocity(
    pos: Vec3,
    road: &FloorParams,
    ramps: &Query<(&Transform, &MovingRamp), Without<XrTrackingRoot>>,
) -> (f32, Vec3, GroundKind, f32) {
    let mut best_y = road.ground_y_at(pos);
    let mut best_vel = Vec3::ZERO;
    let mut best_kind = GroundKind::Road;
    // Slope dy/dz of the best ramp under the player. 0.0 means flat.
    let mut best_slope_dy_dz = 0.0;

    for (t, r) in ramps.iter() {
        let center = t.translation;

        // Ramps are axis-aligned rectangles in X/Z, but may be inclined in Y along Z.
        // We compute the surface height under the player by interpolating along Z.
        let dx = (pos.x - center.x).abs();
        let dz_local = pos.z - center.z;
        let dz_abs = dz_local.abs();

        if dx <= r.half_extents.x && dz_abs <= r.half_extents.y {
            // Ask the ramp for the top surface height at this world Z.
            // Returns `None` if the ramp has no valid surface at that Z (e.g., degenerate).
            let Some(top_surface_y) = r.surface_top_y_at(pos.z) else {
                continue;
            };
            // dy/dz slope of the ramp surface (positive means it rises as Z increases).
            // Used to apply a gravity component along the ramp direction.
            let slope_dy_dz = r.surface_slope_dy_dz();

            if top_surface_y > best_y {
                best_y = top_surface_y;
                best_vel = r.current_vel;
                best_kind = GroundKind::Ramp;
                best_slope_dy_dz = slope_dy_dz;
            }
        }
    }

    (best_y, best_vel, best_kind, best_slope_dy_dz)
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

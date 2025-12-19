use bevy::prelude::*;
use bevy::pbr::MeshMaterial3d;
use bevy_mod_openxr::resources::OxrViews;
use bevy_mod_xr::session::XrTrackingRoot;
use bevy_xr_utils::actions::XRUtilsActionState;
use crate::scene::{FloorParams, PlayerSpawn};

// -------------------------
// XR input marker components
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

// -------------------------
// Moving ramps
// -------------------------

/// A moving ramp that travels forward (Z) only.
///
/// - Ramps advance away from the player along +Z.
/// - Ramps are despawned once they go past the end bound.
/// - `current_vel` is used to carry the player when grounded on a ramp.
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

// -------------------------
// Ramps: spawning (multi-lane, endless-ish)
// -------------------------

/// Shared render assets used by all moving ramps.
#[derive(Resource, Clone, Debug)]
pub(crate) struct RampRenderAssets {
    pub(crate) mesh: Handle<Mesh>,
    pub(crate) material: Handle<StandardMaterial>,
}

/// Configuration controlling how many ramps exist, how they are spaced, and their height limits.
#[derive(Resource, Clone, Debug)]
pub(crate) struct RampSpawnConfig {
    /// Number of parallel lanes across X.
    pub(crate) lanes: usize,
    /// Distance (meters) between lane centers.
    pub(crate) lane_spacing: f32,

    /// Ramp size (X width, Y thickness, Z depth).
    pub(crate) ramp_size: Vec3,

    /// Z spacing between consecutive ramps in a lane.
    pub(crate) z_gap: f32,
    /// How many ramps to pre-fill per lane on first run.
    pub(crate) initial_per_lane: usize,

    /// Max |delta-y| between consecutive ramps in a lane (jumpable vertical step).
    pub(crate) max_step_y: f32,
    /// Minimum and maximum ramp height above floor top.
    pub(crate) y_min_above_floor: f32,
    pub(crate) y_max_above_floor: f32,

    /// Random X offset inside each lane.
    pub(crate) lane_jitter_x: f32,

    /// Move speed (m/s) along Z.
    pub(crate) z_speed_mps: f32,
    /// How far (in Z) ramps can travel before despawning.
    pub(crate) travel_distance_z: f32,

    /// How often to spawn new ramps per lane.
    pub(crate) spawn_every_s: f32,
}

impl Default for RampSpawnConfig {
    fn default() -> Self {
        Self {
            // Wide floor: multiple lanes so there are alternative paths.
            lanes: 5,
            lane_spacing: 2.2,
            // Size tuned to feel jumpable.
            ramp_size: Vec3::new(2.0, 0.25, 2.0),
            // Distance between consecutive ramps per lane.
            z_gap: 5.0,
            // Pre-fill each lane so it looks populated from the start.
            initial_per_lane: 6,
            // Jump feasibility: limit vertical changes per step.
            max_step_y: 1.2,
            // Absolute height limits above the floor.
            y_min_above_floor: 0.9,
            y_max_above_floor: 5.0,
            // Small random jitter within each lane.
            lane_jitter_x: 0.35,
            // Conveyor speed.
            z_speed_mps: 2.5,
            // Past this distance, ramps despawn.
            travel_distance_z: 90.0,
            // Spawn cadence.
            spawn_every_s: 1.6,
        }
    }
}

/// Internal state for deterministic-ish random spawning.
#[derive(Resource, Clone, Debug)]
pub(crate) struct RampSpawnState {
    pub(crate) initialized: bool,
    pub(crate) seed: u64,
    pub(crate) lane_last_y: Vec<f32>,
    pub(crate) accum_s: f32,
}

impl Default for RampSpawnState {
    fn default() -> Self {
        Self {
            initialized: false,
            seed: 0xC0FFEE_u64,
            lane_last_y: Vec::new(),
            accum_s: 0.0,
        }
    }
}

fn rng_next_u32(seed: &mut u64) -> u32 {
    // xorshift64*
    *seed ^= *seed >> 12;
    *seed ^= *seed << 25;
    *seed ^= *seed >> 27;
    ((*seed).wrapping_mul(0x2545F4914F6CDD1D) >> 32) as u32
}

fn rng_f32_01(seed: &mut u64) -> f32 {
    let x = rng_next_u32(seed);
    (x & 0x00FF_FFFF) as f32 / 16_777_215.0
}

fn rng_f32_range(seed: &mut u64, min: f32, max: f32) -> f32 {
    min + (max - min) * rng_f32_01(seed)
}

/// Creates the shared ramp mesh/material and the spawn config/state.
///
/// Call this once at Startup so the resources exist before spawning.
pub(crate) fn setup_ramp_spawner(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let config = RampSpawnConfig::default();

    let ramp_mesh = meshes.add(Cuboid::new(
        config.ramp_size.x,
        config.ramp_size.y,
        config.ramp_size.z,
    ));
    let ramp_material = materials.add(StandardMaterial {
        // Different from floor on purpose.
        base_color: Color::srgb(0.90, 0.10, 0.60),
        perceptual_roughness: 0.9,
        metallic: 0.0,
        ..default()
    });

    commands.insert_resource(RampRenderAssets {
        mesh: ramp_mesh,
        material: ramp_material,
    });
    commands.insert_resource(config);
    commands.insert_resource(RampSpawnState::default());
}

fn lane_x_from_index(lane: usize, lanes: usize, spacing: f32) -> f32 {
    let center = (lanes as f32 - 1.0) * 0.5;
    (lane as f32 - center) * spacing
}

fn choose_lane_y(seed: &mut u64, y_min: f32, y_max: f32, max_step: f32, prev_y: f32) -> f32 {
    let dy = rng_f32_range(seed, -max_step, max_step);
    (prev_y + dy).clamp(y_min, y_max)
}

fn spawn_one_ramp(
    commands: &mut Commands,
    assets: &RampRenderAssets,
    pos: Vec3,
    config: &RampSpawnConfig,
    z_start: f32,
    z_end: f32,
) {
    let half_extents = Vec2::new(config.ramp_size.x * 0.5, config.ramp_size.z * 0.5);
    let thickness = config.ramp_size.y;

    commands.spawn((
        Mesh3d(assets.mesh.clone()),
        MeshMaterial3d(assets.material.clone()),
        Transform::from_translation(pos),
        GlobalTransform::default(),
        Visibility::default(),
        InheritedVisibility::default(),
        ViewVisibility::default(),
        MovingRamp {
            half_extents,
            thickness,
            base_center: pos,
            x_amp: 0.0,
            y_amp: 0.0,
            x_freq: 0.0,
            y_freq: 0.0,
            phase: 0.0,
            z_min: z_start,
            z_max: z_end + (config.ramp_size.z * 0.5) + 0.5,
            z_speed_mps: config.z_speed_mps,
            z_dir: 1.0,
            current_vel: Vec3::ZERO,
        },
    ));
}

/// Spawns many moving ramps across the wide floor.
///
/// - Creates multiple parallel lanes across X.
/// - Each lane is a random walk in height (Y), clamped to the configured min/max.
/// - Consecutive ramps differ by at most `max_step_y` so jumps stay possible.
/// - Ramps are moved/despawned by [`move_ramps`].
pub(crate) fn spawn_moving_ramps(
    mut commands: Commands,
    time: Res<Time>,
    assets: Res<RampRenderAssets>,
    config: Res<RampSpawnConfig>,
    mut state: ResMut<RampSpawnState>,
    floor: Res<FloorParams>,
) {
    let floor_y = floor.top_y;
    let y_min = floor_y + config.y_min_above_floor;
    let y_max = floor_y + config.y_max_above_floor;

    // Floor bounds.
    let x_min = floor.center.x - floor.half_extents.x;
    let x_max = floor.center.x + floor.half_extents.x;
    let z_start = floor.center.z - floor.half_extents.y;
    let z_end = floor.center.z + floor.half_extents.y;

    // Ramp footprint bounds (we only spawn if the whole ramp is above the floor).
    let ramp_half_x = config.ramp_size.x * 0.5;
    let ramp_half_z = config.ramp_size.z * 0.5;

    let x_spawn_min = x_min + ramp_half_x;
    let x_spawn_max = x_max - ramp_half_x;
    let z_spawn_min = z_start + ramp_half_z;
    let z_spawn_max = z_end - ramp_half_z;

    // If the floor is too small to fit ramps, do nothing.
    if x_spawn_min >= x_spawn_max || z_spawn_min >= z_spawn_max {
        return;
    }

    // Spawn at the beginning (but fully on the floor).
    let z_spawn = z_spawn_min;

    // One-time initialization + prefill across the whole floor.
    if !state.initialized {
        state.lane_last_y = vec![floor_y + 1.2; config.lanes];

        let available = (z_end - z_start).max(0.0);
        let max_count = ((available / config.z_gap).floor() as usize).saturating_add(1);
        let fill_per_lane = config.initial_per_lane.min(max_count).max(1);

        for lane in 0..config.lanes {
            let lane_center_x = lane_x_from_index(lane, config.lanes, config.lane_spacing);

            for i in 0..fill_per_lane {
                let z = z_spawn_min + (i as f32) * config.z_gap;
                if z > z_spawn_max {
                    break;
                }

                let mut x = lane_center_x
                    + rng_f32_range(&mut state.seed, -config.lane_jitter_x, config.lane_jitter_x);
                x = x.clamp(x_spawn_min, x_spawn_max);

                let prev_y = state.lane_last_y[lane];
                let y = choose_lane_y(&mut state.seed, y_min, y_max, config.max_step_y, prev_y);
                state.lane_last_y[lane] = y;

                spawn_one_ramp(
                    &mut commands,
                    &assets,
                    Vec3::new(x, y, z),
                    &config,
                    z_start,
                    z_end,
                );
            }
        }

        state.initialized = true;
        state.accum_s = 0.0;
        return;
    }

    // Spawn over time: always at the start edge.
    state.accum_s += time.delta_secs();
    if state.accum_s < config.spawn_every_s {
        return;
    }
    state.accum_s = 0.0;

    for lane in 0..config.lanes {
        let lane_center_x = lane_x_from_index(lane, config.lanes, config.lane_spacing);
        let mut x = lane_center_x
            + rng_f32_range(&mut state.seed, -config.lane_jitter_x, config.lane_jitter_x);
        x = x.clamp(x_spawn_min, x_spawn_max);

        let prev_y = state.lane_last_y[lane];
        let y = choose_lane_y(&mut state.seed, y_min, y_max, config.max_step_y, prev_y);
        state.lane_last_y[lane] = y;

        spawn_one_ramp(
            &mut commands,
            &assets,
            Vec3::new(x, y, z_spawn),
            &config,
            z_start,
            z_end,
        );
    }
}

// -------------------------
// Locomotion
// -------------------------

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
            move_speed_mps: 2.8,
            turn_speed_rad_s: 120.0_f32.to_radians(),
            stick_deadzone: 0.15,
            jump_velocity_mps: 4.2,
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
    Ramp,
}

/// Main player locomotion system.
///
/// - Reads XR input (move/turn/jump)
/// - Applies smooth turning around the current head pivot
/// - Moves in the direction the user is looking
/// - Applies jump + gravity
/// - Computes ground height from the road and from moving surfaces (ramps)
/// - Carries the player by the underlying moving surface velocity when standing on it
/// - If the player falls off the ramps and lands back on the road, resets to the spawn point
pub(crate) fn handle_locomotion(
    move_query: Query<&XRUtilsActionState, With<MoveAction>>,
    turn_query: Query<&XRUtilsActionState, With<TurnAction>>,
    jump_query: Query<&XRUtilsActionState, With<JumpAction>>,
    mut xr_root: Query<&mut Transform, (With<XrTrackingRoot>, Without<MovingRamp>)>,
    time: Res<Time>,
    views: Option<Res<OxrViews>>,
    settings: Res<LocomotionSettings>,
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

/// Updates all [`MovingRamp`] entities by advancing them forward in Z only.
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
    let dt = time.delta_secs();

    for (e, mut tr, mut r) in &mut q {
        let vz = r.z_speed_mps * r.z_dir;
        tr.translation.z += vz * dt;

        // Forward-only velocity.
        r.current_vel = Vec3::new(0.0, 0.0, vz);

        // Despawn once past the end.
        if tr.translation.z >= r.z_max {
            commands.entity(e).despawn();
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
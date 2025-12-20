use bevy::pbr::MeshMaterial3d;
use bevy::prelude::*;

use crate::scene::{FloorParams, FloorTopY};

/// Ramp slope direction.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RampSlopeDir {
    Up,
    Down,
}

/// Ramp profile used for spawning.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RampProfile {
    /// A flat platform (no incline).
    Flat,
    /// An inclined ramp segment.
    Inclined { dir: RampSlopeDir, angle_deg: f32 },
}

impl RampProfile {
    fn is_up(self) -> bool {
        matches!(
            self,
            RampProfile::Inclined {
                dir: RampSlopeDir::Up,
                ..
            }
        )
    }

    fn is_down(self) -> bool {
        matches!(
            self,
            RampProfile::Inclined {
                dir: RampSlopeDir::Down,
                ..
            }
        )
    }
}

/// Component attached to every moving ramp.
///
/// Ramps only move forward (+Z). X is decided at spawn time and never changes.
/// Y is represented by a *segment* (start/end) which can be flat or inclined.
#[derive(Component, Clone, Copy, Debug)]
pub struct MovingRamp {
    /// Half-extents in X/Z for simple footprint tests.
    pub half_extents: Vec2,
    /// Thickness in Y.
    pub thickness: f32,

    /// Forward speed in meters per second.
    pub z_speed_mps: f32,
    /// Minimum allowed Z before despawn.
    pub z_min: f32,
    /// Maximum allowed Z before despawn.
    pub z_max: f32,

    /// Current velocity of the ramp (world-space). Updated every frame.
    pub current_vel: Vec3,

    /// The ramp profile (flat or inclined).
    pub profile: RampProfile,

    /// Segment Z bounds in world space (used by player to evaluate surface height).
    pub segment_z_start: f32,
    pub segment_z_end: f32,

    /// Segment Y bounds in world space (start/end height of the top surface centerline).
    pub segment_y_start: f32,
    pub segment_y_end: f32,
}

impl MovingRamp {
    /// Returns the world-space Y of the *top surface* of this ramp at a given world-space Z.
    ///
    /// This is the helper the player uses to decide whether it is standing on the ramp.
    ///
    /// - For a [`RampProfile::Flat`] segment, the surface is constant.
    /// - For an inclined segment, the surface is linearly interpolated between `segment_y_start`
    ///   and `segment_y_end` across `[segment_z_start, segment_z_end]`.
    ///
    /// If `world_z` is outside the segment bounds, this returns `None`.
    pub fn surface_top_y_at(&self, world_z: f32) -> Option<f32> {
        if world_z < self.segment_z_start || world_z > self.segment_z_end {
            return None;
        }

        let len = self.segment_z_end - self.segment_z_start;
        if len.abs() <= f32::EPSILON {
            // Degenerate segment; treat as flat.
            return Some(self.segment_y_start + self.thickness * 0.5);
        }

        let t = (world_z - self.segment_z_start) / len;
        let centerline_y = self.segment_y_start + (self.segment_y_end - self.segment_y_start) * t;
        Some(centerline_y + self.thickness * 0.5)
    }

    /// Returns the world-space *slope* dy/dz of the ramp top surface.
    ///
    /// Positive means the surface goes up as Z increases.
    pub fn surface_slope_dy_dz(&self) -> f32 {
        let len = self.segment_z_end - self.segment_z_start;
        if len.abs() <= f32::EPSILON {
            return 0.0;
        }
        (self.segment_y_end - self.segment_y_start) / len
    }

    /// True if this ramp segment is an inclined UP ramp.
    pub fn is_inclined_up(&self) -> bool {
        matches!(
            self.profile,
            RampProfile::Inclined {
                dir: RampSlopeDir::Up,
                ..
            }
        )
    }

    /// True if this ramp segment is an inclined DOWN ramp.
    pub fn is_inclined_down(&self) -> bool {
        matches!(
            self.profile,
            RampProfile::Inclined {
                dir: RampSlopeDir::Down,
                ..
            }
        )
    }

    /// Returns the incline angle in degrees if this segment is inclined.
    pub fn incline_angle_deg(&self) -> Option<f32> {
        match self.profile {
            RampProfile::Inclined { angle_deg, .. } => Some(angle_deg),
            RampProfile::Flat => None,
        }
    }
}

/// Render assets shared by all ramps.
#[derive(Resource, Clone)]
pub struct RampRenderAssets {
    pub mesh: Handle<Mesh>,
    pub material: Handle<StandardMaterial>,
}

/// Configuration for ramp spawning.
#[derive(Resource, Clone, Debug)]
pub struct RampSpawnConfig {
    pub lanes: usize,

    /// Spacing between lanes in meters.
    pub lane_spacing_m: f32,

    /// Base ramp dimensions (X, Y, Z) in meters.
    ///
    /// - Flat ramps use this size as-is.
    /// - Inclined ramps are stretched along Z by [`Self::inclined_length_multiplier`].
    pub ramp_dimensions_m: Vec3,

    /// How much longer inclined ramps are compared to flat ones (Z axis multiplier).
    pub inclined_length_multiplier: f32,

    /// Distance between ramp spawns along Z in meters.
    pub z_spacing_m: f32,

    /// Random Z jitter applied per ramp at spawn time (in meters). Prevents perfect alignment.
    pub z_spawn_jitter_m: f32,

    /// Number of initial ramps to prefill per lane.
    pub initial_ramps_per_lane: usize,

    /// Minimum height above the floor in meters.
    pub min_height_above_floor_m: f32,

    /// Maximum height above the floor in meters.
    pub max_height_above_floor_m: f32,

    /// Random X jitter per lane in meters.
    pub lane_x_jitter_m: f32,

    /// Ramp speed in the Z direction in meters per second.
    pub ramp_speed_z_mps: f32,

    /// Interval between ramp spawns in seconds.
    pub spawn_interval_s: f32,

    /// Max allowed vertical difference between adjacent lanes for ramps spawned at the same Z step.
    pub max_vertical_step_cross_lane_m: f32,

    /// Base probability of spawning a flat segment in the mid-band.
    pub flat_probability_mid_band: f32,

    /// Y band (relative to floor top): below this we allow long UP streaks.
    pub low_band_y_m: f32,

    /// Y band (relative to floor top): above this we allow long DOWN streaks.
    pub high_band_y_m: f32,

    /// Allowed incline angles (degrees).
    ///
    /// We keep this conservative to avoid "sticky" feeling on steep slopes.
    pub allowed_incline_angles_deg: [f32; 2],
}

impl Default for RampSpawnConfig {
    fn default() -> Self {
        Self {
            lanes: 4, // Ramps per row (lanes)
            lane_spacing_m: 2.2,
            ramp_dimensions_m: Vec3::new(2.0, 0.25, 3.38),
            inclined_length_multiplier: 2.2,
            z_spacing_m: 5.0,
            z_spawn_jitter_m: 3.0,
            initial_ramps_per_lane: 6,
            min_height_above_floor_m: 1.0,
            max_height_above_floor_m: 20.0,
            lane_x_jitter_m: 0.35,
            ramp_speed_z_mps: 2.5,
            spawn_interval_s: 1.6 / 0.5, // Spawn more slowly (fewer ramps)
            max_vertical_step_cross_lane_m: 0.9,
            flat_probability_mid_band: 0.50,
            low_band_y_m: 3.0,
            high_band_y_m: 16.0,
            allowed_incline_angles_deg: [20.0, 30.0],
        }
    }
}

/// Internal state for the spawner.
#[derive(Resource, Clone, Debug)]
pub struct RampSpawnState {
    pub initialized: bool,
    pub accum_s: f32,
    pub seed: u32,

    /// Per-lane next "start height" for the next segment (relative to floor top).
    pub lane_next_y: Vec<f32>,

    /// Per-lane last profile (used to enforce combination rules).
    pub lane_last_profile: Vec<RampProfile>,

    /// Per-lane streak counters (how many ups/downs in a row).
    pub lane_up_streak: Vec<u32>,
    pub lane_down_streak: Vec<u32>,
}

impl Default for RampSpawnState {
    fn default() -> Self {
        Self {
            initialized: false,
            accum_s: 0.0,
            seed: 0xC0FFEE_u32,
            lane_next_y: Vec::new(),
            lane_last_profile: Vec::new(),
            lane_up_streak: Vec::new(),
            lane_down_streak: Vec::new(),
        }
    }
}

/// Setup the ramp spawner: creates the shared mesh/material and inserts config/state.
///
/// The mesh is a simple cuboid. For inclined ramps we rotate the transform.
pub(crate) fn setup_ramp_spawner(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let config = RampSpawnConfig::default();

    let ramp_mesh = meshes.add(Cuboid::new(
        config.ramp_dimensions_m.x,
        config.ramp_dimensions_m.y,
        config.ramp_dimensions_m.z,
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

/// Spawn ramps continuously.
///
/// Ramps always spawn fully on the floor (their footprint must fit inside the floor).
/// Spawning follows *path continuity* rules per lane:
/// - In the mid-band, we forbid UP->UP and DOWN->DOWN.
/// - Near the bottom, UP streaks are allowed (to climb).
/// - Near the top, DOWN streaks are allowed (to descend).
pub(crate) fn spawn_moving_ramps(
    mut commands: Commands,
    time: Res<Time>,
    floor: Res<FloorParams>,
    floor_top_y: Option<Res<FloorTopY>>,
    config: Res<RampSpawnConfig>,
    mut state: ResMut<RampSpawnState>,
    assets: Res<RampRenderAssets>,
) {
    let floor_top_y = floor_top_y.map(|r| r.0).unwrap_or(0.0);

    // Floor bounds.
    let x_min = floor.center.x - floor.half_extents.x;
    let x_max = floor.center.x + floor.half_extents.x;
    let z_start = floor.center.z - floor.half_extents.y;
    let z_end = floor.center.z + floor.half_extents.y;

    // Ramp footprint bounds (we only spawn if the whole ramp is above the floor).
    let ramp_half_x = config.ramp_dimensions_m.x * 0.5;
    // Use the *largest* possible Z half-extent so we never spawn a ramp that would hang off the floor.
    let ramp_half_z = (config.ramp_dimensions_m.z * config.inclined_length_multiplier) * 0.5;

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

    // Lazy-init per-lane state.
    if state.lane_next_y.len() != config.lanes {
        state.lane_next_y = vec![config.min_height_above_floor_m; config.lanes];
        state.lane_last_profile = vec![RampProfile::Flat; config.lanes];
        state.lane_up_streak = vec![0; config.lanes];
        state.lane_down_streak = vec![0; config.lanes];
    }

    // One-time prefill so you see ramps immediately.
    if !state.initialized {
        state.initialized = true;

        for i in 0..config.initial_ramps_per_lane {
            let z_base = z_spawn_min + (i as f32) * config.z_spacing_m;
            if z_base > z_spawn_max {
                break;
            }

            let mut prev_lane_end_y: Option<f32> = None;

            for lane in 0..config.lanes {
                let lane_center_x = lane_center_x(lane, config.lanes, config.lane_spacing_m);

                let mut x = lane_center_x
                    + rng_f32_range(
                        &mut state.seed,
                        -config.lane_x_jitter_m,
                        config.lane_x_jitter_m,
                    );
                x = x.clamp(x_spawn_min, x_spawn_max);

                // Build a path-continuous segment in this lane.
                let (profile, start_y_rel, end_y_rel) =
                    choose_next_lane_segment(&config, &mut state, lane);

                // Clamp to the previous lane end-Y for the same Z-step
                // so sideways moves remain feasible.
                let end_y_rel = clamp_to_neighbor_lane(&config, end_y_rel, prev_lane_end_y);

                let mut z = z_base
                    + rng_f32_range(
                        &mut state.seed,
                        -config.z_spawn_jitter_m,
                        config.z_spawn_jitter_m,
                    );
                z = z.clamp(z_spawn_min, z_spawn_max);

                spawn_one_ramp(
                    &mut commands,
                    &assets,
                    &config,
                    x,
                    floor_top_y + start_y_rel,
                    floor_top_y + end_y_rel,
                    z,
                    z_start,
                    z_end,
                    profile,
                );

                // Update continuity state to the actual end.
                state.lane_next_y[lane] = end_y_rel;
                prev_lane_end_y = Some(end_y_rel);
            }
        }
    }

    // Continuous spawn cadence.
    state.accum_s += time.delta_secs();
    if state.accum_s < config.spawn_interval_s {
        return;
    }
    state.accum_s = 0.0;

    let mut prev_lane_end_y: Option<f32> = None;

    for lane in 0..config.lanes {
        let lane_center_x = lane_center_x(lane, config.lanes, config.lane_spacing_m);
        let mut x = lane_center_x
            + rng_f32_range(
                &mut state.seed,
                -config.lane_x_jitter_m,
                config.lane_x_jitter_m,
            );
        x = x.clamp(x_spawn_min, x_spawn_max);

        let (profile, start_y_rel, end_y_rel) = choose_next_lane_segment(&config, &mut state, lane);
        let end_y_rel = clamp_to_neighbor_lane(&config, end_y_rel, prev_lane_end_y);

        let mut z = z_spawn
            + rng_f32_range(
                &mut state.seed,
                -config.z_spawn_jitter_m,
                config.z_spawn_jitter_m,
            );
        z = z.clamp(z_spawn_min, z_spawn_max);

        spawn_one_ramp(
            &mut commands,
            &assets,
            &config,
            x,
            floor_top_y + start_y_rel,
            floor_top_y + end_y_rel,
            z,
            z_start,
            z_end,
            profile,
        );

        state.lane_next_y[lane] = end_y_rel;
        prev_lane_end_y = Some(end_y_rel);
    }
}

/// Updates all [`MovingRamp`] entities by advancing them forward in Z only.
///
/// When a ramp leaves the floor bounds it is despawned.
pub(crate) fn move_ramps(
    mut commands: Commands,
    time: Res<Time>,
    mut ramps: Query<(Entity, &mut Transform, &mut MovingRamp)>,
) {
    let dt = time.delta_secs();
    for (e, mut t, mut r) in &mut ramps {
        r.current_vel = Vec3::new(0.0, 0.0, r.z_speed_mps);
        t.translation.z += r.z_speed_mps * dt;

        // Keep the segment Z bounds aligned with the moving transform.
        let dz = r.z_speed_mps * dt;
        r.segment_z_start += dz;
        r.segment_z_end += dz;

        if t.translation.z > r.z_max || t.translation.z < r.z_min {
            commands.entity(e).despawn();
        }
    }
}

/// Chooses the next segment profile for a lane and returns (profile, start_y_rel, end_y_rel).
///
/// Rules:
/// - In the mid-band, we forbid UP->UP and DOWN->DOWN.
/// - If the lane is near the bottom (<= low_band_y_m), we allow multi-UP streaks.
/// - If the lane is near the top (>= high_band_y_m), we allow multi-DOWN streaks.
fn choose_next_lane_segment(
    config: &RampSpawnConfig,
    state: &mut RampSpawnState,
    lane: usize,
) -> (RampProfile, f32, f32) {
    let start_y_rel = state.lane_next_y[lane];
    let last = state.lane_last_profile[lane];

    let in_low_band = start_y_rel <= config.low_band_y_m;
    let in_high_band = start_y_rel >= config.high_band_y_m;

    let profile = choose_next_profile(config, state, lane, last, in_low_band, in_high_band);

    // Hard rule: after an UP ramp, the next segment must not be another UP segment.
    // Flat is allowed (same height), or Down is allowed (descend). This prevents impossible climbs.
    let profile = if last.is_up() && profile.is_up() {
        RampProfile::Inclined {
            dir: RampSlopeDir::Down,
            angle_deg: config.allowed_incline_angles_deg[0],
        }
    } else {
        profile
    };

    // The vertical delta depends on the *actual* segment length.
    let base_len_z = config.ramp_dimensions_m.z;
    let mut end_y_rel = start_y_rel;

    match profile {
        RampProfile::Flat => {
            end_y_rel = start_y_rel;
        }
        RampProfile::Inclined { dir, angle_deg } => {
            let len_z = base_len_z * config.inclined_length_multiplier;
            let rise = (angle_deg.to_radians().tan()) * len_z;
            match dir {
                RampSlopeDir::Up => end_y_rel = start_y_rel + rise,
                RampSlopeDir::Down => end_y_rel = start_y_rel - rise,
            }
        }
    }

    // Clamp to global bounds. If clamped hard, adjust profile/streaks to avoid "sticking".
    if end_y_rel > config.max_height_above_floor_m {
        end_y_rel = config.max_height_above_floor_m;
    } else if end_y_rel < config.min_height_above_floor_m {
        end_y_rel = config.min_height_above_floor_m;
    }

    // Update lane history.
    state.lane_last_profile[lane] = profile;
    if profile.is_up() {
        state.lane_up_streak[lane] = state.lane_up_streak[lane].saturating_add(1);
        state.lane_down_streak[lane] = 0;
    } else if profile.is_down() {
        state.lane_down_streak[lane] = state.lane_down_streak[lane].saturating_add(1);
        state.lane_up_streak[lane] = 0;
    } else {
        // Flat breaks both streaks.
        state.lane_up_streak[lane] = 0;
        state.lane_down_streak[lane] = 0;
    }

    (profile, start_y_rel, end_y_rel)
}

/// Decide the next RampProfile using the combination rules and altitude bands.
fn choose_next_profile(
    config: &RampSpawnConfig,
    state: &mut RampSpawnState,
    lane: usize,
    last: RampProfile,
    in_low_band: bool,
    in_high_band: bool,
) -> RampProfile {
    // Pick an angle.
    let angle = {
        let idx = (rng_f32_01(&mut state.seed) * (config.allowed_incline_angles_deg.len() as f32))
            .floor()
            .clamp(0.0, (config.allowed_incline_angles_deg.len() - 1) as f32)
            as usize;
        config.allowed_incline_angles_deg[idx]
    };

    // Flat vs inclined probability.
    // We keep this at an even 50/50 split so spawning does not bias toward one type.
    // (Other hard rules below can still override direction to keep paths playable.)
    let flat_p = config.flat_probability_mid_band;

    let r = rng_f32_01(&mut state.seed);
    if r < flat_p {
        return RampProfile::Flat;
    }

    // Bias direction based on altitude.
    // - Low band: strongly prefer Up.
    // - High band: strongly prefer Down.
    // - Mid: neutral, but apply no UP->UP and no DOWN->DOWN.
    //
    // Extra rule: if the previous segment was an UP ramp, the next segment must not end higher
    // than the current height (otherwise it becomes very hard to reach). So we force the next
    // inclined segment to be DOWN (flat is still allowed by `flat_p`).
    let dir = match last {
        RampProfile::Inclined {
            dir: RampSlopeDir::Up,
            ..
        } => RampSlopeDir::Down,
        _ => {
            if in_low_band {
                // Allow long UP streaks.
                if rng_f32_01(&mut state.seed) < 0.85 {
                    RampSlopeDir::Up
                } else {
                    RampSlopeDir::Down
                }
            } else if in_high_band {
                // Allow long DOWN streaks.
                if rng_f32_01(&mut state.seed) < 0.85 {
                    RampSlopeDir::Down
                } else {
                    RampSlopeDir::Up
                }
            } else {
                // Mid band: enforce combinations.
                // Forbid Up->Up and Down->Down.
                match last {
                    RampProfile::Inclined {
                        dir: RampSlopeDir::Up,
                        ..
                    } => RampSlopeDir::Down,
                    RampProfile::Inclined {
                        dir: RampSlopeDir::Down,
                        ..
                    } => RampSlopeDir::Up,
                    RampProfile::Flat => {
                        if rng_f32_01(&mut state.seed) < 0.5 {
                            RampSlopeDir::Up
                        } else {
                            RampSlopeDir::Down
                        }
                    }
                }
            }
        }
    };

    // Safety: if we are in low band but we already have a long up streak, sprinkle flats.
    if in_low_band && state.lane_up_streak[lane] >= 3 && rng_f32_01(&mut state.seed) < 0.25 {
        return RampProfile::Flat;
    }

    // Safety: if we are in high band but we already have a long down streak, sprinkle flats.
    if in_high_band && state.lane_down_streak[lane] >= 3 && rng_f32_01(&mut state.seed) < 0.25 {
        return RampProfile::Flat;
    }

    RampProfile::Inclined {
        dir,
        angle_deg: angle,
    }
}

fn spawn_one_ramp(
    commands: &mut Commands,
    assets: &RampRenderAssets,
    config: &RampSpawnConfig,
    x: f32,
    y_start: f32,
    y_end: f32,
    z: f32,
    z_start: f32,
    z_end: f32,
    profile: RampProfile,
) {
    let half_x = config.ramp_dimensions_m.x * 0.5;
    let base_half_z = config.ramp_dimensions_m.z * 0.5;
    let thickness = config.ramp_dimensions_m.y;

    // Flat uses the base length; inclined is stretched along Z.
    let z_multiplier = match profile {
        RampProfile::Inclined { .. } => config.inclined_length_multiplier,
        RampProfile::Flat => 1.0,
    };

    let half_z = base_half_z * z_multiplier;
    let half_extents = Vec2::new(half_x, half_z);

    // We place the mesh centered between start/end in Y.
    // For inclined ramps we rotate around X so the local +Z axis becomes the incline direction.
    let center_y = (y_start + y_end) * 0.5;

    let mut transform = Transform::from_translation(Vec3::new(x, center_y, z));

    match profile {
        RampProfile::Flat => {
            // No rotation.
        }
        RampProfile::Inclined { dir, angle_deg } => {
            // If your world uses +Z forward, then an "Up" ramp should raise as Z increases.
            // That corresponds to rotating around X by a *negative* angle in Bevy's right-handed system.
            // We keep this consistent with the (y_start -> y_end) relationship.
            let sign = match dir {
                RampSlopeDir::Up => -1.0,
                RampSlopeDir::Down => 1.0,
            };
            transform.rotation = Quat::from_rotation_x(sign * angle_deg.to_radians());
        }
    }

    // Stretch inclined ramps so they feel less "grabby" and have fewer sudden slope changes.
    transform.scale = Vec3::new(1.0, 1.0, z_multiplier);

    // Segment bounds in world space (the player can use these to compute surface height).
    let segment_z_start = z - half_extents.y;
    let segment_z_end = z + half_extents.y;

    commands.spawn((
        Mesh3d(assets.mesh.clone()),
        MeshMaterial3d(assets.material.clone()),
        transform,
        MovingRamp {
            half_extents,
            thickness,
            z_speed_mps: config.ramp_speed_z_mps,
            z_min: z_start,
            z_max: z_end + half_z + 0.5,
            current_vel: Vec3::new(0.0, 0.0, config.ramp_speed_z_mps),
            profile,
            segment_z_start,
            segment_z_end,
            segment_y_start: y_start,
            segment_y_end: y_end,
        },
    ));
}

fn clamp_to_neighbor_lane(config: &RampSpawnConfig, y: f32, neighbor_y: Option<f32>) -> f32 {
    let Some(ny) = neighbor_y else {
        return y;
    };
    y.clamp(
        ny - config.max_vertical_step_cross_lane_m,
        ny + config.max_vertical_step_cross_lane_m,
    )
}

fn lane_center_x(lane: usize, lanes: usize, lane_spacing: f32) -> f32 {
    let lanes_f = lanes as f32;
    (lane as f32 - (lanes_f - 1.0) * 0.5) * lane_spacing
}

fn rng_u32(seed: &mut u32) -> u32 {
    // Simple LCG.
    *seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
    *seed
}

fn rng_f32_01(seed: &mut u32) -> f32 {
    let v = rng_u32(seed);
    // Keep 24 bits for a stable float in [0,1).
    let mant = (v >> 8) as u32;
    (mant as f32) / ((1u32 << 24) as f32)
}

fn rng_f32_range(seed: &mut u32, min: f32, max: f32) -> f32 {
    min + (max - min) * rng_f32_01(seed)
}

use bevy::pbr::MeshMaterial3d;
use bevy::prelude::*;

use crate::gameplay::{DownhillPhase, GameplayMode, GameplayState};
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
    /// A flat jump pad that auto-launches the player.
    Jump,
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

    fn is_flat_like(self) -> bool {
        matches!(self, RampProfile::Flat | RampProfile::Jump)
    }
}

/// Footprint shape for ramp surface checks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RampFootprint {
    Rect,
    Circle,
}

/// Component attached to every moving ramp.
///
/// Ramps only move forward (+Z). X is decided at spawn time and never changes.
/// Y is represented by a *segment* (start/end) which can be flat, jump, or inclined.
#[derive(Component, Clone, Copy, Debug)]
pub struct MovingRamp {
    /// Half-extents in X/Z for simple footprint tests.
    pub half_extents: Vec2,
    /// Footprint shape used for surface tests.
    pub footprint: RampFootprint,
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

    /// The ramp profile (flat, jump, or inclined).
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
    /// - For a [`RampProfile::Flat`] or [`RampProfile::Jump`] segment, the surface is constant.
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
            return Some(self.segment_y_start);
        }

        // Normalized position along the segment [0,1].
        let t = (world_z - self.segment_z_start) / len;

        // Compute the top surface Y in a way that is anchored to the “meaningful” end:
        // - Downhill ramps (player descends): measure from the highest point.
        // - Uphill ramps (player ascends): measure from the lowest point.
        // This is equivalent to linear interpolation between start/end, but makes the intent explicit
        // and stays correct even if (for any reason) start/end heights end up swapped.
        let top_y = match self.profile {
            RampProfile::Flat | RampProfile::Jump => self.segment_y_start,
            RampProfile::Inclined {
                dir: RampSlopeDir::Down,
                ..
            } => {
                let high = self.segment_y_start.max(self.segment_y_end);
                let drop = (self.segment_y_start - self.segment_y_end).abs();
                high - drop * t
            }
            RampProfile::Inclined {
                dir: RampSlopeDir::Up,
                ..
            } => {
                let low = self.segment_y_start.min(self.segment_y_end);
                let rise = (self.segment_y_start - self.segment_y_end).abs();
                low + rise * t
            }
        };

        Some(top_y)
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
    
}

/// Render assets shared by all ramps.
#[derive(Resource, Clone)]
pub struct RampRenderAssets {
    pub mesh: Handle<Mesh>,
    pub jump_mesh: Handle<Mesh>,
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
    /// Chance for a flat segment to become a jump pad (RandomGameplay only).
    pub jump_probability: f32,

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
            lanes: 3, // Three lanes (ramps per row)
            lane_spacing_m: 4.0,
            ramp_dimensions_m: Vec3::new(2.0, 0.25, 3.38),
            inclined_length_multiplier: 2.2,
            z_spacing_m: 5.0,
            z_spawn_jitter_m: 3.0,
            initial_ramps_per_lane: 6,
            min_height_above_floor_m: 1.0,
            max_height_above_floor_m: 40.0,
            lane_x_jitter_m: 0.8,
            ramp_speed_z_mps: 5.0,
            spawn_interval_s: 1.6 / 1.0, // Adjusted for faster ramps to keep spacing
            max_vertical_step_cross_lane_m: 0.9,
            flat_probability_mid_band: 0.50,
            jump_probability: 0.15,
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

    /// Per-lane lowest endpoint height of the last spawned segment.
    pub lane_last_low_y: Vec<f32>,
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
            lane_last_low_y: Vec::new(),
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
    let jump_radius = config
        .ramp_dimensions_m
        .x
        .min(config.ramp_dimensions_m.z)
        * 0.5;
    let jump_mesh = meshes.add(Cylinder::new(jump_radius, config.ramp_dimensions_m.y));

    let ramp_material = materials.add(StandardMaterial {
        // Different from floor on purpose.
        base_color: Color::srgb(0.90, 0.10, 0.60),
        perceptual_roughness: 0.9,
        metallic: 0.0,
        ..default()
    });

    commands.insert_resource(RampRenderAssets {
        mesh: ramp_mesh,
        jump_mesh,
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
    gameplay: Option<ResMut<GameplayState>>,
) {
    let mut gameplay = gameplay;
    let mode = gameplay
        .as_ref()
        .map(|g| g.current)
        .unwrap_or(GameplayMode::RandomGameplay);

    let floor_top_y = floor_top_y.map(|r| r.0).unwrap_or(0.0);

    // Floor bounds.
    let x_min = floor.center.x - floor.half_extents.x;
    let x_max = floor.center.x + floor.half_extents.x;
    let z_start = floor.center.z - floor.half_extents.y;
    let z_end = floor.center.z + floor.half_extents.y;

    // Ramp footprint bounds (we only spawn if the whole ramp is above the floor).
    let ramp_half_x = config.ramp_dimensions_m.x * 0.5;
    // Use the *largest* possible Z half-extent so we never spawn a ramp that would hang off the floor.
    // Uphill ramps are intentionally longer than downhill ones.
    let max_z_mult =
        (config.inclined_length_multiplier * 0.70).max(config.inclined_length_multiplier * 0.70);
    let ramp_half_z = (config.ramp_dimensions_m.z * max_z_mult) * 0.5;

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
    let lane_x_jitter = lane_safe_x_jitter_m(&config);

    // Lazy-init per-lane state.
    if state.lane_next_y.len() != config.lanes {
        state.lane_next_y = vec![config.min_height_above_floor_m; config.lanes];
        state.lane_last_profile = vec![RampProfile::Flat; config.lanes];
        state.lane_up_streak = vec![0; config.lanes];
        state.lane_down_streak = vec![0; config.lanes];
        state.lane_last_low_y = vec![config.min_height_above_floor_m; config.lanes];
    }

    // One-time prefill so you see ramps immediately.
    if !state.initialized {
        state.initialized = true;

        for i in 0..config.initial_ramps_per_lane {
            let z_base = z_spawn_min + (i as f32) * config.z_spacing_m;
            if z_base > z_spawn_max {
                break;
            }

            let (phase, down_angle_deg) = if mode == GameplayMode::DownhillGameplay {
                let Some(gameplay) = gameplay.as_mut() else {
                    return;
                };
                downhill_plan_for_row(&config, &state, gameplay)
            } else {
                (DownhillPhase::FlatUp, None)
            };
            let skip_lane =
                if mode == GameplayMode::DownhillGameplay && config.lanes > 1 {
                    Some((rng_u32(&mut state.seed) as usize) % config.lanes)
                } else {
                    None
                };

            let mut prev_lane_end_y: Option<f32> = None;

            for lane in 0..config.lanes {
                let lane_center_x = lane_center_x(lane, config.lanes, config.lane_spacing_m);

                let mut x =
                    lane_center_x + rng_f32_range(&mut state.seed, -lane_x_jitter, lane_x_jitter);
                x = x.clamp(x_spawn_min, x_spawn_max);

                // Build a path-continuous segment in this lane.
                let (profile, start_y_rel, end_y_rel) = match mode {
                    GameplayMode::RandomGameplay => {
                        choose_next_lane_segment(&config, &mut state, lane)
                    }
                    GameplayMode::DownhillGameplay => choose_next_lane_segment_downhill(
                        &config,
                        &state,
                        lane,
                        phase,
                        down_angle_deg,
                    ),
                };

                // Clamp to the previous lane end-Y for the same Z-step
                // so sideways moves remain feasible.
                let (profile, end_y_rel) = clamp_to_neighbor_lane(
                    &config,
                    profile,
                    start_y_rel,
                    end_y_rel,
                    prev_lane_end_y,
                );

                // Ensure the segment remains monotonic after clamps:
                // - Downhill: highest point is the start; never allow the end to rise above it.
                // - Uphill: lowest point is the start; never allow the end to dip below it.
                let (profile, end_y_rel) = enforce_profile_monotonic(profile, start_y_rel, end_y_rel);

                let mut z = z_base
                    + rng_f32_range(
                        &mut state.seed,
                        -config.z_spawn_jitter_m,
                        config.z_spawn_jitter_m,
                    );
                z = z.clamp(z_spawn_min, z_spawn_max);

                if skip_lane == Some(lane) {
                    update_lane_history(&mut state, lane, profile);
                    state.lane_next_y[lane] = if mode == GameplayMode::DownhillGameplay {
                        if profile.is_up() {
                            start_y_rel
                        } else {
                            end_y_rel
                        }
                    } else {
                        next_lane_anchor_y(profile, start_y_rel, end_y_rel)
                    };
                    state.lane_last_low_y[lane] = start_y_rel.min(end_y_rel);
                    prev_lane_end_y = Some(end_y_rel);
                    continue;
                }

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

                update_lane_history(&mut state, lane, profile);
                // Update continuity state to the actual end.
                state.lane_next_y[lane] = if mode == GameplayMode::DownhillGameplay {
                    if profile.is_up() {
                        start_y_rel
                    } else {
                        end_y_rel
                    }
                } else {
                    next_lane_anchor_y(profile, start_y_rel, end_y_rel)
                };
                state.lane_last_low_y[lane] = start_y_rel.min(end_y_rel);
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

    let (phase, down_angle_deg) = if mode == GameplayMode::DownhillGameplay {
        let Some(gameplay) = gameplay.as_mut() else {
            return;
        };
        downhill_plan_for_row(&config, &state, gameplay)
    } else {
        (DownhillPhase::FlatUp, None)
    };

    let skip_lane = if mode == GameplayMode::DownhillGameplay && config.lanes > 1 {
        Some((rng_u32(&mut state.seed) as usize) % config.lanes)
    } else {
        None
    };

    let mut prev_lane_end_y: Option<f32> = None;

    for lane in 0..config.lanes {
        let lane_center_x = lane_center_x(lane, config.lanes, config.lane_spacing_m);
        let mut x =
            lane_center_x + rng_f32_range(&mut state.seed, -lane_x_jitter, lane_x_jitter);
        x = x.clamp(x_spawn_min, x_spawn_max);

        let (profile, start_y_rel, end_y_rel) = match mode {
            GameplayMode::RandomGameplay => choose_next_lane_segment(&config, &mut state, lane),
            GameplayMode::DownhillGameplay => choose_next_lane_segment_downhill(
                &config,
                &state,
                lane,
                phase,
                down_angle_deg,
            ),
        };
        let (profile, end_y_rel) = clamp_to_neighbor_lane(
            &config,
            profile,
            start_y_rel,
            end_y_rel,
            prev_lane_end_y,
        );
        let (profile, end_y_rel) = enforce_profile_monotonic(profile, start_y_rel, end_y_rel);

        let mut z = z_spawn
            + rng_f32_range(
                &mut state.seed,
                -config.z_spawn_jitter_m,
                config.z_spawn_jitter_m,
            );
        z = z.clamp(z_spawn_min, z_spawn_max);

        if skip_lane == Some(lane) {
            update_lane_history(&mut state, lane, profile);
            state.lane_next_y[lane] = if mode == GameplayMode::DownhillGameplay {
                if profile.is_up() {
                    start_y_rel
                } else {
                    end_y_rel
                }
            } else {
                next_lane_anchor_y(profile, start_y_rel, end_y_rel)
            };
            state.lane_last_low_y[lane] = start_y_rel.min(end_y_rel);
            prev_lane_end_y = Some(end_y_rel);
            continue;
        }

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

        update_lane_history(&mut state, lane, profile);
        state.lane_next_y[lane] = if mode == GameplayMode::DownhillGameplay {
            if profile.is_up() {
                start_y_rel
            } else {
                end_y_rel
            }
        } else {
            next_lane_anchor_y(profile, start_y_rel, end_y_rel)
        };
        state.lane_last_low_y[lane] = start_y_rel.min(end_y_rel);
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
    let prev_anchor_y_rel = state.lane_next_y[lane];
    let last_low_y_rel = state.lane_last_low_y[lane];
    let last = state.lane_last_profile[lane];

    let in_low_band = prev_anchor_y_rel <= config.low_band_y_m;
    let in_high_band = prev_anchor_y_rel >= config.high_band_y_m;

    let profile = choose_next_profile(config, state, lane, last, in_low_band, in_high_band);

    let mut profile = profile;

    // The vertical delta depends on the *actual* segment length.
    let base_len_z = config.ramp_dimensions_m.z;
    let flat_step_m = config.max_vertical_step_cross_lane_m;
    let next_anchor_y_rel = (prev_anchor_y_rel + flat_step_m).min(config.max_height_above_floor_m);
    let mut start_y_rel = prev_anchor_y_rel;
    let mut end_y_rel = prev_anchor_y_rel;

    match profile {
        RampProfile::Flat | RampProfile::Jump => {
            start_y_rel = next_anchor_y_rel;
            end_y_rel = next_anchor_y_rel;
        }
        RampProfile::Inclined { dir, angle_deg } => {
            // IMPORTANT:
            // The ramp mesh is rotated by `angle_deg` in `spawn_one_ramp`.
            // A rotated cuboid produces a height delta of `sin(angle) * length` along its local Z.
            // So we must use `sin`, not `tan`, otherwise the *visual* high point will drift above
            // the logical `segment_y_*` heights and you'll see downhill ramps whose top looks
            // higher than previous ramps.
            let angle_rad = angle_deg.to_radians();

            // Reduce UP ramp length by 30% (makes climbs less "long/rectangular").
            // Downhill ramps are intentionally shorter (x0.70) to make drops snappier.
            let len_z = match dir {
                RampSlopeDir::Up => base_len_z * config.inclined_length_multiplier * 0.70,
                RampSlopeDir::Down => base_len_z * config.inclined_length_multiplier * 0.70,
            };

            let rise = angle_rad.sin() * len_z;
            match dir {
                // Align the *highest* point of an UP ramp to the previous height.
                // If the last segment was also UP, anchor below its lowest endpoint.
                RampSlopeDir::Up => {
                    let up_anchor_y_rel = if last.is_up() {
                        last_low_y_rel
                    } else {
                        prev_anchor_y_rel
                    };
                    end_y_rel = up_anchor_y_rel;
                    start_y_rel = up_anchor_y_rel - rise;

                    // If the lower end would hit the floor, fall back to a higher flat segment.
                    if start_y_rel < config.min_height_above_floor_m {
                        profile = RampProfile::Flat;
                        start_y_rel = next_anchor_y_rel;
                        end_y_rel = next_anchor_y_rel;
                    }
                }
                RampSlopeDir::Down => {
                    // Align the highest point of a DOWN ramp to the same baseline
                    // a flat/up segment would start at.
                    start_y_rel = next_anchor_y_rel;
                    end_y_rel = next_anchor_y_rel - rise;
                    if end_y_rel < config.min_height_above_floor_m {
                        profile = RampProfile::Flat;
                        start_y_rel = next_anchor_y_rel;
                        end_y_rel = next_anchor_y_rel;
                    }
                }
            }
        }
    }

    // After an UP ramp, a flat segment must not be above the low endpoint of that ramp.
    if profile.is_flat_like() && last.is_up() {
        let flat_y = last_low_y_rel.clamp(
            config.min_height_above_floor_m,
            config.max_height_above_floor_m,
        );
        start_y_rel = flat_y;
        end_y_rel = flat_y;
    }

    // Clamp to global bounds. If clamped hard, adjust profile/streaks to avoid "sticking".
    if end_y_rel > config.max_height_above_floor_m {
        end_y_rel = config.max_height_above_floor_m;
    } else if end_y_rel < config.min_height_above_floor_m {
        end_y_rel = config.min_height_above_floor_m;
    }

    (profile, start_y_rel, end_y_rel)
}

/// Updates per-lane profile history and streak counters.
fn update_lane_history(state: &mut RampSpawnState, lane: usize, profile: RampProfile) {
    state.lane_last_profile[lane] = profile;
    if profile.is_up() {
        state.lane_up_streak[lane] = state.lane_up_streak[lane].saturating_add(1);
        state.lane_down_streak[lane] = 0;
    } else if profile.is_down() {
        state.lane_down_streak[lane] = state.lane_down_streak[lane].saturating_add(1);
        state.lane_up_streak[lane] = 0;
    } else {
        // Flat or jump pads break both streaks.
        state.lane_up_streak[lane] = 0;
        state.lane_down_streak[lane] = 0;
    }
}

/// Computes the phase and optional downhill angle for the next spawn row.
fn downhill_plan_for_row(
    config: &RampSpawnConfig,
    state: &RampSpawnState,
    gameplay: &mut GameplayState,
) -> (DownhillPhase, Option<f32>) {
    let mut phase = refresh_downhill_phase(config, state, gameplay);
    let mut down_angle_deg = None;

    if phase == DownhillPhase::DownRamps {
        down_angle_deg = select_downhill_angle_deg(config, state);
        if down_angle_deg.is_none() {
            gameplay.downhill_phase = DownhillPhase::FlatUp;
            phase = DownhillPhase::FlatUp;
        }
    }

    (phase, down_angle_deg)
}

/// Switches phases when all lanes reach the top or bottom bounds.
fn refresh_downhill_phase(
    config: &RampSpawnConfig,
    state: &RampSpawnState,
    gameplay: &mut GameplayState,
) -> DownhillPhase {
    let max_y = config.max_height_above_floor_m;
    let min_y = config.min_height_above_floor_m;
    let eps = 0.02;

    match gameplay.downhill_phase {
        DownhillPhase::FlatUp => {
            if state.lane_next_y.iter().all(|y| *y >= max_y - eps) {
                gameplay.downhill_phase = DownhillPhase::DownRamps;
            }
        }
        DownhillPhase::DownRamps => {
            if state.lane_next_y.iter().all(|y| *y <= min_y + eps) {
                gameplay.downhill_phase = DownhillPhase::FlatUp;
            }
        }
    }

    gameplay.downhill_phase
}

/// Picks a downhill angle that keeps all lanes above the minimum height.
fn select_downhill_angle_deg(config: &RampSpawnConfig, state: &RampSpawnState) -> Option<f32> {
    if state.lane_next_y.is_empty() {
        return None;
    }

    let min_y = config.min_height_above_floor_m;
    let mut max_drop = f32::INFINITY;
    for y in &state.lane_next_y {
        max_drop = max_drop.min(*y - min_y);
    }
    if max_drop <= 0.0 {
        return None;
    }

    let len_z = config.ramp_dimensions_m.z * config.inclined_length_multiplier * 0.70;
    let mut best_angle = None;
    let mut best_rise = 0.0;
    for angle_deg in config.allowed_incline_angles_deg {
        let rise = angle_deg.to_radians().sin() * len_z;
        if rise <= max_drop + 1e-3 && rise >= best_rise {
            best_rise = rise;
            best_angle = Some(angle_deg);
        }
    }

    if best_angle.is_some() {
        return best_angle;
    }

    // If no allowed angle fits, use a shallower angle to hit the minimum exactly.
    let ratio = (max_drop / len_z).clamp(0.0, 1.0);
    Some(ratio.asin().to_degrees())
}

/// Chooses the next segment for DownhillGameplay (flat-up then incline ramps).
fn choose_next_lane_segment_downhill(
    config: &RampSpawnConfig,
    state: &RampSpawnState,
    lane: usize,
    phase: DownhillPhase,
    down_angle_deg: Option<f32>,
) -> (RampProfile, f32, f32) {
    let prev_anchor_y_rel = state.lane_next_y[lane];
    let flat_step_m = config.max_vertical_step_cross_lane_m;

    match phase {
        DownhillPhase::FlatUp => {
            let next_y_rel =
                (prev_anchor_y_rel + flat_step_m).min(config.max_height_above_floor_m);
            (RampProfile::Flat, next_y_rel, next_y_rel)
        }
        DownhillPhase::DownRamps => {
            let Some(angle_deg) = down_angle_deg else {
                let min_y = config.min_height_above_floor_m;
                return (RampProfile::Flat, min_y, min_y);
            };
            let len_z = config.ramp_dimensions_m.z * config.inclined_length_multiplier * 0.70;
            let rise = angle_deg.to_radians().sin() * len_z;
            let end_y_rel = prev_anchor_y_rel;
            let start_y_rel = prev_anchor_y_rel - rise;
            (
                RampProfile::Inclined {
                    dir: RampSlopeDir::Up,
                    angle_deg,
                },
                start_y_rel,
                end_y_rel,
            )
        }
    }
}

/// Decide the next RampProfile using the combination rules and altitude bands.
fn choose_next_profile(
    config: &RampSpawnConfig,
    state: &mut RampSpawnState,
    lane: usize,
    _last: RampProfile,
    _in_low_band: bool,
    _in_high_band: bool,
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
    // Jump pads are a subset of flat ramps so the overall split stays stable.
    let flat_p = config.flat_probability_mid_band;

    let r = rng_f32_01(&mut state.seed);
    if r < flat_p {
        if rng_f32_01(&mut state.seed) < config.jump_probability {
            return RampProfile::Jump;
        }
        return RampProfile::Flat;
    }

    // Height-based chance to allow an UP ramp: 0 at min height, 1 at max height.
    let min_y = config.min_height_above_floor_m;
    let max_y = config.max_height_above_floor_m;
    let denom = (max_y - min_y).max(f32::EPSILON);
    let height_t = ((state.lane_next_y[lane] - min_y) / denom).clamp(0.0, 1.0);
    // Bias UP when high, DOWN when low (slightly stronger than linear).
    let bias_pow = 1.3;
    let up_bias = height_t.powf(bias_pow);
    let down_bias = (1.0 - height_t).powf(bias_pow) * 0.49;
    let up_prob = if up_bias + down_bias > 0.0 {
        up_bias / (up_bias + down_bias)
    } else {
        0.5
    };

    let dir = if rng_f32_01(&mut state.seed) < up_prob {
        RampSlopeDir::Up
    } else {
        RampSlopeDir::Down
    };

    RampProfile::Inclined { dir, angle_deg: angle }
}

/// Spawns a single ramp entity and sets its surface metadata.
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
    let thickness = config.ramp_dimensions_m.y;
    let (half_x, base_half_z) = match profile {
        RampProfile::Jump => {
            let radius = config
                .ramp_dimensions_m
                .x
                .min(config.ramp_dimensions_m.z)
                * 0.5;
            (radius, radius)
        }
        _ => (
            config.ramp_dimensions_m.x * 0.5,
            config.ramp_dimensions_m.z * 0.5,
        ),
    };

    // Flat uses the base length; inclined is stretched along Z.
    let z_multiplier = match profile {
        // Uphill: 30% shorter
        RampProfile::Inclined {
            dir: RampSlopeDir::Up,
            ..
        } => config.inclined_length_multiplier * 0.70,
        // Downhill: 30% shorter
        RampProfile::Inclined {
            dir: RampSlopeDir::Down,
            ..
        } => config.inclined_length_multiplier * 0.70,
        RampProfile::Flat | RampProfile::Jump => 1.0,
    };

    let half_z = base_half_z * z_multiplier;
    let half_extents = Vec2::new(half_x, half_z);

    // `segment_y_start/end` store the world-space **top surface** heights at the segment ends.
    // Because the ramp mesh is a rotated cuboid, its translation.y is NOT the top surface.
    // We adjust the mesh center so that the rendered top surface endpoints match y_start/y_end.
    let center_y = match profile {
        RampProfile::Flat | RampProfile::Jump => {
            // Flat: top surface is simply center + thickness/2
            (y_start + y_end) * 0.5 - thickness * 0.5
        }
        RampProfile::Inclined { dir, angle_deg } => {
            let sign = match dir {
                RampSlopeDir::Up => -1.0,
                RampSlopeDir::Down => 1.0,
            };
            let alpha = sign * angle_deg.to_radians();
            // Average of the endpoint top-surface heights equals: center_y + (thickness/2)*cos(alpha)
            // => center_y = avg_top - (thickness/2)*cos(alpha)
            (y_start + y_end) * 0.5 - (thickness * 0.5) * alpha.cos()
        }
    };

    let mut transform = Transform::from_translation(Vec3::new(x, center_y, z));

    match profile {
        RampProfile::Flat | RampProfile::Jump => {
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

    let mesh = match profile {
        RampProfile::Jump => assets.jump_mesh.clone(),
        _ => assets.mesh.clone(),
    };
    let footprint = match profile {
        RampProfile::Jump => RampFootprint::Circle,
        _ => RampFootprint::Rect,
    };

    commands.spawn((
        Mesh3d(mesh),
        MeshMaterial3d(assets.material.clone()),
        transform,
        MovingRamp {
            half_extents,
            footprint,
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

/// Clamp the end height to a neighbor lane so lateral moves stay reasonable.
fn clamp_to_neighbor_lane(
    config: &RampSpawnConfig,
    profile: RampProfile,
    start_y_rel: f32,
    end_y_rel: f32,
    neighbor_y: Option<f32>,
) -> (RampProfile, f32) {
    let Some(ny) = neighbor_y else {
        return (profile, end_y_rel);
    };

    let min_y = ny - config.max_vertical_step_cross_lane_m;
    let max_y = ny + config.max_vertical_step_cross_lane_m;

    if end_y_rel < min_y || end_y_rel > max_y {
        if profile.is_up() {
            // Use the peak height for comparison; if it doesn't fit, keep it flat.
            return (RampProfile::Flat, start_y_rel);
        }
    }

    (profile, end_y_rel)
}

/// Enforces the slope direction; if clamped past it, fall back to Flat.
fn enforce_profile_monotonic(profile: RampProfile, start_y_rel: f32, end_y_rel: f32) -> (RampProfile, f32) {
    match profile {
        RampProfile::Inclined {
            dir: RampSlopeDir::Down,
            angle_deg,
        } => {
            // Downhill: the highest point is the start.
            // If clamps pushed the end above the start, convert to Flat at the start height.
            if end_y_rel > start_y_rel {
                (RampProfile::Flat, start_y_rel)
            } else {
                (
                    RampProfile::Inclined {
                        dir: RampSlopeDir::Down,
                        angle_deg,
                    },
                    end_y_rel,
                )
            }
        }
        RampProfile::Inclined {
            dir: RampSlopeDir::Up,
            angle_deg,
        } => {
            // Uphill: the lowest point is the start.
            // If clamps pulled the end below the start, convert to Flat at the start height.
            if end_y_rel < start_y_rel {
                (RampProfile::Flat, start_y_rel)
            } else {
                (
                    RampProfile::Inclined {
                        dir: RampSlopeDir::Up,
                        angle_deg,
                    },
                    end_y_rel,
                )
            }
        }
        RampProfile::Flat => (RampProfile::Flat, end_y_rel),
        RampProfile::Jump => (RampProfile::Jump, end_y_rel),
    }
}

/// Returns the world X center for a lane index.
fn lane_center_x(lane: usize, lanes: usize, lane_spacing: f32) -> f32 {
    let lanes_f = lanes as f32;
    (lane as f32 - (lanes_f - 1.0) * 0.5) * lane_spacing
}

/// Returns the next anchor height for continuity between segments.
fn next_lane_anchor_y(profile: RampProfile, start_y_rel: f32, end_y_rel: f32) -> f32 {
    if profile.is_down() {
        start_y_rel
    } else {
        end_y_rel
    }
}

/// Caps lane jitter so adjacent lanes do not overlap.
fn lane_safe_x_jitter_m(config: &RampSpawnConfig) -> f32 {
    // Keep a small margin so ramps in adjacent lanes never overlap.
    let margin = 0.05;
    let max_jitter = (config.lane_spacing_m - config.ramp_dimensions_m.x) * 0.5;
    let max_jitter = (max_jitter - margin).max(0.0);
    config.lane_x_jitter_m.min(max_jitter)
}

/// Deterministic LCG used for ramp spawning randomness.
fn rng_u32(seed: &mut u32) -> u32 {
    // Simple LCG.
    *seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
    *seed
}

/// Returns a random float in [0, 1) using the LCG.
fn rng_f32_01(seed: &mut u32) -> f32 {
    let v = rng_u32(seed);
    // Keep 24 bits for a stable float in [0,1).
    let mant = (v >> 8) as u32;
    (mant as f32) / ((1u32 << 24) as f32)
}

/// Returns a random float in [min, max) using the LCG.
fn rng_f32_range(seed: &mut u32, min: f32, max: f32) -> f32 {
    min + (max - min) * rng_f32_01(seed)
}

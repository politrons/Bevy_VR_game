use bevy::prelude::*;
use bevy::pbr::MeshMaterial3d;

use crate::scene::{FloorParams, FloorTopY};

/// Component attached to every moving ramp.
///
/// Ramps only move forward (+Z). X/Y are decided at spawn time and never change.
#[derive(Component, Clone, Copy, Debug)]
pub struct MovingRamp {
    /// Half-extents in X/Z for simple top-surface collision tests.
    pub half_extents: Vec2,
    /// Thickness in Y for computing the top surface.
    pub thickness: f32,

    /// Forward speed in meters per second.
    pub z_speed_mps: f32,
    /// Minimum allowed Z before despawn.
    pub z_min: f32,
    /// Maximum allowed Z before despawn.
    pub z_max: f32,

    /// Current velocity of the ramp (world-space). Updated every frame.
    pub current_vel: Vec3,
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
    /// Ramp dimensions (X, Y, Z) in meters.
    pub ramp_dimensions_m: Vec3,
    /// Distance between ramp spawns along Z in meters.
    pub z_spacing_m: f32,
    /// Random Z jitter applied per ramp at spawn time (in meters). Prevents perfect alignment.
    pub z_spawn_jitter_m: f32,
    /// Number of initial ramps to prefill per lane.
    pub initial_ramps_per_lane: usize,
    /// Maximum allowed vertical step in Y for ramps along the same lane.
    pub max_vertical_step_along_lane_m: f32,
    /// Minimum height above the floor in meters.
    pub min_height_above_floor_m: f32,
    /// Maximum height above the floor in meters.
    pub max_height_above_floor_m: f32,
    /// Random X jitter per lane in meters.
    pub lane_x_jitter_m: f32,
    /// Ramp speed in the Z direction in meters per second.
    pub ramp_speed_z_mps: f32,
    /// Distance traveled along Z before despawning (meters).
    #[allow(dead_code)]
    pub travel_distance_z_m: f32,
    /// Interval between ramp spawns in seconds.
    pub spawn_interval_s: f32,
    /// Max allowed vertical difference between adjacent lanes for ramps spawned at the same Z step.
    pub max_vertical_step_cross_lane_m: f32,
}

impl Default for RampSpawnConfig {
    fn default() -> Self {
        Self {
            lanes: 5,
            lane_spacing_m: 2.2,
            ramp_dimensions_m: Vec3::new(2.0, 0.25, 2.0),
            z_spacing_m: 5.0,
            z_spawn_jitter_m: 3.0,
            initial_ramps_per_lane: 6,
            max_vertical_step_along_lane_m: 1.2,
            max_vertical_step_cross_lane_m: 0.9,
            min_height_above_floor_m: 1.0,
            max_height_above_floor_m: 20.0,
            lane_x_jitter_m: 0.35,
            ramp_speed_z_mps: 2.5,
            travel_distance_z_m: 90.0,
            spawn_interval_s: 1.6 / 0.5, // Spawn more slowly (fewer ramps)
        }
    }
}

/// Internal state for the spawner.
#[derive(Resource, Clone, Debug)]
pub struct RampSpawnState {
    pub initialized: bool,
    pub accum_s: f32,
    pub seed: u32,
    pub lane_next_y: Vec<f32>,
    /// Per-lane vertical direction (+1.0 = trending up, -1.0 = trending down).
    pub lane_y_dir: Vec<f32>,
}

impl Default for RampSpawnState {
    fn default() -> Self {
        Self {
            initialized: false,
            accum_s: 0.0,
            seed: 0xC0FFEE_u32,
            lane_next_y: Vec::new(),
            lane_y_dir: Vec::new(),
        }
    }
}

/// Setup the ramp spawner: creates the shared mesh/material and inserts config/state.
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
/// Ramps always spawn fully on the floor at the start edge.
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
    let ramp_half_z = config.ramp_dimensions_m.z * 0.5;

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
        // Alternate initial direction per lane to create multiple "paths".
        state.lane_y_dir = (0..config.lanes)
            .map(|i| if i % 2 == 0 { 1.0 } else { -1.0 })
            .collect();
    }

    // Safety: ensure lane_y_dir is always the right length.
    if state.lane_y_dir.len() != config.lanes {
        state.lane_y_dir = (0..config.lanes)
            .map(|i| if i % 2 == 0 { 1.0 } else { -1.0 })
            .collect();
    }

    // One-time prefill so you see ramps immediately.
    if !state.initialized {
        state.initialized = true;
        // Prefill by Z-step first, then lanes, so we can clamp adjacent lanes at the same Z.
        for i in 0..config.initial_ramps_per_lane {
            let z_base = z_spawn_min + (i as f32) * config.z_spacing_m;
            if z_base > z_spawn_max {
                break;
            }

            let mut prev_lane_y: Option<f32> = None;

            for lane in 0..config.lanes {
                let lane_center_x = lane_center_x(lane, config.lanes, config.lane_spacing_m);

                let mut x = lane_center_x
                    + rng_f32_range(&mut state.seed, -config.lane_x_jitter_m, config.lane_x_jitter_m);
                x = x.clamp(x_spawn_min, x_spawn_max);

                // Choose the next Y for this lane (random walk), then clamp to the previous lane's Y
                // for the SAME Z-step so sideways moves remain feasible.
                let mut y_above = choose_next_lane_y(&config, &mut state, lane);
                y_above = clamp_to_neighbor_lane(&config, y_above, prev_lane_y);

                let mut z = z_base
                    + rng_f32_range(&mut state.seed, -config.z_spawn_jitter_m, config.z_spawn_jitter_m);
                z = z.clamp(z_spawn_min, z_spawn_max);

                spawn_one_ramp(
                    &mut commands,
                    &assets,
                    &config,
                    x,
                    floor_top_y + y_above,
                    z,
                    z_start,
                    z_end,
                );

                prev_lane_y = Some(y_above);
            }
        }
    }

    // Continuous spawn cadence.
    state.accum_s += time.delta_secs();
    if state.accum_s < config.spawn_interval_s {
        return;
    }
    state.accum_s = 0.0;

    let mut prev_lane_y: Option<f32> = None;

    for lane in 0..config.lanes {
        let lane_center_x = lane_center_x(lane, config.lanes, config.lane_spacing_m);
        let mut x = lane_center_x
            + rng_f32_range(&mut state.seed, -config.lane_x_jitter_m, config.lane_x_jitter_m);
        x = x.clamp(x_spawn_min, x_spawn_max);

        let mut y_above = choose_next_lane_y(&config, &mut state, lane);
        y_above = clamp_to_neighbor_lane(&config, y_above, prev_lane_y);

        let mut z = z_spawn + rng_f32_range(&mut state.seed, -config.z_spawn_jitter_m, config.z_spawn_jitter_m);
        z = z.clamp(z_spawn_min, z_spawn_max);

        spawn_one_ramp(
            &mut commands,
            &assets,
            &config,
            x,
            floor_top_y + y_above,
            z,
            z_start,
            z_end,
        );

        prev_lane_y = Some(y_above);
    }
}

/// Updates all [`MovingRamp`] entities by advancing them forward in Z only.
pub(crate) fn move_ramps(
    mut commands: Commands,
    time: Res<Time>,
    mut ramps: Query<(Entity, &mut Transform, &mut MovingRamp)>,
) {
    let dt = time.delta_secs();
    for (e, mut t, mut r) in &mut ramps {
        r.current_vel = Vec3::new(0.0, 0.0, r.z_speed_mps);
        t.translation.z += r.z_speed_mps * dt;
        if t.translation.z > r.z_max || t.translation.z < r.z_min {
            commands.entity(e).despawn();
        }
    }
}

fn spawn_one_ramp(
    commands: &mut Commands,
    assets: &RampRenderAssets,
    config: &RampSpawnConfig,
    x: f32,
    y: f32,
    z: f32,
    z_start: f32,
    z_end: f32,
) {
    let _ = config; // kept for future expansions

    let half_extents = Vec2::new(config.ramp_dimensions_m.x * 0.5, config.ramp_dimensions_m.z * 0.5);
    let thickness = config.ramp_dimensions_m.y;

    commands.spawn((
        Mesh3d(assets.mesh.clone()),
        MeshMaterial3d(assets.material.clone()),
        Transform::from_translation(Vec3::new(x, y, z)),
        MovingRamp {
            half_extents,
            thickness,
            z_speed_mps: config.ramp_speed_z_mps,
            z_min: z_start,
            z_max: z_end + (config.ramp_dimensions_m.z * 0.5) + 0.5,
            current_vel: Vec3::new(0.0, 0.0, config.ramp_speed_z_mps),
        },
    ));
}

fn clamp_to_neighbor_lane(config: &RampSpawnConfig, y: f32, neighbor_y: Option<f32>) -> f32 {
    let Some(ny) = neighbor_y else { return y; };
    y.clamp(
        ny - config.max_vertical_step_cross_lane_m,
        ny + config.max_vertical_step_cross_lane_m,
    )
}

fn lane_center_x(lane: usize, lanes: usize, lane_spacing: f32) -> f32 {
    let lanes_f = lanes as f32;
    (lane as f32 - (lanes_f - 1.0) * 0.5) * lane_spacing
}

fn choose_next_lane_y(config: &RampSpawnConfig, state: &mut RampSpawnState, lane: usize) -> f32 {
    // Directional random walk with "bounce" so lanes reliably go up AND down.
    let cur = state.lane_next_y[lane];

    // Occasionally flip direction (adds long-term variety).
    // ~7% chance per ramp spawn step.
    if rng_f32_01(&mut state.seed) < 0.07 {
        state.lane_y_dir[lane] *= -1.0;
    }

    // Step magnitude: always positive, direction decides sign.
    let mag = rng_f32_range(&mut state.seed, 0.1, config.max_vertical_step_along_lane_m);
    let mut next = cur + state.lane_y_dir[lane] * mag;

    // Bounce at bounds (don’t "stick" at max/min).
    if next >= config.max_height_above_floor_m {
        next = config.max_height_above_floor_m;
        state.lane_y_dir[lane] = -1.0;
    } else if next <= config.min_height_above_floor_m {
        next = config.min_height_above_floor_m;
        state.lane_y_dir[lane] = 1.0;
    }

    state.lane_next_y[lane] = next;
    next
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

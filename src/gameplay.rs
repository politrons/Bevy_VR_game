use bevy::prelude::*;

/// Gameplay modes that define how ramps are generated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GameplayMode {
    /// Uses the current random ramp generator.
    RandomGameplay,
    /// Deterministic flow: flat climb to max, then incline ramps to min.
    DownhillGameplay,
    /// Deterministic flow: jump pads climb to max, then incline ramps to min.
    JumpGameplay,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DownhillPhase {
    /// Flat ramps step upward until the max height is reached.
    FlatUp,
    /// Use Up ramps to descend back toward the minimum height.
    DownRamps,
}

const DEFAULT_MODES: [GameplayMode; 3] = [
    GameplayMode::RandomGameplay,
    GameplayMode::DownhillGameplay,
    GameplayMode::JumpGameplay,
];
const DEFAULT_SWITCH_INTERVAL_S: f32 = 120.0;
const DEFAULT_SEED: u32 = 0xA5A5_1234;

/// Global gameplay state used to pick the active ramp generation mode.
#[derive(Resource, Debug)]
pub struct GameplayState {
    pub modes: Vec<GameplayMode>,
    pub current: GameplayMode,
    pub last_switch_s: f32,
    pub switch_interval_s: f32,
    pub seed: u32,
    pub downhill_phase: DownhillPhase,
}

/// Initializes the gameplay mode list and sets the first mode.
pub(crate) fn setup_gameplay(mut commands: Commands) {
    let current = DEFAULT_MODES[0];
    commands.insert_resource(GameplayState {
        modes: DEFAULT_MODES.to_vec(),
        current,
        last_switch_s: 0.0,
        switch_interval_s: DEFAULT_SWITCH_INTERVAL_S,
        seed: DEFAULT_SEED,
        downhill_phase: DownhillPhase::FlatUp,
    });
}

/// Switches to a random gameplay mode every fixed interval.
pub(crate) fn update_gameplay_mode(time: Res<Time>, mut state: ResMut<GameplayState>) {
    let now = time.elapsed_secs();
    if now - state.last_switch_s < state.switch_interval_s {
        return;
    }
    state.last_switch_s = now;

    if state.modes.is_empty() {
        return;
    }

    let modes_len = state.modes.len();
    let mut idx = rng_index(&mut state.seed, modes_len);
    if state.modes.len() > 1 && state.modes[idx] == state.current {
        idx = (idx + 1) % state.modes.len();
    }
    let next = state.modes[idx];
    if next != state.current
        && matches!(next, GameplayMode::DownhillGameplay | GameplayMode::JumpGameplay)
    {
        state.downhill_phase = DownhillPhase::FlatUp;
    }
    state.current = next;
}

fn rng_index(seed: &mut u32, len: usize) -> usize {
    if len == 0 {
        return 0;
    }
    (rng_u32(seed) as usize) % len
}

fn rng_u32(seed: &mut u32) -> u32 {
    *seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
    *seed
}

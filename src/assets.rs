use bevy::prelude::*;
use log::{error, info};
// -------------------------
// Optional texture probe (Android/Quest)
// -------------------------

// Loads an asset once and logs Loaded/Failed. This does NOT apply the texture to any mesh/material.
#[derive(Debug, Clone)]
pub(crate) struct ProbeItem {
    pub(crate) handle: Handle<Image>,
    pub(crate) path: String,
    pub(crate) done: bool,
}

#[derive(Resource)]
pub(crate) struct TextureProbe {
    pub(crate) probes: Vec<ProbeItem>,
    pub(crate) ticks: u32,
}

pub(crate) fn probe_texture_load_once(
    probe: Option<ResMut<TextureProbe>>,
    asset_server: Res<AssetServer>,
) {
    let Some(mut probe) = probe else {
        return;
    };

    probe.ticks = probe.ticks.saturating_add(1);
    let ticks = probe.ticks;

    if ticks == 1 {
        info!("TextureProbe tick=1 (system running)");
    }

    for p in &mut probe.probes {
        if p.done {
            continue;
        }

        let state = asset_server.get_load_state(p.handle.id());

        match state {
            Some(bevy::asset::LoadState::Loaded) => {
                info!(
                    "TextureProbe loaded ✅: assets/{} (id={:?})",
                    p.path,
                    p.handle.id()
                );
                p.done = true;
            }
            Some(bevy::asset::LoadState::Failed(err)) => {
                error!(
                    "TextureProbe failed ❌: assets/{} (id={:?}) err={:?}",
                    p.path,
                    p.handle.id(),
                    err
                );
                p.done = true;
            }
            _ => {
                if ticks % 120 == 0 {
                    info!(
                        "TextureProbe state: {:?} assets/{} (id={:?}) ticks={}",
                        state,
                        p.path,
                        p.handle.id(),
                        ticks
                    );
                }
            }
        }
    }
}
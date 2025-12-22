use crate::controller::create_action_entities;

use bevy::asset::{AssetMetaCheck, AssetPlugin};
use bevy::camera::ClearColorConfig;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::ecs::schedule::common_conditions::resource_exists;
use bevy::prelude::*;
use bevy::render::view::{Hdr, Msaa, NoIndirectDrawing};
use bevy_mod_openxr::{add_xr_plugins, openxr_session_running, resources::OxrViews};
use bevy_mod_xr::camera::{XrCamera, XrViewInit};
use bevy_mod_xr::session::XrSessionCreated;
use bevy_xr_utils::actions::{
    XRUtilsActionSystems,
    XRUtilsActionsPlugin,
};
// -------------------------
// Logging
// -------------------------

#[cfg(target_os = "android")]
pub fn setup_logging() {
    // Requires `android_logger` + `log` in Cargo.toml.
    android_logger::init_once(
        android_logger::Config::default()
            .with_tag("bevy")
            .with_max_level(log::LevelFilter::Info),
    );
    log::info!("Logging initialized (android)");
}

#[cfg(not(target_os = "android"))]
pub fn setup_logging() {
    println!("Logging initialized (non-android)");
}

// -------------------------
// Modules
// -------------------------

mod assets;
mod controller_cubes;
mod player;
mod ramp;
mod scene;
mod controller;
// -------------------------
// Imports used by the root module
// -------------------------
use crate::player::{
    handle_player, PlayerKinematics, PlayerProgress, PlayerSettings
    ,
};
use crate::controller_cubes::register_controller_cubes;
use crate::ramp::{move_ramps, setup_ramp_spawner, spawn_moving_ramps, RampRenderAssets, RampSpawnConfig, RampSpawnState};
use crate::scene::{
    setup_scene, snap_player_to_floor_once, FloorParams, FloorTopY, PlayerSpawn,
};

#[bevy_main]
fn main() {
    setup_logging();

    let mut app = App::new();
    app
        .insert_resource(ClearColor(Color::srgb(0.53, 0.81, 0.92)))
        .insert_resource(PlayerSettings::default())
        .insert_resource(PlayerKinematics::default())
        .insert_resource(PlayerProgress::default())
        .add_plugins(add_xr_plugins(
            DefaultPlugins.set(AssetPlugin {
                // Android / Quest tip: avoid `.meta` lookups that can fail depending on packaging.
                // If you rely on processed assets + meta, switch this back to `Always`.
                meta_check: AssetMetaCheck::Never,
                ..default()
            }),
        ))
        .add_plugins(XRUtilsActionsPlugin)
        .add_systems(Startup, setup_scene)
        .add_systems(Startup, setup_ramp_spawner.after(setup_scene))
        .add_systems(XrSessionCreated, tune_xr_cameras.after(XrViewInit))
        .add_systems(Startup, create_action_entities.before(XRUtilsActionSystems::CreateEvents))
        // Gameplay systems (guarded for Quest lifecycle quirks)
        .add_systems(Update, move_ramps.run_if(openxr_session_running))
        .add_systems(
            Update,
            spawn_moving_ramps
                .run_if(openxr_session_running)
                .run_if(resource_exists::<RampRenderAssets>)
                .run_if(resource_exists::<RampSpawnConfig>)
                .run_if(resource_exists::<RampSpawnState>)
                .run_if(resource_exists::<PlayerSpawn>),
        )
        .add_systems(
            Update,
            snap_player_to_floor_once
                .run_if(openxr_session_running)
                .run_if(resource_exists::<FloorParams>)
                .run_if(resource_exists::<FloorTopY>)
                .run_if(resource_exists::<PlayerSpawn>),
        )
        .add_systems(
            Update,
            handle_player
                .run_if(openxr_session_running)
                .run_if(resource_exists::<FloorParams>)
                .run_if(resource_exists::<PlayerSpawn>)
                .run_if(resource_exists::<PlayerProgress>)
                .run_if(resource_exists::<OxrViews>),
        )
        ;
    register_controller_cubes(&mut app);
    app.run();
}

// -------------------------
// XR camera tuning (right-eye flicker fix)
// -------------------------

fn tune_xr_cameras(mut commands: Commands, mut cams: Query<(Entity, &mut Camera, &XrCamera)>) {
    for (e, mut cam, _xr_cam) in &mut cams {
        cam.is_active = true;
        cam.clear_color = ClearColorConfig::Custom(Color::srgb(0.53, 0.81, 0.92));
        commands.entity(e).remove::<Hdr>();
        commands.entity(e).insert(Tonemapping::None);
        commands.entity(e).insert(Msaa::Off);
        // Fix for right-eye flicker: disable indirect drawing per camera.
        commands.entity(e).insert(NoIndirectDrawing);
    }
}

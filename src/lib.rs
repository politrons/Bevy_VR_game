use crate::controller::create_action_entities;

use bevy::animation::{AnimationPlayer, AnimationTarget};
use bevy::asset::{AssetMetaCheck, AssetPlugin};
use bevy::camera::primitives::Aabb;
use bevy::camera::ClearColorConfig;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::ecs::schedule::common_conditions::resource_exists;
use bevy::gltf::{
    GltfExtras, GltfMaterialExtras, GltfMaterialName, GltfMeshExtras, GltfMeshName,
    GltfSceneExtras,
};
use bevy::mesh::morph::{MeshMorphWeights, MorphWeights};
use bevy::mesh::skinning::SkinnedMesh;
use bevy::mesh::Mesh3d;
use bevy::light::{DirectionalLight, PointLight, SpotLight};
use bevy::pbr::{MeshMaterial3d, StandardMaterial};
use bevy::prelude::*;
use bevy::render::view::{Hdr, Msaa, NoIndirectDrawing};
use bevy::transform::components::{GlobalTransform, TransformTreeChanged};
use bevy::transform::TransformSystems;
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
mod gameplay;
mod player;
mod ramp;
mod scene;
mod controller;
mod shooting;
// -------------------------
// Imports used by the root module
// -------------------------
use crate::player::{
    handle_player, PlayerKinematics, PlayerProgress, PlayerSettings
    ,
};
use crate::controller::register_controller_cubes;
use crate::gameplay::{setup_gameplay, update_gameplay_mode, GameplayState};
use crate::ramp::{
    move_ramps, prepare_flat_ramp_model, prepare_grind_ramp_model, prepare_jump_ramp_model,
    prepare_monster_attack_model, prepare_monster_dead_model, prepare_monster_walking_model,
    prepare_slide_ramp_model,
    register_monster_animation_players, setup_ramp_spawner, spawn_moving_ramps, update_monsters,
    update_ramp_lod, FlatRampModel, GrindRampModel, JumpRampModel, MonsterAttackModel,
    MonsterDeadModel, MonsterRenderAssets, MonsterWalkingModel, RampLodMaterials,
    RampRenderAssets, RampSpawnConfig, RampSpawnState, SlideRampModel,
};
use crate::shooting::{move_bullets, setup_bullet_assets, spawn_bullets, BulletAssets, BulletFireState};
use crate::scene::{
    setup_scene, snap_player_to_floor_once, spawn_planet_floor, spawn_space_skybox,
    spawn_skateboard, spin_planet_floor, sync_space_sky_position, FloorParams, FloorTopY,
    PlanetFloorAssets, PlanetFloorState, PlayerSpawn, SkateboardAssets, SkateboardState,
    SpaceSkyAssets, SpaceSkyState,
};

#[bevy_main]
fn main() {
    setup_logging();

    let mut app = App::new();
    register_scene_types(&mut app);
    app
        .insert_resource(ClearColor(Color::srgb(0.53, 0.81, 0.92)))
        .insert_resource(PlayerSettings::default())
        .insert_resource(PlayerKinematics::default())
        .insert_resource(PlayerProgress::default())
        .insert_resource(BulletFireState::default())
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
        .add_systems(Startup, setup_gameplay)
        .add_systems(Startup, setup_ramp_spawner.after(setup_scene))
        .add_systems(Startup, setup_bullet_assets)
        .add_systems(
            Update,
            prepare_flat_ramp_model
                .run_if(resource_exists::<FlatRampModel>)
                .run_if(resource_exists::<RampRenderAssets>),
        )
        .add_systems(
            Update,
            prepare_jump_ramp_model
                .run_if(resource_exists::<JumpRampModel>)
                .run_if(resource_exists::<RampRenderAssets>),
        )
        .add_systems(
            Update,
            prepare_grind_ramp_model
                .run_if(resource_exists::<GrindRampModel>)
                .run_if(resource_exists::<RampRenderAssets>),
        )
        .add_systems(
            Update,
            prepare_slide_ramp_model
                .run_if(resource_exists::<SlideRampModel>)
                .run_if(resource_exists::<RampRenderAssets>),
        )
        .add_systems(
            Update,
            prepare_monster_attack_model
                .run_if(resource_exists::<MonsterAttackModel>)
                .run_if(resource_exists::<MonsterRenderAssets>),
        )
        .add_systems(
            Update,
            prepare_monster_dead_model
                .run_if(resource_exists::<MonsterDeadModel>)
                .run_if(resource_exists::<MonsterRenderAssets>),
        )
        .add_systems(
            Update,
            prepare_monster_walking_model
                .run_if(resource_exists::<MonsterWalkingModel>)
                .run_if(resource_exists::<MonsterRenderAssets>),
        )
        .add_systems(
            Update,
            spawn_planet_floor
                .run_if(resource_exists::<PlanetFloorAssets>)
                .run_if(resource_exists::<PlanetFloorState>)
                .run_if(resource_exists::<FloorParams>),
        )
        .add_systems(
            Update,
            spawn_space_skybox
                .run_if(resource_exists::<SpaceSkyAssets>)
                .run_if(resource_exists::<SpaceSkyState>)
                .run_if(resource_exists::<FloorParams>),
        )
        .add_systems(
            Update,
            spawn_skateboard
                .run_if(openxr_session_running)
                .run_if(resource_exists::<SkateboardAssets>)
                .run_if(resource_exists::<SkateboardState>),
        )
        .add_systems(Update, sync_space_sky_position.run_if(openxr_session_running))
        .add_systems(Update, spin_planet_floor)
        .add_systems(XrSessionCreated, tune_xr_cameras.after(XrViewInit))
        .add_systems(Startup, create_action_entities.before(XRUtilsActionSystems::CreateEvents))
        // Gameplay systems (guarded for Quest lifecycle quirks)
        .add_systems(
            Update,
            update_gameplay_mode
                .run_if(openxr_session_running)
                .run_if(resource_exists::<GameplayState>),
        )
        .add_systems(Update, move_ramps.run_if(openxr_session_running))
        .add_systems(Update, update_monsters.run_if(openxr_session_running))
        .add_systems(
            Update,
            register_monster_animation_players.run_if(openxr_session_running),
        )
        .add_systems(
            PostUpdate,
            update_ramp_lod
                .after(TransformSystems::Propagate)
                .run_if(openxr_session_running)
                .run_if(resource_exists::<RampLodMaterials>)
                .run_if(resource_exists::<RampSpawnConfig>),
        )
        .add_systems(
            Update,
            spawn_moving_ramps
                .run_if(openxr_session_running)
                .run_if(resource_exists::<RampRenderAssets>)
                .run_if(resource_exists::<RampSpawnConfig>)
                .run_if(resource_exists::<RampSpawnState>)
                .run_if(resource_exists::<FloorTopY>)
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
        .add_systems(
            Update,
            spawn_bullets
                .run_if(openxr_session_running)
                .run_if(resource_exists::<BulletAssets>),
        )
        .add_systems(Update, move_bullets.run_if(openxr_session_running))
        ;
    register_controller_cubes(&mut app);
    app.run();
}

// -------------------------
// XR camera tuning (right-eye flicker fix)
// -------------------------

fn register_scene_types(app: &mut App) {
    app.register_type::<Transform>()
        .register_type::<GlobalTransform>()
        .register_type::<TransformTreeChanged>()
        .register_type::<Visibility>()
        .register_type::<InheritedVisibility>()
        .register_type::<ViewVisibility>()
        .register_type::<Name>()
        .register_type::<Children>()
        .register_type::<ChildOf>()
        .register_type::<Mesh3d>()
        .register_type::<MeshMaterial3d<StandardMaterial>>()
        .register_type::<Aabb>()
        .register_type::<SkinnedMesh>()
        .register_type::<AnimationPlayer>()
        .register_type::<AnimationTarget>()
        .register_type::<MorphWeights>()
        .register_type::<MeshMorphWeights>()
        .register_type::<GltfExtras>()
        .register_type::<GltfSceneExtras>()
        .register_type::<GltfMeshExtras>()
        .register_type::<GltfMaterialExtras>()
        .register_type::<GltfMeshName>()
        .register_type::<GltfMaterialName>()
        .register_type::<DirectionalLight>()
        .register_type::<PointLight>()
        .register_type::<SpotLight>();
}

/// Tweaks XR camera settings to avoid right-eye flicker artifacts.
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

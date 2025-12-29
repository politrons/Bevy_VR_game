use bevy::ecs::message::MessageWriter;
use bevy::ecs::world::World;
use bevy::gltf::{Gltf, GltfLoaderSettings, GltfMesh};
use bevy::math::Isometry3d;
use bevy::prelude::*;
use bevy_mod_openxr::{
    action_binding::{OxrSendActionBindings, OxrSuggestActionBinding},
    action_set_attaching::OxrAttachActionSet,
    action_set_syncing::{OxrActionSetSyncSet, OxrSyncActionSet},
    openxr_session_running,
    resources::OxrInstance,
    session::OxrSession,
};
use bevy_mod_xr::session::{XrPreDestroySession, XrSessionCreated};
use bevy_xr_utils::actions::{
    ActionType, ActiveSet, XRUtilsAction, XRUtilsActionSet, XRUtilsBinding,
};
use openxr::Posef;

use crate::player::{JumpAction, MoveAction, PushAction, ShootAction, SprintAction, TurnAction};

const RIGHT_CONTROLLER_MODEL_SCALE: f32 = 0.36;
const RIGHT_CONTROLLER_MODEL_PITCH_RAD: f32 = -std::f32::consts::FRAC_PI_2;
const RIGHT_CONTROLLER_MODEL_YAW_RAD: f32 = 0.0;
const RIGHT_CONTROLLER_MODEL_ROLL_RAD: f32 = 0.0;

#[derive(Resource, Default)]
pub(crate) struct RightControllerModelStatus {
    pub(crate) spawned: bool,
    pub(crate) failed: bool,
}

/// Spawns XR action entities for locomotion and gameplay.
///
/// This builds an `XRUtilsActionSet` (marked as `ActiveSet`) and attaches bindings
/// for Quest Touch controllers.
///
/// Key design notes:
/// - `ActionType::Vector` is used for thumbsticks.
/// - `ActionType::Bool` is used for click-like inputs.
/// - We use `ActionType::Float` for analog inputs (trigger values).
///
/// If inputs appear "dead" on device, the most common causes are:
/// - the set is not marked `ActiveSet`, or
/// - the binding paths don't exist for the runtime/controller profile.
///
/// So we keep bindings conservative (thumbsticks + click inputs).
pub fn create_action_entities(mut commands: Commands) {
    log::info!("bevy_quest_hello::controller: Spawning XR action entities (locomotion + sprint)");

    // One set for all actions (MUST be ActiveSet).
    let set = commands
        .spawn((
            XRUtilsActionSet {
                name: "locomotion".into(),
                pretty_name: "Locomotion Set".into(),
                // Keep this simple; u32::MIN == 0 anyway.
                priority: 0,
            },
            ActiveSet,
        ))
        .id();

    // LEFT stick: move (strafe + forward/back).
    let move_action = commands
        .spawn((
            XRUtilsAction {
                action_name: "move_input".into(),
                localized_name: "Move".into(),
                action_type: ActionType::Vector,
            },
            MoveAction,
        ))
        .id();

    let move_binding_left_oculus = commands
        .spawn(XRUtilsBinding {
            profile: "/interaction_profiles/oculus/touch_controller".into(),
            binding: "/user/hand/left/input/thumbstick".into(),
        })
        .id();

    let move_binding_left_meta = commands
        .spawn(XRUtilsBinding {
            profile: "/interaction_profiles/meta/touch_controller".into(),
            binding: "/user/hand/left/input/thumbstick".into(),
        })
        .id();

    commands.entity(move_action).add_child(move_binding_left_oculus);
    commands.entity(move_action).add_child(move_binding_left_meta);

    // RIGHT stick: rotate.
    let turn_action = commands
        .spawn((
            XRUtilsAction {
                action_name: "turn_input".into(),
                localized_name: "Turn".into(),
                action_type: ActionType::Vector,
            },
            TurnAction,
        ))
        .id();

    let turn_binding_right_oculus = commands
        .spawn(XRUtilsBinding {
            profile: "/interaction_profiles/oculus/touch_controller".into(),
            binding: "/user/hand/right/input/thumbstick".into(),
        })
        .id();

    let turn_binding_right_meta = commands
        .spawn(XRUtilsBinding {
            profile: "/interaction_profiles/meta/touch_controller".into(),
            binding: "/user/hand/right/input/thumbstick".into(),
        })
        .id();

    commands.entity(turn_action).add_child(turn_binding_right_oculus);
    commands.entity(turn_action).add_child(turn_binding_right_meta);

    // Jump: right controller A button.
    let jump_action = commands
        .spawn((
            XRUtilsAction {
                action_name: "jump".into(),
                localized_name: "Jump".into(),
                action_type: ActionType::Bool,
            },
            JumpAction,
        ))
        .id();

    // NOTE: do NOT bind Jump to `.../a/touch`.
    // `touch` becomes true when your thumb rests on the button, which makes Jump look
    // "always pressed" and breaks edge-triggered jump logic ("just pressed").
    let jump_binding_a_click_oculus = commands
        .spawn(XRUtilsBinding {
            profile: "/interaction_profiles/oculus/touch_controller".into(),
            binding: "/user/hand/right/input/a/click".into(),
        })
        .id();

    let jump_binding_a_click_meta = commands
        .spawn(XRUtilsBinding {
            profile: "/interaction_profiles/meta/touch_controller".into(),
            binding: "/user/hand/right/input/a/click".into(),
        })
        .id();

    commands.entity(jump_action).add_child(jump_binding_a_click_oculus);
    commands.entity(jump_action).add_child(jump_binding_a_click_meta);

    // Push: right controller B button.
    //
    // IMPORTANT: use `/click` (not `/touch`) to avoid “always pressed” behavior.
    let push_action = commands
        .spawn((
            XRUtilsAction {
                action_name: "push".into(),
                localized_name: "Push".into(),
                action_type: ActionType::Bool,
            },
            PushAction,
        ))
        .id();

    let push_binding_b_click_oculus = commands
        .spawn(XRUtilsBinding {
            profile: "/interaction_profiles/oculus/touch_controller".into(),
            binding: "/user/hand/right/input/b/click".into(),
        })
        .id();

    let push_binding_b_click_meta = commands
        .spawn(XRUtilsBinding {
            profile: "/interaction_profiles/meta/touch_controller".into(),
            binding: "/user/hand/right/input/b/click".into(),
        })
        .id();

    commands.entity(push_action).add_child(push_binding_b_click_oculus);
    commands.entity(push_action).add_child(push_binding_b_click_meta);

    // Shoot: right trigger value (upper trigger).
    let shoot_action = commands
        .spawn((
            XRUtilsAction {
                action_name: "shoot".into(),
                localized_name: "Shoot".into(),
                action_type: ActionType::Float,
            },
            ShootAction,
        ))
        .id();

    let shoot_binding_trigger_value_oculus = commands
        .spawn(XRUtilsBinding {
            profile: "/interaction_profiles/oculus/touch_controller".into(),
            binding: "/user/hand/right/input/trigger/value".into(),
        })
        .id();

    let shoot_binding_trigger_value_meta = commands
        .spawn(XRUtilsBinding {
            profile: "/interaction_profiles/meta/touch_controller".into(),
            binding: "/user/hand/right/input/trigger/value".into(),
        })
        .id();

    commands
        .entity(shoot_action)
        .add_child(shoot_binding_trigger_value_oculus);
    commands
        .entity(shoot_action)
        .add_child(shoot_binding_trigger_value_meta);

    // Sprint: use LEFT thumbstick click (very reliable Bool binding).
    //
    // You previously asked for the "upper trigger" (left grip). That path is runtime-dependent
    // and may not exist as a click boolean, so keep this stable first.
    let sprint_action = commands
        .spawn((
            XRUtilsAction {
                action_name: "sprint".into(),
                localized_name: "Sprint".into(),
                action_type: ActionType::Bool,
            },
            SprintAction,
        ))
        .id();

    let sprint_binding_left_stick_click_oculus = commands
        .spawn(XRUtilsBinding {
            profile: "/interaction_profiles/oculus/touch_controller".into(),
            binding: "/user/hand/left/input/thumbstick/click".into(),
        })
        .id();

    let sprint_binding_left_stick_click_meta = commands
        .spawn(XRUtilsBinding {
            profile: "/interaction_profiles/meta/touch_controller".into(),
            binding: "/user/hand/left/input/thumbstick/click".into(),
        })
        .id();

    commands.entity(sprint_action).add_child(sprint_binding_left_stick_click_oculus);
    commands.entity(sprint_action).add_child(sprint_binding_left_stick_click_meta);


    // Put all actions in the same set.
    commands.entity(set).add_child(move_action);
    commands.entity(set).add_child(turn_action);
    commands.entity(set).add_child(jump_action);
    commands.entity(set).add_child(push_action);
    commands.entity(set).add_child(shoot_action);
    commands.entity(set).add_child(sprint_action);
}

// -------------------------
// Controller cubes (OpenXR grip poses)
// -------------------------

/// Registers systems for controller pose actions and cubes.
pub(crate) fn register_controller_cubes(app: &mut App) {
    app.add_systems(Startup, setup_controller_cubes)
        .add_systems(OxrSendActionBindings, suggest_controller_bindings)
        .add_systems(
            XrSessionCreated,
            create_controller_actions
                .before(attach_controller_action_set)
                .before(spawn_controller_cubes),
        )
        .add_systems(XrSessionCreated, attach_controller_action_set)
        .add_systems(XrSessionCreated, spawn_controller_cubes)
        .add_systems(
            PreUpdate,
            sync_controller_action_set
                .before(OxrActionSetSyncSet)
                .run_if(openxr_session_running),
        )
        .add_systems(Update, swap_right_controller_model.run_if(openxr_session_running))
        .add_systems(XrPreDestroySession, cleanup_controller_cubes);
}

#[derive(Resource)]
pub(crate) struct ControllerActions {
    pub(crate) set: openxr::ActionSet,
    pub(crate) left_pose: openxr::Action<Posef>,
    pub(crate) right_pose: openxr::Action<Posef>,
    pub(crate) left_aim_pose: openxr::Action<Posef>,
    pub(crate) right_aim_pose: openxr::Action<Posef>,
}

#[derive(Resource, Clone)]
pub(crate) struct ControllerCubeAssets {
    pub(crate) mesh: Handle<Mesh>,
    pub(crate) left_material: Handle<StandardMaterial>,
    pub(crate) right_material: Handle<StandardMaterial>,
    pub(crate) gun_material: Handle<StandardMaterial>,
    pub(crate) right_gltf: Handle<Gltf>,
}

#[derive(Component)]
pub(crate) struct ControllerCube;

#[derive(Component)]
pub(crate) struct LeftController;

#[derive(Component)]
pub(crate) struct RightController;

#[derive(Component)]
pub(crate) struct ControllerAim;

#[derive(Component)]
pub(crate) struct RightControllerPlaceholder;

#[derive(Component)]
pub(crate) struct RightControllerModel;

#[derive(Component)]
pub(crate) struct LeftControllerAim;

#[derive(Component)]
pub(crate) struct RightControllerAim;

/// Creates shared mesh/material assets for controller cubes.
pub(crate) fn setup_controller_cubes(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mesh = meshes.add(Cuboid::new(0.09, 0.09, 0.09));
    let left_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.15, 0.75, 0.25),
        emissive: Color::srgb(0.05, 0.35, 0.08).into(),
        unlit: true,
        ..default()
    });
    let right_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.20, 0.35, 0.90),
        emissive: Color::srgb(0.06, 0.12, 0.40).into(),
        unlit: true,
        ..default()
    });

    let gun_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.65, 0.65, 0.65),
        emissive: Color::srgb(0.12, 0.12, 0.12).into(),
        unlit: true,
        ..default()
    });

    let right_gltf: Handle<Gltf> = asset_server.load_with_settings(
        "textures/gun.glb",
        |settings: &mut GltfLoaderSettings| {
            settings.load_cameras = false;
            settings.load_lights = false;
            settings.load_animations = false;
        },
    );

    commands.insert_resource(ControllerCubeAssets {
        mesh,
        left_material,
        right_material,
        gun_material,
        right_gltf,
    });
    commands.insert_resource(RightControllerModelStatus::default());
}

/// Creates the OpenXR action set and pose actions once per app.
pub(crate) fn create_controller_actions(world: &mut World) {
    if world.contains_resource::<ControllerActions>() {
        return;
    }
    let Some(instance) = world.get_resource::<OxrInstance>() else {
        return;
    };

    let set = match instance.create_action_set("controllers", "Controllers", 0) {
        Ok(set) => set,
        Err(err) => {
            log::warn!("Unable to create controller action set: {}", err);
            return;
        }
    };

    let left_pose = match set.create_action("left_grip_pose", "Left Grip Pose", &[]) {
        Ok(action) => action,
        Err(err) => {
            log::warn!("Unable to create left pose action: {}", err);
            return;
        }
    };
    let right_pose = match set.create_action("right_grip_pose", "Right Grip Pose", &[]) {
        Ok(action) => action,
        Err(err) => {
            log::warn!("Unable to create right pose action: {}", err);
            return;
        }
    };
    let left_aim_pose = match set.create_action("left_aim_pose", "Left Aim Pose", &[]) {
        Ok(action) => action,
        Err(err) => {
            log::warn!("Unable to create left aim pose action: {}", err);
            return;
        }
    };
    let right_aim_pose = match set.create_action("right_aim_pose", "Right Aim Pose", &[]) {
        Ok(action) => action,
        Err(err) => {
            log::warn!("Unable to create right aim pose action: {}", err);
            return;
        }
    };

    world.insert_resource(ControllerActions {
        set,
        left_pose,
        right_pose,
        left_aim_pose,
        right_aim_pose,
    });
}

/// Suggests grip-pose bindings for Oculus/Meta Touch controllers.
pub(crate) fn suggest_controller_bindings(
    actions: Option<Res<ControllerActions>>,
    mut bindings: MessageWriter<OxrSuggestActionBinding>,
) {
    let Some(actions) = actions else {
        return;
    };
    for profile in [
        "/interaction_profiles/oculus/touch_controller",
        "/interaction_profiles/meta/touch_controller",
    ] {
        bindings.write(OxrSuggestActionBinding {
            action: actions.left_pose.as_raw(),
            interaction_profile: profile.into(),
            bindings: vec!["/user/hand/left/input/grip/pose".into()],
        });
        bindings.write(OxrSuggestActionBinding {
            action: actions.right_pose.as_raw(),
            interaction_profile: profile.into(),
            bindings: vec!["/user/hand/right/input/grip/pose".into()],
        });
        bindings.write(OxrSuggestActionBinding {
            action: actions.left_aim_pose.as_raw(),
            interaction_profile: profile.into(),
            bindings: vec!["/user/hand/left/input/aim/pose".into()],
        });
        bindings.write(OxrSuggestActionBinding {
            action: actions.right_aim_pose.as_raw(),
            interaction_profile: profile.into(),
            bindings: vec!["/user/hand/right/input/aim/pose".into()],
        });
    }
}

/// Attaches the controller action set when the session is created.
pub(crate) fn attach_controller_action_set(
    actions: Option<Res<ControllerActions>>,
    mut attach: MessageWriter<OxrAttachActionSet>,
) {
    let Some(actions) = actions else {
        return;
    };
    attach.write(OxrAttachActionSet(actions.set.clone()));
}

/// Syncs the controller action set each frame while running.
pub(crate) fn sync_controller_action_set(
    actions: Option<Res<ControllerActions>>,
    mut sync: MessageWriter<OxrSyncActionSet>,
) {
    let Some(actions) = actions else {
        return;
    };
    sync.write(OxrSyncActionSet(actions.set.clone()));
}

/// Spawns cube entities bound to controller pose spaces.
pub(crate) fn spawn_controller_cubes(
    actions: Option<Res<ControllerActions>>,
    assets: Option<Res<ControllerCubeAssets>>,
    mut model_status: Option<ResMut<RightControllerModelStatus>>,
    session: Res<OxrSession>,
    mut commands: Commands,
    existing: Query<Entity, With<ControllerCube>>,
) {
    let (Some(actions), Some(assets)) = (actions, assets) else {
        return;
    };
    if let Some(status) = model_status.as_mut() {
        status.spawned = false;
        status.failed = false;
    }
    for entity in &existing {
        commands.entity(entity).despawn();
    }

    let left_space = match session.create_action_space(
        &actions.left_pose,
        openxr::Path::NULL,
        Isometry3d::IDENTITY,
    ) {
        Ok(space) => space,
        Err(err) => {
            log::warn!("Unable to create left controller space: {}", err);
            return;
        }
    };
    let right_space = match session.create_action_space(
        &actions.right_pose,
        openxr::Path::NULL,
        Isometry3d::IDENTITY,
    ) {
        Ok(space) => space,
        Err(err) => {
            log::warn!("Unable to create right controller space: {}", err);
            return;
        }
    };
    let left_aim_space = match session.create_action_space(
        &actions.left_aim_pose,
        openxr::Path::NULL,
        Isometry3d::IDENTITY,
    ) {
        Ok(space) => space,
        Err(err) => {
            log::warn!("Unable to create left aim space: {}", err);
            return;
        }
    };
    let right_aim_space = match session.create_action_space(
        &actions.right_aim_pose,
        openxr::Path::NULL,
        Isometry3d::IDENTITY,
    ) {
        Ok(space) => space,
        Err(err) => {
            log::warn!("Unable to create right aim space: {}", err);
            return;
        }
    };

    commands.spawn((
        ControllerCube,
        LeftController,
        left_space,
        Mesh3d(assets.mesh.clone()),
        MeshMaterial3d(assets.left_material.clone()),
        Name::new("LeftControllerCube"),
    ));

    let right_root = commands.spawn((
        ControllerCube,
        RightController,
        right_space,
        Name::new("RightControllerRoot"),
    )).id();

    let placeholder = commands
        .spawn((
            ControllerCube,
            RightControllerPlaceholder,
            Mesh3d(assets.mesh.clone()),
            MeshMaterial3d(assets.right_material.clone()),
            Name::new("RightControllerPlaceholder"),
        ))
        .id();
    commands.entity(right_root).add_child(placeholder);

    commands.spawn((
        ControllerAim,
        LeftControllerAim,
        left_aim_space,
        Name::new("LeftControllerAim"),
    ));
    commands.spawn((
        ControllerAim,
        RightControllerAim,
        right_aim_space,
        Name::new("RightControllerAim"),
    ));
}

/// Swaps the right controller placeholder cube for the loaded GLB scene.
pub(crate) fn swap_right_controller_model(
    assets: Option<Res<ControllerCubeAssets>>,
    asset_server: Res<AssetServer>,
    gltfs: Res<Assets<Gltf>>,
    gltf_meshes: Res<Assets<GltfMesh>>,
    status: Option<ResMut<RightControllerModelStatus>>,
    right_roots: Query<Entity, With<RightController>>,
    placeholders: Query<Entity, With<RightControllerPlaceholder>>,
    mut commands: Commands,
) {
    let (Some(assets), Some(mut status)) = (assets, status) else {
        return;
    };
    if status.spawned || status.failed {
        return;
    }

    let Some(state) = asset_server.get_load_state(assets.right_gltf.id()) else {
        return;
    };
    match state {
        bevy::asset::LoadState::Loaded => {
            let Some(gltf) = gltfs.get(&assets.right_gltf) else {
                return;
            };
            if gltf.meshes.is_empty() {
                log::warn!("Right controller model has no meshes.");
                status.failed = true;
                return;
            }
            let mut primitives = Vec::new();
            for mesh_handle in &gltf.meshes {
                let Some(gltf_mesh) = gltf_meshes.get(mesh_handle) else {
                    return;
                };
                for primitive in &gltf_mesh.primitives {
                    let material = primitive
                        .material
                        .clone()
                        .unwrap_or_else(|| assets.gun_material.clone());
                    primitives.push((primitive.mesh.clone(), material));
                }
            }
            if primitives.is_empty() {
                log::warn!("Right controller model has no mesh primitives.");
                status.failed = true;
                return;
            }
            let mut spawned_any = false;
            for root in &right_roots {
                let model = commands
                    .spawn((
                        ControllerCube,
                        RightControllerModel,
                        Transform {
                            rotation: Quat::from_euler(
                                EulerRot::XYZ,
                                RIGHT_CONTROLLER_MODEL_PITCH_RAD,
                                RIGHT_CONTROLLER_MODEL_YAW_RAD,
                                RIGHT_CONTROLLER_MODEL_ROLL_RAD,
                            ),
                            scale: Vec3::splat(RIGHT_CONTROLLER_MODEL_SCALE),
                            ..default()
                        },
                        Name::new("RightControllerModel"),
                    ))
                    .id();
                commands.entity(model).with_children(|parent| {
                    for (mesh, material) in &primitives {
                        parent.spawn((
                            Mesh3d(mesh.clone()),
                            MeshMaterial3d(material.clone()),
                        ));
                    }
                });
                commands.entity(root).add_child(model);
                spawned_any = true;
            }
            if !spawned_any {
                return;
            }
            for placeholder in &placeholders {
                commands.entity(placeholder).despawn();
            }
            status.spawned = true;
        }
        bevy::asset::LoadState::Failed(err) => {
            log::warn!("Right controller model failed to load: {:?}", err);
            status.failed = true;
        }
        _ => {}
    }
}

/// Despawns controller cubes when the session ends.
pub(crate) fn cleanup_controller_cubes(
    mut commands: Commands,
    cubes: Query<Entity, With<ControllerCube>>,
    aims: Query<Entity, With<ControllerAim>>,
    mut model_status: Option<ResMut<RightControllerModelStatus>>,
) {
    for entity in &cubes {
        commands.entity(entity).despawn();
    }
    for entity in &aims {
        commands.entity(entity).despawn();
    }
    if let Some(status) = model_status.as_mut() {
        status.spawned = false;
        status.failed = false;
    }
}

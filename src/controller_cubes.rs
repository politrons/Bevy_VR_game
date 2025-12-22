use bevy::prelude::*;
use bevy::ecs::message::MessageWriter;
use bevy::ecs::world::World;
use bevy::math::Isometry3d;
use bevy_mod_openxr::{
    action_binding::{OxrSendActionBindings, OxrSuggestActionBinding},
    action_set_attaching::OxrAttachActionSet,
    action_set_syncing::{OxrActionSetSyncSet, OxrSyncActionSet},
    openxr_session_running,
    resources::OxrInstance,
    session::OxrSession,
};
use bevy_mod_xr::session::{XrPreDestroySession, XrSessionCreated};
use openxr::Posef;

#[derive(Resource)]
pub(crate) struct ControllerActions {
    pub(crate) set: openxr::ActionSet,
    pub(crate) left_pose: openxr::Action<Posef>,
    pub(crate) right_pose: openxr::Action<Posef>,
}

#[derive(Resource, Clone)]
pub(crate) struct ControllerCubeAssets {
    pub(crate) mesh: Handle<Mesh>,
    pub(crate) left_material: Handle<StandardMaterial>,
    pub(crate) right_material: Handle<StandardMaterial>,
}

#[derive(Component)]
pub(crate) struct ControllerCube;

pub(crate) fn setup_controller_cubes(
    mut commands: Commands,
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

    commands.insert_resource(ControllerCubeAssets {
        mesh,
        left_material,
        right_material,
    });
}

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

    world.insert_resource(ControllerActions {
        set,
        left_pose,
        right_pose,
    });
}

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
    }
}

pub(crate) fn attach_controller_action_set(
    actions: Option<Res<ControllerActions>>,
    mut attach: MessageWriter<OxrAttachActionSet>,
) {
    let Some(actions) = actions else {
        return;
    };
    attach.write(OxrAttachActionSet(actions.set.clone()));
}

pub(crate) fn sync_controller_action_set(
    actions: Option<Res<ControllerActions>>,
    mut sync: MessageWriter<OxrSyncActionSet>,
) {
    let Some(actions) = actions else {
        return;
    };
    sync.write(OxrSyncActionSet(actions.set.clone()));
}

pub(crate) fn spawn_controller_cubes(
    actions: Option<Res<ControllerActions>>,
    assets: Option<Res<ControllerCubeAssets>>,
    session: Res<OxrSession>,
    mut commands: Commands,
    existing: Query<Entity, With<ControllerCube>>,
) {
    let (Some(actions), Some(assets)) = (actions, assets) else {
        return;
    };
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

    commands.spawn((
        ControllerCube,
        left_space,
        Mesh3d(assets.mesh.clone()),
        MeshMaterial3d(assets.left_material.clone()),
        Name::new("LeftControllerCube"),
    ));

    commands.spawn((
        ControllerCube,
        right_space,
        Mesh3d(assets.mesh.clone()),
        MeshMaterial3d(assets.right_material.clone()),
        Name::new("RightControllerCube"),
    ));
}

pub(crate) fn cleanup_controller_cubes(
    mut commands: Commands,
    cubes: Query<Entity, With<ControllerCube>>,
) {
    for entity in &cubes {
        commands.entity(entity).despawn();
    }
}

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
        .add_systems(XrPreDestroySession, cleanup_controller_cubes);
}

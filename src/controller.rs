use bevy::prelude::*;
use bevy_xr_utils::actions::{
    ActionType, ActiveSet, XRUtilsAction, XRUtilsActionSet, XRUtilsBinding,
};
use log::info;
use crate::player::{JumpAction, MoveAction, SprintAction, TurnAction};

/// Spawns XR action entities and bindings for locomotion.
///
/// This wires Meta Quest Touch controller inputs to your marker components:
/// - `MoveAction`: left thumbstick (Vector2)
/// - `TurnAction`: right thumbstick (Vector2)
/// - `JumpAction`: right A button click (Bool)
/// - `SprintAction`: left upper trigger/grip (Bool)
///
/// The action set is spawned with `ActiveSet` so XR utils will generate input events.
pub fn create_action_entities(mut commands: Commands) {
    info!("Spawning XR action entities (locomotion + sprint)");
    let set = commands
        .spawn((
            XRUtilsActionSet {
                name: "locomotion".into(),
                pretty_name: "Locomotion Set".into(),
                priority: 0,
            },
            ActiveSet,
        ))
        .id();

    // LEFT stick: strafe + forward/back.
    let move_action = commands
        .spawn((
            XRUtilsAction {
                action_name: "move_input".into(),
                localized_name: "move_input".into(),
                action_type: ActionType::Vector,
            },
            MoveAction,
        ))
        .id();

    let move_binding_left = commands
        .spawn(XRUtilsBinding {
            profile: "/interaction_profiles/oculus/touch_controller".into(),
            binding: "/user/hand/left/input/thumbstick".into(),
        })
        .id();

    commands.entity(move_action).add_child(move_binding_left);

    // RIGHT stick: rotate.
    let turn_action = commands
        .spawn((
            XRUtilsAction {
                action_name: "turn_input".into(),
                localized_name: "turn_input".into(),
                action_type: ActionType::Vector,
            },
            TurnAction,
        ))
        .id();

    let turn_binding_right = commands
        .spawn(XRUtilsBinding {
            profile: "/interaction_profiles/oculus/touch_controller".into(),
            binding: "/user/hand/right/input/thumbstick".into(),
        })
        .id();

    commands.entity(turn_action).add_child(turn_binding_right);

    // Jump: right controller A button.
    let jump_action = commands
        .spawn((
            XRUtilsAction {
                action_name: "jump".into(),
                localized_name: "jump".into(),
                action_type: ActionType::Bool,
            },
            JumpAction,
        ))
        .id();

    let jump_binding_a = commands
        .spawn(XRUtilsBinding {
            profile: "/interaction_profiles/oculus/touch_controller".into(),
            binding: "/user/hand/right/input/a/click".into(),
        })
        .id();

    commands.entity(jump_action).add_child(jump_binding_a);

    // Sprint: left thumbstick click (reliable Bool across Touch runtimes).
    let sprint_action = commands
        .spawn((
            XRUtilsAction {
                action_name: "sprint".into(),
                localized_name: "sprint".into(),
                action_type: ActionType::Bool,
            },
            SprintAction,
        ))
        .id();

    let sprint_binding_left_thumbstick_click = commands
        .spawn(XRUtilsBinding {
            profile: "/interaction_profiles/oculus/touch_controller".into(),
            binding: "/user/hand/left/input/thumbstick/click".into(),
        })
        .id();

    commands.entity(sprint_action).add_child(sprint_binding_left_thumbstick_click);

    // Put all actions in the same set.
    commands.entity(set).add_child(move_action);
    commands.entity(set).add_child(turn_action);
    commands.entity(set).add_child(jump_action);
    commands.entity(set).add_child(sprint_action);
}
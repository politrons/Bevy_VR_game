use bevy::prelude::*;
use bevy_xr_utils::actions::{
    ActionType, ActiveSet, XRUtilsAction, XRUtilsActionSet, XRUtilsBinding,
};

use crate::player::{JumpAction, MoveAction, SprintAction, TurnAction};

/// Spawns XR action entities for locomotion and gameplay.
///
/// This builds an `XRUtilsActionSet` (marked as `ActiveSet`) and attaches bindings
/// for Quest Touch controllers.
///
/// Key design notes:
/// - `ActionType::Vector` is used for thumbsticks.
/// - `ActionType::Bool` is used for click-like inputs.
/// - This project/version does NOT support `ActionType::Scalar`.
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
    commands.entity(set).add_child(sprint_action);
}
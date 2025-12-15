use bevy::prelude::*;
use bevy_mod_openxr::{add_xr_plugins, openxr_session_running, helper_traits::ToQuat, resources::OxrViews};
use bevy_mod_xr::session::XrTrackingRoot;
use bevy_xr_utils::actions::{
    ActionType, ActiveSet, XRUtilsAction, XRUtilsActionSet, XRUtilsActionState,
    XRUtilsActionSystems, XRUtilsActionsPlugin, XRUtilsBinding,
};

#[derive(Component)]
struct LocomotionAction;

/// Configura la aplicación con plugins XR y el plugin de acciones.
pub fn run() {
    App::new()
        .add_plugins(add_xr_plugins(DefaultPlugins))
        .add_plugins(bevy_mod_xr::hand_debug_gizmos::HandGizmosPlugin)
        .add_plugins(XRUtilsActionsPlugin) // plugin de acciones
        .add_systems(Startup, setup_scene)
        .add_systems(Startup, create_action_entities.before(XRUtilsActionSystems::CreateEvents))
        .add_systems(Update, handle_locomotion.run_if(openxr_session_running))
        .run();
}

#[bevy_main]
fn main() {
    run();
}

/// Crea el suelo y la luz.
fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(10.0, 0.1, 10.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.2, 0.5, 0.3))),
        Transform::from_xyz(0.0, -0.05, 0.0),
    ));
    commands.spawn((
        PointLight {
            intensity: 2000.0,
            range: 50.0,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0),
    ));
}

/// Crea el conjunto y la acción de locomoción, y los bindings para los mandos Quest.
fn create_action_entities(mut commands: Commands) {
    // Conjunto de acciones
    let set = commands
        .spawn((
            XRUtilsActionSet {
                name: "locomotion".into(),
                pretty_name: "Locomotion Set".into(),
                priority: u32::MIN,
            },
            ActiveSet,
        ))
        .id();

    // Acción vectorial para el movimiento
    let action = commands
        .spawn((
            XRUtilsAction {
                action_name: "move_input".into(),
                localized_name: "move_input".into(),
                action_type: ActionType::Vector,
            },
            LocomotionAction,
        ))
        .id();

    // Bindings para el thumbstick del mando derecho e izquierdo de Oculus/Meta [oai_citation:0‡raw.githubusercontent.com](https://raw.githubusercontent.com/awtterpip/bevy_oxr/main/crates/bevy_xr_utils/examples/actions.rs#:~:text=%2F%2Fcreate%20a%20binding%20let%20binding_index,into%28%29%2C).
    let binding_left = commands
        .spawn(XRUtilsBinding {
            profile: "/interaction_profiles/oculus/touch_controller".into(),
            binding: "/user/hand/left/input/thumbstick".into(),
        })
        .id();
    let binding_right = commands
        .spawn(XRUtilsBinding {
            profile: "/interaction_profiles/oculus/touch_controller".into(),
            binding: "/user/hand/right/input/thumbstick".into(),
        })
        .id();

    // Asociar bindings a la acción y la acción al set.
    commands.entity(action).add_child(binding_left);
    commands.entity(action).add_child(binding_right);
    commands.entity(set).add_child(action);
}

/// Lee el valor de la acción de locomoción y mueve el XrTrackingRoot.
fn handle_locomotion(
    action_query: Query<&XRUtilsActionState, With<LocomotionAction>>,
    mut oxr_root: Query<&mut Transform, With<XrTrackingRoot>>,
    time: Res<Time>,
    mut views: ResMut<OxrViews>,
) {
    for state in action_query.iter() {
        if let XRUtilsActionState::Vector(vector_state) = state {
            // El vector de entrada es [x, y] en la estructura current_state
            let input_vector = Vec3::new(
                vector_state.current_state[0],
                0.0,
                -vector_state.current_state[1],
            );
            // Velocidad de movimiento
            let speed = 3.0;

            if let Ok(mut root_pos) = oxr_root.single_mut() {
                // Usamos la orientación del visor para mover en la dirección correcta
                if let Some(view) = views.first() {
                    let reference_quat =
                        root_pos.rotation * view.pose.orientation.to_quat();
                    let locomotion = reference_quat.mul_vec3(input_vector);
                    root_pos.translation += locomotion * speed * time.delta_secs();
                }
            }
        }
    }
}
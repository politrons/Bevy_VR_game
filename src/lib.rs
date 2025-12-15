use bevy::camera::ClearColorConfig;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::prelude::*;
use bevy::render::view::{Hdr, Msaa, NoIndirectDrawing};

use bevy_mod_openxr::{add_xr_plugins, openxr_session_running, resources::OxrViews};
use bevy_mod_xr::camera::{XrCamera, XrViewInit};
use bevy_mod_xr::session::XrSessionCreated;

use bevy_xr_utils::actions::{
    ActionType, ActiveSet, XRUtilsAction, XRUtilsActionSet, XRUtilsActionState,
    XRUtilsActionSystems, XRUtilsActionsPlugin, XRUtilsBinding,
};

use log::info;

#[derive(Component)]
struct LocomotionAction;

#[bevy_main]
fn main() {
    let mut app = App::new()
        .insert_resource(ClearColor(Color::BLACK))
        .add_plugins(add_xr_plugins(DefaultPlugins))
        .add_systems(Startup, setup_scene)
        .add_systems(XrSessionCreated, tune_xr_cameras.after(XrViewInit))
        .add_plugins(XRUtilsActionsPlugin)
        .add_systems(Startup, create_action_entities.before(XRUtilsActionSystems::CreateEvents), )
        .add_systems(Update, handle_locomotion.run_if(openxr_session_running))
        .add_systems(Update, log_stereo_state.run_if(openxr_session_running))
        .run();
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cube_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.2, 0.7, 1.0),
        unlit: true,
        ..default()
    });

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.4, 0.4, 0.4))),
        MeshMaterial3d(cube_mat),
        Transform::from_xyz(0.0, 1.2, -2.0),
    ));

    let origin_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.2, 0.2),
        unlit: true,
        ..default()
    });

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.06, 0.06, 0.06))),
        MeshMaterial3d(origin_mat),
        Transform::from_xyz(0.0, 1.6, 0.0),
    ));
}

fn tune_xr_cameras(mut commands: Commands, mut cams: Query<(Entity, &mut Camera, &XrCamera)>) {
    for (e, mut cam, xr_cam) in &mut cams {
        cam.is_active = true;

        cam.clear_color = ClearColorConfig::Custom(Color::BLACK);
        commands.entity(e).remove::<Hdr>();
        commands.entity(e).insert(Tonemapping::None);
        commands.entity(e).insert(Msaa::Off);
        // Critical change: opt out of indirect drawing on BOTH XR cameras.
        commands.entity(e).insert(NoIndirectDrawing);

        info!("tuned xr cam eye={} entity={:?}", xr_cam.0, e);
    }
}

fn create_action_entities(mut commands: Commands) {
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

    commands.entity(action).add_child(binding_left);
    commands.entity(action).add_child(binding_right);
    commands.entity(set).add_child(action);
}

fn handle_locomotion(
    action_query: Query<&XRUtilsActionState, With<LocomotionAction>>,
    mut xr_root: Query<&mut Transform, With<bevy_mod_xr::session::XrTrackingRoot>>,
    time: Res<Time>,
    views: Res<OxrViews>,
) {
    let Ok(mut root) = xr_root.single_mut() else {
        return;
    };

    for state in &action_query {
        let XRUtilsActionState::Vector(vector_state) = state else {
            continue;
        };

        let input = Vec3::new(
            vector_state.current_state[0],
            0.0,
            -vector_state.current_state[1],
        );
        let speed = 2.0;

        let yaw = views
            .first()
            .map(|v| {
                let q = v.pose.orientation;
                let q = Quat::from_xyzw(q.x, q.y, q.z, q.w);
                let (y, _, _) = q.to_euler(EulerRot::YXZ);
                Quat::from_rotation_y(y)
            })
            .unwrap_or(Quat::IDENTITY);

        root.translation += yaw.mul_vec3(input) * speed * time.delta_secs();
    }
}

fn log_stereo_state(
    time: Res<Time>,
    views: Res<OxrViews>,
    cams: Query<(&XrCamera, &Camera, Option<&GlobalTransform>)>,
    mut acc: Local<f32>,
) {
    *acc += time.delta_secs();
    if *acc < 1.0 {
        return;
    }
    *acc = 0.0;

    if views.len() >= 2 {
        let l = &views[0];
        let r = &views[1];
        info!(
            "views: L=({:.4},{:.4},{:.4}) R=({:.4},{:.4},{:.4})",
            l.pose.position.x,
            l.pose.position.y,
            l.pose.position.z,
            r.pose.position.x,
            r.pose.position.y,
            r.pose.position.z
        );
    } else {
        info!("views: len={} (expected 2)", views.len());
    }

    for (xr_cam, cam, gtr) in &cams {
        let vp = cam
            .viewport
            .as_ref()
            .map(|v| (v.physical_position, v.physical_size));

        let pos = gtr.map(|g| g.translation()).unwrap_or(Vec3::NAN);

        info!(
            "cam: eye={} active={} viewport={:?} clear={:?} pos=({:.3},{:.3},{:.3})",
            xr_cam.0, cam.is_active, vp, cam.clear_color, pos.x, pos.y, pos.z
        );
    }
}

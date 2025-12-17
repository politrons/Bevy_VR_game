use bevy::camera::ClearColorConfig;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::prelude::*;
use bevy::render::view::{Hdr, Msaa, NoIndirectDrawing};
use bevy_mod_openxr::{add_xr_plugins, openxr_session_running, resources::OxrViews};
use bevy_mod_xr::camera::{XrCamera, XrViewInit};
use bevy_mod_xr::session::{XrSessionCreated, XrTrackingRoot};
use bevy_xr_utils::actions::{
    ActionType, ActiveSet, XRUtilsAction, XRUtilsActionSet, XRUtilsActionState,
    XRUtilsActionSystems, XRUtilsActionsPlugin, XRUtilsBinding,
};
use log::info;
use bevy_mod_xr::hand_debug_gizmos::HandGizmosPlugin;

#[derive(Component)]
struct MoveAction;

#[derive(Component)]
struct TurnAction;

#[derive(Component)]
struct JumpAction;

#[derive(Resource, Debug, Clone, Copy)]
struct LocomotionSettings {
    move_speed_mps: f32,
    turn_speed_rad_s: f32,
    stick_deadzone: f32,
    jump_velocity_mps: f32,
    gravity_mps2: f32,
}

impl Default for LocomotionSettings {
    fn default() -> Self {
        Self {
            move_speed_mps: 2.0,
            // ~120 deg/s
            turn_speed_rad_s: 120.0_f32.to_radians(),
            stick_deadzone: 0.15,
            jump_velocity_mps: 3.5,
            gravity_mps2: -9.81,
        }
    }
}

#[derive(Resource, Debug, Default)]
struct PlayerKinematics {
    vertical_velocity: f32,
}

#[derive(Resource, Debug, Clone, Copy)]
struct FloorParams {
    center: Vec3,
    half_extents: Vec2, // x and z half sizes
    top_y: f32,
    fall_floor_y: f32,
}

impl FloorParams {
    fn ground_y_at(&self, p: Vec3) -> f32 {
        let dx = (p.x - self.center.x).abs();
        let dz = (p.z - self.center.z).abs();
        if dx <= self.half_extents.x && dz <= self.half_extents.y {
            self.top_y
        } else {
            self.fall_floor_y
        }
    }

    fn is_on_floor(&self, pos: Vec3) -> bool {
        let dx = (pos.x - self.center.x).abs();
        let dz = (pos.z - self.center.z).abs();
        dx <= self.half_extents.x && dz <= self.half_extents.y
    }
}

#[bevy_main]
fn main() {
    let mut app = App::new()
        // Sky-blue background.
        .insert_resource(ClearColor(Color::srgb(0.53, 0.81, 0.92)))
        .insert_resource(LocomotionSettings::default())
        .insert_resource(PlayerKinematics::default())
        .add_plugins(add_xr_plugins(DefaultPlugins))
        .add_systems(Startup, setup_scene)
        .add_systems(XrSessionCreated, tune_xr_cameras.after(XrViewInit))
        .add_plugins(XRUtilsActionsPlugin)
        .add_systems(Startup, create_action_entities.before(XRUtilsActionSystems::CreateEvents))
        .add_systems(Update, handle_locomotion.run_if(openxr_session_running))
        .run();
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // A couple of reference cubes.
    let cube_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.2, 0.7, 1.0),
        unlit: true,
        ..default()
    });
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.4, 0.4, 0.4))),
        MeshMaterial3d(cube_mat),
        Transform::from_xyz(0.0, 16.2, -2.0),
    ));

    let origin_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.2, 0.2),
        unlit: true,
        ..default()
    });
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.06, 0.06, 0.06))),
        MeshMaterial3d(origin_mat),
        Transform::from_xyz(0.0, 16.6, 0.0),
    ));

    // A raised long floor with a big drop if you walk off the edges.
    // We build it as multiple meshes so the Y faces (top/bottom) can have a different color than the X/Z sides.
    let floor_len = 100.0;
    let floor_with = 20.0;
    let floor_thickness = 0.2;

    // Raise the floor high so looking over the edge feels like a cliff.
    let floor_top_y = 15.0;
    let floor_center = Vec3::new(0.0, floor_top_y - floor_thickness * 0.5, 0.0);

    let floor_top_bottom_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.25, 0.25, 0.25),
        unlit: true,
        ..default()
    });

    let floor_side_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.10, 0.10, 0.10),
        unlit: true,
        ..default()
    });

    // Top floor (gives us the top/bottom Y faces with the "floor" color).
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(floor_with, floor_thickness, floor_len))),
        MeshMaterial3d(floor_top_bottom_mat.clone()),
        Transform::from_translation(floor_center),
    ));

    // Side walls (X faces) with a darker color to make the cliff edge readable.
    let wall_h = floor_top_y; // extend down to y=0 (visual), but you can tweak this.
    let wall_center_y = floor_top_y * 0.5;

    // Left wall
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.3, wall_h, floor_len))),
        MeshMaterial3d(floor_side_mat.clone()),
        Transform::from_xyz(-floor_with * 0.5, wall_center_y, 0.0),
    ));
    // Right wall
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.3, wall_h, floor_len))),
        MeshMaterial3d(floor_side_mat.clone()),
        Transform::from_xyz(floor_with * 0.5, wall_center_y, 0.0),
    ));

    // End walls (Z faces)
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(floor_with, wall_h, 0.3))),
        MeshMaterial3d(floor_side_mat.clone()),
        Transform::from_xyz(0.0, wall_center_y, -floor_len * 0.5),
    ));
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(floor_with, wall_h, 0.3))),
        MeshMaterial3d(floor_side_mat.clone()),
        Transform::from_xyz(0.0, wall_center_y, floor_len * 0.5),
    ));

    // Dynamic ground: on-road uses the road top; off-road uses a very low floor so you "fall".
    commands.insert_resource(FloorParams {
        center: Vec3::new(0.0, 0.0, 0.0),
        half_extents: Vec2::new(floor_with * 0.5, floor_len * 0.5),
        top_y: floor_top_y,
        fall_floor_y: -200.0,
    });
}

fn tune_xr_cameras(mut commands: Commands, mut cams: Query<(Entity, &mut Camera, &XrCamera)>) {
    for (e, mut cam, xr_cam) in &mut cams {
        cam.is_active = true;
        cam.clear_color = ClearColorConfig::Custom(Color::srgb(0.53, 0.81, 0.92));
        commands.entity(e).remove::<Hdr>();
        commands.entity(e).insert(Tonemapping::None);
        commands.entity(e).insert(Msaa::Off);
        // Fix for the right-eye flicker: disable indirect drawing per camera.
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

    // RIGHT stick: rotate (smooth turn) around your head position.
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

    // Put all actions in the same set.
    commands.entity(set).add_child(move_action);
    commands.entity(set).add_child(turn_action);
    commands.entity(set).add_child(jump_action);
}

fn handle_locomotion(
    move_query: Query<&XRUtilsActionState, With<MoveAction>>,
    turn_query: Query<&XRUtilsActionState, With<TurnAction>>,
    jump_query: Query<&XRUtilsActionState, With<JumpAction>>,
    mut xr_root: Query<&mut Transform, With<XrTrackingRoot>>,
    time: Res<Time>,
    views: Res<OxrViews>,
    settings: Res<LocomotionSettings>,
    road: Res<FloorParams>,
    mut kin: ResMut<PlayerKinematics>,
) {
    let Ok(mut root) = xr_root.single_mut() else {
        return;
    };

    let dt = time.delta_secs();

    // Read action states.
    let move_xy = move_query
        .iter()
        .find_map(|s| match s {
            XRUtilsActionState::Vector(v) => Some(v.current_state),
            _ => None,
        })
        .unwrap_or([0.0, 0.0]);

    let turn_xy = turn_query
        .iter()
        .find_map(|s| match s {
            XRUtilsActionState::Vector(v) => Some(v.current_state),
            _ => None,
        })
        .unwrap_or([0.0, 0.0]);

    let jump_pressed_edge = jump_query
        .iter()
        .find_map(|s| match s {
            XRUtilsActionState::Bool(b) => Some(b.current_state && b.changed_since_last_sync),
            _ => None,
        })
        .unwrap_or(false);

    // Head yaw + a pivot point (avg eye position) in *tracking-space*.
    let Some((head_yaw, pivot_local)) = head_yaw_and_pivot(&views) else {
        return;
    };

    // 1) Turn (right stick X) around your current head position to avoid weird orbiting.
    let turn_x = apply_deadzone(turn_xy[0], settings.stick_deadzone);
    if turn_x.abs() > 0.0 {
        let delta_yaw = -turn_x * settings.turn_speed_rad_s * dt;

        let old_rot = root.rotation;
        let delta_rot = Quat::from_rotation_y(delta_yaw);
        let new_rot = delta_rot * old_rot;

        // Keep the pivot world position stable while rotating.
        let world_pivot_before = old_rot.mul_vec3(pivot_local);
        let world_pivot_after = new_rot.mul_vec3(pivot_local);
        root.translation += world_pivot_before - world_pivot_after;

        root.rotation = new_rot;
    }

    // 2) Move (left stick) in the direction you are looking (root rotation + head yaw).
    let move_x = apply_deadzone(move_xy[0], settings.stick_deadzone);
    let move_y = apply_deadzone(move_xy[1], settings.stick_deadzone);
    let move_input = Vec3::new(move_x, 0.0, -move_y);

    if move_input.length_squared() > 0.0 {
        let world_yaw = root.rotation * head_yaw;
        root.translation += world_yaw.mul_vec3(move_input) * settings.move_speed_mps * dt;
    }

    // 3) Jump + gravity (simple kinematics).
    // You are only "grounded" if you are within the road bounds; otherwise you fall.
    let ground_y = road.ground_y_at(root.translation);
    let grounded = road.is_on_floor(root.translation) && root.translation.y <= ground_y + 0.001;

    if jump_pressed_edge && grounded {
        kin.vertical_velocity = settings.jump_velocity_mps;
    }

    // Integrate vertical motion.
    if !grounded || kin.vertical_velocity > 0.0 {
        kin.vertical_velocity += settings.gravity_mps2 * dt;
        root.translation.y += kin.vertical_velocity * dt;

        // Only clamp to the local ground.
        if root.translation.y <= ground_y {
            root.translation.y = ground_y;
            kin.vertical_velocity = 0.0;
        }
    }
}

fn head_yaw_and_pivot(views: &OxrViews) -> Option<(Quat, Vec3)> {
    if views.is_empty() {
        return None;
    }

    // Orientation: take first view and extract yaw only.
    let q = views[0].pose.orientation;
    let q = Quat::from_xyzw(q.x, q.y, q.z, q.w);
    let (y, _, _) = q.to_euler(EulerRot::YXZ);
    let yaw = Quat::from_rotation_y(y);

    // Position: average up to first two views (left/right eye) for an approximate head center.
    let mut sum = Vec3::ZERO;
    let mut n = 0.0;
    for v in views.iter().take(2) {
        sum += Vec3::new(v.pose.position.x, v.pose.position.y, v.pose.position.z);
        n += 1.0;
    }
    let avg = if n > 0.0 { sum / n } else { Vec3::ZERO };

    // Pivot only in XZ (we rotate around the vertical axis).
    let pivot = Vec3::new(avg.x, 0.0, avg.z);

    Some((yaw, pivot))
}

fn apply_deadzone(v: f32, deadzone: f32) -> f32 {
    if v.abs() < deadzone {
        0.0
    } else {
        v
    }
}

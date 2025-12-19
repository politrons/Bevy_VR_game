use bevy::asset::{AssetMetaCheck, AssetPlugin};
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
use log::{error, info};

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
// Locomotion components/resources
// -------------------------

#[derive(Component)]
struct MoveAction;

#[derive(Component)]
struct TurnAction;


#[derive(Component)]
struct JumpAction;

#[derive(Component, Debug, Clone, Copy)]
struct MovingPlatform {
    half_extents: Vec2, // x and z half sizes
    thickness: f32,
    z_min: f32,
    z_max: f32,
    speed_mps: f32,
    dir: f32, // +1 or -1 along Z
}

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

#[derive(Resource, Debug, Clone, Copy)]
struct FloorTopY(f32);

#[derive(Resource, Debug, Default)]
struct DidSnapToFloor(bool);


// -------------------------
// Optional texture probe (Android/Quest)
// -------------------------

// Load an asset once and log Loaded/Failed. This does NOT apply the texture to any mesh/material.
#[derive(Debug, Clone)]
struct ProbeItem {
    handle: Handle<Image>,
    path: String,
    done: bool,
}

#[derive(Resource)]
struct TextureProbe {
    probes: Vec<ProbeItem>,
    ticks: u32,
}

fn probe_texture_load_once(probe: Option<ResMut<TextureProbe>>, asset_server: Res<AssetServer>) {
    let Some(mut probe) = probe else {
        return;
    };

    probe.ticks = probe.ticks.saturating_add(1);
    let ticks = probe.ticks;

    // Log once early to confirm the system is actually running.
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
                // Log every ~120 frames so we can see if it's stuck.
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


// -------------------------
// App entry
// -------------------------

#[bevy_main]
fn main() {
    setup_logging();

    App::new()
        .insert_resource(ClearColor(Color::srgb(0.53, 0.81, 0.92)))
        .insert_resource(LocomotionSettings::default())
        .insert_resource(PlayerKinematics::default())
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
        .add_systems(XrSessionCreated, tune_xr_cameras.after(XrViewInit))
        .add_systems(Startup, create_action_entities.before(XRUtilsActionSystems::CreateEvents))
        .add_systems(Update, probe_texture_load_once)
        .add_systems(Update, move_platforms.run_if(openxr_session_running))
        .add_systems(Update, snap_player_to_floor_once.run_if(openxr_session_running))
        .add_systems(Update, handle_locomotion.run_if(openxr_session_running))
        .run();
}

// -------------------------
// Scene
// -------------------------

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    _images: ResMut<Assets<Image>>,
    asset_server: Res<AssetServer>,
) {
    // Reference cubes
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

    // Road/floor parameters
    let floor_len = 100.0_f32;
    let floor_with = 20.0_f32;
    let floor_thickness = 0.2_f32;

    let floor_top_y = 15.0_f32;
    let floor_center_y = floor_top_y - floor_thickness * 0.5;

    // Dark sides to read the cliff edge
    let floor_side_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.10, 0.10, 0.10),
        unlit: true,
        ..default()
    });

    // Plain untextured floor (single slab)
    let floor_top_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.25, 0.25, 0.25),
        unlit: true,
        ..default()
    });

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(floor_with, floor_thickness, floor_len))),
        MeshMaterial3d(floor_top_mat),
        Transform::from_xyz(0.0, floor_center_y, 0.0),
    ));

    // Side walls
    let wall_thickness = 0.3;
    let wall_eps = 0.01;

    let wall_h = (floor_top_y - floor_thickness).max(0.0);
    let wall_center_y = wall_h * 0.5;

    let wall_x = floor_with * 0.5 + wall_thickness * 0.5 + wall_eps;
    let wall_z = floor_len * 0.5 + wall_thickness * 0.5 + wall_eps;

    // Left/right walls
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(wall_thickness, wall_h, floor_len))),
        MeshMaterial3d(floor_side_mat.clone()),
        Transform::from_xyz(-wall_x, wall_center_y, 0.0),
    ));
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(wall_thickness, wall_h, floor_len))),
        MeshMaterial3d(floor_side_mat.clone()),
        Transform::from_xyz(wall_x, wall_center_y, 0.0),
    ));

    // End walls
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(floor_with, wall_h, wall_thickness))),
        MeshMaterial3d(floor_side_mat.clone()),
        Transform::from_xyz(0.0, wall_center_y, -wall_z),
    ));
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(floor_with, wall_h, wall_thickness))),
        MeshMaterial3d(floor_side_mat.clone()),
        Transform::from_xyz(0.0, wall_center_y, wall_z),
    ));

    // Ground logic: on-road clamps to top, off-road falls down
    commands.insert_resource(FloorParams {
        center: Vec3::new(0.0, 0.0, 0.0),
        half_extents: Vec2::new(floor_with * 0.5, floor_len * 0.5),
        top_y: floor_top_y,
        fall_floor_y: -200.0,
    });
    commands.insert_resource(FloorTopY(floor_top_y));
    commands.insert_resource(DidSnapToFloor(false));
    // Try loading textures (not used on any mesh) to validate Android asset loading.
    // We probe both common APK layouts:
    // - assets/textures/...  (expected)
    let p1 = "textures/grass.png".to_string();
    let h1: Handle<Image> = asset_server.load(p1.clone());
    info!("TextureProbe requested: assets/{}", p1);

    // Now that PNG support is enabled, actually use the texture on a visible mesh.
    // If this appears, AssetServer loading + material texturing are working end-to-end.
    let grass_mat = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        base_color_texture: Some(h1.clone()),
        unlit: true,
        ..default()
    });
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(0.8, 0.8, 0.8))),
        MeshMaterial3d(grass_mat),
        Transform::from_xyz(2.4, 16.2, -2.0),
    ));

    // Moving platform you can ride.
    // It travels back-and-forth along Z over the road.
    let platform_half_extents = Vec2::new(1.0, 1.0);
    let platform_thickness = 0.25_f32;
    let platform_top_y = floor_top_y + 0.6; // above the main floor

    let platform_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.9, 0.9, 0.9),
        unlit: true,
        ..default()
    });

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(
            platform_half_extents.x * 2.0,
            platform_thickness,
            platform_half_extents.y * 2.0,
        ))),
        MeshMaterial3d(platform_mat),
        Transform::from_xyz(0.0, platform_top_y - platform_thickness * 0.5, -20.0),
        MovingPlatform {
            half_extents: platform_half_extents,
            thickness: platform_thickness,
            z_min: -45.0,
            z_max: 45.0,
            speed_mps: 2.0,
            dir: 1.0,
        },
    ));

    commands.insert_resource(TextureProbe {
        probes: vec![
            ProbeItem {
                handle: h1,
                path: p1,
                done: false,
            },
        ],
        ticks: 0,
    });
}

fn snap_player_to_floor_once(
    mut xr_root: Query<&mut Transform, With<XrTrackingRoot>>,
    floor: Res<FloorTopY>,
    mut did: ResMut<DidSnapToFloor>,
) {
    if did.0 {
        return;
    }

    let Ok(mut root) = xr_root.single_mut() else {
        return;
    };

    root.translation.y = floor.0;
    did.0 = true;

    info!("Snapped XrTrackingRoot to floor y={}", floor.0);
}

// (see above for correct version)

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

// -------------------------
// Actions
// -------------------------

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

    // Put all actions in the same set.
    commands.entity(set).add_child(move_action);
    commands.entity(set).add_child(turn_action);
    commands.entity(set).add_child(jump_action);
}

// -------------------------
// Locomotion
// -------------------------

fn handle_locomotion(
    move_query: Query<&XRUtilsActionState, With<MoveAction>>,
    turn_query: Query<&XRUtilsActionState, With<TurnAction>>,
    jump_query: Query<&XRUtilsActionState, With<JumpAction>>,
    mut xr_root: Query<&mut Transform, (With<XrTrackingRoot>, Without<MovingPlatform>)>,
    time: Res<Time>,
    views: Res<OxrViews>,
    settings: Res<LocomotionSettings>,
    road: Res<FloorParams>,
    platforms: Query<(&Transform, &MovingPlatform), Without<XrTrackingRoot>>,
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

    // Head yaw + a pivot point (avg eye position) in tracking-space.
    let Some((head_yaw, pivot_local)) = head_yaw_and_pivot(&views) else {
        return;
    };

    // 1) Turn (right stick X) around your current head position.
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

    // 2) Move (left stick) in the direction you are looking.
    let move_x = apply_deadzone(move_xy[0], settings.stick_deadzone);
    let move_y = apply_deadzone(move_xy[1], settings.stick_deadzone);
    let move_input = Vec3::new(move_x, 0.0, -move_y);

    if move_input.length_squared() > 0.0 {
        let world_yaw = root.rotation * head_yaw;
        root.translation += world_yaw.mul_vec3(move_input) * settings.move_speed_mps * dt;
    }

    // 3) Jump + gravity.
    // Ground can be either the road or a moving platform (whichever is higher under the player).
    let (ground_y, ground_vel) = ground_y_and_velocity(root.translation, &road, &platforms);
    let grounded = (root.translation.y - ground_y).abs() <= 0.05;

    // If we're standing on something that moves (platform), carry the player with it.
    root.translation += ground_vel * dt;

    if jump_pressed_edge && grounded {
        kin.vertical_velocity = settings.jump_velocity_mps;
    }

    if !grounded || kin.vertical_velocity > 0.0 {
        kin.vertical_velocity += settings.gravity_mps2 * dt;
        root.translation.y += kin.vertical_velocity * dt;

        if root.translation.y <= ground_y {
            root.translation.y = ground_y;
            kin.vertical_velocity = 0.0;
        }
    }
}

fn move_platforms(mut q: Query<(&mut Transform, &mut MovingPlatform)>, time: Res<Time>) {
    let dt = time.delta_secs();

    for (mut t, mut p) in &mut q {
        t.translation.z += p.dir * p.speed_mps * dt;

        if t.translation.z > p.z_max {
            t.translation.z = p.z_max;
            p.dir = -1.0;
        } else if t.translation.z < p.z_min {
            t.translation.z = p.z_min;
            p.dir = 1.0;
        }
    }
}

fn ground_y_and_velocity(
    pos: Vec3,
    road: &FloorParams,
    platforms: &Query<(&Transform, &MovingPlatform), Without<XrTrackingRoot>>,
) -> (f32, Vec3) {
    // Start with the road ground.
    let mut best_y = road.ground_y_at(pos);
    let mut best_vel = Vec3::ZERO;

    for (t, p) in platforms.iter() {
        // Platform center and top surface.
        let center = t.translation;
        let top_y = center.y + p.thickness * 0.5;

        // Check if player is within the platform XZ footprint.
        let dx = (pos.x - center.x).abs();
        let dz = (pos.z - center.z).abs();
        if dx <= p.half_extents.x && dz <= p.half_extents.y {
            if top_y > best_y {
                best_y = top_y;
                // Platform moves along Z only.
                best_vel = Vec3::new(0.0, 0.0, p.dir * p.speed_mps);
            }
        }
    }

    (best_y, best_vel)
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

    // Position: average up to first two views (left/right eye).
    let mut sum = Vec3::ZERO;
    let mut n = 0.0;
    for v in views.iter().take(2) {
        sum += Vec3::new(v.pose.position.x, v.pose.position.y, v.pose.position.z);
        n += 1.0;
    }

    let avg = if n > 0.0 { sum / n } else { Vec3::ZERO };

    // Pivot only in XZ.
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

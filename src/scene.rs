use bevy::prelude::*;
use bevy_mod_xr::session::XrTrackingRoot;

#[derive(Resource, Debug, Clone, Copy)]
pub(crate) struct PlayerSpawn {
    pub(crate) pos: Vec3,
    pub(crate) rot: Quat,
}

// -------------------------
// Floor + scene setup
// -------------------------

/// Parameters describing the main playable floor (the big road/platform) and the fallback
/// "void" ground used when the player walks outside the road bounds.
///
/// This is used by locomotion/gravity to decide the current `ground_y` under the player.
/// If the player is within the floor's XZ bounds, `top_y` is returned; otherwise `fall_floor_y`.
#[derive(Resource, Debug, Clone, Copy)]
pub(crate) struct FloorParams {
    pub(crate) center: Vec3,
    pub(crate) half_extents: Vec2, // x and z half sizes
    pub(crate) top_y: f32,
    pub(crate) fall_floor_y: f32,
}

impl FloorParams {
    /// Returns the ground Y for a world position `p`.
    ///
    /// - If `p` is inside the road rectangle (XZ footprint), returns `self.top_y`.
    /// - Otherwise returns `self.fall_floor_y` (so the player falls off the world).
    pub(crate) fn ground_y_at(&self, p: Vec3) -> f32 {
        let dx = (p.x - self.center.x).abs();
        let dz = (p.z - self.center.z).abs();
        if dx <= self.half_extents.x && dz <= self.half_extents.y {
            self.top_y
        } else {
            self.fall_floor_y
        }
    }

    /// Returns `true` if `pos` is within the floor XZ footprint (ignores Y).
    ///
    /// This is mainly useful for debugging / sanity checks.
    #[allow(dead_code)]
    pub(crate) fn is_on_floor(&self, pos: Vec3) -> bool {
        let dx = (pos.x - self.center.x).abs();
        let dz = (pos.z - self.center.z).abs();
        dx <= self.half_extents.x && dz <= self.half_extents.y
    }
}

/// Convenience resource holding the top surface Y of the main floor.
///
/// Used during initial spawn to snap the XR tracking root to a sensible height.
#[derive(Resource, Debug, Clone, Copy)]
pub(crate) struct FloorTopY(pub(crate) f32);

/// One-shot flag used to ensure we only snap the XR tracking root to the floor once.
///
/// Without this, repeated snapping would fight against locomotion/gravity updates.
#[derive(Resource, Debug, Default)]
pub(crate) struct DidSnapToFloor(pub(crate) bool);

/// Creates the initial scene: a large road (playable floor), side/end walls, a textured
/// debug cube (to validate Android asset loading), and a moving platform the player can ride.
///
/// Also initializes resources used by other systems:
/// - [`FloorParams`] for ground detection
/// - [`FloorTopY`] and [`DidSnapToFloor`] for the initial spawn snap
/// - [`TextureProbe`] for logging asset load state on device
pub(crate) fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    _images: ResMut<Assets<Image>>,
    asset_server: Res<AssetServer>,
) {
    // Road/floor parameters
    // Half the previous size (width and length)
    let floor_len = 50.0_f32;
    let floor_with = 10.0_f32;
    let floor_thickness = 0.2_f32;

    let floor_top_y = 15.0_f32;
    let floor_center_y = floor_top_y - floor_thickness * 0.5;

    // Player spawn point: on the main floor near the beginning of the run.
    commands.insert_resource(PlayerSpawn {
        pos: Vec3::new(0.0, floor_top_y, -20.0),
        rot: Quat::IDENTITY,
    });

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
}

/// Snaps the XR tracking root's Y translation to the floor height exactly once.
///
/// This is a pragmatic "spawn fix": it ensures the player starts on the floor even if the
/// initial tracking origin is slightly off.
pub(crate) fn snap_player_to_floor_once(
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

    // Force the root to sit on the floor at startup.
    root.translation.y = floor.0;
    did.0 = true;
}

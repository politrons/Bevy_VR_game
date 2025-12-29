use bevy::gltf::{Gltf, GltfLoaderSettings, GltfMesh};
use bevy::prelude::*;
use bevy_mod_xr::session::XrTrackingRoot;

const PLANET_MODEL_RADIUS_M: f32 = 1.0;
const PLANET_VISUAL_RADIUS_M: f32 = 41.666668;
const PLANET_ROTATION_SPEED_RAD_PER_SEC: f32 = 0.012;
const PLANET_ROTATION_AXIS: Vec3 = Vec3::Y;

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

/// Tag for entities that render the main floor and its boundary walls.
#[derive(Component)]
pub(crate) struct FloorVisual;

/// Handles for the planet visual that replaces the default floor graphics.
#[derive(Resource, Clone)]
pub(crate) struct PlanetFloorAssets {
    pub(crate) gltf: Handle<Gltf>,
    pub(crate) fallback_material: Handle<StandardMaterial>,
}

/// Tracks whether the planet visual has been spawned.
#[derive(Resource, Debug, Default)]
pub(crate) struct PlanetFloorState {
    pub(crate) spawned: bool,
    pub(crate) failed: bool,
}

/// Rotating planet surface that visually replaces the floor.
#[derive(Component, Debug, Clone, Copy)]
pub(crate) struct PlanetFloor {
    pub(crate) axis: Vec3,
    pub(crate) speed_rad_per_sec: f32,
}

/// Creates the initial scene: a large road (playable floor), side/end walls, a textured
/// debug cube (to validate Android asset loading), and a moving platform the player can ride.
///
/// Also initializes resources used by other systems:
/// - [`FloorParams`] for ground detection
/// - [`FloorTopY`] and [`DidSnapToFloor`] for the initial spawn snap
/// - [`TextureProbe`] for logging asset load state on device
pub(crate) fn setup_scene(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    _images: ResMut<Assets<Image>>,
) {
    commands.insert_resource(AmbientLight {
        color: Color::srgb(0.45, 0.45, 0.45),
        brightness: 0.12,
        affects_lightmapped_meshes: false,
    });
    commands.spawn((
        DirectionalLight {
            illuminance: 4500.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(0.0, 80.0, 0.0).looking_at(Vec3::ZERO, Vec3::Y),
        Name::new("SunLight"),
    ));

    // Road/floor parameters
    // Half the previous size (width and length)
    let floor_len = 50.0_f32;
    let floor_with = 12.0_f32;
    let floor_thickness = 0.2_f32;

    let floor_top_y = 15.0_f32;
    let floor_center_y = floor_top_y - floor_thickness * 0.5;

    // Player spawn point: very high above the middle of the floor (free-fall start).
    // Floor is centered at z=0, so "middle of the length" is z=0.
    let spawn_height_above_floor = 80.0_f32;
    commands.insert_resource(PlayerSpawn {
        pos: Vec3::new(0.0, floor_top_y + spawn_height_above_floor, 0.0),
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
        FloorVisual,
        Mesh3d(meshes.add(Cuboid::new(floor_with, floor_thickness, floor_len))),
        MeshMaterial3d(floor_top_mat),
        Transform::from_xyz(0.0, floor_center_y, 0.0),
        Name::new("FloorTop"),
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
        FloorVisual,
        Mesh3d(meshes.add(Cuboid::new(wall_thickness, wall_h, floor_len))),
        MeshMaterial3d(floor_side_mat.clone()),
        Transform::from_xyz(-wall_x, wall_center_y, 0.0),
        Name::new("WallLeft"),
    ));
    commands.spawn((
        FloorVisual,
        Mesh3d(meshes.add(Cuboid::new(wall_thickness, wall_h, floor_len))),
        MeshMaterial3d(floor_side_mat.clone()),
        Transform::from_xyz(wall_x, wall_center_y, 0.0),
        Name::new("WallRight"),
    ));

    // End walls
    commands.spawn((
        FloorVisual,
        Mesh3d(meshes.add(Cuboid::new(floor_with, wall_h, wall_thickness))),
        MeshMaterial3d(floor_side_mat.clone()),
        Transform::from_xyz(0.0, wall_center_y, -wall_z),
        Name::new("WallBack"),
    ));
    commands.spawn((
        FloorVisual,
        Mesh3d(meshes.add(Cuboid::new(floor_with, wall_h, wall_thickness))),
        MeshMaterial3d(floor_side_mat.clone()),
        Transform::from_xyz(0.0, wall_center_y, wall_z),
        Name::new("WallFront"),
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

    let planet_fallback_material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.20, 0.20, 0.20),
        ..default()
    });
    let planet_gltf: Handle<Gltf> = asset_server.load_with_settings(
        "textures/planet.glb",
        |settings: &mut GltfLoaderSettings| {
            settings.load_cameras = false;
            settings.load_lights = false;
            settings.load_animations = false;
        },
    );
    commands.insert_resource(PlanetFloorAssets {
        gltf: planet_gltf,
        fallback_material: planet_fallback_material,
    });
    commands.insert_resource(PlanetFloorState::default());
}

/// Spawns the planet visual once the GLB has loaded, then hides the default floor meshes.
pub(crate) fn spawn_planet_floor(
    assets: Option<Res<PlanetFloorAssets>>,
    asset_server: Res<AssetServer>,
    gltfs: Res<Assets<Gltf>>,
    gltf_meshes: Res<Assets<GltfMesh>>,
    floor: Option<Res<FloorParams>>,
    mut state: Option<ResMut<PlanetFloorState>>,
    floor_visuals: Query<Entity, With<FloorVisual>>,
    mut commands: Commands,
) {
    let (Some(assets), Some(floor), Some(mut state)) = (assets, floor, state) else {
        return;
    };
    if state.spawned || state.failed {
        return;
    }

    let Some(load_state) = asset_server.get_load_state(assets.gltf.id()) else {
        return;
    };
    match load_state {
        bevy::asset::LoadState::Loaded => {
            let Some(gltf) = gltfs.get(&assets.gltf) else {
                return;
            };
            if gltf.meshes.is_empty() {
                log::warn!("Planet model has no meshes.");
                state.failed = true;
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
                        .unwrap_or_else(|| assets.fallback_material.clone());
                    primitives.push((primitive.mesh.clone(), material));
                }
            }
            if primitives.is_empty() {
                log::warn!("Planet model has no mesh primitives.");
                state.failed = true;
                return;
            }

            let scale = PLANET_VISUAL_RADIUS_M / PLANET_MODEL_RADIUS_M;
            let planet = commands
                .spawn((
                    PlanetFloor {
                        axis: PLANET_ROTATION_AXIS,
                        speed_rad_per_sec: PLANET_ROTATION_SPEED_RAD_PER_SEC,
                    },
                    Transform {
                        translation: Vec3::new(
                            floor.center.x,
                            floor.top_y - PLANET_VISUAL_RADIUS_M,
                            floor.center.z,
                        ),
                        scale: Vec3::splat(scale),
                        ..default()
                    },
                    Name::new("PlanetFloor"),
                ))
                .id();
            commands.entity(planet).with_children(|parent| {
                for (mesh, material) in &primitives {
                    parent.spawn((
                        Mesh3d(mesh.clone()),
                        MeshMaterial3d(material.clone()),
                        Transform::default(),
                    ));
                }
            });

            for entity in &floor_visuals {
                commands.entity(entity).insert(Visibility::Hidden);
            }
            state.spawned = true;
        }
        bevy::asset::LoadState::Failed(err) => {
            log::warn!("Planet model failed to load: {:?}", err);
            state.failed = true;
        }
        _ => {}
    }
}

/// Rotates the planet visual slowly to give a subtle sense of motion.
pub(crate) fn spin_planet_floor(
    time: Res<Time>,
    mut planets: Query<(&mut Transform, &PlanetFloor)>,
) {
    let dt = time.delta_secs();
    if dt <= 0.0 {
        return;
    }
    for (mut transform, planet) in &mut planets {
        let axis = if planet.axis.length_squared() > 0.0 {
            planet.axis.normalize()
        } else {
            Vec3::Y
        };
        transform.rotate(Quat::from_axis_angle(
            axis,
            planet.speed_rad_per_sec * dt,
        ));
    }
}

/// Snaps the XR tracking root's transform to the configured PlayerSpawn exactly once.
///
/// This is a pragmatic "spawn fix": it ensures the player starts at the desired spawn even if the
/// initial tracking origin is slightly off.
pub(crate) fn snap_player_to_floor_once(
    mut xr_root: Query<&mut Transform, With<XrTrackingRoot>>,
    spawn: Res<PlayerSpawn>,
    mut did: ResMut<DidSnapToFloor>,
) {
    if did.0 {
        return;
    }

    let Ok(mut root) = xr_root.single_mut() else {
        return;
    };

    // Force the root to match our configured spawn once at startup.
    // This enables the "start high in the sky" free-fall camera.
    root.translation = spawn.pos;
    root.rotation = spawn.rot;
    did.0 = true;
}

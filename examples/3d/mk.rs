use avian3d::prelude::*;
use bevy::prelude::*;
use bevy::window::WindowResolution;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum SurfaceKind {
    Road,
    Grass,
    Sand,
    Ice,
}

const TILE_SIZE: f32 = 6.0;
const HALF_TILES: i32 = 18;
const GROUND_THICKNESS: f32 = 0.24;
const GROUND_Y: f32 = -0.12;

const LAYER_GROUND: u32 = 1 << 0;
const LAYER_KART: u32 = 1 << 1;

#[derive(Component)]
struct Kart;

#[derive(Component)]
struct KartHud;

#[derive(Component)]
struct CameraTarget;

#[derive(Clone, Copy, Debug)]
struct SurfaceParams {
    grip: f32,
    rolling_drag: f32,
    max_speed_mult: f32,
}

impl SurfaceKind {
    fn params(self) -> SurfaceParams {
        match self {
            SurfaceKind::Road => SurfaceParams {
                grip: 1.0,
                rolling_drag: 0.8,
                max_speed_mult: 1.0,
            },
            SurfaceKind::Grass => SurfaceParams {
                grip: 0.75,
                rolling_drag: 2.8,
                max_speed_mult: 0.85,
            },
            SurfaceKind::Sand => SurfaceParams {
                grip: 0.85,
                rolling_drag: 2.0,
                max_speed_mult: 0.9,
            },
            SurfaceKind::Ice => SurfaceParams {
                grip: 0.28,
                rolling_drag: 0.35,
                max_speed_mult: 1.05,
            },
        }
    }

    fn ui_name(self) -> &'static str {
        match self {
            SurfaceKind::Road => "Road",
            SurfaceKind::Grass => "Grass",
            SurfaceKind::Sand => "Sand",
            SurfaceKind::Ice => "Ice",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DriftTier {
    None,
    Blue,
    Red,
    Purple,
}

impl DriftTier {
    fn ui_name(self) -> &'static str {
        match self {
            DriftTier::None => "None",
            DriftTier::Blue => "Blue",
            DriftTier::Red => "Red",
            DriftTier::Purple => "Purple",
        }
    }
}

#[derive(Component, Clone, Copy)]
struct KartConfig {
    wheel_offsets: [Vec3; 4],
    suspension_rest: f32,
    suspension_travel: f32,
    suspension_ray_start: f32,
    suspension_spring_accel: f32,
    suspension_damper_accel: f32,
    upright_accel: f32,
    upright_damping: f32,
    engine_accel: f32,
    brake_accel: f32,
    max_speed: f32,
    reverse_max_speed: f32,
    steer_yaw_accel: f32,
    drift_steer_mult: f32,
    drift_grip_mult: f32,
    lateral_friction_accel: f32,
    air_control_yaw_accel: f32,
    boost_accel: f32,
}

impl Default for KartConfig {
    fn default() -> Self {
        Self {
            wheel_offsets: [
                Vec3::new(0.75, -0.2, 1.05),
                Vec3::new(-0.75, -0.2, 1.05),
                Vec3::new(0.75, -0.2, -1.05),
                Vec3::new(-0.75, -0.2, -1.05),
            ],
            suspension_rest: 0.95,
            suspension_travel: 0.55,
            suspension_ray_start: 0.5,
            suspension_spring_accel: 55.0,
            suspension_damper_accel: 10.0,
            upright_accel: 28.0,
            upright_damping: 5.5,
            engine_accel: 38.0,
            brake_accel: 55.0,
            max_speed: 42.0,
            reverse_max_speed: 14.0,
            steer_yaw_accel: 14.0,
            drift_steer_mult: 1.75,
            drift_grip_mult: 0.55,
            lateral_friction_accel: 14.0,
            air_control_yaw_accel: 4.0,
            boost_accel: 65.0,
        }
    }
}

#[derive(Component)]
struct KartState {
    grounded: bool,
    surface: SurfaceKind,
    ground_normal: Vec3,

    drift_active: bool,
    drift_dir: f32,
    drift_time: f32,
    drift_tier: DriftTier,

    boost_timer: f32,
    boost_cooldown: f32,

    last_wheel_compression: [f32; 4],
}

impl Default for KartState {
    fn default() -> Self {
        Self {
            grounded: false,
            surface: SurfaceKind::Road,
            ground_normal: Vec3::Y,
            drift_active: false,
            drift_dir: 0.0,
            drift_time: 0.0,
            drift_tier: DriftTier::None,
            boost_timer: 0.0,
            boost_cooldown: 0.0,
            last_wheel_compression: [0.0; 4],
        }
    }
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Mini Kart (Bevy + Avian3D)".into(),
                resolution: WindowResolution::new(1280, 720),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(PhysicsPlugins::default())
        .insert_resource(Gravity(Vec3::NEG_Y * 28.0))
        .add_systems(Startup, setup)
        .add_systems(
            PhysicsSchedule,
            kart_arcade_physics.in_set(PhysicsStepSystems::BroadPhase),
        )
        .add_systems(Update, (camera_follow, update_hud))
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 7.0, -14.0).looking_at(Vec3::new(0.0, 1.0, 0.0), Vec3::Y),
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 14_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(20.0, 35.0, 20.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    commands.insert_resource(AmbientLight {
        brightness: 400.0,
        ..default()
    });

    spawn_test_ground(&mut commands, &mut meshes, &mut materials);

    let kart_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.85, 0.18, 0.2),
        perceptual_roughness: 0.85,
        ..default()
    });
    let wheel_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.08, 0.08, 0.09),
        perceptual_roughness: 0.95,
        ..default()
    });

    let body_mesh = meshes.add(Cuboid::new(1.6, 0.55, 2.6));
    let wheel_mesh = meshes.add(Cylinder::new(0.32, 0.22));

    let kart_entity = commands
        .spawn((
            Kart,
            CameraTarget,
            Mesh3d(body_mesh),
            MeshMaterial3d(kart_mat),
            Transform::from_translation(Vec3::new(0.0, 2.5, 0.0)),
            RigidBody::Dynamic,
            Collider::cuboid(1.6, 0.55, 2.6),
            KartConfig::default(),
            KartState::default(),
        ))
        .insert((
            CollisionLayers::from_bits(LAYER_KART, 0xFFFF_FFFF ^ LAYER_GROUND),
            LinearVelocity::default(),
            AngularVelocity::default(),
            LinearDamping(0.0),
            AngularDamping(0.0),
            Friction::ZERO,
            Restitution::ZERO,
        ))
        .id();

    for offset in KartConfig::default().wheel_offsets {
        let wheel_entity = commands
            .spawn((
                Mesh3d(wheel_mesh.clone()),
                MeshMaterial3d(wheel_mat.clone()),
                Transform::from_translation(offset).with_rotation(Quat::from_rotation_z(1.57)),
            ))
            .id();
        commands.entity(kart_entity).add_child(wheel_entity);
    }

    commands.spawn((
        Text::new(""),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
        KartHud,
    ));
}

fn spawn_test_ground(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    let mesh = meshes.add(Plane3d::default().mesh().size(TILE_SIZE, TILE_SIZE));

    let road_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.18, 0.18, 0.2),
        perceptual_roughness: 0.98,
        ..default()
    });
    let grass_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.22, 0.48, 0.25),
        perceptual_roughness: 1.0,
        ..default()
    });
    let sand_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.7, 0.63, 0.38),
        perceptual_roughness: 0.95,
        ..default()
    });
    let ice_mat = materials.add(StandardMaterial {
        base_color: Color::srgb(0.65, 0.85, 0.92),
        perceptual_roughness: 0.02,
        metallic: 0.0,
        ..default()
    });

    for tz in -HALF_TILES..=HALF_TILES {
        for tx in -HALF_TILES..=HALF_TILES {
            let kind = sample_surface_kind(tx, tz);
            let material = match kind {
                SurfaceKind::Road => road_mat.clone(),
                SurfaceKind::Grass => grass_mat.clone(),
                SurfaceKind::Sand => sand_mat.clone(),
                SurfaceKind::Ice => ice_mat.clone(),
            };

            commands.spawn((
                Mesh3d(mesh.clone()),
                MeshMaterial3d(material),
                Transform::from_translation(Vec3::new(
                    tx as f32 * TILE_SIZE,
                    GROUND_Y,
                    tz as f32 * TILE_SIZE,
                )),
            ));
        }
    }

    // One big ground collider for physics (tiles are visual only).
    let side = TILE_SIZE * (HALF_TILES * 2 + 1) as f32;
    commands.spawn((
        RigidBody::Static,
        Collider::cuboid(side, GROUND_THICKNESS, side),
        Transform::from_translation(Vec3::new(0.0, GROUND_Y, 0.0)),
        CollisionLayers::from_bits(LAYER_GROUND, 0xFFFF_FFFF ^ LAYER_KART),
        Friction::ZERO,
        Restitution::ZERO,
    ));
}

fn sample_surface_kind(tx: i32, tz: i32) -> SurfaceKind {
    let mut x = (tx as u32).wrapping_mul(0x9E37_79B9);
    let mut z = (tz as u32).wrapping_mul(0x85EB_CA6B);
    x ^= x >> 16;
    z ^= z >> 16;
    let h = x
        .wrapping_add(z.rotate_left(11))
        .wrapping_mul(0xC2B2_AE35)
        ^ 0x27D4_EB2F;
    match (h % 100) as u8 {
        0..=54 => SurfaceKind::Road,
        55..=74 => SurfaceKind::Grass,
        75..=89 => SurfaceKind::Sand,
        _ => SurfaceKind::Ice,
    }
}

fn sample_surface_at(x: f32, z: f32) -> SurfaceKind {
    let tx = (x / TILE_SIZE).floor() as i32;
    let tz = (z / TILE_SIZE).floor() as i32;
    sample_surface_kind(tx, tz)
}

fn kart_arcade_physics(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time<Physics>>,
    mut spatial: SpatialQuery,
    mut karts: Query<(Entity, Forces, &KartConfig, &mut KartState), With<Kart>>,
) {
    spatial.update_pipeline();

    let dt = time.delta_secs().max(1.0 / 600.0);

    for (entity, mut forces, config, mut state) in &mut karts {
        let filter = SpatialQueryFilter::from_excluded_entities([entity]);

        // --- Raycast suspension + ground sampling ---
        let rot = forces.rotation().0;
        let pos = forces.position().0;

        let mut hit_count = 0usize;
        let mut normal_sum = Vec3::ZERO;
        let mut surface_votes = (0u8, 0u8, 0u8, 0u8); // road, grass, sand, ice

        let mut wheel_hits = [None; 4];
        let max_distance = config.suspension_rest + config.suspension_travel + config.suspension_ray_start;

        for (i, local_offset) in config.wheel_offsets.iter().copied().enumerate() {
            let wheel_world = pos + rot * local_offset;
            let origin = wheel_world + Vec3::Y * config.suspension_ray_start;

            if let Some(hit) = spatial.cast_ray(origin, Dir3::NEG_Y, max_distance, true, &filter) {
                let hit_distance = (hit.distance - config.suspension_ray_start).max(0.0);
                if hit_distance <= config.suspension_rest + config.suspension_travel {
                    hit_count += 1;
                    normal_sum += hit.normal;

                    let hit_point = origin + Vec3::NEG_Y * hit.distance;
                    let kind = sample_surface_at(hit_point.x, hit_point.z);
                    match kind {
                        SurfaceKind::Road => surface_votes.0 += 1,
                        SurfaceKind::Grass => surface_votes.1 += 1,
                        SurfaceKind::Sand => surface_votes.2 += 1,
                        SurfaceKind::Ice => surface_votes.3 += 1,
                    }

                    let compression = (config.suspension_rest - hit_distance)
                        .clamp(0.0, config.suspension_travel);
                    wheel_hits[i] = Some((wheel_world, hit.normal, compression));
                }
            }
        }

        state.grounded = hit_count >= 2;
        state.ground_normal = if hit_count > 0 {
            normal_sum.normalize_or_zero()
        } else {
            Vec3::Y
        };

        if hit_count > 0 {
            state.surface = {
                let (r, g, s, i) = surface_votes;
                let (best, _) = [
                    (SurfaceKind::Road, r),
                    (SurfaceKind::Grass, g),
                    (SurfaceKind::Sand, s),
                    (SurfaceKind::Ice, i),
                ]
                .into_iter()
                .max_by_key(|(_, c)| *c)
                .unwrap();
                best
            };
        }

        let support_scale = 1.0 / (hit_count as f32).max(1.0);

        for (i, hit) in wheel_hits.iter().copied().enumerate() {
            let Some((wheel_world, normal, compression)) = hit else {
                state.last_wheel_compression[i] = 0.0;
                continue;
            };/

            let compression_rate = (compression - state.last_wheel_compression[i]) / dt;
            state.last_wheel_compression[i] = compression;

            let spring = (compression / config.suspension_travel) * config.suspension_spring_accel;
            let damper = -compression_rate * config.suspension_damper_accel;

            let accel = (spring + damper).clamp(-40.0, 120.0) * support_scale;
            forces.apply_linear_acceleration_at_point(normal * accel, wheel_world);
        }

        // --- Controls ---
        let throttle: f32 = if keys.pressed(KeyCode::KeyW) { 1.0 } else { 0.0 }
            + if keys.pressed(KeyCode::KeyS) { -1.0 } else { 0.0 };
        let steer: f32 = if keys.pressed(KeyCode::KeyA) { 1.0 } else { 0.0 }
            + if keys.pressed(KeyCode::KeyD) { -1.0 } else { 0.0 };
        let drift_pressed = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);
        let drift_just_pressed = keys.just_pressed(KeyCode::ShiftLeft) || keys.just_pressed(KeyCode::ShiftRight);
        let drift_just_released =
            keys.just_released(KeyCode::ShiftLeft) || keys.just_released(KeyCode::ShiftRight);
        let boost_pressed = keys.just_pressed(KeyCode::ControlLeft) || keys.just_pressed(KeyCode::ControlRight);

        // --- Drift (mini-turbo charge) ---
        if drift_just_pressed && state.grounded && steer.abs() > 0.1 && !state.drift_active {
            state.drift_active = true;
            state.drift_dir = steer.signum();
            state.drift_time = 0.0;
            state.drift_tier = DriftTier::None;
        }

        if state.drift_active && drift_pressed && state.grounded {
            state.drift_time += dt;
            state.drift_tier = if state.drift_time >= 2.4 {
                DriftTier::Purple
            } else if state.drift_time >= 1.6 {
                DriftTier::Red
            } else if state.drift_time >= 0.8 {
                DriftTier::Blue
            } else {
                DriftTier::None
            };
        }

        let should_end_drift = state.drift_active && (drift_just_released || !state.grounded);
        if should_end_drift {
            if state.grounded {
                match state.drift_tier {
                    DriftTier::Blue => state.boost_timer = state.boost_timer.max(0.7),
                    DriftTier::Red => state.boost_timer = state.boost_timer.max(1.0),
                    DriftTier::Purple => state.boost_timer = state.boost_timer.max(1.35),
                    DriftTier::None => {}
                }
            }
            state.drift_active = false;
            state.drift_dir = 0.0;
            state.drift_time = 0.0;
            state.drift_tier = DriftTier::None;
        }

        // --- Boost (manual) ---
        if boost_pressed && state.boost_cooldown <= 0.0 {
            state.boost_timer = state.boost_timer.max(0.85);
            state.boost_cooldown = 2.5;
        }
        state.boost_timer = (state.boost_timer - dt).max(0.0);
        state.boost_cooldown = (state.boost_cooldown - dt).max(0.0);

        // --- Planar basis ---
        let desired_up = state.ground_normal;
        let forward_raw = rot * Vec3::NEG_Z;
        let forward = (forward_raw - desired_up * forward_raw.dot(desired_up)).normalize_or_zero();
        let right = desired_up.cross(forward).normalize_or_zero();

        let vel = forces.linear_velocity();
        let planar_vel = vel - desired_up * vel.dot(desired_up);
        let vertical_speed = vel.dot(desired_up);
        let planar_speed = planar_vel.length();
        let forward_speed = planar_vel.dot(forward);
        let side_speed = planar_vel.dot(right);

        // --- Surface response ---
        let surface = state.surface.params();
        let grip = surface.grip * if state.drift_active { config.drift_grip_mult } else { 1.0 };
        let rolling_drag = surface.rolling_drag;
        let max_speed = if throttle >= 0.0 {
            config.max_speed * surface.max_speed_mult
        } else {
            config.reverse_max_speed
        };

        // --- Engine / brakes ---
        if throttle.abs() > 0.01 {
            let accel = if throttle >= 0.0 {
                config.engine_accel
            } else {
                config.engine_accel * 0.7
            };
            forces.apply_linear_acceleration(forward * (throttle * accel));
        }

        // Dedicated braking when trying to go opposite direction.
        if throttle < 0.0 && forward_speed > 2.0 {
            forces.apply_linear_acceleration(-forward * config.brake_accel);
        }
        if throttle > 0.0 && forward_speed < -2.0 {
            forces.apply_linear_acceleration(forward * config.brake_accel);
        }

        // --- Boost acceleration ---
        if state.boost_timer > 0.0 && state.grounded {
            forces.apply_linear_acceleration(forward * config.boost_accel);
        }

        // --- Steering ---
        let steer_mult = if state.drift_active {
            config.drift_steer_mult
        } else {
            1.0
        };
        if state.grounded {
            forces.apply_angular_acceleration(desired_up * (steer * config.steer_yaw_accel * steer_mult));
        } else {
            forces.apply_angular_acceleration(Vec3::Y * (steer * config.air_control_yaw_accel));
        }

        // --- Lateral friction / rolling drag ---
        if state.grounded {
            let lateral = -right * side_speed * (config.lateral_friction_accel * grip);
            forces.apply_linear_acceleration(lateral);
            forces.apply_linear_acceleration(-planar_vel * rolling_drag);

            // Kill vertical bounce when grounded (raycast suspension tends to oscillate without this).
            if vertical_speed > 0.0 {
                forces.apply_linear_acceleration(-desired_up * vertical_speed * 8.0);
            }
        } else {
            forces.apply_linear_acceleration(-planar_vel * (rolling_drag * 0.15));
        }

        // --- Speed limiting ---
        if planar_speed > max_speed {
            let excess = planar_speed - max_speed;
            forces.apply_linear_acceleration(-planar_vel.normalize_or_zero() * (excess * 6.0));
        }

        // --- Upright stabilization (anti-flip) ---
        let current_up = rot * Vec3::Y;
        let tilt_axis = current_up.cross(desired_up);
        let ang_vel = forces.angular_velocity();
        let yaw = desired_up * ang_vel.dot(desired_up);
        let ang_no_yaw = ang_vel - yaw;

        forces.apply_angular_acceleration(tilt_axis * config.upright_accel);
        forces.apply_angular_acceleration(-ang_no_yaw * config.upright_damping);
    }
}

fn camera_follow(
    mut camera_query: Query<&mut Transform, (With<Camera3d>, Without<CameraTarget>)>,
    target_query: Query<(&Transform, &LinearVelocity), With<CameraTarget>>,
    time: Res<Time>,
) {
    let Some((target_transform, velocity)) = target_query.iter().next() else {
        return;
    };
    let Some(mut cam_transform) = camera_query.iter_mut().next() else {
        return;
    };

    let forward = target_transform.rotation * Vec3::NEG_Z;
    let planar_speed = Vec3::new(velocity.0.x, 0.0, velocity.0.z).length();
    let distance = 10.5 + planar_speed * 0.18;
    let height = 4.8 + planar_speed * 0.08;

    let desired =
        target_transform.translation - forward * distance + Vec3::Y * height + Vec3::X * 0.0;
    cam_transform.translation = cam_transform
        .translation
        .lerp(desired, 1.0 - (-time.delta_secs() * 6.0).exp());
    cam_transform.look_at(target_transform.translation + Vec3::Y * 1.2, Vec3::Y);
}

fn update_hud(
    mut hud: Query<&mut Text, With<KartHud>>,
    kart: Query<(&LinearVelocity, &KartState), With<Kart>>,
) {
    let Some((vel, state)) = kart.iter().next() else {
        return;
    };
    let Some(mut text) = hud.iter_mut().next() else {
        return;
    };

    let speed = Vec3::new(vel.0.x, 0.0, vel.0.z).length();
    let drifting = if state.drift_active { "Yes" } else { "No" };
    let grounded = if state.grounded { "Yes" } else { "No" };
    let surface = if state.grounded {
        state.surface.ui_name()
    } else {
        "Air"
    };

    text.0 = format!(
        "W/S throttle+brake  A/D steer  Shift drift  Ctrl boost\nSpeed: {:>5.1}  Grounded: {}  Surface: {}\nDrift: {}  Tier: {}  Boost: {:>4.2}s  Cooldown: {:>4.2}s",
        speed,
        grounded,
        surface,
        drifting,
        state.drift_tier.ui_name(),
        state.boost_timer,
        state.boost_cooldown
    );
}

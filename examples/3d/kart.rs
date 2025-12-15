use avian3d::prelude::*;
use bevy::prelude::*;
use bevy::window::WindowResolution;

// --- Components ---

#[derive(Component)]
struct Kart {
    acceleration: f32,
    turn_speed: f32,
    max_speed: f32,
    is_drifting: bool,
}

impl Default for Kart {
    fn default() -> Self {
        Self {
            acceleration: 0.5, // Applied per frame as velocity change
            turn_speed: 2.0,
            max_speed: 40.0,
            is_drifting: false,
        }
    }
}

#[derive(Component)]
struct GameState {
    laps: i32,
    checkpoints_passed: usize,
    total_checkpoints: usize,
    game_over: bool,
}

#[derive(Component)]
struct BoostPad {
    strength: f32,
}

#[derive(Component)]
struct JumpPad {
    strength: f32,
}

#[derive(Component)]
struct Checkpoint {
    index: usize,
}

#[derive(Component)]
struct CameraTarget;

// --- Setup ---

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Bevy Kart Racer".into(),
                resolution: WindowResolution::new(1280, 720),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(PhysicsPlugins::default())
        .insert_resource(Gravity(Vec3::NEG_Y * 20.0))
        .add_systems(Startup, setup)
        .add_systems(
            Update,
            (
                kart_control_system,
                camera_follow_system,
                game_logic_system,
                trigger_detection_system,
                update_ui_system,
            ),
        )
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 10.0, -20.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Light
    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(10.0, 20.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Ground
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(200.0, 200.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.3, 0.5, 0.3))),
        RigidBody::Static,
        Collider::cuboid(200.0, 0.1, 200.0),
        Transform::from_xyz(0.0, 0.0, 0.0),
    ));

    // Track generation
    spawn_track(&mut commands, &mut meshes, &mut materials);

    // Kart
    let kart_mesh = meshes.add(Cuboid::new(1.0, 0.5, 2.0));
    let wheel_mesh = meshes.add(Cylinder::new(0.3, 0.2));
    let kart_mat = materials.add(Color::srgb(0.8, 0.2, 0.2));
    let wheel_mat = materials.add(Color::srgb(0.1, 0.1, 0.1));

    let start_pos = Vec3::new(0.0, 1.0, 0.0);

    let kart = commands
        .spawn((
            Mesh3d(kart_mesh),
            MeshMaterial3d(kart_mat),
            RigidBody::Dynamic,
            Collider::cuboid(1.0, 0.5, 2.0),
            Transform::from_translation(start_pos),
            Kart::default(),
            LinearVelocity::default(),
            AngularVelocity::default(),
            LinearDamping(1.0),
            AngularDamping(4.0),
            CameraTarget,
        ))
        .id();

    // Wheels
    let wheel_offsets = [
        Vec3::new(0.6, -0.2, 0.8),
        Vec3::new(-0.6, -0.2, 0.8),
        Vec3::new(0.6, -0.2, -0.8),
        Vec3::new(-0.6, -0.2, -0.8),
    ];

    for offset in wheel_offsets {
        let child = commands
            .spawn((
                Mesh3d(wheel_mesh.clone()),
                MeshMaterial3d(wheel_mat.clone()),
                Transform::from_translation(offset).with_rotation(Quat::from_rotation_z(1.57)),
            ))
            .id();
        commands.entity(kart).add_child(child);
    }

    // UI
    commands.spawn((
        Text::new("Lap: 1/2"),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(10.0),
            left: Val::Px(10.0),
            ..default()
        },
    ));

    commands.spawn(GameState {
        laps: 0,
        checkpoints_passed: 0,
        total_checkpoints: 4,
        game_over: false,
    });
}

fn spawn_track(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) {
    let wall_mat = materials.add(Color::srgb(0.5, 0.5, 0.5));
    let boost_mat = materials.add(Color::srgb(0.0, 0.0, 1.0));
    let jump_mat = materials.add(Color::srgb(1.0, 1.0, 0.0));
    let checkpoint_mat = materials.add(Color::srgba(0.0, 1.0, 0.0, 0.3));

    let walls = [
        (
            Vec3::new(25.0, 1.0, 0.0),
            Vec3::new(1.0, 2.0, 100.0),
            Quat::IDENTITY,
        ),
        (
            Vec3::new(-25.0, 1.0, 0.0),
            Vec3::new(1.0, 2.0, 100.0),
            Quat::IDENTITY,
        ),
        (
            Vec3::new(0.0, 1.0, 50.0),
            Vec3::new(50.0, 2.0, 1.0),
            Quat::IDENTITY,
        ),
        (
            Vec3::new(0.0, 1.0, -50.0),
            Vec3::new(50.0, 2.0, 1.0),
            Quat::IDENTITY,
        ),
        (
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(10.0, 2.0, 20.0),
            Quat::IDENTITY,
        ),
    ];

    for (pos, size, rot) in walls {
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::from_size(size))),
            MeshMaterial3d(wall_mat.clone()),
            RigidBody::Static,
            Collider::cuboid(size.x, size.y, size.z),
            Transform::from_translation(pos).with_rotation(rot),
        ));
    }

    let ramps = [
        (
            Vec3::new(15.0, 2.0, 20.0),
            Vec3::new(10.0, 0.5, 20.0),
            -0.3f32,
            0.0f32,
        ),
        (
            Vec3::new(15.0, 2.0, -20.0),
            Vec3::new(10.0, 0.5, 20.0),
            0.3f32,
            0.0f32,
        ),
    ];

    for (pos, size, slope, y_rot) in ramps {
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::from_size(size))),
            MeshMaterial3d(wall_mat.clone()),
            RigidBody::Static,
            Collider::cuboid(size.x, size.y, size.z),
            Transform::from_translation(pos)
                .with_rotation(Quat::from_rotation_y(y_rot) * Quat::from_rotation_x(slope)),
        ));
    }

    // Boost Pad
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(5.0, 0.1, 5.0))),
        MeshMaterial3d(boost_mat),
        Collider::cuboid(5.0, 0.5, 5.0),
        Sensor,
        BoostPad { strength: 100.0 },
        Transform::from_xyz(-15.0, 0.1, 0.0),
    ));

    // Jump Pad
    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(5.0, 0.2, 2.0))),
        MeshMaterial3d(jump_mat),
        Collider::cuboid(5.0, 0.5, 2.0),
        Sensor,
        JumpPad { strength: 15.0 },
        Transform::from_xyz(15.0, 0.1, -40.0),
    ));

    let checkpoints = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(15.0, 0.0, 30.0),
        Vec3::new(0.0, 0.0, -40.0),
        Vec3::new(-15.0, 0.0, -10.0),
    ];

    for (i, pos) in checkpoints.iter().enumerate() {
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::new(20.0, 5.0, 1.0))),
            MeshMaterial3d(checkpoint_mat.clone()),
            Collider::cuboid(20.0, 5.0, 1.0),
            Sensor,
            Checkpoint { index: i },
            Transform::from_translation(*pos + Vec3::Y * 2.5),
        ));
    }
}

// --- Systems ---

fn kart_control_system(
    keys: Res<ButtonInput<KeyCode>>,
    mut query: Query<(
        &mut LinearVelocity,
        &mut AngularVelocity,
        &Transform,
        &mut Kart,
    )>,
) {
    for (mut velocity, mut ang_vel, transform, mut kart) in &mut query {
        let mut drive = 0.0;
        let mut steer = 0.0;

        if keys.pressed(KeyCode::ArrowUp) {
            drive += 1.0;
        }
        if keys.pressed(KeyCode::ArrowDown) {
            drive -= 1.0;
        }
        if keys.pressed(KeyCode::ArrowLeft) {
            steer += 1.0;
        }
        if keys.pressed(KeyCode::ArrowRight) {
            steer -= 1.0;
        }

        kart.is_drifting = keys.pressed(KeyCode::ShiftLeft) || keys.pressed(KeyCode::ShiftRight);

        let rotation = transform.rotation;
        let forward = rotation * Vec3::NEG_Z;
        let right = rotation * Vec3::X;

        // Acceleration (Direct velocity modification)
        let current_speed = velocity.0.dot(forward);
        if drive != 0.0 && current_speed.abs() < kart.max_speed {
            // Add velocity in forward direction
            velocity.0 += forward * drive * kart.acceleration;
        }

        // Steering (Direct angular velocity)
        let turn_mult = if kart.is_drifting { 1.5 } else { 1.0 };
        if steer != 0.0 {
            // Apply torque around global Y to stay flat
            ang_vel.0.y += steer * kart.turn_speed * turn_mult * 0.05; // Scaling for direct vel change
        }

        // Lateral Friction / Drift Logic
        // Kill sideways velocity
        let side_vel = velocity.0.dot(right);

        let friction_factor = if kart.is_drifting { 0.95 } else { 0.8 }; // 0.8 = strong grip, 0.95 = slide

        // Remove part of side velocity
        let side_vec = right * side_vel;
        velocity.0 -= side_vec * (1.0 - friction_factor);

        // Limit max angular velocity
        ang_vel.0.y = ang_vel.0.y.clamp(-3.0, 3.0);
    }
}

fn trigger_detection_system(
    mut collision_event_reader: MessageReader<CollisionStart>,
    mut kart_query: Query<&mut LinearVelocity>, // No ExternalForce needed
    boost_query: Query<&BoostPad>,
    jump_query: Query<&JumpPad>,
    checkpoint_query: Query<&Checkpoint>,
    mut game_state_query: Query<&mut GameState>,
) {
    if let Some(mut game_state) = game_state_query.iter_mut().next() {
        for event in collision_event_reader.read() {
            let (e1, e2) = (event.collider1, event.collider2);

            // Need to match entity
            let kart_ent = if let Ok(_) = kart_query.get(e1) {
                Some(e1)
            } else if let Ok(_) = kart_query.get(e2) {
                Some(e2)
            } else {
                None
            };

            if let Some(kart_entity) = kart_ent {
                let other = if kart_entity == e1 { e2 } else { e1 };

                if let Ok(boost) = boost_query.get(other) {
                    if let Ok(mut vel) = kart_query.get_mut(kart_entity) {
                        vel.0 = vel.0.normalize_or_zero() * (vel.0.length() + boost.strength);
                    }
                }

                if let Ok(jump) = jump_query.get(other) {
                    if let Ok(mut vel) = kart_query.get_mut(kart_entity) {
                        vel.0.y += jump.strength;
                    }
                }

                if let Ok(cp) = checkpoint_query.get(other) {
                    let next_cp = game_state.checkpoints_passed % game_state.total_checkpoints;
                    if cp.index == next_cp {
                        game_state.checkpoints_passed += 1;
                        if game_state.checkpoints_passed > 0
                            && game_state.checkpoints_passed % game_state.total_checkpoints == 0
                        {
                            game_state.laps += 1;
                        }
                    }
                }
            }
        }
    }
}

fn game_logic_system(game_state_query: Query<&GameState>, mut exit: MessageWriter<AppExit>) {
    if let Some(game_state) = game_state_query.iter().next() {
        if game_state.laps >= 2 {
            println!("RACE FINISHED! You completed 2 laps.");
            exit.write(AppExit::Success);
        }
    }
}

fn update_ui_system(game_state_query: Query<&GameState>, mut text_query: Query<&mut Text>) {
    if let Some(game_state) = game_state_query.iter().next() {
        for mut text in &mut text_query {
            text.0 = format!("Lap: {}/2", game_state.laps + 1);
        }
    }
}

fn camera_follow_system(
    mut camera_query: Query<&mut Transform, (With<Camera3d>, Without<CameraTarget>)>,
    target_query: Query<(&Transform, &LinearVelocity), With<CameraTarget>>,
    time: Res<Time>,
) {
    if let Some((target_transform, velocity)) = target_query.iter().next() {
        if let Some(mut camera_transform) = camera_query.iter_mut().next() {
            let forward = target_transform.rotation * Vec3::NEG_Z;
            let flat_velocity = Vec3::new(velocity.0.x, 0.0, velocity.0.z);
            let speed = flat_velocity.length();

            let distance = 10.0 + speed * 0.2;
            let height = 5.0 + speed * 0.1;

            let desired_pos =
                target_transform.translation - (forward * distance) + (Vec3::Y * height);

            camera_transform.translation = camera_transform
                .translation
                .lerp(desired_pos, time.delta_secs() * 5.0);
            camera_transform.look_at(target_transform.translation + Vec3::Y * 2.0, Vec3::Y);
        }
    }
}

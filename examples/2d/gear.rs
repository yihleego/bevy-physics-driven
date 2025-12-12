use avian2d::prelude::*;
use bevy::prelude::*;
use physics_driven::debug::debug::Debug2dPlugin;

// 主函数：设置和运行Bevy应用程序
fn main() {
    App::new()
        // 添加Bevy的默认插件，提供基本功能，如窗口管理、输入处理等
        .add_plugins(DefaultPlugins)
        // 添加物理引擎插件，用于处理2D物理模拟
        .add_plugins(PhysicsPlugins::default())
        // 添加自定义的2D调试插件，用于可视化物理组件
        .add_plugins(Debug2dPlugin)
        // 插入全局重力资源，设置为Vec2::ZERO表示零重力，适用于自顶向下的视图
        .insert_resource(Gravity(Vec2::ZERO)) 
        // 初始化拖动状态资源
        .init_resource::<DragState>()
        // 在启动时运行setup系统，用于初始化场景
        .add_systems(Startup, setup)
        // 在每次更新时运行rotate_driver和mouse_drag系统，处理齿轮旋转和鼠标拖动
        .add_systems(Update, (rotate_driver, mouse_drag))
        // 运行应用程序
        .run();
}

#[derive(Resource, Default)]
// DragState 资源：用于存储鼠标拖动操作的状态
struct DragState {
    // 鼠标拖动时创建的临时物理体
    mouse_body: Option<Entity>,
    // 鼠标物理体和被拖动实体之间的关节
    joint: Option<Entity>,
    // 当前被拖动的实体
    dragged_entity: Option<Entity>,
}

#[derive(Component)]
// Driver 组件：标记为驱动齿轮的实体
struct Driver;

#[derive(Component)]
// Draggable 组件：标记为可被鼠标拖动的实体
struct Draggable;

#[derive(Component)]
// Pinned 组件：标记被固定到背景的实体，包含其关节和锚点
struct Pinned {
    // 固定关节的实体ID
    joint: Entity,
    // 锚定点的实体ID
    anchor: Entity,
}

// setup 系统：初始化场景，包括相机、齿轮和说明文本
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    //  spawn 2D相机
    commands.spawn(Camera2d);

    // 定义齿轮的基本参数
    let gear_radius = 60.0;
    let tooth_len = 15.0;
    let tooth_width = 15.0;
    let teeth = 12;

    // 驱动齿轮 (左侧)
    // is_driver 参数为 true，表示这是一个驱动齿轮
    spawn_gear(
        &mut commands,
        &mut meshes,
        &mut materials,
        Vec2::new(-150.0, 0.0),
        gear_radius,
        tooth_len,
        tooth_width,
        teeth,
        true,
    );

    // 其他齿轮 (散布在场景中)
    // is_driver 参数为 false，表示这些是普通齿轮
    let positions = vec![
        Vec2::new(100.0, 100.0),
        Vec2::new(100.0, -100.0),
        Vec2::new(250.0, 0.0),
        Vec2::new(0.0, 200.0),
    ];

    for pos in positions {
        spawn_gear(
            &mut commands,
            &mut meshes,
            &mut materials,
            pos,
            gear_radius,
            tooth_len,
            tooth_width,
            teeth,
            false,
        );
    }

    // 在屏幕上显示操作说明
    commands.spawn((
        Text::new("Drag gears to connect them to the red driver gear!\nWhen you release a gear, it pins to the background."),
        Node {
            position_type: PositionType::Absolute,
            top: Val::Px(12.0),
            left: Val::Px(12.0),
            ..default()
        },
    ));
}

// spawn_gear 函数：生成一个齿轮实体，包括其物理碰撞体、视觉表现和驱动逻辑
fn spawn_gear(
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<ColorMaterial>>,
    pos: Vec2,          // 齿轮位置
    radius: f32,        // 齿轮半径
    t_len: f32,         // 齿的长度
    t_width: f32,       // 齿的宽度
    teeth: usize,       // 齿的数量
    is_driver: bool,    // 是否为驱动齿轮
) {
    // 1. 创建复合碰撞体
    let mut parts = Vec::new();
    // 齿轮中心圆盘碰撞体
    parts.push((Vec2::ZERO, 0.0, Collider::circle(radius)));

    // 计算每个齿的旋转步长
    let angle_step = std::f32::consts::TAU / teeth as f32;
    for i in 0..teeth {
        let angle = i as f32 * angle_step;
        // 齿的偏移量，使其稍微向外延伸
        let offset = Vec2::from_angle(angle) * (radius + t_len * 0.5 - 2.0);
        // 添加齿的矩形碰撞体
        parts.push((offset, angle, Collider::rectangle(t_len, t_width)));
    }

    // 将所有碰撞体组合成一个复合碰撞体
    let collider = Collider::compound(parts);

    // 根据是否为驱动齿轮设置颜色
    let color = if is_driver {
        Color::srgb(0.9, 0.3, 0.3) // 红色用于驱动齿轮
    } else {
        Color::srgb(0.4, 0.6, 0.9) // 蓝色用于其他齿轮
    };

    // 生成齿轮实体及其组件
    let gear_ent = commands
        .spawn((
            // 2D网格：圆形
            Mesh2d(meshes.add(Circle::new(radius))),
            // 2D材质：彩色
            MeshMaterial2d(materials.add(ColorMaterial::from(color))),
            // 变换组件：位置
            Transform::from_translation(pos.extend(0.0)),
            // 刚体类型：动态
            RigidBody::Dynamic,
            // 碰撞体
            collider,
            // 可拖动组件
            Draggable,
            // 质量
            Mass(5.0),
            // 摩擦力
            Friction::new(0.2),
            // 恢复系数 (弹性)
            Restitution::new(0.0),
            // 角阻尼：模拟“桌面”摩擦，使其最终停止旋转
            AngularDamping(2.0),
            // 线性阻尼
            LinearDamping(2.0),
        ))
        .id();

    // 2. 视觉齿 (子实体)
    // 创建齿的网格和材质
    let t_mesh = meshes.add(Rectangle::new(t_len, t_width));
    let t_mat = materials.add(ColorMaterial::from(color));

    for i in 0..teeth {
        let angle = i as f32 * angle_step;
        // 齿的偏移量
        let offset = Vec2::from_angle(angle) * (radius + t_len * 0.5 - 2.0);
        // 为每个齿生成一个子实体
        let child = commands
            .spawn((
                Mesh2d(t_mesh.clone()),
                MeshMaterial2d(t_mat.clone()),
                Transform::from_translation(offset.extend(-0.1))
                    .with_rotation(Quat::from_rotation_z(angle)),
            ))
            .id();
        // 将齿作为子实体添加到齿轮上
        commands.entity(gear_ent).add_child(child);
    }

    // 3. 驱动器逻辑
    if is_driver {
        // 将齿轮标记为驱动器
        commands.entity(gear_ent).insert(Driver);

        // 将驱动器固定到背景
        let anchor = commands
            .spawn((
                Transform::from_translation(pos.extend(0.0)),
                RigidBody::Static,
            ))
            .id();

        // 创建一个旋转关节将驱动器固定在锚点上
        let joint = commands.spawn(RevoluteJoint::new(anchor, gear_ent)).id();

        // 将 Pinned 组件插入到驱动齿轮上
        commands.entity(gear_ent).insert(Pinned { joint, anchor });
    }
}

// rotate_driver 系统：持续旋转驱动齿轮
fn rotate_driver(mut query: Query<&mut AngularVelocity, With<Driver>>) {
    // 遍历所有带有 Driver 组件的实体，并设置其角速度
    for mut vel in &mut query {
        vel.0 = -2.5; // 设置一个负的角速度，使其逆时针旋转
    }
}

// mouse_drag 系统：处理鼠标拖动事件
fn mouse_drag(
    mut commands: Commands,
    mouse: Res<ButtonInput<MouseButton>>,
    mut drag_state: ResMut<DragState>,
    windows: Query<&Window>,
    camera: Query<(&Camera, &GlobalTransform)>,
    spatial: SpatialQuery,
    draggable_q: Query<Entity, With<Draggable>>,
    mut bodies: Query<&mut Transform, With<RigidBody>>,
    pinned_q: Query<&Pinned>,
) {
    // 获取窗口和相机信息
    let Ok(window) = windows.single() else { return };
    let Ok((cam, cam_t)) = camera.single() else {
        return;
    };

    // 如果鼠标在窗口内
    if let Some(cp) = window.cursor_position() {
        // 将鼠标位置转换为世界坐标
        if let Ok(world_pos) = cam.viewport_to_world_2d(cam_t, cp) {
            // 开始拖动：鼠标左键刚按下时
            if mouse.just_pressed(MouseButton::Left) {
                // 查找鼠标下的实体
                let hits = spatial.point_intersections(world_pos, &SpatialQueryFilter::default());

                for ent in hits {
                    // 只拖动带有 Draggable 组件的实体
                    if draggable_q.contains(ent) {
                        // 如果实体已被固定，则取消固定
                        if let Ok(pinned) = pinned_q.get(ent) {
                            commands.entity(pinned.joint).despawn();
                            commands.entity(pinned.anchor).despawn();
                            commands.entity(ent).remove::<Pinned>();
                        }

                        // 在鼠标位置生成一个临时的运动学刚体来拉动齿轮
                        let mouse_body = commands
                            .spawn((
                                Transform::from_translation(world_pos.extend(0.0)),
                                RigidBody::Kinematic,
                            ))
                            .id();

                        // 使用一个刚性关节将齿轮连接到鼠标体
                        let joint = commands
                            .spawn(
                                RevoluteJoint::new(mouse_body, ent)
                                    .with_point_compliance(0.0000001), // 设置关节的柔度
                            )
                            .id();

                        // 更新拖动状态
                        drag_state.mouse_body = Some(mouse_body);
                        drag_state.joint = Some(joint);
                        drag_state.dragged_entity = Some(ent);
                        break;
                    }
                }
            }

            // 更新拖动：鼠标左键持续按下时
            if mouse.pressed(MouseButton::Left) {
                if let Some(mb) = drag_state.mouse_body {
                    // 更新运动学鼠标体的位置
                    if let Ok(mut t) = bodies.get_mut(mb) {
                        t.translation = world_pos.extend(0.0);
                    }
                }
            }

            // 结束拖动：鼠标左键刚释放时
            if mouse.just_released(MouseButton::Left) {
                // 移除鼠标体
                if let Some(mb) = drag_state.mouse_body {
                    commands.entity(mb).despawn();
                }
                // 移除关节
                if let Some(j) = drag_state.joint {
                    commands.entity(j).despawn();
                }

                // 将齿轮固定到当前位置
                if let Some(gear_ent) = drag_state.dragged_entity {
                    if let Ok(t) = bodies.get(gear_ent) {
                        let pos = t.translation;
                        // 创建一个静态锚点
                        let anchor = commands
                            .spawn((Transform::from_translation(pos), RigidBody::Static))
                            .id();

                        // 创建一个旋转关节
                        let joint = commands.spawn(RevoluteJoint::new(anchor, gear_ent)).id();

                        // 将 Pinned 组件插入到齿轮上
                        commands.entity(gear_ent).insert(Pinned { joint, anchor });
                    }
                }

                // 重置拖动状态
                drag_state.mouse_body = None;
                drag_state.joint = None;
                drag_state.dragged_entity = None;
            }
        }
    }
}

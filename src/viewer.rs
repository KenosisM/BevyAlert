//! VXL 查看器 —— 把读到的体素段用 Bevy 的 3D 管线画出来。
//!
//! 本步目标只有一个：**证明「RA2 的 VXL 资产」能被 Bevy 的 wgpu 管线渲染出来**。
//! 所以摆放是糙的：每个 .vxl 并排摆开，各算各的；真实整机的部件定位要用 HVA 的
//! 3×4 矩阵（车体 / 炮塔 / 炮管各自的位移与旋转），那是下一步。
//!
//! 相机与光照都是按"看清模型"选的，不是按游戏的等距视角（游戏是 2:1 等距 + VPL 光照）。

use bevy::prelude::*;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};

use crate::voxel_mesh;
use crate::vxl;

#[derive(Resource)]
pub struct ViewerFiles(pub Vec<String>);

/// 设了环境变量 `RA2_SHOT=<path.png>` 就自动截一张图再退出 ——
/// 用来在没有人盯屏幕的时候自证"确实画出来了"（本模型读不了图片，只能靠像素统计）。
#[derive(Resource)]
struct ShotPlan {
    path: String,
    take_at: f32,
    exit_at: f32,
}

pub fn run(paths: Vec<String>) {
    if paths.is_empty() {
        println!("usage: ra2engine view <file.vxl> [more.vxl ...]");
        return;
    }
    let mut app = App::new();
    app.add_plugins(DefaultPlugins)
        .insert_resource(ViewerFiles(paths))
        .add_systems(Startup, setup);
    if let Ok(p) = std::env::var("RA2_SHOT") {
        app.insert_resource(ShotPlan { path: p, take_at: 2.5, exit_at: 5.0 });
        app.add_systems(Update, auto_shot);
    }
    app.run();
}

fn auto_shot(
    mut commands: Commands,
    time: Res<Time>,
    plan: Res<ShotPlan>,
    mut taken: Local<bool>,
    mut exit: MessageWriter<AppExit>,
) {
    let t = time.elapsed_secs();
    if !*taken && t >= plan.take_at {
        println!("shot: capturing -> {}", plan.path);
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(plan.path.clone()));
        *taken = true;
    }
    if t >= plan.exit_at {
        exit.write(AppExit::Success);
    }
}

fn setup(
    mut commands: Commands,
    files: Res<ViewerFiles>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let material = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        perceptual_roughness: 0.95,
        metallic: 0.0,
        ..default()
    });

    let mut cursor_x = 0.0f32;
    let mut lo = Vec3::splat(f32::MAX);
    let mut hi = Vec3::splat(f32::MIN);
    let (mut total_faces, mut total_skipped, mut total_verts) = (0usize, 0usize, 0usize);

    for path in &files.0 {
        let Ok(bytes) = std::fs::read(path) else {
            println!("  [skip] cannot read {path}");
            continue;
        };
        let Some(v) = vxl::parse_vxl(&bytes) else {
            println!("  [skip] parse failed {path}");
            continue;
        };
        let parts: Vec<(&vxl::VxlSection, [f32; 3])> =
            v.sections.iter().map(|s| (s, [0.0f32, 0.0, 0.0])).collect();
        let (mut mesh, stats) = voxel_mesh::build_mesh(&v.palette, &parts, &[], false, 1.0);

        let short = path.rsplit(['/', '\\']).next().unwrap_or(path);
        let (mut ext_lo, mut ext_hi) = (
            Vec3::splat(f32::MAX),
            Vec3::splat(f32::MIN),
        );
        for s in &v.sections {
            let e = Vec3::new(s.x_size as f32, s.z_size as f32, s.y_size as f32);
            ext_lo = ext_lo.min(Vec3::ZERO);
            ext_hi = ext_hi.max(e);
        }
        if ext_hi.x < ext_lo.x {
            ext_lo = Vec3::ZERO;
            ext_hi = Vec3::ONE;
        }
        let width = (ext_hi - ext_lo).x.max(1.0);
        let shift = Vec3::new(cursor_x - ext_lo.x, 0.0, 0.0);

        mesh.translate_by(shift);
        lo = lo.min(ext_lo + shift);
        hi = hi.max(ext_hi + shift);
        cursor_x += width + 12.0;

        let handle = meshes.add(mesh);
        commands.spawn((Mesh3d(handle), MeshMaterial3d(material.clone())));

        total_faces += stats.faces;
        total_skipped += stats.skipped_hidden;
        total_verts += stats.vertices;
        println!(
            "  {short:<16} {:<5}体素  可见面 {:>6}  隐面剔除 {:>6}  顶点 {:>7}",
            v.sections.iter().map(|s| s.voxels.len()).sum::<usize>(),
            stats.faces,
            stats.skipped_hidden,
            stats.vertices
        );
    }

    if hi.x < lo.x {
        lo = Vec3::ZERO;
        hi = Vec3::ONE;
    }
    let centre = (lo + hi) * 0.5;
    let radius = ((hi - lo).length() * 0.5).max(4.0);

    // 相机取距：几个部件是**并排**摆的，包围盒往往是"很扁很长"的一条（实测 117 × 12 × 34）。
    // Bevy 的 `fov` 是**垂直**视角，横着摆的长条最容易从左右两侧被切掉，
    // 所以按"最长边"而不是"对角半径"退到能容下的距离。
    let extent = (hi - lo).max_element().max(1.0);
    let distance = extent * 1.05 + (hi - lo).length() * 0.25;
    commands.spawn((
        Camera3d::default(),
        Transform::from_translation(centre + Vec3::new(0.7, 0.55, 0.95).normalize() * distance)
            .looking_at(centre, Vec3::Y),
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 60_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::YXZ, 0.9, -0.55, 0.0)),
    ));
    commands.spawn((
        PointLight {
            intensity: 9_000_000.0,
            range: radius * 12.0,
            ..default()
        },
        Transform::from_translation(centre + Vec3::new(radius, radius * 2.0, radius)),
    ));
    commands.insert_resource(GlobalAmbientLight {
        color: Color::WHITE,
        brightness: 600.0,
        ..default()
    });

    println!(
        "viewer: {} 个文件, 合计可见面 {} (剔除隐面 {}), 顶点 {}; 包围盒 {:?}..{:?}",
        files.0.len(),
        total_faces,
        total_skipped,
        total_verts,
        lo.to_array(),
        hi.to_array()
    );
}

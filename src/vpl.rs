//! voxels.vpl 解析 + VPL 光照页计算 —— 对等移植。
//!
//! 来源：`I:\bevy\RA2REASM\src\compat\vpl.cpp`（`parseVpl`）与
//! `I:\bevy\RA2REASM\src\compat\vxlrender.cpp`（`preCalculateVplLighting`）。
//!
//! 为什么要 VPL：体素不是「乘一个颜色」来打光的。法线选的是一张**光照页**，而那张页是
//! 整张 256 项调色板的重映射。所以体素的颜色是**换索引**而不是改亮度 —— 这就是为什么
//! 直接 Lambert 乘色永远对不上游戏：它表达不了页自己的色相偏移和截断。

use crate::mat4::{self, DEG2RAD};
use crate::vxl_normals;

#[derive(Debug, Clone, Default)]
pub struct Vpl {
    pub first_remap: u32,
    pub last_remap: u32,
    pub num_sections: u32,
    /// 每张 256 字节。
    pub lookup_sections: Vec<Vec<u8>>,
    pub valid: bool,
}

impl Vpl {
    /// 引擎把页钳到最后一个 section 而不是回绕。
    pub fn palette_index(&self, page: i32, color: u8) -> u8 {
        if self.lookup_sections.is_empty() {
            return color;
        }
        let last = self.num_sections as i32 - 1;
        let p = if page < last { page } else { last };
        let p = if p < 0 { 0 } else { p as usize };
        self.lookup_sections[p][color as usize]
    }
}

pub fn parse_vpl(data: &[u8]) -> Result<Vpl, String> {
    let u32_at = |at: usize| -> u32 {
        u32::from_le_bytes([data[at], data[at + 1], data[at + 2], data[at + 3]])
    };
    if data.len() < 16 + 768 {
        return Err("too small to be a VPL".into());
    }
    let mut out = Vpl {
        first_remap: u32_at(0),
        last_remap: u32_at(4),
        num_sections: u32_at(8),
        ..Default::default()
    };
    // u32(12) 未知且未使用。

    let table_start = 16 + 768;
    if out.num_sections == 0 || table_start + out.num_sections as usize * 256 > data.len() {
        return Err("VPL lookup tables run past the end of the file".into());
    }
    out.lookup_sections.reserve(out.num_sections as usize);
    for i in 0..out.num_sections as usize {
        let p = table_start + i * 256;
        out.lookup_sections.push(data[p..p + 256].to_vec());
    }
    out.valid = true;
    Ok(out)
}

/// `vxlrender.cpp preCalculateVplLighting`：把段法线表里的每个法线映射到该物体朝向在
/// voxels.vpl 里对应的光照页。Blinn-Phong，由 WorldAlteringEditor 逆出来的。
///
/// 返回 256 字节（每项是一页的亮度值）。
pub fn pre_calculate_vpl_lighting(normals_mode: u8, direction: f32) -> Vec<u8> {
    let normals = vxl_normals::vxl_normals(normals_mode);
    let normal_count = normals.len();

    // YR 的主光 = 单位 -X 绕 Z 转 45 度；TS 从另一个角度打光。
    let yr_light = mat4::transform_normal(&[-1.0f32, 0.0, 0.0], &mat4::rot_z(45.0 * DEG2RAD));
    let rotation_from_facing = 6.283_185_5_f32 * direction / 256.0;
    let light = mat4::transform_normal(
        &yr_light,
        &mat4::rot_z(rotation_from_facing - 45.0 * DEG2RAD),
    );
    let viewer = [0.0f32, 0.0, 1.0];
    let halfway = mat4::normalize3(&mat4::add3(&light, &viewer));

    const K_SPECULAR_STRENGTH: f32 = 3.0; // YR 用的常数

    let mut pages = vec![0u8; 256];
    for i in 0..normal_count {
        let n = normals[i];
        let diffuse = mat4::dot3(&n, &light).max(0.0);
        let halfway_dot = mat4::dot3(&n, &halfway);
        let mut specular =
            halfway_dot / (K_SPECULAR_STRENGTH - halfway_dot * K_SPECULAR_STRENGTH + halfway_dot);
        specular = specular.max(0.0);
        pages[i] = ((diffuse + specular) * 16.0).clamp(0.0, 255.0) as u8;
    }
    // 三个特殊法线索引中性受光。
    pages[253] = 16;
    pages[254] = 16;
    pages[255] = 16;
    pages
}

/// `vxlrender.cpp voxelWorldMatrix`：等距投影的固定世界旋转。
pub fn voxel_world_matrix() -> mat4::Mat4 {
    let world = mat4::rot_x(60.0 * DEG2RAD);
    let world = mat4::mul(&mat4::rot_y(180.0 * DEG2RAD), &world);
    mat4::mul(&mat4::rot_z(-45.0 * DEG2RAD), &world)
}

/// `vxlrender.cpp voxelBaseMatrices`。
pub fn voxel_base_matrices() -> (mat4::Mat4, mat4::Mat4, mat4::Mat4) {
    let canvas = 400.0f32;
    let persp = mat4::ortho_gl(canvas, canvas, 1.0, canvas);
    let lookat = mat4::look_at(&[0.0, 0.0, -10.0], &[0.0, 0.0, 0.0], &[0.0, 1.0, 0.0]);
    let trans = mat4::translate(0.0, 0.0, 10.0);
    (persp, lookat, trans)
}

/// `vxlrender.cpp voxelObjectMatrix`。
pub fn voxel_object_matrix(
    direction: f32,
    tilt_pitch_deg: f32,
    tilt_yaw_deg: f32,
    turret_voxel_offset: f32,
) -> mat4::Mat4 {
    // Rz(tiltYaw) * Ry(pitch) * Rz(facing - tiltYaw)，行向量从右往左作用。
    //
    // pitch 必须绕模型自己的 Y 轴，不是 X 轴：行向量先把模型转到 `facing`，此时 X 指向车头，
    // 绕 X 转会把车翻倒而不是抬头。facing 之后再 Ry(pitch) 才是绕侧轴俯仰。
    let facing_yaw = (90.0 - direction / 256.0 * 360.0) - tilt_yaw_deg;
    let object = mat4::rot_z(tilt_yaw_deg * DEG2RAD);
    let object = mat4::mul(&mat4::rot_y(tilt_pitch_deg * DEG2RAD), &object);
    let object = mat4::mul(&mat4::rot_z(facing_yaw * DEG2RAD), &object);
    mat4::mul(&mat4::translate(0.18 * turret_voxel_offset, 0.0, 0.0), &object)
}

/// `vxlrender.cpp voxelLightingForSection`：段的法线→页表，以及 Lambert 兜底要的
/// 段空间光方向。
///
/// `section_frame` 是已经乘进 hvaMultiplier 与 minBounds 的那个 T(minBounds)*frameRot。
pub fn voxel_lighting_for_section(
    normals_mode: u8,
    direction: f32,
    object: &mat4::Mat4,
    world: &mat4::Mat4,
    section_frame: &mat4::Mat4,
) -> (mat4::Vec3, Vec<u8>) {
    let normal_to_page = pre_calculate_vpl_lighting(normals_mode, direction);

    // 世界光透过段自身完整变换的逆，得到段空间光方向（与参考实现的 lightDirs[s] 同配方）。
    let shadow_transform_pitch = 210.0 * DEG2RAD;
    let shadow_transform_yaw = 120.0 * DEG2RAD;
    let shadow_transform = mat4::mul(
        &mat4::rot_z(shadow_transform_pitch),
        &mat4::rot_y(shadow_transform_yaw),
    );
    let v = mat4::mul(
        &mat4::mul(&mat4::mul(object, world), section_frame),
        &shadow_transform,
    );
    let light_dir = match mat4::invert(&v) {
        Some(inv) => {
            let mut dump = [0.0f32; 16];
            for i in 0..4 {
                for j in 0..4 {
                    dump[i * 4 + j] = inv[i][j];
                }
            }
            mat4::extract_rotation_vector(&dump)
        }
        None => [0.0, 0.0, 1.0],
    };
    (light_dir, normal_to_page)
}

//! 体素 → Bevy Mesh（**可见面提取**）。
//!
//! 做法：先把该段的体素灌进一张三维占用表，然后**只对没有邻居的那一面出三角形** ——
//! 实心车体的内部体素一个面都不出，面数从 `体素数 × 6` 掉到表面量级。
//!
//! ## 坐标系（关键，别凭直觉改）
//! 由 `I:\WeaverWeb\js\core\vox.js` 的实测结论（用户 2026-10-14 拍板）：
//! **VXL 的 x = 车长方向（朝向是 -x）、y = 左、z = 上**。
//! 而 Bevy 世界是 **Y-up**，所以体素坐标到世界坐标要做一次轴换位：
//!
//! ```text
//! world = (vxl.x, vxl.z, vxl.y)      // VXL 的「上」变成世界的 +Y
//! ```
//!
//! 该映射的行列式 = -1（含一次镜像），所以**模型朝向可能与游戏里左右相反**；
//! WeaverWeb 那边也踩过同一个坑（它写的是 `(vx,vy,vz)=(-my,-mx,+mz)`，并注明
//! 「推导推不出来，只能实测」）。本步只要能看到模型，朝向留待与游戏对照后再定。

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;

use crate::hva::Hva;
use crate::vxl::VxlSection;

/// 六个面：方向向量 + 该面四个角在单位立方体里的位置（体素坐标，以最小角为原点）。
/// 顶点按**从外面看逆时针**排列（Bevy 默认 `FrontFace::Ccw`）。
const FACES: [([i32; 3], [[f32; 3]; 4]); 6] = [
    ([1, 0, 0], [[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [1.0, 1.0, 1.0], [1.0, 0.0, 1.0]]),
    ([-1, 0, 0], [[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 1.0], [0.0, 1.0, 0.0]]),
    ([0, 1, 0], [[0.0, 1.0, 0.0], [0.0, 1.0, 1.0], [1.0, 1.0, 1.0], [1.0, 1.0, 0.0]]),
    ([0, -1, 0], [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 0.0, 1.0], [0.0, 0.0, 1.0]]),
    ([0, 0, 1], [[0.0, 0.0, 1.0], [1.0, 0.0, 1.0], [1.0, 1.0, 1.0], [0.0, 1.0, 1.0]]),
    ([0, 0, -1], [[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0], [1.0, 0.0, 0.0]]),
];

/// 体素坐标 → 世界坐标（VXL 上轴 z 变世界 Y）。
fn to_world(p: [f32; 3]) -> [f32; 3] {
    [p[0], p[2], p[1]]
}

/// sRGB（显示器编码）→ 线性。
///
/// 顶点色 attribute 是**线性**值：Bevy 的 PBR 把 `in.color` 直接乘进 `base_color`
/// （`bevy_pbr/src/render/pbr_fragment.wesl:52` —— `base_color = in.color`），整条管线
/// 都在线性空间跑，直到输出阶段才编码回 sRGB。而调色板里的字节是 **sRGB 编码的显示
/// 值**，原样丢进去等于少做一次解码 —— 颜色整体偏亮（中灰 0.5 会显示成 0.73，
/// 亮 47%）。这是「单位一片惨白」的其中一条。
fn srgb_to_linear(c: f32) -> f32 {
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

fn palette_rgba(pal: &[u8], c: u8, albedo: f32) -> [f32; 4] {
    let i = c as usize * 3;
    if i + 2 < pal.len() {
        [
            srgb_to_linear(pal[i] as f32 / 255.0) * albedo,
            srgb_to_linear(pal[i + 1] as f32 / 255.0) * albedo,
            srgb_to_linear(pal[i + 2] as f32 / 255.0) * albedo,
            1.0,
        ]
    } else {
        [1.0, 0.0, 1.0, 1.0]
    }
}

pub struct MeshStats {
    pub faces: usize,
    pub vertices: usize,
    pub skipped_hidden: usize,
}

/// 1 个体素单位折合多少世界单位。
///
/// RA2 的 vxl 渲染里 1 体素单位 = 1 **屏幕像素**（`vpl::voxel_base_matrices` 的正交投影
/// 就是像素 1:1），而 1 格 = 60 屏幕像素 = 1 个世界单位 ⇒ 除以 60。
pub const PX_TO_WORLD: f32 = 1.0 / 60.0;

/// 把若干段拼成一个 Mesh。`offset` 是各段在体素空间里的平移（单位：体素），
/// 用来把车体 / 炮塔 / 炮管摆到一起（本步先传零，部件相对位置要用 HVA 才算得准）。
///
/// `hvas[i]` 是 `parts[i]` 对应的 HVA（没有就传 `None`）：只取 **frame 0** 的矩阵，
/// 它把这一段从"局部"摆到"模型空间"。段号按 `parts` 里的序号算。
///
/// * `scaled = false` —— 位置直接用**体素索引**。`viewer` 用这个，它只关心形状。
/// * `scaled = true` —— 按 `sec.scale_of(轴)` 与 `sec.bounds` 折成**世界单位**，这才是
///   VXL 的真实尺寸（`x_size × scale_of(0)` 就是模型在该轴的跨度）；`vxl_gpu.rs` 的
///   `section_frame = T(bounds) * frameRot` 用的也正是这两项。
///
/// 颜色 = **体素色索引直接查单位调色板得到的真 RGB**（线性空间），每个面另带一条几何法线。
/// 光照交给场景里的真光源（见 `game.rs` 的光源段），不再按朝向烘颜色。
///
/// `albedo` = 漫反射本色增益（1.0 = 调色板本色），在**线性空间**乘，随后进 PBR 的
/// `base_color`。乘在这里而不是乘渲染结果，是因为它调的正是"物体表面有多亮"这件事
/// —— 光源那边的 `--sun-gain/--sky` 调的是"有多少光落上去"，两者是独立旋钮。
pub fn build_mesh(
    palette: &[u8],
    parts: &[(&VxlSection, [f32; 3])],
    hvas: &[Option<&Hva>],
    scaled: bool,
    albedo: f32,
) -> (Mesh, MeshStats) {
    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut normals: Vec<[f32; 3]> = Vec::new();
    let mut colors: Vec<[f32; 4]> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();
    let mut stats = MeshStats { faces: 0, vertices: 0, skipped_hidden: 0 };

    for (pi, (sec, offset)) in parts.iter().enumerate() {
        let xs = sec.x_size as usize;
        let ys = sec.y_size as usize;
        let zs = sec.z_size as usize;
        if xs == 0 || ys == 0 || zs == 0 {
            continue;
        }
        // 体素索引 → 世界单位的两项：每格边长、以及段的 minBounds。
        let (s, b) = if scaled {
            (
                [
                    sec.scale_of(0) * PX_TO_WORLD,
                    sec.scale_of(1) * PX_TO_WORLD,
                    sec.scale_of(2) * PX_TO_WORLD,
                ],
                [
                    sec.bounds[0] * PX_TO_WORLD,
                    sec.bounds[1] * PX_TO_WORLD,
                    sec.bounds[2] * PX_TO_WORLD,
                ],
            )
        } else {
            ([1.0f32, 1.0, 1.0], [0.0f32, 0.0, 0.0])
        };
        // HVA frame 0：把这一段从局部摆到模型空间。行向量矩阵 `v' = v·R + T`；
        // 平移行要先按 `hvaMultiplier × 格边长` 缩放（照 `vxl_gpu.rs:610-612`），
        // 再随整体折成世界单位。
        let hva = hvas.get(pi).and_then(|o| *o).and_then(|h| {
            if !h.has_section(pi as i32) {
                return None;
            }
            let mut m = h.matrix_for(pi as i32, 0);
            m[12] *= sec.hva_multiplier * sec.scale_of(0) * PX_TO_WORLD;
            m[13] *= sec.hva_multiplier * sec.scale_of(1) * PX_TO_WORLD;
            m[14] *= sec.hva_multiplier * sec.scale_of(2) * PX_TO_WORLD;
            Some(m)
        });
        let mut occ = vec![false; xs * ys * zs];
        let at = |x: usize, y: usize, z: usize| (z * ys + y) * xs + x;
        for v in &sec.voxels {
            occ[at(v.x as usize, v.y as usize, v.z as usize)] = true;
        }

        for v in &sec.voxels {
            let (x, y, z) = (v.x as i32, v.y as i32, v.z as i32);
            // 索引 → 真 RGB。不再经过 `voxels.vpl` 的按朝向页重映射：那是 26 年前用调色板
            // 模拟光照的做法，光照应当由场景里的真光源给出（见 `game.rs` 的光源段）。
            let rgba = palette_rgba(palette, v.color, albedo);
            for (dir, corners) in FACES.iter() {
                let (nx, ny, nz) = (x + dir[0], y + dir[1], z + dir[2]);
                let blocked = nx >= 0
                    && ny >= 0
                    && nz >= 0
                    && (nx as usize) < xs
                    && (ny as usize) < ys
                    && (nz as usize) < zs
                    && occ[at(nx as usize, ny as usize, nz as usize)];
                if blocked {
                    stats.skipped_hidden += 1;
                    continue;
                }
                let base = positions.len() as u32;
                for c in corners {
                    let local = [
                        b[0] + (c[0] + v.x as f32 + offset[0]) * s[0],
                        b[1] + (c[1] + v.y as f32 + offset[1]) * s[1],
                        b[2] + (c[2] + v.z as f32 + offset[2]) * s[2],
                    ];
                    let n = [dir[0] as f32, dir[1] as f32, dir[2] as f32];
                    match hva {
                        Some(m) => {
                            positions.push(to_world([
                                local[0] * m[0] + local[1] * m[4] + local[2] * m[8] + m[12],
                                local[0] * m[1] + local[1] * m[5] + local[2] * m[9] + m[13],
                                local[0] * m[2] + local[1] * m[6] + local[2] * m[10] + m[14],
                            ]));
                            normals.push(to_world([
                                n[0] * m[0] + n[1] * m[4] + n[2] * m[8],
                                n[0] * m[1] + n[1] * m[5] + n[2] * m[9],
                                n[0] * m[2] + n[1] * m[6] + n[2] * m[10],
                            ]));
                        }
                        None => {
                            positions.push(to_world(local));
                            normals.push(to_world(n));
                        }
                    }
                    colors.push(rgba);
                }
                indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
                stats.faces += 1;
            }
        }
    }

    stats.vertices = positions.len();
    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors);
    mesh.insert_indices(Indices::U32(indices));
    (mesh, stats)
}

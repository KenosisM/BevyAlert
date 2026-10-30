//! 地形几何 —— **全工程唯一的建模来源**。
//!
//! 每格 = 一个孤立的菱形柱：
//! * 顶面四角高度 = 本格 `level` + `ramp::corner_heights` 对应位（1 层 = `LEVEL_HEIGHT`）；
//! * 四个侧壁落到地面 y = 0，两端都在地面上的边不建（平地的四圈省掉）；
//! * 不查邻居、不补落差墙、不做"抬半格消锯齿"。相邻格高低不同时共享边本来就不等高，
//!   靠闭合柱体 + 深度缓冲决出可见性。
//!
//! 贴不贴图与几何无关：`build_mesh` 出一份 `TerrainMesh`（位置 / 法线 / UV / 顶点色），
//! `game.rs` 按 `[Render] LogicMesh` 决定挂不挂 `ATTRIBUTE_COLOR`、材质用 `unlit` 还是
//! 图集。**这里没有模式分支、没有第二套算法、没有需要两处同步的常数。**
//!
//! 世界坐标：格 (rx, ry) 在 XZ 平面上占 1×1，高度沿 Y，1 单位 = 1 个 level。
//! 相机负责把它投影成 RA2 的 2:1 菱形（见 `game.rs`）。

use std::collections::HashMap;

use crate::costgrid::{TerrainBuild, TerrainCell};
use crate::ramp;

/// 平地顶面纹理的槽尺寸 —— **正视图**，不是砖图那种投影好的菱形。
///
/// 一块平地砖在屏幕上占 60×30 的菱形、面积 900 像素。正视图 30×30 与它面积相等，
/// 也就是「一个屏幕像素对应一个纹素」，没有凭空造出来的细节；这里取 32 是为了让槽对齐
/// 2 的幂、采样不踩槽边。再放大只是插值出来的假锐利，不会多出信息。
pub const SLOT_W: u32 = 32;
pub const SLOT_H: u32 = 32;
/// 图集固定列数（行数按需）
pub const ATLAS_COLS: u32 = 64;
/// extra（崖壁 / 坡面）的槽尺寸。RA2 里最大那张是 60×74（崖壁，高 4 级），
/// 其余坡面的是 59×29 / 30×24 / 30×8 之类，统一给一个够大的槽。
pub const XSLOT_W: u32 = 64;
pub const XSLOT_H: u32 = 80;
/// 坡砖**逆推**后的槽尺寸（图集第三块）。
///
/// 逆推输出的纹理是「斜面在 (u,v) 空间的正视图」，边长要跟屏幕投影的实际跨度对齐，
/// 这样每个屏幕像素正好对应一个纹素、重采样不损失细节。最长的跨度出现在对角坡上
/// （两角抬升，投影是 30×15 的斜条，斜边长 √(30²+15²) ≈ 34）；两层陡坡到 60 出头，
/// 统一给 64 足够，每块砖只用自己的实际尺寸子矩形（UV 按实际尺寸算）。
pub const SSLOT_W: u32 = 64;
pub const SSLOT_H: u32 = 64;
/// 图集最后一行：**逆推后的通用崖壁**（一格宽 × 崖壁高的正视矩形）。
pub const WALL_SLOT_H: u32 = 96;
/// 通用崖壁槽的宽度：**两张一格宽的崖壁并排**（第二张左右镜像）。
///
/// 【为什么要两张】崖壁纹理是「一条格边」宽（32px），而补半格补出来的那堵墙站在
/// 对角弦上 —— 弦长 60px，正好是两条格边。只有一张时整幅铺上去要横向拉宽近两倍，
/// 岩石纹理被抻成一片糊。存两张以后：柱子的侧面只吃左半张（一条边 1:1），
/// 补半格的墙吃整两张（弦 1:1），两边都不再拉伸。
///
/// 第二张做成镜像，所以两张拼起来的接缝左右对称，看不出是同一张重复。
pub const WALL_SLOT_W: u32 = SLOT_W * 2;

/// 一个 level 折合多少世界单位（世界 XZ 上 1 格 = 1 单位）。
///
/// 推导：RA2 的等距是 **2:1**（砖 60 宽 30 高）。这种投影下 1×1 的格在屏幕上的菱形
/// 高度 = √2/2 世界单位，对应 RA2 的 30px；而 RA2 里 level 每升 1 级砖在屏幕上抬 15px
/// （正好半个砖高）。所以 level 高度 = (√2/2 ÷ 2) ÷ (√3/2) = 1/√6 世界单位。
///
/// 单位和 `world.rs` 的 `surface_height` 换算保持一致：那边给的是像素
/// （乘过 `pitch_y`），除以 `pitch_y` 得到 level 数，再乘这里就是世界高度。
pub const LEVEL_HEIGHT: f32 = 0.408_248_3;

/// 等距屏幕 1 像素对应的世界位移（2:1 等距：世界 +X 投 (+30,+15)px、+Z 投 (−30,+15)px）。
/// 于是"屏幕往右 1px"= 世界 (1/60, 0, −1/60)、"屏幕往下 1px"= 世界 (1/30, 0, 1/30)。
/// 有了它就能把参考实现里**以像素给出的** extra 位置（ExtraX/ExtraY/ExtraW/ExtraH）
/// 原样搬到世界坐标里 —— 不用去猜它盖的是哪条边。
const E_SX: [f32; 3] = [1.0 / 60.0, 0.0, -1.0 / 60.0];
const E_SY: [f32; 3] = [1.0 / 30.0, 0.0, 1.0 / 30.0];
/// 图集里一块砖的 UV 槽
#[derive(Clone, Copy, Debug)]
pub struct TileSlot {
    pub u0: f32,
    pub v0: f32,
    pub u1: f32,
    pub v1: f32,
}

/// 地图实际用到的砖烘成的一张 RGBA 图集。
///
/// 分两块：**上块是主菱形**（每格 64×32），**下块是 extra**（崖壁 / 坡面，每格 64×80）。
/// 两块都用 `tile_id << 8 | level` 作键。
pub struct TileAtlas {
    pub w: u32,
    pub h: u32,
    pub rgba: Vec<u8>,
    /// `tile_id << 8 | level` -> 主菱形槽
    pub slots: HashMap<u64, TileSlot>,
    /// `tile_id << 8 | level` -> extra 槽
    pub extra_slots: HashMap<u64, TileSlot>,
    /// `tile_id << 8 | level` -> 坡砖逆推槽（斜面的正视纹理）
    pub slope_slots: HashMap<u64, TileSlot>,
    /// `tile_id << 8 | level` -> 这张 extra 图从第几行开始是**崖壁**（上面全是平台色）。
    ///
    /// 逐行实测得出：60x74 的那批是 19，30x74 的那批 10~16，30x44 的那批 8~11。
    /// 补半格时，顶面用这条线以上的像素，竖直面用这条线以下的像素。
    pub extra_split: HashMap<u64, i32>,
    /// **通用崖壁纹理**：全图所有格子的竖直侧面共用这一张。
    ///
    /// `(RGBA, 宽, 高)` —— **独立的一张纹理，不进图集**。
    ///
    /// 崖壁必须单独出来：它的 UV 要按屏幕像素坐标走、并且 `Repeat` 平铺，横纹才能像砖图
    /// 里画的那样保持水平。塞在图集槽里只能取 0..1 一次，格边那 30° 斜度会把横纹剪成斜线。
    ///
    /// `None` 表示这张图上没有带 extra 的悬崖砖，侧面退回顶面纹理。
    pub wall: Option<(Vec<u8>, u32, u32)>,
    /// 那张通用崖壁纹理覆盖多少**层**高（= 崖壁段像素行数 ÷ 15，RA2 里一层正好 15px）。
    ///
    /// 竖直面按这个高度分段铺，否则一根 12 层的柱子会把 4 层的纹理纵向拉长三倍 ——
    /// 岩石纹理拉成一片细长竖条，看着像木栅栏。
    pub wall_layers: f32,
    /// **悬崖顶贴图**：选中那张悬崖砖 extra 的槽（整幅 60×74）。
    ///
    /// 只给「补出来的那半个菱形」用。补半格是按几何判出来的（对角双高），落到的格子
    /// 可能是 `Cliff`，也可能是 `RAMP` —— 坡砖自己的 extra 是坡面，里面没有悬崖顶。
    /// 所以这里备一张统一的：extra 最上面那段上尖三角就是原版画好的平台沿，
    /// 不管哪个半菱形补起来，都贴它，颜色和周围台面天然一致。
    pub wall_top: Option<TileSlot>,
}

/// 同一块砖在不同高度层用的是**不同的调色板**，颜色并不一样，所以 level 也要算进键里。
fn atlas_key(tile_id: u64, level: i32) -> u64 {
    (tile_id << 8) | (level.clamp(0, 18) as u64)
}

/// 坡砖四个角在**砖内像素坐标**里的位置（相对砖矩形左上角，顺序 top / right / bottom / left）。
///
/// 平地时就是菱形的四个尖点 `(30,0) (60,15) (30,30) (0,15)`；每个角再按
/// `ramp::corner_heights` 往上抬 `15px × 层数` —— RA2 里一层正好是半个砖高，
/// 所以「抬一层 = 屏幕上移 15 像素」。抬升只往上走，结果永远落在 60×30 画布内。
#[inline]
fn slope_corners(c: &TerrainCell) -> [(f32, f32); 4] {
    let h = ramp::corner_heights(c.ramp_type);
    let (w, hgt) = (c.w as f32, c.h as f32);
    let (hw, hh) = (w * 0.5, hgt * 0.5);
    [
        (hw, 0.0 - 15.0 * h[0] as f32),
        (w, hh - 15.0 * h[1] as f32),
        (hw, hgt - 15.0 * h[2] as f32),
        (0.0, hh - 15.0 * h[3] as f32),
    ]
}

/// 把坡砖的**屏幕投影**反解成斜面本身的纹理。
///
/// 【为什么必须做】`Slope01.tem` 的主图只有 14 行、每行 30 像素、每行左移 2 像素 ——
/// 那不是「画在菱形里的假立体」，而是斜面四角被抬高 15px 之后在屏幕上的**真投影**
/// （平地的 900 像素菱形被压成 30×15 的平行四边形，面积正好一半）。既然几何要真的
/// 倾斜，同一张投影图再斜一次就是二次投影、必然变形，所以得先把投影除掉，
/// 还原出斜面自己的 (u,v) 纹理。
///
/// 做法：面是平面、相机是正交投影，正交投影对平面是**仿射**的，所以屏幕坐标随 (u,v)
/// 线性变化 —— 对每个目标纹素算一次屏幕坐标、取该点所在的那个源像素即可。
/// 一次重采样、没有插值，像素画的硬边不会糊。
///
/// 源图 = 主图 ∪ extra（extra 后盖主图，与 wwread.cpp 的 cell 绘制次序一致）。
/// 返回 `(RGBA, 宽, 高)`：宽高取投影的实际跨度，让一个屏幕像素正好对应一个纹素。
fn unproject_slope(c: &TerrainCell, pal: &[u8]) -> (Vec<u8>, u32, u32, usize) {
    let s = slope_corners(c);
    // top→right 是 +u（沿 +rx），top→left 是 +v（沿 +ry）
    let e1 = (s[1].0 - s[0].0, s[1].1 - s[0].1);
    let e2 = (s[3].0 - s[0].0, s[3].1 - s[0].1);
    let uw = (e1.0 * e1.0 + e1.1 * e1.1).sqrt().round().max(1.0) as u32;
    let vh = (e2.0 * e2.0 + e2.1 * e2.1).sqrt().round().max(1.0) as u32;

    // 源像素：extra 优先（索引 0 视为没画，退回主图），再主菱形，越界当作空。
    let sample = |x: i32, y: i32| -> u8 {
        if c.has_extra
            && c.ex_w > 0
            && c.ex_h > 0
            && x >= c.ex_x
            && x < c.ex_x + c.ex_w
            && y >= c.ex_y
            && y < c.ex_y + c.ex_h
        {
            let i = ((y - c.ex_y) * c.ex_w + (x - c.ex_x)) as usize;
            let v = c.extra.get(i).copied().unwrap_or(0);
            if v != 0 {
                return v;
            }
        }
        if x >= 0 && x < c.w && y >= 0 && y < c.h {
            return c.pixels.get((y * c.w + x) as usize).copied().unwrap_or(0);
        }
        0
    };

    let mut out = vec![0u8; (uw as usize) * (vh as usize) * 4];
    let mut miss = 0usize;
    for j in 0..vh {
        let v = (j as f32 + 0.5) / vh as f32;
        for i in 0..uw {
            let u = (i as f32 + 0.5) / uw as f32;
            let sx = s[0].0 + u * e1.0 + v * e2.0;
            let sy = s[0].1 + u * e1.1 + v * e2.1;
            let mut idx = sample(sx.floor() as i32, sy.floor() as i32);
            if idx == 0 {
                miss += 1;
                // 采样点落在砖的有效像素外（砖的实际绘制边界比理论平行四边形小 1~2 像素，
                // `Slope01.tem` 实测左移 2、最高到 13 行）。向外扩几圈取最近实色，否则逆推出
                // 的纹理会在边缘留透明带，贴上去就是背景色的细缝。
                let (cx, cy) = (sx.floor() as i32, sy.floor() as i32);
                'grow: for r in 1..=5i32 {
                    for d in -r..=r {
                        for (ox, oy) in [(d, -r), (d, r), (-r, d), (r, d)] {
                            let v = sample(cx + ox, cy + oy);
                            if v != 0 {
                                idx = v;
                                break 'grow;
                            }
                        }
                    }
                }
            }
            if idx == 0 {
                continue;
            }
            let k = idx as usize * 3;
            if k + 2 >= pal.len() {
                continue;
            }
            let o = ((j as usize * uw as usize) + i as usize) * 4;
            out[o] = pal[k];
            out[o + 1] = pal[k + 1];
            out[o + 2] = pal[k + 2];
            out[o + 3] = 255;
        }
    }
    (out, uw, vh, miss)
}

/// 把平地砖的**屏幕投影菱形**反解成顶视图的方形纹理。
///
/// 【为什么必须反解】砖图是 2:1 等距投影之后再画的（60×30 的菱形），它只在那一个视角下
/// 才是对的。要按「给 mesh 贴图」的正路走，纹理必须是**顶视图**，投影交给显卡 ——
/// 视角怎么变都对，纹理也能脱离砖单独复用。
///
/// 面是平面、相机是正交投影，所以屏幕坐标随顶视图的 (u,v) **线性**变化：对每个目标纹素
/// 算一次屏幕坐标、取那个源像素即可。一次重采样、没有插值，像素画的硬边不会糊。
///
/// 方向约定：u 沿 +rx（菱形上尖点 → 右尖点），v 沿 +ry（上尖点 → 左尖点）。
/// 于是菱形的上/右/下/左四个尖点正好落在纹理的 (0,0) (1,0) (1,1) (0,1) 四角上 ——
/// `build_mesh` 的 UV 就是照这个直接取槽的四角，不需要任何偏移。
fn unproject_flat(c: &TerrainCell, pal: &[u8]) -> Vec<u8> {
    // 菱形四尖点在砖内像素坐标：上 (w/2,0)、右 (w,h/2)、下 (w/2,h)、左 (0,h/2)。
    // 【不要拿 cell_x/cell_y 来挪】试过，整张图当场变大花脸 —— 这两个偏移不是「菱形在
    // 砖内的位置」，而是那张 tile 图错位覆盖到哪个子格，直接套进来每块砖都错位。
    let (w, h) = (c.w as f32, c.h as f32);
    let top = (w * 0.5, 0.0);
    let e1 = (w * 0.5, h * 0.5);
    let e2 = (-w * 0.5, h * 0.5);
    let n = SLOT_W as usize;
    let mut out = vec![0u8; n * n * 4];
    let sample = |x: i32, y: i32| -> u8 {
        if x < 0 || y < 0 || x >= c.w || y >= c.h {
            return 0;
        }
        c.pixels.get((y * c.w + x) as usize).copied().unwrap_or(0)
    };
    let mut miss = 0usize;
    for j in 0..n {
        let v = (j as f32 + 0.5) / n as f32;
        for i in 0..n {
            let u = (i as f32 + 0.5) / n as f32;
            let sx = top.0 + u * e1.0 + v * e2.0;
            let sy = top.1 + u * e1.1 + v * e2.1;
            let (cx, cy) = (sx.floor() as i32, sy.floor() as i32);
            let mut idx = sample(cx, cy);
            if idx == 0 {
                // 砖的有效像素边界是逐行 4px 的台阶，比数学菱形小一圈；采样点落进那条缝隙
                // 就向外找最近一个实色。不补的话贴上去每格边上留一圈透明，被 Mask 丢掉
                // 就是背景色细缝。
                'grow: for r in 1..=6i32 {
                    for d in -r..=r {
                        for (ox, oy) in [(d, -r), (d, r), (-r, d), (r, d)] {
                            let v2 = sample(cx + ox, cy + oy);
                            if v2 != 0 {
                                idx = v2;
                                break 'grow;
                            }
                        }
                    }
                }
            }
            if idx == 0 {
                miss += 1;
                continue;
            }
            let k = idx as usize * 3;
            if k + 2 >= pal.len() {
                continue;
            }
            let o = (j * n + i) * 4;
            out[o] = pal[k];
            out[o + 1] = pal[k + 1];
            out[o + 2] = pal[k + 2];
            out[o + 3] = 255;
        }
    }
    // 【槽里一个透明像素都不能留】顶面的 UV 铺满整个槽，槽里的透明像素会被 Mask 丢掉，
    // 露出底下的清屏色 —— 屏幕上就是一块块洋红。而砖图的有效像素边界是逐行 4px 的台阶，
    // 跟数学菱形差一点点；`Cliff` 这类多 cell 砖的错位 cell 更是缺一大片。
    // 所以：先用**最近的有效像素**一轮轮往外扩散，还填不到的用整张砖图的平均色压平。
    {
        for _ in 0..(n as i32) {
            let mut filled = 0usize;
            for j in 0..n {
                for i in 0..n {
                    let o = (j * n + i) * 4;
                    if out[o + 3] != 0 {
                        continue;
                    }
                    'nb: for (dx, dy) in [(1i32, 0i32), (-1, 0), (0, 1), (0, -1)] {
                        let x = i as i32 + dx;
                        let y = j as i32 + dy;
                        if x < 0 || y < 0 || x >= n as i32 || y >= n as i32 {
                            continue;
                        }
                        let o2 = (y as usize * n + x as usize) * 4;
                        if out[o2 + 3] != 0 {
                            out[o] = out[o2];
                            out[o + 1] = out[o2 + 1];
                            out[o + 2] = out[o2 + 2];
                            out[o + 3] = 255;
                            filled += 1;
                            break 'nb;
                        }
                    }
                }
            }
            if filled == 0 {
                break;
            }
        }
        if out.chunks_exact(4).any(|p| p[3] == 0) {
            let (mut r, mut g, mut b, mut cnt) = (0u32, 0u32, 0u32, 0u32);
            for y in 0..c.h {
                for x in 0..c.w {
                    let idx = sample(x, y);
                    if idx == 0 {
                        continue;
                    }
                    let k = idx as usize * 3;
                    if k + 2 >= pal.len() {
                        continue;
                    }
                    r += pal[k] as u32;
                    g += pal[k + 1] as u32;
                    b += pal[k + 2] as u32;
                    cnt += 1;
                }
            }
            let avg = if cnt > 0 {
                [(r / cnt) as u8, (g / cnt) as u8, (b / cnt) as u8]
            } else {
                [128, 128, 128]
            };
            for p in out.chunks_exact_mut(4) {
                if p[3] == 0 {
                    p[0] = avg[0];
                    p[1] = avg[1];
                    p[2] = avg[2];
                    p[3] = 255;
                }
            }
        }
    }
    let _ = miss;
    out
}

/// 把 extra 里那段**崖壁**从斜压状态逆推成一格宽的正视矩形纹理。
///
/// 【为什么要逆推】砖文件里的 extra 是**已经按 2:1 等距投影画好**的：崖壁的上下沿都跟着
/// 菱形的 30° 斜边走（所以一块悬崖砖的图看起来是个六边形柱）。直接把它当矩形贴到 3D 的
/// 竖直面上，等于把斜边当直角用 —— 贴出来要么一圈黑三角、要么整面糊成竖条。
/// 必须先在纹素层面把投影除掉，还原成「一格宽 × 崖壁高」的正视图。
///
/// 源区域 = extra 的 `[0,ex_w) × [split, ex_h)`。它在屏幕上是一个平行四边形：
/// 横向走满 `ex_w` 的同时纵向斜移 `ex_w/2`（等距 2:1 的斜率正好 0.5）。
/// 于是每个目标纹素按 (u,v) 反算屏幕坐标取源像素即可，一次重采样、不插值。
///
/// `v = 0` 是崖壁底、`v = 1` 是崖壁顶。
/// 从悬崖砖的 extra 里**直接裁出那块岩壁矩形**。
///
/// 【这张图本来就是正视图，不需要任何投影反算】把 `Cliff09` 用 `isourb.pal` 导出就能
/// 看明白，它的 extra 是一个立在格子上的悬崖柱，从上到下三段：
///
/// | 位置 | 内容 | 实测行 |
/// |---|---|---|
/// | 最上 | 上尖的**浅色三角** —— 平台边沿的地面 | y 0–19 |
/// | 中间 | **黄褐色横向层理岩壁**，左右竖边、上下横边，规整矩形 | y 19–59 |
/// | 最下 | 下尖的**浅色三角** —— 低地的地面 | y 59–74 |
///
/// 中间那块没有任何斜切。之前一直按「等距平行四边形」去逆推它，等于把正视图当斜视图
/// 反算，出来的是满墙竖纹 —— 那是反复贴错的真正原因。
///
/// 所以这里只做一件事：**逐行找出最长的崖壁段**（岩壁偏黄褐，`R−B` 明显大于 0；
/// 上下两块浅灰地砖和石板没有这个特征），把这段整宽裁出来。
/// 宽度原样保留（60 列 = 一格菱形的屏幕宽 = 两条格边），不做缩放。
fn crop_wall(extra: &[u8], ex_w: i32, ex_h: i32, split: i32, pal: &[u8]) -> (Vec<u8>, u32, u32) {
    let warm = |x: i32, y: i32| -> bool {
        let idx = extra.get((y * ex_w + x) as usize).copied().unwrap_or(0);
        if idx == 0 {
            return false;
        }
        let k = idx as usize * 3;
        if k + 2 >= pal.len() {
            return false;
        }
        pal[k] as i32 - pal[k + 2] as i32 > 20
    };
    // 【要求整行几乎全是岩石】阈值只写「过半」时，紧贴平台沿和低地那几行会被算进来 ——
    // 那几行一半是浅色地砖，贴到墙上就是一整格深色糊块（矮墙分段时正好整段吃到它）。
    // 现在要 85% 以上都是暖色，把过渡行全部排除，裁出来的才是干净的一段岩壁。
    let is_band = |y: i32| -> bool {
        let (mut hit, mut n) = (0i32, 0i32);
        for x in 0..ex_w {
            let idx = extra.get((y * ex_w + x) as usize).copied().unwrap_or(0);
            if idx == 0 {
                continue;
            }
            n += 1;
            if warm(x, y) {
                hit += 1;
            }
        }
        n > 0 && hit * 100 > n * 85
    };
    let _ = &is_band;
    // 【严格按「顶 / 壁 / 底」三段切，只取壁】这张 extra 从上是：
    //   顶 = 上尖浅色三角（平台沿），壁 = 黄褐横纹岩石，底 = 下尖浅色三角（低地）。
    // `split` 是壁开始的行（逐行 `R−B > 20` 判出来的，实测 19），底段固定 15px
    // （一层）。两头都不要 —— 混进来一点，Repeat 平铺时就会在墙上印出一格一个的菱形斑。
    let (mut by0, mut by1) = (split.clamp(0, ex_h.saturating_sub(1)), ex_h - 15);
    if by1 <= by0 + 4 {
        // 认不出来就退回逐行扫描，别因为一条边界把整面墙丢掉。
        by0 = 0;
        by1 = ex_h;
    }
    // 一行崖壁都没认出来（比如整块图偏灰），退回整幅 —— 宁可带上边沿也不要空白。
    if by1 <= by0 {
        by0 = 0;
        by1 = ex_h;
    }
    let w = ex_w.max(1) as u32;
    let h = (by1 - by0).max(1) as u32;
    let mut out = vec![0u8; (w * h * 4) as usize];
    // 【第 0 行存岩壁的**底**，不是顶】全工程的侧面 UV 都按「v0 = 崖壁底、v1 = 崖壁顶」
    // 写死了（柱子侧面、补半格的弦墙都是）。这里按屏幕顺序（第 0 行是顶）存，就会让
    // 上沿和下沿都落在纹理同一条边上 —— 一整面墙被采成一条像素拉平，出来就是纯黑块。
    for j in 0..h {
        let sy = by1 - 1 - j as i32;
        for i in 0..w {
            let idx = extra
                .get((sy * ex_w + i as i32) as usize)
                .copied()
                .unwrap_or(0);
            if idx == 0 {
                continue;
            }
            let k = idx as usize * 3;
            if k + 2 >= pal.len() {
                continue;
            }
            let o = ((j * w + i) * 4) as usize;
            out[o] = pal[k];
            out[o + 1] = pal[k + 1];
            out[o + 2] = pal[k + 2];
            out[o + 3] = 255;
        }
    }
    (out, w, h)
}

/// 把图集里一个槽内 `alpha = 0` 的像素用最近的有效像素颜色填满（纹理外扩 / bleed）。
///
/// 【当前状态：**没有接上调用**，保留着备查】2026-10-17 试过一次接到主菱形和 extra 上，
/// 用户当场反馈「悬崖贴图错」，已回滚。要再启用，先只接主菱形、别碰 extra（extra 的
/// 透明角正是墙的 UV 边界，填色会污染贴图），并且**单独一次启动**验证。
///
/// 【它想解决什么】主图菱形的有效像素边界是**逐行 4px 的台阶**（`tmp.rs::decode_diamond`），
/// 而几何菱形的边是数学直线 —— 两者之间有不到 1 像素的错位带。采样点落进去就拿到
/// `alpha = 0`，被 `AlphaMode::Mask(0.5)` 丢掉，于是那个像素既没被这块砖写、也没被
/// 邻砖写，露出清屏色（洋红）—— 就是「每格一圈粉色边框」。
///
/// 只填槽内 `sw × sh`（= 砖的实际尺寸），槽的边距不动 —— UV 不会采到那里。
#[allow(dead_code)]
fn bleed_slot(rgba: &mut [u8], aw: u32, sx: u32, sy: u32, sw: u32, sh: u32) {
    #[inline]
    fn solid(rgba: &[u8], aw: u32, sx: u32, sy: u32, x: u32, y: u32) -> bool {
        let o = (((sy + y) * aw + sx + x) * 4) as usize;
        rgba[o + 3] != 0
    }
    const N: [(i32, i32); 8] = [
        (0, -1), (1, -1), (1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1),
    ];
    // 【填满整个槽矩形，不留一个透明像素】槽里那四块透明的角在屏幕上仍落在**本格
    // 几何菱形之内**：几何菱形就是整个槽矩形（UV 0..1 铺满），所以透明像素被 Mask
    // 丢掉就等于在本格中间挖了个洞 —— 表现是尖角处零星洋红。相邻格在角点处精确
    // 相接（同一对整数网格坐标），填满的角像素不会盖到别人身上。
    let max_rounds = (sw.max(sh) + 4) as usize;
    for _round in 0..max_rounds {
        let mut pending: Vec<(u32, u32, u8, u8, u8)> = Vec::new();
        for y in 0..sh {
            for x in 0..sw {
                if solid(rgba, aw, sx, sy, x, y) {
                    continue;
                }
                let mut found: Option<(u8, u8, u8)> = None;
                for (dx, dy) in N {
                    let nx = x as i32 + dx;
                    let ny = y as i32 + dy;
                    if nx < 0 || ny < 0 || nx >= sw as i32 || ny >= sh as i32 {
                        continue;
                    }
                    let (nx, ny) = (nx as u32, ny as u32);
                    if solid(rgba, aw, sx, sy, nx, ny) {
                        let o = (((sy + ny) * aw + sx + nx) * 4) as usize;
                        found = Some((rgba[o], rgba[o + 1], rgba[o + 2]));
                        break;
                    }
                }
                if let Some((r, g, b)) = found {
                    pending.push((x, y, r, g, b));
                }
            }
        }
        if pending.is_empty() {
            break;
        }
        for (x, y, r, g, b) in pending {
            let o = (((sy + y) * aw + sx + x) * 4) as usize;
            rgba[o] = r;
            rgba[o + 1] = g;
            rgba[o + 2] = b;
            rgba[o + 3] = 255;
        }
    }
}

/// 这格要按「斜面」处理吗 —— 也就是贴图走「逆推成正视图」那条路。
///
/// 两个来源任一成立即可：TMP 自己的 `RampType` 非平，或者砖集名以 `RAMP` 开头。
/// 只用前者不够：地图里大量 `RAMP25` 这类砖的 `RampType` 是 0，主图却是斜面的投影
/// （平行四边形、只有十几行像素）。按菱形四角去采它，采到的全是砖外 —— 屏幕上就是洋红。
fn is_slope_cell(c: &TerrainCell) -> bool {
    ramp::is_sloped(c.ramp_type) || c.ramp_set
}

/// 把地图用到的砖烘成图集。`iso_level` 是按 level 的调色板（19 级，每级 RGB 768 字节）。
pub fn build_atlas(tb: &TerrainBuild, iso_level: &[Vec<u8>], plain_pal: &[u8]) -> TileAtlas {
    // 第一遍：去重，定顺序
    let mut keys: Vec<u64> = Vec::new();
    let mut idx_of: HashMap<u64, usize> = HashMap::new();
    for c in &tb.cells {
        if c.pixels.is_empty() || c.w <= 0 || c.h <= 0 {
            continue;
        }
        let k = atlas_key(c.tile_id, c.level);
        if idx_of.contains_key(&k) {
            continue;
        }
        idx_of.insert(k, keys.len());
        keys.push(k);
    }
    // extra 去重（跟主菱形同一套键）
    let mut xkeys: Vec<u64> = Vec::new();
    let mut xidx: HashMap<u64, usize> = HashMap::new();
    for c in &tb.cells {
        if !c.has_extra || c.ex_w <= 0 || c.ex_h <= 0 || c.extra.is_empty() {
            continue;
        }
        let k = atlas_key(c.tile_id, c.level);
        if xidx.contains_key(&k) {
            continue;
        }
        xidx.insert(k, xkeys.len());
        xkeys.push(k);
    }
    // 第三组：坡砖。这一组的槽里存的是**逆推后的正视图**，不是投影图 —— 见 `unproject_slope`。
    // 平地砖走主菱形槽。
    let mut skeys: Vec<u64> = Vec::new();
    let mut sidx: HashMap<u64, usize> = HashMap::new();
    for c in &tb.cells {
        if !is_slope_cell(c) || c.pixels.is_empty() || c.w <= 0 || c.h <= 0 {
            continue;
        }
        let k = atlas_key(c.tile_id, c.level);
        if sidx.contains_key(&k) {
            continue;
        }
        sidx.insert(k, skeys.len());
        skeys.push(k);
    }
    let n = keys.len();
    let nx = xkeys.len();
    let ns = skeys.len();
    let rows = ((n as u32) + ATLAS_COLS - 1) / ATLAS_COLS;
    let xrows = ((nx as u32) + ATLAS_COLS - 1) / ATLAS_COLS;
    let srows = ((ns as u32) + ATLAS_COLS - 1) / ATLAS_COLS;
    let main_h = rows.max(1) * SLOT_H;
    // 主菱形区在上、extra 区居中、坡面逆推区在最后
    let xh = main_h + xrows * XSLOT_H;
    // 【列宽按最大那个槽算】三块的槽宽不一样（主菱形 32、extra 64、坡面 64）。
    // 以前一直用 SLOT_W 算图集宽度，正好那时三者都是 64 看不出来；主菱形改成 32 之后，
    // extra 槽按 64 排版就会冲出图集右边界 —— `u0` 算出 1.56 这种大于 1 的值，
    // 采到的是别的槽或图集外。
    let col_w = SLOT_W.max(XSLOT_W).max(SSLOT_W);
    // 最后一行留给**逆推后的通用崖壁**（一格宽 × 崖壁高，进过反投影的正视图）。
    let (aw, ah) = (ATLAS_COLS * col_w, xh + srows * SSLOT_H + WALL_SLOT_H);
    // 【定位色】图集里的空白一律填品红：没被任何砖写到的像素 = RGBA(255,0,255,0)。
    // alpha 恒为 0（Mask 材质会丢弃，正常画面看不见它），但它一旦被画出来就是刺眼的洋红，
    // 于是"某块砖没烘进图集""UV 采到了隔壁空槽"这类问题一眼就能认出来 —— 黑色认不出来。
    let mut rgba = vec![0u8; (aw as usize) * (ah as usize) * 4];
    for px in rgba.chunks_exact_mut(4) {
        px[0] = 255;
        px[2] = 255;
    }
    let mut slots: HashMap<u64, TileSlot> = HashMap::with_capacity(n);
    let mut extra_slots: HashMap<u64, TileSlot> = HashMap::with_capacity(nx);
    // extra 里「平台段 / 崖壁段」的分界行，见 TileAtlas::extra_split 的说明。
    let mut extra_split: HashMap<u64, i32> = HashMap::with_capacity(nx);

    // 第二遍：把每块砖的**调色板索引**按它的 level 调色板转成 RGBA 写进槽
    let mut written = vec![false; n];
    for c in &tb.cells {
        if c.pixels.is_empty() || c.w <= 0 || c.h <= 0 {
            continue;
        }
        let k = atlas_key(c.tile_id, c.level);
        let Some(&si) = idx_of.get(&k) else { continue };
        if written[si] {
            continue;
        }
        written[si] = true;
        let sx = (si as u32 % ATLAS_COLS) * SLOT_W;
        let sy = (si as u32 / ATLAS_COLS) * SLOT_H;
        let pal = &iso_level[c.level.clamp(0, 18) as usize];
        let tex = unproject_flat(c, pal);
        for y in 0..SLOT_H {
            for x in 0..SLOT_W {
                let o2 = ((y * SLOT_W + x) * 4) as usize;
                if tex[o2 + 3] == 0 {
                    continue; // 透明
                }
                let px = sx + x;
                let py = sy + y;
                if px >= aw || py >= ah {
                    continue;
                }
                let o = ((py * aw + px) * 4) as usize;
                rgba[o] = tex[o2];
                rgba[o + 1] = tex[o2 + 1];
                rgba[o + 2] = tex[o2 + 2];
                rgba[o + 3] = 255;
            }
        }
        // 【槽内填色】主菱形的有效像素边界是**逐行 4px 的台阶**，而几何菱形的边是数学
        // 直线，采样点会落进那条不到 1 像素的缝里拿到 alpha=0，被 Mask 丢掉 —— 屏幕上就是
        // 零星的洋红小点。把槽内透明像素用最近的有效像素填满，缝就没了。
        // 只填主菱形槽，不碰 extra（extra 的透明角正是墙的 UV 边界，填了会污染崖壁）。
        bleed_slot(&mut rgba, aw, sx, sy, SLOT_W.min(aw - sx), SLOT_H.min(ah - sy));
        slots.insert(
            k,
            TileSlot {
                u0: sx as f32 / aw as f32,
                v0: sy as f32 / ah as f32,
                u1: (sx + SLOT_W) as f32 / aw as f32,
                v1: (sy + SLOT_H) as f32 / ah as f32,
            },
        );
    }

    // extra（崖壁 / 坡面）：跟主菱形一样按该 level 的调色板转成 RGBA，写进下半块
    let mut xwritten = vec![false; nx];
    // extra 的原始像素尺寸，只给下面「挑一张做通用崖壁」用。
    let mut extra_dims: HashMap<u64, (i32, i32)> = HashMap::with_capacity(nx);
    for c in &tb.cells {
        if !c.has_extra || c.ex_w <= 0 || c.ex_h <= 0 || c.extra.is_empty() {
            continue;
        }
        let k = atlas_key(c.tile_id, c.level);
        let Some(&si) = xidx.get(&k) else { continue };
        if xwritten[si] {
            continue;
        }
        xwritten[si] = true;
        extra_dims.insert(k, (c.ex_w, c.ex_h));
        let sx = (si as u32 % ATLAS_COLS) * XSLOT_W;
        let sy = main_h + (si as u32 / ATLAS_COLS) * XSLOT_H;
        let pal = &iso_level[c.level.clamp(0, 18) as usize];
        for y in 0..c.ex_h {
            for x in 0..c.ex_w {
                let ch = c.extra[(y * c.ex_w + x) as usize];
                if ch == 0 {
                    continue;
                }
                let kk = (ch as usize) * 3;
                if kk + 2 >= pal.len() {
                    continue;
                }
                let px = sx + x as u32;
                let py = sy + y as u32;
                if px >= aw || py >= ah {
                    continue;
                }
                let o = ((py * aw + px) * 4) as usize;
                rgba[o] = pal[kk];
                rgba[o + 1] = pal[kk + 1];
                rgba[o + 2] = pal[kk + 2];
                rgba[o + 3] = 255;
            }
        }
        extra_slots.insert(
            k,
            TileSlot {
                u0: sx as f32 / aw as f32,
                v0: sy as f32 / ah as f32,
                u1: (sx + c.ex_w as u32) as f32 / aw as f32,
                v1: (sy + c.ex_h as u32) as f32 / ah as f32,
            },
        );
        // 分段：逐行算 R−B 平均，> 20 判为崖壁（棕色岩纹），取最长连续崖壁段的起点。
        // 这条线以上是平台色（补半格的顶面取它），以下是崖壁（贴到竖直面上）。
        // 只在 Cliff 家族上算：斜坡砖的 extra 是坡面，没有这个分界。
        if c.cliff {
            let mut kinds: Vec<bool> = Vec::with_capacity(c.ex_h as usize);
            for y in 0..c.ex_h {
                let (mut sum, mut n) = (0i32, 0i32);
                for x in 0..c.ex_w {
                    let ch = c.extra[(y * c.ex_w + x) as usize];
                    if ch == 0 {
                        continue;
                    }
                    let kk = (ch as usize) * 3;
                    if kk + 2 >= pal.len() {
                        continue;
                    }
                    sum += pal[kk] as i32 - pal[kk + 2] as i32;
                    n += 1;
                }
                kinds.push(n > 0 && sum / n > 20);
            }
            let (mut best_at, mut best_len, mut run) = (0, 0, 0);
            for (y, &is_wall) in kinds.iter().enumerate() {
                if is_wall {
                    run += 1;
                    if run > best_len {
                        best_len = run;
                        best_at = y + 1 - run;
                    }
                } else {
                    run = 0;
                }
            }
            if best_len > 0 {
                extra_split.insert(k, best_at as i32);
            }
        }
    }

    // 坡砖逆推（第三块）：把屏幕投影还原成斜面自己的 (u,v) 纹理
    let mut swritten = vec![false; ns];
    let mut miss_total = 0usize;
    let mut tex_total = 1usize;
    let mut slope_slots: HashMap<u64, TileSlot> = HashMap::with_capacity(ns);
    for c in &tb.cells {
        if !is_slope_cell(c) || c.pixels.is_empty() || c.w <= 0 || c.h <= 0 {
            continue;
        }
        let k = atlas_key(c.tile_id, c.level);
        let Some(&si) = sidx.get(&k) else { continue };
        if swritten[si] {
            continue;
        }
        swritten[si] = true;
        let px = (si as u32 % ATLAS_COLS) * SSLOT_W;
        let py = xh + (si as u32 / ATLAS_COLS) * SSLOT_H;
        let pal = &iso_level[c.level.clamp(0, 18) as usize];
        let (src, uw0, vh0, miss) = unproject_slope(c, pal);
        miss_total += miss;
        tex_total += (uw0 * vh0) as usize;
        let uw = uw0.min(SSLOT_W);
        let vh = vh0.min(SSLOT_H);
        for y in 0..vh {
            for x in 0..uw {
                let o = ((y * uw0 + x) * 4) as usize;
                if src[o + 3] == 0 {
                    continue;
                }
                let q = (((py + y) * aw + px + x) * 4) as usize;
                rgba[q] = src[o];
                rgba[q + 1] = src[o + 1];
                rgba[q + 2] = src[o + 2];
                rgba[q + 3] = 255;
            }
        }
        // 【把槽里的透明像素填满】逆推时有 14%~15% 的纹素落在砖的有效像素之外（砖的真实绘制
        // 边界比理论平行四边形小一圈），那些位置保持透明 = 图集里的品红定位色。坡面的 UV
        // 铺满整个槽，采样到就是一块块洋红。用最近的有效像素往外扩散填掉。
        bleed_slot(&mut rgba, aw, px, py, uw, vh);
        slope_slots.insert(
            k,
            TileSlot {
                u0: px as f32 / aw as f32,
                v0: py as f32 / ah as f32,
                u1: (px + uw) as f32 / aw as f32,
                v1: (py + vh) as f32 / ah as f32,
            },
        );
    }


    println!(
        "slope unproject {ns} tiles, {tex_total} texels, {:.2}% missed",
        100.0 * miss_total as f32 / tex_total as f32
    );

    // RA2_DUMP_SLOPE=<out.png>：把第三块（坡砖逆推区）单独导出来看。
    if let Ok(p) = std::env::var("RA2_DUMP_SLOPE") {
        let ch = srows * SSLOT_H;
        if ch > 0 {
            let row_bytes = (aw * 4) as usize;
            let mut sub = vec![0u8; row_bytes * ch as usize];
            for y in 0..ch as usize {
                let s = ((xh as usize) + y) * row_bytes;
                sub[y * row_bytes..(y + 1) * row_bytes].copy_from_slice(&rgba[s..s + row_bytes]);
            }
            let _ = crate::png_out::write_rgba(&p, aw, ch, &sub);
            println!("slope atlas -> {p} ({aw}x{ch})");
        }
    }

    // 挑一张做**通用崖壁**：全图柱子的竖直侧面都用它。
    // 打分按「先看宽度，再看崖壁段的行数」—— 实测 60 宽的那批（`Cliff09/10`，60×74、
    // 崖壁段 55 行）最完整，30 宽的那批只有半个屏宽的岩壁，拼起来会看出接缝。
    let mut wall_key: Option<u64> = None;
    let mut wall_layers = 1.0f32;
    let mut best_score = i64::MIN;
    let mut wall_info = (0i32, 0i32, 0i32);
    for (&k, &(ew, eh)) in &extra_dims {
        if !extra_slots.contains_key(&k) {
            continue;
        }
        let sp = extra_split.get(&k).copied().unwrap_or(0);
        let rows = eh - sp;
        if rows <= 0 || eh <= 0 {
            continue;
        }
        // 打分：先看「有没有真的算出崖壁段」，再优先 60 宽那批（画的是整片岩壁），
        // 最后看崖壁段多长。选中的那张下面会**逆推成正视图**再进图集。
        let has_band = extra_split.contains_key(&k);
        let wide = if ew >= 60 { 1i64 } else { 0 };
        let score = (has_band as i64) * 100_000_000 + wide * 10_000_000 + rows as i64 * 100 + ew as i64;
        if score > best_score {
            best_score = score;
            wall_key = Some(k);
            wall_layers = eh as f32 / 15.0;
            wall_info = (ew, eh, sp);
        }
    }

    // 把选中的那张**逆推**成一格宽的正视矩形，写进图集最后一行。
    let mut wall: Option<(Vec<u8>, u32, u32)> = None;
    let mut wall_top: Option<TileSlot> = None;
    if let Some(k) = wall_key {
        // 悬崖顶就取这张砖自己的 extra 槽（`extra_slots` 里存的是整幅，含最上面那段
        // 上尖三角）。补出来的半菱形按固定 UV 取那一段。
        wall_top = extra_slots.get(&k).copied();
        let src = tb
            .cells
            .iter()
            .find(|c| c.has_extra && !c.extra.is_empty() && atlas_key(c.tile_id, c.level) == k);
        if let Some(cell) = src {
            // 【调色板用最亮那一级】着色链是「TMP 像素索引 → 调色板 → RGB」，改颜色的只有
            // 中间这步：`iso_level` 是 `recalculate_palette` 按地图光照逐级压暗的版本，
            // 崖壁砖铺在低地上，拿到的就是最暗一级 —— 画好的黄褐岩石再被压掉一截就成了黑。
            // 用 `.pal` 原文件试过（`plain_pal`）：索引 0 会映射成纯黑，整面墙直接变黑块，
            // 更糟。所以取光照表里最高的一级，等于把这层压暗抵消掉。
            let pal = &iso_level[18];
            // 只裁中间那块岩壁矩形，上下两片浅色地面三角不要 —— 它们贴到墙上就是两条灰带。
            let (tex, tw, th) = crop_wall(&cell.extra, cell.ex_w, cell.ex_h, wall_info.2, pal);
            // 纹理覆盖几层高：裁出来的行数 ÷ 15px（RA2 一层正好 15px）。
            wall_layers = th as f32 / 15.0;
            let solid = tex.chunks_exact(4).filter(|p| p[3] != 0).count();
            println!(
                "wall crop: extra {}x{} -> rock band {}x{} ({:.2} layers), solid {}/{}",
                cell.ex_w,
                cell.ex_h,
                tw,
                th,
                wall_layers,
                solid,
                tex.len() / 4
            );
            // 【崖壁单独成一张纹理，不进图集】它必须能 `Repeat` 平铺：UV 按屏幕像素坐标
            // 走，横纹才能和砖图里画的一样是水平的。图集槽只能取 0..1 一次，横纹会被
            // 格边的 30° 斜度剪成斜条纹 —— 那就是"崖壁 UV 错乱"的根。
            // extra 里有些像素是 alpha=0（岩壁和上下三角之间的过渡）。纹理要 `Repeat`
            // 平铺，这些空洞会被反复采到 —— 底色留白就是一条条白斜纹，留黑就是黑纹。
            // 所以先算岩石的平均色垫底，再把实色像素盖上去。
            let (mut ar, mut ag, mut ab, mut an) = (0u32, 0u32, 0u32, 0u32);
            for i in 0..(tw * th) as usize {
                let o2 = i * 4;
                if tex[o2 + 3] == 0 {
                    continue;
                }
                ar += tex[o2] as u32;
                ag += tex[o2 + 1] as u32;
                ab += tex[o2 + 2] as u32;
                an += 1;
            }
            let fill = if an > 0 {
                [(ar / an) as u8, (ag / an) as u8, (ab / an) as u8]
            } else {
                [128, 120, 100]
            };
            let mut own = vec![0u8; (tw * th * 4) as usize];
            for i in 0..(tw * th) as usize {
                let o2 = i * 4;
                own[o2] = fill[0];
                own[o2 + 1] = fill[1];
                own[o2 + 2] = fill[2];
                own[o2 + 3] = 255;
            }
            for i in 0..(tw * th) as usize {
                let o2 = i * 4;
                if tex[o2 + 3] == 0 {
                    continue;
                }
                own[o2] = tex[o2];
                own[o2 + 1] = tex[o2 + 1];
                own[o2 + 2] = tex[o2 + 2];
            }
            wall = Some((own, tw, th));
            println!(
                "wall texture: extra {}x{} band row {} ({:.2} layers) -> standalone {}x{}",
                wall_info.0, wall_info.1, wall_info.2, wall_layers, tw, th
            );
        }
    }

    // 放在最后：崖壁槽是在这个函数末尾才写进 `rgba` 的，早于它导出会看到一片空白。
    if let Ok(p) = std::env::var("RA2_DUMP_TILEATLAS") {
        match crate::png_out::write_rgba(&p, aw, ah, &rgba) {
            Ok(()) => println!("tile atlas -> {p} ({aw}x{ah}) main_h={main_h}"),
            Err(e) => println!("tile atlas {p}: {e}"),
        }
    }

    TileAtlas {
        w: aw,
        h: ah,
        rgba,
        slots,
        extra_slots,
        slope_slots,
        extra_split,
        wall,
        wall_layers,
        wall_top,
    }
}

impl TileAtlas {
    pub fn slot_of(&self, tile_id: u64, level: i32) -> Option<TileSlot> {
        self.slots.get(&atlas_key(tile_id, level)).copied()
    }

    /// 取这一个 cell 的 extra 槽（崖壁 / 坡面）。没有 extra 时返回 None。
    pub fn extra_slot_of(&self, tile_id: u64, level: i32) -> Option<TileSlot> {
        self.extra_slots.get(&atlas_key(tile_id, level)).copied()
    }

    /// 这张 extra 图从第几行开始是崖壁（上面是平台色）。0 表示没算出分界。
    pub fn extra_split_of(&self, tile_id: u64, level: i32) -> i32 {
        self.extra_split
            .get(&atlas_key(tile_id, level))
            .copied()
            .unwrap_or(0)
    }

    /// 取坡砖**逆推后**的斜面正视槽。有它说明这格的贴图已经还原成 (u,v) 空间，
    /// 几何要按真实倾斜面走；没有就说明是平地，走主菱形槽。
    pub fn slope_slot_of(&self, tile_id: u64, level: i32) -> Option<TileSlot> {
        self.slope_slots.get(&atlas_key(tile_id, level)).copied()
    }
}

// ============================================================================ 地形网格

/// 砖上一点 `(px, py)`（相对砖矩形左上角的像素）对应的世界点。
///
/// `E_SX` / `E_SY` 是等距投影的**逆** —— 沿它们把像素矩形展开到世界水平面，
/// 相机投影回来正好落在那个像素位置上。于是每块砖就是一个屏幕对齐的四边形。
#[inline]
fn at_px(base: [f32; 3], px: f32, py: f32) -> [f32; 3] {
    // 【量化到 1/8192】相邻两格的共用角点是分开算出来的：一格走 `py`、另一格走 `py + 60`
    // 再折回来，两条式子的结果差一两个 ulp。光栅化时那点差会漏出一个像素，露出清屏色 ——
    // 就是满图零星的小洋红点。量化比像素小三个数量级，不会让几何变形，但能保证同一角点
    // 在两格里算出**完全相同的浮点数**。
    const Q: f32 = 8192.0;
    let q = |v: f32| (v * Q).round() / Q;
    [
        q(base[0] + px * E_SX[0] + py * E_SY[0]),
        q(base[1]),
        q(base[2] + px * E_SX[2] + py * E_SY[2]),
    ]
}

/// 一格的世界基点：菱形**上尖点**落在地面高度上的位置，再抬 `level` 层。
///
/// `build_mesh` 和 debug 高亮替身都从这里取 —— **不许各自再写一遍**，
/// 否则改一处、另一处就跟地形错位。
#[inline]
fn cell_base(rx: i32, ry: i32, level: i32) -> [f32; 3] {
    [
        rx as f32 - 30.0 * E_SX[0],
        level as f32 * LEVEL_HEIGHT,
        ry as f32 - 30.0 * E_SX[2],
    ]
}

/// 菱形四角相对砖矩形左上角（即 [`at_px`] 的原点）的像素偏移，顺序 **上 / 右 / 下 / 左**。
#[inline]
fn corner_offsets(w: f32, h: f32) -> [(f32, f32); 4] {
    let (hw, hh) = (w * 0.5, h * 0.5);
    [(hw, 0.0), (w, hh), (hw, h), (0.0, hh)]
}

/// 相机光轴单位向量 —— **必须与 `game.rs` 的 `CAM_DIR`（归一化后）一致**。
/// 正交投影下沿它平移一个点，屏幕坐标分毫不动，只有深度变，所以专拿来做深度偏置。
const AXIS: [f32; 3] = [0.612_372, 0.5, 0.612_372];

/// 把一个世界点沿相机光轴推进 `b`（世界单位）。这个方向上拉开的差距**只体现在深度上**，
/// 屏幕上的位置与像素归属完全不变。
#[inline]
fn along_axis(p: [f32; 3], b: f32) -> [f32; 3] {
    [
        p[0] + b * AXIS[0],
        p[1] + b * AXIS[1],
        p[2] + b * AXIS[2],
    ]
}

/// 【地形可调项】全部来自 `rulesmd.ini` 的 `[Terrain]` 段，缺项用这里的默认值。
///
/// 这些东西以前是写死在 `build_mesh` 里的常数，改一个数就要重编译一次 —— 现在只改 ini。
#[derive(Clone, Copy)]
pub struct TerrainCfg {
    /// 顶面 UV 向内缩多少**纹素**。0 = 正好铺满槽（边缘会采到槽外的空白）。
    pub face_inset: f32,
    /// 墙脚比低侧顶面再往下埋多少**层**。共边处光栅化漏 1px 时用它盖住。
    pub wall_foot_drop: f32,
    /// 坡砖四角高度的倍率。1.0 = 按 `ramp::RAMP_CORNERS` 原值（陡坡跨两层）。
    pub ramp_scale: f32,
    /// 逻辑模式顶面的交替明暗（`(rx^ry)` 为奇的格乘这个数）。
    pub top_alt_shade: f32,
    /// 高差小于这个层数的格边不出墙。防止浮点尾差生成 1px 高的墙（纹理会被压成竖条）。
    pub min_wall_height: f32,
    /// 坡砖贴图来源：true = 用 `unproject_slope` 逆推的坡槽；false = 和普通砖一样用主菱形槽。
    pub slope_projected: bool,
}

impl Default for TerrainCfg {
    fn default() -> Self {
        TerrainCfg {
            face_inset: 0.5,
            wall_foot_drop: 0.0,
            ramp_scale: 1.0,
            top_alt_shade: 0.96,
            min_wall_height: 0.05,
            slope_projected: false,
        }
    }
}

pub struct TerrainMesh {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub uvs: Vec<[f32; 2]>,
    /// 逐顶点颜色（线性空间）。只在逻辑 mesh 模式下挂到 Mesh 上；贴图模式忽略。
    pub colors: Vec<[f32; 4]>,
    pub indices: Vec<u32>,
    /// **崖壁面的独立缓冲**。竖直面要贴一张自己的纹理、UV 按屏幕像素坐标走并 `Repeat`
    /// 平铺，所以不能和地形顶面共用同一张图集、同一个材质。
    pub wall_positions: Vec<[f32; 3]>,
    pub wall_normals: Vec<[f32; 3]>,
    pub wall_uvs: Vec<[f32; 2]>,
    pub wall_indices: Vec<u32>,
}

impl TerrainMesh {
    fn push_quad(&mut self, p: [[f32; 3]; 4], uv: [[f32; 2]; 4], color: [[f32; 4]; 4]) {
        // 法线按真实几何算：平地是 (0,1,0)，斜面朝坡的上方偏。
        // 以前一律写常量 (0,1,0)，那样斜面在实时光照里会假平（N·L 不随坡度变）。
        let e1 = [p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]];
        let e2 = [p[3][0] - p[0][0], p[3][1] - p[0][1], p[3][2] - p[0][2]];
        let mut n = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ];
        // 地形从上方看，朝上的面一律取 +Y 那一侧（+X×+Z 叉出 −Y，要翻一次）。
        if n[1] < 0.0 {
            n = [-n[0], -n[1], -n[2]];
        }
        // 【竖直面一律朝相机】柱子侧面和补半格的弦墙都是竖的，`n_y ≈ 0`，上面那条 +Y 规则
        // 管不到 —— 方向完全由调用方给的顶点顺序决定。弦两端的顺序在四种对角组合里并不
        // 统一，于是同一段墙上会有一格绕序反了、法线朝内，`N·L` 变负，整面墙黑成一块
        // （实测黑块像素 RGB(22,15,4)，而纹理本身平均亮度 80）。
        // 相机固定在 (+x, +z) 这一侧，竖直面的法线统一朝这一侧，就不会再有孤立黑格。
        if n[1].abs() < 0.5 && n[0] + n[2] < 0.0 {
            n = [-n[0], -n[1], -n[2]];
        }
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        let mut n = if len > 1.0e-6 {
            [n[0] / len, n[1] / len, n[2] / len]
        } else {
            [0.0, 1.0, 0.0]
        };
        // 【竖直面把法线抬成 45° 朝上】平行光从斜上方来，纯水平法线的 `N·L` 只剩一点点，
        // 背光那一侧直接被压成一块黑（实测 RGB(22,15,4)，纹理本身亮度 80）。原版是手绘 2D，
        // 所有可见崖壁亮度一致；把竖直面法线往 +Y 抬到 45°，亮度就接近台面，朝向差异还在。
        if n[1].abs() < 0.5 {
            let h = (n[0] * n[0] + n[2] * n[2]).sqrt();
            if h > 1.0e-6 {
                let k = std::f32::consts::FRAC_1_SQRT_2;
                n = [n[0] / h * k, k, n[2] / h * k];
            }
        }
        let base = self.positions.len() as u32;
        for i in 0..4 {
            self.positions.push(p[i]);
            self.normals.push(n);
            self.uvs.push(uv[i]);
            self.colors.push(color[i]);
        }
        // 两个三角形（双面渲染，绕序不敏感）
        self.indices
            .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    /// 单个三角形，法线一律取 +Y 那一侧（只用于补出来的水平半格）。
    fn push_tri(&mut self, p: [[f32; 3]; 3], uv: [[f32; 2]; 3], color: [[f32; 4]; 3]) {
        let e1 = [p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]];
        let e2 = [p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2]];
        let mut n = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ];
        if n[1] < 0.0 {
            n = [-n[0], -n[1], -n[2]];
        }
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        let n = if len > 1.0e-6 {
            [n[0] / len, n[1] / len, n[2] / len]
        } else {
            [0.0, 1.0, 0.0]
        };
        let base = self.positions.len() as u32;
        for i in 0..3 {
            self.positions.push(p[i]);
            self.normals.push(n);
            self.uvs.push(uv[i]);
            self.colors.push(color[i]);
        }
        self.indices.extend_from_slice(&[base, base + 1, base + 2]);
    }
    /// 竖直崖壁面：推到自己的缓冲里，法线规则和 `push_quad` 一致（竖直面朝相机、抬 45°）。
    fn push_wall_quad(&mut self, p: [[f32; 3]; 4], uv: [[f32; 2]; 4]) {
        let e1 = [p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]];
        let e2 = [p[3][0] - p[0][0], p[3][1] - p[0][1], p[3][2] - p[0][2]];
        let mut n = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ];
        if n[1] < 0.0 {
            n = [-n[0], -n[1], -n[2]];
        }
        if n[0] + n[2] < 0.0 {
            n = [-n[0], -n[1], -n[2]];
        }
        let len = (n[0] * n[0] + n[2] * n[2]).sqrt();
        let n = if len > 1.0e-6 {
            [n[0] / len, 0.0, n[2] / len]
        } else {
            [0.0, 1.0, 0.0]
        };
        let base = self.wall_positions.len() as u32;
        for i in 0..4 {
            self.wall_positions.push(p[i]);
            self.wall_normals.push(n);
            self.wall_uvs.push(uv[i]);
        }
        self.wall_indices
            .extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }
}

/// 建整张地形：每格一个**屏幕对齐的四边形**（主图），外加它自带的 extra 矩形。
///
/// 位置（照 `TmpRenderer.Draw` / `wwread.cpp`）：
/// * 主图矩形左上角 = 格角屏幕位置 − (30, 0)px（菱形上顶点落在砖内 (30, 0) 处），
///   高度按 `level` 上移（RA2 的 `oy = (dy − z)·15`）。
/// * extra 矩形左上角 = 主图左上角 + `(ex_x, ex_y)`（`TmpRenderer.cs:136-137`）。
/// * 两者都只是**平图**，尺寸取 `w×h` / `ex_w×ex_h`，不倾斜、不贴几何。
///
/// 遮挡交给 GPU 深度缓冲：四边形都在 `y = level·LEVEL_HEIGHT` 的水平面内，
/// 格与格的菱形区域互不重叠（相邻菱形只相切），空白角部是透明像素会被 alpha 裁掉。
///
/// 【同一层内的深度平局 —— 高台四角跳动的根因】上一条对**主图之间**成立，但 extra
/// 图不止一格那么大：崖壁图最高 74px ≈ 2.5 格，它会盖到**同 level 邻居**的菱形上，
/// 于是同一水平面内出现两块**不透明**四边形在屏幕上重叠（实测 hillbtwn：8502 个
/// 四边形里 36131 对同 level 重叠）。同平面 + 同屏幕位置 ⇒ 两个片元的世界点**完全
/// 相同**，深度理论值相等，只差光栅化的舍入。而深度缓冲的量化步长约
/// `(far−near)/2^23`（这里 far−near≈2·cam_span），这个舍入量恰好卡在量化边界：
/// 相机亚像素一动，谁赢谁输就翻一次 —— 表现出来就是崖壁与地面交界处成片跳动，
/// 高台拐角（两面崖壁 + 地面三方交汇）最显眼。
///
/// RA2 用画家算法（按砖序，后画的覆盖先画的）定这个顺序，这里用同一招：给每块砖一个
/// 随**屏幕纵深**单调递增的微小偏移，靠下的（更近相机）压过靠上的。屏幕纵深 ∝
/// `rx + ry`（屏幕 y = 15·(X+Z)），所以偏置正比于 `rx + ry`。
/// **偏移方向是相机光轴（见 `along_axis`），不是世界 Y** —— 世界 Y 会平移屏幕位置，
/// 相邻砖被错开亚像素，边界上就漏出背景细线。
/// 取模 8 把累计量锁死在 `7·DEPTH_EPS` 以内：需要判序的两块砖屏幕纵深最多差 3 格
/// （崖壁图 2.5 格），差 8 格的砖在屏幕上相隔 120px 以上，不可能重叠。
///
/// 【为什么是 1e-4 而不是 1e-3】这个偏置只在**等距**下完全隐形（沿光轴推进不改屏幕位置）。
/// 切到俯视后光轴变成 `+Y`，同一个位移就变成真实的屏幕位移：`7e-3` 世界单位在
/// 60px/格 下约 0.3px，最近邻采样直接露成一条洋红背景细线。压到 `7e-4`（约 0.03px）
/// 之后看不出来，而深度差的量级仍比深度缓冲的量化步长（约 1e-5）大一个数量级，
/// 平局照样打破。
const DEPTH_EPS: f32 = 1.0e-4;

/// 裙边向下延伸的层数（已废弃：崖壁现在完全由 TMP 的 extra 数据决定，见 `extra_wall`）。
/// 值保留着只是因为地图边缘还没有专门的收边做法，将来要用时从这里取。
#[allow(dead_code)]
const SKIRT_LEVELS: f32 = 3.0;

/// 逻辑 mesh 模式下顶面的高度色阶：低处深绿 → 高处浅粉，13 级覆盖 level 0..12。
fn level_color(z: i32) -> [u8; 3] {
    const LUT: [[u8; 3]; 13] = [
        [58, 92, 56],
        [66, 100, 58],
        [76, 108, 60],
        [88, 116, 62],
        [100, 124, 66],
        [112, 130, 72],
        [124, 134, 80],
        [136, 136, 90],
        [148, 138, 100],
        [158, 140, 110],
        [168, 142, 120],
        [178, 146, 130],
        [188, 152, 140],
    ];
    LUT[(z.max(0) as usize).min(LUT.len() - 1)]
}

/// 压暗：**在 sRGB 字节上乘**（不是在已经过 gamma 的线性值上乘）。
/// 0.62 / 0.55 / 0.52 / 0.44 这几个数是这套配色定死的，别当可调参数。
fn shade(c: [u8; 3], k: f32) -> [u8; 3] {
    [
        (c[0] as f32 * k).min(255.0) as u8,
        (c[1] as f32 * k).min(255.0) as u8,
        (c[2] as f32 * k).min(255.0) as u8,
    ]
}

/// sRGB 字节 → Bevy 顶点色要的线性 RGBA。直接除以 255 会偏亮，必须做一次伽马反变换。
fn lin_color(c: [u8; 3]) -> [f32; 4] {
    let f = |v: u8| (v as f32 / 255.0).powf(2.2);
    [f(c[0]), f(c[1]), f(c[2]), 1.0]
}

/// 一格顶面**中心**的世界坐标。基点取自 [`cell_base`]，与 `build_mesh` 是同一个函数，
/// 不可能错位。
pub fn cell_center_at(rx: i32, ry: i32, level: i32, w: i32, h: i32) -> [f32; 3] {
    at_px(cell_base(rx, ry, level), w as f32 * 0.5, h as f32 * 0.5)
}

/// 一格顶面**相对格中心**的四个角（顺序同 `build_mesh` 的 `at`：上 / 右 / 下 / 左）。
/// y 只含 ramp 抬升量，不含 `level`。
///
/// 因为与 `level` 无关，debug 高亮可以按 `(w, h, ramp_type)` 预建一次 mesh，
/// 之后每帧只挪 `Transform`，不用重建顶点。
pub fn cell_local_corners(w: i32, h: i32, ramp_type: u8) -> [[f32; 3]; 4] {
    let (w, h) = (w as f32, h as f32);
    let (hw, hh) = (w * 0.5, h * 0.5);
    let rc = ramp::corner_heights(ramp_type);
    let at = corner_offsets(w, h);
    let mut out = [[0.0f32; 3]; 4];
    for i in 0..4 {
        let q = at_px([0.0, 0.0, 0.0], at[i].0 - hw, at[i].1 - hh);
        out[i] = [q[0], rc[i] as f32 * LEVEL_HEIGHT, q[2]];
    }
    out
}

/// 建整张地形的**唯一**几何来源。
///
/// 顶点、索引、UV、顶点色与"这次贴不贴图"完全无关：贴图模式与逻辑模式吃的是**同一份
/// `TerrainMesh`**，区别只在 `game.rs` 挂不挂 `ATTRIBUTE_COLOR`、用 `unlit` 材质还是
/// 图集材质。这里没有任何模式参数、没有第二套算法、没有需要两处同步的常数。
pub fn build_mesh(
    tb: &TerrainBuild,
    atlas: &TileAtlas,
    gw: i32,
    gh: i32,
    cfg: &TerrainCfg,
) -> TerrainMesh {
    let mut m = TerrainMesh {
        positions: Vec::new(),
        normals: Vec::new(),
        uvs: Vec::new(),
        colors: Vec::new(),
        indices: Vec::new(),
        wall_positions: Vec::new(),
        wall_normals: Vec::new(),
        wall_uvs: Vec::new(),
        wall_indices: Vec::new(),
    };
    if gw <= 0 || gh <= 0 {
        return m;
    }

    // 【一张高度场，一套规则】地形不再是「每格一个独立柱体」，而是四角高度已知的连续曲面：
    //
    //   格 (rx,ry) 的四个角点，世界坐标就是 (gx, h, gy) —— 整数网格加层高，
    //   角点网格坐标固定为
    //     角0 上尖 (rx,   ry  )   角1 右尖 (rx+1, ry  )
    //     角2 下尖 (rx+1, ry+1)   角3 左尖 (rx,   ry+1)
    //
    // 这个对应不是凑的，是从 `cell_base` + `corner_offsets` 反算出来的：`at_px` 把
    // 砖内像素 (30,0)/(60,15)/(30,30)/(0,15) 展开到世界水平面，四个结果正好落在上面
    // 那四个整数点上（x = gx、z = gy）。所以不需要逐像素反推、不需要量化、不需要
    // 深度偏置去糊缝 —— 相邻格的公共角点在两格里算出的是**同一个整数**。
    //
    // 顶面：每格自己的四个角高度（`level` + 坡砖四角修正），四边形，四个角就在网格点上。
    // 墙面：**每条格边只出一次**，上下沿分别取两侧顶面在该边端点的高度（高的是上沿、
    //       低的是下沿）。两侧同高时这条边上没有面 —— 同层平地的相邻顶面只在边线上
    //       接触，本来就不需要墙，也就没有重合面去抢深度。
    //
    // 为什么这样就闭合：墙的下沿端点 = 低侧顶面在这个角点的顶点，XZ 完全同点、Y 相同；
    // 上沿端点 = 高侧顶面在同一个角点的顶点。于是墙与两侧顶面精确咬合，凸角、凹角、
    // 坡砖的半层高度都由同一套规则产生，不存在「谁忘了补一块」。
    const OFF: [(i32, i32); 4] = [(0, 0), (1, 0), (1, 1), (0, 1)];

    struct Frame<'a> {
        c: &'a TerrainCell,
        slot: TileSlot,
        /// 四角高度，单位=层（可含坡砖的半层）
        h: [f32; 4],
    }

    let mut frames: Vec<Frame> = Vec::with_capacity(tb.cells.len());
    let mut heights: HashMap<(i32, i32), [f32; 4]> = HashMap::with_capacity(tb.cells.len());
    let mut size_hist: HashMap<(i32, i32), usize> = HashMap::new();

    for c in &tb.cells {
        if c.rx < 0 || c.ry < 0 || c.rx >= gw || c.ry >= gh {
            continue;
        }
        // 【坡砖走第三块槽】坡砖的主图是斜面的投影（平行四边形），不是菱形，
        // 拿它按菱形四角采样会大片落到砖外。正视纹理已由 `unproject_slope` 烘进第三块。
        let slot = if cfg.slope_projected && is_slope_cell(c) {
            atlas.slope_slot_of(c.tile_id, c.level)
        } else {
            None
        }
        .or_else(|| atlas.slot_of(c.tile_id, c.level));
        let Some(slot) = slot else {
            // 拿不到槽的格没有几何，那块地方是空的（露出清屏色）。诊断照旧打出来。
            println!(
                "mesh: no atlas slot rx={} ry={} level={} tile={} pix={} w={} h={}",
                c.rx,
                c.ry,
                c.level,
                c.tile_id,
                c.pixels.len(),
                c.w,
                c.h
            );
            continue;
        };
        *size_hist.entry((c.w, c.h)).or_insert(0) += 1;
        // 【四角高度 = level + ramp 表】`ramp::corner_heights` 查的是 theater 的 TileSet，
        // 不是按邻居推的；坡砖朝下的角可以是负的，那正是坡面该有的形状。
        // 必须按它抬角，否则坡度全变成一格一级的台阶。
        // ramp 13..16 是跨两层的陡坡：下尖抬 2 层后与上尖的屏幕投影重合（顶面退化成
        // 一条线），那是真实几何 —— 这一面正好朝向相机，可见部分是它四周的斜面。
        let rc = ramp::corner_heights(c.ramp_type);
        let mut h = [0.0f32; 4];
        for i in 0..4 {
            h[i] = c.level as f32 + rc[i] as f32 * cfg.ramp_scale;
        }
        heights.insert((c.rx, c.ry), h);
        frames.push(Frame { c, slot, h });
    }

    // ---------------------------------------------------------------- 顶面
    for f in &frames {
        let c = f.c;
        let mut top = [[0.0f32; 3]; 4];
        for k in 0..4 {
            top[k] = [
                (c.rx + OFF[k].0) as f32,
                f.h[k] * LEVEL_HEIGHT,
                (c.ry + OFF[k].1) as f32,
            ];
        }
        // 顶面 UV = 槽的四角。纹理是正视图（`unproject_flat` 反解），菱形四个尖点
        // 正好对应纹理的 (0,0)/(1,0)/(1,1)/(0,1)。四角向内缩半个纹素：槽是紧挨着排的，
        // 往外扩会踩到隔壁槽的空白。
        let iu = (f.slot.u1 - f.slot.u0) * (cfg.face_inset / SLOT_W as f32);
        let iv = (f.slot.v1 - f.slot.v0) * (cfg.face_inset / SLOT_H as f32);
        let uvs = [
            [f.slot.u0 + iu, f.slot.v0 + iv],
            [f.slot.u1 - iu, f.slot.v0 + iv],
            [f.slot.u1 - iu, f.slot.v1 - iv],
            [f.slot.u0 + iu, f.slot.v1 - iv],
        ];
        // 逻辑模式取色：顶面按 level，`(rx^ry)` 为奇的格乘 0.96 制造相邻格的轻微明暗差。
        let cc = level_color(c.level);
        let topc = if (c.rx ^ c.ry) & 1 == 0 {
            cc
        } else {
            shade(cc, cfg.top_alt_shade)
        };
        m.push_quad(top, uvs, [lin_color(topc); 4]);
    }

    // ---------------------------------------------------------------- 墙面
    let (wall_w, wall_h) = match &atlas.wall {
        Some((_, w, h)) => (*w as f32, *h as f32),
        None => (60.0, 40.0),
    };
    // 一个角点在屏幕上的横坐标（像素）与竖坐标（像素，世界 y=0 时）。
    // 等距投影：屏幕 x = 30·(gx − gy)，屏幕 y = 15·(gx + gy)，每抬一层再上移 15px。
    let sx_of = |g: (i32, i32)| 30.0 * (g.0 - g.1) as f32;
    let sy_of = |g: (i32, i32), h: f32| 15.0 * (g.0 + g.1) as f32 - 15.0 * h;

    let mut wall_quads = 0usize;
    for f in &frames {
        let c = f.c;
        let me = (c.rx, c.ry);
        for i in 0..4 {
            let j = (i + 1) % 4;
            let (dx, dy) = match i {
                0 => (0, -1),
                1 => (1, 0),
                2 => (0, 1),
                _ => (-1, 0),
            };
            let nb_key = (c.rx + dx, c.ry + dy);
            // 【一条边只出一次】两侧都会走到这条边，交给网格坐标较小的一侧生成；
            // 邻居不在地图上时，本侧必须自己生成（否则地图外沿没有墙）。
            let nb = heights.get(&nb_key);
            if nb.is_some() && me > nb_key {
                continue;
            }
            // 邻居那两个角在本格里的编号（公共边两端的角点是一一对应的）
            let (na, nbk) = match i {
                0 => (3, 2),
                1 => (0, 3),
                2 => (1, 0),
                _ => (2, 1),
            };
            // 邻居缺失 = 地图边界，按地面 0 层算，墙一直落到地面
            let nh = nb.copied().unwrap_or([0.0; 4]);
            // 端点高度：高的一侧做上沿、低的一侧做下沿
            let hi_a = f.h[i].max(nh[na]);
            let lo_a = f.h[i].min(nh[na]);
            let hi_b = f.h[j].max(nh[nbk]);
            let lo_b = f.h[j].min(nh[nbk]);
            // 两端都没有高差 = 这条边上两侧顶面齐平，没有面积，不出墙
            if hi_a - lo_a < cfg.min_wall_height && hi_b - lo_b < cfg.min_wall_height {
                continue;
            }
            let ga = (c.rx + OFF[i].0, c.ry + OFF[i].1);
            let gb = (c.rx + OFF[j].0, c.ry + OFF[j].1);
            m.push_wall_quad(
                [
                    [ga.0 as f32, hi_a * LEVEL_HEIGHT, ga.1 as f32],
                    [gb.0 as f32, hi_b * LEVEL_HEIGHT, gb.1 as f32],
                    [gb.0 as f32, (lo_b - cfg.wall_foot_drop) * LEVEL_HEIGHT, gb.1 as f32],
                    [ga.0 as f32, (lo_a - cfg.wall_foot_drop) * LEVEL_HEIGHT, ga.1 as f32],
                ],
                [
                    [sx_of(ga) / wall_w, sy_of(ga, hi_a) / wall_h],
                    [sx_of(gb) / wall_w, sy_of(gb, hi_b) / wall_h],
                    [sx_of(gb) / wall_w, sy_of(gb, lo_b) / wall_h],
                    [sx_of(ga) / wall_w, sy_of(ga, lo_a) / wall_h],
                ],
            );
            wall_quads += 1;
        }
    }

    let mut sizes: Vec<((i32, i32), usize)> = size_hist.into_iter().collect();
    sizes.sort_by(|a, b| b.1.cmp(&a.1));
    println!("terrain tile sizes {:?}", &sizes[..sizes.len().min(6)]);
    println!("terrain mesh quads={} wall_quads={}", m.indices.len() / 6, wall_quads);
    m
}

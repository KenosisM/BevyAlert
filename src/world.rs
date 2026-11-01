//! 世界层 —— 对等移植。
//!
//! 来源：`I:\bevy\RA2REASM\src\engine\world.cpp` / `world.h`（GroundField、坡道几何，
//! 以及后续的 Entity / World 模拟半段），以及 `src\tools\wwread.cpp` 里
//! `cmdMap` / `cmdPlay` / `cmdSimNav` 逐字重复的那段 GroundField 装配。
//!
//! 本文件目前只含**地面高度场**（#32）；重力碗模拟（Entity / update）随后并入。

use crate::costgrid::TerrainBuild;
use crate::mapfile::MapFile;
use crate::nav::Cell;

// ============================================================================ 坡道几何

/// 20 种坡道类型，照地图编辑器（FA2sp）存法：格子四个角各抬高几级，名字也沿用编辑器。
#[derive(Clone, Copy, Debug, Default)]
pub struct RampCorners {
    pub top: i8,
    pub right: i8,
    pub bottom: i8,
    pub left: i8,
}

pub const K_RAMP_TYPES: i32 = 21;

pub const K_RAMP_CORNERS: [RampCorners; 21] = [
    RampCorners { top: 0, right: 0, bottom: 0, left: 0 },
    RampCorners { top: 0, right: 1, bottom: 1, left: 0 },
    RampCorners { top: 0, right: 0, bottom: 1, left: 1 },
    RampCorners { top: 1, right: 0, bottom: 0, left: 1 },
    RampCorners { top: 1, right: 1, bottom: 0, left: 0 },
    RampCorners { top: 0, right: 0, bottom: 1, left: 0 },
    RampCorners { top: 0, right: 0, bottom: 0, left: 1 },
    RampCorners { top: 1, right: 0, bottom: 0, left: 0 },
    RampCorners { top: 0, right: 1, bottom: 0, left: 0 },
    RampCorners { top: 0, right: 1, bottom: 1, left: 1 },
    RampCorners { top: 1, right: 0, bottom: 1, left: 1 },
    RampCorners { top: 1, right: 1, bottom: 0, left: 1 },
    RampCorners { top: 1, right: 1, bottom: 1, left: 0 },
    RampCorners { top: 0, right: 1, bottom: 2, left: 1 },
    RampCorners { top: 1, right: 0, bottom: 1, left: 2 },
    RampCorners { top: 2, right: 1, bottom: 0, left: 1 },
    RampCorners { top: 1, right: 2, bottom: 1, left: 0 },
    RampCorners { top: 0, right: 1, bottom: 0, left: 1 },
    RampCorners { top: 1, right: 0, bottom: 1, left: 0 },
    RampCorners { top: 0, right: 1, bottom: 0, left: 1 },
    RampCorners { top: 1, right: 0, bottom: 1, left: 0 },
];

/// 坡道倾角所绕的水平轴（yaw 度）：轴横过坡面，于是车顺着梯度倾、而不是侧滚。
/// `voxelObjectMatrix` 把 Ry(pitch) 加在朝向旋转**之后**，所以那时模型的 Y 就是横向轴，
/// 这张表就是相对它表达的。平地与鞍型（0, 17-20）没有倾角。
pub const K_RAMP_TILT_YAW: [f32; 21] = [
    0.0, 0.0, -90.0, -180.0, -270.0, // 0 平，1-4 单向
    -45.0, -135.0, -225.0, -315.0, // 5-8 对角
    -45.0, -135.0, -225.0, -315.0, // 9-12 对角
    -45.0, -135.0, -225.0, -315.0, // 13-16 对角
    0.0, 0.0, 0.0, 0.0, // 17-20 鞍型 / 平顶（无倾角）
];

/// 坡道格四角的高度，索引 `[u][v]`：u 沿 +rx，v 沿 +ry。
/// (0,0) 是该格的 Bottom 角，(1,0) 是 Left，(1,1) 是 Top，(0,1) 是 Right。
///
/// 编辑器那套角名落到本引擎的角上**是反过来的**：type 1 坡道的 Top 与 Left 角抬高，而它的砖
/// 朝 rx+1 那一格爬 —— 所以编辑器抬的那个角就是 (0,0)，它的 +u 方向正是 +rx 的爬升方向。
/// （上游拿四块采样坡道周围的格高核对过。）
pub fn ramp_corner_heights(ramp_type: i32) -> [[i8; 2]; 2] {
    let mut h = [[0i8; 2]; 2];
    if ramp_type <= 0 || ramp_type >= K_RAMP_TYPES {
        return h;
    }
    let c = K_RAMP_CORNERS[ramp_type as usize];
    h[0][0] = c.top;
    h[1][0] = c.right;
    h[1][1] = c.bottom;
    h[0][1] = c.left;
    h
}

/// 坡道格那块砖画出来的平面：沿 +rx / +ry 每格爬几级，以及格心处比本格自己的高度高几级。
/// 每一种坡道都描述一个平面（鞍型的梯度为零），所以这是精确的。
pub fn ramp_plane(ramp_type: i32) -> (f32, f32, f32) {
    let h = ramp_corner_heights(ramp_type);
    let grad_x = 0.5 * ((h[1][0] + h[1][1]) as f32 - (h[0][0] + h[0][1]) as f32);
    let grad_y = 0.5 * ((h[0][1] + h[1][1]) as f32 - (h[0][0] + h[1][0]) as f32);
    let centre = 0.25 * ((h[0][0] + h[1][0] + h[0][1] + h[1][1]) as f32);
    (grad_x, grad_y, centre)
}

/// 一层高的 lepton 数。照搬 `Rendering/RampHeight.cs:27`。
const LEVEL_LEPTONS: i32 = 104;

/// 格心处高出本格自身高度多少（单位：**半级**），下标是 RampType。
/// 照搬 `Rendering/RampHeight.cs:31-38` 的 `HalfLevelsAtCentre`。
const HALF_LEVELS_AT_CENTRE: [u8; K_RAMP_TYPES as usize] = [
    0, // 0  平
    1, 1, 1, 1, // 1-4  单向坡
    0, 0, 0, 0, // 5-8  对角对，在格心抵消
    2, 2, 2, 2, // 9-12 抬角对
    2, 2, 2, 2, // 13-16 双高变体
    1, 1, 1, 1, // 17-20 平顶半坡
];

/// 高度（lepton）折成屏幕垂直像素：`Tactical::Z_Lepton_To_Pixel`。
/// 照搬 `Rendering/RampHeight.cs:56-64`；阈值取 RA2/YR 的 7 层（TS 是 9 层）。
fn z_pixel(leptons: i32) -> i32 {
    // 砖宽 60（RA2 的 BlockWidth）
    let per_lepton =
        (std::f64::consts::PI / 3.0).sin() * 60.0 / (256.0 * std::f64::consts::SQRT_2);
    let fudge = if leptons >= 7 * LEVEL_LEPTONS { 1.0 } else { 0.0 };
    (leptons as f64 * per_lepton + fudge + 0.5) as i32
}

/// 站在坡道格上的模型要抬高多少像素（相对该格自己的高度）。
/// 照搬 `Rendering/RampHeight.cs:47-53` 的 `PixelLift`。
pub fn ramp_pixel_lift(ramp_type: i32, cell_height: i32) -> i32 {
    if ramp_type <= 0 || ramp_type >= K_RAMP_TYPES {
        return 0;
    }
    let half = HALF_LEVELS_AT_CENTRE[ramp_type as usize] as i32;
    if half == 0 {
        return 0;
    }
    let ground = LEVEL_LEPTONS * cell_height;
    z_pixel(ground + half * (LEVEL_LEPTONS / 2)) - z_pixel(ground)
}

/// 坡道格上的模型倾斜：`(pitch 度, 轴 yaw 度)`。地形烘焙与运行时车辆读的是同一份。
pub fn ramp_tilt(ramp_type: i32) -> (f32, f32) {
    if ramp_type <= 0 || ramp_type >= K_RAMP_TYPES {
        return (0.0, 0.0);
    }
    // 轴是每型固定的；pitch 仍按本格坡度算。
    let yaw = K_RAMP_TILT_YAW[ramp_type as usize];
    let (gx, gy, _c) = ramp_plane(ramp_type);
    let x = gx * 104.0;
    let y = gy * 104.0;
    let len = (x * x + y * y).sqrt();
    if len <= 0.0 {
        // 🔴 上游此处 return 时 `axisYawDeg` 仍保留上面赋好的值（只有 pitchDeg 为 0）。
        return (0.0, yaw);
    }
    const K_RAD2DEG: f32 = 57.29577951308232;
    // 取负：模型机头是 +X，倾斜是绕横向轴的 Ry(pitch)，而 matRotY 把 (1,0,0) 送到
    // (cos, 0, -sin) —— 所以正的 pitch 会把机头压下去。爬坡必须抬起来，故要翻符号。
    (-len.atan2(256.0) * K_RAD2DEG, yaw)
}

// ============================================================================ 地面高度场

/// 玩法格网上的地面高度场。资产层从 terrain cells 建它（地面高度 + 桥面）；世界采样它，
/// 于是单位站在哪一格就画在哪一格的高度上，走路时实时变高。
#[derive(Clone, Debug)]
pub struct GroundField {
    /// 格网范围：rx ∈ 0..w、ry ∈ 0..h
    pub w: i32,
    pub h: i32,
    /// 格子栅格 (rx,ry) 跨 [1, mapW+mapH-1]，比地图自己的宽高大；保留 mapWidth 是因为
    /// 等距投影要用真实地图宽。
    pub map_width: i32,
    /// RA2 等距格网
    pub pitch_x: i32,
    pub pitch_y: i32,
    /// 每格地面高度，索引 ry*w+rx
    pub level: Vec<i16>,
    /// 桥面非零
    pub bridge: Vec<u8>,
    /// RampType（0 = 平）
    pub ramp: Vec<u8>,
    /// 把物体抬到本格地面之上的像素数
    pub lift_px: Vec<i16>,
    pub tilt_pitch: Vec<i16>,
    pub tilt_yaw: Vec<i16>,
}

impl Default for GroundField {
    fn default() -> Self {
        GroundField {
            w: 0,
            h: 0,
            map_width: 0,
            pitch_x: 30,
            pitch_y: 15,
            level: Vec::new(),
            bridge: Vec::new(),
            ramp: Vec::new(),
            lift_px: Vec::new(),
            tilt_pitch: Vec::new(),
            tilt_yaw: Vec::new(),
        }
    }
}

impl GroundField {
    pub fn valid(&self) -> bool {
        self.w > 0 && self.h > 0 && self.level.len() == (self.w as usize) * (self.h as usize)
    }
    pub fn in_(&self, rx: i32, ry: i32) -> bool {
        rx >= 0 && ry >= 0 && rx < self.w && ry < self.h
    }
    pub fn level_at(&self, rx: i32, ry: i32) -> i32 {
        if !self.in_(rx, ry) {
            return 0;
        }
        self.level[(ry as usize) * (self.w as usize) + (rx as usize)] as i32
    }
    pub fn bridge_at(&self, rx: i32, ry: i32) -> bool {
        if !self.in_(rx, ry) || self.bridge.is_empty() {
            return false;
        }
        self.bridge[(ry as usize) * (self.w as usize) + (rx as usize)] != 0
    }
    pub fn lift_at(&self, rx: i32, ry: i32) -> i32 {
        if !self.in_(rx, ry) || self.lift_px.is_empty() {
            return 0;
        }
        self.lift_px[(ry as usize) * (self.w as usize) + (rx as usize)] as i32
    }
    pub fn ramp_at(&self, rx: i32, ry: i32) -> i32 {
        if !self.in_(rx, ry) || self.ramp.is_empty() {
            return 0;
        }
        self.ramp[(ry as usize) * (self.w as usize) + (rx as usize)] as i32
    }
    pub fn tilt_at(&self, rx: i32, ry: i32) -> (i32, i32) {
        if !self.in_(rx, ry) {
            return (0, 0);
        }
        let i = (ry as usize) * (self.w as usize) + (rx as usize);
        let mut pitch = 0;
        let mut yaw = 0;
        if i < self.tilt_pitch.len() {
            pitch = self.tilt_pitch[i] as i32;
        }
        if i < self.tilt_yaw.len() {
            yaw = self.tilt_yaw[i] as i32;
        }
        (pitch, yaw)
    }

    // 【已删除 cell_at_world】世界点 → 格只允许有一份实现，那份已在 `game.rs::pick_cell`。
    // 旧实现把 `surface_height` 当格单位加到 (rx, ry) 上，高处目标两轴各滑约 level/2 格；
    // 而且它没有任何调用点（只有自测），留着就是第二套口径。取格子统一走拾取射线那条。

    /// 一格的地表高度，以 level-0 平面为基准的像素数：它的 level，加上桥面所在的那 4 级，
    /// 再加上坡道面在格心处多出来的像素。
    /// 上游 `World::anchorOf`（world.cpp:823-842）：格坐标（可为小数）→ **世界像素**，
    /// 即等距投影后、未缩放的屏幕位置。单位摆放、点选、框选全部以这个口径算。
    ///
    /// 注意那个 `- (mapWidth+1)`：地形 cell 与地图渲染器的单位层都把格 (rx,ry) 放在
    /// `dy = rx + ry - (mapWidth+1)` 这一行；漏掉它整层单位会整体下沉 (mapWidth+1) 行。
    /// 高度用**小数**格位置取，单位跨格时才不会在坡道的上坡半格悬空、下坡半格埋地。
    pub fn anchor_of(&self, cell_x: f32, cell_y: f32) -> (f32, f32) {
        if !self.valid() {
            return (cell_x, cell_y);
        }
        let mw = self.map_width as f32;
        let dx = cell_x - cell_y + (mw - 1.0);
        let dy = cell_x + cell_y - (mw + 1.0);
        let px = (dx + 1.0) * self.pitch_x as f32;
        let py = (dy + 1.0) * self.pitch_y as f32 - self.surface_height(cell_x, cell_y);
        (px, py)
    }

    pub fn cell_surface(&self, rx: i32, ry: i32) -> f32 {
        let deck = if self.bridge_at(rx, ry) { 4 } else { 0 }; // 桥面高 4 级
        ((self.level_at(rx, ry) + deck) * self.pitch_y) as f32
    }

    /// 同上，但给的是任意小数位置：格心带各自的高度，格与格之间插值。
    /// 坡道是在它自己的格内倾斜的，给整格一个常数抬升会让模型在上坡半格悬空、下坡半格埋进地里
    /// —— 那就是下坡时看得见的「下沉」。
    pub fn surface_height(&self, fx: f32, fy: f32) -> f32 {
        if !self.valid() {
            return 0.0;
        }
        // 拥有这个点的格。格心在整数坐标上，所以包含它的格就是最近那个，局部偏移从它的
        // 减 rx / 减 ry 角量起。
        let cx = fx + 0.5;
        let cy = fy + 0.5;
        let rx = cx.floor() as i32;
        let ry = cy.floor() as i32;
        let u = cx - rx as f32; // 0..1，沿 +rx 横过该格
        let v = cy - ry as f32; // 0..1，沿 +ry 横过该格
        // 坡道格的砖是过它四角高度的平面，所以插值这四个角就能把模型摆在**地图真正画出来的**
        // 那个面上 —— 它站在哪一格、以及整个穿越过程都成立。
        let c = ramp_corner_heights(self.ramp_at(rx, ry));
        let base = self.cell_surface(rx, ry);
        let py = self.pitch_y as f32;
        let h00 = base + (c[0][0] as f32) * py;
        let h10 = base + (c[1][0] as f32) * py;
        let h01 = base + (c[0][1] as f32) * py;
        let h11 = base + (c[1][1] as f32) * py;
        let a = h00 + (h10 - h00) * u;
        let b = h01 + (h11 - h01) * u;
        a + (b - a) * v
    }
}

/// 上游 `cmdMap` / `cmdPlay` / `cmdSimNav` 里逐字重复的那段：从 terrain cells 建高度场。
/// 返回（高度场，坡道格计数）—— 计数上游只累加没使用，这里带出来供对照打印。
pub fn build_ground_field(map: &MapFile, tb: &TerrainBuild) -> (GroundField, usize) {
    let mut gmax_x = 0;
    let mut gmax_y = 0;
    for c in &tb.cells {
        gmax_x = gmax_x.max(c.rx);
        gmax_y = gmax_y.max(c.ry);
    }
    let mut gf = GroundField {
        w: gmax_x + 1, // 栅格比地图宽高大：rx,ry 跨 [1, W+H-1]
        h: gmax_y + 1,
        map_width: map.width,
        ..Default::default()
    };
    let gcells = (gf.w as usize) * (gf.h as usize);
    gf.level = vec![0i16; gcells];
    gf.bridge = vec![0u8; gcells];
    gf.ramp = vec![0u8; gcells];
    gf.lift_px = vec![0i16; gcells];
    gf.tilt_pitch = vec![0i16; gcells];
    gf.tilt_yaw = vec![0i16; gcells];
    let mut ramped = 0usize;
    for c in &tb.cells {
        if c.rx < 0 || c.ry < 0 || c.rx >= gf.w || c.ry >= gf.h {
            continue;
        }
        let i = (c.ry as usize) * (gf.w as usize) + (c.rx as usize);
        gf.level[i] = c.z as i16;
        if c.bridge {
            gf.bridge[i] = 1;
        }
        gf.ramp[i] = c.ramp_type;
        gf.lift_px[i] = ramp_pixel_lift(c.ramp_type as i32, c.level) as i16;
        let (p, yw) = ramp_tilt(c.ramp_type as i32);
        gf.tilt_pitch[i] = p.round() as i16;
        gf.tilt_yaw[i] = yw.round() as i16;
        if c.ramp_type > 0 {
            ramped += 1;
        }
    }
    (gf, ramped)
}

// ============================================================================ 重力碗模拟

use std::sync::Arc;

use bevy::prelude::*;

use crate::crowd;
use crate::nav::{self, CostGrid, FlowField, Footprint};

/// 八个格邻居，顺序与 `nav::facing_octant` 一致（上游 world.cpp 的 kNeighX/Y）。
use crate::nav::{K_DIR_X as K_NEIGH_X,K_DIR_Y as K_NEIGH_Y};

/// 机头要离下一步的朝向多近才允许迈步（256 = 一整圈）。转弯是在车**站的地方**转：车头按
/// `turn_rate` 摆过来、进到 `kFaceTol` 以内才起步。设 0 就永远等不到（步进式转角到不了精确值），
/// 设 32（一个八分位）就会横着起步。
const K_FACE_TOL: f32 = 8.0;

/// 滚到碗底平地后还能滚多少格。够它横穿平地（从一侧掉进来的编队会铺开而不是堆在落点），
/// 又有限（平地终究会静下来）。
const K_ROLL_BUDGET: i32 = 999;

/// 一辆车 —— 上游 `Entity` 的**寻路那一半**。
/// 渲染字段（pix/pal/zbuf/rgba/art/anchor/mode/…）不搬：由 Bevy 侧承担。
#[derive(Component, Clone, Debug)]
pub struct Unit {
    /// 稳定索引：只用来把遍历顺序钉死（`occupied` 里存的 tag 是「排序位置 + 1」）。
    pub idx: i32,
    /// Shared command identity; None before the first order.
    pub command_id: Option<crate::simulation::CommandId>,
    pub parking: Option<crate::formation::Target>,
    pub contact_hold: bool,
    pub arrival: crate::arrival::Arrival,
    /// Smoothed objective, never a post-solve acceleration limit.
    pub preferred_velocity: Option<bevy::math::Vec2>,
    pub traffic_phase:crate::traffic::Phase,
    pub traffic_hold:bool,
    pub recovery_phase:crate::recovery::Phase,
    pub no_progress_seconds:f64,

    pub on_ground: bool,
    /// 位置（格单位）。上游 onGround 实体用 (cellX,cellY)、非地面用 (x,y)；这里统一成格坐标。
    pub cell_x: f32,
    pub cell_y: f32,
    /// 当前速度（格/秒）
    pub vx: f32,
    pub vy: f32,
    pub speed: f32,

    /// 正在走的那条腿：恰好两格 [from, to]。多格路线 = 重解出来的绕行。
    pub path: Vec<Cell>,
    /// 沿这条腿已走过的格数
    pub leg_t: f32,
    /// 指令真正要它上的那一格；x < 0 = 没有指令
    pub goal: Cell,
    /// 上一 tick 迈出的格 —— 运动层拒绝直接折返，免得被挤着在两格间来回抖
    pub last_step: Cell,
    /// 它跟随的那张场。一次编队指令只解**一张**多源场，所有车共用。
    pub guide: Option<Arc<FlowField>>,
    /// 距它可以动身还有几秒（编队指令给每车盖一个不同的微小延迟）
    pub start_delay: f32,
    /// 连续多少 tick 想动却没动成（卡死判据）
    pub stall_ticks: u32,
    /// 切向逃逸剩余 tick（卡死后横着走一会儿，绕开对头）
    pub escape_ticks: i32,
    /// 连续多少 tick 没挪窝。用来判定「闲下来了」——
    /// 比「离目标多近」可靠：碗底格是全队共享的，按距离判会让大多数车永远不算到位。
    pub still_ticks: u32,
    /// 炮塔是否「闲」（由 still_ticks 推出）。**只是显示层概念**，不参与导航。
    pub turret_idle: bool,
    /// 逃逸方向 +1/-1
    pub escape_side: f32,
    /// 还在平地上自由滚动的剩余格数（碗底惯性）
    pub roll: i32,
    /// 滚下墙到平地时授给的行动预算；只在「被**已停**的车挡住」时按 tick 扣 1
    pub ap_budget: i32,
    /// 滚进平地时的方向（平地上唯一还说得上话的方向）
    pub enter_dx: i32,
    pub enter_dy: i32,

    /// 朝向（游戏单位 0..255）。`dir_target` 是下一步的朝向（八分位）。
    pub dir_angle: f32,
    pub dir_target: f32,
    /// 炮塔角：瞄最终目的地方位，可以是小数 —— 它是瞄准，不是驾驶
    pub turret_angle: f32,
    pub turn_rate: f32,
    pub turret_rate: f32,

    pub cell_z: i32,
    pub ramp_type: i32,
    pub tilt_pitch: i32,
    pub tilt_yaw: i32,
    pub ground_dirty: bool,
}

impl Default for Unit {
    fn default() -> Self {
        Unit {
            idx: 0,
            command_id: None,
            parking: None,
            contact_hold: false,
            arrival: crate::arrival::Arrival::default(),
            preferred_velocity: None,
            traffic_phase:crate::traffic::Phase::Free,
            traffic_hold:false,
            recovery_phase:crate::recovery::Phase::Monitoring,
            no_progress_seconds:0.0,
            on_ground: true,
            cell_x: 0.0,
            cell_y: 0.0,
            vx: 0.0,
            vy: 0.0,
            speed: 6.0,
            path: Vec::new(),
            leg_t: 0.0,
            goal: Cell::new(-1, -1),      // 上游 goal{-1,-1}
            last_step: Cell::new(-1, -1), // 上游 lastStep{-1,-1}
            guide: None,
            start_delay: 0.0,
            stall_ticks: 0,
            escape_ticks: 0,
            escape_side: 1.0,
            still_ticks: 0,
            turret_idle: false,
            roll: 0,
            ap_budget: 0,
            enter_dx: 0,
            enter_dy: 0,
            dir_angle: 0.0,
            dir_target: 0.0,
            turret_angle: 0.0,
            turn_rate: 320.0,
            turret_rate: 900.0,
            cell_z: 0,
            ramp_type: 0,
            tilt_pitch: 0,
            tilt_yaw: 0,
            ground_dirty: true,
        }
    }
}

/// 模拟的共享世界态：地面高度场、可通行网格、足迹，以及复用的占用缓冲。
#[derive(Resource)]
pub struct SimEnv {
    pub ground: GroundField,
    pub nav: CostGrid,
    pub footprint: Footprint,
    /// 本 tick 开始时各格的占用（tag = 排序位置 + 1；0 = 空）
    pub occupied: Vec<i32>,
    pub tick: u64,
    pub runtime: crate::simulation::Runtime,
    pub buffers: crate::simulation::Buffers,
    pub reservations: crate::formation::Reservations,

    // ---- 连续运动层（路线 C）------------------------------------------------
    /// true = 走 `update_sim_crowd`（互斥力 + 路权 + 压力场），false = 走旧的离散重力碗
    pub use_crowd: bool,
    pub velocity_mode: crowd::VelocityMode,
    pub traffic:crate::traffic::Traffic,
    pub recovery:crate::recovery::Recovery,
    /// 拥挤度压力场（低频重建）
    pub pressure: crowd::Pressure,
    pub crowd_params: crowd::Params,
    /// 距上次重建压力场的 tick 数
    pub since_pressure: u32,
}

impl Default for SimEnv {
    fn default() -> Self {
        SimEnv {
            ground: GroundField::default(),
            nav: CostGrid::default(),
            footprint: Footprint::default(),
            occupied: Vec::new(),
            tick: 0,
            runtime: crate::simulation::Runtime::default(),
            buffers: crate::simulation::Buffers::default(),
            reservations: crate::formation::Reservations::default(),
            use_crowd: true,
            velocity_mode: crowd::VelocityMode::Orca,
            traffic:crate::traffic::Traffic::default(),
            recovery:crate::recovery::Recovery::default(),
            pressure: crowd::Pressure::default(),
            crowd_params: crowd::Params::default(),
            since_pressure: 0,
        }
    }
}

/// 把 `a` 朝 `t` 转最多 `max_step` 个游戏单位，走 256 圈上的短路。这就是把一次离散的转向
/// 变成看得见的过程的东西。
pub(crate) fn turn_toward(a: f32, t: f32, max_step: f32) -> f32 {
    let mut d = (t - a) % 256.0;
    if d > 128.0 {
        d -= 256.0;
    }
    if d < -128.0 {
        d += 256.0;
    }
    if d > max_step {
        d = max_step;
    }
    if d < -max_step {
        d = -max_step;
    }
    let mut r = (a + d) % 256.0;
    if r < 0.0 {
        r += 256.0;
    }
    r
}

/// `a` 还要摆多少才能到 `t`（朝向单位，0..128）。
fn angle_gap(a: f32, t: f32) -> f32 {
    let mut d = (t - a) % 256.0;
    if d > 128.0 {
        d -= 256.0;
    }
    if d < -128.0 {
        d += 256.0;
    }
    d.abs()
}

/// 每 tick 推进重力碗 —— 上游 `World::update` 的对等移植。
///
/// 🔴 顺序是语义的一部分：先 `rebuildOccupancy`，再按**稳定顺序**逐车处理，而且 `occupied`
/// 在 tick 内**就地更新**（队首让出的格当 tick 就能被后面顶上 —— 这就是「后面的球推前面的球」）。
/// 所以先把顺序取出来（按 `idx` 排序），再按序 `get_mut`；`occupied`/`claimed`/`committed`
/// 都是本地缓冲，邻居的 `roll` 用一份镜像读，于是不需要跨实体借用。
pub fn update_sim(time: Res<Time<Fixed>>, mut env: ResMut<SimEnv>, mut q: Query<(Entity, &mut Unit)>) {
    if env.use_crowd { return; }
    let dt = time.delta_secs();

    let mut order: Vec<(i32, Entity)> = q.iter_mut().map(|(e, u)| (u.idx, e)).collect();
    order.sort_by_key(|&(i, _)| i);

    let env = &mut *env;
    let ground = &env.ground;
    let nav = &env.nav;
    if !ground.valid() || nav.cost.is_empty() {
        return;
    }
    let gw = ground.w;
    let gh = ground.h;
    let n = (gw as usize) * (gh as usize);

    // --- rebuildOccupancy：按位置写 occupied（tag = 位置 + 1）--------------------------
    let mut occupied = std::mem::take(&mut env.occupied);
    occupied.clear();
    occupied.resize(n, 0);
    for (i, &(_, e)) in order.iter().enumerate() {
        let u = q.get_mut(e).unwrap().1;
        if !u.on_ground || u.cell_x < 0.0 {
            continue;
        }
        let cx = u.cell_x.round() as i32;
        let cy = u.cell_y.round() as i32;
        if cx >= 0 && cy >= 0 && cx < gw && cy < gh {
            occupied[(cy as usize) * (gw as usize) + (cx as usize)] = i as i32 + 1;
        }
    }

    // 本 tick 已承诺的腿：两条腿共享中点（同一方格的两条对角线）不能都走 —— 否则两车会在
    // 那一格的当中面对面。
    let mut committed: Vec<(Cell, Cell)> = Vec::with_capacity(order.len());
    for &(_, e) in &order {
        let u = q.get_mut(e).unwrap().1;
        if !u.on_ground || u.leg_t <= 0.0 || u.path.len() < 2 {
            continue;
        }
        committed.push((u.path[0], u.path[1]));
    }
    // 本 tick 的认领：`occupied` 是世界在本 tick 开始时的样子，走车过程中从不写它，
    // 于是一辆车站着的格始终归它；这张图才是让「认领」成为原子的东西 —— 两辆车不可能在
    // 同一 tick 里都拿到同一个空格。
    let mut claimed = vec![0i32; n];
    for (i, &(_, e)) in order.iter().enumerate() {
        let u = q.get_mut(e).unwrap().1;
        if !u.on_ground || u.leg_t <= 0.0 || u.path.len() < 2 {
            continue;
        }
        // 只预留下一个格心。预留整条剩余路线试过、严格更糟：重解出来的绕行能有几十格，
        // 把它整条占住会把其他车饿到死锁 —— 正是「先到的把后面的全堵死」那个失败。
        let tx = u.path[1].x;
        let ty = u.path[1].y;
        if tx >= 0 && ty >= 0 && ty < gh {
            let ti = (ty as usize) * (gw as usize) + (tx as usize);
            if ti < claimed.len() {
                claimed[ti] = i as i32 + 1;
            }
        }
    }

    // 邻居 `roll` 的镜像：处理一辆车时要读邻居的 roll（哪怕它本 tick 已被改过），
    // 而邻居此刻可能正被 ECS 借走。
    let mut rolls: Vec<i32> = order
        .iter()
        .map(|&(_, e)| q.get_mut(e).unwrap().1.roll)
        .collect();

    // --- 逐车 ----------------------------------------------------------------------
    for i in 0..order.len() {
        let e = order[i].1;
        let self_tag = i as i32 + 1;
        let mut u = q.get_mut(e).unwrap().1;
        // 🔴 两个边界都对 ground 自己的范围查：occupied/claimed 是按 gw*gh 分配的，
        // 用错的宽高建下标就会跑出数组。
        // 本 tick 的起点位置（用来判「有没有挪窝」）
        let px_before = u.cell_x;
        let py_before = u.cell_y;
        let cell_idx = |x: i32, y: i32| -> i32 {
            if x < 0 || y < 0 || x >= gw || y >= gh {
                -1
            } else {
                y * gw + x
            }
        };

        if !u.on_ground {
            u.cell_x += u.vx * dt;
            u.cell_y += u.vy * dt;
            rolls[i] = u.roll;
            continue;
        }

        if u.leg_t > 0.0 && u.path.len() >= 2 {
            if nav::step_cost(nav,u.path[0],u.path[1]).is_none() {
                u.path.clear();u.leg_t=0.0;u.vx=0.0;u.vy=0.0;continue;
            }
            // 走腿中：把它推到对面格心。一步一旦开始就必须走完 —— 停在两格之间的身体不属于
            // 任何一格，既不能被超越也不能被跟随。
            let a = u.path[0];
            let b = u.path[1];
            let ldx = (b.x - a.x) as f32;
            let ldy = (b.y - a.y) as f32;
            let len = (ldx * ldx + ldy * ldy).sqrt();
            u.leg_t += u.speed * dt;
            if u.leg_t >= len {
                u.cell_x = b.x as f32;
                u.cell_y = b.y as f32;
                if u.path.len() > 2 {
                    // 多格路线（重解出来的绕行）：这条腿走的时候只预留了下一个格心，再下一格
                    // 可能已被别人占了，而这条分支从不查 occupied —— 在这里补查。
                    let after = u.path[2];
                    let after_idx = cell_idx(after.x, after.y);
                    let after_taken = after_idx >= 0
                        && ((occupied[after_idx as usize] != 0
                            && occupied[after_idx as usize] != self_tag)
                            || (claimed[after_idx as usize] != 0
                                && claimed[after_idx as usize] != self_tag));
                    if after_taken {
                        u.path.clear();
                        u.leg_t = 0.0;
                    } else {
                        u.path.remove(0);
                        u.leg_t = 1e-4;
                        let next_idx = cell_idx(u.path[1].x, u.path[1].y);
                        if next_idx >= 0 {
                            claimed[next_idx as usize] = self_tag;
                        }
                    }
                } else {
                    u.path.clear();
                    u.leg_t = 0.0;
                }
                u.vx = 0.0;
                u.vy = 0.0;
            } else {
                let f = if len > 0.0 { u.leg_t / len } else { 0.0 };
                u.cell_x = a.x as f32 + ldx * f;
                u.cell_y = a.y as f32 + ldy * f;
                u.vx = (if len > 0.0 { ldx / len } else { 0.0 }) * u.speed;
                u.vy = (if len > 0.0 { ldy / len } else { 0.0 }) * u.speed;
            }
        } else {
            u.path.clear();
            u.leg_t = 0.0;
            u.vx = 0.0;
            u.vy = 0.0;
            let cx = u.cell_x.round() as i32;
            let cy = u.cell_y.round() as i32;

            // 🔴 这里**没有「到达」**，而这正是重点。流体里的一个分子不会到某格就停下：
            // 它一直顺着场往下掉，直到地面顶上来接住它 —— 直到它自己这一层及以下全被占满、
            // 它被原地压实。所以车永远不离开场、也从不被命令停下。
            if u.goal.x >= 0 {
                let guide = u.guide.clone();
                if let Some(gf) = guide {
                    if gf.in_(cx, cy) {
                        let here = gf.dist_at(cx, cy);
                        // 已经在平地上把动量花完的球守住自己的格。墙上的球不受此限：那里有真
                        // 的下坡可取，正是它把球带到平地上去的。
                        let floor_parked = here <= 1e-6 && u.roll <= 0;

                        // 最省的下坡邻居（不管交通）：等的时候车头就对着它，路一空就出发。
                        let mut dx = 0;
                        let mut dy = 0;
                        let mut best_any = f32::INFINITY;
                        // 以及本 tick 真正空闲的最省严格下坡邻居。
                        let mut bx = 0;
                        let mut by = 0;
                        let mut best_free = f32::INFINITY;
                        let mut have_free = false;
                        for d in 0..8 {
                            let ax = cx + K_NEIGH_X[d];
                            let ay = cy + K_NEIGH_Y[d];
                            let ti = cell_idx(ax, ay);
                            if ti < 0 || nav::step_cost(nav,Cell::new(cx,cy),Cell::new(ax,ay)).is_none() {
                                continue;
                            }
                            let s = gf.dist_at(ax, ay);
                            let score=s+nav::step_cost(nav,Cell::new(cx,cy),Cell::new(ax,ay)).unwrap();
                            if s < here && score < best_any {
                                best_any = score;
                                dx = K_NEIGH_X[d];
                                dy = K_NEIGH_Y[d];
                            }
                            if s < here && score < best_free
                                && occupied[ti as usize] == 0
                                && claimed[ti as usize] == 0
                            {
                                best_free = score;
                                bx = ax;
                                by = ay;
                                have_free = true;
                            }
                        }
                        if dx != 0 || dy != 0 {
                            u.dir_target = nav::facing_octant(dx, dy) as f32;
                        }

                        // 没有严格更省的格是空闲的。横穿等值线（同代价的一步，分子沿等势面滑动）
                        // 是允许的，但**仅当**这一步真的打开了下坡的路：落点自身必须还有一个空闲、
                        // 严格更省的邻居。
                        let mut sidestep = false;
                        if !have_free {
                            // 碗底平地上的惯性：球从墙滚上平地就保持来时的方向继续滚 ——
                            // 下坡没了也不停，因为平地上没有东西拦它。取来向上投影**最大**的
                            // 那个空闲平地格。迎面优先；前方的格被占时，旁边两格接着算 ——
                            // 那是球从一辆停着的球旁边挤过去，而不是停在它后面。
                            if gf.dist_at(cx, cy) <= 1e-6 {
                                let mut ix = u.enter_dx;
                                let mut iy = u.enter_dy;
                                if ix == 0 && iy == 0 {
                                    // 没见过它滚进来（不该发生）
                                    ix = cx - u.last_step.x;
                                    iy = cy - u.last_step.y;
                                }
                                let mut best_proj = i32::MIN;
                                let mut best_perp = i32::MAX;
                                for d in 0..8 {
                                    let ax = cx + K_NEIGH_X[d];
                                    let ay = cy + K_NEIGH_Y[d];
                                    let ti = cell_idx(ax, ay);
                                    if ti < 0 || nav::step_cost(nav,Cell::new(cx,cy),Cell::new(ax,ay)).is_none() {
                                        continue;
                                    }
                                    if ax == u.last_step.x && ay == u.last_step.y {
                                        continue; // 不折返
                                    }
                                    if occupied[ti as usize] != 0 || claimed[ti as usize] != 0 {
                                        continue;
                                    }
                                    if gf.dist_at(ax, ay) > here + 1e-3 {
                                        continue; // 只走平地，绝不往上爬
                                    }
                                    let ox = ax - cx;
                                    let oy = ay - cy;
                                    let proj = ix * ox + iy * oy;
                                    if proj < 1 {
                                        continue; // 只往前：不与之平齐，也不往后
                                    }
                                    let mut cross = ix * oy - iy * ox;
                                    if cross < 0 {
                                        cross = -cross;
                                    }
                                    if proj > best_proj || (proj == best_proj && cross < best_perp) {
                                        best_proj = proj;
                                        best_perp = cross;
                                        bx = ax;
                                        by = ay;
                                        sidestep = true;
                                    }
                                }
                            }
                            // 还在墙上：横穿等值线的一步可以绕开阻塞，但落点必须自己还有更省的
                            // 邻居（opensDown），否则已静止的队形会沿等值线永远挪动。
                            if !sidestep && here > 1e-6 {
                                for d in 0..8 {
                                    let ax = cx + K_NEIGH_X[d];
                                    let ay = cy + K_NEIGH_Y[d];
                                    let ti = cell_idx(ax, ay);
                                    if ti < 0 || nav::step_cost(nav,Cell::new(cx,cy),Cell::new(ax,ay)).is_none() {
                                        continue;
                                    }
                                    if ax == u.last_step.x && ay == u.last_step.y {
                                        continue;
                                    }
                                    if occupied[ti as usize] != 0 || claimed[ti as usize] != 0 {
                                        continue;
                                    }
                                    if gf.dist_at(ax, ay) > here + 1e-3 {
                                        continue; // 至多等代价
                                    }
                                    let side = gf.dist_at(ax, ay);
                                    let mut opens_down = false;
                                    for d2 in 0..8 {
                                        let nx = ax + K_NEIGH_X[d2];
                                        let ny = ay + K_NEIGH_Y[d2];
                                        let ni = cell_idx(nx, ny);
                                        if ni < 0 || nav::step_cost(nav,Cell::new(ax,ay),Cell::new(nx,ny)).is_none() {
                                            continue;
                                        }
                                        if occupied[ni as usize] != 0
                                            || claimed[ni as usize] != 0
                                        {
                                            continue;
                                        }
                                        if gf.dist_at(nx, ny) < side - 1e-3 {
                                            opens_down = true;
                                            break;
                                        }
                                    }
                                    if !opens_down {
                                        continue;
                                    }
                                    bx = ax;
                                    by = ay;
                                    sidestep = true;
                                    break;
                                }
                            }
                        }

                        if (have_free || sidestep) && !floor_parked {
                            // 拒绝一条与已承诺腿交叉的步：两个端点不同，claimed 看不出问题 ——
                            // 撞的是两格的中点。两条腿共享中点，当且仅当 from+to 的和相同。
                            let mut crosses = false;
                            for (cf, cs) in &committed {
                                if cf.x == cx && cf.y == cy {
                                    continue;
                                }
                                if cx + bx == cf.x + cs.x && cy + by == cf.y + cs.y {
                                    crosses = true;
                                    break;
                                }
                            }
                            // 只有机头摆到这一步朝向上才允许迈步。`dir_target` 重新指向真正
                            // 选中的那一格。
                            u.dir_target = nav::facing_octant(bx - cx, by - cy) as f32;
                            let faced = angle_gap(u.dir_angle, u.dir_target) <= K_FACE_TOL;
                            if !crosses && faced {
                                let ti = cell_idx(bx, by);
                                u.path.clear();
                                u.path.push(Cell::new(cx, cy));
                                u.path.push(Cell::new(bx, by));
                                u.leg_t = 1e-4; // 起步：从这刻起承诺走到对面格心
                                u.last_step = Cell::new(cx, cy);
                                claimed[ti as usize] = self_tag;
                                committed.push((Cell::new(cx, cy), Cell::new(bx, by)));
                                // 后面那辆球推前面那辆：occupied 是本 tick 开始时的图，但刚
                                // 起步的球其实已经让出了它在走的格 —— 后面那辆正想要它，否则
                                // 会一直读到「被占」而卡住。这里让图保持活的，压力就能在一 tick
                                // 内沿一列传下去。
                                let from_idx = cell_idx(cx, cy);
                                if from_idx >= 0 && from_idx != ti {
                                    occupied[from_idx as usize] = 0;
                                    occupied[ti as usize] = self_tag;
                                }
                                // 花掉一步平地动量，或者在把球从墙上带下来的那步领一份新预算。
                                let on_floor_now = gf.dist_at(bx, by) <= 1e-6;
                                if on_floor_now && here > 1e-6 {
                                    u.roll = if u.ap_budget > 0 {
                                        u.ap_budget
                                    } else {
                                        K_ROLL_BUDGET
                                    };
                                    u.enter_dx = bx - cx;
                                    u.enter_dy = by - cy;
                                }
                            }
                        }
                        // 等不是白等 —— 但只有「撞上已经停下」的那种等才花动量：
                        // 那才是真正消耗动量、也才可能把一群人永远卡死的碰撞。
                        let mut parked_neighbour = false;
                        for d in 0..8 {
                            let ni = cell_idx(cx + K_NEIGH_X[d], cy + K_NEIGH_Y[d]);
                            if ni < 0 {
                                continue;
                            }
                            let who = occupied[ni as usize];
                            if who != 0
                                && who != self_tag
                                && ((who - 1) as usize) < rolls.len()
                                && rolls[(who - 1) as usize] <= 0
                            {
                                parked_neighbour = true;
                                break;
                            }
                        }
                        if u.leg_t <= 0.0 && parked_neighbour && u.roll > 0 {
                            u.roll -= 1;
                        }
                    }
                }
            }
            // 还是卡着：有人站在它必须去的地方，也没有等代价的空格可滑。那就等 —— 是被压实，
            // 不是被命令停下。这里什么都不解：还在下落的邻居会自己让开。
        }

        // 车体朝向。一步在走的时候朝向**锁死**在那一步上 —— 机头不会中途去找新方向 ——
        // 但仍然会把最后几度摆完。炮塔不是车体：它瞄指令那一格、转得更快。
        if u.path.len() >= 2 && u.leg_t > 0.0 {
            u.dir_target = nav::facing_octant(u.path[1].x - u.path[0].x, u.path[1].y - u.path[0].y) as f32;
        }
        u.dir_angle = turn_toward(u.dir_angle, u.dir_target, u.turn_rate * dt);
        aim_turret(&mut u, nav, dt);

        // 闲下来了就撤销指令。
        //
        // 判据不是「离目标多近」—— 碗底格是全队共享的（`path_to_nearest_source` 常给同一格），
        // 按距离判只会清掉站在那格上的那一辆，其余车永远瞄着那个共享点，看起来就是
        // 「所有炮塔都指着第一个停下的单位」。改成**看它自己有没有挪窝**：
        // 连续 20 tick（约 1/3 秒）没动、也不在半路，就算闲下来。
        let moved_now = (u.cell_x - px_before).abs() + (u.cell_y - py_before).abs() > 1e-4;
        if moved_now || u.leg_t > 0.0 {
            u.still_ticks = 0;
        } else {
            u.still_ticks = u.still_ticks.saturating_add(1);
        }
        u.turret_idle = u.still_ticks > 20;

        // 坐在自己脚下那格的地面上（资产层的重栅格化钩子属渲染，由 Bevy 侧承担）。
        if ground.valid() {
            let rx = u.cell_x.round() as i32;
            let ry = u.cell_y.round() as i32;
            let z = ground.level_at(rx, ry);
            let ramp = if u.on_ground { ground.ramp_at(rx, ry) } else { 0 };
            if u.ground_dirty || z != u.cell_z || ramp != u.ramp_type {
                u.ground_dirty = false;
                u.cell_z = z;
                u.ramp_type = ramp;
                let (p, y) = ground.tilt_at(rx, ry);
                u.tilt_pitch = p;
                u.tilt_yaw = y;
            }
        }

        rolls[i] = u.roll;
    }

    env.occupied = occupied;

    env.tick += 1;
}


// ============================================================================ Continuous ECS adapter

/// Snapshot ECS data, call the shared fixed-step core, then write it back once.
/// The explicit mode guard also protects callers outside the game schedule.
pub fn update_sim_crowd(mut env: ResMut<SimEnv>, mut q: Query<(Entity, &mut Unit)>) {
    if !env.use_crowd { return; }
    let mut order: Vec<_> = q.iter().map(|(e, u)| (u.idx, e)).collect();
    order.sort_by_key(|&(id, _)| id);
    let mut units: Vec<_> = order.iter().map(|&(_, e)| q.get(e).unwrap().1.clone()).collect();
    crate::simulation::step_continuous(&mut env, &mut units);
    for ((_, e), unit) in order.into_iter().zip(units) { *q.get_mut(e).unwrap().1 = unit; }
}

/// 炮塔瞄准（两套运动系统共用）。
///
/// 有指令时：**瞄接下来要去的地方**（流场前瞻），不是瞄最初那个格子 —— 否则走过头就朝后看。
/// 离目标 1.5 格内改瞄目标格，收尾要稳。
/// 无指令时：以 turret_rate 的 20% 慢慢摆回车头方向，别硬僵在最后那个点上。
pub(crate) fn aim_turret(u: &mut Unit, grid: &CostGrid, dt: f32) {
    // 「闲」只由 turret_idle 决定，绝不看 goal
    if u.goal.x < 0 || u.turret_idle {
        u.turret_angle = turn_toward(u.turret_angle, u.dir_angle, u.turret_rate * 0.2 * dt);
        return;
    }
    let dxg = u.goal.x as f32 - u.cell_x;
    let dyg = u.goal.y as f32 - u.cell_y;
    let dist = (dxg * dxg + dyg * dyg).sqrt();

    let mut aim: Option<f32> = None;
    if dist > 1.5 {
        if let Some((ax, ay)) = flow_lookahead(u, grid, 4) {
            let bx = ax - u.cell_x;
            let by = ay - u.cell_y;
            if bx.abs() + by.abs() > 0.05 {
                aim = Some(nav::facing_angle_of(bx, by));
            }
        }
    }
    let target = aim.unwrap_or_else(|| {
        if dxg.abs() + dyg.abs() > 0.05 {
            nav::facing_angle_of(dxg, dyg)
        } else {
            u.dir_angle
        }
    });
    u.turret_angle = turn_toward(u.turret_angle, target, u.turret_rate * dt);
}

/// 沿流场往前看 `steps` 步，返回那个格的**中心点**。
///
/// 为什么需要它：流场里 `Unit::goal` 只是「分给它的碗底格」。单位顺流线走过头、
/// 或被挤到目标另一侧之后，`goal` 就落到身后了 —— 炮塔照着 `goal` 转，就会朝后看。
/// A* 里没这问题，因为那儿 goal 就是终点本身。
///
/// 做法：从当前格开始，每步挑 8 邻域里 `dist` 最小的格；挑不出更小的就停。
/// 找不到场 / 已经在最省格上 → 返回 None，调用方退回原逻辑。
fn flow_lookahead(u: &Unit, grid:&CostGrid, steps: i32) -> Option<(f32, f32)> {
    let gf = u.guide.as_ref()?;
    let mut cx = u.cell_x.round() as i32;
    let mut cy = u.cell_y.round() as i32;
    if !gf.in_(cx, cy) {
        return None;
    }
    let mut moved=false;
    for _ in 0..steps.max(1) {
        let Some(next)=nav::flow_step(grid,gf,Cell::new(cx,cy)) else {break;};
        cx=next.x;cy=next.y;moved=true;
    }
    if moved {
        Some((cx as f32, cy as f32))
    } else {
        None
    }
}

// ============================================================================ 单测

#[cfg(test)]
mod tests {
    use super::*;


    /// 炮塔前瞻：无论单位在碗底的哪一侧，前瞻点都必须比当前位置**更靠近**碗底，
    /// 也就是“永远指向接下来要走的方向”，不会因为 goal 落在身后就朝后看。
    #[test]
    fn flow_lookahead_points_forward_not_backward() {
        use crate::nav::{compute_flow_field_cpu_multi, CostGrid, FlowField};

        let mut g = CostGrid::default();
        g.w = 16;
        g.h = 16;
        g.cost = vec![1u8; 16 * 16];

        let mut ff = FlowField::default();
        assert!(compute_flow_field_cpu_multi(&g, &[crate::nav::Cell::new(8, 8)], &mut ff));

        let mut u = Unit::default();
        u.guide = Some(std::sync::Arc::new(ff.clone()));
        u.goal = crate::nav::Cell::new(8, 8);

        // 单位在碗底左边 6 格
        u.cell_x = 2.0;
        u.cell_y = 8.0;
        let (ax, ay) = flow_lookahead(&u, &g, 4).expect("应该能前瞻");
        let d_now = ((2.0f32 - 8.0).powi(2) + (8.0 - 8.0f32).powi(2)).sqrt();
        let d_ahead = ((ax - 8.0).powi(2) + (ay - 8.0).powi(2)).sqrt();
        assert!(d_ahead < d_now, "前瞻点 {ax},{ay} 没有更靠近碗底（{d_ahead:.2} vs {d_now:.2}）");
        assert!(ax > 2.0, "在左边就该往右看，实得 x={ax}");
    }

    #[test]
    fn ramp_plane_of_type_one() {
        // type 1：抬 Top 与 Left 角 ⇒ 沿 +rx 爬 1 级、沿 +ry 不爬、格心高 0.5 级
        let (gx, gy, c) = ramp_plane(1);
        assert!((gx - 1.0).abs() < 1e-6, "gx={gx}");
        assert!((gy - 0.0).abs() < 1e-6, "gy={gy}");
        assert!((c - 0.5).abs() < 1e-6, "c={c}");
    }

    #[test]
    fn ramp_pixel_lift_matches_the_game_table() {
        // RampHeight：ramp 1 在格心抬半级 = 52 leptons ≈ 7 px
        assert_eq!(ramp_pixel_lift(1, 0), 7);
        assert_eq!(ramp_pixel_lift(0, 0), 0);
        // 5-8 是对角对，格心处抵消
        assert_eq!(ramp_pixel_lift(5, 0), 0);
        // 9-12 抬满一级 = 104 leptons ≈ 15 px
        assert_eq!(ramp_pixel_lift(9, 0), 15);
    }

    #[test]
    fn type_one_ramp_tilts_about_22_degrees() {
        let (pitch, yaw) = ramp_tilt(1);
        assert!((pitch.abs() - 22.11).abs() < 0.1, "pitch={pitch}");
        assert!((yaw - 0.0).abs() < 1e-6, "yaw={yaw}");
    }

    #[test]
    fn flat_and_saddle_types_have_no_pitch() {
        for t in [0, 17, 18, 19, 20] {
            let (p, _y) = ramp_tilt(t);
            assert_eq!(p, 0.0, "type {t} 不该有倾角");
        }
    }

    #[test]
    fn surface_height_interpolates_a_ramp_cell() {
        // 一格 type 1 坡道放在平地上：沿 +rx 从低到高线性插值
        let mut gf = GroundField {
            w: 8,
            h: 8,
            map_width: 4,
            ..Default::default()
        };
        gf.level = vec![0i16; 64];
        gf.ramp = vec![0u8; 64];
        gf.ramp[3 * 8 + 3] = 1; // (rx=3, ry=3) 是 type 1 坡道
        // 该格四角：h00=0, h10=1, h11=1, h01=0 级 ⇒ 面从 u=0 的 0 爬到 u=1 的 1
        let lo = gf.surface_height(2.6, 3.0); // u≈0.1
        let hi = gf.surface_height(3.4, 3.0); // u≈0.9
        assert!(hi > lo, "沿 +rx 应升高：lo={lo} hi={hi}");
    }

    #[test]
    fn turn_toward_takes_the_short_way() {
        // 250 -> 10：短路是 +16
        assert!((turn_toward(250.0, 10.0, 100.0) - 10.0).abs() < 1e-3);
        // 步长只给 8：转到 258 ≡ 2
        assert!((turn_toward(250.0, 10.0, 8.0) - 2.0).abs() < 1e-3);
        // 结果始终归一到 [0,256)
        let r = turn_toward(2.0, 250.0, 300.0);
        assert!((0.0..256.0).contains(&r), "r={r}");
    }

    #[test]
    fn angle_gap_is_at_most_half_a_turn() {
        assert!((angle_gap(0.0, 10.0) - 10.0).abs() < 1e-3);
        assert!((angle_gap(0.0, 250.0) - 6.0).abs() < 1e-3, "应走短路");
        assert!((angle_gap(0.0, 128.0) - 128.0).abs() < 1e-3);
    }
}

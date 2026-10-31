//! 可运行的演示：把地图、寻路网格和重力碗模拟放进一个 Bevy 窗口。
//!
//! 渲染 = **画面平贴**（2026-10-16 用户定，经 `ramp01.tem` 实测确认）：
//! TMP 的砖图案**本身就是这块砖在屏幕上的最终外观**（顶面 + 立面 / 坡面都画在图里），
//! 所以像 RA2 一样把它**平贴**成屏幕对齐的四边形，不引入任何几何倾斜；
//! 砖去重烘成一张图集，遮挡靠 GPU 深度缓冲。
//! 真 3D 的高度 / 坡角只留给物理侧（`world.rs`），与画面解耦。
//!
//! 用法：`ra2engine game [<map> <gameDir>] [--ini rulesmd.ini]`
//! **可调项全在工程根的 `rulesmd.ini` 里**（视口 / 单位 / 光源 / 亮度，见该文件注释）；
//! 命令行同名参数（`--units` / `--cellpx` / `--albedo` / `--sun` …）只用来临时覆盖。
//! 设 `RA2_SHOT=<out.png>` 会在 2.5s 自动截一张图再退出（无人盯屏时自证画出来了）。

use bevy::asset::RenderAssetUsages;
use bevy::camera::ScalingMode;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::image::{ImageSampler, ImageSamplerDescriptor};
use bevy::input::mouse::MouseWheel;
use bevy::light::{CascadeShadowConfigBuilder, DirectionalLightShadowMap, NotShadowCaster};
use bevy::mesh::Indices;
use bevy::prelude::*;
use bevy::render::render_resource::{Extent3d, PrimitiveTopology, TextureDimension, TextureFormat};
use bevy::render::view::Msaa;
use bevy::render::view::screenshot::{save_to_disk, Screenshot};
use bevy::time::Fixed;
use bevy::window::PrimaryWindow;

use crate::costgrid;
use crate::debugview;
use crate::hva;
use crate::ini::Ini;
use crate::lighting;
use crate::mapfile;
use crate::mix;
use crate::nav::{self, Cell, Footprint};
use crate::play::{self, Selection};
use crate::terrain;
use crate::theater;
use crate::voxel_mesh;
use crate::vxl;
use crate::world::{self, SimEnv, Unit};

/// 相机方向：从 `(1, √(2/3), 1)` 看向地图中心，得到 2:1 等距（水平 45°、俯角 30°）。
/// 这是纯观感参数，与格式无关。
const CAM_DIR: [f32; 3] = [1.0, 0.816_497, 1.0];

/// 相机的屏幕右 / 上方向（由 `CAM_DIR` + 世界 up = Y 推出来，见 `setup` 里的
/// `looking_at`）。构图时把地形顶点投到这两个方向上取极值，就知道画面该开多大。
const CAM_RIGHT: [f32; 3] = [0.707_106_8, 0.0, -0.707_106_8];
const CAM_UP: [f32; 3] = [-0.353_553_4, 0.866_025_4, -0.353_553_4];

/// 等距下「1 格」在屏幕上占的水平宽度（2 × 0.7071）：世界 +X / +Z 各投出 0.7071。
/// 用来把「1 格 = N 屏幕像素」换算成「1 世界单位 = 多少屏幕像素」。
const ISO_SPAN: f32 = 1.414_213_6;

/// 视图状态：缩放 + 相机看向的世界点。照抄 C++ `Play` 的 `camX / camY / zoom`
/// （`engine/play.cpp:48`、`step` :265-302）。
#[derive(Resource)]
pub(crate) struct View {
    /// 缩放系数。滚轮改它，范围与 C++ 一致 `[0.35, 2.5]`。
    zoom: f32,
    /// **未缩放时**「1 格 = 多少屏幕像素」（`--cellpx`，RA2 原版是 60）。0 = 自动适配全图。
    cell_px: f32,
    /// 相机看向的世界点（格坐标）。平移改它。
    look: Vec3,
    /// `cell_px == 0` 时用的视口世界范围。
    fit: Vec2,
    /// 地形在**屏幕基**下的范围：`map_h = (h0, h1)` 是地形顶点投影到 `CAM_RIGHT`
    /// 的极值、`map_v` 是投到 `CAM_UP` 的极值（`run` 里逐顶点算出来的）。
    /// 平移时相机被夹在它里面 —— 照 `play.cpp:138 clampCamera`。
    map_h: Vec2,
    map_v: Vec2,
    /// 俯视时的同一对范围：`map_h_top` = 世界 X 的极值、`map_v_top` = 世界 **−Z** 的极值
    /// （俯视屏幕上方 = −Z）。等距那套是按 `CAM_RIGHT/CAM_UP` 算的，方向不同不能混用。
    map_h_top: Vec2,
    map_v_top: Vec2,
    /// 是否从上往下看。debug 面板的按钮切它，`cam_basis` 按它给两套基向量。
    pub(crate) top_down: bool,
    /// 光标进到离屏幕边缘这么多像素以内就开始平移（`[View] EdgeScroll`）。0 = 关掉。
    edge_scroll: f32,
    /// 平移速度，单位是"每秒多少**未缩放像素**"（`[View] PanSpeed`）。
    pan_speed: f32,
}

impl View {
    /// 每世界单位多少屏幕像素（含缩放）。视口的世界范围与窗口**恒同比**（见 `size`），
    /// 所以水平和垂直两个方向一致，取水平即可。
    ///
    /// 【必须按窗口实算，不能再用 `cell_px / ISO_SPAN * zoom`】`cell_px == 0`（自动
    /// 适配全图，`启动.bat` 就是这个模式）时那个表达式恒等于 0，而平移速度是
    /// `760 / px_per_unit` —— 于是每帧 `look += 向量 * inf`，相机一贴边就炸成 NaN，
    /// 整个画面直接没了（用户报的"鼠标贴边不移动视口而是直接灰屏"就是这个）。
    fn px_per_unit(&self, win: Vec2) -> f32 {
        let s = self.size(win);
        if s.x <= 0.0 {
            return 1.0;
        }
        win.x / s.x
    }

    /// 视口要覆盖的世界范围（宽 = 屏幕水平方向上的世界跨度）。带缩放。
    ///
    /// 【恒与窗口同比】`ScalingMode::AutoMin` 会在水平、垂直两个方向上各给一个缩放，
    /// 宽高比不一致时画面会被**非等比拉伸**。这里先把范围按窗口比例摆好，它算出来
    /// 才是等比 —— 也才能让 `px_per_unit` 有一个唯一的值。
    fn size(&self, win: Vec2) -> Vec2 {
        // 未缩放时视口要覆盖的世界跨度。`cell_px > 0`：按「1 格 = N 像素」定，本身就
        // 与窗口同比。`cell_px == 0`：拿 `fit`（要装下的地形范围）等比放大到刚好装下。
        let raw = if self.cell_px > 0.0 {
            win / (self.cell_px / ISO_SPAN)
        } else {
            let ar = if win.y > 0.0 { win.x / win.y } else { 1.0 };
            if self.fit.x / self.fit.y > ar {
                Vec2::new(self.fit.x, self.fit.x / ar)
            } else {
                Vec2::new(self.fit.y * ar, self.fit.y)
            }
        };
        raw / self.zoom.max(0.01)
    }
}

/// 把视口中心夹到 `range` 里，使宽 `span` 的视口整个落在其中。
///
/// 对等 `play.cpp:138-144 clampCamera` 的 `min(max(camX,0), max(0, mapW-viewW))`，
/// 那是「视口左边缘不越出地图」的中心制写法。
///
/// **唯一一处主动偏离 C++**：C++ 在视口比地图还大时 `max(0, …)` 会把相机锁到
/// `camX = 0`（视口左上角对齐地图左上角），地图被顶到画面一角。它那边是等距像素
/// 坐标、`mapW` 远大于视口，只有极端缩小才碰得到；我们默认就是「缩小到看全图」，
/// 一走那条分支开局地图就偏到一侧、另一侧是大片背景。这里改成**居中**。
fn clamp_axis(c: f32, range: Vec2, span: f32) -> f32 {
    let lo = range.x + span * 0.5;
    let hi = range.y - span * 0.5;
    if lo > hi {
        (range.x + range.y) * 0.5
    } else {
        c.clamp(lo, hi)
    }
}

/// 相机的 (光轴方向, 屏幕右, 屏幕上, `look_at` 的 up 提示)。
///
/// - **等距**：三个常量（`CAM_DIR` / `CAM_RIGHT` / `CAM_UP`），`up` 用世界 Y。
/// - **俯视**：相机在看向点的正上方，光轴 = `+Y`；屏幕右 = `+X`、屏幕上 = `−Z`。
///   `up` 提示**必须**换成 `−Z`：正下方看时 `Vec3::Y` 与视线平行，`look_at` 的绕轴
///   朝向不可解，画面会自己转。
fn cam_basis(top_down: bool) -> (Vec3, Vec3, Vec3, Vec3) {
    if top_down {
        (Vec3::Y, Vec3::X, Vec3::NEG_Z, Vec3::NEG_Z)
    } else {
        (
            Vec3::from_array(CAM_DIR).normalize(),
            Vec3::from_array(CAM_RIGHT),
            Vec3::from_array(CAM_UP),
            Vec3::Y,
        )
    }
}

/// 待执行的指令（由鼠标或启动参数写入，由 exclusive 系统执行）。
#[derive(Resource)]
struct PendingOrder(Option<Cell>);

/// 启动时一次性交给 `setup` 的东西：地形描画数据 + 出生点 + 单位模型。
/// （相机的初始状态不在这里 —— 那是 `View` 资源，`setup` 直接读它。）
#[derive(Resource)]
struct BootData {
    /// 地形几何：每格一个菱形柱（四角 = level + ramp 表）+ 它自带的 UV。
    /// 拾取 / 命中 / 渲染共用这一份 —— 贴图模式与逻辑模式没有第二份几何。
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    uvs: Vec<[f32; 2]>,
    /// 逐顶点颜色（线性空间）。只在 `logic_mesh = true` 时挂到 Mesh 上；贴图模式忽略。
    colors: Vec<[f32; 4]>,
    indices: Vec<u32>,
    /// 砖图集：上半块是主菱形槽，下半块是 extra 槽。
    atlas_rgba: Vec<u8>,
    atlas_w: u32,
    atlas_h: u32,
    /// 崖壁面单独一份：顶点 / UV / 索引，配一张自己的纹理（`Repeat` 采样）。
    /// 竖直面必须和地形顶面分开——UV 走屏幕像素坐标、要靠平铺才对得上砖图的横岩层。
    wall_positions: Vec<[f32; 3]>,
    wall_normals: Vec<[f32; 3]>,
    wall_uvs: Vec<[f32; 2]>,
    wall_indices: Vec<u32>,
    /// 崖壁纹理 `(RGBA, 宽, 高)`。没有悬崖砖时为空。
    wall_rgba: Vec<u8>,
    wall_w: u32,
    wall_h: u32,
    spawn: Vec<(i32, i32)>,
    first_order: Option<Cell>,
    /// 单位模型：车体 / 炮塔（已按 `scale_of`/`bounds` 折成世界单位）
    unit_body: Option<Mesh>,
    unit_turret: Option<Mesh>,
    /// 相机沿光轴要覆盖的半径（地形占多深）。near/far = `CAM_DIST ∓ 它`，
    /// 收窄后阴影贴图的 texel 才够细（见 `run` 里的量法与光源段）。
    cam_span: f32,
    /// 光源：`(方位角°, 仰角°, 受光面倍率, 天空/环境倍率)`
    sun: [f32; 4],
    /// 体素漫反射本色增益（1.0 = 调色板本色）
    albedo: f32,
    /// 逻辑 mesh 模式开关（`[Render] LogicMesh`）。`setup` 据此决定贴不贴图集。
    logic_mesh: bool,
    /// 是否开方向光阴影（`[Render] Shadows`）。
    shadows: bool,
}

#[derive(Resource)]
struct ShotPlan {
    path: String,
    take_at: f32,
    exit_at: f32,
}

fn parse_rc(s: &str) -> Option<Cell> {
    let (a, b) = s.split_once(',')?;
    Some(Cell::new(a.trim().parse().ok()?, b.trim().parse().ok()?))
}

/// 可调项配置文件的默认位置（工作目录下 = 工程根）。
///
/// 名字与游戏自身的 `rulesmd.ini` **同名但不同物**：那个在 `GameDir` 里、由 `costgrid`
/// 读；这个是引擎自己的旋钮表，放在工程根。**所有可调项都写这里**，别再往命令行堆。
const CFG_INI: &str = "rulesmd.ini";

/// 引擎全部可调项。优先级：**命令行 > `rulesmd.ini` > 内置默认**。
///
/// 命令行留着是为了"临时对照"——跑一张图只改一个数时不必动 ini；日常调参（亮度、
/// 光源、视口、单位）只改 `rulesmd.ini`。
#[derive(Clone)]
struct Cfg {
    map: String,
    game_dir: String,
    unit: String,
    units: i32,
    zone: String,
    cell_px: f32,
    look: Option<Cell>,
    from: Option<Cell>,
    to: Option<Cell>,
    sun_az: f32,
    sun_el: f32,
    sun_gain: f32,
    sky_gain: f32,
    albedo: f32,
    /// 地形**上色方式**开关：不贴地形图集，顶面按 `level` 分色、侧面压暗，用来看几何对不对。
    /// `[Render] LogicMesh=1/0`。**只决定贴不贴图与材质，几何一个顶点都不变。**
    logic_mesh: bool,
    /// 地形几何可调项（`[Terrain]` 段），原样转交给 `terrain::build_mesh`。
    terrain: terrain::TerrainCfg,
    /// 窗口尺寸（`[Window] Width/Height`）。截图分辨率就是它。
    win_w: u32,
    win_h: u32,
    /// 清屏色（`[Render] ClearColor`）：`magenta` / `black` / `r,g,b`（0..255）。
    /// 【定位色不是配色】没被任何面覆盖到的像素露的就是它。
    clear: [f32; 3],
    /// 方向光阴影贴图边长（`[Render] ShadowMapSize`）。越大影子越锐，显存也越大。
    shadow_size: u32,
    /// 是否开阴影（`[Render] Shadows`）。
    shadows: bool,
    /// 截图：开拍时刻与拍完停留多久（`[Shot] Delay/Hold`，秒）。
    shot_delay: f32,
    shot_hold: f32,
    /// 光标到屏幕边缘多少像素内开始平移；0 = 关掉边缘滚动（`[View] EdgeScroll`）。
    edge_scroll: f32,
    /// 平移速度（未缩放像素/秒，`[View] PanSpeed`）。
    pan_speed: f32,
    /// 模拟步进频率（`[Sim] TickHz`）。改它会改变单位运动的平滑度。
    tick_hz: f32,
    /// 用连续运动层（`[Sim] Crowd=1`）还是旧的离散重力碗（0）。
    crowd: bool,
}

impl Cfg {
    /// 内置默认（`rulesmd.ini` 缺失或某项没写时兜底）。
    fn builtin() -> Cfg {
        Cfg {
            map: String::new(),
            game_dir: String::new(),
            unit: "htnk".into(),
            units: 12,
            zone: "Track".into(),
            cell_px: 0.0,
            look: None,
            from: None,
            to: None,
            sun_az: 225.0,
            sun_el: 40.0,
            sun_gain: 0.72,
            sky_gain: 0.28,
            albedo: 1.4,
            logic_mesh: false,
            terrain: terrain::TerrainCfg::default(),
            win_w: 1280,
            win_h: 720,
            clear: [1.0, 0.0, 1.0],
            shadow_size: 4096,
            shadows: true,
            shot_delay: 2.5,
            shot_hold: 2.5,
            edge_scroll: 24.0,
            pan_speed: 760.0,
            tick_hz: 60.0,
            crowd: false,
        }
    }

    /// 读 `rulesmd.ini` 并覆盖内置默认。文件不在只提示一句，照常用内置值启动。
    fn load(path: &str) -> Cfg {
        let mut c = Cfg::builtin();
        let text = match std::fs::read(path) {
            Ok(b) => String::from_utf8_lossy(&b).into_owned(),
            Err(e) => {
                println!("[cfg] {path}: {e} —— 用内置默认值");
                return c;
            }
        };
        c.apply(&Ini::parse(&text));
        println!("[cfg] {path} 已加载");
        c
    }

    /// 把 ini 里**写了**的项覆盖上去；没写的保持原值。
    fn apply(&mut self, ini: &Ini) {
        if let Some(v) = sec_str(ini, "Launch", "Map") {
            self.map = v;
        }
        if let Some(v) = sec_str(ini, "Launch", "GameDir") {
            self.game_dir = v;
        }
        if let Some(v) = sec_str(ini, "Launch", "Unit") {
            self.unit = v;
        }
        if let Some(v) = sec_num(ini, "Launch", "Units") {
            self.units = v as i32;
        }
        if let Some(v) = sec_str(ini, "Launch", "Zone") {
            self.zone = v;
        }
        if let Some(v) = sec_num(ini, "View", "CellPx") {
            self.cell_px = v;
        }
        if let Some(v) = sec_rc(ini, "View", "Look") {
            self.look = Some(v);
        }
        if let Some(v) = sec_rc(ini, "View", "From") {
            self.from = Some(v);
        }
        if let Some(v) = sec_rc(ini, "View", "To") {
            self.to = Some(v);
        }
        if let Some(v) = sec_num(ini, "Sun", "Azimuth") {
            self.sun_az = v;
        }
        if let Some(v) = sec_num(ini, "Sun", "Elevation") {
            self.sun_el = v;
        }
        if let Some(v) = sec_num(ini, "Sun", "Gain") {
            self.sun_gain = v;
        }
        if let Some(v) = sec_num(ini, "Sun", "Sky") {
            self.sky_gain = v;
        }
        if let Some(v) = sec_num(ini, "Render", "Albedo") {
            self.albedo = v;
        }
        // 唯一的渲染开关：贴不贴地形图集（顶点色 + unlit ⇄ 图集 + 受光），
        // 并决定 debug 覆盖层注册不注册。**几何一行不变** —— `build_mesh` 没有模式参数。
        if let Some(v) = sec_num(ini, "Render", "LogicMesh") {
            self.logic_mesh = v != 0.0;
        }
        // 地形几何的可调项：全是"改一个数看一眼"的东西，一律走 ini，不重编译。
        if let Some(v) = sec_num(ini, "Terrain", "FaceInset") {
            self.terrain.face_inset = v;
        }
        if let Some(v) = sec_num(ini, "Terrain", "WallFootDrop") {
            self.terrain.wall_foot_drop = v;
        }
        if let Some(v) = sec_num(ini, "Terrain", "RampScale") {
            self.terrain.ramp_scale = v;
        }
        if let Some(v) = sec_num(ini, "Terrain", "TopAltShade") {
            self.terrain.top_alt_shade = v;
        }
        if let Some(v) = sec_num(ini, "Terrain", "MinWallHeight") {
            self.terrain.min_wall_height = v;
        }
        if let Some(v) = sec_num(ini, "Terrain", "SlopeProjected") {
            self.terrain.slope_projected = v != 0.0;
        }
        if let Some(v) = sec_num(ini, "Window", "Width") {
            self.win_w = v.max(1.0) as u32;
        }
        if let Some(v) = sec_num(ini, "Window", "Height") {
            self.win_h = v.max(1.0) as u32;
        }
        if let Some(v) = sec_str(ini, "Render", "ClearColor").and_then(|v| parse_color(&v)) {
            self.clear = v;
        }
        if let Some(v) = sec_num(ini, "Render", "ShadowMapSize") {
            self.shadow_size = v.max(1.0) as u32;
        }
        if let Some(v) = sec_num(ini, "Render", "Shadows") {
            self.shadows = v != 0.0;
        }
        if let Some(v) = sec_num(ini, "Shot", "Delay") {
            self.shot_delay = v;
        }
        if let Some(v) = sec_num(ini, "Shot", "Hold") {
            self.shot_hold = v;
        }
        if let Some(v) = sec_num(ini, "View", "EdgeScroll") {
            self.edge_scroll = v.max(0.0);
        }
        if let Some(v) = sec_num(ini, "View", "PanSpeed") {
            self.pan_speed = v;
        }
        if let Some(v) = sec_num(ini, "Sim", "TickHz") {
            self.tick_hz = v.max(1.0);
        }
        if let Some(v) = sec_num(ini, "Sim", "Crowd") {
            self.crowd = v != 0.0;
        }
    }
}

/// 颜色解析：`magenta` / `black` / `white` / `r,g,b`（0..255）。
fn parse_color(s: &str) -> Option<[f32; 3]> {
    let t = s.trim();
    match t.to_ascii_lowercase().as_str() {
        "magenta" => return Some([1.0, 0.0, 1.0]),
        "black" => return Some([0.0, 0.0, 0.0]),
        "white" => return Some([1.0, 1.0, 1.0]),
        "gray" | "grey" => return Some([0.5, 0.5, 0.5]),
        _ => {}
    }
    let p: Vec<&str> = t.split(',').collect();
    if p.len() != 3 {
        return None;
    }
    let mut out = [0.0f32; 3];
    for i in 0..3 {
        out[i] = p[i].trim().parse::<f32>().ok()? / 255.0;
    }
    Some(out)
}

fn sec_str(ini: &Ini, sec: &str, key: &str) -> Option<String> {
    ini.get(sec).and_then(|s| s.value(key)).map(|v| v.to_string())
}

fn sec_num(ini: &Ini, sec: &str, key: &str) -> Option<f32> {
    ini.get(sec).and_then(|s| s.number(key)).map(|v| v as f32)
}

fn sec_rc(ini: &Ini, sec: &str, key: &str) -> Option<Cell> {
    ini.get(sec).and_then(|s| s.value(key)).and_then(parse_rc)
}

pub fn run(args: Vec<String>) {
    // 参数分两类：位置参数（map、gameDir）与 `--key value`。
    // 摊平成表再处理，是因为 `--ini` 本身也要先被认出来才能去读配置文件。
    let mut cli: Vec<(String, String)> = Vec::new();
    let mut pos: Vec<String> = Vec::new();
    let mut i = 0usize;
    while i < args.len() {
        let a = args[i].clone();
        match a.strip_prefix("--") {
            Some(k) => {
                cli.push((k.to_string(), args.get(i + 1).cloned().unwrap_or_default()));
                i += 2;
            }
            None => {
                pos.push(a);
                i += 1;
            }
        }
    }

    // 默认值 = rulesmd.ini（不在则内置默认）；下面的命令行只做临时覆盖。
    let ini_path = cli
        .iter()
        .rev()
        .find(|(k, _)| k == "ini")
        .map(|(_, v)| v.clone())
        .unwrap_or_else(|| CFG_INI.to_string());
    let mut cfg = Cfg::load(&ini_path);

    // 位置参数覆盖 ini 的 [Launch] Map / GameDir（.map 拖到 bat 上走的就是这条路）。
    if let Some(v) = pos.first() {
        cfg.map = v.clone();
    }
    if let Some(v) = pos.get(1) {
        cfg.game_dir = v.clone();
    }

    for (k, v) in &cli {
        match k.as_str() {
            "ini" => {}
            "units" => cfg.units = v.parse().unwrap_or(cfg.units),
            "from" => cfg.from = parse_rc(v),
            "to" => cfg.to = parse_rc(v),
            "zone" => cfg.zone = v.clone(),
            "cellpx" => cfg.cell_px = v.parse().unwrap_or(cfg.cell_px),
            "look" => cfg.look = parse_rc(v),
            "unit" => cfg.unit = v.clone(),
            "sun" => {
                if let Some((a, e)) = v.split_once(',') {
                    cfg.sun_az = a.trim().parse().unwrap_or(cfg.sun_az);
                    cfg.sun_el = e.trim().parse().unwrap_or(cfg.sun_el);
                }
            }
            "sun-gain" => cfg.sun_gain = v.parse().unwrap_or(cfg.sun_gain),
            "sky" => cfg.sky_gain = v.parse().unwrap_or(cfg.sky_gain),
            "albedo" => cfg.albedo = v.parse().unwrap_or(cfg.albedo),
            other => println!("unknown arg --{other}"),
        }
    }

    if cfg.map.is_empty() || cfg.game_dir.is_empty() {
        println!("usage: ra2engine game [<map> <gameDir>] [--ini rulesmd.ini] [--units N] [--from rx,ry] [--to rx,ry]");
        println!("       可调项都在 rulesmd.ini 里；map / gameDir 也可以只写在它的 [Launch] 段。");
        return;
    }

    let map_path = cfg.map.clone();
    let game_dir = cfg.game_dir.clone();
    let units_n = cfg.units;
    let from = cfg.from;
    let to = cfg.to;
    let zone = cfg.zone.clone();
    let cell_px = cfg.cell_px;
    let look = cfg.look;
    let unit_name = cfg.unit.clone();
    let albedo = cfg.albedo;
    let logic_mesh = cfg.logic_mesh;
    let terrain_cfg = cfg.terrain;
    let (sun_az, sun_el, sun_gain, sky_gain) =
        (cfg.sun_az, cfg.sun_el, cfg.sun_gain, cfg.sky_gain);

    // --- 建场景：地图 → theater → 砖 → cost grid / 地面高度场 -------------------------
    let bytes = match std::fs::read(&map_path) {
        Ok(b) => b,
        Err(e) => {
            println!("{map_path}: {e}");
            return;
        }
    };
    let map = match mapfile::parse_map(&bytes) {
        Ok(m) => m,
        Err(e) => {
            println!("{map_path}: {e}");
            return;
        }
    };
    let Some(mut th) = theater::theater_info(&map.theater) else {
        println!("unknown theater '{}'", map.theater);
        return;
    };
    let t0 = std::time::Instant::now();
    let mut idx = mix::AssetIndex::new(&game_dir);
    if let Err(e) = idx.build("", false) {
        println!("index failed: {e}");
        return;
    }
    let th_ini = match idx.read(&th.ini_name) {
        Ok(b) => Ini::parse(&String::from_utf8_lossy(&b)),
        Err(e) => {
            println!("cannot read {}: {e}", th.ini_name);
            return;
        }
    };
    if let Err(e) = theater::parse_theater(&th_ini, &mut th) {
        println!("theater parse: {e}");
        return;
    }
    let numbering = theater::build_tile_numbering(&th);
    let sets = costgrid::build_set_images(&idx, &th);
    let tb = costgrid::build_terrain_cells(&map, &th, &numbering, &sets, false);
    let rules = costgrid::read_rules_ini(&idx, &["rulesmd.ini", "rules.ini"]);
    let art = costgrid::read_rules_ini(&idx, &["artmd.ini", "art.ini"]);
    let (raw_grid, stats) = costgrid::build_cost_grid(&map, &rules, &art, &tb, &zone);
    let (gf, ramped) = world::build_ground_field(&map, &tb);
    let footprint = Footprint::default();
    let nav_grid = nav::erode_grid(&raw_grid, &footprint);
    println!(
        "scene  {}x{} theater={} cells={} blocked={} plain={} ramped={} ({:.1}s)",
        raw_grid.w,
        raw_grid.h,
        th.name,
        tb.cells.len(),
        stats.blocked,
        stats.plain,
        ramped,
        t0.elapsed().as_secs_f32()
    );
    if tb.cells.is_empty() {
        println!("no terrain cells to build a mesh from");
        return;
    }

    // --- 19 级 iso 调色板（level 0..18）：`.pal` 的六位值经本图光照乘法重建 -----------
    let raw_iso = match idx.read(&th.palettes.iso) {
        Ok(b) => b,
        Err(e) => {
            println!("cannot read iso palette {}: {e}", th.palettes.iso);
            return;
        }
    };
    let iso_level: Vec<Vec<u8>> = (0..=18)
        .map(|z| {
            lighting::recalculate_palette(
                &raw_iso,
                lighting::lighting_mults(&map.lighting, z, true),
                false,
            )
        })
        .collect();

    // --- 地形几何：唯一来源 `terrain::build_mesh` ---------------------------------------
    // 世界空间的菱形柱（四角 = level + ramp 表），拾取 / 命中 / 渲染 / 地面高度全用它。
    // 旧的 CPU 逐像素烘图（`terrain_bake`）已整个删除 —— 那是同一件事的第二种做法。
    let (gw, gh) = (raw_grid.w, raw_grid.h);
    let t1 = std::time::Instant::now();
    let atlas = terrain::build_atlas(&tb, &iso_level, &raw_iso);
    let mesh = terrain::build_mesh(&tb, &atlas, gw, gh, &terrain_cfg);

    // 几何只有一份，跟 LogicMesh 无关；这里打出来的只是**上色/材质**走哪条路。
    println!(
        "terrain draw  logic_mesh={} (geometry identical either way)",
        cfg.logic_mesh
    );
    // 高度范围（level 单位）：给相机构图留边、也用来核对斜坡是否真的抬起来了。
    let (mut hi_lv, mut lo_lv) = (f32::MIN, f32::MAX);
    for c in &tb.cells {
        let lv = c.level as f32;
        lo_lv = lo_lv.min(lv);
        hi_lv = hi_lv.max(lv);
    }
    println!(
        "terrain {} verts / {} tris ({:.1}s)",
        mesh.positions.len(),
        mesh.triangle_count(),
        t1.elapsed().as_secs_f32()
    );
    println!(
        "levels {lo_lv:.0}..{hi_lv:.0} -> world Y {:.2}..{:.2}",
        lo_lv * terrain::LEVEL_HEIGHT,
        hi_lv * terrain::LEVEL_HEIGHT
    );

    // --- 单位模型（VXL）：车体 + 炮塔 --------------------------------------------------
    // 用户口径（2026-10-16 修正）：**VPL 作废**。体素只做「索引 → 真 RGB」这一步查表
    // （单位调色板本体色，不烘任何光照），明暗由场景里的**真光源**现算 —— 这样光源一改，
    // 同一辆车的明暗立刻跟着变；C++ 那套「按朝向选光照页 + 烘 32 个朝向」是在没有实时光
    // 的年代用调色板凑出来的，不需要在现代引擎里复现。
    let unit_pal = {
        let raw = idx.read(&th.palettes.unit).unwrap_or_default();
        lighting::expand_palette(&raw)
    };
    let load_vxl = |suffix: &str| -> Option<vxl::Vxl> {
        let name = format!("{unit_name}{suffix}.vxl");
        let bytes = idx
            .read(&name)
            .ok()
            .or_else(|| std::fs::read(format!("assets/{name}")).ok())?;
        vxl::parse_vxl(&bytes)
    };
    // 部件 + 各自的 HVA（`<stem>.hva`，取 frame 0）。缺 HVA 就是单位阵 —— 与参考实现一致。
    let load_part = |suffix: &str| -> (Option<vxl::Vxl>, Option<hva::Hva>) {
        let hname = format!("{unit_name}{suffix}.hva");
        let hbytes = idx
            .read(&hname)
            .ok()
            .or_else(|| std::fs::read(format!("assets/{hname}")).ok());
        (
            load_vxl(suffix),
            hbytes.and_then(|b| hva::parse_hva(&b).ok()),
        )
    };
    // 组装一个部件组：把各 stem 的 sections 拼起来，HVA 也按同样顺序合并成一张全局表
    // （照 `appendVxlPart` 的追加方式，`wwread.cpp:243-257`）。
    let build = |parts: &[(Option<vxl::Vxl>, Option<hva::Hva>)]| -> Option<(Mesh, usize)> {
        let mut secs: Vec<&vxl::VxlSection> = Vec::new();
        let mut merged = hva::Hva::default();
        for (v, h) in parts {
            let n = v.as_ref().map(|v| v.sections.len()).unwrap_or(0);
            merged.append_part(h.as_ref(), n);
            if let Some(v) = v {
                secs.extend(v.sections.iter());
            }
        }
        if secs.is_empty() {
            return None;
        }
        let refs: Vec<(&vxl::VxlSection, [f32; 3])> =
            secs.iter().map(|s| (*s, [0.0f32, 0.0, 0.0])).collect();
        let hvas = vec![Some(&merged); refs.len()];
        let (m, st) = voxel_mesh::build_mesh(&unit_pal, &refs, &hvas, true, albedo);
        Some((m, st.faces))
    };
    // 车体 = `<name>.vxl`；炮塔 = `<name>tur.vxl` + `<name>barl.vxl`（炮管跟着炮塔转）。
    let unit_body = build(&[load_part("")]);
    let unit_turret = build(&[load_part("tur"), load_part("barl")]);
    match (&unit_body, &unit_turret) {
        (Some((_, bf)), Some((_, tf))) => {
            println!("unit   {unit_name}: hull {bf} faces, turret {tf} faces")
        }
        (Some((_, bf)), None) => println!("unit   {unit_name}: hull {bf} faces (no turret)"),
        _ => println!("unit   {unit_name}: not found -> placeholder boxes"),
    }

    // 自检 + 构图：地形在世界里实际占多大。相机必须按**这个**算，不能按格网尺寸 ——
    // 可寻路格网（`raw_grid`）比地图本身大一圈，照它构图会把地形缩成画面中间一小块。
    let (mut bx0, mut bx1) = (f32::MAX, f32::MIN);
    let (mut by0, mut by1) = (f32::MAX, f32::MIN);
    let (mut bz0, mut bz1) = (f32::MAX, f32::MIN);
    for p in &mesh.positions {
        bx0 = bx0.min(p[0]);
        bx1 = bx1.max(p[0]);
        by0 = by0.min(p[1]);
        by1 = by1.max(p[1]);
        bz0 = bz0.min(p[2]);
        bz1 = bz1.max(p[2]);
    }
    let (mut rx0, mut rx1) = (i32::MAX, i32::MIN);
    let (mut ry0, mut ry1) = (i32::MAX, i32::MIN);
    for c in &tb.cells {
        rx0 = rx0.min(c.rx);
        rx1 = rx1.max(c.rx);
        ry0 = ry0.min(c.ry);
        ry1 = ry1.max(c.ry);
    }

    // 逐顶点投到相机的屏幕基上取极值。地形的有效区域是**斜置的**（`rx,ry` 上的菱形），
    // 拿包围盒的 (Δx+Δz) 去估会算大一圈，地形就被缩到画面中间一小块。
    let (mut sh0, mut sh1) = (f32::MAX, f32::MIN);
    let (mut sv0, mut sv1) = (f32::MAX, f32::MIN);
    // 俯视那一套：屏幕右 = 世界 +X、屏幕上 = 世界 −Z。同一遍循环里量完，省一次遍历。
    let (mut th0, mut th1) = (f32::MAX, f32::MIN);
    let (mut tv0, mut tv1) = (f32::MAX, f32::MIN);
    // 沿**光轴**的跨度。正交相机的 near/far 只决定裁剪，不影响投影大小，但**阴影贴图**
    // 的采样密度是被视锥切片的包围球直径除出来的（`bevy_light/src/cascade.rs:280`）——
    // near/far 留得越宽，影子越糊。所以这里量一下地形到底占多深。
    let (mut sd0, mut sd1) = (f32::MAX, f32::MIN);
    let cd = Vec3::from_array(CAM_DIR).normalize();
    for p in &mesh.positions {
        let h = p[0] * CAM_RIGHT[0] + p[1] * CAM_RIGHT[1] + p[2] * CAM_RIGHT[2];
        let v = p[0] * CAM_UP[0] + p[1] * CAM_UP[1] + p[2] * CAM_UP[2];
        let d = p[0] * cd.x + p[1] * cd.y + p[2] * cd.z;
        sh0 = sh0.min(h);
        sh1 = sh1.max(h);
        sv0 = sv0.min(v);
        sv1 = sv1.max(v);
        th0 = th0.min(p[0]);
        th1 = th1.max(p[0]);
        tv0 = tv0.min(-p[2]);
        tv1 = tv1.max(-p[2]);
        sd0 = sd0.min(d);
        sd1 = sd1.max(d);
    }
    let mut min_width = (sh1 - sh0) * 1.06 + 4.0;
    let mut min_height = (sv1 - sv0) * 1.06 + 4.0;
    let mut cam_at = [(bx0 + bx1) * 0.5, (by0 + by1) * 0.5, (bz0 + bz1) * 0.5];
    // 【1 格 = N 像素】RA2 原版就是 1 格 60 像素宽；把视口定成同一比例，画面才和参考
    // 实现的整图渲染一一对应 —— 否则整幅地图被塞进窗口，砖的细节全糊掉、没法比对。
    // 窗口实际尺寸在 setup 里才有，所以这里只记 N，换算留给 setup。
    if cell_px > 0.0 {
        min_width = cell_px;
        min_height = cell_px;
    }
    if let Some(lc) = look {
        cam_at[0] = lc.x as f32;
        cam_at[2] = lc.y as f32;
    }
    // 相机沿光轴要覆盖的半径。只影响 near/far（裁剪）与阴影精度，不改投影尺寸。
    let cam_span = {
        let c = Vec3::from_array(cam_at).dot(cd);
        ((sd1 - c).max(c - sd0)).max((sd1 - sd0) * 0.5) * 1.2 + 8.0
    };
    println!("cells  rx {rx0}..{rx1} ry {ry0}..{ry1}");
    println!("world  x {bx0:.1}..{bx1:.1} z {bz0:.1}..{bz1:.1} y {by0:.2}..{by1:.2}");
    println!(
        "screen {:.1}x{:.1} world units -> view {min_width:.1}x{min_height:.1}",
        sh1 - sh0,
        sv1 - sv0
    );

    // --- 出生点：在 from 附近用编队格铺开 --------------------------------------------
    let start_default = Cell::new(gw / 4 + 2, gh / 4 + 2);
    let Some(start)=play::nearest_usable(&nav_grid,from.unwrap_or(start_default)) else {
        eprintln!("no legal spawn cell");return;
    };
    let mut spawn_cells = nav::formation_cells(&nav_grid, &footprint, start, units_n);
    if spawn_cells.is_empty() {
        spawn_cells.push(start);
    }
    let spawn: Vec<(i32, i32)> = spawn_cells.iter().map(|c| (c.x, c.y)).collect();
    let first_order = to.or_else(|| Some(Cell::new(gw * 3 / 4, gh * 3 / 4)));

    // --- App ------------------------------------------------------------------------
    let mut app = App::new();
    // 窗口尺寸走 ini：截图用的分辨率就是它。
    app.add_plugins(DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "ra2engine".into(),
            resolution: (cfg.win_w, cfg.win_h).into(),
            ..default()
        }),
        ..default()
    }));
    // 【定位色，不是配色】背景 = 品红（255,0,255）。**凡是没有东西覆盖到的像素都露成洋红** ——
    // 地形是 Mask 材质的平贴砖（砖位图只有菱形是实体，矩形四角 alpha=0 被丢弃），只要相邻砖
    // 没盖住、或图集某槽没烘进去，露出来的就是这层底色。于是"地块之间的黑缝"会直接被染成
    // 洋红：一眼分清是**漏背景**，还是**砖像素本身就是黑**。排查完改回黑。
    // （bevy 不给 ClearColor 时默认就是纯黑 —— 正是要排查的那个"黑"。）
    // 设环境变量 RA2_CLEAR_BLACK=1 就换成纯黑再跑一次 —— 洋红是「洞的标记」，
    // 要分清某块异常到底是几何漏了（换黑就变黑）还是贴图采到了 alpha=0（仍然是洋红）。
    let mut clear = cfg.clear;
    if std::env::var("RA2_CLEAR_BLACK").is_ok() {
        clear = [0.0, 0.0, 0.0];
    }
    app.insert_resource(ClearColor(Color::srgb(clear[0], clear[1], clear[2])));
    let runtime = crate::simulation::Runtime::from_hz(cfg.tick_hz as f64, 0);
    app.insert_resource(Time::<Fixed>::from_duration(runtime.fixed_step()));
    app.insert_resource(SimEnv {
        ground: gf,
        nav: nav_grid,
        footprint,
        occupied: Vec::new(),
        tick: 0,
        runtime,
        // 连续运动层（路线 C）。想退回旧的离散重力碗做对照，把这里改成 false。
        // ⚠️ 暂时回到旧的离散重力碗：新的连续运动层还没在真实地图上验过，
        // 已知问题：脚本指令下单位可能不动。改 true 就切回新系统。
        use_crowd: cfg.crowd,
        pressure: crate::crowd::Pressure::default(),
        crowd_params: crate::crowd::Params::default(),
        since_pressure: 0,
    });
    app.insert_resource(Selection(Vec::new()));
    // 阴影贴图：默认 2048（`directional_light.rs:204`）。我们的正交视锥切片包围球直径
    // 约 250 世界单位，2048 下 texel ≈ 0.12 世界单位 ≈ 5 屏幕像素，影子边缘会糊成一团；
    // 提到 4096 后约 2.5 像素，够锐。
    app.insert_resource(DirectionalLightShadowMap {
        size: cfg.shadow_size as usize,
    });
    app.insert_resource(PendingOrder(first_order));
    // 【按格子坐标对准镜头】`RA2_SHOT_CELL=<rx>,<ry>` 把初始看向点挪到这一格的中心
    // （用它自己的 `level` 算高度），`RA2_SHOT_ZOOM=<倍数>` 再放大。
    // 以前查局部问题只能在整图里猜坐标、改一处看一眼来回好几轮；现在直接咬住出问题的
    // 那一格截图，坐标是从地图里读的，不会盯错地方。
    let mut zoom0 = 1.0f32;
    if let Ok(c) = std::env::var("RA2_SHOT_CELL") {
        if let Some((a, b)) = c.split_once(',') {
            if let (Ok(rx), Ok(ry)) = (a.trim().parse::<i32>(), b.trim().parse::<i32>()) {
                let lv = tb
                    .cells
                    .iter()
                    .find(|t| t.rx == rx && t.ry == ry)
                    .map(|t| t.level)
                    .unwrap_or(0);
                let p = terrain::cell_center_at(rx, ry, lv, 60, 30);
                cam_at = [p[0], cam_at[1], p[2]];
                println!(
                    "shot cell ({rx},{ry}) level {lv} -> look ({:.2}, {:.2}, {:.2})",
                    cam_at[0], cam_at[1], cam_at[2]
                );
            } else {
                println!("RA2_SHOT_CELL: expect <rx>,<ry>, got '{c}'");
            }
        }
    }
    if let Ok(z) = std::env::var("RA2_SHOT_ZOOM") {
        if let Ok(v) = z.parse::<f32>() {
            zoom0 = v;
        }
    }

    app.insert_resource(View {
        zoom: zoom0,
        cell_px,
        look: Vec3::from_array(cam_at),
        fit: Vec2::new(min_width, min_height),
        map_h: Vec2::new(sh0, sh1),
        map_v: Vec2::new(sv0, sv1),
        map_h_top: Vec2::new(th0, th1),
        map_v_top: Vec2::new(tv0, tv1),
        top_down: false,
        edge_scroll: cfg.edge_scroll,
        pan_speed: cfg.pan_speed,
    });
    app.insert_resource(BootData {
        positions: mesh.positions,
        normals: mesh.normals,
        uvs: mesh.uvs,
        colors: mesh.colors,
        indices: mesh.indices,
        atlas_rgba: atlas.rgba,
        atlas_w: atlas.w,
        atlas_h: atlas.h,
        wall_positions: mesh.wall_positions,
        wall_normals: mesh.wall_normals,
        wall_uvs: mesh.wall_uvs,
        wall_indices: mesh.wall_indices,
        wall_rgba: atlas
            .wall
            .as_ref()
            .map(|(d, _, _)| d.clone())
            .unwrap_or_default(),
        wall_w: atlas.wall.as_ref().map(|(_, w, _)| *w).unwrap_or(1),
        wall_h: atlas.wall.as_ref().map(|(_, _, h)| *h).unwrap_or(1),
        spawn,
        first_order,
        unit_body: unit_body.map(|(m, _)| m),
        unit_turret: unit_turret.map(|(m, _)| m),
        cam_span,
        sun: [sun_az, sun_el, sun_gain, sky_gain],
        albedo,
        logic_mesh,
        shadows: cfg.shadows,
    });
    // --- debug 覆盖层（只在逻辑模式里存在）------------------------------------------
    // 面板要显示的字段在这里一次性摘出来：只存标量，不搬 `pixels` / `zbuf`（那是几十 MB 位图）。
    // `tile_id` 里只编了 set 序号，set 名与 TMP 文件名都得从 theater 的 TileSet 表取。
    app.insert_resource(debugview::DebugCells::from_cells(&tb.cells, &th));
    app.insert_resource(debugview::LogicMode(logic_mesh));
    if logic_mesh {
        println!(
            "debug cells {}  theater {} ({})",
            tb.cells.len(),
            th.name,
            th.extension
        );
    }
    app.add_systems(Startup, (setup, select_all_initial).chain());
    // 两套运动系统二选一：`SimEnv::use_crowd`（默认 true = 连续运动层）
    app.add_systems(
        FixedUpdate,
        world::update_sim_crowd.run_if(|env: Res<SimEnv>| env.use_crowd),
    );
    app.add_systems(
        FixedUpdate,
        world::update_sim.run_if(|env: Res<SimEnv>| !env.use_crowd),
    );
    // 指向格每帧只算一次，先于用它的系统（下单）跑。
    app.init_resource::<CursorCell>();
    app.add_systems(Update, update_cursor_cell.before(select_input));
    app.add_systems(
        Update,
        (update_view, sync_units, select_input, auto_order, auto_move),
    );
    // debug 覆盖层只在逻辑模式下注册（贴图模式一行都不跑）。
    // 【两种模式都注册】hover 打的是「鼠标指着哪一格 + 那格的 TMP 名」，跟贴不贴图无关。
    // 之前只在逻辑模式挂，结果贴图模式下排查问题只能靠猜坐标 —— 这一条就是为了能在
    // 真实贴图画面里直接读出问题格。面板本身仍然只在逻辑模式显示（`LogicMode` 控制）。
    app.init_resource::<debugview::MarkerMeshes>();
    app.add_systems(Startup, debugview::setup.after(setup));
    app.add_systems(Update, (debugview::hover, debugview::sync_label));
    app.add_observer(debugview::on_toggle);
    if let Ok(p) = std::env::var("RA2_SHOT") {
        // `RA2_SHOT_AT=<秒>`：什么时候截（默认 2.5s）。排查"跑一会儿才出现"的问题时调它。
        let take_at = std::env::var("RA2_SHOT_AT")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(cfg.shot_delay);
        app.insert_resource(ShotPlan {
            path: p,
            take_at,
            exit_at: take_at + cfg.shot_hold,
        });
        app.add_systems(Update, auto_shot);
    }
    app.run();
}

fn setup(
    mut commands: Commands,
    boot: Res<BootData>,
    view_state: Res<View>,
    windows: Query<&Window, With<PrimaryWindow>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut images: ResMut<Assets<Image>>,
    mut pending: ResMut<PendingOrder>,
) {
    // --- 地形渲染层：一个 mesh（每格一个菱形柱）+ 砖图集 ------------------------------
    // 几何不在这里算：`terrain::build_mesh` 是唯一来源，这里只负责把图集接上去。
    // 贴图模式与逻辑模式吃同一份 mesh，区别只在挂不挂顶点色、用不用 `unlit`。
    //
    // 菱形几何的意义：砖的位图是 60×30 矩形，只有中间菱形是有效像素。用矩形当几何时，
    // 三角形把整块矩形的位置都占了、深度也写了，alpha 测试又把菱形外的像素丢掉 ——
    // 那些像素谁都没写，透出清屏色，就是满屏砖缝。菱形几何让几何边界 = 纹理有效边界。
    let mut image = Image::new(
        Extent3d {
            width: boot.atlas_w,
            height: boot.atlas_h,
            depth_or_array_layers: 1,
        },
        TextureDimension::D2,
        boot.atlas_rgba.clone(),
        TextureFormat::Rgba8UnormSrgb,
        RenderAssetUsages::default(),
    );
    // 像素画：最近邻，并**把 `lod_max_clamp` 压成 0 关掉 mipmap** —— 图集里相邻槽是
    // 完全不同的砖，一旦切到 mip 级别就会跨槽混色，每块砖四周渗出一圈黑边。
    let mut sampler = ImageSamplerDescriptor::nearest();
    sampler.lod_max_clamp = 0.0;
    image.sampler = ImageSampler::Descriptor(sampler);
    let atlas_tex = images.add(image);

    let mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, boot.positions.clone())
    .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, boot.uvs.clone())
    // 法线按真实几何给（平地全部朝上，坡格朝坡的上方偏）。以前写死 (0,1,0)，
    // 斜面在实时光照里就会假平 —— N·L 不随坡度变，坡看起来像块贴纸。
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, boot.normals.clone())
    .with_inserted_indices(Indices::U32(boot.indices.clone()));
    // 【顶点色只在逻辑模式挂】`Mesh::ATTRIBUTE_COLOR` 存在时 bevy_pbr 会推 `VERTEX_COLORS`
    // 这个 shader def（`mesh.rs:3464` / `prepass/mod.rs:570`），把顶点色乘进 base color。
    // 贴图模式下挂上去的话，按 level 分出来的绿/粉色会压到砖图上，整片地形被染色。
    let mesh = if boot.logic_mesh {
        mesh.with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, boot.colors.clone())
    } else {
        mesh
    };

    // 逻辑模式：不贴图集、**不走光照**，顶点色原样直出。
    // 【为什么 unlit】逻辑模式要读的是地图数据本身，不是"阳光下长什么样"：顶点色已经按
    // 每个顶点自己的高度给了色阶、侧面压了暗，明暗关系全部写在数据里；再过一遍光源就是
    // 在数据之上叠一层跟地图无关的东西（同一个坡，太阳一换角度就变样），反而看不出倾斜。
    // `unlit` 在 `pbr.wesl:83` 是 `out.color = base_color` 直出，而 `base_color` 已经被
    // `VERTEX_COLORS` 分支（`pbr_fragment.wesl:52`）换成顶点色 —— 所以画面 = 顶点色。
    let material = if boot.logic_mesh {
        materials.add(StandardMaterial {
            base_color: Color::WHITE,
            unlit: true,
            cull_mode: None,
            ..default()
        })
    } else {
        materials.add(StandardMaterial {
            base_color_texture: Some(atlas_tex),
            base_color: Color::WHITE,
            // 【贴图模式不走光照】颜色 = 图集里那块砖的原色。走光照时，法线稍微朝内
            // 一点（竖墙的绕序在各处并不统一）`N·L` 就变负，整面墙被压成纯黑 ——
            // 那不是 UV 也不是调色板，是光照。原版是手绘 2D，砖图自带明暗，不需要再来一遍。
            unlit: true,
            reflectance: 0.0,
            metallic: 0.0,
            perceptual_roughness: 1.0,
            cull_mode: None,
            // 图集槽里没写到的格子是 RGBA(255,0,255,0)（洋红定位色）。Mask 丢掉它们。
            alpha_mode: AlphaMode::Mask(0.5),
            ..default()
        })
    };
    // 【地形只接收阴影，不投阴影】`NotShadowCaster` 让地形不进阴影贴图的剔除列表
    // （`bevy_light/src/lib.rs:365` 的 `Without<NotShadowCaster>`），但**接收**照旧。
    // 地形若参与投影，它自己会当遮挡物，平原被自己上方的砖块盖成暗块。RA2 也一样：
    // 只有单位/建筑投影，地形是纯受影面。
    commands.spawn((
        Mesh3d(meshes.add(mesh)),
        MeshMaterial3d(material),
        NotShadowCaster,
    ));

    // --- 崖壁面：独立网格 + 独立纹理（Repeat 平铺）----------------------------------
    //
    // 竖直面不能和地形顶面共用一张图集：它的 UV 按**屏幕像素坐标**走，一格正好一个纹理宽、
    // 一层正好 15px 高，超出 0..1 由采样器平铺 —— 只有这样砖图上画的水平岩层才是水平的。
    // 图集槽只能取一次 0..1，横纹必被格边的 30° 斜度剪成斜条纹。
    if !boot.wall_indices.is_empty() && !boot.wall_rgba.is_empty() {
        let mut wimg = Image::new(
            Extent3d {
                width: boot.wall_w,
                height: boot.wall_h,
                depth_or_array_layers: 1,
            },
            TextureDimension::D2,
            boot.wall_rgba.clone(),
            TextureFormat::Rgba8UnormSrgb,
            RenderAssetUsages::default(),
        );
        // 【Repeat + 最近邻】Repeat 是这道菜的关键：UV 会超出 0..1（一格一格累加），
        // 必须平铺。最近邻保证原版那种锐利像素，不被插值糊掉。
        let mut ws = ImageSamplerDescriptor::nearest();
        ws.address_mode_u = bevy::image::ImageAddressMode::Repeat;
        ws.address_mode_v = bevy::image::ImageAddressMode::Repeat;
        ws.lod_max_clamp = 0.0;
        wimg.sampler = ImageSampler::Descriptor(ws);
        let wall_tex = images.add(wimg);
        let wmesh = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        )
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, boot.wall_positions.clone())
        .with_inserted_attribute(Mesh::ATTRIBUTE_UV_0, boot.wall_uvs.clone())
        .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, boot.wall_normals.clone())
        .with_inserted_indices(Indices::U32(boot.wall_indices.clone()));
        let wmat = materials.add(StandardMaterial {
            base_color_texture: Some(wall_tex),
            base_color: Color::WHITE,
            unlit: true,
            reflectance: 0.0,
            metallic: 0.0,
            perceptual_roughness: 1.0,
            cull_mode: None,
            // 【不透明，不要 Mask】这张纹理整幅都是岩石，没有需要挖掉的区域。用 Mask 时
            // 只要纹理里有一行 alpha 偏低（裁出来的岩壁段靠近上下的过渡行），那一整层
            // 墙面就被整条丢掉 —— 表现是一堵墙上每隔 15px（正好一层）漏一条清屏色的缝。
            alpha_mode: AlphaMode::Opaque,
            ..default()
        });
        commands.spawn((
            Mesh3d(meshes.add(wmesh)),
            MeshMaterial3d(wmat),
            NotShadowCaster,
        ));
        println!(
            "wall mesh: {} verts / {} tris, texture {}x{}",
            boot.wall_positions.len(),
            boot.wall_indices.len() / 3,
            boot.wall_w,
            boot.wall_h
        );
    }

    // --- 相机：2:1 等距正交；缩放 / 平移由 `update_view` 每帧写 ----------------------
    // 「1 格 = N 像素」的换算在 `View::px_per_unit`：世界 +X 一格在屏幕上是**菱形包围盒宽**
    //   = |CAM_RIGHT·X̂| + |CAM_RIGHT·Ẑ| = 0.7071 + 0.7071 = 1.4142（世界单位），
    // 所以"每世界单位多少像素" k = N / 1.4142；而正交视口的 k = 窗口宽 / min_w。
    let win = match windows.single() {
        Ok(w) => Vec2::new(w.width(), w.height()),
        Err(_) => Vec2::new(1280.0, 720.0),
    };
    let s = view_state.size(win);
    let (dir, _, _, up_hint) = cam_basis(view_state.top_down);
    // near/far 贴住地形：正交投影下它们不影响画面大小，只决定裁剪与**阴影贴图**的
    // 采样密度（`cascade.rs:280` 拿视锥切片的包围球直径去除贴图边长）。原来是
    // `1 .. 2*CAM_DIST`（深度 4000），影子 texel 会被摊到约 2 个世界单位，糊成一片。
    let cam_near = (boot_cam_dist() - boot.cam_span).max(1.0);
    let cam_far = boot_cam_dist() + boot.cam_span;
    commands.spawn((
        Camera3d::default(),
        // 【必须关 MSAA】地形是「一块砖一个菱形」的像素画：相邻菱形的屏幕投影只共享一条
        // 数学直线，而砖本身的有效像素边界是逐行 4px 的台阶。开着 4x MSAA 时，边界像素
        // 只被部分采样覆盖 → 该像素拿到「半个砖 + 半个背景」→ 每格四周一圈淡粉色描边
        // （背景是洋红定位色）。FA2sp 全程整数像素、没有 alpha，所以它没有这个问题。
        // 像素画本来也不该抗锯齿，关掉之后边缘就是 RA2 那种硬边。
        Msaa::Off,
        // 【必须 Linear】C++ 是「调色板颜色直出」：查表得到的 8 位色原样写进像素。Bevy 默认
        // 的 `Tonemapping::TonyMcMapface` 是一条 film 曲线（LUT），会把纯红 (1,0,0) 压成暗砖红、
        // 把中灰整体提亮 —— 所以拿它对照 C++ 永远对不上色。`Linear` 是**恒等**分支
        // （`tonemapping.wesl` 的 `TONEMAP_METHOD_LINEAR`：只乘 `2^exposure`，不做任何曲线），
        // 于是「线性场景色 × 曝光 → sRGB 编码」正好是 C++ 那条直出链，颜色一比一。
        Tonemapping::Linear,
        Projection::from(OrthographicProjection {
            scaling_mode: ScalingMode::AutoMin {
                min_width: s.x,
                min_height: s.y,
            },
            near: cam_near,
            far: cam_far,
            ..OrthographicProjection::default_3d()
        }),
        Transform::from_translation(view_state.look + dir * boot_cam_dist())
            .looking_at(view_state.look, up_hint),
    ));

    // --- 单位：VXL 车体 + 炮塔（现代渲染 = GPU 几何）----------------------------------
    // 没读到 VXL 时退回一个红方块，至少还看得见车在哪。
    let cube = meshes.add(Cuboid::new(0.6, 0.3, 0.4));
    let placeholder = materials.add(StandardMaterial {
        base_color: Color::srgb(0.92, 0.25, 0.22),
        unlit: true,
        ..default()
    });
    // VXL 的颜色全在**顶点色**里（体素色索引查单位调色板的真 RGB，线性空间），材质本体白色。
    // 【走 PBR —— 不再 unlit】顶点色只是**本色**，明暗由场景里的真光源现算：光源一转，
    // 同一辆车的明暗立刻跟着变。两条必须的约束：
    //  · `reflectance = 0`：彻底关掉镜面项。默认 0.5 会给纯红装甲的 G/B 打进白高光
    //    （实测：纯红 `#FF0000` 被污染成 `#C42020`）。
    //  · `metallic = 0` + `perceptual_roughness = 1`：哑光军械漆，不要 PBR 的高光条纹。
    // 地面砖也一并转了 PBR（同样关镜面）—— 因为 unlit 材质会**跳过整个光照项**
    // （`pbr.wesl:83`），连阴影都不参与，单位投下来的影子无处可落。
    let unit_mat = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        reflectance: 0.0,
        metallic: 0.0,
        perceptual_roughness: 1.0,
        cull_mode: None,
        ..default()
    });
    let body_handle = boot.unit_body.as_ref().map(|m| meshes.add(m.clone()));
    let tur_handle = boot.unit_turret.as_ref().map(|m| meshes.add(m.clone()));
    for (n, (rx, ry)) in boot.spawn.iter().enumerate() {
        let unit = Unit {
            idx: n as i32,
            cell_x: *rx as f32,
            cell_y: *ry as f32,
            ..Default::default()
        };
        let part = UnitPart {
            idx: n as i32,
            turret: false,
        };
        match &body_handle {
            Some(h) => {
                commands.spawn((
                    unit,
                    part,
                    Mesh3d(h.clone()),
                    MeshMaterial3d(unit_mat.clone()),
                    Transform::default(),
                ));
            }
            None => {
                commands.spawn((
                    unit,
                    part,
                    Mesh3d(cube.clone()),
                    MeshMaterial3d(placeholder.clone()),
                    Transform::default(),
                ));
            }
        }
        if let Some(h) = &tur_handle {
            commands.spawn((
                UnitPart {
                    idx: n as i32,
                    turret: true,
                },
                Mesh3d(h.clone()),
                MeshMaterial3d(unit_mat.clone()),
                Transform::default(),
            ));
        }
    }

    // --- 光源：一组可调的太阳 + 环境光 ------------------------------------------------
    // VXL 和地形的明暗都由它现算 —— 改 `--sun az,el`（或运行时调）同一辆车的受光面立刻跟着变。
    // 这正是「RA2 的光源只是一个特殊值」的落地：C++ 那套 VPL 页相当于「某个固定光源下的
    // 结果」，这里换成真光源，方向/强度/颜色都可调。
    //
    // 强度标定的目标：**法线朝上的面**显示 = 调色板本色（地面砖因此和 unlit 时一样亮）。
    // 两条链（都在 bevy 源码里核过）：
    //  · 方向光：uniform = `color * illuminance`（`bevy_pbr/src/render/light.rs:1828`），
    //    漫反射项带一个 1/π（`pbr_lighting.wesl` 的 `Fd_Burley`）。朝上的面 N·L = sin(仰角)，
    //    所以它拿到 `illuminance / π × sin(el) × EXPOSURE` 倍本色。
    //  · 环境光：uniform = `color * brightness`（`GlobalAmbientLight::brightness`），
    //    着色器里**不除 π**（`pbr_ambient.wesl`），倍率 = `brightness × EXPOSURE`。
    // `EXPOSURE` = 相机没挂 `Exposure` 组件时 tonemapping 用的
    // `Exposure::default().exposure()`（`bevy_render/src/view/mod.rs:1286`）= `exp2(-9.7)/1.2`。
    // 令两者之和 = `sun_gain + sky_gain`（默认 1.0）就得到「朝上的面 = 本色」。
    const EXPOSURE: f32 = 1.0019e-3;
    let (az, el) = (boot.sun[0].to_radians(), boot.sun[1].to_radians());
    // 从场景指向太阳的单位向量：az 绕世界 +Y 轴，0 = 从 +Z 一侧照来；el 是仰角。
    let to_sun = Vec3::new(el.cos() * az.sin(), el.sin(), el.cos() * az.cos());
    let illum = boot.sun[2] * std::f32::consts::PI / (EXPOSURE * el.sin().max(0.05));
    commands.spawn((
        DirectionalLight {
            illuminance: illum,
            // 默认是**关**的（`directional_light.rs:152 shadow_maps_enabled: false`）。
            shadow_maps_enabled: boot.shadows,
            ..default()
        },
        // 方向光沿自己的 -Z 照射（bevy 惯例），所以把 -Z 转到「太阳 → 场景」。
        Transform::from_rotation(Quat::from_rotation_arc(Vec3::NEG_Z, -to_sun)),
        // **一级**就够：级联是为了缓解透视相机的近处采样不足，正交投影没有这个问题
        // （`cascade.rs:62` 的注释）。远近两端贴住地形，切片越薄 texel 越细。
        CascadeShadowConfigBuilder {
            num_cascades: 1,
            minimum_distance: cam_near,
            maximum_distance: cam_far,
            ..default()
        }
        .build(),
    ));
    commands.insert_resource(GlobalAmbientLight {
        brightness: boot.sun[3] / EXPOSURE,
        ..default()
    });
    println!(
        "sun    az {:.0},el {:.0} -> sun dir ({:.3},{:.3},{:.3}), illum {:.0} lux, sky {:.2}",
        boot.sun[0], boot.sun[1], to_sun.x, to_sun.y, to_sun.z, illum, boot.sun[3]
    );
    println!("albedo {:.2} (vxl diffuse gain)", boot.albedo);

    // 启动指令交给 exclusive 系统去下（它需要 &mut World 才能写各车的 guide/goal）
    pending.0 = boot.first_order;
    println!("spawned {} units at {:?}", boot.spawn.len(), boot.spawn.first());
}

/// 单位在世界里的位置（格心 + 地面高度）。摆单位、算屏幕坐标都用它。
fn unit_pos(u: &Unit, gf: &world::GroundField) -> Vec3 {
    let y = gf.surface_height(u.cell_x, u.cell_y) / gf.pitch_y as f32 * terrain::LEVEL_HEIGHT;
    Vec3::new(u.cell_x + 0.5, y, u.cell_y + 0.5)
}

/// 单位模型的一段：`turret = false` 是车体（跟 `dir_angle` 转），`true` 是炮塔（跟
/// `turret_angle` 转）—— 两者是独立实体，各自一个 VXL 段的网格。
#[derive(Component)]
struct UnitPart {
    idx: i32,
    turret: bool,
}

/// 单位姿态 —— 照抄 `vxlrender.cpp voxelObjectMatrix`（本仓 `vpl.rs::voxel_object_matrix`）
/// 的旋转段，以及 `wwread.cpp rampTilt` 给出的 pitch / yaw 取值。
///
/// C++ 那三行是**行向量左乘**，`matMul(A, object)` 表示 A **后**作用：
/// ```text
/// object = Rz(tiltYaw)
/// object = matMul(Ry(pitch), object)       // Ry(pitch) · Rz(tiltYaw)
/// object = matMul(Rz(facingYaw), object)   // Rz(facingYaw) · Ry(pitch) · Rz(tiltYaw)
/// ```
/// `p·(A·B) = (p·A)·B` ⇒ 实际几何作用次序是 **先 `Rz(facingYaw)`、再 `Ry(pitch)`、
/// 最后 `Rz(tiltYaw)`**。（先前我把次序按"右到左"读成先 tiltYaw，整条链装反了。）
///
/// 轴映射来自 `voxel_mesh::to_world`：VXL 的 `x`（车头）→ 世界 X、`z`（上）→ 世界 Y、
/// `y`（侧）→ 世界 Z。逐项共轭：
/// * 模型 `Rz(θ)`（绕上轴）→ 世界 `Ry(θ)`（同号）；
/// * 模型 `Ry(θ)`（绕侧轴）→ 世界 `Rz(−θ)`。符号由 C++ 自己的注释定：matRotY 把
///   `(1,0,0)` 送到 `(cos,0,−sin)`，正 pitch 压车头；映射到世界是 `(cosθ,−sinθ,0)`，
///   即绕世界 Y 轴的 `−θ`。`rampTilt` 已给 `pitch = −atan2(len, 256)`（上坡抬头），这个
///   负号就落在这里。
/// * `facingYaw = 90 − dir/256·360 − tiltYaw` 与 `voxelObjectMatrix` 逐字一致。
///
/// 于是列向量序（最后作用的写在最左）：
/// `Ry(tiltYaw) · Rz(−pitch) · Ry(facingYaw)`。
///
/// `facing` 的角度含义由 `facingAngleOf` 定死（朝向 0 = cell 的 −y 方向、64 = +x），
/// `dir = 0/32/64/96` 四点都对上：世界方向依次为 `(0,0,−1)`、`(1,0,−1)`、`(1,0,0)`、
/// `(1,0,1)` —— 正是 `Ry(facing)` 作用在车头 `+X` 上的结果。
fn unit_rotation(dir_angle: f32, tilt_pitch: f32, tilt_yaw: f32) -> Quat {
    let facing = (90.0 - dir_angle / 256.0 * 360.0 - tilt_yaw).to_radians();
    Quat::from_rotation_y(tilt_yaw.to_radians())
        * Quat::from_rotation_z(-tilt_pitch.to_radians())
        * Quat::from_rotation_y(facing)
}

/// 把每辆车放到它那一格的地面上，并让车体 / 炮塔各按自己的角度转。
///
/// 车体跟 `dir_angle`、炮塔跟 `turret_angle`；**两者都带坡面 tilt** —— 照 `world.cpp:952-959`
/// 的 `drawPart` 调用，炮塔传的也是 `e.tiltPitch / e.tiltYaw`（先前这里给炮塔传 0 是错的）。
/// 炮塔的旋转中心就是模型原点：`drawPart` 的 `turretVoxelOffset` 传 `0.0f`，而 `DrawItem`
/// 的 `turretOffset` 全仓从未被赋非零值，所以不需要额外的平移。
fn sync_units(
    env: Res<SimEnv>,
    units: Query<&Unit>,
    mut parts: Query<(&UnitPart, &mut Transform)>,
) {
    let by_idx: std::collections::HashMap<i32, &Unit> =
        units.iter().map(|u| (u.idx, u)).collect();
    for (p, mut t) in &mut parts {
        let Some(u) = by_idx.get(&p.idx) else { continue };
        t.translation = unit_pos(u, &env.ground);
        let ang = if p.turret { u.turret_angle } else { u.dir_angle };
        t.rotation = unit_rotation(ang, u.tilt_pitch as f32, u.tilt_yaw as f32);
    }
}

/// 一根视线打在地形上的性质。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum Hit {
    /// 射线进了这一格、在格内部继续下降穿过它的顶面 —— 真正「指到格子」。
    Face(Cell),
    /// 射线一头撞在这一格的竖直侧壁上 —— 悬崖外立面。
    Wall(Cell),
}

/// 沿视线步进的步长（世界单位，1 格 = 1 单位）。
///
/// 必须明显小于「一层高度对应的水平距离」，否则会一步跨过整格顶面：
/// 1 层 = 0.40825，正交斜视下对应每轴 0.5 格，取 1/8 格留足余量。
const PICK_STEP: f32 = 0.125;

/// 沿视线步进，找第一处「射线高度 ≤ 该格地面」的位置。
///
/// 【为什么不用固定点迭代】正交斜视下，同一个屏幕点把求交平面抬高 h，交点会沿
/// (+x,+z) 平移 `h/|d.y|·|d.x| = 1.224h`，即**每层漂半格**。12 层台地上第一轮和最后
/// 一轮差 6 格，在悬崖处来回震荡、4 轮跑完停在哪全看路线。步进没有这个问题：
/// 高度和格是同时前进的，答案就是几何真值。
///
/// 地面高度认 `level × LEVEL_HEIGHT`，与 `terrain::build_mesh` 画出来的平顶面同一个数；
/// **不用** `GroundField::surface_height` —— 它额外加了桥面 4 级与坡道抬升，桥在本阶段
/// 没有 3D 面，照它求交会指到空气里。
///
/// 与相机无关，`level_of` 直接给格高度，方便单测。
pub(crate) fn march_ray(
    o: Vec3,
    d: Vec3,
    level_of: impl Fn(i32, i32) -> i32,
) -> Option<Hit> {
    if d.y >= -1.0e-6 {
        return None;
    }
    // 地形全在 y = 0 之上，射线走到与 y = 0 平面相交就够；多走一步兜住浮点误差。
    let t_end = -o.y / d.y;
    if t_end <= 0.0 {
        return None;
    }
    let mut t = 0.0f32;
    let mut prev: Option<Cell> = None;
    while t <= t_end + PICK_STEP {
        let p = o + d * t;
        let c = Cell::new((p.x - 0.5).round() as i32, (p.z - 0.5).round() as i32);
        let ground = level_of(c.x, c.y) as f32 * terrain::LEVEL_HEIGHT;
        if p.y <= ground {
            // 上一步和这一步同格 = 在格内部穿过顶面；刚跨进来就低于地面 = 撞在侧壁上。
            return Some(if prev == Some(c) {
                Hit::Face(c)
            } else {
                Hit::Wall(c)
            });
        }
        prev = Some(c);
        t += PICK_STEP;
    }
    None
}

pub(crate) fn march_cell(
    cam: &Camera,
    cam_tf: &GlobalTransform,
    cursor: Vec2,
    env: &SimEnv,
) -> Option<Hit> {
    let ray = cam.viewport_to_world(cam_tf, cursor).ok()?;
    march_ray(ray.origin, *ray.direction, |x, y| env.ground.level_at(x, y))
}

/// 命中结果 + 上一次的选择 → 新的选择。`Wall` 一律保留 `prev`（侧壁滞回）。
pub(crate) fn resolve(hit: Option<Hit>, prev: Option<Cell>) -> Option<Cell> {
    match hit {
        Some(Hit::Face(c)) => Some(c),
        Some(Hit::Wall(_)) => prev,
        None => None,
    }
}

/// 屏幕点 → 格。**全工程唯一**的取格子实现 —— 指向高亮、debug 面板、下指令共用这一份。
///
/// 【侧壁滞回】悬崖侧壁在等距投影下盖住一大片屏幕。若「命中谁就是谁」，鼠标在墙上
/// 稍微一动格子就变；用户口径是**只要指在墙上就不动，直到真的指到格子顶面才改**。
/// 所以 `Wall` 一律返回 `prev`（上一次真正指到的格），只有 `Face` 才更新。
pub(crate) fn pick_cell(
    cam: &Camera,
    cam_tf: &GlobalTransform,
    cursor: Vec2,
    env: &SimEnv,
    prev: Option<Cell>,
) -> Option<Cell> {
    resolve(march_cell(cam, cam_tf, cursor, env), prev)
}

/// 当前鼠标指向的格 —— 全工程唯一的一份「指向」状态。
///
/// 面板、高亮、下指令都读它。之前 `select_input` 自己算一次、`debugview::hover` 又算一次，
/// 两份口径只要有一点差别就会出现「面板指到的格」和「部队真去的格」不一样。
#[derive(Resource, Default)]
pub(crate) struct CursorCell(pub Option<Cell>);

/// 每帧把 `CursorCell` 推到最新的指向格。放在 `select_input` 之前跑。
fn update_cursor_cell(
    windows: Query<&Window, With<PrimaryWindow>>,
    cams: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
    env: Res<SimEnv>,
    mode: Res<debugview::LogicMode>,
    mut cursor_cell: ResMut<CursorCell>,
) {
    let Ok(win) = windows.single() else { return };
    let Ok((cam, cam_tf)) = cams.single() else { return };
    let Some(cur) = win.cursor_position() else {
        cursor_cell.0 = None;
        return;
    };
    // 指在 debug 面板上时保持不动（那是 UI，不是地面）。贴图模式没有面板，别误屏蔽。
    if mode.0 && debugview::cursor_on_panel(cur) {
        return;
    }
    let next = pick_cell(cam, cam_tf, cur, &env, cursor_cell.0);
    if next != cursor_cell.0 {
        cursor_cell.0 = next;
    }
}

/// 鼠标：**左键点击 = 下指令**（用户要求保持）、**左键拖拽 = 框选**、**右键 = 单选**。
/// 单击/拖拽的 5 像素判定、框选判据、26 像素的点选半径都照抄 C++
/// `Play::step / selectAt / selectRect`（engine/play.cpp:232-302）。
fn select_input(
    buttons: Res<ButtonInput<MouseButton>>,
    windows: Query<&Window, With<PrimaryWindow>>,
    cams: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
    env: Res<SimEnv>,
    cursor_cell: Res<CursorCell>,
    units: Query<(Entity, &Unit)>,
    mut sel: ResMut<Selection>,
    mut pending: ResMut<PendingOrder>,
    mut drag: Local<Option<Vec2>>,
) {
    let Ok(win) = windows.single() else { return };
    let Ok((cam, cam_tf)) = cams.single() else { return };
    let cur = win.cursor_position();
    // 光标在 debug 面板上时不走选/令：按钮的点击也是左键，否则点一下按钮顺手给地面下条指令。
    if cur.is_some_and(debugview::cursor_on_panel) {
        return;
    }

    let screen_of = |u: &Unit| -> Option<Vec2> {
        cam.world_to_viewport(cam_tf, unit_pos(u, &env.ground)).ok()
    };

    if buttons.just_pressed(MouseButton::Left) {
        *drag = Some(cur.unwrap_or(Vec2::ZERO));
    }
    if buttons.just_released(MouseButton::Left) {
        let end = cur.unwrap_or(Vec2::ZERO);
        if let Some(d0) = drag.take() {
            if (end - d0).abs().max_element() < 5.0 {
                // 用 `CursorCell`（面板显示的那一格），不在这里再算一次 —— 面板指哪就打哪。
                if let Some(cell) = cursor_cell.0 {
                    println!("order  -> cell ({},{})", cell.x, cell.y);
                    pending.0 = Some(cell);
                }
            } else {
                let (x0, x1) = (d0.x.min(end.x), d0.x.max(end.x));
                let (y0, y1) = (d0.y.min(end.y), d0.y.max(end.y));
                let mut picked = Vec::new();
                for (e, u) in &units {
                    if let Some(p) = screen_of(u) {
                        if p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1 {
                            picked.push(e);
                        }
                    }
                }
                println!("select -> {} units (rect)", picked.len());
                sel.0 = picked;
            }
        }
    }
    if buttons.just_pressed(MouseButton::Right) {
        let Some(cur) = cur else { return };
        let mut best = 26.0f32; // C++ `selectAt` 的 26 屏幕像素半径
        let mut hit = None;
        for (e, u) in &units {
            if let Some(p) = screen_of(u) {
                let d = (p - cur).length();
                if d < best {
                    best = d;
                    hit = Some(e);
                }
            }
        }
        println!("select -> {:?} (single)", hit.is_some());
        sel.0 = hit.into_iter().collect();
    }
}

/// 缩放与相机平移 —— 照抄 C++ `Play::step`（engine/play.cpp:265-302）：
/// 滚轮 `zoom *= 1.18^wheel` 夹在 `[0.35, 2.5]`；平移速度 `760 / zoom` 世界像素/秒
/// （即屏幕上恒定 760 像素/秒）；来源是 WASD/方向键 + 鼠标贴屏幕边缘；末尾
/// `clampCamera` 把视口夹进地形范围。
fn update_view(
    time: Res<Time>,
    windows: Query<&Window, With<PrimaryWindow>>,
    keys: Res<ButtonInput<KeyCode>>,
    mut wheel: MessageReader<MouseWheel>,
    mut view: ResMut<View>,
    mut cams: Query<(&mut Projection, &mut Transform), With<Camera3d>>,
) {
    let Ok(win) = windows.single() else { return };
    let win_sz = Vec2::new(win.width(), win.height());
    // 窗口最小化时尺寸会是 0：`px_per_unit` 里除它没意义，直接跳过这一帧。
    if win_sz.x < 1.0 || win_sz.y < 1.0 {
        return;
    }

    // 平移 / 夹边界 / 相机朝向全部按当前模式取基向量。两套范围各自独立：
    // 等距那套是沿 `CAM_RIGHT/CAM_UP` 量出来的，俯视是 X 与 −Z 的极值，混用会把相机夹到错的地方。
    let (d, r, u, up_hint) = cam_basis(view.top_down);
    let (map_h, map_v) = if view.top_down {
        (view.map_h_top, view.map_v_top)
    } else {
        (view.map_h, view.map_v)
    };

    let mut pan = Vec2::ZERO;
    if keys.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]) {
        pan.x -= 1.0;
    }
    if keys.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]) {
        pan.x += 1.0;
    }
    if keys.any_pressed([KeyCode::KeyW, KeyCode::ArrowUp]) {
        pan.y -= 1.0;
    }
    if keys.any_pressed([KeyCode::KeyS, KeyCode::ArrowDown]) {
        pan.y += 1.0;
    }
    let edge = view.edge_scroll;
    if let Some(c) = win.cursor_position().filter(|_| edge > 0.0) {
        if c.x <= edge {
            pan.x -= 1.0;
        }
        if c.x >= win_sz.x - edge {
            pan.x += 1.0;
        }
        if c.y <= edge {
            pan.y -= 1.0;
        }
        if c.y >= win_sz.y - edge {
            pan.y += 1.0;
        }
    }
    if pan != Vec2::ZERO {
        // 屏幕右 / 下 对应的世界方向（沿它们平移，投影上就是画布平移）。
        // 速度用**本帧的旧 zoom** 算（C++ 是先平移、后滚轮、最后夹边界）。
        let speed = view.pan_speed / view.px_per_unit(win_sz);
        let right = r;
        let down = -u;
        view.look += (right * pan.x + down * pan.y) * speed * time.delta_secs();
    }

    let mut w = 0.0f32;
    for ev in wheel.read() {
        w += ev.y;
    }
    if w != 0.0 {
        view.zoom = (view.zoom * 1.18f32.powf(w)).clamp(0.35, 2.5);
    }

    // 夹视口（C++ `play.cpp:138 clampCamera`，原实现整个漏了）：视口不许越出地形 ——
    // 越过之后相机外面什么都没有，整屏只剩背景色。`look` 是视口中心，所以用
    // 屏幕基 (CAM_RIGHT, CAM_UP, CAM_DIR) 分解后逐轴夹，再合成回去。
    let s = view.size(win_sz);
    // 【像素对齐】FA2sp 的滚动是整数像素（`MapCoord2ScreenCoord` 全是整数乘加），砖的
    // 菱形边界永远压在像素格上；我们这里是浮点相机，多一步吸附，让「砖边界 ↔ 屏幕像素」
    // 的相位固定。否则相机一动相位就漂，菱形边压在像素中间 —— 关掉 MSAA 后不再变粉，
    // 但会看到 1px 抖动。`k` = 每世界单位多少像素，所以 1/k 就是 1 像素对应的世界长度。
    let k = view.px_per_unit(win_sz);
    let snap = |x: f32| if k > 0.0 { (x * k).round() / k } else { x };
    let h = snap(clamp_axis(view.look.dot(r), map_h, s.x));
    let v = snap(clamp_axis(view.look.dot(u), map_v, s.y));
    view.look = r * h + u * v + d * view.look.dot(d);

    if let Ok((mut proj, mut tf)) = cams.single_mut() {
        if let Projection::Orthographic(o) = &mut *proj {
            o.scaling_mode = ScalingMode::AutoMin {
                min_width: s.x,
                min_height: s.y,
            };
        }
        tf.translation = view.look + d * boot_cam_dist();
        tf.look_at(view.look, up_hint);
    }
}

/// 相机离看向点的距离：正交投影下只影响近/远平面，取一个够大的定值。
fn boot_cam_dist() -> f32 {
    2000.0
}

/// 开局全选（照 C++ `runScripted` 的做法）：演示时立刻能点地下指令；
/// 之后由鼠标框选 / 右键单选改写。
fn select_all_initial(mut sel: ResMut<Selection>, q: Query<Entity, With<Unit>>) {
    sel.0 = q.iter().collect();
}

/// 执行排队中的指令。exclusive：`issue_order_at` 要 `&mut World`。
fn auto_order(world: &mut World) {
    let dest = { world.resource_mut::<PendingOrder>().0.take() };
    if let Some(d) = dest {
        let n = play::issue_order_at(world, d);
        println!("order -> ({},{}) : {n} units pathed", d.x, d.y);
    }
}

/// 无人盯屏时自证「确实画出来了」：2.5s 截图、5s 退出。
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

/// 每隔两秒在屏幕上打一行「跑了几 tick、车都在哪」，方便无窗口时看进度。
fn auto_move(time: Res<Time>, env: Res<SimEnv>, q: Query<&Unit>, mut last: Local<f32>) {
    let t = time.elapsed_secs();
    if t - *last < 2.0 {
        return;
    }
    *last = t;
    let mut n = 0;
    let mut sx = 0.0;
    let mut sy = 0.0;
    for u in &q {
        n += 1;
        sx += u.cell_x;
        sy += u.cell_y;
    }
    if n > 0 {
        println!(
            "tick {}: {} units, mean cell = ({:.1},{:.1})",
            env.tick,
            n,
            sx / n as f32,
            sy / n as f32
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 自动适配全图（`cell_px = 0`，`启动.bat` 的默认）时的视图。数值按 `hillbtwn`
    /// 的实测（`game` 启动打的那行 `screen 84.9x48.4 world units -> view 93.9x55.3`）：
    /// 地形投到 `CAM_RIGHT` 上跨 84.9、投到 `CAM_UP` 上跨 48.4，`fit` 是它加 6% 余量，
    /// 初始 `look` 是地形包围盒中心（投影后落在两个范围的正中）。
    fn view_auto() -> View {
        View {
            zoom: 1.0,
            cell_px: 0.0,
            look: Vec3::new(64.5, 2.0, 64.5),
            fit: Vec2::new(93.9, 55.3),
            map_h: Vec2::new(-42.45, 42.45),
            map_v: Vec2::new(-67.7, -19.3),
            // 俯视用的那两套范围：世界 X 与 −Z 的极值。这里只是填一份自洽的值，
            // 本组测试只验等距那条路。
            map_h_top: Vec2::new(0.0, 128.0),
            map_v_top: Vec2::new(-128.0, 0.0),
            top_down: false,
            edge_scroll: 24.0,
            pan_speed: 760.0,
        }
    }

    /// 回归：`cell_px = 0` 时 `px_per_unit` 曾经恒等于 0（旧实现是
    /// `cell_px / ISO_SPAN * zoom`），平移速度 `760 / 0 = inf`，
    /// 鼠标一贴边相机就变 NaN —— 画面整片消失。它必须是有限正数。
    #[test]
    fn auto_fit_px_per_unit_is_finite_and_positive() {
        let win = Vec2::new(1280.0, 720.0);
        for zoom in [0.35, 1.0, 2.5] {
            let mut v = view_auto();
            v.zoom = zoom;
            let k = v.px_per_unit(win);
            assert!(k.is_finite() && k > 0.0, "zoom={zoom} k={k}");
            assert!(760.0 / k < 1.0e6, "平移速度应有界");
        }
    }

    /// 视口世界范围必须与窗口同比，否则 `AutoMin` 会把画面非等比拉伸。
    #[test]
    fn size_matches_window_aspect() {
        let win = Vec2::new(1280.0, 720.0);
        let ar = win.x / win.y;
        for (cell_px, zoom) in [(0.0f32, 1.0f32), (0.0, 0.5), (60.0, 1.0), (60.0, 2.5)] {
            let mut v = view_auto();
            v.cell_px = cell_px;
            v.zoom = zoom;
            let s = v.size(win);
            assert!(
                (s.x / s.y - ar).abs() < 1.0e-3,
                "cell_px={cell_px} zoom={zoom} s={s:?}"
            );
            // `fit` 里要装下的地形范围必须都还在视口里（contain）
            if cell_px == 0.0 && zoom == 1.0 {
                assert!(s.x >= v.fit.x - 1.0e-3 && s.y >= v.fit.y - 1.0e-3, "s={s:?}");
            }
        }
    }

    /// 「1 格 = 60 像素」模式下，每世界单位的像素数就是 `60 / ISO_SPAN`。
    #[test]
    fn cell_px_mode_keeps_ra2_scale() {
        let win = Vec2::new(1280.0, 720.0);
        let mut v = view_auto();
        v.cell_px = 60.0;
        let k = v.px_per_unit(win);
        let want = 60.0 / ISO_SPAN;
        assert!((k - want).abs() < 1.0e-3, "k={k} want={want}");
    }

    /// 夹相机：视口整体落在 `range` 内；视口比 range 还大时**居中**（不是贴角）。
    #[test]
    fn clamp_axis_keeps_viewport_inside() {
        let range = Vec2::new(0.0, 640.0);
        assert_eq!(clamp_axis(-100.0, range, 200.0), 100.0);
        assert_eq!(clamp_axis(9999.0, range, 200.0), 540.0);
        assert_eq!(clamp_axis(320.0, range, 200.0), 320.0);
        // 视口(span)比 range 大：居中，绝不贴角
        assert_eq!(clamp_axis(150.0, Vec2::new(0.0, 100.0), 300.0), 50.0);
        assert_eq!(clamp_axis(-50.0, Vec2::new(0.0, 100.0), 300.0), 50.0);
        // 退化 range（空地图）不能出 inf/NaN
        let c = clamp_axis(0.0, Vec2::ZERO, 0.0);
        assert!(c.is_finite());
    }

    /// 默认的「缩小到看全图」下，视口比地形还宽/高 **且初始 `look` 本就在中心** ——
    /// 夹相机不许把画面顶到一侧（C++ 的 `max(0, mapW - viewW)` 会顶到左上角）。
    #[test]
    fn full_fit_view_stays_centered() {
        let win = Vec2::new(1280.0, 720.0);
        let v = view_auto();
        let s = v.size(win);
        assert!(s.x > v.map_h.y - v.map_h.x && s.y > v.map_v.y - v.map_v.x);
        let r = Vec3::from_array(CAM_RIGHT);
        let u = Vec3::from_array(CAM_UP);
        let h = clamp_axis(v.look.dot(r), v.map_h, s.x);
        let vv = clamp_axis(v.look.dot(u), v.map_v, s.y);
        assert!((h - v.look.dot(r)).abs() < 1.0e-3, "h 被挪了：{h}");
        assert!((vv - (v.map_v.x + v.map_v.y) * 0.5).abs() < 1.0e-3);
    }

    /// 回归：鼠标一直压在右下角（`pan = (+1, +1)`，`panY` 往下 → `-CAM_UP` 方向）
    /// 推 10 秒 —— 相机必须全程有限、且停在视口范围内的边缘，绝不冲出地图。
    #[test]
    fn edge_pan_stays_inside_map() {
        let win = Vec2::new(1280.0, 720.0);
        let mut v = view_auto();
        v.zoom = 2.5; // 放大到视口比地形小，才推得动
        let r = Vec3::from_array(CAM_RIGHT);
        let u = Vec3::from_array(CAM_UP);
        let d = Vec3::from_array(CAM_DIR).normalize();
        for frame in 0..600 {
            let dt = 1.0 / 60.0;
            let speed = 760.0 / v.px_per_unit(win);
            v.look += (r - u) * speed * dt;
            let s = v.size(win);
            let h = clamp_axis(v.look.dot(r), v.map_h, s.x);
            let vv = clamp_axis(v.look.dot(u), v.map_v, s.y);
            v.look = r * h + u * vv + d * v.look.dot(d);
            assert!(v.look.is_finite(), "frame {frame}: look 爆了 {:?}", v.look);
        }
        // 推到底：h 贴右边缘、v 贴下边缘
        let s = v.size(win);
        assert!((v.look.dot(r) - (v.map_h.y - s.x * 0.5)).abs() < 0.5);
        assert!((v.look.dot(u) - (v.map_v.x + s.y * 0.5)).abs() < 0.5);
    }

    /// 台阶：`rx <= 69` 是 12 层台地，右边一律 0 层。
    fn step_level(x: i32, _y: i32) -> i32 {
        if x <= 69 {
            12
        } else {
            0
        }
    }

    /// 射线落进台地内部、在同一格里穿过顶面 → `Face`（这是「指到格子」）。
    #[test]
    fn march_ray_inside_plateau_is_face() {
        let h = 12.0 * terrain::LEVEL_HEIGHT;
        let o = Vec3::new(69.5, h + 5.0, 68.5);
        let d = Vec3::new(-1.0, -1.0, -1.0).normalize();
        match march_ray(o, d, step_level) {
            Some(Hit::Face(c)) => assert!(c.x <= 69, "落点应在台地上：{c:?}"),
            other => panic!("应判 Face，实得 {other:?}"),
        }
    }

    /// 射线从低地上方扑向台地竖面 → `Wall`（这是「指在悬崖壁上」）。
    #[test]
    fn march_ray_into_cliff_side_is_wall() {
        let h = 12.0 * terrain::LEVEL_HEIGHT;
        // 起点在 (70,·) 低地格上方、只比台地顶高 0.1 —— 跨过 x = 70 那面墙时已经低于顶面。
        let o = Vec3::new(70.2, h + 0.1, 68.5);
        let d = Vec3::new(-1.0, -1.0, -1.0).normalize();
        match march_ray(o, d, step_level) {
            Some(Hit::Wall(c)) => assert!(c.x <= 69, "墙属于台地格：{c:?}"),
            other => panic!("应判 Wall，实得 {other:?}"),
        }
    }

    /// 侧壁滞回：`Wall` 不动、`Face` 才改、打不到地形才清空。
    #[test]
    fn resolve_keeps_selection_on_wall() {
        let prev = Cell::new(69, 68);
        assert_eq!(resolve(Some(Hit::Wall(Cell::new(30, 30))), Some(prev)), Some(prev));
        assert_eq!(
            resolve(Some(Hit::Face(Cell::new(31, 31))), Some(prev)),
            Some(Cell::new(31, 31))
        );
        assert_eq!(resolve(None, Some(prev)), None);
    }
}

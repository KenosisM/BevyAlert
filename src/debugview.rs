//! 逻辑 mesh 模式（`[Render] LogicMesh=1`）的 debug 覆盖层。
//!
//! 三样东西，只在逻辑模式下存在（`game.rs` 里按 `logic_mesh` 条件注册）：
//!
//! 1. **鼠标指向哪格，哪格染黄** —— 一个到顶点色的替身 mesh，几何与地形**同一套**公式
//!    （`terrain::cell_local_corners` / `terrain::cell_center_at`），按 `(w,h,ramp)` 缓存，
//!    每帧只挪 `Transform`，不重建地形 mesh。
//! 2. **左上角信息面板** —— 中文（系统宋体），显示格坐标 / 层高 / ramp / TMP 文件名 /
//!    砖集 / cell 在砖内的偏移 / extra 矩形与它在砖内的位置。
//!    TMP 名由 `theater::tile_asset_name` 从 `tile_id` 反查，与读素材同一个函数。
//!    同一份信息另有一行单行版打到控制台（换格时才打），验证时直接读 cmd，不用截图。
//! 3. **俯视图按钮** —— 再按一下切回等距。相机基在 `game.rs` 的 `cam_basis` 里切换。
//!
//! 字体走 `C:/Windows/Fonts/simsun.ttc`（用户口径：引擎必须支持中文，直接读系统宋体）。
//! API 细节与坑见 `docs/调研-bevy-ui-按钮与中文字体.md`。

use std::collections::HashMap;

use bevy::asset::RenderAssetUsages;
use bevy::mesh::Indices;
use bevy::prelude::*;
use bevy::render::render_resource::PrimitiveTopology;
use bevy::ui_widgets::{Activate, Button};

use crate::costgrid::TerrainCell;
use crate::game::View;
use crate::terrain;
use crate::theater::{self, Theater};

/// 系统宋体。`.ttc` 里第一个 face 就是 SimSun。
const FONT_PATH: &str = "C:/Windows/Fonts/simsun.ttc";

/// 面板占据的屏幕矩形（左上角）。拾取时落在这里面就不选格子 —— 避免点按钮时把地形选走。
const PANEL_X: f32 = 8.0;
const PANEL_Y: f32 = 8.0;
const PANEL_W: f32 = 430.0;
/// 只用于「光标在不在面板上」的判定（点按钮时别顺手给地面下指令）。面板本身的高度
/// 由内容撑开，所以这个值要贴着实际排版高度，否则面板下方会留一圈点不动的死区。
const PANEL_H: f32 = 190.0;

/// 高亮面相对地形顶面抬起的世界高度。约 1 屏幕像素，压住 z-fighting。
const MARKER_LIFT: f32 = 0.02;

const NO_HOVER: &str = "指向地形查看格子信息";

/// 逻辑模式开关。`game.rs` 在 `run()` 里按 `cfg.logic_mesh` 填，debug 层据此决定生不生效。
#[derive(Resource, Clone, Copy)]
pub struct LogicMode(pub bool);

/// 一格的**精简**信息。刻意不搬 `pixels` / `zbuf` / `extra` —— 那些是几 MB 的位图，
/// debug 面板一个数字都用不到。
#[derive(Clone)]
pub struct CellInfo {
    pub rx: i32,
    pub ry: i32,
    pub dx: i32,
    pub dy: i32,
    pub z: i32,
    pub level: i32,
    pub ramp_type: u8,
    pub terrain_type: u8,
    pub bridge: bool,
    pub w: i32,
    pub h: i32,
    pub has_extra: bool,
    /// extra 矩形，相对**这一片 cell 的左上角**（不是砖左上角）。
    pub ex_x: i32,
    pub ex_y: i32,
    pub ex_w: i32,
    pub ex_h: i32,
    /// 这一片 cell 在 TMP 整砖里的等距偏移。砖 = `w x h` 像素，cell 摆在里面。
    /// 定位 extra 的世界位置必须用 `cell + extra`，只用 `extra` 会差一个 cell 的偏移。
    pub cell_x: i32,
    pub cell_y: i32,
    pub tile_id: u64,
    /// 这格用到的 TMP 文件名，如 `Cliff0901.tem`（一个文件里可能有好几个 cell，
    /// 具体第几个看 `cell` 序号）。
    pub asset: String,
}

/// 按 `(rx, ry)` 索引的格信息表 + tile set 名字表（`tile_id` 只存了 set 序号）。
#[derive(Resource, Default)]
pub struct DebugCells {
    pub map: HashMap<(i32, i32), CellInfo>,
    pub set_names: Vec<String>,
}

impl DebugCells {
    /// `cells` 里的 `tile_id` 只编了 `set_idx << 32 | in_set << 16 | variant << 8 | cell`，
    /// 名字得回到 theater 的 TileSet 表里查 —— 文件名走 `theater::tile_asset_name`，
    /// 与真正去 MIX 里读素材用的是同一个函数，不另写一份拼名规则。
    pub fn from_cells(cells: &[TerrainCell], th: &Theater) -> Self {
        let set_names: Vec<String> = th.tile_sets.iter().map(|s| s.set_name.clone()).collect();
        let mut map = HashMap::with_capacity(cells.len());
        for c in cells {
            let set_idx = (c.tile_id >> 32) as usize;
            let in_set = ((c.tile_id >> 16) & 0xFFFF) as i32;
            let variant = ((c.tile_id >> 8) & 0xFF) as i32;
            let asset = match th.tile_sets.get(set_idx) {
                Some(ts) => theater::tile_asset_name(th, ts, in_set, variant),
                None => format!("? (set {set_idx} 不在表中)"),
            };
            map.insert(
                (c.rx, c.ry),
                CellInfo {
                    rx: c.rx,
                    ry: c.ry,
                    dx: c.dx,
                    dy: c.dy,
                    z: c.z,
                    level: c.level,
                    ramp_type: c.ramp_type,
                    terrain_type: c.terrain_type,
                    bridge: c.bridge,
                    w: c.w,
                    h: c.h,
                    has_extra: c.has_extra,
                    ex_x: c.ex_x,
                    ex_y: c.ex_y,
                    ex_w: c.ex_w,
                    ex_h: c.ex_h,
                    cell_x: c.cell_x,
                    cell_y: c.cell_y,
                    tile_id: c.tile_id,
                    asset,
                },
            );
        }
        Self { map, set_names }
    }
}

#[derive(Component)]
pub(crate) struct Marker;

#[derive(Component)]
pub(crate) struct DebugText;

/// 俯视图切换按钮的标记（`game.rs` 的 observer 用它认按钮）。
#[derive(Component)]
pub struct ToggleTopDown;

#[derive(Component)]
pub(crate) struct ToggleLabel;

/// 按 `(w, h, ramp_type)` 缓存的高亮替身 mesh。同一套几何只建一次。
#[derive(Resource, Default)]
pub(crate) struct MarkerMeshes(HashMap<(i32, i32, u8), Handle<Mesh>>);

/// 建面板 / 按钮 / 高亮替身。只在逻辑模式下真的干活。
pub fn setup(
    mut commands: Commands,
    mode: Res<LogicMode>,
    mut fonts: ResMut<Assets<Font>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if !mode.0 {
        return;
    }

    // --- 中文字体：直接读系统宋体 ------------------------------------------------
    // 不能靠 `FontSource::family("SimSun")`：bevy_text 的 `system_font_discovery`
    // (= parley/system) 默认没开，Parley 的 collection 里没有系统字体。读字节绕开它。
    let font: Option<Handle<Font>> = match std::fs::read(FONT_PATH) {
        Ok(b) => {
            println!("debug ui font: {} bytes ({FONT_PATH})", b.len());
            Some(fonts.add(Font::from_bytes(b)))
        }
        Err(e) => {
            println!("debug ui font: 读不到 {FONT_PATH}（{e}）—— 中文会显示空白");
            None
        }
    };
    let font_source = font.clone().map(FontSource::Handle).unwrap_or_default();
    let text_font = TextFont {
        font: font_source,
        font_size: 14.0.into(),
        ..default()
    };

    // --- 左上角面板 ---------------------------------------------------------------
    commands.spawn((
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(PANEL_X),
            top: Val::Px(PANEL_Y),
            width: Val::Px(PANEL_W),
            padding: UiRect::all(Val::Px(8.0)),
            flex_direction: FlexDirection::Column,
            row_gap: Val::Px(6.0),
            ..Node::DEFAULT
        },
        BackgroundColor(Color::srgba(0.03, 0.04, 0.06, 0.85)),
        children![
            (
                ToggleTopDown,
                Button,
                Node {
                    align_self: AlignSelf::FlexStart,
                    padding: UiRect::axes(Val::Px(10.0), Val::Px(5.0)),
                    ..Node::DEFAULT
                },
                BackgroundColor(Color::srgb(0.16, 0.20, 0.28)),
                children![(
                    ToggleLabel,
                    Text::new("俯视图"),
                    text_font.clone(),
                    TextColor(Color::WHITE),
                )],
            ),
            (
                DebugText,
                Text::new(NO_HOVER),
                text_font.clone(),
                TextColor(Color::srgb(0.90, 0.95, 1.0)),
            ),
        ],
    ));

    // --- 高亮替身 -----------------------------------------------------------------
    // unlit 纯黄：逻辑模式本来就不吃光源，高亮也不该吃。
    let mat = materials.add(StandardMaterial {
        base_color: Color::srgb(1.0, 0.86, 0.10),
        unlit: true,
        cull_mode: None,
        ..default()
    });
    let first = meshes.add(marker_mesh(60, 30, 0));
    commands.spawn((
        Marker,
        Mesh3d(first),
        MeshMaterial3d(mat),
        Transform::default(),
        Visibility::Hidden,
    ));
}

/// 一格顶面的黄色替身（本地坐标，中心在原点）。
fn marker_mesh(w: i32, h: i32, ramp_type: u8) -> Mesh {
    let c = terrain::cell_local_corners(w, h, ramp_type);
    Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    )
    .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, c.to_vec())
    .with_inserted_attribute(Mesh::ATTRIBUTE_NORMAL, vec![[0.0f32, 1.0, 0.0]; 4])
    .with_inserted_indices(Indices::U32(vec![0, 1, 2, 0, 2, 3]))
}

/// 每帧：光标 → 格 → 挪高亮 + 刷面板。只在换格时真的动 —— 否则每帧都在改 UI 文本，
/// 会触发 Parley 重排版。
pub fn hover(
    cells: Res<DebugCells>,
    cursor_cell: Res<crate::game::CursorCell>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut cache: ResMut<MarkerMeshes>,
    mut marker: Query<(&mut Transform, &mut Visibility, &mut Mesh3d), With<Marker>>,
    mut texts: Query<&mut Text, (With<DebugText>, Without<ToggleLabel>)>,
    mut last: Local<Option<Option<(i32, i32)>>>,
) {
    // 指向格不在这里算：全工程只有 `game::update_cursor_cell` 一处，
    // 面板、黄框、下指令读的是同一个值。
    let key = cursor_cell.0.map(|c| (c.x, c.y));
    if last.as_ref() == Some(&key) {
        return;
    }
    *last = Some(key);

    // 控制台同步打一份单行版：验证/排查直接读 cmd，不用截图。
    let info = key.and_then(|(x, y)| cells.map.get(&(x, y)));
    match info {
        Some(i) => println!("hover -> {}", console_line(i, &cells.set_names)),
        None => println!("hover -> -"),
    }

    let Ok((mut tf, mut vis, mut mesh3d)) = marker.single_mut() else {
        return;
    };
    match info {
        Some(i) => {
            let center = terrain::cell_center_at(i.rx, i.ry, i.level, i.w, i.h);
            tf.translation = Vec3::from(center) + Vec3::Y * MARKER_LIFT;
            let ramp = i.ramp_type.min(20);
            let handle = cache
                .0
                .entry((i.w, i.h, ramp))
                .or_insert_with(|| meshes.add(marker_mesh(i.w, i.h, ramp)))
                .clone();
            mesh3d.0 = handle;
            *vis = Visibility::Visible;
            if let Ok(mut t) = texts.single_mut() {
                t.0 = describe(i, &cells.set_names);
            }
        }
        None => {
            *vis = Visibility::Hidden;
            if let Ok(mut t) = texts.single_mut() {
                t.0 = NO_HOVER.to_string();
            }
        }
    }
}

/// 屏幕点 → 格。**不在这里实现**：全工程只有 `game::march_cell / pick_cell` 一份，
/// 指向状态只有 `game::CursorCell` 一份。否则面板指到的格与下指令的格会各说各话
/// （曾经就是如此：面板按地形高度迭代、指令按 y=0 求交，12 层台地上两者差 (−6,−6)）。

fn in_panel(p: Vec2) -> bool {
    p.x >= PANEL_X - 4.0
        && p.x <= PANEL_X + PANEL_W + 4.0
        && p.y >= PANEL_Y - 4.0
        && p.y <= PANEL_Y + PANEL_H + 4.0
}

/// 光标是否落在 debug 面板上。`select_input` 用它避免「点按钮顺手给地面下一条指令」。
pub fn cursor_on_panel(p: Vec2) -> bool {
    in_panel(p)
}

/// `tile_id` 的四个字段 + set 名。面板正文与控制台日志共用，免得两边各解一次位。
/// 返回 `(set_idx, in_set, variant, cell, set_name)`。
fn tile_parts<'a>(i: &CellInfo, set_names: &'a [String]) -> (i32, i32, i32, i32, &'a str) {
    let set_idx = (i.tile_id >> 32) as i32;
    let in_set = ((i.tile_id >> 16) & 0xFFFF) as i32;
    let variant = ((i.tile_id >> 8) & 0xFF) as i32;
    let cell = (i.tile_id & 0xFF) as i32;
    let set_name = set_names
        .get(set_idx.max(0) as usize)
        .map(|s| s.as_str())
        .unwrap_or("?");
    (set_idx, in_set, variant, cell, set_name)
}

/// extra 相对**砖左上角**的位置 = cell 偏移 + extra 偏移。
///
/// TMP 里 `ex_x/ex_y` 是相对这一片 cell 的左上角，不是整砖的左上角（2026-10-18 实测：
/// Cliff09 的 cell1 是 `cell(30,15) + extra(30,-45) = 砖(60,-30)`）。定位 extra 的
/// 世界位置必须走这个加法。面板与控制台共用这一份，不许各算一次。
fn extra_in_tile(i: &CellInfo) -> (i32, i32) {
    (i.cell_x + i.ex_x, i.cell_y + i.ex_y)
}

/// 面板正文。字段与 `costgrid --cell` 探针一致，方便两边对着看。
fn describe(i: &CellInfo, set_names: &[String]) -> String {
    let (set_idx, in_set, variant, cell, set_name) = tile_parts(i, set_names);
    let (tx, ty) = extra_in_tile(i);
    let extra = if i.has_extra {
        format!(
            "extra {}x{} @ cell({},{}) → 砖({},{})",
            i.ex_w, i.ex_h, i.ex_x, i.ex_y, tx, ty
        )
    } else {
        "无 extra".to_string()
    };
    format!(
        "格 ({},{})   偏移 ({},{})\n\
         层 {}   世界 Y {:.2}   z {}\n\
         坡 {}   地形 {}   桥 {}\n\
         TMP {}   cell {}\n\
         set {}   #{}   内序 {}   变体 {}\n\
         cell 偏移 ({},{})   砖 {}x{}\n\
         {}",
        i.rx,
        i.ry,
        i.dx,
        i.dy,
        i.level,
        i.level as f32 * terrain::LEVEL_HEIGHT,
        i.z,
        i.ramp_type,
        i.terrain_type,
        if i.bridge { "是" } else { "否" },
        i.asset,
        cell,
        set_name,
        set_idx,
        in_set,
        variant,
        i.cell_x,
        i.cell_y,
        i.w,
        i.h,
        extra,
    )
}

/// 同一份信息的单行版，打到控制台 —— 验证时不靠截图，直接在 cmd 里读。
fn console_line(i: &CellInfo, set_names: &[String]) -> String {
    let (set_idx, in_set, variant, cell, set_name) = tile_parts(i, set_names);
    let (tx, ty) = extra_in_tile(i);
    format!(
        "({},{}) L{} z{} ramp{} terr{} bridge{} TMP {} cell{} set {:?} #{}/in{}/var{} \
         cellXY({},{}) tile {}x{} ext{} {}",
        i.rx,
        i.ry,
        i.level,
        i.z,
        i.ramp_type,
        i.terrain_type,
        if i.bridge { 'Y' } else { 'N' },
        i.asset,
        cell,
        set_name,
        set_idx,
        in_set,
        variant,
        i.cell_x,
        i.cell_y,
        i.w,
        i.h,
        if i.has_extra { 'Y' } else { 'N' },
        if i.has_extra {
            format!(
                "{}x{}@cell({},{})->tile({},{})",
                i.ex_w, i.ex_h, i.ex_x, i.ex_y, tx, ty
            )
        } else {
            "-".to_string()
        }
    )
}

/// 点按钮：俯视 ⇄ 等距。只认带 `ToggleTopDown` 的按钮 —— `Activate` 是所有
/// `bevy_ui_widgets::Button` 共用的，将来加了别的按钮不能一起被切。
pub fn on_toggle(
    activate: On<Activate>,
    buttons: Query<(), With<ToggleTopDown>>,
    mut view: ResMut<View>,
) {
    if buttons.get(activate.entity).is_err() {
        return;
    }
    view.top_down = !view.top_down;
    println!(
        "view -> {}",
        if view.top_down { "top-down" } else { "isometric" }
    );
}

/// 按钮文字跟着模式走。只在真的变了的时候写 `Text`，否则每帧重排版。
pub fn sync_label(
    view: Res<View>,
    mut q: Query<&mut Text, With<ToggleLabel>>,
    mut last: Local<Option<bool>>,
) {
    if last.is_some_and(|v| v == view.top_down) {
        return;
    }
    *last = Some(view.top_down);
    let s = if view.top_down { "返回等距" } else { "俯视图" };
    for mut t in &mut q {
        t.0 = s.to_string();
    }
}

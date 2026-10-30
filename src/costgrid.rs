//! 地图 → 可寻路网格 —— 对等移植。
//!
//! 来源：`I:\bevy\RA2REASM\src\tools\wwread.cpp` 的
//! `buildTerrainCells`（1163-1660，取其"每格 terrainType"那一段）+
//! `landTypeForTmp` / `zoneIndex` / `parseLandSpeeds` / `buildCostGrid`（3158-3285）。
//!
//! 链条：地图的全局砖号 →（theater 的 TileNumbering）→ tileset + 段内序号 →
//! 该 tileset 的 TMP 里那一块的 `terrainType` →（`landTypeForTmp`）→ land type →
//! （rules 的 `[<land>]` 段，按移动 zone 取百分比）→ 通行代价。
//! 中间夹一次 `fix_tiles`（LAT / ramp 修正），它会改写砖号，所以必须在上面的链条之前跑。
//!
//! 【本轮未搬】上游 `TerrainCell` 还带渲染字段（`pixels` / `zbuf` / `extra` / `palette` /
//! `depthTest` …），以及 `buildTerrainCells` 里喂它们的那半段（TMP 像素、`isoLevel` 高度层调色板、
//! `SetImages.variants` / `extra` / `zDatas`）。那些是**地形渲染**要的，Rust 侧渲染还没做，
//! 所以这里只取寻路真正读的 `rx` / `ry` / `terrainType`（外加 ramp / z，供修正与桥判定）。
//! 要补的话位置很明确：`SetImages` 里加像素表，`TerrainCell` 里补回字段即可。

use std::collections::HashMap;

use crate::ini::{strtol10, Ini};
use crate::mapfile::MapFile;
use crate::nav::CostGrid;
use crate::mix::AssetIndex;
use crate::theater::{tile_asset_name, Theater, TileNumbering};
use crate::tilefix::{fix_tiles, TileFixInput};
use crate::tmp::parse_tmp;

// ============================================================================ land 类型
// 上游 wwread.cpp:3156-3162

/// rules.ini 的 Land Characteristics 段给出每种地形在每个移动 zone 下的速度百分比
/// （0% = 不可通行）。下面的顺序**也是** TMP `TerrainType` 字节的索引顺序。
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(usize)]
pub enum LandIndex {
    Clear = 0,
    Rough,
    Road,
    Water,
    Rock,
    Wall,
    Tiberium,
    Weeds,
    Beach,
    Ice,
    Railroad,
    Tunnel,
}

pub const LAND_COUNT: usize = 12;

pub const LAND_NAMES: [&str; LAND_COUNT] = [
    "Clear", "Rough", "Road", "Water", "Rock", "Wall", "Tiberium", "Weeds", "Beach", "Ice",
    "Railroad", "Tunnel",
];

/// zones：Foot, Track, Wheel, Float, Hover, Amphibious
#[derive(Clone, Copy)]
pub struct LandSpeed {
    pub pct: [i32; 6],
    pub present: bool,
}

impl Default for LandSpeed {
    fn default() -> Self {
        LandSpeed {
            pct: [100; 6],
            present: false,
        }
    }
}

/// 引擎的映射（`IsometricTileTypeClass::Land_Type`）：TMP 地形字节 → land 类型。
/// 与参考实现以及真实地图上见到的字节值一致。
pub fn land_type_for_tmp(b: i32) -> LandIndex {
    match b {
        0x0 => LandIndex::Clear,
        0x1 | 0x2 | 0x3 | 0x4 => LandIndex::Ice,
        0x5 => LandIndex::Tunnel,
        0x6 => LandIndex::Railroad,
        0x7 | 0x8 | 0xf => LandIndex::Rock,
        0x9 => LandIndex::Water,
        0xa => LandIndex::Beach,
        0xb | 0xc => LandIndex::Road,
        0xd => LandIndex::Clear,
        0xe => LandIndex::Rough,
        _ => LandIndex::Clear,
    }
}

/// 上游 `zoneIndex`。未识别的名字一律 Track（车辆默认）。
pub fn zone_index(z: &str) -> usize {
    match z {
        "Foot" | "foot" => 0,
        "Wheel" | "wheel" => 2,
        "Float" | "float" => 3,
        "Hover" | "hover" => 4,
        "Amphibious" | "amphibious" => 5,
        _ => 1,
    }
}

/// 上游 `parseLandSpeeds`。
pub fn parse_land_speeds(rules: &Ini, out: &mut [LandSpeed; LAND_COUNT]) {
    const ZONES: [&str; 6] = ["Foot", "Track", "Wheel", "Float", "Hover", "Amphibious"];
    for (i, name) in LAND_NAMES.iter().enumerate() {
        let Some(s) = rules.get(name) else { continue };
        out[i].present = true;
        for (k, key) in ZONES.iter().enumerate() {
            if let Some(v) = s.value(key) {
                out[i].pct[k] = strtol10(v).0 as i32;
            }
        }
    }
}

// ============================================================================ tileset 的 TMP 字段
// 上游 buildTerrainCells 里的 imagesFor / resolveTile（1163-1295）

/// 一个 tileset 里一块砖的图像信息。**只保留寻路链读到的字段**（见模块头说明）。
#[derive(Default, Clone)]
pub struct TileImages {
    /// 砖图尺寸，取自**第一个**成功解析的变体（上游 `TileImages::w/h`）
    pub w: i32,
    pub h: i32,
    /// 每个存在的变体有多少 cell（对应上游 `variants[v].size()`）
    pub variant_cells: Vec<usize>,
    /// 每个变体里每个 cell 的像素（对应上游 `variants[v][cell]`）。
    /// 🔴 **空 = 该 cell 不存在**（上游 `cells.push_back(c.present ? c.pixels : {})`）——
    /// 它的**长度**就是上游的 `px.size()`，pass 2 那道尺寸校验读的正是它。
    /// TMP 每 cell 都是 `cx*cy`；SHP 每帧各自大小。
    pub variant_pixels: Vec<Vec<Vec<u8>>>,
    /// 第一个成功解析的变体里，每 cell 的 RampType（按 `subTile` 索引）
    pub ramps: Vec<u8>,
    pub heights: Vec<u8>,
    pub terrain_types: Vec<u8>,
    /// 每 cell 的 z 面（**按 cell 索引，不是按变体**）；该 cell 没有 z 面时为空。
    pub z_datas: Vec<Vec<u8>>,
    /// extra 矩形**相对本 cell 自身左上角**的偏移与尺寸（按 cell 索引，取第一个变体）。
    ///
    /// 注意是「相对本 cell」不是「相对整砖」：`x_extra - x` 已经减掉了这一片 cell 的
    /// 绘制偏移。要还原成砖内坐标必须再加 `xs/ys`（Cliff09 的 cell1 是
    /// `cell(30,15) + extra(0,-60) = 砖(30,-45)`）。TMP 原始值 `x_extra` 才是相对砖的。
    pub ex_x: Vec<i32>,
    pub ex_y: Vec<i32>,
    pub ex_w: Vec<i32>,
    pub ex_h: Vec<i32>,
    /// 每 cell 的绘制偏移，就是 TMP 头里那对 `x/y`（2x2 组里是 (0,0)/(30,15)/(-30,15)/(0,30)）。
    /// 上游 `TileImages` 没有这一对；本项目需要它来判「这块砖画在整格还是半个菱形上」。
    pub xs: Vec<i32>,
    pub ys: Vec<i32>,
    /// 每个变体里每个 cell 的 extra 像素（悬崖立面 / 坡道面 / 桥面）
    pub extra: Vec<Vec<Vec<u8>>>,
    pub extra_z: Vec<Vec<Vec<u8>>>,
}

#[derive(Default, Clone)]
pub struct SetImages {
    /// 按段内砖序号索引
    pub tiles: Vec<TileImages>,
}

/// gamemd 按格坐标纯函数式地挑地面砖变体（`CellClass` 的变体选择 0x4814F0）：
/// 变体多于 4 个的 set 按 `(x&7, y&7)` 查一张 8x8 格子；更少的用引擎内置的 4x4 拉丁方，
/// 按 `(x&3, y&3)` 查。格子是为了打破重复贴图的观感。
/// 上游 `pickTileVariant`（wwread.cpp:1066-1083），表照抄。
fn pick_tile_variant(rx: i32, ry: i32, count: usize) -> usize {
    if count <= 1 {
        return 0;
    }
    let (x, y) = (rx, ry);
    let v: i32 = if count > 4 {
        // (3*x + 2*y) & 7 —— 相邻格（含格缝两侧）不会取到同一个值。
        const K_LAT: [i32; 64] = [
            0, 3, 6, 1, 4, 7, 2, 5, 2, 5, 0, 3, 6, 1, 4, 7, 4, 7, 2, 5, 0, 3, 6, 1, 6, 1, 4, 7,
            2, 5, 0, 3, 0, 3, 6, 1, 4, 7, 2, 5, 2, 5, 0, 3, 6, 1, 4, 7, 4, 7, 2, 5, 0, 3, 6, 1,
            6, 1, 4, 7, 2, 5, 0, 3,
        ];
        K_LAT[((x & 7) + (y & 7) * 8) as usize]
    } else {
        // 引擎内置的 4x4 拉丁方（0x81CCA8）。
        const K_PAT: [i32; 16] = [0, 1, 2, 3, 3, 2, 1, 0, 2, 3, 0, 1, 1, 0, 3, 2];
        K_PAT[(((y & 3) << 2) | (x & 3)) as usize]
    };
    let v = v as usize;
    if v >= count {
        v % count
    } else {
        v
    }
}

/// 从 MIX 里把每个启用 tileset 的 TMP 读出来，取每块的 ramp / height / terrainType。
///
/// 上游 `imagesFor` 会一路枚举 `variant = 0..=26` 直到取不到文件；**NewUrban 特殊**：
/// `.ubn` 取不到时回落到 `.tem`（urbannmd 的砖只出了 temperate 版）。
pub fn build_set_images(idx: &AssetIndex, th: &Theater) -> HashMap<i32, SetImages> {
    let mut cache: HashMap<i32, SetImages> = HashMap::new();
    for (set_idx, ts) in th.tile_sets.iter().enumerate() {
        if !ts.enabled {
            continue;
        }
        let set_idx = set_idx as i32;
        let mut si = SetImages {
            tiles: vec![TileImages::default(); ts.tiles_in_set.max(0) as usize],
            ..Default::default()
        };
        for j in 0..ts.tiles_in_set {
            let ti = &mut si.tiles[j as usize];
            let mut parsed_any = false;
            for v in 0..=26 {
                let nm = tile_asset_name(th, ts, j, v);
                let bytes = match idx.read(&nm) {
                    Ok(b) => b,
                    Err(_) => {
                        if th.suffix == "ubn" {
                            match nm.rfind(".ubn") {
                                Some(p) => {
                                    let alt = format!("{}.tem", &nm[..p]);
                                    match idx.read(&alt) {
                                        Ok(b) => b,
                                        Err(_) => break,
                                    }
                                }
                                None => break,
                            }
                        } else {
                            break;
                        }
                    }
                };
                if bytes.is_empty() {
                    break;
                }
                if let Some(tmp) = parse_tmp(&bytes) {
                    // 🔴 上游这里有个**真的不对称**，必须照抄：
                    //     `ti.w = tmp.cx; ti.h = tmp.cy;` 写在变体循环里**没有** `!parsedAny` 守卫，
                    //     所以**尺寸会被最后一个成功解析的变体覆盖**；而 ramp/height/terrainType
                    //     只在第一个变体时设（见下）。pass 2 拿这个尺寸做"cell 是否真的存在"的校验。
                    //     我先前把它改成"只取第一个"，看着更合理，却让那道校验形同虚设。
                    ti.w = tmp.cx;
                    ti.h = tmp.cy;
                    if !parsed_any {
                        // 上游：ramp/height/terrainType、zDatas、exX/exY/exW/exH 都**只取第一个变体**
                        ti.ramps.clear();
                        ti.heights.clear();
                        ti.terrain_types.clear();
                        ti.z_datas.clear();
                        ti.ex_x.clear();
                        ti.ex_y.clear();
                        ti.ex_w.clear();
                        ti.ex_h.clear();
                        ti.xs.clear();
                        ti.ys.clear();
                        for c in &tmp.tiles {
                            ti.ramps.push(c.ramp_type as u8);
                            ti.heights.push(c.height as u8);
                            ti.terrain_types.push(c.terrain_type as u8);
                            ti.z_datas.push(c.z_data.clone());
                            ti.ex_x.push(c.x_extra - c.x);
                            ti.ex_y.push(c.y_extra - c.y);
                            ti.ex_w.push(c.cx_extra);
                            ti.ex_h.push(c.cy_extra);
                            ti.xs.push(c.x);
                            ti.ys.push(c.y);
                        }
                    }
                    ti.variant_cells.push(tmp.tiles.len());
                    // 🔴 上游 `cells.push_back(c.present ? c.pixels : std::vector<uint8_t>())`
                    //    （wwread.cpp:1219）：**cell 不存在时像素是空 vector**，size 为 0。
                    //    `TmpTile::pixels` 对不存在的 cell 本来就是空的，直接搬即等价。
                    ti.variant_pixels
                        .push(tmp.tiles.iter().map(|c| c.pixels.clone()).collect());
                    // extra / extraZ 是**每个变体都 push**（与 zDatas 不同）
                    ti.extra.push(
                        tmp.tiles
                            .iter()
                            .map(|c| if c.has_extra_data { c.extra.clone() } else { Vec::new() })
                            .collect(),
                    );
                    ti.extra_z.push(
                        tmp.tiles
                            .iter()
                            .map(|c| if c.has_extra_data { c.extra_z.clone() } else { Vec::new() })
                            .collect(),
                    );
                    parsed_any = true;
                } else if let Some(shp) = crate::shp::parse_shp(&bytes) {
                    // 🔴 上游在 TMP 失败时试 SHP，成功就把**整份 SHP 的帧集合**当成**一个**变体
                    // （每帧是一个 cell）。少了这一支，变体的**数量**就与上游不同，
                    // `pick_tile_variant` 会选到不同的项，尺寸校验也就永不触发。
                    if shp.frames.is_empty() {
                        break;
                    }
                    if ti.w == 0 {
                        ti.w = shp.frames[0].w as i32;
                        ti.h = shp.frames[0].h as i32;
                    }
                    if !parsed_any {
                        ti.z_datas = vec![Vec::new(); shp.frames.len()];
                        for _ in &shp.frames {
                            ti.ramps.push(0);
                            ti.heights.push(0);
                            ti.terrain_types.push(0);
                            ti.xs.push(0);
                            ti.ys.push(0);
                        }
                    }
                    ti.variant_cells.push(shp.frames.len());
                    ti.variant_pixels
                        .push(shp.frames.iter().map(|f| f.pixels.clone()).collect());
                    ti.extra.push(shp.frames.iter().map(|_| Vec::new()).collect());
                    ti.extra_z.push(shp.frames.iter().map(|_| Vec::new()).collect());
                    parsed_any = true;
                } else {
                    // TMP 与 SHP 都不认 ⇒ 上游 `!ok` 就 `break`，那些砖的 `variants` 为空、
                    // pass 2 把它们记成 `missing` 并跳过。
                    break;
                }
            }
        }
        cache.insert(set_idx, si);
    }
    cache
}

// ============================================================================ 地形层
// 上游 buildTerrainCells 的 pass 1 / fixTiles / pass 2（1274-1420）

#[derive(Debug, Clone, Default)]
pub struct TerrainCell {
    pub dx: i32,
    pub dy: i32,
    pub rx: i32,
    pub ry: i32,
    /// 以砖为单位的地面高度
    pub z: i32,
    /// TMP 的 RampType（0 = 平）
    pub ramp_type: u8,
    /// TMP 的 TerrainType（地形特征索引）
    pub terrain_type: u8,
    /// 这一格是桥面
    pub bridge: bool,
    pub level: i32,
    /// 这块砖的像素（调色板索引，0 = 透明）。**渲染用**；寻路不读。
    pub pixels: Vec<u8>,
    /// 砖尺寸（= `ti.w/ti.h`，标准 60x30）
    pub w: i32,
    pub h: i32,
    /// 屏幕左上角（上游 `ox/oy`：`dx*pitchX`、`(dy - z)*pitchY`）
    pub ox: i32,
    pub oy: i32,

    // --- 渲染字段（上游 `TerrainCell` 的那一半；寻路不读）---------------------------
    /// 主菱形的逐像素 z（空 = 这块砖没有 z 面）
    pub zbuf: Vec<i16>,
    pub depth_test: bool,
    pub write_depth: bool,
    /// extra 矩形（悬崖立面 / 坡道面 / 桥面），尺寸 `ex_w x ex_h` 像素。
    pub has_extra: bool,
    pub extra: Vec<u8>,
    pub extra_z: Vec<i16>,
    /// extra 相对**本 cell**左上角的偏移。砖内位置 = `cell_x + ex_x`（不是 `ex_x` 单独）。
    pub ex_x: i32,
    pub ex_y: i32,
    pub ex_w: i32,
    pub ex_h: i32,
    /// 这块砖在 TMP 里的 cell 绘制偏移（头上那对 `x/y`）。
    /// `x = ±30` = 这块砖画在**半个菱形**上；`x = 0` = 整格居中。
    pub cell_x: i32,
    pub cell_y: i32,
    pub extra_depth_test: bool,
    /// 唯一标识这块砖：`set_idx<<32 | in_set<<16 | variant<<8 | cell`。
    /// 3D 化后图集靠它去重（渲染只认它 + `level`）。
    pub tile_id: u64,
    /// 这块砖属于 Cliff 家族（`set_name` 以 `cliff` 开头，不分大小写）。
    ///
    /// 【用途】只有悬崖的撑边砖才需要在渲染时补半个菱形把平台边缘抹平。
    /// 斜坡砖（`RAMPxx`）同样带 extra，那一片是坡面，不能当悬崖补，所以判据里要排除它。
    pub cliff: bool,
    /// 这块砖属于 RAMP 家族（`set_name` 以 `ramp` 开头，不分大小写）。
    ///
    /// 【用途】斜坡砖的主图是斜面的**投影**（平行四边形，常只有十几行像素），不是完整
    /// 菱形。图集不能按菱形四角去采它，必须走「逆推成正视图」那条路。只靠 TMP 的
    /// `RampType` 字段不够 —— 地图里大量 `RAMPxx` 砖的那个字段是 0。
    pub ramp_set: bool,
}

#[derive(Default)]
pub struct TerrainBuild {
    /// 画家序，从后到前
    pub cells: Vec<TerrainCell>,
    /// 每网格格子的砖号，已过 LAT / ramp 修正
    pub tile_num: Vec<i32>,
    pub ramp_at: Vec<u8>,
    pub height_at: Vec<u8>,
    /// 画了砖的地方为 1
    pub touched: Vec<u8>,
    pub missing: usize,
    pub blank: usize,
    pub miss_gap: usize,
    pub miss_no_var: usize,
    pub miss_size: usize,
    /// set → (用到, 空白)
    pub set_use: HashMap<i32, (usize, usize)>,
}

/// 上游 `resolveTile`：由（全局砖号, subTile）查出这一块的 ramp / height。
fn resolve_tile(
    tn: i32,
    sub_tile: u8,
    th: &Theater,
    numbering: &TileNumbering,
    sets: &HashMap<i32, SetImages>,
) -> (u8, u8) {
    let mut out_ramp = 0u8;
    let mut out_height = 0u8;
    if tn < 0 || tn as usize >= numbering.tile_to_set.len() {
        return (out_ramp, out_height);
    }
    let set_idx = numbering.tile_to_set[tn as usize];
    if set_idx < 0 || set_idx as usize >= th.tile_sets.len() {
        return (out_ramp, out_height);
    }
    if !th.tile_sets[set_idx as usize].enabled {
        return (out_ramp, out_height);
    }
    let in_set = tn - numbering.set_first_tile[set_idx as usize];
    if in_set < 0 {
        return (out_ramp, out_height);
    }
    let Some(si) = sets.get(&set_idx) else {
        return (out_ramp, out_height);
    };
    if in_set as usize >= si.tiles.len() {
        return (out_ramp, out_height);
    }
    let ti = &si.tiles[in_set as usize];
    if ti.variant_cells.is_empty() {
        return (out_ramp, out_height);
    }
    let mut cell = sub_tile as usize;
    let first = ti.variant_cells[0];
    if cell >= first && first > 0 {
        cell = first - 1;
    }
    if cell < ti.ramps.len() {
        out_ramp = ti.ramps[cell];
    }
    if cell < ti.heights.len() {
        out_height = ti.heights[cell];
    }
    (out_ramp, out_height)
}

/// 解析整个地面砖层。与上游 `buildTerrainCells` 在**寻路所需的部分**上一致。
pub fn build_terrain_cells(
    map: &MapFile,
    th: &Theater,
    numbering: &TileNumbering,
    sets: &HashMap<i32, SetImages>,
    no_fix_tiles: bool,
) -> TerrainBuild {
    let grid_w = 2 * map.width - 1;
    let grid_cells = grid_w as usize * map.height as usize;

    let mut out = TerrainBuild {
        tile_num: vec![0; grid_cells],
        ramp_at: vec![0; grid_cells],
        height_at: vec![0; grid_cells],
        touched: vec![0; grid_cells],
        ..Default::default()
    };

    // 第 1 趟：地图原始砖号 + 每格的 ramp / height。
    for row in 0..map.height {
        for dx in 0..grid_w {
            let Some(t) = map.grid_at(dx, row) else { continue };
            let k = dx as usize * map.height as usize + row as usize;
            out.tile_num[k] = t.tile_num;
            let (r, h) = resolve_tile(t.tile_num, t.sub_tile, th, numbering, sets);
            out.ramp_at[k] = r;
            out.height_at[k] = h;
        }
    }

    // 引擎在地图加载后立刻重算 LAT 系统与坡道平滑。
    if !no_fix_tiles {
        fix_tiles(TileFixInput {
            width: map.width,
            height: map.height,
            tile_num: &mut out.tile_num,
            ramp: &out.ramp_at,
            set_first_tile: &numbering.set_first_tile,
            tile_to_set: &numbering.tile_to_set,
            ids: &th.fixup,
        });
    }

    // 按修正后的结果重读 ramp / height。
    for row in 0..map.height {
        for dx in 0..grid_w {
            let Some(t) = map.grid_at(dx, row) else { continue };
            let k = dx as usize * map.height as usize + row as usize;
            let (r, h) = resolve_tile(out.tile_num[k], t.sub_tile, th, numbering, sets);
            out.ramp_at[k] = r;
            out.height_at[k] = h;
        }
    }

    // 第 2 趟：每块地面砖产出一个 cell，从后到前（画家序）。
    for dy in 0..2 * map.height {
        for dx in 0..grid_w {
            if ((dx + dy) & 1) != 0 {
                continue; // dx 与 dy 同奇偶
            }
            let Some(t) = map.grid_at(dx, dy / 2) else { continue };
            if t.dy != dy {
                continue;
            }

            let k = dx as usize * map.height as usize + (dy / 2) as usize;
            let tn = out.tile_num[k];
            if tn < 0 || tn as usize >= numbering.tile_to_set.len() {
                continue;
            }
            let set_idx = numbering.tile_to_set[tn as usize];
            if set_idx < 0 || set_idx as usize >= th.tile_sets.len() {
                continue;
            }
            let in_set = tn - numbering.set_first_tile[set_idx as usize];
            if !th.tile_sets[set_idx as usize].enabled || in_set < 0 {
                continue;
            }

            {
                let e = out.set_use.entry(set_idx).or_insert((0, 0));
                e.0 += 1;
            }
            let Some(si) = sets.get(&set_idx) else {
                out.missing += 1;
                out.miss_gap += 1;
                continue;
            };
            if in_set as usize >= si.tiles.len() {
                out.missing += 1;
                out.miss_gap += 1;
                continue;
            }
            let ti = &si.tiles[in_set as usize];
            if ti.variant_cells.is_empty() {
                out.missing += 1;
                out.miss_no_var += 1;
                continue;
            }
            let mut cell = t.sub_tile as usize;
            let first = ti.variant_cells[0];
            if cell >= first && first > 0 {
                cell = first - 1;
            }
            // 🔴 上游在这里还有一道校验（wwread.cpp:1362）：
            //     `if (tw <= 0 || thh <= 0 || int(px.size()) < tw * thh) { ++missing; continue; }`
            // `tw/thh` = `ti.w/ti.h`（**最后一个**成功解析变体的 cx/cy，不是第一个）；
            // `px` = 被 `pick_tile_variant` 选中那个变体的第 `cell` 块像素。TMP 砖里
            // `present=false` 的 cell 其像素是空 vector（见 build_set_images 的填法），
            // 所以这道校验其实等价于"这个 cell 必须在砖里真的存在"。
            // arivruns 吃掉 643 格、duel 吃掉 52 格都出自这里，漏掉就会被当成可通行平地。
            let tw = ti.w;
            let thh = ti.h;
            let variant = pick_tile_variant(t.rx, t.ry, ti.variant_cells.len());
            let px: &[u8] = ti
                .variant_pixels
                .get(variant)
                .and_then(|v| v.get(cell))
                .map(|v| v.as_slice())
                .unwrap_or(&[]);
            let vbytes = px.len();
            if tw <= 0 || thh <= 0 || vbytes < (tw as usize) * (thh as usize) {
                out.missing += 1;
                out.miss_size += 1;
                continue;
            }
            out.touched[k] = 1;

            let terrain_type = if cell < ti.terrain_types.len() {
                ti.terrain_types[cell]
            } else {
                0
            };
            let z = t.z as i32;
            // `tileZBase`：这块砖所在的整条等距线上的 z 基准
            let z_base = ((t.rx + t.ry) * 15) as i16;
            // z 面：按 cell 索引（不是按变体）；只有它够长才拿它做深度
            let zd: Option<&Vec<u8>> = ti.z_datas.get(cell);
            let use_z = zd.map(|v| v.len() >= px.len()).unwrap_or(false);
            let mut zbuf: Vec<i16> = Vec::new();
            if use_z {
                let v = zd.unwrap();
                zbuf = (0..px.len()).map(|i| z_base - (v[i] as i16)).collect();
            }
            let mut opaque = px.iter().filter(|&&c| c != 0).count();
            // extra 矩形（照抄 wwread.cpp:1399-1423）
            let mut has_extra = false;
            let mut extra: Vec<u8> = Vec::new();
            let mut extra_z: Vec<i16> = Vec::new();
            let (mut ex_x, mut ex_y, mut ex_w, mut ex_h) = (0, 0, 0, 0);
            let mut extra_depth_test = false;
            if cell < ti.ex_x.len()
                && ti.ex_w[cell] > 0
                && ti.ex_h[cell] > 0
                && variant < ti.extra.len()
                && cell < ti.extra[variant].len()
            {
                let ex = &ti.extra[variant][cell];
                let ew = ti.ex_w[cell];
                let eh = ti.ex_h[cell];
                if ex.len() >= (ew as usize) * (eh as usize) {
                    let use_xz = variant < ti.extra_z.len()
                        && cell < ti.extra_z[variant].len()
                        && ti.extra_z[variant][cell].len() >= ex.len();
                    for &ch in ex {
                        if ch != 0 {
                            opaque += 1;
                        }
                    }
                    has_extra = true;
                    extra = ex.clone();
                    ex_w = ew;
                    ex_h = eh;
                    ex_x = ti.ex_x[cell];
                    ex_y = ti.ex_y[cell];
                    extra_depth_test = use_xz;
                    if use_xz {
                        let ez = &ti.extra_z[variant][cell];
                        extra_z = (0..extra.len()).map(|i| z_base - (ez[i] as i16)).collect();
                    }
                }
            }
            if opaque == 0 {
                out.blank += 1;
                out.set_use.entry(set_idx).or_insert((0, 0)).1 += 1;
            }
            out.cells.push(TerrainCell {
                dx,
                dy,
                rx: t.rx,
                ry: t.ry,
                z,
                ramp_type: out.ramp_at[k],
                terrain_type,
                bridge: set_idx == th.bridge_set
                    || set_idx == th.train_bridge_set
                    || set_idx == th.wood_bridge_set,
                level: z.clamp(0, 18),
                pixels: px.to_vec(),
                w: tw,
                h: thh,
                // 上游 `c.ox = dx * pitchX; c.oy = (dy - int(t->z)) * pitchY;`（30 / 15）
                ox: dx * 30,
                oy: (dy - z) * 15,
                zbuf,
                depth_test: use_z,
                write_depth: use_z,
                has_extra,
                extra,
                extra_z,
                ex_x,
                ex_y,
                ex_w,
                ex_h,
                cell_x: ti.xs.get(cell).copied().unwrap_or(0),
                cell_y: ti.ys.get(cell).copied().unwrap_or(0),
                extra_depth_test,
                tile_id: ((set_idx as u64) << 32)
                    | ((in_set as u64) << 16)
                    | ((variant as u64) << 8)
                    | (cell as u64),
                cliff: th
                    .tile_sets
                    .get(set_idx.max(0) as usize)
                    .is_some_and(|s| s.set_name.len() >= 5 && s.set_name[..5].eq_ignore_ascii_case("cliff")),
                ramp_set: th
                    .tile_sets
                    .get(set_idx.max(0) as usize)
                    .is_some_and(|s| s.set_name.len() >= 4 && s.set_name[..4].eq_ignore_ascii_case("ramp")),
            });
        }
    }
    out
}

// ============================================================================ cost grid
// 上游 buildCostGrid（3210-3249）

/// 从 MIX 里读第一个能读到的 ini（上游同样是按候选名顺序试）。
pub fn read_rules_ini(idx: &AssetIndex, candidates: &[&str]) -> Ini {
    for nm in candidates {
        if let Ok(b) = idx.read(nm) {
            return Ini::parse(&String::from_utf8_lossy(&b));
        }
    }
    Ini::default()
}

/// `build_cost_grid` 的分解统计，用来把"地形不可通行"和"建筑占地"两段的数字分开对照。
#[derive(Debug, Default, Clone)]
pub struct CostStats {
    pub blocked: usize,
    pub plain: usize,
    /// 地形百分比为 0 的格数
    pub terrain_blocked: usize,
    /// 建筑铺到的格数（已剔除越界；含互相重叠）
    pub building_cells: usize,
    /// 建筑把**本来非零**的格清零的次数
    pub building_cleared: usize,
    /// (fw, fh) → 栋数，用来与 C++ 侧逐项对照 Foundation 解析
    pub building_fp_hist: HashMap<(i32, i32), usize>,
}

/// `sscanf("%d")` 在位置 `from` 处的一次尝试：返回 `(值, 下一个位置)`，认不出数字给 `None`。
/// `%d` 会跳过前导空白，所以这里复用 `strtol10` 的跳过逻辑。
fn sscanf_int(s: &str, from: usize) -> Option<(i32, usize)> {
    let rest = s.get(from..)?;
    let (v, used) = strtol10(rest);
    if used == 0 {
        None
    } else {
        Some((v as i32, from + used))
    }
}

/// 上游 `buildCostGrid`。返回 `(网格, 统计)`。
pub fn build_cost_grid(
    map: &MapFile,
    rules: &Ini,
    art: &Ini,
    tb: &TerrainBuild,
    zone: &str,
) -> (CostGrid, CostStats) {
    let mut ls = [LandSpeed::default(); LAND_COUNT];
    parse_land_speeds(rules, &mut ls);
    let zi = zone_index(zone);

    // 格子栅格 (rx,ry) 比 map.width/height 大：它跨 [1, W+H-1]。
    // 按**真正存在**的格子定尺寸，其余位置标成不可通行。
    let mut max_x = 0;
    let mut max_y = 0;
    for c in &tb.cells {
        max_x = max_x.max(c.rx);
        max_y = max_y.max(c.ry);
    }
    let mut out = CostGrid {
        w: max_x + 1,
        h: max_y + 1,
        cost: Vec::new(),
        cut_corners: false,
    };
    let n = out.w as usize * out.h as usize;
    out.cost = vec![0u8; n];

    let mut terr = vec![0u8; n];
    let mut present = vec![0u8; n];
    for c in &tb.cells {
        if c.rx < 0 || c.ry < 0 || c.rx >= out.w || c.ry >= out.h {
            continue;
        }
        let i = c.ry as usize * out.w as usize + c.rx as usize;
        terr[i] = c.terrain_type;
        present[i] = 1;
    }

    let mut stats = CostStats::default();
    for i in 0..n {
        if present[i] == 0 {
            continue; // 这里没有砖：不属于地图
        }
        let lt = land_type_for_tmp(terr[i] as i32) as usize;
        let pct = if ls[lt].present { ls[lt].pct[zi] } else { 100 };
        if pct <= 0 {
            out.cost[i] = 0;
            stats.blocked += 1;
            stats.terrain_blocked += 1;
        } else {
            out.cost[i] = (100 / pct).clamp(1, 255) as u8;
            if out.cost[i] == 1 {
                stats.plain += 1;
            }
        }
    }

    // 地图里建筑的静态阻挡（art.ini 的 Foundation 占地）。
    for s in &map.structures {
        let mut fw = 1i32;
        let mut fh = 1i32;
        if let Some(a) = art.get(&s.name) {
            if let Some(f) = a.value("foundation") {
                // 🔴 上游是 `sscanf(f, "%dx%d", &fw, &fh)`，必须照抄它的**部分匹配**语义：
                // 第一个 `%d` 一匹配到就赋值；后面的字面量 `'x'` 不匹配就停下，
                // **已赋的值保留、未赋的保持初值 1**。所以 `Foundation=3` 得到 `fw=3, fh=1`
                // —— 既不是 3x3 也不是 1x1。真实地图上确实有这样的条目
                // （hillbtwn 的 17 栋），用 `split('x')` 把它退化成 1x1 会少算 34 格。
                if let Some((v, next)) = sscanf_int(f, 0) {
                    fw = v;
                    if f.get(next..next + 1) == Some("x") {
                        if let Some((v2, _)) = sscanf_int(f, next + 1) {
                            fh = v2;
                        }
                    }
                }
                if fw <= 0 || fh <= 0 {
                    fw = 1;
                    fh = 1;
                }
            }
        }
        *stats.building_fp_hist.entry((fw, fh)).or_insert(0) += 1;
        for dy in 0..fh {
            for dx in 0..fw {
                let rx = s.rx + dx;
                let ry = s.ry + dy;
                if !out.in_bounds(rx, ry) {
                    continue;
                }
                let i = ry as usize * out.w as usize + rx as usize;
                stats.building_cells += 1;
                if out.cost[i] != 0 {
                    stats.building_cleared += 1;
                    stats.blocked += 1;
                }
                out.cost[i] = 0;
            }
        }
    }
    (out, stats)
}

//! Westwood `.map` 容器解析 —— 对等移植。
//!
//! 来源：`I:\bevy\RA2REASM\src\compat\mapfile.cpp` / `mapfile.h`。
//!
//! 一个 `.map` 是纯 INI（不是 MIX 容器）。地形本体在 `[IsoMapPack5]`：一串编号 key
//! 承载 base64，拼起来是 LZO 分块的流，解出来是**每格 11 字节**的记录。覆盖物
//! （树、矿石、路）在 `[OverlayPack]` / `[OverlayDataPack]`，两条 LCW 流，按 `rx + 512*ry` 索引。

use crate::compress::decode_chunked;
use crate::ini::{ini_bool, strtol10, Ini};
use crate::lighting::{parse_lighting, MapLighting};

/// 一格格坐标的上限：`rx`/`ry` 都在 0..=511（上游 mapfile.cpp:184 的 `rx > 511 || ry > 511`）。
const RC_LIMIT: u16 = 511;

/// `IsoMapPack5` 里的一条等距格。
///
/// `rx`/`ry` 是**游戏等距坐标**，同时决定屏幕位置和覆盖物查表；`dx`/`dy` 是它在**渲染网格**里
/// 的位置，其中 `dy` 数的是**菱形行**（每个地图行两行），于是 dx 与 dy 永远同奇偶 ——
/// 这才是网格无缝拼合的原因。
#[derive(Debug, Clone, Copy, Default)]
pub struct IsoTile {
    pub rx: i32,
    pub ry: i32,
    pub dx: i32,
    pub dy: i32,
    /// 全剧 tile 号，要过 theater 才能解析。
    pub tile_num: i32,
    pub sub_tile: u8,
    /// 高度层，把精灵按 `z * TileHeight/2` 上移。
    pub z: u8,
    pub ice_growth: u8,
    /// 包里没有这一格；替换成了 Level-0 的空地砖。
    pub empty: bool,
}

/// `[Terrain]`：KEY 是打包位置 `ry * 1000 + rx`，VALUE 是类型名（树、矿石柱……）。
/// 地形对象按名字查，不按行号。
#[derive(Debug, Clone, Default)]
pub struct MapTerrain {
    pub name: String,
    pub rx: i32,
    pub ry: i32,
}

/// `[Structures]` / `[Units]` / `[Infantry]` / `[Aircraft]` 共用的行形状：
/// `Owner,Name,Health,Rx,Ry,Facing,...`
#[derive(Debug, Clone, Default)]
pub struct MapStructure {
    pub owner: String,
    pub name: String,
    pub health: i32,
    pub rx: i32,
    pub ry: i32,
    pub facing: i32,
    pub on_bridge: bool,
}

/// 地图上的车辆。RA2 的车辆（和飞机）是体素而不是 SHP，所以这张表喂的是体素路径；
/// （少见的）SHP 单位和所有步兵刻意留到后面单独过。
#[derive(Debug, Clone, Default)]
pub struct MapUnit {
    pub owner: String,
    pub name: String,
    pub rx: i32,
    pub ry: i32,
    /// 游戏朝向，0..255。
    pub facing: i32,
    pub on_bridge: bool,
}

/// `[Infantry]`：`Owner,Name,Health,Rx,Ry,SubCell,Facing,Mission,Tag,Veterancy,Group,OnBridge`。
/// 站立帧取自 art 序列的 Ready/Guard，精灵是 art 段名（rules 的 `Image=` 指向），
/// SubCell 把格心挪四分之一格。
#[derive(Debug, Clone, Default)]
pub struct MapInfantry {
    pub owner: String,
    pub name: String,
    pub rx: i32,
    pub ry: i32,
    pub sub_cell: i32,
    /// 游戏朝向，0..255。
    pub facing: i32,
    pub on_bridge: bool,
}

/// `[Aircraft]`：`Owner,Name,Health,Rx,Ry,Facing,...`；`onBridge` 是倒数第 4 个字段。
/// 飞机是体素，和 `[Units]` 一样。
#[derive(Debug, Clone, Default)]
pub struct MapAircraft {
    pub owner: String,
    pub name: String,
    pub rx: i32,
    pub ry: i32,
    /// 游戏朝向，0..255。
    pub facing: i32,
    pub on_bridge: bool,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct MapOverlay {
    pub rx: i32,
    pub ry: i32,
    /// `0xFF` 表示"没有覆盖物"，会被过滤掉。
    pub id: u8,
    pub data: u8,
    pub dx: i32,
    pub dy: i32,
    pub z: u8,
}

#[derive(Debug, Clone, Default)]
pub struct MapFile {
    pub name: String,
    /// `[Basic] Theater`，例如 `"TEMPERATE"`。
    pub theater: String,
    pub new_theater: bool,

    pub width: i32,
    pub height: i32,

    /// 地图自己的 `[Lighting]` 块；缺省时用引擎默认值。
    pub lighting: MapLighting,

    /// `[Map] LocalSize = x,y,w,h` —— 可玩矩形，参考实现的"自动取景"拿它和整图比。
    pub local_x: i32,
    pub local_y: i32,
    pub local_width: i32,
    pub local_height: i32,

    /// 网格是 `(2*width - 1)` 列 × `height` 格。索引：`dx * height + row`。
    pub tiles: Vec<IsoTile>,
    pub overlays: Vec<MapOverlay>,
    pub terrains: Vec<MapTerrain>,
    pub structures: Vec<MapStructure>,
    pub units: Vec<MapUnit>,
    pub infantries: Vec<MapInfantry>,
    pub aircrafts: Vec<MapAircraft>,

    /// `IsoMapPack5` 真正携带了的条目数。
    pub present_tiles: usize,
    /// dx/dy 落到网格之外的条目数。
    pub out_of_bounds: usize,
    pub max_z: u8,
}

// --- C 字符串数值工具的等价物 ---------------------------------------------------------------
//
// 数值解析统一走 `ini::strtol10`（上游是 `std::strtol`）。

/// 上游 `parseRect`（mapfile.cpp:23-36）：从值里抠出 4 个整数，跳掉它们之间的一切非数字。
fn parse_rect(v: &str, out: &mut [i32; 4]) -> bool {
    let b = v.as_bytes();
    let mut at = 0usize;
    for slot in out.iter_mut() {
        while at < b.len() && !b[at].is_ascii_digit() && b[at] != b'-' {
            at += 1;
        }
        if at >= b.len() {
            return false;
        }
        let (val, used) = strtol10(v.get(at..).unwrap_or(""));
        if used == 0 {
            return false;
        }
        *slot = val as i32;
        at += used;
    }
    true
}

/// 上游 `Split`（mapfile.cpp:258-278）：按 `,` 切开，每段先削掉尾部的空格与 `\r`，再去掉头部空白。
fn split_csv(v: &str) -> Vec<String> {
    let mut f = Vec::new();
    let mut at = 0usize;
    loop {
        let comma = v[at..].find(',');
        let mut part = match comma {
            Some(c) => v[at..at + c].to_string(),
            None => v[at..].to_string(),
        };
        while part.ends_with(' ') || part.ends_with('\r') {
            part.pop();
        }
        f.push(part.trim_start_matches([' ', '\t']).to_string());
        match comma {
            Some(c) => at = at + c + 1,
            None => break,
        }
    }
    f
}

/// 上游 `Split::num`：字段越界给 `dflt`，字段存在但不是数字给 0（strtol 的行为）。
fn field_num(f: &[String], i: usize, dflt: i32) -> i32 {
    if i >= f.len() {
        return dflt;
    }
    strtol10(&f[i]).0 as i32
}

/// 上游 `base64Decode`（mapfile.cpp:40-65）。跳 `=`、空白与任何非法字符；只要解出过字节就算成功。
pub fn base64_decode(input: &str) -> Option<Vec<u8>> {
    fn value(c: u8) -> i32 {
        match c {
            b'A'..=b'Z' => (c - b'A') as i32,
            b'a'..=b'z' => (c - b'a') as i32 + 26,
            b'0'..=b'9' => (c - b'0') as i32 + 52,
            b'+' => 62,
            b'/' => 63,
            _ => -1,
        }
    }
    let mut out: Vec<u8> = Vec::with_capacity(input.len() / 4 * 3 + 3);
    let mut acc: u32 = 0;
    let mut bits: i32 = 0;
    for &c in input.as_bytes() {
        if c == b'=' || c == b'\n' || c == b'\r' || c == b' ' || c == b'\t' {
            continue;
        }
        let v = value(c);
        if v < 0 {
            continue; // 地图文件偶尔带一个多余的尾巴字节
        }
        acc = (acc << 6) | v as u32;
        bits += 6;
        if bits >= 8 {
            bits -= 8;
            out.push(((acc >> bits) & 0xFF) as u8);
        }
    }
    if out.is_empty() {
        None
    } else {
        Some(out)
    }
}

impl MapFile {
    /// 上游 `MapFile::gridAt`（mapfile.cpp:67-72）。
    pub fn grid_at(&self, dx: i32, row: i32) -> Option<&IsoTile> {
        if dx < 0 || row < 0 || row >= self.height {
            return None;
        }
        let i = dx as usize * self.height as usize + row as usize;
        self.tiles.get(i)
    }

    /// 上游 `MapFile::tileRC`（mapfile.cpp:74-79）。
    pub fn tile_rc(&self, rx: i32, ry: i32) -> Option<&IsoTile> {
        let dx = rx - ry + self.width - 1;
        let dy = rx + ry - self.width - 1;
        if dy < 0 {
            return None;
        }
        self.grid_at(dx, dy / 2)
    }
}

/// `data` 必须是整个 `.map` 文件的内容。
pub fn parse_map(data: &[u8]) -> Result<MapFile, String> {
    let mut out = MapFile::default();
    if data.is_empty() {
        return Err("empty map".into());
    }

    let text = String::from_utf8_lossy(data);
    let ini = Ini::parse(&text);

    let map = ini
        .get("map")
        .ok_or_else(|| "no [Map] section (map may use BasedOn inheritance, which needs the base file)".to_string())?;
    let size_str = map.value("size").ok_or_else(|| "[Map] has no usable Size".to_string())?;
    let mut rect = [0i32; 4];
    if !parse_rect(size_str, &mut rect) {
        return Err("[Map] has no usable Size".into());
    }
    out.width = rect[2];
    out.height = rect[3];
    if out.width <= 0 || out.height <= 0 || out.width > 512 || out.height > 512 {
        return Err("[Map] Size out of range".into());
    }

    if let Some(local) = map.value("localsize") {
        let mut lrect = [0i32; 4];
        if parse_rect(local, &mut lrect) {
            out.local_x = lrect[0];
            out.local_y = lrect[1];
            out.local_width = lrect[2];
            out.local_height = lrect[3];
        }
    }

    // `Theater` 住在 [Map] 而不是 [Basic]。[Basic] 也有个 `Size`，但那是编辑器口径的另一种量法，
    // 与 IsoMapPack5 的格数对得上的是 [Map] Size。
    if let Some(t) = map.value("theater") {
        out.theater = t.to_string();
    }
    if let Some(basic) = ini.get("basic") {
        if out.theater.is_empty() {
            if let Some(t) = basic.value("theater") {
                out.theater = t.to_string();
            }
        }
        if let Some(n) = basic.value("newtheater") {
            out.new_theater = ini_bool(n, false);
        }
    }

    let cols = 2 * out.width - 1;
    out.tiles = vec![IsoTile::default(); cols as usize * out.height as usize];

    // --- 地形砖 --------------------------------------------------------------------------
    if ini.get("isomappack5").is_none() {
        return Err("no [IsoMapPack5] section".into());
    }
    let b64 = ini.concatenated_values("isomappack5");
    let packed = base64_decode(&b64).ok_or_else(|| "[IsoMapPack5] is not decodable base64".to_string())?;

    // 先把每格铺成 Level-0 的空地砖：它们给每格真实的坐标，于是包没覆盖到的格子照样画得出地面。
    for row in 0..out.height {
        for x in 0..cols {
            let dy = row * 2 + (x % 2);
            let rx = (x + dy) / 2 + 1;
            let ry = dy - rx + out.width + 1;
            let t = &mut out.tiles[x as usize * out.height as usize + row as usize];
            *t = IsoTile::default();
            t.dx = x;
            t.dy = dy;
            t.rx = rx;
            t.ry = ry;
            t.tile_num = 0;
            t.empty = true;
        }
    }

    let cells = cols as usize * out.height as usize;
    // +4 给包尾部携带的 LZO 结束标记。
    let mut unpack = vec![0u8; cells * 11 + 4];
    // 解压前给每格写哨兵 rx = ry = 0x4088：它过不了 `rx <= 511 / ry <= 511` 的检查，
    // 于是"包没覆盖到的格"和"包根本没提过的格"分不出来（正是上游要的效果）。
    const SENTINEL: [u8; 4] = [0x88, 0x40, 0x88, 0x40];
    for i in 0..cells {
        unpack[i * 11..i * 11 + 4].copy_from_slice(&SENTINEL);
    }
    decode_chunked(&packed, &mut unpack, false);

    for i in 0..cells {
        let p = &unpack[i * 11..i * 11 + 11];
        let rx = u16::from_le_bytes([p[0], p[1]]);
        let ry = u16::from_le_bytes([p[2], p[3]]);
        let mut tile_num = i32::from_le_bytes([p[4], p[5], p[6], p[7]]);
        let sub_tile = p[8];
        let z = p[9];
        let ice = p[10];

        // 0xFFFF（以及任何比它大的）表示"这里没有砖"。
        if tile_num >= 65535 || tile_num < 0 {
            tile_num = 0;
        }
        if rx > RC_LIMIT || ry > RC_LIMIT {
            continue; // 没被碰过的哨兵格
        }

        let dx = rx as i32 - ry as i32 + out.width - 1;
        let dy = rx as i32 + ry as i32 - out.width - 1;
        out.present_tiles += 1;
        if dx < 0 || dx >= cols || dy < 0 || dy >= 2 * out.height {
            out.out_of_bounds += 1;
            continue;
        }
        let t = &mut out.tiles[dx as usize * out.height as usize + (dy / 2) as usize];
        *t = IsoTile::default();
        t.dx = dx;
        t.dy = dy;
        t.rx = rx as i32;
        t.ry = ry as i32;
        t.tile_num = tile_num;
        t.sub_tile = sub_tile;
        t.z = z;
        t.ice_growth = ice;
        t.empty = false;
        out.max_z = out.max_z.max(z);
    }

    // --- 覆盖物 --------------------------------------------------------------------------
    // OverlayPack 是 LCW（"Format80"），按 `rx + 512*ry` 索引；OverlayDataPack 装配套的附加值。
    // 两条都是零填充，`0xFF` 表示没有。
    let ovl_pack = ini.get("overlaypack");
    let ovl_data = ini.get("overlaydatapack");
    if ovl_pack.is_some() && ovl_data.is_some() {
        let mut ids = vec![0u8; 1usize << 18];
        let mut dat = vec![0u8; 1usize << 18];
        // 上游只解一次 base64 就复用同一个 `enc` 缓冲；这里各自解，结果相同。
        if let Some(enc) = base64_decode(&ini.concatenated_values("overlaypack")) {
            decode_chunked(&enc, &mut ids, true);
        }
        if let Some(enc) = base64_decode(&ini.concatenated_values("overlaydatapack")) {
            decode_chunked(&enc, &mut dat, true);
        }

        for row in 0..out.height {
            for x in (0..cols).rev() {
                let t = out.tiles[x as usize * out.height as usize + row as usize];
                let idx = t.rx as usize + 512usize * t.ry as usize;
                if idx >= ids.len() {
                    continue;
                }
                if ids[idx] == 0xFF {
                    continue;
                }
                out.overlays.push(MapOverlay {
                    rx: t.rx,
                    ry: t.ry,
                    dx: t.dx,
                    dy: t.dy,
                    z: t.z,
                    id: ids[idx],
                    data: dat[idx],
                });
            }
        }
    }

    // --- 地形对象（树、矿石柱）------------------------------------------------------------
    if let Some(sec) = ini.get("terrain") {
        for e in &sec.entries {
            if e.value.is_empty() {
                continue;
            }
            let (pos, used) = strtol10(&e.key);
            if used == 0 {
                continue;
            }
            let t = MapTerrain {
                name: e.value.clone(),
                rx: (pos % 1000) as i32,
                ry: (pos / 1000) as i32,
            };
            if t.rx < 0 || t.ry < 0 || t.rx > 511 || t.ry > 511 {
                continue;
            }
            out.terrains.push(t);
        }
    }

    // --- 建筑与单位 ----------------------------------------------------------------------
    // `Owner,Name,Health,Rx,Ry,Facing,...`。字段太少的行直接跳过而不是补默认值 ——
    // 一条畸形条目不该变成一个蹲在 (0,0) 的对象。
    {
        let parse_roster = |section: &str, out: &mut MapFile| {
            let Some(sec) = ini.get(section) else { return };
            for e in &sec.entries {
                if e.value.is_empty() {
                    continue;
                }
                let f = split_csv(&e.value);
                if f.len() < 6 {
                    continue;
                }

                if section == "structures" {
                    let mut m = MapStructure {
                        owner: f[0].clone(),
                        name: f[1].clone(),
                        health: field_num(&f, 2, 0),
                        rx: field_num(&f, 3, 0),
                        ry: field_num(&f, 4, 0),
                        facing: field_num(&f, 5, 0) & 0xFF,
                        on_bridge: false,
                    };
                    if f.len() > 15 {
                        m.on_bridge = f[15] == "1";
                    }
                    if m.rx > 511 || m.ry > 511 {
                        continue;
                    }
                    out.structures.push(m);
                } else if section == "units" {
                    let mut m = MapUnit {
                        owner: f[0].clone(),
                        name: f[1].clone(),
                        rx: field_num(&f, 3, 0),
                        ry: field_num(&f, 4, 0),
                        facing: field_num(&f, 5, 0) & 0xFF,
                        on_bridge: false,
                    };
                    if f.len() > 10 {
                        m.on_bridge = f[10] == "1";
                    }
                    if m.rx > 511 || m.ry > 511 {
                        continue;
                    }
                    out.units.push(m);
                } else if section == "infantry" {
                    // `[Infantry]`：facing 是 entries[7]，onBridge 是 entries[11]。
                    // 少见的 5 字段（无 facing）行会被下面这行挡掉。
                    if f.len() < 8 {
                        continue;
                    }
                    let mut m = MapInfantry {
                        owner: f[0].clone(),
                        name: f[1].clone(),
                        rx: field_num(&f, 3, 0),
                        ry: field_num(&f, 4, 0),
                        sub_cell: field_num(&f, 5, 0),
                        facing: field_num(&f, 7, 0) & 0xFF,
                        on_bridge: false,
                    };
                    if f.len() > 11 {
                        m.on_bridge = f[11] == "1";
                    }
                    if m.rx > 511 || m.ry > 511 {
                        continue;
                    }
                    out.infantries.push(m);
                } else if section == "aircraft" {
                    // `[Aircraft]`：facing 是 entries[5]，onBridge 是 entries[length-4]。
                    let mut m = MapAircraft {
                        owner: f[0].clone(),
                        name: f[1].clone(),
                        rx: field_num(&f, 3, 0),
                        ry: field_num(&f, 4, 0),
                        facing: field_num(&f, 5, 0) & 0xFF,
                        on_bridge: false,
                    };
                    if f.len() >= 4 {
                        m.on_bridge = f[f.len() - 4] == "1";
                    }
                    if m.rx > 511 || m.ry > 511 {
                        continue;
                    }
                    out.aircrafts.push(m);
                }
            }
        };

        parse_roster("structures", &mut out);
        parse_roster("units", &mut out);
        parse_roster("infantry", &mut out);
        parse_roster("aircraft", &mut out);
    }

    // --- 光照 ----------------------------------------------------------------------------
    // 留在 map 上而不是渲染器里，因为它是地图数据：同一张砖在不同地图里该有不同的亮度。
    parse_lighting(&ini, &mut out.lighting);

    Ok(out)
}

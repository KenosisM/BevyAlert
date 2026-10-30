//! theater 解析 —— 对等移植。
//!
//! 来源：`I:\bevy\RA2REASM\src\compat\theater.h` / `theater.cpp`。
//!
//! 一个 theater 就是一种地表外观（Temperate / Snow / Urban / Lunar …）。它管的是**地形词汇**：
//! 每个 `[TileSetNNNN]` 段描述一组砖，砖的素材名由 `FileName + 两位编号 + 变体字母 + theater 扩展名`
//! 拼出来。注意扩展名是 theater 自己的（`.tem` / `.sno` / `.urb` / `.ubn` / `.des` / `.lun`），
//! **不是 `.tmp`** —— 这是最容易搞错的一处。

use crate::ini::{ini_bool, ini_color, ini_int, Ini};
use crate::tilefix::TileFixupIds;

#[derive(Debug, Clone, Default)]
pub struct TheaterPalettes {
    /// `temperat.pal` / `snow.pal` / `urban.pal` / `lunar.pal`（覆盖物用）
    pub base: String,
    /// `isotem.pal` / `isosno.pal` / `isourb.pal` / `isolun.pal`
    pub iso: String,
    /// `unittem.pal` / `unitsno.pal` / `uniturb.pal` / `unitlun.pal`
    pub unit: String,
}

#[derive(Debug, Clone)]
pub struct TileSet {
    /// `[TileSetNNNN]` 里的 NNNN
    pub index: i32,
    /// `TilesInSet > 0` 且 FileName 是真的
    pub enabled: bool,
    pub set_name: String,
    /// 逻辑名；素材由它拼出来，见 `tile_asset_name`
    pub file_name: String,
    pub tiles_in_set: i32,
    pub morphable: bool,
    pub allow_to_place: bool,
    pub allow_burrowing: bool,
    pub marble_madness: i32,
    pub non_marble_madness: i32,
    pub to_snow_theater: i32,
    pub has_low_radar: bool,
    pub has_high_radar: bool,
    pub low_radar: [i32; 3],
    pub high_radar: [i32; 3],
}

impl Default for TileSet {
    fn default() -> Self {
        TileSet {
            index: 0,
            enabled: false,
            set_name: String::new(),
            file_name: String::new(),
            tiles_in_set: 0,
            morphable: false,
            allow_to_place: true,
            allow_burrowing: true,
            marble_madness: -1,
            non_marble_madness: -1,
            to_snow_theater: -1,
            has_low_radar: false,
            has_high_radar: false,
            low_radar: [0, 0, 0],
            high_radar: [0, 0, 0],
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct Theater {
    /// 被请求时的名字
    pub name: String,
    /// `tem` / `sno` / `urb` / `ubn` / `des` / `lun`
    pub suffix: String,
    /// `temperatmd.ini` 之流
    pub ini_name: String,
    /// `.tem` / `.sno` / … —— **不是** `.tmp`
    pub extension: String,
    /// NewTheater 素材后缀，如 `"ga"` → `"gat"`
    pub new_theater_char: char,
    pub palettes: TheaterPalettes,

    // [General] 里引擎按角色使用的砖 id
    pub clear_tile: i32,
    pub rough_tile: i32,
    pub sand_tile: i32,
    pub green_tile: i32,
    pub pave_tile: i32,
    pub ramp_base: i32,
    pub ramp_smooth: i32,
    pub bridge_set: i32,
    pub train_bridge_set: i32,
    pub wood_bridge_set: i32,

    /// 每个 `[TileSetNNNN]` 段，按顺序，无论启用与否。
    /// 未启用的段照样占一个索引，因为地图是按编号引用 tileset 的。
    pub tile_sets: Vec<TileSet>,
    pub enabled_count: usize,
    pub disabled_count: usize,

    /// `[General]` 里 LAT / ramp 修正所依据的 id，见 `fix_tiles`
    pub fixup: TileFixupIds,
}

struct TheaterDef {
    key: &'static str,
    suffix: &'static str,
    ini_name: &'static str,
    base_pal: &'static str,
    extension: &'static str,
    new_theater_char: char,
}

/// theater → 调色板的映射硬编码在引擎里；theater INI 里没有任何东西指明调色板。
/// 已对着发行素材核对过：isotem/isosno/isourb/isolun、unittem/unitsno/uniturb/unitlun 都存在。
/// 砖的扩展名是 theater 自己的，不是 `.tmp` —— 已核对 Clear01.tem / .sno / .urb / .lun 都能取到。
///
/// YR 发行的是**六**个 theater 而不是四个：NewUrban 和 Desert 在真实地图里出现过
/// （`austintx.map` 就写着 NEWURBAN）。iso/unit 调色板六者都遵循 `iso<suffix>.pal` /
/// `unit<suffix>.pal`，只有覆盖物调色板不规则，必须逐个列。
const THEATERS: [TheaterDef; 6] = [
    TheaterDef { key: "temperat", suffix: "tem", ini_name: "temperatmd.ini", base_pal: "temperat.pal", extension: ".tem", new_theater_char: 'T' },
    TheaterDef { key: "snow", suffix: "sno", ini_name: "snowmd.ini", base_pal: "snow.pal", extension: ".sno", new_theater_char: 'A' },
    TheaterDef { key: "urban", suffix: "urb", ini_name: "urbanmd.ini", base_pal: "urban.pal", extension: ".urb", new_theater_char: 'U' },
    TheaterDef { key: "newurban", suffix: "ubn", ini_name: "urbannmd.ini", base_pal: "urbann.pal", extension: ".ubn", new_theater_char: 'N' },
    TheaterDef { key: "desert", suffix: "des", ini_name: "desertmd.ini", base_pal: "desert.pal", extension: ".des", new_theater_char: 'D' },
    TheaterDef { key: "lunar", suffix: "lun", ini_name: "lunarmd.ini", base_pal: "lunar.pal", extension: ".lun", new_theater_char: 'L' },
];

fn matches(asked: &str, canonical: &str, suffix: &str) -> bool {
    let a = asked.trim().to_ascii_lowercase();
    if a == canonical || a == suffix {
        return true;
    }
    // "temperate" 之类
    let ab = a.as_bytes();
    let cb = canonical.as_bytes();
    if ab.len() > 4 && cb.len() >= 4 && ab[..4] == cb[..4] {
        return true;
    }
    if a == format!("{canonical}md") {
        return true;
    }
    false
}

pub fn theater_names() -> &'static str {
    "temperat | snow | urban | newurban | desert | lunar"
}

/// 按名字解析 theater。接受全名，也接受三字母代码。
pub fn theater_info(name: &str) -> Option<Theater> {
    for d in &THEATERS {
        if !matches(name, d.key, d.suffix) {
            continue;
        }
        return Some(Theater {
            name: d.key.to_string(),
            suffix: d.suffix.to_string(),
            ini_name: d.ini_name.to_string(),
            extension: d.extension.to_string(),
            new_theater_char: d.new_theater_char,
            palettes: TheaterPalettes {
                base: d.base_pal.to_string(),
                iso: format!("iso{}.pal", d.suffix),
                unit: format!("unit{}.pal", d.suffix),
            },
            bridge_set: -1,
            train_bridge_set: -1,
            wood_bridge_set: -1,
            ..Default::default()
        });
    }
    None
}

/// 引擎全局编号砖：每个段占一个索引，一个启用的段贡献 `TilesInSet` 个连续的全局砖号。
#[derive(Debug, Clone, Default)]
pub struct TileNumbering {
    /// 段索引 → 第一个全局砖号
    pub set_first_tile: Vec<i32>,
    /// 全局砖号 → 段索引
    pub tile_to_set: Vec<i32>,
}

/// 上游 `buildTileNumbering`（theater.cpp:71-88）。
pub fn build_tile_numbering(th: &Theater) -> TileNumbering {
    let mut out = TileNumbering {
        set_first_tile: vec![0; th.tile_sets.len()],
        tile_to_set: Vec::new(),
    };
    let mut tile_num = 0i32;
    for (i, ts) in th.tile_sets.iter().enumerate() {
        out.set_first_tile[i] = tile_num;
        // 每个段的 TilesInSet 都算进去，包括没有可用素材的段。引擎从不检查这一点：
        // 它从 TileSet0000 往上走，每段加 TilesInSet 个全局号，所以一个 `TilesInSet=1`
        // 而 FileName 为空的段照样吃掉一个号。**跳过它会把之后每一个砖号都挪一位** ——
        // 表现出来是整张地图配错 TMP，而不是少了一块砖。
        for _ in 0..ts.tiles_in_set {
            out.tile_to_set.push(i as i32);
            tile_num += 1;
        }
    }
    out
}

/// 一块砖的归档文件名：`FileName + <tileInSet+1 两位> + [变体字母] + theater 扩展名`，
/// 于是 `Droads` 的第 3 块、无变体、temperate → `"Droads03.tem"`。
/// 扩展名是 theater 的，**不是** `".tmp"`。变体 0 表示没有字母；1..26 表示 `'a'..'z'`。
pub fn tile_asset_name(th: &Theater, ts: &TileSet, tile_in_set: i32, variant: i32) -> String {
    let mut name = format!("{}{:02}", ts.file_name, tile_in_set + 1);
    if variant > 0 {
        name.push(char::from(b'a' + (variant - 1) as u8));
    }
    name.push_str(&th.extension);
    name
}

/// 上游 `parseTheater`（theater.cpp:99-202）。
pub fn parse_theater(ini: &Ini, out: &mut Theater) -> Result<(), String> {
    out.tile_sets.clear();
    out.enabled_count = 0;
    out.disabled_count = 0;

    if let Some(g) = ini.get("general") {
        let num = |k: &str, dflt: i32| -> i32 {
            match g.value(k) {
                Some(v) => ini_int(v, dflt),
                None => dflt,
            }
        };
        out.clear_tile = num("cleartile", out.clear_tile);
        out.rough_tile = num("roughtile", out.rough_tile);
        out.sand_tile = num("sandtile", out.sand_tile);
        out.green_tile = num("greentile", out.green_tile);
        out.pave_tile = num("pavetile", out.pave_tile);
        out.ramp_base = num("rampbase", out.ramp_base);
        out.ramp_smooth = num("rampsmooth", out.ramp_smooth);
        out.bridge_set = num("bridgeset", out.bridge_set);
        out.train_bridge_set = num("trainbridgeset", out.train_bridge_set);
        out.wood_bridge_set = num("woodbridgeset", out.wood_bridge_set);

        // LAT / ramp 修正用的 key。-1 表示这个 theater 没定义该 set，
        // 于是 `fix_tiles` 会跳过对应规则，而不是碰巧匹配到 set 0。
        let f = &mut out.fixup;
        f.rough_tile = num("roughtile", -1);
        f.sand_tile = num("sandtile", -1);
        f.green_tile = num("greentile", -1);
        f.pave_tile = num("pavetile", -1);
        f.blue_mold_tile = num("bluemoldtile", -1);
        f.crystal_tile = num("crystaltile", -1);
        f.swamp_tile = num("swamptile", -1);

        f.clear_to_rough_lat = num("cleartoroughlat", -1);
        f.clear_to_sand_lat = num("cleartosandlat", -1);
        f.clear_to_green_lat = num("cleartogreenlat", -1);
        f.clear_to_pave_lat = num("cleartopavelat", -1);
        f.clear_to_blue_mold_lat = num("cleartobluemoldlat", -1);
        f.clear_to_crystal_lat = num("cleartocrystallat", -1);
        f.water_to_swamp_lat = num("watertoswamplat", -1);

        f.crystal_cliff = num("crystalcliff", -1);
        f.shore_pieces = num("shorepieces", -1);
        f.water_bridge = num("waterbridge", -1);
        f.paved_roads = num("pavedroads", -1);
        f.medians = num("medians", -1);
        f.misc_pave_tile = num("miscpavetile", -1);

        f.ramp_base = num("rampbase", -1);
        f.ramp_smooth = num("rampsmooth", -1);

        // snowmd.ini 是唯一没有 Medians 的 theater；Ares 的 MediansFix 用 set 71
        // （snow 的 "Paved road bits"）顶上，于是铺装 LAT 会连到那些砖。
        if f.medians == -1 && out.suffix == "sno" {
            f.medians = 71;
        }
    }

    for s in &ini.sections {
        let nb = s.name.as_bytes();
        if nb.len() < 8 {
            continue;
        }
        // 上游把段名小写后比较（`IniFile` 存的就是小写）；这里存的是原始大小写，按字节忽略大小写比。
        if !nb[..7].eq_ignore_ascii_case(b"tileset") {
            continue;
        }
        if !nb[7..].iter().all(|c| c.is_ascii_digit()) {
            continue;
        }

        let mut t = TileSet::default();
        t.index = ini_int(&s.name[7..], 0);
        let str_of = |k: &str, dflt: &str| -> String {
            match s.value(k) {
                Some(v) => v.to_string(),
                None => dflt.to_string(),
            }
        };
        t.set_name = str_of("setname", "");
        t.file_name = str_of("filename", "");
        t.tiles_in_set = ini_int(&str_of("tilesinset", "0"), 0);
        t.morphable = ini_bool(&str_of("morphable", ""), false);
        t.allow_to_place = ini_bool(&str_of("allowtoplace", ""), true);
        t.allow_burrowing = ini_bool(&str_of("allowburrowing", ""), true);
        t.marble_madness = ini_int(&str_of("marblemadness", "-1"), -1);
        t.non_marble_madness = ini_int(&str_of("nonmarblemadness", "-1"), -1);
        t.to_snow_theater = ini_int(&str_of("tosnowtheater", "-1"), -1);
        let low = str_of("lowradarcolor", "");
        let high = str_of("highradarcolor", "");
        t.has_low_radar = ini_color(&low, &mut t.low_radar);
        t.has_high_radar = ini_color(&high, &mut t.high_radar);

        // 没有砖的段是"停用的占位符"：真正的 FileName 就在它上面被注释掉了。
        // 这些槽位照样占一个索引 —— 地图按编号引用 tileset，所以要把它们留在表里，
        // 只在编号砖的时候跳过。
        t.enabled = t.tiles_in_set > 0
            && !t.file_name.is_empty()
            && !t.file_name.eq_ignore_ascii_case("blank");
        if t.enabled {
            out.enabled_count += 1;
        } else {
            out.disabled_count += 1;
        }
        out.tile_sets.push(t);
    }
    Ok(())
}

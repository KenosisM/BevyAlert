//! 调色板重建 —— 对等移植。
//!
//! 来源：`I:\bevy\RA2REASM\src\compat\lighting.cpp` 的 `recalculatePalette` /
//! `quantizeTsIntensity` / `clamp2`，以及 `lighting.h` 的 `PaletteMults`。
//!
//! 为什么要它：`.pal` 存的是**六位**值，而引擎画体素时走的是"已经算过光照的物体调色板"。
//! 直接把 `.pal` 的原始字节丢给光栅器会让所有颜色只剩四分之一亮度；而先做位对齐（×4）
//! 再拉伸到 255 也是错的 —— 引擎是在六位里做乘法再拉到 255，先移位会让每个颜色偏最多 3。
//! 正解就是这里这一个公式。

use crate::ini::Ini;

/// 地图的 `[Lighting]` 段。
///
/// 默认值是**引擎自己的**，不是零：一张完全没有 `[Lighting]` 的地图仍然按 Level 0.032、
/// Ambient 1.0 渲染。真实地图会大幅覆盖它们 —— 一张写 `Ambient 0.8`、`Blue 0.95` 的地图，
/// 整片表面暗 20% 且偏色，所以这里弄错的代价比少画一个对象层还大。
///
/// 来源：`I:\bevy\RA2REASM\src\compat\lighting.h` 的 `struct MapLighting`。
#[derive(Debug, Clone, Copy)]
pub struct MapLighting {
    pub level: f64,
    pub ambient: f64,
    pub red: f64,
    pub green: f64,
    pub blue: f64,
    pub ground: f64,
}

impl Default for MapLighting {
    fn default() -> Self {
        Self {
            level: 0.032,
            ambient: 1.0,
            red: 1.0,
            green: 1.0,
            blue: 1.0,
            ground: 0.0,
        }
    }
}

/// `strtod` 的等价物（上游 lighting.cpp:12-16 的 `dblOf` 用它）：吃下能认出的数值前缀，
/// 认不出就返回 0。Rust 的 `str::parse` 对 `"1.5abc"` 会整体失败给 0，而 strtod 给 1.5。
fn strtod(v: &str) -> f64 {
    let b = v.as_bytes();
    let mut i = 0usize;
    while i < b.len() && (b[i] as char).is_ascii_whitespace() {
        i += 1;
    }
    let start = i;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        i += 1;
    }
    let mut saw_digit = false;
    while i < b.len() && b[i].is_ascii_digit() {
        i += 1;
        saw_digit = true;
    }
    if i < b.len() && b[i] == b'.' {
        i += 1;
        while i < b.len() && b[i].is_ascii_digit() {
            i += 1;
            saw_digit = true;
        }
    }
    if !saw_digit {
        return 0.0;
    }
    let mut end = i;
    if end < b.len() && (b[end] == b'e' || b[end] == b'E') {
        let mut k = end + 1;
        if k < b.len() && (b[k] == b'+' || b[k] == b'-') {
            k += 1;
        }
        let ds = k;
        while k < b.len() && b[k].is_ascii_digit() {
            k += 1;
        }
        if k > ds {
            end = k;
        }
    }
    v[start..end].parse::<f64>().unwrap_or(0.0)
}

fn dbl_of(ini: &Ini, section: &str, key: &str, fallback: f64) -> f64 {
    let Some(s) = ini.get(section) else { return fallback };
    let Some(v) = s.value(key) else { return fallback };
    if v.is_empty() {
        return fallback;
    }
    strtod(v)
}

/// 上游 `parseLighting`（lighting.cpp:20-29）。
pub fn parse_lighting(ini: &Ini, out: &mut MapLighting) {
    if ini.get("lighting").is_none() {
        return;
    }
    out.level = dbl_of(ini, "lighting", "level", out.level);
    out.ambient = dbl_of(ini, "lighting", "ambient", out.ambient);
    out.red = dbl_of(ini, "lighting", "red", out.red);
    out.green = dbl_of(ini, "lighting", "green", out.green);
    out.blue = dbl_of(ini, "lighting", "blue", out.blue);
    out.ground = dbl_of(ini, "lighting", "ground", out.ground);
}

/// 上游 `lightingMults`（lighting.cpp:40-49）。`level` 是这份调色板为之构建的砖高；
/// 为 level 0 构建的和为 level 18 构建的差 `18 * Level`。
pub fn lighting_mults(l: &MapLighting, level: i32, apply_tints: bool) -> PaletteMults {
    let mut m = PaletteMults::default();
    m.ambient = l.ambient - l.ground + l.level * level as f64;
    if apply_tints {
        m.red = l.red;
        m.green = l.green;
        m.blue = l.blue;
    }
    m
}

/// gamemd 把光照强度量化到它 LightConvert 表的 63 档
/// （CellClass::ComputeLighting 0x484180）。参考实现在每次渲染前无条件打开这个开关。
pub fn quantize_ts_intensity(intensity: f64) -> f64 {
    let mut brightness = (intensity * 1000.0) as i32;
    if brightness < 0 {
        brightness = 0;
    }
    if brightness > 2000 {
        brightness = 2000;
    }
    let shade = 254i32.min((261 * brightness) >> 11);
    let level = 62i32.min(127 * shade * 62 / 32258);
    level as f64 / 31.0
}

fn clamp2(v: f64) -> f64 {
    v.max(0.0).min(2.0)
}

/// ApplyLighting 留在调色板上的东西：ambient 是一个平铺的强度，tint 三元组是独立的逐通道增益。
#[derive(Clone, Copy)]
pub struct PaletteMults {
    pub ambient: f64,
    pub red: f64,
    pub green: f64,
    pub blue: f64,
}

impl Default for PaletteMults {
    fn default() -> Self {
        Self {
            ambient: 1.0,
            red: 1.0,
            green: 1.0,
            blue: 1.0,
        }
    }
}

/// 调色板缓冲的**兜底色 = 品红**。
///
/// 下面两个函数都按「原始 .pal 有多少字节就写多少项」工作，数据缺项时剩下的槽位就保持
/// 缓冲初值。初值取 0 就是纯黑 —— 黑在画面上认不出来（"这块是数据缺失还是本来就是黑的？"），
/// 所以初值改成洋红：**凡是看到洋红，就是那一项调色板没数据**。正常 .pal 是 768 字节、
/// 会填满全部 256 项，所以这条兜底在正常路径上永不出现。
fn magenta_palette() -> Vec<u8> {
    let mut p = vec![0u8; 768];
    for i in 0..256usize {
        p[i * 3] = 255;
        p[i * 3 + 2] = 255;
    }
    p
}

/// Recalculate：原始六位调色板进，算过光照的八位调色板出。
///
/// `raw768` 必须是 `.pal` 自己的字节（0..63），**不是**预展开的八位表。
/// `object_palette` 为真时索引 240..=254 原样保留 —— 建筑靠这个在任意光照下都保住重映射槽。
pub fn recalculate_palette(raw768: &[u8], mm: PaletteMults, object_palette: bool) -> Vec<u8> {
    // gamemd（CellClass::ComputeLighting 0x484180, normalize 0x5558E0）：ambient 和每个 tint
    // 各自钳到 [0,2]；tint 三元组先归一化到最大通道为 1，最大值进 intensity，再钳一次 [0,2]。
    // 这两次钳制把主导通道的增益封在 2 倍。
    let amb = clamp2(mm.ambient);
    let tr = clamp2(mm.red);
    let tg = clamp2(mm.green);
    let tb = clamp2(mm.blue);
    let mx = tr.max(tg).max(tb);

    let mut rmult = 0.0f64;
    let mut gmult = 0.0f64;
    let mut bmult = 0.0f64;
    if mx >= 0.001 {
        let mut intensity = (amb * mx).min(2.0);
        intensity = quantize_ts_intensity(intensity);
        rmult = intensity * (tr / mx);
        gmult = intensity * (tg / mx);
        bmult = intensity * (tb / mx);
    }

    let mut out = magenta_palette();
    for i in 0..256usize {
        let k = i * 3;
        if k + 2 >= raw768.len() {
            break;
        }
        let (mut rm, mut gm, mut bm) = (rmult, gmult, bmult);
        // 物体调色板把顶部的颜色排除在光照之外，重映射槽（和纯白 UI 色）才能原样存活。
        if object_palette && (240..=254).contains(&i) {
            rm = 1.0;
            gm = 1.0;
            bm = 1.0;
        }
        out[k] = (raw768[k] as f64 * rm / 63.0 * 255.0).min(255.0) as u8;
        out[k + 1] = (raw768[k + 1] as f64 * gm / 63.0 * 255.0).min(255.0) as u8;
        out[k + 2] = (raw768[k + 2] as f64 * bm / 63.0 * 255.0).min(255.0) as u8;
    }
    out
}

/// 六位调色板 → 八位（0..=255），**不施加任何光照**。
///
/// 用户口径（2026-10-16）：VPL 那套「按面法线选一张光照页，再用页把 256 项索引整体
/// 重映射」作废 —— 体素渲染改成「索引直接查成真 RGB，再交给现代着色器打光」。
/// 所以单位调色板要的是**本体色**：把 `.pal` 的 0..63 展开到 0..255 就够了，
/// 不乘 ambient、不乘 tint、也不做 gamemd 的 LightConvert 量化。
pub fn expand_palette(raw768: &[u8]) -> Vec<u8> {
    let mut out = magenta_palette();
    for i in 0..256usize {
        let k = i * 3;
        if k + 2 >= raw768.len() {
            break;
        }
        for c in 0..3 {
            out[k + c] = (raw768[k + c] as f64 / 63.0 * 255.0).min(255.0) as u8;
        }
    }
    out
}

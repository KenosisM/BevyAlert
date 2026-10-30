//! SHP 帧解析 —— 对等移植。
//!
//! 来源：`I:\bevy\RA2REASM\src\compat\shp.cpp` / `shp.h`。
//!
//! 为什么地图链需要它：theater 的砖**不一定**是 TMP —— 有些 tileset 走 SHP 图。
//! 上游 `buildTerrainCells` 在 TMP 解析失败时会试 SHP，成功就把**整份 SHP 的帧集合**
//! 当成一个变体（每帧是一个 cell）。少了这一支，变体的**数量**就与上游不同，
//! `pickTileVariant` 会选到不同的项，`px.size() < w*h` 那道校验也就永远不触发。
//!
//! 一帧的像素按行存：每行以 u16 行长沙开头，且**行长含它自己那两个字节**。
//! RLE 是 OpenRA 的 `RLEZerosCompression`（ShpTSLoader 用的那一套）：
//!     cmd == 0 -> 再读一个字节，跳过（留零）那么多像素
//!     cmd != 0 -> 写入这一个像素值
//! 它**不是**"重复 N 次"的游程码；相邻像素是逐个写的。当成游程码读会得到完全错的形状。
//!
//! 三个细节都必须与参考实现一致，否则图像会静默错位：
//!   * 只有 `format == 3` 才走 RLE 路径，不是 `(format & 2)`；
//!   * 一行在 `lineStart + lineLength` 结束，而 `lineLength` 把自己算进去了；
//!   * 解码缓冲补到**偶数宽且偶数高**，写指针**跨行连续**（一次清零可能溢出到下一行再被覆盖）。

#[derive(Debug, Clone, Default)]
pub struct ShpFrame {
    pub x: u16,
    pub y: u16,
    pub w: u16,
    pub h: u16,
    pub format: u8,
    pub pixels: Vec<u8>,
}

#[derive(Debug, Clone, Default)]
pub struct ShpFile {
    pub width: u16,
    pub height: u16,
    pub frames: Vec<ShpFrame>,
}

fn rd16(b: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([b[at], b[at + 1]])
}

/// 上游 `decodeShpFrame`（shp.cpp:30-95）。
fn decode_shp_frame(b: &[u8], offset: u32, w: i32, h: i32, format: i32) -> Vec<u8> {
    let dw = w + (w & 1);
    let dh = h + (h & 1);
    let mut buf = vec![0u8; (dw as usize) * (dh as usize)];

    if w > 0 && h > 0 && offset > 0 && (offset as usize) < b.len() {
        if format != 3 {
            if format == 2 {
                let mut p = offset as usize;
                for y in 0..h {
                    if p + 2 > b.len() {
                        break;
                    }
                    let line_len = (b[p] as i32 | (b[p + 1] as i32) << 8) - 2;
                    p += 2;
                    if line_len <= 0 || p + line_len as usize > b.len() {
                        break;
                    }
                    let n = (line_len as usize).min(dw as usize);
                    let dst = dw as usize * y as usize;
                    buf[dst..dst + n].copy_from_slice(&b[p..p + n]);
                    p += line_len as usize;
                }
            } else {
                // format 0/1 是原始数据，每行存的正好是 w 字节（**不是**补齐后的 dw），
                // 落在补齐行的开头。
                for y in 0..h {
                    let src = offset as usize + y as usize * w as usize;
                    if src + w as usize > b.len() {
                        break;
                    }
                    let dst = dw as usize * y as usize;
                    buf[dst..dst + w as usize].copy_from_slice(&b[src..src + w as usize]);
                }
            }
        } else {
            let mut p = offset as usize;
            for y in 0..h {
                if p + 2 > b.len() {
                    break;
                }
                let line_len = b[p] as i32 | (b[p + 1] as i32) << 8;
                if line_len < 2 {
                    break; // 含自身；比 2 小就是坏数据
                }
                let line_end = (p + line_len as usize).min(b.len());
                p += 2;
                let mut d = dw as usize * y as usize;
                while p < line_end {
                    let cmd = b[p];
                    p += 1;
                    if cmd == 0 {
                        if p >= line_end {
                            break;
                        }
                        d += b[p] as usize;
                        p += 1;
                    } else {
                        if d < buf.len() {
                            buf[d] = cmd;
                        }
                        d += 1;
                    }
                }
                p = line_end;
            }
        }
    }

    if dw == w && dh == h {
        return buf;
    }
    // 补过的缓冲裁回 w x h：逐行拷前 w 个。
    let mut out = vec![0u8; (w as usize) * (h as usize)];
    for y in 0..h {
        let src = dw as usize * y as usize;
        let dst = w as usize * y as usize;
        out[dst..dst + w as usize].copy_from_slice(&buf[src..src + w as usize]);
    }
    out
}

/// 上游 `parseShp`（shp.cpp:99-123）。
pub fn parse_shp(data: &[u8]) -> Option<ShpFile> {
    if data.len() < 8 {
        return None;
    }
    let mut out = ShpFile {
        width: rd16(data, 2),
        height: rd16(data, 4),
        frames: Vec::new(),
    };
    let count = rd16(data, 6);
    out.frames.reserve(count as usize);

    for i in 0..count as usize {
        let base = 8 + i * 24;
        if base + 24 > data.len() {
            break;
        }
        let f = ShpFrame {
            x: rd16(data, base),
            y: rd16(data, base + 2),
            w: rd16(data, base + 4),
            h: rd16(data, base + 6),
            format: data[base + 8],
            pixels: Vec::new(),
        };
        let offset = u32::from_le_bytes([
            data[base + 20],
            data[base + 21],
            data[base + 22],
            data[base + 23],
        ]);
        let px = decode_shp_frame(data, offset, f.w as i32, f.h as i32, f.format as i32);
        out.frames.push(ShpFrame { pixels: px, ..f });
    }
    Some(out)
}

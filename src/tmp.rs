//! TMP 砖解析 —— 对等移植。
//!
//! 来源：`I:\bevy\RA2REASM\src\compat\tmp.h` / `tmp.cpp`。
//!
//! 一格的 TMP 砖：图标是一颗菱形，**每行长度不存**，由几何推出来（上行每行 +4 像素、
//! 下行每行 −4，越过中线后起始 x 再右移 2）。这是它与 SHP 帧唯一的实质区别。
//!
//! 对寻路真正要紧的只有每块的 `height` / `terrain_type` / `ramp_type`（在 52 字节的
//! per-tile 头里）；像素解码照搬是为了以后渲染要用，不影响 cost grid。

const HEADER_SIZE: usize = 16; // 4 x int32

/// 上游 `decodeDiamond`（tmp.cpp:18-39）。
fn decode_diamond(src: &[u8], cx: i32, cy: i32) -> Vec<u8> {
    let avail = src.len();
    let mut out = vec![0u8; (cx * cy) as usize];
    let mut r = 0usize;
    let mut x = cx / 2;
    let mut w = 0i32;
    for y in 0..cy {
        if y < cy / 2 {
            w += 4;
            x -= 2;
        } else {
            w -= 4;
            x += 2;
        }
        if w <= 0 {
            continue;
        }
        for k in 0..w {
            if r >= avail {
                return out;
            }
            let tx = x + k;
            if tx >= 0 && tx < cx {
                out[(y * cx + tx) as usize] = src[r];
            }
            r += 1;
        }
    }
    out
}

#[derive(Debug, Clone, Default)]
pub struct TmpTile {
    /// 这张砖的索引项是不是零
    pub present: bool,

    pub x: i32,
    pub y: i32,
    pub x_extra: i32,
    pub y_extra: i32,
    pub cx_extra: i32,
    pub cy_extra: i32,

    pub has_extra_data: bool,
    pub has_z_data: bool,
    pub has_damaged_data: bool,

    pub height: i8,
    pub terrain_type: i8,
    pub ramp_type: i8,
    pub radar_left: [u8; 3],
    pub radar_right: [u8; 3],

    /// `cx * cy`，已按菱形解码，0 表示空。
    pub pixels: Vec<u8>,
    /// z 面，与 `pixels` 同一菱形顺序，于是两者下标一一对应。
    /// 一张砖的深度是 `z_base - z_data[i]`。
    pub z_data: Vec<u8>,
    pub extra: Vec<u8>,
    pub extra_z: Vec<u8>,
}

#[derive(Debug, Clone, Default)]
pub struct TmpFile {
    pub cblocks_x: i32,
    pub cblocks_y: i32,
    /// 砖宽，48 或 60
    pub cx: i32,
    /// 砖高，`cx / 2`
    pub cy: i32,
    /// `cblocks_x * cblocks_y`，行主序
    pub tiles: Vec<TmpTile>,
}

/// 上游 `parseTmp`（tmp.cpp:43-117）。
pub fn parse_tmp(data: &[u8]) -> Option<TmpFile> {
    if data.len() < HEADER_SIZE {
        return None;
    }
    let rd32 = |at: usize| -> i32 { i32::from_le_bytes([data[at], data[at + 1], data[at + 2], data[at + 3]]) };
    let at32 = |at: usize| -> u32 { u32::from_le_bytes([data[at], data[at + 1], data[at + 2], data[at + 3]]) };

    let mut out = TmpFile {
        cblocks_x: rd32(0),
        cblocks_y: rd32(4),
        cx: rd32(8),
        cy: rd32(12),
        tiles: Vec::new(),
    };
    if out.cblocks_x <= 0 || out.cblocks_y <= 0 {
        return None;
    }
    if out.cx <= 0 || out.cx > 256 || out.cy * 2 != out.cx {
        return None;
    }

    let size = data.len();
    let c_tiles = out.cblocks_x as usize * out.cblocks_y as usize;
    if HEADER_SIZE + c_tiles * 4 > size {
        return None;
    }
    out.tiles = vec![TmpTile::default(); c_tiles];

    for i in 0..c_tiles {
        let at = rd32(HEADER_SIZE + i * 4);
        if at <= 0 {
            continue;
        }
        // 52 字节的 per-tile 头，然后是图标。
        let hdr_at = at as usize;
        if hdr_at + 52 > size {
            continue;
        }

        let h = &data[hdr_at..];
        let t = &mut out.tiles[i];
        t.present = true;
        t.x = i32::from_le_bytes([h[0], h[1], h[2], h[3]]);
        t.y = i32::from_le_bytes([h[4], h[5], h[6], h[7]]);
        let extra_ofs = i32::from_le_bytes([h[8], h[9], h[10], h[11]]);
        let _z_ofs = i32::from_le_bytes([h[12], h[13], h[14], h[15]]);
        let _extra_z_ofs = i32::from_le_bytes([h[16], h[17], h[18], h[19]]);
        t.x_extra = i32::from_le_bytes([h[20], h[21], h[22], h[23]]);
        t.y_extra = i32::from_le_bytes([h[24], h[25], h[26], h[27]]);
        t.cx_extra = i32::from_le_bytes([h[28], h[29], h[30], h[31]]);
        t.cy_extra = i32::from_le_bytes([h[32], h[33], h[34], h[35]]);
        let bits = at32(hdr_at + 36);
        t.has_extra_data = (bits & 1) != 0;
        t.has_z_data = (bits & 2) != 0;
        t.has_damaged_data = (bits & 4) != 0;
        t.height = h[40] as i8;
        t.terrain_type = h[41] as i8;
        t.ramp_type = h[42] as i8;
        for c in 0..3 {
            t.radar_left[c] = h[43 + c];
            t.radar_right[c] = h[46 + c];
        }

        // 图标紧跟头之后，z 面又紧跟图标之后。
        let icon_at = hdr_at + 52;
        let diamond_bytes = (out.cx as usize * out.cy as usize) / 2;
        if icon_at + diamond_bytes > size {
            continue;
        }
        t.pixels = decode_diamond(&data[icon_at..], out.cx, out.cy);

        if t.has_z_data && icon_at + 2 * diamond_bytes <= size {
            t.z_data = decode_diamond(&data[icon_at + diamond_bytes..], out.cx, out.cy);
        }

        if t.has_extra_data && t.cx_extra > 0 && t.cy_extra > 0 {
            let ext_at = if extra_ofs > 0 {
                hdr_at + extra_ofs as usize
            } else {
                icon_at + diamond_bytes
            };
            let ext_bytes = t.cx_extra as usize * t.cy_extra as usize;
            if ext_at + ext_bytes <= size {
                t.extra = data[ext_at..ext_at + ext_bytes].to_vec();
                // 覆盖物自己的 z 面就在覆盖物图像之后，只要这张砖带 z 面就有。
                if t.has_z_data && ext_at + 2 * ext_bytes <= size {
                    t.extra_z = data[ext_at + ext_bytes..ext_at + 2 * ext_bytes].to_vec();
                }
            }
        }
    }
    Some(out)
}

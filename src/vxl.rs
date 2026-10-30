//! VXL（体素动画）解析 —— 对等移植。
//!
//! 来源：`I:\WeaverWeb\js\core\preview.js` 的 `parseVxl`(:523) 与 `decodeVxlSpans`(:478)。
//! 那两份实现在 WeaverWeb 里对 18 个样本 / 游戏原版文件实测通过（「坏段 = 0」），本条照抄其算法，
//! 不发明、不"等价改写"。
//!
//! 文件布局（原注释口径）：
//! ```text
//! 主头 802 B
//!   FileType[16]  "Voxel Animation"
//!   i32 @16  NumPalettes
//!   i32 @20  NumSections
//!   i32 @24  NumSections2
//!   i32 @28  BodySize
//!   u8  @32  起止重映射起点
//!   u8  @33  起止重映射终点
//!   u8  @34  PaletteData[768]         == 256 × RGB，直接用（本步不做 VPL 光照）
//! 段头 28 B × N，从 802 起连续
//!   Name[16] | i32 Number | i32 U1(=1) | i32 U2(=2)
//! 段体（长 BodySize，从 802 + N*28 起）
//!   每段三块：spanStart 表 i32[X*Y]、spanEnd 表 i32[X*Y]、span 数据
//! 段尾 92 B × N，排在文件**末尾**、连续
//!   i32 @0  SpanStartOffset   ⚠️ 相对 BodyOfs
//!   i32 @4  SpanEndOffset     ⚠️ 相对 BodyOfs
//!   i32 @8  SpanDataOffset    ⚠️ 相对 BodyOfs
//!   f32 @12 Det               HVA 位移的单位（实测 1/12 或 10/12）
//!   f32 @16 Transform[12]
//!   f32 @64 MinBounds[3]      == -(Size/2)，居中量
//!   f32 @76 MaxBounds[3]
//!   u8  @88 XSize | u8 @89 YSize | u8 @90 ZSize | u8 @91 NormalsType
//! ```
//!
//! span 数据的编码是**列 RLE 的三重块**（`Voxel.pas:1001-1058`）：
//! `skip(u8) → count(u8) → count × (color u8, normal u8) → repeat(u8)`，
//! 其中 `repeat` 必须等于 `count`（格式自带校验，不等即记一次 `bad`）。

#[derive(Debug, Clone, Copy)]
pub struct Voxel {
    pub x: u8,
    pub y: u8,
    pub z: u8,
    pub color: u8,
    pub normal: u8,
}

#[derive(Debug, Clone)]
pub struct VxlSection {
    pub name: String,
    pub number: i32,
    pub x_size: u8,
    pub y_size: u8,
    pub z_size: u8,
    pub normals_type: u8,
    /// 尾段 @12 的 f32。C++ 那边叫 `hvaMultiplier`：HVA 位移的单位（实测 1/12 或 10/12）。
    pub hva_multiplier: f32,
    /// min[3] + max[3]（尾段 @64 / @76）。模型被归一化进这两个界里。
    pub bounds: [f32; 6],
    pub voxels: Vec<Voxel>,
    pub bad: i32,
}

impl VxlSection {
    pub fn span(&self, axis: usize) -> f32 {
        self.bounds[3 + axis] - self.bounds[axis]
    }

    /// `VxlSection::scaleOf`：格边长 = 跨度 / 该轴格数。**是算出来的，不是存着的**。
    pub fn scale_of(&self, axis: usize) -> f32 {
        let n = match axis {
            0 => self.x_size,
            1 => self.y_size,
            _ => self.z_size,
        } as f32;
        if n <= 0.0 {
            return 0.0;
        }
        self.span(axis) / n
    }
}

#[derive(Debug, Clone)]
pub struct Vxl {
    pub sections: Vec<VxlSection>,
    pub palette: Vec<u8>,
}

fn i32_at(b: &[u8], o: usize) -> Option<i32> {
    let s = b.get(o..o + 4)?;
    Some(i32::from_le_bytes([s[0], s[1], s[2], s[3]]))
}

fn f32_at(b: &[u8], o: usize) -> Option<f32> {
    let s = b.get(o..o + 4)?;
    Some(f32::from_le_bytes([s[0], s[1], s[2], s[3]]))
}

fn cstr(b: &[u8], o: usize, n: usize) -> String {
    let mut s = String::new();
    for i in 0..n {
        let Some(&c) = b.get(o + i) else { break };
        if c == 0 {
            break;
        }
        s.push(c as char);
    }
    s
}

fn i32_array(b: &[u8], o: usize, count: usize) -> Vec<i32> {
    let mut out = vec![0i32; count];
    for (i, slot) in out.iter_mut().enumerate() {
        *slot = i32_at(b, o + i * 4).unwrap_or(0);
    }
    out
}

fn decode_spans(
    xs: usize,
    ys: usize,
    zs: usize,
    ss: &[i32],
    se: &[i32],
    sd: &[u8],
) -> (Vec<Voxel>, i32) {
    let mut voxels = Vec::new();
    let mut bad = 0i32;
    for y in 0..ys {
        for x in 0..xs {
            let i = y * xs + x;
            let (Some(&start), Some(&end)) = (ss.get(i), se.get(i)) else { continue };
            if start == -1 || end == -1 {
                continue;
            }
            let mut z: usize = 0;
            let mut ofs: usize = start as usize;
            while z < zs && (z as i32) < end {
                if ofs >= sd.len() {
                    break;
                }
                z += sd[ofs] as usize;
                ofs += 1;
                let num = if ofs < sd.len() { sd[ofs] } else { 0 };
                ofs += 1;
                if z >= zs {
                    break;
                }
                for _ in 0..num {
                    if ofs + 1 >= sd.len() {
                        break;
                    }
                    voxels.push(Voxel {
                        x: x as u8,
                        y: y as u8,
                        z: z as u8,
                        color: sd[ofs],
                        normal: sd[ofs + 1],
                    });
                    ofs += 2;
                    z += 1;
                }
                let n2: i32 = if ofs < sd.len() { sd[ofs] as i32 } else { -1 };
                ofs += 1;
                if n2 != num as i32 {
                    bad += 1;
                }
            }
        }
    }
    (voxels, bad)
}

pub fn parse_vxl(b: &[u8]) -> Option<Vxl> {
    if b.len() < 802 {
        return None;
    }
    let nsec = i32_at(b, 20)?;
    let body_size = i32_at(b, 28)?;
    if nsec <= 0 || nsec > 4096 {
        return None;
    }
    let nsec = nsec as usize;
    let head_ofs = 802usize;
    let body_ofs = head_ofs + nsec * 28;
    let tail_ofs = body_ofs + body_size.max(0) as usize;
    if tail_ofs + nsec * 92 > b.len() {
        return None;
    }

    let palette = b.get(34..34 + 768).map(|s| s.to_vec()).unwrap_or_default();

    let mut sections = Vec::with_capacity(nsec);
    for i in 0..nsec {
        let h = head_ofs + i * 28;
        let name = cstr(b, h, 16);
        let number = i32_at(b, h + 16).unwrap_or(0);

        let t = tail_ofs + i * 92;
        let sso = i32_at(b, t)?;
        let seo = i32_at(b, t + 4)?;
        let sdo = i32_at(b, t + 8)?;
        let hva_multiplier = f32_at(b, t + 12).unwrap_or(0.0);
        let mut bounds = [0f32; 6];
        for (k, slot) in bounds.iter_mut().enumerate() {
            *slot = f32_at(b, t + 64 + k * 4).unwrap_or(0.0);
        }        let xs = *b.get(t + 88)? as usize;
        let ys = *b.get(t + 89)? as usize;
        let zs = *b.get(t + 90)? as usize;
        let normals_type = *b.get(t + 91)?;

        let sc = xs * ys;
        let bad_span = sc == 0
            || sso < 0
            || seo < 0
            || sdo < 0
            || body_ofs + sso as usize + sc * 4 > b.len()
            || body_ofs + seo as usize + sc * 4 > b.len();
        if bad_span {
            sections.push(VxlSection {
                name,
                number,
                x_size: xs as u8,
                y_size: ys as u8,
                z_size: zs as u8,
                normals_type,
                hva_multiplier,
                bounds,
                voxels: Vec::new(),
                bad: 0,
            });
            continue;
        }

        let ss = i32_array(b, body_ofs + sso as usize, sc);
        let se = i32_array(b, body_ofs + seo as usize, sc);

        let mut i2 = sc - 1;
        while i2 > 0 && se[i2] == -1 {
            i2 -= 1;
        }
        let mut i3 = 0usize;
        while i3 < sc - 1 && ss[i3] == -1 {
            i3 += 1;
        }
        let sd_len = ((se[i2] - ss[i3] + 2).max(2)) as usize;
        let sd_start = body_ofs + sdo as usize;
        let sd_end = (sd_start + sd_len).min(b.len());
        let sd = if sd_start <= sd_end { &b[sd_start..sd_end] } else { &[] };

        let (voxels, bad) = decode_spans(xs, ys, zs, &ss, &se, sd);
        sections.push(VxlSection {
            name,
            number,
            x_size: xs as u8,
            y_size: ys as u8,
            z_size: zs as u8,
            normals_type,
            hva_multiplier,
            bounds,
            voxels,
            bad,
        });
    }
    Some(Vxl { sections, palette })
}

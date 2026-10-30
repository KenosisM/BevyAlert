//! HVA（体素动画）解析 —— 对等移植。
//!
//! 来源：`I:\bevy\RA2REASM\src\compat\vxl.cpp` 的 `parseHva` / `HvaFile::hasSection` /
//! `HvaFile::matrixFor`，以及 `tools\wwread.cpp` 的 `appendVxlPart`（多部件拼接方式）。
//!
//! 文件布局：
//! ```text
//! 头部 16 B（未用，"HVA Animation" 一类的标识）
//! i32 @16  NumFrames
//! i32 @20  SectionCount
//! 段名 SectionCount × 16 B（以 NUL 截断）
//! 矩阵 NumFrames × SectionCount × 12 个 f32
//! ```
//!
//! 矩阵按 **frame-major** 排：`(frame * sectionCount + section) * 12`。
//! 每个段是 3×4：存的是**三列**，读回来要当成**三行** —— 才是渲染器要乘的那个行向量矩阵。

#[derive(Clone, Default, Debug)]
pub struct Hva {
    pub num_frames: i32,
    pub section_names: Vec<String>,
    /// frame-major：`(frame * sectionCount + section) * 12`。
    pub values: Vec<f32>,
    pub valid: bool,
}

impl Hva {
    /// 无效或缺失的 HVA 让这里恒为假，于是每个段都退回单位变换 —— 与参考实现一致。
    pub fn has_section(&self, index: i32) -> bool {
        index >= 0 && index < self.section_names.len() as i32 && self.num_frames > 0
    }

    /// 段在 `frame` 的行向量矩阵，展开成 4x4 行主序（`out[0..4]` 是第 0 行）。
    pub fn matrix_for(&self, section: i32, frame: i32) -> [f32; 16] {
        let mut m: [f32; 16] = [
            1.0, 0.0, 0.0, 0.0, //
            0.0, 1.0, 0.0, 0.0, //
            0.0, 0.0, 1.0, 0.0, //
            0.0, 0.0, 0.0, 1.0,
        ];
        if !self.has_section(section) {
            return m;
        }
        let mut frame = frame;
        if frame < 0 {
            frame = 0;
        }
        if frame >= self.num_frames {
            frame = self.num_frames - 1;
        }
        let base = (frame as usize * self.section_names.len() + section as usize) * 12;
        if base + 12 > self.values.len() {
            return m;
        }
        let v = &self.values[base..base + 12];
        // 存的是三列；读回成三行。
        m[0] = v[0];
        m[1] = v[4];
        m[2] = v[8];
        m[3] = 0.0;
        m[4] = v[1];
        m[5] = v[5];
        m[6] = v[9];
        m[7] = 0.0;
        m[8] = v[2];
        m[9] = v[6];
        m[10] = v[10];
        m[11] = 0.0;
        m[12] = v[3];
        m[13] = v[7];
        m[14] = v[11];
        m[15] = 1.0;
        m
    }

    /// `wwread.cpp appendVxlPart`：把一个部件的 HVA 接进共享表。
    ///
    /// 只取 **frame 0** 的对应行（`base = i * 12`）；部件缺 HVA 或行越界时填单位矩阵。
    /// 结果表的 `num_frames` 固定为 1。
    pub fn append_part(&mut self, part: Option<&Hva>, section_count: usize) {
        for i in 0..section_count {
            let name = part
                .and_then(|h| h.section_names.get(i))
                .cloned()
                .unwrap_or_else(|| "part".to_string());
            self.section_names.push(name);
            let base = i * 12;
            match part {
                Some(h) if h.valid && base + 12 <= h.values.len() => {
                    for k in 0..12 {
                        self.values.push(h.values[base + k]);
                    }
                }
                _ => {
                    for k in [1.0f32, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0] {
                        self.values.push(k);
                    }
                }
            }
        }
        self.num_frames = 1;
        self.valid = true;
    }
}

fn i32_at(b: &[u8], o: usize) -> Option<i32> {
    let s = b.get(o..o + 4)?;
    Some(i32::from_le_bytes([s[0], s[1], s[2], s[3]]))
}

fn f32_at(b: &[u8], o: usize) -> f32 {
    match b.get(o..o + 4) {
        Some(s) => f32::from_le_bytes([s[0], s[1], s[2], s[3]]),
        None => 0.0,
    }
}

fn cstr16(b: &[u8], o: usize) -> String {
    let mut s = String::new();
    for i in 0..16 {
        let Some(&c) = b.get(o + i) else { break };
        if c == 0 {
            break;
        }
        s.push(c as char);
    }
    s
}

pub fn parse_hva(data: &[u8]) -> Result<Hva, String> {
    if data.len() < 24 {
        return Err("too small to be an HVA".into());
    }
    let num_frames = i32_at(data, 16).unwrap_or(0);
    let section_count = i32_at(data, 20).unwrap_or(0);
    if num_frames <= 0 || section_count <= 0 || section_count > 64 {
        return Err("bad HVA header".into());
    }
    let mut out = Hva {
        num_frames,
        ..Default::default()
    };
    for i in 0..section_count as usize {
        out.section_names.push(cstr16(data, 24 + i * 16));
    }
    let pos = 24 + section_count as usize * 16;
    let total = num_frames as usize * section_count as usize * 12;
    if pos + total * 4 > data.len() {
        return Err("HVA matrices run past the end of the file".into());
    }
    out.values.reserve(total);
    for i in 0..total {
        out.values.push(f32_at(data, pos + i * 4));
    }
    out.valid = true;
    Ok(out)
}

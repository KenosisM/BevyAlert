use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct Entry {
    pub key: String,
    pub value: String,
}

#[derive(Debug, Clone)]
pub struct Section {
    pub name: String,
    pub entries: Vec<Entry>,
}

impl Section {
    pub fn value(&self, key: &str) -> Option<&str> {
        self.entries
            .iter()
            .find(|e| e.key.eq_ignore_ascii_case(key))
            .map(|e| e.value.as_str())
    }

    pub fn number(&self, key: &str) -> Option<f64> {
        self.value(key).and_then(|v| v.trim().trim_end_matches('%').parse().ok())
    }

    pub fn flag(&self, key: &str) -> Option<bool> {
        self.value(key).map(|v| {
            let v = v.trim();
            v.eq_ignore_ascii_case("yes") || v.eq_ignore_ascii_case("true") || v == "1"
        })
    }
}

#[derive(Debug, Clone, Default)]
pub struct Ini {
    pub sections: Vec<Section>,
    index: HashMap<String, usize>,
}

impl Ini {
    pub fn parse(text: &str) -> Ini {
        let mut ini = Ini::default();
        let mut cur: Option<usize> = None;
        for raw in text.lines() {
            let line = strip_comment(raw).trim();
            if line.is_empty() {
                continue;
            }
            if let Some(rest) = line.strip_prefix('[') {
                let Some(end) = rest.find(']') else { continue };
                let name = rest[..end].trim();
                if name.is_empty() {
                    continue;
                }
                ini.sections.push(Section { name: name.to_string(), entries: Vec::new() });
                let at = ini.sections.len() - 1;
                ini.index.entry(name.to_ascii_uppercase()).or_insert(at);
                cur = Some(at);
                continue;
            }
            let Some(at) = cur else { continue };
            let Some(eq) = line.find('=') else { continue };
            let key = line[..eq].trim();
            if key.is_empty() {
                continue;
            }
            ini.sections[at].entries.push(Entry {
                key: key.to_string(),
                value: line[eq + 1..].trim().to_string(),
            });
        }
        ini
    }

    pub fn get(&self, name: &str) -> Option<&Section> {
        self.index.get(&name.to_ascii_uppercase()).map(|&i| &self.sections[i])
    }

    /// 段里每个值，按文件顺序、无分隔符拼接。
    /// 地图文件把 base64 包拆成编号 key（`1=`、`2=`……），整包就是它们的拼接。
    /// 上游 `IniFile::concatenatedValues`（ini.cpp:91-100）。
    pub fn concatenated_values(&self, name: &str) -> String {
        match self.get(name) {
            Some(s) => s.entries.iter().map(|e| e.value.as_str()).collect(),
            None => String::new(),
        }
    }

    pub fn len(&self) -> usize {
        self.sections.len()
    }
}

fn strip_comment(line: &str) -> &str {
    match line.find(';') {
        Some(i) => &line[..i],
        None => line,
    }
}

/// `strtol(s, &end, 10)` 的等价物：跳过前导空白、可选正负号、吃连续数字。
/// 返回 `(值, 消耗字节数)`；没有数字时返回 `(0, 0)`，与 C 的"`end == nptr`"一致。
///
/// 上游 ini.cpp / mapfile.cpp 都用 `std::strtol`，而 Rust 的 `str::parse` 对 `"12abc"`
/// 会整体失败给 0，strtol 给 12 —— 这个差别会静默改变解析结果，所以要照抄。
pub fn strtol10(s: &str) -> (i64, usize) {
    let b = s.as_bytes();
    let mut i = 0usize;
    while i < b.len() && (b[i] as char).is_ascii_whitespace() {
        i += 1;
    }
    let mut neg = false;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        neg = b[i] == b'-';
        i += 1;
    }
    let digits_at = i;
    let mut v: i64 = 0;
    while i < b.len() && b[i].is_ascii_digit() {
        v = v.saturating_mul(10).saturating_add((b[i] - b'0') as i64);
        i += 1;
    }
    if i == digits_at {
        return (0, 0);
    }
    (if neg { -v } else { v }, i)
}

/// 上游 `iniBool`（ini.cpp:102-108）。
pub fn ini_bool(v: &str, fallback: bool) -> bool {
    let s = v.trim().to_ascii_lowercase();
    if s.is_empty() {
        return fallback;
    }
    match s.as_str() {
        "yes" | "true" | "1" => true,
        "no" | "false" | "0" => false,
        _ => fallback,
    }
}

/// 上游 `iniInt`（ini.cpp:110-118）：空值走 fallback，认不出数字给 0。
pub fn ini_int(v: &str, fallback: i32) -> i32 {
    let s = v.trim();
    if s.is_empty() {
        return fallback;
    }
    let (n, used) = strtol10(s);
    if used == 0 {
        return fallback;
    }
    n as i32
}

/// 上游 `iniColor`（ini.cpp:120-134）：`"r,g,b"` 抠三个整数，形状不对返回 false。
pub fn ini_color(v: &str, out: &mut [i32; 3]) -> bool {
    let b = v.as_bytes();
    let mut at = 0usize;
    for slot in out.iter_mut() {
        while at < b.len() && (b[at] == b' ' || b[at] == b',') {
            at += 1;
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

//! MIX 归档读取 —— 对等移植。
//!
//! 来源：`I:\bevy\RA2REASM\src\compat\{crc32,blowfish,keyderive,mix,mixindex}.cpp`
//! 与 `ww.h` / `mixindex.h`。
//!
//! 为什么需要它：RA2 的 `ra2.mix` / `ra2md.mix` 里只有几十条记录，而且**全是别的 MIX**；
//! 真正的素材在两三层下面。所以一个名字要先展开嵌套容器才解析得了。
//!
//! 三块拼起来才是"按名字取文件"：
//!   ① 文件名 → 32 位 id（`mix_file_id`，Westwood 的补位 CRC32）
//!   ② MIX 头 → 条目表（明文 / Blowfish 加密 / TD 三种布局）
//!   ③ 嵌套展开（最多 3 层，逐个条目试探它是不是另一个 MIX）

use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::io::{Read, Seek, SeekFrom};

use crate::blowfish_tables::{BF_INIT_P, BF_INIT_S};

// ============================================================================ crc32 / id
// 来源：compat/crc32.cpp

fn crc32(data: &[u8]) -> u32 {
    // 表按需生成（上游是 g_table/g_ready 的一次性初始化）。
    let mut table = [0u32; 256];
    for (i, slot) in table.iter_mut().enumerate() {
        let mut c = i as u32;
        for _ in 0..8 {
            c = if c & 1 != 0 { 0xEDB88320 ^ (c >> 1) } else { c >> 1 };
        }
        *slot = c;
    }
    let mut c = 0xFFFFFFFFu32;
    for &b in data {
        c = table[((c ^ b as u32) & 0xFF) as usize] ^ (c >> 8);
    }
    !c
}

/// 文件名 → MIX 记录 id（上游 `mixFileId`，crc32.cpp:32-70）。
pub fn mix_file_id(name_in: &str) -> u32 {
    let name: Vec<u8> = name_in.bytes().map(|b| b.to_ascii_uppercase()).collect();

    // 8 个十六进制字符本身就是一个 id。
    if name.len() == 8 && name.iter().all(|b| b.is_ascii_hexdigit()) {
        let mut id = 0u32;
        for &b in &name {
            let d = b.to_ascii_lowercase();
            let v = if d >= b'a' { d - b'a' + 10 } else { d - b'0' };
            id = (id << 4) | v as u32;
        }
        return id;
    }

    let n = name.len();
    if n == 0 {
        return 0;
    }
    if n & 3 == 0 {
        return crc32(&name);
    }

    // Westwood 把名字补到 4 的倍数：先写"这一组的余数长度"，再用**最后一组首字符**填满。
    // 漏掉这一步会让每一次查找都落空。
    let buf_len = n + 1 + 3 - (n & 3);
    let mut buf = vec![0u8; buf_len];
    buf[..n].copy_from_slice(&name);
    let a = n & !3usize;
    let fill = buf[a];
    buf[n] = (n - a) as u8;
    for i in (n + 1)..buf_len {
        buf[i] = fill;
    }
    crc32(&buf)
}

// ============================================================================ Blowfish
// 来源：compat/blowfish.cpp + ww.h

/// 上游 `Blowfish::f`。两处加法都是**模 2^32** 的（C++ 的 u32 加法自然回绕），
/// Rust 的 debug 构建下必须显式 `wrapping_add`，否则会 panic。
fn bf_f(s: &[[u32; 256]; 4], x: u32) -> u32 {
    (s[0][((x >> 24) & 0xFF) as usize].wrapping_add(s[1][((x >> 16) & 0xFF) as usize])
        ^ s[2][((x >> 8) & 0xFF) as usize])
        .wrapping_add(s[3][(x & 0xFF) as usize])
}

/// 上游 `Blowfish::encryptBlock`。抽成自由函数是为了 key schedule 能在构造过程中
/// 一边写 `p`/`s` 一边调用它（Rust 的方法调用会借走整个 `self`）。
fn bf_encrypt_block(p: &[u32; 18], s: &[[u32; 256]; 4], l: &mut u32, r: &mut u32) {
    let (mut xl, mut xr) = (*l, *r);
    for i in 0..16 {
        xl ^= p[i];
        xr ^= bf_f(s, xl);
        std::mem::swap(&mut xl, &mut xr);
    }
    std::mem::swap(&mut xl, &mut xr);
    xr ^= p[16];
    xl ^= p[17];
    *l = xl;
    *r = xr;
}

/// 上游 `Blowfish::decryptBlock`。
fn bf_decrypt_block(p: &[u32; 18], s: &[[u32; 256]; 4], l: &mut u32, r: &mut u32) {
    let (mut xl, mut xr) = (*l, *r);
    for i in (2..=17).rev() {
        xl ^= p[i];
        xr ^= bf_f(s, xl);
        std::mem::swap(&mut xl, &mut xr);
    }
    std::mem::swap(&mut xl, &mut xr);
    xr ^= p[1];
    xl ^= p[0];
    *l = xl;
    *r = xr;
}

pub struct Blowfish {
    p: [u32; 18],
    s: [[u32; 256]; 4],
}

impl Blowfish {
    /// 上游 `Blowfish::Blowfish`（blowfish.cpp:7-36）——标准 key schedule。
    pub fn new(key: &[u8]) -> Blowfish {
        let mut bf = Blowfish {
            p: BF_INIT_P,
            s: BF_INIT_S,
        };
        let key_len = if key.is_empty() { 1 } else { key.len() };
        let key_byte = |j: usize| -> u8 {
            if key.is_empty() {
                0
            } else {
                key[j % key_len]
            }
        };

        let mut j = 0usize;
        for i in 0..18 {
            let mut d = 0u32;
            for _ in 0..4 {
                d = (d << 8) | key_byte(j) as u32;
                j += 1;
            }
            bf.p[i] ^= d;
        }

        let mut l = 0u32;
        let mut r = 0u32;
        let mut i = 0usize;
        while i < 18 {
            bf_encrypt_block(&bf.p, &bf.s, &mut l, &mut r);
            bf.p[i] = l;
            bf.p[i + 1] = r;
            i += 2;
        }
        for b in 0..4 {
            let mut i = 0usize;
            while i < 256 {
                bf_encrypt_block(&bf.p, &bf.s, &mut l, &mut r);
                bf.s[b][i] = l;
                bf.s[b][i + 1] = r;
                i += 2;
            }
        }
        bf
    }

    pub fn decrypt_block(&self, l: &mut u32, r: &mut u32) {
        bf_decrypt_block(&self.p, &self.s, l, r);
    }
}

/// Blowfish ECB 解密，8 字节一块，**块内容按大端解释**。
/// 把端序搞反会得到看起来像样、实际是垃圾的数字（上游原文）。
fn decrypt_region(bf: &Blowfish, src: &[u8]) -> Vec<u8> {
    let n = src.len();
    let mut dst = vec![0u8; n];
    let mut o = 0usize;
    while o + 8 <= n {
        let mut l = u32::from_be_bytes([src[o], src[o + 1], src[o + 2], src[o + 3]]);
        let mut r = u32::from_be_bytes([src[o + 4], src[o + 5], src[o + 6], src[o + 7]]);
        bf.decrypt_block(&mut l, &mut r);
        dst[o..o + 4].copy_from_slice(&l.to_be_bytes());
        dst[o + 4..o + 8].copy_from_slice(&r.to_be_bytes());
        o += 8;
    }
    dst
}

// ============================================================================ key derive
// 来源：compat/keyderive.cpp

/// 320 位 RSA 模数，大端。E = 0x10001。
const MODULUS_BE: [u8; 40] = [
    0x51, 0xbc, 0xda, 0x08, 0x6d, 0x39, 0xfc, 0xe4, 0x56, 0x51, 0x60, 0xd6, 0x51, 0x71, 0x3f, 0xa2,
    0xe8, 0xaa, 0x54, 0xfa, 0x66, 0x82, 0xb0, 0x4a, 0xab, 0xdd, 0x0e, 0x6a, 0xf8, 0xb0, 0xc1, 0xe6,
    0xd1, 0xfb, 0x4f, 0x3d, 0xaa, 0x43, 0x7f, 0x15,
];

/// 12 个 limb（384 位）留出余量，好让 `add_mod` 永不越界。
const LIMBS: usize = 12;

#[derive(Clone, Copy)]
struct Uint384 {
    v: [u32; LIMBS],
}

impl Uint384 {
    fn zero() -> Uint384 {
        Uint384 { v: [0; LIMBS] }
    }
    fn from_bytes_le(p: &[u8]) -> Uint384 {
        let mut u = Uint384::zero();
        for (i, &b) in p.iter().enumerate() {
            u.v[i / 4] |= (b as u32) << (8 * (i % 4));
        }
        u
    }
    fn from_bytes_be(p: &[u8]) -> Uint384 {
        let mut u = Uint384::zero();
        let n = p.len();
        for i in 0..n {
            let b = p[n - 1 - i];
            u.v[i / 4] |= (b as u32) << (8 * (i % 4));
        }
        u
    }
    fn to_bytes_lsb(&self, out: &mut [u8]) {
        for (i, slot) in out.iter_mut().enumerate() {
            *slot = ((self.v[i / 4] >> (8 * (i % 4))) & 0xFF) as u8;
        }
    }
    fn cmp(&self, b: &Uint384) -> i32 {
        for i in (0..LIMBS).rev() {
            if self.v[i] < b.v[i] {
                return -1;
            }
            if self.v[i] > b.v[i] {
                return 1;
            }
        }
        0
    }
    fn sub(&mut self, b: &Uint384) {
        let mut borrow = 0u64;
        for i in 0..LIMBS {
            // 多精度减法里单个 limb 借位是常态，必须回绕：上游是 `uint64_t` 的无符号减法，
            // Rust 的 debug 构建下普通 `-` 会 panic（release 下才回绕）。
            let t = (self.v[i] as u64)
                .wrapping_sub(b.v[i] as u64)
                .wrapping_sub(borrow);
            self.v[i] = t as u32;
            borrow = (t >> 32) & 1;
        }
    }
}

/// `a, b < n < 2^320` 且 `LIMBS = 384`，所以一次条件减就够。
fn add_mod(r: Uint384, b: &Uint384, n: &Uint384) -> Uint384 {
    let mut r = r;
    let mut carry = 0u64;
    for i in 0..LIMBS {
        let t = r.v[i] as u64 + b.v[i] as u64 + carry;
        r.v[i] = t as u32;
        carry = t >> 32;
    }
    if r.cmp(n) >= 0 {
        r.sub(n);
    }
    r
}

/// 双倍累加，不需要除法。
fn mul_mod(mut a: Uint384, b: &Uint384, n: &Uint384) -> Uint384 {
    let mut r = Uint384::zero();
    for bit in 0..320 {
        if (b.v[bit / 32] >> (bit % 32)) & 1 != 0 {
            r = add_mod(r, &a, n);
        }
        a = add_mod(a, &a, n);
    }
    r
}

fn mod_pow(mut base: Uint384, mut exp: u32, n: &Uint384) -> Uint384 {
    while base.cmp(n) >= 0 {
        base.sub(n);
    }
    let mut result = Uint384::zero();
    result.v[0] = 1;
    while exp != 0 {
        if exp & 1 != 0 {
            result = mul_mod(result, &base, n);
        }
        exp >>= 1;
        if exp != 0 {
            base = mul_mod(base, &base, n);
        }
    }
    result
}

/// 80 字节 key source（从加密 MIX 头读出来）→ 56 字节 Blowfish 密钥。
///
/// 不是固定密钥：每个加密 MIX 自带一份 80 字节 key source，过一遍 RSA 公钥运算才变成
/// Blowfish 密钥。布局同 XCC 的 `get_blowfish_key` / ccmixar 的 `blowfishKeyFromKeySource`。
pub fn blowfish_key_from_key_source(ks: &[u8]) -> [u8; 56] {
    let n = Uint384::from_bytes_be(&MODULUS_BE);
    // 两段 40 字节都是**小端**消费的，也就是 key source 相对文件是字节倒着读的。
    // 两份参考实现都一致。
    let r1 = mod_pow(Uint384::from_bytes_le(&ks[0..40]), 0x10001, &n);
    let r2 = mod_pow(Uint384::from_bytes_le(&ks[40..80]), 0x10001, &n);

    let mut buf = [0u8; 78];
    {
        let (a, b) = buf.split_at_mut(39);
        r1.to_bytes_lsb(a);
        r2.to_bytes_lsb(b);
    }
    let mut key = [0u8; 56];
    key.copy_from_slice(&buf[..56]);
    key
}

// ============================================================================ MIX 归档
// 来源：compat/mix.cpp + ww.h

const FLAG_ENCRYPTED: u32 = 0x0002_0000; // 头（索引）是 Blowfish 加密的
const FLAG_DIGEST: u32 = 0x0001_0000; // 末尾 20 字节 SHA1

#[derive(Debug, Clone, Copy, Default)]
pub struct MixEntry {
    pub id: u32,
    /// 相对数据区起点的偏移。
    pub offset: u32,
    pub size: u32,
}

#[derive(Debug, Clone, Default)]
pub struct MixArchive {
    pub flags: u32,
    pub encrypted: bool,
    pub td_header: bool,
    pub has_digest: bool,
    pub body_size: u32,
    pub data_start: u64,
    /// 第一条记录之前没有被索引的字节。
    pub leading_gap: u64,
    pub entries: Vec<MixEntry>,
}

impl MixArchive {
    pub const NONE: u32 = 0xFFFF_FFFF;

    pub fn index_of(&self, id: u32) -> u32 {
        for (i, e) in self.entries.iter().enumerate() {
            if e.id == id {
                return i as u32;
            }
        }
        Self::NONE
    }
}

/// 声明的几何是否**正好**用完这一整段？
///
/// 这是决定性的检查，也是让盲探安全的原因：一个头几个字节碰巧像 MIX 头的 .shp/.dll
/// 满足不了 `dataStart + bodySize + trailer == segmentSize`。
fn layout_ok(seg_size: u64, data_start: u64, body_size: u32, has_digest: bool) -> bool {
    data_start + body_size as u64 + if has_digest { 20 } else { 0 } == seg_size
}

/// 记录必须落在 body 内且互不重叠。
///
/// 刻意**不**要求它们严丝合缝铺满 body：真实归档会把 bodySize 向上对齐并在记录之间留小缝，
/// 有些重打包的 mod 归档甚至在第一条记录前带一段没被索引的前缀。
fn entries_ok(e: &[MixEntry], body_size: u32) -> Option<(u64, u64)> {
    if e.is_empty() {
        return None;
    }
    let mut sorted: Vec<&MixEntry> = e.iter().collect();
    sorted.sort_by_key(|x| x.offset);

    let mut end = 0u64;
    for x in &sorted {
        if x.offset as u64 + x.size as u64 > body_size as u64 {
            return None;
        }
        if (x.offset as u64) < end {
            return None; // 重叠
        }
        end = x.offset as u64 + x.size as u64;
    }
    Some((sorted[0].offset as u64, end))
}

fn read_entries(buf: &[u8], at: u64, count: u32) -> Option<Vec<MixEntry>> {
    let need = at + count as u64 * 12;
    if need > buf.len() as u64 {
        return None;
    }
    let at = at as usize;
    let mut out = Vec::with_capacity(count as usize);
    for i in 0..count as usize {
        let p = at + i * 12;
        out.push(MixEntry {
            id: u32::from_le_bytes([buf[p], buf[p + 1], buf[p + 2], buf[p + 3]]),
            offset: u32::from_le_bytes([buf[p + 4], buf[p + 5], buf[p + 6], buf[p + 7]]),
            size: u32::from_le_bytes([buf[p + 8], buf[p + 9], buf[p + 10], buf[p + 11]]),
        });
    }
    Some(out)
}

/// "一段 segSize 字节的 MIX"的公共解析器，读取全走 `read_at`。
fn open_mix_core(
    seg_size: u64,
    read_at: &dyn Fn(u64, usize) -> Option<Vec<u8>>,
) -> Result<MixArchive, String> {
    let seg_read = |off: u64, n: usize| -> Option<Vec<u8>> {
        if off > seg_size {
            return None;
        }
        let n = if off + n as u64 > seg_size {
            (seg_size - off) as usize
        } else {
            n
        };
        let buf = read_at(off, n)?;
        if buf.len() != n {
            return None;
        }
        Some(buf)
    };

    let head = seg_read(0, seg_size.min(92) as usize).ok_or("cannot read header")?;
    if head.len() < 10 {
        return Err("cannot read header".into());
    }
    let flags = u32::from_le_bytes([head[0], head[1], head[2], head[3]]);

    // --- 加密头：flags 的 0x00020000 位 --------------------------------------
    // flags(4) | keySource(80) @4 | ECB 密文 @84，密文里是 { count u16, bodySize u32, index 12*n }。
    // key source 每个文件各异，过一遍 320 位 RSA 公钥运算才成为 Blowfish 密钥。
    if flags & FLAG_ENCRYPTED != 0 {
        if seg_size < 92 {
            return Err("encrypted header truncated".into());
        }
        let key = blowfish_key_from_key_source(&head[4..84]);
        let bf = Blowfish::new(&key);

        let first = seg_read(84, 8).ok_or("cannot read key block")?;
        if first.len() < 8 {
            return Err("cannot read key block".into());
        }
        let dec = decrypt_region(&bf, &first);
        let count = u16::from_le_bytes([dec[0], dec[1]]) as u32;
        let body_size = u32::from_le_bytes([dec[2], dec[3], dec[4], dec[5]]);
        if count == 0 || count > 1_000_000 {
            return Err("encrypted index reports an implausible file count".into());
        }

        let raw = 6 + count as u64 * 12;
        let padded = (raw + 7) & !7u64;
        if 84 + padded > seg_size {
            return Err("encrypted index exceeds segment".into());
        }
        let enc = seg_read(84, padded as usize).ok_or("short encrypted index read")?;
        if (enc.len() as u64) < padded {
            return Err("short encrypted index read".into());
        }
        let idx = decrypt_region(&bf, &enc);

        let ents = read_entries(&idx, 6, count).ok_or("encrypted index truncated")?;
        let has_digest = flags & FLAG_DIGEST != 0;
        let (min_off, _max_end) =
            entries_ok(&ents, body_size).ok_or("encrypted index failed validation")?;
        // 索引是 ECB 加密的，所以 body 从下一个块边界开始 —— 除非索引本来就结束在块边界上。
        for ds in [84 + raw, 84 + padded] {
            if !layout_ok(seg_size, ds, body_size, has_digest) {
                continue;
            }
            return Ok(MixArchive {
                flags,
                encrypted: true,
                has_digest,
                body_size,
                data_start: ds,
                leading_gap: min_off,
                entries: ents,
                ..Default::default()
            });
        }
        return Err("encrypted geometry does not match segment size".into());
    }

    // --- 明文头：count u16 @4, bodySize u32 @6, index @10 --------------------
    if (flags & 0xFFFF) == 0 && ((flags & 0xFFFF_0000) & !FLAG_DIGEST) == 0 {
        let count = u16::from_le_bytes([head[4], head[5]]) as u32;
        let body_size = u32::from_le_bytes([head[6], head[7], head[8], head[9]]);
        let ds = 10 + count as u64 * 12;
        let has_digest = flags & FLAG_DIGEST != 0;
        if count != 0 && ds <= seg_size {
            if let Some(idx) = seg_read(0, ds as usize) {
                if let Some(ents) = read_entries(&idx, 10, count) {
                    if let Some((min_off, _)) = entries_ok(&ents, body_size) {
                        if layout_ok(seg_size, ds, body_size, has_digest) {
                            return Ok(MixArchive {
                                flags,
                                has_digest,
                                body_size,
                                data_start: ds,
                                leading_gap: min_off,
                                entries: ents,
                                ..Default::default()
                            });
                        }
                    }
                }
            }
        }
    }

    // --- TD 式头：count u16 @0, bodySize u32 @2, index @6 -------------------
    // 偏移 0 与 `flags` 共用，所以只有在排除掉现代布局之后才试它。这不是遗留垃圾：
    // RA2 自己的 theme.mix / thememd.mix 就是 TD 格式的音乐包。
    {
        let count = u16::from_le_bytes([head[0], head[1]]) as u32;
        let body_size = u32::from_le_bytes([head[2], head[3], head[4], head[5]]);
        if count > 0 && (count >> 12) == 0 {
            let ds = 6 + count as u64 * 12;
            if ds <= seg_size {
                if let Some(idx) = seg_read(0, ds as usize) {
                    if let Some(ents) = read_entries(&idx, 6, count) {
                        if let Some((min_off, _)) = entries_ok(&ents, body_size) {
                            let slack = seg_size as i64 - ds as i64 - body_size as i64;
                            if (0..=4096).contains(&slack) {
                                return Ok(MixArchive {
                                    td_header: true,
                                    body_size,
                                    data_start: ds,
                                    leading_gap: min_off,
                                    entries: ents,
                                    ..Default::default()
                                });
                            }
                        }
                    }
                }
            }
        }
    }

    Err("not a MIX (no layout matched)".into())
}

/// 解析整块内存里的 MIX。
pub fn open_mix(data: &[u8]) -> Result<MixArchive, String> {
    if data.len() < 10 {
        return Err("file too small".into());
    }
    let size = data.len() as u64;
    let rd = |off: u64, n: usize| -> Option<Vec<u8>> {
        if off > size {
            return None;
        }
        let n = if off + n as u64 > size {
            (size - off) as usize
        } else {
            n
        };
        Some(data[off as usize..off as usize + n].to_vec())
    };
    open_mix_core(size, &rd)
}

// ============================================================================ 文件读取

/// 随机访问读取。归档能到几百 MB，所以按区间读而不是整份吞。
pub struct FileReader {
    file: RefCell<std::fs::File>,
    size: u64,
    path: String,
}

impl FileReader {
    pub fn open(path: &str) -> std::io::Result<FileReader> {
        let file = std::fs::File::open(path)?;
        let size = file.metadata()?.len();
        Ok(FileReader {
            file: RefCell::new(file),
            size,
            path: path.to_string(),
        })
    }

    pub fn read_at(&self, offset: u64, n: usize) -> Option<Vec<u8>> {
        let mut f = self.file.borrow_mut();
        if f.seek(SeekFrom::Start(offset)).is_err() {
            return None;
        }
        let mut buf = vec![0u8; n];
        let mut got = 0usize;
        while got < n {
            match f.read(&mut buf[got..]) {
                Ok(0) => break,
                Ok(k) => got += k,
                Err(_) => return None,
            }
        }
        buf.truncate(got);
        Some(buf)
    }

    pub fn size(&self) -> u64 {
        self.size
    }

    pub fn path(&self) -> &str {
        &self.path
    }
}

/// 解析位于更大文件中 `[base, base+size)` 的 MIX，只读头和索引。
pub fn open_mix_at(fr: &FileReader, base: u64, size: u64) -> Result<MixArchive, String> {
    if size < 10 || base + size > fr.size() {
        return Err("segment out of range".into());
    }
    let rd = |off: u64, n: usize| -> Option<Vec<u8>> { fr.read_at(base + off, n) };
    open_mix_core(size, &rd)
}

// ============================================================================ AssetIndex
// 来源：compat/mixindex.cpp

const MAX_DEPTH: i32 = 3;
const NESTED_MAX_BYTES: u64 = 512 * 1024 * 1024;
/// 比 MIX 头索引还小的东西不可能是容器。
const NESTED_MIN_BYTES: u64 = 92;

#[derive(Debug, Clone, Copy, Default)]
pub struct AssetRef {
    pub source: u32,
    pub entry: u32,
}

struct MixSource {
    file_idx: u32,
    base: u64,
    size: u64,
    depth: i32,
    rank: f64,
    label: String,
    ar: MixArchive,
}

/// "按名字给我素材"这一层。
pub struct AssetIndex {
    readers: Vec<FileReader>,
    reader_by_path: HashMap<String, u32>,
    sources: Vec<MixSource>,
    by_id: HashMap<u32, AssetRef>,
    id_to_name: HashMap<u32, String>,
    skipped: Vec<String>,
    probed: HashSet<String>,
    probe_budget: usize,
    game_dir: String,
}

fn to_lower(s: &str) -> String {
    s.to_ascii_lowercase()
}

fn base_name_of(path: &str) -> String {
    match path.rfind(['/', '\\']) {
        Some(p) => path[p + 1..].to_string(),
        None => path.to_string(),
    }
}

/// 上游 `endsWithMix`（mixindex.cpp:31-34）。**必须按字节比较**：游戏目录里有中文名文件
/// （`将rules的建筑整理出建筑ID供编写AI使用.js`），按字符切片会落在 UTF-8 序列中间而 panic。
fn ends_with_mix(s: &str) -> bool {
    let b = s.as_bytes();
    b.len() >= 4 && b[b.len() - 4..].eq_ignore_ascii_case(b".mix")
}

/// XCC 的 `local mix database.dat` 里嵌的名字表。
pub fn parse_lmd_names(d: &[u8]) -> Option<Vec<String>> {
    const SIG: &[u8] = b"XCC by Olaf van der Spek";
    let sig_len = SIG.len();
    if d.len() < sig_len + 4 || &d[..sig_len] != SIG {
        return None;
    }

    // 签名和计数之间的头不是固定长度，所以找一个偏移，使名字正好在结尾处用完。
    let scan_end = d.len().min(sig_len + 64);
    for off in sig_len..scan_end {
        if off + 4 > d.len() {
            break;
        }
        let count = u32::from_le_bytes([d[off], d[off + 1], d[off + 2], d[off + 3]]);
        if count == 0 || count > 200_000 {
            continue;
        }
        let mut pos = off + 4;
        let mut out: Vec<String> = Vec::with_capacity(count as usize);
        let mut ok = true;
        for _ in 0..count {
            if pos >= d.len() {
                ok = false;
                break;
            }
            let Some(rel) = d[pos..].iter().position(|&b| b == 0) else {
                ok = false;
                break;
            };
            out.push(String::from_utf8_lossy(&d[pos..pos + rel]).into_owned());
            pos = pos + rel + 1;
        }
        if ok && pos == d.len() {
            return Some(out);
        }
    }
    None
}

/// 顶层归档的权重，对应游戏的搜索顺序。
pub fn mix_rank_of(base_name: &str) -> f64 {
    let s = to_lower(base_name);
    // expandmdNN 覆盖一切，NN 越大越优先。
    if s.len() > 8 && s.starts_with("expandmd") {
        let mut n = 0i64;
        for c in s[8..].bytes() {
            if !c.is_ascii_digit() {
                break;
            }
            n = n * 10 + (c - b'0') as i64;
        }
        return 100000.0 + n as f64;
    }
    const ORDER: [&str; 6] = [
        "ra2.mix",
        "ra2md.mix",
        "language.mix",
        "langmd.mix",
        "multimd.mix",
        "mapsmd03.mix",
    ];
    for (i, name) in ORDER.iter().enumerate() {
        if s == *name {
            return (i + 1) as f64;
        }
    }
    0.0
}

#[derive(Debug, Clone, Copy)]
pub struct TmpHit {
    pub source: u32,
    pub entry: u32,
    pub size: u64,
    pub cx: i32,
    pub cy: i32,
    pub cblocks_x: i32,
    pub cblocks_y: i32,
}

impl AssetIndex {
    pub fn new(game_dir: &str) -> AssetIndex {
        AssetIndex {
            readers: Vec::new(),
            reader_by_path: HashMap::new(),
            sources: Vec::new(),
            by_id: HashMap::new(),
            id_to_name: HashMap::new(),
            skipped: Vec::new(),
            probed: HashSet::new(),
            probe_budget: 60000,
            game_dir: game_dir.to_string(),
        }
    }

    pub fn asset_count(&self) -> usize {
        self.by_id.len()
    }
    pub fn source_count(&self) -> usize {
        self.sources.len()
    }
    pub fn name_count(&self) -> usize {
        self.id_to_name.len()
    }
    pub fn skipped(&self) -> &[String] {
        &self.skipped
    }

    fn reader_for(&mut self, path: &str) -> u32 {
        if let Some(&i) = self.reader_by_path.get(path) {
            return i;
        }
        match FileReader::open(path) {
            Ok(fr) => {
                let idx = self.readers.len() as u32;
                self.readers.push(fr);
                self.reader_by_path.insert(path.to_string(), idx);
                idx
            }
            Err(e) => {
                self.skipped.push(format!("{path}: {e}"));
                u32::MAX
            }
        }
    }

    fn add_source(&mut self, file_idx: u32, base: u64, size: u64, depth: i32, rank: f64, label: String, ar: MixArchive) {
        self.sources.push(MixSource {
            file_idx,
            base,
            size,
            depth,
            rank,
            label,
            ar,
        });
        let idx = self.sources.len() as u32 - 1;
        self.index_source(idx);
    }

    fn index_source(&mut self, src_idx: u32) {
        let s = &self.sources[src_idx as usize];
        let entries: Vec<u32> = s.ar.entries.iter().map(|e| e.id).collect();
        let rank = s.rank;
        for (i, id) in entries.into_iter().enumerate() {
            let better = match self.by_id.get(&id) {
                None => true,
                Some(cur) => rank > self.sources[cur.source as usize].rank,
            };
            if better {
                self.by_id.insert(
                    id,
                    AssetRef {
                        source: src_idx,
                        entry: i as u32,
                    },
                );
            }
        }
    }

    fn harvest_lmd(&mut self, src_idx: u32) {
        let id = mix_file_id("local mix database.dat");
        let e = self.sources[src_idx as usize].ar.index_of(id);
        if e == MixArchive::NONE {
            return;
        }
        let Ok(bytes) = self.read_ref(AssetRef {
            source: src_idx,
            entry: e,
        }) else {
            return;
        };
        let Some(names) = parse_lmd_names(&bytes) else {
            return;
        };
        for n in names {
            if n.is_empty() {
                continue;
            }
            let nid = mix_file_id(&n);
            self.id_to_name.entry(nid).or_insert(n);
        }
    }

    fn load_name_library(&mut self, path: &str) -> Result<usize, String> {
        let raw = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
        let mut cur = String::new();
        let mut added = 0usize;
        let mut flush = |cur: &mut String, added: &mut usize, id_to_name: &mut HashMap<u32, String>| {
            while cur.ends_with('\r') || cur.ends_with(' ') {
                cur.pop();
            }
            if !cur.is_empty() && !cur.starts_with(';') && !cur.starts_with('#') {
                let nid = mix_file_id(cur);
                if !id_to_name.contains_key(&nid) {
                    id_to_name.insert(nid, cur.clone());
                    *added += 1;
                }
            }
            cur.clear();
        };
        for &c in &raw {
            if c == b'\n' {
                flush(&mut cur, &mut added, &mut self.id_to_name);
            } else if c != b'\r' {
                cur.push(c as char);
            }
        }
        flush(&mut cur, &mut added, &mut self.id_to_name);
        Ok(added)
    }

    /// 扫 gameDir 下的 `*.mix`、展开嵌套、建 id→素材 表。
    pub fn build(&mut self, name_file: &str, verbose: bool) -> Result<(), String> {
        if !name_file.is_empty() {
            match self.load_name_library(name_file) {
                Ok(n) => println!(
                    "name library: {name_file} -> {} names ({n} new)",
                    self.id_to_name.len()
                ),
                Err(e) => println!("warning: {e}"),
            }
        }

        let mut archives: Vec<(f64, String)> = Vec::new();
        let entries = std::fs::read_dir(&self.game_dir)
            .map_err(|e| format!("no .mix files found in {}: {e}", self.game_dir))?;
        for ent in entries.flatten() {
            if !ent.path().is_file() {
                continue;
            }
            let name = ent.file_name().to_string_lossy().into_owned();
            if !ends_with_mix(&name) {
                continue;
            }
            archives.push((mix_rank_of(&name), ent.path().to_string_lossy().into_owned()));
        }
        if archives.is_empty() {
            return Err(format!("no .mix files found in {}", self.game_dir));
        }
        archives.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

        for (rank, path) in archives {
            let fi = self.reader_for(&path);
            if fi == u32::MAX {
                continue;
            }
            let size = self.readers[fi as usize].size();
            match open_mix_at(&self.readers[fi as usize], 0, size) {
                Ok(ar) => self.add_source(fi, 0, size, 0, rank, base_name_of(&path), ar),
                Err(e) => {
                    if verbose {
                        println!("  skip {:<18} {e}", base_name_of(&path));
                    }
                }
            }
        }
        if self.sources.is_empty() {
            return Err(format!("no readable MIX archives in {}", self.game_dir));
        }

        // 先收内嵌名字表，再展开 —— 这样新发现的嵌套容器才有名字可用。
        for i in 0..self.sources.len() {
            self.harvest_lmd(i as u32);
        }

        self.expand_nested(verbose);
        Ok(())
    }

    fn expand_nested(&mut self, verbose: bool) {
        // 层序很重要。把各层混在一起一次探完，会让几千条顶层记录吃掉全部预算，
        // 更深的容器永远轮不到。
        for round in 0..=MAX_DEPTH {
            let layer: Vec<u32> = (0..self.sources.len() as u32)
                .filter(|&i| self.sources[i as usize].depth == round)
                .collect();

            let mut new_ones = 0usize;
            for si in layer {
                let file_idx = self.sources[si as usize].file_idx;
                let src_base = self.sources[si as usize].base;
                let data_start = self.sources[si as usize].ar.data_start;
                let rank = self.sources[si as usize].rank;
                let depth = self.sources[si as usize].depth;
                // 下面 push 时 sources 可能重新分配，所以先把要用的拷出来，用位置索引而不是引用。
                let ents = self.sources[si as usize].ar.entries.clone();

                for en in ents {
                    if (en.size as u64) < NESTED_MIN_BYTES || (en.size as u64) > NESTED_MAX_BYTES {
                        continue;
                    }
                    if self.probed.len() >= self.probe_budget {
                        break;
                    }
                    let key = format!("{file_idx}#{:08X}", en.id);
                    if !self.probed.insert(key) {
                        continue;
                    }

                    let base = src_base + data_start + en.offset as u64;
                    // 这里一次误判就曾让 Web 工具递归到崩，所以 open_mix_at 坚持几何必须严格吻合。
                    let Ok(inner) = open_mix_at(&self.readers[file_idx as usize], base, en.size as u64)
                    else {
                        continue;
                    };

                    let label = {
                        let n = self.name_of(en.id);
                        if n.is_empty() {
                            format!("#{:08X}", en.id)
                        } else {
                            n
                        }
                    };
                    self.add_source(file_idx, base, en.size as u64, depth + 1, rank + 0.5, label, inner);
                    let last = self.sources.len() as u32 - 1;
                    self.harvest_lmd(last);
                    new_ones += 1;
                }
            }
            if verbose {
                println!(
                    "  depth {round}: {} sources (+{new_ones}), {} probes",
                    self.sources.len(),
                    self.probed.len()
                );
            }
            if new_ones == 0 {
                break;
            }
        }
    }

    pub fn find(&self, name: &str) -> Option<AssetRef> {
        self.by_id.get(&mix_file_id(name)).copied()
    }

    pub fn read_ref(&self, r: AssetRef) -> Result<Vec<u8>, String> {
        let Some(s) = self.sources.get(r.source as usize) else {
            return Err("bad source".into());
        };
        let Some(en) = s.ar.entries.get(r.entry as usize) else {
            return Err("bad entry".into());
        };
        let at = s.base + s.ar.data_start + en.offset as u64;
        self.readers[s.file_idx as usize]
            .read_at(at, en.size as usize)
            .ok_or_else(|| "read failed".to_string())
    }

    pub fn read(&self, name: &str) -> Result<Vec<u8>, String> {
        if let Some(r) = self.find(name) {
            return self.read_ref(r);
        }
        // 回落到游戏目录旁的散装文件：我们拿来测的参考素材（theater INI、调色板）常常没打进 MIX。
        if !self.game_dir.is_empty() {
            let p = std::path::Path::new(&self.game_dir).join(name);
            if p.is_file() {
                if let Ok(bytes) = std::fs::read(&p) {
                    return Ok(bytes);
                }
            }
        }
        Err(format!("not found: {name}"))
    }

    pub fn name_of(&self, id: u32) -> String {
        self.id_to_name.get(&id).cloned().unwrap_or_default()
    }

    pub fn names_under(&self, source_idx: u32, limit: usize) -> Vec<String> {
        let mut out = Vec::new();
        let Some(s) = self.sources.get(source_idx as usize) else {
            return out;
        };
        for e in &s.ar.entries {
            let n = self.name_of(e.id);
            out.push(if n.is_empty() {
                format!("#{:08X}", e.id)
            } else {
                n
            });
            if out.len() >= limit {
                break;
            }
        }
        out
    }

    /// 内容嗅探：一个 TMP 砖以 4 个 i32 开头，且永远是 `cy == cx/2`、`cx ∈ {48, 60}`。
    /// 只读 16 字节就够便宜，于是可以在不知道文件名的情况下定位 tileset 容器。
    pub fn find_tmp_entries(&self) -> Vec<TmpHit> {
        let mut hits = Vec::new();
        for (si, s) in self.sources.iter().enumerate() {
            for (e, en) in s.ar.entries.iter().enumerate() {
                if en.size < 16 {
                    continue;
                }
                let at = s.base + s.ar.data_start + en.offset as u64;
                let Some(head) = self.readers[s.file_idx as usize].read_at(at, 16) else {
                    continue;
                };
                if head.len() < 16 {
                    continue;
                }
                let rd = |o: usize| i32::from_le_bytes([head[o], head[o + 1], head[o + 2], head[o + 3]]);
                let (cbx, cby, cx, cy) = (rd(0), rd(4), rd(8), rd(12));
                if cbx <= 0 || cby <= 0 || cx <= 0 || cx > 256 || cy * 2 != cx {
                    continue;
                }
                let c_tiles = cbx as u64 * cby as u64;
                if 16 + c_tiles * 4 > en.size as u64 {
                    continue;
                }
                hits.push(TmpHit {
                    source: si as u32,
                    entry: e as u32,
                    size: en.size as u64,
                    cx,
                    cy,
                    cblocks_x: cbx,
                    cblocks_y: cby,
                });
            }
        }
        hits
    }
}

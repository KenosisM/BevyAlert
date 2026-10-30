//! Westwood 的两个流式解码器 —— 对等移植。
//!
//! 来源：`I:\bevy\RA2REASM\src\compat\compress.cpp` / `compress.h`。
//!
//!   LZO1X      LZO1X-1 原始流，被 "Format5" 分块容器用（IsoMapPack5）
//!   LCW        "Format80"，被 OverlayPack / OverlayDataPack 用
//!
//! 两者都只解不压：引擎从不把它们写回去。
//!
//! 【翻译约定】上游 `lzoCore` 是一堆 goto：literal/match 段的交替穿插**就是格式规范本身**，
//! 任何"改写成整齐循环"的做法都会静默产出错字节（上游注释原话）。所以这里用 Rust 的
//! label 块（`'matchloop` / `'md`）与 `break` 逐条对应 goto，控制流形状与上游一一对应。
//!
//! 【指针 → 索引】上游用 `uint8_t*`，`mPos = op - (1 + 0x0800)` 这类减法会**故意下溢**，
//! 随后靠 `if (mPos < out) goto lookbehind_overrun;` 检出。这里改用 `isize` 偏移量表达同一语义：
//! 负数即"落在了输出起点之前"，与上游的指针比较等价。所有可能回绕的减法都用 `wrapping_sub`
//! 保持上游的无符号回绕行为（Rust 的 debug 构建下普通 `-` 会 panic，上游的 u32 `--t` 不回绕到负数）。
//!
//! 【Rust 必需的自保】上游在拷贝循环外一次性验证空间（`t + 2` / `t + 5`），循环内不再检查，
//! 因为 C++ 越界写不崩。Rust 索引越界会 panic，所以拷贝循环内额外挡 `op >= out_cap`
//! 与 `m_pos` 越出输出范围；这两条在正常流上永不触发（外层已保证空间），只为不 panic。
//! 这是基础设施差异导致的自保，**不是**对上游语义的改动，特此写明。

// 上游 compress.cpp:9-13
const OUTPUT_OVERRUN: i32 = -5;
const LOOKBEHIND_OVERRUN: i32 = -6;
/// LZO 一次匹配/字面量拷贝按 4 字节对齐，可能合法地越过逻辑末尾最多 3 字节，
/// 所以输出缓冲带余量（上游 compress.cpp:15）。
const SLACK: usize = 16;

/// LZO1X-1 解压内核，是上游 `lzoCore` 的逐行翻译。
///
/// 返回 `(status, out_len)`：`status >= 0` 即成功（0 = 遇到 EOF 标记），
/// 负数是 `OUTPUT_OVERRUN` / `LOOKBEHIND_OVERRUN` / `-1`（空输入）。
fn lzo_core(input: &[u8], out: &mut [u8]) -> (i32, usize) {
    let in_len = input.len();
    let out_cap = out.len();
    let mut ip: usize = 0;
    let mut op: usize = 0;
    let mut t: u32 = 0;
    let mut m_pos: isize = 0;
    let mut got_first_literal_run = false;
    let mut got_match = false;

    if in_len == 0 {
        return (-1, 0);
    }

    // 上游 compress.cpp:41-60：文件开头的第一个 literal run 有两种编码。
    if input[ip] > 17 {
        let first = input[ip] as u32;
        ip += 1;
        t = first - 17;
        if t < 4 {
            if out_cap - op < t as usize {
                return (OUTPUT_OVERRUN, op);
            }
            for _ in 0..t {
                out[op] = input[ip];
                op += 1;
                ip += 1;
            }
            if ip >= in_len {
                return (OUTPUT_OVERRUN, op);
            }
            t = input[ip] as u32;
            ip += 1;
        } else {
            if out_cap - op < t as usize {
                return (OUTPUT_OVERRUN, op);
            }
            if in_len - ip < t as usize {
                return (OUTPUT_OVERRUN, op);
            }
            for _ in 0..t {
                out[op] = input[ip];
                op += 1;
                ip += 1;
            }
            got_first_literal_run = true;
        }
    }

    // 上游 compress.cpp:62 的主 `while (true)`。
    loop {
        // 上游 `first_literal_run:` 标签落在 literal 拷贝之后，也就是说 literal 路径执行完
        // **自然掉进**它；只有 `t >= 16` 时才是 `goto match` 跳过它。
        let mut do_first_literal_run = true;

        if got_first_literal_run {
            got_first_literal_run = false;
        } else {
            if ip >= in_len {
                return (OUTPUT_OVERRUN, op);
            }
            t = input[ip] as u32;
            ip += 1;
            if t >= 16 {
                do_first_literal_run = false;
            } else {
                if t == 0 {
                    loop {
                        if ip >= in_len {
                            return (OUTPUT_OVERRUN, op);
                        }
                        if input[ip] != 0 {
                            break;
                        }
                        t += 255;
                        ip += 1;
                    }
                    t += 15 + input[ip] as u32;
                    ip += 1;
                }
                if out_cap - op < t as usize + 3 {
                    return (OUTPUT_OVERRUN, op);
                }
                if in_len - ip < 4 {
                    return (OUTPUT_OVERRUN, op);
                }
                out[op..op + 4].copy_from_slice(&input[ip..ip + 4]);
                op += 4;
                ip += 4;
                t -= 1;
                if t > 0 {
                    if t >= 4 {
                        loop {
                            if in_len - ip < 4 {
                                return (OUTPUT_OVERRUN, op);
                            }
                            if out_cap - op < 4 {
                                return (OUTPUT_OVERRUN, op);
                            }
                            out[op..op + 4].copy_from_slice(&input[ip..ip + 4]);
                            op += 4;
                            ip += 4;
                            t -= 4;
                            if t < 4 {
                                break;
                            }
                        }
                        while t > 0 {
                            if ip >= in_len || op >= out_cap {
                                return (OUTPUT_OVERRUN, op);
                            }
                            out[op] = input[ip];
                            op += 1;
                            ip += 1;
                            t -= 1;
                        }
                    } else {
                        while t > 0 {
                            if ip >= in_len || op >= out_cap {
                                return (OUTPUT_OVERRUN, op);
                            }
                            out[op] = input[ip];
                            op += 1;
                            ip += 1;
                            t -= 1;
                        }
                    }
                }
            }
        }

        // 上游 `first_literal_run:`（compress.cpp:110-127）
        if do_first_literal_run {
            if ip >= in_len {
                return (OUTPUT_OVERRUN, op);
            }
            t = input[ip] as u32;
            ip += 1;
            if t < 16 {
                m_pos = op as isize - (1 + 0x0800);
                m_pos -= (t >> 2) as isize;
                if ip >= in_len {
                    return (OUTPUT_OVERRUN, op);
                }
                let back = input[ip] as isize;
                ip += 1;
                m_pos -= back << 2;
                if m_pos < 0 {
                    return (LOOKBEHIND_OVERRUN, op);
                }
                if out_cap - op < 3 {
                    return (OUTPUT_OVERRUN, op);
                }
                out[op] = out[m_pos as usize];
                op += 1;
                m_pos += 1;
                out[op] = out[m_pos as usize];
                op += 1;
                m_pos += 1;
                out[op] = out[m_pos as usize];
                op += 1;
                got_match = true;
            }
            // t >= 16 时直接进 match（等价于上游的 `goto match`）。
        }

        // 上游 `match:` 的 `do { ... } while (true)`（compress.cpp:129-207）。
        'matchloop: loop {
            // `match_done:` 标签（compress.cpp:196）——`goto match_done` 就是 `break 'md`。
            'md: {
                if got_match {
                    got_match = false;
                    break 'md;
                }
                if t >= 64 {
                    m_pos = op as isize - 1;
                    m_pos -= ((t >> 2) & 7) as isize;
                    if ip >= in_len {
                        return (OUTPUT_OVERRUN, op);
                    }
                    let back = input[ip] as isize;
                    ip += 1;
                    m_pos -= back << 3;
                    t = (t >> 5) - 1;
                    if m_pos < 0 {
                        return (LOOKBEHIND_OVERRUN, op);
                    }
                    if out_cap - op < t as usize + 2 {
                        return (OUTPUT_OVERRUN, op);
                    }
                    out[op] = out[m_pos as usize];
                    op += 1;
                    m_pos += 1;
                    out[op] = out[m_pos as usize];
                    op += 1;
                    m_pos += 1;
                    // 上游 `do { ... } while (--t > 0);` = 执行 t 次。
                    loop {
                        if op >= out_cap || m_pos < 0 || m_pos as usize >= out_cap {
                            return (OUTPUT_OVERRUN, op);
                        }
                        out[op] = out[m_pos as usize];
                        op += 1;
                        m_pos += 1;
                        t = t.wrapping_sub(1);
                        if t == 0 {
                            break;
                        }
                    }
                    break 'md;
                } else if t >= 32 {
                    t &= 31;
                    if t == 0 {
                        loop {
                            if ip >= in_len {
                                return (OUTPUT_OVERRUN, op);
                            }
                            if input[ip] != 0 {
                                break;
                            }
                            t += 255;
                            ip += 1;
                        }
                        t += 31 + input[ip] as u32;
                        ip += 1;
                    }
                    m_pos = op as isize - 1;
                    if in_len - ip < 2 {
                        return (OUTPUT_OVERRUN, op);
                    }
                    let w = (input[ip] as u32 | ((input[ip + 1] as u32) << 8)) >> 2;
                    ip += 2;
                    m_pos -= w as isize;
                } else if t >= 16 {
                    m_pos = op as isize;
                    m_pos -= ((t & 8) << 11) as isize;
                    t &= 7;
                    if t == 0 {
                        loop {
                            if ip >= in_len {
                                return (OUTPUT_OVERRUN, op);
                            }
                            if input[ip] != 0 {
                                break;
                            }
                            t += 255;
                            ip += 1;
                        }
                        t += 7 + input[ip] as u32;
                        ip += 1;
                    }
                    if in_len - ip < 2 {
                        return (OUTPUT_OVERRUN, op);
                    }
                    let w = (input[ip] as u32 | ((input[ip + 1] as u32) << 8)) >> 2;
                    ip += 2;
                    m_pos -= w as isize;
                    if m_pos == op as isize {
                        return (0, op); // eof_found
                    }
                    m_pos -= 0x4000;
                } else {
                    m_pos = op as isize - 1;
                    m_pos -= (t >> 2) as isize;
                    if ip >= in_len {
                        return (OUTPUT_OVERRUN, op);
                    }
                    let back = input[ip] as isize;
                    ip += 1;
                    m_pos -= back << 2;
                    if m_pos < 0 {
                        return (LOOKBEHIND_OVERRUN, op);
                    }
                    if out_cap - op < 2 {
                        return (OUTPUT_OVERRUN, op);
                    }
                    out[op] = out[m_pos as usize];
                    op += 1;
                    m_pos += 1;
                    out[op] = out[m_pos as usize];
                    op += 1;
                    break 'md;
                }

                // 共用拷贝尾（上游 compress.cpp:174-193）：t>=32 与 t>=16 落到这里。
                if m_pos < 0 {
                    return (LOOKBEHIND_OVERRUN, op);
                }
                if out_cap - op < t as usize + 5 {
                    return (OUTPUT_OVERRUN, op);
                }
                if t >= 6 && (op as isize - m_pos) >= 4 {
                    // 上游 `memcpy(op, mPos, 4)`：`op - mPos >= 4`（上面的条件）保证源与目标不重叠，
                    // 所以这里经一个 4 字节暂存即可 —— Rust 不允许同时可变/不可变借用同一个 slice。
                    let mut word = [0u8; 4];
                    word.copy_from_slice(&out[m_pos as usize..m_pos as usize + 4]);
                    out[op..op + 4].copy_from_slice(&word);
                    op += 4;
                    m_pos += 4;
                    t -= 2;
                    loop {
                        let mut word = [0u8; 4];
                        word.copy_from_slice(&out[m_pos as usize..m_pos as usize + 4]);
                        out[op..op + 4].copy_from_slice(&word);
                        op += 4;
                        m_pos += 4;
                        t -= 4;
                        if t < 4 {
                            break;
                        }
                    }
                    while t > 0 {
                        out[op] = out[m_pos as usize];
                        op += 1;
                        m_pos += 1;
                        t -= 1;
                    }
                } else {
                    out[op] = out[m_pos as usize];
                    op += 1;
                    m_pos += 1;
                    out[op] = out[m_pos as usize];
                    op += 1;
                    m_pos += 1;
                    // 上游 `do { ... } while (--t > 0);`
                    loop {
                        if op >= out_cap || m_pos < 0 || m_pos as usize >= out_cap {
                            return (OUTPUT_OVERRUN, op);
                        }
                        out[op] = out[m_pos as usize];
                        op += 1;
                        m_pos += 1;
                        t = t.wrapping_sub(1);
                        if t == 0 {
                            break;
                        }
                    }
                }
            }

            // 上游 `match_done:`（compress.cpp:196-206）
            if ip < 2 {
                return (OUTPUT_OVERRUN, op);
            }
            t = (input[ip - 2] & 3) as u32;
            if t == 0 {
                break 'matchloop;
            }
            if out_cap - op < t as usize || in_len - ip < t as usize {
                return (OUTPUT_OVERRUN, op);
            }
            out[op] = input[ip];
            op += 1;
            ip += 1;
            if t > 1 {
                out[op] = input[ip];
                op += 1;
                ip += 1;
                if t > 2 {
                    out[op] = input[ip];
                    op += 1;
                    ip += 1;
                }
            }
            if ip >= in_len {
                return (OUTPUT_OVERRUN, op);
            }
            t = input[ip] as u32;
            ip += 1;
        }
        // `break 'matchloop` 之后回到主循环顶部，与上游 `while (true)` 一致。
    }
}

/// 原始 LZO1X-1 解压。最多写 `out_cap` 字节并返回写了多少。流损坏时返回 `None`。
/// 绝不读过 `input` 的末尾。
pub fn lzo1x_decode(input: &[u8], out: &mut [u8]) -> Option<usize> {
    // 上游 `if (!in || !out) return false;` 是判空指针；Rust 的 slice 不可能是空指针，
    // 能对应的只有"输入为空"（lzoCore 内部会返回 -1）。out 为空则没有可写目标，也无意义。
    if input.is_empty() || out.is_empty() {
        return None;
    }
    let (status, len) = lzo_core(input, out);
    if status >= 0 {
        Some(len)
    } else {
        None
    }
}

/// LCW 的 `emitFrom`：从 `src` 取 `count` 字节写到 `*w`（`src` 可以落在写头之后 —— 游程就是这么编码的）。
/// 返回 `Err(())` 表示引用了还没产出的数据；`Ok(true)` 表示输出满了、提前收工。
fn lcw_emit_from(out: &mut [u8], src: usize, count: usize, w: &mut usize) -> Result<bool, ()> {
    let cap = out.len();
    if src >= *w {
        return Err(());
    }
    for i in 0..count {
        if *w >= cap {
            return Ok(true);
        }
        out[*w] = out[src + i];
        *w += 1;
    }
    Ok(false)
}

/// LCW（"Format80"）。遇到 end-of-image 命令或输出填满就停。
/// 只有输入在命令中途耗尽才返回 `None`。
pub fn lcw_decode(input: &[u8], out: &mut [u8]) -> Option<usize> {
    let in_len = input.len();
    let out_cap = out.len();
    let mut r: usize = 0;
    let mut w: usize = 0;

    loop {
        if r >= in_len {
            return None;
        }
        let code = input[r];
        r += 1;
        let mut stop = false;

        if (code & 0x80) == 0 {
            // 命令 0（0ccc pppp p）：从 `w - ppppppppppp` 拷 `ccc + 3` 字节
            if r >= in_len {
                return None;
            }
            let count = (code >> 4) as usize + 3;
            let back = ((code & 0x0F) as usize) << 8 | input[r] as usize;
            r += 1;
            if back > w {
                return None;
            }
            match lcw_emit_from(out, w - back, count, &mut w) {
                Ok(s) => stop = s,
                Err(()) => return None,
            }
        } else if (code & 0x40) == 0 {
            // 命令 1（10cc cccc）：拷 `cc cccc` 个字面量字节；0 表示图像结束
            let count = (code & 0x3F) as usize;
            if count == 0 {
                return Some(w);
            }
            if r + count > in_len {
                return None;
            }
            for _ in 0..count {
                if w >= out_cap {
                    stop = true;
                    break;
                }
                out[w] = input[r];
                w += 1;
                r += 1;
            }
        } else {
            let c3 = (code & 0x3F) as usize;
            if c3 == 0x3E {
                // 命令 3（11111110 c c v）：用 `v` 填 `c c` 字节
                if r + 3 > in_len {
                    return None;
                }
                let count = input[r] as usize | (input[r + 1] as usize) << 8;
                let v = input[r + 2];
                r += 3;
                for _ in 0..count {
                    if w >= out_cap {
                        stop = true;
                        break;
                    }
                    out[w] = v;
                    w += 1;
                }
            } else if c3 == 0x3F {
                // 命令 4（11111111 c c p p）：从绝对位置 `p p` 拷 `c c` 字节
                if r + 4 > in_len {
                    return None;
                }
                let count = input[r] as usize | (input[r + 1] as usize) << 8;
                let src = input[r + 2] as usize | (input[r + 3] as usize) << 8;
                r += 4;
                match lcw_emit_from(out, src, count, &mut w) {
                    Ok(s) => stop = s,
                    Err(()) => return None,
                }
            } else {
                // 命令 2（11cc cccc p p）：从绝对位置 `p p` 拷 `cc cccc + 3` 字节
                if r + 2 > in_len {
                    return None;
                }
                let count = c3 + 3;
                let src = input[r] as usize | (input[r + 1] as usize) << 8;
                r += 2;
                match lcw_emit_from(out, src, count, &mut w) {
                    Ok(s) => stop = s,
                    Err(()) => return None,
                }
            }
        }

        if stop {
            return Some(w);
        }
    }
}

/// "Format5" / "Format80" 分块容器：一串 `u16 压缩长 | u16 解压长 | 体`，
/// 解到目标填满或流耗尽为止。
///
/// `dest` 必须**事先**持有"流没覆盖到的尾巴"该有的内容 —— 地图文件依赖这一点
/// （没被覆盖的格子保持它的哨兵字节）。`format80` 为真时块体用 LCW，否则用 LZO。
/// 返回写入了多少字节。
pub fn decode_chunked(input: &[u8], dest: &mut [u8], format80: bool) -> usize {
    let in_len = input.len();
    let dest_cap = dest.len();
    if in_len == 0 || dest_cap == 0 {
        return 0;
    }

    let mut scratch: Vec<u8> = Vec::new();
    if !format80 {
        scratch = vec![0u8; 0xFFFF + SLACK];
    }

    let mut r: usize = 0;
    let mut w: usize = 0;
    while w < dest_cap {
        if r + 4 > in_len {
            break;
        }
        let size_in = input[r] as usize | (input[r + 1] as usize) << 8;
        let size_out = input[r + 2] as usize | (input[r + 3] as usize) << 8;
        r += 4;
        if size_in == 0 || size_out == 0 {
            break;
        }
        if r + size_in > in_len {
            break;
        }
        if size_out > dest_cap - w {
            break;
        }

        if format80 {
            // LCW 的反向引用留在块内，所以直接解进目标。
            // 上游忽略返回值（它按 `sizeOut` 推进写头，不信解码器报的长度），这里照办。
            let _ = lcw_decode(&input[r..r + size_in], &mut dest[w..w + size_out]);
            w += size_out;
        } else {
            let produced = match lzo1x_decode(&input[r..r + size_in], &mut scratch) {
                Some(n) => n,
                None => break,
            };
            if produced > size_out {
                break;
            }
            dest[w..w + produced].copy_from_slice(&scratch[..produced]);
            w += produced;
        }
        r += size_in;
    }
    w
}

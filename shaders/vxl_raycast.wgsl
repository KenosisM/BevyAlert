// VXL 体素光线投射（本体 + 地面足迹）—— 对等转译。
//
// 来源：I:\bevy\RA2REASM\src\render\gpurenderer.cpp 的 kVoxelRaycastHLSL（D3D11 compute）。
// 逐行搬运：分支、常数、字段顺序、边界判断一处不动，只做 HLSL → WGSL 的语法改写。
// 与 HLSL 逐条对应的说明写在原地（原注释保留，改写处注明「WGSL:」）。
//
// 与 HLSL 的差异（全部是语言层面，无语义差异）：
//   - UAV 名 color → colorOut、shadow → shadowOut（HLSL 原注释已警告不能叫 color）。
//   - float3/int3/uint4 → vec3<f32>/vec3<i32>/vec4<u32>。
//   - saturate(x) → clamp(x, 0.0, 1.0)。
//   - Texture2D.Load(int3(texel,0)) → textureLoad(tex, texel, 0)。
//   - RWTexture2D 标量赋值 → textureStore(tex, coord, vec4(v, 0, 0, 0))。
//   - out 参数 → 返回值结构体。

struct Cb {
    // 与 HLSL cbuffer C 逐字段对齐（总 256 字节，host 侧按同一顺序上传）。
    cw: i32,
    ch: i32,
    vplPages: i32,
    uv: i32,
    dbg: i32,
    secIndex: i32,
    shadowDebug: i32,
    pad3: i32,
    amb: f32,
    dif: f32,
    zb: f32,
    rp: f32,
    invR0: vec4<f32>,
    invR1: vec4<f32>,
    invR2: vec4<f32>,
    invR3: vec4<f32>,
    mvpR0: vec4<f32>,
    mvpR1: vec4<f32>,
    mvpR2: vec4<f32>,
    mvpR3: vec4<f32>,
    secScale: vec4<f32>,
    secGrid: vec4<i32>,
    secLight: vec4<f32>,
    shadowCfg: vec4<i32>,
    shadowTune: vec4<f32>,
};

@group(0) @binding(0) var gSec: texture_3d<u32>;
@group(0) @binding(1) var gPalette: texture_2d<f32>;
@group(0) @binding(2) var gVpl: texture_2d<u32>;
@group(0) @binding(3) var gNormalPage: texture_2d<u32>;
@group(0) @binding(4) var gNormals: texture_2d<f32>;
@group(0) @binding(5) var gLightDepth: texture_2d<f32>;
@group(0) @binding(6) var colorOut: texture_storage_2d<rgba8uint, write>;
// HLSL 里 zBody/zShadow 是 R16_SINT。WebGPU 的 storage 纹理没有 16 位整数格式
// （合法集只有 8/16/32 位的 rgba、32 位的 r/rg/rgba），所以这里用 r32sint，语义不变。
@group(0) @binding(7) var zBody: texture_storage_2d<r32sint, write>;
@group(0) @binding(8) var zShadow: texture_storage_2d<r32sint, write>;
// HLSL 里是 R8_UINT；同样不在合法集里，改用 rgba8uint（只写 .x，读回时取首字节）。
@group(0) @binding(9) var shadowOut: texture_storage_2d<rgba8uint, write>;
@group(0) @binding(10) var<uniform> cb: Cb;

// v * M，行向量 v 与 M 的四行。
fn unproj(v: vec4<f32>) -> vec4<f32> {
    return v.x * cb.invR0 + v.y * cb.invR1 + v.z * cb.invR2 + v.w * cb.invR3;
}

// --- 光空间 -------------------------------------------------------------------
struct LightFrame {
    origin: vec3<f32>,
    dir: vec3<f32>,
    right: vec3<f32>,
    up: vec3<f32>,
    radius: f32,
};

fn lightFrameOf() -> LightFrame {
    var f: LightFrame;
    let boxMax = vec3<f32>(f32(cb.secGrid.x), f32(cb.secGrid.y), f32(cb.secGrid.z)) * cb.secScale.xyz;
    let centre = boxMax * 0.5;
    let extent = boxMax;
    f.dir = normalize(cb.secLight.xyz);
    // 与光源平行的 up 参考会让叉积为零，产生 NaN 基；任何别的轴都可以。
    // （HLSL 里这个局部变量叫 ref；ref 是 WGSL 保留字，改名 upRef。）
    let upRef = select(vec3<f32>(0.0, 1.0, 0.0), vec3<f32>(0.0, 0.0, 1.0), abs(f.dir.y) > 0.99);
    f.right = normalize(cross(upRef, f.dir));
    f.up = cross(f.dir, f.right);
    f.radius = 0.5 * length(extent);
    f.origin = centre - f.dir * f.radius;
    return f;
}

fn toLightSpace(frame: LightFrame, p: vec3<f32>) -> vec3<f32> {
    let rel = p - frame.origin;
    let uv = vec2<f32>(dot(rel, frame.right), dot(rel, frame.up)) / (2.0 * frame.radius) + 0.5;
    return vec3<f32>(uv.x, uv.y, dot(rel, frame.dir) / (2.0 * frame.radius));
}

// --- PCF tent 滤波 ------------------------------------------------------------
// 移植自 Unity 的 ShadowSamplingTent.hlsl：footprint 是一顶 45 度帐篷，每个 texel 的权重
// 是它在帐篷下的面积。可分离，所以 5x5 是 9 次双线性取样而非 25 次。

fn tentTriangleArea(triangleHeight: f32) -> f32 {
    return triangleHeight - 0.5;
}

struct Tent3 {
    computed: vec4<f32>,
    uncut: vec4<f32>,
};

// 3 宽帐篷覆盖的四个 texel 的面积，`offset` 是中心偏移，范围 [-0.5, 0.5]。
fn tent3Areas(offset: f32) -> Tent3 {
    var r: Tent3;
    let outer = (offset + 0.5) * (offset + 0.5) * 0.5;
    r.uncut.x = outer - offset;
    r.uncut.w = outer;
    r.computed.x = r.uncut.x;
    r.computed.w = r.uncut.w;
    r.uncut.y = tentTriangleArea(1.5 - offset);
    let leftSlope = min(offset, 0.0);
    r.computed.y = r.uncut.y - leftSlope * leftSlope;
    r.uncut.z = tentTriangleArea(1.5 + offset);
    let rightSlope = max(offset, 0.0);
    r.computed.z = r.uncut.z - rightSlope * rightSlope;
    return r;
}

struct Tent5 {
    a: vec3<f32>,
    b: vec3<f32>,
};

fn tent5Weights(offset: f32) -> Tent5 {
    let t = tent3Areas(offset);
    var r: Tent5;
    r.a.x = 0.16 * t.computed.x;
    r.a.y = 0.16 * t.uncut.y;
    r.a.z = 0.16 * (t.computed.y + 1.0);
    r.b.x = 0.16 * (t.computed.z + 1.0);
    r.b.y = 0.16 * t.uncut.z;
    r.b.z = 0.16 * t.computed.w;
    return r;
}

struct Tent7 {
    a: vec4<f32>,
    b: vec4<f32>,
};

// 7 宽帐篷：HLSL 里有、当前 sampleShadowPCF 未调用。照抄保留（不许删）。
fn tent7Weights(offset: f32) -> Tent7 {
    let t = tent3Areas(offset);
    var r: Tent7;
    r.a.x = 0.081632 * t.computed.x;
    r.a.y = 0.081632 * t.uncut.y;
    r.a.z = 0.081632 * (t.uncut.y + 1.0);
    r.a.w = 0.081632 * (t.uncut.y + 2.0);
    r.b.x = 0.081632 * (t.computed.z + 2.0);
    r.b.y = 0.081632 * (t.uncut.z + 1.0);
    r.b.z = 0.081632 * t.uncut.z;
    r.b.w = 0.081632 * t.computed.w;
    return r;
}

// 返回 1 = 受光，0 = 全遮蔽。
fn sampleShadowPCF(coord: vec3<f32>, tent: i32) -> f32 {
    let res = cb.shadowCfg.x;
    if (res <= 0) {
        return 1.0;
    }
    if (coord.x < 0.0 || coord.y < 0.0 || coord.x > 1.0 || coord.y > 1.0) {
        return 1.0;
    }

    if (tent <= 0) {
        let texel = vec2<i32>(clamp(coord.xy, vec2<f32>(0.0, 0.0), vec2<f32>(1.0, 1.0)) * f32(res));
        let d = textureLoad(gLightDepth, texel, 0).r;
        return select(0.0, 1.0, coord.z - cb.shadowTune.x <= d);
    }

    let centre = coord.xy * f32(res);
    let nearest = floor(centre + 0.5);
    let offset = centre - nearest;

    let wx = tent5Weights(offset.x);
    let wy = tent5Weights(offset.y);
    // 每次双线性取样覆盖 2x2 texel，所以取样的权重是这两个 texel 权重之和。
    let fetchW = vec3<f32>(wx.a.x, wx.a.z, wx.b.y) + vec3<f32>(wx.a.y, wx.b.x, wx.b.z);
    let fetchV = vec3<f32>(wy.a.x, wy.a.z, wy.b.y) + vec3<f32>(wy.a.y, wy.b.x, wy.b.z);
    let fetchOffX = vec3<f32>(wx.a.y, wx.b.x, wx.b.z) / fetchW + vec3<f32>(-2.5, -0.5, 1.5);
    let fetchOffY = vec3<f32>(wy.a.y, wy.b.x, wy.b.z) / fetchV + vec3<f32>(-2.5, -0.5, 1.5);
    let origin = nearest / f32(res);

    var sum = 0.0;
    for (var i = 0; i < 3; i = i + 1) {
        for (var j = 0; j < 3; j = j + 1) {
            let uv = origin + vec2<f32>(fetchOffX[i], fetchOffY[j]) / f32(res);
            let texel = vec2<i32>(clamp(uv, vec2<f32>(0.0, 0.0), vec2<f32>(1.0, 1.0)) * f32(res));
            let d = textureLoad(gLightDepth, texel, 0).r;
            sum = sum + fetchW[i] * fetchV[j] * select(0.0, 1.0, coord.z - cb.shadowTune.x <= d);
        }
    }
    return sum;
}

// --- 本体 ---------------------------------------------------------------------
@compute @workgroup_size(8, 8, 1)
fn cs(@builtin(global_invocation_id) dtid: vec3<u32>) {
    if (dtid.x >= u32(cb.cw) || dtid.y >= u32(cb.ch)) {
        return;
    }
    let cx = i32(dtid.x);
    let cy = i32(dtid.y);
    // 调试（cb.dbg == 4）：把体素盒 y = dim.y/2 那一层切片直接摊到画布左上角。
    // 与 DDA 完全无关，用来确认 3D 纹理到底有没有内容。
    if (cb.dbg == 4) {
        let dimD = cb.secGrid.xyz;
        let px = i32(dtid.x);
        let py = i32(dtid.y);
        if (px < dimD.x && py < dimD.z) {
            let v = textureLoad(gSec, vec3<i32>(px, dimD.y / 2, py), 0);
            let on = v.a > 0u && v.r > 0u;
            textureStore(
                colorOut,
                vec2<i32>(dtid.xy),
                select(vec4<u32>(0u, 0u, 0u, 255u), vec4<u32>(255u, 255u, 255u, 255u), on),
            );
        }
        return;
    }
    let ndcx = (f32(cx) - 0.5 * f32(cb.cw)) / (0.5 * f32(cb.cw));
    let ndcy = -(f32(cy) - 0.5 * f32(cb.ch)) / (0.5 * f32(cb.ch));
    // matOrthoGL 是 GL 约定：NDC z 跨 [-1, 1]。
    let m0 = unproj(vec4<f32>(ndcx, ndcy, -1.0, 1.0));
    let m1 = unproj(vec4<f32>(ndcx, ndcy, 1.0, 1.0));
    // invMvp 反演整条链，所以 m0/m1 回到段自身的 pre-frame 空间：体素 (i,j,k) 位于
    // (i*scale.x, j*scale.y, k*scale.z)，**没有** minBounds 偏移 —— minBounds 已经作为
    // sectionFrame 的 T(minBounds) 烘进矩阵里了。
    let g0 = m0.xyz / cb.secScale.xyz;
    let g1 = m1.xyz / cb.secScale.xyz;
    let dir = g1 - g0;
    let dim = cb.secGrid.xyz;
    var tEnter: f32 = -1e30;
    var tExit: f32 = 1e30;
    var stp = vec3<i32>(0, 0, 0);
    var tMax = vec3<f32>(0.0, 0.0, 0.0);
    var tDelta = vec3<f32>(0.0, 0.0, 0.0);
    var cur = vec3<i32>(0, 0, 0);
    var skip = false;

    if (abs(dir.x) < 1e-9) {
        if (g0.x < 0.0 || g0.x >= f32(dim.x)) {
            skip = true;
        }
        stp.x = 0;
        tDelta.x = 1e30;
        tMax.x = 1e30;
        cur.x = 0;
    } else {
        let invD = 1.0 / dir.x;
        let t0a = (0.0 - g0.x) * invD;
        let t1a = (f32(dim.x) - g0.x) * invD;
        tEnter = max(tEnter, min(t0a, t1a));
        tExit = min(tExit, max(t0a, t1a));
        stp.x = select(-1, 1, dir.x > 0.0);
        tDelta.x = abs(invD);
    }
    if (abs(dir.y) < 1e-9) {
        if (g0.y < 0.0 || g0.y >= f32(dim.y)) {
            skip = true;
        }
        stp.y = 0;
        tDelta.y = 1e30;
        tMax.y = 1e30;
        cur.y = 0;
    } else {
        let invD = 1.0 / dir.y;
        let t0a = (0.0 - g0.y) * invD;
        let t1a = (f32(dim.y) - g0.y) * invD;
        tEnter = max(tEnter, min(t0a, t1a));
        tExit = min(tExit, max(t0a, t1a));
        stp.y = select(-1, 1, dir.y > 0.0);
        tDelta.y = abs(invD);
    }
    if (abs(dir.z) < 1e-9) {
        if (g0.z < 0.0 || g0.z >= f32(dim.z)) {
            skip = true;
        }
        stp.z = 0;
        tDelta.z = 1e30;
        tMax.z = 1e30;
        cur.z = 0;
    } else {
        let invD = 1.0 / dir.z;
        let t0a = (0.0 - g0.z) * invD;
        let t1a = (f32(dim.z) - g0.z) * invD;
        tEnter = max(tEnter, min(t0a, t1a));
        tExit = min(tExit, max(t0a, t1a));
        stp.z = select(-1, 1, dir.z > 0.0);
        tDelta.z = abs(invD);
    }

    // 没进入本段的网格盒：什么也不写（画布已在绘制开始时清空）。
    if (skip || tEnter > tExit) {
        return;
    }
    let tStart = max(tEnter, 0.0);
    let startG = g0 + tStart * dir;
    cur = clamp(vec3<i32>(floor(startG)), vec3<i32>(0, 0, 0), dim - vec3<i32>(1, 1, 1));
    if (stp.x == 0) {
        tMax.x = 1e30;
    } else {
        let nb = select(floor(startG.x), floor(startG.x) + 1.0, stp.x > 0);
        tMax.x = tStart + (nb - startG.x) * (1.0 / dir.x);
    }
    if (stp.y == 0) {
        tMax.y = 1e30;
    } else {
        let nb = select(floor(startG.y), floor(startG.y) + 1.0, stp.y > 0);
        tMax.y = tStart + (nb - startG.y) * (1.0 / dir.y);
    }
    if (stp.z == 0) {
        tMax.z = 1e30;
    } else {
        let nb = select(floor(startG.z), floor(startG.z) + 1.0, stp.z > 0);
        tMax.z = tStart + (nb - startG.z) * (1.0 / dir.z);
    }

    var hitVox = vec4<u32>(0u, 0u, 0u, 0u);
    var found = false;
    // 网格遍历。射线以网格单位参数化，t 归一化到盒穿越：tEnter/tExit 界定它，
    // tDelta 是每轴跨一格所需的步长。
    for (var it = 0; it < 512; it = it + 1) {
        if (cur.x >= 0 && cur.x < dim.x && cur.y >= 0 && cur.y < dim.y && cur.z >= 0 && cur.z < dim.z) {
            let vox = textureLoad(gSec, cur, 0);
            if (vox.a > 0u && vox.r > 0u) {
                found = true;
                hitVox = vox;
                break;
            }
        }
        if (tMax.x <= tMax.y && tMax.x <= tMax.z) {
            if (tMax.x > tExit) {
                break;
            }
            cur.x = cur.x + stp.x;
            tMax.x = tMax.x + tDelta.x;
        } else if (tMax.y <= tMax.z) {
            if (tMax.y > tExit) {
                break;
            }
            cur.y = cur.y + stp.y;
            tMax.y = tMax.y + tDelta.y;
        } else {
            if (tMax.z > tExit) {
                break;
            }
            cur.z = cur.z + stp.z;
            tMax.z = tMax.z + tDelta.z;
        }
    }

    if (found) {
        let palIdx = u32(f32(hitVox.r) + 0.5);
        let normal = u32(f32(hitVox.g) + 0.5);
        var rgb = vec3<f32>(0.0, 0.0, 0.0);
        // 对段自身的光做 Lambert，与参考实现烘出来的一致。太阳只会拿掉光，从不加光。
        var lit: f32 = 1.0;
        if (cb.uv != 0) {
            // VplFile::paletteIndex 把页钳到最后一个 section 而不是回绕。
            let rawPage = i32(textureLoad(gNormalPage, vec2<i32>(i32(normal), cb.secIndex), 0).r);
            let page = min(rawPage, cb.vplPages - 1);
            let remapped = textureLoad(gVpl, vec2<i32>(i32(palIdx), page), 0).r;
            rgb = textureLoad(gPalette, vec2<i32>(i32(remapped), 0), 0).rgb;
        } else {
            let nrm = textureLoad(gNormals, vec2<i32>(i32(normal), 0), 0).xyz;
            let nl = max(dot(nrm, cb.secLight.xyz), 0.0);
            rgb = textureLoad(gPalette, vec2<i32>(i32(palIdx), 0), 0).rgb * (cb.amb + cb.dif * nl);
        }

        // 自遮挡：命中点是体素中心（射线可能从体素面的任意处进入，用进入点会把采样
        // 放到表面错误的一侧，把表面自己遮住）。
        if (cb.shadowCfg.z != 0) {
            let frame = lightFrameOf();
            let hitLocal = (vec3<f32>(cur) + vec3<f32>(0.5, 0.5, 0.5)) * cb.secScale.xyz;
            let texelWorld = 2.0 * frame.radius / f32(cb.shadowCfg.x);
            let nrm = textureLoad(gNormals, vec2<i32>(i32(normal), 0), 0).xyz;
            let coord = toLightSpace(frame, hitLocal + nrm * (cb.shadowTune.y * texelWorld));
            let shade = sampleShadowPCF(coord, cb.shadowCfg.y);
            lit = 1.0 - shade * cb.shadowTune.z;
        }

        // 调试（cb.dbg == 5）：把阴影因子本身当灰度画出来（白 = 全受光，黑 = 全遮挡）。
        // 用来确认太阳 pass 真的写进了深度图、而本体真的采到了 —— 没有它就只能靠"图看着暗不暗"猜。
        if (cb.dbg == 5) {
            let g = u32(clamp(lit * 255.0, 0.0, 255.0));
            textureStore(
                colorOut,
                vec2<i32>(dtid.xy),
                vec4<u32>(g, g, g, 255u),
            );
            return;
        }

        textureStore(
            colorOut,
            vec2<i32>(dtid.xy),
            vec4<u32>(
                u32(clamp(rgb.r * lit * 255.0, 0.0, 255.0)),
                u32(clamp(rgb.g * lit * 255.0, 0.0, 255.0)),
                u32(clamp(rgb.b * lit * 255.0, 0.0, 255.0)),
                255u,
            ),
        );
        let zb = i32(cb.zb + (f32(cy) * cb.rp) / f32(cb.ch));
        textureStore(zBody, vec2<i32>(dtid.xy), vec4<i32>(zb, 0, 0, 0));
        if (cb.dbg != 0) {
            textureStore(zShadow, vec2<i32>(dtid.xy), vec4<i32>(3, 0, 0, 0));
        } else {
            textureStore(zShadow, vec2<i32>(dtid.xy), vec4<i32>(zb - 64 + (cy >> 1u), 0, 0, 0));
        }
    }
}

// --- 地面足迹（原版影子）-------------------------------------------------------
// vxlrender.cpp drawShadow：每一列只要含体素，就把它在 z=0 面上的基点投到屏幕、盖一个
// 2 px 的点，整个足迹按游戏固定的光向量在屏幕 x 上位移 3 px。
@compute @workgroup_size(8, 8, 1)
fn csShadowBlob(@builtin(global_invocation_id) dtid: vec3<u32>) {
    if (dtid.x >= u32(cb.secGrid.x) || dtid.y >= u32(cb.secGrid.y)) {
        return;
    }
    let colX = i32(dtid.x);
    let colY = i32(dtid.y);
    if (cb.shadowDebug == 4) {
        return; // 诊断：禁用足迹 pass
    }
    var occupied = false;
    for (var z = 0; z < cb.secGrid.z && !occupied; z = z + 1) {
        let v = textureLoad(gSec, vec3<i32>(colX, colY, z), 0);
        if (v.a > 0u && v.r > 0u) {
            occupied = true;
        }
    }
    if (cb.shadowDebug == 2) {
        // 诊断：点亮整列以验证网格读取，再用另一种颜色画投影点，两个阶段可分辨。
        if (occupied) {
            let gp0 = vec3<f32>(f32(colX) * cb.secScale.x, f32(colY) * cb.secScale.y, 0.0);
            let c0 = gp0.x * cb.mvpR0 + gp0.y * cb.mvpR1 + gp0.z * cb.mvpR2 + cb.mvpR3;
            if (abs(c0.w) > 1e-9) {
                let dx = i32((c0.x / c0.w * 0.5 + 0.5) * f32(cb.cw));
                let dy = i32((0.5 - c0.y / c0.w * 0.5) * f32(cb.ch));
                if (dx >= 0 && dy >= 0 && dx < cb.cw && dy < cb.ch) {
                    textureStore(
                        colorOut,
                        vec2<i32>(dx, dy),
                        vec4<u32>(0u, 0u, 255u, 255u),
                    ); // 蓝：列读取正常
                }
            }
        }
        return;
    }
    if (!occupied) {
        return;
    }
    // 列在 z=0 面上的基点，走段的 forward mvp（行向量：p * M）。
    let gp = vec3<f32>(f32(colX) * cb.secScale.x, f32(colY) * cb.secScale.y, 0.0);
    let clip = gp.x * cb.mvpR0 + gp.y * cb.mvpR1 + gp.z * cb.mvpR2 + cb.mvpR3;
    if (abs(clip.w) < 1e-9) {
        return;
    }
    // 画布位置，再加参考实现的 3 px 屏幕 x 光位移。
    let sx = (clip.x / clip.w * 0.5 + 0.5) * f32(cb.cw) + 3.0;
    let sy = (0.5 - clip.y / clip.w * 0.5) * f32(cb.ch);
    // 2 px 宽的点，与 drawShadow 一致。
    for (var s = 0; s < 2; s = s + 1) {
        let px = i32(sx) + s;
        let py = i32(sy);
        if (py < 0 || py >= cb.ch || px < 0 || px >= cb.cw) {
            continue;
        }
        textureStore(shadowOut, vec2<i32>(px, py), vec4<u32>(1u, 0u, 0u, 0u));
        // 影子与本体共用深度层，让合成时的 zS > old 测试把两者排得一致：影子紧贴本体之后，
        // 随画布行倾斜。
        let zv = i32(cb.zb + (f32(py) * cb.rp) / f32(cb.ch)) - 64 + (py >> 1u);
        textureStore(zShadow, vec2<i32>(px, py), vec4<i32>(zv, 0, 0, 0));
    }
}

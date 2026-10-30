// VXL 太阳阴影深度图 —— 对等转译。
//
// 来源：I:\bevy\RA2REASM\src\render\gpurenderer.cpp 的 kVoxelRaycastHLSL 里的
// `csShadowDepth` 入口（D3D11 compute）。逐行搬运。
//
// 这一段是「原版没有」的增强（原版只有地面足迹 blob），单独一个模块是因为
// gLightDepthOut 是 R32_FLOAT 且需要读写（read_write），而本体 pass 只读它。
//
// 与 HLSL 的差异（语言层面）：
//   - RWTexture2D<float> 的读 → textureLoad(t, coord)；写 → textureStore(t, coord, v)。
//   - out 参数 → 返回值。

struct Cb {
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
@group(0) @binding(1) var gLightDepthOut: texture_storage_2d<r32float, read_write>;
@group(0) @binding(2) var<uniform> cb: Cb;

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
    let upRef = select(vec3<f32>(0.0, 1.0, 0.0), vec3<f32>(0.0, 0.0, 1.0), abs(f.dir.y) > 0.99);
    f.right = normalize(cross(upRef, f.dir));
    f.up = cross(f.dir, f.right);
    f.radius = 0.5 * length(extent);
    f.origin = centre - f.dir * f.radius;
    return f;
}

// 每个线程负责光空间贴图的一个 texel，投出它代表的射线、穿过本体 pass 走的那张网格，
// 存下到第一个实体体素的距离。
@compute @workgroup_size(8, 8, 1)
fn csShadowDepth(@builtin(global_invocation_id) dtid: vec3<u32>) {
    let res = cb.shadowCfg.x;
    if (res <= 0 || dtid.x >= u32(res) || dtid.y >= u32(res)) {
        return;
    }

    let frame = lightFrameOf();
    // 这个 texel 的射线起点：过盒心、面向太阳的那个平面。
    let uv = (vec2<f32>(f32(dtid.x), f32(dtid.y)) + vec2<f32>(0.5, 0.5)) / f32(res);
    let origin = frame.origin + frame.right * ((uv.x - 0.5) * 2.0 * frame.radius)
        + frame.up * ((uv.y - 0.5) * 2.0 * frame.radius);
    let dir = frame.dir;

    // 网格单位，与本体 pass 的设置完全一致：体素 (i,j,k) 位于 (i*scale.x, ...)，
    // 没有 minBounds 偏移（它已烘进段的 frame 矩阵）。
    let g0 = origin / cb.secScale.xyz;
    let gDir = dir / cb.secScale.xyz;
    let dim = cb.secGrid.xyz;

    var tEnter: f32 = -1e30;
    var tExit: f32 = 1e30;
    var step = vec3<i32>(0, 0, 0);
    var tMax = vec3<f32>(0.0, 0.0, 0.0);
    var tDelta = vec3<f32>(0.0, 0.0, 0.0);
    var cell = vec3<i32>(0, 0, 0);
    var miss = false;

    for (var axis = 0; axis < 3; axis = axis + 1) {
        let o = select(select(g0.z, g0.y, axis == 1), g0.x, axis == 0);
        let d = select(select(gDir.z, gDir.y, axis == 1), gDir.x, axis == 0);
        let extent = select(
            select(f32(dim.z), f32(dim.y), axis == 1),
            f32(dim.x),
            axis == 0,
        );
        if (abs(d) < 1e-9) {
            if (o < 0.0 || o >= extent) {
                miss = true;
            }
            if (axis == 0) {
                step.x = 0;
                tDelta.x = 1e30;
                tMax.x = 1e30;
            } else if (axis == 1) {
                step.y = 0;
                tDelta.y = 1e30;
                tMax.y = 1e30;
            } else {
                step.z = 0;
                tDelta.z = 1e30;
                tMax.z = 1e30;
            }
        } else {
            let inv = 1.0 / d;
            let near = (0.0 - o) * inv;
            let far = (extent - o) * inv;
            tEnter = max(tEnter, min(near, far));
            tExit = min(tExit, max(near, far));
            let s = select(-1, 1, d > 0.0);
            let t = select(floor(o), floor(o) + 1.0, s > 0);
            if (axis == 0) {
                step.x = s;
                tDelta.x = abs(inv);
                tMax.x = tEnter + (t - o) * inv;
            } else if (axis == 1) {
                step.y = s;
                tDelta.y = abs(inv);
                tMax.y = tEnter + (t - o) * inv;
            } else {
                step.z = s;
                tDelta.z = abs(inv);
                tMax.z = tEnter + (t - o) * inv;
            }
        }
    }

    // 这条射线什么也没有：记 1.0（远平面）。texel 已被 host 清成「无遮挡」。
    if (miss || tEnter > tExit) {
        let texel = vec2<i32>(i32(dtid.x), i32(dtid.y));
        if (textureLoad(gLightDepthOut, texel).r > 1.0) {
            textureStore(gLightDepthOut, texel, vec4<f32>(1.0, 0.0, 0.0, 0.0));
        }
        return;
    }

    let startG = g0 + max(tEnter, 0.0) * gDir;
    cell = clamp(vec3<i32>(floor(startG)), vec3<i32>(0, 0, 0), dim - vec3<i32>(1, 1, 1));
    for (var it = 0; it < 512; it = it + 1) {
        if (cell.x >= 0 && cell.x < dim.x && cell.y >= 0 && cell.y < dim.y && cell.z >= 0
            && cell.z < dim.z)
        {
            let vox = textureLoad(gSec, cell, 0);
            if (vox.a > 0u && vox.r > 0u) {
                // 沿光方向的距离，归一化到贴图范围。这里不用 toLightSpace：这个 texel 本身
                // 就是光空间位置，只有深度在变，而且是沿射线量出来的，不是从命中点重建的。
                let rel = startG * cb.secScale.xyz - frame.origin;
                let depth = dot(rel, frame.dir) / (2.0 * frame.radius);
                let texel = vec2<i32>(i32(dtid.x), i32(dtid.y));
                if (depth < textureLoad(gLightDepthOut, texel).r) {
                    textureStore(gLightDepthOut, texel, vec4<f32>(depth, 0.0, 0.0, 0.0));
                }
                return;
            }
        }
        if (tMax.x <= tMax.y && tMax.x <= tMax.z) {
            if (tMax.x > tExit) {
                break;
            }
            cell.x = cell.x + step.x;
            tMax.x = tMax.x + tDelta.x;
        } else if (tMax.y <= tMax.z) {
            if (tMax.y > tExit) {
                break;
            }
            cell.y = cell.y + step.y;
            tMax.y = tMax.y + tDelta.y;
        } else {
            if (tMax.z > tExit) {
                break;
            }
            cell.z = cell.z + step.z;
            tMax.z = tMax.z + tDelta.z;
        }
    }
    // 跑到头也没命中：清成 1.0 的初值保持不变。
}

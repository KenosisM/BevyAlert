//! 4x4 矩阵与向量 —— 对等移植。
//!
//! 来源：`I:\bevy\RA2REASM\src\compat\mat4.h`。
//!
//! 布局与约定必须逐位一致：行主序 `m[row][col]`，行向量左乘 `v * M`；投影用 **GL 约定**
//! （z 映射到 `[-1, 1]`），**不是** DirectX 的 `[0, 1]`。原注释说得很清楚——体素栅格化
//! 量化到整像素，差一个 ulp 真的会翻一个像素，所以这里是「必须一致」，不是「等价即可」。
//!
//! `mul` 的词序也照抄：`((a0*b0 + a1*b1) + (a2*b2 + a3*b3))`，左结合、两两分组，
//! 让编译器没有重排加法顺序的自由。

pub const DEG2RAD: f32 = std::f32::consts::PI / 180.0;

pub type Mat4 = [[f32; 4]; 4];
pub type Vec3 = [f32; 3];
pub type Vec4 = [f32; 4];

pub fn identity() -> Mat4 {
    let mut r = [[0.0f32; 4]; 4];
    r[0][0] = 1.0;
    r[1][1] = 1.0;
    r[2][2] = 1.0;
    r[3][3] = 1.0;
    r
}

pub fn mul(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut r = [[0.0f32; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            r[i][j] = (a[i][0] * b[0][j] + a[i][1] * b[1][j]) + (a[i][2] * b[2][j] + a[i][3] * b[3][j]);
        }
    }
    r
}

/// 行向量变换 `v * M`。
pub fn transform_row(v: &Vec4, m: &Mat4) -> Vec4 {
    [
        v[0] * m[0][0] + v[1] * m[1][0] + v[2] * m[2][0] + v[3] * m[3][0],
        v[0] * m[0][1] + v[1] * m[1][1] + v[2] * m[2][1] + v[3] * m[3][1],
        v[0] * m[0][2] + v[1] * m[1][2] + v[2] * m[2][2] + v[3] * m[3][2],
        v[0] * m[0][3] + v[1] * m[1][3] + v[2] * m[2][3] + v[3] * m[3][3],
    ]
}

/// 行向量变换，忽略平移。
pub fn transform_normal(v: &Vec3, m: &Mat4) -> Vec3 {
    [
        v[0] * m[0][0] + v[1] * m[1][0] + v[2] * m[2][0],
        v[0] * m[0][1] + v[1] * m[1][1] + v[2] * m[2][1],
        v[0] * m[0][2] + v[1] * m[1][2] + v[2] * m[2][2],
    ]
}

pub fn rot_x(a: f32) -> Mat4 {
    let (c, s) = (a.cos(), a.sin());
    let mut r = identity();
    r[1][1] = c;
    r[1][2] = s;
    r[2][1] = -s;
    r[2][2] = c;
    r
}

pub fn rot_y(a: f32) -> Mat4 {
    let (c, s) = (a.cos(), a.sin());
    let mut r = identity();
    r[0][0] = c;
    r[0][2] = -s;
    r[2][0] = s;
    r[2][2] = c;
    r
}

pub fn rot_z(a: f32) -> Mat4 {
    let (c, s) = (a.cos(), a.sin());
    let mut r = identity();
    r[0][0] = c;
    r[0][1] = s;
    r[1][0] = -s;
    r[1][1] = c;
    r
}

pub fn translate(x: f32, y: f32, z: f32) -> Mat4 {
    let mut r = identity();
    r[3][0] = x;
    r[3][1] = y;
    r[3][2] = z;
    r
}

/// GL 约定：z 映射到 `[-1, 1]`。保留这个，不要换 DirectX 的。
pub fn ortho_gl(width: f32, height: f32, z_near: f32, z_far: f32) -> Mat4 {
    let left = -width / 2.0;
    let right = width / 2.0;
    let bottom = -height / 2.0;
    let top = height / 2.0;
    let mut r = identity();
    r[0][0] = 2.0 / (right - left);
    r[1][1] = 2.0 / (top - bottom);
    r[2][2] = -2.0 / (z_far - z_near);
    r[3][0] = -(right + left) / (right - left);
    r[3][1] = -(top + bottom) / (top - bottom);
    r[3][2] = -(z_far + z_near) / (z_far - z_near);
    r
}

/// System.Numerics 的 CreateLookAt：右手系，行主序存储。
pub fn look_at(eye: &Vec3, target: &Vec3, up: &Vec3) -> Mat4 {
    let zaxis = normalize3(&sub3(eye, target));
    let xaxis = normalize3(&cross3(up, &zaxis));
    let yaxis = cross3(&zaxis, &xaxis);
    let mut r = identity();
    r[0][0] = xaxis[0];
    r[0][1] = yaxis[0];
    r[0][2] = zaxis[0];
    r[1][0] = xaxis[1];
    r[1][1] = yaxis[1];
    r[1][2] = zaxis[1];
    r[2][0] = xaxis[2];
    r[2][1] = yaxis[2];
    r[2][2] = zaxis[2];
    r[3][0] = -dot3(&xaxis, eye);
    r[3][1] = -dot3(&yaxis, eye);
    r[3][2] = -dot3(&zaxis, eye);
    r
}

/// 通用 4x4 求逆。用 double 做高斯消元，和 `Matrix4x4.Invert` 对这些仿射矩阵的结果一致。
pub fn invert(src: &Mat4) -> Option<Mat4> {
    let mut a = [[0.0f64; 8]; 4];
    for i in 0..4 {
        for j in 0..4 {
            a[i][j] = src[i][j] as f64;
        }
        for j in 0..4 {
            a[i][4 + j] = if i == j { 1.0 } else { 0.0 };
        }
    }
    for col in 0..4 {
        let mut piv = col;
        for r in (col + 1)..4 {
            if a[r][col].abs() > a[piv][col].abs() {
                piv = r;
            }
        }
        if a[piv][col].abs() < 1e-12 {
            return None;
        }
        if piv != col {
            for j in 0..8 {
                let t = a[col][j];
                a[col][j] = a[piv][j];
                a[piv][j] = t;
            }
        }
        let d = a[col][col];
        for j in 0..8 {
            a[col][j] /= d;
        }
        for r in 0..4 {
            if r == col {
                continue;
            }
            let f = a[r][col];
            if f == 0.0 {
                continue;
            }
            for j in 0..8 {
                a[r][j] -= f * a[col][j];
            }
        }
    }
    let mut out = [[0.0f32; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            out[i][j] = a[i][4 + j] as f32;
        }
    }
    Some(out)
}

pub fn dot3(a: &Vec3, b: &Vec3) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn add3(a: &Vec3, b: &Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub fn sub3(a: &Vec3, b: &Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn scale3(a: &Vec3, s: f32) -> Vec3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

pub fn cross3(a: &Vec3, b: &Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub fn normalize3(v: &Vec3) -> Vec3 {
    let len = dot3(v, v).sqrt();
    if len < 1e-20 {
        return [0.0, 0.0, 0.0];
    }
    [v[0] / len, v[1] / len, v[2] / len]
}

/// `vxlrender.cpp extractRotationVector`：从一个 16 元素行主序矩阵里取出旋转后的 z 轴方向，
/// 并把平移分量（原点投影）减掉。`m16[4*k + j]` 中 k 是行号。
pub fn extract_rotation_vector(m16: &[f32; 16]) -> Vec3 {
    let col_mul = |v: [f32; 4]| -> [f32; 4] {
        let mut out = [0.0f32; 4];
        for j in 0..4 {
            for k in 0..4 {
                out[j] += m16[4 * k + j] * v[k];
            }
        }
        out
    };
    let t = col_mul([0.0, 0.0, 1.0, 1.0]);
    let o = col_mul([0.0, 0.0, 0.0, 1.0]);
    let mut tv = t;
    tv[0] -= o[0] * t[3] / o[3];
    tv[1] -= o[1] * t[3] / o[3];
    tv[2] -= o[2] * t[3] / o[3];
    let len = (tv[0] * tv[0] + tv[1] * tv[1] + tv[2] * tv[2]).sqrt();
    if len < 1e-20 {
        return [0.0, 0.0, 0.0];
    }
    [tv[0] / len, tv[1] / len, tv[2] / len]
}

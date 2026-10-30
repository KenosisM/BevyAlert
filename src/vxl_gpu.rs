//! VXL GPU 光线投射 —— host 侧，对等移植。
//!
//! 来源：`I:\bevy\RA2REASM\src\render\gpurenderer.cpp` 的 `registerVoxelModel`
//! 与 `drawVoxelModel`（D3D11 compute），配 `shaders/vxl_raycast.wgsl`。
//!
//! 做法与那份实现逐条对应：
//!   - 每段一张 3D 纹理（r=调色板索引, g=法线索引, b=0, a=solid）。
//!   - 每段一次 dispatch，段按顺序画（画家算法，后面的盖前面的）。
//!   - 每段的 mvp 由 sectionFrame(T(minBounds)*frameRot) * object * world * trans * lookat * persp
//!     拼出，再把它的逆的**四行**传给 shader。
//!   - 光照：法线→VPL 页表（每段一行 256 项）+ 段空间光方向。

use std::borrow::Cow;

use bytemuck::{Pod, Zeroable};

use crate::hva::Hva;
use crate::mat4;
use crate::vpl::{self, Vpl};
use crate::vxl::Vxl;

pub const CANVAS: u32 = 400;
/// buffer↔texture 拷贝的每行字节数必须是它的倍数。
const ROW_ALIGN: u32 = 256;

/// 与 `shaders/vxl_raycast.wgsl` 的 `struct Cb` 逐字段一致，共 256 字节。
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable, Default)]
struct RayCb {
    cw: i32,
    ch: i32,
    vpl_pages: i32,
    uv: i32,
    dbg: i32,
    sec_index: i32,
    shadow_debug: i32,
    pad3: i32,
    amb: f32,
    dif: f32,
    zb: f32,
    rp: f32,
    inv_r0: [f32; 4],
    inv_r1: [f32; 4],
    inv_r2: [f32; 4],
    inv_r3: [f32; 4],
    mvp_r0: [f32; 4],
    mvp_r1: [f32; 4],
    mvp_r2: [f32; 4],
    mvp_r3: [f32; 4],
    sec_scale: [f32; 4],
    sec_grid: [i32; 4],
    sec_light: [f32; 4],
    shadow_cfg: [i32; 4],
    shadow_tune: [f32; 4],
}

/// `render::ShadowConfig::Mode` 的对应物。
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ShadowMode {
    /// 不画影子。
    Off,
    /// 原版：每根有体素的柱子戳一个 2 px 的点，按固定的 3 px 屏幕 x 偏移。参考观感。
    Blob,
    /// 真的从太阳打一盏光，用光空间深度图给自己遮挡。
    Sun,
}

/// `gpurenderer.cpp ShadowSettings` 的对应物（宿主侧的一处真相）。
#[derive(Clone, Copy)]
pub struct ShadowSettings {
    pub mode: ShadowMode,
    pub resolution: i32,
    pub tent: i32,
    pub strength: f32,
    pub depth_bias: f32,
    pub normal_bias: f32,
}

impl Default for ShadowSettings {
    fn default() -> Self {
        // 与 C++ 一致：默认 blob（原版画面）；分辨率是 2 的幂，从 64 起按
        // resolutionScale*192 往上取，上限 512 —— 默认 scale=1 得 256。
        let mut res = 64;
        while res < 192 && res < 512 {
            res *= 2;
        }
        Self {
            mode: ShadowMode::Blob,
            resolution: res,
            tent: 1,
            strength: 0.5,
            depth_bias: 0.004,
            normal_bias: 1.0,
        }
    }
}

impl ShadowSettings {
    pub fn sun(&self) -> bool {
        self.mode == ShadowMode::Sun
    }
    pub fn blob(&self) -> bool {
        self.mode == ShadowMode::Blob
    }
}

#[derive(Clone, Copy)]
pub struct RenderOptions {
    pub facing: f32,
    pub tilt_pitch: f32,
    pub tilt_yaw: f32,
    pub turret_offset: f32,
    pub use_vpl: bool,
    pub shadow: ShadowSettings,
    pub z_base: i32,
    pub row_pitch: i32,
}

impl Default for RenderOptions {
    fn default() -> Self {
        Self {
            facing: 0.0,
            tilt_pitch: 0.0,
            tilt_yaw: 0.0,
            turret_offset: 0.0,
            use_vpl: true,
            shadow: ShadowSettings::default(),
            z_base: 0,
            row_pitch: 15,
        }
    }
}

pub struct Canvas {
    pub rgba: Vec<u8>,
    /// 每像素的本体深度（HLSL 里是 R16_SINT，这里按 r32sint 读回）。
    pub z_body: Vec<i32>,
    pub z_shadow: Vec<i32>,
    /// 每像素 0/1：该处是地面足迹影子（HLSL 里是 R8_UINT，这里取 rgba8uint 的首字节）。
    pub shadow_mask: Vec<u8>,
    /// 非透明像素数（alpha != 0）。
    pub opaque: usize,
    /// 非透明像素的包围盒 (min_x, min_y, max_x, max_y)。
    pub bbox: Option<(u32, u32, u32, u32)>,
}

pub struct VxlGpu {
    device: wgpu::Device,
    queue: wgpu::Queue,
    pipeline_ray: wgpu::ComputePipeline,
    pipeline_blob: wgpu::ComputePipeline,
    /// 太阳阴影 pass。设备不支持 R32Float 的 storage 读写时为 None（太阳模式会退回 blob）。
    pipeline_shadow: Option<wgpu::ComputePipeline>,
    layout: wgpu::BindGroupLayout,
    shadow_layout: wgpu::BindGroupLayout,
    sun_supported: bool,
}

impl VxlGpu {
    pub fn new() -> Result<Self, String> {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let adapter = bevy::tasks::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            force_fallback_adapter: false,
            compatible_surface: None,
            ..Default::default()
        }))
        .map_err(|e| format!("request_adapter: {e}"))?;
        let info = adapter.get_info();
        println!(
            "gpu: {} ({:?}, {:?})",
            info.name, info.backend, info.device_type
        );

        // 太阳阴影的深度图是一张 R32Float storage 纹理，而且 shader 要**读回**它做 min 合并。
        // WebGPU 默认只给 R32Float 写权限，读写要 adapter-specific 的格式特性 —— 所以先在
        // adapter 上问，再决定要不要 device 上开这个 feature。
        let sun_supported = adapter
            .features()
            .contains(wgpu::Features::TEXTURE_ADAPTER_SPECIFIC_FORMAT_FEATURES)
            && adapter
                .get_texture_format_features(wgpu::TextureFormat::R32Float)
                .flags
                .contains(wgpu::TextureFormatFeatureFlags::STORAGE_READ_WRITE);
        let mut required = wgpu::Features::empty();
        if sun_supported {
            required |= wgpu::Features::TEXTURE_ADAPTER_SPECIFIC_FORMAT_FEATURES;
        }

        let (device, queue) = bevy::tasks::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("ra2engine-vxl"),
            required_features: required,
            required_limits: wgpu::Limits::downlevel_defaults().using_resolution(adapter.limits()),
            ..Default::default()
        }))
        .map_err(|e| format!("request_device: {e}"))?;
        println!("gpu: sun shadow {}", if sun_supported { "supported" } else { "unavailable" });

        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("vxl_raycast"),
            source: wgpu::ShaderSource::Wgsl(Cow::Borrowed(include_str!("../shaders/vxl_raycast.wgsl"))),
        });

        let compute = wgpu::ShaderStages::COMPUTE;
        let entries = [
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: compute,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Uint,
                    view_dimension: wgpu::TextureViewDimension::D3,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: compute,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: false },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 2,
                visibility: compute,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Uint,
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 3,
                visibility: compute,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Uint,
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 4,
                visibility: compute,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: false },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 5,
                visibility: compute,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: false },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            storage(6, wgpu::TextureFormat::Rgba8Uint),
            storage(7, wgpu::TextureFormat::R32Sint),
            storage(8, wgpu::TextureFormat::R32Sint),
            storage(9, wgpu::TextureFormat::Rgba8Uint),
            wgpu::BindGroupLayoutEntry {
                binding: 10,
                visibility: compute,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            },
        ];
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("vxl_raycast_bgl"),
            entries: &entries,
        });
        let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("vxl_raycast_pl"),
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let pipeline_ray = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("vxl_raycast"),
            layout: Some(&pl),
            module: &module,
            entry_point: Some("cs"),
            compilation_options: Default::default(),
            cache: None,
        });
        let pipeline_blob = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("vxl_shadow_blob"),
            layout: Some(&pl),
            module: &module,
            entry_point: Some("csShadowBlob"),
            compilation_options: Default::default(),
            cache: None,
        });

        // --- 太阳阴影 pass（单独模块 + 单独 bind group layout）-------------------
        // 它的深度图是 read_write storage，而本体 pass 只把它当 sampled texture 读，
        // 所以两者必须用不同的 bind group layout / pipeline。
        let shadow_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("vxl_shadow_bgl"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Uint,
                        view_dimension: wgpu::TextureViewDimension::D3,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::StorageTexture {
                        access: wgpu::StorageTextureAccess::ReadWrite,
                        format: wgpu::TextureFormat::R32Float,
                        view_dimension: wgpu::TextureViewDimension::D2,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });
        let pipeline_shadow = if sun_supported {
            let shadow_module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("vxl_shadow"),
                source: wgpu::ShaderSource::Wgsl(Cow::Borrowed(include_str!(
                    "../shaders/vxl_shadow.wgsl"
                ))),
            });
            let shadow_pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("vxl_shadow_pl"),
                bind_group_layouts: &[Some(&shadow_layout)],
                immediate_size: 0,
            });
            Some(
                device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                    label: Some("vxl_shadow_depth"),
                    layout: Some(&shadow_pl),
                    module: &shadow_module,
                    entry_point: Some("csShadowDepth"),
                    compilation_options: Default::default(),
                    cache: None,
                }),
            )
        } else {
            None
        };

        Ok(Self {
            device,
            queue,
            pipeline_ray,
            pipeline_blob,
            pipeline_shadow,
            layout,
            shadow_layout,
            sun_supported,
        })
    }

    /// 设备是否支持太阳阴影（R32Float 的存储读写）。
    pub fn sun_supported(&self) -> bool {
        self.sun_supported
    }

    /// 一次完整的绘制：清画布 →（太阳）逐段光空间深度 → 逐段 raycast → 逐段足迹影子 → 读回。
    pub fn render(
        &self,
        vxl: &Vxl,
        hva: Option<&Hva>,
        vpl: Option<&Vpl>,
        palette768: &[u8],
        opt: &RenderOptions,
    ) -> Result<Canvas, String> {
        if vxl.sections.is_empty() {
            return Err("no sections".into());
        }

        // --- 每段的 3D 体素纹理 ---------------------------------------------------
        let mut sec_tex = Vec::new();
        // 纹理句柄保活：view 是否独自保住底层纹理，不该靠猜。
        let mut sec_tex_keep = Vec::new();
        for (si, sec) in vxl.sections.iter().enumerate() {
            let (gw, gh, gd) = (sec.x_size as u32, sec.y_size as u32, sec.z_size as u32);
            if gw == 0 || gh == 0 || gd == 0 {
                return Err("section with a zero dimension".into());
            }
            let mut buf = vec![0u8; (gw * gh * gd * 4) as usize];
            let mut solid = 0usize;
            let (mut xmin, mut xmax, mut ymin, mut ymax, mut zmin, mut zmax) =
                (u32::MAX, 0u32, u32::MAX, 0u32, u32::MAX, 0u32);
            for v in &sec.voxels {
                if v.color == 0 {
                    continue;
                }
                if v.z as u32 >= gd {
                    continue;
                }
                // 索引是 (z*gh + y)*gw + x —— 与 C++ 的 core 侧填充同序。
                let o = (((v.z as u32 * gh + v.y as u32) * gw + v.x as u32) * 4) as usize;
                buf[o] = v.color;
                buf[o + 1] = v.normal;
                buf[o + 3] = 255; // solid
                solid += 1;
                xmin = xmin.min(v.x as u32);
                xmax = xmax.max(v.x as u32);
                ymin = ymin.min(v.y as u32);
                ymax = ymax.max(v.y as u32);
                zmin = zmin.min(v.z as u32);
                zmax = zmax.max(v.z as u32);
            }
            println!(
                "  sec{si}: grid={gw}x{gh}x{gd} solid={solid} extent x[{xmin}..{xmax}] y[{ymin}..{ymax}] z[{zmin}..{zmax}]"
            );
            let tex = self.create_tex_3d(gw, gh, gd);
            self.queue.write_texture(
                wgpu::TexelCopyTextureInfo {
                    texture: &tex,
                    mip_level: 0,
                    origin: wgpu::Origin3d::ZERO,
                    aspect: wgpu::TextureAspect::All,
                },
                &buf,
                wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(gw * 4),
                    rows_per_image: Some(gh),
                },
                wgpu::Extent3d {
                    width: gw,
                    height: gh,
                    depth_or_array_layers: gd,
                },
            );
            sec_tex.push(tex.create_view(&Default::default()));
            sec_tex_keep.push(tex);
        }

        // --- 调色板 256x1 --------------------------------------------------------
        let pal_tex = self.create_tex_2d(256, 1, wgpu::TextureFormat::Rgba8Unorm);
        let mut pal = vec![0u8; 256 * 4];
        for i in 0..256 {
            pal[i * 4] = palette768.get(i * 3).copied().unwrap_or(0);
            pal[i * 4 + 1] = palette768.get(i * 3 + 1).copied().unwrap_or(0);
            pal[i * 4 + 2] = palette768.get(i * 3 + 2).copied().unwrap_or(0);
            pal[i * 4 + 3] = 255;
        }
        self.upload_2d(&pal_tex, &pal, 256, 1, 256 * 4);

        // --- VPL 256 x 页数（或 1x1 dummy）---------------------------------------
        let (vpl_pages, vpl_tex) = match vpl {
            Some(v) if opt.use_vpl && v.valid && !v.lookup_sections.is_empty() => {
                let n = v.lookup_sections.len() as u32;
                let tex = self.create_tex_2d(256, n, wgpu::TextureFormat::R8Uint);
                let mut flat = Vec::with_capacity((n * 256) as usize);
                for page in &v.lookup_sections {
                    flat.extend_from_slice(page);
                }
                self.upload_2d(&tex, &flat, 256, n, 256);
                (v.num_sections as i32, tex)
            }
            _ => {
                let tex = self.create_tex_2d(1, 1, wgpu::TextureFormat::R8Uint);
                self.upload_2d(&tex, &[0u8], 1, 1, 1);
                (0, tex)
            }
        };
        let use_vpl = if vpl_pages > 0 && opt.use_vpl { 1 } else { 0 };

        // --- 法线表 256x1 float4 -------------------------------------------------
        let nm_mode = vxl.sections.first().map(|s| s.normals_type).unwrap_or(0);
        let table = crate::vxl_normals::vxl_normals(nm_mode);
        let norm_tex = self.create_tex_2d(256, 1, wgpu::TextureFormat::Rgba32Float);
        let mut nbuf = vec![0u8; 256 * 16];
        for i in 0..256usize {
            let n = table[i.min(table.len() - 1)];
            for k in 0..3 {
                nbuf[i * 16 + k * 4..i * 16 + k * 4 + 4].copy_from_slice(&n[k].to_le_bytes());
            }
        }
        self.upload_2d(&norm_tex, &nbuf, 256, 1, 256 * 16);

        // --- 法线→页表 256 x 段数 -------------------------------------------------
        let sec_count = vxl.sections.len() as u32;
        let np_tex = self.create_tex_2d(256, sec_count, wgpu::TextureFormat::R8Uint);

        // --- 光空间深度图 ----------------------------------------------------------
        // 开太阳时它是一张 resolution² 的 R32Float（先清成 1.0 = 远平面 = "无遮挡"）；
        // 关太阳时给个 1x1 的合法绑定，本体 pass 不会去读它。
        // 设备不支持 R32Float 的存储读写时，太阳开不了 —— 退回原版足迹，而不是干脆没影子。
        let sun = opt.shadow.sun() && self.pipeline_shadow.is_some();
        let blob = opt.shadow.blob() || (opt.shadow.sun() && !sun);
        let shadow_res = if sun { opt.shadow.resolution.max(8) as u32 } else { 1 };
        let ld_tex = if sun {
            self.create_shadow_2d(shadow_res)
        } else {
            self.create_tex_2d(1, 1, wgpu::TextureFormat::R32Float)
        };
        if sun {
            self.clear_f32_2d(&ld_tex, shadow_res, 1.0);
        } else {
            self.upload_2d(&ld_tex, &1.0f32.to_le_bytes(), 1, 1, 4);
        }

        // --- 画布 -----------------------------------------------------------------
        let rgba_tex = self.create_storage_2d(CANVAS, CANVAS, wgpu::TextureFormat::Rgba8Uint);
        let zb_tex = self.create_storage_2d(CANVAS, CANVAS, wgpu::TextureFormat::R32Sint);
        let zs_tex = self.create_storage_2d(CANVAS, CANVAS, wgpu::TextureFormat::R32Sint);
        let sh_tex = self.create_storage_2d(CANVAS, CANVAS, wgpu::TextureFormat::Rgba8Uint);
        self.clear_2d(&rgba_tex, 4);
        self.clear_2d(&zb_tex, 4);
        self.clear_2d(&zs_tex, 4);
        self.clear_2d(&sh_tex, 4);

        let rgba_v = rgba_tex.create_view(&Default::default());
        let zb_v = zb_tex.create_view(&Default::default());
        let zs_v = zs_tex.create_view(&Default::default());
        let sh_v = sh_tex.create_view(&Default::default());
        let pal_v = pal_tex.create_view(&Default::default());
        let vpl_v = vpl_tex.create_view(&Default::default());
        let norm_v = norm_tex.create_view(&Default::default());
        let np_v = np_tex.create_view(&Default::default());
        let ld_v = ld_tex.create_view(&Default::default());

        // 每段一个常量缓冲。queue.write_buffer 是**队列**操作，会在 submit 之前统一落到显存，
        // 而 dispatch 是命令缓冲里的指令 —— 共用一个 buffer 的话，三段 dispatch 都会读到
        // 最后一次写入的值（实测：后两个 section 完全被最后一个覆盖，车前两个部件凭空消失）。
        let cb_bufs: Vec<wgpu::Buffer> = (0..vxl.sections.len())
            .map(|i| {
                self.device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some(&format!("rayCb{i}")),
                    size: std::mem::size_of::<RayCb>() as u64,
                    usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                })
            })
            .collect();

        let world = vpl::voxel_world_matrix();
        let (persp, lookat, trans) = vpl::voxel_base_matrices();
        let object = vpl::voxel_object_matrix(opt.facing, opt.tilt_pitch, opt.tilt_yaw, opt.turret_offset);

        let mut cb = RayCb {
            cw: CANVAS as i32,
            ch: CANVAS as i32,
            vpl_pages,
            uv: use_vpl,
            dbg: std::env::var("RA2_VXL_DBG").ok().and_then(|s| s.parse().ok()).unwrap_or(0),
            amb: 1.3,
            dif: 1.3,
            zb: opt.z_base as f32,
            rp: opt.row_pitch as f32,
            shadow_cfg: [
                opt.shadow.resolution,
                opt.shadow.tent,
                if sun { 1 } else { 0 },
                if blob { 1 } else { 0 },
            ],
            shadow_tune: [
                opt.shadow.depth_bias,
                opt.shadow.normal_bias,
                opt.shadow.strength,
                0.0,
            ],
            ..Default::default()
        };

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("vxl") });
        let groups = CANVAS.div_ceil(8);

        // 第一遍只做"每段要什么"：算矩阵、写页表、填常量。阴影 pass 与本体 pass 共用这些结果。
        let mut plans: Vec<Option<RayCb>> = Vec::with_capacity(vxl.sections.len());
        for (s, sec) in vxl.sections.iter().enumerate() {
            // sectionFrame = T(minBounds) * frameRot。frameRot 取 HVA 的 frame 0；该部件没有 HVA
            // （或该行越界）时是单位阵 —— 与 appendVxlPart 的兜底一致。它的平移按
            // hvaMultiplier * 格边长缩放。
            let mut frame_rot = match hva {
                Some(h) => mat_from_flat(h.matrix_for(s as i32, 0)),
                None => mat4::identity(),
            };
            let scale = [
                sec.scale_of(0),
                sec.scale_of(1),
                sec.scale_of(2),
            ];
            frame_rot[3][0] *= sec.hva_multiplier * scale[0];
            frame_rot[3][1] *= sec.hva_multiplier * scale[1];
            frame_rot[3][2] *= sec.hva_multiplier * scale[2];
            let section_frame = mat4::mul(
                &mat4::translate(
                    sec.bounds[0],
                    sec.bounds[1],
                    sec.bounds[2],
                ),
                &frame_rot,
            );
            let mvp = mat4::mul(
                &mat4::mul(
                    &mat4::mul(&mat4::mul(&section_frame, &object), &world),
                    &trans,
                ),
                &lookat,
            );
            let mvp = mat4::mul(&mvp, &persp);

            let Some(inv) = mat4::invert(&mvp) else {
                println!("  section {s}: mvp not invertible, skipped");
                plans.push(None);
                continue;
            };

            let (light_dir, ntp) =
                vpl::voxel_lighting_for_section(sec.normals_type, opt.facing, &object, &world, &section_frame);
            // 每段一行 256 项，行 r 对应段 r；只更新本段那一行。
            self.queue.write_texture(
                wgpu::TexelCopyTextureInfo {
                    texture: &np_tex,
                    mip_level: 0,
                    origin: wgpu::Origin3d { x: 0, y: s as u32, z: 0 },
                    aspect: wgpu::TextureAspect::All,
                },
                &ntp,
                wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(256),
                    rows_per_image: Some(1),
                },
                wgpu::Extent3d {
                    width: 256,
                    height: 1,
                    depth_or_array_layers: 1,
                },
            );

            let mut c = cb;
            c.sec_index = s as i32;
            c.sec_scale = [scale[0], scale[1], scale[2], 0.0];
            c.sec_grid = [sec.x_size as i32, sec.y_size as i32, sec.z_size as i32, 0];
            c.sec_light = [light_dir[0], light_dir[1], light_dir[2], 0.0];
            for i in 0..4 {
                c.inv_r0[i] = inv[0][i];
                c.inv_r1[i] = inv[1][i];
                c.inv_r2[i] = inv[2][i];
                c.inv_r3[i] = inv[3][i];
                c.mvp_r0[i] = mvp[0][i];
                c.mvp_r1[i] = mvp[1][i];
                c.mvp_r2[i] = mvp[2][i];
                c.mvp_r3[i] = mvp[3][i];
            }
            self.queue.write_buffer(&cb_bufs[s], 0, bytemuck::bytes_of(&c));
            plans.push(Some(c));
        }

        // 太阳阴影：先把**所有**段的光空间深度写进同一张图（逐段按 min 合并），最后才让任何一段的
        // 本体 pass 去采样它。顺序就是在这里定的 —— 所以后面段会被前面段遮挡（炮管反过来遮车体
        // 也一样）。C++ 那段注释把这条写得很明确：先跑完所有段的深度，再跑本体。
        if sun {
            let shadow_groups = shadow_res.div_ceil(8);
            let shadow_binds: Vec<wgpu::BindGroup> = (0..vxl.sections.len())
                .map(|s| {
                    self.device.create_bind_group(&wgpu::BindGroupDescriptor {
                        label: Some("vxl_shadow_bg"),
                        layout: &self.shadow_layout,
                        entries: &[
                            wgpu::BindGroupEntry {
                                binding: 0,
                                resource: wgpu::BindingResource::TextureView(&sec_tex[s]),
                            },
                            wgpu::BindGroupEntry {
                                binding: 1,
                                resource: wgpu::BindingResource::TextureView(&ld_v),
                            },
                            wgpu::BindGroupEntry {
                                binding: 2,
                                resource: cb_bufs[s].as_entire_binding(),
                            },
                        ],
                    })
                })
                .collect();
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("vxl_shadow_depth"),
                timestamp_writes: None,
            });
            pass.set_pipeline(self.pipeline_shadow.as_ref().unwrap());
            for s in 0..vxl.sections.len() {
                if plans[s].is_none() {
                    continue;
                }
                pass.set_bind_group(0, &shadow_binds[s], &[]);
                pass.dispatch_workgroups(shadow_groups, shadow_groups, 1);
            }
        }

        // 本体：逐段 raycast（后画的盖先画的 = 画家算法），再逐段戳地面足迹。
        for s in 0..vxl.sections.len() {
            if plans[s].is_none() {
                continue;
            }
            let sec = &vxl.sections[s];
            let bind = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("vxl_raycast_bg"),
                layout: &self.layout,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: wgpu::BindingResource::TextureView(&sec_tex[s]),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: wgpu::BindingResource::TextureView(&pal_v),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: wgpu::BindingResource::TextureView(&vpl_v),
                    },
                    wgpu::BindGroupEntry {
                        binding: 3,
                        resource: wgpu::BindingResource::TextureView(&np_v),
                    },
                    wgpu::BindGroupEntry {
                        binding: 4,
                        resource: wgpu::BindingResource::TextureView(&norm_v),
                    },
                    wgpu::BindGroupEntry {
                        binding: 5,
                        resource: wgpu::BindingResource::TextureView(&ld_v),
                    },
                    wgpu::BindGroupEntry {
                        binding: 6,
                        resource: wgpu::BindingResource::TextureView(&rgba_v),
                    },
                    wgpu::BindGroupEntry {
                        binding: 7,
                        resource: wgpu::BindingResource::TextureView(&zb_v),
                    },
                    wgpu::BindGroupEntry {
                        binding: 8,
                        resource: wgpu::BindingResource::TextureView(&zs_v),
                    },
                    wgpu::BindGroupEntry {
                        binding: 9,
                        resource: wgpu::BindingResource::TextureView(&sh_v),
                    },
                    wgpu::BindGroupEntry {
                        binding: 10,
                        resource: cb_bufs[s].as_entire_binding(),
                    },
                ],
            });

            {
                let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                    label: Some("vxl_raycast"),
                    timestamp_writes: None,
                });
                pass.set_pipeline(&self.pipeline_ray);
                pass.set_bind_group(0, &bind, &[]);
                pass.dispatch_workgroups(groups, groups, 1);
            }

            // 地面足迹（原版影子）：一列一个线程。
            if blob {
                let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                    label: Some("vxl_shadow_blob"),
                    timestamp_writes: None,
                });
                pass.set_pipeline(&self.pipeline_blob);
                pass.set_bind_group(0, &bind, &[]);
                pass.dispatch_workgroups(
                    (sec.x_size as u32).div_ceil(8),
                    (sec.y_size as u32).div_ceil(8),
                    1,
                );
            }
        }

        self.queue.submit(Some(encoder.finish()));

        let rgba = self.copy_out(&rgba_tex, 4);
        let zb_raw = self.copy_out(&zb_tex, 4);
        let zs_raw = self.copy_out(&zs_tex, 4);
        let sh_raw = self.copy_out(&sh_tex, 4);

        let n = (CANVAS * CANVAS) as usize;
        let mut canvas = Canvas {
            rgba,
            z_body: (0..n)
                .map(|i| i32::from_le_bytes([zb_raw[i * 4], zb_raw[i * 4 + 1], zb_raw[i * 4 + 2], zb_raw[i * 4 + 3]]))
                .collect(),
            z_shadow: (0..n)
                .map(|i| i32::from_le_bytes([zs_raw[i * 4], zs_raw[i * 4 + 1], zs_raw[i * 4 + 2], zs_raw[i * 4 + 3]]))
                .collect(),
            shadow_mask: (0..n).map(|i| sh_raw[i * 4]).collect(),
            opaque: 0,
            bbox: None,
        };
        let (mut min_x, mut min_y, mut max_x, mut max_y) = (u32::MAX, u32::MAX, 0u32, 0u32);
        for y in 0..CANVAS {
            for x in 0..CANVAS {
                let o = ((y * CANVAS + x) * 4) as usize;
                if canvas.rgba[o + 3] != 0 {
                    canvas.opaque += 1;
                    min_x = min_x.min(x);
                    min_y = min_y.min(y);
                    max_x = max_x.max(x);
                    max_y = max_y.max(y);
                }
            }
        }
        if canvas.opaque > 0 {
            canvas.bbox = Some((min_x, min_y, max_x, max_y));
        }
        let _ = ld_v;
        Ok(canvas)
    }

    /// 把一张画布纹理拷回 CPU，去掉每行的对齐填充。
    fn copy_out(&self, tex: &wgpu::Texture, bpp: u32) -> Vec<u8> {
        let row = CANVAS * bpp;
        let padded = row.div_ceil(ROW_ALIGN) * ROW_ALIGN;
        let size = (padded * CANVAS) as u64;
        let buf = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("readback"),
            size,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        let mut enc = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        enc.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &buf,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(padded),
                    rows_per_image: Some(CANVAS),
                },
            },
            wgpu::Extent3d {
                width: CANVAS,
                height: CANVAS,
                depth_or_array_layers: 1,
            },
        );
        self.queue.submit(Some(enc.finish()));

        let slice = buf.slice(..);
        let (tx, rx) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |r| {
            let _ = tx.send(r);
        });
        let _ = self.device.poll(wgpu::PollType::wait_indefinitely());
        if let Ok(Err(e)) = rx.recv() {
            println!("  readback map failed: {e}");
        }

        let mapped = match slice.get_mapped_range() {
            Ok(m) => m,
            Err(e) => {
                println!("  readback range failed: {e}");
                return vec![0u8; (row * CANVAS) as usize];
            }
        };
        let mut out = Vec::with_capacity((row * CANVAS) as usize);
        for y in 0..CANVAS as usize {
            let off = y * padded as usize;
            out.extend_from_slice(&mapped[off..off + row as usize]);
        }
        drop(mapped);
        buf.unmap();
        out
    }

    fn create_tex_3d(&self, w: u32, h: u32, d: u32) -> wgpu::Texture {
        self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("vxl_section"),
            size: wgpu::Extent3d {
                width: w,
                height: h,
                depth_or_array_layers: d,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D3,
            format: wgpu::TextureFormat::Rgba8Uint,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        })
    }

    fn create_tex_2d(&self, w: u32, h: u32, format: wgpu::TextureFormat) -> wgpu::Texture {
        self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("vxl_2d"),
            size: wgpu::Extent3d {
                width: w,
                height: h,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        })
    }

    fn create_storage_2d(&self, w: u32, h: u32, format: wgpu::TextureFormat) -> wgpu::Texture {
        self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("vxl_canvas"),
            size: wgpu::Extent3d {
                width: w,
                height: h,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage: wgpu::TextureUsages::STORAGE_BINDING
                | wgpu::TextureUsages::COPY_SRC
                | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        })
    }

    /// 太阳阴影深度图用的纹理：既要能当 storage（写，且 shader 要读回做 min）也要能当
    /// sampled texture（本体 pass 采样）。同一张纹理在两个不同的 compute pass 里分别扮演这两个
    /// 角色 —— wgpu 允许跨 pass，只要不在同一个 pass 里同时是两个。
    fn create_shadow_2d(&self, size: u32) -> wgpu::Texture {
        self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("vxl_light_depth"),
            size: wgpu::Extent3d {
                width: size,
                height: size,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::R32Float,
            usage: wgpu::TextureUsages::STORAGE_BINDING
                | wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        })
    }

    /// 把整张 R32Float 填成给定值。默认调用值是 1.0 —— 那是"远平面"，也就是"这个 texel
    /// 的光线什么都没碰到"，与 C++ 里给 lightDepthUAV 清 1.0 是同一件事。
    fn clear_f32_2d(&self, tex: &wgpu::Texture, size: u32, value: f32) {
        let row = size * 4;
        let padded = row.div_ceil(ROW_ALIGN) * ROW_ALIGN;
        let mut data = vec![0u8; (padded * size) as usize];
        let bytes = value.to_le_bytes();
        for y in 0..size as usize {
            for x in 0..size as usize {
                let o = y * padded as usize + x * 4;
                data[o..o + 4].copy_from_slice(&bytes);
            }
        }
        self.queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            &data,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(padded),
                rows_per_image: Some(size),
            },
            wgpu::Extent3d {
                width: size,
                height: size,
                depth_or_array_layers: 1,
            },
        );
    }

    fn upload_2d(&self, tex: &wgpu::Texture, data: &[u8], w: u32, h: u32, row_bytes: u32) {
        self.queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            data,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(row_bytes),
                rows_per_image: Some(h),
            },
            wgpu::Extent3d {
                width: w,
                height: h,
                depth_or_array_layers: 1,
            },
        );
    }

    fn clear_2d(&self, tex: &wgpu::Texture, bpp: u32) {
        let row = CANVAS * bpp;
        let padded = row.div_ceil(ROW_ALIGN) * ROW_ALIGN;
        let zeros = vec![0u8; (padded * CANVAS) as usize];
        self.queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            &zeros,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(padded),
                rows_per_image: Some(CANVAS),
            },
            wgpu::Extent3d {
                width: CANVAS,
                height: CANVAS,
                depth_or_array_layers: 1,
            },
        );
    }
}

/// HVA 的 16 个 float（行主序）转成 `Mat4`。`Hva::matrix_for` 保持 C++ 的一维 16 元素布局，
/// 这里只做搬运。
fn mat_from_flat(m: [f32; 16]) -> mat4::Mat4 {
    let mut out = [[0.0f32; 4]; 4];
    for r in 0..4 {
        for c in 0..4 {
            out[r][c] = m[r * 4 + c];
        }
    }
    out
}

fn storage(binding: u32, format: wgpu::TextureFormat) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::COMPUTE,
        ty: wgpu::BindingType::StorageTexture {
            access: wgpu::StorageTextureAccess::WriteOnly,
            format,
            view_dimension: wgpu::TextureViewDimension::D2,
        },
        count: None,
    }
}

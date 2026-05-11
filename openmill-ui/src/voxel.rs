use eframe::wgpu;
use bytemuck::{Pod, Zeroable};

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct VoxelUniforms {
    pub stock_min: [f32; 4],
    pub stock_max: [f32; 4],
    /// Tool **tip** position at the start of the swept move.
    pub tool_start: [f32; 4],
    /// Tool **tip** position at the end of the swept move.
    pub tool_end: [f32; 4],
    /// Unit vector along the tool body, pointing from tip toward shank, at the
    /// start pose (xyz; w unused).
    pub tool_axis_start: [f32; 4],
    /// Same at the end pose.
    pub tool_axis_end: [f32; 4],
    /// `x = cutter_radius`, `y = cutting_length` (flute length), z/w unused.
    pub tool_params: [f32; 4],
    /// `x = op_id` (0-based, cast to u32 in shader). Carved voxels are
    /// stamped with `op_id + 2` so the render shader can color the surface
    /// by which operation removed the adjacent material. y/z/w reserved.
    pub op_info: [f32; 4],
}

pub struct VoxelVolume {
    pub texture: wgpu::Texture,
    pub view: wgpu::TextureView,
    pub compute_pipeline: wgpu::ComputePipeline,
    pub render_pipeline: wgpu::RenderPipeline,
    pub compute_bgl: wgpu::BindGroupLayout,
    pub render_bgl: wgpu::BindGroupLayout,
    pub uniform_buffer: wgpu::Buffer,
    pub resolution: [u32; 3],

    /// SDF of the target part mesh, sampled on a regular grid spanning
    /// the same world-space AABB as the voxel volume. `None`-ish state is
    /// represented by a 1×1×1 zero-filled texture so the render bind
    /// group never needs to be rebuilt as the SDF appears/disappears.
    pub sdf_texture: wgpu::Texture,
    pub sdf_view: wgpu::TextureView,
    pub sdf_sampler: wgpu::Sampler,
    /// `true` once a real SDF has been computed for the current model.
    /// Verify mode checks this before requesting a deviation render.
    pub sdf_ready: bool,
}

impl VoxelVolume {
    const COMPUTE_SHADER: &'static str = r#"
        // Voxel encoding (r32uint):
        //   0u       — filled, original uncut stock material
        //   1u       — empty, untagged background (outside stock AABB, or
        //              pre-initialised by reset())
        //   2u + N   — empty, removed by op N (0-based op index)
        //
        // The render shader uses the per-voxel op tag to color the surface
        // by which operation cut the adjacent material.

        struct Uniforms {
            stock_min:       vec4<f32>,
            stock_max:       vec4<f32>,
            tool_start:      vec4<f32>,
            tool_end:        vec4<f32>,
            tool_axis_start: vec4<f32>,
            tool_axis_end:   vec4<f32>,
            tool_params:     vec4<f32>, // x = radius, y = cutting_length
            op_info:         vec4<f32>, // x = op_id (0-based, cast to u32)
        };

        @group(0) @binding(0) var voxels: texture_storage_3d<r32uint, write>;
        @group(0) @binding(1) var<uniform> u: Uniforms;

        fn dist_to_segment(p: vec3<f32>, a: vec3<f32>, b: vec3<f32>) -> f32 {
            let pa = p - a;
            let ba = b - a;
            let denom = max(dot(ba, ba), 1e-8);
            let h = clamp(dot(pa, ba) / denom, 0.0, 1.0);
            return length(pa - ba * h);
        }

        @compute @workgroup_size(4, 4, 4)
        fn main(@builtin(global_invocation_id) id: vec3<u32>) {
            let dims = vec3<f32>(textureDimensions(voxels));
            if (f32(id.x) >= dims.x || f32(id.y) >= dims.y || f32(id.z) >= dims.z) { return; }

            let norm_pos = vec3<f32>(id) / dims;
            let world_pos = u.stock_min.xyz + norm_pos * (u.stock_max.xyz - u.stock_min.xyz);

            let radius   = u.tool_params.x;
            let cut_len  = max(u.tool_params.y, 0.001);
            let op_tag   = u32(u.op_info.x) + 2u;

            // Sweep the **entire tool body** between the start and end poses.
            // At each height h up the tool body we have a line segment from
            //   tool_start.xyz + axis_start * h
            //   tool_end.xyz   + axis_end   * h
            // The swept volume of all those segments is what the tool removes.
            let SAMPLES: i32 = 6;
            for (var i = 0; i <= SAMPLES; i = i + 1) {
                let h = f32(i) / f32(SAMPLES) * cut_len;
                let a = u.tool_start.xyz + u.tool_axis_start.xyz * h;
                let b = u.tool_end.xyz   + u.tool_axis_end.xyz   * h;
                let d = dist_to_segment(world_pos, a, b);
                if (d < radius) {
                    textureStore(voxels, id, vec4<u32>(op_tag, 0u, 0u, 0u));
                    return;
                }
            }
        }
    "#;

    const RENDER_SHADER: &'static str = r#"
        // op_info layout
        //   x = op_id (carve-time only; render reads from voxel cells)
        //   y = verify_mode (0 = op-color render, 1 = deviation heatmap)
        //   z = tolerance_mm (green band half-width in verify mode)
        //   w = reserved
        struct Uniforms {
            stock_min:       vec4<f32>,
            stock_max:       vec4<f32>,
            tool_start:      vec4<f32>,
            tool_end:        vec4<f32>,
            tool_axis_start: vec4<f32>,
            tool_axis_end:   vec4<f32>,
            tool_params:     vec4<f32>,
            op_info:         vec4<f32>,
        };

        struct ViewUniforms {
            view_proj: mat4x4<f32>,
            light_dir: vec4<f32>,
            camera_pos: vec4<f32>,
        };

        @group(0) @binding(0) var voxels: texture_3d<u32>;
        @group(0) @binding(1) var<uniform> u: Uniforms;
        @group(0) @binding(2) var sdf_tex: texture_3d<f32>;
        @group(0) @binding(3) var sdf_smp: sampler;
        @group(1) @binding(0) var<uniform> v: ViewUniforms;

        struct VertexOutput {
            @builtin(position) position: vec4<f32>,
            @location(0) world_pos: vec3<f32>,
        };

        @vertex
        fn vs_main(@builtin(vertex_index) vid: u32) -> VertexOutput {
            var corners = array<vec3<f32>, 8>(
                vec3<f32>(0.0, 0.0, 0.0),
                vec3<f32>(1.0, 0.0, 0.0),
                vec3<f32>(1.0, 1.0, 0.0),
                vec3<f32>(0.0, 1.0, 0.0),
                vec3<f32>(0.0, 0.0, 1.0),
                vec3<f32>(1.0, 0.0, 1.0),
                vec3<f32>(1.0, 1.0, 1.0),
                vec3<f32>(0.0, 1.0, 1.0),
            );
            var idx = array<u32, 36>(
                0u, 2u, 1u,   0u, 3u, 2u,
                4u, 5u, 6u,   4u, 6u, 7u,
                0u, 4u, 7u,   0u, 7u, 3u,
                1u, 2u, 6u,   1u, 6u, 5u,
                0u, 1u, 5u,   0u, 5u, 4u,
                3u, 7u, 6u,   3u, 6u, 2u,
            );
            let pos = corners[idx[vid]];

            var out: VertexOutput;
            let world = u.stock_min.xyz + pos * (u.stock_max.xyz - u.stock_min.xyz);
            out.position = v.view_proj * vec4(world, 1.0);
            out.world_pos = world;
            return out;
        }

        // Sample the raw u32 stored at a world-space point. Outside the
        // texture domain we return `1u` (empty background) so the ray-
        // march behaves like air there.
        fn voxel_raw(p: vec3<f32>) -> u32 {
            let n = (p - u.stock_min.xyz) / (u.stock_max.xyz - u.stock_min.xyz);
            if (any(n < vec3(0.0)) || any(n > vec3(1.0))) { return 1u; }
            let d = vec3<f32>(textureDimensions(voxels));
            let i = vec3<i32>(clamp(n, vec3(0.0), vec3(1.0)) * (d - 1.0));
            return textureLoad(voxels, i, 0).r;
        }

        // 1.0 when the sampled point is solid (0u), else 0.0. Used by the
        // gradient-normal estimator so the central differences resolve a
        // proper unit boundary.
        fn solid_amount(p: vec3<f32>) -> f32 {
            return select(0.0, 1.0, voxel_raw(p) == 0u);
        }

        // Outward-facing surface normal via central differences on
        // `solid_amount`. Identical to the previous gradient method except
        // it works on the new u32 encoding.
        fn voxel_normal(p: vec3<f32>) -> vec3<f32> {
            let h = (u.stock_max.xyz - u.stock_min.xyz) / vec3<f32>(textureDimensions(voxels));
            let dx = solid_amount(p + vec3(h.x, 0.0, 0.0)) - solid_amount(p - vec3(h.x, 0.0, 0.0));
            let dy = solid_amount(p + vec3(0.0, h.y, 0.0)) - solid_amount(p - vec3(0.0, h.y, 0.0));
            let dz = solid_amount(p + vec3(0.0, 0.0, h.z)) - solid_amount(p - vec3(0.0, 0.0, h.z));
            let g = vec3(dx, dy, dz);
            let len = length(g);
            if (len < 1e-4) { return vec3(0.0, 0.0, 1.0); }
            return -g / len;
        }

        // Surface color by op tag. `tag` is the raw u32 from the **empty
        // voxel adjacent to the surface** — see ray-march below for how
        // that's tracked. Tags 0/1 mean "no op cut here", which renders
        // as the warm uncut stock color so the original outer faces look
        // material-like; higher tags map to a distinct hue per op.
        fn op_color(tag: u32) -> vec3<f32> {
            if (tag < 2u) { return vec3(0.90, 0.62, 0.30); } // uncut stock
            var palette = array<vec3<f32>, 12>(
                vec3(0.35, 0.75, 0.95),
                vec3(0.95, 0.45, 0.45),
                vec3(0.55, 0.85, 0.45),
                vec3(0.95, 0.75, 0.35),
                vec3(0.75, 0.55, 0.95),
                vec3(0.45, 0.85, 0.75),
                vec3(0.95, 0.55, 0.75),
                vec3(0.85, 0.85, 0.45),
                vec3(0.55, 0.75, 0.95),
                vec3(0.95, 0.65, 0.45),
                vec3(0.65, 0.95, 0.85),
                vec3(0.85, 0.45, 0.65),
            );
            return palette[(tag - 2u) % 12u];
        }

        // Heatmap for stock-vs-target deviation. `d_mm` is the signed
        // distance from the carved surface to the nominal part mesh
        // (positive = excess material, negative = gouge into part).
        // `tol_mm` defines the green "on size" band. Outside that we
        // ramp warm (gouge) or cool (excess) by `|d| / (5 × tol)`.
        fn deviation_color(d_mm: f32, tol_mm: f32) -> vec3<f32> {
            let abs_d = abs(d_mm);
            if (abs_d <= tol_mm) {
                return vec3(0.20, 0.80, 0.30);
            }
            let t = clamp((abs_d - tol_mm) / max(tol_mm * 5.0, 0.01), 0.0, 1.0);
            if (d_mm > 0.0) {
                // Excess material: cyan → deep blue.
                return mix(vec3(0.25, 0.75, 0.95), vec3(0.08, 0.20, 0.85), t);
            } else {
                // Gouge / under-cut: amber → red.
                return mix(vec3(0.95, 0.65, 0.20), vec3(0.90, 0.10, 0.10), t);
            }
        }

        // Sample the SDF (in mm) at a world-space point. Texture coords
        // are `n = (p - stock_min) / extent`, identical to voxel lookup
        // so the SDF resolution can differ from the voxel resolution.
        // Linear filtering gives a smooth heatmap across cells.
        fn sdf_at(p: vec3<f32>) -> f32 {
            let n = (p - u.stock_min.xyz) / (u.stock_max.xyz - u.stock_min.xyz);
            return textureSampleLevel(sdf_tex, sdf_smp, n, 0.0).r;
        }

        @fragment
        fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
            let ray_origin = v.camera_pos.xyz;
            let ray_dir = normalize(in.world_pos - ray_origin);

            let dims = vec3<f32>(textureDimensions(voxels));
            let voxel_size = length(u.stock_max.xyz - u.stock_min.xyz) / length(dims);
            let ray_step = ray_dir * voxel_size;

            // Ray-march from the bounding-box entry point toward solid.
            // `last_empty` tracks the most-recently sampled empty voxel's
            // op tag; when we eventually hit a solid voxel we color the
            // surface using that tag. Pre-seed with `0u` so original outer
            // faces (no carving on the air side) render as uncut stock.
            var p = in.world_pos;
            var last_empty: u32 = 0u;
            let verify = u.op_info.y > 0.5;
            for (var i = 0; i < 384; i = i + 1) {
                let raw = voxel_raw(p);
                if (raw == 0u) {
                    // Hit solid material — shade as a surface.
                    let n = voxel_normal(p);
                    let l = normalize(v.light_dir.xyz);
                    let lambert = max(dot(n, l), 0.0);
                    let view_dir = -ray_dir;
                    let nv = max(dot(n, view_dir), 0.0);
                    var base: vec3<f32>;
                    if (verify) {
                        // "Verify" view: shade by signed distance from
                        // the carved surface to the target mesh. The
                        // user sees instantly where the stock is too
                        // big, on-size, or gouged.
                        let d = sdf_at(p);
                        base = deviation_color(d, max(u.op_info.z, 0.001));
                    } else {
                        base = op_color(last_empty);
                    }
                    let shade = base * (lambert * 0.7 + 0.3) + vec3(0.05) * nv;
                    return vec4(shade, 1.0);
                }
                last_empty = raw;
                p = p + ray_step;
                let n = (p - u.stock_min.xyz) / (u.stock_max.xyz - u.stock_min.xyz);
                if (any(n < vec3(-0.05)) || any(n > vec3(1.05))) { break; }
            }
            discard;
        }
    "#;

    pub fn new(device: &wgpu::Device, resolution: [u32; 3], viewport_layout: &wgpu::BindGroupLayout, target_format: wgpu::TextureFormat) -> Self {
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("voxel_texture"),
            size: wgpu::Extent3d {
                width: resolution[0],
                height: resolution[1],
                depth_or_array_layers: resolution[2],
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D3,
            format: wgpu::TextureFormat::R32Uint,
            usage: wgpu::TextureUsages::STORAGE_BINDING | wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });

        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());

        let compute_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("voxel_compute_bgl"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::COMPUTE,
                    ty: wgpu::BindingType::StorageTexture {
                        access: wgpu::StorageTextureAccess::WriteOnly,
                        format: wgpu::TextureFormat::R32Uint,
                        view_dimension: wgpu::TextureViewDimension::D3,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
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

        let render_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("voxel_render_bgl"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Uint,
                        view_dimension: wgpu::TextureViewDimension::D3,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT | wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                // SDF used for the "Verify" deviation heatmap. Always
                // bound (a 1×1×1 zero-filled dummy keeps the binding
                // valid when no model has been analysed yet).
                // `filterable: false` + `NonFiltering` sampler keeps the
                // SDF on `R32Float` without requiring the
                // `FLOAT32_FILTERABLE` device feature — the heatmap
                // sampling falls back to nearest, which is fine for a
                // visual indicator at 128³ resolution.
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: false },
                        view_dimension: wgpu::TextureViewDimension::D3,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 3,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::NonFiltering),
                    count: None,
                },
            ],
        });

        let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("voxel_uniform_buffer"),
            size: std::mem::size_of::<VoxelUniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let compute_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("voxel_compute_shader"),
            source: wgpu::ShaderSource::Wgsl(Self::COMPUTE_SHADER.into()),
        });

        let compute_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("voxel_compute_layout"),
            bind_group_layouts: &[&compute_bgl],
            push_constant_ranges: &[],
        });

        let compute_pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("voxel_compute_pipeline"),
            layout: Some(&compute_pipeline_layout),
            module: &compute_shader,
            entry_point: "main",
            compilation_options: Default::default(),
            cache: None,
        });

        let render_shader_module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("voxel_render_shader"),
            source: wgpu::ShaderSource::Wgsl(Self::RENDER_SHADER.into()),
        });

        let render_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("voxel_render_layout"),
            bind_group_layouts: &[&render_bgl, viewport_layout],
            push_constant_ranges: &[],
        });

        let render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("voxel_render_pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &render_shader_module,
                entry_point: "vs_main",
                buffers: &[],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &render_shader_module,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        // 1×1×1 placeholder SDF — kept "always allocated" so the render
        // bind group references a real texture even when no model has
        // been analysed. Replaced by `upload_sdf` when the user enables
        // Verify mode. `R32Float` rather than `R16Snorm` because the
        // float format is in wgpu's default feature set on every
        // backend; r16snorm needs `TEXTURE_FORMAT_16BIT_NORM` enabled
        // at device creation, which eframe doesn't request by default.
        let sdf_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("sdf_texture"),
            size: wgpu::Extent3d { width: 1, height: 1, depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D3,
            format: wgpu::TextureFormat::R32Float,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        let sdf_view = sdf_texture.create_view(&wgpu::TextureViewDescriptor::default());
        // Non-filtering sampler so the SDF stays on `R32Float` without
        // requiring the `FLOAT32_FILTERABLE` wgpu device feature.
        let sdf_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("sdf_sampler"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Nearest,
            min_filter: wgpu::FilterMode::Nearest,
            mipmap_filter: wgpu::FilterMode::Nearest,
            ..Default::default()
        });

        Self {
            texture,
            view,
            compute_pipeline,
            render_pipeline,
            compute_bgl,
            render_bgl,
            uniform_buffer,
            resolution,
            sdf_texture,
            sdf_view,
            sdf_sampler,
            sdf_ready: false,
        }
    }

    /// Upload a precomputed SDF (signed-distance field, f32 mm) to GPU.
    /// The old texture/view are replaced; the caller must rebuild any
    /// bind group that referenced the previous view.
    pub fn upload_sdf(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        data: &[f32],
        resolution: [u32; 3],
    ) {
        let expected = (resolution[0] * resolution[1] * resolution[2]) as usize;
        if data.len() != expected {
            log::warn!(
                "upload_sdf: data length {} != expected {} for resolution {:?}",
                data.len(), expected, resolution
            );
            return;
        }
        let tex = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("sdf_texture"),
            size: wgpu::Extent3d {
                width: resolution[0],
                height: resolution[1],
                depth_or_array_layers: resolution[2],
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D3,
            format: wgpu::TextureFormat::R32Float,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        queue.write_texture(
            wgpu::ImageCopyTexture {
                texture: &tex,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            bytemuck::cast_slice(data),
            wgpu::ImageDataLayout {
                offset: 0,
                bytes_per_row: Some(resolution[0] * 4),
                rows_per_image: Some(resolution[1]),
            },
            wgpu::Extent3d {
                width: resolution[0],
                height: resolution[1],
                depth_or_array_layers: resolution[2],
            },
        );
        self.sdf_view = tex.create_view(&wgpu::TextureViewDescriptor::default());
        self.sdf_texture = tex;
        self.sdf_ready = true;
    }

    pub fn reset(&self, queue: &wgpu::Queue) {
        // Initial state: every voxel is `0u` = filled (uncut original
        // stock). Carving sets voxels to `2u + op_id`. The render shader
        // treats 0 as solid and anything else as empty.
        let texel_count = (self.resolution[0] * self.resolution[1] * self.resolution[2]) as usize;
        let data = vec![0u8; texel_count * 4];
        queue.write_texture(
            wgpu::ImageCopyTexture {
                texture: &self.texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            &data,
            wgpu::ImageDataLayout {
                offset: 0,
                bytes_per_row: Some(self.resolution[0] * 4),
                rows_per_image: Some(self.resolution[1]),
            },
            wgpu::Extent3d {
                width: self.resolution[0],
                height: self.resolution[1],
                depth_or_array_layers: self.resolution[2],
            },
        );
    }
}

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
}

impl VoxelVolume {
    const COMPUTE_SHADER: &'static str = r#"
        struct Uniforms {
            stock_min:       vec4<f32>,
            stock_max:       vec4<f32>,
            tool_start:      vec4<f32>,
            tool_end:        vec4<f32>,
            tool_axis_start: vec4<f32>,
            tool_axis_end:   vec4<f32>,
            tool_params:     vec4<f32>, // x = radius, y = cutting_length
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

            // Sweep the **entire tool body** between the start and end poses.
            // At each height h up the tool body we have a line segment from
            //   tool_start.xyz + axis_start * h
            //   tool_end.xyz   + axis_end   * h
            // The swept volume of all those segments is what the tool removes.
            // Sampling several heights along the body approximates that
            // volume — without this, only the thin tip-path tube was carved
            // and material above the tip never disappeared.
            let SAMPLES: i32 = 6;
            for (var i = 0; i <= SAMPLES; i = i + 1) {
                let h = f32(i) / f32(SAMPLES) * cut_len;
                let a = u.tool_start.xyz + u.tool_axis_start.xyz * h;
                let b = u.tool_end.xyz   + u.tool_axis_end.xyz   * h;
                let d = dist_to_segment(world_pos, a, b);
                if (d < radius) {
                    textureStore(voxels, id, vec4<u32>(0, 0, 0, 0));
                    return;
                }
            }
        }
    "#;

    const RENDER_SHADER: &'static str = r#"
        struct Uniforms {
            stock_min:       vec4<f32>,
            stock_max:       vec4<f32>,
            tool_start:      vec4<f32>,
            tool_end:        vec4<f32>,
            tool_axis_start: vec4<f32>,
            tool_axis_end:   vec4<f32>,
            tool_params:     vec4<f32>,
        };

        struct ViewUniforms {
            view_proj: mat4x4<f32>,
            light_dir: vec4<f32>,
            camera_pos: vec4<f32>,
        };

        @group(0) @binding(0) var voxels: texture_3d<u32>;
        @group(0) @binding(1) var<uniform> u: Uniforms;
        @group(1) @binding(0) var<uniform> v: ViewUniforms;

        struct VertexOutput {
            @builtin(position) position: vec4<f32>,
            @location(0) world_pos: vec3<f32>,
        };

        @vertex
        fn vs_main(@builtin(vertex_index) vid: u32) -> VertexOutput {
            // Unit-cube corners (0..1 on each axis), then 36 indices forming
            // 12 triangles — six faces with consistent winding so every face
            // rasterises and the raymarcher can sample from any view angle.
            var corners = array<vec3<f32>, 8>(
                vec3<f32>(0.0, 0.0, 0.0),  // 0
                vec3<f32>(1.0, 0.0, 0.0),  // 1
                vec3<f32>(1.0, 1.0, 0.0),  // 2
                vec3<f32>(0.0, 1.0, 0.0),  // 3
                vec3<f32>(0.0, 0.0, 1.0),  // 4
                vec3<f32>(1.0, 0.0, 1.0),  // 5
                vec3<f32>(1.0, 1.0, 1.0),  // 6
                vec3<f32>(0.0, 1.0, 1.0),  // 7
            );
            var idx = array<u32, 36>(
                // -Z (bottom)
                0u, 2u, 1u,   0u, 3u, 2u,
                // +Z (top)
                4u, 5u, 6u,   4u, 6u, 7u,
                // -X (left)
                0u, 4u, 7u,   0u, 7u, 3u,
                // +X (right)
                1u, 2u, 6u,   1u, 6u, 5u,
                // -Y (front)
                0u, 1u, 5u,   0u, 5u, 4u,
                // +Y (back)
                3u, 7u, 6u,   3u, 6u, 2u,
            );
            let pos = corners[idx[vid]];

            var out: VertexOutput;
            let world = u.stock_min.xyz + pos * (u.stock_max.xyz - u.stock_min.xyz);
            out.position = v.view_proj * vec4(world, 1.0);
            out.world_pos = world;
            return out;
        }

        // Sample the voxel field at a world-space point. Returns 1.0 if the
        // voxel is filled (still has material) or 0.0 if it's been carved /
        // outside the stock volume.
        fn voxel_at(p: vec3<f32>) -> f32 {
            let n = (p - u.stock_min.xyz) / (u.stock_max.xyz - u.stock_min.xyz);
            if (any(n < vec3(0.0)) || any(n > vec3(1.0))) {
                return 0.0;
            }
            let d = vec3<f32>(textureDimensions(voxels));
            let i = vec3<i32>(clamp(n, vec3(0.0), vec3(1.0)) * (d - 1.0));
            return f32(textureLoad(voxels, i, 0).r);
        }

        // Surface normal from the voxel field gradient (central differences).
        // The voxel field is 1 inside the stock, 0 outside, so the gradient
        // points from solid into empty — that's the outward-facing surface
        // normal we want for Lambertian shading.
        fn voxel_normal(p: vec3<f32>) -> vec3<f32> {
            let h = (u.stock_max.xyz - u.stock_min.xyz) / vec3<f32>(textureDimensions(voxels));
            let dx = voxel_at(p + vec3(h.x, 0.0, 0.0)) - voxel_at(p - vec3(h.x, 0.0, 0.0));
            let dy = voxel_at(p + vec3(0.0, h.y, 0.0)) - voxel_at(p - vec3(0.0, h.y, 0.0));
            let dz = voxel_at(p + vec3(0.0, 0.0, h.z)) - voxel_at(p - vec3(0.0, 0.0, h.z));
            let g = vec3(dx, dy, dz);
            let len = length(g);
            if (len < 1e-4) { return vec3(0.0, 0.0, 1.0); }
            return -g / len;
        }

        @fragment
        fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
            let ray_origin = v.camera_pos.xyz;
            let ray_dir = normalize(in.world_pos - ray_origin);

            // Step size proportional to one voxel — fewer skipped surfaces
            // and the carved cavities show up at the right scale.
            let dims = vec3<f32>(textureDimensions(voxels));
            let voxel_size = length(u.stock_max.xyz - u.stock_min.xyz) / length(dims);
            let ray_step = ray_dir * voxel_size;

            var p = in.world_pos;
            for (var i = 0; i < 384; i = i + 1) {
                let v_here = voxel_at(p);
                if (v_here > 0.5) {
                    // Lit shading using the gradient-based surface normal so
                    // concave carve cavities render visibly distinct from the
                    // convex outer faces — without this every voxel surface
                    // had identical colour and the stock looked unchanged.
                    let n = voxel_normal(p);
                    let l = normalize(v.light_dir.xyz);
                    let lambert = max(dot(n, l), 0.0);
                    // Two-tone: warm orange exterior, slightly cooler tone
                    // inside cavities (where the normal flips toward camera).
                    let view_dir = -ray_dir;
                    let nv = max(dot(n, view_dir), 0.0);
                    let base = mix(vec3(0.55, 0.30, 0.10), vec3(0.95, 0.55, 0.20), nv);
                    let shade = base * (lambert * 0.75 + 0.25);
                    return vec4(shade, 1.0);
                }
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

        Self {
            texture,
            view,
            compute_pipeline,
            render_pipeline,
            compute_bgl,
            render_bgl,
            uniform_buffer,
            resolution,
        }
    }

    pub fn reset(&self, queue: &wgpu::Queue) {
        let texel_count = (self.resolution[0] * self.resolution[1] * self.resolution[2]) as usize;
        let data = vec![0xFFu8; texel_count * 4];
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

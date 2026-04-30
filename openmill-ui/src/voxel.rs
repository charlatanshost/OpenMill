use eframe::wgpu;
use bytemuck::{Pod, Zeroable};

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct VoxelUniforms {
    pub stock_min: [f32; 4],
    pub stock_max: [f32; 4],
    pub tool_start: [f32; 4],
    pub tool_end: [f32; 4],
    pub tool_params: [f32; 4], // x = radius, y = length
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
            stock_min: vec4<f32>,
            stock_max: vec4<f32>,
            tool_start: vec4<f32>,
            tool_end: vec4<f32>,
            tool_params: vec4<f32>,
        };

        @group(0) @binding(0) var voxels: texture_storage_3d<r32uint, write>;
        @group(0) @binding(1) var<uniform> u: Uniforms;

        fn dist_to_segment(p: vec3<f32>, a: vec3<f32>, b: vec3<f32>) -> f32 {
            let pa = p - a;
            let ba = b - a;
            let h = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
            return length(pa - ba * h);
        }

        @compute @workgroup_size(4, 4, 4)
        fn main(@builtin(global_invocation_id) id: vec3<u32>) {
            let dims = vec3<f32>(textureDimensions(voxels));
            if (f32(id.x) >= dims.x || f32(id.y) >= dims.y || f32(id.z) >= dims.z) { return; }

            let norm_pos = vec3<f32>(id) / dims;
            let world_pos = u.stock_min.xyz + norm_pos * (u.stock_max.xyz - u.stock_min.xyz);

            let dist = dist_to_segment(world_pos, u.tool_start.xyz, u.tool_end.xyz);
            if (dist < u.tool_params.x) {
                textureStore(voxels, id, vec4<u32>(0, 0, 0, 0));
            }
        }
    "#;

    const RENDER_SHADER: &'static str = r#"
        struct Uniforms {
            stock_min: vec4<f32>,
            stock_max: vec4<f32>,
            tool_start: vec4<f32>,
            tool_end: vec4<f32>,
            tool_params: vec4<f32>,
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
            var pos: vec3<f32>;
            let v0 = vec3<f32>(0.0, 0.0, 0.0);
            let v1 = vec3<f32>(1.0, 0.0, 0.0);
            let v2 = vec3<f32>(1.0, 1.0, 0.0);
            let v3 = vec3<f32>(0.0, 1.0, 0.0);
            let v4 = vec3<f32>(0.0, 0.0, 1.0);
            let v5 = vec3<f32>(1.0, 0.0, 1.0);
            let v6 = vec3<f32>(1.0, 1.0, 1.0);
            let v7 = vec3<f32>(0.0, 1.0, 1.0);

            switch (vid) {
                case 0u, 5u, 34u: { pos = v0; }
                case 1u, 11u, 32u: { pos = v1; }
                case 2u, 3u, 8u: { pos = v2; }
                case 4u, 20u, 29u: { pos = v3; }
                case 18u, 23u, 35u: { pos = v4; }
                case 6u, 12u, 31u: { pos = v5; }
                case 9u, 10u, 16u, 26u: { pos = v6; }
                case 15u, 22u, 28u: { pos = v7; }
                // Remaining cases for the 36 vertices
                case 7u, 17u: { pos = v5; }
                case 13u, 19u: { pos = v4; }
                case 14u, 21u: { pos = v7; }
                case 24u, 25u: { pos = v2; }
                case 27u, 30u: { pos = v6; }
                default: { pos = v0; }
            }
            
            var out: VertexOutput;
            let world = u.stock_min.xyz + pos * (u.stock_max.xyz - u.stock_min.xyz);
            out.position = v.view_proj * vec4(world, 1.0);
            out.world_pos = world;
            return out;
        }

        @fragment
        fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
            let ray_origin = v.camera_pos.xyz;
            let ray_dir = normalize(in.world_pos - ray_origin);
            
            var p = in.world_pos;
            // Adaptive step size based on resolution
            let step_size = length(u.stock_max.xyz - u.stock_min.xyz) / 256.0;
            let ray_step = ray_dir * step_size;
            
            for (var i = 0; i < 256; i++) {
                let norm_pos = (p - u.stock_min.xyz) / (u.stock_max.xyz - u.stock_min.xyz);
                if (any(norm_pos < vec3(-0.01)) || any(norm_pos > vec3(1.01))) { break; }
                
                let tex_dims = vec3<f32>(textureDimensions(voxels));
                let val = textureLoad(voxels, vec3<i32>(clamp(norm_pos, vec3(0.0), vec3(1.0)) * (tex_dims - 1.0)), 0).r;
                
                if (val > 0u) {
                    // Shading
                    let l = normalize(v.light_dir.xyz);
                    let diffuse = max(dot(vec3(0.0, 0.0, 1.0), l), 0.0) * 0.6 + 0.4;
                    return vec4<f32>(vec3(0.8, 0.4, 0.1) * diffuse, 1.0);
                }
                p += ray_step;
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

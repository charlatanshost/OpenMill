//! 3D viewport: orbit camera, grid, mesh rendering via wgpu render-to-texture.

use bytemuck::{Pod, Zeroable};
use eframe::egui;
use eframe::egui_wgpu;
use eframe::wgpu;
use eframe::wgpu::util::DeviceExt;

// ── Vertex ──────────────────────────────────────────────────────────────────

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
struct Vertex {
    position: [f32; 3],
    normal: [f32; 3],
    color: [f32; 3],
}

const VERTEX_ATTRS: [wgpu::VertexAttribute; 3] = [
    wgpu::VertexAttribute {
        format: wgpu::VertexFormat::Float32x3,
        offset: 0,
        shader_location: 0,
    },
    wgpu::VertexAttribute {
        format: wgpu::VertexFormat::Float32x3,
        offset: 12,
        shader_location: 1,
    },
    wgpu::VertexAttribute {
        format: wgpu::VertexFormat::Float32x3,
        offset: 24,
        shader_location: 2,
    },
];

fn vertex_layout() -> wgpu::VertexBufferLayout<'static> {
    wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &VERTEX_ATTRS,
    }
}

// ── Uniforms ────────────────────────────────────────────────────────────────

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
struct Uniforms {
    view_proj: [[f32; 4]; 4],
    light_dir: [f32; 4], // xyz = direction, w = pad
    camera_pos: [f32; 4], // xyz = position, w = pad
}

// ── WGSL shader ─────────────────────────────────────────────────────────────

const SHADER_SRC: &str = r#"
struct Uniforms {
    view_proj: mat4x4<f32>,
    light_dir: vec4<f32>,
    camera_pos: vec4<f32>,
};

@group(0) @binding(0)
var<uniform> u: Uniforms;

struct VsOut {
    @builtin(position) clip_pos: vec4<f32>,
    @location(0) normal: vec3<f32>,
    @location(1) color: vec3<f32>,
};

@vertex
fn vs_main(
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec3<f32>,
) -> VsOut {
    var out: VsOut;
    out.clip_pos = u.view_proj * vec4(position, 1.0);
    out.normal = normal;
    out.color = color;
    return out;
}

@fragment
fn fs_lit(in: VsOut) -> @location(0) vec4<f32> {
    let n = normalize(in.normal);
    let l = normalize(u.light_dir.xyz);
    let d = max(dot(n, l), 0.0) * 0.7 + 0.3;
    return vec4(in.color * d, 1.0);
}

@fragment
fn fs_flat(in: VsOut) -> @location(0) vec4<f32> {
    return vec4(in.color, 1.0);
}
"#;

// ── Orbit camera ────────────────────────────────────────────────────────────

pub struct OrbitCamera {
    pub target: [f32; 3],
    pub distance: f32,
    pub azimuth: f32,
    pub elevation: f32,
    pub fov_y: f32,
}

impl Default for OrbitCamera {
    fn default() -> Self {
        Self {
            target: [0.0, 0.0, 0.0],
            distance: 150.0,
            azimuth: 0.8,  // ~45 deg
            elevation: 0.5, // ~30 deg
            fov_y: 0.8,     // ~45 deg
        }
    }
}

impl OrbitCamera {
    pub fn eye(&self) -> [f32; 3] {
        let ce = self.elevation.cos();
        [
            self.target[0] + self.distance * ce * self.azimuth.cos(),
            self.target[1] + self.distance * ce * self.azimuth.sin(),
            self.target[2] + self.distance * self.elevation.sin(),
        ]
    }

    pub fn view_proj(&self, aspect: f32) -> [[f32; 4]; 4] {
        let view = look_at(self.eye(), self.target, [0.0, 0.0, 1.0]);
        let proj = perspective(self.fov_y, aspect, 0.1, 50000.0);
        mat4_mul(proj, view)
    }

    /// Handle mouse drag and scroll for orbit/pan/zoom.
    pub fn handle_input(&mut self, response: &egui::Response, scroll: f32) {
        // Left drag: orbit
        if response.dragged_by(egui::PointerButton::Primary) {
            let d = response.drag_delta();
            self.azimuth -= d.x * 0.005;
            self.elevation = (self.elevation + d.y * 0.005)
                .clamp(-1.5, 1.5);
        }
        // Middle drag or right drag: pan
        if response.dragged_by(egui::PointerButton::Middle)
            || response.dragged_by(egui::PointerButton::Secondary)
        {
            let d = response.drag_delta();
            let speed = self.distance * 0.0015;
            let right = [self.azimuth.sin(), -self.azimuth.cos(), 0.0];
            self.target[0] += right[0] * d.x * speed;
            self.target[1] += right[1] * d.x * speed;
            self.target[2] += d.y * speed;
        }
        // Scroll: zoom
        if scroll.abs() > 0.0 {
            self.distance *= 1.0 - scroll * 0.002;
            self.distance = self.distance.clamp(1.0, 50000.0);
        }
    }

    /// Centre the camera on a bounding box.
    pub fn focus_on_aabb(&mut self, mins: [f32; 3], maxs: [f32; 3]) {
        self.target = [
            (mins[0] + maxs[0]) * 0.5,
            (mins[1] + maxs[1]) * 0.5,
            (mins[2] + maxs[2]) * 0.5,
        ];
        let dx = maxs[0] - mins[0];
        let dy = maxs[1] - mins[1];
        let dz = maxs[2] - mins[2];
        let diag = (dx * dx + dy * dy + dz * dz).sqrt();
        self.distance = diag * 1.5;
    }
}

// ── Viewport renderer ───────────────────────────────────────────────────────

pub struct Viewport {
    mesh_pipeline: wgpu::RenderPipeline,
    line_pipeline: wgpu::RenderPipeline,
    uniform_buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
    bind_group_layout: wgpu::BindGroupLayout,

    grid_buffer: wgpu::Buffer,
    grid_count: u32,

    mesh_buffer: Option<wgpu::Buffer>,
    mesh_count: u32,

    stock_buffer: Option<wgpu::Buffer>,
    stock_count: u32,

    tool_buffer: Option<wgpu::Buffer>,
    tool_count: u32,

    path_buffer: Option<wgpu::Buffer>,
    path_count: u32,

    collision_buffer: Option<wgpu::Buffer>,
    collision_count: u32,

    color_tex: wgpu::Texture,
    color_view: wgpu::TextureView,
    depth_tex: wgpu::Texture,
    depth_view: wgpu::TextureView,
    tex_size: [u32; 2],
    pub texture_id: egui::TextureId,
    target_format: wgpu::TextureFormat,
    voxel_volume: Option<crate::voxel::VoxelVolume>,
    voxel_compute_bg: Option<wgpu::BindGroup>,
    voxel_render_bg: Option<wgpu::BindGroup>,
    voxel_uniforms: crate::voxel::VoxelUniforms,
}

impl Viewport {
    pub fn new(render_state: &egui_wgpu::RenderState) -> Self {
        let device = &render_state.device;
        let target_format = render_state.target_format;

        // Shader
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("viewport_shader"),
            source: wgpu::ShaderSource::Wgsl(SHADER_SRC.into()),
        });

        // Uniform buffer + bind group
        let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("viewport_uniforms"),
            size: std::mem::size_of::<Uniforms>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("viewport_bgl"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("viewport_bg"),
            layout: &bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            }],
        });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("viewport_pl"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });

        let depth_stencil = wgpu::DepthStencilState {
            format: wgpu::TextureFormat::Depth32Float,
            depth_write_enabled: true,
            depth_compare: wgpu::CompareFunction::Less,
            stencil: Default::default(),
            bias: Default::default(),
        };

        // Mesh pipeline (triangles, lit, back-face culled)
        let mesh_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("mesh_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[vertex_layout()],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_lit",
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None, // STL winding is unreliable
                ..Default::default()
            },
            depth_stencil: Some(depth_stencil.clone()),
            multisample: Default::default(),
            multiview: None,
            cache: None,
        });

        // Line pipeline (lines, flat color, no culling)
        let line_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("line_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[vertex_layout()],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_flat",
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                ..Default::default()
            },
            depth_stencil: Some(depth_stencil),
            multisample: Default::default(),
            multiview: None,
            cache: None,
        });

        // Grid
        let grid_verts = build_grid(200.0, 10.0);
        let grid_count = grid_verts.len() as u32;
        let grid_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("grid_buffer"),
            contents: bytemuck::cast_slice(&grid_verts),
            usage: wgpu::BufferUsages::VERTEX,
        });

        // Initial render target (1x1, will resize)
        let (color_tex, color_view, depth_tex, depth_view) =
            create_render_targets(device, target_format, 1, 1);

        let texture_id = render_state.renderer.write().register_native_texture(
            device,
            &color_view,
            wgpu::FilterMode::Linear,
        );

        Self {
            mesh_pipeline,
            line_pipeline,
            uniform_buffer,
            bind_group,
            bind_group_layout,
            grid_buffer,
            grid_count,
            mesh_buffer: None,
            mesh_count: 0,
            stock_buffer: None,
            stock_count: 0,
            tool_buffer: None,
            tool_count: 0,
            path_buffer: None,
            path_count: 0,
            collision_buffer: None,
            collision_count: 0,
            color_tex,
            color_view,
            depth_tex,
            depth_view,
            tex_size: [1, 1],
            texture_id,
            target_format,
            voxel_volume: None,
            voxel_compute_bg: None,
            voxel_render_bg: None,
            voxel_uniforms: crate::voxel::VoxelUniforms {
                stock_min: [0.0; 4],
                stock_max: [0.0; 4],
                tool_start: [0.0; 4],
                tool_end: [0.0; 4],
                tool_params: [0.0; 4],
            },
        }
    }

    /// Upload a parry3d TriMesh as the model to render.
    pub fn upload_mesh(
        &mut self,
        device: &wgpu::Device,
        mesh: &parry3d::shape::TriMesh,
    ) {
        let verts = mesh_to_vertices(mesh);
        self.mesh_count = verts.len() as u32;
        if verts.is_empty() {
            self.mesh_buffer = None;
            return;
        }
        self.mesh_buffer = Some(device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("mesh_buffer"),
                contents: bytemuck::cast_slice(&verts),
                usage: wgpu::BufferUsages::VERTEX,
            },
        ));
    }

    /// Upload stock geometry as a wireframe.
    pub fn upload_stock(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        aabb: &parry3d::bounding_volume::Aabb,
        stock: &openmill_core::model::StockShape,
    ) {
        let mut v = Vec::new();
        let color = [1.0, 0.65, 0.0]; // Orange wireframe
        let zero_n = [0.0; 3];

        let add_line = |v: &mut Vec<Vertex>, p1: [f32; 3], p2: [f32; 3]| {
            v.push(Vertex { position: p1, normal: zero_n, color });
            v.push(Vertex { position: p2, normal: zero_n, color });
        };

        match stock {
            openmill_core::model::StockShape::BoundingBox { margin } => {
                let min_x = aabb.mins.x - margin.x as f32;
                let min_y = aabb.mins.y - margin.y as f32;
                let min_z = aabb.mins.z - margin.z as f32;
                let max_x = aabb.maxs.x + margin.x as f32;
                let max_y = aabb.maxs.y + margin.y as f32;
                let max_z = aabb.maxs.z + margin.z as f32;

                let p = [
                    [min_x, min_y, min_z], [max_x, min_y, min_z],
                    [max_x, max_y, min_z], [min_x, max_y, min_z],
                    [min_x, min_y, max_z], [max_x, min_y, max_z],
                    [max_x, max_y, max_z], [min_x, max_y, max_z],
                ];
                // Bottom
                add_line(&mut v, p[0], p[1]); add_line(&mut v, p[1], p[2]);
                add_line(&mut v, p[2], p[3]); add_line(&mut v, p[3], p[0]);
                // Top
                add_line(&mut v, p[4], p[5]); add_line(&mut v, p[5], p[6]);
                add_line(&mut v, p[6], p[7]); add_line(&mut v, p[7], p[4]);
                // Sides
                add_line(&mut v, p[0], p[4]); add_line(&mut v, p[1], p[5]);
                add_line(&mut v, p[2], p[6]); add_line(&mut v, p[3], p[7]);
            }
            openmill_core::model::StockShape::Cylinder { diameter, height } => {
                let cx = (aabb.mins.x + aabb.maxs.x) * 0.5;
                let cy = (aabb.mins.y + aabb.maxs.y) * 0.5;
                let min_z = aabb.mins.z;
                let max_z = min_z + *height as f32;
                let r = (*diameter * 0.5) as f32;

                let segments = 32;
                let mut prev_x = cx + r;
                let mut prev_y = cy;
                for i in 1..=segments {
                    let angle = (i as f32 / segments as f32) * std::f32::consts::TAU;
                    let x = cx + r * angle.cos();
                    let y = cy + r * angle.sin();
                    
                    // Bottom circle
                    add_line(&mut v, [prev_x, prev_y, min_z], [x, y, min_z]);
                    // Top circle
                    add_line(&mut v, [prev_x, prev_y, max_z], [x, y, max_z]);
                    // Vertical lines (4 of them)
                    if i % 8 == 0 {
                        add_line(&mut v, [x, y, min_z], [x, y, max_z]);
                    }

                    prev_x = x;
                    prev_y = y;
                }
            }
        }

        self.stock_count = v.len() as u32;
        if v.is_empty() {
            self.stock_buffer = None;
            return;
        }
        self.stock_buffer = Some(device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("stock_buffer"),
                contents: bytemuck::cast_slice(&v),
                usage: wgpu::BufferUsages::VERTEX,
            },
        ));

        // Initialize Voxel Volume
        let res = [128, 128, 128]; // Lower resolution for stability first
        let voxel_volume = crate::voxel::VoxelVolume::new(device, res, &self.bind_group_layout, self.target_format);
        
        let stock_min = match stock {
            openmill_core::model::StockShape::BoundingBox { margin } => [
                aabb.mins.x - margin.x as f32,
                aabb.mins.y - margin.y as f32,
                aabb.mins.z - margin.z as f32,
                1.0
            ],
            openmill_core::model::StockShape::Cylinder { diameter, height: _ } => [
                (aabb.mins.x + aabb.maxs.x) * 0.5 - (*diameter as f32 * 0.5),
                (aabb.mins.y + aabb.maxs.y) * 0.5 - (*diameter as f32 * 0.5),
                aabb.mins.z,
                1.0
            ],
        };
        let stock_max = match stock {
            openmill_core::model::StockShape::BoundingBox { margin } => [
                aabb.maxs.x + margin.x as f32,
                aabb.maxs.y + margin.y as f32,
                aabb.maxs.z + margin.z as f32,
                1.0
            ],
            openmill_core::model::StockShape::Cylinder { diameter, height } => [
                (aabb.mins.x + aabb.maxs.x) * 0.5 + (*diameter as f32 * 0.5),
                (aabb.mins.y + aabb.maxs.y) * 0.5 + (*diameter as f32 * 0.5),
                aabb.mins.z + *height as f32,
                1.0
            ],
        };

        self.voxel_compute_bg = Some(device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("voxel_compute_bg"),
            layout: &voxel_volume.compute_bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&voxel_volume.view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: voxel_volume.uniform_buffer.as_entire_binding(),
                },
            ],
        }));

        self.voxel_render_bg = Some(device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("voxel_render_bg"),
            layout: &voxel_volume.render_bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&voxel_volume.view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: voxel_volume.uniform_buffer.as_entire_binding(),
                },
            ],
        }));
        
        voxel_volume.reset(queue);

        // Initial uniform update
        self.voxel_uniforms = crate::voxel::VoxelUniforms {
            stock_min,
            stock_max,
            tool_start: [0.0; 4],
            tool_end: [0.0; 4],
            tool_params: [0.0; 4],
        };
        self.voxel_volume = Some(voxel_volume);
    }

    pub fn reset_voxel(&self, queue: &wgpu::Queue) {
        if let Some(ref vol) = self.voxel_volume {
            vol.reset(queue);
        }
    }

    pub fn carve_voxel(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        start: &nalgebra::Isometry3<f32>,
        end: &nalgebra::Isometry3<f32>,
        tool: &openmill_core::Tool,
    ) {
        let Some(ref vol) = self.voxel_volume else { return };
        let Some(ref bg) = self.voxel_compute_bg else { return };

        // Update uniforms
        let start_pos = start.translation.vector;
        let end_pos = end.translation.vector;
        
        self.voxel_uniforms.tool_start = [start_pos.x, start_pos.y, start_pos.z, 1.0];
        self.voxel_uniforms.tool_end = [end_pos.x, end_pos.y, end_pos.z, 1.0];
        self.voxel_uniforms.tool_params = [tool.shape.diameter() as f32 * 0.5, tool.overall_length as f32, 0.0, 0.0];

        queue.write_buffer(&vol.uniform_buffer, 0, bytemuck::cast_slice(&[self.voxel_uniforms]));

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("voxel_carve_encoder") });
        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor { label: Some("voxel_carve_pass"), timestamp_writes: None });
            compute_pass.set_pipeline(&vol.compute_pipeline);
            compute_pass.set_bind_group(0, bg, &[]);
            
            // Dispatch over the volume
            // Optimized dispatch would only cover the tool bounding box
            let x = (vol.resolution[0] + 3) / 4;
            let y = (vol.resolution[1] + 3) / 4;
            let z = (vol.resolution[2] + 3) / 4;
            compute_pass.dispatch_workgroups(x, y, z);
        }
        queue.submit(std::iter::once(encoder.finish()));
    }

    /// Upload tool wireframe for simulation at a specific pose.
    pub fn upload_tool(
        &mut self,
        device: &wgpu::Device,
        pose: Option<&nalgebra::Isometry3<f32>>,
        tool: Option<&openmill_core::Tool>,
    ) {
        if pose.is_none() || tool.is_none() {
            self.tool_buffer = None;
            self.tool_count = 0;
            return;
        }
        let pose = pose.unwrap();
        let tool = tool.unwrap();

        let mut v = Vec::new();
        let color = [0.2, 0.8, 1.0]; // Light blue wireframe for tool
        let zero_n = [0.0; 3];

        let add_line = |v: &mut Vec<Vertex>, p1: nalgebra::Point3<f32>, p2: nalgebra::Point3<f32>| {
            // Transform points by pose
            let tp1 = pose * p1;
            let tp2 = pose * p2;
            v.push(Vertex { position: [tp1.x, tp1.y, tp1.z], normal: zero_n, color });
            v.push(Vertex { position: [tp2.x, tp2.y, tp2.z], normal: zero_n, color });
        };

        // Draw a simple cylinder
        let r = tool.shape.diameter() as f32 / 2.0;
        let h = tool.overall_length as f32;
        let segments = 16;
        let mut prev_pt = nalgebra::Point3::new(r, 0.0, 0.0);
        
        for i in 1..=segments {
            let angle = (i as f32 / segments as f32) * std::f32::consts::TAU;
            let x = r * angle.cos();
            let y = r * angle.sin();
            let pt = nalgebra::Point3::new(x, y, 0.0);
            let pt_top = nalgebra::Point3::new(x, y, h);
            let prev_top = nalgebra::Point3::new(prev_pt.x, prev_pt.y, h);
            
            // Bottom circle
            add_line(&mut v, prev_pt, pt);
            // Top circle
            add_line(&mut v, prev_top, pt_top);
            // Vertical lines
            if i % 4 == 0 {
                add_line(&mut v, pt, pt_top);
            }
            prev_pt = pt;
        }

        self.tool_count = v.len() as u32;
        if v.is_empty() {
            self.tool_buffer = None;
            return;
        }
        self.tool_buffer = Some(device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("tool_buffer"),
                contents: bytemuck::cast_slice(&v),
                usage: wgpu::BufferUsages::VERTEX,
            },
        ));
    }

    pub fn upload_toolpath(&mut self, device: &wgpu::Device, paths: &[openmill_core::Toolpath]) {
        let mut v = Vec::new();
        let zero_n = [0.0; 3];

        for tp in paths {
            for i in 0..tp.points.len().saturating_sub(1) {
                let p1 = tp.points[i].position;
                let p2 = tp.points[i + 1].position;
                let mtype = tp.points[i+1].move_type;

                // Color based on move type
                let color = match mtype {
                    openmill_core::toolpath::MoveType::Rapid => [1.0, 0.2, 0.2],    // Red
                    openmill_core::toolpath::MoveType::Linear => [0.2, 0.8, 1.0],   // Cyan
                    openmill_core::toolpath::MoveType::LeadIn | openmill_core::toolpath::MoveType::LeadOut => [0.2, 1.0, 0.2], // Green
                    openmill_core::toolpath::MoveType::Retract => [1.0, 1.0, 0.2], // Yellow
                };

                v.push(Vertex { 
                    position: [p1.x as f32, p1.y as f32, p1.z as f32], 
                    normal: zero_n, 
                    color 
                });
                v.push(Vertex { 
                    position: [p2.x as f32, p2.y as f32, p2.z as f32], 
                    normal: zero_n, 
                    color 
                });
            }
        }

        self.path_count = v.len() as u32;
        if v.is_empty() {
            self.path_buffer = None;
            return;
        }

        self.path_buffer = Some(device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("path_buffer"),
                contents: bytemuck::cast_slice(&v),
                usage: wgpu::BufferUsages::VERTEX,
            },
        ));
    }

    pub fn upload_collisions(&mut self, device: &wgpu::Device, points: &[nalgebra::Point3<f32>]) {
        let mut v = Vec::new();
        let color = [1.0, 0.0, 0.0]; // Bright Red for collisions
        let zero_n = [0.0; 3];
        let size = 1.0f32; // Size of the collision X

        for p in points {
            // Draw a small 3D cross at each collision point
            v.push(Vertex { position: [p.x - size, p.y, p.z], normal: zero_n, color });
            v.push(Vertex { position: [p.x + size, p.y, p.z], normal: zero_n, color });
            v.push(Vertex { position: [p.x, p.y - size, p.z], normal: zero_n, color });
            v.push(Vertex { position: [p.x, p.y + size, p.z], normal: zero_n, color });
            v.push(Vertex { position: [p.x, p.y, p.z - size], normal: zero_n, color });
            v.push(Vertex { position: [p.x, p.y, p.z + size], normal: zero_n, color });
        }

        self.collision_count = v.len() as u32;
        if v.is_empty() {
            self.collision_buffer = None;
            return;
        }

        self.collision_buffer = Some(device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("collision_buffer"),
                contents: bytemuck::cast_slice(&v),
                usage: wgpu::BufferUsages::VERTEX,
            },
        ));
    }

    /// Render the scene to the offscreen texture.
    pub fn render(
        &mut self,
        render_state: &egui_wgpu::RenderState,
        camera: &OrbitCamera,
        width: u32,
        height: u32,
        sim_progress: f32,
        show_full_path: bool,
    ) {
        let device = &render_state.device;
        let queue = &render_state.queue;
        let w = width.max(1);
        let h = height.max(1);

        // Resize render targets if needed.
        if self.tex_size != [w, h] {
            let (ct, cv, dt, dv) =
                create_render_targets(device, self.target_format, w, h);
            self.color_tex = ct;
            self.color_view = cv;
            self.depth_tex = dt;
            self.depth_view = dv;
            self.tex_size = [w, h];

            render_state
                .renderer
                .write()
                .update_egui_texture_from_wgpu_texture(
                    device,
                    &self.color_view,
                    wgpu::FilterMode::Linear,
                    self.texture_id,
                );
        }
        // Update uniforms.
        let aspect = w as f32 / h as f32;
        let dir = [0.3, 0.5, 0.8];
        let light_dir = [dir[0], dir[1], dir[2], 0.0];

        // Camera position
        let cam_pos = camera.eye();
        let camera_pos = [cam_pos[0], cam_pos[1], cam_pos[2], 1.0];

        queue.write_buffer(
            &self.uniform_buffer,
            0,
            bytemuck::bytes_of(&Uniforms {
                view_proj: camera.view_proj(aspect),
                light_dir,
                camera_pos,
            }),
        );

        // Render pass.
        let mut encoder =
            device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: None });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("viewport_pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &self.color_view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.12,
                            g: 0.12,
                            b: 0.14,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                ..Default::default()
            });

            pass.set_bind_group(0, &self.bind_group, &[]);

            // Draw grid.
            pass.set_pipeline(&self.line_pipeline);
            pass.set_vertex_buffer(0, self.grid_buffer.slice(..));
            pass.draw(0..self.grid_count, 0..1);

            // Draw mesh.
            if let Some(ref buf) = self.mesh_buffer {
                pass.set_pipeline(&self.mesh_pipeline);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..self.mesh_count, 0..1);
            }

            // Draw stock wireframe.
            if let Some(ref buf) = self.stock_buffer {
                pass.set_pipeline(&self.line_pipeline);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..self.stock_count, 0..1);
            }

            // Draw tool wireframe.
            if let Some(ref buf) = self.tool_buffer {
                pass.set_pipeline(&self.line_pipeline);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..self.tool_count, 0..1);
            }

            // Draw voxel volume.
            if let (Some(ref vol), Some(ref bg)) = (&self.voxel_volume, &self.voxel_render_bg) {
                pass.set_pipeline(&vol.render_pipeline);
                pass.set_bind_group(0, bg, &[]);
                pass.set_bind_group(1, &self.bind_group, &[]);
                pass.draw(0..36, 0..1);

                // Restore standard bind group for subsequent draw calls (toolpath, etc)
                pass.set_bind_group(0, &self.bind_group, &[]);
            }

            // Draw path wireframe.
            if let Some(ref buf) = self.path_buffer {
                pass.set_pipeline(&self.line_pipeline);
                pass.set_vertex_buffer(0, buf.slice(..));
                
                let draw_count = if show_full_path {
                    self.path_count
                } else {
                    let c = (self.path_count as f32 * sim_progress) as u32;
                    c - (c % 2) // Snap to even number because it's a line list (2 verts per line)
                };
                
                if draw_count > 0 {
                    pass.draw(0..draw_count, 0..1);
                }
            }

            // Draw collision markers.
            if let Some(ref buf) = self.collision_buffer {
                pass.set_pipeline(&self.line_pipeline);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..self.collision_count, 0..1);
            }
        }

        queue.submit(std::iter::once(encoder.finish()));
    }
}

// ── Render target creation ──────────────────────────────────────────────────

fn create_render_targets(
    device: &wgpu::Device,
    format: wgpu::TextureFormat,
    w: u32,
    h: u32,
) -> (
    wgpu::Texture,
    wgpu::TextureView,
    wgpu::Texture,
    wgpu::TextureView,
) {
    let size = wgpu::Extent3d {
        width: w,
        height: h,
        depth_or_array_layers: 1,
    };

    let color = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("vp_color"),
        size,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    });

    let depth = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("vp_depth"),
        size,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Depth32Float,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });

    let cv = color.create_view(&Default::default());
    let dv = depth.create_view(&Default::default());
    (color, cv, depth, dv)
}

// ── Grid generation ─────────────────────────────────────────────────────────

fn build_grid(extent: f32, step: f32) -> Vec<Vertex> {
    let mut v = Vec::new();
    let n = (extent / step) as i32;
    let zero_n = [0.0f32; 3];

    let gray = [0.25, 0.25, 0.25];
    for i in -n..=n {
        let t = i as f32 * step;
        // Along X
        v.push(Vertex { position: [-extent, t, 0.0], normal: zero_n, color: gray });
        v.push(Vertex { position: [extent, t, 0.0], normal: zero_n, color: gray });
        // Along Y
        v.push(Vertex { position: [t, -extent, 0.0], normal: zero_n, color: gray });
        v.push(Vertex { position: [t, extent, 0.0], normal: zero_n, color: gray });
    }

    // Axis indicators
    let axis_len = extent * 0.5;
    // X — red
    v.push(Vertex { position: [0.0, 0.0, 0.0], normal: zero_n, color: [0.9, 0.2, 0.2] });
    v.push(Vertex { position: [axis_len, 0.0, 0.0], normal: zero_n, color: [0.9, 0.2, 0.2] });
    // Y — green
    v.push(Vertex { position: [0.0, 0.0, 0.0], normal: zero_n, color: [0.2, 0.9, 0.2] });
    v.push(Vertex { position: [0.0, axis_len, 0.0], normal: zero_n, color: [0.2, 0.9, 0.2] });
    // Z — blue
    v.push(Vertex { position: [0.0, 0.0, 0.0], normal: zero_n, color: [0.3, 0.3, 0.95] });
    v.push(Vertex { position: [0.0, 0.0, axis_len * 0.5], normal: zero_n, color: [0.3, 0.3, 0.95] });

    v
}

// ── Mesh conversion ─────────────────────────────────────────────────────────

fn mesh_to_vertices(mesh: &parry3d::shape::TriMesh) -> Vec<Vertex> {
    let verts = mesh.vertices();
    let indices = mesh.indices();
    let color = [0.55, 0.60, 0.70];
    let mut out = Vec::with_capacity(indices.len() * 3);

    for tri in indices {
        let p0 = verts[tri[0] as usize];
        let p1 = verts[tri[1] as usize];
        let p2 = verts[tri[2] as usize];

        let e1 = [p1.x - p0.x, p1.y - p0.y, p1.z - p0.z];
        let e2 = [p2.x - p0.x, p2.y - p0.y, p2.z - p0.z];
        let normal = normalize3(cross3(e1, e2));

        out.push(Vertex { position: [p0.x, p0.y, p0.z], normal, color });
        out.push(Vertex { position: [p1.x, p1.y, p1.z], normal, color });
        out.push(Vertex { position: [p2.x, p2.y, p2.z], normal, color });
    }
    out
}

// ── Math helpers (column-major 4x4 matrices, Z-up) ──────────────────────────

fn look_at(eye: [f32; 3], target: [f32; 3], up: [f32; 3]) -> [[f32; 4]; 4] {
    let f = normalize3(sub3(target, eye));
    let s = normalize3(cross3(f, up));
    let u = cross3(s, f);
    // Column-major for WGSL mat4x4
    [
        [s[0], u[0], -f[0], 0.0],
        [s[1], u[1], -f[1], 0.0],
        [s[2], u[2], -f[2], 0.0],
        [-dot3(s, eye), -dot3(u, eye), dot3(f, eye), 1.0],
    ]
}

fn perspective(fov_y: f32, aspect: f32, near: f32, far: f32) -> [[f32; 4]; 4] {
    let f = 1.0 / (fov_y * 0.5).tan();
    // wgpu clip Z: 0..1, right-handed
    [
        [f / aspect, 0.0, 0.0, 0.0],
        [0.0, f, 0.0, 0.0],
        [0.0, 0.0, far / (near - far), -1.0],
        [0.0, 0.0, near * far / (near - far), 0.0],
    ]
}

fn mat4_mul(a: [[f32; 4]; 4], b: [[f32; 4]; 4]) -> [[f32; 4]; 4] {
    let mut r = [[0.0f32; 4]; 4];
    for j in 0..4 {
        for i in 0..4 {
            for k in 0..4 {
                r[j][i] += a[k][i] * b[j][k];
            }
        }
    }
    r
}

fn cross3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn dot3(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn sub3(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn normalize3(v: [f32; 3]) -> [f32; 3] {
    let len = dot3(v, v).sqrt();
    if len < 1e-12 {
        [0.0, 0.0, 1.0]
    } else {
        [v[0] / len, v[1] / len, v[2] / len]
    }
}

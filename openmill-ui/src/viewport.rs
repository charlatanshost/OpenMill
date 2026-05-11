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

// Ghost / X-ray mesh: a translucent always-visible silhouette of the part,
// used in Simulation view so the user can see where the tool is relative to
// the finished part shape even when stock material is between camera and part.
@fragment
fn fs_ghost(in: VsOut) -> @location(0) vec4<f32> {
    let n = normalize(in.normal);
    let l = normalize(u.light_dir.xyz);
    // Front faces glow more, back faces fade — produces a clear silhouette.
    let facing = abs(dot(n, l)) * 0.5 + 0.5;
    let tint   = vec3(0.95, 0.25, 0.30);
    return vec4(tint * facing, 0.45);
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

    /// Build a world-space ray from a 2D point inside the viewport rect.
    /// `(rect_x, rect_y)` is the viewport size in pixels; `(px, py)` is the
    /// click position relative to the rect's top-left corner.
    /// Returns `(origin, direction)` with `direction` unit-length.
    pub fn ray_from_screen(
        &self,
        px: f32, py: f32,
        rect_w: f32, rect_h: f32,
    ) -> (nalgebra::Point3<f32>, nalgebra::Vector3<f32>) {
        let aspect = rect_w / rect_h.max(1.0);

        // Pull the same VP matrix as the GPU sees.
        let vp_arr = self.view_proj(aspect);
        let vp = nalgebra::Matrix4::from_columns(&[
            nalgebra::Vector4::new(vp_arr[0][0], vp_arr[0][1], vp_arr[0][2], vp_arr[0][3]),
            nalgebra::Vector4::new(vp_arr[1][0], vp_arr[1][1], vp_arr[1][2], vp_arr[1][3]),
            nalgebra::Vector4::new(vp_arr[2][0], vp_arr[2][1], vp_arr[2][2], vp_arr[2][3]),
            nalgebra::Vector4::new(vp_arr[3][0], vp_arr[3][1], vp_arr[3][2], vp_arr[3][3]),
        ]);
        let inv = vp.try_inverse().unwrap_or_else(nalgebra::Matrix4::identity);

        // Pixel → NDC. NDC y is flipped relative to screen y.
        let nx = (px / rect_w) * 2.0 - 1.0;
        let ny = -((py / rect_h) * 2.0 - 1.0);

        // Unproject near (z = 0 in NDC) and far (z = 1).
        let near_clip = nalgebra::Vector4::new(nx, ny, 0.0, 1.0);
        let far_clip  = nalgebra::Vector4::new(nx, ny, 1.0, 1.0);
        let near_w = inv * near_clip;
        let far_w  = inv * far_clip;
        let near = nalgebra::Point3::new(near_w.x / near_w.w, near_w.y / near_w.w, near_w.z / near_w.w);
        let far  = nalgebra::Point3::new(far_w.x  / far_w.w,  far_w.y  / far_w.w,  far_w.z  / far_w.w);
        let dir = (far - near).normalize();
        (near, dir)
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

    // ── Camera presets ────────────────────────────────────────────────────
    //
    // Standard machinist views. The azimuth/elevation values lock the
    // camera to the principal planes; distance and target are left alone
    // so the user keeps whatever framing they had after orbiting.

    /// Top-down view (+Z toward camera). The default "WCS top" most
    /// CAM software opens with.
    pub fn view_top(&mut self) {
        self.azimuth = -std::f32::consts::FRAC_PI_2;
        self.elevation = std::f32::consts::FRAC_PI_2 - 0.001;
    }
    /// Looking along +Y toward the part — front elevation.
    pub fn view_front(&mut self) {
        self.azimuth = -std::f32::consts::FRAC_PI_2;
        self.elevation = 0.0;
    }
    /// Looking along -X toward the part — right side.
    pub fn view_right(&mut self) {
        self.azimuth = 0.0;
        self.elevation = 0.0;
    }
    /// Three-quarter isometric — the same starting orientation as the
    /// default camera. Use this as "go back to a sensible angle".
    pub fn view_iso(&mut self) {
        self.azimuth = 0.8;
        self.elevation = 0.5;
    }
}

// ── Viewport renderer ───────────────────────────────────────────────────────

pub struct Viewport {
    mesh_pipeline: wgpu::RenderPipeline,
    /// Translucent always-on-top mesh pipeline used in Sim view to show the
    /// part silhouette through the stock — lets the user spot tool-mesh
    /// collisions even when the stock voxel still has material in front.
    ghost_mesh_pipeline: wgpu::RenderPipeline,
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

    /// Wireframe box representing the machine's linear travel envelope. Drawn
    /// in a dim red so it reads as a hard limit, not a normal scene element.
    envelope_buffer: Option<wgpu::Buffer>,
    envelope_count: u32,

    /// Triangle-list buffer for the currently picked face cluster (yellow
    /// highlight overlay). Always rendered on top.
    pick_buffer: Option<wgpu::Buffer>,
    pick_count: u32,

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
    /// GPU marching-cubes extractor. Lazily created on the first stock
    /// upload (it depends on the wgpu device). Renders a smooth
    /// iso-surface mesh of the carved stock alongside (or in place of)
    /// the voxel ray-march.
    mc: Option<crate::marching_cubes::MarchingCubes>,
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

        // Ghost-mesh pipeline: translucent X-ray silhouette of the part,
        // drawn in Sim view on top of the voxel stock so collisions between
        // the tool and the finished-part surface are visible even when stock
        // is still in the way. Always passes depth so it's never occluded.
        let ghost_depth_stencil = wgpu::DepthStencilState {
            format: wgpu::TextureFormat::Depth32Float,
            depth_write_enabled: false,
            depth_compare: wgpu::CompareFunction::Always,
            stencil: Default::default(),
            bias: Default::default(),
        };
        let ghost_mesh_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("ghost_mesh_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[vertex_layout()],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_ghost",
                targets: &[Some(wgpu::ColorTargetState {
                    format: target_format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: Some(ghost_depth_stencil),
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
            ghost_mesh_pipeline,
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
            envelope_buffer: None,
            envelope_count: 0,
            pick_buffer: None,
            pick_count: 0,
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
                stock_min:       [0.0; 4],
                stock_max:       [0.0; 4],
                tool_start:      [0.0; 4],
                tool_end:        [0.0; 4],
                tool_axis_start: [0.0, 0.0, 1.0, 0.0],
                tool_axis_end:   [0.0, 0.0, 1.0, 0.0],
                tool_params:     [0.0; 4],
                op_info:         [0.0; 4],
            },
            mc: None,
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
    /// Build a wireframe box for the machine's linear travel limits. Drawn
    /// in a dim red so it reads as a hard boundary distinct from the
    /// orange stock outline.
    pub fn upload_envelope(
        &mut self,
        device: &wgpu::Device,
        limits: &openmill_core::AxisLimits,
    ) {
        let (xn, xp) = (limits.x.0 as f32, limits.x.1 as f32);
        let (yn, yp) = (limits.y.0 as f32, limits.y.1 as f32);
        let (zn, zp) = (limits.z.0 as f32, limits.z.1 as f32);
        let color = [0.85, 0.25, 0.25]; // dim red
        let zero_n = [0.0; 3];
        let p = [
            [xn, yn, zn], [xp, yn, zn], [xp, yp, zn], [xn, yp, zn],
            [xn, yn, zp], [xp, yn, zp], [xp, yp, zp], [xn, yp, zp],
        ];
        let mut v: Vec<Vertex> = Vec::with_capacity(24);
        let mut line = |a: [f32; 3], b: [f32; 3]| {
            v.push(Vertex { position: a, normal: zero_n, color });
            v.push(Vertex { position: b, normal: zero_n, color });
        };
        // bottom rectangle
        line(p[0], p[1]); line(p[1], p[2]); line(p[2], p[3]); line(p[3], p[0]);
        // top rectangle
        line(p[4], p[5]); line(p[5], p[6]); line(p[6], p[7]); line(p[7], p[4]);
        // verticals
        line(p[0], p[4]); line(p[1], p[5]); line(p[2], p[6]); line(p[3], p[7]);

        self.envelope_count = v.len() as u32;
        self.envelope_buffer = Some(device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("envelope_buffer"),
                contents: bytemuck::cast_slice(&v),
                usage: wgpu::BufferUsages::VERTEX,
            },
        ));
    }

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
        // 256³ × 4 bytes = 64 MB. ~4× the surface detail vs 128³ and well
        // within every modern GPU's storage texture budget. The naive
        // full-volume compute dispatch still keeps up at this size for
        // typical stocks; if it ever doesn't, the next move is a
        // bounding-box-clipped dispatch.
        let res = [256, 256, 256];
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
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: wgpu::BindingResource::TextureView(&voxel_volume.sdf_view),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: wgpu::BindingResource::Sampler(&voxel_volume.sdf_sampler),
                },
            ],
        }));
        
        voxel_volume.reset(queue);

        // Initial uniform update
        self.voxel_uniforms = crate::voxel::VoxelUniforms {
            stock_min,
            stock_max,
            tool_start:      [0.0; 4],
            tool_end:        [0.0; 4],
            tool_axis_start: [0.0, 0.0, 1.0, 0.0],
            tool_axis_end:   [0.0, 0.0, 1.0, 0.0],
            tool_params:     [0.0; 4],
            op_info:         [0.0; 4],
        };
        self.voxel_volume = Some(voxel_volume);

        // The marching-cubes extractor is built lazily — see
        // `extract_iso_surface` — so model import only pays for it
        // when the user actually toggles Smooth on. If a stock was
        // previously bound, drop its bind group; the new voxel
        // texture will get re-bound on the next extraction request.
        if let Some(mc) = self.mc.as_mut() {
            mc.bind_group = None;
            mc.has_geometry = false;
        }
    }

    pub fn reset_voxel(&mut self, queue: &wgpu::Queue) {
        if let Some(ref vol) = self.voxel_volume {
            vol.reset(queue);
        }
        // After a stock reset the cached iso-surface no longer
        // describes the new voxel state.
        if let Some(mc) = self.mc.as_mut() {
            mc.has_geometry = false;
        }
    }

    /// Re-extract the iso-surface mesh from the current voxel state.
    /// Builds the MC pipeline + buffers on first call so unimported /
    /// non-Smooth sessions never allocate it. Cheap on 128³ (~few ms
    /// per dispatch); 256³ may run 15–25 ms on a typical desktop GPU.
    pub fn extract_iso_surface(&mut self, device: &wgpu::Device, queue: &wgpu::Queue) {
        let Some(vol) = self.voxel_volume.as_ref() else { return };
        if self.mc.is_none() {
            self.mc = Some(crate::marching_cubes::MarchingCubes::new(device));
        }
        let mc = self.mc.as_mut().expect("mc just initialised");
        if mc.bind_group.is_none() {
            mc.bind_voxel(device, &vol.view, &vol.uniform_buffer);
        }
        mc.extract(device, queue, vol.resolution);
    }

    #[allow(dead_code)]
    pub fn iso_surface_ready(&self) -> bool {
        self.mc.as_ref().map(|m| m.has_geometry).unwrap_or(false)
    }

    /// Replace the SDF texture sampled by Verify mode. The render bind
    /// group is rebuilt to point at the new texture view.
    pub fn upload_sdf(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        data: &[f32],
        resolution: [u32; 3],
    ) {
        let Some(vol) = self.voxel_volume.as_mut() else { return };
        vol.upload_sdf(device, queue, data, resolution);
        // Rebuild the render bind group so the SDF view in slot 2 points
        // at the new texture rather than the placeholder.
        self.voxel_render_bg = Some(device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("voxel_render_bg"),
            layout: &vol.render_bgl,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&vol.view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: vol.uniform_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: wgpu::BindingResource::TextureView(&vol.sdf_view),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: wgpu::BindingResource::Sampler(&vol.sdf_sampler),
                },
            ],
        }));
    }

    /// Stamp the verify-related fields of `op_info` into the voxel uniform
    /// buffer. Call once per frame from the app's update loop so the
    /// shader sees the user's current toggle state and tolerance choice.
    pub fn set_verify_uniforms(
        &mut self,
        queue: &wgpu::Queue,
        verify_mode: bool,
        tolerance_mm: f32,
    ) {
        // op_info: [op_id, verify_mode, tolerance_mm, reserved]
        // op_id (x) belongs to the most recent carve and is preserved.
        self.voxel_uniforms.op_info[1] = if verify_mode { 1.0 } else { 0.0 };
        self.voxel_uniforms.op_info[2] = tolerance_mm;
        if let Some(ref vol) = self.voxel_volume {
            queue.write_buffer(&vol.uniform_buffer, 0, bytemuck::cast_slice(&[self.voxel_uniforms]));
        }
    }

    pub fn carve_voxel(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        start: &nalgebra::Isometry3<f32>,
        end: &nalgebra::Isometry3<f32>,
        tool: &openmill_core::Tool,
        op_id: u32,
    ) {
        let Some(ref vol) = self.voxel_volume else { return };
        let Some(ref bg) = self.voxel_compute_bg else { return };

        // Tip positions
        let start_pos = start.translation.vector;
        let end_pos = end.translation.vector;

        // Tool axis at each pose: pose rotation applied to local +Z. The
        // toolpath builder sets each pose so that local +Z aligns with the
        // tool body (pointing from the tip toward the spindle).
        let axis_start = start.rotation * nalgebra::Vector3::z();
        let axis_end   = end.rotation   * nalgebra::Vector3::z();

        self.voxel_uniforms.tool_start = [start_pos.x, start_pos.y, start_pos.z, 1.0];
        self.voxel_uniforms.tool_end   = [end_pos.x,   end_pos.y,   end_pos.z,   1.0];
        self.voxel_uniforms.tool_axis_start = [axis_start.x, axis_start.y, axis_start.z, 0.0];
        self.voxel_uniforms.tool_axis_end   = [axis_end.x,   axis_end.y,   axis_end.z,   0.0];
        // Cutting length is the flute length — the shank above shouldn't
        // remove material in a normal cut (and modelling it as cutting would
        // gouge stock above the tool).
        let cutting_len = tool.shape.flute_length() as f32;
        self.voxel_uniforms.tool_params = [
            tool.shape.diameter() as f32 * 0.5,
            cutting_len.max(0.5),
            0.0,
            0.0,
        ];
        // Stamp this carve with the op's id so the surface picks up the
        // right palette entry. Stored as f32 — the shader rounds back to
        // u32 before use.
        self.voxel_uniforms.op_info = [op_id as f32, 0.0, 0.0, 0.0];

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

    /// Upload a **solid-shaded** tool + holder mesh for the current pose.
    ///
    /// Emits a triangle-list with per-vertex normals so the lit mesh
    /// pipeline (Lambertian via [`fs_lit`]) renders it like Fusion's
    /// rendered tool — flutes in cyan-ish steel, shank/holder in darker
    /// graphite. The cutter geometry is approximated per [`ToolShape`]
    /// (ball-end uses a UV-sphere hemisphere, drill uses a cone tip, etc.);
    /// every variant falls back to a flat-end cylinder if no specialised
    /// path matches.
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

        let mut v: Vec<Vertex> = Vec::new();
        let cutter_color  = [0.78, 0.86, 0.95]; // brushed steel / cobalt
        let shank_color   = [0.42, 0.44, 0.50]; // graphite shank
        let holder_color  = [0.30, 0.31, 0.36]; // collet / holder body

        build_tool_mesh(&mut v, tool, cutter_color, shank_color, holder_color);

        // Transform every vertex by the pose. Doing it CPU-side keeps the
        // mesh pipeline shader unchanged — no per-object model matrix.
        for vert in &mut v {
            let p = nalgebra::Point3::new(vert.position[0], vert.position[1], vert.position[2]);
            let n = nalgebra::Vector3::new(vert.normal[0], vert.normal[1], vert.normal[2]);
            let tp = pose * p;
            let tn = pose.rotation * n;
            vert.position = [tp.x, tp.y, tp.z];
            vert.normal = [tn.x, tn.y, tn.z];
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

    /// Upload toolpath geometry for line-strip rendering.
    ///
    /// Each segment is colored by its `MoveType`. When `stock_aabb` is
    /// supplied, cutting segments whose endpoints sit outside the stock
    /// envelope (above the top face or beyond the side faces by more than
    /// the tool radius) are flagged as **air cuts** and dimmed — the tool
    /// is moving but not engaged with material. This is purely geometric
    /// (no voxel sampling) so prior material removal doesn't influence the
    /// classification, but it's enough to catch the common case of
    /// over-shooting raster patterns and dwell-above-stock segments.
    pub fn upload_toolpath(
        &mut self,
        device: &wgpu::Device,
        paths: &[openmill_core::Toolpath],
        stock_aabb: Option<&parry3d::bounding_volume::Aabb>,
        tool_radius_mm: f32,
    ) {
        let mut v = Vec::new();
        let zero_n = [0.0; 3];
        let r = tool_radius_mm.max(0.0);

        // Test whether a tool-tip point lies within (or just outside) the
        // stock envelope. Tip above the stock top counts as definitely
        // not cutting; tip outside the XY footprint by more than the tool
        // radius likewise.
        let in_stock = |p: &nalgebra::Point3<f64>| -> bool {
            let Some(aabb) = stock_aabb else { return true };
            let px = p.x as f32;
            let py = p.y as f32;
            let pz = p.z as f32;
            pz <= aabb.maxs.z + 1e-3
                && px >= aabb.mins.x - r
                && px <= aabb.maxs.x + r
                && py >= aabb.mins.y - r
                && py <= aabb.maxs.y + r
        };

        for tp in paths {
            for i in 0..tp.points.len().saturating_sub(1) {
                let p1 = tp.points[i].position;
                let p2 = tp.points[i + 1].position;
                let mtype = tp.points[i + 1].move_type;
                let is_cut = matches!(
                    mtype,
                    openmill_core::toolpath::MoveType::Linear
                        | openmill_core::toolpath::MoveType::LeadIn
                        | openmill_core::toolpath::MoveType::LeadOut
                );
                let air_cut = is_cut && !in_stock(&p1) && !in_stock(&p2);

                // Base color by move type, then desaturate when the
                // segment is "air cutting" so the user instantly sees
                // wasted motion.
                let mut color = match mtype {
                    openmill_core::toolpath::MoveType::Rapid => [1.0, 0.2, 0.2],
                    openmill_core::toolpath::MoveType::Linear => [0.2, 0.8, 1.0],
                    openmill_core::toolpath::MoveType::LeadIn
                    | openmill_core::toolpath::MoveType::LeadOut => [0.2, 1.0, 0.2],
                    openmill_core::toolpath::MoveType::Retract => [1.0, 1.0, 0.2],
                };
                if air_cut {
                    for c in &mut color { *c *= 0.35; }
                }

                v.push(Vertex {
                    position: [p1.x as f32, p1.y as f32, p1.z as f32],
                    normal: zero_n,
                    color,
                });
                v.push(Vertex {
                    position: [p2.x as f32, p2.y as f32, p2.z as f32],
                    normal: zero_n,
                    color,
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

    /// Upload triangles for the currently picked face cluster. The triangles
    /// are rendered as a yellow opaque overlay on top of everything else so
    /// the user can see exactly which faces they've selected. Pass an empty
    /// slice (or call `clear_pick`) to remove the highlight.
    pub fn upload_pick_triangles(
        &mut self,
        device: &wgpu::Device,
        triangles: &[[nalgebra::Point3<f32>; 3]],
    ) {
        let mut v = Vec::with_capacity(triangles.len() * 3);
        let color = [1.0, 0.85, 0.1]; // bright amber
        for tri in triangles {
            let p0 = tri[0]; let p1 = tri[1]; let p2 = tri[2];
            let n = (p1 - p0).cross(&(p2 - p0));
            let n_unit = if n.norm() > 1e-8 { n.normalize() } else { nalgebra::Vector3::z() };
            let normal = [n_unit.x, n_unit.y, n_unit.z];
            for p in [p0, p1, p2] {
                v.push(Vertex { position: [p.x, p.y, p.z], normal, color });
            }
        }
        self.pick_count = v.len() as u32;
        if v.is_empty() {
            self.pick_buffer = None;
            return;
        }
        self.pick_buffer = Some(device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("pick_buffer"),
                contents: bytemuck::cast_slice(&v),
                usage: wgpu::BufferUsages::VERTEX,
            },
        ));
    }

    pub fn clear_pick(&mut self) {
        self.pick_buffer = None;
        self.pick_count = 0;
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
    ///
    /// `show_mesh` draws the original part mesh (Plan view). `show_voxel`
    /// draws the carved stock volume (Simulation view). Stock wireframe,
    /// toolpath, and tool always render.
    pub fn render(
        &mut self,
        render_state: &egui_wgpu::RenderState,
        camera: &OrbitCamera,
        width: u32,
        height: u32,
        sim_progress: f32,
        show_full_path: bool,
        show_mesh: bool,
        show_voxel: bool,
        show_envelope: bool,
        show_iso_surface: bool,
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

            // Draw solid mesh — Plan view only.
            if show_mesh {
                if let Some(ref buf) = self.mesh_buffer {
                    pass.set_pipeline(&self.mesh_pipeline);
                    pass.set_vertex_buffer(0, buf.slice(..));
                    pass.draw(0..self.mesh_count, 0..1);
                }
            }

            // Draw stock wireframe.
            if let Some(ref buf) = self.stock_buffer {
                pass.set_pipeline(&self.line_pipeline);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..self.stock_count, 0..1);
            }

            // Draw machine travel envelope when requested.
            if show_envelope {
                if let Some(ref buf) = self.envelope_buffer {
                    pass.set_pipeline(&self.line_pipeline);
                    pass.set_vertex_buffer(0, buf.slice(..));
                    pass.draw(0..self.envelope_count, 0..1);
                }
            }

            // Draw solid-shaded tool + holder (triangle list, lit).
            if let Some(ref buf) = self.tool_buffer {
                pass.set_pipeline(&self.mesh_pipeline);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..self.tool_count, 0..1);
            }

            // Draw the picked-face highlight on top of the mesh / stock so
            // the user always sees which face they've selected.
            if let Some(ref buf) = self.pick_buffer {
                pass.set_pipeline(&self.mesh_pipeline);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..self.pick_count, 0..1);
            }

            // Draw simulated stock — Simulation view only. We pick one
            // of two paths:
            //   - `show_iso_surface`: marching-cubes mesh through the
            //     lit pipeline. Pro-CAM look (Fusion / FreeCAD).
            //   - else: ray-march the voxel volume. Cheaper to update
            //     during play and works with Verify mode's SDF heatmap.
            if show_voxel {
                let drew_iso = if show_iso_surface {
                    if let Some(mc) = self.mc.as_ref() {
                        if mc.has_geometry {
                            pass.set_pipeline(&self.mesh_pipeline);
                            pass.set_bind_group(0, &self.bind_group, &[]);
                            mc.draw(&mut pass);
                            true
                        } else {
                            false
                        }
                    } else {
                        false
                    }
                } else {
                    false
                };
                if !drew_iso {
                    if let (Some(ref vol), Some(ref bg)) = (&self.voxel_volume, &self.voxel_render_bg) {
                        pass.set_pipeline(&vol.render_pipeline);
                        pass.set_bind_group(0, bg, &[]);
                        pass.set_bind_group(1, &self.bind_group, &[]);
                        pass.draw(0..36, 0..1);
                        // Restore standard bind group for subsequent draw calls.
                        pass.set_bind_group(0, &self.bind_group, &[]);
                    }
                }

                // Ghost-mesh overlay: drawn AFTER the voxel so the part
                // silhouette shows through the stock. depth=Always means it's
                // never occluded; alpha blending keeps the underlying stock
                // visible. The user can see the tool gouging into the part
                // even when stock material is still in front.
                if let Some(ref buf) = self.mesh_buffer {
                    pass.set_pipeline(&self.ghost_mesh_pipeline);
                    pass.set_vertex_buffer(0, buf.slice(..));
                    pass.draw(0..self.mesh_count, 0..1);
                }
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

// ── Solid tool / holder mesh builder ────────────────────────────────────────
//
// Emits a triangle list (CCW from outside) with per-vertex normals so the
// existing lit mesh pipeline shades the tool like a rendered Fusion tool.
// The whole mesh sits in **local tool space**: tip at the origin, +Z along
// the body toward the shank. Callers transform every vertex by the active
// tool pose before uploading.

const TOOL_SEGMENTS: usize = 32;

fn build_tool_mesh(
    v: &mut Vec<Vertex>,
    tool: &openmill_core::Tool,
    cutter_color: [f32; 3],
    shank_color: [f32; 3],
    holder_color: [f32; 3],
) {
    use openmill_core::ToolShape::*;
    let fl = tool.shape.flute_length() as f32;
    let ol = tool.overall_length.max(tool.shape.flute_length()) as f32;
    let shank_r = (tool.shank_diameter * 0.5).max(0.1) as f32;
    let cutter_r = (tool.shape.diameter() * 0.5).max(0.1) as f32;

    // ── Cutter body ─────────────────────────────────────────────────────
    match &tool.shape {
        FlatEnd { .. } => {
            cylinder_side(v, 0.0, fl, cutter_r, cutter_r, cutter_color);
            disc_cap(v, 0.0, cutter_r, [0.0, 0.0, -1.0], cutter_color);
        }
        BallEnd { .. } => {
            let r = cutter_r;
            cylinder_side(v, r, fl, r, r, cutter_color);
            hemisphere(v, r, r, cutter_color);
        }
        BullNose { corner_radius, .. } => {
            let cr = (*corner_radius as f32).min(cutter_r).max(0.0);
            cylinder_side(v, cr, fl, cutter_r, cutter_r, cutter_color);
            torus_quarter(v, cr, cutter_r, cutter_color);
        }
        Drill { point_angle, .. } => {
            let half = (*point_angle as f32 * 0.5).to_radians();
            let tip_h = (cutter_r / half.tan().max(0.05)).min(fl);
            cylinder_side(v, tip_h, fl, cutter_r, cutter_r, cutter_color);
            cone_tip(v, 0.0, tip_h, cutter_r, cutter_color);
        }
        ChamferMill { tip_diameter, diameter, .. } => {
            let r_tip = (*tip_diameter as f32 * 0.5).max(0.05);
            let r_maj = (*diameter as f32 * 0.5).max(r_tip);
            // Tapered cone from tip up to fl-height.
            cone_frustum_side(v, 0.0, fl, r_tip, r_maj, cutter_color);
            disc_cap(v, 0.0, r_tip, [0.0, 0.0, -1.0], cutter_color);
        }
        TaperedMill { tip_diameter, .. } => {
            let r_tip = (*tip_diameter as f32 * 0.5).max(0.05);
            cone_frustum_side(v, 0.0, fl, r_tip, cutter_r, cutter_color);
            disc_cap(v, 0.0, r_tip, [0.0, 0.0, -1.0], cutter_color);
        }
        Lollipop { diameter, neck_diameter, .. } => {
            let r_head = (*diameter as f32 * 0.5).max(0.1);
            let r_neck = (*neck_diameter as f32 * 0.5).max(0.05);
            // Sphere head centered at z = r_head, neck above.
            sphere(v, r_head, r_head, cutter_color);
            cylinder_side(v, r_head, fl, r_neck, r_neck, shank_color);
        }
        _ => {
            // Generic fallback: flat-end cylinder for dovetail / thread mill.
            cylinder_side(v, 0.0, fl, cutter_r, cutter_r, cutter_color);
            disc_cap(v, 0.0, cutter_r, [0.0, 0.0, -1.0], cutter_color);
        }
    }

    // ── Shank above the flutes ──────────────────────────────────────────
    if ol > fl + 0.1 {
        cylinder_side(v, fl, ol, shank_r, shank_r, shank_color);
        disc_cap(v, ol, shank_r, [0.0, 0.0, 1.0], shank_color);
    }

    // ── Optional holder profile (axisymmetric (z, r) sweep). Sits above
    // the shank end so the user sees collision danger up to the spindle.
    if let Some(holder) = &tool.holder {
        holder_swept(v, &holder.profile, ol, holder_color);
    }
}

/// Side wall of a cylinder/frustum from `z0` to `z1`, radius `r0` at `z0`
/// and `r1` at `z1`. Normals point radially outward.
fn cylinder_side(v: &mut Vec<Vertex>, z0: f32, z1: f32, r0: f32, r1: f32, color: [f32; 3]) {
    if z1 <= z0 + 1e-6 { return; }
    let n_seg = TOOL_SEGMENTS;
    for i in 0..n_seg {
        let a0 = (i     as f32 / n_seg as f32) * std::f32::consts::TAU;
        let a1 = ((i+1) as f32 / n_seg as f32) * std::f32::consts::TAU;
        let (c0, s0) = (a0.cos(), a0.sin());
        let (c1, s1) = (a1.cos(), a1.sin());
        let p00 = [r0 * c0, r0 * s0, z0];
        let p10 = [r0 * c1, r0 * s1, z0];
        let p01 = [r1 * c0, r1 * s0, z1];
        let p11 = [r1 * c1, r1 * s1, z1];
        // Outward radial normals; for tapered frustums add a vertical
        // component so the lighting reads the slope. Keep it simple — use
        // the midpoint surface normal for the whole quad.
        let dz = z1 - z0;
        let dr = r1 - r0;
        let slope_len = (dz * dz + dr * dr).sqrt().max(1e-6);
        let nz = -dr / slope_len;
        let nrad = dz / slope_len;
        let n0 = normalize3([c0 * nrad, s0 * nrad, nz]);
        let n1 = normalize3([c1 * nrad, s1 * nrad, nz]);
        push_tri(v, p00, p10, p11, n0, n1, n1, color);
        push_tri(v, p00, p11, p01, n0, n1, n0, color);
    }
}

fn cone_frustum_side(v: &mut Vec<Vertex>, z0: f32, z1: f32, r0: f32, r1: f32, color: [f32; 3]) {
    cylinder_side(v, z0, z1, r0, r1, color);
}

/// Solid cone tip from `z_tip` (point) up to `z_base` (radius `r_base`).
fn cone_tip(v: &mut Vec<Vertex>, z_tip: f32, z_base: f32, r_base: f32, color: [f32; 3]) {
    let n_seg = TOOL_SEGMENTS;
    let tip = [0.0, 0.0, z_tip];
    let dz = z_base - z_tip;
    let slope = (dz * dz + r_base * r_base).sqrt().max(1e-6);
    for i in 0..n_seg {
        let a0 = (i     as f32 / n_seg as f32) * std::f32::consts::TAU;
        let a1 = ((i+1) as f32 / n_seg as f32) * std::f32::consts::TAU;
        let (c0, s0) = (a0.cos(), a0.sin());
        let (c1, s1) = (a1.cos(), a1.sin());
        let p0 = [r_base * c0, r_base * s0, z_base];
        let p1 = [r_base * c1, r_base * s1, z_base];
        let n_tip = normalize3([(c0 + c1) * 0.5 * dz / slope, (s0 + s1) * 0.5 * dz / slope, r_base / slope]);
        let n0   = normalize3([c0 * dz / slope, s0 * dz / slope, r_base / slope]);
        let n1   = normalize3([c1 * dz / slope, s1 * dz / slope, r_base / slope]);
        push_tri(v, tip, p0, p1, n_tip, n0, n1, color);
    }
}

/// Tip-down hemisphere of radius `r` centred at `z_center`. Used for
/// ball-end cutters; bottom half (z ≤ z_center) faces outward / downward.
fn hemisphere(v: &mut Vec<Vertex>, z_center: f32, r: f32, color: [f32; 3]) {
    let n_seg = TOOL_SEGMENTS;
    let n_lat = TOOL_SEGMENTS / 2;
    for j in 0..n_lat {
        let t0 = j as f32 / n_lat as f32; // 0..1 from bottom to equator
        let t1 = (j + 1) as f32 / n_lat as f32;
        // Latitude angle measured from the south pole (tip) upward.
        let p0 = (t0 * std::f32::consts::FRAC_PI_2).sin();
        let p1 = (t1 * std::f32::consts::FRAC_PI_2).sin();
        let z0 = z_center - r * (1.0 - t0 * std::f32::consts::FRAC_PI_2).cos().abs(); // simpler form below
        // Recompute using spherical lat angle for accuracy:
        let lat0 = t0 * std::f32::consts::FRAC_PI_2 + std::f32::consts::FRAC_PI_2; // pi/2..pi
        let lat1 = t1 * std::f32::consts::FRAC_PI_2 + std::f32::consts::FRAC_PI_2;
        let _ = z0; let _ = p0; let _ = p1;
        let r0 = r * lat0.sin();
        let r1 = r * lat1.sin();
        let z0 = z_center + r * lat0.cos(); // cos is negative in this range → below center
        let z1 = z_center + r * lat1.cos();
        for i in 0..n_seg {
            let a0 = (i     as f32 / n_seg as f32) * std::f32::consts::TAU;
            let a1 = ((i+1) as f32 / n_seg as f32) * std::f32::consts::TAU;
            let (c0, s0) = (a0.cos(), a0.sin());
            let (c1, s1) = (a1.cos(), a1.sin());
            let p00 = [r0 * c0, r0 * s0, z0];
            let p10 = [r0 * c1, r0 * s1, z0];
            let p01 = [r1 * c0, r1 * s0, z1];
            let p11 = [r1 * c1, r1 * s1, z1];
            let n00 = normalize3([p00[0], p00[1], p00[2] - z_center]);
            let n10 = normalize3([p10[0], p10[1], p10[2] - z_center]);
            let n01 = normalize3([p01[0], p01[1], p01[2] - z_center]);
            let n11 = normalize3([p11[0], p11[1], p11[2] - z_center]);
            push_tri(v, p00, p10, p11, n00, n10, n11, color);
            push_tri(v, p00, p11, p01, n00, n11, n01, color);
        }
    }
}

/// Full sphere of radius `r` centred at `z_center` — used for lollipop heads.
fn sphere(v: &mut Vec<Vertex>, z_center: f32, r: f32, color: [f32; 3]) {
    let n_seg = TOOL_SEGMENTS;
    let n_lat = TOOL_SEGMENTS / 2;
    for j in 0..n_lat {
        let lat0 = (j as f32 / n_lat as f32) * std::f32::consts::PI;
        let lat1 = ((j + 1) as f32 / n_lat as f32) * std::f32::consts::PI;
        let r0 = r * lat0.sin();
        let r1 = r * lat1.sin();
        let z0 = z_center + r * lat0.cos();
        let z1 = z_center + r * lat1.cos();
        for i in 0..n_seg {
            let a0 = (i     as f32 / n_seg as f32) * std::f32::consts::TAU;
            let a1 = ((i+1) as f32 / n_seg as f32) * std::f32::consts::TAU;
            let (c0, s0) = (a0.cos(), a0.sin());
            let (c1, s1) = (a1.cos(), a1.sin());
            let p00 = [r0 * c0, r0 * s0, z0];
            let p10 = [r0 * c1, r0 * s1, z0];
            let p01 = [r1 * c0, r1 * s0, z1];
            let p11 = [r1 * c1, r1 * s1, z1];
            let n00 = normalize3([p00[0], p00[1], p00[2] - z_center]);
            let n10 = normalize3([p10[0], p10[1], p10[2] - z_center]);
            let n01 = normalize3([p01[0], p01[1], p01[2] - z_center]);
            let n11 = normalize3([p11[0], p11[1], p11[2] - z_center]);
            push_tri(v, p00, p10, p11, n00, n10, n11, color);
            push_tri(v, p00, p11, p01, n00, n11, n01, color);
        }
    }
}

/// Quarter-torus filling the corner radius on a bull-nose cutter. Centre
/// of the tube ring lies on a circle of radius `cutter_r - cr` at z = cr.
fn torus_quarter(v: &mut Vec<Vertex>, cr: f32, cutter_r: f32, color: [f32; 3]) {
    if cr <= 1e-6 { return; }
    let n_seg = TOOL_SEGMENTS;
    let n_tube = (TOOL_SEGMENTS / 4).max(4);
    let ring_r = (cutter_r - cr).max(0.0);
    for j in 0..n_tube {
        // Tube angle: 0 = bottom of corner (z=0, r=cutter_r-cr... no, r=cutter_r when below center)
        // We want sweep from theta=-pi/2 (pointing -Z, tube point at z=0)
        // up to theta=0 (pointing +X, tube point at z=cr).
        let t0 = j     as f32 / n_tube as f32;
        let t1 = (j+1) as f32 / n_tube as f32;
        let th0 = -std::f32::consts::FRAC_PI_2 + t0 * std::f32::consts::FRAC_PI_2;
        let th1 = -std::f32::consts::FRAC_PI_2 + t1 * std::f32::consts::FRAC_PI_2;
        let cr0_z = cr + cr * th0.sin();
        let cr1_z = cr + cr * th1.sin();
        let cr0_r = ring_r + cr * th0.cos();
        let cr1_r = ring_r + cr * th1.cos();
        for i in 0..n_seg {
            let a0 = (i     as f32 / n_seg as f32) * std::f32::consts::TAU;
            let a1 = ((i+1) as f32 / n_seg as f32) * std::f32::consts::TAU;
            let (c0, s0) = (a0.cos(), a0.sin());
            let (c1, s1) = (a1.cos(), a1.sin());
            let p00 = [cr0_r * c0, cr0_r * s0, cr0_z];
            let p10 = [cr0_r * c1, cr0_r * s1, cr0_z];
            let p01 = [cr1_r * c0, cr1_r * s0, cr1_z];
            let p11 = [cr1_r * c1, cr1_r * s1, cr1_z];
            // Outward normal: from ring centre at (ring_r*c, ring_r*s, cr)
            let ring_c0 = [ring_r * c0, ring_r * s0, cr];
            let ring_c1 = [ring_r * c1, ring_r * s1, cr];
            let n00 = normalize3(sub3(p00, ring_c0));
            let n10 = normalize3(sub3(p10, ring_c1));
            let n01 = normalize3(sub3(p01, ring_c0));
            let n11 = normalize3(sub3(p11, ring_c1));
            push_tri(v, p00, p10, p11, n00, n10, n11, color);
            push_tri(v, p00, p11, p01, n00, n11, n01, color);
        }
    }
}

/// Flat disc cap at height `z`, radius `r`. `n_dir` is `[0,0,-1]` for the
/// bottom face (cutter tip) or `[0,0,1]` for the top of the shank.
fn disc_cap(v: &mut Vec<Vertex>, z: f32, r: f32, n_dir: [f32; 3], color: [f32; 3]) {
    if r <= 1e-6 { return; }
    let n_seg = TOOL_SEGMENTS;
    let centre = [0.0, 0.0, z];
    let outward = n_dir[2];
    for i in 0..n_seg {
        let a0 = (i     as f32 / n_seg as f32) * std::f32::consts::TAU;
        let a1 = ((i+1) as f32 / n_seg as f32) * std::f32::consts::TAU;
        let p0 = [r * a0.cos(), r * a0.sin(), z];
        let p1 = [r * a1.cos(), r * a1.sin(), z];
        // Wind the triangle so the surface normal matches `n_dir`.
        if outward > 0.0 {
            push_tri(v, centre, p0, p1, n_dir, n_dir, n_dir, color);
        } else {
            push_tri(v, centre, p1, p0, n_dir, n_dir, n_dir, color);
        }
    }
}

/// Sweep an axisymmetric `(z, r)` profile around +Z to produce the holder
/// silhouette. The profile is in **holder coordinates** (z=0 at the
/// spindle-nose face). `z_offset` is where the spindle-nose face sits in
/// tool-local coords — typically the shank top.
fn holder_swept(v: &mut Vec<Vertex>, profile: &[(f64, f64)], z_offset: f32, color: [f32; 3]) {
    if profile.len() < 2 { return; }
    for k in 0..profile.len() - 1 {
        let (z0, r0) = (profile[k].0 as f32 + z_offset, profile[k].1 as f32);
        let (z1, r1) = (profile[k+1].0 as f32 + z_offset, profile[k+1].1 as f32);
        if r0.max(r1) < 0.01 { continue; }
        cylinder_side(v, z0, z1, r0, r1, color);
    }
}

fn push_tri(
    v: &mut Vec<Vertex>,
    p0: [f32; 3], p1: [f32; 3], p2: [f32; 3],
    n0: [f32; 3], n1: [f32; 3], n2: [f32; 3],
    color: [f32; 3],
) {
    v.push(Vertex { position: p0, normal: n0, color });
    v.push(Vertex { position: p1, normal: n1, color });
    v.push(Vertex { position: p2, normal: n2, color });
}

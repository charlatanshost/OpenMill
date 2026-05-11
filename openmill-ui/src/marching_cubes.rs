//! GPU marching-cubes iso-surface extraction over the voxel volume.
//!
//! Replaces (or augments) the ray-marched volume render with a smooth
//! triangle mesh — the same look pro CAM software (Fusion, Mastercam,
//! PowerMill) gives to its simulated stock surface.
//!
//! Pipeline:
//!   1. A compute shader visits every `(Nx-1) × (Ny-1) × (Nz-1)` cell of
//!      the voxel grid. Each cell looks at its 8 corners, computes a
//!      marching-cubes case index, and looks up the canonical Bourke
//!      triangle list. Vertex positions are interpolated along cube
//!      edges; per-vertex normals come from the voxel-field gradient at
//!      the vertex position so shading reads as smooth, not faceted.
//!   2. Vertices are appended to a single storage buffer using an
//!      atomic counter. The counter's value is then copied into the
//!      `vertex_count` slot of an indirect-draw command buffer.
//!   3. The render pass issues `draw_indirect` against that buffer with
//!      the regular lit mesh pipeline.
//!
//! Memory: the vertex buffer is sized once at construction for the
//! worst case the caller expects. For 128³ voxels the realistic surface
//! triangle count is well under 1 M tris (3 M vertices), so allocating
//! 64 MB (~1.8 M tris × 36 bytes) is comfortable headroom on every
//! desktop GPU. If a stock blows past that the counter saturates and
//! the surface gets clipped — better than panicking on an OOM.

use eframe::wgpu;
use eframe::wgpu::util::DeviceExt;

/// Buffer capacity in *vertices*. Each vertex is 9 floats × 4 bytes =
/// 36 bytes, so this gives ≈ 16 MB. ~150 k triangles is enough surface
/// detail for typical hobby parts at 128³–256³ voxel resolution;
/// beyond this the compute shader stops appending and the surface
/// gets clipped (better than panicking on an OOM).
const MAX_VERTICES: u32 = 450_000;
const FLOATS_PER_VERTEX: u32 = 9;

pub struct MarchingCubes {
    pub compute_pipeline: wgpu::ComputePipeline,
    pub vertex_buffer: wgpu::Buffer,
    /// `atomic<u32>` allocator. The value at offset 0 is the running
    /// vertex count; cleared to 0 before each dispatch.
    pub count_buffer: wgpu::Buffer,
    /// Indirect-draw arguments. Layout: `[vertex_count, instance_count,
    /// first_vertex, first_instance]` as four `u32` words. We rewrite
    /// `vertex_count` from `count_buffer` after each extract; the other
    /// three are constant.
    pub indirect_buffer: wgpu::Buffer,
    pub bind_group_layout: wgpu::BindGroupLayout,
    /// Built lazily by [`bind_voxel`] once the caller knows which voxel
    /// texture + uniforms the MC shader should sample.
    pub bind_group: Option<wgpu::BindGroup>,
    /// True once at least one extraction has run. The caller uses this
    /// to gate `draw_indirect` calls — drawing against an unfilled
    /// indirect buffer on some backends is undefined behaviour.
    pub has_geometry: bool,
}

impl MarchingCubes {
    pub fn new(device: &wgpu::Device) -> Self {
        // ── Vertex buffer (storage + vertex usage) ─────────────────
        let vertex_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("mc_vertex_buffer"),
            size: (MAX_VERTICES as u64) * (FLOATS_PER_VERTEX as u64) * 4,
            usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::VERTEX,
            mapped_at_creation: false,
        });

        // ── Atomic counter (cleared before each dispatch) ─────────
        let count_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mc_count_buffer"),
            contents: bytemuck::cast_slice(&[0u32]),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_SRC
                | wgpu::BufferUsages::COPY_DST,
        });

        // ── Indirect-draw arguments: [count, 1, 0, 0] ──────────────
        let indirect_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("mc_indirect_buffer"),
            contents: bytemuck::cast_slice(&[0u32, 1u32, 0u32, 0u32]),
            usage: wgpu::BufferUsages::INDIRECT | wgpu::BufferUsages::COPY_DST,
        });

        // ── Bind group layout ──────────────────────────────────────
        let bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("mc_bgl"),
                entries: &[
                    // 0: voxel texture (read-only u32)
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
                    // 1: voxel uniforms (for stock_min/stock_max → world-space
                    //    vertex positions)
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
                    // 2: output vertex buffer (storage, read+write)
                    wgpu::BindGroupLayoutEntry {
                        binding: 2,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                    // 3: atomic counter (storage)
                    wgpu::BindGroupLayoutEntry {
                        binding: 3,
                        visibility: wgpu::ShaderStages::COMPUTE,
                        ty: wgpu::BindingType::Buffer {
                            ty: wgpu::BufferBindingType::Storage { read_only: false },
                            has_dynamic_offset: false,
                            min_binding_size: None,
                        },
                        count: None,
                    },
                ],
            });

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("mc_compute_shader"),
            source: wgpu::ShaderSource::Wgsl(build_shader_source().into()),
        });

        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("mc_compute_layout"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });

        let compute_pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
            label: Some("mc_compute_pipeline"),
            layout: Some(&layout),
            module: &shader,
            entry_point: "main",
            compilation_options: Default::default(),
            cache: None,
        });

        Self {
            compute_pipeline,
            vertex_buffer,
            count_buffer,
            indirect_buffer,
            bind_group_layout,
            bind_group: None,
            has_geometry: false,
        }
    }

    /// Build (or rebuild) the bind group against a specific voxel
    /// texture and uniform buffer. Call this whenever the voxel volume
    /// is recreated (new stock).
    pub fn bind_voxel(
        &mut self,
        device: &wgpu::Device,
        voxel_view: &wgpu::TextureView,
        voxel_uniforms: &wgpu::Buffer,
    ) {
        self.bind_group = Some(device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("mc_bg"),
            layout: &self.bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(voxel_view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: voxel_uniforms.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: self.vertex_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: self.count_buffer.as_entire_binding(),
                },
            ],
        }));
        self.has_geometry = false;
    }

    /// Dispatch one extraction pass over the voxel grid. Caller passes
    /// the current voxel resolution (the same `resolution` used to
    /// create the voxel volume). Safe to call every frame the voxels
    /// have changed — at 128³ voxels the dispatch is in the
    /// single-millisecond range on a typical desktop GPU.
    pub fn extract(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        resolution: [u32; 3],
    ) {
        let Some(ref bind_group) = self.bind_group else { return };

        // Reset the atomic counter to 0 before dispatching.
        queue.write_buffer(&self.count_buffer, 0, bytemuck::cast_slice(&[0u32]));

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("mc_extract_encoder"),
        });
        {
            let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("mc_extract_pass"),
                timestamp_writes: None,
            });
            pass.set_pipeline(&self.compute_pipeline);
            pass.set_bind_group(0, bind_group, &[]);
            // One thread per cell. We process (Nx-1)(Ny-1)(Nz-1) cells.
            // Workgroup size 4×4×4 → divide by 4 and round up.
            let cx = (resolution[0].saturating_sub(1) + 3) / 4;
            let cy = (resolution[1].saturating_sub(1) + 3) / 4;
            let cz = (resolution[2].saturating_sub(1) + 3) / 4;
            pass.dispatch_workgroups(cx, cy, cz);
        }
        // Copy the counter into `indirect_buffer.vertex_count` so the
        // upcoming `draw_indirect` picks up the count produced this
        // dispatch. The other three indirect fields (instance_count = 1,
        // first_vertex = 0, first_instance = 0) were written once at
        // buffer init and don't change.
        encoder.copy_buffer_to_buffer(&self.count_buffer, 0, &self.indirect_buffer, 0, 4);

        queue.submit(std::iter::once(encoder.finish()));
        self.has_geometry = true;
    }

    /// Issue an indirect draw using the extracted vertex buffer. The
    /// caller must have already set up the lit-mesh pipeline + view
    /// bind group on the render pass.
    pub fn draw<'a>(&'a self, pass: &mut wgpu::RenderPass<'a>) {
        if !self.has_geometry {
            return;
        }
        pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
        pass.draw_indirect(&self.indirect_buffer, 0);
    }
}

// ── Compute shader (with embedded MC tables) ────────────────────────────────

const COMPUTE_SHADER: &str = r#"
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

// The output buffer is a flat `array<f32>` — nine consecutive floats
// per vertex (`position.xyz`, `normal.xyz`, `color.xyz`) — so the data
// matches the lit-mesh pipeline's 36-byte tightly-packed vertex layout
// exactly. A `struct` of three `vec3<f32>`s would compile but storage
// buffers force 16-byte alignment on each `vec3`, blowing the stride to
// 48 bytes and mis-aligning the vertex attributes.

@group(0) @binding(0) var voxels: texture_3d<u32>;
@group(0) @binding(1) var<uniform> u: Uniforms;
@group(0) @binding(2) var<storage, read_write> verts: array<f32>;
@group(0) @binding(3) var<storage, read_write> counter: atomic<u32>;

fn write_vertex(idx: u32, p: vec3<f32>, n: vec3<f32>, c: vec3<f32>) {
    let base = idx * 9u;
    verts[base + 0u] = p.x;
    verts[base + 1u] = p.y;
    verts[base + 2u] = p.z;
    verts[base + 3u] = n.x;
    verts[base + 4u] = n.y;
    verts[base + 5u] = n.z;
    verts[base + 6u] = c.x;
    verts[base + 7u] = c.y;
    verts[base + 8u] = c.z;
}

// Marching-cubes edge endpoint table — which two corners each of the
// 12 cube edges connects. Order matches Paul Bourke's reference.
const EDGE_VERTS: array<vec2<u32>, 12> = array<vec2<u32>, 12>(
    vec2<u32>(0u, 1u), vec2<u32>(1u, 2u), vec2<u32>(2u, 3u), vec2<u32>(3u, 0u),
    vec2<u32>(4u, 5u), vec2<u32>(5u, 6u), vec2<u32>(6u, 7u), vec2<u32>(7u, 4u),
    vec2<u32>(0u, 4u), vec2<u32>(1u, 5u), vec2<u32>(2u, 6u), vec2<u32>(3u, 7u),
);

// Corner offsets (Bourke ordering): bottom face 0..3 CCW, top face 4..7 CCW.
const CORNER_OFFS: array<vec3<i32>, 8> = array<vec3<i32>, 8>(
    vec3<i32>(0, 0, 0),
    vec3<i32>(1, 0, 0),
    vec3<i32>(1, 1, 0),
    vec3<i32>(0, 1, 0),
    vec3<i32>(0, 0, 1),
    vec3<i32>(1, 0, 1),
    vec3<i32>(1, 1, 1),
    vec3<i32>(0, 1, 1),
);

// Triangle table: 256 cases × 16 entries, flattened to a single 1-D
// const array indexed as `case * 16 + slot`. A flat `array<i32, N>` is
// the most portable shape for a large WGSL constant — nested-array
// constants miscompile on some naga backends (DX12 / older Metal).
// Standard Bourke table; -1 terminates the triangle list per case.
const TRI_TABLE: array<i32, 4096> = array<i32, 4096>(
__TRI_TABLE_ROWS__
);

fn tri_entry(case_idx: u32, slot: u32) -> i32 {
    return TRI_TABLE[case_idx * 16u + slot];
}

fn corner_filled(cell: vec3<i32>, corner: u32) -> bool {
    let o = CORNER_OFFS[corner];
    let p = cell + o;
    let dims = vec3<i32>(textureDimensions(voxels));
    if (p.x < 0 || p.y < 0 || p.z < 0 || p.x >= dims.x || p.y >= dims.y || p.z >= dims.z) {
        return false;
    }
    // Voxel encoding: 0u = filled, anything else = empty.
    return textureLoad(voxels, p, 0).r == 0u;
}

fn corner_world(cell: vec3<i32>, corner: u32) -> vec3<f32> {
    let dims = vec3<f32>(textureDimensions(voxels));
    let extent = u.stock_max.xyz - u.stock_min.xyz;
    let coord = vec3<f32>(cell + CORNER_OFFS[corner]) / max(dims - vec3<f32>(1.0), vec3<f32>(1.0));
    return u.stock_min.xyz + coord * extent;
}

// Gradient-based outward normal at `p`, sampling neighbouring voxels.
// Identical idea to the volumetric renderer's `voxel_normal` so the
// extracted surface lights consistently with any remaining ray-march.
fn surface_normal(p: vec3<f32>) -> vec3<f32> {
    let dims = vec3<f32>(textureDimensions(voxels));
    let h = (u.stock_max.xyz - u.stock_min.xyz) / dims;
    let s = vec3<f32>(1.0);
    let sx = solid_at(p + vec3<f32>(h.x, 0.0, 0.0)) - solid_at(p - vec3<f32>(h.x, 0.0, 0.0));
    let sy = solid_at(p + vec3<f32>(0.0, h.y, 0.0)) - solid_at(p - vec3<f32>(0.0, h.y, 0.0));
    let sz = solid_at(p + vec3<f32>(0.0, 0.0, h.z)) - solid_at(p - vec3<f32>(0.0, 0.0, h.z));
    let g = vec3<f32>(sx, sy, sz);
    let len = length(g);
    if (len < 1e-4) { return vec3<f32>(0.0, 0.0, 1.0); }
    let _ = s;
    return -g / len;
}

fn solid_at(p: vec3<f32>) -> f32 {
    let n = (p - u.stock_min.xyz) / (u.stock_max.xyz - u.stock_min.xyz);
    if (any(n < vec3<f32>(0.0)) || any(n > vec3<f32>(1.0))) { return 0.0; }
    let d = vec3<f32>(textureDimensions(voxels));
    let i = vec3<i32>(clamp(n, vec3<f32>(0.0), vec3<f32>(1.0)) * (d - vec3<f32>(1.0)));
    let v = textureLoad(voxels, i, 0).r;
    return select(0.0, 1.0, v == 0u);
}

fn edge_position(cell: vec3<i32>, edge: u32) -> vec3<f32> {
    let pair = EDGE_VERTS[edge];
    let a = corner_world(cell, pair.x);
    let b = corner_world(cell, pair.y);
    // Both corners straddle the surface; with binary voxels the
    // interpolation parameter t is always 0.5 — half-way along the
    // edge. (For a smoother surface from a continuous SDF, you'd use
    // the actual sign-change-zero crossing here.)
    return (a + b) * 0.5;
}

fn op_color(tag: u32) -> vec3<f32> {
    if (tag < 2u) { return vec3<f32>(0.90, 0.62, 0.30); }
    var palette: array<vec3<f32>, 12> = array<vec3<f32>, 12>(
        vec3<f32>(0.35, 0.75, 0.95),
        vec3<f32>(0.95, 0.45, 0.45),
        vec3<f32>(0.55, 0.85, 0.45),
        vec3<f32>(0.95, 0.75, 0.35),
        vec3<f32>(0.75, 0.55, 0.95),
        vec3<f32>(0.45, 0.85, 0.75),
        vec3<f32>(0.95, 0.55, 0.75),
        vec3<f32>(0.85, 0.85, 0.45),
        vec3<f32>(0.55, 0.75, 0.95),
        vec3<f32>(0.95, 0.65, 0.45),
        vec3<f32>(0.65, 0.95, 0.85),
        vec3<f32>(0.85, 0.45, 0.65),
    );
    return palette[(tag - 2u) % 12u];
}

// Sample the op tag from an adjacent empty voxel so the surface picks
// up the right palette entry, matching the ray-march behaviour.
fn surface_op_tag(p: vec3<f32>) -> u32 {
    // Walk a fraction of a voxel along the surface normal into the
    // empty side — that cell's value is the op tag the user expects.
    let n = surface_normal(p);
    let dims = vec3<f32>(textureDimensions(voxels));
    let h = (u.stock_max.xyz - u.stock_min.xyz) / dims;
    let probe = p + n * (length(h) * 0.6);
    let np = (probe - u.stock_min.xyz) / (u.stock_max.xyz - u.stock_min.xyz);
    if (any(np < vec3<f32>(0.0)) || any(np > vec3<f32>(1.0))) { return 1u; }
    let d = vec3<f32>(textureDimensions(voxels));
    let i = vec3<i32>(clamp(np, vec3<f32>(0.0), vec3<f32>(1.0)) * (d - vec3<f32>(1.0)));
    return textureLoad(voxels, i, 0).r;
}

@compute @workgroup_size(4, 4, 4)
fn main(@builtin(global_invocation_id) id: vec3<u32>) {
    let dims = vec3<u32>(textureDimensions(voxels));
    // Last layer of voxels has no cell on its +X/+Y/+Z side.
    if (id.x >= dims.x - 1u || id.y >= dims.y - 1u || id.z >= dims.z - 1u) {
        return;
    }
    let cell = vec3<i32>(id);

    // Build the 8-bit case index from corner occupancy. Bit i is set
    // when corner i is "filled" (still has material).
    var case_idx: u32 = 0u;
    if (corner_filled(cell, 0u)) { case_idx = case_idx | 1u; }
    if (corner_filled(cell, 1u)) { case_idx = case_idx | 2u; }
    if (corner_filled(cell, 2u)) { case_idx = case_idx | 4u; }
    if (corner_filled(cell, 3u)) { case_idx = case_idx | 8u; }
    if (corner_filled(cell, 4u)) { case_idx = case_idx | 16u; }
    if (corner_filled(cell, 5u)) { case_idx = case_idx | 32u; }
    if (corner_filled(cell, 6u)) { case_idx = case_idx | 64u; }
    if (corner_filled(cell, 7u)) { case_idx = case_idx | 128u; }
    if (case_idx == 0u || case_idx == 255u) { return; }

    var i: u32 = 0u;
    loop {
        if (i >= 15u) { break; }
        let e0 = tri_entry(case_idx, i);
        if (e0 < 0) { break; }
        let e1 = tri_entry(case_idx, i + 1u);
        let e2 = tri_entry(case_idx, i + 2u);

        let p0 = edge_position(cell, u32(e0));
        let p1 = edge_position(cell, u32(e1));
        let p2 = edge_position(cell, u32(e2));
        let n0 = surface_normal(p0);
        let n1 = surface_normal(p1);
        let n2 = surface_normal(p2);
        let c0 = op_color(surface_op_tag(p0));
        let c1 = op_color(surface_op_tag(p1));
        let c2 = op_color(surface_op_tag(p2));

        let base = atomicAdd(&counter, 3u);
        // Each vertex consumes 9 f32 slots, so we need `(base + 3) * 9`
        // slots to be valid before writing.
        if ((base + 3u) * 9u <= arrayLength(&verts)) {
            write_vertex(base + 0u, p0, n0, c0);
            write_vertex(base + 1u, p1, n1, c1);
            write_vertex(base + 2u, p2, n2, c2);
        }
        i = i + 3u;
    }
}
"#;

// ── Marching-cubes triangle table generation ───────────────────────────────
//
// The 256×16 table is the standard "edge sequence" formulation used in
// every reference implementation (Paul Bourke, Lorensen & Cline). Storing
// the data as Rust constants and substituting it into the shader source
// at startup keeps the WGSL file readable while letting us share the
// table with any future CPU-side fallback.

fn build_shader_source() -> String {
    // Flatten 256 × 16 to a single comma-separated list. 16 values per
    // line so the generated source stays inspectable if naga ever
    // prints it back at us.
    let mut rows = String::new();
    let mut first = true;
    for row in TRI_TABLE_RUST.iter() {
        for v in row.iter() {
            if !first {
                rows.push_str(", ");
            }
            first = false;
            rows.push_str(&v.to_string());
        }
        rows.push('\n');
    }
    COMPUTE_SHADER.replace("__TRI_TABLE_ROWS__", &rows)
}

// The triangle table from Paul Bourke's reference implementation.
// `-1` terminates the list for each case. Up to 5 triangles per cell
// → up to 15 edge indices; 16th column always `-1`.
#[rustfmt::skip]
const TRI_TABLE_RUST: [[i32; 16]; 256] = [
    [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 8, 3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 1, 9,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 8, 3, 9, 8, 1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 2,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 8, 3, 1, 2,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 2,10, 0, 2, 9,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 2, 8, 3, 2,10, 8,10, 9, 8,-1,-1,-1,-1,-1,-1,-1],
    [ 3,11, 2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0,11, 2, 8,11, 0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 9, 0, 2, 3,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1,11, 2, 1, 9,11, 9, 8,11,-1,-1,-1,-1,-1,-1,-1],
    [ 3,10, 1,11,10, 3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0,10, 1, 0, 8,10, 8,11,10,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 9, 0, 3,11, 9,11,10, 9,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 8,10,10, 8,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 7, 8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 3, 0, 7, 3, 4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 1, 9, 8, 4, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 1, 9, 4, 7, 1, 7, 3, 1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 2,10, 8, 4, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 4, 7, 3, 0, 4, 1, 2,10,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 2,10, 9, 0, 2, 8, 4, 7,-1,-1,-1,-1,-1,-1,-1],
    [ 2,10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4,-1,-1,-1,-1],
    [ 8, 4, 7, 3,11, 2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [11, 4, 7,11, 2, 4, 2, 0, 4,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 0, 1, 8, 4, 7, 2, 3,11,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 7,11, 9, 4,11, 9,11, 2, 9, 2, 1,-1,-1,-1,-1],
    [ 3,10, 1, 3,11,10, 7, 8, 4,-1,-1,-1,-1,-1,-1,-1],
    [ 1,11,10, 1, 4,11, 1, 0, 4, 7,11, 4,-1,-1,-1,-1],
    [ 4, 7, 8, 9, 0,11, 9,11,10,11, 0, 3,-1,-1,-1,-1],
    [ 4, 7,11, 4,11, 9, 9,11,10,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 5, 4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 5, 4, 0, 8, 3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 5, 4, 1, 5, 0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 8, 5, 4, 8, 3, 5, 3, 1, 5,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 2,10, 9, 5, 4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 0, 8, 1, 2,10, 4, 9, 5,-1,-1,-1,-1,-1,-1,-1],
    [ 5, 2,10, 5, 4, 2, 4, 0, 2,-1,-1,-1,-1,-1,-1,-1],
    [ 2,10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8,-1,-1,-1,-1],
    [ 9, 5, 4, 2, 3,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0,11, 2, 0, 8,11, 4, 9, 5,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 5, 4, 0, 1, 5, 2, 3,11,-1,-1,-1,-1,-1,-1,-1],
    [ 2, 1, 5, 2, 5, 8, 2, 8,11, 4, 8, 5,-1,-1,-1,-1],
    [10, 3,11,10, 1, 3, 9, 5, 4,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 9, 5, 0, 8, 1, 8,10, 1, 8,11,10,-1,-1,-1,-1],
    [ 5, 4, 0, 5, 0,11, 5,11,10,11, 0, 3,-1,-1,-1,-1],
    [ 5, 4, 8, 5, 8,10,10, 8,11,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 7, 8, 5, 7, 9,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 3, 0, 9, 5, 3, 5, 7, 3,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 7, 8, 0, 1, 7, 1, 5, 7,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 5, 3, 3, 5, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 7, 8, 9, 5, 7,10, 1, 2,-1,-1,-1,-1,-1,-1,-1],
    [10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3,-1,-1,-1,-1],
    [ 8, 0, 2, 8, 2, 5, 8, 5, 7,10, 5, 2,-1,-1,-1,-1],
    [ 2,10, 5, 2, 5, 3, 3, 5, 7,-1,-1,-1,-1,-1,-1,-1],
    [ 7, 9, 5, 7, 8, 9, 3,11, 2,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7,11,-1,-1,-1,-1],
    [ 2, 3,11, 0, 1, 8, 1, 7, 8, 1, 5, 7,-1,-1,-1,-1],
    [11, 2, 1,11, 1, 7, 7, 1, 5,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 5, 8, 8, 5, 7,10, 1, 3,10, 3,11,-1,-1,-1,-1],
    [ 5, 7, 0, 5, 0, 9, 7,11, 0, 1, 0,10,11,10, 0,-1],
    [11,10, 0,11, 0, 3,10, 5, 0, 8, 0, 7, 5, 7, 0,-1],
    [11,10, 5, 7,11, 5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [10, 6, 5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 8, 3, 5,10, 6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 0, 1, 5,10, 6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 8, 3, 1, 9, 8, 5,10, 6,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 6, 5, 2, 6, 1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 6, 5, 1, 2, 6, 3, 0, 8,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 6, 5, 9, 0, 6, 0, 2, 6,-1,-1,-1,-1,-1,-1,-1],
    [ 5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8,-1,-1,-1,-1],
    [ 2, 3,11,10, 6, 5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [11, 0, 8,11, 2, 0,10, 6, 5,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 1, 9, 2, 3,11, 5,10, 6,-1,-1,-1,-1,-1,-1,-1],
    [ 5,10, 6, 1, 9, 2, 9,11, 2, 9, 8,11,-1,-1,-1,-1],
    [ 6, 3,11, 6, 5, 3, 5, 1, 3,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 8,11, 0,11, 5, 0, 5, 1, 5,11, 6,-1,-1,-1,-1],
    [ 3,11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9,-1,-1,-1,-1],
    [ 6, 5, 9, 6, 9,11,11, 9, 8,-1,-1,-1,-1,-1,-1,-1],
    [ 5,10, 6, 4, 7, 8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 3, 0, 4, 7, 3, 6, 5,10,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 9, 0, 5,10, 6, 8, 4, 7,-1,-1,-1,-1,-1,-1,-1],
    [10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4,-1,-1,-1,-1],
    [ 6, 1, 2, 6, 5, 1, 4, 7, 8,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7,-1,-1,-1,-1],
    [ 8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6,-1,-1,-1,-1],
    [ 7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9,-1],
    [ 3,11, 2, 7, 8, 4,10, 6, 5,-1,-1,-1,-1,-1,-1,-1],
    [ 5,10, 6, 4, 7, 2, 4, 2, 0, 2, 7,11,-1,-1,-1,-1],
    [ 0, 1, 9, 4, 7, 8, 2, 3,11, 5,10, 6,-1,-1,-1,-1],
    [ 9, 2, 1, 9,11, 2, 9, 4,11, 7,11, 4, 5,10, 6,-1],
    [ 8, 4, 7, 3,11, 5, 3, 5, 1, 5,11, 6,-1,-1,-1,-1],
    [ 5, 1,11, 5,11, 6, 1, 0,11, 7,11, 4, 0, 4,11,-1],
    [ 0, 5, 9, 0, 6, 5, 0, 3, 6,11, 6, 3, 8, 4, 7,-1],
    [ 6, 5, 9, 6, 9,11, 4, 7, 9, 7,11, 9,-1,-1,-1,-1],
    [10, 4, 9, 6, 4,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 4,10, 6, 4, 9,10, 0, 8, 3,-1,-1,-1,-1,-1,-1,-1],
    [10, 0, 1,10, 6, 0, 6, 4, 0,-1,-1,-1,-1,-1,-1,-1],
    [ 8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1,10,-1,-1,-1,-1],
    [ 1, 4, 9, 1, 2, 4, 2, 6, 4,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4,-1,-1,-1,-1],
    [ 0, 2, 4, 4, 2, 6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 8, 3, 2, 8, 2, 4, 4, 2, 6,-1,-1,-1,-1,-1,-1,-1],
    [10, 4, 9,10, 6, 4,11, 2, 3,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 8, 2, 2, 8,11, 4, 9,10, 4,10, 6,-1,-1,-1,-1],
    [ 3,11, 2, 0, 1, 6, 0, 6, 4, 6, 1,10,-1,-1,-1,-1],
    [ 6, 4, 1, 6, 1,10, 4, 8, 1, 2, 1,11, 8,11, 1,-1],
    [ 9, 6, 4, 9, 3, 6, 9, 1, 3,11, 6, 3,-1,-1,-1,-1],
    [ 8,11, 1, 8, 1, 0,11, 6, 1, 9, 1, 4, 6, 4, 1,-1],
    [ 3,11, 6, 3, 6, 0, 0, 6, 4,-1,-1,-1,-1,-1,-1,-1],
    [ 6, 4, 8,11, 6, 8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 7,10, 6, 7, 8,10, 8, 9,10,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 7, 3, 0,10, 7, 0, 9,10, 6, 7,10,-1,-1,-1,-1],
    [10, 6, 7, 1,10, 7, 1, 7, 8, 1, 8, 0,-1,-1,-1,-1],
    [10, 6, 7,10, 7, 1, 1, 7, 3,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7,-1,-1,-1,-1],
    [ 2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9,-1],
    [ 7, 8, 0, 7, 0, 6, 6, 0, 2,-1,-1,-1,-1,-1,-1,-1],
    [ 7, 3, 2, 6, 7, 2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 2, 3,11,10, 6, 8,10, 8, 9, 8, 6, 7,-1,-1,-1,-1],
    [ 2, 0, 7, 2, 7,11, 0, 9, 7, 6, 7,10, 9,10, 7,-1],
    [ 1, 8, 0, 1, 7, 8, 1,10, 7, 6, 7,10, 2, 3,11,-1],
    [11, 2, 1,11, 1, 7,10, 6, 1, 6, 7, 1,-1,-1,-1,-1],
    [ 8, 9, 6, 8, 6, 7, 9, 1, 6,11, 6, 3, 1, 3, 6,-1],
    [ 0, 9, 1,11, 6, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 7, 8, 0, 7, 0, 6, 3,11, 0,11, 6, 0,-1,-1,-1,-1],
    [ 7,11, 6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 7, 6,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 0, 8,11, 7, 6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 1, 9,11, 7, 6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 8, 1, 9, 8, 3, 1,11, 7, 6,-1,-1,-1,-1,-1,-1,-1],
    [10, 1, 2, 6,11, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 2,10, 3, 0, 8, 6,11, 7,-1,-1,-1,-1,-1,-1,-1],
    [ 2, 9, 0, 2,10, 9, 6,11, 7,-1,-1,-1,-1,-1,-1,-1],
    [ 6,11, 7, 2,10, 3,10, 8, 3,10, 9, 8,-1,-1,-1,-1],
    [ 7, 2, 3, 6, 2, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 7, 0, 8, 7, 6, 0, 6, 2, 0,-1,-1,-1,-1,-1,-1,-1],
    [ 2, 7, 6, 2, 3, 7, 0, 1, 9,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6,-1,-1,-1,-1],
    [10, 7, 6,10, 1, 7, 1, 3, 7,-1,-1,-1,-1,-1,-1,-1],
    [10, 7, 6, 1, 7,10, 1, 8, 7, 1, 0, 8,-1,-1,-1,-1],
    [ 0, 3, 7, 0, 7,10, 0,10, 9, 6,10, 7,-1,-1,-1,-1],
    [ 7, 6,10, 7,10, 8, 8,10, 9,-1,-1,-1,-1,-1,-1,-1],
    [ 6, 8, 4,11, 8, 6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 6,11, 3, 0, 6, 0, 4, 6,-1,-1,-1,-1,-1,-1,-1],
    [ 8, 6,11, 8, 4, 6, 9, 0, 1,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 4, 6, 9, 6, 3, 9, 3, 1,11, 3, 6,-1,-1,-1,-1],
    [ 6, 8, 4, 6,11, 8, 2,10, 1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 2,10, 3, 0,11, 0, 6,11, 0, 4, 6,-1,-1,-1,-1],
    [ 4,11, 8, 4, 6,11, 0, 2, 9, 2,10, 9,-1,-1,-1,-1],
    [10, 9, 3,10, 3, 2, 9, 4, 3,11, 3, 6, 4, 6, 3,-1],
    [ 8, 2, 3, 8, 4, 2, 4, 6, 2,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 4, 2, 4, 6, 2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8,-1,-1,-1,-1],
    [ 1, 9, 4, 1, 4, 2, 2, 4, 6,-1,-1,-1,-1,-1,-1,-1],
    [ 8, 1, 3, 8, 6, 1, 8, 4, 6, 6,10, 1,-1,-1,-1,-1],
    [10, 1, 0,10, 0, 6, 6, 0, 4,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 6, 3, 4, 3, 8, 6,10, 3, 0, 3, 9,10, 9, 3,-1],
    [10, 9, 4, 6,10, 4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 9, 5, 7, 6,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 8, 3, 4, 9, 5,11, 7, 6,-1,-1,-1,-1,-1,-1,-1],
    [ 5, 0, 1, 5, 4, 0, 7, 6,11,-1,-1,-1,-1,-1,-1,-1],
    [11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5,-1,-1,-1,-1],
    [ 9, 5, 4,10, 1, 2, 7, 6,11,-1,-1,-1,-1,-1,-1,-1],
    [ 6,11, 7, 1, 2,10, 0, 8, 3, 4, 9, 5,-1,-1,-1,-1],
    [ 7, 6,11, 5, 4,10, 4, 2,10, 4, 0, 2,-1,-1,-1,-1],
    [ 3, 4, 8, 3, 5, 4, 3, 2, 5,10, 5, 2,11, 7, 6,-1],
    [ 7, 2, 3, 7, 6, 2, 5, 4, 9,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7,-1,-1,-1,-1],
    [ 3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0,-1,-1,-1,-1],
    [ 6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8,-1],
    [ 9, 5, 4,10, 1, 6, 1, 7, 6, 1, 3, 7,-1,-1,-1,-1],
    [ 1, 6,10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4,-1],
    [ 4, 0,10, 4,10, 5, 0, 3,10, 6,10, 7, 3, 7,10,-1],
    [ 7, 6,10, 7,10, 8, 5, 4,10, 4, 8,10,-1,-1,-1,-1],
    [ 6, 9, 5, 6,11, 9,11, 8, 9,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 6,11, 0, 6, 3, 0, 5, 6, 0, 9, 5,-1,-1,-1,-1],
    [ 0,11, 8, 0, 5,11, 0, 1, 5, 5, 6,11,-1,-1,-1,-1],
    [ 6,11, 3, 6, 3, 5, 5, 3, 1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 2,10, 9, 5,11, 9,11, 8,11, 5, 6,-1,-1,-1,-1],
    [ 0,11, 3, 0, 6,11, 0, 9, 6, 5, 6, 9, 1, 2,10,-1],
    [11, 8, 5,11, 5, 6, 8, 0, 5,10, 5, 2, 0, 2, 5,-1],
    [ 6,11, 3, 6, 3, 5, 2,10, 3,10, 5, 3,-1,-1,-1,-1],
    [ 5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2,-1,-1,-1,-1],
    [ 9, 5, 6, 9, 6, 0, 0, 6, 2,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8,-1],
    [ 1, 5, 6, 2, 1, 6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 3, 6, 1, 6,10, 3, 8, 6, 5, 6, 9, 8, 9, 6,-1],
    [10, 1, 0,10, 0, 6, 9, 5, 0, 5, 6, 0,-1,-1,-1,-1],
    [ 0, 3, 8, 5, 6,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [10, 5, 6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [11, 5,10, 7, 5,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [11, 5,10,11, 7, 5, 8, 3, 0,-1,-1,-1,-1,-1,-1,-1],
    [ 5,11, 7, 5,10,11, 1, 9, 0,-1,-1,-1,-1,-1,-1,-1],
    [10, 7, 5,10,11, 7, 9, 8, 1, 8, 3, 1,-1,-1,-1,-1],
    [11, 1, 2,11, 7, 1, 7, 5, 1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2,11,-1,-1,-1,-1],
    [ 9, 7, 5, 9, 2, 7, 9, 0, 2, 2,11, 7,-1,-1,-1,-1],
    [ 7, 5, 2, 7, 2,11, 5, 9, 2, 3, 2, 8, 9, 8, 2,-1],
    [ 2, 5,10, 2, 3, 5, 3, 7, 5,-1,-1,-1,-1,-1,-1,-1],
    [ 8, 2, 0, 8, 5, 2, 8, 7, 5,10, 2, 5,-1,-1,-1,-1],
    [ 9, 0, 1, 5,10, 3, 5, 3, 7, 3,10, 2,-1,-1,-1,-1],
    [ 9, 8, 2, 9, 2, 1, 8, 7, 2,10, 2, 5, 7, 5, 2,-1],
    [ 1, 3, 5, 3, 7, 5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 8, 7, 0, 7, 1, 1, 7, 5,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 0, 3, 9, 3, 5, 5, 3, 7,-1,-1,-1,-1,-1,-1,-1],
    [ 9, 8, 7, 5, 9, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 5, 8, 4, 5,10, 8,10,11, 8,-1,-1,-1,-1,-1,-1,-1],
    [ 5, 0, 4, 5,11, 0, 5,10,11,11, 3, 0,-1,-1,-1,-1],
    [ 0, 1, 9, 8, 4,10, 8,10,11,10, 4, 5,-1,-1,-1,-1],
    [10,11, 4,10, 4, 5,11, 3, 4, 9, 4, 1, 3, 1, 4,-1],
    [ 2, 5, 1, 2, 8, 5, 2,11, 8, 4, 5, 8,-1,-1,-1,-1],
    [ 0, 4,11, 0,11, 3, 4, 5,11, 2,11, 1, 5, 1,11,-1],
    [ 0, 2, 5, 0, 5, 9, 2,11, 5, 4, 5, 8,11, 8, 5,-1],
    [ 9, 4, 5, 2,11, 3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 2, 5,10, 3, 5, 2, 3, 4, 5, 3, 8, 4,-1,-1,-1,-1],
    [ 5,10, 2, 5, 2, 4, 4, 2, 0,-1,-1,-1,-1,-1,-1,-1],
    [ 3,10, 2, 3, 5,10, 3, 8, 5, 4, 5, 8, 0, 1, 9,-1],
    [ 5,10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2,-1,-1,-1,-1],
    [ 8, 4, 5, 8, 5, 3, 3, 5, 1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 4, 5, 1, 0, 5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5,-1,-1,-1,-1],
    [ 9, 4, 5,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 4,11, 7, 4, 9,11, 9,10,11,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 8, 3, 4, 9, 7, 9,11, 7, 9,10,11,-1,-1,-1,-1],
    [ 1,10,11, 1,11, 4, 1, 4, 0, 7, 4,11,-1,-1,-1,-1],
    [ 3, 1, 4, 3, 4, 8, 1,10, 4, 7, 4,11,10,11, 4,-1],
    [ 4,11, 7, 9,11, 4, 9, 2,11, 9, 1, 2,-1,-1,-1,-1],
    [ 9, 7, 4, 9,11, 7, 9, 1,11, 2,11, 1, 0, 8, 3,-1],
    [11, 7, 4,11, 4, 2, 2, 4, 0,-1,-1,-1,-1,-1,-1,-1],
    [11, 7, 4,11, 4, 2, 8, 3, 4, 3, 2, 4,-1,-1,-1,-1],
    [ 2, 9,10, 2, 7, 9, 2, 3, 7, 7, 4, 9,-1,-1,-1,-1],
    [ 9,10, 7, 9, 7, 4,10, 2, 7, 8, 7, 0, 2, 0, 7,-1],
    [ 3, 7,10, 3,10, 2, 7, 4,10, 1,10, 0, 4, 0,10,-1],
    [ 1,10, 2, 8, 7, 4,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 9, 1, 4, 1, 7, 7, 1, 3,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1,-1,-1,-1,-1],
    [ 4, 0, 3, 7, 4, 3,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 4, 8, 7,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 9,10, 8,10,11, 8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 0, 9, 3, 9,11,11, 9,10,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 1,10, 0,10, 8, 8,10,11,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 1,10,11, 3,10,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 2,11, 1,11, 9, 9,11, 8,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 0, 9, 3, 9,11, 1, 2, 9, 2,11, 9,-1,-1,-1,-1],
    [ 0, 2,11, 8, 0,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 3, 2,11,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 2, 3, 8, 2, 8,10,10, 8, 9,-1,-1,-1,-1,-1,-1,-1],
    [ 9,10, 2, 0, 9, 2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 2, 3, 8, 2, 8,10, 0, 1, 8, 1,10, 8,-1,-1,-1,-1],
    [ 1,10, 2,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 1, 3, 8, 9, 1, 8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 9, 1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [ 0, 3, 8,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
    [-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1],
];

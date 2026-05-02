# OpenMill User Guide & Technical Manual

Welcome to the comprehensive guide for OpenMill. This document covers the toolpath strategies, simulation pipeline, post-processing rules, and developer-facing extension points.

---

## 1. Toolpath Strategies

OpenMill ships nine strategies covering 3-axis indexed work, 3+2 / 4+1 indexed milling, and simultaneous 5-axis. **Roughing strategies are stock-aware** — they plan from the stock envelope (part AABB grown by margin, or cylinder bounds), not just the part footprint, so the outer stock margin actually gets cleared.

### 3+2 Indexed
The table is locked at fixed `(A, C)` angles for the entire operation. A 3-axis zigzag raster runs in the resulting tilted plane.
- **Use when:** every face of the part is reachable at one fixed orientation, or for a single side of a fixturing setup.
- **Output:** rotary index move (`G0 A C`) followed by 3-axis cuts in `G94` feed-per-minute.
- **Parameters:** `A angle`, `C angle`, `Step-down`, `Step-over` (fraction of tool diameter), `Feed rate`.

### 4+1 Indexed
The A axis is locked at a fixed tilt; the C axis is **stepped** between passes — each row of cuts uses a different C angle.
- **Use when:** wrapping operations around a cylinder, or sweeping a partial cone, in a single setup without re-fixturing.
- **Output:** one toolpath per C-index position, each emitting its own rotary index move and 3-axis raster.
- **Parameters:** `A angle (fixed)`, `C start`, `C step`, `C range`, `Step-down`, `Step-over`, `Feed rate`.

### Adaptive Clearing (Roughing)
Stock-aware layered raster pocket-clear with engagement-angle-driven step-over.
- **Step-over derivation:** `WoC = D · sin(θ/2)` where θ is the configured max engagement angle.
- **Z range:** scans from the **stock top** down to the part bottom, clamping each layer to the part surface via raycast so it never gouges.
- **Status:** this is a starter implementation — true trochoidal medial-axis paths require a 2D voronoi solver and are still future work. The output is a layered raster but it produces valid stock-aware G-code today.
- **Parameters:** `Max engagement` (deg), `Step-down`, `Feed rate`.

### Contour Parallel (Waterline)
Z-level finishing that scans from the **stock top** down to the part bottom. At each Z level it raycasts inward from the stock perimeter, offsets by tool radius, and emits a perimeter pass.
- **Use when:** finishing vertical or near-vertical walls.
- **Parameters:** `Step-down`, `Feed rate`, `Tolerance`.

### Surface Normal 5-Axis (Finishing)
Simultaneous 5-axis finishing where the tool axis is tilted to follow (or offset from) the surface normal at each contact point.
- **Patterns:**
    - `Parallel` — raster passes across the surface.
    - `Scallop` — concentric loops centred on the part footprint, ideal for constant cusp height.
    - `Spiral` — continuous spiral path to avoid lead-in/out marks.
    - `Rotary` — 360° rotary scan around the X axis, centred on the YZ centroid of the part AABB.
- **Lead & Tilt Angles:**
    - `Lead Angle`: tilts the tool forward in the direction of travel.
    - `Tilt Angle`: tilts the tool sideways.
    - *Tip:* a small lead angle on a ball-end mill avoids cutting with the zero-surface-speed tip.

### Swarf 5-Axis (Flank Machining)
The cutter side is held tangent to a ruled surface — turbine vanes, blade edges, tapered walls.
- **How it works:** raycasts at top and bottom Z bands to find ruling pairs, then aligns the tool axis from bottom to top.
- **Parameters:**
    - `Z Top` / `Z Bottom`: axial extent of the wall.
    - `Num passes` and `Step-down`: optional multiple axial levels for deeper walls.
    - `Overhang compensation` (toggle).

### Geodesic Parallel (Finishing)
Maintains a constant distance along the surface from a boundary so paths don't bunch up on steep walls. The current implementation uses Z height as a distance proxy; a true heat-method geodesic solver is on the roadmap.

### 5-Axis Pencil Tracing
Detects concave edges by the dihedral angle between adjacent triangles and traces them with a ball-end mill, tilting the tool axis to bisect the corner angle for optimal reach.
- **Parameter:** `Concavity Threshold` (deg).

### 5-Axis Drilling
Plunge-retract cycles where the tool axis aligns with each hole's drill axis.
- **Hole editor:** add/remove holes with explicit `(X, Y, Z)` position, `(X, Y, Z)` axis vector, and `Depth`. New holes default to the loaded model's top centre pointing −Z.
- **Output:** rapid to safe height → linear plunge to depth → retract.

---

## 2. Stock-Aware Roughing

Every model in OpenMill has both a **part AABB** (the loaded mesh) and a **stock AABB** (the envelope of raw material before cutting). Roughing strategies must plan from the stock envelope or the outer margin never gets cleared.

`WorkpieceModel::stock_aabb()` returns the right bounds for whichever stock shape is configured:

- **`StockShape::BoundingBox { margin }`** — part AABB grown by the per-axis margin.
- **`StockShape::Cylinder { diameter, height }`** — cylinder envelope centred on the part footprint.

Strategies that use the stock AABB: 3+2, 4+1, Adaptive Clearing, Contour Parallel.
Strategies that use the part AABB: Surface Normal, Geodesic, Swarf, Pencil, Drilling (these work at the part surface by design).

---

## 3. Tool Library

Tools persist as JSON in the `tools/` directory. The library auto-loads on startup.

### Geometry
Nine cutter shapes with proper 2D profile preview (auto-scaled to fit the available area):
- **Flat End** — square-end mill.
- **Ball End** — ball-nose mill.
- **Bull Nose** — toroid with corner radius.
- **Chamfer Mill** — taper from tip to major diameter.
- **Drill** — point-angle drill.
- **Tapered Mill** — conical end mill.
- **Lollipop** — undercutting spherical mill.
- **Dovetail** — T-slot cutter (flares wider at the bottom).
- **Thread Mill** — single- or multi-tooth thread cutter, drawn at the configured pitch.

Each tool also carries `flutes`, `overall_length`, `shank_diameter`, and an optional `holder` profile.

### Feed-and-Speed Presets

Each tool carries a list of named **feed-and-speed presets**:

```rust
struct FeedSpeedPreset {
    name:        String,
    material:    String,
    spindle_rpm: f64,
    feed_rate:   f64,   // mm/min cutting
    plunge_rate: f64,   // mm/min lead-in
    coolant:     Coolant,
    notes:       String,
}
```

In the Tool tab, the **Feed/Speed Presets** collapsible offers:
- **➕ Add Empty** — create a blank preset.
- **✨ Load Starter Set** — replace presets with computed starters for common materials, using `RPM = SFM·1000 / (π·D)` and `Feed = chip_load · flutes · RPM`. Supplies presets for 6061 Aluminum, Mild Steel, HDPE/Plastic, Hardwood, and Brass.
- **★ default toggle** — pick which preset is auto-applied when adding a new operation with this tool.

Apply a preset to an operation from the operation editor — the preset's `RPM`, `feed`, `plunge`, and `coolant` are written onto the op.

### Coolant Modes
| Mode | G-code on | G-code off |
|---|---|---|
| None | (nothing emitted) | — |
| Mist | `M7` | `M9` |
| Flood | `M8` | `M9` |
| Mist + Flood | `M7 M8` | `M9` |
| Through-spindle | `M88` (LinuxCNC) | `M9` |

### Tool-Change G-code
Each tool can carry a free-form `tool_change_gcode` string. After the post emits `M6 T#` (LinuxCNC) or `M0` pause (GRBL), every line of this string is appended to the program. Use it for probing routines, applying tool-length offsets (`G43 H<n>`), homing rotary axes, or anything else specific to that tool's setup.

---

## 4. Operation Editor

An operation pairs a tool with a strategy and adds runtime context the strategy doesn't know about:

- **Spindle** (RPM)
- **Feed** (mm/min cutting). `0.0` = use whatever the strategy embeds in each toolpath point.
- **Plunge** (mm/min lead-in). `0.0` = use `feed × 0.5`.
- **Coolant** (see table above)
- **Custom op G-code** — multi-line free-form text injected after the spindle/coolant come on. Common uses: tool-length offset (`G43 H1`), datum shift, fixture probing.

The **Preset** dropdown lists every preset on the operation's tool. Picking one writes spindle / feed / plunge / coolant onto the op in one click.

Feed-rate overrides are applied via `Toolpath::with_op_feeds(cutting, plunge)`:
- `MoveType::Linear` and `MoveType::LeadOut` get the cutting feed.
- `MoveType::LeadIn` gets the plunge feed.
- Strategy-supplied feeds are preserved when the op-level value is `0.0`.

---

## 5. Plan vs Simulation View

The viewport has two distinct modes, switchable from the bottom panel:

### 🧭 Plan View (default)
- **Shows:** the original part mesh, stock wireframe, generated toolpaths, tool wireframe at the current scrub position.
- **Hides:** the voxel stock.
- **No carving runs** — used for path planning and toolpath review.

### 🛠 Simulation View
- **Shows:** voxel stock with surface-gradient shading, stock wireframe, toolpaths, tool wireframe, and the part mesh as a **translucent red X-ray ghost** so it's always visible through the stock for collision checking.
- **Hides:** the solid part mesh.
- **Carves:** every advance of the simulation playhead removes voxels along the swept volume of the tool body.

Switching into Sim view forces a fresh voxel rebuild from the start of the current op (previous ops are replayed automatically so accumulation is correct).

### Voxel Carving
Implemented as a WGSL compute shader (`openmill-ui/src/voxel.rs`). For each move it samples the **entire tool body** (not just the tip) along the tool axis at multiple heights, taking the minimum capsule distance at each pose:

```
for each height h in [0..flute_length]:
    a = tool_start + axis_start * h
    b = tool_end   + axis_end   * h
    if dist_to_segment(world_pos, a, b) < radius:
        carve voxel
```

The cutting length is the tool's flute length — the shank above doesn't carve. Without this multi-sample sweep, only a thin tube along the tip path got removed and the bulk of the stock looked unchanged.

### Voxel Rendering
Surface normals are computed from the gradient of the voxel field (central differences across one voxel) so concave cavities (where stock was removed) render visibly distinct from the convex outer faces. Without this every voxel surface had identical shading and the stock appeared static even while carving.

### Collision Detection
Each simulation step casts a ray from the previous tool tip to the current tip against `model.mesh`. Hits get logged as red 3D-cross markers. The translucent ghost mesh in Sim view makes the actual collision geometry visible at all times.

---

## 6. Post-Processing

OpenMill translates internal toolpaths into machine-ready G-code via the `PostProcessor` trait. Two dialects ship out of the box.

### Per-Operation Emission Order
The exporter and the live G-code terminal both follow the same contract:

1. `tool_change(tool)` — only on tool changes between ops. Emits:
   - `M5` (stop spindle)
   - `M9` (stop coolant)
   - `M6 T<id>` *(LinuxCNC)* or `M0 (Tool change…)` *(GRBL — no real M6)*
   - Lines from the tool's `tool_change_gcode`, if any.
2. `op_preamble(op, tool)` — at the start of every operation:
   - `M3 S<rpm>` (spindle on at op RPM)
   - Coolant on (`M7`/`M8`/`M7 M8`/`M88`)
   - Lines from the op's `gcode_command`.
3. `process_toolpath(tp)` for each toolpath in the op.
4. `op_postamble(op)` — at the end of every operation: `M9` (coolant off).
5. Repeat 1–4 per op.
6. `footer()` once at program end: `M5`, `M9`, `M30`, `%` (LinuxCNC).

### G93 vs G94 — Feed Mode Auto-Detection
Each toolpath is classified by checking whether all points share the same tool-axis direction (within 1 × 10⁻⁶ rad):

- **Indexed pass** (constant axis): emits a `G0 A C` rotary index move first, then runs in `G94` feed-per-minute — same as a standard 3-axis program.
- **Continuous pass** (varying axis): runs in `G93` inverse-time so the controller synchronises all five axes across each segment.

`G94` is restored at the end of every continuous pass.

### Inverse-Time Feed Math
For each `G1` segment in continuous mode, the F-word is `1.0 / time_in_minutes`, where `time_in_minutes` is the maximum of:
- linear travel time (machine-frame XYZ distance ÷ programmed feed),
- A-axis travel time (`|ΔA| / MAX_ROTARY_VELOCITY`),
- C-axis travel time (`|ΔC| / MAX_ROTARY_VELOCITY`).

`MAX_ROTARY_VELOCITY` defaults to `3600 deg/min`. Near the A ≈ 0° singularity a tiny orientation change can produce a huge ΔC; dividing by `MAX_ROTARY_VELOCITY` clamps the rotary velocity and slows the segment naturally instead of commanding infinite C velocity.

### Kinematics & IK Fallback
`joints_for_point(pt)` resolves XYZ/AC machine joints:
- Near-vertical tool axis (within singularity threshold) → `A=0, C=0` and raw workpiece XYZ. This is the correct behaviour for 3-axis cuts.
- IK error (out-of-limits, singularity) → falls back to the same `A=0, C=0` so the post-processor never silently drops a line and the G-code terminal stays populated.

### Supported Controllers
- **LinuxCNC** (RS274NGC): `%` delimiters, `O` program number, `G54`–`G59`, `M6` true tool change, `M88` through-spindle coolant.
- **GRBL**: simpler header (no `%`, no `O`), `M0` pause for manual tool changes.

---

## 7. Saving and Loading

| Object | Where | Format |
|---|---|---|
| Tools | `tools/tool_<id>.json` | one tool per file, auto-loaded on startup |
| Machines | `machines/<name>.json` | one config per file, loaded into the Machine Library |
| Job (model + ops) | `*.omj` (planned) | full job export — *currently in-memory only between sessions* |

Existing tool JSON files keep working when new fields are added — every new field on `Tool`, `Operation`, etc. is `#[serde(default)]`.

---

## 8. Technical Architecture for Developers

OpenMill is built around small, well-typed traits. Two extension points cover most additions:

### Adding a Strategy
Implement [`ToolpathStrategy`](./openmill-core/src/strategies/traits.rs) in `openmill-core`:

```rust
pub trait ToolpathStrategy {
    type Params: Default + Serialize + DeserializeOwned;
    fn name(&self) -> &str;
    fn generate(
        &self,
        model: &WorkpieceModel,
        tool: &Tool,
        machine: &MachineConfig,
        params: &Self::Params,
    ) -> Result<Vec<Toolpath>>;
}
```

Then register it in `strategies/mod.rs`, add its name to the `STRATEGIES` list in `openmill-ui/src/app.rs`, and wire up the dispatch arm + the params editor.

For roughing strategies, use `model.stock_aabb()` for scan bounds. For finishing, `model.aabb` is correct.

### Adding a Post-Processor
Implement [`PostProcessor`](./openmill-post/src/traits.rs):

```rust
pub trait PostProcessor {
    fn header(&self, config: &PostConfig) -> String;
    fn process_toolpath(&self, tp: &Toolpath, machine: &MachineConfig)
        -> Result<Vec<(String, Option<usize>)>>;
    fn tool_change(&self, tool: &Tool) -> String;
    fn op_preamble(&self, op: &Operation, tool: &Tool) -> String;
    fn op_postamble(&self, op: &Operation) -> String;
    fn footer(&self) -> String;
}
```

Each `process_toolpath` line is paired with the source point index so the UI can map clicks in the G-code terminal back to simulation playhead positions.

### Raycasting & Geometry
Powered by `parry3d`. Used for tool-mesh collision detection, surface raycasting in roughing/finishing strategies, and AABB queries.

### Workspace Map
- **`openmill-core`** — geometry, kinematics (`TableTable` IK), `WorkpieceModel`, `StockShape`, strategies, tool & feed-speed types.
- **`openmill-post`** — `PostProcessor` trait, `LinuxCncPost`, `GrblPost`, inverse-time feed math.
- **`openmill-sim`** — simulation primitives.
- **`openmill-ui`** — `egui` desktop app, `wgpu` viewport, voxel volume + raymarched stock shader, ghost-mesh X-ray pipeline.

---

*Happy Milling!*

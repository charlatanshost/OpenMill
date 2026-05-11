# OpenMill User Guide & Technical Manual

Welcome to the comprehensive guide for OpenMill. This document covers the toolpath strategies, feature recognition, simulation pipeline, post-processing rules, and developer-facing extension points.

---

## 1. Toolpath Strategies

OpenMill ships **13 strategies** covering 3-axis indexed work, 3+2 / 4+1 indexed milling, simultaneous 5-axis, and feature-driven cycles. **Roughing strategies are stock-aware** — they plan from the stock envelope (part AABB grown by margin, or cylinder bounds), not just the part footprint, so the outer stock margin actually gets cleared.

### Cross-cutting strategy controls

Three parameters live on every cutting strategy (and are applied uniformly by `apply_common_transforms` in `openmill-ui/src/generate.rs`, so adding a new strategy automatically picks them up):

- **Cut Direction** — `Climb` / `Conventional` / `Either`. `Conventional` reverses every cutting block (sequence of `Linear` / `LeadIn` / `LeadOut` points) as a post-pass; rapids stay put so approach moves still line up.
- **Z Range** — optional inclusive `top_mm` / `bottom_mm`. Cutting points outside the range are dropped; rapids and retracts are kept so the tool can still travel above the cut zone.
- **Spring pass** — boolean + `feed_fraction` (default 0.5). Clones the last "pass unit" (suffix from the last `Rapid` through end of the toolpath) at the reduced feed and appends it to the toolpath. Cleans up cutter deflection on finishing passes.

Order of application: direction → Z range → spring pass.

### Cusp-height step-over

Surface strategies (3+2, 4+1, Surface Normal 5-Axis, Geodesic) accept an optional `step_over_cusp_mm` field. When present it overrides the step-over fraction and derives the lateral step from the tool's tip radius:

```
step_over = 2 · √(2 · R · h − h²)
```

`R` is the ball radius for ball-end tools, the corner radius for bull-nose, or unsupported for flat tools (the helper falls back to the fraction in that case).

### 3+2 Indexed
The table is locked at fixed `(A, C)` angles for the entire operation. A 3-axis zigzag raster runs in the resulting tilted plane.
- **Use when:** every face of the part is reachable at one fixed orientation, or for a single side of a fixturing setup.
- **Output:** rotary index move (`G0 A C`) followed by 3-axis cuts in `G94` feed-per-minute.
- **Parameters:** `A angle`, `C angle`, `Step-down`, `Step-over` (fraction of tool diameter), `Feed rate`, `Stock to leave`.

### 4+1 Indexed
The A axis is locked at a fixed tilt; the C axis is **stepped** between passes — each row of cuts uses a different C angle.
- **Use when:** wrapping operations around a cylinder, or sweeping a partial cone, in a single setup without re-fixturing.
- **Output:** one toolpath per C-index position, each emitting its own rotary index move and 3-axis raster.
- **Parameters:** `A angle (fixed)`, `C start`, `C step`, `C range`, `Step-down`, `Step-over`, `Feed rate`, `Stock to leave`.

### Adaptive Clearing (Roughing)
Stock-aware layered raster pocket-clear with engagement-angle-driven step-over.
- **Step-over derivation:** `WoC = D · sin(θ/2)` where θ is the configured max engagement angle.
- **Z range:** scans from the **stock top** down to the part bottom, clamping each layer to the part surface via raycast so it never gouges.
- **Status:** this is a starter implementation — true trochoidal medial-axis paths require a 2D voronoi solver and are still future work. The output is a layered raster but it produces valid stock-aware G-code today.
- **Parameters:** `Max engagement` (deg), `Step-down`, `Feed rate`, `Stock to leave`.

### Simultaneous 5-Axis Roughing
Stock-aware layered raster that tilts the tool axis along the local surface normal so the cutter approaches each layer's surface at a favourable angle.
- **How it works:** at each raster point, raycast to the part surface to read the local normal, then tilt the tool by `lead_angle` from the normal (capped by `max_tilt_deg` so the machine doesn't rotate excessively in steep areas).
- **Parameters:** `Step-down`, `Step-over` (fraction of D), `Feed rate`, `Stock to leave`, `Max tilt` (deg from +Z, default 45°), `Lead angle` (deg).

### Pocket Clearing (Roughing)
Bounded raster clear of a list of detected **pocket features** (see §2). For each pocket, a layered raster runs inside its circular footprint with raycast Z-clamping to the actual mesh surface so walls and floor edges aren't gouged.
- **Use when:** you want to clear a specific cavity rather than the whole stock volume — typically combined with Adaptive Clearing on the surrounding stock.
- **Parameters:** `Step-down`, `Step-over`, `Feed rate`, `Stock to leave`. Pockets are populated by the Features panel.

### Contour Parallel (Waterline)
Z-level finishing that scans from the **stock top** down to the part bottom. At each Z level it raycasts inward from the stock perimeter, offsets by tool radius (plus any stock-to-leave skin), and emits a perimeter pass.
- **Use when:** finishing vertical or near-vertical walls.
- **Parameters:** `Step-down`, `Feed rate`, `Tolerance`, `Stock to leave`.

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
- **Stock to leave** offsets each contact point along the surface normal (with `tool_r + skin` for ball-end tools).

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
Canned-cycle drilling where the tool axis aligns with each hole's drill axis.
- **Hole editor:** add/remove holes manually with explicit `(X, Y, Z)` position, `(X, Y, Z)` axis vector, and `Depth`. New holes default to the loaded model's top centre pointing −Z. Detected hole features can also be assigned via the Features panel.
- **Cycle type:** picks which canned cycle the strategy expands into:
  - **Drill (`G81`)** — straight plunge + retract.
  - **Counter-bore (`G82`)** — plunge + dwell at bottom + retract. `Dwell at bottom` (s) parameter.
  - **Peck (`G83`)** — multi-step plunge with full retract to clearance between pecks. `Peck depth` parameter (0 = ¼ of hole depth).
  - **Chip-break (`G73`)** — multi-step plunge with a short retract (`Chip-break retract` mm) between pecks; tip stays in the hole.
  - **Bore (`G85`)** — feed in, feed out (no rapid retract) for precision bores.
- **Break-through:** optional distance (mm) the tool drills past the hole bottom — clears the burr on through-holes.
- **Output:** rapid to safe height → cycle-specific plunge / peck / dwell → retract.

### Tapping
Rigid or floating tap cycles. Synchronised feed rate is derived from `feed = rpm × pitch`.
- **Parameters:** `Thread pitch` (mm/rev), `Spindle RPM (synced)`, `Peck depth` (0 = full-depth single pass), `Dwell at bottom` (s).
- **Thread direction:** `Right` (default, M3) or `Left` (M4) — the post emits the right spindle direction so the tap unthreads cleanly. Routed via `spindle_command_for(op)` in `openmill-post/src/traits.rs`.
- **Tap mode:** `Rigid` or `Floating`. Rigid forces the Fanuc post to emit `M29 S<rpm>` before the cycle so the controller synchronises spindle rotation with Z feed. Floating tap holders absorb the mismatch mechanically — no M29.
- **Hole input:** populated from detected hole features via the Features panel.

### Thread Milling
Helical interpolation of a thread mill around the bore (or boss) of each hole. The helix is discretised into `segments_per_rev` linear segments per revolution — portable across controllers without true `G2`/`G3` support.
- **Parameters:** `Thread Ø (major)`, `Thread pitch`, `Thread depth`, `Feed rate`, `Segments / rev`, `Direction` (climb / conventional).
- **Thread direction:** `Right` / `Left` — sets the helix sense and the spindle command.
- **Type:** `Internal (bore)` or `External (boss)` — flips the helix offset so the cutter rides the correct side of the wall.
- **Hole input:** populated from detected hole features via the Features panel.

---

## 2. Feature Recognition & Manual Picking

The **Features panel** (left side panel) turns mesh geometry into op inputs.

### Auto-detection
- **🔍 Holes** — `detect_holes(&TriMesh)` finds connected clusters of near-horizontal-normal triangles whose normals point inward toward a common axis. Filters by triangle count, diameter range (0.2–200 mm), depth (≥ 0.5 mm), and an inward-normal vote (≥ 70% of cluster normals point toward the cluster centroid). Vertical-axis holes only.
- **🔍 Pockets** — `detect_pockets(&TriMesh)` finds connected clusters of horizontal up-facing triangles below the model top, then raycasts straight up from the cluster centroid to verify there's clearance above.

### Manual face picking
- **🎯 Pick Face Mode** toggles a click-to-pick mode in the viewport. Click any face — the picker raycasts the mesh, flood-fills coplanar neighbours (within ~5° dihedral tolerance), and emits an auto-classified feature:
    - normal `nz > 0.7` and below model top → **Pocket**
    - `|nz| < 0.35` with depth ≥ 0.5 mm → **Hole**
    - everything else → **FlatFace**
- The picked cluster is highlighted **amber** in the viewport. **Drag still orbits** the camera; only discrete left-clicks pick.

### Assigning to operations
Each feature row exposes context-appropriate **→** buttons:
- Holes → **Drilling**, **Tapping**, **Thread Mill**
- Pockets → **Pocket Clearing**

A bulk-assign row groups all matching features and lets you replace an op's feature list in one click.

---

## 3. Stock-Aware Roughing

Every model has both a **part AABB** (the loaded mesh) and a **stock AABB** (the envelope of raw material). `WorkpieceModel::stock_aabb()` returns the right bounds:

- **`StockShape::BoundingBox { margin }`** — part AABB grown by per-axis margin.
- **`StockShape::Cylinder { diameter, height }`** — cylinder envelope centred on the part footprint.

Strategies that use the **stock AABB**: 3+2, 4+1, Adaptive Clearing, Simultaneous 5-Axis Roughing, Contour Parallel.
Strategies that use the **part AABB**: Surface Normal, Geodesic, Swarf, Pencil, Drilling, Tapping, Thread Milling (these work at the part surface by design). Pocket Clearing uses each pocket feature's own bounds.

### Stock to Leave
Each operation carries a `stock_to_leave` field (mm) that's automatically injected into the strategy params at generate time. Strategies that have a `stock_to_leave` field pick it up; the rest ignore the extra key.

Workflow:
1. **Roughing** op (e.g. Adaptive Clearing): `stock_to_leave = 0.3 mm` — leaves a 0.3 mm skin everywhere.
2. **Finishing** op (e.g. Contour Parallel): `stock_to_leave = 0.0 mm` — skims the skin off to size.

Strategies honour the skin by lifting cutting moves above the part surface (3-axis Z clamp) or by offsetting along the surface normal (Contour Parallel, Surface Normal 5-Axis).

### Lead-In / Lead-Out
Per-operation `LeadConfig` (Plunge / Ramp / Arc) is applied as a cross-cutting post-pass by `apply_leads(toolpath, &lead)` in `openmill-core/src/toolpath/leads.rs`. The lead replaces the first cut point of each cutting block with a Plunge / Ramp / Arc approach and mirrors it on exit. Five-axis paths (varying tool axis) are left untouched — leads only make sense in a flat cutting plane.

---

## 4. Stock & Model Setup

### Stock Dimensions
The Stock section supports both **mm** and **decimal inch** (e.g. 1.5 in, 2.375 in — never fractions) via a unit toggle. Internal storage is always millimetres.

When a model is loaded, the editor shows the **absolute stock dimensions** (W × D × H) and derives the symmetric per-axis margin under the hood: `margin[i] = (stock_size[i] − part_size[i]) / 2`. Without a model, the editor falls back to per-side margin entry.

### Model Position
The **Model position** sub-section translates the part anywhere in the work area:
- X / Y / Z drag values in the current unit.
- **⟲ Reset** moves back to the imported origin.
- **⌖ Centre on origin** drops the AABB centroid at `(0, 0, 0)`.

Position changes immediately rebuild the in-memory mesh from `mesh_orig` translated by the offset, re-upload to the viewport, and re-upload the stock wireframe (which auto-tracks the new AABB). The position is persisted to the job so save/load round-trips preserve it.

---

## 5. Tool Library

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

### Holder Definition & Collision
The `ToolHolder` profile is a `Vec<(z, r)>` describing the holder silhouette in tool-local coordinates. The simulator runs `holder_collision(tool, pose, mesh)` on every step — for each pose it transforms mesh vertices into tool-local space and reports any vertex inside the holder envelope. Without a holder defined, no holder check runs.

---

## 6. Operation Editor

An operation pairs a tool with a strategy and adds runtime context the strategy doesn't know about:

- **Spindle** (RPM)
- **Feed** (mm/min cutting). `0.0` = use whatever the strategy embeds in each toolpath point.
- **Plunge** (mm/min lead-in). `0.0` = use `feed × 0.5`.
- **Coolant** (see table above)
- **Stock to leave** (mm) — auto-injected into strategy params at generate time.
- **Custom op G-code** — multi-line free-form text injected after the spindle/coolant come on. Common uses: tool-length offset (`G43 H1`), datum shift, fixture probing.

The **Preset** dropdown lists every preset on the operation's tool. Picking one writes spindle / feed / plunge / coolant onto the op in one click.

Feed-rate overrides are applied via `Toolpath::with_op_feeds(cutting, plunge)`:
- `MoveType::Linear` and `MoveType::LeadOut` get the cutting feed.
- `MoveType::LeadIn` gets the plunge feed.
- Strategy-supplied feeds are preserved when the op-level value is `0.0`.

Operations are reorderable with **⬆ / ⬇** buttons in the list. Reordering swaps both the operation and its generated toolpath, updates `selected_op` so focus follows the moved op, and forces a sim rebuild on the next entry to Simulation view (op order changes the stock progression).

Each row also shows a **per-op estimated runtime** (e.g. `[3m 22s]`); the panel header shows the **job-wide total**.

---

## 7. Plan vs Simulation View

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
Surface normals are computed from the gradient of the voxel field (central differences across one voxel) so concave cavities (where stock was removed) render visibly distinct from the convex outer faces.

### Collision Detection
Each simulation step runs:
- **Tip-path raycast** from the previous tool-tip position to the current one, against `model.mesh`.
- **Holder envelope check** at the current pose, transforming mesh vertices into tool-local space and testing against the holder profile.

Hits get logged as red 3D-cross markers. The translucent ghost mesh in Sim view makes the actual collision geometry visible at all times.

---

## 8. Verification & Time Estimates

### Verification
**Verify → ✓ Verify all toolpaths** runs `verify_job(toolpaths, machine, tools, model)` and logs a list of `Issue { level, message, toolpath_idx, point_idx }`. Errors are prefixed `✗ ERROR:`, warnings `⚠ warn :`. Checks include:
- Linear axis limits per point (X / Y / Z).
- Rotary range (A / C) via IK, skipped near the singularity where the post emits `A=0 C=0` itself.
- `MoveType::Linear` / `LeadIn` / `LeadOut` with `feed_rate <= 0` (error) or implausibly high feed (warning).
- Cuts below the part floor (warning).
- Holder collisions sampled along the toolpath (error).

**Export auto-runs the verifier** and refuses to write G-code if any **errors** fire. Warnings don't block export — they're just logged. You can always run Verify manually to triage warnings before exporting.

### Time Estimates
`Toolpath::duration_minutes(rapid_mm_per_min)` sums `distance / rate` per segment, using the segment feed for cuts and a configurable rapid speed for `Rapid` / `Retract` moves (UI defaults to **5000 mm/min**). The Operations panel shows per-op time and a job-wide total formatted as `Hh Mm Ss`.

This is a programmer's estimate — it doesn't model acceleration / deceleration or tool changes. Real machine time is typically 5–15% higher.

---

## 9. Post-Processing

OpenMill translates internal toolpaths into machine-ready G-code via the `PostProcessor` trait. Three dialects ship out of the box: **LinuxCNC**, **GRBL**, and **Fanuc**.

### Per-Operation Emission Order
The exporter and the live G-code terminal both follow the same contract:

1. `tool_change(tool)` — only on tool changes between ops. Emits:
   - `M5` (stop spindle)
   - `M9` (stop coolant)
   - `M6 T<id>` *(LinuxCNC)*, `T<id> M6` + `G43.4 H<id>` + `M01` *(Fanuc)*, or `M0 (Tool change…)` *(GRBL — no real M6)*
   - Lines from the tool's `tool_change_gcode`, if any.
2. `op_preamble(op, tool)` — at the start of every operation:
   - `M3` or `M4 S<rpm>` (spindle on at op RPM, sense per `spindle_command_for(op)` — flips to `M4` for left-hand Tapping / Thread Milling).
   - **Rigid tap M29** (Fanuc only): if the op's strategy is Tapping and its `tap_mode == rigid`, emit `M29 S<rpm>` before the cycle.
   - Coolant on (`M7`/`M8`/`M7 M8`/`M88`)
   - Lines from the op's `gcode_command`.
3. `process_toolpath(tp)` for each toolpath in the op.
4. `op_postamble(op)` — at the end of every operation: `M9` (coolant off).
5. Repeat 1–4 per op.
6. `footer()` once at program end: `M5`, `M9`, `M30`, `%` (LinuxCNC / Fanuc).

### G93 vs G94 — Feed Mode Auto-Detection
Each toolpath is classified by checking whether all points share the same tool-axis direction (within 1 × 10⁻⁶ rad):

- **Indexed pass** (constant axis): emits a `G0 A C` rotary index move first, then runs in `G94` feed-per-minute — same as a standard 3-axis program.
- **Continuous pass** (varying axis): runs in `G93` inverse-time (LinuxCNC) or TCPM `G43.4` with tool-axis `I/J/K` words (Fanuc) so the controller synchronises all five axes across each segment.

`G94` is restored at the end of every continuous pass.

### Fanuc TCPM Mode
The Fanuc post emits sequence-numbered (`N10`, `N20`, …) blocks framed by `%` delimiters. For continuous 5-axis passes it sets `G43.4 H<n>` once on tool change and then emits each `G1` as tool-tip XYZ + `I/J/K` tool-axis vector — the controller resolves rotary positions from the kinematic model. This is the convention Fusion 360 and Mastercam use on Fanuc 30i/31i.

For indexed passes the Fanuc post falls back to joint-space A/C output with plain `G94`, so the same post still works on older controllers without TCPM.

The header emits a standard Fanuc safe-start block (`G17 G21 G40 G49 G80 G90`) so the program is in a known state regardless of the previous job's modal state.

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
- **LinuxCNC** (RS274NGC): `%` delimiters, `O` program number, `G54`–`G59`, `M6` true tool change, `M88` through-spindle coolant, `G93` inverse-time for continuous 5-axis.
- **GRBL**: simpler header (no `%`, no `O`), `M0` pause for manual tool changes.
- **Fanuc** (Fanuc / Haas / Brother / Mazak family): `%` delimiters, `O` program number, safe-start block, sequence numbers on every motion line, `T# M6` + `G43.4 H#` + `M01` operator-verify on tool change, `M29` rigid-tap synchronisation, TCPM (`G43.4`) for continuous 5-axis with `I/J/K` tool-axis words.

---

## 10. Saving and Loading

| Object | Where | Format |
|---|---|---|
| Tools | `tools/tool_<id>.json` | one tool per file, auto-loaded on startup |
| Machines | `machines/<name>.json` | one config per file, loaded into the Machine Library |
| Job | `*.json` | full job export — model path, position, stock, ops, features, tools, machine, settings |

**File → Save Job…** writes the entire `Job` struct as JSON. **File → Open Job…** reads it back and:
1. Resets live state (selection, features, voxel sim, viewport overlays).
2. Sets `self.job` first so subsequent state derives from the loaded job.
3. Loads the model from `model_path` (relative to the job file's directory).
4. Re-applies the saved `model_position` after the model loads, then re-uploads mesh and stock.

Existing tool / machine / job JSON files keep working when new fields are added — every new field is `#[serde(default)]`.

---

## 11. Technical Architecture for Developers

OpenMill is built around small, well-typed traits. Most contributions land in one of four places. Concrete step-by-step instructions live in [CONTRIBUTING.md](./CONTRIBUTING.md); the trait surfaces and conventions are below.

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

Then register it in `strategies/mod.rs`, re-export from `openmill-core/src/lib.rs`, add a UI editor arm + a `default_params_for` entry in `openmill-ui/src/app/strategy_params.rs`, add a dispatch arm in `openmill-ui/src/generate.rs`, and append the name to the `STRATEGIES` list rendered in the operations panel.

Params hygiene:
- `#[serde(default)]` on every field — old saved jobs must keep deserialising.
- Include `direction: CutDirection`, `z_range: ZRange`, `spring_pass: SpringPass` if you want the cross-cutting controls to round-trip through the UI (they're applied uniformly by `apply_common_transforms`).
- Roughing: use `model.stock_aabb()`. Finishing: use `model.aabb`.
- If the strategy honours `stock_to_leave`, add `pub stock_to_leave: f64` and the op-level value is auto-injected by `inject_stock_to_leave` in `generate.rs`.
- Surface strategies that compute step-over should also accept `step_over_cusp_mm: Option<f64>` and resolve via `step_over_mm_for(cusp, fraction, tool)`.

Collision invariants for safe path emission (regression-tested):
- Every `Rapid` must place the tool at `safe_z` — never on the part surface.
- Between cutting passes always emit a horizontal `Rapid` at `safe_z` over the next pass start, then a `LeadIn`. A bare `LeadIn` from `safe_z` to a mid-height point slices diagonally through the part.

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

Helpers worth knowing about:
- `is_indexed_pass(toolpath)` — true when every point shares one tool axis. Use to switch between `G94` feed-per-minute and `G93` inverse-time (or TCPM `G43.4`).
- `spindle_command_for(op)` — returns `"M3"` or `"M4"`; reads the Tapping / Thread Milling `thread_direction` JSON field for left-hand threads.
- For rigid tapping, read `op.params["tap_mode"]` and emit `M29 S<rpm>` in `op_preamble` before the cycle (see `openmill-post/src/fanuc.rs`).

Register in `openmill-post/src/lib.rs`: `pub mod my_post;`, add the name to `POST_PROCESSOR_NAMES`, and a `get_post()` arm.

### Adding a Cross-Cutting Strategy Parameter
Cross-cutting params (Cut Direction, Z Range, Spring Pass, …) live in `openmill-core/src/strategies/transforms.rs` and are applied uniformly by `apply_common_transforms` in `openmill-ui/src/generate.rs`. To add a new one:

1. Define the type + `apply_<thing>(tp, &param)` in `transforms.rs`.
2. Append the field to `CommonStrategyParams` (with `#[serde(default)]`) and call `apply_<thing>(…)` from `apply_common`.
3. Add the same field with `#[serde(default)]` to every strategy `Params` struct that should round-trip the value through the UI (otherwise the UI's serialise-back step drops it).
4. Render the editor in `common_strategy_controls` in `app/strategy_params.rs`.

### Adding a Feature Detector
Detectors live in [`openmill-core/src/feature.rs`](./openmill-core/src/feature.rs) and produce `Vec<Feature>`. The existing detectors (`detect_holes`, `detect_pockets`) are heuristic: classify triangles by normal direction → flood-fill connected components → filter by geometry → emit `FeatureKind` instances. Manual picking goes through `pick_face(mesh, ray, tol_deg) → PickedFace` which raycasts (Möller–Trumbore) and flood-fills coplanar neighbours.

Re-export from `openmill-core/src/lib.rs` (flat module) and wire a button into the features panel in `openmill-ui/src/app/panels.rs`.

### Raycasting & Geometry
Powered by `parry3d`. Used for tool-mesh collision detection, surface raycasting in roughing/finishing strategies, AABB queries, feature detection, and the SDF / deviation-to-target heatmap (`openmill-core/src/sdf.rs`).

### Persistence Conventions
- All fields on `Tool`, `Operation`, `MachineConfig`, and every `…Params` struct must be `#[serde(default)]`. Old `tools/*.json`, `machines/*.json`, and saved jobs must keep loading after schema additions.
- `.omp` project bundles (`openmill-ui/src/bundle.rs`) zip the job + imported mesh + every tool + the machine config into a single portable file.
- Autosave (`openmill-ui/src/autosave.rs`) writes `<job>.autosave.json` on a timer; recovery prompts on the next launch.
- Undo / Redo (`openmill-ui/src/history.rs`) snapshots the live job state on every UI-driven edit.

### Workspace Map
- **`openmill-core`** — geometry, kinematics (`TableTable` IK), `WorkpieceModel`, `StockShape`, strategies + cross-cutting transforms, features, verifier, SDF, tool / feed-speed types, holder collision, leads.
- **`openmill-post`** — `PostProcessor` trait, `LinuxCncPost`, `GrblPost`, `FanucPost`, inverse-time feed math, TCPM `I/J/K` emission.
- **`openmill-sim`** — full-body collision checker and simulation primitives.
- **`openmill-ui`** — `egui` desktop app, `wgpu` viewport, GPU voxel carving, GPU Marching-Cubes iso-surface, ghost-mesh X-ray pipeline, click-to-pick face selection, background generation worker, undo/redo, autosave, `.omp` bundle, setup-sheet HTML export.

---

*Happy Milling!*

# Contributing to OpenMill 🛠️

Thank you for your interest in contributing to OpenMill! We welcome contributions from developers, machinists, and designers of all skill levels.

## 🌈 How to Contribute

### 1. Reporting Bugs
- Use the GitHub Issue Tracker to report bugs.
- Include your OS, GPU model, the strategy and post-processor in use, and a sample STL/3MF (or `.omp` bundle) if the bug is related to geometry, simulation, or G-code output.

### 2. Feature Requests
- Open an issue with the `feature request` label.
- Provide a clear description of the use case and how it would benefit the CAM workflow.

### 3. Pull Requests
- Fork the repository.
- Create a new branch for your feature or bugfix.
- Ensure `cargo build --workspace` succeeds with no new warnings.
- Ensure `cargo test --workspace` passes (note any pre-existing failures in the PR body).
- Run `cargo fmt` before submitting.
- Use the PR template in `.github/PULL_REQUEST_TEMPLATE/template.md`.

## 🏗️ Development Setup

OpenMill is a Rust workspace. To get started:

```bash
# Clone and build
git clone https://github.com/charlatanshost/OpenMill.git
cd OpenMill
cargo build

# Run tests
cargo test --workspace

# Run the UI in debug mode
cargo run -p openmill-ui
```

## 📂 Source Layout

```
openmill-core/src/
  strategies/
    mod.rs                  re-exports every strategy + transforms
    traits.rs               ToolpathStrategy trait
    transforms.rs           CutDirection, ZRange, SpringPass, ThreadDirection,
                            TapMode, step_over_from_cusp (cross-cutting params)
    <strategy>.rs           one file per strategy (e.g. adaptive_clearing.rs)
  toolpath/
    types.rs                ToolpathPoint, MoveType, OperationType, Toolpath
    leads.rs                LeadConfig + apply_leads
  kinematics.rs             table-table IK, MachineConfig, PostConfig
  feature.rs                detect_holes, detect_pockets, pick_face
  verify.rs                 verify_job + per-check helpers
  sdf.rs                    deviation-to-target signed distance field
  model.rs                  WorkpieceModel, StockShape, stock_aabb
  tool.rs                   Tool, ToolShape, ToolHolder, Coolant, FeedSpeedPreset
  job.rs                    Job, Operation, JobSettings, StockDef
  import/                   STL + 3MF readers
  lib.rs                    flat re-exports — keep alphabetised
openmill-post/src/
  traits.rs                 PostProcessor trait + spindle_command_for helper
  feed_rate.rs              joints_for_point, compute_inverse_time_feed_with_kin
  linuxcnc.rs, grbl.rs, fanuc.rs
  lib.rs                    POST_PROCESSOR_NAMES list, get_post(name)
openmill-sim/src/
  collision.rs              full tool + holder collision checker
openmill-ui/src/
  app/
    mod.rs                  top-level App state
    menu.rs, sidebar.rs, panels.rs, tabs.rs
    file_actions.rs         open/save/import flows
    strategy_params.rs      per-strategy parameter editor (one match arm per strategy)
  generate.rs               background generation worker + dispatch
  history.rs                undo/redo
  autosave.rs, bundle.rs    crash-recovery + .omp project zip
  setup_sheet.rs            HTML shop-floor report
  voxel.rs                  WGSL compute-shader voxel carving
  marching_cubes.rs         GPU iso-surface extraction
  viewport.rs               wgpu renderer (camera, grid, mesh, paths, voxels)
  main.rs
```

## 🎯 How to Extend OpenMill

The four most common contributions and the exact files to touch.

### A. Add a new toolpath strategy

1. **Create the strategy file** at `openmill-core/src/strategies/<my_strategy>.rs`. It must implement [`ToolpathStrategy`](./openmill-core/src/strategies/traits.rs):
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
   Conventions for the `Params` struct:
   - Use **`#[serde(default)]`** on every field so old saved jobs keep deserialising.
   - Include the cross-cutting fields if your strategy is a cutting strategy:
     ```rust
     #[serde(default)] pub direction: crate::strategies::CutDirection,
     #[serde(default)] pub z_range:   crate::strategies::ZRange,
     #[serde(default)] pub spring_pass: crate::strategies::SpringPass,
     ```
   - **Roughing**: use `model.stock_aabb()` for scan bounds; clamp cut Z via raycast against `model.mesh`.
   - **Finishing**: use `model.aabb` and project cuts onto the surface.
   - **Stock to leave**: if the strategy honours it, add `pub stock_to_leave: f64` with `#[serde(default)]` — the op-level value will be auto-injected by `inject_stock_to_leave` in `openmill-ui/src/generate.rs`.
   - **Cusp height** (surface finishing only): add `pub step_over_cusp_mm: Option<f64>` and resolve via `crate::strategies::transforms::step_over_mm_for(cusp, fraction, tool)`.

2. **Avoid collision foot-guns** in path emission:
   - Every `Rapid` must place the tool at `safe_z` (above the part / stock).
   - Between passes always emit a horizontal rapid at `safe_z` followed by a `LeadIn`, never a diagonal `LeadIn` from `safe_z` into the part.
   - Cutting Z must be clamped to the part surface (raycast) plus `stock_to_leave`.

3. **Register the strategy**:
   - `openmill-core/src/strategies/mod.rs`: `pub mod my_strategy;` + `pub use my_strategy::{MyStrategy, MyStrategyParams};`
   - `openmill-core/src/lib.rs`: add to the `strategies::{ … }` re-export (keep alphabetised).

4. **Wire it into the UI** (`openmill-ui/src/`):
   - `app/strategy_params.rs`:
     - Add a `"My Strategy" => serde_json::to_value(MyStrategyParams::default()).unwrap(),` line to `default_params_for`.
     - Add a `"My Strategy" => { … }` match arm to `show_strategy_params`. Use the `drag`, `cusp_height_control`, and `common_strategy_controls` helpers.
   - `generate.rs`: add a `"My Strategy" => { … }` arm to `dispatch_strategy`.
   - `app/sidebar.rs` (or wherever `STRATEGIES` lives): append `"My Strategy"` so the user can pick it.

5. **Tests**: add a `#[cfg(test)] mod tests` at the bottom of the strategy file. At minimum verify a non-empty toolpath on a simple mesh and that every `Rapid` lands at `safe_z`.

### B. Add a new post-processor

1. **Create the file** at `openmill-post/src/<my_post>.rs` implementing [`PostProcessor`](./openmill-post/src/traits.rs):
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
   Per-op emission order is documented on the trait. Each `process_toolpath` line must be paired with `Some(point_index)` for cut moves so the G-code terminal can map clicks back to the simulation playhead.
2. **Auto-mode helpers** (see `linuxcnc.rs` and `fanuc.rs`):
   - `is_indexed_pass(toolpath)` returns true when every point shares one tool axis. Use it to switch between `G94` feed-per-minute and `G93` inverse-time (or TCPM `G43.4`).
   - `spindle_command_for(op)` returns `"M3"` / `"M4"` and respects the Tapping / Thread Milling `thread_direction` JSON field.
   - For rigid tapping, read `op.params["tap_mode"]` and emit `M29 S<rpm>` in `op_preamble` before the cycle (see Fanuc post for the pattern).
3. **Register**:
   - `openmill-post/src/lib.rs`: add `pub mod my_post;`, add the name to `POST_PROCESSOR_NAMES`, and add a match arm to `get_post()`.
4. **Tests**: each post in the workspace has unit tests for header / footer / tool change / continuous-vs-indexed output. Add the equivalents — at minimum a sequence-number / sync-feed / canned-cycle invariant.

### C. Add a feature detector

1. Add a `pub fn detect_<thing>(mesh: &TriMesh) -> Vec<Feature>` in `openmill-core/src/feature.rs`. Existing detectors classify triangles by normal, flood-fill connected components, then filter by geometry (see `detect_holes` / `detect_pockets`).
2. Add a variant to `FeatureKind` if the new thing doesn't fit.
3. Re-export from `openmill-core/src/lib.rs` (it's a flat re-export module).
4. Wire a button into `section_features` in `openmill-ui/src/app/panels.rs` (or wherever the feature panel lives). Reuse the existing `→ Drilling` / `→ Pocket Clearing` button pattern to assign the feature to an op.

### D. Add a cross-cutting strategy parameter

Cross-cutting params (Cut Direction, Z Range, Spring Pass, etc.) live in `openmill-core/src/strategies/transforms.rs` and are applied uniformly by `apply_common_transforms` in `openmill-ui/src/generate.rs`.

To add a new one:
1. Add the parameter type + `apply_<thing>(tp, &param)` in `transforms.rs`.
2. Add the field to `CommonStrategyParams` (with `#[serde(default)]`).
3. Append `apply_<thing>(tp, &common.<thing>)` to `apply_common`.
4. Add the same field with `#[serde(default)]` to every strategy `Params` struct that wants the UI to round-trip it (because `strategy_params.rs` deserialises the strategy struct and re-serialises it — anything not in the struct gets dropped).
5. Render the editor in `common_strategy_controls` in `app/strategy_params.rs`.

## ✅ Coding Standards

- **Vanilla Rust**, `nalgebra` for math, `parry3d` for geometry queries, `wgpu` for GPU.
- Keep `openmill-core` free of UI / `wgpu` / `egui` dependencies.
- All new fields on persisted types (`Tool`, `Operation`, `MachineConfig`, every `…Params` struct) must be `#[serde(default)]` so existing `tools/*.json`, `machines/*.json`, and saved jobs keep loading.
- Roughing strategies must use `model.stock_aabb()` for scan bounds, not `model.aabb`.
- Default to no comments. Only comment a **why** that the code can't express (a constraint, a workaround, a non-obvious invariant). Don't write comments that just restate what the next line does.
- Don't add backwards-compat shims, `// removed` comments, or unused parameters with leading underscores in fresh code — delete what's no longer wanted.
- Add a unit test next to anything new that has a checkable invariant. Regression tests for fixed bugs are especially welcome — name them after the bug (`every_rapid_starts_at_safe_z`, `rigid_tap_emits_m29`, etc.).

## 🗺️ Roadmap

### ✓ Done
- 12 → 13 strategies including stock-aware roughing (Adaptive, Multi-Axis Roughing, 3+2, 4+1, Pocket, Contour Parallel) and 5-axis finishing (Surface Normal, Swarf, Geodesic, Pencil).
- Cross-cutting cut-direction + Z-range + spring-pass transforms applied uniformly across every strategy.
- Cusp-height step-over for surface strategies.
- Drilling cycle types (G81 / G82 / G83 / G73 / G85) with peck depth, dwell, chipbreak retract, break-through.
- Thread direction (RH / LH) → M3 / M4 spindle selection.
- Rigid (M29) vs floating tap; internal vs external thread mill.
- G93 / G94 auto-mode selection per toolpath; inverse-time feed math near singularities.
- Three post-processors: LinuxCNC, GRBL, Fanuc (TCPM `G43.4`, sequence numbers, safe-start block).
- Per-tool feed-and-speed presets, per-op coolant + custom G-code, tool-change G-code.
- Stock-aware roughing and op-level `stock_to_leave` auto-injected into strategy params.
- Lead-in / lead-out (Plunge / Ramp / Arc) applied as cross-cutting post-pass.
- Plan / Simulation view split with translucent ghost-mesh X-ray overlay.
- GPU voxel carving (full tool body, gradient-shaded normals) + GPU Marching-Cubes iso-surface.
- Feature pipeline: auto-detect Holes / Pockets and 🎯 click-to-pick face mode with auto-classification.
- Stock dimensions in mm or decimal inch, model position editor, position persisted to job file.
- Job save / load (JSON), `.omp` bundle (zip), autosave + recovery, Undo / Redo.
- HTML setup sheet export (model preview, ops table, tools, time estimate).
- Tool-holder collision detection in sim.
- Travel envelope visualisation in viewport.
- Drag-reorder operations (⬆/⬇), per-op + job-wide cycle-time estimates.
- Pre-export verifier (axis limits, rotary range, undefined feeds, gouges, holder collisions) — export auto-runs and refuses on errors.
- Background generation worker (long ops don't freeze the UI).
- SDF-based deviation-to-target heatmap (Tier 4 verify).

### 🛠 In progress / Next up

**Tier 1 — Strategy / param coverage**
- [ ] EntryMode (Plunge / Ramp / Helical) on Adaptive + Pocket Clearing.
- [ ] Rest-machining (use voxel grid to mask the next op).
- [ ] True trochoidal medial-axis paths in `AdaptiveClearing` (current is a stock-aware raster).
- [ ] True heat-method geodesic solver for `GeodesicParallel` (currently uses Z-height as a distance proxy).

**Tier 2 — Quality of life**
- [ ] Setup / WCS management (multi-side jobs in one job file).
- [ ] Fixture visualisation + collision avoidance.
- [ ] Material-removal stats in sim (volume + MRR).
- [ ] Operation templates (save reusable patterns).

**Tier 3 — Strategy / detection coverage**
- [ ] Inclined / counter-bore / threaded hole detection.
- [ ] Non-circular pocket detection (rectangle / polygon footprint).
- [ ] Edge detection for chamfer-mill ops.
- [ ] DXF / 2D profile import.
- [ ] Probing strategy (G38.2 cycles).

**Tier 4 — Polish**
- [ ] Tool import from Fusion / FreeCAD JSON.
- [ ] Spindle-load / chip-thinning warnings.
- [ ] 5-axis tool-axis smoothing (B-spline filter for jerk reduction).
- [ ] Toolpath edit (delete a single pass without regenerating).
- [ ] Multi-toolpath ops (rough + semi-finish + finish in one op).

**Extensibility (parallel track)**
- [ ] Mesh-based stock support (`StockDef::MeshFile`).
- [ ] S-curve acceleration in post-processors.
- [ ] Additional post-processors (Mach3/4, Centroid).

## ⚖️ Code of Conduct
Please be respectful and constructive in all interactions. We aim to build a welcoming community for everyone interested in open-source manufacturing.

---

*Thank you for helping us make OpenMill the best open-source CAM tool!*

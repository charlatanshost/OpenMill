# Contributing to OpenMill 🛠️

Thank you for your interest in contributing to OpenMill! We welcome contributions from developers, machinists, and designers of all skill levels.

## 🌈 How to Contribute

### 1. Reporting Bugs
- Use the GitHub Issue Tracker to report bugs.
- Include your OS, GPU model, and a sample STL/3MF file if the bug is related to geometry or simulation.

### 2. Feature Requests
- Open an issue with the `feature request` label.
- Provide a clear description of the use case and how it would benefit the CAM workflow.

### 3. Pull Requests
- Fork the repository.
- Create a new branch for your feature or bugfix.
- Ensure all tests pass with `cargo test --workspace`.
- Run `cargo fmt` before submitting.
- Provide a detailed description of your changes in the PR.

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

### Coding Standards
- We use **Vanilla Rust** with a focus on high-performance math (`nalgebra`) and GPU interaction (`wgpu`).
- Avoid adding heavy dependencies unless necessary.
- Keep the `openmill-core` crate free of UI logic.
- All new fields on persisted types (`Tool`, `Operation`, `MachineConfig`, etc.) should be `#[serde(default)]` so existing `tools/*.json` and `machines/*.json` files keep loading.
- Roughing strategies must use `model.stock_aabb()` for scan bounds, not `model.aabb`.

### Where to Extend
- **New strategy:** implement `ToolpathStrategy` in `openmill-core/src/strategies/`, register it in `mod.rs`, add it to `STRATEGIES` in `openmill-ui/src/app.rs`, wire up the dispatch arm and a params editor in `show_strategy_params`. If the strategy honours `stock_to_leave`, add a `pub stock_to_leave: f64` field with `#[serde(default)]` and the op-level value will be auto-injected at generate time.
- **New post-processor:** implement `PostProcessor` in `openmill-post`, add it to `POST_PROCESSOR_NAMES` and `get_post()` in `openmill-post/src/lib.rs`. Follow the per-op emission contract documented on the `PostProcessor` trait.
- **New feature detector:** add a free function in `openmill-core/src/feature.rs` that takes `&TriMesh` and returns `Vec<Feature>`. Re-export from `lib.rs`. Wire a button into `section_features` in `openmill-ui/src/app.rs`.
- **New verifier check:** add a `fn check_*` to `openmill-core/src/verify.rs` and call it from `verify_job`. Use `Issue::err` for blocking issues and `Issue::warn` for advisories.

## 🗺️ Roadmap

### ✓ Done
- 4+1 indexed milling.
- G93 / G94 auto-mode selection per toolpath; inverse-time feed math near singularities.
- Per-tool feed-and-speed presets, per-op coolant + custom G-code.
- Stock-aware roughing (3+2, 4+1, Adaptive Clearing, Contour Parallel) and op-level `stock_to_leave` auto-injected into strategy params.
- Plan / Simulation view split with translucent ghost-mesh X-ray overlay.
- Full tool-body voxel carving with gradient-shaded surface normals.
- Adaptive Clearing implemented as stock-aware raster (no longer panics).
- Pocket Clearing strategy + auto pocket detection.
- Tapping (rigid `feed = rpm × pitch`) and Thread Milling (helical) strategies.
- Feature pipeline: auto-detect Holes / Pockets and 🎯 click-to-pick face mode with auto-classification.
- Stock dimensions in mm or decimal inch, model position editor, position persisted to job file.
- Job open/load round-trip (model + position + ops + features + tools + machine).
- Tool-holder collision detection in sim.
- Drag-reorder operations (⬆/⬇).
- Pre-export verifier (axis limits, rotary range, undefined feeds, gouges, holder collisions); export auto-runs and refuses on errors.
- Estimated machining time per op + job total.

### 🛠 In progress / Next up

**Tier 2 — Important quality-of-life**
- [ ] Lead-in / lead-out arcs for finishing.
- [ ] Helical / ramp entry for plunges.
- [ ] Setup / WCS management (multi-side jobs in one job file).
- [ ] Fixture visualization + collision avoidance.
- [ ] Material-removal stats in sim (volume + MRR).
- [ ] Undo / Redo.

**Tier 3 — Strategy / detection coverage**
- [ ] Inclined / counter-bore / threaded hole detection.
- [ ] Non-circular pocket detection (rectangle / polygon footprint).
- [ ] Edge detection for chamfer-mill ops.
- [ ] DXF / 2D profile import.
- [ ] Probing strategy (G38.2 cycles).
- [ ] Climb-only raster option (no zigzag).

**Tier 4 — Polish**
- [ ] Material library UI.
- [ ] Tool import from Fusion / FreeCAD JSON.
- [ ] Operation templates (save reusable patterns).
- [ ] Spindle-load / chip-thinning warnings.
- [ ] 5-axis tool-axis smoothing (B-spline filter for jerk reduction).
- [ ] Toolpath edit (delete a single pass without regenerating).
- [ ] Multi-toolpath ops (rough + semi-finish + finish in one op).

**Strategy depth (parallel track)**
- [ ] True trochoidal medial-axis paths in `AdaptiveClearing` (current is a stock-aware raster).
- [ ] True heat-method geodesic solver for `GeodesicParallel` (currently uses Z-height as a distance proxy).
- [ ] Mesh-based stock support (`StockDef::MeshFile`).
- [ ] S-curve acceleration in post-processors.
- [ ] Additional post-processors (Mach3/4, Centroid, Haas).

## ⚖️ Code of Conduct
Please be respectful and constructive in all interactions. We aim to build a welcoming community for everyone interested in open-source manufacturing.

---

*Thank you for helping us make OpenMill the best open-source CAM tool!*

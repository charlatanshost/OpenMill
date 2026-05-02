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
- **New strategy:** implement `ToolpathStrategy` in `openmill-core/src/strategies/`, register it in `mod.rs`, add it to `STRATEGIES` in `openmill-ui/src/app.rs`, wire up the dispatch arm and a params editor in `show_strategy_params`.
- **New post-processor:** implement `PostProcessor` in `openmill-post`, add it to `POST_PROCESSOR_NAMES` and `get_post()` in `openmill-post/src/lib.rs`. Follow the per-op emission contract documented on the `PostProcessor` trait.

## 🗺️ Roadmap

**Done**
- [x] 4+1 indexed milling.
- [x] G93 / G94 auto-mode selection per toolpath.
- [x] Per-tool feed-and-speed presets, per-op coolant + custom G-code.
- [x] Stock-aware roughing (3+2, 4+1, Adaptive Clearing, Contour Parallel).
- [x] Plan / Simulation view split with translucent ghost-mesh collision overlay.

**Next up**
- [ ] True trochoidal medial-axis paths in `AdaptiveClearing` (the current implementation is a stock-aware raster).
- [ ] True heat-method geodesic solver for `GeodesicParallel` (currently uses Z-height as a distance proxy).
- [ ] Mesh-based stock support (`StockDef::MeshFile`).
- [ ] Tool-holder collision detection (currently only the cutter is checked).
- [ ] Hole auto-detection from imported mesh (currently manual hole list in 5-Axis Drilling).
- [ ] Job save/load (`*.omj`) — full round-trip of model + ops + tools + machine.
- [ ] S-curve acceleration in post-processors.
- [ ] Additional post-processors (Mach3/4, Centroid, Haas).

## ⚖️ Code of Conduct
Please be respectful and constructive in all interactions. We aim to build a welcoming community for everyone interested in open-source manufacturing.

---

*Thank you for helping us make OpenMill the best open-source CAM tool!*

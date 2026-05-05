# OpenMill 🚀

**OpenMill** is a high-performance, open-source 5-axis CAM (Computer-Aided Manufacturing) platform designed for hobbyist and professional CNC machines. Built entirely in Rust, it provides a robust engine for complex toolpath generation, real-time material removal simulation, and flexible post-processing.

![OpenMill UI](https://raw.githubusercontent.com/charlatanshost/OpenMill/main/docs/assets/ui_hero.png) *(Placeholder for actual screenshot)*

## ✨ Key Features

### 🛠️ Toolpath Strategies (12)
OpenMill ships strategies that scale from 3-axis indexed work to simultaneous 5-axis:
- **3+2 Indexed**: Lock both rotary axes at fixed angles and run a stock-aware 3-axis raster in the resulting tilted plane.
- **4+1 Indexed**: Lock A, step C between passes — ideal for wrapping operations around a part or reaching multiple faces in a single setup.
- **Adaptive Clearing** *(roughing)*: Stock-aware layered raster with engagement-angle-derived step-over.
- **Pocket Clearing** *(roughing)*: Bounded raster clear of detected pocket features, layer-by-layer with raycast Z-clamping.
- **Contour Parallel** *(finishing)*: Z-level waterline that scans from the stock top down through the part.
- **Surface Normal 5-Axis** *(finishing)*: Parallel / Scallop / Spiral / Rotary patterns with lead-angle and side-tilt control.
- **Swarf 5-Axis** *(finishing)*: Tool flank aligned with ruled surfaces (turbine vanes, tapered walls).
- **Geodesic Parallel** *(finishing)*: Surface-aware paths with constant step-over on complex geometry.
- **5-Axis Pencil Tracing** *(finishing)*: Concave-corner cleanup with bisecting tool tilt.
- **5-Axis Drilling**: Tool axis automatically aligned to each hole's drill axis.
- **Tapping**: Rigid-tap cycles with synchronised feed (`feed = rpm × pitch`) and optional pecking.
- **Thread Milling**: Helical interpolation with climb / conventional toggle and configurable segments per revolution.

All roughing strategies are **stock-aware** — they plan from the stock envelope (part AABB grown by margin, or cylinder bounds), not just the part shape, so the outer stock margin actually gets cleared.

Every operation carries a **stock-to-leave** value (mm) that's automatically injected into the strategy params at generate time, so you can rough with `0.3 mm` and follow with a finishing op at `0.0` without juggling the same number in two places.

### 🔍 Feature Recognition & Manual Picking
A first-class **Features** pipeline that turns mesh geometry into op inputs:
- **Auto-detect Holes**: scans the mesh for vertical cylindrical walls and emits hole features (position, axis, diameter, depth).
- **Auto-detect Pockets**: finds horizontal floor faces with open clearance above and emits pocket features (centre, floor Z, top Z, radius, area).
- **🎯 Pick Face Mode**: click any face in the viewport — the picker raycasts the mesh, flood-fills coplanar neighbours, auto-classifies the cluster (pocket / hole / generic flat), and lights it up amber.
- **Per-feature assignment**: each row exposes **→ Drilling / → Tapping / → Thread Mill / → Pocket Clearing** to push the feature onto the matching op type, with bulk-assign shortcuts for "all detected holes → drilling".

### 🧰 Professional Tool Management
A persistent Tool Library system with full feed-and-speed planning:
- **Persistent Storage**: Tools auto-save to `tools/tool_<id>.json` and load on startup.
- **Rich Geometry**: Flat, Ball, Bull-Nose, Chamfer, Drill, Tapered, Lollipop, Dovetail, and Thread-Mill shapes — each with proper auto-scaling 2D profile preview.
- **Feed/Speed Presets**: Save named presets per tool (RPM, cutting feed, plunge feed, coolant). One click applies a preset to the active operation.
- **Starter Presets**: Each new tool is auto-seeded with starter presets for common materials (6061 Aluminum, Mild Steel, HDPE, Hardwood, Brass) computed from the tool's diameter and flute count.
- **Coolant Modes**: None / Mist (M7) / Flood (M8) / Mist+Flood / Through-spindle (M88), wired through to G-code emission.
- **Tool-Change G-code**: Per-tool free-form G-code injected after every M6 (probing routines, length offsets, etc.).
- **Tool-Holder Collision**: Define a holder profile and the simulator checks the holder envelope against the part mesh on every step — catches the most common 5-axis crash mode.

### 🏗️ Machine Library
Define your machine's physical constraints and post-processing defaults:
- **Travel Limits**: Soft limits for X, Y, Z linear axes and A, C rotary axes.
- **Post-Processor Mapping**: Pick LinuxCNC or GRBL per machine; configure program number, work-offset (G54–G59), and units.
- **Pivot Configuration**: Customizable trunnion pivot offsets for table-table kinematics.
- Configurations persist as `machines/<name>.json`.

### 📐 Stock & Model Setup
- **Absolute stock dimensions** in mm or **decimal inches** (1.5 in, 2.375 in — not fractions). Internal storage is always mm; the unit toggle is display-only.
- **Stock auto-tracks the part**: the stock envelope is derived from the part AABB grown by per-axis margin (or cylinder bounds) and updates automatically when the model moves.
- **Model position editor**: translate the part anywhere in the work area with X/Y/Z drag values, **⟲ Reset** to imported origin, **⌖ Centre on origin** to drop the AABB centroid at (0,0,0). Position persists across save/load.

### 🎮 Real-Time Simulation
Two distinct viewport modes, switchable from the bottom panel:
- **🧭 Plan View**: Solid part mesh visible for path setup and toolpath review. No carving runs.
- **🛠 Simulation View**: Hides the part mesh and shows the **voxel stock** as it gets carved by the tool body's full swept volume. The part is overlaid as a translucent red **X-ray ghost** so collisions between tool and finished part are visible even when stock material is still in front.
- **GPU compute carving**: WGSL compute shader sweeps the full tool body (not just the tip) along the tool axis at every move, with gradient-shaded surface normals so cavities and step-down terraces are clearly visible.
- **Live tool-mesh + holder collision detection** via `parry3d` raycasting and the holder envelope check; collisions render as red markers in 3D.

### ✓ Verification & Estimates
- **Pre-export verifier** — auto-runs before every G-code export and refuses to write on errors. Checks travel limits (X/Y/Z), rotary range (A/C via IK), undefined / implausibly-high feeds, cuts below part floor, and tool-holder collisions sampled along the path.
- **Estimated machining time** per operation and a job-wide total, shown in the Operations panel as `Hh Mm Ss`.

### 📄 Flexible Post-Processing
- Native support for **LinuxCNC** (RS274NGC) and **GRBL**.
- **Auto-detected mode**: indexed passes (constant tool axis) emit `G94` feed-per-minute; continuous 5-axis passes emit `G93` inverse-time so all five axes stay synchronised.
- **Per-operation emission**: spindle on, coolant on, free-form op G-code → toolpath → coolant off (`M9`).
- **Tool-change safety**: stops spindle (`M5`), stops coolant (`M9`), then `M6 T#` (LinuxCNC) or `M0` pause (GRBL), then runs any tool-level setup G-code.
- **Inverse-time feed near singularities**: rotary-axis velocity clamped to a configurable max so near-vertical-axis moves slow naturally instead of commanding infinite C-axis velocity.
- **Custom post-processor API** for extending to other controllers via the `PostProcessor` trait.

### 💾 Job Save / Load
Round-trip a full job (stock, model position, tools, operations, features, machine config) as a single JSON file. Reopen the job later and everything is back: model in the right position, ops in the right order, presets attached.

## 🏗️ Architecture

| Crate | Responsibility |
|---|---|
| [`openmill-core`](./openmill-core) | Geometry, kinematics, model + stock + position, **strategies**, **features**, **verifier**, toolpath types. |
| [`openmill-sim`](./openmill-sim) | Material-removal simulation primitives. |
| [`openmill-post`](./openmill-post) | G-code dialects (LinuxCNC, GRBL), inverse-time feed math, per-op preamble/postamble. |
| [`openmill-ui`](./openmill-ui) | Desktop GUI built with `egui` + `wgpu`; voxel carving & raymarched stock rendering, ghost-mesh X-ray, click-to-pick. |

## 🚀 Getting Started

### Prerequisites
- [Rust](https://rustup.rs/) (latest stable)
- A GPU with Vulkan, Metal, or DX12 support (for simulation rendering)

### Build and Run
```bash
# Clone the repository
git clone https://github.com/charlatanshost/OpenMill.git
cd OpenMill

# Build the workspace
cargo build --release

# Run the UI
cargo run -p openmill-ui --release
```

On Windows you can also use the included `launch.bat`.

### First-Job Walkthrough
1. **File → Import Model** an STL or 3MF.
2. (Optional) **Position** the model in the work area or **⌖ Centre on origin**.
3. Configure stock dimensions (mm or decimal inch) in the **Stock** section.
4. Open the **Tools** tab and confirm the tool library; tweak feed/speed presets if needed.
5. In **Features**, click **🔍 Holes** / **🔍 Pockets** to auto-detect, or toggle **🎯 Pick Face Mode** to click features manually.
6. Add an operation: pick a strategy, pick a tool, optionally apply a feed/speed preset, set **Stock to leave** for roughing.
7. For feature-driven ops (Drilling, Tapping, Thread Milling, Pocket Clearing), use the **→** buttons in the Features panel to assign features to the selected op.
8. Click **Generate Toolpath** — the path renders in the viewport in Plan view.
9. Switch to **🛠 Simulation** view, press Play, and watch the stock get carved (with the part visible as a translucent ghost).
10. **Verify → ✓ Verify all toolpaths** (or just hit Export — verification runs automatically).
11. **File → Save Job...** to keep the setup, or **Export → Machine Default** to write the `.nc` / `.ngc` / `.gcode`.

## 🤝 Contributing
Contributions are welcome. See [CONTRIBUTING.md](./CONTRIBUTING.md) for the development workflow and roadmap.

## ⚖️ License
Licensed under either of [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE) at your option.

# OpenMill 🚀

**OpenMill** is a high-performance, open-source 5-axis CAM (Computer-Aided Manufacturing) platform designed for hobbyist and professional CNC machines. Built entirely in Rust, it provides a robust engine for complex toolpath generation, real-time material removal simulation, and flexible post-processing.

![OpenMill UI](https://raw.githubusercontent.com/charlatanshost/OpenMill/main/docs/assets/ui_hero.png) *(Placeholder for actual screenshot)*

## ✨ Key Features

### 🛠️ Toolpath Strategies (13)
OpenMill ships strategies that scale from 3-axis indexed work to simultaneous 5-axis:
- **3+2 Indexed**: Lock both rotary axes at fixed angles and run a stock-aware 3-axis raster in the resulting tilted plane.
- **4+1 Indexed**: Lock A, step C between passes — ideal for wrapping operations around a part or reaching multiple faces in a single setup.
- **Adaptive Clearing** *(roughing)*: Stock-aware layered raster with engagement-angle-derived step-over.
- **Simultaneous 5-Axis Roughing** *(roughing)*: Stock-aware layered raster that tilts the tool axis along the local surface normal (capped by a `max_tilt_deg` and an optional `lead_angle`).
- **Pocket Clearing** *(roughing)*: Bounded raster clear of detected pocket features, layer-by-layer with raycast Z-clamping.
- **Contour Parallel** *(finishing)*: Z-level waterline that scans from the stock top down through the part.
- **Surface Normal 5-Axis** *(finishing)*: Parallel / Scallop / Spiral / Rotary patterns with lead-angle and side-tilt control.
- **Swarf 5-Axis** *(finishing)*: Tool flank aligned with ruled surfaces (turbine vanes, tapered walls).
- **Geodesic Parallel** *(finishing)*: Surface-aware paths with constant step-over on complex geometry.
- **5-Axis Pencil Tracing** *(finishing)*: Concave-corner cleanup with bisecting tool tilt.
- **5-Axis Drilling**: Tool axis automatically aligned to each hole's drill axis. Picks per-hole canned cycle (Drill `G81`, Counter-bore `G82`, Peck `G83`, Chip-break `G73`, Bore `G85`) with peck depth, dwell, chipbreak retract, and break-through distance.
- **Tapping**: Rigid (`M29 S<rpm>` on Fanuc/Haas) or floating tap holder, right- or left-hand threads (M3/M4), synchronised feed (`feed = rpm × pitch`), optional pecking.
- **Thread Milling**: Helical interpolation, internal/external, right- or left-hand, climb/conventional, configurable segments per revolution.

All roughing strategies are **stock-aware** — they plan from the stock envelope (part AABB grown by margin, or cylinder bounds), not just the part shape, so the outer stock margin actually gets cleared.

Every operation carries a **stock-to-leave** value (mm) that's automatically injected into the strategy params at generate time, so you can rough with `0.3 mm` and follow with a finishing op at `0.0` without juggling the same number in two places.

### 🎚️ Cross-cutting strategy controls
Three knobs that every cutting strategy honours, applied as post-pass transforms by `apply_common_transforms` so you don't have to wire them into each strategy:
- **Cut Direction** — `Climb` (default), `Conventional` (reverses every cutting block), or `Either` (no change).
- **Z Range** — optional inclusive top / bottom Z limits. Cutting moves outside the range are dropped; rapids and retracts are kept so the tool can still travel.
- **Spring pass** — re-cuts the last "pass unit" (suffix from the last rapid through end of toolpath) at a configurable feed fraction (default 0.5×), cleaning up cutter deflection on finishing passes.

Surface strategies (3+2, 4+1, Surface Normal 5-Axis, Geodesic) also accept a **cusp height (mm)** override that derives step-over from the tool's tip radius (`2·√(2Rh − h²)` for ball / bull-nose, with a flat-tool fallback).

### ⬇️ Lead-In / Lead-Out
Per-operation lead config attaches a `Plunge` / `Ramp` / `Arc` lead-in and matching lead-out to every cutting block. Applied as a post-pass to whatever the strategy emitted, so it works uniformly across all 13 strategies.

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
- Native support for **LinuxCNC** (RS274NGC), **GRBL**, and **Fanuc** (Fanuc/Haas/Brother family).
- **Auto-detected mode**: indexed passes (constant tool axis) emit `G94` feed-per-minute; continuous 5-axis passes emit `G93` inverse-time (LinuxCNC) or **TCPM `G43.4`** with tool-axis `I/J/K` words (Fanuc) so all five axes stay synchronised.
- **Per-operation emission**: spindle on (`M3`/`M4` based on thread hand), coolant on, free-form op G-code → toolpath → coolant off (`M9`).
- **Rigid tapping**: Fanuc post emits `M29 S<rpm>` ahead of the tapping move when the op selects `Rigid` mode.
- **Tool-change safety**: stops spindle (`M5`), stops coolant (`M9`), then `M6 T#` (LinuxCNC), `T# M6` + `G43.4 H#` + `M01` operator-verify (Fanuc), or `M0` pause (GRBL); each followed by the tool's own setup G-code.
- **Sequence numbers** (`N10`, `N20`, …) on every motion block in the Fanuc post for operator hand-edits.
- **Inverse-time feed near singularities**: rotary-axis velocity clamped to a configurable max so near-vertical-axis moves slow naturally instead of commanding infinite C-axis velocity.
- **Custom post-processor API** for extending to other controllers via the `PostProcessor` trait.

### 💾 Save, Load, Bundle, Autosave, Undo
- **Job JSON** — round-trip the entire job (stock, model position, tools, operations, features, machine config) as a single file. Existing JSON files keep loading when new fields are added because every persisted field is `#[serde(default)]`.
- **`.omp` project bundle** — single-file ZIP containing the job, the imported mesh, every tool, and the chosen machine config. Drop the file anywhere and reopen.
- **Autosave** — periodic write to `<job>.autosave.json` so a crash or hang doesn't lose work; offers to recover on the next launch.
- **Undo / Redo** — `Ctrl+Z` / `Ctrl+Y` over the live job state.
- **Setup sheet** — HTML report (model preview, ops table, tools, machining time estimate) generated from any job for shop-floor handoff.

## 🏗️ Architecture

| Crate | Responsibility |
|---|---|
| [`openmill-core`](./openmill-core) | Geometry, kinematics (`TableTable` IK), model + stock + position, **strategies** (incl. cross-cutting transforms), **features**, **verifier**, **SDF** (deviation-to-target), toolpath types + leads. |
| [`openmill-sim`](./openmill-sim) | Material-removal simulation primitives, full-body tool + holder collision checker (`parry3d`). |
| [`openmill-post`](./openmill-post) | G-code dialects (LinuxCNC, GRBL, Fanuc), inverse-time feed math, TCPM, per-op preamble/postamble. |
| [`openmill-ui`](./openmill-ui) | Desktop GUI built with `egui` + `wgpu`; GPU voxel carving + GPU Marching-Cubes iso-surface, ghost-mesh X-ray, click-to-pick, background generation worker, undo/redo, autosave, `.omp` bundles, HTML setup sheets. |

### Source layout (where to add things)
```
openmill-core/src/
  strategies/          one .rs file per strategy + transforms.rs (cross-cutting params)
  toolpath/            ToolpathPoint, MoveType, LeadConfig
  kinematics.rs        table-table IK and machine config
  feature.rs           hole / pocket / face detectors
  verify.rs            pre-export checks
  sdf.rs               deviation-to-target field
openmill-post/src/
  traits.rs            PostProcessor trait + shared helpers (spindle direction)
  linuxcnc.rs, grbl.rs, fanuc.rs    one file per dialect
openmill-ui/src/
  app/                 top-level UI split across menu, sidebar, panels, tabs, strategy_params
  generate.rs          background generation worker + dispatch
  history.rs           undo/redo stack
  autosave.rs, bundle.rs, setup_sheet.rs
  voxel.rs, marching_cubes.rs, viewport.rs    wgpu rendering
```

Full developer extension guide is in [CONTRIBUTING.md](./CONTRIBUTING.md) and [GUIDE.md §11](./GUIDE.md).

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

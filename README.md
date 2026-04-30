# OpenMill 🚀

**OpenMill** is a high-performance, open-source 5-axis CAM (Computer-Aided Manufacturing) platform designed for hobbyist and professional CNC machines. Built entirely in Rust, it provides a robust engine for complex toolpath generation, real-time material removal simulation, and flexible post-processing.

![OpenMill UI](https://raw.githubusercontent.com/charlatanshost/OpenMill/main/docs/assets/ui_hero.png) *(Placeholder for actual screenshot)*

## ✨ Key Features

### 🛠️ Advanced 5-Axis Strategies
OpenMill supports a wide range of sophisticated toolpath strategies, transitioning seamlessly from 3-axis to simultaneous 5-axis operations:
- **Swarf Milling**: Align the tool flank with ruled surfaces for high-efficiency finishing.
- **Surface Normal Finishing**: Patterns (Parallel, Scallop, Spiral) that dynamically tilt the tool to stay normal to the part surface.
- **Geodesic Parallel**: Surface-aware toolpaths that maintain constant step-over on complex, organic geometries.
- **4+1 Indexed Milling**: Simplified multi-axis workflow using local coordinate systems for traditional rastering.
- **5-Axis Pencil Tracing**: Automated cleanup of concave corners with bisecting tool tilt.
- **5-Axis Drilling**: Re-orient the tool axis normal to hole entries for precise deep-hole cycles.

### 🧰 Professional Tool Management
A comprehensive Tool Library system featuring:
- **Persistent Storage**: Automatic saving and loading of tools from a JSON-based library.
- **Rich Geometry Support**: Specialized parameters for Flat, Ball, Bull-nose, Chamfer, and Drill shapes.
- **Visualizer**: Real-time 2D silhouette rendering of tool profiles including tapers and tips.

### 🏗️ Machine Library
Define your machine's physical constraints and post-processing defaults:
- **Travel Limits**: Soft limits for linear (X, Y, Z) and rotary (A, C) axes.
- **Post-Processor Mapping**: Link specific machines to their preferred controllers (LinuxCNC, GRBL, etc.).
- **Pivot Configuration**: Customizable trunnion pivot offsets for table-table kinematics.

### 🎮 Real-Time Simulation
- **Voxel-Based Material Removal**: Watch material disappear in real-time as the tool moves.
- **Collision Detection**: Advanced checking against part, fixture, and machine limits using `parry3d`.
- **Volumetric Rendering**: High-performance visualization powered by `wgpu`.

### 📄 Flexible Post-Processing
- Native support for **LinuxCNC** and **GRBL**.
- Support for **RTCP** (Rotation Tool Center Point) and table-table kinematics.
- Custom post-processor API for extending to other machine controllers.

## 🏗️ Architecture

| Crate | Responsibility |
|---|---|
| [`openmill-core`](./openmill-core) | Geometric queries, toolpath generation, kinematics, and core traits. |
| [`openmill-sim`](./openmill-sim) | Material removal simulation and volumetric collision detection. |
| [`openmill-post`](./openmill-post) | G-code generation and machine-specific translation. |
| [`openmill-ui`](./openmill-ui) | Modern GUI built with `egui` and `wgpu`. |

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

## 🤝 Contributing
Contributions are welcome! Please feel free to submit a Pull Request.

## ⚖️ License
Licensed under either of [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE) at your option.

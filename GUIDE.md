# OpenMill User Guide & Technical Manual

Welcome to the comprehensive guide for OpenMill. This document covers the core functionalities, advanced strategies, and technical implementation details of the platform.

---

## 1. 5-Axis Toolpath Strategies

OpenMill provides a variety of 5-axis strategies tailored for different machining requirements.

### Swarf Milling (Flank Machining)
Ideal for ruled surfaces like turbine blades or tapered walls.
- **How it works**: The algorithm slices the mesh at top and bottom Z levels to find "rulings." The side of the cutter is then aligned with these rulings.
- **Parameters**:
    - `Z Top / Z Bottom`: Define the axial extent of the wall.
    - `Step-down`: If specified, performs multiple axial passes down the ruling.

### Surface Normal Finishing
A versatile simultaneous 5-axis strategy.
- **Patterns**:
    - `Parallel`: Raster passes across the surface.
    - `Scallop`: Concentric loops maintaining constant cusp height for uniform finish.
    - `Spiral`: Continuous spiral path to avoid lead-in/out marks.
- **Lead & Tilt Angles**:
    - `Lead Angle`: Tilts the tool forward in the direction of travel.
    - `Tilt Angle`: Tilts the tool sideways.
    - *Tip*: Use a small lead angle with ball-end mills to avoid cutting with the zero-surface-speed tip.

### Geodesic Parallel
Maintains a constant distance along the surface from a boundary.
- **Benefit**: Unlike world-projected paths, geodesic paths do not "bunch up" on steep walls, ensuring a perfectly uniform surface finish across complex shapes.

### 5-Axis Pencil Tracing
- **Detection**: Automatically identifies concave corners where the dihedral angle between faces is sharper than a threshold.
- **Execution**: Traces the corner with a ball-end mill, tilting the tool to bisect the corner angle for optimal reach.

### 5-Axis Drilling
- **Orientation**: Automatically aligns the tool axis with the normal of the hole entry face.
- **Safety**: Includes rapid retracts and multi-axis reorientation between hole locations.

---

## 2. Machine & Tool Libraries

The Tool Library is central to the CAM workflow.

### Machine Library
Define and save your machine's physical characteristics, including Axis Limits, Kinematics (pivot offsets), and default Post-Processor. Configurations are persisted in the `/machines` directory.

### Tool Library
- **Persistence**: Tools are stored in the `/tools` directory as individual JSON files. The application automatically loads these on startup.
- **Creation**: New tools can be added with specific shapes (Flat, Ball, Bull, etc.).

### Tool Parameters & Types
OpenMill supports a wide range of tool geometries:
- **Flat End Mill**: Standard for 3-axis roughing and floor finishing.
- **Ball End Mill**: Essential for 3D finishing and surface normal 5-axis.
- **Bull-Nose (Corner Radius)**: For smooth floor-to-wall transitions.
- **Chamfer Tool**: For edge breaking and deburring.
- **Drill**: Specialized for high-efficiency hole cycles.
- **Tapered Mill**: Supports conical shapes for specialized engraving or wall finishing.

The **Tool Visualizer** provides a high-fidelity 2D profile view, allowing you to verify tip geometry and taper angles before they are used in path generation.

---

## 3. Simulation & Verification

OpenMill's simulation engine is designed to give you absolute confidence before you hit "Cycle Start" on your machine.

### Volumetric Stock Removal (Phase 2)
OpenMill uses a state-of-the-art **GPU Voxel Engine** for material removal.
- **Voxel Grid**: The stock is represented as a high-resolution 3D grid (typically 256^3 or 512^3).
- **Compute Shaders**: Whenever the tool moves, a WGSL compute shader calculates the tool's swept volume and subtracts it from the grid in real-time.
- **Raymarching**: The stock is rendered using a raymarching pass, providing a solid, volumetric look that correctly handles undercuts and 5-axis cavities.

### Interactive G-Code Terminal
- **Bidirectional Sync**: Click any line of G-code to immediately jump the simulation to that point.
- **Trail Rendering**: The toolpath is rendered as a wireframe in the viewport. You can choose to see the "Full Path" or just the "Progressive Trail" that matches the current simulation progress.

---

## 4. Post-Processing

OpenMill translates internal toolpaths into machine-ready G-code.

### Supported Controllers
- **LinuxCNC**: Supports 5-axis table-table and head-table configurations.
- **GRBL**: Optimized for 3-axis and indexed 4-axis machines.

### Kinematics
OpenMill handles the inverse kinematics required to translate WCS (Workpiece Coordinate System) points into machine joint angles (X, Y, Z, A, C). Ensure your **Machine Config** in the UI matches your actual hardware setup.

---

## 5. Technical Architecture for Developers

OpenMill is built with modularity in mind:
- **`ToolpathStrategy` Trait**: Implement this trait in `openmill-core` to add new CAM strategies.
- **`PostProcessor` Trait**: Implement this in `openmill-post` to support new machine controllers.
- **Raycasting Engine**: Powered by `parry3d`, used for everything from toolpath projection to collision detection.

---

*Happy Milling!*

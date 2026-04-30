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

## 🗺️ Roadmap
- [ ] True heat-method geodesic solver for `GeodesicParallel`.
- [ ] S-curve acceleration in post-processors.
- [ ] Collision detection for tool holders and machine components.
- [ ] Support for 4-axis rotary wrapping.

## ⚖️ Code of Conduct
Please be respectful and constructive in all interactions. We aim to build a welcoming community for everyone interested in open-source manufacturing.

---

*Thank you for helping us make OpenMill the best open-source CAM tool!*

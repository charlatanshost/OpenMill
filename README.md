# OpenMill

Open-source 5-axis CAM for hobbyist CNC machines.

## Crates

| Crate | Description |
|---|---|
| `openmill-core` | Geometry, toolpath algorithms, and kinematics |
| `openmill-sim` | Collision detection and material removal simulation |
| `openmill-post` | G-code post-processing (LinuxCNC, GRBL) |
| `openmill-ui` | GUI application (egui + wgpu) |

## Build

```
cargo build --workspace
cargo run -p openmill-ui
```

## License

Licensed under either of [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE) at your option.

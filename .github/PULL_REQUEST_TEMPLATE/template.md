## Summary

<!-- 1–3 sentences: what changed and why. Link the issue if any. -->

## Type of change

- [ ] Bug fix
- [ ] New strategy (`ToolpathStrategy` impl)
- [ ] New post-processor (`PostProcessor` impl)
- [ ] UI / viewport / voxel rendering
- [ ] Kinematics or G-code emission
- [ ] Documentation
- [ ] Refactor / cleanup

## What changed

<!-- A short bullet list of the concrete code changes, with file:line references where helpful. -->

-

## Verification

- [ ] `cargo build --workspace` succeeds with no new warnings
- [ ] `cargo test --workspace` passes (or note pre-existing failures)
- [ ] Manually exercised in the UI for any feature affecting the viewport, tool library, or operation editor
- [ ] Sample G-code output checked for any post-processor change

## Compatibility notes

<!-- Did you add fields to a persisted type (Tool, Operation, MachineConfig)? Are they `#[serde(default)]` so existing JSON keeps loading? Did you change a public trait signature? -->

## Screenshots / G-code snippets

<!-- For UI changes attach a before/after screenshot. For post-processor changes paste a short G-code excerpt. -->

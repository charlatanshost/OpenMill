## Summary

<!-- 1–3 sentences: what changed and why. Link the issue if any. -->

## Type of change

- [ ] Bug fix
- [ ] New strategy (`ToolpathStrategy` impl)
- [ ] New post-processor (`PostProcessor` impl)
- [ ] New cross-cutting strategy param (in `transforms.rs`, applied via `apply_common_transforms`)
- [ ] New feature detector (`detect_*` in `openmill-core/src/feature.rs`)
- [ ] New verifier check (in `openmill-core/src/verify.rs`)
- [ ] UI / viewport / voxel / marching-cubes rendering
- [ ] Persistence (`.omp` bundle, autosave, undo/redo, setup sheet)
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
- [ ] For new strategies: confirmed every `Rapid` lands at `safe_z` and between-pass linking goes Rapid→LeadIn (not bare LeadIn)

## Extension checklist (if adding a strategy)

<!-- Skip if not applicable. -->

- [ ] Implements `ToolpathStrategy` in `openmill-core/src/strategies/<name>.rs`
- [ ] Re-exported from `strategies/mod.rs` and `openmill-core/src/lib.rs`
- [ ] `default_params_for` entry in `openmill-ui/src/app/strategy_params.rs`
- [ ] `show_strategy_params` editor arm
- [ ] `dispatch_strategy` arm in `openmill-ui/src/generate.rs`
- [ ] Strategy name added to the `STRATEGIES` list
- [ ] Cross-cutting fields (`direction`, `z_range`, `spring_pass`) included on the params struct if it's a cutting strategy
- [ ] `stock_to_leave` field if applicable
- [ ] Unit tests for at least one non-trivial invariant

## Compatibility notes

<!--
Did you add fields to a persisted type (Tool, Operation, MachineConfig, any …Params struct)?
Are they `#[serde(default)]` so existing JSON keeps loading?
Did you change a public trait signature?
-->

## Screenshots / G-code snippets

<!-- For UI changes attach a before/after screenshot. For post-processor changes paste a short G-code excerpt. -->


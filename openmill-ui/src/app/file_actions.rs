//! File menu actions and generation / G-code emission helpers.
//!
//! Open / save / import / export commands invoked by the File and Export
//! menu bars. Also hosts `generate_selected_toolpath`,
//! `update_sim_gcode`, and `update_sim_viewport` — they share the same
//! "act on the currently selected operation" shape and are the bridge
//! between the strategy layer and the G-code panel / 3-D path overlay.

use std::path::PathBuf;

use openmill_core::*;
use openmill_post::{get_post, PostProcessor};

use crate::autosave::Autosave;
use crate::bundle::{self, ModelBlob};
use crate::generate;
use super::OpenMillApp;

// ── File actions ────────────────────────────────────────────────────────────

impl OpenMillApp {
    pub(super) fn action_open_job(&mut self) {
        let file = rfd::FileDialog::new()
            .add_filter("OpenMill Job", &["json"])
            .set_title("Open Job")
            .pick_file();

        let Some(path) = file else { return };
        let json = match std::fs::read_to_string(&path) {
            Ok(s) => s,
            Err(e) => { self.log.push(format!("Failed to read file: {e}")); return; }
        };
        let job = match serde_json::from_str::<Job>(&json) {
            Ok(j) => j,
            Err(e) => { self.log.push(format!("Failed to parse job: {e}")); return; }
        };

        // Reset live state. This needs to happen BEFORE try_load_model so
        // the new model's positioning + stock derive from the loaded job,
        // not whatever was on screen.
        let model_path_relative = job.model_path.clone();
        let model_position = nalgebra::Vector3::new(
            job.model_position[0], job.model_position[1], job.model_position[2],
        );
        self.toolpaths = vec![Vec::new(); job.operations.len()];
        self.selected_tool = None;
        self.selected_op   = None;
        self.features.clear();
        self.sim_op_idx = None;
        self.sim_progress = 0.0;
        self.sim_prev_progress = 0.0;
        self.sim_collisions.clear();
        self.sim_gcode.clear();
        self.model = None;
        if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
            let _ = rs;
            vp.clear_pick();
            vp.upload_collisions(&rs.device, &[]);
        }
        self.job = job;

        // Try to load the model — relative paths resolve against the job file's
        // directory.
        if let Some(mp) = model_path_relative {
            let model_path = path.parent()
                .map(|p| p.join(&mp))
                .unwrap_or_else(|| PathBuf::from(&mp));
            self.imported_model = ModelBlob::from_path(&model_path).ok();
            self.try_load_model(&model_path);
            // Re-apply the saved position now that the model is loaded.
            if model_position.norm_squared() > 1e-12 {
                if let Some(m) = self.model.as_mut() {
                    m.set_position(model_position);
                    if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
                        vp.upload_mesh(&rs.device, &m.mesh);
                        vp.upload_stock(&rs.device, &rs.queue, &m.aabb, &m.stock);
                    }
                }
            }
        }
        self.history.reset(&self.job);
        self.autosave.reset_baseline();
        self.log.push(format!(
            "Opened job from {} — {} ops, {} tools.",
            path.display(),
            self.job.operations.len(),
            self.job.tools.len(),
        ));
    }

    pub(super) fn action_save_job(&mut self) {
        let file = rfd::FileDialog::new()
            .add_filter("OpenMill Job", &["json"])
            .set_title("Save Job")
            .set_file_name(&format!("{}.json", self.job.name))
            .save_file();

        if let Some(path) = file {
            match serde_json::to_string_pretty(&self.job) {
                Ok(json) => match std::fs::write(&path, &json) {
                    Ok(()) => {
                        // Manual save → drop the autosave so the next launch
                        // doesn't offer a stale recovery for the same state.
                        Autosave::clear();
                        self.autosave.reset_baseline();
                        self.log.push(format!("Saved job to {}", path.display()));
                    }
                    Err(e) => self.log.push(format!("Failed to write file: {e}")),
                },
                Err(e) => self.log.push(format!("Serialization error: {e}")),
            }
        }
    }

    /// Render the current job to an HTML setup sheet and prompt for a save
    /// location. The sheet is fully self-contained (CSS inlined, no JS) so
    /// it prints cleanly and survives archival.
    pub(super) fn action_export_setup_sheet(&mut self) {
        let file = rfd::FileDialog::new()
            .add_filter("HTML Document", &["html", "htm"])
            .set_title("Save Setup Sheet")
            .set_file_name(&format!("{}_setup.html", self.job.name))
            .save_file();
        let Some(path) = file else { return };
        let html = crate::setup_sheet::render_html(&self.job, &self.toolpaths);
        match std::fs::write(&path, html) {
            Ok(()) => self.log.push(format!("Setup sheet saved to {}", path.display())),
            Err(e) => self.log.push(format!("Failed to write setup sheet: {e}")),
        }
    }

    /// Save a single-file `.omp` project bundle: zipped job + model.
    pub(super) fn action_save_bundle(&mut self) {
        let file = rfd::FileDialog::new()
            .add_filter("OpenMill Project", &["omp"])
            .set_title("Save Project Bundle")
            .set_file_name(&format!("{}.omp", self.job.name))
            .save_file();

        let Some(path) = file else { return };
        match bundle::save_bundle(&path, &self.job, self.imported_model.as_ref()) {
            Ok(()) => {
                // The bundle is now the canonical on-disk copy — drop the
                // autosave so the next launch doesn't offer a stale recovery.
                Autosave::clear();
                self.autosave.reset_baseline();
                self.log.push(format!("Saved bundle to {}", path.display()));
            }
            Err(e) => self.log.push(format!("Failed to save bundle: {e:#}")),
        }
    }

    /// Open a `.omp` bundle. Replaces the current job and model.
    pub(super) fn action_open_bundle(&mut self) {
        let file = rfd::FileDialog::new()
            .add_filter("OpenMill Project", &["omp"])
            .set_title("Open Project Bundle")
            .pick_file();

        let Some(path) = file else { return };
        let loaded = match bundle::load_bundle(&path) {
            Ok(b) => b,
            Err(e) => {
                self.log.push(format!("Failed to open bundle: {e:#}"));
                return;
            }
        };

        // Reset live state.
        let model_position = nalgebra::Vector3::new(
            loaded.job.model_position[0],
            loaded.job.model_position[1],
            loaded.job.model_position[2],
        );
        self.toolpaths = vec![Vec::new(); loaded.job.operations.len()];
        self.selected_tool = None;
        self.selected_op = None;
        self.features.clear();
        self.sim_op_idx = None;
        self.sim_progress = 0.0;
        self.sim_prev_progress = 0.0;
        self.sim_collisions.clear();
        self.sim_gcode.clear();
        self.model = None;
        if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
            vp.clear_pick();
            vp.upload_collisions(&rs.device, &[]);
        }
        self.job = loaded.job;
        self.imported_model = loaded.model_blob;

        if let Some(mut model) = loaded.model {
            model.stock = self.job.stock.to_shape();
            // Re-apply saved position by rebuilding `mesh` from `mesh_orig`.
            if model_position.norm_squared() > 1e-12 {
                model.set_position(model_position);
            }
            if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
                vp.upload_mesh(&rs.device, &model.mesh);
                vp.upload_stock(&rs.device, &rs.queue, &model.aabb, &model.stock);
                let mins = model.aabb.mins;
                let maxs = model.aabb.maxs;
                self.camera.focus_on_aabb(
                    [mins.x, mins.y, mins.z],
                    [maxs.x, maxs.y, maxs.z],
                );
            }
            self.model = Some(model);
        }
        self.sdf_uploaded = false;

        self.history.reset(&self.job);
        self.autosave.reset_baseline();
        self.log.push(format!(
            "Opened bundle from {} — {} ops, {} tools.",
            path.display(),
            self.job.operations.len(),
            self.job.tools.len(),
        ));
    }

    pub(super) fn action_import_model(&mut self) {
        let file = rfd::FileDialog::new()
            .add_filter("3D Models", &["stl", "3mf"])
            .set_title("Import Model")
            .pick_file();

        if let Some(path) = file {
            self.job.model_path = Some(
                path.file_name()
                    .map(|n| n.to_string_lossy().into_owned())
                    .unwrap_or_else(|| path.to_string_lossy().into_owned()),
            );
            // Snapshot the source file so we can re-pack it into a `.omp`
            // bundle later. Best-effort: if the read fails we still try to
            // import normally so the user sees the geometry.
            self.imported_model = ModelBlob::from_path(&path).ok();
            self.try_load_model(&path);
        }
    }

    fn try_load_model(&mut self, path: &std::path::Path) {
        let ext = path
            .extension()
            .map(|e| e.to_string_lossy().to_lowercase())
            .unwrap_or_default();

        let result = match ext.as_str() {
            "stl" => import_stl(path),
            "3mf" => import_3mf(path),
            _ => {
                self.log.push(format!("Unknown file type: .{ext}"));
                return;
            }
        };

        match result {
            Ok(mut model) => {
                model.stock = self.job.stock.to_shape();
                let aabb = &model.aabb;
                let size = aabb.maxs - aabb.mins;
                self.log.push(format!(
                    "Imported model: {:.1} x {:.1} x {:.1} mm, {} vertices",
                    size.x, size.y, size.z,
                    model.mesh.vertices().len(),
                ));

                // Upload mesh to GPU and focus camera.
                if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
                    vp.upload_mesh(&rs.device, &model.mesh);
                    vp.upload_stock(&rs.device, &rs.queue, aabb, &model.stock);
                    let mins = aabb.mins;
                    let maxs = aabb.maxs;
                    self.camera.focus_on_aabb(
                        [mins.x, mins.y, mins.z],
                        [maxs.x, maxs.y, maxs.z],
                    );
                }

                self.model = Some(model);
                // New model invalidates any features detected on the previous one,
                // along with any picked-face highlight from the prior session.
                self.features.clear();
                self.job.model_position = [0.0, 0.0, 0.0];
                // SDF (for Verify mode) is keyed to the loaded model. Drop
                // the cached upload so the next verify toggle recomputes
                // it against the new geometry.
                self.sdf_uploaded = false;
                if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
                    let _ = rs;
                    vp.clear_pick();
                }
            }
            Err(e) => self.log.push(format!("Import failed: {e}")),
        }
    }

    /// Run the verifier on every generated toolpath and log the findings.
    /// Returns `true` if any error-level issues were found.
    pub(super) fn action_verify(&mut self) -> bool {
        let all_paths: Vec<openmill_core::Toolpath> = self.toolpaths
            .iter()
            .flat_map(|v| v.iter().cloned())
            .collect();
        if all_paths.is_empty() {
            self.log.push("Verify: no toolpaths generated yet.".into());
            return false;
        }
        let issues = openmill_core::verify_job(
            &all_paths,
            &self.job.machine,
            &self.job.tools,
            self.model.as_ref(),
        );
        if issues.is_empty() {
            self.log.push("Verify: ✓ no issues.".into());
            return false;
        }
        let n_err = issues.iter().filter(|i| i.level == openmill_core::IssueLevel::Error).count();
        let n_warn = issues.len() - n_err;
        self.log.push(format!(
            "Verify: {n_err} error(s), {n_warn} warning(s) — see below."
        ));
        for i in &issues {
            let prefix = match i.level {
                openmill_core::IssueLevel::Error   => "  ✗ ERROR:",
                openmill_core::IssueLevel::Warning => "  ⚠ warn :",
            };
            self.log.push(format!("{prefix} {}", i.message));
        }
        n_err > 0
    }

    pub(super) fn action_export_gcode(&mut self, post: &dyn PostProcessor) {
        let any_paths = self.toolpaths.iter().any(|v| !v.is_empty());
        if !any_paths {
            self.log.push("No toolpaths generated. Generate toolpaths first.".into());
            return;
        }

        // Auto-verify before exporting. If any errors fire, refuse to write
        // and ask the user to fix them. Warnings don't block — they're just
        // logged.
        if self.action_verify() {
            self.log.push(
                "Export aborted: verification reported errors. Fix them or use Verify menu to review."
                .into(),
            );
            return;
        }

        let file = rfd::FileDialog::new()
            .add_filter("G-code", &["nc", "ngc", "gcode"])
            .set_title("Export G-code")
            .set_file_name("output.nc")
            .save_file();

        let Some(path) = file else { return };

        let config = self.job.machine.post_config.clone();
        let mut output = post.header(&config);

        // Iterate operations in order. Emit a tool change whenever the active
        // tool changes, then op preamble → all toolpaths for that op → op
        // postamble. This matches the documented PostProcessor contract.
        let mut active_tool_id: Option<u32> = None;

        for (op_idx, op) in self.job.operations.iter().enumerate() {
            if op_idx >= self.toolpaths.len() || self.toolpaths[op_idx].is_empty() {
                continue;
            }
            let Some(tool) = self.job.tools.iter().find(|t| t.id == op.tool_id) else {
                self.log.push(format!("Op \"{}\": tool T{} not found, skipped.", op.name, op.tool_id));
                continue;
            };

            if active_tool_id != Some(tool.id) {
                output.push_str(&post.tool_change(tool));
                active_tool_id = Some(tool.id);
            }

            output.push_str(&post.op_preamble(op, tool));

            let plunge = if op.plunge_rate > 0.0 {
                op.plunge_rate
            } else if op.feed_rate > 0.0 {
                op.feed_rate * 0.5
            } else {
                0.0
            };

            for tp in &self.toolpaths[op_idx] {
                let posted_tp = if op.feed_rate > 0.0 || plunge > 0.0 {
                    tp.with_op_feeds(op.feed_rate, plunge)
                } else {
                    tp.clone()
                };
                match post.process_toolpath(&posted_tp, &self.job.machine) {
                    Ok(gcode_lines) => {
                        for (line, _) in gcode_lines {
                            output.push_str(&line);
                        }
                    }
                    Err(e) => {
                        self.log.push(format!("Post-process error: {e}"));
                        return;
                    }
                }
            }

            output.push_str(&post.op_postamble(op));
        }

        output.push_str(&post.footer());

        match std::fs::write(&path, &output) {
            Ok(()) => {
                let lines = output.lines().count();
                self.log.push(format!(
                    "Exported {} lines of G-code to {}",
                    lines,
                    path.display()
                ));
            }
            Err(e) => self.log.push(format!("Failed to write G-code: {e}")),
        }
    }

    /// Generate toolpath for the selected operation.
    pub(super) fn generate_selected_toolpath(&mut self) {
        let Some(idx) = self.selected_op else { return };
        if idx >= self.job.operations.len() {
            return;
        }

        let Some(ref model) = self.model else {
            self.log.push("Import a model first.".into());
            return;
        };

        let op = &self.job.operations[idx];
        if !op.enabled {
            self.log.push("Operation is disabled.".into());
            return;
        }

        let Some(tool) = self.job.tools.iter().find(|t| t.id == op.tool_id) else {
            self.log.push(format!("Tool T{} not found.", op.tool_id));
            return;
        };

        let result = generate::dispatch(op, model, tool, &self.job.machine);

        match result {
            Ok(paths) => {
                let total_pts: usize = paths.iter().map(|p: &Toolpath| p.points.len()).sum();
                self.log.push(format!(
                    "Generated {} toolpath(s), {} points for \"{}\".",
                    paths.len(),
                    total_pts,
                    self.job.operations[idx].name,
                ));
                while self.toolpaths.len() <= idx {
                    self.toolpaths.push(Vec::new());
                }
                self.toolpaths[idx] = paths;
                self.update_sim_gcode();
                self.update_sim_viewport();
            }
            Err(e) => self.log.push(format!("Generation failed: {e}")),
        }
    }

    pub(super) fn update_sim_gcode(&mut self) {
        self.sim_gcode.clear();
        let Some(idx) = self.selected_op else { return };
        if idx >= self.toolpaths.len() || idx >= self.job.operations.len() { return; }

        let post = get_post(&self.job.machine.post_processor);
        let op = &self.job.operations[idx];
        let tool = self.job.tools.iter().find(|t| t.id == op.tool_id);

        // Op preamble (spindle on, coolant on, custom op gcode).
        if let Some(t) = tool {
            for line in post.op_preamble(op, t).lines() {
                self.sim_gcode.push((format!("{line}\n"), None));
            }
        }

        let plunge = if op.plunge_rate > 0.0 {
            op.plunge_rate
        } else if op.feed_rate > 0.0 {
            op.feed_rate * 0.5
        } else {
            0.0
        };

        for (tp_idx, tp) in self.toolpaths[idx].iter().enumerate() {
            let posted_tp = if op.feed_rate > 0.0 || plunge > 0.0 {
                tp.with_op_feeds(op.feed_rate, plunge)
            } else {
                tp.clone()
            };
            match post.process_toolpath(&posted_tp, &self.job.machine) {
                Ok(lines) => self.sim_gcode.extend(lines),
                Err(e) => self.log.push(format!(
                    "Post-processor error on toolpath {tp_idx} (\"{}\"): {e}",
                    tp.name
                )),
            }
        }

        // Op postamble (coolant off).
        for line in post.op_postamble(op).lines() {
            self.sim_gcode.push((format!("{line}\n"), None));
        }
    }

    pub(super) fn update_sim_viewport(&mut self) {
        let Some(idx) = self.selected_op else { return };
        if idx >= self.toolpaths.len() { return; }

        // Resolve the stock envelope + tool radius for air-cut shading.
        // Without these the viewport falls back to plain move-type colors.
        let stock_aabb = self.model.as_ref().map(|m| m.stock_aabb());
        let tool_radius = self.job.operations.get(idx)
            .and_then(|op| self.job.tools.iter().find(|t| t.id == op.tool_id))
            .map(|t| (t.shape.diameter() * 0.5) as f32)
            .unwrap_or(0.0);

        if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
            vp.upload_toolpath(&rs.device, &self.toolpaths[idx], stock_aabb.as_ref(), tool_radius);
        }
    }
}

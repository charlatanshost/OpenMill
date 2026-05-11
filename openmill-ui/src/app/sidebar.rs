//! Left-hand side panel and its sub-sections.
//!
//! Renders the stacked sections in the Manufacturing tab: job info, tools,
//! features, operations, settings, machine. Also hosts feature-driven helpers
//! (assigning detected holes / pockets onto an operation's feature list).
//! `handle_face_pick` lives here too because the picking UX is implemented in
//! the features section even though the actual click event is captured by
//! the central viewport panel.

use eframe::egui;
use openmill_core::*;

use super::{
    default_params_for, fmt_minutes, show_strategy_params, AppTab, FeatureAssignTarget,
    OpenMillApp, StockUnits, STRATEGIES,
};

// ── Side panel ──────────────────────────────────────────────────────────────

impl OpenMillApp {
    pub(super) fn show_side_panel(&mut self, ctx: &egui::Context) {
        egui::SidePanel::left("side_panel")
            .resizable(true)
            .default_width(300.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    self.section_job_info(ui);
                    ui.separator();
                    self.section_tools(ui);
                    ui.separator();
                    self.section_features(ui);
                    ui.separator();
                    self.section_operations(ui);
                    ui.separator();
                    self.section_settings(ui);
                    ui.separator();
                    self.section_machine(ui);
                });
            });
    }

    // ── Job info ─────────────────────────────────────────────────────────

    fn section_job_info(&mut self, ui: &mut egui::Ui) {
        ui.collapsing("Job", |ui| {
            ui.horizontal(|ui| {
                ui.label("Name:");
                ui.text_edit_singleline(&mut self.job.name);
            });
            ui.horizontal(|ui| {
                ui.label("Model:");
                if let Some(ref p) = self.job.model_path {
                    ui.label(p.as_str());
                } else {
                    ui.weak("(none)");
                }
            });
            if self.model.is_some() {
                let aabb = &self.model.as_ref().unwrap().aabb;
                let size = aabb.maxs - aabb.mins;
                ui.label(format!(
                    "Size: {:.1} x {:.1} x {:.1} mm",
                    size.x, size.y, size.z
                ));
            }

            ui.separator();
            ui.label("Stock:");
            let mut stock_changed = false;

            // Unit toggle (mm / decimal inch). Affects display only — the
            // underlying job is always stored in millimetres.
            ui.horizontal(|ui| {
                ui.label("Units:");
                ui.selectable_value(&mut self.stock_units, StockUnits::Mm,   "mm");
                ui.selectable_value(&mut self.stock_units, StockUnits::Inch, "inch (decimal)");
            });
            let units = self.stock_units;
            let to_mm = units.to_mm();
            // Display speed in current units — slower drag in inches because
            // 1 inch = 25.4 mm, so a 0.1-step in inches ≈ 2.5 mm.
            let drag_speed = if units == StockUnits::Mm { 0.5 } else { 0.025 };

            // Snapshot the part dimensions in mm before borrowing self.job.stock,
            // so we can convert between (margin) and (absolute size) without a
            // borrow-checker conflict against self.model.
            let part_dims_mm: Option<[f64; 3]> = self.model.as_ref().map(|m| {
                let s = m.aabb.maxs - m.aabb.mins;
                [s.x as f64, s.y as f64, s.z as f64]
            });

            let mut current_type = match self.job.stock {
                openmill_core::job::StockDef::BoundingBox { .. } => 0,
                openmill_core::job::StockDef::Cylinder    { .. } => 1,
                _ => 0,
            };

            ui.horizontal(|ui| {
                ui.label("Type:");
                if egui::ComboBox::from_id_salt("stock_type")
                    .selected_text(if current_type == 0 { "Box" } else { "Cylinder" })
                    .show_ui(ui, |ui| {
                        let mut c = false;
                        c |= ui.selectable_value(&mut current_type, 0, "Box").changed();
                        c |= ui.selectable_value(&mut current_type, 1, "Cylinder").changed();
                        c
                    })
                    .inner
                    .unwrap_or(false)
                {
                    if current_type == 0 {
                        self.job.stock = openmill_core::job::StockDef::BoundingBox { margin: [5.0, 5.0, 5.0] };
                    } else {
                        self.job.stock = openmill_core::job::StockDef::Cylinder { diameter: 50.0, height: 20.0 };
                    }
                    stock_changed = true;
                }
            });

            match &mut self.job.stock {
                openmill_core::job::StockDef::BoundingBox { margin } => {
                    if let Some(part_mm) = part_dims_mm {
                        // Edit absolute stock dimensions (W × D × H). The
                        // symmetric per-axis margin is derived from
                        //   margin[i] = (stock_size[i] - part_size[i]) / 2.
                        let mut size_disp = [
                            (part_mm[0] + 2.0 * margin[0]) / to_mm,
                            (part_mm[1] + 2.0 * margin[1]) / to_mm,
                            (part_mm[2] + 2.0 * margin[2]) / to_mm,
                        ];
                        ui.horizontal(|ui| {
                            ui.label("Size W:");
                            if ui.add(egui::DragValue::new(&mut size_disp[0]).speed(drag_speed).suffix(units.suffix())).changed() {
                                margin[0] = ((size_disp[0] * to_mm) - part_mm[0]) * 0.5;
                                stock_changed = true;
                            }
                            ui.label("D:");
                            if ui.add(egui::DragValue::new(&mut size_disp[1]).speed(drag_speed).suffix(units.suffix())).changed() {
                                margin[1] = ((size_disp[1] * to_mm) - part_mm[1]) * 0.5;
                                stock_changed = true;
                            }
                            ui.label("H:");
                            if ui.add(egui::DragValue::new(&mut size_disp[2]).speed(drag_speed).suffix(units.suffix())).changed() {
                                margin[2] = ((size_disp[2] * to_mm) - part_mm[2]) * 0.5;
                                stock_changed = true;
                            }
                        });
                        // Show the derived per-side offset as a hint.
                        ui.weak(format!(
                            "Offset per side: {:.3} × {:.3} × {:.3} {}",
                            margin[0] / to_mm,
                            margin[1] / to_mm,
                            margin[2] / to_mm,
                            units.label(),
                        ));
                    } else {
                        // No model loaded — fall back to per-side margin
                        // editing in the current unit.
                        let mut margin_disp = [
                            margin[0] / to_mm,
                            margin[1] / to_mm,
                            margin[2] / to_mm,
                        ];
                        ui.horizontal(|ui| {
                            ui.label("Margin X:");
                            if ui.add(egui::DragValue::new(&mut margin_disp[0]).speed(drag_speed).suffix(units.suffix())).changed() {
                                margin[0] = margin_disp[0] * to_mm;
                                stock_changed = true;
                            }
                            ui.label("Y:");
                            if ui.add(egui::DragValue::new(&mut margin_disp[1]).speed(drag_speed).suffix(units.suffix())).changed() {
                                margin[1] = margin_disp[1] * to_mm;
                                stock_changed = true;
                            }
                            ui.label("Z:");
                            if ui.add(egui::DragValue::new(&mut margin_disp[2]).speed(drag_speed).suffix(units.suffix())).changed() {
                                margin[2] = margin_disp[2] * to_mm;
                                stock_changed = true;
                            }
                        });
                        ui.weak("Load a model to set absolute stock dimensions.");
                    }
                }
                openmill_core::job::StockDef::Cylinder { diameter, height } => {
                    let mut diameter_disp = *diameter / to_mm;
                    let mut height_disp   = *height   / to_mm;
                    ui.horizontal(|ui| {
                        ui.label("Diameter:");
                        if ui.add(egui::DragValue::new(&mut diameter_disp).speed(drag_speed).suffix(units.suffix())).changed() {
                            *diameter = diameter_disp * to_mm;
                            stock_changed = true;
                        }
                        ui.label("Height:");
                        if ui.add(egui::DragValue::new(&mut height_disp).speed(drag_speed).suffix(units.suffix())).changed() {
                            *height = height_disp * to_mm;
                            stock_changed = true;
                        }
                    });
                }
                _ => {}
            }

            if stock_changed {
                if let Some(model) = &mut self.model {
                    model.stock = self.job.stock.to_shape();
                    if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
                        vp.upload_stock(&rs.device, &rs.queue, &model.aabb, &model.stock);
                    }
                }
            }

            // ── Model position in the work area ─────────────────────────
            // Translates the imported mesh by `position` so the user can
            // place it anywhere in the workpiece coordinate system. Stock
            // follows automatically because it's derived from the part AABB.
            if let Some(model) = self.model.as_ref() {
                ui.separator();
                ui.label("Model position (workpiece origin offset):");
                let mut pos_disp = [
                    model.position.x / to_mm,
                    model.position.y / to_mm,
                    model.position.z / to_mm,
                ];
                let mut pos_changed = false;
                ui.horizontal(|ui| {
                    ui.label("X:");
                    pos_changed |= ui.add(egui::DragValue::new(&mut pos_disp[0]).speed(drag_speed).suffix(units.suffix())).changed();
                    ui.label("Y:");
                    pos_changed |= ui.add(egui::DragValue::new(&mut pos_disp[1]).speed(drag_speed).suffix(units.suffix())).changed();
                    ui.label("Z:");
                    pos_changed |= ui.add(egui::DragValue::new(&mut pos_disp[2]).speed(drag_speed).suffix(units.suffix())).changed();
                });
                ui.horizontal(|ui| {
                    if ui.button("⟲ Reset").on_hover_text(
                        "Move the model back to the imported origin (0, 0, 0)."
                    ).clicked() {
                        pos_disp = [0.0, 0.0, 0.0];
                        pos_changed = true;
                    }
                    if ui.button("⌖ Centre on origin").on_hover_text(
                        "Translate the model so its AABB centroid sits at the workpiece origin."
                    ).clicked() {
                        // Subtract current centroid from current position so the new
                        // centroid lands at (0, 0, 0).
                        let s = model.aabb.maxs - model.aabb.mins;
                        let cx = (model.aabb.mins.x as f64 + s.x as f64 * 0.5) - model.position.x;
                        let cy = (model.aabb.mins.y as f64 + s.y as f64 * 0.5) - model.position.y;
                        let cz = (model.aabb.mins.z as f64 + s.z as f64 * 0.5) - model.position.z;
                        pos_disp = [-cx / to_mm, -cy / to_mm, -cz / to_mm];
                        pos_changed = true;
                    }
                });
                if pos_changed {
                    let new_pos = nalgebra::Vector3::new(
                        pos_disp[0] * to_mm,
                        pos_disp[1] * to_mm,
                        pos_disp[2] * to_mm,
                    );
                    // Mirror onto the persistent job so save/load round-trips
                    // preserve the position.
                    self.job.model_position = [new_pos.x, new_pos.y, new_pos.z];
                    if let Some(m) = self.model.as_mut() {
                        m.set_position(new_pos);
                        if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
                            vp.upload_mesh(&rs.device, &m.mesh);
                            vp.upload_stock(&rs.device, &rs.queue, &m.aabb, &m.stock);
                        }
                        // Voxel volume bounds also depend on model AABB;
                        // force a rebuild on the next sim-mode entry.
                        self.sim_op_idx = None;
                        self.sim_progress = 0.0;
                        self.sim_prev_progress = 0.0;
                    }
                }
            }
        });
    }

    // ── Tools ────────────────────────────────────────────────────────────

    fn section_tools(&mut self, ui: &mut egui::Ui) {
        ui.collapsing("Tools", |ui| {
            // Simple list linking to the Tools tab
            for (i, tool) in self.job.tools.iter().enumerate() {
                let selected = self.selected_tool == Some(i);
                let label = format!("T{} — {} ({:.1}mm)", tool.id, tool.name, tool.shape.diameter());
                if ui.selectable_label(selected, &label).clicked() {
                    self.selected_tool = Some(i);
                    self.active_tab = AppTab::Tools;
                }
            }
            if ui.button("+ Manage Tools...").clicked() {
                self.active_tab = AppTab::Tools;
            }
        });
    }

    // ── Features ─────────────────────────────────────────────────────────
    //
    // A "feature" is a recognised piece of part geometry (a hole, a pocket,
    // a flat face) that the user can assign to an operation. Today the
    // detector finds vertical-axis cylindrical holes; pocket and thread-mill
    // detection are scoped for future work.

    fn section_features(&mut self, ui: &mut egui::Ui) {
        ui.collapsing(format!("Features ({})", self.features.len()), |ui| {
            // Manual face-pick toggle. When on, a left-click in the viewport
            // (without dragging) raycasts the mesh and adds the picked face
            // cluster as a feature.
            ui.horizontal(|ui| {
                let was_on = self.pick_face_mode;
                if ui.toggle_value(&mut self.pick_face_mode, "🎯 Pick Face Mode")
                    .on_hover_text(
                        "When on: click any face in the viewport to add it as a feature.\n\
                         Drag still orbits the camera. Click again to add another."
                    ).changed()
                {
                    if !self.pick_face_mode && was_on {
                        // Turning it off — clear the highlight.
                        if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
                            let _ = rs;
                            vp.clear_pick();
                        }
                    }
                }
                if self.pick_face_mode {
                    ui.weak("(click a face to add)");
                }
            });

            // Detection actions
            ui.horizontal(|ui| {
                let model_loaded = self.model.is_some();
                ui.add_enabled_ui(model_loaded, |ui| {
                    if ui.button("🔍 Holes").on_hover_text(
                        "Scan the imported mesh for vertical cylindrical holes."
                    ).clicked() {
                        if let Some(model) = &self.model {
                            let mut found = openmill_core::detect_holes(&model.mesh);
                            let mut next_id = self.features.iter().map(|f| f.id).max().unwrap_or(0) + 1;
                            for f in &mut found { f.id = next_id; next_id += 1; }
                            let added = found.len();
                            self.features.extend(found);
                            self.log.push(format!(
                                "Hole detection: {} new feature(s) (total {}).",
                                added, self.features.len(),
                            ));
                        }
                    }
                    if ui.button("🔍 Pockets").on_hover_text(
                        "Scan the imported mesh for open-top pockets (horizontal floor faces with clearance above)."
                    ).clicked() {
                        if let Some(model) = &self.model {
                            let mut found = openmill_core::detect_pockets(&model.mesh);
                            let mut next_id = self.features.iter().map(|f| f.id).max().unwrap_or(0) + 1;
                            for f in &mut found { f.id = next_id; next_id += 1; }
                            let added = found.len();
                            self.features.extend(found);
                            self.log.push(format!(
                                "Pocket detection: {} new feature(s) (total {}).",
                                added, self.features.len(),
                            ));
                        }
                    }
                });
                if !self.features.is_empty() && ui.button("🗑 Clear All").clicked() {
                    let n = self.features.len();
                    self.features.clear();
                    self.log.push(format!("Cleared {n} feature(s)."));
                }
                if !model_loaded {
                    ui.weak("(import a model first)");
                }
            });

            // Feature list with per-row delete + assign actions.
            let mut to_delete: Option<usize> = None;
            let mut assign_action: Option<(usize, FeatureAssignTarget)> = None;
            for (idx, f) in self.features.iter().enumerate() {
                ui.group(|ui| {
                    ui.horizontal(|ui| {
                        ui.strong(format!("[{}] #{}", f.type_label(), f.id));
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.small_button("🗑").on_hover_text("Remove this feature").clicked() {
                                to_delete = Some(idx);
                            }
                        });
                    });
                    ui.label(&f.label);
                    ui.horizontal_wrapped(|ui| {
                        match f.kind {
                            openmill_core::FeatureKind::Hole { .. } => {
                                if ui.small_button("→ Drilling").on_hover_text(
                                    "Append this hole to the selected 5-Axis Drilling op."
                                ).clicked() {
                                    assign_action = Some((idx, FeatureAssignTarget::Drilling));
                                }
                                if ui.small_button("→ Tapping").on_hover_text(
                                    "Append this hole to the selected Tapping op."
                                ).clicked() {
                                    assign_action = Some((idx, FeatureAssignTarget::Tapping));
                                }
                                if ui.small_button("→ Thread Mill").on_hover_text(
                                    "Append this hole to the selected Thread Milling op."
                                ).clicked() {
                                    assign_action = Some((idx, FeatureAssignTarget::ThreadMill));
                                }
                            }
                            openmill_core::FeatureKind::Pocket { .. } => {
                                if ui.small_button("→ Pocket Clearing").on_hover_text(
                                    "Append this pocket to the selected Pocket Clearing op."
                                ).clicked() {
                                    assign_action = Some((idx, FeatureAssignTarget::PocketClearing));
                                }
                            }
                            openmill_core::FeatureKind::FlatFace { .. } => {}
                        }
                    });
                });
            }
            if let Some(i) = to_delete {
                self.features.remove(i);
            }
            if let Some((i, target)) = assign_action {
                self.assign_feature_to_op(i, target);
            }

            // Bulk-assign shortcuts.
            if !self.features.is_empty() {
                ui.add_space(4.0);
                let n_holes = self.features.iter()
                    .filter(|f| matches!(f.kind, openmill_core::FeatureKind::Hole { .. }))
                    .count();
                let n_pockets = self.features.iter()
                    .filter(|f| matches!(f.kind, openmill_core::FeatureKind::Pocket { .. }))
                    .count();
                if n_holes > 0 {
                    ui.horizontal(|ui| {
                        ui.label(format!("Bulk assign all {n_holes} holes to:"));
                        if ui.small_button("Drilling").clicked()    { self.assign_all_to_op(FeatureAssignTarget::Drilling); }
                        if ui.small_button("Tapping").clicked()     { self.assign_all_to_op(FeatureAssignTarget::Tapping); }
                        if ui.small_button("Thread Mill").clicked() { self.assign_all_to_op(FeatureAssignTarget::ThreadMill); }
                    });
                }
                if n_pockets > 0 {
                    ui.horizontal(|ui| {
                        ui.label(format!("Bulk assign all {n_pockets} pockets to:"));
                        if ui.small_button("Pocket Clearing").clicked() {
                            self.assign_all_to_op(FeatureAssignTarget::PocketClearing);
                        }
                    });
                }
            }
        });
    }

    /// Append a single feature to the selected operation, if its strategy
    /// matches the requested target.
    fn assign_feature_to_op(&mut self, feature_idx: usize, target: FeatureAssignTarget) {
        let Some(op_idx) = self.selected_op else {
            self.log.push(format!("Select a {} op first.", target.expected_strategy()));
            return;
        };
        let Some(feature) = self.features.get(feature_idx).cloned() else { return };
        let op = &mut self.job.operations[op_idx];
        if op.strategy != target.expected_strategy() {
            self.log.push(format!(
                "Selected op \"{}\" is a {} — switch to a {} op.",
                op.name, op.strategy, target.expected_strategy()
            ));
            return;
        }
        let added = target.append_one(op, &feature);
        if let Some(n) = added {
            self.log.push(format!(
                "Added feature #{} to op \"{}\" ({n} item(s) total).",
                feature.id, op.name
            ));
        } else {
            self.log.push(format!(
                "Feature #{} kind doesn't match {} — skipped.",
                feature.id, target.expected_strategy()
            ));
        }
    }

    /// Cast a ray from the click position into the loaded mesh, flood-fill
    /// coplanar neighbours, and add the result as a new auto-classified
    /// `Feature`. Highlights the picked face cluster in the viewport.
    pub(super) fn handle_face_pick(&mut self, click_pos: egui::Pos2, rect: egui::Rect) {
        let Some(model) = self.model.as_ref() else {
            self.log.push("Pick face: no model loaded.".into());
            return;
        };

        // Click position relative to the viewport's top-left.
        let px = click_pos.x - rect.left();
        let py = click_pos.y - rect.top();
        let (origin, dir) = self.camera.ray_from_screen(px, py, rect.width(), rect.height());

        // Tolerance in degrees: triangles within this dihedral angle of the
        // seed are flooded into the cluster. ~5° handles "really flat" face
        // groups well; loosen for curved surfaces, tighten for tessellated planes.
        let angle_tol_deg = 5.0;
        let pick = match openmill_core::pick_face(&model.mesh, origin, dir, angle_tol_deg) {
            Some(p) => p,
            None => {
                self.log.push("Pick face: ray missed the mesh.".into());
                return;
            }
        };

        // Auto-classify and add to the features list.
        let next_id = self.features.iter().map(|f| f.id).max().unwrap_or(0) + 1;
        let feature = pick.to_feature(&model.mesh, next_id);
        self.log.push(format!(
            "Picked {}: {} ({} triangles, {:.1}mm² total).",
            feature.type_label(), feature.label, pick.cluster_tris.len(), pick.area,
        ));
        self.features.push(feature);

        // Upload the cluster as a highlight overlay.
        let tris: Vec<[nalgebra::Point3<f32>; 3]> = pick.cluster_tris.iter().map(|&t| {
            let tri = model.mesh.indices()[t];
            [
                model.mesh.vertices()[tri[0] as usize],
                model.mesh.vertices()[tri[1] as usize],
                model.mesh.vertices()[tri[2] as usize],
            ]
        }).collect();
        if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
            vp.upload_pick_triangles(&rs.device, &tris);
        }
    }

    /// Replace the selected op's feature-driven list with **all** matching
    /// features in the library.
    fn assign_all_to_op(&mut self, target: FeatureAssignTarget) {
        let Some(op_idx) = self.selected_op else {
            self.log.push(format!("Select a {} op first.", target.expected_strategy()));
            return;
        };
        // Snapshot the matching features before borrowing the op mutably.
        let features_snapshot: Vec<openmill_core::Feature> = self.features.clone();
        let op = &mut self.job.operations[op_idx];
        if op.strategy != target.expected_strategy() {
            self.log.push(format!(
                "Selected op \"{}\" is a {} — switch to a {} op.",
                op.name, op.strategy, target.expected_strategy()
            ));
            return;
        }
        let n = target.replace_all(op, &features_snapshot);
        self.log.push(format!(
            "Assigned {n} feature(s) to op \"{}\" (replaced previous list).",
            op.name,
        ));
    }

    // ── Operations ───────────────────────────────────────────────────────

    fn section_operations(&mut self, ui: &mut egui::Ui) {
        ui.collapsing("Operations", |ui| {
            let mut remove_idx = None;
            let mut generate_requested = false;

            // Estimated runtime per op + grand total. Conservative rapid speed
            // until we add a per-machine override; matches typical hobby CNC
            // values (5000 mm/min ≈ 197 ipm).
            const RAPID_MM_MIN: f64 = 5000.0;
            let op_metrics: Vec<openmill_core::ToolpathMetrics> = self.toolpaths.iter()
                .map(|paths| openmill_core::aggregate_metrics(paths, RAPID_MM_MIN))
                .collect();
            let durations: Vec<f64> = op_metrics.iter().map(|m| m.total_minutes()).collect();
            let mut job_total = openmill_core::ToolpathMetrics::default();
            for m in &op_metrics { job_total.merge(m); }
            if job_total.total_minutes() > 0.0 {
                ui.weak(format!(
                    "Total: {} ({:.0} mm cut + {:.0} mm rapid)",
                    fmt_minutes(job_total.total_minutes()),
                    job_total.cut_distance_mm,
                    job_total.rapid_distance_mm,
                ));
                ui.weak(format!(
                    "       cut {} · rapid {}",
                    fmt_minutes(job_total.cut_minutes),
                    fmt_minutes(job_total.rapid_minutes),
                ));
                ui.separator();
            }

            let mut selection_changed = false;
            // Defer reorder until after the loop so we don't mutate `self`
            // while iterating its operations Vec.
            let mut move_up: Option<usize> = None;
            let mut move_down: Option<usize> = None;
            let n_ops = self.job.operations.len();
            for (i, op) in self.job.operations.iter().enumerate() {
                let selected = self.selected_op == Some(i);
                let est = durations.get(i).copied().unwrap_or(0.0);
                let status = if i < self.toolpaths.len() && !self.toolpaths[i].is_empty() {
                    if est > 0.0 {
                        format!(" [{}]", fmt_minutes(est))
                    } else {
                        " [generated]".to_string()
                    }
                } else {
                    String::new()
                };
                let enabled = if op.enabled { "" } else { " (disabled)" };
                let label = format!("{}{}{}", op.name, enabled, status);
                ui.horizontal(|ui| {
                    // Reorder controls — ops execute in list order at export
                    // time, so users need to be able to shuffle them.
                    ui.add_enabled_ui(i > 0, |ui| {
                        if ui.small_button("⬆").on_hover_text("Move up").clicked() {
                            move_up = Some(i);
                        }
                    });
                    ui.add_enabled_ui(i + 1 < n_ops, |ui| {
                        if ui.small_button("⬇").on_hover_text("Move down").clicked() {
                            move_down = Some(i);
                        }
                    });
                    if ui.selectable_label(selected, &label).clicked() {
                        self.selected_op = if selected { None } else { Some(i) };
                        selection_changed = true;
                    }
                });
            }

            // Apply at most one reorder per frame. Swapping also moves the
            // matching toolpaths so the parallel array stays in sync, and
            // updates `selected_op` so the user keeps focus on the op they
            // just shuffled.
            let mut apply_swap = |a: usize, b: usize, this: &mut Self| {
                this.job.operations.swap(a, b);
                if a < this.toolpaths.len() && b < this.toolpaths.len() {
                    this.toolpaths.swap(a, b);
                }
                this.selected_op = match this.selected_op {
                    Some(x) if x == a => Some(b),
                    Some(x) if x == b => Some(a),
                    other => other,
                };
                // Op order affects stock progression — force a rebuild on the
                // next sim entry.
                this.sim_op_idx = None;
                this.sim_progress = 0.0;
                this.sim_prev_progress = 0.0;
                selection_changed = true;
            };
            if let Some(i) = move_up   { apply_swap(i, i - 1, self); }
            if let Some(i) = move_down { apply_swap(i, i + 1, self); }

            if selection_changed {
                self.update_sim_gcode();
                self.update_sim_viewport();
            }

            // Selected operation details
            if let Some(idx) = self.selected_op {
                if idx < self.job.operations.len() {
                    let metrics_for_selected = op_metrics.get(idx).copied();
                    // Geometric air-cut distance for the selected op.
                    // Hidden when the op hasn't generated yet or there's
                    // no model / tool to anchor the stock envelope.
                    let air_cut_mm = {
                        let stock = self.model.as_ref().map(|m| m.stock_aabb());
                        let tool_r = self.job.operations.get(idx)
                            .and_then(|op| self.job.tools.iter().find(|t| t.id == op.tool_id))
                            .map(|t| t.shape.diameter() * 0.5)
                            .unwrap_or(0.0);
                        self.toolpaths.get(idx).map(|paths| {
                            paths.iter().map(|tp| tp.air_cut_distance(stock.as_ref(), tool_r)).sum::<f64>()
                        }).unwrap_or(0.0)
                    };
                    ui.group(|ui| {
                        let op = &mut self.job.operations[idx];

                        ui.horizontal(|ui| {
                            ui.label("Name:");
                            ui.text_edit_singleline(&mut op.name);
                        });

                        ui.checkbox(&mut op.enabled, "Enabled");

                        // Per-op cycle-time and distance breakdown when the
                        // toolpath has been generated.
                        if let Some(m) = metrics_for_selected {
                            if m.total_minutes() > 0.0 {
                                ui.weak(format!(
                                    "Time {} · cut {} · rapid {}",
                                    fmt_minutes(m.total_minutes()),
                                    fmt_minutes(m.cut_minutes),
                                    fmt_minutes(m.rapid_minutes),
                                ));
                                ui.weak(format!(
                                    "Distance {:.0} mm ({:.0} cut + {:.0} rapid) · {} points",
                                    m.total_distance_mm(),
                                    m.cut_distance_mm,
                                    m.rapid_distance_mm,
                                    m.point_count,
                                ));
                                if air_cut_mm > 0.5 && m.cut_distance_mm > 0.0 {
                                    let pct = 100.0 * air_cut_mm / m.cut_distance_mm;
                                    ui.weak(format!(
                                        "Air-cut: {:.0} mm of {:.0} ({:.0}%) — dim segments in viewport",
                                        air_cut_mm, m.cut_distance_mm, pct,
                                    ));
                                }
                            }
                        }

                        // Strategy picker
                        ui.horizontal(|ui| {
                            ui.label("Strategy:");
                            egui::ComboBox::from_id_salt("op_strategy")
                                .selected_text(op.strategy.as_str())
                                .show_ui(ui, |ui| {
                                    for &s in STRATEGIES {
                                        if ui.selectable_label(op.strategy == s, s).clicked() {
                                            op.strategy = s.to_string();
                                            op.params = default_params_for(s);
                                        }
                                    }
                                });
                        });

                        // Tool picker
                        ui.horizontal(|ui| {
                            ui.label("Tool:");
                            egui::ComboBox::from_id_salt("op_tool")
                                .selected_text(
                                    self.job.tools.iter()
                                        .find(|t| t.id == op.tool_id)
                                        .map(|t| format!("T{} {}", t.id, t.name))
                                        .unwrap_or_else(|| format!("T{}", op.tool_id)),
                                )
                                .show_ui(ui, |ui| {
                                    for t in &self.job.tools {
                                        let lbl = format!("T{} — {}", t.id, t.name);
                                        if ui.selectable_label(op.tool_id == t.id, &lbl).clicked() {
                                            op.tool_id = t.id;
                                        }
                                    }
                                });
                        });

                        // Feed / speed / coolant — preset-driven, op-overridable.
                        ui.separator();
                        ui.label("Feeds & Speeds:");

                        // Preset picker (only when the op's tool has presets).
                        let tool_for_op = self.job.tools.iter().find(|t| t.id == op.tool_id).cloned();
                        if let Some(tool) = &tool_for_op {
                            if !tool.presets.is_empty() {
                                ui.horizontal(|ui| {
                                    ui.label("Preset:");
                                    egui::ComboBox::from_id_salt("op_preset")
                                        .selected_text("(apply preset…)")
                                        .show_ui(ui, |ui| {
                                            for (p_idx, p) in tool.presets.iter().enumerate() {
                                                if ui.selectable_label(false,
                                                    format!("{} — S{:.0} F{:.0}", p.name, p.spindle_rpm, p.feed_rate)
                                                ).clicked() {
                                                    op.spindle_speed = p.spindle_rpm;
                                                    op.feed_rate     = p.feed_rate;
                                                    op.plunge_rate   = p.plunge_rate;
                                                    op.coolant       = p.coolant;
                                                    self.log.push(format!(
                                                        "Applied preset \"{}\" (idx {}) to op \"{}\"",
                                                        p.name, p_idx, op.name
                                                    ));
                                                }
                                            }
                                        });
                                });
                            }
                        }

                        ui.horizontal(|ui| {
                            ui.label("Spindle:");
                            ui.add(egui::DragValue::new(&mut op.spindle_speed).speed(10.0).suffix(" RPM"));
                        });
                        ui.horizontal(|ui| {
                            ui.label("Feed:");
                            ui.add(egui::DragValue::new(&mut op.feed_rate).speed(10.0).suffix(" mm/min"))
                                .on_hover_text("Cutting feed rate. 0 = use strategy default.");
                        });
                        ui.horizontal(|ui| {
                            ui.label("Plunge:");
                            ui.add(egui::DragValue::new(&mut op.plunge_rate).speed(10.0).suffix(" mm/min"))
                                .on_hover_text("Plunge feed rate (lead-in moves). 0 = use cutting feed × 0.5.");
                        });
                        ui.horizontal(|ui| {
                            ui.label("Coolant:");
                            egui::ComboBox::from_id_salt("op_coolant")
                                .selected_text(op.coolant.label())
                                .show_ui(ui, |ui| {
                                    for c in [
                                        Coolant::None, Coolant::Mist, Coolant::Flood,
                                        Coolant::MistFlood, Coolant::Through,
                                    ] {
                                        ui.selectable_value(&mut op.coolant, c, c.label());
                                    }
                                });
                        });

                        ui.separator();
                        ui.horizontal(|ui| {
                            ui.label("Stock to leave:");
                            ui.add(egui::DragValue::new(&mut op.stock_to_leave).speed(0.05).range(0.0..=10.0).suffix(" mm"))
                                .on_hover_text(
                                    "Skin of material left on the part by this op so a finishing op can take it off.\n\
                                     Roughing typical: 0.2–0.5 mm.   Finishing: 0.0.\n\
                                     Auto-injected into the strategy params at generate time."
                                );
                        });

                        // ── Lead-in / lead-out ──────────────────────────
                        ui.separator();
                        ui.collapsing("Lead in / out", |ui| {
                            ui.horizontal(|ui| {
                                ui.label("Kind:");
                                egui::ComboBox::from_id_salt("lead_kind")
                                    .selected_text(match op.leads.kind {
                                        LeadKind::None => "None (vertical plunge)",
                                        LeadKind::Ramp => "Ramp",
                                        LeadKind::Arc  => "Arc",
                                    })
                                    .show_ui(ui, |ui| {
                                        ui.selectable_value(&mut op.leads.kind, LeadKind::None, "None (vertical plunge)");
                                        ui.selectable_value(&mut op.leads.kind, LeadKind::Ramp, "Ramp");
                                        ui.selectable_value(&mut op.leads.kind, LeadKind::Arc,  "Arc");
                                    });
                            });
                            ui.add_enabled_ui(op.leads.kind != LeadKind::None, |ui| {
                                ui.horizontal(|ui| {
                                    ui.label("Length / radius:");
                                    ui.add(egui::DragValue::new(&mut op.leads.length_mm)
                                        .speed(0.1).range(0.1..=50.0).suffix(" mm"));
                                });
                                if op.leads.kind == LeadKind::Ramp {
                                    ui.horizontal(|ui| {
                                        ui.label("Ramp angle:");
                                        ui.add(egui::DragValue::new(&mut op.leads.angle_deg)
                                            .speed(0.5).range(0.5..=45.0).suffix("°"));
                                    });
                                }
                                if op.leads.kind == LeadKind::Arc {
                                    ui.horizontal(|ui| {
                                        ui.label("Arc steps:");
                                        let mut s = op.leads.arc_steps as f64;
                                        if ui.add(egui::DragValue::new(&mut s)
                                            .speed(1.0).range(3.0..=32.0)).changed()
                                        {
                                            op.leads.arc_steps = s.max(3.0) as usize;
                                        }
                                    });
                                }
                                ui.weak("Applied as a post-pass. 5-axis paths (tilted axis) are skipped.");
                            });
                        });

                        ui.separator();
                        ui.label("Custom op G-code (run after spindle/coolant on):");
                        ui.add(
                            egui::TextEdit::multiline(&mut op.gcode_command)
                                .desired_rows(2)
                                .desired_width(f32::INFINITY)
                                .hint_text("e.g. G43 H1   (tool length offset)"),
                        );

                        // Strategy-specific params
                        ui.separator();
                        ui.label("Parameters:");
                        let model_aabb = self.model.as_ref().map(|m| m.aabb.clone());
                        show_strategy_params(ui, op, model_aabb.as_ref());

                        ui.separator();

                        ui.horizontal(|ui| {
                            if ui.button("Generate Toolpath").clicked() {
                                generate_requested = true;
                            }
                            if ui.button("Remove").clicked() {
                                remove_idx = Some(idx);
                            }
                        });
                    });
                }
            }

            if let Some(idx) = remove_idx {
                self.job.operations.remove(idx);
                if idx < self.toolpaths.len() {
                    self.toolpaths.remove(idx);
                }
                self.selected_op = None;
            }

            if generate_requested {
                self.generate_selected_toolpath();
            }

            ui.separator();

            // ── Generate All ────────────────────────────────────────────
            // Progress UI while a background task is running, otherwise a
            // single button to kick it off.
            if let Some(task) = self.generation.as_ref() {
                let frac = if task.total == 0 {
                    0.0
                } else {
                    task.completed as f32 / task.total as f32
                };
                let current_name = task.current
                    .and_then(|i| task.op_names.get(i))
                    .map(String::as_str)
                    .unwrap_or("");
                ui.horizontal(|ui| {
                    ui.label(format!("{}/{}", task.completed, task.total));
                    let bar = egui::ProgressBar::new(frac.clamp(0.0, 1.0))
                        .text(if current_name.is_empty() {
                            "generating…".to_string()
                        } else {
                            format!("⏳ {current_name}")
                        });
                    ui.add(bar);
                });
                if ui.button("Cancel").clicked() {
                    if let Some(t) = self.generation.as_ref() {
                        t.request_cancel();
                    }
                }
            } else {
                ui.horizontal(|ui| {
                    if ui.button("Generate All")
                        .on_hover_text("Generate toolpaths for every enabled operation in the background.")
                        .clicked()
                    {
                        self.action_generate_all();
                    }
                });
            }

            ui.separator();

            // Add operation
            if ui.button("Add Operation").clicked() {
                let tool = self.job.tools.first().cloned();
                let next_tool = tool.as_ref().map(|t| t.id).unwrap_or(1);
                // Seed feeds/coolant from the tool's default preset if it has one.
                let (rpm, feed, plunge, coolant) = tool
                    .as_ref()
                    .and_then(|t| t.default_preset.and_then(|i| t.presets.get(i)))
                    .map(|p| (p.spindle_rpm, p.feed_rate, p.plunge_rate, p.coolant))
                    .unwrap_or((10000.0, 0.0, 0.0, Coolant::None));
                let op = Operation {
                    name: format!("Op {}", self.job.operations.len() + 1),
                    tool_id: next_tool,
                    strategy: "3+2 Indexed".into(),
                    params: default_params_for("3+2 Indexed"),
                    spindle_speed: rpm,
                    feed_rate: feed,
                    plunge_rate: plunge,
                    coolant,
                    gcode_command: String::new(),
                    stock_to_leave: 0.0,
                    leads: LeadConfig::default(),
                    enabled: true,
                };
                self.job.operations.push(op);
                self.toolpaths.push(Vec::new());
                self.selected_op = Some(self.job.operations.len() - 1);
                self.log.push("Added operation.".into());
            }
        });
    }

    // ── Settings ─────────────────────────────────────────────────────────

    fn section_settings(&mut self, ui: &mut egui::Ui) {
        ui.collapsing("Settings", |ui| {
            let s = &mut self.job.settings;
            ui.horizontal(|ui| {
                ui.label("Safe Z:");
                ui.add(egui::DragValue::new(&mut s.safe_z).speed(0.5).suffix(" mm"));
            });
            ui.horizontal(|ui| {
                ui.label("Clearance Z:");
                ui.add(egui::DragValue::new(&mut s.clearance_z).speed(0.1).suffix(" mm"));
            });
            ui.horizontal(|ui| {
                ui.label("Lead-in:");
                ui.add(egui::DragValue::new(&mut s.lead_in).speed(0.1).suffix(" mm"));
            });
            ui.horizontal(|ui| {
                ui.label("Tolerance:");
                ui.add(egui::DragValue::new(&mut s.tolerance).speed(0.001).suffix(" mm"));
            });
        });
    }

    // ── Machine ──────────────────────────────────────────────────────────

    fn section_machine(&mut self, ui: &mut egui::Ui) {
        ui.collapsing("Machine", |ui| {
            ui.horizontal(|ui| {
                ui.label("Active Machine:");
                ui.strong(&self.job.machine.name);
            });
            ui.label(format!("Post: {}", self.job.machine.post_processor));
            
            if ui.button("🔧 Machine Library...").clicked() {
                self.active_tab = AppTab::Machine;
            }

            ui.separator();
            ui.label("Quick Travel Limits:");
            ui.horizontal(|ui| {
                ui.label("X:");
                ui.add(egui::DragValue::new(&mut self.job.machine.travel_limits.x.0).speed(1.0).suffix(" min"));
                ui.add(egui::DragValue::new(&mut self.job.machine.travel_limits.x.1).speed(1.0).suffix(" max"));
            });
            ui.horizontal(|ui| {
                ui.label("Y:");
                ui.add(egui::DragValue::new(&mut self.job.machine.travel_limits.y.0).speed(1.0).suffix(" min"));
                ui.add(egui::DragValue::new(&mut self.job.machine.travel_limits.y.1).speed(1.0).suffix(" max"));
            });
            ui.horizontal(|ui| {
                ui.label("Z:");
                ui.add(egui::DragValue::new(&mut self.job.machine.travel_limits.z.0).speed(1.0).suffix(" min"));
                ui.add(egui::DragValue::new(&mut self.job.machine.travel_limits.z.1).speed(1.0).suffix(" max"));
            });
        });
    }
}

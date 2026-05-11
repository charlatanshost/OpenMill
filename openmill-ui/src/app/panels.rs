//! Central viewport, bottom status/sim controls, and right-hand G-code panel.
//!
//! These three panels share a lot of state — the bottom panel's sim
//! controls drive the central panel's tool-pose interpolation, and the
//! right panel's G-code lines correspond to point indices animated in the
//! sim. Keeping them in one file makes it easy to reason about which
//! state they touch.

use eframe::egui;

use super::{OpenMillApp, ViewMode};

// ── Central panel ───────────────────────────────────────────────────────────

impl OpenMillApp {
    pub(super) fn show_central_panel(&mut self, ctx: &egui::Context) {
        egui::CentralPanel::default().show(ctx, |ui| {
            let size = ui.available_size();
            let (rect, response) =
                ui.allocate_exact_size(size, egui::Sense::click_and_drag());

            // Camera input.
            let scroll = if response.hovered() {
                ui.input(|i| i.smooth_scroll_delta.y)
            } else {
                0.0
            };
            self.camera.handle_input(&response, scroll);

            // ── Face picking ───────────────────────────────────────────────
            // A discrete left-click (no drag) while pick-face mode is active
            // raycasts the mesh, floods coplanar neighbours into a feature,
            // and highlights the cluster.
            if self.pick_face_mode && response.clicked() {
                if let Some(click_pos) = response.interact_pointer_pos() {
                    self.handle_face_pick(click_pos, rect);
                }
            }

            // Render and display.
            if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
                let w = (size.x as u32).max(1);
                let h = (size.y as u32).max(1);
                let show_mesh  = self.view_mode == ViewMode::Plan;
                let show_voxel = self.view_mode == ViewMode::Simulation;
                if self.show_envelope {
                    vp.upload_envelope(&rs.device, &self.job.machine.travel_limits);
                }
                // Push the user's current verify state into the voxel
                // uniform buffer so the fragment shader can route between
                // op-color and deviation rendering.
                vp.set_verify_uniforms(
                    &rs.queue,
                    self.verify_mode && self.sdf_uploaded,
                    self.verify_tolerance_mm,
                );
                vp.render(
                    rs, &self.camera, w, h,
                    self.sim_progress, self.sim_show_full_path,
                    show_mesh, show_voxel, self.show_envelope,
                    self.show_iso_surface,
                );

                ui.painter().image(
                    vp.texture_id,
                    rect,
                    egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)),
                    egui::Color32::WHITE,
                );
            }

            // Overlay: model info at top-left of viewport.
            if self.model.is_some() {
                let painter = ui.painter();
                let text_pos = rect.left_top() + egui::vec2(8.0, 8.0);
                let model = self.model.as_ref().unwrap();
                let aabb = &model.aabb;
                let sz = aabb.maxs - aabb.mins;
                let info = format!(
                    "{:.1} x {:.1} x {:.1} mm | {} verts | {} tris",
                    sz.x, sz.y, sz.z,
                    model.mesh.vertices().len(),
                    model.mesh.indices().len(),
                );
                painter.text(
                    text_pos,
                    egui::Align2::LEFT_TOP,
                    info,
                    egui::FontId::proportional(13.0),
                    egui::Color32::from_rgba_premultiplied(200, 200, 200, 180),
                );
            }

            // ── Camera preset toolbar (top-right overlay) ──────────────
            // Snap-to-view buttons matching the conventions in Fusion and
            // FreeCAD: Top / Front / Right / Iso / Fit. Rendered as a
            // floating UI on top of the wgpu texture so it stays anchored
            // even as the panel resizes.
            let toolbar_top_right = rect.right_top() + egui::vec2(-12.0, 12.0);
            egui::Area::new(egui::Id::new("camera_presets"))
                .fixed_pos(toolbar_top_right)
                .anchor(egui::Align2::RIGHT_TOP, egui::vec2(-12.0, 12.0))
                .show(ctx, |ui| {
                    egui::Frame::popup(ui.style())
                        .fill(egui::Color32::from_rgba_premultiplied(20, 20, 25, 200))
                        .show(ui, |ui| {
                            ui.horizontal(|ui| {
                                if ui.small_button("Top").on_hover_text("Top view (looking down -Z)").clicked() {
                                    self.camera.view_top();
                                }
                                if ui.small_button("Front").on_hover_text("Front view (looking along +Y)").clicked() {
                                    self.camera.view_front();
                                }
                                if ui.small_button("Right").on_hover_text("Right view (looking along -X)").clicked() {
                                    self.camera.view_right();
                                }
                                if ui.small_button("Iso").on_hover_text("Three-quarter isometric").clicked() {
                                    self.camera.view_iso();
                                }
                                ui.separator();
                                if ui.small_button("Fit").on_hover_text("Frame the part in view").clicked() {
                                    if let Some(m) = self.model.as_ref() {
                                        let mins = m.aabb.mins;
                                        let maxs = m.aabb.maxs;
                                        self.camera.focus_on_aabb(
                                            [mins.x, mins.y, mins.z],
                                            [maxs.x, maxs.y, maxs.z],
                                        );
                                    }
                                }
                            });
                        });
                });
        });
    }
}

// ── Bottom panel (status log) ───────────────────────────────────────────────

impl OpenMillApp {
    pub(super) fn show_bottom_panel(&mut self, ctx: &egui::Context) {
        egui::TopBottomPanel::bottom("status_bar")
            .resizable(true)
            .default_height(100.0)
            .show(ctx, |ui| {
                // ── View mode (always visible) ──────────────────────────
                ui.horizontal(|ui| {
                    ui.heading("View:");
                    let was_plan = self.view_mode == ViewMode::Plan;
                    if ui.selectable_label(was_plan, "🧭 Plan").on_hover_text(
                        "Show the original part mesh — best for setting up tools and reviewing toolpaths."
                    ).clicked() && !was_plan {
                        self.view_mode = ViewMode::Plan;
                        // Stop the sim playhead — Plan view doesn't carve.
                        self.sim_playing = false;
                    }
                    let was_sim = self.view_mode == ViewMode::Simulation;
                    ui.separator();
                    ui.checkbox(&mut self.show_envelope, "Travel envelope")
                        .on_hover_text("Show the machine's linear-axis travel limits as a wireframe box.");
                    ui.separator();
                    if ui.selectable_label(was_sim, "🛠 Simulation").on_hover_text(
                        "Hide the part mesh and show the voxel stock as it gets carved."
                    ).clicked() && !was_sim {
                        self.view_mode = ViewMode::Simulation;
                        // Force a fresh voxel rebuild from the start of the
                        // current op (the update loop replays prior ops next
                        // frame because sim_op_idx no longer matches).
                        self.sim_op_idx = None;
                        self.sim_progress = 0.0;
                        self.sim_prev_progress = 0.0;
                        if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
                            vp.reset_voxel(&rs.queue);
                            vp.upload_collisions(&rs.device, &[]);
                        }
                        self.sim_collisions.clear();
                    }
                    // ── Verify deviation heatmap (Sim view only). ───────
                    ui.add_enabled_ui(self.view_mode == ViewMode::Simulation && self.model.is_some(), |ui| {
                        let was = self.verify_mode;
                        if ui.checkbox(&mut self.verify_mode, "✓ Verify")
                            .on_hover_text(
                                "Color the carved stock surface by signed distance to the target part.\n\
                                 Green = on size, blue = excess material, red = gouge."
                            ).changed() && self.verify_mode && !was
                        {
                            // Toggle just flipped on — compute + upload the
                            // SDF once for the current model. The first
                            // frame may stutter (~200 ms for a typical
                            // part); the result is cached until the model
                            // or its position changes.
                            self.compute_and_upload_sdf();
                        }
                    });
                    if self.verify_mode {
                        ui.add(egui::DragValue::new(&mut self.verify_tolerance_mm)
                            .speed(0.005).range(0.001..=5.0).suffix(" mm tol"))
                            .on_hover_text("Half-width of the green \"on size\" band.");
                    }
                    ui.add_enabled_ui(self.view_mode == ViewMode::Simulation, |ui| {
                        let was = self.show_iso_surface;
                        if ui.checkbox(&mut self.show_iso_surface, "✦ Smooth")
                            .on_hover_text(
                                "Render the carved stock as a marching-cubes iso-surface\n\
                                 instead of ray-marching the voxel grid. Smooth lit mesh\n\
                                 like Fusion / FreeCAD."
                            ).changed() && self.show_iso_surface && !was
                        {
                            // First time it flips on, kick a one-shot
                            // extraction so the mesh exists immediately.
                            // Subsequent frames re-extract as the voxel
                            // grid is carved.
                            self.extract_iso_surface_once();
                        }
                    });
                });
                ui.separator();

                if let Some(op_idx) = self.selected_op {
                    if op_idx < self.toolpaths.len() && !self.toolpaths[op_idx].is_empty() {
                        ui.horizontal(|ui| {
                            ui.heading("Simulation");
                            let sim_active = self.view_mode == ViewMode::Simulation;
                            ui.add_enabled_ui(sim_active, |ui| {
                                if ui.button(if self.sim_playing { "Pause" } else { "Play" }).clicked() {
                                    self.sim_playing = !self.sim_playing;
                                }
                                if ui.button("Reset").clicked() {
                                    self.sim_progress = 0.0;
                                    self.sim_prev_progress = 0.0;
                                    self.sim_playing = false;
                                    if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
                                        vp.reset_voxel(&rs.queue);
                                        vp.upload_collisions(&rs.device, &[]);
                                    }
                                    // Force rebuild of prior-op carving on next frame.
                                    self.sim_op_idx = None;
                                    self.sim_collisions.clear();
                                }
                                ui.add(egui::Slider::new(&mut self.sim_progress, 0.0..=1.0).text("Progress"));
                                ui.add(egui::Slider::new(&mut self.sim_speed, 0.1..=5.0).text("Speed"));
                                ui.checkbox(&mut self.sim_show_full_path, "Show Full Path");
                            });
                            if !sim_active {
                                ui.weak("(switch to Simulation view to carve)");
                            }
                        });
                        ui.separator();
                    }
                }

                ui.horizontal(|ui| {
                    ui.heading("Log");
                    if ui.small_button("Clear").clicked() {
                        self.log.clear();
                    }
                });
                ui.separator();
                egui::ScrollArea::vertical()
                    .stick_to_bottom(true)
                    .show(ui, |ui| {
                        for msg in &self.log {
                            ui.label(msg);
                        }
                    });
            });
    }

    pub(super) fn show_right_panel(&mut self, ctx: &egui::Context) {
        if self.selected_op.is_none() || self.sim_gcode.is_empty() {
            return;
        }

        let total_points = if let Some(idx) = self.selected_op {
            if idx < self.toolpaths.len() {
                self.toolpaths[idx].iter().map(|p| p.points.len()).sum::<usize>()
            } else { 1 }
        } else { 1 };

        egui::SidePanel::right("gcode_terminal")
            .resizable(true)
            .default_width(250.0)
            .show(ctx, |ui| {
                ui.heading("G-Code Terminal");
                ui.separator();
                
                egui::ScrollArea::vertical().show(ui, |ui| {
                    for (line_text, opt_idx) in &self.sim_gcode {
                        let text = line_text.trim_end();
                        if text.is_empty() { continue; }
                        
                        let label = egui::SelectableLabel::new(false, text);
                        let response = ui.add(label);
                        if response.clicked() {
                            if let Some(idx) = opt_idx {
                                // Jump simulation progress to this index
                                if total_points > 1 {
                                    self.sim_progress = *idx as f32 / (total_points - 1) as f32;
                                    self.sim_playing = false;
                                }
                            }
                        }
                    }
                });
            });
    }

    pub(super) fn get_sim_tool_pose(&self) -> Option<(nalgebra::Isometry3<f32>, openmill_core::Tool)> {
        self.get_tool_pose_at(self.sim_progress)
    }

    pub(super) fn get_tool_pose_at(&self, progress: f32) -> Option<(nalgebra::Isometry3<f32>, openmill_core::Tool)> {
        if let Some(op_idx) = self.selected_op {
            if op_idx < self.toolpaths.len() && !self.toolpaths[op_idx].is_empty() {
                let all_paths = &self.toolpaths[op_idx];
                let mut total_points = 0;
                for p in all_paths {
                    total_points += p.points.len();
                }
                if total_points == 0 { return None; }

                let exact_idx = progress * ((total_points - 1) as f32);
                let idx1 = exact_idx.floor() as usize;
                let idx2 = (idx1 + 1).min(total_points - 1);
                let t = exact_idx - idx1 as f32;

                let mut flat_points = Vec::with_capacity(total_points);
                let mut current_tool = None;
                for path in all_paths {
                    for pt in &path.points {
                        flat_points.push(pt);
                        if current_tool.is_none() {
                            current_tool = self.job.tools.iter().find(|tool| tool.id == path.tool_id).cloned();
                        }
                    }
                }
                
                if flat_points.is_empty() { return None; }
                let tool = current_tool.unwrap_or_else(|| openmill_core::Tool::flat_end(1, "Fallback", 6.0, 20.0));

                let p1 = flat_points[idx1];
                let p2 = flat_points[idx2];

                let pos = p1.position.coords.lerp(&p2.position.coords, t as f64);
                let orient_v = p1.orientation.into_inner().lerp(&p2.orientation.into_inner(), t as f64).normalize();

                let translation = nalgebra::Translation3::new(pos.x as f32, pos.y as f32, pos.z as f32);
                let rotation = nalgebra::UnitQuaternion::rotation_between(
                    &nalgebra::Vector3::z_axis(),
                    &nalgebra::Vector3::new(orient_v.x as f32, orient_v.y as f32, orient_v.z as f32)
                ).unwrap_or(nalgebra::UnitQuaternion::identity());

                return Some((nalgebra::Isometry3::from_parts(translation, rotation), tool));
            }
        }
        None
    }
}

//! Top menu bar and tab bar.
//!
//! `show_menu_bar` is the File / Edit / Verify / Export / Help dropdown
//! at the top of the window. `show_tab_bar` renders the row below it
//! (Manufacturing / Tool Library / Machine).

use eframe::egui;
use openmill_core::Job;
use openmill_post::{get_post, POST_PROCESSOR_NAMES};

use super::{AppTab, OpenMillApp};

// ── Menu bar ────────────────────────────────────────────────────────────────

impl OpenMillApp {
    pub(super) fn show_menu_bar(&mut self, ctx: &egui::Context) {
        egui::TopBottomPanel::top("menu_bar").show(ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                // ── File ─────────────────────────────────────────────
                ui.menu_button("File", |ui| {
                    if ui.button("New Job").clicked() {
                        self.job = Job::default();
                        self.model = None;
                        self.imported_model = None;
                        self.toolpaths.clear();
                        self.selected_tool = None;
                        self.selected_op = None;
                        self.features.clear();
                        self.sim_op_idx = None;
                        self.sim_progress = 0.0;
                        self.sim_prev_progress = 0.0;
                        self.sim_collisions.clear();
                        self.sim_gcode.clear();
                        if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
                            let _ = rs;
                            vp.clear_pick();
                            vp.upload_collisions(&rs.device, &[]);
                        }
                        self.history.reset(&self.job);
                        self.autosave.reset_baseline();
                        self.log.push("New job created.".into());
                        ui.close_menu();
                    }

                    if ui.button("Open Job...").clicked() {
                        self.action_open_job();
                        ui.close_menu();
                    }

                    if ui.button("Save Job...").clicked() {
                        self.action_save_job();
                        ui.close_menu();
                    }

                    ui.separator();

                    if ui.button("Open Project (.omp)...")
                        .on_hover_text("Single-file project bundle: job + embedded model.")
                        .clicked()
                    {
                        self.action_open_bundle();
                        ui.close_menu();
                    }

                    if ui.button("Save Project (.omp)...")
                        .on_hover_text("Bundle job and current model into a single portable file.")
                        .clicked()
                    {
                        self.action_save_bundle();
                        ui.close_menu();
                    }

                    ui.separator();

                    if ui.button("Import Model (STL/3MF)...").clicked() {
                        self.action_import_model();
                        ui.close_menu();
                    }
                });

                // ── Edit ─────────────────────────────────────────────
                ui.menu_button("Edit", |ui| {
                    let undo_btn = egui::Button::new("Undo")
                        .shortcut_text(ctx.format_shortcut(&egui::KeyboardShortcut::new(
                            egui::Modifiers::CTRL, egui::Key::Z,
                        )));
                    if ui.add_enabled(self.history.can_undo(), undo_btn).clicked() {
                        if self.history.undo(&mut self.job) {
                            self.on_job_replaced();
                            self.log.push("Undo".into());
                        }
                        ui.close_menu();
                    }
                    let redo_btn = egui::Button::new("Redo")
                        .shortcut_text(ctx.format_shortcut(&egui::KeyboardShortcut::new(
                            egui::Modifiers::CTRL, egui::Key::Y,
                        )));
                    if ui.add_enabled(self.history.can_redo(), redo_btn).clicked() {
                        if self.history.redo(&mut self.job) {
                            self.on_job_replaced();
                            self.log.push("Redo".into());
                        }
                        ui.close_menu();
                    }
                });

                // ── Verify ────────────────────────────────────────────
                ui.menu_button("Verify", |ui| {
                    if ui.button("✓ Verify all toolpaths").on_hover_text(
                        "Run pre-export checks: travel limits, rotary range, undefined feeds, gouges, holder collisions."
                    ).clicked() {
                        self.action_verify();
                        ui.close_menu();
                    }
                });

                // ── Export ────────────────────────────────────────────
                ui.menu_button("Export", |ui| {
                    if ui.button(format!("⚙ Machine Default ({})", self.job.machine.post_processor)).clicked() {
                        let post = get_post(&self.job.machine.post_processor);
                        self.action_export_gcode(post.as_ref());
                        ui.close_menu();
                    }
                    ui.separator();
                    for &name in POST_PROCESSOR_NAMES {
                        if ui.button(format!("{name} G-code...")).clicked() {
                            let post = get_post(name);
                            self.action_export_gcode(post.as_ref());
                            ui.close_menu();
                        }
                    }
                    ui.separator();
                    if ui.button("Setup Sheet (HTML)...")
                        .on_hover_text("Printable shop-floor summary: tools, ops, runtime estimates.")
                        .clicked()
                    {
                        self.action_export_setup_sheet();
                        ui.close_menu();
                    }
                });

                // ── Help ─────────────────────────────────────────────
                ui.menu_button("Help", |ui| {
                    if ui.button("About").clicked() {
                        self.log.push(
                            "OpenMill — Open-source 5-axis CAM. https://github.com/charlatanshost/OpenMill".into(),
                        );
                        ui.close_menu();
                    }
                });
            });
        });
    }

    pub(super) fn show_tab_bar(&mut self, ctx: &egui::Context) {
        egui::TopBottomPanel::top("tab_bar")
            .frame(egui::Frame::none().fill(ctx.style().visuals.window_fill()))
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 0.0;
                    
                    let tabs = [
                        (AppTab::Manufacturing, "Manufacturing"),
                        (AppTab::Tools, "Tool Library"),
                        (AppTab::Machine, "Machine"),
                    ];

                    for (tab, label) in tabs {
                        if ui.selectable_value(&mut self.active_tab, tab, label).clicked() {
                            // Tab changed
                        }
                    }
                });
                ui.separator();
            });
    }
}

//! Machine + Tool library tabs.
//!
//! Top-tab content for the `Machine` and `Tools` selections. Both render a
//! full-window panel with their own sub-layouts.

use std::path::PathBuf;

use eframe::egui;
use openmill_core::*;
use openmill_post::POST_PROCESSOR_NAMES;

use super::OpenMillApp;

// ── Tool shape mutators ─────────────────────────────────────────────────────

fn set_shape_diameter(shape: &mut ToolShape, d: f64) {
    match shape {
        ToolShape::FlatEnd { diameter, .. }
        | ToolShape::BallEnd { diameter, .. }
        | ToolShape::BullNose { diameter, .. }
        | ToolShape::ChamferMill { diameter, .. }
        | ToolShape::Drill { diameter, .. }
        | ToolShape::Lollipop { diameter, .. }
        | ToolShape::Dovetail { diameter, .. }
        | ToolShape::ThreadMill { diameter, .. } => *diameter = d,
        ToolShape::TaperedMill { tip_diameter, .. } => *tip_diameter = d, // User edits tip for tapered
    }
}

fn set_shape_flute_length(shape: &mut ToolShape, fl: f64) {
    match shape {
        ToolShape::FlatEnd { flute_length, .. }
        | ToolShape::BallEnd { flute_length, .. }
        | ToolShape::BullNose { flute_length, .. }
        | ToolShape::ChamferMill { flute_length, .. }
        | ToolShape::Drill { flute_length, .. }
        | ToolShape::TaperedMill { flute_length, .. }
        | ToolShape::Lollipop { flute_length, .. }
        | ToolShape::Dovetail { flute_length, .. }
        | ToolShape::ThreadMill { flute_length, .. } => *flute_length = fl,
    }
}

impl OpenMillApp {
    pub(super) fn show_machine_tab(&mut self, ctx: &egui::Context) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("Machine Library");
            ui.separator();

            ui.horizontal(|ui| {
                if ui.button("➕ Create New Machine").clicked() {
                    self.machine_library.machines.push(openmill_core::kinematics::default_machines::default_trunnion_config());
                }
                if ui.button("💾 Save Library").clicked() {
                    for m in &self.machine_library.machines {
                        let _ = MachineLibrary::save_machine("machines", m);
                    }
                    self.log.push("Machine library saved to /machines directory.".into());
                }
                if ui.button("🔄 Reload From Disk").clicked() {
                    if let Ok(lib) = MachineLibrary::load("machines") {
                        self.machine_library = lib;
                        self.log.push("Machine library reloaded.".into());
                    }
                }
            });

            ui.separator();

            let mut to_delete = None;
            egui::ScrollArea::vertical().show(ui, |ui| {
                for (idx, machine) in self.machine_library.machines.iter_mut().enumerate() {
                    let _id = ui.make_persistent_id(format!("machine_{idx}"));
                    ui.group(|ui| {
                        // ── Header row ──────────────────────────────────
                        ui.horizontal(|ui| {
                            ui.heading(&machine.name);
                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                if ui.button("🗑").clicked() {
                                    to_delete = Some(idx);
                                }
                                if ui.button("✅ Select for Job").clicked() {
                                    self.job.machine = machine.clone();
                                    self.log.push(format!("Machine \"{}\" is now active for this job.", machine.name));
                                }
                            });
                        });

                        ui.horizontal(|ui| {
                            ui.label("Name:");
                            ui.text_edit_singleline(&mut machine.name);
                        });

                        ui.separator();

                        // ── Post-Processor Selection ────────────────────
                        ui.group(|ui| {
                            ui.strong("Post-Processor");

                            ui.horizontal(|ui| {
                                ui.label("Controller:");
                                egui::ComboBox::from_id_salt(format!("post_sel_{idx}"))
                                    .selected_text(&machine.post_processor)
                                    .show_ui(ui, |ui| {
                                        for &name in POST_PROCESSOR_NAMES {
                                            ui.selectable_value(&mut machine.post_processor, name.to_string(), name);
                                        }
                                    });
                            });

                            ui.horizontal(|ui| {
                                ui.label("Program Number:");
                                ui.add(egui::DragValue::new(&mut machine.post_config.program_number).speed(1.0));
                            });

                            ui.horizontal(|ui| {
                                ui.label("Work Offset:");
                                egui::ComboBox::from_id_salt(format!("wcs_{idx}"))
                                    .selected_text(&machine.post_config.work_offset)
                                    .show_ui(ui, |ui| {
                                        for wcs in &["G54", "G55", "G56", "G57", "G58", "G59"] {
                                            ui.selectable_value(&mut machine.post_config.work_offset, wcs.to_string(), *wcs);
                                        }
                                    });
                            });

                            ui.horizontal(|ui| {
                                ui.label("Units:");
                                let is_metric = machine.post_config.units == Units::Metric;
                                if ui.selectable_label(is_metric, "Metric (G21)").clicked() {
                                    machine.post_config.units = Units::Metric;
                                }
                                if ui.selectable_label(!is_metric, "Imperial (G20)").clicked() {
                                    machine.post_config.units = Units::Imperial;
                                }
                            });
                        });

                        ui.separator();

                        // ── Axis Limits & Kinematics ────────────────────
                        ui.columns(2, |cols| {
                            cols[0].group(|ui| {
                                ui.strong("Linear Axis Limits (mm)");
                                ui.horizontal(|ui| {
                                    ui.label("X:");
                                    ui.add(egui::DragValue::new(&mut machine.travel_limits.x.0).speed(1.0).suffix(" min"));
                                    ui.add(egui::DragValue::new(&mut machine.travel_limits.x.1).speed(1.0).suffix(" max"));
                                });
                                ui.horizontal(|ui| {
                                    ui.label("Y:");
                                    ui.add(egui::DragValue::new(&mut machine.travel_limits.y.0).speed(1.0).suffix(" min"));
                                    ui.add(egui::DragValue::new(&mut machine.travel_limits.y.1).speed(1.0).suffix(" max"));
                                });
                                ui.horizontal(|ui| {
                                    ui.label("Z:");
                                    ui.add(egui::DragValue::new(&mut machine.travel_limits.z.0).speed(1.0).suffix(" min"));
                                    ui.add(egui::DragValue::new(&mut machine.travel_limits.z.1).speed(1.0).suffix(" max"));
                                });
                            });

                            cols[1].group(|ui| {
                                ui.strong("Rotary Kinematics");
                                match &mut machine.axes {
                                    KinematicType::TableTable { a_axis, c_axis } => {
                                        ui.label("Table-Table (A-C Trunnion)");
                                        ui.collapsing("A Axis (Tilt)", |ui| {
                                            let mut min = a_axis.min_angle.to_degrees();
                                            let mut max = a_axis.max_angle.to_degrees();
                                            let mut home = a_axis.home_angle.to_degrees();
                                            ui.horizontal(|ui| {
                                                ui.label("Range:");
                                                if ui.add(egui::DragValue::new(&mut min).speed(1.0).suffix("°")).changed() { a_axis.min_angle = min.to_radians(); }
                                                ui.label("to");
                                                if ui.add(egui::DragValue::new(&mut max).speed(1.0).suffix("°")).changed() { a_axis.max_angle = max.to_radians(); }
                                            });
                                            ui.horizontal(|ui| {
                                                ui.label("Home:");
                                                if ui.add(egui::DragValue::new(&mut home).speed(1.0).suffix("°")).changed() { a_axis.home_angle = home.to_radians(); }
                                            });
                                        });
                                        ui.collapsing("C Axis (Rotate)", |ui| {
                                            let mut min = c_axis.min_angle.to_degrees();
                                            let mut max = c_axis.max_angle.to_degrees();
                                            let mut home = c_axis.home_angle.to_degrees();
                                            ui.horizontal(|ui| {
                                                ui.label("Range:");
                                                if ui.add(egui::DragValue::new(&mut min).speed(1.0).suffix("°")).changed() { c_axis.min_angle = min.to_radians(); }
                                                ui.label("to");
                                                if ui.add(egui::DragValue::new(&mut max).speed(1.0).suffix("°")).changed() { c_axis.max_angle = max.to_radians(); }
                                            });
                                            ui.horizontal(|ui| {
                                                ui.label("Home:");
                                                if ui.add(egui::DragValue::new(&mut home).speed(1.0).suffix("°")).changed() { c_axis.home_angle = home.to_radians(); }
                                            });
                                        });
                                    }
                                }
                            });
                        });

                        ui.horizontal(|ui| {
                            ui.label("Pivot Offset (mm):");
                            ui.add(egui::DragValue::new(&mut machine.pivot_offset.x).speed(0.1).suffix(" X"));
                            ui.add(egui::DragValue::new(&mut machine.pivot_offset.y).speed(0.1).suffix(" Y"));
                            ui.add(egui::DragValue::new(&mut machine.pivot_offset.z).speed(0.1).suffix(" Z"));
                        });
                    });
                    ui.add_space(10.0);
                }
            });

            if let Some(idx) = to_delete {
                let name = self.machine_library.machines[idx].name.clone();
                self.machine_library.machines.remove(idx);
                let _ = MachineLibrary::delete_machine("machines", &name);
            }
        });
    }

    pub(super) fn show_tools_tab(&mut self, ctx: &egui::Context) {
        egui::SidePanel::left("tool_list_panel").width_range(200.0..=300.0).show(ctx, |ui| {
            ui.heading("Tool Library");
            ui.separator();

            egui::ScrollArea::vertical().show(ui, |ui| {
                for (i, tool) in self.job.tools.iter().enumerate() {
                    let selected = self.selected_tool == Some(i);
                    ui.horizontal(|ui| {
                        if ui.selectable_label(selected, format!("T{} — {}", tool.id, tool.name)).clicked() {
                            self.selected_tool = Some(i);
                        }
                    });
                }
            });

            ui.separator();
            if ui.button("Add New Tool").clicked() {
                let next_id = self.job.tools.iter().map(|t| t.id).max().unwrap_or(0) + 1;
                let tool = Tool::flat_end(next_id, "New Tool", 6.0, 20.0);
                self.save_tool_to_dir(&tool);
                self.job.tools.push(tool);
                self.selected_tool = Some(self.job.tools.len() - 1);
            }
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            let mut save_requested = false;
            let mut delete_requested = false;
            if let Some(idx) = self.selected_tool {
                if idx < self.job.tools.len() {
                    ui.columns(2, |cols| {
                        // Left Column: Parameters
                        cols[0].group(|ui| {
                            ui.heading("Tool Parameters");
                            ui.separator();
                            let tool = &mut self.job.tools[idx];

                            ui.horizontal(|ui| {
                                ui.label("Name:");
                                ui.text_edit_singleline(&mut tool.name);
                            });
                            ui.horizontal(|ui| {
                                ui.label("ID / T-Number:");
                                ui.add(egui::DragValue::new(&mut tool.id));
                            });

                            ui.separator();
                            ui.label("Geometry:");
                            let mut ge_changed = false;
                            egui::ComboBox::from_id_salt("edit_tool_type")
                                .selected_text(match &tool.shape {
                                    ToolShape::FlatEnd { .. } => "Flat End",
                                    ToolShape::BallEnd { .. } => "Ball End",
                                    ToolShape::BullNose { .. } => "Bull Nose",
                                    ToolShape::ChamferMill { .. } => "Chamfer Mill",
                                    ToolShape::Drill { .. } => "Drill",
                                    ToolShape::TaperedMill { .. } => "Tapered Mill",
                                    ToolShape::Lollipop { .. } => "Lollipop",
                                    ToolShape::Dovetail { .. } => "Dovetail",
                                    ToolShape::ThreadMill { .. } => "Thread Mill",
                                })
                                .show_ui(ui, |ui| {
                                    let d = tool.shape.diameter();
                                    let fl = tool.shape.flute_length();
                                    if ui.selectable_label(matches!(tool.shape, ToolShape::FlatEnd{..}), "Flat End").clicked() { tool.shape = ToolShape::FlatEnd { diameter: d, flute_length: fl }; ge_changed = true; }
                                    if ui.selectable_label(matches!(tool.shape, ToolShape::BallEnd{..}), "Ball End").clicked() { tool.shape = ToolShape::BallEnd { diameter: d, flute_length: fl }; ge_changed = true; }
                                    if ui.selectable_label(matches!(tool.shape, ToolShape::BullNose{..}), "Bull Nose").clicked() { tool.shape = ToolShape::BullNose { diameter: d, corner_radius: 1.0, flute_length: fl }; ge_changed = true; }
                                    if ui.selectable_label(matches!(tool.shape, ToolShape::ChamferMill{..}), "Chamfer Mill").clicked() { tool.shape = ToolShape::ChamferMill { tip_diameter: 0.1, diameter: d, taper_angle: 90.0, flute_length: fl }; ge_changed = true; }
                                    if ui.selectable_label(matches!(tool.shape, ToolShape::Drill{..}), "Drill").clicked() { tool.shape = ToolShape::Drill { diameter: d, point_angle: 118.0, flute_length: fl }; ge_changed = true; }
                                    if ui.selectable_label(matches!(tool.shape, ToolShape::TaperedMill{..}), "Tapered Mill").clicked() { tool.shape = ToolShape::TaperedMill { tip_diameter: 1.0, taper_angle: 3.0, flute_length: fl }; ge_changed = true; }
                                    if ui.selectable_label(matches!(tool.shape, ToolShape::Lollipop{..}), "Lollipop").clicked() { tool.shape = ToolShape::Lollipop { diameter: d, neck_diameter: d*0.7, flute_length: fl }; ge_changed = true; }
                                    if ui.selectable_label(matches!(tool.shape, ToolShape::Dovetail{..}), "Dovetail").clicked() { tool.shape = ToolShape::Dovetail { diameter: d, width: 5.0, angle: 60.0, flute_length: fl }; ge_changed = true; }
                                    if ui.selectable_label(matches!(tool.shape, ToolShape::ThreadMill{..}), "Thread Mill").clicked() { tool.shape = ToolShape::ThreadMill { diameter: d, pitch: 1.5, num_teeth: 1, flute_length: fl }; ge_changed = true; }
                                });

                            ui.horizontal(|ui| {
                                ui.label("Diameter:");
                                let mut d = tool.shape.diameter();
                                if ui.add(egui::DragValue::new(&mut d).speed(0.1).suffix(" mm")).changed() {
                                    set_shape_diameter(&mut tool.shape, d);
                                    ge_changed = true;
                                }
                            });
                            ui.horizontal(|ui| {
                                ui.label("Flute length:");
                                let mut fl = tool.shape.flute_length();
                                if ui.add(egui::DragValue::new(&mut fl).speed(0.1).suffix(" mm")).changed() {
                                    set_shape_flute_length(&mut tool.shape, fl);
                                    ge_changed = true;
                                }
                            });

                            match &mut tool.shape {
                                ToolShape::BullNose { corner_radius, .. } => {
                                    ui.horizontal(|ui| {
                                        ui.label("Corner radius:");
                                        ge_changed |= ui.add(egui::DragValue::new(corner_radius).speed(0.1).suffix(" mm")).changed();
                                    });
                                }
                                ToolShape::ChamferMill { tip_diameter, taper_angle, .. } => {
                                    ui.horizontal(|ui| { ui.label("Tip diameter:"); ge_changed |= ui.add(egui::DragValue::new(tip_diameter).speed(0.1)).changed(); });
                                    ui.horizontal(|ui| { ui.label("Taper angle:"); ge_changed |= ui.add(egui::DragValue::new(taper_angle).speed(1.0)).changed(); });
                                }
                                ToolShape::Drill { point_angle, .. } => {
                                    ui.horizontal(|ui| { ui.label("Point angle:"); ge_changed |= ui.add(egui::DragValue::new(point_angle).speed(1.0)).changed(); });
                                }
                                ToolShape::TaperedMill { tip_diameter, taper_angle, .. } => {
                                    ui.horizontal(|ui| { ui.label("Tip diameter:"); ge_changed |= ui.add(egui::DragValue::new(tip_diameter).speed(0.1)).changed(); });
                                    ui.horizontal(|ui| { ui.label("Taper angle:"); ge_changed |= ui.add(egui::DragValue::new(taper_angle).speed(0.1)).changed(); });
                                }
                                ToolShape::Lollipop { neck_diameter, .. } => {
                                    ui.horizontal(|ui| { ui.label("Neck diameter:"); ge_changed |= ui.add(egui::DragValue::new(neck_diameter).speed(0.1)).changed(); });
                                }
                                ToolShape::Dovetail { width, angle, .. } => {
                                    ui.horizontal(|ui| { ui.label("Slot width:"); ge_changed |= ui.add(egui::DragValue::new(width).speed(0.1)).changed(); });
                                    ui.horizontal(|ui| { ui.label("Angle:"); ge_changed |= ui.add(egui::DragValue::new(angle).speed(1.0)).changed(); });
                                }
                                ToolShape::ThreadMill { pitch, num_teeth, .. } => {
                                    ui.horizontal(|ui| { ui.label("Pitch:"); ge_changed |= ui.add(egui::DragValue::new(pitch).speed(0.01)).changed(); });
                                    ui.horizontal(|ui| {
                                        ui.label("Teeth:");
                                        let mut nt = *num_teeth as f64;
                                        if ui.add(egui::DragValue::new(&mut nt).speed(1.0)).changed() {
                                            *num_teeth = nt.max(1.0) as usize;
                                            ge_changed = true;
                                        }
                                    });
                                }
                                _ => {}
                            }

                            ge_changed |= ui.horizontal(|ui| {
                                ui.label("Overall length:");
                                ui.add(egui::DragValue::new(&mut tool.overall_length).speed(1.0).suffix(" mm")).changed()
                            }).inner;
                            ge_changed |= ui.horizontal(|ui| {
                                ui.label("Shank diameter:");
                                ui.add(egui::DragValue::new(&mut tool.shank_diameter).speed(0.1).suffix(" mm")).changed()
                            }).inner;
                            ge_changed |= ui.horizontal(|ui| {
                                ui.label("Flutes:");
                                let mut f = tool.flutes as f64;
                                let r = ui.add(egui::DragValue::new(&mut f).speed(1.0).range(1.0..=12.0));
                                if r.changed() { tool.flutes = f.max(1.0) as u32; }
                                r.changed()
                            }).inner;

                            // ── Feed-and-speed presets ────────────────────────
                            ui.separator();
                            ui.collapsing(format!("Feed/Speed Presets ({})", tool.presets.len()), |ui| {
                                ui.horizontal(|ui| {
                                    if ui.button("➕ Add Empty").clicked() {
                                        let d = tool.shape.diameter();
                                        tool.presets.push(FeedSpeedPreset {
                                            name: "New Preset".into(),
                                            material: String::new(),
                                            spindle_rpm: 10000.0,
                                            feed_rate: 0.05 * tool.flutes as f64 * 10000.0,
                                            plunge_rate: 200.0,
                                            coolant: Coolant::None,
                                            notes: String::new(),
                                        });
                                        if tool.default_preset.is_none() {
                                            tool.default_preset = Some(tool.presets.len() - 1);
                                        }
                                        ge_changed = true;
                                        let _ = d; // silence unused
                                    }
                                    if ui.button("✨ Load Starter Set").on_hover_text(
                                        "Replace presets with a starter set computed from this tool's diameter and flute count."
                                    ).clicked() {
                                        tool.presets = starter_presets_for(tool.shape.diameter(), tool.flutes);
                                        tool.default_preset = if tool.presets.is_empty() { None } else { Some(0) };
                                        ge_changed = true;
                                    }
                                    if !tool.presets.is_empty() && ui.button("🗑 Clear All").clicked() {
                                        tool.presets.clear();
                                        tool.default_preset = None;
                                        ge_changed = true;
                                    }
                                });

                                let mut to_remove: Option<usize> = None;
                                let mut new_default: Option<usize> = None;
                                for (p_idx, preset) in tool.presets.iter_mut().enumerate() {
                                    let is_default = tool.default_preset == Some(p_idx);
                                    ui.group(|ui| {
                                        ui.horizontal(|ui| {
                                            if ui.selectable_label(is_default, "★").on_hover_text("Set as default preset").clicked() {
                                                new_default = Some(p_idx);
                                                ge_changed = true;
                                            }
                                            ge_changed |= ui.text_edit_singleline(&mut preset.name).changed();
                                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                                if ui.small_button("🗑").clicked() {
                                                    to_remove = Some(p_idx);
                                                }
                                            });
                                        });
                                        ui.horizontal(|ui| {
                                            ui.label("Material:");
                                            ge_changed |= ui.text_edit_singleline(&mut preset.material).changed();
                                        });
                                        ui.horizontal(|ui| {
                                            ui.label("Spindle:");
                                            ge_changed |= ui.add(egui::DragValue::new(&mut preset.spindle_rpm).speed(10.0).suffix(" RPM")).changed();
                                            ui.label("Feed:");
                                            ge_changed |= ui.add(egui::DragValue::new(&mut preset.feed_rate).speed(10.0).suffix(" mm/min")).changed();
                                        });
                                        ui.horizontal(|ui| {
                                            ui.label("Plunge:");
                                            ge_changed |= ui.add(egui::DragValue::new(&mut preset.plunge_rate).speed(10.0).suffix(" mm/min")).changed();
                                            ui.label("Coolant:");
                                            egui::ComboBox::from_id_salt(format!("preset_coolant_{p_idx}"))
                                                .selected_text(preset.coolant.label())
                                                .show_ui(ui, |ui| {
                                                    for c in [Coolant::None, Coolant::Mist, Coolant::Flood, Coolant::MistFlood, Coolant::Through] {
                                                        if ui.selectable_value(&mut preset.coolant, c, c.label()).changed() {
                                                            ge_changed = true;
                                                        }
                                                    }
                                                });
                                        });
                                        ui.horizontal(|ui| {
                                            ui.label("Notes:");
                                            ge_changed |= ui.text_edit_singleline(&mut preset.notes).changed();
                                        });
                                    });
                                }
                                if let Some(i) = to_remove {
                                    tool.presets.remove(i);
                                    // Fix up default index after removal.
                                    tool.default_preset = match tool.default_preset {
                                        Some(d) if d == i => if tool.presets.is_empty() { None } else { Some(0) },
                                        Some(d) if d > i => Some(d - 1),
                                        other => other,
                                    };
                                    ge_changed = true;
                                }
                                if let Some(i) = new_default {
                                    tool.default_preset = Some(i);
                                }
                            });

                            // ── Tool-change G-code ─────────────────────────────
                            ui.separator();
                            ui.label("Tool-change G-code (runs after M6):");
                            let mut tc_text = tool.tool_change_gcode.clone().unwrap_or_default();
                            if ui.add(
                                egui::TextEdit::multiline(&mut tc_text)
                                    .desired_rows(2)
                                    .desired_width(f32::INFINITY)
                                    .hint_text("e.g. G43 H1   (apply tool-length offset)"),
                            ).changed() {
                                tool.tool_change_gcode = if tc_text.trim().is_empty() {
                                    None
                                } else {
                                    Some(tc_text)
                                };
                                ge_changed = true;
                            }

                            if ge_changed {
                                save_requested = true;
                            }

                            ui.add_space(20.0);
                            if ui.button("🗑 Delete Tool").clicked() {
                                // Defer the removal until after both columns
                                // have finished borrowing `self.job.tools[idx]`
                                // — otherwise the right-column profile preview
                                // panics with index-out-of-bounds.
                                delete_requested = true;
                            }
                        });

                        // Right Column: Profile Visualization
                        cols[1].group(|ui| {
                            ui.heading("Tool Profile Preview");
                            ui.separator();
                            let tool = &self.job.tools[idx];

                            let (rect, _response) = ui.allocate_at_least(
                                egui::vec2(ui.available_width(), 400.0),
                                egui::Sense::hover(),
                            );
                            let painter = ui.painter_at(rect);

                            let d  = tool.shape.diameter()    as f32;
                            let fl = tool.shape.flute_length() as f32;
                            let ol = tool.overall_length      as f32;
                            let shank_d = tool.shank_diameter as f32;

                            // ── Auto-scale: fit overall length and the widest
                            // radial dimension into the rect with margin so the
                            // preview always reflects parameter edits even when
                            // the tool is very small or very tall. ────────────
                            let max_radius = (d * 0.5)
                                .max(shank_d * 0.5)
                                .max(match &tool.shape {
                                    ToolShape::Lollipop { diameter, .. } => *diameter as f32 * 0.5,
                                    ToolShape::Dovetail { diameter, .. } => *diameter as f32 * 0.5,
                                    ToolShape::ChamferMill { diameter, .. } => *diameter as f32 * 0.5,
                                    ToolShape::TaperedMill { .. } => d * 0.5,
                                    _ => 0.0,
                                })
                                .max(0.5);
                            let total_h = ol.max(fl + 1.0);

                            let avail_w = rect.width() - 24.0;
                            let avail_h = rect.height() - 24.0;
                            let scale_x = avail_w / (max_radius * 2.0);
                            let scale_y = avail_h / total_h;
                            let scale   = scale_x.min(scale_y).max(0.1);

                            // Anchor the cutting tip near the bottom of the rect
                            // so the shank grows upward into the available area.
                            let center = egui::pos2(rect.center().x, rect.bottom() - 12.0);

                            let cyan = egui::Color32::from_rgb(0, 180, 255);
                            let dark = egui::Color32::from_gray(80);

                            // Shank: from the top of the cutting flutes up to overall length.
                            let shank_top = center + egui::vec2(0.0, -ol * scale);
                            let flute_top = center + egui::vec2(0.0, -fl * scale);
                            painter.rect_filled(
                                egui::Rect::from_min_max(
                                    shank_top + egui::vec2(-shank_d * 0.5 * scale, 0.0),
                                    flute_top + egui::vec2( shank_d * 0.5 * scale, 0.0),
                                ),
                                0.0,
                                dark,
                            );

                            // Cutter shape
                            match &tool.shape {
                                ToolShape::FlatEnd { diameter, .. } => {
                                    let r = *diameter as f32 * 0.5 * scale;
                                    painter.rect_filled(
                                        egui::Rect::from_min_max(
                                            center + egui::vec2(-r, -fl * scale),
                                            center + egui::vec2( r, 0.0),
                                        ),
                                        0.0,
                                        cyan,
                                    );
                                }
                                ToolShape::BullNose { diameter, corner_radius, .. } => {
                                    let r  = *diameter as f32 * 0.5 * scale;
                                    let cr = (*corner_radius as f32 * scale).min(r);
                                    // Shaft above the corner
                                    painter.rect_filled(
                                        egui::Rect::from_min_max(
                                            center + egui::vec2(-r, -fl * scale),
                                            center + egui::vec2( r, -cr),
                                        ),
                                        0.0,
                                        cyan,
                                    );
                                    // Flat between the two corner radii
                                    let inner = r - cr;
                                    painter.rect_filled(
                                        egui::Rect::from_min_max(
                                            center + egui::vec2(-inner, -cr),
                                            center + egui::vec2( inner, 0.0),
                                        ),
                                        0.0,
                                        cyan,
                                    );
                                    // Corner-radius circles
                                    painter.circle_filled(center + egui::vec2(-inner, -cr), cr, cyan);
                                    painter.circle_filled(center + egui::vec2( inner, -cr), cr, cyan);
                                }
                                ToolShape::BallEnd { diameter, .. } => {
                                    let r = *diameter as f32 * 0.5 * scale;
                                    painter.rect_filled(
                                        egui::Rect::from_min_max(
                                            center + egui::vec2(-r, -fl * scale),
                                            center + egui::vec2( r, -r),
                                        ),
                                        0.0,
                                        cyan,
                                    );
                                    painter.circle_filled(center + egui::vec2(0.0, -r), r, cyan);
                                }
                                ToolShape::Drill { diameter, point_angle, .. } => {
                                    let r = *diameter as f32 * 0.5 * scale;
                                    let half = (*point_angle as f32 * 0.5).to_radians();
                                    let tan_half = half.tan().max(0.05);
                                    let tip_h = (r / tan_half).min(fl * scale);
                                    painter.rect_filled(
                                        egui::Rect::from_min_max(
                                            center + egui::vec2(-r, -fl * scale),
                                            center + egui::vec2( r, -tip_h),
                                        ),
                                        0.0,
                                        cyan,
                                    );
                                    painter.add(egui::Shape::convex_polygon(
                                        vec![
                                            center + egui::vec2(-r, -tip_h),
                                            center + egui::vec2( r, -tip_h),
                                            center,
                                        ],
                                        cyan,
                                        egui::Stroke::NONE,
                                    ));
                                }
                                ToolShape::ChamferMill { tip_diameter, diameter, .. } => {
                                    let r_tip   = *tip_diameter as f32 * 0.5 * scale;
                                    let r_major = *diameter     as f32 * 0.5 * scale;
                                    painter.add(egui::Shape::convex_polygon(
                                        vec![
                                            center + egui::vec2(-r_major, -fl * scale),
                                            center + egui::vec2( r_major, -fl * scale),
                                            center + egui::vec2( r_tip,    0.0),
                                            center + egui::vec2(-r_tip,    0.0),
                                        ],
                                        cyan,
                                        egui::Stroke::NONE,
                                    ));
                                }
                                ToolShape::TaperedMill { tip_diameter, .. } => {
                                    let r_tip   = *tip_diameter as f32 * 0.5 * scale;
                                    let r_major = d * 0.5 * scale;
                                    painter.add(egui::Shape::convex_polygon(
                                        vec![
                                            center + egui::vec2(-r_major, -fl * scale),
                                            center + egui::vec2( r_major, -fl * scale),
                                            center + egui::vec2( r_tip,    0.0),
                                            center + egui::vec2(-r_tip,    0.0),
                                        ],
                                        cyan,
                                        egui::Stroke::NONE,
                                    ));
                                }
                                ToolShape::Lollipop { diameter, neck_diameter, .. } => {
                                    let r_head = *diameter as f32 * 0.5 * scale;
                                    let r_neck = *neck_diameter as f32 * 0.5 * scale;
                                    // Neck rises from the equator of the head to the top of the flutes.
                                    painter.rect_filled(
                                        egui::Rect::from_min_max(
                                            center + egui::vec2(-r_neck, -fl * scale),
                                            center + egui::vec2( r_neck, -r_head),
                                        ),
                                        0.0,
                                        egui::Color32::from_gray(100),
                                    );
                                    painter.circle_filled(center + egui::vec2(0.0, -r_head), r_head, cyan);
                                }
                                ToolShape::Dovetail { diameter, width, angle, .. } => {
                                    // Slot width = axial cutter height; angle = side undercut.
                                    let r_major = *diameter as f32 * 0.5 * scale;
                                    let h       = (*width   as f32 * scale).min(fl * scale);
                                    let undercut = h * (*angle as f32).to_radians().tan();
                                    let r_neck = (r_major - undercut).max(0.5);
                                    // Neck (above the flare)
                                    painter.rect_filled(
                                        egui::Rect::from_min_max(
                                            center + egui::vec2(-r_neck, -fl * scale),
                                            center + egui::vec2( r_neck, -h),
                                        ),
                                        0.0,
                                        egui::Color32::from_gray(100),
                                    );
                                    // Flared cutter — wider at the bottom (classic dovetail T-slot profile).
                                    painter.add(egui::Shape::convex_polygon(
                                        vec![
                                            center + egui::vec2(-r_neck, -h),
                                            center + egui::vec2( r_neck, -h),
                                            center + egui::vec2( r_major, 0.0),
                                            center + egui::vec2(-r_major, 0.0),
                                        ],
                                        cyan,
                                        egui::Stroke::NONE,
                                    ));
                                }
                                ToolShape::ThreadMill { diameter, pitch, num_teeth, .. } => {
                                    let r = *diameter as f32 * 0.5 * scale;
                                    painter.rect_filled(
                                        egui::Rect::from_min_max(
                                            center + egui::vec2(-r, -fl * scale),
                                            center + egui::vec2( r, 0.0),
                                        ),
                                        0.0,
                                        cyan,
                                    );
                                    // Draw `num_teeth` triangular thread crests at `pitch` spacing.
                                    let pitch_px = (*pitch as f32 * scale).max(2.0);
                                    let crest    = pitch_px * 0.5;
                                    for i in 0..(*num_teeth).max(1) {
                                        let y = -(i as f32 + 0.5) * pitch_px;
                                        if -y > fl * scale { break; }
                                        painter.add(egui::Shape::convex_polygon(
                                            vec![
                                                center + egui::vec2(-r, y),
                                                center + egui::vec2( r, y),
                                                center + egui::vec2( r + crest, y - crest),
                                                center + egui::vec2(-r - crest, y - crest),
                                            ],
                                            egui::Color32::from_gray(220),
                                            egui::Stroke::NONE,
                                        ));
                                    }
                                }
                            }

                            // Tip indicator (red baseline at Z=0)
                            let tip_half = max_radius * scale + 6.0;
                            painter.line_segment(
                                [
                                    center + egui::vec2(-tip_half, 0.0),
                                    center + egui::vec2( tip_half, 0.0),
                                ],
                                egui::Stroke::new(1.0, egui::Color32::RED),
                            );

                            // Scale label so the user knows the current zoom.
                            painter.text(
                                rect.left_top() + egui::vec2(8.0, 4.0),
                                egui::Align2::LEFT_TOP,
                                format!("⌀{:.2}mm  ⇣{:.1}mm  scale {:.1} px/mm", d, ol, scale),
                                egui::FontId::proportional(11.0),
                                egui::Color32::from_gray(160),
                            );
                        });
                    });

                    if delete_requested {
                        let removed = self.job.tools.remove(idx);
                        // Best-effort: unlink the on-disk preset so the tool
                        // doesn't resurrect itself on the next launch.
                        let path = PathBuf::from("tools").join(format!("tool_{}.json", removed.id));
                        match std::fs::remove_file(&path) {
                            Ok(()) => self.log.push(format!(
                                "Deleted tool T{} \"{}\" ({}).",
                                removed.id, removed.name, path.display()
                            )),
                            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                                self.log.push(format!("Deleted tool T{} \"{}\".", removed.id, removed.name));
                            }
                            Err(e) => self.log.push(format!(
                                "Deleted tool T{} \"{}\" but couldn't remove {}: {e}",
                                removed.id, removed.name, path.display()
                            )),
                        }
                        self.selected_tool = None;
                    } else if save_requested {
                        self.save_tool_to_dir(&self.job.tools[idx]);
                    }
                }
            } else {
                ui.centered_and_justified(|ui| {
                    ui.label("Select a tool from the list to edit.");
                });
            }
        });
    }
}

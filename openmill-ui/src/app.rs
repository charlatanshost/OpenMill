use std::path::PathBuf;

use eframe::egui;
use openmill_core::*;
use openmill_post::{GrblPost, LinuxCncPost, PostConfig, PostProcessor, Units};

// ── Strategy names ──────────────────────────────────────────────────────────

const STRATEGIES: &[&str] = &[
    "3+2 Indexed",
    "Adaptive Clearing",
    "Contour Parallel",
    "Surface Normal 5-Axis",
    "Swarf 5-Axis",
];

// ── App state ───────────────────────────────────────────────────────────────

pub struct OpenMillApp {
    job: Job,
    model: Option<WorkpieceModel>,

    // Per-operation generated toolpaths (parallel to job.operations).
    toolpaths: Vec<Vec<Toolpath>>,

    // Selection
    selected_tool: Option<usize>,
    selected_op: Option<usize>,

    // "Add tool" form
    add_tool_name: String,
    add_tool_diameter: f64,
    add_tool_flute_len: f64,
    add_tool_shape: usize, // 0 = Flat, 1 = Ball

    // Status log
    log: Vec<String>,
}

impl Default for OpenMillApp {
    fn default() -> Self {
        Self {
            job: Job::default(),
            model: None,
            toolpaths: Vec::new(),
            selected_tool: None,
            selected_op: None,
            add_tool_name: "New Tool".into(),
            add_tool_diameter: 6.0,
            add_tool_flute_len: 20.0,
            add_tool_shape: 0,
            log: vec!["OpenMill started.".into()],
        }
    }
}

// ── eframe::App ─────────────────────────────────────────────────────────────

impl eframe::App for OpenMillApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.show_menu_bar(ctx);
        self.show_bottom_panel(ctx);
        self.show_side_panel(ctx);
        self.show_central_panel(ctx);
    }
}

// ── Menu bar ────────────────────────────────────────────────────────────────

impl OpenMillApp {
    fn show_menu_bar(&mut self, ctx: &egui::Context) {
        egui::TopBottomPanel::top("menu_bar").show(ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                // ── File ─────────────────────────────────────────────
                ui.menu_button("File", |ui| {
                    if ui.button("New Job").clicked() {
                        self.job = Job::default();
                        self.model = None;
                        self.toolpaths.clear();
                        self.selected_tool = None;
                        self.selected_op = None;
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

                    if ui.button("Import Model (STL/3MF)...").clicked() {
                        self.action_import_model();
                        ui.close_menu();
                    }
                });

                // ── Export ────────────────────────────────────────────
                ui.menu_button("Export", |ui| {
                    if ui.button("LinuxCNC G-code...").clicked() {
                        self.action_export_gcode(&LinuxCncPost);
                        ui.close_menu();
                    }
                    if ui.button("GRBL G-code...").clicked() {
                        self.action_export_gcode(&GrblPost);
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
}

// ── Side panel ──────────────────────────────────────────────────────────────

impl OpenMillApp {
    fn show_side_panel(&mut self, ctx: &egui::Context) {
        egui::SidePanel::left("side_panel")
            .resizable(true)
            .default_width(300.0)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    self.section_job_info(ui);
                    ui.separator();
                    self.section_tools(ui);
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
        });
    }

    // ── Tools ────────────────────────────────────────────────────────────

    fn section_tools(&mut self, ui: &mut egui::Ui) {
        ui.collapsing("Tools", |ui| {
            // Tool list
            let mut remove_idx = None;
            for (i, tool) in self.job.tools.iter().enumerate() {
                let selected = self.selected_tool == Some(i);
                let label = format!("T{} — {} ({:.1}mm)", tool.id, tool.name, tool.shape.diameter());
                if ui.selectable_label(selected, &label).clicked() {
                    self.selected_tool = if selected { None } else { Some(i) };
                }
            }

            // Selected tool details
            if let Some(idx) = self.selected_tool {
                if idx < self.job.tools.len() {
                    ui.group(|ui| {
                        let tool = &mut self.job.tools[idx];
                        ui.horizontal(|ui| {
                            ui.label("Name:");
                            ui.text_edit_singleline(&mut tool.name);
                        });
                        ui.horizontal(|ui| {
                            ui.label("ID:");
                            ui.add(egui::DragValue::new(&mut tool.id));
                        });
                        let shape_name = match &tool.shape {
                            ToolShape::FlatEnd { .. } => "Flat End",
                            ToolShape::BallEnd { .. } => "Ball End",
                            ToolShape::BullNose { .. } => "Bull Nose",
                        };
                        ui.label(format!("Type: {shape_name}"));

                        ui.horizontal(|ui| {
                            ui.label("Diameter:");
                            let mut d = tool.shape.diameter();
                            if ui.add(egui::DragValue::new(&mut d).speed(0.1).suffix(" mm")).changed() {
                                set_shape_diameter(&mut tool.shape, d);
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("Flute length:");
                            let mut fl = tool.shape.flute_length();
                            if ui.add(egui::DragValue::new(&mut fl).speed(0.1).suffix(" mm")).changed() {
                                set_shape_flute_length(&mut tool.shape, fl);
                            }
                        });
                        ui.horizontal(|ui| {
                            ui.label("Overall length:");
                            ui.add(egui::DragValue::new(&mut tool.overall_length).speed(0.1).suffix(" mm"));
                        });
                        ui.horizontal(|ui| {
                            ui.label("Shank diameter:");
                            ui.add(egui::DragValue::new(&mut tool.shank_diameter).speed(0.1).suffix(" mm"));
                        });

                        if ui.button("Remove tool").clicked() {
                            remove_idx = Some(idx);
                        }
                    });
                }
            }

            if let Some(idx) = remove_idx {
                self.job.tools.remove(idx);
                self.selected_tool = None;
            }

            // Add tool form
            ui.separator();
            ui.label("Add tool:");
            ui.horizontal(|ui| {
                ui.label("Name:");
                ui.text_edit_singleline(&mut self.add_tool_name);
            });
            ui.horizontal(|ui| {
                ui.label("Type:");
                egui::ComboBox::from_id_salt("add_tool_shape")
                    .selected_text(match self.add_tool_shape {
                        0 => "Flat End",
                        _ => "Ball End",
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.add_tool_shape, 0, "Flat End");
                        ui.selectable_value(&mut self.add_tool_shape, 1, "Ball End");
                    });
            });
            ui.horizontal(|ui| {
                ui.label("Diameter:");
                ui.add(egui::DragValue::new(&mut self.add_tool_diameter).speed(0.1).suffix(" mm"));
            });
            ui.horizontal(|ui| {
                ui.label("Flute len:");
                ui.add(egui::DragValue::new(&mut self.add_tool_flute_len).speed(0.1).suffix(" mm"));
            });
            if ui.button("Add").clicked() {
                let next_id = self.job.tools.iter().map(|t| t.id).max().unwrap_or(0) + 1;
                let tool = if self.add_tool_shape == 0 {
                    Tool::flat_end(next_id, &self.add_tool_name, self.add_tool_diameter, self.add_tool_flute_len)
                } else {
                    Tool::ball_end(next_id, &self.add_tool_name, self.add_tool_diameter, self.add_tool_flute_len)
                };
                self.log.push(format!("Added tool T{} — {}", tool.id, tool.name));
                self.job.tools.push(tool);
            }
        });
    }

    // ── Operations ───────────────────────────────────────────────────────

    fn section_operations(&mut self, ui: &mut egui::Ui) {
        ui.collapsing("Operations", |ui| {
            let mut remove_idx = None;
            let mut generate_requested = false;

            for (i, op) in self.job.operations.iter().enumerate() {
                let selected = self.selected_op == Some(i);
                let status = if i < self.toolpaths.len() && !self.toolpaths[i].is_empty() {
                    " [generated]"
                } else {
                    ""
                };
                let enabled = if op.enabled { "" } else { " (disabled)" };
                let label = format!("{}{}{}", op.name, enabled, status);
                if ui.selectable_label(selected, &label).clicked() {
                    self.selected_op = if selected { None } else { Some(i) };
                }
            }

            // Selected operation details
            if let Some(idx) = self.selected_op {
                if idx < self.job.operations.len() {
                    ui.group(|ui| {
                        let op = &mut self.job.operations[idx];

                        ui.horizontal(|ui| {
                            ui.label("Name:");
                            ui.text_edit_singleline(&mut op.name);
                        });

                        ui.checkbox(&mut op.enabled, "Enabled");

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

                        ui.horizontal(|ui| {
                            ui.label("Spindle:");
                            ui.add(egui::DragValue::new(&mut op.spindle_speed).speed(10.0).suffix(" RPM"));
                        });

                        // Strategy-specific params
                        ui.separator();
                        ui.label("Parameters:");
                        show_strategy_params(ui, op);

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

            // Add operation
            if ui.button("Add Operation").clicked() {
                let next_tool = self.job.tools.first().map(|t| t.id).unwrap_or(1);
                let op = Operation {
                    name: format!("Op {}", self.job.operations.len() + 1),
                    tool_id: next_tool,
                    strategy: "3+2 Indexed".into(),
                    params: default_params_for("3+2 Indexed"),
                    spindle_speed: 10000.0,
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
            let m = &self.job.machine;
            ui.label(format!("Name: {}", m.name));
            match &m.axes {
                KinematicType::TableTable { a_axis, c_axis } => {
                    ui.label("Kinematics: Table-Table (A-C trunnion)");
                    ui.label(format!(
                        "A limits: {:.0}..{:.0} deg",
                        a_axis.min_angle.to_degrees(),
                        a_axis.max_angle.to_degrees(),
                    ));
                    ui.label(format!(
                        "C limits: {:.0}..{:.0} deg",
                        c_axis.min_angle.to_degrees(),
                        c_axis.max_angle.to_degrees(),
                    ));
                }
            }
            let tl = &m.travel_limits;
            ui.label(format!(
                "Travel: X {:.0}..{:.0}  Y {:.0}..{:.0}  Z {:.0}..{:.0} mm",
                tl.x.0, tl.x.1, tl.y.0, tl.y.1, tl.z.0, tl.z.1,
            ));
        });
    }
}

// ── Central panel ───────────────────────────────────────────────────────────

impl OpenMillApp {
    fn show_central_panel(&mut self, ctx: &egui::Context) {
        egui::CentralPanel::default().show(ctx, |ui| {
            if let Some(ref model) = self.model {
                let aabb = &model.aabb;
                let size = aabb.maxs - aabb.mins;
                ui.heading("Model loaded");
                ui.label(format!(
                    "Bounding box: {:.1} x {:.1} x {:.1} mm",
                    size.x, size.y, size.z
                ));
                let n_verts = model.mesh.vertices().len();
                let n_tris = model.mesh.indices().len();
                ui.label(format!("Vertices: {}  Triangles: {}", n_verts, n_tris));

                // Show toolpath stats
                let total_paths: usize = self.toolpaths.iter().map(|v| v.len()).sum();
                let total_points: usize = self.toolpaths.iter()
                    .flat_map(|v| v.iter())
                    .map(|tp| tp.points.len())
                    .sum();
                if total_paths > 0 {
                    ui.separator();
                    ui.label(format!("Toolpaths: {}  Points: {}", total_paths, total_points));
                }

                ui.separator();
                ui.colored_label(
                    egui::Color32::from_gray(120),
                    "3D viewport will be implemented with wgpu rendering.",
                );
            } else {
                ui.centered_and_justified(|ui| {
                    ui.label(
                        egui::RichText::new("Import a model via File > Import Model")
                            .size(20.0)
                            .color(egui::Color32::from_gray(140)),
                    );
                });
            }
        });
    }
}

// ── Bottom panel (status log) ───────────────────────────────────────────────

impl OpenMillApp {
    fn show_bottom_panel(&mut self, ctx: &egui::Context) {
        egui::TopBottomPanel::bottom("status_bar")
            .resizable(true)
            .default_height(100.0)
            .show(ctx, |ui| {
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
}

// ── File actions ────────────────────────────────────────────────────────────

impl OpenMillApp {
    fn action_open_job(&mut self) {
        let file = rfd::FileDialog::new()
            .add_filter("OpenMill Job", &["json"])
            .set_title("Open Job")
            .pick_file();

        if let Some(path) = file {
            match std::fs::read_to_string(&path) {
                Ok(json) => match serde_json::from_str::<Job>(&json) {
                    Ok(job) => {
                        // Try to load the model if path is set.
                        if let Some(ref mp) = job.model_path {
                            let model_path = path.parent()
                                .map(|p| p.join(mp))
                                .unwrap_or_else(|| PathBuf::from(mp));
                            self.try_load_model(&model_path);
                        }
                        self.toolpaths = vec![Vec::new(); job.operations.len()];
                        self.job = job;
                        self.selected_tool = None;
                        self.selected_op = None;
                        self.log.push(format!("Opened job from {}", path.display()));
                    }
                    Err(e) => self.log.push(format!("Failed to parse job: {e}")),
                },
                Err(e) => self.log.push(format!("Failed to read file: {e}")),
            }
        }
    }

    fn action_save_job(&mut self) {
        let file = rfd::FileDialog::new()
            .add_filter("OpenMill Job", &["json"])
            .set_title("Save Job")
            .set_file_name(&format!("{}.json", self.job.name))
            .save_file();

        if let Some(path) = file {
            match serde_json::to_string_pretty(&self.job) {
                Ok(json) => match std::fs::write(&path, &json) {
                    Ok(()) => self.log.push(format!("Saved job to {}", path.display())),
                    Err(e) => self.log.push(format!("Failed to write file: {e}")),
                },
                Err(e) => self.log.push(format!("Serialization error: {e}")),
            }
        }
    }

    fn action_import_model(&mut self) {
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
            Ok(model) => {
                let aabb = &model.aabb;
                let size = aabb.maxs - aabb.mins;
                self.log.push(format!(
                    "Imported model: {:.1} x {:.1} x {:.1} mm, {} vertices",
                    size.x, size.y, size.z,
                    model.mesh.vertices().len(),
                ));
                self.model = Some(model);
            }
            Err(e) => self.log.push(format!("Import failed: {e}")),
        }
    }

    fn action_export_gcode(&mut self, post: &dyn PostProcessor) {
        let all_paths: Vec<&Toolpath> = self.toolpaths.iter()
            .flat_map(|v| v.iter())
            .collect();

        if all_paths.is_empty() {
            self.log.push("No toolpaths generated. Generate toolpaths first.".into());
            return;
        }

        let file = rfd::FileDialog::new()
            .add_filter("G-code", &["nc", "ngc", "gcode"])
            .set_title("Export G-code")
            .set_file_name("output.nc")
            .save_file();

        let Some(path) = file else { return };

        let config = PostConfig {
            program_number: 1000,
            work_offset: "G54".into(),
            units: Units::Metric,
        };

        let mut output = post.header(&config);

        for tp in &all_paths {
            // Find matching tool for tool change.
            if let Some(tool) = self.job.tools.iter().find(|t| t.id == tp.tool_id) {
                output.push_str(&post.tool_change(tool));
            }
            match post.process_toolpath(tp, &self.job.machine) {
                Ok(gcode) => output.push_str(&gcode),
                Err(e) => {
                    self.log.push(format!("Post-process error: {e}"));
                    return;
                }
            }
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
    pub fn generate_selected_toolpath(&mut self) {
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

        let result = match op.strategy.as_str() {
            "3+2 Indexed" => {
                let params: ThreePlusTwoParams =
                    serde_json::from_value(op.params.clone()).unwrap_or_default();
                ThreePlusTwo.generate(model, tool, &self.job.machine, &params)
            }
            other => {
                self.log.push(format!("Strategy \"{other}\" is not yet implemented."));
                return;
            }
        };

        match result {
            Ok(paths) => {
                let total_pts: usize = paths.iter().map(|p| p.points.len()).sum();
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
            }
            Err(e) => self.log.push(format!("Generation failed: {e}")),
        }
    }
}

// ── Strategy param editors ──────────────────────────────────────────────────

fn default_params_for(strategy: &str) -> serde_json::Value {
    match strategy {
        "3+2 Indexed" => serde_json::to_value(ThreePlusTwoParams::default()).unwrap(),
        "Adaptive Clearing" => serde_json::to_value(AdaptiveClearingParams::default()).unwrap(),
        "Contour Parallel" => serde_json::to_value(ContourParallelParams::default()).unwrap(),
        "Surface Normal 5-Axis" => serde_json::to_value(SurfaceNormal5AxisParams::default()).unwrap(),
        "Swarf 5-Axis" => serde_json::to_value(Swarf5AxisParams::default()).unwrap(),
        _ => serde_json::Value::Object(Default::default()),
    }
}

fn show_strategy_params(ui: &mut egui::Ui, op: &mut Operation) {
    // Deserialize, show editors, serialize back on change.
    match op.strategy.as_str() {
        "3+2 Indexed" => {
            let mut p: ThreePlusTwoParams =
                serde_json::from_value(op.params.clone()).unwrap_or_default();
            let mut changed = false;
            changed |= drag(ui, "A angle", &mut p.a_deg, 0.5, "deg");
            changed |= drag(ui, "C angle", &mut p.c_deg, 0.5, "deg");
            changed |= drag(ui, "Step-down", &mut p.step_down, 0.1, "mm");
            changed |= drag(ui, "Step-over", &mut p.step_over, 0.01, "");
            changed |= drag(ui, "Feed rate", &mut p.feed_rate, 10.0, "mm/min");
            if changed {
                op.params = serde_json::to_value(&p).unwrap();
            }
        }
        "Adaptive Clearing" => {
            let mut p: AdaptiveClearingParams =
                serde_json::from_value(op.params.clone()).unwrap_or_default();
            let mut changed = false;
            changed |= drag(ui, "Max engagement", &mut p.max_engagement_deg, 1.0, "deg");
            changed |= drag(ui, "Step-down", &mut p.step_down, 0.1, "mm");
            changed |= drag(ui, "Feed rate", &mut p.feed_rate, 10.0, "mm/min");
            if changed {
                op.params = serde_json::to_value(&p).unwrap();
            }
        }
        "Contour Parallel" => {
            let mut p: ContourParallelParams =
                serde_json::from_value(op.params.clone()).unwrap_or_default();
            let mut changed = false;
            changed |= drag(ui, "Step-down", &mut p.step_down, 0.1, "mm");
            changed |= drag(ui, "Feed rate", &mut p.feed_rate, 10.0, "mm/min");
            changed |= drag(ui, "Tolerance", &mut p.tolerance, 0.001, "mm");
            if changed {
                op.params = serde_json::to_value(&p).unwrap();
            }
        }
        "Surface Normal 5-Axis" => {
            let mut p: SurfaceNormal5AxisParams =
                serde_json::from_value(op.params.clone()).unwrap_or_default();
            let mut changed = false;
            changed |= drag(ui, "Lead angle", &mut p.lead_angle, 0.5, "deg");
            changed |= drag(ui, "Tilt angle", &mut p.tilt_angle, 0.5, "deg");
            changed |= drag(ui, "Step-over", &mut p.step_over, 0.01, "");
            changed |= drag(ui, "Feed rate", &mut p.feed_rate, 10.0, "mm/min");
            changed |= drag(ui, "Tolerance", &mut p.tolerance, 0.001, "mm");
            if changed {
                op.params = serde_json::to_value(&p).unwrap();
            }
        }
        "Swarf 5-Axis" => {
            let mut p: Swarf5AxisParams =
                serde_json::from_value(op.params.clone()).unwrap_or_default();
            let mut changed = false;
            let mut np = p.num_passes as f64;
            if drag(ui, "Passes", &mut np, 1.0, "") {
                p.num_passes = np.max(1.0) as usize;
                changed = true;
            }
            changed |= drag(ui, "Step-down", &mut p.step_down, 0.1, "mm");
            changed |= drag(ui, "Feed rate", &mut p.feed_rate, 10.0, "mm/min");
            ui.checkbox(&mut p.overhang_compensation, "Overhang compensation");
            if changed {
                op.params = serde_json::to_value(&p).unwrap();
            }
        }
        _ => {
            ui.weak("(unknown strategy)");
        }
    }
}

/// Helper: labeled drag-value row. Returns true if changed.
fn drag(ui: &mut egui::Ui, label: &str, value: &mut f64, speed: f64, suffix: &str) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label(format!("{label}:"));
        let suffix_str = if suffix.is_empty() {
            String::new()
        } else {
            format!(" {suffix}")
        };
        changed = ui
            .add(egui::DragValue::new(value).speed(speed).suffix(suffix_str))
            .changed();
    });
    changed
}

// ── Tool shape mutators ─────────────────────────────────────────────────────

fn set_shape_diameter(shape: &mut ToolShape, d: f64) {
    match shape {
        ToolShape::FlatEnd { diameter, .. }
        | ToolShape::BallEnd { diameter, .. }
        | ToolShape::BullNose { diameter, .. } => *diameter = d,
    }
}

fn set_shape_flute_length(shape: &mut ToolShape, fl: f64) {
    match shape {
        ToolShape::FlatEnd { flute_length, .. }
        | ToolShape::BallEnd { flute_length, .. }
        | ToolShape::BullNose { flute_length, .. } => *flute_length = fl,
    }
}

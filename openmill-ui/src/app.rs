use std::path::PathBuf;
use parry3d::query::RayCast;

use eframe::egui;
use eframe::egui_wgpu;
use openmill_core::*;
use openmill_post::{PostProcessor, POST_PROCESSOR_NAMES, get_post};

use crate::viewport::{OrbitCamera, Viewport};

// ── Strategy names ──────────────────────────────────────────────────────────

const STRATEGIES: &[&str] = &[
    "3+2 Indexed",
    "Adaptive Clearing",
    "Contour Parallel",
    "Surface Normal 5-Axis",
    "Swarf 5-Axis",
    "Geodesic Parallel",
    "5-Axis Pencil Tracing",
    "5-Axis Drilling",
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

    // 3D viewport
    render_state: Option<egui_wgpu::RenderState>,
    viewport: Option<Viewport>,
    camera: OrbitCamera,

    // Status log
    log: Vec<String>,

    // Simulation
    sim_playing: bool,
    sim_progress: f32,
    sim_prev_progress: f32,
    sim_speed: f32,
    sim_gcode: Vec<(String, Option<usize>)>,
    sim_show_full_path: bool,
    sim_op_idx: Option<usize>,
    sim_collisions: Vec<nalgebra::Point3<f32>>,

    // Tab system
    active_tab: AppTab,

    // Libraries
    machine_library: MachineLibrary,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AppTab {
    Manufacturing,
    Machine,
    Tools,
}


impl OpenMillApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let render_state = cc.wgpu_render_state.clone();
        let viewport = render_state.as_ref().map(Viewport::new);

        let mut app = Self {
            job: Job::default(),
            model: None,
            toolpaths: Vec::new(),
            selected_tool: None,
            selected_op: None,
            render_state,
            viewport,
            camera: OrbitCamera::default(),
            log: vec!["OpenMill started.".into()],
            sim_playing: false,
            sim_progress: 0.0,
            sim_prev_progress: 0.0,
            sim_speed: 1.0,
            sim_gcode: Vec::new(),
            sim_show_full_path: false,
            sim_op_idx: None,
            sim_collisions: Vec::new(),
            active_tab: AppTab::Manufacturing,
            machine_library: MachineLibrary::load("machines").unwrap_or_default(),
        };
        app.load_tools_from_dir();
        app
    }

    fn load_tools_from_dir(&mut self) {
        let dir = PathBuf::from("tools");
        if !dir.exists() {
            let _ = std::fs::create_dir(&dir);
            return;
        }
        if let Ok(entries) = std::fs::read_dir(dir) {
            for entry in entries.flatten() {
                if entry.path().extension().map_or(false, |ext| ext == "json") {
                    if let Ok(content) = std::fs::read_to_string(entry.path()) {
                        if let Ok(tool) = serde_json::from_str::<Tool>(&content) {
                            if !self.job.tools.iter().any(|t| t.id == tool.id) {
                                self.job.tools.push(tool);
                            }
                        }
                    }
                }
            }
        }
    }

    fn save_tool_to_dir(&self, tool: &Tool) {
        let dir = PathBuf::from("tools");
        if !dir.exists() { let _ = std::fs::create_dir(&dir); }
        let filename = format!("tool_{}.json", tool.id);
        let path = dir.join(filename);
        if let Ok(content) = serde_json::to_string_pretty(tool) {
            let _ = std::fs::write(path, content);
        }
    }
}

// ── eframe::App ─────────────────────────────────────────────────────────────

impl eframe::App for OpenMillApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        if self.sim_playing {
            let dt = ctx.input(|i| i.stable_dt).min(0.1);
            // Advance progress. Let's say a full toolpath takes 10 seconds at 1.0 speed
            self.sim_progress += (dt * self.sim_speed) / 10.0;
            if self.sim_progress >= 1.0 {
                self.sim_progress = 1.0;
                self.sim_playing = false;
            }
            ctx.request_repaint();
        }

        if self.sim_progress > self.sim_prev_progress {
            let p1 = self.get_tool_pose_at(self.sim_prev_progress);
            let p2 = self.get_tool_pose_at(self.sim_progress);
            if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
                if let (Some((pose1, tool)), Some((pose2, _))) = (p1, p2) {
                    vp.carve_voxel(&rs.device, &rs.queue, &pose1, &pose2, &tool);

                    // Real-time collision detection against the mesh
                    if let Some(ref model) = self.model {
                        let p1_v = pose1.translation.vector;
                        let p2_v = pose2.translation.vector;
                        let ray_dir = if (p2_v - p1_v).norm() > 0.0001 { (p2_v - p1_v).normalize() } else { nalgebra::Vector3::z() };
                        let dist = (p2_v - p1_v).norm();
                        
                        let ray = parry3d::query::Ray::new(
                            p1_v.into(),
                            ray_dir,
                        );
                        
                        // Check if the tool tip path intersects the mesh (simple tip collision for now)
                        if let Some(toi) = model.mesh.cast_local_ray(&ray, dist, true) {
                            let hit_pt = ray.point_at(toi);
                            self.sim_collisions.push(hit_pt);
                            vp.upload_collisions(&rs.device, &self.sim_collisions);
                        }
                    }
                }
            }
        }
        self.sim_prev_progress = self.sim_progress;

        // If we switched operations or need to catch up the stock state
        if let Some(target_op_idx) = self.selected_op {
            if self.sim_op_idx != Some(target_op_idx) {
                if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
                    // Reset to initial stock
                    vp.reset_voxel(&rs.queue);
                    
                    // Fast-forward all PREVIOUS operations
                    for prev_idx in 0..target_op_idx {
                        if prev_idx < self.toolpaths.len() {
                            for tp in &self.toolpaths[prev_idx] {
                                if let Some(tool) = self.job.tools.iter().find(|t| t.id == tp.tool_id) {
                                    for i in 0..tp.points.len().saturating_sub(1) {
                                        let pt1 = &tp.points[i];
                                        let pt2 = &tp.points[i+1];
                                        
                                        // Skip rapid moves for stock removal (unless user wants to see gouges)
                                        if pt2.move_type == MoveType::Rapid { continue; }

                                        let pos1 = pt1.position.cast::<f32>();
                                        let pos2 = pt2.position.cast::<f32>();
                                        let orient1 = pt1.orientation.into_inner().cast::<f32>();
                                        let orient2 = pt2.orientation.into_inner().cast::<f32>();

                                        let rot1 = nalgebra::UnitQuaternion::rotation_between(&nalgebra::Vector3::z(), &orient1)
                                            .unwrap_or(nalgebra::UnitQuaternion::identity());
                                        let rot2 = nalgebra::UnitQuaternion::rotation_between(&nalgebra::Vector3::z(), &orient2)
                                            .unwrap_or(nalgebra::UnitQuaternion::identity());

                                        let pose1 = nalgebra::Isometry3::from_parts(pos1.into(), rot1);
                                        let pose2 = nalgebra::Isometry3::from_parts(pos2.into(), rot2);

                                        vp.carve_voxel(&rs.device, &rs.queue, &pose1, &pose2, tool);
                                    }
                                }
                            }
                        }
                    }
                    self.sim_op_idx = Some(target_op_idx);
                    self.sim_progress = 0.0;
                    self.sim_prev_progress = 0.0;
                    self.sim_collisions.clear();
                    vp.upload_collisions(&rs.device, &[]);
                }
            }
        }

        let sim_tool_data = if self.sim_progress > 0.0 || self.sim_playing {
            self.get_sim_tool_pose()
        } else {
            None
        };

        // Upload the tool continuously if simulation is active or we are paused at a progress
        if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
            if let Some((pose, tool)) = &sim_tool_data {
                vp.upload_tool(&rs.device, Some(pose), Some(tool));
            } else {
                vp.upload_tool(&rs.device, None, None);
            }
        }

        self.show_menu_bar(ctx);
        self.show_tab_bar(ctx);

        match self.active_tab {
            AppTab::Manufacturing => {
                self.show_bottom_panel(ctx);
                self.show_side_panel(ctx);
                self.show_right_panel(ctx);
                self.show_central_panel(ctx);
            }
            AppTab::Machine => {
                self.show_machine_tab(ctx);
            }
            AppTab::Tools => {
                self.show_tools_tab(ctx);
            }
        }
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

    fn show_tab_bar(&mut self, ctx: &egui::Context) {
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

            ui.separator();
            ui.label("Stock:");
            let mut stock_changed = false;
            let mut current_type = match self.job.stock {
                openmill_core::job::StockDef::BoundingBox { .. } => 0,
                openmill_core::job::StockDef::Cylinder { .. } => 1,
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
                    ui.horizontal(|ui| {
                        ui.label("Margin X:");
                        stock_changed |= ui.add(egui::DragValue::new(&mut margin[0]).speed(0.1)).changed();
                        ui.label("Y:");
                        stock_changed |= ui.add(egui::DragValue::new(&mut margin[1]).speed(0.1)).changed();
                        ui.label("Z:");
                        stock_changed |= ui.add(egui::DragValue::new(&mut margin[2]).speed(0.1)).changed();
                    });
                }
                openmill_core::job::StockDef::Cylinder { diameter, height } => {
                    ui.horizontal(|ui| {
                        ui.label("Diameter:");
                        stock_changed |= ui.add(egui::DragValue::new(diameter).speed(0.1).suffix(" mm")).changed();
                        ui.label("Height:");
                        stock_changed |= ui.add(egui::DragValue::new(height).speed(0.1).suffix(" mm")).changed();
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

    // ── Operations ───────────────────────────────────────────────────────

    fn section_operations(&mut self, ui: &mut egui::Ui) {
        ui.collapsing("Operations", |ui| {
            let mut remove_idx = None;
            let mut generate_requested = false;

            let mut selection_changed = false;
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
                    selection_changed = true;
                }
            }

            if selection_changed {
                self.update_sim_gcode();
                self.update_sim_viewport();
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

// ── Central panel ───────────────────────────────────────────────────────────

impl OpenMillApp {
    fn show_central_panel(&mut self, ctx: &egui::Context) {
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

            // Render and display.
            if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
                let w = (size.x as u32).max(1);
                let h = (size.y as u32).max(1);
                vp.render(rs, &self.camera, w, h, self.sim_progress, self.sim_show_full_path);

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
                if let Some(op_idx) = self.selected_op {
                    if op_idx < self.toolpaths.len() && !self.toolpaths[op_idx].is_empty() {
                        ui.horizontal(|ui| {
                            ui.heading("Simulation");
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
                                self.sim_collisions.clear();
                            }
                            ui.add(egui::Slider::new(&mut self.sim_progress, 0.0..=1.0).text("Progress"));
                            ui.add(egui::Slider::new(&mut self.sim_speed, 0.1..=5.0).text("Speed"));
                            ui.checkbox(&mut self.sim_show_full_path, "Show Full Path");
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

    fn show_right_panel(&mut self, ctx: &egui::Context) {
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

    fn get_sim_tool_pose(&self) -> Option<(nalgebra::Isometry3<f32>, openmill_core::Tool)> {
        self.get_tool_pose_at(self.sim_progress)
    }

    fn get_tool_pose_at(&self, progress: f32) -> Option<(nalgebra::Isometry3<f32>, openmill_core::Tool)> {
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

        let config = self.job.machine.post_config.clone();

        let mut output = post.header(&config);

        for tp in &all_paths {
            // Find matching tool for tool change.
            if let Some(tool) = self.job.tools.iter().find(|t| t.id == tp.tool_id) {
                output.push_str(&post.tool_change(tool));
            }
            match post.process_toolpath(tp, &self.job.machine) {
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
            "Surface Normal 5-Axis" => {
                let params: SurfaceNormal5AxisParams =
                    serde_json::from_value(op.params.clone()).unwrap_or_default();
                SurfaceNormal5Axis.generate(model, tool, &self.job.machine, &params)
            }
            "Contour Parallel" => {
                let params: ContourParallelParams =
                    serde_json::from_value(op.params.clone()).unwrap_or_default();
                ContourParallel.generate(model, tool, &self.job.machine, &params)
            }
            "Geodesic Parallel" => {
                let params: GeodesicParams =
                    serde_json::from_value(op.params.clone()).unwrap_or_default();
                GeodesicParallel.generate(model, tool, &self.job.machine, &params)
            }
            "5-Axis Pencil Tracing" => {
                let params: PencilParams =
                    serde_json::from_value(op.params.clone()).unwrap_or_default();
                PencilTracing.generate(model, tool, &self.job.machine, &params)
            }
            "5-Axis Drilling" => {
                let params: DrillingParams =
                    serde_json::from_value(op.params.clone()).unwrap_or_default();
                Drilling5Axis.generate(model, tool, &self.job.machine, &params)
            }
            other => {
                self.log.push(format!("Strategy \"{other}\" is not yet implemented."));
                return;
            }
        };

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

    fn update_sim_gcode(&mut self) {
        self.sim_gcode.clear();
        let Some(idx) = self.selected_op else { return };
        if idx >= self.toolpaths.len() { return; }

        let post = get_post(&self.job.machine.post_processor);
        for tp in &self.toolpaths[idx] {
            if let Ok(lines) = post.process_toolpath(tp, &self.job.machine) {
                self.sim_gcode.extend(lines);
            }
        }
    }

    fn update_sim_viewport(&mut self) {
        let Some(idx) = self.selected_op else { return };
        if idx >= self.toolpaths.len() { return; }

        if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
            vp.upload_toolpath(&rs.device, &self.toolpaths[idx]);
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
        "Geodesic Parallel" => serde_json::to_value(GeodesicParams::default()).unwrap(),
        "5-Axis Pencil Tracing" => serde_json::to_value(PencilParams::default()).unwrap(),
        "5-Axis Drilling" => serde_json::to_value(DrillingParams::default()).unwrap(),
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

            ui.horizontal(|ui| {
                ui.label("Pattern:");
                egui::ComboBox::from_id_salt("5axis_pattern")
                    .selected_text(match p.pattern {
                        DrivePattern::Parallel => "Parallel",
                        DrivePattern::Rotary => "Rotary",
                        DrivePattern::Scallop => "Scallop",
                        DrivePattern::Spiral => "Spiral",
                    })
                    .show_ui(ui, |ui| {
                        changed |= ui.selectable_value(&mut p.pattern, DrivePattern::Parallel, "Parallel").changed();
                        changed |= ui.selectable_value(&mut p.pattern, DrivePattern::Rotary, "Rotary").changed();
                        changed |= ui.selectable_value(&mut p.pattern, DrivePattern::Scallop, "Scallop").changed();
                        changed |= ui.selectable_value(&mut p.pattern, DrivePattern::Spiral, "Spiral").changed();
                    });
            });

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
            changed |= drag(ui, "Z Top", &mut p.z_top, 0.1, "mm");
            changed |= drag(ui, "Z Bottom", &mut p.z_bottom, 0.1, "mm");
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
        "Geodesic Parallel" => {
            let mut p: GeodesicParams =
                serde_json::from_value(op.params.clone()).unwrap_or_default();
            let mut changed = false;
            changed |= drag(ui, "Step-over", &mut p.step_over, 0.1, "mm");
            changed |= drag(ui, "Feed rate", &mut p.feed_rate, 10.0, "mm/min");
            changed |= drag(ui, "Tolerance", &mut p.tolerance, 0.001, "mm");
            if changed {
                op.params = serde_json::to_value(&p).unwrap();
            }
        }
        "5-Axis Pencil Tracing" => {
            let mut p: PencilParams =
                serde_json::from_value(op.params.clone()).unwrap_or_default();
            let mut changed = false;
            changed |= drag(ui, "Concavity Threshold", &mut p.threshold_deg, 1.0, "deg");
            changed |= drag(ui, "Feed rate", &mut p.feed_rate, 10.0, "mm/min");
            if changed {
                op.params = serde_json::to_value(&p).unwrap();
            }
        }
        "5-Axis Drilling" => {
            let mut p: DrillingParams =
                serde_json::from_value(op.params.clone()).unwrap_or_default();
            let mut changed = false;
            ui.label(format!("Holes: {}", p.holes.len()));
            if ui.button("Clear Holes").clicked() {
                p.holes.clear();
                changed = true;
            }
            if p.holes.is_empty() && ui.button("Add Test Hole").clicked() {
                p.holes.push(openmill_core::strategies::drilling::Hole {
                    position: nalgebra::Point3::new(0.0, 0.0, 10.0),
                    axis: nalgebra::Vector3::z(),
                    depth: 10.0,
                });
                changed = true;
            }

            changed |= drag(ui, "Feed rate", &mut p.feed_rate, 10.0, "mm/min");
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
    fn show_machine_tab(&mut self, ctx: &egui::Context) {
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

    fn show_tools_tab(&mut self, ctx: &egui::Context) {
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

                            if ge_changed {
                                save_requested = true;
                            }

                            ui.add_space(20.0);
                            if ui.button("🗑 Delete Tool").clicked() {
                                self.job.tools.remove(idx);
                                self.selected_tool = None;
                            }
                        });

                        // Right Column: Profile Visualization
                        cols[1].group(|ui| {
                            ui.heading("Tool Profile Preview");
                            ui.separator();
                            let tool = &self.job.tools[idx];
                            
                            let (rect, _response) = ui.allocate_at_least(egui::vec2(ui.available_width(), 400.0), egui::Sense::hover());
                            let painter = ui.painter_at(rect);
                            
                            // Simple 2D profile draw
                            let center = rect.center();
                            let scale = 5.0; // pixels per mm
                            
                            let d = tool.shape.diameter() as f32;
                            let fl = tool.shape.flute_length() as f32;
                            let ol = tool.overall_length as f32;
                            
                            // Draw shank
                            painter.rect_filled(
                                egui::Rect::from_min_max(
                                    center + egui::vec2(-tool.shank_diameter as f32 * 0.5 * scale, -ol * scale),
                                    center + egui::vec2(tool.shank_diameter as f32 * 0.5 * scale, -fl * scale)
                                ),
                                0.0,
                                egui::Color32::from_gray(80)
                            );
                            
                            // Draw cutter based on shape
                            match &tool.shape {
                                ToolShape::FlatEnd { diameter, .. } | ToolShape::BullNose { diameter, .. } => {
                                    painter.rect_filled(
                                        egui::Rect::from_min_max(
                                            center + egui::vec2(-*diameter as f32 * 0.5 * scale, -fl * scale),
                                            center + egui::vec2(*diameter as f32 * 0.5 * scale, 0.0)
                                        ),
                                        0.0,
                                        egui::Color32::from_rgb(0, 180, 255)
                                    );
                                }
                                ToolShape::BallEnd { diameter, .. } => {
                                    let r = *diameter as f32 * 0.5 * scale;
                                    painter.rect_filled(egui::Rect::from_min_max(center + egui::vec2(-r, -fl*scale), center + egui::vec2(r, -r)), 0.0, egui::Color32::from_rgb(0, 180, 255));
                                    painter.circle_filled(center + egui::vec2(0.0, -r), r, egui::Color32::from_rgb(0, 180, 255));
                                }
                                ToolShape::Drill { diameter, point_angle, .. } => {
                                    let r = *diameter as f32 * 0.5 * scale;
                                    let tip_h = r / ((*point_angle as f32 * 0.5).to_radians().tan());
                                    painter.rect_filled(egui::Rect::from_min_max(center + egui::vec2(-r, -fl*scale), center + egui::vec2(r, -tip_h)), 0.0, egui::Color32::from_rgb(0, 180, 255));
                                    painter.add(egui::Shape::convex_polygon(
                                        vec![center + egui::vec2(-r, -tip_h), center + egui::vec2(r, -tip_h), center],
                                        egui::Color32::from_rgb(0, 180, 255),
                                        egui::Stroke::NONE
                                    ));
                                }
                                ToolShape::ChamferMill { tip_diameter, diameter, .. } => {
                                    let r_tip = *tip_diameter as f32 * 0.5 * scale;
                                    let r_major = *diameter as f32 * 0.5 * scale;
                                    painter.add(egui::Shape::convex_polygon(
                                        vec![center + egui::vec2(-r_major, -fl*scale), center + egui::vec2(r_major, -fl*scale), center + egui::vec2(r_tip, 0.0), center + egui::vec2(-r_tip, 0.0)],
                                        egui::Color32::from_rgb(0, 180, 255),
                                        egui::Stroke::NONE
                                    ));
                                }
                                ToolShape::TaperedMill { tip_diameter, .. } => {
                                    let r_tip = *tip_diameter as f32 * 0.5 * scale;
                                    let r_major = d * 0.5 * scale;
                                    painter.add(egui::Shape::convex_polygon(
                                        vec![center + egui::vec2(-r_major, -fl*scale), center + egui::vec2(r_major, -fl*scale), center + egui::vec2(r_tip, 0.0), center + egui::vec2(-r_tip, 0.0)],
                                        egui::Color32::from_rgb(0, 180, 255),
                                        egui::Stroke::NONE
                                    ));
                                }
                                ToolShape::Lollipop { diameter, neck_diameter, .. } => {
                                    let r_head = *diameter as f32 * 0.5 * scale;
                                    let r_neck = *neck_diameter as f32 * 0.5 * scale;
                                    painter.rect_filled(egui::Rect::from_min_max(center + egui::vec2(-r_neck, -fl*scale), center + egui::vec2(r_neck, -r_head)), 0.0, egui::Color32::from_gray(100));
                                    painter.circle_filled(center + egui::vec2(0.0, -r_head), r_head, egui::Color32::from_rgb(0, 180, 255));
                                }
                                ToolShape::ThreadMill { diameter, .. } => {
                                    let r = *diameter as f32 * 0.5 * scale;
                                    painter.rect_filled(egui::Rect::from_min_max(center + egui::vec2(-r, -fl*scale), center + egui::vec2(r, 0.0)), 0.0, egui::Color32::from_rgb(0, 180, 255));
                                    for i in 0..5 { // Simulated threads
                                        let y = -i as f32 * 10.0;
                                        painter.line_segment([center + egui::vec2(-r-2.0, y), center + egui::vec2(r+2.0, y-5.0)], egui::Stroke::new(1.0, egui::Color32::BLACK));
                                    }
                                }
                                _ => {
                                    painter.rect_filled(egui::Rect::from_min_max(center + egui::vec2(-d*0.5*scale, -fl*scale), center + egui::vec2(d*0.5*scale, 0.0)), 0.0, egui::Color32::from_rgb(0, 180, 255));
                                }
                            }
                            
                            // Draw tip indicator
                            painter.line_segment(
                                [center + egui::vec2(-d*scale, 0.0), center + egui::vec2(d*scale, 0.0)],
                                egui::Stroke::new(1.0, egui::Color32::RED)
                            );
                        });
                    });

                    if save_requested {
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

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
    "4+1 Indexed",
    "Adaptive Clearing",
    "Pocket Clearing",
    "Contour Parallel",
    "Surface Normal 5-Axis",
    "Swarf 5-Axis",
    "Geodesic Parallel",
    "5-Axis Pencil Tracing",
    "5-Axis Drilling",
    "Tapping",
    "Thread Milling",
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

    /// Current 3D viewport mode. `Plan` shows the original part mesh for path
    /// setup; `Simulation` hides the mesh and shows the carved voxel stock so
    /// the user sees what material is left after each operation.
    view_mode: ViewMode,

    /// Unit displayed in the stock-size editor (mm or decimal inch). Doesn't
    /// affect the underlying job — the data model is always millimetres.
    stock_units: StockUnits,

    /// Recognised geometric features on the current model (holes, etc.).
    /// Populated by `Detect Holes` in the Features section, used by ops like
    /// 5-Axis Drilling. Cleared when a new model is imported.
    features: Vec<openmill_core::Feature>,

    /// When `true`, a left-click in the viewport raycasts the mesh and
    /// flood-fills coplanar neighbours into a `Feature` (auto-classified by
    /// normal direction). Drag still orbits the camera so the user can keep
    /// the same UX.
    pick_face_mode: bool,

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ViewMode {
    /// Show the original part mesh — used for setting up jobs and reviewing
    /// generated toolpaths.
    Plan,
    /// Hide the part mesh, show the voxel stock as it gets carved — used to
    /// verify what material is left after each operation.
    Simulation,
}

/// Where to send a feature when the user clicks "→ <Strategy>" in the
/// Features panel. Each variant maps to a strategy name and knows how to
/// append to (or replace) the op's feature-driven list.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FeatureAssignTarget {
    Drilling,
    Tapping,
    ThreadMill,
    PocketClearing,
}

impl FeatureAssignTarget {
    fn expected_strategy(self) -> &'static str {
        match self {
            FeatureAssignTarget::Drilling       => "5-Axis Drilling",
            FeatureAssignTarget::Tapping        => "Tapping",
            FeatureAssignTarget::ThreadMill     => "Thread Milling",
            FeatureAssignTarget::PocketClearing => "Pocket Clearing",
        }
    }
    /// Append the feature to the op's list. Returns the new list length, or
    /// `None` if the feature kind is incompatible.
    fn append_one(self, op: &mut openmill_core::Operation, f: &openmill_core::Feature) -> Option<usize> {
        match self {
            FeatureAssignTarget::Drilling => {
                let hole = openmill_core::feature_to_hole(f)?;
                let mut p: openmill_core::DrillingParams =
                    serde_json::from_value(op.params.clone()).unwrap_or_default();
                p.holes.push(hole);
                let n = p.holes.len();
                op.params = serde_json::to_value(&p).unwrap();
                Some(n)
            }
            FeatureAssignTarget::Tapping => {
                let hole = openmill_core::feature_to_hole(f)?;
                let mut p: openmill_core::TappingParams =
                    serde_json::from_value(op.params.clone()).unwrap_or_default();
                p.holes.push(hole);
                let n = p.holes.len();
                op.params = serde_json::to_value(&p).unwrap();
                Some(n)
            }
            FeatureAssignTarget::ThreadMill => {
                let hole = openmill_core::feature_to_hole(f)?;
                let mut p: openmill_core::ThreadMillingParams =
                    serde_json::from_value(op.params.clone()).unwrap_or_default();
                p.holes.push(hole);
                let n = p.holes.len();
                op.params = serde_json::to_value(&p).unwrap();
                Some(n)
            }
            FeatureAssignTarget::PocketClearing => {
                let pocket = openmill_core::PocketRef::from_feature(f)?;
                let mut p: openmill_core::PocketClearingParams =
                    serde_json::from_value(op.params.clone()).unwrap_or_default();
                p.pockets.push(pocket);
                let n = p.pockets.len();
                op.params = serde_json::to_value(&p).unwrap();
                Some(n)
            }
        }
    }
    /// Replace the op's list with every compatible feature. Returns the new length.
    fn replace_all(self, op: &mut openmill_core::Operation, features: &[openmill_core::Feature]) -> usize {
        match self {
            FeatureAssignTarget::Drilling => {
                let mut p: openmill_core::DrillingParams =
                    serde_json::from_value(op.params.clone()).unwrap_or_default();
                p.holes = features.iter().filter_map(openmill_core::feature_to_hole).collect();
                let n = p.holes.len();
                op.params = serde_json::to_value(&p).unwrap();
                n
            }
            FeatureAssignTarget::Tapping => {
                let mut p: openmill_core::TappingParams =
                    serde_json::from_value(op.params.clone()).unwrap_or_default();
                p.holes = features.iter().filter_map(openmill_core::feature_to_hole).collect();
                let n = p.holes.len();
                op.params = serde_json::to_value(&p).unwrap();
                n
            }
            FeatureAssignTarget::ThreadMill => {
                let mut p: openmill_core::ThreadMillingParams =
                    serde_json::from_value(op.params.clone()).unwrap_or_default();
                p.holes = features.iter().filter_map(openmill_core::feature_to_hole).collect();
                let n = p.holes.len();
                op.params = serde_json::to_value(&p).unwrap();
                n
            }
            FeatureAssignTarget::PocketClearing => {
                let mut p: openmill_core::PocketClearingParams =
                    serde_json::from_value(op.params.clone()).unwrap_or_default();
                p.pockets = features.iter().filter_map(openmill_core::PocketRef::from_feature).collect();
                let n = p.pockets.len();
                op.params = serde_json::to_value(&p).unwrap();
                n
            }
        }
    }
}

/// Unit displayed in the stock-size editor. The on-disk data model always
/// stores millimetres; this only affects what the user sees and edits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StockUnits {
    Mm,
    /// Decimal inches (e.g. 1.5 in, not 1-1/2).
    Inch,
}

impl StockUnits {
    /// One unit in millimetres.
    pub fn to_mm(self) -> f64 {
        match self {
            StockUnits::Mm   => 1.0,
            StockUnits::Inch => 25.4,
        }
    }
    pub fn suffix(self) -> &'static str {
        match self {
            StockUnits::Mm   => " mm",
            StockUnits::Inch => " in",
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            StockUnits::Mm   => "mm",
            StockUnits::Inch => "in",
        }
    }
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
            view_mode: ViewMode::Plan,
            stock_units: StockUnits::Mm,
            features: Vec::new(),
            pick_face_mode: false,
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

        // Voxel carving and collision tracking only run in Simulation view —
        // Plan view leaves the original mesh untouched.
        if self.view_mode == ViewMode::Simulation && self.sim_progress > self.sim_prev_progress {
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
                        
                        // Check if the tool tip path intersects the mesh (simple tip collision)
                        if let Some(toi) = model.mesh.cast_local_ray(&ray, dist, true) {
                            let hit_pt = ray.point_at(toi);
                            self.sim_collisions.push(hit_pt);
                            vp.upload_collisions(&rs.device, &self.sim_collisions);
                        }

                        // Holder collision: tool's holder envelope vs mesh
                        // vertices at the END pose. Catches the most common
                        // 5-axis crash mode where the spindle nose hits the
                        // part long before the cutter does.
                        if let Some(hit) = openmill_core::holder_collision(&tool, &pose2, &model.mesh) {
                            self.sim_collisions.push(hit);
                            vp.upload_collisions(&rs.device, &self.sim_collisions);
                        }
                    }
                }
            }
        }
        self.sim_prev_progress = self.sim_progress;

        // If we switched operations or need to catch up the stock state.
        // Only do this work in Simulation mode — Plan mode never touches the
        // voxel volume.
        if self.view_mode == ViewMode::Simulation {
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
        } // end if Simulation view

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
    fn handle_face_pick(&mut self, click_pos: egui::Pos2, rect: egui::Rect) {
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
            let durations: Vec<f64> = self.toolpaths.iter()
                .map(|paths| paths.iter().map(|tp| tp.duration_minutes(RAPID_MM_MIN)).sum::<f64>())
                .collect();
            let job_total: f64 = durations.iter().sum();
            if job_total > 0.0 {
                ui.weak(format!("Total estimated runtime: {}", fmt_minutes(job_total)));
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
                vp.render(
                    rs, &self.camera, w, h,
                    self.sim_progress, self.sim_show_full_path,
                    show_mesh, show_voxel,
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
        self.log.push(format!(
            "Opened job from {} — {} ops, {} tools.",
            path.display(),
            self.job.operations.len(),
            self.job.tools.len(),
        ));
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
                // New model invalidates any features detected on the previous one,
                // along with any picked-face highlight from the prior session.
                self.features.clear();
                self.job.model_position = [0.0, 0.0, 0.0];
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
    fn action_verify(&mut self) -> bool {
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

    fn action_export_gcode(&mut self, post: &dyn PostProcessor) {
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

        // Inject the op-level stock-to-leave into the strategy params JSON.
        // Strategies that have a `stock_to_leave` field will pick it up; the
        // rest ignore the extra key. This keeps the per-op skin a single
        // source of truth instead of forcing the user to set it twice.
        let params_json = inject_stock_to_leave(&op.params, op.stock_to_leave);

        let result = match op.strategy.as_str() {
            "3+2 Indexed" => {
                let params: ThreePlusTwoParams =
                    serde_json::from_value(params_json.clone()).unwrap_or_default();
                ThreePlusTwo.generate(model, tool, &self.job.machine, &params)
            }
            "4+1 Indexed" => {
                let params: FourPlusOneParams =
                    serde_json::from_value(params_json.clone()).unwrap_or_default();
                FourPlusOne.generate(model, tool, &self.job.machine, &params)
            }
            "Adaptive Clearing" => {
                let params: AdaptiveClearingParams =
                    serde_json::from_value(params_json.clone()).unwrap_or_default();
                AdaptiveClearing.generate(model, tool, &self.job.machine, &params)
            }
            "Surface Normal 5-Axis" => {
                let params: SurfaceNormal5AxisParams =
                    serde_json::from_value(params_json.clone()).unwrap_or_default();
                SurfaceNormal5Axis.generate(model, tool, &self.job.machine, &params)
            }
            "Contour Parallel" => {
                let params: ContourParallelParams =
                    serde_json::from_value(params_json.clone()).unwrap_or_default();
                ContourParallel.generate(model, tool, &self.job.machine, &params)
            }
            "Swarf 5-Axis" => {
                let params: Swarf5AxisParams =
                    serde_json::from_value(params_json.clone()).unwrap_or_default();
                Swarf5Axis.generate(model, tool, &self.job.machine, &params)
            }
            "Geodesic Parallel" => {
                let params: GeodesicParams =
                    serde_json::from_value(params_json.clone()).unwrap_or_default();
                GeodesicParallel.generate(model, tool, &self.job.machine, &params)
            }
            "5-Axis Pencil Tracing" => {
                let params: PencilParams =
                    serde_json::from_value(params_json.clone()).unwrap_or_default();
                PencilTracing.generate(model, tool, &self.job.machine, &params)
            }
            "5-Axis Drilling" => {
                let params: DrillingParams =
                    serde_json::from_value(params_json.clone()).unwrap_or_default();
                Drilling5Axis.generate(model, tool, &self.job.machine, &params)
            }
            "Tapping" => {
                let params: TappingParams =
                    serde_json::from_value(params_json.clone()).unwrap_or_default();
                Tapping.generate(model, tool, &self.job.machine, &params)
            }
            "Thread Milling" => {
                let params: ThreadMillingParams =
                    serde_json::from_value(params_json.clone()).unwrap_or_default();
                ThreadMilling.generate(model, tool, &self.job.machine, &params)
            }
            "Pocket Clearing" => {
                let params: PocketClearingParams =
                    serde_json::from_value(params_json.clone()).unwrap_or_default();
                PocketClearing.generate(model, tool, &self.job.machine, &params)
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

    fn update_sim_viewport(&mut self) {
        let Some(idx) = self.selected_op else { return };
        if idx >= self.toolpaths.len() { return; }

        if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
            vp.upload_toolpath(&rs.device, &self.toolpaths[idx]);
        }
    }
}

// ── Strategy param editors ──────────────────────────────────────────────────

/// Format a duration in minutes as `Hh Mm Ss` / `Mm Ss` / `Ss`. Used for
/// per-op and job-total runtime estimates in the Operations panel.
fn fmt_minutes(minutes: f64) -> String {
    let total_secs = (minutes * 60.0).round() as i64;
    let h = total_secs / 3600;
    let m = (total_secs % 3600) / 60;
    let s = total_secs % 60;
    if h > 0 {
        format!("{h}h {m}m {s}s")
    } else if m > 0 {
        format!("{m}m {s}s")
    } else {
        format!("{s}s")
    }
}

/// Inject the op-level `stock_to_leave` into a strategy params JSON object.
/// Strategies whose params struct has a `stock_to_leave` field will pick this
/// up via serde; the rest will deserialise as if the key wasn't there. Keeping
/// this as a JSON-side mutation lets us avoid a per-strategy plumbing arm.
fn inject_stock_to_leave(params: &serde_json::Value, value: f64) -> serde_json::Value {
    let mut out = params.clone();
    if let serde_json::Value::Object(obj) = &mut out {
        obj.insert("stock_to_leave".to_string(), serde_json::json!(value));
    }
    out
}

fn default_params_for(strategy: &str) -> serde_json::Value {
    match strategy {
        "3+2 Indexed" => serde_json::to_value(ThreePlusTwoParams::default()).unwrap(),
        "4+1 Indexed" => serde_json::to_value(FourPlusOneParams::default()).unwrap(),
        "Adaptive Clearing" => serde_json::to_value(AdaptiveClearingParams::default()).unwrap(),
        "Contour Parallel" => serde_json::to_value(ContourParallelParams::default()).unwrap(),
        "Surface Normal 5-Axis" => serde_json::to_value(SurfaceNormal5AxisParams::default()).unwrap(),
        "Swarf 5-Axis" => serde_json::to_value(Swarf5AxisParams::default()).unwrap(),
        "Geodesic Parallel" => serde_json::to_value(GeodesicParams::default()).unwrap(),
        "5-Axis Pencil Tracing" => serde_json::to_value(PencilParams::default()).unwrap(),
        "5-Axis Drilling" => serde_json::to_value(DrillingParams::default()).unwrap(),
        "Tapping" => serde_json::to_value(TappingParams::default()).unwrap(),
        "Thread Milling" => serde_json::to_value(ThreadMillingParams::default()).unwrap(),
        "Pocket Clearing" => serde_json::to_value(PocketClearingParams::default()).unwrap(),
        _ => serde_json::Value::Object(Default::default()),
    }
}

fn show_strategy_params(
    ui: &mut egui::Ui,
    op: &mut Operation,
    model_aabb: Option<&parry3d::bounding_volume::Aabb>,
) {
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
        "4+1 Indexed" => {
            let mut p: FourPlusOneParams =
                serde_json::from_value(op.params.clone()).unwrap_or_default();
            let mut changed = false;
            changed |= drag(ui, "A angle (fixed)", &mut p.a_deg, 0.5, "deg");
            ui.separator();
            ui.weak("C axis — stepped between passes:");
            changed |= drag(ui, "C start", &mut p.c_start_deg, 0.5, "deg");
            changed |= drag(ui, "C step", &mut p.c_step_deg, 0.5, "deg");
            changed |= drag(ui, "C range", &mut p.c_range_deg, 1.0, "deg");
            ui.separator();
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

            ui.horizontal(|ui| {
                ui.label(format!("Holes: {}", p.holes.len()));
                if ui.button("Add Hole").clicked() {
                    // Default a new hole to the top-centre of the loaded model
                    // (or origin if no model is loaded), pointing -Z.
                    let (pos, depth) = match model_aabb {
                        Some(aabb) => {
                            let cx = (aabb.mins.x + aabb.maxs.x) as f64 * 0.5;
                            let cy = (aabb.mins.y + aabb.maxs.y) as f64 * 0.5;
                            let top_z = aabb.maxs.z as f64;
                            let model_h = (aabb.maxs.z - aabb.mins.z) as f64;
                            (
                                nalgebra::Point3::new(cx, cy, top_z),
                                model_h.max(5.0),
                            )
                        }
                        None => (nalgebra::Point3::new(0.0, 0.0, 10.0), 10.0),
                    };
                    p.holes.push(openmill_core::strategies::drilling::Hole {
                        position: pos,
                        axis: nalgebra::Vector3::new(0.0, 0.0, -1.0),
                        depth,
                    });
                    changed = true;
                }
                if !p.holes.is_empty() && ui.button("Clear All").clicked() {
                    p.holes.clear();
                    changed = true;
                }
            });

            let mut remove_hole = None;
            for (h_idx, hole) in p.holes.iter_mut().enumerate() {
                ui.group(|ui| {
                    ui.horizontal(|ui| {
                        ui.strong(format!("Hole {}", h_idx + 1));
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.small_button("🗑").clicked() {
                                remove_hole = Some(h_idx);
                            }
                        });
                    });
                    ui.horizontal(|ui| {
                        ui.label("Pos:");
                        if ui.add(egui::DragValue::new(&mut hole.position.x).speed(0.5).prefix("X ")).changed() { changed = true; }
                        if ui.add(egui::DragValue::new(&mut hole.position.y).speed(0.5).prefix("Y ")).changed() { changed = true; }
                        if ui.add(egui::DragValue::new(&mut hole.position.z).speed(0.5).prefix("Z ")).changed() { changed = true; }
                    });
                    ui.horizontal(|ui| {
                        ui.label("Axis:");
                        if ui.add(egui::DragValue::new(&mut hole.axis.x).speed(0.05).prefix("X ")).changed() { changed = true; }
                        if ui.add(egui::DragValue::new(&mut hole.axis.y).speed(0.05).prefix("Y ")).changed() { changed = true; }
                        if ui.add(egui::DragValue::new(&mut hole.axis.z).speed(0.05).prefix("Z ")).changed() { changed = true; }
                    });
                    changed |= drag(ui, "Depth", &mut hole.depth, 0.1, "mm");
                });
            }
            if let Some(i) = remove_hole {
                p.holes.remove(i);
                changed = true;
            }

            ui.separator();
            changed |= drag(ui, "Feed rate", &mut p.feed_rate, 10.0, "mm/min");
            changed |= drag(ui, "Dwell", &mut p.dwell, 0.1, "s");
            if changed {
                op.params = serde_json::to_value(&p).unwrap();
            }
        }
        "Tapping" => {
            let mut p: TappingParams =
                serde_json::from_value(op.params.clone()).unwrap_or_default();
            let mut changed = false;
            ui.label(format!("Holes: {} (use the Features panel to populate)", p.holes.len()));
            if !p.holes.is_empty() && ui.button("Clear All Holes").clicked() {
                p.holes.clear();
                changed = true;
            }
            ui.separator();
            changed |= drag(ui, "Thread pitch", &mut p.thread_pitch, 0.05, "mm/rev");
            changed |= drag(ui, "Spindle RPM (synced)", &mut p.spindle_rpm, 10.0, "rpm");
            changed |= drag(ui, "Peck depth (0 = full depth)", &mut p.peck_depth, 0.1, "mm");
            changed |= drag(ui, "Dwell at bottom", &mut p.dwell, 0.1, "s");
            ui.weak(format!(
                "Synced feed: {:.0} mm/min  (= rpm × pitch)",
                p.spindle_rpm * p.thread_pitch
            ));
            if changed {
                op.params = serde_json::to_value(&p).unwrap();
            }
        }
        "Thread Milling" => {
            let mut p: ThreadMillingParams =
                serde_json::from_value(op.params.clone()).unwrap_or_default();
            let mut changed = false;
            ui.label(format!("Holes: {} (use the Features panel to populate)", p.holes.len()));
            if !p.holes.is_empty() && ui.button("Clear All Holes").clicked() {
                p.holes.clear();
                changed = true;
            }
            ui.separator();
            changed |= drag(ui, "Thread Ø (major)", &mut p.thread_diameter, 0.05, "mm");
            changed |= drag(ui, "Thread pitch", &mut p.thread_pitch, 0.05, "mm/rev");
            changed |= drag(ui, "Thread depth", &mut p.thread_depth, 0.1, "mm");
            changed |= drag(ui, "Feed rate", &mut p.feed_rate, 10.0, "mm/min");
            let mut seg = p.segments_per_rev as f64;
            if drag(ui, "Segments / rev", &mut seg, 1.0, "") {
                p.segments_per_rev = seg.max(8.0) as usize;
                changed = true;
            }
            ui.horizontal(|ui| {
                ui.label("Direction:");
                if ui.selectable_label(p.climb,  "Climb").clicked()        && !p.climb { p.climb = true;  changed = true; }
                if ui.selectable_label(!p.climb, "Conventional").clicked() && p.climb  { p.climb = false; changed = true; }
            });
            if changed {
                op.params = serde_json::to_value(&p).unwrap();
            }
        }
        "Pocket Clearing" => {
            let mut p: PocketClearingParams =
                serde_json::from_value(op.params.clone()).unwrap_or_default();
            let mut changed = false;
            ui.label(format!("Pockets: {} (use the Features panel to populate)", p.pockets.len()));
            if !p.pockets.is_empty() && ui.button("Clear All Pockets").clicked() {
                p.pockets.clear();
                changed = true;
            }
            ui.separator();
            changed |= drag(ui, "Step-down", &mut p.step_down, 0.1, "mm");
            changed |= drag(ui, "Step-over", &mut p.step_over, 0.01, "");
            changed |= drag(ui, "Feed rate", &mut p.feed_rate, 10.0, "mm/min");
            changed |= drag(ui, "Stock to leave", &mut p.stock_to_leave, 0.05, "mm");
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

mod file_actions;
mod menu;
mod panels;
mod sidebar;
mod strategy_params;
mod tabs;

use strategy_params::{default_params_for, show_strategy_params};

use std::path::PathBuf;
use parry3d::query::RayCast;

use eframe::egui;
use eframe::egui_wgpu;
use openmill_core::*;

use crate::autosave::{self, Autosave, Recovery};
use crate::bundle::ModelBlob;
use crate::generate::{GenMsg, GenerationTask};
use crate::history::History;
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
    "Simultaneous 5-Axis Roughing",
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

    /// When `true`, the viewport renders the machine's linear-travel limits
    /// as a wireframe box. Off by default to keep the scene uncluttered;
    /// users flip it on to verify their part is inside the work area.
    show_envelope: bool,

    /// "Verify" view: shade the carved stock surface by signed distance to
    /// the target part mesh — Fusion-style deviation heatmap. The SDF is
    /// computed lazily the first time the toggle flips on, then cached.
    verify_mode: bool,
    verify_tolerance_mm: f32,

    /// Render the simulated stock as a marching-cubes iso-surface
    /// (smooth lit mesh) instead of ray-marching the voxel volume.
    /// Off by default; the ray-march is the path that supports Verify
    /// mode's deviation heatmap and updates incrementally during play.
    show_iso_surface: bool,
    /// Set once the SDF has been computed and uploaded for the current
    /// model. Cleared whenever the model or its position changes.
    sdf_uploaded: bool,

    // Tab system
    active_tab: AppTab,

    // Libraries
    machine_library: MachineLibrary,

    // Undo / redo + autosave
    history: History,
    autosave: Autosave,
    /// If `Some`, the previous session had an unsaved autosave on disk.
    /// A modal is shown until the user picks Recover or Discard.
    recovery_offer: Option<Recovery>,

    /// Raw bytes of the currently imported model, kept so a `.omp` bundle
    /// save can re-embed the original file faithfully. `None` if no model
    /// has been imported, or if the model was loaded by re-deriving from
    /// a non-bundled source we can't preserve.
    imported_model: Option<ModelBlob>,

    /// Active background-generation task, if any. While `Some`, the UI shows
    /// a progress bar + cancel button and routes finished toolpaths into
    /// `self.toolpaths` as messages arrive.
    generation: Option<GenerationTask>,
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

        let job = Job::default();
        let recovery_offer = autosave::load_recovery();

        let mut app = Self {
            history: History::new(&job),
            autosave: Autosave::new(),
            recovery_offer,
            imported_model: None,
            generation: None,
            job,
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
            show_envelope: false,
            verify_mode: false,
            verify_tolerance_mm: 0.05,
            show_iso_surface: false,
            sdf_uploaded: false,
            active_tab: AppTab::Manufacturing,
            machine_library: MachineLibrary::load("machines").unwrap_or_default(),
        };
        app.load_tools_from_dir();
        // Loading tools mutates the job — re-baseline so it isn't undoable
        // back to "no tools".
        app.history.reset(&app.job);
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
        // Global undo/redo shortcuts. Consume before egui's TextEdit gets the
        // event so Ctrl+Z works even when a numeric field is focused.
        let (do_undo, do_redo) = ctx.input_mut(|i| {
            let undo = i.consume_shortcut(&egui::KeyboardShortcut::new(
                egui::Modifiers::CTRL, egui::Key::Z,
            ));
            let redo = i.consume_shortcut(&egui::KeyboardShortcut::new(
                egui::Modifiers::CTRL, egui::Key::Y,
            )) || i.consume_shortcut(&egui::KeyboardShortcut::new(
                egui::Modifiers::CTRL | egui::Modifiers::SHIFT, egui::Key::Z,
            ));
            (undo, redo)
        });
        if do_undo && self.history.undo(&mut self.job) {
            self.on_job_replaced();
            self.log.push("Undo".into());
        }
        if do_redo && self.history.redo(&mut self.job) {
            self.on_job_replaced();
            self.log.push("Redo".into());
        }

        // Crash-recovery prompt. Blocks the rest of the UI until resolved so
        // the user can't start editing into a stale baseline.
        if self.recovery_offer.is_some() {
            self.show_recovery_modal(ctx);
        }

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
            // Stamp this carve with the active op's id so the carved
            // surface picks up the per-op color palette.
            let op_id = self.selected_op.unwrap_or(0) as u32;
            if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
                if let (Some((pose1, tool)), Some((pose2, _))) = (p1, p2) {
                    vp.carve_voxel(&rs.device, &rs.queue, &pose1, &pose2, &tool, op_id);

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
                    // Re-extract the iso-surface from the now-modified
                    // voxel grid so Smooth view tracks the live carve.
                    // Cheap (~5 ms at 128³ on a typical desktop GPU);
                    // skipped entirely when the toggle is off.
                    if self.show_iso_surface {
                        vp.extract_iso_surface(&rs.device, &rs.queue);
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

                                        vp.carve_voxel(&rs.device, &rs.queue, &pose1, &pose2, tool, prev_idx as u32);
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
                    // Voxel grid was replayed from scratch — refresh
                    // the iso-surface mesh once so Smooth view picks up
                    // the new stock state on the next frame.
                    if self.show_iso_surface {
                        vp.extract_iso_surface(&rs.device, &rs.queue);
                    }
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

        // Drain any messages the background generator emitted this frame.
        self.poll_generation(ctx);

        // After all panels have run their mutating callbacks: snapshot for
        // undo and tick the autosave timer.
        self.history.frame(&self.job);
        if let Some(p) = self.autosave.tick(&self.job) {
            self.log.push(format!("Autosaved to {}", p.display()));
        }
    }
}

// ── Background generation ───────────────────────────────────────────────────

impl OpenMillApp {
    /// Drain messages from the background generator and merge results into
    /// `self.toolpaths`. Called once per frame.
    fn poll_generation(&mut self, ctx: &egui::Context) {
        // Pull all pending messages out of the task in a scoped borrow so
        // we can call mutating methods on `self` (update_sim_*) below.
        let (msgs, task_finished) = {
            let Some(task) = self.generation.as_mut() else { return };
            (task.poll(), task.is_finished())
        };
        let mut any_finished = false;
        let mut done = false;
        for msg in msgs {
            match msg {
                GenMsg::Started { op_idx, op_name } => {
                    self.log.push(format!("Generating op {}: \"{}\"...", op_idx + 1, op_name));
                }
                GenMsg::Finished { op_idx, paths } => {
                    let total_pts: usize = paths.iter().map(|p| p.points.len()).sum();
                    let name = self.job.operations.get(op_idx).map(|o| o.name.clone())
                        .unwrap_or_default();
                    self.log.push(format!(
                        "  ✓ {} toolpath(s), {} pts for \"{}\".",
                        paths.len(), total_pts, name,
                    ));
                    while self.toolpaths.len() <= op_idx {
                        self.toolpaths.push(Vec::new());
                    }
                    self.toolpaths[op_idx] = paths;
                    any_finished = true;
                }
                GenMsg::Failed { op_idx, op_name, error } => {
                    self.log.push(format!("  ✗ op {} \"{}\": {error}", op_idx + 1, op_name));
                }
                GenMsg::Skipped { op_idx, op_name, reason } => {
                    self.log.push(format!("  – skipped op {} \"{}\": {reason}", op_idx + 1, op_name));
                }
                GenMsg::Done { cancelled } => {
                    self.log.push(if cancelled {
                        "Generation cancelled.".into()
                    } else {
                        "Generation complete.".into()
                    });
                    done = true;
                }
            }
        }
        if any_finished {
            // Refresh the G-code panel + toolpath overlay for the currently
            // selected op, in case its result just landed.
            self.update_sim_gcode();
            self.update_sim_viewport();
        }
        if done || task_finished {
            if let Some(task) = self.generation.take() {
                task.join();
            }
        } else {
            // Keep repainting while the worker is running so the progress
            // bar animates smoothly even without user input.
            ctx.request_repaint();
        }
    }

    /// Spawn the background worker for every enabled operation in the job.
    fn action_generate_all(&mut self) {
        if self.generation.is_some() {
            self.log.push("Generation already running.".into());
            return;
        }
        let Some(ref model) = self.model else {
            self.log.push("Import a model first.".into());
            return;
        };
        if self.job.operations.is_empty() {
            self.log.push("No operations to generate.".into());
            return;
        }
        // Pre-size toolpaths so completed messages can index in directly.
        self.toolpaths.resize_with(self.job.operations.len(), Vec::new);
        let task = GenerationTask::spawn(&self.job, model);
        if task.total == 0 {
            self.log.push("No enabled operations to generate.".into());
            task.join();
            return;
        }
        self.log.push(format!("Generating {} operation(s)...", task.total));
        self.generation = Some(task);
    }
}

// ── Undo / redo support ─────────────────────────────────────────────────────

impl OpenMillApp {
    /// Reset transient state that depends on the structure of `job`. Called
    /// after an undo/redo replaces the job, since the operation/tool indices
    /// may now point to different (or removed) items.
    fn on_job_replaced(&mut self) {
        self.toolpaths = vec![Vec::new(); self.job.operations.len()];
        if let Some(idx) = self.selected_op {
            if idx >= self.job.operations.len() {
                self.selected_op = None;
            }
        }
        if let Some(idx) = self.selected_tool {
            if idx >= self.job.tools.len() {
                self.selected_tool = None;
            }
        }
        self.sim_op_idx = None;
        self.sim_progress = 0.0;
        self.sim_prev_progress = 0.0;
        self.sim_collisions.clear();
        self.sim_gcode.clear();
        // Features were detected against the live model and don't live in the
        // job — clear them so stale picks don't reference removed ops.
        self.features.clear();
        if let (Some(rs), Some(vp)) = (&self.render_state, &mut self.viewport) {
            vp.upload_collisions(&rs.device, &[]);
            vp.clear_pick();
        }
    }

    /// Re-extract the iso-surface from the current voxel state. Called
    /// from the UI when the user toggles Smooth on, and from the update
    /// loop whenever the voxel volume has been carved.
    pub(super) fn extract_iso_surface_once(&mut self) {
        let Some(rs) = self.render_state.as_ref() else { return };
        let Some(vp) = self.viewport.as_mut() else { return };
        vp.extract_iso_surface(&rs.device, &rs.queue);
    }

    /// Compute the signed-distance field for the current model and upload
    /// it to GPU so Verify mode can shade the carved stock by deviation.
    /// Resolution is fixed at 128³ — at that size a typical hobby part
    /// takes a few hundred ms on a modern multi-core CPU. The work runs
    /// synchronously: it's an explicit user action and freezing the UI
    /// for a fraction of a second is preferable to plumbing a worker
    /// thread + bind-group rebuild dance.
    pub(super) fn compute_and_upload_sdf(&mut self) {
        let Some(ref model) = self.model else {
            self.log.push("Verify: no model loaded.".into());
            return;
        };
        let Some(rs) = self.render_state.as_ref() else { return };
        let Some(vp) = self.viewport.as_mut() else { return };

        // Anchor the SDF to the **stock** bounds, not the part AABB — the
        // carved stock surface (what the shader will sample) lives inside
        // the stock volume, including the margin around the part.
        let aabb = model.stock_aabb();
        let res = openmill_core::sdf::DEFAULT_RESOLUTION;
        self.log.push(format!(
            "Verify: computing SDF at {}×{}×{}…",
            res[0], res[1], res[2],
        ));
        let t0 = std::time::Instant::now();
        let data = openmill_core::sdf::compute_sdf(&model.mesh, &aabb, res);
        let dt = t0.elapsed().as_millis();
        vp.upload_sdf(&rs.device, &rs.queue, &data, res);
        self.sdf_uploaded = true;
        self.log.push(format!("Verify: SDF ready ({} ms).", dt));
    }

    fn show_recovery_modal(&mut self, ctx: &egui::Context) {
        let mut decision: Option<bool> = None; // Some(true) = recover, Some(false) = discard
        let info = self.recovery_offer.as_ref().map(|r| {
            let when = r.modified
                .map(autosave::relative_age)
                .unwrap_or_else(|| "unknown time".into());
            (r.job.name.clone(), r.job.operations.len(), r.job.tools.len(), when)
        });
        if let Some((name, n_ops, n_tools, when)) = info {
            egui::Window::new("Recover unsaved job?")
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, 0.0))
                .show(ctx, |ui| {
                    ui.label("An autosave from a previous session was found:");
                    ui.add_space(4.0);
                    ui.label(format!("  Name: {}", name));
                    ui.label(format!("  Ops:  {} / Tools: {}", n_ops, n_tools));
                    ui.label(format!("  Last saved: {}", when));
                    ui.add_space(8.0);
                    ui.horizontal(|ui| {
                        if ui.button("Recover").clicked() {
                            decision = Some(true);
                        }
                        if ui.button("Discard").clicked() {
                            decision = Some(false);
                        }
                    });
                });
        }
        match decision {
            Some(true) => {
                if let Some(r) = self.recovery_offer.take() {
                    self.job = r.job;
                    self.on_job_replaced();
                    self.history.reset(&self.job);
                    self.autosave.reset_baseline();
                    self.log.push("Recovered job from autosave.".into());
                }
            }
            Some(false) => {
                Autosave::clear();
                self.recovery_offer = None;
                self.log.push("Discarded autosave.".into());
            }
            None => {}
        }
    }
}







// ── Strategy param editors ──────────────────────────────────────────────────

/// Format a duration in minutes as `Hh Mm Ss` / `Mm Ss` / `Ss`. Used for
/// per-op and job-total runtime estimates in the Operations panel.
pub(super) fn fmt_minutes(minutes: f64) -> String {
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


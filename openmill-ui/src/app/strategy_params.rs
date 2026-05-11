//! Per-strategy parameter editors.
//!
//! `show_strategy_params` renders the right-hand-side controls in the
//! operations panel for whichever strategy the selected op is using.
//! `default_params_for` returns a sensible starting `params` JSON when the
//! user changes an op's strategy from the picker.

use eframe::egui;
use openmill_core::*;

pub(super) fn default_params_for(strategy: &str) -> serde_json::Value {
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
        "Simultaneous 5-Axis Roughing" => serde_json::to_value(MultiAxisRoughingParams::default()).unwrap(),
        _ => serde_json::Value::Object(Default::default()),
    }
}

pub(super) fn show_strategy_params(
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
            changed |= cusp_height_control(ui, &mut p.step_over_cusp_mm);
            changed |= drag(ui, "Feed rate", &mut p.feed_rate, 10.0, "mm/min");
            changed |= common_strategy_controls(ui, &mut p.direction, &mut p.z_range, &mut p.spring_pass);
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
            changed |= cusp_height_control(ui, &mut p.step_over_cusp_mm);
            changed |= drag(ui, "Feed rate", &mut p.feed_rate, 10.0, "mm/min");
            changed |= common_strategy_controls(ui, &mut p.direction, &mut p.z_range, &mut p.spring_pass);
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
            changed |= common_strategy_controls(ui, &mut p.direction, &mut p.z_range, &mut p.spring_pass);
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
            changed |= common_strategy_controls(ui, &mut p.direction, &mut p.z_range, &mut p.spring_pass);
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
            changed |= cusp_height_control(ui, &mut p.step_over_cusp_mm);
            changed |= drag(ui, "Feed rate", &mut p.feed_rate, 10.0, "mm/min");
            changed |= drag(ui, "Tolerance", &mut p.tolerance, 0.001, "mm");
            changed |= common_strategy_controls(ui, &mut p.direction, &mut p.z_range, &mut p.spring_pass);
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
            changed |= cusp_height_control(ui, &mut p.step_over_cusp_mm);
            changed |= drag(ui, "Feed rate", &mut p.feed_rate, 10.0, "mm/min");
            changed |= drag(ui, "Tolerance", &mut p.tolerance, 0.001, "mm");
            changed |= common_strategy_controls(ui, &mut p.direction, &mut p.z_range, &mut p.spring_pass);
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
            changed |= common_strategy_controls(ui, &mut p.direction, &mut p.z_range, &mut p.spring_pass);
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
            // ── Cycle picker ────────────────────────────────────
            ui.horizontal(|ui| {
                ui.label("Cycle:");
                egui::ComboBox::from_id_salt("drill_cycle")
                    .selected_text(match p.cycle {
                        CycleType::Drill       => "Drill (G81)",
                        CycleType::CounterBore => "Counter-bore (G82)",
                        CycleType::Peck        => "Peck (G83)",
                        CycleType::ChipBreak   => "Chip-break (G73)",
                        CycleType::Bore        => "Bore (G85)",
                    })
                    .show_ui(ui, |ui| {
                        changed |= ui.selectable_value(&mut p.cycle, CycleType::Drill,       "Drill (G81)").changed();
                        changed |= ui.selectable_value(&mut p.cycle, CycleType::CounterBore, "Counter-bore (G82)").changed();
                        changed |= ui.selectable_value(&mut p.cycle, CycleType::Peck,        "Peck (G83) — full retract").changed();
                        changed |= ui.selectable_value(&mut p.cycle, CycleType::ChipBreak,   "Chip-break (G73) — short retract").changed();
                        changed |= ui.selectable_value(&mut p.cycle, CycleType::Bore,        "Bore (G85) — feed both ways").changed();
                    });
            });
            changed |= drag(ui, "Feed rate", &mut p.feed_rate, 10.0, "mm/min");

            // Cycle-specific parameters. Each variant only shows the
            // controls it actually consumes so the UI doesn't drown the
            // user in irrelevant fields.
            match p.cycle {
                CycleType::CounterBore => {
                    changed |= drag(ui, "Dwell at bottom", &mut p.dwell, 0.1, "s");
                }
                CycleType::Peck | CycleType::ChipBreak => {
                    changed |= drag(ui, "Peck depth (0 = ¼ total)", &mut p.peck_depth, 0.1, "mm");
                    if p.cycle == CycleType::ChipBreak {
                        changed |= drag(ui, "Chip-break retract", &mut p.chipbreak_retract, 0.05, "mm");
                    }
                }
                CycleType::Drill | CycleType::Bore => {}
            }
            changed |= drag(ui, "Break-through (0 = stop at depth)", &mut p.break_through, 0.1, "mm");

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
            ui.horizontal(|ui| {
                ui.label("Hand:");
                changed |= ui.selectable_value(&mut p.thread_direction, ThreadDirection::RightHand, "Right (M3)").changed();
                changed |= ui.selectable_value(&mut p.thread_direction, ThreadDirection::LeftHand,  "Left (M4)").changed();
            });
            ui.horizontal(|ui| {
                ui.label("Mode:");
                changed |= ui.selectable_value(&mut p.tap_mode, TapMode::Rigid, "Rigid")
                    .on_hover_text("Spindle-synchronised tapping. Fanuc/Haas post emits M29 S<rpm> before the cycle.")
                    .changed();
                changed |= ui.selectable_value(&mut p.tap_mode, TapMode::Floating, "Floating")
                    .on_hover_text("Floating tap holder absorbs feed/pitch mismatch. No M29.")
                    .changed();
            });
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
                ui.label("Cut:");
                if ui.selectable_label(p.climb,  "Climb").clicked()        && !p.climb { p.climb = true;  changed = true; }
                if ui.selectable_label(!p.climb, "Conventional").clicked() && p.climb  { p.climb = false; changed = true; }
            });
            ui.horizontal(|ui| {
                ui.label("Hand:");
                changed |= ui.selectable_value(&mut p.thread_direction, ThreadDirection::RightHand, "Right").changed();
                changed |= ui.selectable_value(&mut p.thread_direction, ThreadDirection::LeftHand,  "Left").changed();
            });
            ui.horizontal(|ui| {
                ui.label("Type:");
                changed |= ui.selectable_value(&mut p.internal, true,  "Internal (bore)").changed();
                changed |= ui.selectable_value(&mut p.internal, false, "External (boss)").changed();
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
            changed |= common_strategy_controls(ui, &mut p.direction, &mut p.z_range, &mut p.spring_pass);
            if changed {
                op.params = serde_json::to_value(&p).unwrap();
            }
        }
        "Simultaneous 5-Axis Roughing" => {
            let mut p: MultiAxisRoughingParams =
                serde_json::from_value(op.params.clone()).unwrap_or_default();
            let mut changed = false;
            changed |= drag(ui, "Step-down", &mut p.step_down, 0.1, "mm");
            changed |= drag(ui, "Step-over", &mut p.step_over, 0.01, "");
            changed |= drag(ui, "Feed rate", &mut p.feed_rate, 10.0, "mm/min");
            changed |= drag(ui, "Max tilt", &mut p.max_tilt_deg, 0.5, "deg");
            changed |= drag(ui, "Lead angle", &mut p.lead_angle, 0.5, "deg");
            changed |= common_strategy_controls(ui, &mut p.direction, &mut p.z_range, &mut p.spring_pass);
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

/// Shared editor for every strategy's `CutDirection` + `ZRange` +
/// `SpringPass` fields. Returns `true` if the user edited anything so
/// the caller can re-serialise the params blob.
fn common_strategy_controls(
    ui: &mut egui::Ui,
    direction: &mut CutDirection,
    z_range: &mut ZRange,
    spring_pass: &mut SpringPass,
) -> bool {
    let mut changed = false;
    ui.separator();
    ui.weak("Cut direction & Z range:");

    // ── Direction picker ────────────────────────────────────────────
    ui.horizontal(|ui| {
        ui.label("Direction:");
        changed |= ui.selectable_value(direction, CutDirection::Climb, "Climb")
            .on_hover_text("Strategy's native direction — climb for every current strategy.")
            .changed();
        changed |= ui.selectable_value(direction, CutDirection::Conventional, "Conventional")
            .on_hover_text("Reverse every cutting pass as a post-pass. Useful on rigid manual machines where climb chatters.")
            .changed();
        changed |= ui.selectable_value(direction, CutDirection::Either, "Either")
            .on_hover_text("Leave the strategy's pattern alone.")
            .changed();
    });

    // ── Z range toggle + values ────────────────────────────────────
    let mut has_top = z_range.top_mm.is_some();
    let mut has_bot = z_range.bottom_mm.is_some();
    ui.horizontal(|ui| {
        if ui.checkbox(&mut has_top, "Top Z").changed() {
            z_range.top_mm = if has_top { Some(z_range.top_mm.unwrap_or(0.0)) } else { None };
            changed = true;
        }
        if has_top {
            let mut v = z_range.top_mm.unwrap_or(0.0);
            if ui.add(egui::DragValue::new(&mut v).speed(0.1).suffix(" mm")).changed() {
                z_range.top_mm = Some(v);
                changed = true;
            }
        }
        ui.add_space(8.0);
        if ui.checkbox(&mut has_bot, "Bottom Z").changed() {
            z_range.bottom_mm = if has_bot { Some(z_range.bottom_mm.unwrap_or(-10.0)) } else { None };
            changed = true;
        }
        if has_bot {
            let mut v = z_range.bottom_mm.unwrap_or(-10.0);
            if ui.add(egui::DragValue::new(&mut v).speed(0.1).suffix(" mm")).changed() {
                z_range.bottom_mm = Some(v);
                changed = true;
            }
        }
    });

    // ── Spring pass ─────────────────────────────────────────────────
    ui.horizontal(|ui| {
        if ui.checkbox(&mut spring_pass.enabled, "Spring pass")
            .on_hover_text("Re-cut the final pass at reduced feed to clean up cutter deflection.")
            .changed()
        { changed = true; }
        if spring_pass.enabled {
            ui.label("× feed:");
            if ui.add(egui::DragValue::new(&mut spring_pass.feed_fraction)
                .speed(0.05).range(0.05..=1.0)).changed()
            { changed = true; }
        }
    });

    changed
}

/// Renders an optional `Cusp height (mm)` row. When set, this overrides
/// the strategy's step-over fraction. Returns `true` on edit.
fn cusp_height_control(ui: &mut egui::Ui, cusp_mm: &mut Option<f64>) -> bool {
    let mut changed = false;
    let mut enabled = cusp_mm.is_some();
    ui.horizontal(|ui| {
        if ui.checkbox(&mut enabled, "Use cusp height")
            .on_hover_text("Specify the surface scallop height directly. Overrides Step-over for ball/bull tools.")
            .changed()
        {
            *cusp_mm = if enabled { Some(cusp_mm.unwrap_or(0.02)) } else { None };
            changed = true;
        }
        if let Some(v) = cusp_mm.as_mut() {
            if ui.add(egui::DragValue::new(v).speed(0.005).range(0.001..=1.0).suffix(" mm")).changed() {
                changed = true;
            }
        }
    });
    changed
}

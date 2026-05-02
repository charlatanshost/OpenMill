//! LinuxCNC / RS274NGC post-processor.
//!
//! Cutting-mode selection:
//! - **Indexed (3+2 or 4+1):** tool axis is constant within each pass.
//!   Uses **G94 feed-per-minute** — the same as a standard 3-axis program.
//!   The rotary index move is emitted as a separate G0 before the first cut.
//! - **Continuous 5-axis:** tool axis varies point-to-point.
//!   Uses **G93 inverse-time** so that the controller can synchronise all
//!   five axes across each segment.

use anyhow::Result;
use openmill_core::{
    Coolant, MachineConfig, MoveType, Operation, PostConfig, TableTable, Tool, Toolpath,
    ToolpathPoint, Units,
};

use crate::feed_rate::{compute_inverse_time_feed_with_kin, joints_for_point};
use crate::traits::PostProcessor;

/// LinuxCNC post-processor.
pub struct LinuxCncPost;

/// Returns `true` when all points in the toolpath share the same tool-axis
/// direction (within 1 × 10⁻⁶ radians).  Used to switch between G93 and G94.
fn is_indexed_pass(toolpath: &Toolpath) -> bool {
    let Some(first) = toolpath.points.first() else { return true };
    let ref_axis = first.orientation;
    toolpath.points.iter().all(|pt| {
        (pt.orientation.as_ref() - ref_axis.as_ref()).norm() < 1e-6
    })
}

impl PostProcessor for LinuxCncPost {
    fn header(&self, config: &PostConfig) -> String {
        let unit_code = match config.units {
            Units::Metric  => "G21",
            Units::Imperial => "G20",
        };
        format!(
            "%\nO{:04}\nG90 {unit_code}\n{}\n",
            config.program_number, config.work_offset,
        )
    }

    fn process_toolpath(
        &self,
        toolpath: &Toolpath,
        machine: &MachineConfig,
    ) -> Result<Vec<(String, Option<usize>)>> {
        let tt = TableTable::new(machine.clone())?;
        let mut out = Vec::new();

        let indexed = is_indexed_pass(toolpath);
        let mode_comment = if indexed {
            "(indexed 3-axis pass — G94 feed-per-minute)"
        } else {
            "(continuous 5-axis — G93 inverse-time)"
        };

        out.push((format!("({})\n", toolpath.name), None));
        out.push((format!("{mode_comment}\n"), None));

        // For indexed passes emit the rotary index position as a G0 before
        // any cutting move, then stay in G94 for all linear axes.
        // For continuous passes enter G93 inverse-time mode.
        if indexed {
            // Emit the index move (rotary axes only) using the first point's
            // tool orientation so the controller positions A and C before
            // the spindle descends.
            if let Some(first) = toolpath.points.first() {
                let [_x, _y, _z, a, c] = joints_for_point(&tt, first)?;
                if a.abs() > 1e-3 || c.abs() > 1e-3 {
                    out.push((
                        format!("G0 A{a:.4} C{c:.4} (rotary index)\n"),
                        None,
                    ));
                }
            }
            out.push(("G94\n".to_string(), None)); // feed-per-minute
        } else {
            out.push(("G93\n".to_string(), None)); // inverse-time
        }

        let mut prev: Option<&ToolpathPoint> = None;

        for (i, pt) in toolpath.points.iter().enumerate() {
            let [x, y, z, a, c] = joints_for_point(&tt, pt)?;

            match pt.move_type {
                MoveType::Rapid | MoveType::Retract => {
                    out.push((
                        format!("G0 X{x:.4} Y{y:.4} Z{z:.4} A{a:.4} C{c:.4}\n"),
                        Some(i),
                    ));
                }
                MoveType::Linear | MoveType::LeadIn | MoveType::LeadOut => {
                    if indexed {
                        // G94: emit the programmed feed directly.
                        let f = pt.feed_rate.max(1.0);
                        out.push((
                            format!("G1 X{x:.4} Y{y:.4} Z{z:.4} A{a:.4} C{c:.4} F{f:.4}\n"),
                            Some(i),
                        ));
                    } else {
                        // G93: inverse-time F word.
                        let f = match prev {
                            Some(from) => compute_inverse_time_feed_with_kin(&tt, from, pt),
                            None => pt.feed_rate.max(1.0),
                        };
                        out.push((
                            format!("G1 X{x:.4} Y{y:.4} Z{z:.4} A{a:.4} C{c:.4} F{f:.4}\n"),
                            Some(i),
                        ));
                    }
                }
            }

            prev = Some(pt);
        }

        // Always restore feed-per-minute at the end of each toolpath block.
        if !indexed {
            out.push(("G94\n".to_string(), None));
        }

        Ok(out)
    }

    fn tool_change(&self, tool: &Tool) -> String {
        // Stop spindle and coolant before swap, then call up the new tool.
        // Spindle/coolant for the next operation are emitted in op_preamble.
        let mut s = String::new();
        s.push_str("M5\n");
        s.push_str(&format!("{}\n", Coolant::gcode_off()));
        s.push_str(&format!("M6 T{} (Tool change: {})\n", tool.id, tool.name));
        if let Some(extra) = &tool.tool_change_gcode {
            for line in extra.lines() {
                let line = line.trim_end();
                if !line.is_empty() {
                    s.push_str(line);
                    s.push('\n');
                }
            }
        }
        s
    }

    fn op_preamble(&self, op: &Operation, _tool: &Tool) -> String {
        let mut s = String::new();
        s.push_str(&format!("(--- Op: {}) \n", op.name));
        if op.spindle_speed > 0.0 {
            s.push_str(&format!("M3 S{:.0}\n", op.spindle_speed));
        }
        if let Some(code) = op.coolant.gcode_on() {
            s.push_str(code);
            s.push('\n');
        }
        for line in op.gcode_command.lines() {
            let line = line.trim_end();
            if !line.is_empty() {
                s.push_str(line);
                s.push('\n');
            }
        }
        s
    }

    fn op_postamble(&self, _op: &Operation) -> String {
        format!("{}\n", Coolant::gcode_off())
    }

    fn footer(&self) -> String {
        "M5\nM9\nM30\n%\n".to_string()
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point3, Vector3};
    use openmill_core::{MoveType, OperationType, Toolpath, ToolpathPoint};

    fn default_machine() -> MachineConfig {
        openmill_core::kinematics::default_trunnion_config()
    }

    #[test]
    fn three_point_toolpath_contains_g0_g1() {
        let machine = default_machine();
        let post = LinuxCncPost;

        let mut tp = Toolpath::new(1, OperationType::Roughing, "test-cut");
        tp.points
            .push(ToolpathPoint::rapid(Point3::new(0.0, 0.0, 10.0)));
        tp.points.push(ToolpathPoint {
            position: Point3::new(10.0, 0.0, 0.0),
            orientation: Vector3::z_axis(),
            feed_rate: 1000.0,
            move_type: MoveType::Linear,
        });
        tp.points.push(ToolpathPoint {
            position: Point3::new(10.0, 0.0, 10.0),
            orientation: Vector3::z_axis(),
            feed_rate: 0.0,
            move_type: MoveType::Retract,
        });

        let gcode_lines = post.process_toolpath(&tp, &machine).unwrap();
        let gcode = gcode_lines.into_iter().map(|(s, _)| s).collect::<String>();

        let g0_count = gcode.matches("G0 ").count();
        let g1_count = gcode.matches("G1 ").count();
        assert_eq!(g0_count, 2, "expected 2 G0 moves (rapid + retract)");
        assert_eq!(g1_count, 1, "expected 1 G1 move");
        // Vertical tool axis → indexed pass → G94, no G93.
        assert!(gcode.contains("G94"), "vertical-axis pass should use G94");
        assert!(!gcode.contains("G93"), "vertical-axis pass should not use G93");
        // Every G1 line must carry an F word.
        for line in gcode.lines() {
            if line.starts_with("G1 ") {
                assert!(line.contains('F'), "G1 line missing F word: {line}");
            }
        }
    }

    #[test]
    fn continuous_5axis_uses_g93() {
        use nalgebra::Unit;
        let machine = default_machine();
        let post = LinuxCncPost;

        // Two points with different (non-vertical) tool axes → continuous pass.
        let axis_a = Unit::new_normalize(Vector3::new(0.2, 0.0, 0.98));
        let axis_b = Unit::new_normalize(Vector3::new(-0.2, 0.0, 0.98));

        let mut tp = Toolpath::new(1, OperationType::Finishing, "5ax-test");
        tp.points.push(ToolpathPoint {
            position: Point3::new(0.0, 0.0, 5.0),
            orientation: axis_a,
            feed_rate: 0.0,
            move_type: MoveType::Rapid,
        });
        tp.points.push(ToolpathPoint {
            position: Point3::new(10.0, 0.0, 0.0),
            orientation: axis_b,
            feed_rate: 800.0,
            move_type: MoveType::Linear,
        });

        let gcode_lines = post.process_toolpath(&tp, &machine).unwrap();
        let gcode = gcode_lines.into_iter().map(|(s, _)| s).collect::<String>();

        assert!(gcode.contains("G93"), "continuous pass should use G93");
        assert!(gcode.contains("G94"), "should restore G94 at end");
    }

    #[test]
    fn header_contains_preamble() {
        let post = LinuxCncPost;
        let config = PostConfig::default();
        let hdr = post.header(&config);
        assert!(hdr.contains('%'), "should start with %");
        assert!(hdr.contains("O1000"), "should contain program number");
        assert!(hdr.contains("G90"), "should contain absolute mode");
        assert!(hdr.contains("G21"), "should contain metric code");
        assert!(hdr.contains("G54"), "should contain work offset");
    }

    #[test]
    fn footer_contains_end_codes() {
        let post = LinuxCncPost;
        let ftr = post.footer();
        assert!(ftr.contains("M5"), "should contain spindle stop");
        assert!(ftr.contains("M30"), "should contain program end");
        assert!(ftr.contains('%'), "should end with %");
    }

    #[test]
    fn tool_change_format() {
        let post = LinuxCncPost;
        let tool = openmill_core::Tool::flat_end(3, "6mm flat", 6.0, 20.0);
        let tc = post.tool_change(&tool);
        assert!(tc.contains("M6"), "should contain tool change");
        assert!(tc.contains("T3"), "should contain tool number");
        assert!(tc.contains("M5"), "should stop spindle before swap");
        assert!(tc.contains("M9"), "should stop coolant before swap");
    }

    #[test]
    fn op_preamble_emits_spindle_and_coolant() {
        let post = LinuxCncPost;
        let tool = openmill_core::Tool::flat_end(1, "6mm flat", 6.0, 20.0);
        let op = openmill_core::Operation {
            name: "Roughing".into(),
            tool_id: 1,
            strategy: "3+2 Indexed".into(),
            params: serde_json::Value::Null,
            spindle_speed: 12000.0,
            feed_rate: 800.0,
            plunge_rate: 200.0,
            coolant: openmill_core::Coolant::Flood,
            gcode_command: "G43 H1".into(),
            enabled: true,
        };
        let pre = post.op_preamble(&op, &tool);
        assert!(pre.contains("M3 S12000"), "should start spindle at op RPM: {pre}");
        assert!(pre.contains("M8"),        "should turn flood coolant on: {pre}");
        assert!(pre.contains("G43 H1"),    "should include free-form gcode: {pre}");
        let post_amble = post.op_postamble(&op);
        assert!(post_amble.contains("M9"), "postamble should turn coolant off");
    }

    #[test]
    fn tool_change_includes_tool_setup_gcode() {
        let post = LinuxCncPost;
        let mut tool = openmill_core::Tool::flat_end(7, "probe", 6.0, 20.0);
        tool.tool_change_gcode = Some("G37".into()); // probe routine
        let tc = post.tool_change(&tool);
        assert!(tc.contains("G37"), "tool-level setup gcode should be emitted");
    }
}

//! GRBL-compatible post-processor.
//!
//! Simpler than LinuxCNC: no program number, no `%` delimiters.
//! Uses G93 inverse-time mode only for **continuous** 5-axis cutting moves.
//! Indexed (3+2 / 4+1) passes use G94 feed-per-minute, same as 3-axis G-code.

use anyhow::Result;
use openmill_core::{
    Coolant, MachineConfig, MoveType, Operation, PostConfig, TableTable, Tool, Toolpath,
    ToolpathPoint, Units,
};

use crate::feed_rate::{compute_inverse_time_feed_with_kin, joints_for_point};
use crate::traits::{spindle_command_for, PostProcessor};

/// GRBL post-processor.
pub struct GrblPost;

/// Returns `true` when all points share the same tool axis (indexed pass).
fn is_indexed_pass(toolpath: &Toolpath) -> bool {
    let Some(first) = toolpath.points.first() else { return true };
    let ref_axis = first.orientation;
    toolpath.points.iter().all(|pt| {
        (pt.orientation.as_ref() - ref_axis.as_ref()).norm() < 1e-6
    })
}

impl PostProcessor for GrblPost {
    fn header(&self, config: &PostConfig) -> String {
        let unit_code = match config.units {
            Units::Metric   => "G21",
            Units::Imperial => "G20",
        };
        format!("{unit_code}\nG90\n")
    }

    fn process_toolpath(
        &self,
        toolpath: &Toolpath,
        machine: &MachineConfig,
    ) -> Result<Vec<(String, Option<usize>)>> {
        let tt = TableTable::new(machine.clone())?;
        let mut out = Vec::new();

        let indexed = is_indexed_pass(toolpath);

        out.push((format!("({})\n", toolpath.name), None));

        if indexed {
            // Emit the rotary index move before cutting.
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
                        let f = pt.feed_rate.max(1.0);
                        out.push((
                            format!("G1 X{x:.4} Y{y:.4} Z{z:.4} A{a:.4} C{c:.4} F{f:.4}\n"),
                            Some(i),
                        ));
                    } else {
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

        if !indexed {
            out.push(("G94\n".to_string(), None)); // restore feed-per-minute
        }

        Ok(out)
    }

    fn tool_change(&self, tool: &Tool) -> String {
        // GRBL has no real M6 — pause for a manual swap. Coolant off first.
        let mut s = String::new();
        s.push_str("M5\n");
        s.push_str(&format!("{}\n", Coolant::gcode_off()));
        s.push_str(&format!("M0 (Tool change: T{} {})\n", tool.id, tool.name));
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
        s.push_str(&format!("({})\n", op.name));
        if op.spindle_speed > 0.0 {
            s.push_str(&format!("{} S{:.0}\n", spindle_command_for(op), op.spindle_speed));
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
        "M5\nM9\nM30\n".to_string()
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
        let post = GrblPost;

        let mut tp = Toolpath::new(1, OperationType::Roughing, "grbl-cut");
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

        assert!(gcode.contains("G0 "), "should contain rapid G0");
        assert!(gcode.contains("G1 "), "should contain linear G1");
    }

    #[test]
    fn header_is_simple() {
        let post = GrblPost;
        let config = PostConfig::default();
        let hdr = post.header(&config);
        assert!(hdr.contains("G21"), "should contain metric code");
        assert!(hdr.contains("G90"), "should contain absolute mode");
        assert!(!hdr.contains('%'), "GRBL should not have % delimiter");
        assert!(!hdr.contains('O'), "GRBL should not have O-word");
    }

    #[test]
    fn footer_is_simple() {
        let post = GrblPost;
        let ftr = post.footer();
        assert!(ftr.contains("M5"), "should stop spindle");
        assert!(ftr.contains("M30"), "should end program");
        assert!(!ftr.contains('%'), "GRBL should not have % delimiter");
    }
}

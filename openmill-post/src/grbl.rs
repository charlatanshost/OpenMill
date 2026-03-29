//! GRBL-compatible post-processor.
//!
//! Simpler than LinuxCNC: no program number, no `%` delimiters.
//! Uses G93 inverse-time mode for 5-axis cutting moves.

use anyhow::Result;
use openmill_core::{MachineConfig, MoveType, TableTable, Tool, Toolpath, ToolpathPoint};

use crate::feed_rate::{compute_inverse_time_feed_with_kin, joints_for_point};
use crate::traits::{PostConfig, PostProcessor, Units};

/// GRBL post-processor.
pub struct GrblPost;

impl PostProcessor for GrblPost {
    fn header(&self, config: &PostConfig) -> String {
        let unit_code = match config.units {
            Units::Metric => "G21",
            Units::Imperial => "G20",
        };
        format!("{unit_code}\nG90\n")
    }

    fn process_toolpath(
        &self,
        toolpath: &Toolpath,
        machine: &MachineConfig,
    ) -> Result<String> {
        let tt = TableTable::new(machine.clone())?;
        let mut out = String::new();

        out.push_str(&format!("({})\n", toolpath.name));
        out.push_str("G93\n");

        let mut prev: Option<&ToolpathPoint> = None;

        for pt in &toolpath.points {
            let [x, y, z, a, c] = joints_for_point(&tt, pt)?;

            match pt.move_type {
                MoveType::Rapid | MoveType::Retract => {
                    out.push_str(&format!(
                        "G0 X{x:.4} Y{y:.4} Z{z:.4} A{a:.4} C{c:.4}\n"
                    ));
                }
                MoveType::Linear | MoveType::LeadIn | MoveType::LeadOut => {
                    let f = match prev {
                        Some(from) => compute_inverse_time_feed_with_kin(&tt, from, pt),
                        None => pt.feed_rate.max(1.0),
                    };
                    out.push_str(&format!(
                        "G1 X{x:.4} Y{y:.4} Z{z:.4} A{a:.4} C{c:.4} F{f:.4}\n"
                    ));
                }
            }

            prev = Some(pt);
        }

        out.push_str("G94\n");
        Ok(out)
    }

    fn tool_change(&self, tool: &Tool) -> String {
        format!("M5\nM0 (Tool change: T{} {})\n", tool.id, tool.name)
    }

    fn footer(&self) -> String {
        "M5\nM30\n".to_string()
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

        let gcode = post.process_toolpath(&tp, &machine).unwrap();

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

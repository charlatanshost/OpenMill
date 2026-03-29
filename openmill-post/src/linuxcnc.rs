//! LinuxCNC / RS274NGC post-processor with G93 inverse-time feed for 5-axis.

use anyhow::Result;
use openmill_core::{MachineConfig, MoveType, TableTable, Tool, Toolpath, ToolpathPoint};

use crate::feed_rate::{compute_inverse_time_feed_with_kin, joints_for_point};
use crate::traits::{PostConfig, PostProcessor, Units};

/// LinuxCNC post-processor.
///
/// All cutting moves use G93 inverse-time feed. Rapids use G0.
pub struct LinuxCncPost;

impl PostProcessor for LinuxCncPost {
    fn header(&self, config: &PostConfig) -> String {
        let unit_code = match config.units {
            Units::Metric => "G21",
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
        format!("M6 T{}\nM3 S10000\n", tool.id)
    }

    fn footer(&self) -> String {
        "M5\nM30\n%\n".to_string()
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

        let gcode = post.process_toolpath(&tp, &machine).unwrap();

        let g0_count = gcode.matches("G0 ").count();
        let g1_count = gcode.matches("G1 ").count();
        assert_eq!(g0_count, 2, "expected 2 G0 moves (rapid + retract)");
        assert_eq!(g1_count, 1, "expected 1 G1 move");
        assert!(gcode.contains("G93"), "should enter inverse-time mode");
        assert!(gcode.contains("G94"), "should restore feed-per-minute");
        // Every G1 line must carry an F word.
        for line in gcode.lines() {
            if line.starts_with("G1 ") {
                assert!(line.contains('F'), "G1 line missing F word: {line}");
            }
        }
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
        assert!(tc.contains("M3"), "should start spindle");
    }
}

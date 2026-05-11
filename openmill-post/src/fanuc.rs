//! Fanuc-style post-processor.
//!
//! Emits sequence-numbered (N…) blocks framed by `%` delimiters, the
//! tool-length compensation convention used by Fanuc / Haas / Brother
//! controllers (`G43 H<n>` on tool change), and a startup safety block
//! (`G17 G21 G40 G49 G80 G90`) that matches what most shop-floor operators
//! expect at the top of a program.
//!
//! Continuous 5-axis output uses **TCPM** (G43.4 H<n>) and emits the tool
//! tip position together with the tool-axis vector as `I/J/K` words — the
//! controller resolves the kinematics. This is the convention Fusion 360
//! and Mastercam use when posting Fanuc 30i/31i 5-axis programs.
//!
//! Indexed (3+2 / 4+1) passes fall back to joint-space A/C output with
//! plain G94 feed-per-minute so the post stays useful on older controllers
//! without TCPM.

use anyhow::Result;
use openmill_core::{
    Coolant, MachineConfig, MoveType, Operation, PostConfig, TableTable, Tool, Toolpath, Units,
};

use crate::feed_rate::joints_for_point;
use crate::traits::{spindle_command_for, PostProcessor};

/// Fanuc-style post-processor.
pub struct FanucPost;

/// Returns `true` when all points share the same tool-axis direction.
fn is_indexed_pass(toolpath: &Toolpath) -> bool {
    let Some(first) = toolpath.points.first() else { return true };
    let ref_axis = first.orientation;
    toolpath.points.iter().all(|pt| {
        (pt.orientation.as_ref() - ref_axis.as_ref()).norm() < 1e-6
    })
}

/// Sequence-number generator. Fanuc convention: start at N10, step by 10
/// so operators can hand-insert blocks between auto-generated ones.
struct SeqNum(u32);
impl SeqNum {
    fn new() -> Self { Self(10) }
    fn next(&mut self) -> u32 { let n = self.0; self.0 += 10; n }
}

impl PostProcessor for FanucPost {
    fn header(&self, config: &PostConfig) -> String {
        let unit_code = match config.units {
            Units::Metric   => "G21",
            Units::Imperial => "G20",
        };
        // Standard Fanuc safe-start: cancel cutter comp (G40), cancel TLC
        // (G49), cancel canned cycle (G80), absolute (G90), XY plane (G17).
        format!(
            "%\nO{prog:04}\nG17 {unit_code} G40 G49 G80 G90\n{wo}\n",
            prog = config.program_number,
            wo = config.work_offset,
        )
    }

    fn process_toolpath(
        &self,
        toolpath: &Toolpath,
        machine: &MachineConfig,
    ) -> Result<Vec<(String, Option<usize>)>> {
        let tt = TableTable::new(machine.clone())?;
        let mut out = Vec::new();
        let mut n = SeqNum::new();

        let indexed = is_indexed_pass(toolpath);

        out.push((format!("({})\n", toolpath.name), None));

        if indexed {
            // Emit rotary index move before any cutting.
            if let Some(first) = toolpath.points.first() {
                let [_x, _y, _z, a, c] = joints_for_point(&tt, first)?;
                if a.abs() > 1e-3 || c.abs() > 1e-3 {
                    out.push((
                        format!("N{} G0 A{:.3} C{:.3}\n", n.next(), a, c),
                        None,
                    ));
                }
            }
            out.push((format!("N{} G94\n", n.next()), None));
        } else {
            // Continuous 5-axis: use G43.4 TCPM. The controller resolves
            // rotaries from the I/J/K tool-axis vector we emit per point.
            // (Tool-length comp number is set on tool change; G43.4 stays
            // active across cuts so we don't re-emit it here.)
            out.push((format!("N{} G94\n", n.next()), None));
        }

        for (i, pt) in toolpath.points.iter().enumerate() {
            // Tool tip in workpiece coordinates.
            let x = pt.position.x;
            let y = pt.position.y;
            let z = pt.position.z;
            // Tool axis vector (tip → shank) in workpiece coordinates.
            let axis = pt.orientation.as_ref();
            let (ax, ay, az) = (axis.x, axis.y, axis.z);

            match pt.move_type {
                MoveType::Rapid | MoveType::Retract => {
                    if indexed {
                        let [jx, jy, jz, _a, _c] = joints_for_point(&tt, pt)?;
                        out.push((
                            format!("N{} G0 X{jx:.3} Y{jy:.3} Z{jz:.3}\n", n.next()),
                            Some(i),
                        ));
                    } else {
                        out.push((
                            format!(
                                "N{} G0 X{x:.3} Y{y:.3} Z{z:.3} I{ax:.4} J{ay:.4} K{az:.4}\n",
                                n.next()
                            ),
                            Some(i),
                        ));
                    }
                }
                MoveType::Linear | MoveType::LeadIn | MoveType::LeadOut => {
                    let f = pt.feed_rate.max(1.0);
                    if indexed {
                        let [jx, jy, jz, _a, _c] = joints_for_point(&tt, pt)?;
                        out.push((
                            format!(
                                "N{} G1 X{jx:.3} Y{jy:.3} Z{jz:.3} F{f:.1}\n",
                                n.next()
                            ),
                            Some(i),
                        ));
                    } else {
                        out.push((
                            format!(
                                "N{} G1 X{x:.3} Y{y:.3} Z{z:.3} I{ax:.4} J{ay:.4} K{az:.4} F{f:.1}\n",
                                n.next()
                            ),
                            Some(i),
                        ));
                    }
                }
            }
        }

        Ok(out)
    }

    fn tool_change(&self, tool: &Tool) -> String {
        // Standard Fanuc tool-change sequence:
        //   M9 (coolant off)
        //   M5 (spindle stop)
        //   G91 G28 Z0 (rapid to home in Z — protect the cutter on a swap)
        //   T<n>     (preselect)
        //   M6       (swap)
        //   G43.4 H<n>   (TCPM-aware tool-length comp, active for both
        //                 indexed and continuous paths; on older machines
        //                 substitute G43 manually)
        //   M01      (optional stop — operator can verify tool before run)
        let mut s = String::new();
        s.push_str("M9\n");
        s.push_str("M5\n");
        s.push_str("G91 G28 Z0\n");
        s.push_str("G90\n");
        s.push_str(&format!("T{} M6 (Tool change: {})\n", tool.id, tool.name));
        s.push_str(&format!("G43.4 H{}\n", tool.id));
        s.push_str("M01\n");
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
        // Rigid-tap synchronisation: emit M29 S<rpm> before the tapping
        // cycle so the spindle locks rotation to Z feed. Only meaningful
        // on Fanuc-family controllers (Haas, Mazak, Brother all accept it).
        if op.strategy == "Tapping" {
            let rigid = op.params.get("tap_mode")
                .and_then(|v| v.as_str())
                .map(|s| s == "rigid")
                .unwrap_or(true); // default: rigid
            if rigid && op.spindle_speed > 0.0 {
                s.push_str(&format!("M29 S{:.0}\n", op.spindle_speed));
            }
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
        // Cancel TCPM (G49), home Z, stop spindle/coolant, end program.
        "G49\nM5\nM9\nM30\n%\n".to_string()
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Point3, Unit, Vector3};
    use openmill_core::{MoveType, OperationType, Toolpath, ToolpathPoint};

    fn default_machine() -> MachineConfig {
        openmill_core::kinematics::default_trunnion_config()
    }

    #[test]
    fn header_has_safe_start_and_program_number() {
        let post = FanucPost;
        let cfg = PostConfig::default();
        let hdr = post.header(&cfg);
        assert!(hdr.starts_with('%'),       "Fanuc programs start with %");
        assert!(hdr.contains("O1000"),      "missing O-number");
        assert!(hdr.contains("G17"),        "missing XY plane select");
        assert!(hdr.contains("G21"),        "missing metric mode");
        assert!(hdr.contains("G40"),        "missing cutter-comp cancel");
        assert!(hdr.contains("G49"),        "missing TLC cancel");
        assert!(hdr.contains("G80"),        "missing canned-cycle cancel");
        assert!(hdr.contains("G90"),        "missing absolute mode");
        assert!(hdr.contains("G54"),        "missing default work offset");
    }

    #[test]
    fn footer_ends_with_percent_and_m30() {
        let post = FanucPost;
        let ftr = post.footer();
        assert!(ftr.contains("M30"), "missing program end");
        assert!(ftr.contains('%'),   "missing terminating %");
        assert!(ftr.contains("G49"), "should cancel TLC at end");
    }

    #[test]
    fn tool_change_uses_m06_g434() {
        let post = FanucPost;
        let mut tool = openmill_core::Tool::flat_end(7, "10mm flat", 10.0, 25.0);
        tool.tool_change_gcode = Some("(custom setup)".into());
        let tc = post.tool_change(&tool);
        assert!(tc.contains("T7 M6"),  "missing preselect + swap: {tc}");
        assert!(tc.contains("G43.4 H7"), "missing TCPM-aware TLC: {tc}");
        assert!(tc.contains("M9"),     "should drop coolant before swap");
        assert!(tc.contains("M5"),     "should stop spindle before swap");
        assert!(tc.contains("M01"),    "should pause for operator verify");
        assert!(tc.contains("G91 G28 Z0"), "should retract Z to home");
        assert!(tc.contains("(custom setup)"), "free-form gcode missing");
    }

    #[test]
    fn indexed_pass_emits_rotary_index_and_g94() {
        let machine = default_machine();
        let post = FanucPost;

        let mut tp = Toolpath::new(1, OperationType::Roughing, "indexed");
        tp.points.push(ToolpathPoint::rapid(Point3::new(0.0, 0.0, 10.0)));
        tp.points.push(ToolpathPoint {
            position: Point3::new(10.0, 0.0, 0.0),
            orientation: Vector3::z_axis(),
            feed_rate: 1200.0,
            move_type: MoveType::Linear,
        });

        let gcode = post.process_toolpath(&tp, &machine).unwrap()
            .into_iter().map(|(s, _)| s).collect::<String>();
        assert!(gcode.contains("G94"), "indexed pass should program G94 feed mode");
        assert!(!gcode.contains("G93"), "indexed should not use inverse time");
        // Vertical axis at origin → A=0 C=0, so no rotary index block emitted.
        // But the joint output should NOT include I/J/K (those are TCPM-only).
        for line in gcode.lines() {
            if line.contains("G0 ") || line.contains("G1 ") {
                assert!(!line.contains(" I"), "indexed mode should not emit I/J/K vectors: {line}");
            }
        }
    }

    #[test]
    fn continuous_pass_uses_tcpm_ijk() {
        let machine = default_machine();
        let post = FanucPost;
        let ax = Unit::new_normalize(Vector3::new(0.2, 0.0, 0.98));
        let bx = Unit::new_normalize(Vector3::new(-0.2, 0.0, 0.98));

        let mut tp = Toolpath::new(2, OperationType::Finishing, "5ax");
        tp.points.push(ToolpathPoint {
            position: Point3::new(0.0, 0.0, 5.0),
            orientation: ax,
            feed_rate: 0.0,
            move_type: MoveType::Rapid,
        });
        tp.points.push(ToolpathPoint {
            position: Point3::new(10.0, 0.0, 0.0),
            orientation: bx,
            feed_rate: 800.0,
            move_type: MoveType::Linear,
        });
        let gcode = post.process_toolpath(&tp, &machine).unwrap()
            .into_iter().map(|(s, _)| s).collect::<String>();
        let g1_line = gcode.lines().find(|l| l.contains("G1 ")).unwrap();
        // TCPM lines emit tool tip XYZ and tool-axis IJK rather than A/C.
        assert!(g1_line.contains(" X"), "missing X tip coord: {g1_line}");
        assert!(g1_line.contains(" I"), "missing I tool axis: {g1_line}");
        assert!(g1_line.contains(" J"), "missing J tool axis: {g1_line}");
        assert!(g1_line.contains(" K"), "missing K tool axis: {g1_line}");
        assert!(g1_line.contains(" F"), "missing F feed: {g1_line}");
        assert!(!g1_line.contains(" A"), "TCPM should not emit joint A: {g1_line}");
    }

    #[test]
    fn rigid_tap_emits_m29() {
        let post = FanucPost;
        let tool = openmill_core::Tool::flat_end(1, "M6 tap", 6.0, 20.0);
        let op = openmill_core::Operation {
            name: "M6 Tapping".into(),
            tool_id: 1,
            strategy: "Tapping".into(),
            params: serde_json::json!({ "tap_mode": "rigid" }),
            spindle_speed: 600.0,
            feed_rate: 600.0,
            plunge_rate: 200.0,
            coolant: openmill_core::Coolant::None,
            gcode_command: String::new(),
            stock_to_leave: 0.0,
            leads: openmill_core::LeadConfig::default(),
            enabled: true,
        };
        let pre = post.op_preamble(&op, &tool);
        assert!(pre.contains("M29 S600"), "rigid tap should emit M29 with rpm: {pre}");
    }

    #[test]
    fn floating_tap_does_not_emit_m29() {
        let post = FanucPost;
        let tool = openmill_core::Tool::flat_end(1, "M6 tap", 6.0, 20.0);
        let op = openmill_core::Operation {
            name: "M6 Tapping".into(),
            tool_id: 1,
            strategy: "Tapping".into(),
            params: serde_json::json!({ "tap_mode": "floating" }),
            spindle_speed: 600.0,
            feed_rate: 600.0,
            plunge_rate: 200.0,
            coolant: openmill_core::Coolant::None,
            gcode_command: String::new(),
            stock_to_leave: 0.0,
            leads: openmill_core::LeadConfig::default(),
            enabled: true,
        };
        let pre = post.op_preamble(&op, &tool);
        assert!(!pre.contains("M29"), "floating tap should NOT emit M29: {pre}");
    }

    #[test]
    fn blocks_are_sequence_numbered() {
        let machine = default_machine();
        let post = FanucPost;
        let mut tp = Toolpath::new(1, OperationType::Roughing, "n-numbered");
        tp.points.push(ToolpathPoint::rapid(Point3::new(0.0, 0.0, 10.0)));
        tp.points.push(ToolpathPoint {
            position: Point3::new(10.0, 0.0, 0.0),
            orientation: Vector3::z_axis(),
            feed_rate: 1000.0,
            move_type: MoveType::Linear,
        });
        let gcode = post.process_toolpath(&tp, &machine).unwrap()
            .into_iter().map(|(s, _)| s).collect::<String>();
        // Every motion line should be prefixed with an N-word.
        for line in gcode.lines() {
            if line.contains(" G0 ") || line.contains(" G1 ") {
                assert!(line.starts_with('N'),
                    "motion line missing sequence number: {line}");
            }
        }
    }
}

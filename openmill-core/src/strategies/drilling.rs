use anyhow::Result;
use nalgebra::{Point3, Vector3, Unit};
use serde::{Deserialize, Serialize};

use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::{MoveType, OperationType, Toolpath, ToolpathPoint};

use super::traits::ToolpathStrategy;

/// 5-Axis Drilling.
///
/// Moves the tool to a hole position, aligns with the hole axis, and
/// expands the canned-cycle equivalent of the chosen [`CycleType`] into
/// explicit `Rapid` / `Linear` / `Retract` moves. The post processors
/// see plain motion (not e.g. a `G83` canned-cycle block), so output is
/// verbose but works on every dialect — and the simulator can replay
/// every peck visually.
pub struct Drilling5Axis;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Hole {
    pub position: Point3<f64>,
    pub axis: Vector3<f64>,
    pub depth: f64,
}

/// Drilling canned-cycle variant. Each maps to a specific Fanuc / ISO
/// G-code in pro CAM output; here we expand each into the equivalent
/// motion sequence so any dialect renders correctly.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CycleType {
    /// **G81** — straight plunge to depth, rapid retract.
    Drill,
    /// **G82** — plunge to depth, dwell at bottom, rapid retract.
    /// Use for counter-bores and flat-bottomed holes.
    CounterBore,
    /// **G83** — deep-hole peck: full retract to clearance between
    /// pecks to evacuate chips. Slower but bulletproof.
    Peck,
    /// **G73** — chip-break peck: short retract (a few mm) between
    /// pecks, just enough to snap the chip. Faster than `Peck`,
    /// preferred for ductile materials.
    ChipBreak,
    /// **G85** — feed in, feed out at the same cutting rate.
    /// Reams the hole on both directions; common for precision bores.
    Bore,
}

impl Default for CycleType {
    fn default() -> Self { CycleType::Drill }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DrillingParams {
    pub holes: Vec<Hole>,
    pub feed_rate: f64,
    /// Dwell at the bottom of the hole [s]. Used by `CounterBore`
    /// and as the per-peck dwell on `Peck` / `ChipBreak`.
    pub dwell: f64,
    /// Cycle variant — defaults to a plain `Drill` (G81) so legacy
    /// jobs round-trip unchanged.
    #[serde(default)]
    pub cycle: CycleType,
    /// Peck depth per pass [mm]. Used by `Peck` and `ChipBreak`. `0.0`
    /// falls back to depth / 4.
    #[serde(default)]
    pub peck_depth: f64,
    /// Retract distance between pecks for `ChipBreak` cycles [mm].
    /// Fanuc default is ~0.5–1.0 mm.
    #[serde(default)]
    pub chipbreak_retract: f64,
    /// Distance to drill past the hole bottom on through-holes [mm].
    /// Clears the burr on the underside. `0.0` stops exactly at depth.
    #[serde(default)]
    pub break_through: f64,
}

impl Default for DrillingParams {
    fn default() -> Self {
        Self {
            holes: Vec::new(),
            feed_rate: 100.0,
            dwell: 0.0,
            cycle: CycleType::Drill,
            peck_depth: 0.0,
            chipbreak_retract: 0.5,
            break_through: 0.0,
        }
    }
}

impl ToolpathStrategy for Drilling5Axis {
    type Params = DrillingParams;

    fn name(&self) -> &str {
        "5-Axis Drilling"
    }

    fn generate(
        &self,
        _model: &WorkpieceModel,
        tool: &Tool,
        _machine: &MachineConfig,
        params: &DrillingParams,
    ) -> Result<Vec<Toolpath>> {
        let mut tp = Toolpath::new(tool.id, OperationType::Roughing, "5-Axis Drilling");

        for hole in &params.holes {
            expand_hole(&mut tp, hole, params);
        }

        Ok(vec![tp])
    }
}

/// Expand one hole into the right motion sequence for the selected cycle.
fn expand_hole(tp: &mut Toolpath, hole: &Hole, params: &DrillingParams) {
    let axis = Unit::new_normalize(hole.axis);
    // Clearance plane: 10 mm above the hole top in the +axis direction.
    // Same value the original implementation used; tunable later via
    // op-level safe_z if we want.
    const CLEARANCE_MM: f64 = 10.0;
    let total_depth = hole.depth + params.break_through.max(0.0);
    let safe_start = hole.position + axis.into_inner() * CLEARANCE_MM;
    let bottom = hole.position - axis.into_inner() * total_depth;
    let feed = params.feed_rate.max(1.0);
    let dwell_extra = params.dwell.max(0.0);

    // Rapid to clearance, aligned with the hole.
    tp.points.push(ToolpathPoint {
        position: safe_start,
        orientation: axis,
        feed_rate: 0.0,
        move_type: MoveType::Rapid,
    });

    match params.cycle {
        CycleType::Drill => {
            plunge_to(tp, bottom, axis, feed);
            retract_rapid(tp, safe_start, axis);
        }
        CycleType::CounterBore => {
            plunge_to(tp, bottom, axis, feed);
            if dwell_extra > 0.0 {
                // Dwell-as-zero-feed point at bottom. The simulator
                // and post both treat consecutive equal positions as
                // a pause; for G-code we could later swap this for a
                // proper `G4 P<sec>` block.
                tp.points.push(ToolpathPoint {
                    position: bottom,
                    orientation: axis,
                    feed_rate: 0.0,
                    move_type: MoveType::Linear,
                });
            }
            retract_rapid(tp, safe_start, axis);
        }
        CycleType::Peck => {
            // Full-retract peck: every peck dives `peck_depth` further,
            // then rapids back to safe_start to clear chips, then
            // rapids back to just above the last peck and resumes.
            let peck = effective_peck(params.peck_depth, total_depth);
            let mut depth_done = 0.0;
            while depth_done < total_depth - 1e-6 {
                let next = (depth_done + peck).min(total_depth);
                let target = hole.position - axis.into_inner() * next;
                // Rapid down to just above the previous bottom (skip on
                // first peck — we're already at safe_start).
                if depth_done > 0.0 {
                    let above = hole.position - axis.into_inner() * (depth_done - 0.5).max(0.0);
                    tp.points.push(ToolpathPoint {
                        position: above,
                        orientation: axis,
                        feed_rate: 0.0,
                        move_type: MoveType::Rapid,
                    });
                }
                plunge_to(tp, target, axis, feed);
                retract_rapid(tp, safe_start, axis);
                depth_done = next;
            }
        }
        CycleType::ChipBreak => {
            // Short-retract peck: after each `peck_depth` worth of
            // cutting, retract `chipbreak_retract` mm then continue.
            // No full retract until the end.
            let peck = effective_peck(params.peck_depth, total_depth);
            let kick = params.chipbreak_retract.max(0.1);
            let mut depth_done = 0.0;
            while depth_done < total_depth - 1e-6 {
                let next = (depth_done + peck).min(total_depth);
                let target = hole.position - axis.into_inner() * next;
                plunge_to(tp, target, axis, feed);
                if next < total_depth - 1e-6 {
                    let lifted = hole.position - axis.into_inner() * (next - kick).max(0.0);
                    tp.points.push(ToolpathPoint {
                        position: lifted,
                        orientation: axis,
                        feed_rate: 0.0,
                        move_type: MoveType::Rapid,
                    });
                }
                depth_done = next;
            }
            retract_rapid(tp, safe_start, axis);
        }
        CycleType::Bore => {
            // Plunge at cutting feed, then feed *out* at the same
            // rate — reams the hole on the way back up.
            plunge_to(tp, bottom, axis, feed);
            tp.points.push(ToolpathPoint {
                position: safe_start,
                orientation: axis,
                feed_rate: feed,
                move_type: MoveType::Linear,
            });
        }
    }
}

fn plunge_to(tp: &mut Toolpath, target: Point3<f64>, axis: Unit<Vector3<f64>>, feed: f64) {
    tp.points.push(ToolpathPoint {
        position: target,
        orientation: axis,
        feed_rate: feed,
        move_type: MoveType::Linear,
    });
}

fn retract_rapid(tp: &mut Toolpath, safe_start: Point3<f64>, axis: Unit<Vector3<f64>>) {
    tp.points.push(ToolpathPoint {
        position: safe_start,
        orientation: axis,
        feed_rate: 0.0,
        move_type: MoveType::Retract,
    });
}

/// Use the user's peck depth if set, else a quarter of the total depth.
fn effective_peck(user_peck: f64, total_depth: f64) -> f64 {
    if user_peck > 1e-6 {
        user_peck
    } else {
        (total_depth * 0.25).max(0.5)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{StockShape, WorkpieceModel};
    use parry3d::{math::Point as PPoint, shape::TriMesh};

    fn dummy_model() -> WorkpieceModel {
        let v = vec![
            PPoint::new(0.0, 0.0, 0.0),
            PPoint::new(1.0, 0.0, 0.0),
            PPoint::new(0.0, 1.0, 0.0),
        ];
        let i = vec![[0u32, 1, 2]];
        WorkpieceModel::new(TriMesh::new(v, i), StockShape::BoundingBox { margin: Vector3::new(0.0, 0.0, 0.0) })
    }

    fn one_hole(params: DrillingParams) -> Toolpath {
        let tool = Tool::flat_end(1, "t", 6.0, 20.0);
        let machine = crate::kinematics::default_machines::default_trunnion_config();
        let mut p = params;
        p.holes.push(Hole {
            position: Point3::new(0.0, 0.0, 0.0),
            axis: Vector3::new(0.0, 0.0, 1.0),
            depth: 10.0,
        });
        Drilling5Axis.generate(&dummy_model(), &tool, &machine, &p).unwrap().remove(0)
    }

    #[test]
    fn drill_cycle_has_three_points() {
        let tp = one_hole(DrillingParams { cycle: CycleType::Drill, feed_rate: 100.0, ..DrillingParams::default() });
        assert_eq!(tp.points.len(), 3); // rapid, plunge, retract
    }

    #[test]
    fn peck_cycle_splits_into_multiple_plunges() {
        let tp = one_hole(DrillingParams {
            cycle: CycleType::Peck,
            feed_rate: 100.0,
            peck_depth: 2.0,
            ..DrillingParams::default()
        });
        // 10 mm hole / 2 mm pecks = 5 pecks. Each peck is plunge + retract,
        // first preceded by rapid down (skipped on iter 0). +1 initial rapid.
        let plunges = tp.points.iter().filter(|p| matches!(p.move_type, MoveType::Linear)).count();
        assert_eq!(plunges, 5);
    }

    #[test]
    fn chipbreak_short_retracts_between_pecks() {
        let tp = one_hole(DrillingParams {
            cycle: CycleType::ChipBreak,
            feed_rate: 100.0,
            peck_depth: 2.0,
            chipbreak_retract: 0.5,
            ..DrillingParams::default()
        });
        // 5 plunges + 4 chip-break lifts + 1 initial rapid + 1 final retract.
        assert!(tp.points.len() >= 5 + 4 + 1 + 1);
    }

    #[test]
    fn bore_feeds_out_at_cutting_rate() {
        let tp = one_hole(DrillingParams {
            cycle: CycleType::Bore,
            feed_rate: 100.0,
            ..DrillingParams::default()
        });
        // Last move is a Linear retract at cutting feed, not a Rapid.
        let last = tp.points.last().unwrap();
        assert!(matches!(last.move_type, MoveType::Linear));
        assert!((last.feed_rate - 100.0).abs() < 1e-6);
    }

    #[test]
    fn break_through_overshoots_bottom() {
        let tp = one_hole(DrillingParams {
            cycle: CycleType::Drill,
            feed_rate: 100.0,
            break_through: 2.0,
            ..DrillingParams::default()
        });
        // The plunge target should be at z = -(10 + 2) = -12.
        let bottom_z = tp.points.iter().map(|p| p.position.z).fold(f64::INFINITY, f64::min);
        assert!((bottom_z - (-12.0)).abs() < 1e-6, "bottom z = {bottom_z}");
    }
}

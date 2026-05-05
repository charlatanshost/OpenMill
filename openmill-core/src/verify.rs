//! Toolpath verification.
//!
//! Pre-export sanity checks against a job. Each check is conservative: a
//! `Warning` flags something probably-OK but worth a look; an `Error` means
//! the post-processor would emit G-code that crashes the machine or behaves
//! undefined.
//!
//! Catches the most common ways to lose a tool:
//! - point outside the machine's linear travel limits
//! - tool axis reachable only with an out-of-range A or C joint
//! - a `MoveType::Linear` (cutting) point with `feed_rate <= 0`
//! - tool-tip path dives **below** the part's bottom (Z < part bottom)
//! - holder collision against the part mesh at a sampled pose
//!
//! The verifier is read-only — it never modifies the job — so the UI can
//! safely call it before every export and surface results in the log.

use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use parry3d::shape::TriMesh;

use crate::kinematics::{InverseKinematics, MachineConfig, TableTable};
use crate::model::WorkpieceModel;
use crate::tool::{holder_collision, Tool};
use crate::toolpath::{MoveType, Toolpath};

/// Severity of a verification finding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IssueLevel {
    Warning,
    Error,
}

/// A single verification finding.
#[derive(Debug, Clone)]
pub struct Issue {
    pub level: IssueLevel,
    /// Free-form human-readable description.
    pub message: String,
    /// Index of the toolpath in the input slice that triggered the issue,
    /// or `None` for job-wide issues.
    pub toolpath_idx: Option<usize>,
    /// Index of the offending point within that toolpath, when relevant.
    pub point_idx: Option<usize>,
}

impl Issue {
    fn err(msg: impl Into<String>) -> Self {
        Self { level: IssueLevel::Error,   message: msg.into(), toolpath_idx: None, point_idx: None }
    }
    fn warn(msg: impl Into<String>) -> Self {
        Self { level: IssueLevel::Warning, message: msg.into(), toolpath_idx: None, point_idx: None }
    }
    fn at(mut self, tp: usize, pt: usize) -> Self {
        self.toolpath_idx = Some(tp);
        self.point_idx = Some(pt);
        self
    }
}

/// Run every available check and return the combined issue list.
///
/// `model` may be `None` — the gouge / holder checks then skip.
pub fn verify_job(
    toolpaths: &[Toolpath],
    machine: &MachineConfig,
    tools: &[Tool],
    model: Option<&WorkpieceModel>,
) -> Vec<Issue> {
    let mut issues = Vec::new();
    if toolpaths.is_empty() {
        issues.push(Issue::warn("No toolpaths to verify."));
        return issues;
    }

    // IK is built once; failure here disables the rotary-limit check rather
    // than aborting the whole verification.
    let tt = TableTable::new(machine.clone()).ok();

    for (tp_idx, tp) in toolpaths.iter().enumerate() {
        check_axis_limits(tp, tp_idx, machine, tt.as_ref(), &mut issues);
        check_feed_rates(tp, tp_idx, &mut issues);
        if let Some(m) = model {
            check_below_part(tp, tp_idx, m, &mut issues);
            check_holder_collisions(tp, tp_idx, tools, m, &mut issues);
        }
    }

    issues
}

fn check_axis_limits(
    tp: &Toolpath,
    tp_idx: usize,
    machine: &MachineConfig,
    tt: Option<&TableTable>,
    issues: &mut Vec<Issue>,
) {
    let lim = &machine.travel_limits;
    for (i, p) in tp.points.iter().enumerate() {
        if p.position.x < lim.x.0 || p.position.x > lim.x.1 {
            issues.push(Issue::err(format!(
                "X={:.3} outside travel [{:.1}, {:.1}] in \"{}\" point #{}",
                p.position.x, lim.x.0, lim.x.1, tp.name, i,
            )).at(tp_idx, i));
        }
        if p.position.y < lim.y.0 || p.position.y > lim.y.1 {
            issues.push(Issue::err(format!(
                "Y={:.3} outside travel [{:.1}, {:.1}] in \"{}\" point #{}",
                p.position.y, lim.y.0, lim.y.1, tp.name, i,
            )).at(tp_idx, i));
        }
        if p.position.z < lim.z.0 || p.position.z > lim.z.1 {
            issues.push(Issue::err(format!(
                "Z={:.3} outside travel [{:.1}, {:.1}] in \"{}\" point #{}",
                p.position.z, lim.z.0, lim.z.1, tp.name, i,
            )).at(tp_idx, i));
        }
        // Rotary limits only checked when IK constructs successfully and the
        // tool axis isn't near-vertical (the post-processor skips IK for
        // vertical and emits A=0, C=0 itself).
        if let Some(tt) = tt {
            if p.orientation.z.abs() < 1.0 - 1e-6 {
                if let Ok(solutions) = tt.ik(&p.position, &p.orientation) {
                    let [_, _, _, a, c] = solutions[0];
                    let a_rad = a.to_radians();
                    let c_rad = c.to_radians();
                    let crate::kinematics::KinematicType::TableTable { a_axis, c_axis } = &machine.axes;
                    if a_rad < a_axis.min_angle || a_rad > a_axis.max_angle {
                        issues.push(Issue::err(format!(
                            "A={:.2}° outside [{:.1}, {:.1}] in \"{}\" point #{}",
                            a, a_axis.min_angle.to_degrees(), a_axis.max_angle.to_degrees(),
                            tp.name, i,
                        )).at(tp_idx, i));
                    }
                    if c_rad < c_axis.min_angle || c_rad > c_axis.max_angle {
                        issues.push(Issue::err(format!(
                            "C={:.2}° outside [{:.1}, {:.1}] in \"{}\" point #{}",
                            c, c_axis.min_angle.to_degrees(), c_axis.max_angle.to_degrees(),
                            tp.name, i,
                        )).at(tp_idx, i));
                    }
                }
            }
        }
    }
}

fn check_feed_rates(tp: &Toolpath, tp_idx: usize, issues: &mut Vec<Issue>) {
    for (i, p) in tp.points.iter().enumerate() {
        let needs_feed = matches!(p.move_type, MoveType::Linear | MoveType::LeadIn | MoveType::LeadOut);
        if needs_feed && p.feed_rate <= 0.0 {
            issues.push(Issue::err(format!(
                "Cutting move with feed = 0 in \"{}\" point #{}",
                tp.name, i,
            )).at(tp_idx, i));
        }
        if needs_feed && p.feed_rate > 100_000.0 {
            issues.push(Issue::warn(format!(
                "Implausibly high feed = {:.0} mm/min in \"{}\" point #{}",
                p.feed_rate, tp.name, i,
            )).at(tp_idx, i));
        }
    }
}

fn check_below_part(tp: &Toolpath, tp_idx: usize, model: &WorkpieceModel, issues: &mut Vec<Issue>) {
    // Only a coarse test: if a *cutting* point drops below the model AABB's
    // bottom, the tool is plunging through the part. Doesn't catch
    // tool-radius gouging on walls — that's a much heavier swept-volume
    // analysis we can add later.
    let z_floor = model.aabb.mins.z as f64;
    for (i, p) in tp.points.iter().enumerate() {
        if matches!(p.move_type, MoveType::Linear | MoveType::LeadIn) {
            // 0.05 mm fudge so floating-point matches don't trip the check.
            if p.position.z < z_floor - 0.05 {
                issues.push(Issue::warn(format!(
                    "Cut at Z={:.3} is below part floor Z={:.3} in \"{}\" point #{}",
                    p.position.z, z_floor, tp.name, i,
                )).at(tp_idx, i));
                // One per toolpath is enough — don't spam the log.
                break;
            }
        }
    }
}

fn check_holder_collisions(
    tp: &Toolpath,
    tp_idx: usize,
    tools: &[Tool],
    model: &WorkpieceModel,
    issues: &mut Vec<Issue>,
) {
    let Some(tool) = tools.iter().find(|t| t.id == tp.tool_id) else { return };
    if tool.holder.is_none() { return; }
    // Sample every Nth point so we don't pay O(verts × points) — the holder
    // typically swept-collides over many adjacent points so a stride still
    // catches it.
    let stride = (tp.points.len() / 64).max(1);
    let mut reported = 0usize;
    for (i, p) in tp.points.iter().enumerate().step_by(stride) {
        let translation = Translation3::new(p.position.x as f32, p.position.y as f32, p.position.z as f32);
        let rotation = UnitQuaternion::rotation_between(
            &Vector3::z(),
            &Vector3::new(p.orientation.x as f32, p.orientation.y as f32, p.orientation.z as f32),
        ).unwrap_or(UnitQuaternion::identity());
        let pose: Isometry3<f32> = Isometry3::from_parts(translation.into(), rotation);
        if holder_collision(tool, &pose, &model.mesh).is_some() {
            issues.push(Issue::err(format!(
                "Tool holder hits the part at \"{}\" point #{}",
                tp.name, i,
            )).at(tp_idx, i));
            reported += 1;
            if reported >= 5 { break; } // limit log spam
        }
    }
}

/// Quick bool helper: any errors in the issue list?
pub fn has_errors(issues: &[Issue]) -> bool {
    issues.iter().any(|i| i.level == IssueLevel::Error)
}

// Type alias so `tt.ik(&pos, &axis)` doesn't require importing the trait at
// every call site for tests.
#[allow(dead_code)]
type _IkBound = dyn Fn(&TriMesh) -> ();

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kinematics::default_machines::default_trunnion_config;
    use crate::strategies::{ThreePlusTwo, ThreePlusTwoParams, ToolpathStrategy};
    use crate::tool::Tool;
    use crate::model::{StockShape, WorkpieceModel};
    use parry3d::math::Point as PPoint;

    fn unit_cube_model() -> WorkpieceModel {
        let v = vec![
            PPoint::<f32>::new(-10.0, -10.0, 0.0), PPoint::<f32>::new(10.0, -10.0, 0.0),
            PPoint::<f32>::new(10.0,  10.0, 0.0), PPoint::<f32>::new(-10.0,  10.0, 0.0),
            PPoint::<f32>::new(-10.0, -10.0, 5.0), PPoint::<f32>::new(10.0, -10.0, 5.0),
            PPoint::<f32>::new(10.0,  10.0, 5.0), PPoint::<f32>::new(-10.0,  10.0, 5.0),
        ];
        let t = vec![
            [0u32, 1, 2], [0, 2, 3],
            [4, 6, 5],    [4, 7, 6],
            [0, 1, 5],    [0, 5, 4],
            [1, 2, 6],    [1, 6, 5],
            [2, 3, 7],    [2, 7, 6],
            [3, 0, 4],    [3, 4, 7],
        ];
        WorkpieceModel::new(
            TriMesh::new(v, t),
            StockShape::BoundingBox { margin: nalgebra::Vector3::new(2.0, 2.0, 2.0) },
        )
    }

    #[test]
    fn flags_zero_feed_on_a_cutting_move() {
        let mut tp = Toolpath::new(1, crate::OperationType::Roughing, "bad");
        tp.points.push(crate::ToolpathPoint::rapid(nalgebra::Point3::new(0.0, 0.0, 5.0)));
        tp.points.push(crate::ToolpathPoint {
            position: nalgebra::Point3::new(10.0, 0.0, 5.0),
            orientation: Vector3::z_axis(),
            feed_rate: 0.0, // ← bad
            move_type: MoveType::Linear,
        });
        let issues = verify_job(&[tp], &default_trunnion_config(), &[], None);
        assert!(issues.iter().any(|i| i.level == IssueLevel::Error && i.message.contains("feed = 0")));
    }

    #[test]
    fn flags_x_outside_travel_limits() {
        let mut tp = Toolpath::new(1, crate::OperationType::Roughing, "out");
        tp.points.push(crate::ToolpathPoint::rapid(nalgebra::Point3::new(99999.0, 0.0, 5.0)));
        let issues = verify_job(&[tp], &default_trunnion_config(), &[], None);
        assert!(issues.iter().any(|i| i.level == IssueLevel::Error && i.message.contains("X=")));
    }

    #[test]
    fn clean_3plus2_toolpath_has_no_errors() {
        let model = unit_cube_model();
        let tool = Tool::flat_end(1, "6mm", 6.0, 20.0);
        // The default trunnion's Z travel is [-150, 0] (spindle nose at Z=0,
        // table descends). Our test toolpath lives in workpiece coords with
        // positive Z above the part — widen the limits so the limit check
        // measures what we actually care about: no other error categories.
        let mut machine = default_trunnion_config();
        machine.travel_limits = crate::AxisLimits {
            x: (-500.0, 500.0),
            y: (-500.0, 500.0),
            z: (-500.0, 500.0),
        };
        let params = ThreePlusTwoParams::default();
        let paths = ThreePlusTwo.generate(&model, &tool, &machine, &params).unwrap();
        let issues = verify_job(&paths, &machine, std::slice::from_ref(&tool), Some(&model));
        // Allow warnings (cuts can dip below the part floor as the cleared
        // pocket nears stock_min.z), but no errors should fire.
        assert!(
            !has_errors(&issues),
            "clean toolpath should have no errors, got: {:#?}",
            issues.iter().filter(|i| i.level == IssueLevel::Error).collect::<Vec<_>>(),
        );
    }
}

//! Lead-in / lead-out post-pass.
//!
//! Strategies emit toolpaths with simple vertical plunges at cut entries —
//! the tool descends straight down through the stock from clearance Z to
//! the cut surface. That's mechanically fine on rigid machines but causes
//! tool deflection, dwell marks, and shortened tool life on softer
//! setups. This module rewrites those entries (and the matching exits)
//! into ramped or arc'd approaches.
//!
//! Scope (intentionally narrow for v1):
//! - Only modifies 3-axis transitions where the tool axis is approximately
//!   vertical (within 5° of +Z) on both sides of the transition. 5-axis
//!   paths pass through untouched so we don't second-guess the strategy.
//! - Targets the `LeadIn`/`LeadOut` move types if the strategy emitted
//!   them, otherwise the first/last `Linear` segment of each contiguous
//!   cutting block.
//! - User opts in per operation via [`LeadConfig::kind`]; the default
//!   `LeadKind::None` is a no-op so existing behaviour is preserved.

use nalgebra::{Point3, Vector3};
use serde::{Deserialize, Serialize};

use crate::toolpath::{MoveType, Toolpath, ToolpathPoint};

/// Shape of the lead motion. See [`LeadConfig`] for the per-op container.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LeadKind {
    /// No lead — strategies' default vertical plunge is kept.
    None,
    /// Diagonal ramp into the cut at `angle_deg` over `length_mm`.
    Ramp,
    /// Quarter-circle arc tangent to the cut direction with radius `length_mm`.
    Arc,
}

impl Default for LeadKind {
    fn default() -> Self { LeadKind::None }
}

/// Per-operation lead-in / lead-out configuration.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LeadConfig {
    pub kind: LeadKind,
    /// Lead length [mm]. For `Arc` this is the radius; for `Ramp` it's
    /// the horizontal run of the approach.
    pub length_mm: f64,
    /// Ramp angle measured from horizontal [degrees]. Ignored for `Arc`.
    /// Typical values: 3–15° for soft materials, 1–5° for steel.
    pub angle_deg: f64,
    /// Number of interpolated points along the arc. Ignored for `Ramp`.
    /// Higher values produce smoother arcs at the cost of more G-code.
    pub arc_steps: usize,
}

impl Default for LeadConfig {
    fn default() -> Self {
        LeadConfig {
            kind: LeadKind::None,
            length_mm: 2.0,
            angle_deg: 5.0,
            arc_steps: 8,
        }
    }
}

impl LeadConfig {
    pub fn is_active(&self) -> bool {
        self.kind != LeadKind::None && self.length_mm > 1e-6
    }
}

/// `axis.z` must be at least this for a tool axis to count as "essentially
/// vertical". `cos(5°) ≈ 0.9962`, so tilts beyond ~5° from +Z opt out of
/// the lead post-pass (preserves whatever entry the strategy designed).
const VERTICAL_Z_MIN: f64 = 0.9962;

fn is_vertical(axis: &Vector3<f64>) -> bool {
    axis.z >= VERTICAL_Z_MIN
}

/// Apply leads to one toolpath in place. Walks the points and looks for
/// rapid → cut and cut → rapid boundaries; inserts intermediate points for
/// each lead. 5-axis or already-leaded paths are skipped.
pub fn apply_leads(tp: &mut Toolpath, lead: &LeadConfig) {
    if !lead.is_active() || tp.points.len() < 2 {
        return;
    }

    // Walk back-to-front so each splice doesn't shift the indices of
    // earlier transitions we still need to visit. Snapshot the immutable
    // values we need from the point before any mutating borrow.
    let mut i = tp.points.len();
    while i > 0 {
        i -= 1;
        let (cur_pos, cur_axis, cur_feed, cur_is_cut) = {
            let p = &tp.points[i];
            (
                p.position,
                p.orientation.into_inner(),
                p.feed_rate,
                matches!(p.move_type, MoveType::Linear | MoveType::LeadIn | MoveType::LeadOut),
            )
        };
        if !cur_is_cut { continue; }

        // ── Lead-out (cut → rapid) at the trailing edge ────────────────
        if i + 1 < tp.points.len() {
            let (next_is_rapid, next_axis) = {
                let n = &tp.points[i + 1];
                (
                    matches!(n.move_type, MoveType::Rapid | MoveType::Retract),
                    n.orientation.into_inner(),
                )
            };
            if next_is_rapid && is_vertical(&cur_axis) && is_vertical(&next_axis) {
                if let Some(prev_cut) = preceding_cut(&tp.points, i) {
                    let prev_pos = tp.points[prev_cut].position;
                    let dir = cut_direction(&prev_pos, &cur_pos);
                    let leads = build_lead_out(&cur_pos, &dir, cur_feed, lead);
                    splice_after(&mut tp.points, i, leads);
                }
            }
        }

        // ── Lead-in (rapid → cut) at the leading edge ──────────────────
        if i > 0 {
            let (prev_is_rapid, prev_axis) = {
                let p = &tp.points[i - 1];
                (
                    matches!(p.move_type, MoveType::Rapid | MoveType::Retract),
                    p.orientation.into_inner(),
                )
            };
            if prev_is_rapid && is_vertical(&cur_axis) && is_vertical(&prev_axis) {
                if let Some(next_cut) = following_cut(&tp.points, i) {
                    let next_pos = tp.points[next_cut].position;
                    let dir = cut_direction(&cur_pos, &next_pos);
                    let leads = build_lead_in(&cur_pos, &dir, cur_feed, lead);
                    splice_before(&mut tp.points, i, leads);
                }
            }
        }
    }
}

fn preceding_cut(points: &[ToolpathPoint], idx: usize) -> Option<usize> {
    (0..idx).rev().find(|&j| matches!(points[j].move_type,
        MoveType::Linear | MoveType::LeadIn | MoveType::LeadOut))
}

fn following_cut(points: &[ToolpathPoint], idx: usize) -> Option<usize> {
    (idx + 1..points.len()).find(|&j| matches!(points[j].move_type,
        MoveType::Linear | MoveType::LeadIn | MoveType::LeadOut))
}

/// 2-D horizontal cut direction (XY only). Returns the +X direction if the
/// two points are vertically aligned — better than a zero vector that would
/// collapse the lead onto the entry point.
fn cut_direction(from: &Point3<f64>, to: &Point3<f64>) -> Vector3<f64> {
    let mut d = to - from;
    d.z = 0.0;
    if d.norm_squared() < 1e-9 {
        return Vector3::new(1.0, 0.0, 0.0);
    }
    d.normalize()
}

/// Build the points of an opt-in lead-in ending exactly at `entry`. The
/// returned points should be spliced *before* the original entry so the
/// path becomes `[..rapid, lead_points.., entry, ..cut..]`.
fn build_lead_in(
    entry: &Point3<f64>,
    cut_dir: &Vector3<f64>,
    feed: f64,
    lead: &LeadConfig,
) -> Vec<ToolpathPoint> {
    match lead.kind {
        LeadKind::None => Vec::new(),
        LeadKind::Ramp => {
            let dz = lead.length_mm * lead.angle_deg.to_radians().tan();
            let start = Point3::new(
                entry.x - cut_dir.x * lead.length_mm,
                entry.y - cut_dir.y * lead.length_mm,
                entry.z + dz,
            );
            vec![ToolpathPoint {
                position: start,
                orientation: nalgebra::Vector3::z_axis(),
                feed_rate: feed.max(1.0),
                move_type: MoveType::LeadIn,
            }]
        }
        LeadKind::Arc => arc_quarter(entry, cut_dir, lead.length_mm, lead.arc_steps, feed, true),
    }
}

fn build_lead_out(
    exit: &Point3<f64>,
    cut_dir: &Vector3<f64>,
    feed: f64,
    lead: &LeadConfig,
) -> Vec<ToolpathPoint> {
    match lead.kind {
        LeadKind::None => Vec::new(),
        LeadKind::Ramp => {
            let dz = lead.length_mm * lead.angle_deg.to_radians().tan();
            let end = Point3::new(
                exit.x + cut_dir.x * lead.length_mm,
                exit.y + cut_dir.y * lead.length_mm,
                exit.z + dz,
            );
            vec![ToolpathPoint {
                position: end,
                orientation: nalgebra::Vector3::z_axis(),
                feed_rate: feed.max(1.0),
                move_type: MoveType::LeadOut,
            }]
        }
        LeadKind::Arc => arc_quarter(exit, cut_dir, lead.length_mm, lead.arc_steps, feed, false),
    }
}

/// Quarter-circle arc tangent to the cut at the entry/exit point. The arc
/// lies in the vertical plane containing `cut_dir` and +Z. When `lead_in`
/// is true the arc descends to the entry; when false it ascends from the
/// exit. Returned points include the tangent point at the cut surface
/// when `lead_in` is false, but exclude it when `lead_in` is true (the
/// existing entry point in the path is the tangent).
fn arc_quarter(
    pivot: &Point3<f64>,
    cut_dir: &Vector3<f64>,
    radius: f64,
    steps: usize,
    feed: f64,
    lead_in: bool,
) -> Vec<ToolpathPoint> {
    let steps = steps.max(2);
    // Centre of arc is `radius` directly above the pivot.
    let cx = pivot.x;
    let cy = pivot.y;
    let cz = pivot.z + radius;
    let mut out = Vec::with_capacity(steps);
    // Sweep from 0 (straight back along -cut_dir at altitude cz, level)
    // to 90° (straight down at pivot). For lead-in we want the path to
    // arrive at the pivot tangent to cut_dir, so the start of the arc is
    // back-and-up.
    for s in 0..steps {
        let t = s as f64 / (steps - 1) as f64;
        let theta = if lead_in {
            // 0 → π/2 : back-up to pivot
            t * std::f64::consts::FRAC_PI_2
        } else {
            // π/2 → 0 : reversed
            (1.0 - t) * std::f64::consts::FRAC_PI_2 + std::f64::consts::FRAC_PI_2
        };
        let dx = -cut_dir.x * radius * theta.cos();
        let dy = -cut_dir.y * radius * theta.cos();
        let dz = -radius * theta.sin();
        let pt = Point3::new(cx + dx, cy + dy, cz + dz);
        // For lead-in skip the last (tangent) point — it equals `pivot`
        // and is already in the path. For lead-out skip the first.
        let on_pivot = if lead_in { s == steps - 1 } else { s == 0 };
        if on_pivot { continue; }
        out.push(ToolpathPoint {
            position: pt,
            orientation: nalgebra::Vector3::z_axis(),
            feed_rate: feed.max(1.0),
            move_type: if lead_in { MoveType::LeadIn } else { MoveType::LeadOut },
        });
    }
    out
}

fn splice_before(points: &mut Vec<ToolpathPoint>, idx: usize, new: Vec<ToolpathPoint>) {
    if new.is_empty() { return; }
    for (k, p) in new.into_iter().enumerate() {
        points.insert(idx + k, p);
    }
}

fn splice_after(points: &mut Vec<ToolpathPoint>, idx: usize, new: Vec<ToolpathPoint>) {
    if new.is_empty() { return; }
    for (k, p) in new.into_iter().enumerate() {
        points.insert(idx + 1 + k, p);
    }
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::toolpath::{OperationType, Toolpath, ToolpathPoint};
    use nalgebra::Vector3;

    fn rapid(p: [f64; 3]) -> ToolpathPoint {
        ToolpathPoint {
            position: Point3::new(p[0], p[1], p[2]),
            orientation: Vector3::z_axis(),
            feed_rate: 0.0,
            move_type: MoveType::Rapid,
        }
    }
    fn cut(p: [f64; 3], f: f64) -> ToolpathPoint {
        ToolpathPoint {
            position: Point3::new(p[0], p[1], p[2]),
            orientation: Vector3::z_axis(),
            feed_rate: f,
            move_type: MoveType::Linear,
        }
    }

    fn simple_path() -> Toolpath {
        let mut tp = Toolpath::new(1, OperationType::Roughing, "leads-test");
        tp.points.push(rapid([0.0, 0.0, 5.0]));   // approach at clearance
        tp.points.push(cut  ([0.0, 0.0, 0.0], 800.0)); // straight plunge
        tp.points.push(cut  ([10.0, 0.0, 0.0], 800.0)); // horizontal cut
        tp.points.push(rapid([10.0, 0.0, 5.0])); // retract
        tp
    }

    #[test]
    fn lead_none_is_noop() {
        let mut tp = simple_path();
        let before = tp.points.len();
        apply_leads(&mut tp, &LeadConfig::default());
        assert_eq!(tp.points.len(), before);
    }

    #[test]
    fn ramp_lead_in_adds_one_point_back_and_up() {
        let mut tp = simple_path();
        let lead = LeadConfig {
            kind: LeadKind::Ramp,
            length_mm: 2.0,
            angle_deg: 10.0,
            arc_steps: 0,
        };
        apply_leads(&mut tp, &lead);
        // Lead-in adds one point before the cut entry, lead-out adds one
        // before the retract. So we expect +2 points total.
        assert_eq!(tp.points.len(), 6, "expected ramp lead-in + lead-out points");
        // The newly inserted lead-in point sits 2 mm back from the cut
        // entry along the cut direction (+X), and elevated.
        let lead_in_pt = &tp.points[1];
        assert!((lead_in_pt.position.x - (-2.0)).abs() < 1e-6, "x");
        assert!(lead_in_pt.position.z > 0.0, "should be above cut surface");
        assert_eq!(lead_in_pt.move_type, MoveType::LeadIn);
    }

    #[test]
    fn arc_lead_in_inserts_multiple_points() {
        let mut tp = simple_path();
        let lead = LeadConfig {
            kind: LeadKind::Arc,
            length_mm: 2.0,
            angle_deg: 0.0,
            arc_steps: 8,
        };
        apply_leads(&mut tp, &lead);
        // Arc emits (steps - 1) = 7 lead-in points + 7 lead-out points.
        assert_eq!(tp.points.len(), 4 + 14);
        // All inserted points have feed > 0 and are tagged as leads.
        for p in &tp.points {
            if matches!(p.move_type, MoveType::LeadIn | MoveType::LeadOut) {
                assert!(p.feed_rate > 0.0);
            }
        }
    }

    #[test]
    fn five_axis_path_is_untouched() {
        let tilted = nalgebra::Unit::new_normalize(Vector3::new(0.2, 0.0, 0.98));
        let mut tp = Toolpath::new(2, OperationType::Finishing, "5ax");
        tp.points.push(ToolpathPoint {
            position: Point3::new(0.0, 0.0, 5.0),
            orientation: tilted,
            feed_rate: 0.0,
            move_type: MoveType::Rapid,
        });
        tp.points.push(ToolpathPoint {
            position: Point3::new(0.0, 0.0, 0.0),
            orientation: tilted,
            feed_rate: 600.0,
            move_type: MoveType::Linear,
        });
        tp.points.push(ToolpathPoint {
            position: Point3::new(10.0, 0.0, 0.0),
            orientation: tilted,
            feed_rate: 600.0,
            move_type: MoveType::Linear,
        });
        let before = tp.points.len();
        apply_leads(&mut tp, &LeadConfig {
            kind: LeadKind::Ramp,
            length_mm: 2.0,
            angle_deg: 10.0,
            arc_steps: 0,
        });
        assert_eq!(tp.points.len(), before, "5-axis paths must pass through unchanged");
    }
}

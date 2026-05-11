//! Cross-strategy post-pass transforms.
//!
//! These mutate a generated [`Toolpath`] without knowing anything about
//! which strategy emitted it. They're applied in
//! `generate::dispatch` after the strategy returns, so adding a new
//! strategy automatically picks up every transform on this page.
//!
//! Both transforms default to no-ops (Climb direction, unbounded Z
//! range), so jobs saved before this module existed deserialize
//! unchanged.

use serde::{Deserialize, Serialize};

use crate::tool::{Tool, ToolShape};
use crate::toolpath::{MoveType, Toolpath};

// ── Thread direction (shared by Tapping + Thread Milling) ─────────────────

/// Thread hand. Right-hand threads are the world default; left-hand is
/// used on certain rotating machinery (left-side wheel studs, gas-line
/// fittings). Posts flip spindle / helix direction accordingly.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ThreadDirection {
    RightHand,
    LeftHand,
}

impl Default for ThreadDirection {
    fn default() -> Self {
        ThreadDirection::RightHand
    }
}

// ── Tap mode ──────────────────────────────────────────────────────────────

/// Rigid vs Floating tap mode. Rigid tapping requires the controller to
/// synchronise spindle rotation with Z feed; on Fanuc / Haas this is
/// enabled by emitting `M29 S<rpm>` before the tapping move. Floating
/// tappers absorb the small feed mismatch mechanically, so no M29.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TapMode {
    Rigid,
    Floating,
}

impl Default for TapMode {
    fn default() -> Self {
        TapMode::Rigid
    }
}

// ── Cut direction ──────────────────────────────────────────────────────────

/// Whether each cutting pass runs in the strategy's native direction
/// (`Climb`), is reversed (`Conventional`), or is left alone.
///
/// "Climb" is the convention used by every modern strategy here — cutter
/// rotation throws chips away from the cut, producing the best surface
/// finish and the lowest cutter deflection. Choosing `Conventional`
/// reverses each cutting pass so the cutter rotates *into* the
/// material; you might prefer it on a rigid manual machine where backlash
/// makes climb unpredictable.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CutDirection {
    /// Strategy's emitted direction — climb on every current strategy.
    Climb,
    /// Reverse every cutting pass (Linear + LeadIn/LeadOut blocks).
    Conventional,
    /// Leave the strategy's pattern alone. Equivalent to `Climb` for
    /// strategies that already emit climb passes; documented separately
    /// so future zigzag-aware strategies can interpret it differently.
    Either,
}

impl Default for CutDirection {
    fn default() -> Self {
        CutDirection::Climb
    }
}

/// Reverse the order of every contiguous "cutting block" in `tp` when
/// the direction is `Conventional`. A cutting block is a run of
/// `Linear / LeadIn / LeadOut` points; `Rapid / Retract` points act as
/// block separators and are left in place.
///
/// Reversing a block flips its travel direction without changing where
/// it enters or leaves stock, so the surrounding rapid moves still line
/// up. `Climb` / `Either` are no-ops.
pub fn apply_direction(tp: &mut Toolpath, direction: CutDirection) {
    if direction != CutDirection::Conventional || tp.points.len() < 3 {
        return;
    }

    let mut new_points = Vec::with_capacity(tp.points.len());
    let mut i = 0;
    while i < tp.points.len() {
        let is_cut = matches!(
            tp.points[i].move_type,
            MoveType::Linear | MoveType::LeadIn | MoveType::LeadOut
        );
        if !is_cut {
            new_points.push(tp.points[i].clone());
            i += 1;
            continue;
        }
        // Found the start of a cutting block — scan to its end.
        let start = i;
        while i < tp.points.len() {
            let is_cut = matches!(
                tp.points[i].move_type,
                MoveType::Linear | MoveType::LeadIn | MoveType::LeadOut
            );
            if !is_cut {
                break;
            }
            i += 1;
        }
        // [start..i) is the block. Append it reversed.
        for k in (start..i).rev() {
            new_points.push(tp.points[k].clone());
        }
    }
    tp.points = new_points;
}

// ── Z range limiter ────────────────────────────────────────────────────────

/// Optional top / bottom Z limits for a strategy. Cutting points whose
/// `position.z` falls outside `[bottom_mm, top_mm]` are skipped at
/// post-pass time.
///
/// Both bounds are optional so users can set only one side (e.g. "don't
/// cut below Z = -5 mm" without an upper limit). The bounds are
/// inclusive — points exactly on the limit are kept.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct ZRange {
    #[serde(default)]
    pub top_mm: Option<f64>,
    #[serde(default)]
    pub bottom_mm: Option<f64>,
}

impl ZRange {
    pub fn is_active(&self) -> bool {
        self.top_mm.is_some() || self.bottom_mm.is_some()
    }
    fn contains(&self, z: f64) -> bool {
        if let Some(top) = self.top_mm {
            if z > top + 1e-9 {
                return false;
            }
        }
        if let Some(bot) = self.bottom_mm {
            if z < bot - 1e-9 {
                return false;
            }
        }
        true
    }
}

/// Drop cutting points outside the supplied Z range. Rapids + retracts
/// are kept so the tool can still travel above the cut zone — the user
/// asked to limit *cutting*, not motion.
///
/// This is a simple filter rather than a clipper: a cutting segment that
/// crosses the boundary disappears entirely rather than being split at
/// the boundary. Good enough for most "cut nothing below Z=−5" use
/// cases; if a strategy generates segments that span large Z deltas you
/// may want the strategy itself to honour the range.
pub fn apply_z_range(tp: &mut Toolpath, range: &ZRange) {
    if !range.is_active() {
        return;
    }
    tp.points.retain(|p| {
        let is_cut = matches!(
            p.move_type,
            MoveType::Linear | MoveType::LeadIn | MoveType::LeadOut
        );
        if !is_cut {
            return true;
        }
        range.contains(p.position.z)
    });
}

// ── Spring pass ────────────────────────────────────────────────────────────

/// Spring (finish) pass: clone the final "pass unit" of a toolpath — the
/// suffix from the last `Rapid` through end of points — and append it at
/// a reduced feed. Used to clean up the slight cutter deflection that
/// accumulates on a heavy finish pass.
///
/// `feed_fraction` scales the cutting feed (e.g. `0.5` halves it).
/// Rapids and retracts retain `feed_rate = 0`. If there is no `Rapid`
/// move (degenerate path) the whole toolpath is duplicated at the
/// reduced feed.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct SpringPass {
    #[serde(default)]
    pub enabled: bool,
    /// Feed scale for the spring pass (0.0..=1.0).
    #[serde(default = "default_spring_feed_fraction")]
    pub feed_fraction: f64,
}

fn default_spring_feed_fraction() -> f64 {
    0.5
}

impl Default for SpringPass {
    fn default() -> Self {
        SpringPass {
            enabled: false,
            feed_fraction: 0.5,
        }
    }
}

pub fn apply_spring_pass(tp: &mut Toolpath, sp: &SpringPass) {
    if !sp.enabled || tp.points.is_empty() {
        return;
    }
    let scale = sp.feed_fraction.clamp(0.01, 1.0);
    // Find the start of the last "pass unit": index of the last Rapid.
    let start = tp.points.iter()
        .rposition(|p| matches!(p.move_type, MoveType::Rapid))
        .unwrap_or(0);
    let suffix: Vec<_> = tp.points[start..]
        .iter()
        .map(|p| {
            let mut q = p.clone();
            if matches!(q.move_type, MoveType::Linear | MoveType::LeadIn | MoveType::LeadOut) {
                q.feed_rate *= scale;
            }
            q
        })
        .collect();
    tp.points.extend(suffix);
}

// ── Step-over from cusp height ─────────────────────────────────────────────

/// Compute a step-over distance (mm) from a target cusp (scallop) height.
/// For a ball-nose or bull-nose cutter the cusp-to-step-over relationship
/// is exact:
///
/// `step_over = 2 * sqrt(2 R h - h²)`
///
/// where `R` is the bottom radius (ball radius or corner radius) and `h`
/// the desired cusp height. Result is clamped to `[0.05 mm, diameter]`.
///
/// For flat or non-radiused tools the formula collapses (R = ∞), so we
/// fall back to a sensible `0.5 × diameter` step.
pub fn step_over_from_cusp(cusp_mm: f64, tool: &Tool) -> f64 {
    let dia = tool.shape.diameter();
    let r = match tool.shape {
        ToolShape::BallEnd { diameter, .. } => diameter * 0.5,
        ToolShape::BullNose { corner_radius, .. } => corner_radius,
        _ => {
            // No tip radius — cusp height is undefined. Return a sane default.
            return (dia * 0.5).max(0.05);
        }
    };
    let h = cusp_mm.clamp(1e-6, r * 0.99); // can't exceed tip radius
    let so = 2.0 * (2.0 * r * h - h * h).max(0.0).sqrt();
    so.clamp(0.05, dia)
}

/// Convenience: pick the actual step-over (mm) for a strategy, given the
/// optional cusp override + the fraction-of-diameter fallback. Most
/// surface strategies use this so the user can pick *either* knob.
pub fn step_over_mm_for(
    cusp_mm: Option<f64>,
    fraction_of_dia: f64,
    tool: &Tool,
) -> f64 {
    match cusp_mm {
        Some(h) if h > 0.0 => step_over_from_cusp(h, tool),
        _ => (tool.shape.diameter() * fraction_of_dia).max(0.05),
    }
}

// ── Common transforms blob (used by generate::dispatch) ────────────────────

/// Shared parameters every strategy honours. Deserialised from the
/// strategy's params JSON via serde — fields the strategy doesn't know
/// about are silently ignored, so the same JSON shape supports a future
/// strategy that adds more.
#[derive(Debug, Clone, Copy, Default, Serialize, Deserialize)]
pub struct CommonStrategyParams {
    #[serde(default)]
    pub direction: CutDirection,
    #[serde(default)]
    pub z_range: ZRange,
    #[serde(default)]
    pub spring_pass: SpringPass,
}

/// Apply every cross-strategy transform in one call. Order matters:
/// Z-range filtering happens *after* direction reversal so the visible
/// "first / last" cut after a reversal still respects the user's depth
/// limits. Spring pass runs last so the duplicated finish pass picks up
/// the same direction / Z-range the user just set.
pub fn apply_common(tp: &mut Toolpath, common: &CommonStrategyParams) {
    apply_direction(tp, common.direction);
    apply_z_range(tp, &common.z_range);
    apply_spring_pass(tp, &common.spring_pass);
}

// ── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::toolpath::{OperationType, Toolpath, ToolpathPoint};
    use nalgebra::{Point3, Vector3};

    fn rapid(p: [f64; 3]) -> ToolpathPoint {
        ToolpathPoint {
            position: Point3::new(p[0], p[1], p[2]),
            orientation: Vector3::z_axis(),
            feed_rate: 0.0,
            move_type: MoveType::Rapid,
        }
    }
    fn cut(p: [f64; 3]) -> ToolpathPoint {
        ToolpathPoint {
            position: Point3::new(p[0], p[1], p[2]),
            orientation: Vector3::z_axis(),
            feed_rate: 800.0,
            move_type: MoveType::Linear,
        }
    }

    #[test]
    fn climb_is_a_no_op() {
        let mut tp = Toolpath::new(1, OperationType::Roughing, "t");
        tp.points.push(rapid([0.0, 0.0, 5.0]));
        tp.points.push(cut  ([0.0, 0.0, 0.0]));
        tp.points.push(cut  ([10.0, 0.0, 0.0]));
        tp.points.push(rapid([10.0, 0.0, 5.0]));
        let before = tp.points.iter().map(|p| p.position.x).collect::<Vec<_>>();
        apply_direction(&mut tp, CutDirection::Climb);
        let after = tp.points.iter().map(|p| p.position.x).collect::<Vec<_>>();
        assert_eq!(before, after);
    }

    #[test]
    fn conventional_reverses_cutting_block() {
        // Rapid → cut(0) → cut(10) → cut(20) → Rapid.
        // After Conventional: Rapid → cut(20) → cut(10) → cut(0) → Rapid.
        let mut tp = Toolpath::new(1, OperationType::Roughing, "t");
        tp.points.push(rapid([0.0, 0.0, 5.0]));
        tp.points.push(cut  ([0.0, 0.0, 0.0]));
        tp.points.push(cut  ([10.0, 0.0, 0.0]));
        tp.points.push(cut  ([20.0, 0.0, 0.0]));
        tp.points.push(rapid([20.0, 0.0, 5.0]));
        apply_direction(&mut tp, CutDirection::Conventional);
        let xs: Vec<f64> = tp.points.iter().map(|p| p.position.x).collect();
        assert_eq!(xs, vec![0.0, 20.0, 10.0, 0.0, 20.0]);
        // Move types preserved: rapids stay at their positions, cuts stay cuts.
        assert!(matches!(tp.points[0].move_type, MoveType::Rapid));
        assert!(matches!(tp.points[1].move_type, MoveType::Linear));
        assert!(matches!(tp.points[4].move_type, MoveType::Rapid));
    }

    #[test]
    fn z_range_filters_cuts_only() {
        let mut tp = Toolpath::new(1, OperationType::Roughing, "t");
        tp.points.push(rapid([0.0, 0.0, 10.0]));
        tp.points.push(cut  ([0.0, 0.0,  0.0]));   // inside
        tp.points.push(cut  ([0.0, 0.0, -3.0]));   // inside
        tp.points.push(cut  ([0.0, 0.0, -8.0]));   // below bottom
        tp.points.push(rapid([0.0, 0.0, 10.0]));   // rapid above top: kept
        let range = ZRange { top_mm: Some(2.0), bottom_mm: Some(-5.0) };
        apply_z_range(&mut tp, &range);
        // The below-bottom cut is gone; rapids untouched.
        let zs: Vec<f64> = tp.points.iter().map(|p| p.position.z).collect();
        assert_eq!(zs, vec![10.0, 0.0, -3.0, 10.0]);
    }

    #[test]
    fn unbounded_range_is_noop() {
        let mut tp = Toolpath::new(1, OperationType::Roughing, "t");
        tp.points.push(cut([0.0, 0.0, 0.0]));
        tp.points.push(cut([0.0, 0.0, -100.0]));
        let before = tp.points.len();
        apply_z_range(&mut tp, &ZRange::default());
        assert_eq!(tp.points.len(), before);
    }

    #[test]
    fn spring_pass_disabled_is_noop() {
        let mut tp = Toolpath::new(1, OperationType::Finishing, "t");
        tp.points.push(rapid([0.0, 0.0, 5.0]));
        tp.points.push(cut  ([0.0, 0.0, 0.0]));
        tp.points.push(cut  ([10.0, 0.0, 0.0]));
        let before = tp.points.len();
        apply_spring_pass(&mut tp, &SpringPass::default());
        assert_eq!(tp.points.len(), before);
    }

    #[test]
    fn spring_pass_clones_last_block_at_reduced_feed() {
        let mut tp = Toolpath::new(1, OperationType::Finishing, "t");
        tp.points.push(rapid([0.0, 0.0, 5.0]));
        tp.points.push(cut  ([0.0, 0.0, 0.0]));
        tp.points.push(cut  ([10.0, 0.0, 0.0]));
        // The "last pass unit" is everything from the last rapid onward.
        let before = tp.points.len();
        let sp = SpringPass { enabled: true, feed_fraction: 0.5 };
        apply_spring_pass(&mut tp, &sp);
        assert_eq!(tp.points.len(), before * 2 - 0); // appended a full clone of the suffix
        // The appended cuts must run at half feed.
        let appended_cut = &tp.points[tp.points.len() - 1];
        assert!((appended_cut.feed_rate - 400.0).abs() < 1e-6,
            "expected 800 * 0.5 = 400, got {}", appended_cut.feed_rate);
    }

    #[test]
    fn cusp_height_smaller_means_smaller_step() {
        let tool = crate::tool::Tool::ball_end(1, "6mm ball", 6.0, 20.0);
        let coarse = step_over_from_cusp(0.05, &tool);
        let fine   = step_over_from_cusp(0.005, &tool);
        assert!(fine < coarse,
            "finer cusp should give a smaller step-over: fine={fine}, coarse={coarse}");
        // Coarse should be roughly 2 * sqrt(2 * 3 * 0.05 - 0.0025) ≈ 1.09 mm.
        assert!(coarse > 0.5 && coarse < 1.5, "coarse step out of range: {coarse}");
    }

    #[test]
    fn cusp_height_falls_back_for_flat_tools() {
        let tool = crate::tool::Tool::flat_end(1, "6mm flat", 6.0, 20.0);
        // Flat tool has no radius — helper returns the fraction-based fallback.
        let so = step_over_from_cusp(0.01, &tool);
        // Fallback is 0.5 × diameter = 3 mm.
        assert!((so - 3.0).abs() < 1e-6, "flat tool fallback wrong: {so}");
    }
}

use nalgebra::{Point3, Unit, Vector3};
use serde::{Deserialize, Serialize};

// ── Move classification ───────────────────────────────────────────────────────

/// What kind of motion a [`ToolpathPoint`] transition represents.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MoveType {
    /// Rapid non-cutting traverse (G0).
    Rapid,
    /// Linear cutting move (G1).
    Linear,
    /// Lead-in arc or ramp entering the cut.
    LeadIn,
    /// Lead-out arc or ramp exiting the cut.
    LeadOut,
    /// Retract to safe clearance height.
    Retract,
}

// ── Operation classification ──────────────────────────────────────────────────

/// High-level classification of the machining strategy that produced a toolpath.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum OperationType {
    Roughing,
    Finishing,
    Contour,
    Drilling,
    Swarf,
}

// ── Single toolpath point ─────────────────────────────────────────────────────

/// A single point along a 5-axis toolpath.
///
/// Positions and orientations are expressed in the **workpiece coordinate
/// system** (WCS).  The tool-axis orientation points **from tip toward shank**.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ToolpathPoint {
    /// Tool tip position in workpiece coordinates [mm].
    pub position: Point3<f64>,
    /// Tool axis direction, tip → shank (unit vector in WCS).
    pub orientation: Unit<Vector3<f64>>,
    /// Programmed feed rate at this point [mm/min].
    pub feed_rate: f64,
    /// Classification of the move leading *to* this point.
    pub move_type: MoveType,
}

impl ToolpathPoint {
    /// Create a rapid-traverse point with a vertical tool axis.
    pub fn rapid(position: Point3<f64>) -> Self {
        ToolpathPoint {
            position,
            orientation: Vector3::z_axis(),
            feed_rate: 0.0,
            move_type: MoveType::Rapid,
        }
    }

    /// Create a retract point with a vertical tool axis.
    pub fn retract(position: Point3<f64>) -> Self {
        ToolpathPoint {
            position,
            orientation: Vector3::z_axis(),
            feed_rate: 0.0,
            move_type: MoveType::Retract,
        }
    }

    /// Euclidean distance in XYZ space to `other` [mm].
    pub fn distance_to(&self, other: &ToolpathPoint) -> f64 {
        (self.position - other.position).norm()
    }
}

// ── Toolpath ──────────────────────────────────────────────────────────────────

/// An ordered sequence of [`ToolpathPoint`]s produced by a single operation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Toolpath {
    /// Ordered path points.
    pub points: Vec<ToolpathPoint>,
    /// ID of the tool that cuts this path (matches [`crate::tool::Tool::id`]).
    pub tool_id: u32,
    /// Strategy type that generated this path.
    pub operation: OperationType,
    /// Human-readable name for display / G-code comments.
    pub name: String,
}

impl Toolpath {
    /// Construct an empty toolpath.
    pub fn new(tool_id: u32, operation: OperationType, name: impl Into<String>) -> Self {
        Toolpath {
            points: Vec::new(),
            tool_id,
            operation,
            name: name.into(),
        }
    }

    /// `true` when there are no points.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Return a copy of this toolpath with cutting and plunge feed rates
    /// overridden by the given values. A value of `0.0` leaves that move type's
    /// feed unchanged so strategy-supplied feeds are preserved when the user
    /// hasn't dialled in an op-level override.
    ///
    /// - `cutting_feed` overrides every `MoveType::Linear` and `MoveType::LeadOut`.
    /// - `plunge_feed` overrides every `MoveType::LeadIn` (the entry move).
    pub fn with_op_feeds(&self, cutting_feed: f64, plunge_feed: f64) -> Toolpath {
        let mut out = self.clone();
        for pt in &mut out.points {
            match pt.move_type {
                MoveType::Linear | MoveType::LeadOut if cutting_feed > 0.0 => {
                    pt.feed_rate = cutting_feed;
                }
                MoveType::LeadIn if plunge_feed > 0.0 => {
                    pt.feed_rate = plunge_feed;
                }
                _ => {}
            }
        }
        out
    }

    /// Total arc length of **cutting** (Linear + Lead*) moves [mm].
    pub fn cutting_length(&self) -> f64 {
        self.points.windows(2).fold(0.0, |acc, w| {
            if matches!(w[1].move_type, MoveType::Linear | MoveType::LeadIn | MoveType::LeadOut) {
                acc + w[0].distance_to(&w[1])
            } else {
                acc
            }
        })
    }

    /// Estimated runtime of this toolpath in **minutes**.
    ///
    /// For each segment:
    /// - cutting moves use the segment's `feed_rate` (mm/min);
    /// - rapids use `rapid_mm_per_min` — pass the machine's rapid speed
    ///   (LinuxCNC default ≈ 5000 mm/min, hobby grbl ≈ 1500).
    ///
    /// Excludes acceleration/deceleration and tool changes — it's a
    /// programmer's estimate, not a controller-accurate dispatch time.
    pub fn duration_minutes(&self, rapid_mm_per_min: f64) -> f64 {
        self.points.windows(2).fold(0.0, |acc, w| {
            let dist = w[0].distance_to(&w[1]);
            let rate = match w[1].move_type {
                MoveType::Rapid | MoveType::Retract => rapid_mm_per_min.max(1.0),
                _ => w[1].feed_rate.max(1.0),
            };
            acc + dist / rate
        })
    }

    /// Compute cut/rapid distance and time split.
    ///
    /// Lead-in/lead-out segments count as cutting (the cutter is engaged or
    /// approaching engagement). Rapid + Retract count as rapids.
    pub fn metrics(&self, rapid_mm_per_min: f64) -> ToolpathMetrics {
        let mut m = ToolpathMetrics::default();
        for w in self.points.windows(2) {
            let dist = w[0].distance_to(&w[1]);
            match w[1].move_type {
                MoveType::Rapid | MoveType::Retract => {
                    m.rapid_distance_mm += dist;
                    m.rapid_minutes += dist / rapid_mm_per_min.max(1.0);
                }
                _ => {
                    m.cut_distance_mm += dist;
                    m.cut_minutes += dist / w[1].feed_rate.max(1.0);
                }
            }
        }
        m.point_count = self.points.len();
        m
    }

    /// Length of `Linear` / `LeadIn` / `LeadOut` segments whose endpoints
    /// both sit outside the supplied stock envelope. The classification
    /// matches the viewport's air-cut shading, so the reported figure is
    /// the distance the user sees rendered dimly.
    ///
    /// `tool_radius_mm` widens the XY footprint to account for the
    /// cutter's radial extent. A `None` envelope returns 0 — no envelope,
    /// no flag.
    pub fn air_cut_distance(
        &self,
        stock_aabb: Option<&parry3d::bounding_volume::Aabb>,
        tool_radius_mm: f64,
    ) -> f64 {
        let Some(aabb) = stock_aabb else { return 0.0 };
        let r = tool_radius_mm.max(0.0) as f32;
        let in_stock = |p: &nalgebra::Point3<f64>| {
            let px = p.x as f32;
            let py = p.y as f32;
            let pz = p.z as f32;
            pz <= aabb.maxs.z + 1e-3
                && px >= aabb.mins.x - r
                && px <= aabb.maxs.x + r
                && py >= aabb.mins.y - r
                && py <= aabb.maxs.y + r
        };
        let mut total = 0.0;
        for w in self.points.windows(2) {
            let is_cut = matches!(
                w[1].move_type,
                MoveType::Linear | MoveType::LeadIn | MoveType::LeadOut
            );
            if !is_cut { continue; }
            if !in_stock(&w[0].position) && !in_stock(&w[1].position) {
                total += w[0].distance_to(&w[1]);
            }
        }
        total
    }
}

// ── Cycle-time + distance metrics ───────────────────────────────────────────

/// Summary of cut/rapid distance + time for one or more toolpaths.
///
/// All distances in mm, all times in minutes. Excludes acceleration, dwell,
/// tool-change time, and any non-motion controller overhead — it's a
/// programmer's estimate, not a controller-accurate dispatch time.
#[derive(Debug, Default, Clone, Copy)]
pub struct ToolpathMetrics {
    pub cut_distance_mm: f64,
    pub rapid_distance_mm: f64,
    pub cut_minutes: f64,
    pub rapid_minutes: f64,
    pub point_count: usize,
}

impl ToolpathMetrics {
    pub fn total_distance_mm(&self) -> f64 {
        self.cut_distance_mm + self.rapid_distance_mm
    }
    pub fn total_minutes(&self) -> f64 {
        self.cut_minutes + self.rapid_minutes
    }
    pub fn merge(&mut self, other: &ToolpathMetrics) {
        self.cut_distance_mm += other.cut_distance_mm;
        self.rapid_distance_mm += other.rapid_distance_mm;
        self.cut_minutes += other.cut_minutes;
        self.rapid_minutes += other.rapid_minutes;
        self.point_count += other.point_count;
    }
}

/// Aggregate metrics across a slice of toolpaths.
pub fn aggregate_metrics(paths: &[Toolpath], rapid_mm_per_min: f64) -> ToolpathMetrics {
    let mut acc = ToolpathMetrics::default();
    for p in paths {
        let m = p.metrics(rapid_mm_per_min);
        acc.merge(&m);
    }
    acc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn toolpath_point_serde_roundtrip() {
        let pt = ToolpathPoint {
            position: Point3::new(1.0, 2.0, 3.0),
            orientation: Vector3::z_axis(),
            feed_rate: 500.0,
            move_type: MoveType::Linear,
        };
        let json = serde_json::to_string(&pt).unwrap();
        let decoded: ToolpathPoint = serde_json::from_str(&json).unwrap();
        assert_eq!(pt.position, decoded.position);
        assert_eq!(pt.move_type, decoded.move_type);
        assert!((pt.feed_rate - decoded.feed_rate).abs() < 1e-12);
    }

    #[test]
    fn toolpath_cutting_length() {
        let mut tp = Toolpath::new(1, OperationType::Roughing, "test");
        tp.points.push(ToolpathPoint::rapid(Point3::new(0.0, 0.0, 5.0)));
        tp.points.push(ToolpathPoint {
            position: Point3::new(0.0, 0.0, 0.0),
            orientation: Vector3::z_axis(),
            feed_rate: 600.0,
            move_type: MoveType::Retract,  // not a cutting move
        });
        tp.points.push(ToolpathPoint {
            position: Point3::new(10.0, 0.0, 0.0),
            orientation: Vector3::z_axis(),
            feed_rate: 600.0,
            move_type: MoveType::Linear,
        });
        // Only the Linear segment (length = 10) counts.
        assert!((tp.cutting_length() - 10.0).abs() < 1e-12);
    }

    #[test]
    fn toolpath_duration_minutes() {
        // 3 points → 2 segments:
        //   (p0 → p1): destination is Retract → uses rapid rate (5000).
        //              dist = 5 mm,  time = 5/5000 = 0.001 min.
        //   (p1 → p2): destination is Linear at 1000 mm/min.
        //              dist = 10 mm, time = 10/1000 = 0.010 min.
        // Total: 0.011 min.
        let mut tp = Toolpath::new(1, OperationType::Roughing, "t");
        tp.points.push(ToolpathPoint::rapid(Point3::new(0.0, 0.0, 5.0)));
        tp.points.push(ToolpathPoint {
            position: Point3::new(0.0, 0.0, 0.0),
            orientation: Vector3::z_axis(),
            feed_rate: 600.0,
            move_type: MoveType::Retract,
        });
        tp.points.push(ToolpathPoint {
            position: Point3::new(10.0, 0.0, 0.0),
            orientation: Vector3::z_axis(),
            feed_rate: 1000.0,
            move_type: MoveType::Linear,
        });
        let t = tp.duration_minutes(5000.0);
        assert!((t - 0.011).abs() < 1e-9, "expected 0.011 min, got {t}");
    }
}

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
}

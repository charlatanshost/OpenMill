use serde::{Deserialize, Serialize};

// ── Tool geometry ─────────────────────────────────────────────────────────────

/// Cutting-end geometry variants.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum ToolShape {
    /// Flat-end (square-end) mill.
    FlatEnd {
        /// Cutter diameter [mm].
        diameter: f64,
        /// Length of fluted cutting region [mm].
        flute_length: f64,
    },
    /// Ball-end (ball-nose) mill — radius = diameter / 2.
    BallEnd {
        /// Cutter diameter [mm].
        diameter: f64,
        /// Length of fluted cutting region [mm].
        flute_length: f64,
    },
    /// Bull-nose (toroid / corner-radius) mill.
    BullNose {
        diameter: f64,
        corner_radius: f64,
        flute_length: f64,
    },
    /// Chamfer mill / Engraving tool.
    ChamferMill {
        /// Tip diameter [mm].
        tip_diameter: f64,
        /// Major diameter [mm].
        diameter: f64,
        /// Included taper angle [degrees] (e.g. 90 for a 45° chamfer).
        taper_angle: f64,
        flute_length: f64,
    },
    /// Standard twist drill or center drill.
    Drill {
        diameter: f64,
        /// Included point angle [degrees] (typically 118 or 135).
        point_angle: f64,
        flute_length: f64,
    },
    /// Tapered end mill (conical finishing).
    TaperedMill {
        /// Tip diameter [mm].
        tip_diameter: f64,
        /// Taper angle per side [degrees].
        taper_angle: f64,
        flute_length: f64,
    },
    /// Lollipop / Under-cutting spherical mill.
    Lollipop {
        /// Head diameter [mm].
        diameter: f64,
        /// Neck/Shank diameter [mm].
        neck_diameter: f64,
        flute_length: f64,
    },
    /// Dovetail / T-slot cutter.
    Dovetail {
        diameter: f64,
        /// Width of the cutting slot [mm].
        width: f64,
        /// Dovetail angle [degrees] (e.g. 60).
        angle: f64,
        flute_length: f64,
    },
    /// Thread mill.
    ThreadMill {
        diameter: f64,
        /// Pitch of the thread [mm].
        pitch: f64,
        /// Number of teeth.
        num_teeth: usize,
        flute_length: f64,
    },
}

impl ToolShape {
    /// Nominal cutter diameter [mm].
    pub fn diameter(&self) -> f64 {
        match self {
            ToolShape::FlatEnd { diameter, .. }
            | ToolShape::BallEnd { diameter, .. }
            | ToolShape::BullNose { diameter, .. }
            | ToolShape::ChamferMill { diameter, .. }
            | ToolShape::Drill { diameter, .. }
            | ToolShape::Lollipop { diameter, .. }
            | ToolShape::Dovetail { diameter, .. }
            | ToolShape::ThreadMill { diameter, .. } => *diameter,
            ToolShape::TaperedMill { tip_diameter, taper_angle, flute_length, .. } => {
                // Return major diameter at top of flutes
                let angle_rad = taper_angle.to_radians();
                *tip_diameter + 2.0 * (*flute_length * angle_rad.tan())
            }
        }
    }

    /// Length of the fluted cutting zone [mm].
    pub fn flute_length(&self) -> f64 {
        match self {
            ToolShape::FlatEnd { flute_length, .. }
            | ToolShape::BallEnd { flute_length, .. }
            | ToolShape::BullNose { flute_length, .. }
            | ToolShape::ChamferMill { flute_length, .. }
            | ToolShape::Drill { flute_length, .. }
            | ToolShape::TaperedMill { flute_length, .. }
            | ToolShape::Lollipop { flute_length, .. }
            | ToolShape::Dovetail { flute_length, .. }
            | ToolShape::ThreadMill { flute_length, .. } => *flute_length,
        }
    }

    pub fn is_ball(&self) -> bool {
        matches!(self, ToolShape::BallEnd { .. })
    }
}

// ── Tool holder ───────────────────────────────────────────────────────────────

/// Axisymmetric holder silhouette as a series of (z, r) profile points.
///
/// Points are sorted by `z` (distance from spindle nose face, increasing).
/// Each pair defines the outer radius of the holder cross-section at that z.
///
/// ```text
///  z=0  ──── spindle nose ────
///  z=5      ╔════╗   r=25 mm
///  z=40     ║    ║   r=25
///  z=41     ╚══╗ ║   r=10 mm  (taper)
///  z=60        ╚═╝   r=10
/// ```
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ToolHolder {
    pub name: String,
    /// `(z_mm, radius_mm)` pairs defining the outer silhouette of the holder.
    /// Must be sorted by `z` (ascending).
    pub profile: Vec<(f64, f64)>,
}

// ── Tool ──────────────────────────────────────────────────────────────────────

/// Complete tool definition as it would appear in a tool library.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Tool {
    /// Unique tool ID (matches T-number in G-code).
    pub id: u32,
    /// Human-readable name / description.
    pub name: String,
    /// Cutting geometry.
    pub shape: ToolShape,
    /// Total tool length from holder face to tip [mm].
    pub overall_length: f64,
    /// Shank diameter [mm].
    pub shank_diameter: f64,
    /// Optional holder; `None` means bare shank only.
    pub holder: Option<ToolHolder>,
}

impl Tool {
    /// Convenience constructor for a flat-end mill.
    pub fn flat_end(
        id: u32,
        name: impl Into<String>,
        diameter: f64,
        flute_length: f64,
    ) -> Self {
        Tool {
            id,
            name: name.into(),
            shape: ToolShape::FlatEnd { diameter, flute_length },
            overall_length: flute_length + 30.0,
            shank_diameter: diameter,
            holder: None,
        }
    }

    /// Convenience constructor for a ball-end mill.
    pub fn ball_end(
        id: u32,
        name: impl Into<String>,
        diameter: f64,
        flute_length: f64,
    ) -> Self {
        Tool {
            id,
            name: name.into(),
            shape: ToolShape::BallEnd { diameter, flute_length },
            overall_length: flute_length + 30.0,
            shank_diameter: diameter,
            holder: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tool_serde_roundtrip() {
        let tool = Tool::flat_end(1, "6mm flat", 6.0, 20.0);
        let json = serde_json::to_string(&tool).unwrap();
        let decoded: Tool = serde_json::from_str(&json).unwrap();
        assert_eq!(tool, decoded);
    }

    #[test]
    fn tool_shape_diameter_accessor() {
        assert_eq!(ToolShape::BallEnd { diameter: 8.0, flute_length: 24.0 }.diameter(), 8.0);
        assert_eq!(ToolShape::BullNose { diameter: 12.0, corner_radius: 1.5, flute_length: 30.0 }.diameter(), 12.0);
    }
}

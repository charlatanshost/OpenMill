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

// ── Coolant ───────────────────────────────────────────────────────────────────

/// Coolant mode requested for an operation or stored in a feed-and-speed preset.
///
/// `gcode_on()` returns the controller-specific code emitted at op start, and
/// `gcode_off()` returns the off code (always `M9` for FANUC-compatible posts).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum Coolant {
    #[default]
    None,
    /// Mist coolant — `M7`.
    Mist,
    /// Flood coolant — `M8`.
    Flood,
    /// Mist + flood simultaneously — `M7 M8`.
    MistFlood,
    /// Through-spindle coolant — `M88` (LinuxCNC).
    Through,
}

impl Coolant {
    pub fn label(&self) -> &'static str {
        match self {
            Coolant::None      => "None",
            Coolant::Mist      => "Mist (M7)",
            Coolant::Flood     => "Flood (M8)",
            Coolant::MistFlood => "Mist+Flood (M7 M8)",
            Coolant::Through   => "Through (M88)",
        }
    }

    /// G-code to enable this coolant mode, or `None` for `Coolant::None`.
    pub fn gcode_on(&self) -> Option<&'static str> {
        match self {
            Coolant::None      => None,
            Coolant::Mist      => Some("M7"),
            Coolant::Flood     => Some("M8"),
            Coolant::MistFlood => Some("M7 M8"),
            Coolant::Through   => Some("M88"),
        }
    }

    /// G-code to disable all coolant — `M9` on every dialect we support.
    pub fn gcode_off() -> &'static str { "M9" }
}

// ── Feed and speed preset ────────────────────────────────────────────────────

/// Saved feed-and-speed combination attached to a [`Tool`].
///
/// Picking a preset on an [`Operation`] copies these values onto the op
/// (spindle, feed, plunge, coolant). Notes are informational only.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FeedSpeedPreset {
    pub name: String,
    /// Material this preset targets (free-form, e.g. "6061 Aluminum").
    #[serde(default)]
    pub material: String,
    /// Spindle speed [RPM].
    pub spindle_rpm: f64,
    /// Cutting feed rate [mm/min] for lateral moves.
    pub feed_rate: f64,
    /// Plunge feed rate [mm/min] for axial entry moves.
    pub plunge_rate: f64,
    #[serde(default)]
    pub coolant: Coolant,
    #[serde(default)]
    pub notes: String,
}

/// Generate a small set of starter presets for a tool of `diameter_mm` with
/// `flutes` flutes. Uses the same formulas as
/// [`crate::Material::rpm_for_diameter`] / [`crate::Material::feed_rate`] for a
/// handful of common materials. Plunge rate is set to 30% of cutting feed.
pub fn starter_presets_for(diameter_mm: f64, flutes: u32) -> Vec<FeedSpeedPreset> {
    let entries: &[(&str, &str, f64, f64, Coolant)] = &[
        // (name,             material,           surface_speed m/min, chip_load mm/tooth, coolant)
        ("6061 Aluminum",     "6061 Aluminum",    200.0, 0.05, Coolant::Mist),
        ("Mild Steel",        "Mild Steel (1018)", 80.0, 0.03, Coolant::Flood),
        ("HDPE / Plastic",    "HDPE Plastic",     300.0, 0.10, Coolant::None),
        ("Hardwood",          "Hardwood (Maple/Oak)", 350.0, 0.12, Coolant::None),
        ("Brass",             "Brass (360)",      120.0, 0.04, Coolant::None),
    ];

    entries
        .iter()
        .map(|(name, material, sfm, chip, coolant)| {
            let rpm = (sfm * 1000.0) / (std::f64::consts::PI * diameter_mm.max(0.1));
            let feed = chip * flutes as f64 * rpm;
            FeedSpeedPreset {
                name: (*name).into(),
                material: (*material).into(),
                spindle_rpm: rpm.round(),
                feed_rate: feed.round(),
                plunge_rate: (feed * 0.3).round().max(20.0),
                coolant: *coolant,
                notes: String::new(),
            }
        })
        .collect()
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

impl ToolHolder {
    /// Linear interpolate the holder's outer radius at axial distance `z`
    /// (measured from the holder base — i.e. from the spindle nose end of
    /// the holder). Out-of-range queries clamp to the nearest profile point.
    pub fn radius_at(&self, z: f64) -> f64 {
        if self.profile.is_empty() { return 0.0; }
        let first = self.profile[0];
        let last  = *self.profile.last().unwrap();
        if z <= first.0 { return first.1; }
        if z >= last.0  { return last.1;  }
        for w in self.profile.windows(2) {
            let (z0, r0) = w[0];
            let (z1, r1) = w[1];
            if z >= z0 && z <= z1 {
                let dz = (z1 - z0).max(1e-9);
                let t = (z - z0) / dz;
                return r0 + t * (r1 - r0);
            }
        }
        last.1
    }
}

// ── Tool ──────────────────────────────────────────────────────────────────────

fn default_flutes() -> u32 { 2 }

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
    /// Number of cutting flutes — used by feed-rate math.
    #[serde(default = "default_flutes")]
    pub flutes: u32,
    /// Saved feed-and-speed combinations; the user picks one on an operation.
    #[serde(default)]
    pub presets: Vec<FeedSpeedPreset>,
    /// Index into `presets` used as the default when this tool is selected.
    #[serde(default)]
    pub default_preset: Option<usize>,
    /// Free-form G-code injected on every tool change for this tool
    /// (e.g. probing routine, axis homing). Emitted after `M6 T#`.
    #[serde(default)]
    pub tool_change_gcode: Option<String>,
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
            flutes: 2,
            presets: starter_presets_for(diameter, 2),
            default_preset: Some(0),
            tool_change_gcode: None,
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
            flutes: 2,
            presets: starter_presets_for(diameter, 2),
            default_preset: Some(0),
            tool_change_gcode: None,
        }
    }
}

// ── Holder collision detection ───────────────────────────────────────────────

/// Test the **holder** silhouette against a triangle mesh at a single tool
/// pose. Returns the world-space mesh vertex that lies inside the holder
/// envelope, or `None` if there's no collision.
///
/// The holder is treated as an axisymmetric volume around the tool's local
/// `+Z` axis (pointing from the cutting tip toward the spindle nose). The
/// holder's `profile` (`(z, r)` pairs) describes the outer silhouette in
/// holder-local space; this routine offsets it by the tool's `flute_length`
/// so that `z = 0` on the profile sits at the top of the cutting region.
///
/// Implementation uses a per-vertex test in tool-local coordinates. For a
/// typical CAM mesh (tens of thousands of vertices) this is fast enough to
/// run every simulation step. Triangles whose vertices all sit outside the
/// holder envelope but whose interior straddles it are *not* caught — for
/// finely tessellated meshes this case is rare in practice.
///
/// Returns `None` immediately when the tool has no holder defined (we don't
/// guess one — explicit holder definitions only).
pub fn holder_collision(
    tool: &Tool,
    pose: &nalgebra::Isometry3<f32>,
    mesh: &parry3d::shape::TriMesh,
) -> Option<nalgebra::Point3<f32>> {
    let holder = tool.holder.as_ref()?;
    if holder.profile.is_empty() { return None; }

    // Holder lives ABOVE the cutting region. Tool-local Z=0 is the tip; the
    // holder's profile-Z = 0 sits at the top of the flutes.
    let z_offset = tool.shape.flute_length() as f32;
    let z_min = z_offset + holder.profile.first().unwrap().0 as f32;
    let z_max = z_offset + holder.profile.last().unwrap().0 as f32;

    let pose_inv = pose.inverse();
    for v in mesh.vertices() {
        let local = pose_inv * *v;
        if local.z < z_min || local.z > z_max { continue; }
        let r_holder = holder.radius_at((local.z - z_offset) as f64) as f32;
        let r_vertex = (local.x * local.x + local.y * local.y).sqrt();
        if r_vertex < r_holder {
            return Some(*v);
        }
    }
    None
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

    #[test]
    fn holder_radius_at_interpolates_linearly() {
        let h = ToolHolder {
            name: "test".into(),
            profile: vec![(0.0, 25.0), (40.0, 25.0), (41.0, 10.0), (60.0, 10.0)],
        };
        // Midpoint between (40, 25) and (41, 10) → r = 17.5
        assert!((h.radius_at(40.5) - 17.5).abs() < 1e-9);
        // Below profile clamps to first.
        assert_eq!(h.radius_at(-5.0), 25.0);
        // Above profile clamps to last.
        assert_eq!(h.radius_at(100.0), 10.0);
    }

    #[test]
    fn holder_collision_detects_a_vertex_inside_envelope() {
        use parry3d::math::Point as PPoint;
        use parry3d::shape::TriMesh;
        let mut tool = Tool::flat_end(1, "6 flat", 6.0, 20.0);
        // 25mm-radius collar from z=20 (top of flutes) to z=60 in tool-local Z.
        tool.holder = Some(ToolHolder {
            name: "collar".into(),
            profile: vec![(0.0, 25.0), (40.0, 25.0)],
        });
        // Mesh: a single triangle that has one vertex inside the holder
        // envelope at z=30 (above the cutting region) — should collide.
        let verts = vec![
            PPoint::<f32>::new(10.0, 0.0, 30.0), // inside (r=10 < 25)
            PPoint::<f32>::new(80.0, 0.0, 30.0), // outside (r=80)
            PPoint::<f32>::new(80.0, 80.0, 30.0),
        ];
        let tris = vec![[0u32, 1, 2]];
        let mesh = TriMesh::new(verts, tris);
        let pose = nalgebra::Isometry3::identity();
        let hit = holder_collision(&tool, &pose, &mesh);
        assert!(hit.is_some(), "should detect the inside vertex");
        let p = hit.unwrap();
        assert!((p.x - 10.0).abs() < 1e-6, "wrong vertex");
    }

    #[test]
    fn holder_collision_returns_none_with_no_holder() {
        use parry3d::math::Point as PPoint;
        use parry3d::shape::TriMesh;
        let tool = Tool::flat_end(1, "6 flat", 6.0, 20.0);
        let mesh = TriMesh::new(
            vec![
                PPoint::<f32>::new(0.0, 0.0, 30.0),
                PPoint::<f32>::new(1.0, 0.0, 30.0),
                PPoint::<f32>::new(0.0, 1.0, 30.0),
            ],
            vec![[0u32, 1, 2]],
        );
        let pose = nalgebra::Isometry3::identity();
        assert!(holder_collision(&tool, &pose, &mesh).is_none());
    }
}

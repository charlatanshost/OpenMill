use serde::{Deserialize, Serialize};

/// A fixture or clamp that the tool must avoid during machining.
///
/// Fixtures are modelled as simple geometric primitives positioned in
/// workpiece coordinates. The collision checker uses these to flag moves
/// that would crash the tool/holder into a clamp or vice jaw.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Fixture {
    /// Human-readable name shown in the UI and warnings.
    pub name: String,
    /// Geometric shape of the fixture.
    pub shape: FixtureShape,
    /// Pose of the fixture in workpiece coordinates.
    pub position: [f64; 3],
    /// Rotation of the fixture as [roll, pitch, yaw] in degrees.
    pub rotation_deg: [f64; 3],
}

/// Simple primitive shapes for fixture definitions.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum FixtureShape {
    /// Rectangular block centred on the fixture origin.
    Box {
        /// Half-extents in X, Y, Z [mm].
        half_extents: [f64; 3],
    },
    /// Upright cylinder centred on the fixture origin.
    Cylinder {
        /// Cylinder radius [mm].
        radius: f64,
        /// Cylinder height [mm].
        height: f64,
    },
}

impl Fixture {
    /// Create a box-shaped fixture at a given position.
    pub fn new_box(name: impl Into<String>, half_extents: [f64; 3], position: [f64; 3]) -> Self {
        Fixture {
            name: name.into(),
            shape: FixtureShape::Box { half_extents },
            position,
            rotation_deg: [0.0; 3],
        }
    }

    /// Create a cylindrical fixture at a given position.
    pub fn new_cylinder(
        name: impl Into<String>,
        radius: f64,
        height: f64,
        position: [f64; 3],
    ) -> Self {
        Fixture {
            name: name.into(),
            shape: FixtureShape::Cylinder { radius, height },
            position,
            rotation_deg: [0.0; 3],
        }
    }
}

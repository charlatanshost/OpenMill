use nalgebra::{Unit, Vector3};
use thiserror::Error;

/// Angular threshold below which sin(A) is considered degenerate.
/// At this threshold A ≈ 0.057° — well inside any practical machine's dead band.
pub const SINGULARITY_THRESHOLD: f64 = 1e-9;

// ── Axis configuration ────────────────────────────────────────────────────────

/// Configuration of a single rotary axis.
#[derive(Debug, Clone)]
pub struct RotaryAxis {
    /// Physical rotation axis direction in machine coordinates.
    pub axis_vector: Unit<Vector3<f64>>,
    /// Minimum travel limit [radians].
    pub min_angle: f64,
    /// Maximum travel limit [radians].
    pub max_angle: f64,
    /// Preferred / home angle [radians].
    pub home_angle: f64,
}

// ── Kinematic topology ────────────────────────────────────────────────────────

/// Supported kinematic topologies.
#[derive(Debug, Clone)]
pub enum KinematicType {
    /// A-C table-table trunnion.
    ///
    /// The workpiece sits on the C table; the C table is carried by the A cradle.
    ///
    /// * **A axis** — tilts the cradle around X (typically ±110°–120°).
    /// * **C axis** — rotates the table around Z (often continuous, ±360°).
    TableTable {
        a_axis: RotaryAxis,
        c_axis: RotaryAxis,
    },
}

// ── Linear travel limits ──────────────────────────────────────────────────────

/// Axis-aligned linear travel limits [mm].
#[derive(Debug, Clone)]
pub struct AxisLimits {
    pub x: (f64, f64),
    pub y: (f64, f64),
    pub z: (f64, f64),
}

// ── Machine configuration ─────────────────────────────────────────────────────

/// Complete machine configuration.
#[derive(Debug, Clone)]
pub struct MachineConfig {
    pub name: String,
    /// Kinematic topology and rotary axis parameters.
    pub axes: KinematicType,
    /// Pivot point of the rotary assembly in machine coordinates [mm].
    ///
    /// This is the point that stays fixed as the rotary axes move.  For a
    /// typical hobby trunnion the pivot sits below the table surface.
    pub pivot_offset: Vector3<f64>,
    /// Linear axis travel limits [mm].
    pub travel_limits: AxisLimits,
}

// ── IK error type ─────────────────────────────────────────────────────────────

/// Errors returned by [`crate::kinematics::traits::InverseKinematics::ik`].
#[derive(Debug, Error)]
pub enum IkError {
    /// The requested tool orientation is at or very near the kinematic
    /// singularity (A ≈ 0°, tool axis vertical).  C becomes degenerate.
    #[error("kinematic singularity: tool axis is near vertical (A ≈ 0°), C is degenerate")]
    Singularity,

    /// All candidate solutions exceed at least one axis travel limit.
    #[error("no IK solution lies within machine travel limits")]
    OutOfLimits,
}

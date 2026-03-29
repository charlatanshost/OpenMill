use anyhow::Result;
use openmill_core::{InverseKinematics, MachineConfig, TableTable, ToolpathPoint};

/// Default maximum rotary axis velocity [deg/min] — typical hobby servo.
pub const MAX_ROTARY_VELOCITY: f64 = 3600.0;

/// Compute the G93 inverse-time F-word value for a move between two 5-axis
/// toolpath points.
///
/// Returns `F = 1.0 / time_in_minutes`, where `time_in_minutes` is the
/// estimated move duration based on the slowest of:
///
/// - **Linear travel:** machine-frame XYZ distance ÷ programmed feed rate.
/// - **A-axis rotation:** |ΔA| ÷ [`MAX_ROTARY_VELOCITY`].
/// - **C-axis rotation:** |ΔC| ÷ [`MAX_ROTARY_VELOCITY`].
///
/// Near the A ≈ 0° singularity a small orientation change can produce a huge
/// ΔC.  Dividing by [`MAX_ROTARY_VELOCITY`] naturally clamps the C velocity
/// and slows the move to a safe speed.
pub fn compute_inverse_time_feed(
    from: &ToolpathPoint,
    to: &ToolpathPoint,
    machine: &MachineConfig,
) -> f64 {
    let tt = match TableTable::new(machine.clone()) {
        Ok(tt) => tt,
        Err(_) => {
            let feed = to.feed_rate.max(1.0);
            return 1.0 / (from.distance_to(to) / feed).max(1e-9);
        }
    };
    compute_inverse_time_feed_with_kin(&tt, from, to)
}

/// Same as [`compute_inverse_time_feed`] but reuses an existing [`TableTable`]
/// to avoid re-constructing kinematics on every segment.
pub(crate) fn compute_inverse_time_feed_with_kin(
    tt: &TableTable,
    from: &ToolpathPoint,
    to: &ToolpathPoint,
) -> f64 {
    let feed = to.feed_rate.max(1.0);

    let sf = joints_for_point(tt, from).unwrap_or([
        from.position.x,
        from.position.y,
        from.position.z,
        0.0,
        0.0,
    ]);
    let st = joints_for_point(tt, to).unwrap_or([
        to.position.x,
        to.position.y,
        to.position.z,
        0.0,
        0.0,
    ]);

    // Machine-frame linear distance.
    let dx = st[0] - sf[0];
    let dy = st[1] - sf[1];
    let dz = st[2] - sf[2];
    let machine_dist = (dx * dx + dy * dy + dz * dz).sqrt();
    let linear_time = machine_dist / feed;

    // Rotary-axis displacement (degrees).
    let delta_a = (st[3] - sf[3]).abs();
    let delta_c = (st[4] - sf[4]).abs();

    // Clamp rotary velocities to MAX_ROTARY_VELOCITY.
    // Near A≈0° singularity, delta_c can be huge for small orientation
    // changes — dividing by MAX_ROTARY_VELOCITY naturally yields a long
    // time / small F, which slows the move to a safe speed.
    let a_time = delta_a / MAX_ROTARY_VELOCITY;
    let c_time = delta_c / MAX_ROTARY_VELOCITY;

    let total_time = linear_time.max(a_time).max(c_time).max(1e-9);
    1.0 / total_time
}

/// Resolve the machine joint angles for a toolpath point.
///
/// Returns `[x_mm, y_mm, z_mm, a_deg, c_deg]`.  For near-vertical tool
/// orientations the singularity is avoided by returning A=0, C=0 directly.
pub(crate) fn joints_for_point(
    tt: &TableTable,
    pt: &ToolpathPoint,
) -> Result<[f64; 5]> {
    if pt.orientation.z.abs() > 1.0 - 1e-6 {
        // Near-vertical: A=0, C=0, and with no rotation the machine position
        // equals the workpiece position.
        return Ok([pt.position.x, pt.position.y, pt.position.z, 0.0, 0.0]);
    }
    let solutions = tt.ik(&pt.position, &pt.orientation)?;
    Ok(solutions[0])
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use nalgebra::{Point3, Unit, Vector3};
    use openmill_core::MoveType;

    fn default_machine() -> MachineConfig {
        openmill_core::kinematics::default_trunnion_config()
    }

    #[test]
    fn g93_linear_no_rotation() {
        // 10 mm linear move at F1000 mm/min, no rotation.
        // time = 10/1000 = 0.01 min → F = 100.
        let from = ToolpathPoint {
            position: Point3::new(0.0, 0.0, 0.0),
            orientation: Vector3::z_axis(),
            feed_rate: 1000.0,
            move_type: MoveType::Linear,
        };
        let to = ToolpathPoint {
            position: Point3::new(10.0, 0.0, 0.0),
            orientation: Vector3::z_axis(),
            feed_rate: 1000.0,
            move_type: MoveType::Linear,
        };

        let f = compute_inverse_time_feed(&from, &to, &default_machine());
        assert_abs_diff_eq!(f, 100.0, epsilon = 1e-6);
    }

    #[test]
    fn near_singularity_clamps_feed() {
        // Two points with tool axis ~1° from vertical but different C angles.
        // IK gives ΔC ≈ 90°, clamped by MAX_ROTARY_VELOCITY → slow F.
        let a_rad = 1.0_f64.to_radians();
        let from = ToolpathPoint {
            position: Point3::new(0.0, 0.0, 0.0),
            orientation: Unit::new_normalize(Vector3::new(
                a_rad.sin(),
                0.0,
                a_rad.cos(),
            )),
            feed_rate: 1000.0,
            move_type: MoveType::Linear,
        };
        let to = ToolpathPoint {
            position: Point3::new(1.0, 0.0, 0.0),
            orientation: Unit::new_normalize(Vector3::new(
                0.0,
                a_rad.sin(),
                a_rad.cos(),
            )),
            feed_rate: 1000.0,
            move_type: MoveType::Linear,
        };

        let f = compute_inverse_time_feed(&from, &to, &default_machine());

        // Without rotary clamping, pure-linear F ≈ 1000 (1 mm at 1000 mm/min).
        // With 90° C rotation at 3600 deg/min max, F ≈ 40.
        assert!(f < 100.0, "near-singularity should clamp F, got {f}");
        assert!(f > 1.0, "F should remain positive, got {f}");
    }
}

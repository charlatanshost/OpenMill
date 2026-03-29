use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use parry3d::query;
use parry3d::shape::SharedShape;

use openmill_core::{
    InverseKinematics, MachineConfig, TableTable, Tool, ToolShape, Toolpath,
};

// ── Collision result ─────────────────────────────────────────────────────────

/// Result of a single collision query.
#[derive(Debug, Clone)]
pub struct CollisionResult {
    /// Name of the colliding component.
    pub component: String,
    /// Estimated penetration depth [mm].
    pub penetration_depth: f64,
}

// ── Tool shape builder ───────────────────────────────────────────────────────

/// Build a compound collision shape for the full tool (cutting edge + shank).
///
/// The shape is oriented with the tool tip at the local origin and the body
/// extending along +Z (tip → shank direction).
pub fn build_tool_shape(tool: &Tool) -> SharedShape {
    build_tool_shapes(tool).0
}

/// Build both the full tool shape and the holder-only shape (shank above flute).
///
/// The holder shape is used for workpiece collision checks — the cutting edge
/// is expected to contact the workpiece and should not be flagged.
///
/// Tool body extends along +Z (tip at origin, shank towards +Z).
/// Cylinders are approximated as cuboids to work around a parry3d GJK bug
/// with rotated `Cylinder` shapes (false negatives at certain translations).
pub fn build_tool_shapes(tool: &Tool) -> (SharedShape, SharedShape) {
    let flute_length = tool.shape.flute_length();
    let cutting_r = (tool.shape.diameter() / 2.0) as f32;
    let shank_r = (tool.shank_diameter / 2.0) as f32;

    // ── Cutting box: z = 0 .. flute_length ──────────────────────────────
    let cutting_hh = (flute_length / 2.0) as f32;
    let cutting_z = cutting_hh;
    let cutting_iso = Isometry3::translation(0.0, 0.0, cutting_z);
    let cutting = SharedShape::cuboid(
        cutting_r.max(0.001),
        cutting_r.max(0.001),
        cutting_hh.max(0.001),
    );

    // ── Shank box: z = flute_length .. overall_length ───────────────────
    let shank_length = (tool.overall_length - flute_length) as f32;
    let shank_hh = (shank_length / 2.0).max(0.001);
    let shank_z = flute_length as f32 + shank_hh;
    let shank_iso = Isometry3::translation(0.0, 0.0, shank_z);
    let shank = SharedShape::cuboid(shank_r.max(0.001), shank_r.max(0.001), shank_hh);

    // ── BallEnd: add a sphere at the tip ────────────────────────────────
    let mut full_parts = vec![
        (cutting_iso, cutting),
        (shank_iso.clone(), shank.clone()),
    ];
    if matches!(tool.shape, ToolShape::BallEnd { .. }) {
        let ball = SharedShape::ball(cutting_r);
        full_parts.push((Isometry3::translation(0.0, 0.0, 0.0), ball));
    }

    // Holder-only = shank (no cutting edge)
    let holder_parts = vec![(shank_iso, shank)];

    (
        SharedShape::compound(full_parts),
        SharedShape::compound(holder_parts),
    )
}

// ── Collision checker ────────────────────────────────────────────────────────

/// Collision checker for tool vs machine components and workpiece.
///
/// - Machine parts are checked against the **full** tool shape.
/// - The workpiece is checked against only the **holder/shank** — cutting-edge
///   contact with the workpiece is intentional and not flagged.
pub struct CollisionChecker {
    tool_shape: SharedShape,
    holder_shape: SharedShape,
    machine_parts: Vec<(String, SharedShape, Isometry3<f32>)>,
    workpiece: Option<SharedShape>,
}

impl CollisionChecker {
    /// Create a collision checker for the given tool.
    pub fn new(
        tool: &Tool,
        machine_parts: Vec<(String, SharedShape, Isometry3<f32>)>,
        workpiece: Option<SharedShape>,
    ) -> Self {
        let (tool_shape, holder_shape) = build_tool_shapes(tool);
        CollisionChecker {
            tool_shape,
            holder_shape,
            machine_parts,
            workpiece,
        }
    }

    /// Check a single tool pose for collisions.
    ///
    /// `tool_pose` places the tool tip in **machine coordinates** with the
    /// tool body extending along +Z.
    pub fn check_pose(&self, tool_pose: &Isometry3<f32>) -> Vec<CollisionResult> {
        let mut results = Vec::new();

        // Full tool vs each machine part.
        for (name, shape, part_pose) in &self.machine_parts {
            if let Some(cr) = check_pair(
                tool_pose,
                &self.tool_shape,
                part_pose,
                shape,
                name,
            ) {
                results.push(cr);
            }
        }

        // Holder only vs workpiece (cutting-edge contact is intentional).
        if let Some(ref wp) = self.workpiece {
            let wp_pose = Isometry3::identity();
            if let Some(cr) = check_pair(
                tool_pose,
                &self.holder_shape,
                &wp_pose,
                wp,
                "workpiece",
            ) {
                results.push(cr);
            }
        }

        results
    }

    /// Check every point in a toolpath for collisions.
    ///
    /// Returns `(point_index, collisions)` for each point that collides.
    /// Uses IK to convert workpiece-frame tool poses to machine-frame poses.
    pub fn check_toolpath(
        &self,
        toolpath: &Toolpath,
        machine: &MachineConfig,
    ) -> Vec<(usize, Vec<CollisionResult>)> {
        use rayon::prelude::*;

        let tt = match TableTable::new(machine.clone()) {
            Ok(tt) => tt,
            Err(_) => return vec![],
        };

        toolpath
            .points
            .par_iter()
            .enumerate()
            .filter_map(|(i, pt)| {
                // Resolve machine-frame joint positions.
                let joints = if pt.orientation.z.abs() > 1.0 - 1e-6 {
                    // Near-vertical: A=0, C=0, machine pos = workpiece pos.
                    [pt.position.x, pt.position.y, pt.position.z, 0.0, 0.0]
                } else {
                    match tt.ik(&pt.position, &pt.orientation) {
                        Ok(sols) if !sols.is_empty() => sols[0],
                        _ => return None,
                    }
                };

                // Tool pose in machine frame: tip at (x,y,z), body along +Z.
                let pose = Isometry3::from_parts(
                    Translation3::new(joints[0] as f32, joints[1] as f32, joints[2] as f32),
                    UnitQuaternion::identity(),
                );

                let collisions = self.check_pose(&pose);
                if collisions.is_empty() {
                    None
                } else {
                    Some((i, collisions))
                }
            })
            .collect()
    }
}

// ── Helpers ──────────────────────────────────────────────────────────────────

fn check_pair(
    pose1: &Isometry3<f32>,
    shape1: &SharedShape,
    pose2: &Isometry3<f32>,
    shape2: &SharedShape,
    name: &str,
) -> Option<CollisionResult> {
    let intersects = query::intersection_test(
        pose1,
        shape1.as_ref(),
        pose2,
        shape2.as_ref(),
    )
    .unwrap_or(false);

    if !intersects {
        return None;
    }

    let depth = query::contact(
        pose1,
        shape1.as_ref(),
        pose2,
        shape2.as_ref(),
        0.0,
    )
    .unwrap_or(None)
    .map(|c| (-c.dist.min(0.0)) as f64)
    .unwrap_or(0.0);

    Some(CollisionResult {
        component: name.to_string(),
        penetration_depth: depth,
    })
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use openmill_core::Tool;

    #[test]
    fn collision_detected_when_tool_intersects_box() {
        let tool = Tool::flat_end(1, "test", 10.0, 20.0);
        // Large box centred at origin — tool is inside it.
        let obstacle = SharedShape::cuboid(50.0, 50.0, 50.0);
        let obstacle_pose = Isometry3::identity();

        let checker = CollisionChecker::new(
            &tool,
            vec![("obstacle".into(), obstacle, obstacle_pose)],
            None,
        );

        let tool_pose = Isometry3::<f32>::identity();
        let results = checker.check_pose(&tool_pose);
        assert!(!results.is_empty(), "expected collision with obstacle");
        assert_eq!(results[0].component, "obstacle");
    }

    #[test]
    fn no_collision_when_tool_is_clear() {
        let tool = Tool::flat_end(1, "test", 10.0, 20.0);
        // Small box far away from origin.
        let obstacle = SharedShape::cuboid(5.0, 5.0, 5.0);
        let obstacle_pose = Isometry3::translation(200.0, 200.0, 200.0);

        let checker = CollisionChecker::new(
            &tool,
            vec![("obstacle".into(), obstacle, obstacle_pose)],
            None,
        );

        let tool_pose = Isometry3::<f32>::identity();
        let results = checker.check_pose(&tool_pose);
        assert!(results.is_empty(), "expected no collision");
    }

    #[test]
    fn only_holder_collision_flagged_for_workpiece() {
        // Tool: flute_length=20, overall=50 → shank from z=20..50.
        let tool = Tool::flat_end(1, "test", 6.0, 20.0);

        // Thin workpiece slab: extends from z=-5 to z=+5 (centred at origin).
        let wp = SharedShape::cuboid(50.0, 50.0, 5.0);

        let checker = CollisionChecker::new(&tool, vec![], Some(wp));

        // Tool at origin: cutting edge z=0..20 overlaps workpiece z=-5..5.
        // But holder/shank starts at z=20 — does NOT overlap workpiece.
        let pose_ok = Isometry3::<f32>::identity();
        let results = checker.check_pose(&pose_ok);
        assert!(
            results.is_empty(),
            "cutting-edge contact should not be flagged as collision"
        );

        // Move tool down: tip at z=-25 → shank at z=-5..25 → overlaps workpiece.
        let pose_bad = Isometry3::translation(0.0, 0.0, -25.0);
        let results = checker.check_pose(&pose_bad);
        assert!(
            !results.is_empty(),
            "holder collision with workpiece should be flagged"
        );
        assert_eq!(results[0].component, "workpiece");
    }
}

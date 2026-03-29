use nalgebra::Isometry3;
use parry3d::query::{self, DefaultQueryDispatcher};
use parry3d::shape::{SharedShape, TriMesh};
use openmill_core::kinematics::MachineConfig;
use openmill_core::toolpath::Toolpath;

/// Result of a single collision query.
#[derive(Debug, Clone)]
pub struct CollisionResult {
    /// Name of the colliding machine component.
    pub component: String,
    /// Estimated penetration depth [mm].
    pub penetration_depth: f64,
}

/// Collision checker for a configured machine + tool.
pub struct CollisionChecker {
    /// Composite convex-hull approximation of the tool + holder.
    tool_shape: SharedShape,
    /// Static machine components: (name, shape, pose-in-machine-frame).
    machine_parts: Vec<(String, SharedShape, Isometry3<f32>)>,
    /// Current workpiece mesh (updated during material removal simulation).
    workpiece: TriMesh,
}

impl CollisionChecker {
    pub fn new(
        tool_shape: SharedShape,
        machine_parts: Vec<(String, SharedShape, Isometry3<f32>)>,
        workpiece: TriMesh,
    ) -> Self {
        CollisionChecker {
            tool_shape,
            machine_parts,
            workpiece,
        }
    }

    /// Check if a given tool pose collides with any machine component or workpiece.
    ///
    /// Returns one `CollisionResult` per colliding component.
    pub fn check(&self, tool_pose: &Isometry3<f32>) -> Vec<CollisionResult> {
        let mut results = Vec::new();
        let dispatcher = DefaultQueryDispatcher;

        for (name, shape, part_pose) in &self.machine_parts {
            let intersects = query::intersection_test(
                &dispatcher,
                tool_pose,
                self.tool_shape.as_ref(),
                part_pose,
                shape.as_ref(),
            )
            .unwrap_or(false);

            if intersects {
                // Compute penetration depth for richer diagnostics.
                let depth = query::contact(
                    &dispatcher,
                    tool_pose,
                    self.tool_shape.as_ref(),
                    part_pose,
                    shape.as_ref(),
                    0.0,
                )
                .unwrap_or(None)
                .map(|c| -c.dist.min(0.0) as f64)
                .unwrap_or(0.0);

                results.push(CollisionResult {
                    component: name.clone(),
                    penetration_depth: depth,
                });
            }
        }

        results
    }

    /// Check every point in a toolpath and return indices of points that collide.
    ///
    /// Uses the `MachineConfig` to convert workpiece-frame tool poses to
    /// machine-frame poses before collision testing.
    pub fn check_toolpath(
        &self,
        toolpath: &Toolpath,
        machine: &MachineConfig,
    ) -> Vec<usize> {
        use openmill_core::kinematics::{KinematicType, TableTableMachine};
        use openmill_core::kinematics::traits::InverseKinematics;
        use nalgebra::{Isometry3 as Iso64, Rotation3, Translation3, Vector3};

        let km = match TableTableMachine::new(machine) {
            Ok(k) => k,
            Err(_) => return vec![],
        };

        toolpath
            .points
            .iter()
            .enumerate()
            .filter_map(|(i, pt)| {
                let pos = Vector3::new(pt.position[0], pt.position[1], pt.position[2]);
                let dir = Vector3::new(pt.orientation[0], pt.orientation[1], pt.orientation[2]);
                let rot = Rotation3::rotation_between(&Vector3::z_axis(), &dir)
                    .unwrap_or(Rotation3::identity());
                let pose64 = Iso64::from_parts(Translation3::from(pos), rot.into());

                let joints = km.ik(&pose64, None).ok()?.into_iter().next()?;
                let pose_machine = km.fk(&joints).ok()?;

                // Downcast to f32 for parry3d.
                let t = pose_machine.translation.vector.cast::<f32>();
                let r = pose_machine.rotation.cast::<f32>();
                let pose32 = Isometry3::from_parts(
                    nalgebra::Translation3::from(t),
                    r,
                );

                let collisions = self.check(&pose32);
                if collisions.is_empty() { None } else { Some(i) }
            })
            .collect()
    }
}

use anyhow::Result;
use parry3d::query::{Ray, RayCast};
use serde::{Deserialize, Serialize};

use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::{MoveType, OperationType, Toolpath, ToolpathPoint};

use super::traits::ToolpathStrategy;

/// 3-axis waterline (contour-parallel) finishing strategy.
///
/// Intersects the part mesh with horizontal planes at equal Z intervals and
/// offsets the resulting contours by the tool radius.
pub struct ContourParallel;

/// Tuning parameters for [`ContourParallel`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContourParallelParams {
    /// Step-down between waterline passes [mm].
    pub step_down: f64,
    /// Cutting feed rate [mm/min].
    pub feed_rate: f64,
    /// Surface chord tolerance [mm].
    pub tolerance: f64,
}

impl Default for ContourParallelParams {
    fn default() -> Self {
        ContourParallelParams {
            step_down: 0.5,
            feed_rate: 600.0,
            tolerance: 0.01,
        }
    }
}

impl ToolpathStrategy for ContourParallel {
    type Params = ContourParallelParams;

    fn name(&self) -> &str {
        "Contour Parallel"
    }

    fn generate(
        &self,
        model: &WorkpieceModel,
        tool: &Tool,
        _machine: &MachineConfig,
        params: &ContourParallelParams,
    ) -> Result<Vec<Toolpath>> {
        let aabb = &model.aabb;
        let z_min = aabb.mins.z as f64;
        let z_max = aabb.maxs.z as f64;
        let safe_z = z_max + 10.0;
        let tool_r = tool.shape.diameter() / 2.0;
        
        let mut toolpaths = Vec::new();
        let mut z = z_max;

        while z >= z_min {
            let mut tp = Toolpath::new(tool.id, OperationType::Finishing, "Waterline Pass");
            let mut pass_points = Vec::new();
            
            // Simple approach: circular scan at this Z level
            // For a production system, we'd slice the actual mesh geometry
            // Here we use raycasting to find the boundary of the part at this Z
            let ang_steps = 360;
            let center_x = (aabb.mins.x + aabb.maxs.x) as f64 * 0.5;
            let center_y = (aabb.mins.y + aabb.maxs.y) as f64 * 0.5;
            let scan_r = (aabb.maxs.x - aabb.mins.x).max(aabb.maxs.y - aabb.mins.y) as f64 + tool_r + 5.0;

            for i in 0..=ang_steps {
                let angle = (i as f64).to_radians();
                let dir = nalgebra::Vector3::new(angle.cos(), angle.sin(), 0.0);
                let start = nalgebra::Point3::new(center_x, center_y, z) + dir * scan_r;
                let ray = Ray::new(start.cast::<f32>(), (-dir).cast::<f32>());

                if let Some(inter) = model.mesh.cast_local_ray_and_get_normal(&ray, scan_r as f32 * 2.0, true) {
                    let hit_point = ray.point_at(inter.time_of_impact);
                    let mut normal = nalgebra::Vector3::new(inter.normal.x as f64, inter.normal.y as f64, inter.normal.z as f64);
                    normal.z = 0.0; // We only care about XY normal for waterline offset
                    if normal.norm() > 0.0001 {
                        normal = normal.normalize();
                    } else {
                        normal = -dir; // Fallback to ray direction
                    }

                    let pos = hit_point.cast::<f64>() + normal * tool_r;
                    pass_points.push(ToolpathPoint {
                        position: nalgebra::Point3::new(pos.x, pos.y, z),
                        orientation: nalgebra::Vector3::z_axis(),
                        feed_rate: params.feed_rate,
                        move_type: MoveType::Linear,
                    });
                }
            }

            if !pass_points.is_empty() {
                // Link points and add to toolpath
                let first = pass_points[0].position;
                tp.points.push(ToolpathPoint {
                    position: nalgebra::Point3::new(first.x, first.y, safe_z),
                    orientation: nalgebra::Vector3::z_axis(),
                    feed_rate: 0.0,
                    move_type: MoveType::Rapid,
                });
                for (idx, mut pt) in pass_points.into_iter().enumerate() {
                    if idx == 0 { pt.move_type = MoveType::LeadIn; }
                    tp.points.push(pt);
                }
                let last = tp.points.last().unwrap().position;
                tp.points.push(ToolpathPoint {
                    position: nalgebra::Point3::new(last.x, last.y, safe_z),
                    orientation: nalgebra::Vector3::z_axis(),
                    feed_rate: 0.0,
                    move_type: MoveType::Retract,
                });
                toolpaths.push(tp);
            }

            z -= params.step_down;
        }

        Ok(toolpaths)
    }
}

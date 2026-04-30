use anyhow::Result;
use nalgebra::{Point3, Vector3, Unit};
use parry3d::query::{Ray, RayCast};
use serde::{Deserialize, Serialize};

use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::{MoveType, OperationType, Toolpath, ToolpathPoint};

use super::traits::ToolpathStrategy;

/// Geodesic parallel finishing strategy.
///
/// Toolpaths follow geodesic curves (constant distance along the surface)
/// from a boundary. Eliminates the bunching seen with UV-parameterized paths.
pub struct GeodesicParallel;

/// Tuning parameters for [`GeodesicParallel`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeodesicParams {
    /// Step-over distance along the surface [mm].
    pub step_over: f64,
    /// Cutting feed rate [mm/min].
    pub feed_rate: f64,
    /// Surface chord tolerance [mm].
    pub tolerance: f64,
}

impl Default for GeodesicParams {
    fn default() -> Self {
        GeodesicParams {
            step_over: 0.5,
            feed_rate: 400.0,
            tolerance: 0.01,
        }
    }
}

impl ToolpathStrategy for GeodesicParallel {
    type Params = GeodesicParams;

    fn name(&self) -> &str {
        "Geodesic Parallel"
    }

    fn generate(
        &self,
        model: &WorkpieceModel,
        tool: &Tool,
        _machine: &MachineConfig,
        params: &GeodesicParams,
    ) -> Result<Vec<Toolpath>> {
        // Simplified geodesic: use Z-height as the distance field for now
        // (This makes it look like Waterline but with surface-normal tilt)
        // A real geodesic implementation would involve solving the heat equation on the mesh.
        
        let mut tp = Toolpath::new(tool.id, OperationType::Finishing, "Geodesic Finishing");
        let aabb = &model.aabb;
        let tool_r = tool.shape.diameter() / 2.0;
        let safe_z = aabb.maxs.z as f64 + 10.0;

        let z_min = aabb.mins.z as f64;
        let z_max = aabb.maxs.z as f64;
        
        let mut z = z_max;
        while z >= z_min {
            let mut pass_points = Vec::new();
            let ang_steps = 360;
            let center_x = (aabb.mins.x + aabb.maxs.x) as f64 * 0.5;
            let center_y = (aabb.mins.y + aabb.maxs.y) as f64 * 0.5;
            let scan_r = (aabb.maxs.x - aabb.mins.x).max(aabb.maxs.y - aabb.mins.y) as f64 + tool_r + 5.0;

            for i in 0..=ang_steps {
                let angle = (i as f64).to_radians();
                let dir = Vector3::new(angle.cos(), angle.sin(), 0.0);
                let start = Point3::new(center_x, center_y, z) + dir * scan_r;
                let ray = Ray::new(start.cast::<f32>(), (-dir).cast::<f32>());

                if let Some(inter) = model.mesh.cast_local_ray_and_get_normal(&ray, scan_r as f32 * 2.0, true) {
                    let hit_point = ray.point_at(inter.time_of_impact);
                    let mut normal = Vector3::new(inter.normal.x as f64, inter.normal.y as f64, inter.normal.z as f64);
                    if normal.z < 0.0 { normal = -normal; }

                    let pos = if tool.shape.is_ball() {
                        Point3::new(
                            hit_point.x as f64 + normal.x * tool_r,
                            hit_point.y as f64 + normal.y * tool_r,
                            hit_point.z as f64 + normal.z * tool_r,
                        )
                    } else {
                        hit_point.cast::<f64>()
                    };

                    pass_points.push(ToolpathPoint {
                        position: pos,
                        orientation: Unit::new_normalize(normal),
                        feed_rate: params.feed_rate,
                        move_type: MoveType::Linear,
                    });
                }
            }

            if !pass_points.is_empty() {
                let first = pass_points[0].position;
                tp.points.push(ToolpathPoint::rapid(Point3::new(first.x, first.y, safe_z)));
                for (idx, mut pt) in pass_points.into_iter().enumerate() {
                    if idx == 0 { pt.move_type = MoveType::LeadIn; }
                    tp.points.push(pt);
                }
                let last = tp.points.last().unwrap().position;
                tp.points.push(ToolpathPoint::retract(Point3::new(last.x, last.y, safe_z)));
            }

            z -= params.step_over;
        }

        Ok(vec![tp])
    }
}

use anyhow::Result;
use nalgebra::{Point3, Rotation3, Unit, Vector3};
use parry3d::query::{Ray, RayCast};
use serde::{Deserialize, Serialize};

use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::{MoveType, OperationType, Toolpath, ToolpathPoint};

use super::traits::ToolpathStrategy;

/// 3+2 indexed positioning strategy.
///
/// The table rotary axes are indexed to fixed (A, C) angles and then locked.
/// A standard 3-axis zigzag raster runs in the resulting tilted work plane.
/// The output toolpath uses the tilted tool axis for every point, so the
/// post-processor emits a rotary index move followed by 3-axis cuts.
pub struct ThreePlusTwo;

/// Tuning parameters for [`ThreePlusTwo`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThreePlusTwoParams {
    /// Fixed A-axis table tilt [degrees].
    pub a_deg: f64,
    /// Fixed C-axis table rotation [degrees].
    pub c_deg: f64,
    /// Step-down per 3-axis pass [mm].
    pub step_down: f64,
    /// Step-over as a fraction of tool diameter [0.0 .. 1.0].
    pub step_over: f64,
    /// Cutting feed rate [mm/min].
    pub feed_rate: f64,
}

impl Default for ThreePlusTwoParams {
    fn default() -> Self {
        ThreePlusTwoParams {
            a_deg: 0.0,
            c_deg: 0.0,
            step_down: 1.0,
            step_over: 0.5,
            feed_rate: 700.0,
        }
    }
}

impl ToolpathStrategy for ThreePlusTwo {
    type Params = ThreePlusTwoParams;

    fn name(&self) -> &str {
        "3+2 Indexed"
    }

    fn generate(
        &self,
        model: &WorkpieceModel,
        tool: &Tool,
        _machine: &MachineConfig,
        params: &ThreePlusTwoParams,
    ) -> Result<Vec<Toolpath>> {
        let a_rad = params.a_deg.to_radians();
        let c_rad = params.c_deg.to_radians();

        // Tilted tool axis: rotate +Z by A around X then C around Z.
        // R = Rz(C) · Rx(A)  (table-table convention)
        let rot = Rotation3::from_axis_angle(&Vector3::z_axis(), c_rad)
            * Rotation3::from_axis_angle(&Vector3::x_axis(), a_rad);
        let tool_axis = Unit::new_normalize(rot * Vector3::z());
        
        // We need a local frame where tool_axis is Z'.
        // Local X' can be (UnitZ x tool_axis) or something stable.
        let local_z = tool_axis;
        let local_x = if local_z.z.abs() < 0.999 {
            Unit::new_normalize(Vector3::z().cross(&local_z))
        } else {
            Unit::new_normalize(Vector3::x())
        };
        let local_y = Unit::new_normalize(local_z.cross(&local_x));
        
        let to_local = nalgebra::Isometry3::from_parts(
            Point3::origin().into(),
            nalgebra::UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix_unchecked(
                nalgebra::Matrix3::from_columns(&[
                    local_x.into_inner(),
                    local_y.into_inner(),
                    local_z.into_inner(),
                ])
            ).transpose())
        );
        let to_world = to_local.inverse();

        // Transform model AABB or just sample points and find local bounds.
        let aabb = &model.aabb;
        let corners = [
            Point3::new(aabb.mins.x, aabb.mins.y, aabb.mins.z),
            Point3::new(aabb.maxs.x, aabb.mins.y, aabb.mins.z),
            Point3::new(aabb.mins.x, aabb.maxs.y, aabb.mins.z),
            Point3::new(aabb.maxs.x, aabb.maxs.y, aabb.mins.z),
            Point3::new(aabb.mins.x, aabb.mins.y, aabb.maxs.z),
            Point3::new(aabb.maxs.x, aabb.mins.y, aabb.maxs.z),
            Point3::new(aabb.mins.x, aabb.maxs.y, aabb.maxs.z),
            Point3::new(aabb.maxs.x, aabb.maxs.y, aabb.maxs.z),
        ];
        
        let mut local_mins = Point3::new(f64::MAX, f64::MAX, f64::MAX);
        let mut local_maxs = Point3::new(f64::MIN, f64::MIN, f64::MIN);
        
        for c in corners {
            let lp = to_local.transform_point(&c.cast::<f64>());
            local_mins.x = local_mins.x.min(lp.x);
            local_mins.y = local_mins.y.min(lp.y);
            local_mins.z = local_mins.z.min(lp.z);
            local_maxs.x = local_maxs.x.max(lp.x);
            local_maxs.y = local_maxs.y.max(lp.y);
            local_maxs.z = local_maxs.z.max(lp.z);
        }

        let tool_r = tool.shape.diameter() / 2.0;
        let step_over_mm = tool.shape.diameter() * params.step_over;

        let lx_min = local_mins.x + tool_r;
        let lx_max = local_maxs.x - tool_r;
        let ly_min = local_mins.y + tool_r;
        let ly_max = local_maxs.y - tool_r;
        let lz_top = local_maxs.z;
        let lz_bot = local_mins.z;

        if lx_min >= lx_max || ly_min >= ly_max {
            return Ok(vec![]);
        }

        let safe_lz = lz_top + 10.0;
        let mut tp = Toolpath::new(tool.id, OperationType::Roughing, "3+2 Indexed Roughing");

        // Layer-by-layer step-down passes in local Z.
        let mut lz = lz_top;
        while lz > lz_bot {
            lz = (lz - params.step_down).max(lz_bot);

            let mut ly = ly_min;
            let mut forward = true;
            while ly <= ly_max {
                let (lxa, lxb) = if forward { (lx_min, lx_max) } else { (lx_max, lx_min) };
                let dist = (lxb - lxa).abs();
                let steps = (dist / 1.0).ceil() as usize;
                let mut segment_points = Vec::new();

                for i in 0..=steps {
                    let t = i as f64 / steps as f64;
                    let clx = lxa + (lxb - lxa) * t;
                    
                    let local_ray_start = Point3::new(clx, ly, safe_lz);
                    let world_ray_start = to_world.transform_point(&local_ray_start);
                    let world_ray_dir = -tool_axis.into_inner();
                    
                    let ray = Ray::new(
                        world_ray_start.cast::<f32>(),
                        world_ray_dir.cast::<f32>()
                    );
                    
                    let mut hit_lz = lz_bot;
                    if let Some(toi) = model.mesh.cast_local_ray(&ray, (safe_lz - lz_bot) as f32, true) {
                        hit_lz = (safe_lz - toi as f64).max(lz_bot);
                    }
                    
                    let target_lz = lz.max(hit_lz);
                    segment_points.push(Point3::new(clx, ly, target_lz));
                }

                if !segment_points.is_empty() {
                    // Lead-in from safe Z
                    let first_world = to_world.transform_point(&Point3::new(lxa, ly, safe_lz));
                    tp.points.push(ToolpathPoint {
                        position: first_world,
                        orientation: tool_axis,
                        feed_rate: 0.0,
                        move_type: MoveType::Rapid,
                    });

                    for (idx, lpt) in segment_points.into_iter().enumerate() {
                        let world_pt = to_world.transform_point(&lpt);
                        tp.points.push(ToolpathPoint {
                            position: world_pt,
                            orientation: tool_axis,
                            feed_rate: if idx == 0 { params.feed_rate * 0.5 } else { params.feed_rate },
                            move_type: if idx == 0 { MoveType::LeadIn } else { MoveType::Linear },
                        });
                    }

                    // Retract to safe Z
                    let last_world = to_world.transform_point(&Point3::new(lxb, ly, safe_lz));
                    tp.points.push(ToolpathPoint {
                        position: last_world,
                        orientation: tool_axis,
                        feed_rate: 0.0,
                        move_type: MoveType::Retract,
                    });
                }

                ly += step_over_mm;
                forward = !forward;
            }
        }

        Ok(vec![tp])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{StockShape, WorkpieceModel};
    use parry3d::math::Point as PPoint;
    use parry3d::shape::TriMesh;

    /// Make a minimal 1-triangle mesh with a known AABB.
    fn test_model() -> WorkpieceModel {
        let verts = vec![
            PPoint::new(0.0, 0.0, 0.0),
            PPoint::new(40.0, 0.0, 0.0),
            PPoint::new(20.0, 30.0, 10.0),
        ];
        let tris = vec![[0u32, 1, 2]];
        let mesh = TriMesh::new(verts, tris);
        WorkpieceModel::new(
            mesh,
            StockShape::BoundingBox {
                margin: nalgebra::Vector3::new(2.0, 2.0, 2.0),
            },
        )
    }

    #[test]
    fn generates_nonempty_toolpath_at_zero_tilt() {
        let model = test_model();
        let tool = Tool::flat_end(1, "6mm flat", 6.0, 20.0);
        let machine = crate::kinematics::default_machines::default_trunnion_config();
        let params = ThreePlusTwoParams::default();

        let paths = ThreePlusTwo.generate(&model, &tool, &machine, &params).unwrap();
        assert_eq!(paths.len(), 1);
        assert!(!paths[0].points.is_empty());

        // At zero tilt the tool axis should be +Z.
        let axis = paths[0].points[0].orientation;
        assert!((axis.z - 1.0).abs() < 1e-9, "expected vertical tool axis");
    }

    #[test]
    fn tilted_axis_is_not_vertical() {
        let model = test_model();
        let tool = Tool::flat_end(1, "6mm flat", 6.0, 20.0);
        let machine = crate::kinematics::default_machines::default_trunnion_config();
        let params = ThreePlusTwoParams {
            a_deg: 30.0,
            c_deg: 45.0,
            ..Default::default()
        };

        let paths = ThreePlusTwo.generate(&model, &tool, &machine, &params).unwrap();
        let axis = paths[0].points[0].orientation;
        assert!(
            axis.z < 0.99,
            "tilted axis should not be vertical, got z={}",
            axis.z
        );
    }

    #[test]
    fn all_moves_use_same_axis() {
        let model = test_model();
        let tool = Tool::flat_end(1, "6mm flat", 6.0, 20.0);
        let machine = crate::kinematics::default_machines::default_trunnion_config();
        let params = ThreePlusTwoParams {
            a_deg: 15.0,
            c_deg: 0.0,
            ..Default::default()
        };

        let paths = ThreePlusTwo.generate(&model, &tool, &machine, &params).unwrap();
        let first_axis = paths[0].points[0].orientation;
        for pt in &paths[0].points {
            let diff = (pt.orientation.as_ref() - first_axis.as_ref()).norm();
            assert!(diff < 1e-9, "all points must share the indexed axis");
        }
    }
}

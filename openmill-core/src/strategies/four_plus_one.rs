use anyhow::Result;
use nalgebra::{Point3, Rotation3, Unit, Vector3};
use parry3d::query::{Ray, RayCast};
use serde::{Deserialize, Serialize};

use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::{MoveType, OperationType, Toolpath, ToolpathPoint};

use super::traits::ToolpathStrategy;

/// 4+1 indexed positioning strategy.
///
/// One rotary axis (A) is locked at a fixed tilt while the other (C) is
/// **stepped** between passes — each row of cuts is performed at a different
/// C angle.  This is distinct from 3+2 (both axes locked for the entire
/// operation) and from true simultaneous 5-axis (both axes moving
/// continuously).  4+1 is the typical strategy for wrapping operations around
/// a cylinder or for reaching multiple faces in a single setup without
/// re-fixturing.
///
/// Post-processor output: a rotary index (C only) before every row of cuts,
/// followed by a 3-axis linear raster in the instantaneous tilted plane.
pub struct FourPlusOne;

/// Tuning parameters for [`FourPlusOne`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FourPlusOneParams {
    /// Fixed A-axis table tilt (locked for the whole operation) [degrees].
    pub a_deg: f64,

    /// Starting C angle for the first pass [degrees].
    pub c_start_deg: f64,

    /// C angle increment between adjacent passes [degrees].
    /// A positive value rotates the table clockwise (machine convention).
    pub c_step_deg: f64,

    /// Total angular range swept by C across all passes [degrees].
    /// Actual pass count = ceil(c_range_deg / c_step_deg) + 1.
    pub c_range_deg: f64,

    /// Step-down per layer [mm].
    pub step_down: f64,

    /// Step-over as a fraction of tool diameter [0.0 .. 1.0].
    pub step_over: f64,

    /// Cutting feed rate [mm/min].
    pub feed_rate: f64,
}

impl Default for FourPlusOneParams {
    fn default() -> Self {
        FourPlusOneParams {
            a_deg: 0.0,
            c_start_deg: 0.0,
            c_step_deg: 45.0,
            c_range_deg: 360.0,
            step_down: 1.0,
            step_over: 0.5,
            feed_rate: 700.0,
        }
    }
}

impl ToolpathStrategy for FourPlusOne {
    type Params = FourPlusOneParams;

    fn name(&self) -> &str {
        "4+1 Indexed"
    }

    fn generate(
        &self,
        model: &WorkpieceModel,
        tool: &Tool,
        _machine: &MachineConfig,
        params: &FourPlusOneParams,
    ) -> Result<Vec<Toolpath>> {
        let a_rad = params.a_deg.to_radians();

        // Number of C-index positions.
        let pass_count = if params.c_step_deg.abs() < 1e-9 {
            1usize
        } else {
            ((params.c_range_deg / params.c_step_deg).abs().ceil() as usize).max(1)
        };

        let tool_r = tool.shape.diameter() / 2.0;
        let step_over_mm = tool.shape.diameter() * params.step_over;

        let mut all_tps: Vec<Toolpath> = Vec::new();

        for pass_idx in 0..pass_count {
            let c_deg = params.c_start_deg + pass_idx as f64 * params.c_step_deg;
            let c_rad = c_deg.to_radians();

            // Tilted tool axis: Rz(C) · Rx(A) · +Z
            let rot = Rotation3::from_axis_angle(&Vector3::z_axis(), c_rad)
                * Rotation3::from_axis_angle(&Vector3::x_axis(), a_rad);
            let tool_axis = Unit::new_normalize(rot * Vector3::z());

            // Build a local frame where tool_axis is Z'.
            let local_z = tool_axis;
            let local_x = if local_z.z.abs() < 0.999 {
                Unit::new_normalize(Vector3::z().cross(&local_z))
            } else {
                Unit::new_normalize(Vector3::x())
            };
            let local_y = Unit::new_normalize(local_z.cross(&local_x));

            let rot_mat = nalgebra::Rotation3::from_matrix_unchecked(
                nalgebra::Matrix3::from_columns(&[
                    local_x.into_inner(),
                    local_y.into_inner(),
                    local_z.into_inner(),
                ]),
            );
            let to_local = nalgebra::Isometry3::from_parts(
                Point3::origin().into(),
                nalgebra::UnitQuaternion::from_rotation_matrix(&rot_mat).inverse(),
            );
            let to_world = to_local.inverse();

            // Project the **stock** AABB (part AABB grown by margin) into the
            // local frame so the layered passes actually clear the outer stock
            // material.
            let stock_aabb = model.stock_aabb();
            let aabb = &stock_aabb;
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

            let lx_min = local_mins.x + tool_r;
            let lx_max = local_maxs.x - tool_r;
            let ly_min = local_mins.y + tool_r;
            let ly_max = local_maxs.y - tool_r;
            let lz_top = local_maxs.z;
            let lz_bot = local_mins.z;

            if lx_min >= lx_max || ly_min >= ly_max {
                continue;
            }

            let safe_lz = lz_top + 10.0;

            let tp_name = format!(
                "4+1 Indexed — A={:.1}° C={:.1}°",
                params.a_deg, c_deg
            );
            let mut tp = Toolpath::new(tool.id, OperationType::Roughing, tp_name);

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
                    let mut segment_pts: Vec<Point3<f64>> = Vec::new();

                    for i in 0..=steps {
                        let t = i as f64 / steps as f64;
                        let clx = lxa + (lxb - lxa) * t;

                        let local_ray_start = Point3::new(clx, ly, safe_lz);
                        let world_ray_start = to_world.transform_point(&local_ray_start);
                        let world_ray_dir = -tool_axis.into_inner();

                        let ray = Ray::new(
                            world_ray_start.cast::<f32>(),
                            world_ray_dir.cast::<f32>(),
                        );

                        let mut hit_lz = lz_bot;
                        if let Some(toi) =
                            model.mesh.cast_local_ray(&ray, (safe_lz - lz_bot) as f32, true)
                        {
                            hit_lz = (safe_lz - toi as f64).max(lz_bot);
                        }

                        let target_lz = lz.max(hit_lz);
                        segment_pts.push(Point3::new(clx, ly, target_lz));
                    }

                    if !segment_pts.is_empty() {
                        // Rapid to safe Z before first cut.
                        let first_world =
                            to_world.transform_point(&Point3::new(lxa, ly, safe_lz));
                        tp.points.push(ToolpathPoint {
                            position: first_world,
                            orientation: tool_axis,
                            feed_rate: 0.0,
                            move_type: MoveType::Rapid,
                        });

                        for (idx, lpt) in segment_pts.into_iter().enumerate() {
                            let world_pt = to_world.transform_point(&lpt);
                            tp.points.push(ToolpathPoint {
                                position: world_pt,
                                orientation: tool_axis,
                                feed_rate: if idx == 0 {
                                    params.feed_rate * 0.5
                                } else {
                                    params.feed_rate
                                },
                                move_type: if idx == 0 {
                                    MoveType::LeadIn
                                } else {
                                    MoveType::Linear
                                },
                            });
                        }

                        // Retract to safe Z.
                        let last_world =
                            to_world.transform_point(&Point3::new(lxb, ly, safe_lz));
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

            if !tp.is_empty() {
                all_tps.push(tp);
            }
        }

        Ok(all_tps)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{StockShape, WorkpieceModel};
    use parry3d::math::Point as PPoint;
    use parry3d::shape::TriMesh;

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
    fn generates_one_toolpath_per_c_index() {
        let model = test_model();
        let tool = Tool::flat_end(1, "6mm flat", 6.0, 20.0);
        let machine = crate::kinematics::default_machines::default_trunnion_config();
        let params = FourPlusOneParams {
            c_start_deg: 0.0,
            c_step_deg: 90.0,
            c_range_deg: 360.0,
            ..Default::default()
        };

        let paths = FourPlusOne.generate(&model, &tool, &machine, &params).unwrap();
        // 360 / 90 = 4 passes (indices 0°, 90°, 180°, 270°)
        assert_eq!(paths.len(), 4);
        for tp in &paths {
            assert!(!tp.points.is_empty(), "each C-index pass should produce points");
        }
    }

    #[test]
    fn each_pass_has_unique_tool_axis() {
        let model = test_model();
        let tool = Tool::flat_end(1, "6mm flat", 6.0, 20.0);
        let machine = crate::kinematics::default_machines::default_trunnion_config();
        let params = FourPlusOneParams {
            a_deg: 0.0,
            c_start_deg: 0.0,
            c_step_deg: 45.0,
            c_range_deg: 90.0, // 3 passes: 0°, 45°, 90°
            ..Default::default()
        };

        let paths = FourPlusOne.generate(&model, &tool, &machine, &params).unwrap();
        assert!(paths.len() >= 2);

        // Adjacent passes must have distinct tool axes (different C angles → different axis).
        let axis0 = paths[0].points[0].orientation;
        let axis1 = paths[1].points[0].orientation;
        let diff = (axis0.as_ref() - axis1.as_ref()).norm();
        assert!(diff > 1e-6, "adjacent C passes must have different tool axes");
    }

    #[test]
    fn within_pass_all_points_share_same_axis() {
        let model = test_model();
        let tool = Tool::flat_end(1, "6mm flat", 6.0, 20.0);
        let machine = crate::kinematics::default_machines::default_trunnion_config();
        let params = FourPlusOneParams {
            c_step_deg: 90.0,
            c_range_deg: 90.0,
            ..Default::default()
        };

        let paths = FourPlusOne.generate(&model, &tool, &machine, &params).unwrap();
        for tp in &paths {
            let first_axis = tp.points[0].orientation;
            for pt in &tp.points {
                let diff = (pt.orientation.as_ref() - first_axis.as_ref()).norm();
                assert!(diff < 1e-9, "all points in a 4+1 pass must share the C-indexed axis");
            }
        }
    }
}

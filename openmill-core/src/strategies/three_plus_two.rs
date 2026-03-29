use anyhow::Result;
use nalgebra::{Point3, Rotation3, Unit, Vector3};
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

        // Stock AABB in workpiece coordinates.
        let aabb = &model.aabb;
        let mins = aabb.mins;
        let maxs = aabb.maxs;

        let tool_r = tool.shape.diameter() / 2.0;
        let step_over_mm = tool.shape.diameter() * params.step_over;

        // Work-plane local axes (X' along stock X, Y' as step-over direction).
        // For the tilted plane we raster in world XY and step in Z layers.
        let x_min = mins.x as f64 + tool_r;
        let x_max = maxs.x as f64 - tool_r;
        let y_min = mins.y as f64 + tool_r;
        let y_max = maxs.y as f64 - tool_r;
        let z_top = maxs.z as f64;
        let z_bot = mins.z as f64;

        if x_min >= x_max || y_min >= y_max || z_top <= z_bot {
            return Ok(vec![]);
        }

        let safe_z = z_top + 5.0;
        let mut tp = Toolpath::new(tool.id, OperationType::Roughing, "3+2 Indexed Roughing");

        // Start with retract to safe height.
        tp.points.push(ToolpathPoint {
            position: Point3::new(x_min, y_min, safe_z),
            orientation: tool_axis,
            feed_rate: 0.0,
            move_type: MoveType::Rapid,
        });

        // Layer-by-layer step-down passes.
        let mut z = z_top;
        while z > z_bot {
            z = (z - params.step_down).max(z_bot);

            // Zigzag raster rows along X, stepping over in Y.
            let mut y = y_min;
            let mut forward = true;
            while y <= y_max {
                let (xa, xb) = if forward {
                    (x_min, x_max)
                } else {
                    (x_max, x_min)
                };

                // Rapid to row start at safe Z.
                tp.points.push(ToolpathPoint {
                    position: Point3::new(xa, y, safe_z),
                    orientation: tool_axis,
                    feed_rate: 0.0,
                    move_type: MoveType::Rapid,
                });
                // Plunge to cutting depth.
                tp.points.push(ToolpathPoint {
                    position: Point3::new(xa, y, z),
                    orientation: tool_axis,
                    feed_rate: params.feed_rate * 0.5,
                    move_type: MoveType::LeadIn,
                });
                // Cut across.
                tp.points.push(ToolpathPoint {
                    position: Point3::new(xb, y, z),
                    orientation: tool_axis,
                    feed_rate: params.feed_rate,
                    move_type: MoveType::Linear,
                });
                // Retract.
                tp.points.push(ToolpathPoint {
                    position: Point3::new(xb, y, safe_z),
                    orientation: tool_axis,
                    feed_rate: 0.0,
                    move_type: MoveType::Retract,
                });

                y += step_over_mm;
                forward = !forward;
            }
        }

        // Final retract.
        tp.points.push(ToolpathPoint {
            position: Point3::new(x_min, y_min, safe_z),
            orientation: tool_axis,
            feed_rate: 0.0,
            move_type: MoveType::Rapid,
        });

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

use anyhow::Result;
use nalgebra::{Point3, Rotation3, Unit, Vector3};
use parry3d::query::{Ray, RayCast};
use serde::{Deserialize, Serialize};

use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::{MoveType, OperationType, Toolpath, ToolpathPoint};

use super::traits::ToolpathStrategy;

/// Simultaneous 5-axis roughing strategy.
///
/// This strategy performs stock-aware layered roughing. For each point in the
/// raster, it raycasts to the underlying part surface to determine the local
/// surface normal, then tilts the tool axis to align with that normal (with
/// optional lead/tilt offsets).
pub struct MultiAxisRoughing;

/// Tuning parameters for [`MultiAxisRoughing`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultiAxisRoughingParams {
    /// Step-down per pass [mm].
    pub step_down: f64,
    /// Step-over as a fraction of tool diameter [0.0 .. 1.0].
    pub step_over: f64,
    /// Cutting feed rate [mm/min].
    pub feed_rate: f64,
    /// Stock to leave on the part surface [mm].
    #[serde(default)]
    pub stock_to_leave: f64,
    /// Maximum allowed tilt angle from vertical [+Z] in degrees.
    /// Prevents excessive machine rotation in steep areas.
    #[serde(default = "default_max_tilt")]
    pub max_tilt_deg: f64,
    /// Lead angle (tilt in direction of motion) [degrees].
    #[serde(default)]
    pub lead_angle: f64,
    #[serde(default)]
    pub direction: crate::strategies::CutDirection,
    #[serde(default)]
    pub z_range: crate::strategies::ZRange,
    #[serde(default)]
    pub spring_pass: crate::strategies::SpringPass,
}

fn default_max_tilt() -> f64 {
    45.0
}

impl Default for MultiAxisRoughingParams {
    fn default() -> Self {
        MultiAxisRoughingParams {
            step_down: 2.0,
            step_over: 0.5,
            feed_rate: 800.0,
            stock_to_leave: 0.5,
            max_tilt_deg: 45.0,
            lead_angle: 0.0,
            direction: crate::strategies::CutDirection::Climb,
            z_range: crate::strategies::ZRange::default(),
            spring_pass: crate::strategies::SpringPass::default(),
        }
    }
}

impl ToolpathStrategy for MultiAxisRoughing {
    type Params = MultiAxisRoughingParams;

    fn name(&self) -> &str {
        "Simultaneous 5-Axis Roughing"
    }

    fn generate(
        &self,
        model: &WorkpieceModel,
        tool: &Tool,
        _machine: &MachineConfig,
        params: &MultiAxisRoughingParams,
    ) -> Result<Vec<Toolpath>> {
        let stock = model.stock_aabb();
        let part = &model.aabb;

        let tool_r = tool.shape.diameter() / 2.0;
        let step_over_mm = (tool.shape.diameter() * params.step_over).max(0.1);

        let x_min = stock.mins.x as f64 + tool_r;
        let x_max = stock.maxs.x as f64 - tool_r;
        let y_min = stock.mins.y as f64 + tool_r;
        let y_max = stock.maxs.y as f64 - tool_r;
        let z_top = stock.maxs.z as f64;
        let z_bot = part.mins.z as f64;

        if x_min >= x_max || y_min >= y_max || z_top <= z_bot {
            return Ok(vec![Toolpath::new(
                tool.id,
                OperationType::Roughing,
                "Multi-Axis Roughing (no material)",
            )]);
        }

        let mut tp = Toolpath::new(tool.id, OperationType::Roughing, "Simultaneous 5-Axis Roughing");
        let safe_z = z_top + 10.0;
        let max_tilt_rad = params.max_tilt_deg.to_radians();

        // Layer-by-layer step-down
        let mut z = z_top;
        while z > z_bot {
            z = (z - params.step_down).max(z_bot);

            let mut y = y_min;
            let mut forward = true;
            while y <= y_max {
                let (xa, xb) = if forward { (x_min, x_max) } else { (x_max, x_min) };
                let dist = (xb - xa).abs();
                let steps = (dist / 1.0).ceil() as usize;
                let mut pass_points = Vec::new();

                for i in 0..=steps {
                    let t = i as f64 / steps as f64;
                    let x = xa + (xb - xa) * t;

                    // Raycast straight down to find the surface normal
                    let ray_start = Point3::new(x as f32, y as f32, safe_z as f32);
                    let ray_dir = Vector3::new(0.0, 0.0, -1.0_f32);
                    let ray = Ray::new(ray_start, ray_dir);

                    let mut surface_z = z_bot;
                    let mut orientation = Unit::new_normalize(Vector3::z());

                    if let Some(inter) = model.mesh.cast_local_ray_and_get_normal(&ray, (safe_z - z_bot) as f32, true) {
                        surface_z = (safe_z - inter.time_of_impact as f64).max(z_bot);
                        let mut normal = Vector3::new(inter.normal.x as f64, inter.normal.y as f64, inter.normal.z as f64);
                        if normal.z < 0.0 { normal = -normal; }

                        // Limit tilt angle
                        let vertical = Vector3::z();
                        let angle = normal.dot(&vertical).acos();
                        if angle > max_tilt_rad {
                            let axis = vertical.cross(&normal).normalize();
                            let rot = Rotation3::from_axis_angle(&Unit::new_normalize(axis), max_tilt_rad);
                            normal = rot * vertical;
                        }
                        orientation = Unit::new_normalize(normal);
                    }

                    // Apply lead angle if any
                    if params.lead_angle != 0.0 {
                        let direction = if forward { 1.0 } else { -1.0 };
                        let travel_vec = Vector3::new(direction, 0.0, 0.0);
                        let tilt_axis = if travel_vec.cross(&orientation.into_inner()).norm() > 0.001 {
                            Unit::new_normalize(travel_vec.cross(&orientation.into_inner()))
                        } else {
                            Unit::new_normalize(Vector3::y())
                        };
                        let lead_rot = Rotation3::from_axis_angle(&tilt_axis, -params.lead_angle.to_radians() * direction);
                        orientation = Unit::new_normalize(lead_rot * orientation.into_inner());
                    }

                    let target_z = z.max(surface_z + params.stock_to_leave);
                    pass_points.push(ToolpathPoint {
                        position: Point3::new(x, y, target_z),
                        orientation,
                        feed_rate: params.feed_rate,
                        move_type: MoveType::Linear,
                    });
                }

                if !pass_points.is_empty() {
                    // Lead-in
                    let first = pass_points[0].position;
                    tp.points.push(ToolpathPoint {
                        position: Point3::new(first.x, first.y, safe_z),
                        orientation: pass_points[0].orientation,
                        feed_rate: 0.0,
                        move_type: MoveType::Rapid,
                    });
                    
                    for (idx, mut pt) in pass_points.into_iter().enumerate() {
                        if idx == 0 { pt.move_type = MoveType::LeadIn; }
                        tp.points.push(pt);
                    }

                    // Retract
                    let last = tp.points.last().unwrap().position;
                    tp.points.push(ToolpathPoint {
                        position: Point3::new(last.x, last.y, safe_z),
                        orientation: tp.points.last().unwrap().orientation,
                        feed_rate: 0.0,
                        move_type: MoveType::Retract,
                    });
                }

                y += step_over_mm;
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

    /// Build a sloped triangle mesh so surface normals aren't all vertical.
    fn test_model() -> WorkpieceModel {
        let verts = vec![
            PPoint::new(0.0, 0.0, 0.0),
            PPoint::new(40.0, 0.0, 0.0),
            PPoint::new(20.0, 30.0, 10.0),
        ];
        let tris = vec![[0u32, 1, 2]];
        WorkpieceModel::new(
            TriMesh::new(verts, tris),
            StockShape::BoundingBox {
                margin: nalgebra::Vector3::new(5.0, 5.0, 5.0),
            },
        )
    }

    #[test]
    fn does_not_panic_and_produces_points() {
        let model = test_model();
        let tool = Tool::flat_end(1, "6mm flat", 6.0, 20.0);
        let machine = crate::kinematics::default_machines::default_trunnion_config();
        let paths = MultiAxisRoughing
            .generate(&model, &tool, &machine, &MultiAxisRoughingParams::default())
            .expect("should not error on a normal model");
        assert_eq!(paths.len(), 1);
        assert!(!paths[0].points.is_empty(), "should produce raster points");
    }

    #[test]
    fn raster_uses_stock_extent_not_part_extent() {
        let model = test_model();
        let tool = Tool::flat_end(1, "6mm flat", 6.0, 20.0);
        let machine = crate::kinematics::default_machines::default_trunnion_config();
        let paths = MultiAxisRoughing
            .generate(&model, &tool, &machine, &MultiAxisRoughingParams::default())
            .unwrap();
        // Stock margin = 5 → stock extends past the part footprint.
        let any_outside_part = paths[0].points.iter().any(|p| p.position.x < 0.0);
        assert!(
            any_outside_part,
            "raster should reach into the stock margin (x < part_min_x)"
        );
    }

    #[test]
    fn tool_axes_are_not_all_vertical() {
        // The sloped triangle mesh should produce normals that tilt the tool
        // away from pure +Z on at least some points.
        let model = test_model();
        let tool = Tool::flat_end(1, "6mm flat", 6.0, 20.0);
        let machine = crate::kinematics::default_machines::default_trunnion_config();
        let paths = MultiAxisRoughing
            .generate(&model, &tool, &machine, &MultiAxisRoughingParams::default())
            .unwrap();
        let any_tilted = paths[0]
            .points
            .iter()
            .any(|p| p.orientation.z < 0.999);
        assert!(
            any_tilted,
            "at least some toolpath points should have a tilted axis"
        );
    }

    #[test]
    fn operation_type_is_roughing() {
        let model = test_model();
        let tool = Tool::flat_end(1, "6mm flat", 6.0, 20.0);
        let machine = crate::kinematics::default_machines::default_trunnion_config();
        let paths = MultiAxisRoughing
            .generate(&model, &tool, &machine, &MultiAxisRoughingParams::default())
            .unwrap();
        assert_eq!(
            paths[0].operation,
            crate::toolpath::OperationType::Roughing,
            "strategy must emit Roughing operation type"
        );
    }

    #[test]
    fn params_serde_roundtrip() {
        let params = MultiAxisRoughingParams::default();
        let json = serde_json::to_string(&params).unwrap();
        let decoded: MultiAxisRoughingParams = serde_json::from_str(&json).unwrap();
        assert!((decoded.step_down - params.step_down).abs() < 1e-9);
        assert!((decoded.max_tilt_deg - 45.0).abs() < 1e-9);
    }

    #[test]
    fn params_deserialize_with_missing_optional_fields() {
        // Simulates opening a job saved before max_tilt_deg / lead_angle
        // existed — serde(default) must fill in sensible values.
        let json = r#"{"step_down":2.0,"step_over":0.5,"feed_rate":800.0}"#;
        let decoded: MultiAxisRoughingParams = serde_json::from_str(json).unwrap();
        assert!((decoded.max_tilt_deg - 45.0).abs() < 1e-9, "max_tilt_deg default");
        assert!((decoded.stock_to_leave - 0.0).abs() < 1e-9, "stock_to_leave default");
        assert!((decoded.lead_angle - 0.0).abs() < 1e-9, "lead_angle default");
    }
}

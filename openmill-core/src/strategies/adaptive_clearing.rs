use anyhow::Result;
use nalgebra::{Point3, Vector3};
use parry3d::query::{Ray, RayCast};
use serde::{Deserialize, Serialize};

use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::{MoveType, OperationType, Toolpath, ToolpathPoint};

use super::traits::ToolpathStrategy;

/// 3-axis adaptive (trochoidal) roughing strategy.
///
/// **Status:** this is a starter implementation — a stock-aware layered raster
/// pocket-clear with the step-over capped by the engagement-angle parameter.
/// True trochoidal medial-axis paths require a 2D voronoi/medial-axis solver
/// and aren't implemented yet; the current output is functionally similar to
/// a 3-axis zigzag rough at zero tilt, but it now respects stock dimensions
/// (so the outer margin actually gets cleared) and produces valid G-code
/// instead of panicking.
pub struct AdaptiveClearing;

/// Tuning parameters for [`AdaptiveClearing`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AdaptiveClearingParams {
    /// Maximum tool engagement angle [degrees]. Used to derive the lateral
    /// step-over via `WoC = D · sin(engagement / 2)`.
    pub max_engagement_deg: f64,
    /// Step-down per pass [mm].
    pub step_down: f64,
    /// Cutting feed rate [mm/min].
    pub feed_rate: f64,
    /// Stock to leave on the part surface [mm] — keeps cutting moves this
    /// far above the actual mesh so a finishing op can skim down to size.
    #[serde(default)]
    pub stock_to_leave: f64,
    /// Cut direction. `Climb` (default) emits the strategy's native
    /// direction; `Conventional` reverses every cutting block as a
    /// post-pass.
    #[serde(default)]
    pub direction: crate::strategies::CutDirection,
    /// Optional top / bottom Z limits. Cutting points outside the range
    /// are dropped at post-pass time.
    #[serde(default)]
    pub z_range: crate::strategies::ZRange,
    #[serde(default)]
    pub spring_pass: crate::strategies::SpringPass,
}

impl Default for AdaptiveClearingParams {
    fn default() -> Self {
        AdaptiveClearingParams {
            max_engagement_deg: 60.0,
            step_down: 2.0,
            feed_rate: 800.0,
            stock_to_leave: 0.0,
            direction: crate::strategies::CutDirection::Climb,
            z_range: crate::strategies::ZRange::default(),
            spring_pass: crate::strategies::SpringPass::default(),
        }
    }
}

impl ToolpathStrategy for AdaptiveClearing {
    type Params = AdaptiveClearingParams;

    fn name(&self) -> &str {
        "Adaptive Clearing"
    }

    fn generate(
        &self,
        model: &WorkpieceModel,
        tool: &Tool,
        _machine: &MachineConfig,
        params: &AdaptiveClearingParams,
    ) -> Result<Vec<Toolpath>> {
        let stock = model.stock_aabb();
        let part  = &model.aabb;

        let tool_d = tool.shape.diameter();
        let tool_r = tool_d / 2.0;
        // WoC ≈ D · sin(θ/2) where θ is the tool engagement angle.
        let engagement_rad = params.max_engagement_deg.to_radians().clamp(1.0_f64.to_radians(), 180.0_f64.to_radians());
        let step_over_mm = (tool_d * (engagement_rad * 0.5).sin()).max(tool_d * 0.05);

        // Raster bounds — keep the cutter inside the stock perimeter.
        let x_min = stock.mins.x as f64 + tool_r;
        let x_max = stock.maxs.x as f64 - tool_r;
        let y_min = stock.mins.y as f64 + tool_r;
        let y_max = stock.maxs.y as f64 - tool_r;
        let z_top = stock.maxs.z as f64;
        let z_bot = part.mins.z  as f64;

        if x_min >= x_max || y_min >= y_max || z_top <= z_bot {
            return Ok(vec![Toolpath::new(
                tool.id,
                OperationType::Roughing,
                "Adaptive Clearing (no material to remove)",
            )]);
        }

        let safe_z = z_top + 10.0;
        let mut tp = Toolpath::new(tool.id, OperationType::Roughing, "Adaptive Clearing");

        // Layer-by-layer step-down through the stock down to the part.
        let step_down = params.step_down.max(0.05);
        let mut z = z_top;
        while z > z_bot {
            z = (z - step_down).max(z_bot);

            let mut y = y_min;
            let mut forward = true;
            while y <= y_max {
                let (xa, xb) = if forward { (x_min, x_max) } else { (x_max, x_min) };
                let dist = (xb - xa).abs();
                let steps = (dist / 1.0).ceil().max(1.0) as usize;
                let mut row: Vec<Point3<f64>> = Vec::new();

                for i in 0..=steps {
                    let t = i as f64 / steps as f64;
                    let x = xa + (xb - xa) * t;

                    // Raycast straight down from above to find the highest
                    // mesh hit at this XY. Layer Z is clamped to that hit
                    // so we don't gouge the part.
                    let ray = Ray::new(
                        Point3::new(x as f32, y as f32, safe_z as f32),
                        Vector3::new(0.0, 0.0, -1.0_f32),
                    );
                    let mut part_top = z_bot;
                    if let Some(toi) = model.mesh.cast_local_ray(&ray, (safe_z - z_bot) as f32, true) {
                        part_top = (safe_z - toi as f64).max(z_bot);
                    }
                    // Hold cutting moves `stock_to_leave` mm above the mesh
                    // surface so a finishing op can skim down to size.
                    let part_top_with_skin = part_top + params.stock_to_leave.max(0.0);
                    let target_z = z.max(part_top_with_skin);
                    row.push(Point3::new(x, y, target_z));
                }

                if !row.is_empty() {
                    // Rapid to safe Z above the row start.
                    tp.points.push(ToolpathPoint {
                        position: Point3::new(xa, y, safe_z),
                        orientation: Vector3::z_axis(),
                        feed_rate: 0.0,
                        move_type: MoveType::Rapid,
                    });
                    for (idx, p) in row.into_iter().enumerate() {
                        tp.points.push(ToolpathPoint {
                            position: p,
                            orientation: Vector3::z_axis(),
                            feed_rate: if idx == 0 { params.feed_rate * 0.5 } else { params.feed_rate },
                            move_type: if idx == 0 { MoveType::LeadIn } else { MoveType::Linear },
                        });
                    }
                    // Retract from the row end.
                    tp.points.push(ToolpathPoint {
                        position: Point3::new(xb, y, safe_z),
                        orientation: Vector3::z_axis(),
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
        let paths = AdaptiveClearing
            .generate(&model, &tool, &machine, &AdaptiveClearingParams::default())
            .expect("AdaptiveClearing should not error on a normal model");
        assert_eq!(paths.len(), 1);
        assert!(!paths[0].points.is_empty(), "should produce raster points");
    }

    #[test]
    fn raster_uses_stock_extent_not_part_extent() {
        // Part AABB is roughly x:0..40, y:0..30. Stock margin = 5 → stock extends to x:-5..45.
        // The raster should put points at X less than 0 (outside the part footprint).
        let model = test_model();
        let tool = Tool::flat_end(1, "6mm flat", 6.0, 20.0);
        let machine = crate::kinematics::default_machines::default_trunnion_config();
        let paths = AdaptiveClearing
            .generate(&model, &tool, &machine, &AdaptiveClearingParams::default())
            .unwrap();
        let any_outside_part = paths[0].points.iter().any(|p| p.position.x < 0.0);
        assert!(any_outside_part, "raster should reach into the stock margin (x < part_min_x)");
    }
}

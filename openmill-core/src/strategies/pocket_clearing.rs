use anyhow::Result;
use nalgebra::{Point3, Vector3};
use parry3d::query::{Ray, RayCast};
use serde::{Deserialize, Serialize};

use crate::feature::{Feature, FeatureKind};
use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::{MoveType, OperationType, Toolpath, ToolpathPoint};

use super::traits::ToolpathStrategy;

/// Pocket clearing strategy.
///
/// Drives a 3-axis raster pocket-clear over each [`FeatureKind::Pocket`]
/// feature. The pocket's open-top Z marks the "stock surface above the
/// pocket" and the floor Z marks the bottom of the cut. Each XY raster
/// position raycasts down to find the nearest mesh surface, clamping the
/// layer Z so the cutter doesn't gouge into walls or the floor's edges.
///
/// Use `Adaptive Clearing` for a full-stock-volume rough; `Pocket Clearing`
/// is for cases where you've identified a specific cavity (or several) on
/// the part and want to clear them as a discrete operation.
pub struct PocketClearing;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PocketClearingParams {
    /// Pocket features to clear. Populate from the Features panel via
    /// "→ Pocket Clearing".
    pub pockets: Vec<PocketRef>,
    /// Step-down per pass [mm].
    pub step_down: f64,
    /// Step-over as a fraction of tool diameter `[0.0..1.0]`.
    pub step_over: f64,
    /// Cutting feed rate [mm/min].
    pub feed_rate: f64,
    /// Margin to leave around the floor and walls [mm] (positive = leave more
    /// stock for a finishing pass).
    pub stock_to_leave: f64,
    #[serde(default)]
    pub direction: crate::strategies::CutDirection,
    #[serde(default)]
    pub z_range: crate::strategies::ZRange,
    #[serde(default)]
    pub spring_pass: crate::strategies::SpringPass,
}

/// Snapshot of a pocket feature embedded in the op params so the op can be
/// regenerated even after the source feature is edited or deleted.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PocketRef {
    pub center:  Point3<f64>,
    pub floor_z: f64,
    pub top_z:   f64,
    pub radius:  f64,
}

impl Default for PocketClearingParams {
    fn default() -> Self {
        Self {
            pockets: Vec::new(),
            step_down: 1.0,
            step_over: 0.5,
            feed_rate: 800.0,
            stock_to_leave: 0.0,
            direction: crate::strategies::CutDirection::Climb,
            z_range: crate::strategies::ZRange::default(),
            spring_pass: crate::strategies::SpringPass::default(),
        }
    }
}

impl PocketRef {
    pub fn from_feature(f: &Feature) -> Option<Self> {
        match f.kind {
            FeatureKind::Pocket { center, floor_z, top_z, radius, area: _ } => Some(Self {
                center, floor_z, top_z, radius,
            }),
            _ => None,
        }
    }
}

impl ToolpathStrategy for PocketClearing {
    type Params = PocketClearingParams;

    fn name(&self) -> &str {
        "Pocket Clearing"
    }

    fn generate(
        &self,
        model: &WorkpieceModel,
        tool: &Tool,
        _machine: &MachineConfig,
        params: &PocketClearingParams,
    ) -> Result<Vec<Toolpath>> {
        let mut tp = Toolpath::new(tool.id, OperationType::Roughing, "Pocket Clearing");
        let tool_d = tool.shape.diameter();
        let tool_r = tool_d * 0.5;
        let step_over_mm = (tool_d * params.step_over).max(tool_d * 0.05);
        let step_down = params.step_down.max(0.05);
        let leave = params.stock_to_leave.max(0.0);

        for pocket in &params.pockets {
            // Raster bounds: a square around the pocket centre, inset by tool_r.
            let r = pocket.radius;
            if r <= tool_r + 0.1 { continue; } // pocket too small for this tool
            let x_min = pocket.center.x - r + tool_r;
            let x_max = pocket.center.x + r - tool_r;
            let y_min = pocket.center.y - r + tool_r;
            let y_max = pocket.center.y + r - tool_r;
            let safe_z = pocket.top_z + 10.0;
            let z_floor = pocket.floor_z + leave;
            if x_min >= x_max || y_min >= y_max || pocket.top_z <= z_floor {
                continue;
            }

            // Layer-by-layer step-down from just under the open top to floor.
            let mut z = pocket.top_z;
            while z > z_floor {
                z = (z - step_down).max(z_floor);

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

                        // Reject XY positions outside the pocket's circular footprint.
                        let dx = x - pocket.center.x;
                        let dy = y - pocket.center.y;
                        if dx * dx + dy * dy > (r - tool_r) * (r - tool_r) {
                            continue;
                        }

                        // Raycast straight down from above to find the nearest
                        // mesh surface and clamp the layer Z to it (plus the
                        // stock-to-leave margin) — keeps the cutter off the
                        // walls and the pocket floor.
                        let ray = Ray::new(
                            parry3d::math::Point::new(x as f32, y as f32, safe_z as f32),
                            Vector3::new(0.0, 0.0, -1.0_f32),
                        );
                        let mut hit_z = z_floor;
                        if let Some(toi) = model.mesh.cast_local_ray(
                            &ray, (safe_z - z_floor) as f32, true,
                        ) {
                            hit_z = (safe_z - toi as f64).max(z_floor) + leave;
                        }
                        let target_z = z.max(hit_z);
                        row.push(Point3::new(x, y, target_z));
                    }

                    if !row.is_empty() {
                        let first = *row.first().unwrap();
                        let last  = *row.last().unwrap();
                        // Rapid above row start.
                        tp.points.push(ToolpathPoint {
                            position: Point3::new(first.x, first.y, safe_z),
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
                        tp.points.push(ToolpathPoint {
                            position: Point3::new(last.x, last.y, safe_z),
                            orientation: Vector3::z_axis(),
                            feed_rate: 0.0,
                            move_type: MoveType::Retract,
                        });
                    }

                    y += step_over_mm;
                    forward = !forward;
                }
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

    fn empty_model() -> WorkpieceModel {
        let verts = vec![
            PPoint::<f32>::new(0.0, 0.0, 0.0),
            PPoint::<f32>::new(10.0, 0.0, 0.0),
            PPoint::<f32>::new(5.0, 10.0, 0.0),
        ];
        let tris = vec![[0u32, 1, 2]];
        WorkpieceModel::new(
            TriMesh::new(verts, tris),
            StockShape::BoundingBox {
                margin: nalgebra::Vector3::new(2.0, 2.0, 2.0),
            },
        )
    }

    #[test]
    fn produces_raster_for_a_pocket() {
        let params = PocketClearingParams {
            pockets: vec![PocketRef {
                center:  Point3::new(0.0, 0.0, 0.0),
                floor_z: -5.0,
                top_z:   0.0,
                radius:  10.0,
            }],
            step_down: 2.0,
            step_over: 0.5,
            feed_rate: 600.0,
            stock_to_leave: 0.0,
            direction: crate::strategies::CutDirection::Climb,
            z_range: crate::strategies::ZRange::default(),
            spring_pass: crate::strategies::SpringPass::default(),
        };
        let tool = Tool::flat_end(1, "6mm flat", 6.0, 20.0);
        let machine = crate::kinematics::default_machines::default_trunnion_config();
        let paths = PocketClearing.generate(&empty_model(), &tool, &machine, &params).unwrap();
        assert_eq!(paths.len(), 1);
        assert!(!paths[0].points.is_empty(), "expected raster points");
    }

    #[test]
    fn skips_pocket_smaller_than_tool() {
        let params = PocketClearingParams {
            pockets: vec![PocketRef {
                center:  Point3::new(0.0, 0.0, 0.0),
                floor_z: -5.0,
                top_z:   0.0,
                radius:  1.0, // 2 mm wide; 6 mm tool can't enter
            }],
            ..Default::default()
        };
        let tool = Tool::flat_end(1, "6mm flat", 6.0, 20.0);
        let machine = crate::kinematics::default_machines::default_trunnion_config();
        let paths = PocketClearing.generate(&empty_model(), &tool, &machine, &params).unwrap();
        assert!(paths[0].points.is_empty(), "small pocket should produce no points");
    }
}

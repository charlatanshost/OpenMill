use anyhow::Result;
use nalgebra::Unit;
use serde::{Deserialize, Serialize};

use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::{MoveType, OperationType, Toolpath, ToolpathPoint};

use super::drilling::Hole;
use super::traits::ToolpathStrategy;

/// Rigid tapping strategy.
///
/// Plunges a tap into each hole at a controlled feed rate synchronised to
/// thread pitch (`feed = rpm × pitch`), then reverses out at the same rate.
/// On controllers that support rigid tapping (LinuxCNC `G33.1`) the post can
/// upgrade these moves to a true synchronised cycle; the generic emission is
/// `G1` plunge + `G1` retract with a comment marker.
///
/// Holes are populated from detected `FeatureKind::Hole` features (see the
/// Features panel) or entered manually like 5-axis Drilling.
pub struct Tapping;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TappingParams {
    /// Holes to tap.
    pub holes: Vec<Hole>,
    /// Thread pitch [mm/rev].
    pub thread_pitch: f64,
    /// Spindle speed [rpm] used to derive feed rate. `0` falls back to the
    /// op-level spindle speed at G-code emission time.
    pub spindle_rpm: f64,
    /// Optional peck depth [mm]. `0` means a single full-depth pass.
    pub peck_depth: f64,
    /// Dwell at the bottom of each hole [s] before reversing.
    pub dwell: f64,
}

impl Default for TappingParams {
    fn default() -> Self {
        Self {
            holes: Vec::new(),
            thread_pitch: 1.0, // M6 × 1.0 default
            spindle_rpm: 600.0,
            peck_depth: 0.0,
            dwell: 0.0,
        }
    }
}

impl ToolpathStrategy for Tapping {
    type Params = TappingParams;

    fn name(&self) -> &str {
        "Tapping"
    }

    fn generate(
        &self,
        _model: &WorkpieceModel,
        tool: &Tool,
        _machine: &MachineConfig,
        params: &TappingParams,
    ) -> Result<Vec<Toolpath>> {
        let mut tp = Toolpath::new(tool.id, OperationType::Roughing, "Tapping");

        // Synchronised feed rate: feed = rpm × pitch.
        // If rpm is 0 the user expects the op-level spindle speed to be used —
        // emit a placeholder feed; the post-processor and op feed-rate override
        // will plug a sensible value in.
        let synced_feed = (params.spindle_rpm * params.thread_pitch).max(1.0);

        for hole in &params.holes {
            let axis = Unit::new_normalize(hole.axis);
            let safe_start = hole.position + axis.into_inner() * 10.0;
            let bottom = hole.position - axis.into_inner() * hole.depth;

            // Rapid to safe position above the hole.
            tp.points.push(ToolpathPoint {
                position: safe_start,
                orientation: axis,
                feed_rate: 0.0,
                move_type: MoveType::Rapid,
            });

            if params.peck_depth > 0.0 && params.peck_depth < hole.depth {
                // Pecking: descend in steps, retract to safe between each.
                let mut current_depth = 0.0;
                while current_depth < hole.depth {
                    current_depth = (current_depth + params.peck_depth).min(hole.depth);
                    let target = hole.position - axis.into_inner() * current_depth;
                    tp.points.push(ToolpathPoint {
                        position: target,
                        orientation: axis,
                        feed_rate: synced_feed,
                        move_type: MoveType::Linear,
                    });
                    tp.points.push(ToolpathPoint {
                        position: safe_start,
                        orientation: axis,
                        feed_rate: synced_feed,
                        move_type: MoveType::Retract,
                    });
                }
            } else {
                // Single-pass tap to full depth.
                tp.points.push(ToolpathPoint {
                    position: bottom,
                    orientation: axis,
                    feed_rate: synced_feed,
                    move_type: MoveType::Linear,
                });
                // Reverse out at the synchronised feed.
                tp.points.push(ToolpathPoint {
                    position: safe_start,
                    orientation: axis,
                    feed_rate: synced_feed,
                    move_type: MoveType::Retract,
                });
            }
        }

        Ok(vec![tp])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{StockShape, WorkpieceModel};
    use nalgebra::{Point3, Vector3};
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
    fn produces_three_points_per_hole() {
        let mut params = TappingParams::default();
        params.holes.push(Hole {
            position: Point3::new(0.0, 0.0, 10.0),
            axis: Vector3::new(0.0, 0.0, -1.0),
            depth: 8.0,
        });
        let tool = Tool::flat_end(1, "M6 tap", 6.0, 20.0);
        let machine = crate::kinematics::default_machines::default_trunnion_config();
        let paths = Tapping.generate(&empty_model(), &tool, &machine, &params).unwrap();
        // safe-rapid + plunge + retract = 3 points per hole.
        assert_eq!(paths[0].points.len(), 3);
    }

    #[test]
    fn synced_feed_equals_rpm_times_pitch() {
        let params = TappingParams {
            holes: vec![Hole {
                position: Point3::new(0.0, 0.0, 5.0),
                axis: Vector3::new(0.0, 0.0, -1.0),
                depth: 5.0,
            }],
            thread_pitch: 1.5,
            spindle_rpm: 600.0,
            ..Default::default()
        };
        let tool = Tool::flat_end(1, "M8 tap", 8.0, 20.0);
        let machine = crate::kinematics::default_machines::default_trunnion_config();
        let paths = Tapping.generate(&empty_model(), &tool, &machine, &params).unwrap();
        // The plunge move should be at 600 × 1.5 = 900 mm/min.
        let plunge = paths[0].points.iter().find(|p| p.move_type == MoveType::Linear).unwrap();
        assert!((plunge.feed_rate - 900.0).abs() < 1e-6, "feed = {}", plunge.feed_rate);
    }
}

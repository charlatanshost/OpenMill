use anyhow::Result;
use parry3d::query::{Ray, RayCast};
use serde::{Deserialize, Serialize};

use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::{OperationType, Toolpath, ToolpathPoint, MoveType};

use super::traits::ToolpathStrategy;

/// Simultaneous 5-axis swarf (flank) milling.
///
/// The side face (flank) of the cutter is held tangent to a ruled surface.
/// Ideal for blades, turbine vanes, and near-vertical ruled walls.
pub struct Swarf5Axis;

/// Tuning parameters for [`Swarf5Axis`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Swarf5AxisParams {
    /// Top Z level of the wall to swarf [mm].
    pub z_top: f64,
    /// Bottom Z level of the wall to swarf [mm].
    pub z_bottom: f64,
    /// Number of passes along the ruled surface [1..].
    pub num_passes: usize,
    /// Axial step-down between passes [mm].
    pub step_down: f64,
    /// Cutting feed rate [mm/min].
    pub feed_rate: f64,
    /// Compensate for tool overhang / deflection.
    pub overhang_compensation: bool,
}

impl Default for Swarf5AxisParams {
    fn default() -> Self {
        Swarf5AxisParams {
            z_top: 10.0,
            z_bottom: 0.0,
            num_passes: 1,
            step_down: 5.0,
            feed_rate: 500.0,
            overhang_compensation: false,
        }
    }
}

impl ToolpathStrategy for Swarf5Axis {
    type Params = Swarf5AxisParams;

    fn name(&self) -> &str {
        "Swarf 5-Axis"
    }

    fn generate(
        &self,
        model: &WorkpieceModel,
        tool: &Tool,
        _machine: &MachineConfig,
        params: &Swarf5AxisParams,
    ) -> Result<Vec<Toolpath>> {
        use nalgebra::{Point3, Vector3, Unit};
        use parry3d::query::Ray;

        let mut tp = Toolpath::new(
            tool.id,
            OperationType::Finishing,
            "Swarf 5-Axis Finishing",
        );

        let tool_r = tool.shape.diameter() / 2.0;
        let aabb = &model.aabb;
        let center_x = (aabb.mins.x + aabb.maxs.x) as f64 * 0.5;
        let center_y = (aabb.mins.y + aabb.maxs.y) as f64 * 0.5;
        let scan_r = (aabb.maxs.x - aabb.mins.x).max(aabb.maxs.y - aabb.mins.y) as f64 + tool_r + 10.0;
        
        let ang_steps = 360;
        let mut rulings = Vec::new();
        
        for i in 0..=ang_steps {
            let angle = (i as f64).to_radians();
            let dir = Vector3::new(angle.cos(), angle.sin(), 0.0);
            
            // Raycast at top and bottom to find the wall boundaries
            let start_top = Point3::new(center_x, center_y, params.z_top) + dir * scan_r;
            let ray_top = Ray::new(start_top.cast::<f32>(), (-dir).cast::<f32>());
            
            let start_bot = Point3::new(center_x, center_y, params.z_bottom) + dir * scan_r;
            let ray_bot = Ray::new(start_bot.cast::<f32>(), (-dir).cast::<f32>());
            
            if let (Some(hit_top), Some(hit_bot)) = (
                model.mesh.cast_local_ray_and_get_normal(&ray_top, scan_r as f32 * 2.0, true),
                model.mesh.cast_local_ray_and_get_normal(&ray_bot, scan_r as f32 * 2.0, true)
            ) {
                let p_top = ray_top.point_at(hit_top.time_of_impact).cast::<f64>();
                let p_bot = ray_bot.point_at(hit_bot.time_of_impact).cast::<f64>();
                
                // Normal for offset (horizontal only for flank contact)
                let n_top = Vector3::new(hit_top.normal.x as f64, hit_top.normal.y as f64, 0.0).normalize();
                let n_bot = Vector3::new(hit_bot.normal.x as f64, hit_bot.normal.y as f64, 0.0).normalize();
                
                let contact_top = p_top + n_top * tool_r;
                let contact_bot = p_bot + n_bot * tool_r;
                
                rulings.push((contact_top, contact_bot));
            }
        }
        
        if rulings.is_empty() {
            // No flank-contact points found — return the empty toolpath rather
            // than an empty Vec so the UI's G-code terminal still shows the
            // post-detected mode header instead of vanishing.
            return Ok(vec![tp]);
        }

        let safe_z = params.z_top.max(params.z_bottom) + 10.0;
        
        // Initial rapid to start
        let first_ruling = rulings[0];
        tp.points.push(ToolpathPoint {
            position: Point3::new(first_ruling.0.x, first_ruling.0.y, safe_z),
            orientation: Vector3::z_axis(),
            feed_rate: 0.0,
            move_type: MoveType::Rapid,
        });

        // Generate passes. If step_down is provided, we can do multiple axial levels.
        // For simple swarf, we usually do one pass at the bottom, but the tool side 
        // contacts the whole ruling.
        let wall_height = (params.z_top - params.z_bottom).abs();
        let num_axial_passes = if params.step_down > 0.0 {
            (wall_height / params.step_down).ceil() as usize
        } else {
            1
        };

        for pass in 1..=num_axial_passes {
            let t_axial = pass as f64 / num_axial_passes as f64;

            // Compute the start position of this pass (first ruling at the
            // interpolated axial height) so we can rapid-over to it at
            // safe-Z before the first cut. Without this the LeadIn would
            // fly diagonally from safe-Z straight into the wall mid-height.
            let (top0, bot0) = rulings[0];
            let start_pos = bot0.coords.lerp(&top0.coords, 1.0 - t_axial);
            tp.points.push(ToolpathPoint {
                position: Point3::new(start_pos.x, start_pos.y, safe_z),
                orientation: Vector3::z_axis(),
                feed_rate: 0.0,
                move_type: MoveType::Rapid,
            });

            for (idx, (top, bot)) in rulings.iter().enumerate() {
                let tool_axis = Unit::new_normalize(top - bot);
                // The tool tip position for a flank cut is interpolated along the ruling
                let pos = bot.coords.lerp(&top.coords, 1.0 - t_axial);

                tp.points.push(ToolpathPoint {
                    position: Point3::from(pos),
                    orientation: tool_axis,
                    feed_rate: if idx == 0 { params.feed_rate * 0.5 } else { params.feed_rate },
                    move_type: if idx == 0 { MoveType::LeadIn } else { MoveType::Linear },
                });
            }

            // Retract if there are more passes
            if pass < num_axial_passes {
                let last = tp.points.last().unwrap().position;
                tp.points.push(ToolpathPoint {
                    position: Point3::new(last.x, last.y, safe_z),
                    orientation: Vector3::z_axis(),
                    feed_rate: 0.0,
                    move_type: MoveType::Retract,
                });
            }
        }

        // Final retract
        let last = tp.points.last().unwrap().position;
        tp.points.push(ToolpathPoint {
            position: Point3::new(last.x, last.y, safe_z),
            orientation: Vector3::z_axis(),
            feed_rate: 0.0,
            move_type: MoveType::Retract,
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

    /// A simple closed wall — four-sided pyramid trunk so the swarf
    /// raycaster finds rulings all around.
    fn wall_model() -> WorkpieceModel {
        let verts = vec![
            PPoint::new(-5.0, -5.0, 0.0), PPoint::new( 5.0, -5.0, 0.0),
            PPoint::new( 5.0,  5.0, 0.0), PPoint::new(-5.0,  5.0, 0.0),
            PPoint::new(-5.0, -5.0, 10.0), PPoint::new( 5.0, -5.0, 10.0),
            PPoint::new( 5.0,  5.0, 10.0), PPoint::new(-5.0,  5.0, 10.0),
        ];
        let tris = vec![
            [0u32, 1, 5], [0, 5, 4],
            [1, 2, 6], [1, 6, 5],
            [2, 3, 7], [2, 7, 6],
            [3, 0, 4], [3, 4, 7],
        ];
        WorkpieceModel::new(
            TriMesh::new(verts, tris),
            StockShape::BoundingBox {
                margin: nalgebra::Vector3::new(2.0, 2.0, 2.0),
            },
        )
    }

    #[test]
    fn between_pass_first_move_is_rapid_at_safe_z() {
        // Regression: multi-axial-pass swarf used to start each new pass
        // with a LeadIn directly from safe-Z to a mid-height point on the
        // wall, slicing diagonally through the part.
        let model = wall_model();
        let tool = crate::tool::Tool::flat_end(1, "6mm flat", 6.0, 20.0);
        let machine = crate::kinematics::default_machines::default_trunnion_config();
        let params = Swarf5AxisParams {
            z_top: 10.0, z_bottom: 0.0,
            num_passes: 1, step_down: 2.5, // forces 4 axial passes
            feed_rate: 500.0,
            overhang_compensation: false,
        };
        let paths = Swarf5Axis.generate(&model, &tool, &machine, &params).unwrap();
        assert!(!paths.is_empty());
        let tp = &paths[0];
        let safe_z = params.z_top.max(params.z_bottom) + 10.0;

        // After every Retract there must be either end-of-toolpath or a
        // Rapid (not a LeadIn) at safe_z.
        for i in 0..tp.points.len().saturating_sub(1) {
            if tp.points[i].move_type == MoveType::Retract {
                let next = &tp.points[i + 1];
                assert!(
                    next.move_type == MoveType::Rapid,
                    "expected Rapid after Retract, got {:?} (regression: \
                     bare LeadIn would slice into the wall)",
                    next.move_type,
                );
                assert!(
                    (next.position.z - safe_z).abs() < 1e-6,
                    "the linking Rapid should sit at safe_z={safe_z}, got {}",
                    next.position.z,
                );
            }
        }
    }
}


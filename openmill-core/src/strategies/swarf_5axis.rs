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
            return Ok(vec![]);
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
            
            for (idx, (top, bot)) in rulings.iter().enumerate() {
                let tool_axis = Unit::new_normalize(top - bot);
                // The tool tip position for a flank cut is interpolated along the ruling
                let pos = bot.coords.lerp(&top.coords, 1.0 - t_axial);
                
                tp.points.push(ToolpathPoint {
                    position: Point3::from(pos),
                    orientation: tool_axis,
                    feed_rate: params.feed_rate,
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

use anyhow::Result;
use nalgebra::{Point3, Vector3, Unit};
use serde::{Deserialize, Serialize};

use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::{MoveType, OperationType, Toolpath, ToolpathPoint};

use super::traits::ToolpathStrategy;

/// 5-Axis Drilling.
///
/// Moves the tool to a hole position, aligns with the hole axis,
/// and performs a plunge-retract cycle.
pub struct Drilling5Axis;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Hole {
    pub position: Point3<f64>,
    pub axis: Vector3<f64>,
    pub depth: f64,
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct DrillingParams {
    pub holes: Vec<Hole>,
    pub feed_rate: f64,
    pub dwell: f64,
}

impl ToolpathStrategy for Drilling5Axis {
    type Params = DrillingParams;

    fn name(&self) -> &str {
        "5-Axis Drilling"
    }

    fn generate(
        &self,
        _model: &WorkpieceModel,
        tool: &Tool,
        _machine: &MachineConfig,
        params: &DrillingParams,
    ) -> Result<Vec<Toolpath>> {
        let mut tp = Toolpath::new(tool.id, OperationType::Roughing, "5-Axis Drilling");

        for hole in &params.holes {
            let axis = Unit::new_normalize(hole.axis);
            let safe_start = hole.position + axis.into_inner() * 10.0;
            let bottom = hole.position - axis.into_inner() * hole.depth;

            // Rapid to safe position
            tp.points.push(ToolpathPoint {
                position: safe_start,
                orientation: axis,
                feed_rate: 0.0,
                move_type: MoveType::Rapid,
            });

            // Plunge
            tp.points.push(ToolpathPoint {
                position: bottom,
                orientation: axis,
                feed_rate: params.feed_rate,
                move_type: MoveType::Linear,
            });

            // Retract
            tp.points.push(ToolpathPoint {
                position: safe_start,
                orientation: axis,
                feed_rate: 0.0,
                move_type: MoveType::Retract,
            });
        }

        Ok(vec![tp])
    }
}

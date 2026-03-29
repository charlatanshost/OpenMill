use anyhow::Result;
use serde::{Deserialize, Serialize};

use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::Toolpath;

use super::traits::ToolpathStrategy;

/// Simultaneous 5-axis swarf (flank) milling.
///
/// The side face (flank) of the cutter is held tangent to a ruled surface.
/// Ideal for blades, turbine vanes, and near-vertical ruled walls.
pub struct Swarf5Axis;

/// Tuning parameters for [`Swarf5Axis`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Swarf5AxisParams {
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
        _model: &WorkpieceModel,
        _tool: &Tool,
        _machine: &MachineConfig,
        _params: &Swarf5AxisParams,
    ) -> Result<Vec<Toolpath>> {
        todo!(
            "Swarf5Axis: fit ruled surface to triangulated wall, \
             align tool flank, check gouging, generate contact path"
        )
    }
}

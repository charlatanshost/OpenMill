use anyhow::Result;
use serde::{Deserialize, Serialize};

use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::Toolpath;

use super::traits::ToolpathStrategy;

/// 3-axis waterline (contour-parallel) finishing strategy.
///
/// Intersects the part mesh with horizontal planes at equal Z intervals and
/// offsets the resulting contours by the tool radius.
pub struct ContourParallel;

/// Tuning parameters for [`ContourParallel`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContourParallelParams {
    /// Step-down between waterline passes [mm].
    pub step_down: f64,
    /// Cutting feed rate [mm/min].
    pub feed_rate: f64,
    /// Surface chord tolerance [mm].
    pub tolerance: f64,
}

impl Default for ContourParallelParams {
    fn default() -> Self {
        ContourParallelParams {
            step_down: 0.5,
            feed_rate: 600.0,
            tolerance: 0.01,
        }
    }
}

impl ToolpathStrategy for ContourParallel {
    type Params = ContourParallelParams;

    fn name(&self) -> &str {
        "Contour Parallel"
    }

    fn generate(
        &self,
        _model: &WorkpieceModel,
        _tool: &Tool,
        _machine: &MachineConfig,
        _params: &ContourParallelParams,
    ) -> Result<Vec<Toolpath>> {
        todo!("ContourParallel: slice mesh at Z-levels, offset contours, link passes")
    }
}

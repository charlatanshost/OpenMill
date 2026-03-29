use anyhow::Result;

use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::Toolpath;

use super::traits::ToolpathStrategy;

/// 3+2 indexed positioning strategy.
///
/// The table rotary axes are indexed to fixed (A, C) angles and then locked.
/// A standard 3-axis strategy runs in the resulting tilted work plane.
/// Produces two G-code segments: a rotary index move, then 3-axis cuts.
pub struct ThreePlusTwo;

/// Tuning parameters for [`ThreePlusTwo`].
#[derive(Debug, Clone)]
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
        _model: &WorkpieceModel,
        _tool: &Tool,
        _machine: &MachineConfig,
        _params: &ThreePlusTwoParams,
    ) -> Result<Vec<Toolpath>> {
        todo!(
            "ThreePlusTwo: rotate model frame by (A, C), run 3-axis sub-strategy, \
             prepend rotary index move in output"
        )
    }
}

use anyhow::Result;

use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::Toolpath;

use super::traits::ToolpathStrategy;

/// 3-axis adaptive (trochoidal) roughing strategy.
///
/// Limits tool engagement angle to a configurable maximum to keep chip load
/// constant.  Analogous to HSMWorks / Fusion 360 "Adaptive Clearing".
pub struct AdaptiveClearing;

/// Tuning parameters for [`AdaptiveClearing`].
#[derive(Debug, Clone)]
pub struct AdaptiveClearingParams {
    /// Maximum tool engagement angle [degrees].
    pub max_engagement_deg: f64,
    /// Step-down per pass [mm].
    pub step_down: f64,
    /// Cutting feed rate [mm/min].
    pub feed_rate: f64,
}

impl Default for AdaptiveClearingParams {
    fn default() -> Self {
        AdaptiveClearingParams {
            max_engagement_deg: 60.0,
            step_down: 2.0,
            feed_rate: 800.0,
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
        _model: &WorkpieceModel,
        _tool: &Tool,
        _machine: &MachineConfig,
        _params: &AdaptiveClearingParams,
    ) -> Result<Vec<Toolpath>> {
        todo!("AdaptiveClearing: medial-axis trochoidal paths per Z-level")
    }
}

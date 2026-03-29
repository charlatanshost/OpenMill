use anyhow::Result;

use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::Toolpath;

/// Trait every toolpath strategy must implement.
///
/// Strategies are stateless parameter objects: all tunable behaviour is
/// expressed through the associated `Params` type.  The `generate` call is
/// pure — it does not mutate the strategy or the model.
///
/// # Example
/// ```rust,ignore
/// let strategy = SurfaceNormal5Axis;
/// let params   = SurfaceNormal5AxisParams { lead_angle: 5.0, ..Default::default() };
/// let paths    = strategy.generate(&model, &tool, &machine, &params)?;
/// ```
pub trait ToolpathStrategy {
    /// Strategy-specific tuning parameters.
    type Params;

    /// A short display name shown in the UI and G-code comments.
    fn name(&self) -> &str;

    /// Compute toolpaths for the given model and tool.
    ///
    /// Returns one or more [`Toolpath`] segments (e.g. roughing passes, then a
    /// finishing pass), all in workpiece coordinates.
    fn generate(
        &self,
        model: &WorkpieceModel,
        tool: &Tool,
        machine: &MachineConfig,
        params: &Self::Params,
    ) -> Result<Vec<Toolpath>>;
}

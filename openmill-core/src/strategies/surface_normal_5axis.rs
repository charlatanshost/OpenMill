use anyhow::Result;

use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::Toolpath;

use super::traits::ToolpathStrategy;

/// Simultaneous 5-axis surface-normal finishing.
///
/// The tool axis is tilted to match (or offset from) the surface normal at
/// each contact point.  Best used with a ball-end mill on sculptured surfaces.
pub struct SurfaceNormal5Axis;

/// Drive-surface scan pattern.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DrivePattern {
    /// Parallel raster passes along the surface U parameter.
    Parallel,
    /// Constant-scallop step-over.
    Scallop,
    /// Inward spiral from boundary.
    Spiral,
}

/// Tuning parameters for [`SurfaceNormal5Axis`].
#[derive(Debug, Clone)]
pub struct SurfaceNormal5AxisParams {
    /// Lead angle (tilt in direction of motion) [degrees].
    pub lead_angle: f64,
    /// Side tilt angle (tilt perpendicular to motion) [degrees].
    pub tilt_angle: f64,
    /// Step-over as a fraction of tool diameter [0.0 .. 1.0].
    pub step_over: f64,
    /// Cutting feed rate [mm/min].
    pub feed_rate: f64,
    /// Surface chord tolerance [mm].
    pub tolerance: f64,
    /// Scan pattern.
    pub pattern: DrivePattern,
}

impl Default for SurfaceNormal5AxisParams {
    fn default() -> Self {
        SurfaceNormal5AxisParams {
            lead_angle: 5.0,
            tilt_angle: 0.0,
            step_over: 0.1,
            feed_rate: 400.0,
            tolerance: 0.005,
            pattern: DrivePattern::Parallel,
        }
    }
}

impl ToolpathStrategy for SurfaceNormal5Axis {
    type Params = SurfaceNormal5AxisParams;

    fn name(&self) -> &str {
        "Surface Normal 5-Axis"
    }

    fn generate(
        &self,
        _model: &WorkpieceModel,
        _tool: &Tool,
        _machine: &MachineConfig,
        _params: &SurfaceNormal5AxisParams,
    ) -> Result<Vec<Toolpath>> {
        todo!(
            "SurfaceNormal5Axis: sample mesh normals, apply lead/tilt, \
             solve IK per point, smooth orientation transitions"
        )
    }
}

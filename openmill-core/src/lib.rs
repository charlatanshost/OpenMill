pub mod import;
pub mod kinematics;
pub mod model;
pub mod strategies;
pub mod tool;
pub mod toolpath;

// Flat re-exports for convenience.
pub use kinematics::{
    AxisLimits, ForwardKinematics, IkError, InverseKinematics, KinematicType, MachineConfig,
    RotaryAxis, TableTable,
};
pub use model::{StockShape, WorkpieceModel};
pub use strategies::{
    AdaptiveClearing, AdaptiveClearingParams,
    ContourParallel, ContourParallelParams,
    DrivePattern, SurfaceNormal5Axis, SurfaceNormal5AxisParams,
    Swarf5Axis, Swarf5AxisParams,
    ThreePlusTwo, ThreePlusTwoParams,
    ToolpathStrategy,
};
pub use tool::{Tool, ToolHolder, ToolShape};
pub use toolpath::{MoveType, OperationType, Toolpath, ToolpathPoint};
pub use import::{import_stl, import_3mf};

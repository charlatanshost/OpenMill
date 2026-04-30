pub mod fixture;
pub mod import;
pub mod job;
pub mod kinematics;
pub mod machine;
pub mod material;
pub mod model;
pub mod strategies;
pub mod tool;
pub mod toolpath;

// Flat re-exports for convenience.
pub use fixture::{Fixture, FixtureShape};
pub use import::{import_3mf, import_stl};
pub use job::{Job, JobSettings, Operation, StockDef};
pub use kinematics::{
    AxisLimits, ForwardKinematics, IkError, InverseKinematics, KinematicType, MachineConfig,
    PostConfig, RotaryAxis, TableTable, Units,
};
pub use machine::MachineLibrary;
pub use material::{Material, MaterialCategory, MaterialLibrary};
pub use model::{StockShape, WorkpieceModel};
pub use strategies::{
    AdaptiveClearing, AdaptiveClearingParams, ContourParallel, ContourParallelParams,
    Drilling5Axis, DrillingParams, DrivePattern, GeodesicParallel, GeodesicParams,
    PencilTracing, PencilParams, SurfaceNormal5Axis, SurfaceNormal5AxisParams,
    Swarf5Axis, Swarf5AxisParams, ThreePlusTwo, ThreePlusTwoParams, ToolpathStrategy,
};
pub use tool::{Tool, ToolHolder, ToolShape};
pub use toolpath::{MoveType, OperationType, Toolpath, ToolpathPoint};

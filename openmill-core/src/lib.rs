pub mod feature;
pub mod fixture;
pub mod import;
pub mod job;
pub mod kinematics;
pub mod machine;
pub mod material;
pub mod model;
pub mod sdf;
pub mod strategies;
pub mod tool;
pub mod toolpath;
pub mod verify;

// Flat re-exports for convenience.
pub use feature::{
    detect_holes, detect_pockets, feature_to_hole, pick_face, Feature, FeatureKind, PickedFace,
};
pub use fixture::{Fixture, FixtureShape};
pub use import::{import_3mf, import_3mf_reader, import_stl, import_stl_reader};
pub use job::{Job, JobSettings, Operation, StockDef};
pub use kinematics::{
    AxisLimits, ForwardKinematics, IkError, InverseKinematics, KinematicType, MachineConfig,
    PostConfig, RotaryAxis, TableTable, Units,
};
pub use machine::MachineLibrary;
pub use material::{Material, MaterialCategory, MaterialLibrary};
pub use model::{StockShape, WorkpieceModel};
pub use strategies::{
    apply_common as apply_common_transforms, apply_direction, apply_spring_pass, apply_z_range,
    step_over_from_cusp, step_over_mm_for, AdaptiveClearing, AdaptiveClearingParams,
    CommonStrategyParams, ContourParallel, ContourParallelParams, CutDirection, CycleType,
    Drilling5Axis, DrillingParams, DrivePattern, FourPlusOne, FourPlusOneParams,
    GeodesicParallel, GeodesicParams, MultiAxisRoughing, MultiAxisRoughingParams,
    PencilTracing, PencilParams, PocketClearing,
    PocketClearingParams, PocketRef, SpringPass, SurfaceNormal5Axis, SurfaceNormal5AxisParams,
    Swarf5Axis, Swarf5AxisParams, TapMode, Tapping, TappingParams, ThreadDirection,
    ThreadMilling, ThreadMillingParams, ThreePlusTwo, ThreePlusTwoParams, ToolpathStrategy,
    ZRange,
};
pub use tool::{
    holder_collision, starter_presets_for, Coolant, FeedSpeedPreset, Tool, ToolHolder, ToolShape,
};
pub use toolpath::{
    aggregate_metrics, apply_leads, LeadConfig, LeadKind, MoveType, OperationType, Toolpath,
    ToolpathMetrics, ToolpathPoint,
};
pub use verify::{has_errors, verify_job, Issue, IssueLevel};

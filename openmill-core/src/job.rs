use serde::{Deserialize, Serialize};

use crate::fixture::Fixture;
use crate::kinematics::MachineConfig;
use crate::tool::{Coolant, Tool};


/// Top-level project container holding everything needed to machine a part.
///
/// A `Job` is the unit of save/load — it captures the full machining setup
/// so it can be serialized to JSON, stored on disk, and reopened later.
///
/// The part geometry (`model_path`) is referenced by file path rather than
/// embedded, keeping the JSON compact.  Call [`import_stl`] or [`import_3mf`]
/// at load time to populate the in-memory [`WorkpieceModel`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Job {
    /// Human-readable project name.
    pub name: String,
    /// Path to the part mesh file (STL or 3MF), relative to the job file.
    pub model_path: Option<String>,
    /// Translation applied to the imported mesh to place it in the work area.
    /// Persisted so the part shows up in the same position when a saved job
    /// is reopened. Defaults to zero for backward compatibility.
    #[serde(default)]
    pub model_position: [f64; 3],
    /// Stock definition.
    pub stock: StockDef,
    /// Machine configuration for this job.
    pub machine: MachineConfig,
    /// Tool library available for this job.
    pub tools: Vec<Tool>,
    /// Fixture / clamp definitions the tool must avoid.
    pub fixtures: Vec<Fixture>,
    /// Ordered list of machining operations.
    pub operations: Vec<Operation>,
    /// Global settings.
    pub settings: JobSettings,
}

/// Stock definition stored in the job file.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum StockDef {
    /// Bounding-box stock with per-axis margin [mm].
    BoundingBox { margin: [f64; 3] },
    /// Cylindrical stock.
    Cylinder { diameter: f64, height: f64 },
    /// Stock defined by a separate mesh file.
    MeshFile { path: String },
}

impl Default for StockDef {
    fn default() -> Self {
        StockDef::BoundingBox {
            margin: [5.0, 5.0, 5.0],
        }
    }
}

impl StockDef {
    /// Convert the job-level stock definition into a core `StockShape`.
    pub fn to_shape(&self) -> crate::model::StockShape {
        match self {
            StockDef::BoundingBox { margin } => crate::model::StockShape::BoundingBox {
                margin: nalgebra::Vector3::new(margin[0], margin[1], margin[2]),
            },
            StockDef::Cylinder { diameter, height } => crate::model::StockShape::Cylinder {
                diameter: *diameter,
                height: *height,
            },
            StockDef::MeshFile { .. } => {
                // Placeholder: fallback to bounding box for now if mesh isn't loaded
                crate::model::StockShape::BoundingBox {
                    margin: nalgebra::Vector3::new(5.0, 5.0, 5.0),
                }
            }
        }
    }
}

/// A single machining operation in the job's operation list.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Operation {
    /// Display name.
    pub name: String,
    /// Which tool (by id) from `Job::tools`.
    pub tool_id: u32,
    /// Strategy name (e.g. "3+2 Indexed", "Adaptive Clearing").
    pub strategy: String,
    /// Strategy parameters serialized as JSON value (strategy-specific).
    pub params: serde_json::Value,
    /// Spindle speed [RPM].
    pub spindle_speed: f64,
    /// Cutting feed rate [mm/min] for lateral moves. `0.0` means "use whatever
    /// the strategy embeds in each toolpath point".
    #[serde(default)]
    pub feed_rate: f64,
    /// Plunge feed rate [mm/min] for axial entry moves. `0.0` falls back to
    /// `feed_rate * 0.5`.
    #[serde(default)]
    pub plunge_rate: f64,
    /// Coolant mode active for this operation.
    #[serde(default)]
    pub coolant: Coolant,
    /// Free-form G-code injected at the start of this operation, after the
    /// spindle and coolant have been turned on. Use for probing, axis homing,
    /// or any controller-specific setup.
    #[serde(default)]
    pub gcode_command: String,
    /// Stock to leave on the finished part [mm]. Roughing ops typically set
    /// this to 0.2–0.5 mm so a finishing op can come back and skim to size.
    /// At generate time the value is auto-injected into the strategy's
    /// `stock_to_leave` parameter (when the strategy honours it).
    #[serde(default)]
    pub stock_to_leave: f64,
    /// Whether this operation is enabled.
    pub enabled: bool,
}

/// Global job settings.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JobSettings {
    /// Safe Z height for rapid retracts above the stock [mm].
    pub safe_z: f64,
    /// Clearance height above the stock top for rapid traverse [mm].
    pub clearance_z: f64,
    /// Lead-in distance for plunge moves [mm].
    pub lead_in: f64,
    /// Global tolerance for toolpath linearization [mm].
    pub tolerance: f64,
}

impl Default for JobSettings {
    fn default() -> Self {
        JobSettings {
            safe_z: 25.0,
            clearance_z: 5.0,
            lead_in: 2.0,
            tolerance: 0.01,
        }
    }
}

impl Default for Job {
    fn default() -> Self {
        use crate::kinematics::default_machines::default_trunnion_config;

        Job {
            name: "Untitled Job".into(),
            model_path: None,
            model_position: [0.0, 0.0, 0.0],
            stock: StockDef::default(),
            machine: default_trunnion_config(),
            tools: vec![
                Tool::flat_end(1, "6mm Flat End", 6.0, 20.0),
                Tool::ball_end(2, "6mm Ball End", 6.0, 20.0),
            ],
            fixtures: Vec::new(),
            operations: Vec::new(),
            settings: JobSettings::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn job_serde_roundtrip() {
        let job = Job::default();
        let json = serde_json::to_string_pretty(&job).unwrap();
        let decoded: Job = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded.name, "Untitled Job");
        assert_eq!(decoded.tools.len(), 2);
        assert_eq!(decoded.machine.name, "Hobby A-C Trunnion");
    }
}

use std::fs;
use std::path::Path;
use anyhow::Result;

use crate::kinematics::MachineConfig;

/// A persistent library of machine configurations.
#[derive(Debug, Default)]
pub struct MachineLibrary {
    pub machines: Vec<MachineConfig>,
}

impl MachineLibrary {
    /// Load all machine configurations from the specified directory.
    pub fn load(dir: impl AsRef<Path>) -> Result<Self> {
        let dir = dir.as_ref();
        if !dir.exists() {
            fs::create_dir_all(dir)?;
            return Ok(MachineLibrary::default());
        }

        let mut machines = Vec::new();
        for entry in fs::read_dir(dir)? {
            let entry = entry?;
            let path = entry.path();
            if path.is_file() && path.extension().map_or(false, |ext| ext == "json") {
                let json = fs::read_to_string(&path)?;
                if let Ok(machine) = serde_json::from_str::<MachineConfig>(&json) {
                    machines.push(machine);
                }
            }
        }

        Ok(MachineLibrary { machines })
    }

    /// Save a machine configuration to the specified directory.
    pub fn save_machine(dir: impl AsRef<Path>, machine: &MachineConfig) -> Result<()> {
        let dir = dir.as_ref();
        if !dir.exists() {
            fs::create_dir_all(dir)?;
        }

        // Sanitize filename: use name, lowercase, no spaces
        let filename = machine.name.to_lowercase().replace(' ', "_") + ".json";
        let path = dir.join(filename);
        let json = serde_json::to_string_pretty(machine)?;
        fs::write(path, json)?;
        Ok(())
    }

    /// Delete a machine configuration from the specified directory.
    pub fn delete_machine(dir: impl AsRef<Path>, name: &str) -> Result<()> {
        let dir = dir.as_ref();
        let filename = name.to_lowercase().replace(' ', "_") + ".json";
        let path = dir.join(filename);
        if path.exists() {
            fs::remove_file(path)?;
        }
        Ok(())
    }
}

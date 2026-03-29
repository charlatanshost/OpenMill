use anyhow::Result;
use openmill_core::{MachineConfig, Tool, Toolpath};
use serde::{Deserialize, Serialize};

// ── Units ────────────────────────────────────────────────────────────────────

/// G-code unit system.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Units {
    Metric,
    Imperial,
}

// ── PostConfig ───────────────────────────────────────────────────────────────

/// Top-level configuration for G-code output.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PostConfig {
    /// Program number (O-word) for controls that require one.
    pub program_number: u32,
    /// Work coordinate offset code, e.g. `"G54"`, `"G55"`.
    pub work_offset: String,
    /// Unit system — determines whether the post emits G21 or G20.
    pub units: Units,
}

impl Default for PostConfig {
    fn default() -> Self {
        PostConfig {
            program_number: 1000,
            work_offset: "G54".into(),
            units: Units::Metric,
        }
    }
}

// ── PostProcessor trait ──────────────────────────────────────────────────────

/// Trait implemented by each G-code dialect (LinuxCNC, GRBL, etc.).
pub trait PostProcessor {
    /// Program header: %, O-number, modal setup, etc.
    fn header(&self, config: &PostConfig) -> String;

    /// Convert a toolpath into G-code lines.
    fn process_toolpath(
        &self,
        toolpath: &Toolpath,
        machine: &MachineConfig,
    ) -> Result<String>;

    /// G-code for a tool change sequence.
    fn tool_change(&self, tool: &Tool) -> String;

    /// Program footer: M5, M30, %, etc.
    fn footer(&self) -> String;
}

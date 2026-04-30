use anyhow::Result;
use openmill_core::{MachineConfig, Tool, Toolpath, PostConfig};

// ── PostProcessor trait ──────────────────────────────────────────────────────

/// Trait implemented by each G-code dialect (LinuxCNC, GRBL, etc.).
pub trait PostProcessor {
    /// Program header: %, O-number, modal setup, etc.
    fn header(&self, config: &PostConfig) -> String;

    /// Convert a toolpath into G-code lines, each paired with the point index that generated it.
    fn process_toolpath(
        &self,
        toolpath: &Toolpath,
        machine: &MachineConfig,
    ) -> Result<Vec<(String, Option<usize>)>>;

    /// G-code for a tool change sequence.
    fn tool_change(&self, tool: &Tool) -> String;

    /// Program footer: M5, M30, %, etc.
    fn footer(&self) -> String;
}

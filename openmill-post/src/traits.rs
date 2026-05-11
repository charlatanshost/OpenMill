use anyhow::Result;
use openmill_core::{MachineConfig, Operation, PostConfig, Tool, Toolpath};

/// Spindle command (`"M3"` clockwise or `"M4"` counter-clockwise) for an
/// op. Threading strategies expose a `thread_direction` field in their
/// params JSON; left-hand threads need M4 so the tap or thread mill
/// winds out cleanly. Every other strategy defaults to M3.
pub fn spindle_command_for(op: &Operation) -> &'static str {
    let is_threading = matches!(op.strategy.as_str(), "Tapping" | "Thread Milling");
    let left_hand = op
        .params
        .get("thread_direction")
        .and_then(|v| v.as_str())
        == Some("left_hand");
    if is_threading && left_hand {
        "M4"
    } else {
        "M3"
    }
}

// ── PostProcessor trait ──────────────────────────────────────────────────────

/// Trait implemented by each G-code dialect (LinuxCNC, GRBL, etc.).
///
/// **Per-operation emission order** (used by the UI G-code terminal and the
/// exporter):
///
/// 1. [`tool_change`] — physical swap (`M5`, `M9`, `M6 T#`, then any tool-level
///    `tool_change_gcode`).
/// 2. [`op_preamble`] — spindle on at op RPM, coolant on, free-form
///    `op.gcode_command`.
/// 3. [`process_toolpath`] for each toolpath in the op.
/// 4. [`op_postamble`] — coolant off (`M9`).
///
/// Steps 1–4 repeat per operation. [`header`] runs once at program start and
/// [`footer`] once at program end.
pub trait PostProcessor {
    /// Program header: %, O-number, modal setup, etc.
    fn header(&self, config: &PostConfig) -> String;

    /// Convert a toolpath into G-code lines, each paired with the point index that generated it.
    fn process_toolpath(
        &self,
        toolpath: &Toolpath,
        machine: &MachineConfig,
    ) -> Result<Vec<(String, Option<usize>)>>;

    /// G-code for a tool change sequence (stop spindle, stop coolant, swap
    /// tool, run any tool-level setup G-code).
    fn tool_change(&self, tool: &Tool) -> String;

    /// G-code emitted at the start of an operation: spindle on at op RPM,
    /// coolant on, then any free-form `op.gcode_command`.
    fn op_preamble(&self, op: &Operation, tool: &Tool) -> String;

    /// G-code emitted at the end of an operation: coolant off.
    fn op_postamble(&self, op: &Operation) -> String;

    /// Program footer: M5, M30, %, etc.
    fn footer(&self) -> String;
}

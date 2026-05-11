//! Background toolpath generation.
//!
//! Generation runs on a worker thread so the UI stays responsive on long
//! strategies. The worker processes operations sequentially (the strategies
//! themselves already parallelise internally via rayon) and emits progress
//! through an mpsc channel. The UI polls each frame and merges completed
//! results into the live `toolpaths` vector.
//!
//! Cancellation is cooperative: the worker checks the `Arc<AtomicBool>`
//! flag between operations. Mid-strategy interruption isn't supported —
//! once an op starts it runs to completion, which keeps the model and
//! strategy params simple (no shared mutable state).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::Arc;
use std::thread::JoinHandle;

use openmill_core::{
    AdaptiveClearing, AdaptiveClearingParams, ContourParallel, ContourParallelParams,
    Drilling5Axis, DrillingParams, FourPlusOne, FourPlusOneParams, GeodesicParallel,
    GeodesicParams, Job, MachineConfig, MultiAxisRoughing, MultiAxisRoughingParams, Operation,
    PencilParams, PencilTracing, PocketClearing, PocketClearingParams, SurfaceNormal5Axis,
    SurfaceNormal5AxisParams, Swarf5Axis, Swarf5AxisParams, Tapping, TappingParams,
    ThreadMilling, ThreadMillingParams, ThreePlusTwo, ThreePlusTwoParams, Tool, Toolpath,
    ToolpathStrategy, WorkpieceModel,
};

// ── Public messages ─────────────────────────────────────────────────────────

#[derive(Debug)]
pub enum GenMsg {
    /// Worker has begun the op at index `op_idx`.
    Started { op_idx: usize, op_name: String },
    /// Op finished with a generated toolpath set (possibly empty).
    Finished { op_idx: usize, paths: Vec<Toolpath> },
    /// Op failed to generate; the message describes why.
    Failed { op_idx: usize, op_name: String, error: String },
    /// Op was skipped — disabled, missing tool, etc.
    Skipped { op_idx: usize, op_name: String, reason: String },
    /// Worker has stopped. `cancelled` is true if the user cancelled.
    Done { cancelled: bool },
}

pub struct GenerationTask {
    cancel: Arc<AtomicBool>,
    rx: Receiver<GenMsg>,
    /// Number of enabled-and-eligible ops the worker plans to run. Used by
    /// the UI to size the progress bar.
    pub total: usize,
    /// Ops the worker has reported finished/failed/skipped.
    pub completed: usize,
    /// Index of the op currently being generated, if any.
    pub current: Option<usize>,
    /// Snapshot of the op names for UI display, since the worker doesn't
    /// hold a reference to `Job::operations`.
    pub op_names: Vec<String>,
    handle: Option<JoinHandle<()>>,
}

impl GenerationTask {
    pub fn spawn(job: &Job, model: &WorkpieceModel) -> Self {
        let cancel = Arc::new(AtomicBool::new(false));
        let (tx, rx) = channel();

        // Snapshot everything the worker needs. Cheap: a few KB of strategy
        // params + one mesh clone.
        let ops: Vec<Operation> = job.operations.clone();
        let tools: Vec<Tool> = job.tools.clone();
        let machine: MachineConfig = job.machine.clone();
        let model_clone: WorkpieceModel = model.clone();
        let cancel_for_worker = cancel.clone();

        let total = ops.iter().filter(|o| o.enabled).count();
        let op_names = ops.iter().map(|o| o.name.clone()).collect();

        let handle = std::thread::spawn(move || {
            run_worker(tx, cancel_for_worker, ops, tools, machine, model_clone);
        });

        Self {
            cancel,
            rx,
            total,
            completed: 0,
            current: None,
            op_names,
            handle: Some(handle),
        }
    }

    /// Drain pending messages from the worker. Returns each message in
    /// arrival order. The caller is expected to call this every frame and
    /// dispatch each `GenMsg` (e.g. merge `Finished` results into the live
    /// toolpaths vector).
    pub fn poll(&mut self) -> Vec<GenMsg> {
        let mut out = Vec::new();
        while let Ok(msg) = self.rx.try_recv() {
            match &msg {
                GenMsg::Started { op_idx, .. } => self.current = Some(*op_idx),
                GenMsg::Finished { .. } | GenMsg::Failed { .. } | GenMsg::Skipped { .. } => {
                    self.completed += 1;
                    self.current = None;
                }
                GenMsg::Done { .. } => {}
            }
            out.push(msg);
        }
        out
    }

    pub fn request_cancel(&self) {
        self.cancel.store(true, Ordering::Relaxed);
    }

    pub fn is_finished(&self) -> bool {
        self.handle
            .as_ref()
            .map(|h| h.is_finished())
            .unwrap_or(true)
    }

    /// Join the worker thread. Best-effort: errors are dropped.
    pub fn join(mut self) {
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

// ── Worker ──────────────────────────────────────────────────────────────────

fn run_worker(
    tx: Sender<GenMsg>,
    cancel: Arc<AtomicBool>,
    ops: Vec<Operation>,
    tools: Vec<Tool>,
    machine: MachineConfig,
    model: WorkpieceModel,
) {
    for (idx, op) in ops.iter().enumerate() {
        if cancel.load(Ordering::Relaxed) {
            let _ = tx.send(GenMsg::Done { cancelled: true });
            return;
        }

        if !op.enabled {
            // Disabled ops aren't part of `total` so don't emit Skipped —
            // they're simply outside the work set.
            continue;
        }

        let Some(tool) = tools.iter().find(|t| t.id == op.tool_id) else {
            let _ = tx.send(GenMsg::Skipped {
                op_idx: idx,
                op_name: op.name.clone(),
                reason: format!("tool T{} not found", op.tool_id),
            });
            continue;
        };

        let _ = tx.send(GenMsg::Started {
            op_idx: idx,
            op_name: op.name.clone(),
        });

        match dispatch(op, &model, tool, &machine) {
            Ok(paths) => {
                let _ = tx.send(GenMsg::Finished { op_idx: idx, paths });
            }
            Err(e) => {
                let _ = tx.send(GenMsg::Failed {
                    op_idx: idx,
                    op_name: op.name.clone(),
                    error: format!("{e:#}"),
                });
            }
        }
    }

    let _ = tx.send(GenMsg::Done { cancelled: false });
}

// ── Strategy dispatch ───────────────────────────────────────────────────────

/// Run the strategy named by `op.strategy` against the given model + tool.
/// Single source of truth for strategy → params decoding so the foreground
/// "Generate" button and the background worker stay in sync.
pub fn dispatch(
    op: &Operation,
    model: &WorkpieceModel,
    tool: &Tool,
    machine: &MachineConfig,
) -> anyhow::Result<Vec<Toolpath>> {
    let mut paths = dispatch_strategy(op, model, tool, machine)?;

    // Cross-strategy transforms: read the shared subset of fields from
    // the op's params JSON (anything the strategy didn't define just
    // takes the default), then apply uniformly to every emitted path.
    // Order matters — direction first (reverses cutting blocks), then
    // Z-range filtering, then leads (so the lead-in/out anchors are
    // already on the final, range-clipped passes).
    let common: openmill_core::CommonStrategyParams =
        serde_json::from_value(op.params.clone()).unwrap_or_default();
    for tp in &mut paths {
        openmill_core::apply_common_transforms(tp, &common);
    }

    if op.leads.is_active() {
        for tp in &mut paths {
            openmill_core::apply_leads(tp, &op.leads);
        }
    }
    Ok(paths)
}

fn dispatch_strategy(
    op: &Operation,
    model: &WorkpieceModel,
    tool: &Tool,
    machine: &MachineConfig,
) -> anyhow::Result<Vec<Toolpath>> {
    let params = inject_stock_to_leave(&op.params, op.stock_to_leave);
    match op.strategy.as_str() {
        "3+2 Indexed" => {
            let p: ThreePlusTwoParams = serde_json::from_value(params).unwrap_or_default();
            ThreePlusTwo.generate(model, tool, machine, &p)
        }
        "4+1 Indexed" => {
            let p: FourPlusOneParams = serde_json::from_value(params).unwrap_or_default();
            FourPlusOne.generate(model, tool, machine, &p)
        }
        "Adaptive Clearing" => {
            let p: AdaptiveClearingParams = serde_json::from_value(params).unwrap_or_default();
            AdaptiveClearing.generate(model, tool, machine, &p)
        }
        "Surface Normal 5-Axis" => {
            let p: SurfaceNormal5AxisParams =
                serde_json::from_value(params).unwrap_or_default();
            SurfaceNormal5Axis.generate(model, tool, machine, &p)
        }
        "Contour Parallel" => {
            let p: ContourParallelParams = serde_json::from_value(params).unwrap_or_default();
            ContourParallel.generate(model, tool, machine, &p)
        }
        "Swarf 5-Axis" => {
            let p: Swarf5AxisParams = serde_json::from_value(params).unwrap_or_default();
            Swarf5Axis.generate(model, tool, machine, &p)
        }
        "Geodesic Parallel" => {
            let p: GeodesicParams = serde_json::from_value(params).unwrap_or_default();
            GeodesicParallel.generate(model, tool, machine, &p)
        }
        "5-Axis Pencil Tracing" => {
            let p: PencilParams = serde_json::from_value(params).unwrap_or_default();
            PencilTracing.generate(model, tool, machine, &p)
        }
        "5-Axis Drilling" => {
            let p: DrillingParams = serde_json::from_value(params).unwrap_or_default();
            Drilling5Axis.generate(model, tool, machine, &p)
        }
        "Tapping" => {
            let p: TappingParams = serde_json::from_value(params).unwrap_or_default();
            Tapping.generate(model, tool, machine, &p)
        }
        "Thread Milling" => {
            let p: ThreadMillingParams = serde_json::from_value(params).unwrap_or_default();
            ThreadMilling.generate(model, tool, machine, &p)
        }
        "Pocket Clearing" => {
            let p: PocketClearingParams = serde_json::from_value(params).unwrap_or_default();
            PocketClearing.generate(model, tool, machine, &p)
        }
        "Simultaneous 5-Axis Roughing" => {
            let p: MultiAxisRoughingParams = serde_json::from_value(params).unwrap_or_default();
            MultiAxisRoughing.generate(model, tool, machine, &p)
        }
        other => anyhow::bail!("strategy \"{other}\" is not implemented"),
    }
}

/// Inject the op-level `stock_to_leave` into a strategy params blob.
/// Mirrors the implementation in `app.rs` so the worker and the foreground
/// path treat the field identically.
fn inject_stock_to_leave(params: &serde_json::Value, value: f64) -> serde_json::Value {
    let mut out = params.clone();
    if let Some(obj) = out.as_object_mut() {
        if !obj.contains_key("stock_to_leave") {
            obj.insert("stock_to_leave".into(), serde_json::Value::from(value));
        }
    }
    out
}

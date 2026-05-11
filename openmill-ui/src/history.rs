//! Undo / redo for `Job` state.
//!
//! Snapshot-based: every frame we serialise the current `Job` and compare
//! against the last committed snapshot. A diff opens a quiet-window — when
//! the job stops changing for [`COMMIT_DEBOUNCE_MS`] the diff is pushed as a
//! single undo step. This coalesces continuous edits (dragging a slider) into
//! one entry while still capturing every discrete change.

use std::collections::VecDeque;
use std::time::Instant;

use openmill_core::Job;

/// Maximum number of past states kept in memory. Each entry is ~a few KB of
/// JSON, so 100 entries ≈ <1 MB.
const MAX_STACK: usize = 100;

/// How long the job must stop changing before the diff is committed as one
/// undo step. Long enough to swallow a slider drag, short enough that a
/// quick discrete edit lands before the user hits Ctrl+Z.
const COMMIT_DEBOUNCE_MS: u128 = 400;

pub struct History {
    past: VecDeque<Vec<u8>>,
    future: Vec<Vec<u8>>,
    /// Bytes of the most-recently committed `Job`. Compared against the live
    /// job each frame to detect changes.
    last_committed: Vec<u8>,
    /// First time we observed an uncommitted diff. `None` while in sync.
    dirty_since: Option<Instant>,
}

impl History {
    pub fn new(job: &Job) -> Self {
        Self {
            past: VecDeque::new(),
            future: Vec::new(),
            last_committed: serialize(job),
            dirty_since: None,
        }
    }

    /// Replace the baseline with `job` and clear all history. Call after
    /// `New Job` / `Open Job` so the prior session isn't reachable via undo.
    pub fn reset(&mut self, job: &Job) {
        self.past.clear();
        self.future.clear();
        self.last_committed = serialize(job);
        self.dirty_since = None;
    }

    /// Call once per frame after the UI has run. Commits any diff that's been
    /// stable for at least `COMMIT_DEBOUNCE_MS`.
    pub fn frame(&mut self, job: &Job) {
        let cur = serialize(job);
        if cur == self.last_committed {
            self.dirty_since = None;
            return;
        }
        let now = Instant::now();
        let started = *self.dirty_since.get_or_insert(now);
        if now.duration_since(started).as_millis() >= COMMIT_DEBOUNCE_MS {
            self.commit(cur);
        }
    }

    pub fn can_undo(&self) -> bool {
        !self.past.is_empty() || self.dirty_since.is_some()
    }

    pub fn can_redo(&self) -> bool {
        !self.future.is_empty()
    }

    /// Force-commit any pending diff, then pop the previous state into `job`.
    /// Returns `true` if the job was changed.
    pub fn undo(&mut self, job: &mut Job) -> bool {
        let cur = serialize(job);
        if cur != self.last_committed {
            self.commit(cur);
        }
        let Some(prev) = self.past.pop_back() else { return false };
        let cur = std::mem::replace(&mut self.last_committed, prev.clone());
        self.future.push(cur);
        if let Ok(j) = deserialize(&prev) {
            *job = j;
            true
        } else {
            // Corrupt snapshot — swap back so state stays consistent.
            self.last_committed = self.future.pop().unwrap_or_default();
            self.past.push_back(prev);
            false
        }
    }

    pub fn redo(&mut self, job: &mut Job) -> bool {
        let Some(next) = self.future.pop() else { return false };
        let cur = std::mem::replace(&mut self.last_committed, next.clone());
        self.past.push_back(cur);
        if let Ok(j) = deserialize(&next) {
            *job = j;
            self.dirty_since = None;
            true
        } else {
            self.last_committed = self.past.pop_back().unwrap_or_default();
            self.future.push(next);
            false
        }
    }

    fn commit(&mut self, new_bytes: Vec<u8>) {
        let prev = std::mem::replace(&mut self.last_committed, new_bytes);
        self.past.push_back(prev);
        if self.past.len() > MAX_STACK {
            self.past.pop_front();
        }
        self.future.clear();
        self.dirty_since = None;
    }
}

fn serialize(job: &Job) -> Vec<u8> {
    serde_json::to_vec(job).unwrap_or_default()
}

fn deserialize(bytes: &[u8]) -> Result<Job, serde_json::Error> {
    serde_json::from_slice(bytes)
}

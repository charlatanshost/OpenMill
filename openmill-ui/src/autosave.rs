//! Background autosave + crash recovery.
//!
//! Writes the current `Job` to a known path on a fixed interval. Writes are
//! atomic (write-to-temp, rename) so a crash mid-write can never produce a
//! truncated file. On startup [`load_recovery`] returns the most recent
//! autosave so the app can offer to restore it.

use std::path::{Path, PathBuf};
use std::time::{Instant, SystemTime};

use openmill_core::Job;

/// How often to attempt an autosave write. Cheap on a small job.
const INTERVAL_SECS: u64 = 30;

/// File name for the autosave; lives in [`autosave_dir`].
const FILE_NAME: &str = "autosave.json";

pub struct Autosave {
    last_attempt: Instant,
    last_bytes: Vec<u8>,
}

impl Autosave {
    pub fn new() -> Self {
        Self {
            // Start with `now` so we don't write on the very first frame.
            last_attempt: Instant::now(),
            last_bytes: Vec::new(),
        }
    }

    /// Call each frame. Writes to disk only when the interval has elapsed
    /// AND the job differs from the last successful write.
    pub fn tick(&mut self, job: &Job) -> Option<PathBuf> {
        if self.last_attempt.elapsed().as_secs() < INTERVAL_SECS {
            return None;
        }
        self.last_attempt = Instant::now();

        let bytes = serde_json::to_vec_pretty(job).ok()?;
        if bytes == self.last_bytes {
            return None;
        }
        let path = autosave_path()?;
        if write_atomic(&path, &bytes).is_err() {
            return None;
        }
        self.last_bytes = bytes;
        Some(path)
    }

    /// Forget the current baseline so the next [`tick`] writes again. Use
    /// when the user has saved manually to a different file and we want the
    /// next autosave to reflect the freshly-loaded state.
    pub fn reset_baseline(&mut self) {
        self.last_bytes.clear();
    }

    /// Delete the autosave file. Call after a successful manual save so a
    /// stale recovery prompt doesn't appear next launch.
    pub fn clear() {
        if let Some(p) = autosave_path() {
            let _ = std::fs::remove_file(p);
        }
    }
}

/// Recovery payload found at startup.
pub struct Recovery {
    pub job: Job,
    pub modified: Option<SystemTime>,
}

/// Look for an existing autosave file. Returns the parsed job if one is
/// present and readable. Caller decides whether to prompt the user.
pub fn load_recovery() -> Option<Recovery> {
    let path = autosave_path()?;
    let bytes = std::fs::read(&path).ok()?;
    let job = serde_json::from_slice::<Job>(&bytes).ok()?;
    let modified = std::fs::metadata(&path).ok().and_then(|m| m.modified().ok());
    Some(Recovery { job, modified })
}

/// Per-OS config directory: `%APPDATA%/OpenMill` on Windows,
/// `~/Library/Application Support/OpenMill` on macOS, `$XDG_CONFIG_HOME` or
/// `~/.config/OpenMill` on Linux. Falls back to the system temp dir.
fn autosave_dir() -> Option<PathBuf> {
    let base = if cfg!(target_os = "windows") {
        std::env::var_os("APPDATA").map(PathBuf::from)
    } else if cfg!(target_os = "macos") {
        std::env::var_os("HOME")
            .map(|h| PathBuf::from(h).join("Library/Application Support"))
    } else {
        std::env::var_os("XDG_CONFIG_HOME")
            .map(PathBuf::from)
            .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))
    };
    let base = base.unwrap_or_else(std::env::temp_dir);
    Some(base.join("OpenMill"))
}

fn autosave_path() -> Option<PathBuf> {
    let dir = autosave_dir()?;
    std::fs::create_dir_all(&dir).ok()?;
    Some(dir.join(FILE_NAME))
}

fn write_atomic(path: &Path, bytes: &[u8]) -> std::io::Result<()> {
    let tmp = path.with_extension("json.tmp");
    std::fs::write(&tmp, bytes)?;
    std::fs::rename(&tmp, path)?;
    Ok(())
}

/// Human-friendly relative timestamp ("just now", "5 minutes ago", "2h ago").
pub fn relative_age(t: SystemTime) -> String {
    let dur = SystemTime::now().duration_since(t).unwrap_or_default();
    let s = dur.as_secs();
    if s < 60 {
        "just now".into()
    } else if s < 3600 {
        format!("{} min ago", s / 60)
    } else if s < 86400 {
        format!("{}h {}m ago", s / 3600, (s % 3600) / 60)
    } else {
        format!("{} day(s) ago", s / 86400)
    }
}

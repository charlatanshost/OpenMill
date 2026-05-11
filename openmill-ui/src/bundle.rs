//! `.omp` project bundle — single-file portable project format.
//!
//! An `.omp` file is a ZIP archive with:
//!   - `manifest.json`  — bundle metadata (format version, app version)
//!   - `job.json`       — the serialized `Job`. `model_path` is rewritten
//!                        to the bundled model's filename so the in-archive
//!                        layout is self-contained.
//!   - `model.<ext>`    — the original imported mesh bytes (STL or 3MF).
//!                        Optional: bundles can be saved without a model.
//!
//! Layout is flat (no subdirectories) so it's easy to inspect with any zip
//! tool. The format is forward-compatible: unknown entries are ignored on
//! load, and `manifest.format_version` lets us evolve the schema.

use std::io::{Cursor, Read, Seek, Write};
use std::path::Path;

use anyhow::{Context, Result};
use openmill_core::{import_3mf_reader, import_stl_reader, Job, WorkpieceModel};
use serde::{Deserialize, Serialize};
use zip::write::SimpleFileOptions;
use zip::{ZipArchive, ZipWriter};

const FORMAT_VERSION: u32 = 1;
const APP_NAME: &str = "OpenMill";

#[derive(Debug, Clone, Serialize, Deserialize)]
struct Manifest {
    format_version: u32,
    app: String,
    /// File name (inside the archive) of the embedded model, if any.
    model_file: Option<String>,
    created_at: String,
}

/// One imported model held in memory so the UI can re-pack it into a bundle.
pub struct ModelBlob {
    /// Original filename without directory (e.g. `bracket.stl`).
    pub filename: String,
    /// Lower-case extension without the dot (`stl` or `3mf`).
    pub ext: String,
    /// Raw file bytes as imported.
    pub bytes: Vec<u8>,
}

impl ModelBlob {
    pub fn from_path(path: &Path) -> Result<Self> {
        let bytes = std::fs::read(path)
            .with_context(|| format!("read model: {}", path.display()))?;
        let filename = path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| "model".into());
        let ext = path
            .extension()
            .map(|e| e.to_string_lossy().to_lowercase())
            .unwrap_or_default();
        Ok(Self { filename, ext, bytes })
    }

    /// Parse the in-memory bytes back into a `WorkpieceModel`.
    pub fn parse(&self) -> Result<WorkpieceModel> {
        match self.ext.as_str() {
            "stl" => {
                let mut cur = Cursor::new(&self.bytes);
                import_stl_reader(&mut cur)
            }
            "3mf" => import_3mf_reader(Cursor::new(&self.bytes)),
            other => anyhow::bail!("unsupported model extension: {other}"),
        }
    }
}

/// Save a project bundle. The model blob is optional — bundles can capture
/// jobs that have no imported part yet (e.g. operations on a future stock).
pub fn save_bundle(path: &Path, job: &Job, model: Option<&ModelBlob>) -> Result<()> {
    let file = std::fs::File::create(path)
        .with_context(|| format!("create bundle: {}", path.display()))?;
    let mut zw = ZipWriter::new(file);
    // Deflate is widely supported and gives ~3-10x compression on STL ASCII
    // / JSON. STL binary compresses less but still benefits.
    let opts = SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);

    // ── manifest.json ────────────────────────────────────────────────────
    let manifest = Manifest {
        format_version: FORMAT_VERSION,
        app: APP_NAME.into(),
        model_file: model.map(|m| sanitize_model_name(&m.filename, &m.ext)),
        created_at: chrono_iso_now(),
    };
    zw.start_file("manifest.json", opts).context("write manifest entry")?;
    zw.write_all(&serde_json::to_vec_pretty(&manifest).context("serialize manifest")?)?;

    // ── job.json ─────────────────────────────────────────────────────────
    // Rewrite model_path so on load the relative reference matches the
    // archive entry. Don't touch the caller's Job — clone first.
    let mut job_to_save = job.clone();
    job_to_save.model_path = manifest.model_file.clone();
    zw.start_file("job.json", opts).context("write job entry")?;
    zw.write_all(
        &serde_json::to_vec_pretty(&job_to_save).context("serialize job")?,
    )?;

    // ── model.<ext> ──────────────────────────────────────────────────────
    if let (Some(m), Some(name)) = (model, manifest.model_file.as_deref()) {
        zw.start_file(name, opts).context("write model entry")?;
        zw.write_all(&m.bytes)?;
    }

    zw.finish().context("finalize bundle archive")?;
    Ok(())
}

/// Result of loading a bundle: the embedded job and (optionally) the parsed
/// model with its original bytes.
pub struct LoadedBundle {
    pub job: Job,
    pub model: Option<WorkpieceModel>,
    pub model_blob: Option<ModelBlob>,
}

pub fn load_bundle(path: &Path) -> Result<LoadedBundle> {
    let file = std::fs::File::open(path)
        .with_context(|| format!("open bundle: {}", path.display()))?;
    load_bundle_reader(file)
        .with_context(|| format!("read bundle: {}", path.display()))
}

fn load_bundle_reader<R: Read + Seek>(reader: R) -> Result<LoadedBundle> {
    let mut archive = ZipArchive::new(reader).context("not a valid .omp archive")?;

    // manifest is optional for forward-compat — fall back to scanning the
    // archive for a model file if it's missing.
    let manifest: Option<Manifest> = read_text_entry(&mut archive, "manifest.json")
        .ok()
        .and_then(|bytes| serde_json::from_slice(&bytes).ok());

    let job_bytes = read_text_entry(&mut archive, "job.json")
        .context("bundle is missing job.json")?;
    let job: Job = serde_json::from_slice(&job_bytes).context("job.json parse failed")?;

    let model_name = manifest
        .as_ref()
        .and_then(|m| m.model_file.clone())
        .or_else(|| job.model_path.clone())
        .or_else(|| find_first_model_entry(&mut archive));

    let (model, model_blob) = if let Some(name) = model_name {
        let bytes = match read_text_entry(&mut archive, &name) {
            Ok(b) => b,
            Err(_) => return Ok(LoadedBundle { job, model: None, model_blob: None }),
        };
        let ext = Path::new(&name)
            .extension()
            .map(|e| e.to_string_lossy().to_lowercase())
            .unwrap_or_default();
        let blob = ModelBlob { filename: name, ext, bytes };
        let parsed = blob.parse().ok();
        (parsed, Some(blob))
    } else {
        (None, None)
    };

    Ok(LoadedBundle { job, model, model_blob })
}

fn read_text_entry<R: Read + Seek>(
    archive: &mut ZipArchive<R>,
    name: &str,
) -> Result<Vec<u8>> {
    let mut entry = archive.by_name(name)
        .with_context(|| format!("missing archive entry: {name}"))?;
    let mut buf = Vec::with_capacity(entry.size() as usize);
    entry.read_to_end(&mut buf).context("read archive entry")?;
    Ok(buf)
}

fn find_first_model_entry<R: Read + Seek>(archive: &mut ZipArchive<R>) -> Option<String> {
    (0..archive.len()).find_map(|i| {
        archive.by_index(i).ok().map(|f| f.name().to_string()).filter(|n| {
            let lc = n.to_lowercase();
            lc.ends_with(".stl") || lc.ends_with(".3mf")
        })
    })
}

fn sanitize_model_name(filename: &str, ext: &str) -> String {
    // Strip any directory components and force a known extension, so the
    // archive layout stays flat and predictable.
    let stem = Path::new(filename)
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| "model".into());
    if ext.is_empty() {
        format!("{stem}.stl")
    } else {
        format!("{stem}.{ext}")
    }
}

/// Lightweight ISO-8601 timestamp without pulling in `chrono`.
/// Format: `YYYY-MM-DDTHH:MM:SSZ` (UTC, second precision).
fn chrono_iso_now() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0);
    // Days since civil epoch (1970-01-01). Algorithm from Howard Hinnant.
    let z = secs.div_euclid(86_400);
    let sod = secs.rem_euclid(86_400);
    let (h, m, s) = (sod / 3600, (sod % 3600) / 60, sod % 60);
    let (y, mo, d) = civil_from_days(z);
    format!("{y:04}-{mo:02}-{d:02}T{h:02}:{m:02}:{s:02}Z")
}

fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = (z - era * 146_097) as u64; // [0, 146097)
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146_096) / 365; // [0, 400)
    let y = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let y = if m <= 2 { y + 1 } else { y };
    (y, m, d)
}

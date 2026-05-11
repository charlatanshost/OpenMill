//! Setup sheet generator — printable HTML summary of a job.
//!
//! The setup sheet is intended for the shop floor: a single self-contained
//! `.html` file that lists every tool, every operation, runtime estimates,
//! and program notes. CSS is inlined, no JavaScript, no external assets, so
//! it prints predictably and survives in a project archive.

use openmill_core::{aggregate_metrics, Job, ToolpathMetrics, Toolpath};

/// Conservative rapid speed used for runtime estimates. Matches the value
/// used by the operations panel.
const RAPID_MM_MIN: f64 = 5000.0;

/// Render the setup sheet as a UTF-8 HTML string.
///
/// `toolpaths` is parallel to `job.operations` — an empty inner vec means
/// the op hasn't been generated yet and its row will show "not generated".
pub fn render_html(job: &Job, toolpaths: &[Vec<Toolpath>]) -> String {
    let title = html_escape(&job.name);
    let when = current_local_string();
    let mut total = ToolpathMetrics::default();
    for paths in toolpaths {
        let m = aggregate_metrics(paths, RAPID_MM_MIN);
        total.merge(&m);
    }

    let mut s = String::new();
    s.push_str(&header(&title, &when));

    // ── Project summary ──
    s.push_str("<h1>Setup Sheet</h1>\n");
    s.push_str(&format!(
        "<p class=\"sub\"><strong>{title}</strong> · generated {when}</p>\n"
    ));

    s.push_str("<table class=\"kv\">\n");
    row(&mut s, "Machine", &html_escape(&job.machine.name));
    row(&mut s, "Post-processor", &html_escape(&job.machine.post_processor));
    if let Some(p) = &job.model_path {
        row(&mut s, "Model", &html_escape(p));
    }
    row(&mut s, "Stock", &stock_label(&job.stock));
    row(
        &mut s,
        "Work offset",
        &html_escape(&job.machine.post_config.work_offset),
    );
    row(&mut s, "Safe Z", &format!("{:.2} mm", job.settings.safe_z));
    row(&mut s, "Clearance Z", &format!("{:.2} mm", job.settings.clearance_z));
    row(&mut s, "Tolerance", &format!("{:.3} mm", job.settings.tolerance));
    if total.total_minutes() > 0.0 {
        row(
            &mut s,
            "Estimated runtime",
            &format!(
                "{} (cut {} · rapid {})",
                fmt_minutes(total.total_minutes()),
                fmt_minutes(total.cut_minutes),
                fmt_minutes(total.rapid_minutes),
            ),
        );
        row(
            &mut s,
            "Estimated travel",
            &format!(
                "{:.0} mm ({:.0} cut + {:.0} rapid)",
                total.total_distance_mm(),
                total.cut_distance_mm,
                total.rapid_distance_mm,
            ),
        );
    }
    s.push_str("</table>\n");

    // ── Tools ──
    s.push_str("<h2>Tools</h2>\n");
    if job.tools.is_empty() {
        s.push_str("<p class=\"empty\">No tools in this job.</p>\n");
    } else {
        s.push_str("<table class=\"data\">\n");
        s.push_str("<thead><tr><th>T#</th><th>Name</th><th>Type</th><th>Ø</th><th>Flute</th><th>Used in ops</th></tr></thead>\n<tbody>\n");
        for t in &job.tools {
            let used_in: Vec<String> = job.operations.iter()
                .enumerate()
                .filter(|(_, op)| op.tool_id == t.id)
                .map(|(i, op)| format!("{}. {}", i + 1, html_escape(&op.name)))
                .collect();
            let used = if used_in.is_empty() {
                "<span class=\"muted\">—</span>".to_string()
            } else {
                used_in.join("<br>")
            };
            s.push_str(&format!(
                "<tr><td>T{}</td><td>{}</td><td>{}</td><td>{:.2} mm</td><td>{:.1} mm</td><td>{}</td></tr>\n",
                t.id,
                html_escape(&t.name),
                tool_shape_label(&t.shape),
                t.shape.diameter(),
                t.shape.flute_length(),
                used,
            ));
        }
        s.push_str("</tbody>\n</table>\n");
    }

    // ── Operations ──
    s.push_str("<h2>Operations</h2>\n");
    if job.operations.is_empty() {
        s.push_str("<p class=\"empty\">No operations in this job.</p>\n");
    } else {
        s.push_str("<table class=\"data\">\n");
        s.push_str("<thead><tr><th>#</th><th>Operation</th><th>Strategy</th><th>Tool</th><th>RPM</th><th>Feed</th><th>Coolant</th><th>Time</th><th>Status</th></tr></thead>\n<tbody>\n");
        for (i, op) in job.operations.iter().enumerate() {
            let m = toolpaths.get(i).map(|p| aggregate_metrics(p, RAPID_MM_MIN));
            let status = match (m, op.enabled) {
                (Some(m), true) if m.total_minutes() > 0.0 => "generated".to_string(),
                (_, false) => "disabled".to_string(),
                _ => "not generated".to_string(),
            };
            let time_str = m
                .filter(|m| m.total_minutes() > 0.0)
                .map(|m| fmt_minutes(m.total_minutes()))
                .unwrap_or_else(|| "—".into());
            let tool_label = job.tools.iter()
                .find(|t| t.id == op.tool_id)
                .map(|t| format!("T{} {}", t.id, html_escape(&t.name)))
                .unwrap_or_else(|| format!("T{}", op.tool_id));
            let row_class = if !op.enabled { " class=\"disabled\"" } else { "" };
            s.push_str(&format!(
                "<tr{row_class}><td>{}</td><td>{}</td><td>{}</td><td>{}</td><td>{:.0}</td><td>{:.0} mm/min</td><td>{}</td><td>{}</td><td>{}</td></tr>\n",
                i + 1,
                html_escape(&op.name),
                html_escape(&op.strategy),
                tool_label,
                op.spindle_speed,
                op.feed_rate,
                html_escape(op.coolant.label()),
                time_str,
                status,
            ));
        }
        s.push_str("</tbody>\n</table>\n");
    }

    // ── Fixtures (if any) ──
    if !job.fixtures.is_empty() {
        s.push_str("<h2>Fixtures</h2>\n<ul>\n");
        for fx in &job.fixtures {
            s.push_str(&format!("<li>{}</li>\n", html_escape(&fx.name)));
        }
        s.push_str("</ul>\n");
    }

    s.push_str("<p class=\"footer\">Generated by OpenMill. Estimates exclude tool changes, accel/decel, and dwell.</p>\n");
    s.push_str("</body>\n</html>\n");
    s
}

// ── Building blocks ─────────────────────────────────────────────────────────

fn header(title: &str, when: &str) -> String {
    format!(
        r##"<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8">
<title>Setup Sheet — {title}</title>
<meta name="generator" content="OpenMill">
<meta name="created" content="{when}">
<style>
  body {{ font-family: -apple-system, "Segoe UI", Roboto, sans-serif; color: #1a1a1a;
         max-width: 900px; margin: 24px auto; padding: 0 16px; line-height: 1.4; }}
  h1 {{ margin: 0 0 4px; }}
  h2 {{ margin-top: 28px; border-bottom: 1px solid #ccc; padding-bottom: 4px; }}
  .sub {{ color: #555; margin: 0 0 18px; }}
  .empty {{ color: #888; font-style: italic; }}
  .muted {{ color: #999; }}
  .footer {{ color: #777; font-size: 12px; margin-top: 32px; border-top: 1px solid #eee; padding-top: 8px; }}
  table {{ border-collapse: collapse; width: 100%; margin-top: 4px; }}
  table.kv td {{ padding: 4px 12px 4px 0; vertical-align: top; }}
  table.kv td:first-child {{ color: #555; width: 160px; }}
  table.data {{ font-size: 14px; }}
  table.data th, table.data td {{ border-bottom: 1px solid #eee; padding: 6px 8px; text-align: left; }}
  table.data th {{ background: #f5f5f5; font-weight: 600; }}
  tr.disabled td {{ color: #999; text-decoration: line-through; }}
  @media print {{ body {{ margin: 0; }} h2 {{ page-break-after: avoid; }} tr {{ page-break-inside: avoid; }} }}
</style>
</head><body>
"##
    )
}

fn row(buf: &mut String, k: &str, v: &str) {
    buf.push_str(&format!("<tr><td>{k}</td><td>{v}</td></tr>\n"));
}

fn stock_label(stock: &openmill_core::StockDef) -> String {
    match stock {
        openmill_core::StockDef::BoundingBox { margin } => format!(
            "Bounding box + margin ({:.1}, {:.1}, {:.1}) mm",
            margin[0], margin[1], margin[2]
        ),
        openmill_core::StockDef::Cylinder { diameter, height } => {
            format!("Cylinder Ø {diameter:.1} × {height:.1} mm")
        }
        openmill_core::StockDef::MeshFile { path } => format!("Mesh: {}", html_escape(path)),
    }
}

fn tool_shape_label(shape: &openmill_core::ToolShape) -> &'static str {
    use openmill_core::ToolShape::*;
    match shape {
        FlatEnd { .. }     => "Flat end",
        BallEnd { .. }     => "Ball end",
        BullNose { .. }    => "Bull nose",
        ChamferMill { .. } => "Chamfer / engraver",
        Drill { .. }       => "Drill",
        TaperedMill { .. } => "Tapered",
        Lollipop { .. }    => "Lollipop",
        Dovetail { .. }    => "Dovetail",
        ThreadMill { .. }  => "Thread mill",
    }
}

fn html_escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '&'  => out.push_str("&amp;"),
            '<'  => out.push_str("&lt;"),
            '>'  => out.push_str("&gt;"),
            '"'  => out.push_str("&quot;"),
            '\'' => out.push_str("&#39;"),
            _    => out.push(c),
        }
    }
    out
}

fn fmt_minutes(minutes: f64) -> String {
    let total_secs = (minutes * 60.0).round() as i64;
    let h = total_secs / 3600;
    let m = (total_secs % 3600) / 60;
    let s = total_secs % 60;
    if h > 0 {
        format!("{h}h {m:02}m {s:02}s")
    } else {
        format!("{m:02}m {s:02}s")
    }
}

fn current_local_string() -> String {
    // Same lightweight UTC ISO timestamp as the bundle module — no chrono
    // dependency. Setup sheets show UTC; users who care can re-print with a
    // saved date in the filename.
    use std::time::{SystemTime, UNIX_EPOCH};
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0);
    let z = secs.div_euclid(86_400);
    let sod = secs.rem_euclid(86_400);
    let (h, m, s) = (sod / 3600, (sod % 3600) / 60, sod % 60);
    let (y, mo, d) = civil_from_days(z);
    format!("{y:04}-{mo:02}-{d:02} {h:02}:{m:02}:{s:02} UTC")
}

fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = (z - era * 146_097) as u64;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146_096) / 365;
    let y = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    let y = if m <= 2 { y + 1 } else { y };
    (y, m, d)
}

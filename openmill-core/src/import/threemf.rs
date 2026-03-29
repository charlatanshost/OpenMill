//! 3MF import — ZIP container with RS-274/3MF XML mesh data.
//!
//! 3MF is an OPC (Open Packaging Conventions) ZIP archive.  The primary model
//! file is usually `3D/3dmodel.model`; its XML describes one or more `<object>`
//! resources each with a `<mesh>` containing `<vertices>` and `<triangles>`.
//!
//! This importer finds the first `.model` file, reads all vertices and
//! triangles from all `<mesh>` elements, and returns a single merged
//! [`WorkpieceModel`].  Degenerate triangles are filtered out.

use std::io::{BufReader, Read, Seek};
use std::path::Path;

use anyhow::{Context, Result};
use nalgebra::{Point3, Vector3};
use parry3d::shape::TriMesh;
use quick_xml::events::Event;
use quick_xml::Reader;

use crate::model::{StockShape, WorkpieceModel};

/// Half-cross-product magnitude threshold for degenerate triangle detection.
const DEGENERATE_THRESHOLD: f32 = 1e-10;

// ── Public API ────────────────────────────────────────────────────────────────

/// Import a `.3mf` file as a [`WorkpieceModel`].
///
/// The file is opened as a ZIP archive.  The first `.model` XML entry is
/// parsed for mesh data.  Degenerate triangles are silently dropped.
pub fn import_3mf(path: &Path) -> Result<WorkpieceModel> {
    let file = std::fs::File::open(path)
        .with_context(|| format!("Cannot open 3MF: {}", path.display()))?;

    let mut archive = zip::ZipArchive::new(file)
        .with_context(|| format!("'{}' is not a valid ZIP/3MF archive", path.display()))?;

    let xml_bytes = find_model_xml(&mut archive)
        .with_context(|| format!("No .model XML found in '{}'", path.display()))?;

    build_from_xml(&xml_bytes)
        .with_context(|| format!("Failed to parse 3MF mesh XML from '{}'", path.display()))
}

// ── ZIP extraction ────────────────────────────────────────────────────────────

/// Locate and read the primary `.model` XML file from the archive.
///
/// Checks `3D/3dmodel.model` first; falls back to the first `*.model` entry.
fn find_model_xml<R: Read + Seek>(archive: &mut zip::ZipArchive<R>) -> Result<Vec<u8>> {
    // Try the canonical 3MF path first.
    if let Ok(mut entry) = archive.by_name("3D/3dmodel.model") {
        let mut buf = Vec::new();
        entry.read_to_end(&mut buf).context("read 3D/3dmodel.model")?;
        return Ok(buf);
    }

    // Fall back: find the first *.model entry by collecting names then reading.
    let model_name: Option<String> = (0..archive.len()).find_map(|i| {
        // read_to_end borrows the archive entry; name() is extracted as owned String
        // so the ZipFile borrow is released immediately.
        archive
            .by_index(i)
            .ok()
            .map(|f| f.name().to_string())
            .filter(|n| n.ends_with(".model"))
    });

    let name = model_name.context("no *.model entry found in archive")?;
    let mut entry = archive.by_name(&name)?;
    let mut buf = Vec::new();
    entry.read_to_end(&mut buf).context("read model XML entry")?;
    Ok(buf)
}

// ── XML parsing ───────────────────────────────────────────────────────────────

fn build_from_xml(data: &[u8]) -> Result<WorkpieceModel> {
    let (vertices, indices) = parse_mesh_xml(data)?;

    if vertices.is_empty() {
        anyhow::bail!("3MF mesh contains no vertices");
    }
    if indices.is_empty() {
        anyhow::bail!("3MF mesh contains no (non-degenerate) triangles");
    }

    let mesh = TriMesh::new(vertices, indices);
    let stock = StockShape::BoundingBox {
        margin: Vector3::new(5.0, 5.0, 5.0),
    };
    Ok(WorkpieceModel::new(mesh, stock))
}

/// Parse `<vertices>` and `<triangles>` from a 3MF model XML byte slice.
///
/// Degenerate triangles are filtered during this pass.
fn parse_mesh_xml(data: &[u8]) -> Result<(Vec<Point3<f32>>, Vec<[u32; 3]>)> {
    let mut reader = Reader::from_reader(BufReader::new(data));

    let mut buf = Vec::new();
    let mut vertices: Vec<Point3<f32>> = Vec::new();
    let mut indices: Vec<[u32; 3]> = Vec::new();

    #[derive(PartialEq)]
    enum State { Other, InVertices, InTriangles }
    let mut state = State::Other;

    loop {
        buf.clear();
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => match e.name().as_ref() {
                b"vertices" => state = State::InVertices,
                b"triangles" => state = State::InTriangles,
                b"vertex" if state == State::InVertices => {
                    vertices.push(parse_vertex(e)?);
                }
                b"triangle" if state == State::InTriangles => {
                    let idx = parse_triangle(e)?;
                    push_if_nondegenerate(&vertices, idx, &mut indices);
                }
                _ => {}
            },
            Ok(Event::Empty(ref e)) => match e.name().as_ref() {
                b"vertex" if state == State::InVertices => {
                    vertices.push(parse_vertex(e)?);
                }
                b"triangle" if state == State::InTriangles => {
                    let idx = parse_triangle(e)?;
                    push_if_nondegenerate(&vertices, idx, &mut indices);
                }
                _ => {}
            },
            Ok(Event::End(ref e)) => match e.name().as_ref() {
                b"vertices" | b"triangles" => state = State::Other,
                _ => {}
            },
            Ok(Event::Eof) => break,
            Err(e) => anyhow::bail!("3MF XML parse error: {:?}", e),
            _ => {}
        }
    }

    Ok((vertices, indices))
}

fn push_if_nondegenerate(
    vertices: &[Point3<f32>],
    idx: [u32; 3],
    out: &mut Vec<[u32; 3]>,
) {
    let [i0, i1, i2] = idx.map(|i| i as usize);
    if i0 >= vertices.len() || i1 >= vertices.len() || i2 >= vertices.len() {
        return; // out-of-bounds index — skip silently
    }
    let p0 = vertices[i0].coords;
    let p1 = vertices[i1].coords;
    let p2 = vertices[i2].coords;
    if (p1 - p0).cross(&(p2 - p0)).norm() >= DEGENERATE_THRESHOLD {
        out.push(idx);
    }
}

// ── Attribute helpers ─────────────────────────────────────────────────────────

fn parse_vertex(e: &quick_xml::events::BytesStart<'_>) -> Result<Point3<f32>> {
    let (mut x, mut y, mut z) = (0.0f32, 0.0f32, 0.0f32);
    for attr in e.attributes().flatten() {
        let v: f32 = std::str::from_utf8(attr.value.as_ref())
            .context("non-UTF8 vertex attribute")?
            .trim()
            .parse()
            .context("cannot parse vertex coordinate as f32")?;
        match attr.key.as_ref() {
            b"x" => x = v,
            b"y" => y = v,
            b"z" => z = v,
            _ => {}
        }
    }
    Ok(Point3::new(x, y, z))
}

fn parse_triangle(e: &quick_xml::events::BytesStart<'_>) -> Result<[u32; 3]> {
    let (mut v1, mut v2, mut v3) = (0u32, 0u32, 0u32);
    for attr in e.attributes().flatten() {
        let v: u32 = std::str::from_utf8(attr.value.as_ref())
            .context("non-UTF8 triangle attribute")?
            .trim()
            .parse()
            .context("cannot parse triangle index as u32")?;
        match attr.key.as_ref() {
            b"v1" => v1 = v,
            b"v2" => v2 = v,
            b"v3" => v3 = v,
            _ => {}
        }
    }
    Ok([v1, v2, v3])
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a minimal in-memory 3MF XML string for a single triangle.
    fn single_triangle_xml() -> &'static [u8] {
        br#"<?xml version="1.0" encoding="UTF-8"?>
<model xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
  <resources>
    <object id="1" type="model">
      <mesh>
        <vertices>
          <vertex x="0" y="0" z="0"/>
          <vertex x="10" y="0" z="0"/>
          <vertex x="0" y="10" z="0"/>
        </vertices>
        <triangles>
          <triangle v1="0" v2="1" v3="2"/>
        </triangles>
      </mesh>
    </object>
  </resources>
</model>"#
    }

    #[test]
    fn parse_single_triangle() {
        let (verts, indices) = parse_mesh_xml(single_triangle_xml()).unwrap();
        assert_eq!(verts.len(), 3);
        assert_eq!(indices.len(), 1);
        assert_eq!(indices[0], [0, 1, 2]);
    }

    #[test]
    fn degenerate_triangle_filtered_in_xml() {
        // A valid triangle followed by a degenerate one (v0 == v1 == v2 == 0)
        let xml = br#"<?xml version="1.0" encoding="UTF-8"?>
<model xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
  <resources>
    <object id="1">
      <mesh>
        <vertices>
          <vertex x="0" y="0" z="0"/>
          <vertex x="10" y="0" z="0"/>
          <vertex x="0" y="10" z="0"/>
        </vertices>
        <triangles>
          <triangle v1="0" v2="1" v3="2"/>
          <triangle v1="0" v2="0" v3="0"/>
        </triangles>
      </mesh>
    </object>
  </resources>
</model>"#;
        let (_, indices) = parse_mesh_xml(xml).unwrap();
        assert_eq!(indices.len(), 1, "degenerate triangle should be filtered");
    }
}

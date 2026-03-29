use std::io::{BufReader, Read, Seek};
use std::path::Path;

use anyhow::{Context, Result};
use nalgebra::{Point3, Vector3};
use parry3d::shape::TriMesh;

use crate::model::{StockShape, WorkpieceModel};

/// Half-cross-product magnitude below which a triangle is considered degenerate.
/// The full cross magnitude equals twice the triangle area, so this is ~1e-10 mm².
const DEGENERATE_THRESHOLD: f32 = 1e-10;

// ── Public API ────────────────────────────────────────────────────────────────

/// Import a binary or ASCII STL file as a [`WorkpieceModel`].
///
/// Degenerate triangles (zero-area, collinear, or duplicate vertices) are
/// silently filtered.  The default stock is a bounding-box with 5 mm margin.
pub fn import_stl(path: &Path) -> Result<WorkpieceModel> {
    let file = std::fs::File::open(path)
        .with_context(|| format!("Cannot open STL: {}", path.display()))?;
    let mut reader = BufReader::new(file);
    import_stl_reader(&mut reader)
        .with_context(|| format!("Failed to parse STL: {}", path.display()))
}

/// Import STL from any `Read + Seek` source (useful for testing with in-memory
/// buffers, or importing from an embedded ZIP entry).
pub fn import_stl_reader<R: Read + Seek>(reader: &mut R) -> Result<WorkpieceModel> {
    let indexed = stl_io::read_stl(reader)
        .map_err(|e| anyhow::anyhow!("STL parse error: {:?}", e))?;
    build_from_indexed(indexed)
}

// ── Internal helpers ──────────────────────────────────────────────────────────

fn build_from_indexed(stl: stl_io::IndexedMesh) -> Result<WorkpieceModel> {
    if stl.vertices.is_empty() {
        anyhow::bail!("STL file contains no vertices");
    }

    let vertices: Vec<Point3<f32>> = stl
        .vertices
        .iter()
        .map(|v| Point3::new(v[0], v[1], v[2]))
        .collect();

    let mut indices: Vec<[u32; 3]> = Vec::with_capacity(stl.faces.len());
    let mut n_degenerate = 0usize;

    for face in &stl.faces {
        let [i0, i1, i2] = [face.vertices[0], face.vertices[1], face.vertices[2]];

        if i0 >= vertices.len() || i1 >= vertices.len() || i2 >= vertices.len() {
            anyhow::bail!("STL face references out-of-bounds vertex index");
        }

        let p0 = vertices[i0].coords;
        let p1 = vertices[i1].coords;
        let p2 = vertices[i2].coords;

        // Filter when the triangle has effectively zero area.
        if (p1 - p0).cross(&(p2 - p0)).norm() < DEGENERATE_THRESHOLD {
            n_degenerate += 1;
            continue;
        }

        indices.push([i0 as u32, i1 as u32, i2 as u32]);
    }

    if n_degenerate > 0 {
        log::debug!("STL import: filtered {} degenerate triangle(s)", n_degenerate);
    }

    if indices.is_empty() {
        anyhow::bail!("STL contains no non-degenerate triangles");
    }

    let mesh = TriMesh::new(vertices, indices);
    let stock = StockShape::BoundingBox {
        margin: Vector3::new(5.0, 5.0, 5.0),
    };

    Ok(WorkpieceModel::new(mesh, stock))
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    // ── STL generation helpers ─────────────────────────────────────────────

    // stl_io 0.7 defines its own Vector<f32> newtype (not nalgebra).
    fn v3(x: f32, y: f32, z: f32) -> stl_io::Vertex {
        stl_io::Vector::new([x, y, z])
    }

    fn tri(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> stl_io::Triangle {
        stl_io::Triangle {
            normal: stl_io::Vector::new([0.0, 0.0, 0.0]),
            vertices: [v3(a[0], a[1], a[2]), v3(b[0], b[1], b[2]), v3(c[0], c[1], c[2])],
        }
    }

    /// Build a closed 10×10×10 mm cube as 12 triangles (2 per face).
    fn cube_triangles(s: f32) -> Vec<stl_io::Triangle> {
        vec![
            // bottom  z = 0
            tri([0., 0., 0.], [s, 0., 0.], [s, s, 0.]),
            tri([0., 0., 0.], [s, s, 0.], [0., s, 0.]),
            // top     z = s
            tri([0., 0., s], [s, s, s], [s, 0., s]),
            tri([0., 0., s], [0., s, s], [s, s, s]),
            // front   y = 0
            tri([0., 0., 0.], [s, 0., s], [s, 0., 0.]),
            tri([0., 0., 0.], [0., 0., s], [s, 0., s]),
            // back    y = s
            tri([0., s, 0.], [s, s, 0.], [s, s, s]),
            tri([0., s, 0.], [s, s, s], [0., s, s]),
            // left    x = 0
            tri([0., 0., 0.], [0., s, 0.], [0., s, s]),
            tri([0., 0., 0.], [0., s, s], [0., 0., s]),
            // right   x = s
            tri([s, 0., 0.], [s, s, s], [s, s, 0.]),
            tri([s, 0., 0.], [s, 0., s], [s, s, s]),
        ]
    }

    fn write_stl_bytes(triangles: Vec<stl_io::Triangle>) -> Vec<u8> {
        let mut buf: Vec<u8> = Vec::new();
        stl_io::write_stl(&mut buf, triangles.into_iter())
            .expect("stl_io::write_stl failed");
        buf
    }

    // ── Test: cube import ──────────────────────────────────────────────────

    /// Import a programmatically generated 10×10×10 cube.
    /// stl_io deduplicates vertices on read, so we expect 8 unique corners
    /// and 12 triangle indices.  The AABB extents must be ≈ 10 mm on each axis.
    #[test]
    fn cube_vertex_count_and_aabb() {
        let bytes = write_stl_bytes(cube_triangles(10.0));
        let mut cursor = Cursor::new(bytes);
        let model = import_stl_reader(&mut cursor)
            .expect("import should succeed for a valid cube STL");

        assert_eq!(model.mesh.vertices().len(), 8, "cube has 8 unique corners");
        assert_eq!(model.mesh.indices().len(), 12, "cube has 12 triangles");

        let ext = model.aabb.maxs - model.aabb.mins; // Vector3<f32>
        assert!((ext.x - 10.0).abs() < 0.01, "X extent = {}", ext.x);
        assert!((ext.y - 10.0).abs() < 0.01, "Y extent = {}", ext.y);
        assert!((ext.z - 10.0).abs() < 0.01, "Z extent = {}", ext.z);
    }

    // ── Test: degenerate triangle filtering ────────────────────────────────

    /// An STL containing one valid triangle and one degenerate triangle
    /// (three coincident vertices) should import with only the valid one.
    #[test]
    fn degenerate_triangle_is_filtered() {
        let triangles = vec![
            // Valid: right-angle triangle with area = 50 mm²
            tri([0., 0., 0.], [10., 0., 0.], [0., 10., 0.]),
            // Degenerate: all three vertices at the same point
            tri([5., 5., 5.], [5., 5., 5.], [5., 5., 5.]),
        ];

        let bytes = write_stl_bytes(triangles);
        let mut cursor = Cursor::new(bytes);
        let model = import_stl_reader(&mut cursor)
            .expect("import should succeed; degenerate triangle is dropped");

        assert_eq!(
            model.mesh.indices().len(),
            1,
            "only the valid triangle should remain after filtering"
        );
    }

    /// Collinear vertices (three points on the same line) also produce a
    /// zero-area triangle and must be filtered.
    #[test]
    fn collinear_triangle_is_filtered() {
        let triangles = vec![
            // Valid triangle
            tri([0., 0., 0.], [10., 0., 0.], [0., 10., 0.]),
            // Collinear: all on the X axis
            tri([0., 0., 0.], [5., 0., 0.], [10., 0., 0.]),
        ];

        let bytes = write_stl_bytes(triangles);
        let mut cursor = Cursor::new(bytes);
        let model = import_stl_reader(&mut cursor).unwrap();

        assert_eq!(model.mesh.indices().len(), 1);
    }

    /// An STL containing only degenerate triangles must return an error.
    #[test]
    fn all_degenerate_returns_error() {
        let triangles = vec![
            tri([0., 0., 0.], [0., 0., 0.], [0., 0., 0.]),
        ];
        let bytes = write_stl_bytes(triangles);
        let mut cursor = Cursor::new(bytes);
        let result = import_stl_reader(&mut cursor);
        assert!(result.is_err(), "expected error when all triangles are degenerate");
    }

    // ── Test: stock defaults ───────────────────────────────────────────────

    #[test]
    fn default_stock_is_5mm_bounding_box() {
        let bytes = write_stl_bytes(cube_triangles(10.0));
        let mut cursor = Cursor::new(bytes);
        let model = import_stl_reader(&mut cursor).unwrap();

        match &model.stock {
            StockShape::BoundingBox { margin } => {
                assert!((margin.x - 5.0).abs() < 1e-9);
                assert!((margin.y - 5.0).abs() < 1e-9);
                assert!((margin.z - 5.0).abs() < 1e-9);
            }
            other => panic!("Expected BoundingBox stock, got {:?}", other),
        }
    }
}

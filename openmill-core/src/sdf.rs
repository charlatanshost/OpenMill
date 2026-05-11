//! Signed-distance field generation from a triangle mesh.
//!
//! Used by the simulator's "Verify" view to color the carved stock surface
//! by how far it sits from the nominal part. The SDF is a regular 3-D
//! grid where each cell stores the signed distance from its centre to the
//! mesh, in **millimetres**: positive = outside the part (excess material
//! left), negative = inside the part (gouge).
//!
//! The result ships as `f32` for upload to an `R32Float` 3-D texture —
//! the format is in the default wgpu feature set on every backend, so
//! Verify mode doesn't require an opt-in device feature.

use nalgebra::Point3;
use parry3d::bounding_volume::Aabb;
use parry3d::query::PointQuery;
use parry3d::shape::{TriMesh, TriMeshFlags};
use rayon::prelude::*;

/// Resolution recommended for interactive use. 128³ = 2 M samples; on a
/// modern multi-core CPU `compute_sdf` finishes in a few hundred ms for
/// typical hobby parts. Lift to 256³ if a deviation map needs to resolve
/// sub-half-millimetre features.
pub const DEFAULT_RESOLUTION: [u32; 3] = [128, 128, 128];

/// Compute a signed-distance field on a regular grid sampling the supplied
/// AABB. The returned vector is row-major over `[x, y, z]` (x varies
/// fastest) with `resolution.x * resolution.y * resolution.z` entries,
/// each storing the signed distance in millimetres.
pub fn compute_sdf(
    mesh: &TriMesh,
    aabb: &Aabb,
    resolution: [u32; 3],
) -> Vec<f32> {
    // Inside/outside testing on a TriMesh requires the ORIENTED flag.
    // The caller's mesh may not have it set (importers don't), so make a
    // local copy with the flag enabled. STL parts are typically tiny
    // (< 100k tris) so the clone is cheap.
    let mut oriented = mesh.clone();
    let _ = oriented.set_flags(TriMeshFlags::ORIENTED);
    let mesh = &oriented;

    let [nx, ny, nz] = resolution;
    let total = (nx * ny * nz) as usize;
    let min = aabb.mins;
    let max = aabb.maxs;
    let step = [
        (max.x - min.x) / (nx.max(1) as f32 - 1.0).max(1.0),
        (max.y - min.y) / (ny.max(1) as f32 - 1.0).max(1.0),
        (max.z - min.z) / (nz.max(1) as f32 - 1.0).max(1.0),
    ];

    // Sampling is embarrassingly parallel — distribute Z planes across
    // rayon. Each plane is `nx * ny` queries; on an 8-core box this gives
    // near-linear speedup.
    let plane = (nx * ny) as usize;
    let mut out: Vec<f32> = (0..nz)
        .into_par_iter()
        .flat_map(|kz| {
            let mut row = Vec::with_capacity(plane);
            let z = min.z + step[2] * kz as f32;
            for ky in 0..ny {
                let y = min.y + step[1] * ky as f32;
                for kx in 0..nx {
                    let x = min.x + step[0] * kx as f32;
                    let p = Point3::new(x, y, z);
                    let proj = mesh.project_local_point(&p, true);
                    let dist = (p - proj.point).norm();
                    let signed = if proj.is_inside { -dist } else { dist };
                    row.push(signed);
                }
            }
            row
        })
        .collect();
    out.truncate(total);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3;
    use parry3d::math::Point as PPoint;

    fn unit_cube() -> TriMesh {
        // 1×1×1 cube centred on the origin. Vertices ±0.5 each axis.
        let v: Vec<PPoint<f32>> = vec![
            PPoint::new(-0.5, -0.5, -0.5),
            PPoint::new( 0.5, -0.5, -0.5),
            PPoint::new( 0.5,  0.5, -0.5),
            PPoint::new(-0.5,  0.5, -0.5),
            PPoint::new(-0.5, -0.5,  0.5),
            PPoint::new( 0.5, -0.5,  0.5),
            PPoint::new( 0.5,  0.5,  0.5),
            PPoint::new(-0.5,  0.5,  0.5),
        ];
        let i = vec![
            [0, 2, 1], [0, 3, 2], // -Z
            [4, 5, 6], [4, 6, 7], // +Z
            [0, 4, 7], [0, 7, 3], // -X
            [1, 2, 6], [1, 6, 5], // +X
            [0, 1, 5], [0, 5, 4], // -Y
            [3, 7, 6], [3, 6, 2], // +Y
        ];
        TriMesh::new(v, i)
    }

    #[test]
    fn cube_sdf_signs_are_correct() {
        let mesh = unit_cube();
        let aabb = Aabb::new(PPoint::new(-1.0, -1.0, -1.0), PPoint::new(1.0, 1.0, 1.0));
        let res = [16, 16, 16];
        let sdf = compute_sdf(&mesh, &aabb, res);

        let at = |x: usize, y: usize, z: usize| -> f32 {
            sdf[(z * res[1] as usize + y) * res[0] as usize + x]
        };
        // Centre of the AABB lies inside the cube → negative.
        let centre = at(8, 8, 8);
        assert!(centre < 0.0, "centre should report negative (inside), got {centre}");
        // A corner of the AABB is well outside the cube → positive.
        let corner = at(0, 0, 0);
        assert!(corner > 0.0, "AABB corner should be outside cube → positive, got {corner}");
    }

    #[test]
    fn sdf_resolution_matches_request() {
        let mesh = unit_cube();
        let aabb = Aabb::new(PPoint::new(-1.0, -1.0, -1.0), PPoint::new(1.0, 1.0, 1.0));
        let res = [8, 8, 8];
        let sdf = compute_sdf(&mesh, &aabb, res);
        assert_eq!(sdf.len(), 512);
        let _ = Vector3::new(0.0_f32, 0.0, 0.0);
    }
}

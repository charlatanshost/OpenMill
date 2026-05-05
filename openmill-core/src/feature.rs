//! Geometric feature recognition.
//!
//! A [`Feature`] is a recognised piece of part geometry — a hole, a pocket, a
//! flat face — that the user can assign to an operation. Features sit between
//! the raw mesh (which knows nothing about CAM intent) and the strategies
//! (which need explicit geometric inputs like hole positions or pocket
//! boundaries).
//!
//! Today this module provides automatic **hole detection** from a triangle
//! mesh — finding clusters of near-vertical-normal triangles whose normals
//! point inward toward a common axis. Pocket and boss detection are
//! placeholders that future work can fill in.

use std::collections::HashMap;

use nalgebra::{Point3, Vector3};
use parry3d::query::{Ray, RayCast};
use parry3d::shape::TriMesh;
use serde::{Deserialize, Serialize};

use crate::strategies::drilling::Hole;

// ── Feature ──────────────────────────────────────────────────────────────────

/// A recognised geometric feature on the part. Features are referenced by id
/// from operation params (e.g. a Drilling op points at one or more `Hole`
/// features rather than embedding the holes inline).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Feature {
    pub id: u32,
    pub label: String,
    pub kind: FeatureKind,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum FeatureKind {
    /// Cylindrical hole detected from a ring of vertical-normal triangles.
    Hole {
        /// Top-centre of the hole in part coordinates.
        position: Point3<f64>,
        /// Unit vector from the top into the hole (normally `−Z` for a
        /// vertical drilled hole).
        axis: Vector3<f64>,
        /// Cylinder diameter [mm].
        diameter: f64,
        /// Hole depth [mm].
        depth: f64,
    },
    /// Closed top-face pocket — a horizontal floor face with no material
    /// directly above it for some Z extent, suggesting a milled-out cavity.
    Pocket {
        /// Centre of the pocket's footprint [mm].
        center: Point3<f64>,
        /// Z height of the pocket floor [mm].
        floor_z: f64,
        /// Z height of the open top [mm].
        top_z: f64,
        /// Approximate radius of the pocket footprint [mm].
        radius: f64,
        /// Approximate floor area [mm²].
        area: f64,
    },
    /// A large-area flat face (e.g. a top face for facing or a step-down
    /// reference). Placeholder for future facing-strategy integration.
    FlatFace {
        center: Point3<f64>,
        normal: Vector3<f64>,
        area: f64,
    },
}

impl Feature {
    pub fn type_label(&self) -> &'static str {
        match self.kind {
            FeatureKind::Hole { .. }     => "Hole",
            FeatureKind::Pocket { .. }   => "Pocket",
            FeatureKind::FlatFace { .. } => "Flat",
        }
    }
}

/// Convert a [`FeatureKind::Hole`] feature into the [`Hole`] type that the
/// drilling strategy consumes. Returns `None` for non-hole features.
pub fn feature_to_hole(f: &Feature) -> Option<Hole> {
    match f.kind {
        FeatureKind::Hole { position, axis, diameter: _, depth } => Some(Hole {
            position,
            axis,
            depth,
        }),
        _ => None,
    }
}

// ── Auto detection ───────────────────────────────────────────────────────────

/// Detect cylindrical holes in a triangle mesh.
///
/// Heuristic:
/// 1. Classify each triangle as a "wall" if its surface normal is near-
///    horizontal (`|nz| < ~0.35`).
/// 2. Build edge-adjacency between wall triangles and run BFS to find
///    connected components — each component is a candidate hole wall.
/// 3. Filter components by triangle count, and by the requirement that most
///    of the cluster's normals point **inward** toward the cluster's XY
///    centroid (filters out outer shell faces).
/// 4. Estimate hole `(x, y)` centre as the centroid of cluster vertices,
///    `z_top`/`z_bot` from the cluster's vertical extent, and diameter as
///    `2 ·` the average horizontal distance from the axis.
///
/// Returns `Hole` features in part-local coordinates. Vertical-axis only —
/// inclined holes need a different detector (future work).
pub fn detect_holes(mesh: &TriMesh) -> Vec<Feature> {
    let verts = mesh.vertices();
    let tris  = mesh.indices();
    let n_tri = tris.len();
    if n_tri == 0 { return Vec::new(); }

    // ── Per-triangle normal + wall classification ───────────────────────────
    let mut normals = Vec::with_capacity(n_tri);
    let mut is_wall = vec![false; n_tri];
    for t in tris {
        let p0 = verts[t[0] as usize];
        let p1 = verts[t[1] as usize];
        let p2 = verts[t[2] as usize];
        let raw = (p1 - p0).cross(&(p2 - p0));
        let len = raw.norm();
        let n = if len > 1e-8 { raw / len } else { Vector3::z() };
        normals.push(n);
    }
    for (i, n) in normals.iter().enumerate() {
        is_wall[i] = n.z.abs() < 0.35;
    }

    // ── Edge → triangle adjacency ───────────────────────────────────────────
    let mut edge_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (i, t) in tris.iter().enumerate() {
        let edges = [
            (t[0].min(t[1]), t[0].max(t[1])),
            (t[1].min(t[2]), t[1].max(t[2])),
            (t[2].min(t[0]), t[2].max(t[0])),
        ];
        for e in edges {
            edge_tris.entry(e).or_default().push(i);
        }
    }
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n_tri];
    for v in edge_tris.values() {
        if v.len() == 2 && is_wall[v[0]] && is_wall[v[1]] {
            adj[v[0]].push(v[1]);
            adj[v[1]].push(v[0]);
        }
    }

    // ── Connected components of wall triangles ─────────────────────────────
    let mut visited = vec![false; n_tri];
    let mut features: Vec<Feature> = Vec::new();
    let mut next_id: u32 = 1;

    for seed in 0..n_tri {
        if !is_wall[seed] || visited[seed] { continue; }
        let mut stack = vec![seed];
        let mut cluster: Vec<usize> = Vec::new();
        while let Some(t) = stack.pop() {
            if visited[t] { continue; }
            visited[t] = true;
            cluster.push(t);
            for &nb in &adj[t] {
                if !visited[nb] { stack.push(nb); }
            }
        }
        // A reasonable hole has at least a handful of triangles around the ring.
        if cluster.len() < 6 { continue; }

        // Cluster centroid (XY) and vertical extent.
        let mut sum = Vector3::<f32>::zeros();
        let mut count = 0f32;
        let mut z_min = f32::MAX;
        let mut z_max = f32::MIN;
        for &t in &cluster {
            let tri = tris[t];
            for &vi in &tri {
                let v = verts[vi as usize].coords;
                sum += v;
                count += 1.0;
                z_min = z_min.min(v.z);
                z_max = z_max.max(v.z);
            }
        }
        let centroid = sum / count.max(1.0);
        let cx = centroid.x;
        let cy = centroid.y;

        // Average distance from axis (approximate radius).
        let mut radius_sum = 0f32;
        let mut radius_count = 0f32;
        for &t in &cluster {
            let tri = tris[t];
            for &vi in &tri {
                let v = verts[vi as usize].coords;
                let dx = v.x - cx;
                let dy = v.y - cy;
                radius_sum += (dx * dx + dy * dy).sqrt();
                radius_count += 1.0;
            }
        }
        let radius = radius_sum / radius_count.max(1.0);
        let diameter = (radius * 2.0) as f64;
        if diameter < 0.2 || diameter > 200.0 { continue; }

        // Inward-normal check: a hole's wall normals point toward the
        // cluster's central axis. The outer shell of the part has wall normals
        // pointing OUTWARD (away from its centroid), so we filter those out.
        let mut inward = 0i32;
        let mut total = 0i32;
        for &t in &cluster {
            let tri = tris[t];
            // Triangle centroid in XY.
            let p0 = verts[tri[0] as usize].coords;
            let p1 = verts[tri[1] as usize].coords;
            let p2 = verts[tri[2] as usize].coords;
            let tc = (p0 + p1 + p2) / 3.0;
            let to_axis = Vector3::new(cx - tc.x, cy - tc.y, 0.0);
            let mag = to_axis.norm();
            if mag < 1e-6 { continue; }
            let to_axis = to_axis / mag;
            let n = normals[t];
            let n_xy = Vector3::new(n.x, n.y, 0.0);
            let n_xy_mag = n_xy.norm();
            if n_xy_mag < 1e-6 { continue; }
            let n_xy = n_xy / n_xy_mag;
            if n_xy.dot(&to_axis) > 0.5 { inward += 1; }
            total += 1;
        }
        if total == 0 || (inward as f32 / total as f32) < 0.7 {
            continue; // not a hole — likely an outer shell
        }

        let depth = (z_max - z_min) as f64;
        if depth < 0.5 { continue; }

        let label = format!("Ø{:.2} × {:.1}mm hole @ ({:.1}, {:.1})", diameter, depth, cx, cy);
        features.push(Feature {
            id: next_id,
            label,
            kind: FeatureKind::Hole {
                position: Point3::new(cx as f64, cy as f64, z_max as f64),
                axis: Vector3::new(0.0, 0.0, -1.0),
                diameter,
                depth,
            },
        });
        next_id += 1;
    }

    features
}

/// Detect open-top pockets in a triangle mesh.
///
/// Heuristic:
/// 1. Classify each triangle as a "floor" if its normal points strongly
///    upward (`nz > 0.8`).
/// 2. Group floor triangles into connected components by edge adjacency.
/// 3. For each cluster, raycast straight up from the centroid. If the ray
///    misses the mesh (or hits something far above), the floor is "open" →
///    it's a pocket. If the ray hits the mesh shortly above, it's a step
///    or a sealed cavity, not a pocket.
/// 4. Filter by area and depth to skip tiny faces.
pub fn detect_pockets(mesh: &TriMesh) -> Vec<Feature> {
    let verts = mesh.vertices();
    let tris  = mesh.indices();
    let n_tri = tris.len();
    if n_tri == 0 { return Vec::new(); }

    let aabb = mesh.local_aabb();
    let model_top = aabb.maxs.z;

    // Per-triangle normal + floor classification.
    let mut normals = Vec::with_capacity(n_tri);
    let mut is_floor = vec![false; n_tri];
    for t in tris {
        let p0 = verts[t[0] as usize];
        let p1 = verts[t[1] as usize];
        let p2 = verts[t[2] as usize];
        let raw = (p1 - p0).cross(&(p2 - p0));
        let len = raw.norm();
        let n = if len > 1e-8 { raw / len } else { Vector3::z() };
        normals.push(n);
    }
    for (i, n) in normals.iter().enumerate() {
        // Up-facing AND below the model's absolute top — the very top face
        // isn't a pocket, it's just the part's roof.
        if n.z > 0.8 {
            let tri = tris[i];
            let z = verts[tri[0] as usize].z
                .min(verts[tri[1] as usize].z)
                .min(verts[tri[2] as usize].z);
            if z < model_top - 0.1 {
                is_floor[i] = true;
            }
        }
    }

    // Edge → triangle adjacency, restricted to floor triangles for components.
    let mut edge_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (i, t) in tris.iter().enumerate() {
        let edges = [
            (t[0].min(t[1]), t[0].max(t[1])),
            (t[1].min(t[2]), t[1].max(t[2])),
            (t[2].min(t[0]), t[2].max(t[0])),
        ];
        for e in edges {
            edge_tris.entry(e).or_default().push(i);
        }
    }
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); n_tri];
    for v in edge_tris.values() {
        if v.len() == 2 && is_floor[v[0]] && is_floor[v[1]] {
            // Same-Z floors only — adjacent steps at different heights are
            // separate pockets.
            let za = avg_z(verts, &tris[v[0]]);
            let zb = avg_z(verts, &tris[v[1]]);
            if (za - zb).abs() < 0.05 {
                adj[v[0]].push(v[1]);
                adj[v[1]].push(v[0]);
            }
        }
    }

    let mut visited = vec![false; n_tri];
    let mut features: Vec<Feature> = Vec::new();
    let mut next_id: u32 = 1;

    for seed in 0..n_tri {
        if !is_floor[seed] || visited[seed] { continue; }
        let mut stack = vec![seed];
        let mut cluster: Vec<usize> = Vec::new();
        while let Some(t) = stack.pop() {
            if visited[t] { continue; }
            visited[t] = true;
            cluster.push(t);
            for &nb in &adj[t] {
                if !visited[nb] { stack.push(nb); }
            }
        }
        if cluster.len() < 2 { continue; }

        // Cluster floor area + centroid (in XY).
        let mut area_sum = 0f32;
        let mut cx = 0f32;
        let mut cy = 0f32;
        let mut cz = 0f32;
        for &t in &cluster {
            let tri = tris[t];
            let p0 = verts[tri[0] as usize].coords;
            let p1 = verts[tri[1] as usize].coords;
            let p2 = verts[tri[2] as usize].coords;
            let a = (p1 - p0).cross(&(p2 - p0)).norm() * 0.5;
            area_sum += a;
            let c = (p0 + p1 + p2) / 3.0;
            cx += c.x * a; cy += c.y * a; cz += c.z * a;
        }
        if area_sum < 1.0 { continue; } // ignore < 1 mm² scraps
        let inv = 1.0 / area_sum;
        cx *= inv; cy *= inv; cz *= inv;

        // Approximate radius from area.
        let radius = (area_sum / std::f32::consts::PI).sqrt();

        // Open-top check: cast a ray UP from the cluster centroid (slightly
        // above the floor) and see whether anything sits above it. If not,
        // this floor opens directly to the model top → it's a pocket.
        let origin  = parry3d::math::Point::new(cx, cy, cz + 0.05);
        let ray     = Ray::new(origin, Vector3::new(0.0, 0.0, 1.0));
        let toi_opt = mesh.cast_local_ray(&ray, 1e6, true);
        let open_top_z = match toi_opt {
            None => model_top,                  // ray escapes — fully open
            Some(toi) => origin.z + toi,        // hit something — capped here
        };
        let depth = (open_top_z - cz).max(0.0);
        // Reject if there's < 0.5 mm of clearance above the floor — it's a
        // step face, not a pocket.
        if depth < 0.5 { continue; }

        let label = format!(
            "Pocket r≈{:.1}mm × {:.1}mm deep @ ({:.1}, {:.1})",
            radius, depth, cx, cy,
        );
        features.push(Feature {
            id: next_id,
            label,
            kind: FeatureKind::Pocket {
                center:  Point3::new(cx as f64, cy as f64, cz as f64),
                floor_z: cz as f64,
                top_z:   open_top_z as f64,
                radius:  radius as f64,
                area:    area_sum as f64,
            },
        });
        next_id += 1;
    }
    features
}

fn avg_z(verts: &[parry3d::math::Point<f32>], tri: &[u32; 3]) -> f32 {
    (verts[tri[0] as usize].z + verts[tri[1] as usize].z + verts[tri[2] as usize].z) / 3.0
}

// ── Manual face picking ──────────────────────────────────────────────────────

/// Result of a face pick: the seed triangle, every triangle the flood-fill
/// reached, and the cluster's averaged surface properties.
#[derive(Debug, Clone)]
pub struct PickedFace {
    /// Triangle hit directly by the pick ray.
    pub seed_tri: usize,
    /// Triangles in the flood-fill cluster (includes `seed_tri`).
    pub cluster_tris: Vec<usize>,
    /// Area-weighted centroid of the cluster [mm].
    pub centroid: Point3<f64>,
    /// Area-weighted average outward normal of the cluster (unit length).
    pub normal: Vector3<f64>,
    /// Total surface area of the cluster [mm²].
    pub area: f64,
}

impl PickedFace {
    /// Convert this pick into a [`Feature`]. Auto-classifies by the cluster
    /// normal direction:
    ///
    /// - `n.z > 0.7`  → an open-top pocket (floor-style) when below the model
    ///   top, or a flat face otherwise.
    /// - `|n.z| < 0.35` and the cluster spans some depth → a hole
    ///   (vertical wall).
    /// - everything else → a generic flat face.
    pub fn to_feature(
        &self,
        mesh: &TriMesh,
        next_id: u32,
    ) -> Feature {
        let model_top = mesh.local_aabb().maxs.z as f64;

        // Cluster Z extent from the actual triangles.
        let mut z_min = f64::MAX;
        let mut z_max = f64::MIN;
        for &t in &self.cluster_tris {
            let tri = mesh.indices()[t];
            for &vi in &tri {
                let z = mesh.vertices()[vi as usize].z as f64;
                z_min = z_min.min(z);
                z_max = z_max.max(z);
            }
        }
        let depth = z_max - z_min;

        // Classify.
        if self.normal.z > 0.7 && self.centroid.z < model_top - 0.1 {
            // Pocket-like floor. Approximate radius from area (πr² = area).
            let radius = (self.area / std::f64::consts::PI).sqrt();
            return Feature {
                id: next_id,
                label: format!(
                    "Picked pocket r≈{:.1}mm × {:.1}mm deep @ ({:.1}, {:.1})",
                    radius, model_top - self.centroid.z, self.centroid.x, self.centroid.y,
                ),
                kind: FeatureKind::Pocket {
                    center: self.centroid,
                    floor_z: self.centroid.z,
                    top_z:   model_top,
                    radius,
                    area:    self.area,
                },
            };
        }
        if self.normal.z.abs() < 0.35 && depth > 0.5 {
            // Cluster of vertical-wall triangles → treat as a hole.
            // Hole axis points down through the cluster; diameter approximated
            // from the cluster's XY span.
            let mut xs = (f32::MAX, f32::MIN);
            let mut ys = (f32::MAX, f32::MIN);
            for &t in &self.cluster_tris {
                let tri = mesh.indices()[t];
                for &vi in &tri {
                    let v = mesh.vertices()[vi as usize];
                    xs.0 = xs.0.min(v.x); xs.1 = xs.1.max(v.x);
                    ys.0 = ys.0.min(v.y); ys.1 = ys.1.max(v.y);
                }
            }
            let cx = ((xs.0 + xs.1) * 0.5) as f64;
            let cy = ((ys.0 + ys.1) * 0.5) as f64;
            let diameter = (((xs.1 - xs.0) as f64).max((ys.1 - ys.0) as f64)).max(0.2);
            return Feature {
                id: next_id,
                label: format!("Picked Ø{:.2} × {:.1}mm hole @ ({:.1}, {:.1})", diameter, depth, cx, cy),
                kind: FeatureKind::Hole {
                    position: Point3::new(cx, cy, z_max),
                    axis:     Vector3::new(0.0, 0.0, -1.0),
                    diameter,
                    depth,
                },
            };
        }
        // Generic flat face.
        Feature {
            id: next_id,
            label: format!(
                "Picked flat face area {:.1}mm² @ ({:.1}, {:.1}, {:.1})",
                self.area, self.centroid.x, self.centroid.y, self.centroid.z,
            ),
            kind: FeatureKind::FlatFace {
                center: self.centroid,
                normal: self.normal,
                area:   self.area,
            },
        }
    }
}

/// Pick a face by raycasting against `mesh` and flood-filling neighbouring
/// triangles whose normals stay within `angle_tol_deg` of the seed normal.
///
/// Returns `None` if the ray misses the mesh. Edge adjacency is built on
/// every call — for very large meshes a future revision could cache this.
pub fn pick_face(
    mesh: &TriMesh,
    ray_origin: Point3<f32>,
    ray_dir: Vector3<f32>,
    angle_tol_deg: f64,
) -> Option<PickedFace> {
    use parry3d::query::{Ray, RayCast};

    let ray = Ray::new(ray_origin.into(), ray_dir);
    let toi = mesh.cast_local_ray(&ray, 1e9, true)?;
    let _hit = ray.point_at(toi);
    // Find which triangle was hit. parry3d's ray cast returns TOI but not the
    // feature index for a TriMesh; we re-test by iterating.
    // For meshes < ~50k tris this is fine; large meshes should swap to a BVH
    // feature query.
    let verts = mesh.vertices();
    let tris  = mesh.indices();
    let mut hit_tri: Option<usize> = None;
    let mut hit_normal = Vector3::z();
    let mut best_t = f32::MAX;
    for (i, t) in tris.iter().enumerate() {
        let p0 = verts[t[0] as usize];
        let p1 = verts[t[1] as usize];
        let p2 = verts[t[2] as usize];
        let raw = (p1 - p0).cross(&(p2 - p0));
        let len = raw.norm();
        if len < 1e-8 { continue; }
        let n = raw / len;
        // Möller–Trumbore.
        let edge1 = p1 - p0;
        let edge2 = p2 - p0;
        let h = ray_dir.cross(&edge2);
        let a = edge1.dot(&h);
        if a.abs() < 1e-8 { continue; }
        let f = 1.0 / a;
        let s = ray.origin - p0;
        let u = f * s.dot(&h);
        if !(0.0..=1.0).contains(&u) { continue; }
        let q = s.cross(&edge1);
        let v = f * ray_dir.dot(&q);
        if v < 0.0 || u + v > 1.0 { continue; }
        let tt = f * edge2.dot(&q);
        if tt > 1e-6 && tt < best_t {
            best_t = tt;
            hit_tri = Some(i);
            hit_normal = n.cast::<f32>();
        }
    }
    let seed = hit_tri?;

    // Flood fill across edge-adjacent triangles whose normal stays within
    // `angle_tol` of the seed.
    let cos_tol = (angle_tol_deg.to_radians()).cos() as f32;

    // Per-tri normals (computed only for cluster check).
    let mut normals: Vec<Vector3<f32>> = Vec::with_capacity(tris.len());
    for t in tris {
        let p0 = verts[t[0] as usize];
        let p1 = verts[t[1] as usize];
        let p2 = verts[t[2] as usize];
        let raw = (p1 - p0).cross(&(p2 - p0));
        let len = raw.norm();
        normals.push(if len > 1e-8 { raw / len } else { Vector3::z() });
    }

    // Edge → tri adjacency.
    let mut edge_tris: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (i, t) in tris.iter().enumerate() {
        let edges = [
            (t[0].min(t[1]), t[0].max(t[1])),
            (t[1].min(t[2]), t[1].max(t[2])),
            (t[2].min(t[0]), t[2].max(t[0])),
        ];
        for e in edges {
            edge_tris.entry(e).or_default().push(i);
        }
    }
    let mut adj: Vec<Vec<usize>> = vec![Vec::new(); tris.len()];
    for v in edge_tris.values() {
        if v.len() == 2 {
            adj[v[0]].push(v[1]);
            adj[v[1]].push(v[0]);
        }
    }

    let seed_normal = hit_normal;
    let mut visited = vec![false; tris.len()];
    let mut cluster = Vec::new();
    let mut stack = vec![seed];
    while let Some(t) = stack.pop() {
        if visited[t] { continue; }
        if normals[t].dot(&seed_normal) < cos_tol { continue; }
        visited[t] = true;
        cluster.push(t);
        for &nb in &adj[t] { if !visited[nb] { stack.push(nb); } }
    }

    // Area-weighted centroid + average normal.
    let mut area_sum = 0f32;
    let mut centroid = Vector3::<f32>::zeros();
    let mut normal_sum = Vector3::<f32>::zeros();
    for &t in &cluster {
        let tri = tris[t];
        let p0 = verts[tri[0] as usize].coords;
        let p1 = verts[tri[1] as usize].coords;
        let p2 = verts[tri[2] as usize].coords;
        let a = (p1 - p0).cross(&(p2 - p0)).norm() * 0.5;
        area_sum += a;
        let c = (p0 + p1 + p2) / 3.0;
        centroid += c * a;
        normal_sum += normals[t] * a;
    }
    let inv = 1.0 / area_sum.max(1e-6);
    centroid *= inv;
    let normal = if normal_sum.norm() > 1e-6 { normal_sum.normalize() } else { seed_normal };

    Some(PickedFace {
        seed_tri: seed,
        cluster_tris: cluster,
        centroid: Point3::new(centroid.x as f64, centroid.y as f64, centroid.z as f64),
        normal: Vector3::new(normal.x as f64, normal.y as f64, normal.z as f64),
        area: area_sum as f64,
    })
}

// ── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use parry3d::math::Point as PPoint;

    /// Build a simple "block with a cylindrical hole" mesh.
    fn block_with_hole(w: f32, d: f32, h: f32, hole_r: f32, hole_seg: usize) -> TriMesh {
        let mut verts: Vec<PPoint<f32>> = Vec::new();
        let mut tris:  Vec<[u32; 3]> = Vec::new();

        // 8 corners of the outer block (centred on origin in XY).
        let hx = w * 0.5;
        let hy = d * 0.5;
        let v_outer = [
            PPoint::new(-hx, -hy, 0.0), PPoint::new( hx, -hy, 0.0),
            PPoint::new( hx,  hy, 0.0), PPoint::new(-hx,  hy, 0.0),
            PPoint::new(-hx, -hy, h),   PPoint::new( hx, -hy, h),
            PPoint::new( hx,  hy, h),   PPoint::new(-hx,  hy, h),
        ];
        for v in v_outer { verts.push(v); }
        // 4 outer side faces (we don't need top/bottom for the test).
        // Front (-Y), Right (+X), Back (+Y), Left (-X). 2 tris each.
        tris.push([0, 1, 5]); tris.push([0, 5, 4]);
        tris.push([1, 2, 6]); tris.push([1, 6, 5]);
        tris.push([2, 3, 7]); tris.push([2, 7, 6]);
        tris.push([3, 0, 4]); tris.push([3, 4, 7]);

        // Cylindrical hole at the centre, going all the way through.
        // Two rings of vertices (top and bottom), `hole_seg` segments each,
        // normals pointing INWARD (toward axis).
        let base_top = verts.len() as u32;
        for i in 0..hole_seg {
            let a = (i as f32) / (hole_seg as f32) * std::f32::consts::TAU;
            verts.push(PPoint::new(hole_r * a.cos(), hole_r * a.sin(), h));
        }
        let base_bot = verts.len() as u32;
        for i in 0..hole_seg {
            let a = (i as f32) / (hole_seg as f32) * std::f32::consts::TAU;
            verts.push(PPoint::new(hole_r * a.cos(), hole_r * a.sin(), 0.0));
        }
        for i in 0..hole_seg {
            let i_next = (i + 1) % hole_seg;
            let t0 = base_top + i as u32;
            let t1 = base_top + i_next as u32;
            let b0 = base_bot + i as u32;
            let b1 = base_bot + i_next as u32;
            // Wind so the surface normal points TOWARD the axis (inward) —
            // that's what the hole detector looks for.
            tris.push([t0, b1, b0]);
            tris.push([t0, t1, b1]);
        }
        TriMesh::new(verts, tris)
    }

    #[test]
    fn detects_a_single_through_hole() {
        let mesh = block_with_hole(40.0, 30.0, 10.0, 3.0, 24);
        let features = detect_holes(&mesh);
        assert_eq!(features.len(), 1, "expected exactly one detected hole");
        if let FeatureKind::Hole { diameter, depth, .. } = features[0].kind {
            assert!((diameter - 6.0).abs() < 0.5, "diameter ≈ 6mm, got {diameter}");
            assert!((depth - 10.0).abs() < 0.5, "depth ≈ 10mm, got {depth}");
        } else {
            panic!("not a hole feature");
        }
    }

    #[test]
    fn pick_face_finds_seed_triangle_and_floods_coplanar_neighbours() {
        // Simple two-triangle co-planar quad on the XY plane at z=0.
        let verts = vec![
            PPoint::<f32>::new(0.0, 0.0, 0.0),
            PPoint::<f32>::new(10.0, 0.0, 0.0),
            PPoint::<f32>::new(10.0, 10.0, 0.0),
            PPoint::<f32>::new(0.0, 10.0, 0.0),
        ];
        let tris = vec![[0u32, 1, 2], [0, 2, 3]];
        let mesh = TriMesh::new(verts, tris);

        // Ray straight down at the quad centre.
        let pick = pick_face(
            &mesh,
            nalgebra::Point3::new(5.0, 5.0, 10.0),
            Vector3::new(0.0, 0.0, -1.0),
            5.0,
        )
        .expect("should hit the quad");
        // Both triangles share the same +Z normal → flood fill picks both.
        assert_eq!(pick.cluster_tris.len(), 2, "flood fill should grab both coplanar tris");
        assert!((pick.normal.z - 1.0).abs() < 1e-6, "normal should be +Z, got {}", pick.normal.z);
        assert!((pick.area - 100.0).abs() < 1e-3, "area should be 10×10 = 100, got {}", pick.area);
    }

    #[test]
    fn ignores_block_with_no_hole() {
        // Block without a hole — should produce no hole features (the outer
        // walls fail the inward-normal test).
        let verts = vec![
            PPoint::new(-10.0, -10.0, 0.0), PPoint::new( 10.0, -10.0, 0.0),
            PPoint::new( 10.0,  10.0, 0.0), PPoint::new(-10.0,  10.0, 0.0),
            PPoint::new(-10.0, -10.0, 5.0), PPoint::new( 10.0, -10.0, 5.0),
            PPoint::new( 10.0,  10.0, 5.0), PPoint::new(-10.0,  10.0, 5.0),
        ];
        let tris = vec![
            [0, 1, 5], [0, 5, 4],
            [1, 2, 6], [1, 6, 5],
            [2, 3, 7], [2, 7, 6],
            [3, 0, 4], [3, 4, 7],
        ];
        let mesh = TriMesh::new(verts, tris);
        let features = detect_holes(&mesh);
        assert!(features.is_empty(), "no holes expected on a plain block, got {:?}", features);
    }
}

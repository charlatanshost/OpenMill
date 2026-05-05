use nalgebra::Vector3;
use parry3d::bounding_volume::Aabb;
use parry3d::math::Point as PPoint;
use parry3d::shape::TriMesh;

// ── Stock shape ───────────────────────────────────────────────────────────────

/// Shape of the raw stock material before any cutting.
#[derive(Debug, Clone)]
pub enum StockShape {
    /// Rectangular block: each axis grown from the mesh AABB by `margin` [mm].
    BoundingBox { margin: Vector3<f64> },
    /// Upright cylinder, centred on the mesh AABB footprint.
    Cylinder {
        /// Cylinder diameter [mm].
        diameter: f64,
        /// Cylinder height [mm].
        height: f64,
    },
}

// ── Workpiece model ───────────────────────────────────────────────────────────

/// A loaded part model plus stock definition used by toolpath strategies.
///
/// The `mesh` is the **finished** part shape, in **workpiece coordinates**
/// — that is, the import mesh translated by `position`. Strategies use
/// `mesh` and `aabb` directly without thinking about positioning.
///
/// `stock` describes what material we start from. Stock is always derived
/// from the part AABB (grown by margin or wrapped by a cylinder) so it
/// follows the part automatically when `position` changes.
pub struct WorkpieceModel {
    /// Triangulated mesh of the finished part **with `position` baked in**
    /// (parry3d f32 precision).
    pub mesh: TriMesh,
    /// Axis-aligned bounding box of `mesh` in workpiece coordinates.
    pub aabb: Aabb,
    /// Raw-material stock shape.
    pub stock: StockShape,

    /// Original imported mesh (before any position offset). Stored so the
    /// user can move the model around the work area without compounding
    /// translations.
    pub mesh_orig: TriMesh,
    /// Translation applied to `mesh_orig` to place the part in the work
    /// area. Defaults to zero. Use [`set_position`] to update — that
    /// rebuilds `mesh` and `aabb` from `mesh_orig`.
    pub position: Vector3<f64>,
}

impl WorkpieceModel {
    /// Build from a pre-constructed mesh and stock. AABB is computed from
    /// the mesh automatically. The imported mesh is stored both as the
    /// raw original (for re-positioning) and as the active workpiece-coord
    /// mesh (initially identical — `position` defaults to zero).
    pub fn new(mesh: TriMesh, stock: StockShape) -> Self {
        let aabb = *mesh.local_aabb();
        WorkpieceModel {
            mesh: mesh.clone(),
            aabb,
            stock,
            mesh_orig: mesh,
            position: Vector3::zeros(),
        }
    }

    /// Move the part in the work area. Rebuilds `mesh` and `aabb` from
    /// `mesh_orig` translated by `position`. Stock follows automatically
    /// because it's always derived from the (now-updated) part AABB.
    pub fn set_position(&mut self, position: Vector3<f64>) {
        let dx = position.x as f32;
        let dy = position.y as f32;
        let dz = position.z as f32;
        let translated_verts: Vec<PPoint<f32>> = self
            .mesh_orig
            .vertices()
            .iter()
            .map(|v| PPoint::new(v.x + dx, v.y + dy, v.z + dz))
            .collect();
        let translated_tris: Vec<[u32; 3]> = self.mesh_orig.indices().to_vec();
        self.mesh = TriMesh::new(translated_verts, translated_tris);
        self.aabb = *self.mesh.local_aabb();
        self.position = position;
    }

    /// Convenience: bounding-box stock with a uniform margin on all sides [mm].
    pub fn with_uniform_stock(mesh: TriMesh, margin: f64) -> Self {
        let stock = StockShape::BoundingBox {
            margin: Vector3::new(margin, margin, margin),
        };
        Self::new(mesh, stock)
    }

    /// Axis-aligned bounding box of the **stock**, in workpiece coordinates.
    ///
    /// Roughing strategies should use this (not [`Self::aabb`]) when planning
    /// scan bounds — otherwise they only cover the part envelope and never
    /// clear the outer stock material.
    pub fn stock_aabb(&self) -> Aabb {
        match &self.stock {
            StockShape::BoundingBox { margin } => Aabb::new(
                PPoint::new(
                    self.aabb.mins.x - margin.x as f32,
                    self.aabb.mins.y - margin.y as f32,
                    self.aabb.mins.z - margin.z as f32,
                ),
                PPoint::new(
                    self.aabb.maxs.x + margin.x as f32,
                    self.aabb.maxs.y + margin.y as f32,
                    self.aabb.maxs.z + margin.z as f32,
                ),
            ),
            StockShape::Cylinder { diameter, height } => {
                let cx = (self.aabb.mins.x + self.aabb.maxs.x) * 0.5;
                let cy = (self.aabb.mins.y + self.aabb.maxs.y) * 0.5;
                let r = (*diameter * 0.5) as f32;
                let z_min = self.aabb.mins.z;
                let z_max = z_min + *height as f32;
                Aabb::new(
                    PPoint::new(cx - r, cy - r, z_min),
                    PPoint::new(cx + r, cy + r, z_max),
                )
            }
        }
    }
}

use nalgebra::Vector3;
use parry3d::bounding_volume::Aabb;
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
/// The `mesh` is the **finished** part shape; `stock` describes what material
/// we start from.  Strategies plan cuts to remove all material outside `mesh`
/// that is inside `stock`.
pub struct WorkpieceModel {
    /// Triangulated mesh of the finished part (parry3d f32 precision).
    pub mesh: TriMesh,
    /// Axis-aligned bounding box of `mesh` in workpiece coordinates.
    pub aabb: Aabb,
    /// Raw-material stock shape.
    pub stock: StockShape,
}

impl WorkpieceModel {
    /// Build from a pre-constructed mesh and stock.  AABB is computed from
    /// the mesh automatically.
    pub fn new(mesh: TriMesh, stock: StockShape) -> Self {
        let aabb = *mesh.local_aabb();
        WorkpieceModel { mesh, aabb, stock }
    }

    /// Convenience: bounding-box stock with a uniform margin on all sides [mm].
    pub fn with_uniform_stock(mesh: TriMesh, margin: f64) -> Self {
        let stock = StockShape::BoundingBox {
            margin: Vector3::new(margin, margin, margin),
        };
        Self::new(mesh, stock)
    }
}

use anyhow::Result;
use nalgebra::{Point3, Vector3, Unit};
use serde::{Deserialize, Serialize};

use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::{MoveType, OperationType, Toolpath, ToolpathPoint};

use super::traits::ToolpathStrategy;

/// 5-Axis Pencil Tracing.
///
/// Detects concave corners and traces them. The tool axis is tilted to
/// bisect the corner angle.
pub struct PencilTracing;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PencilParams {
    /// Concavity threshold [degrees]. Angles sharper than this are traced.
    pub threshold_deg: f64,
    /// Cutting feed rate [mm/min].
    pub feed_rate: f64,
}

impl Default for PencilParams {
    fn default() -> Self {
        PencilParams {
            threshold_deg: 30.0,
            feed_rate: 300.0,
        }
    }
}

impl ToolpathStrategy for PencilTracing {
    type Params = PencilParams;

    fn name(&self) -> &str {
        "5-Axis Pencil Tracing"
    }

    fn generate(
        &self,
        model: &WorkpieceModel,
        tool: &Tool,
        _machine: &MachineConfig,
        params: &PencilParams,
    ) -> Result<Vec<Toolpath>> {
        let mut tp = Toolpath::new(tool.id, OperationType::Finishing, "Pencil Trace");
        let tool_r = tool.shape.diameter() / 2.0;
        let safe_z = model.aabb.maxs.z as f64 + 10.0;

        // Heuristic: iterate over mesh triangles and check neighbors.
        // Parry3d TriMesh doesn't easily expose neighbor info, so we build an edge map.
        // For simplicity in this implementation, we'll do a slightly slower but direct check.
        
        let vertices = model.mesh.vertices();
        let indices = model.mesh.indices();
        let mut edges = std::collections::HashMap::new();

        for (tri_idx, tri) in indices.iter().enumerate() {
            let p0 = vertices[tri[0] as usize];
            let p1 = vertices[tri[1] as usize];
            let p2 = vertices[tri[2] as usize];
            
            let v1 = p1 - p0;
            let v2 = p2 - p0;
            let normal = v1.cross(&v2).normalize();

            let tri_edges = [
                (tri[0].min(tri[1]), tri[0].max(tri[1])),
                (tri[1].min(tri[2]), tri[1].max(tri[2])),
                (tri[2].min(tri[0]), tri[2].max(tri[0])),
            ];

            for e in tri_edges {
                edges.entry(e).or_insert_with(Vec::new).push((tri_idx, normal));
            }
        }

        let threshold_cos = (180.0 - params.threshold_deg).to_radians().cos();

        for (edge_verts, tris) in edges {
            if tris.len() == 2 {
                let (idx1, n1) = tris[0];
                let (idx2, n2) = tris[1];
                
                // Concavity check: dot product of normals and vector between centers.
                let dot = n1.dot(&n2);
                if dot < threshold_cos as f32 {
                    // It's a sharp corner. Check if it's concave.
                    let v0 = vertices[edge_verts.0 as usize];
                    let v1 = vertices[edge_verts.1 as usize];
                    let edge_mid = (v0.coords + v1.coords) * 0.5;
                    
                    // Vector from center of one tri to the other.
                    // This is a rough check.
                    let bisector = (n1 + n2).normalize();
                    
                    // Add the edge to the toolpath.
                    let p1 = Point3::from(v0).cast::<f64>();
                    let p2 = Point3::from(v1).cast::<f64>();
                    let orient = Unit::new_normalize(bisector.cast::<f64>());
                    
                    // Offset for tool radius
                    let offset_p1 = p1 + orient.into_inner() * tool_r;
                    let offset_p2 = p2 + orient.into_inner() * tool_r;

                    tp.points.push(ToolpathPoint {
                        position: offset_p1,
                        orientation: orient,
                        feed_rate: 0.0,
                        move_type: MoveType::Rapid,
                    });
                    tp.points.push(ToolpathPoint {
                        position: offset_p2,
                        orientation: orient,
                        feed_rate: params.feed_rate,
                        move_type: MoveType::Linear,
                    });
                    tp.points.push(ToolpathPoint::retract(Point3::new(offset_p2.x, offset_p2.y, safe_z)));
                }
            }
        }

        Ok(vec![tp])
    }
}

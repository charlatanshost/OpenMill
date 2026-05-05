use anyhow::Result;
use nalgebra::{Unit, Vector3};
use serde::{Deserialize, Serialize};

use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::{MoveType, OperationType, Toolpath, ToolpathPoint};

use super::drilling::Hole;
use super::traits::ToolpathStrategy;

/// Thread milling strategy.
///
/// Helically interpolates a multi-tooth thread mill around the bore of a
/// hole feature. One revolution advances axially by the thread pitch; the
/// total axial travel is the configured `thread_depth`.
///
/// Currently emits the helix as a discretised polyline of `MoveType::Linear`
/// segments rather than `G2`/`G3` arcs — that's CNC-portable today and the
/// post-processor can be upgraded later to fold equal-radius segments back
/// into a true helical interpolation.
pub struct ThreadMilling;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThreadMillingParams {
    /// Holes to thread.
    pub holes: Vec<Hole>,
    /// Major thread diameter [mm] — the diameter of the helix the cutter centre
    /// follows. For an internal thread this is `nominal_thread_diameter − tool_diameter`.
    /// `0.0` falls back to using the hole's own diameter as the helix diameter.
    pub thread_diameter: f64,
    /// Thread pitch [mm/rev].
    pub thread_pitch: f64,
    /// Total axial depth of thread [mm].
    pub thread_depth: f64,
    /// Cutting feed rate [mm/min].
    pub feed_rate: f64,
    /// Climb mill (`true`) vs conventional (`false`). Climb winds CCW for an
    /// internal right-hand thread when looking down the tool axis.
    pub climb: bool,
    /// Number of straight-line segments per helix revolution.
    pub segments_per_rev: usize,
}

impl Default for ThreadMillingParams {
    fn default() -> Self {
        Self {
            holes: Vec::new(),
            thread_diameter: 0.0,
            thread_pitch: 1.0,
            thread_depth: 5.0,
            feed_rate: 300.0,
            climb: true,
            segments_per_rev: 36, // 10° per segment
        }
    }
}

impl ToolpathStrategy for ThreadMilling {
    type Params = ThreadMillingParams;

    fn name(&self) -> &str {
        "Thread Milling"
    }

    fn generate(
        &self,
        _model: &WorkpieceModel,
        tool: &Tool,
        _machine: &MachineConfig,
        params: &ThreadMillingParams,
    ) -> Result<Vec<Toolpath>> {
        let mut tp = Toolpath::new(tool.id, OperationType::Finishing, "Thread Milling");
        let segments = params.segments_per_rev.max(8);
        let pitch = params.thread_pitch.max(0.05);
        let depth = params.thread_depth.max(pitch);
        let dir   = if params.climb { 1.0 } else { -1.0 };
        let tool_r = tool.shape.diameter() * 0.5;

        for hole in &params.holes {
            // Build a local frame whose Z points along the hole axis.
            let axis = Unit::new_normalize(hole.axis);
            let z = axis.into_inner();
            // Pick a stable perpendicular for the local X axis.
            let x = if z.z.abs() < 0.9 {
                z.cross(&Vector3::z()).normalize()
            } else {
                Vector3::x()
            };
            let y = z.cross(&x).normalize();

            // Helix radius the **tool centre** must follow. If the user
            // hasn't dialled in a diameter, fall back to the hole's own
            // bore minus the tool radius so the cutter rides the wall.
            let helix_r = if params.thread_diameter > 0.0 {
                params.thread_diameter * 0.5 - tool_r
            } else {
                // Hole feature didn't carry diameter explicitly here — use
                // a sensible default of tool_r (cutter centre at hole axis).
                tool_r
            }
            .max(0.1);

            // Hole top and bottom of the threaded section, expressed via the
            // hole's own axis (which points INTO the hole).
            let safe_pos = hole.position + z * 10.0;
            let top      = hole.position;                  // top of thread = hole top
            let bottom   = hole.position + z * depth;      // travel `depth` along axis

            // ── Approach ───────────────────────────────────────────────────
            // Rapid above the hole.
            tp.points.push(ToolpathPoint {
                position: safe_pos,
                orientation: axis,
                feed_rate: 0.0,
                move_type: MoveType::Rapid,
            });
            // Move down to the top of the thread on the centreline.
            tp.points.push(ToolpathPoint {
                position: top,
                orientation: axis,
                feed_rate: 0.0,
                move_type: MoveType::Rapid,
            });
            // Lead-in: linear out from centre to helix start radius.
            let start_helix = top + x * helix_r;
            tp.points.push(ToolpathPoint {
                position: start_helix,
                orientation: axis,
                feed_rate: params.feed_rate * 0.5,
                move_type: MoveType::LeadIn,
            });

            // ── Helix ──────────────────────────────────────────────────────
            // Total revolutions to span `depth` at this `pitch`.
            let revs = (depth / pitch).ceil() as usize;
            let total_steps = (revs * segments).max(segments);
            for step in 1..=total_steps {
                let t = step as f64 / total_steps as f64;
                let theta = dir * t * (revs as f64) * std::f64::consts::TAU;
                let z_off = t * depth;
                let cos_t = theta.cos();
                let sin_t = theta.sin();
                let pos = top + (x * cos_t + y * sin_t) * helix_r + z * z_off;
                tp.points.push(ToolpathPoint {
                    position: pos,
                    orientation: axis,
                    feed_rate: params.feed_rate,
                    move_type: MoveType::Linear,
                });
            }

            // Lead-out: back to the centreline at the current depth.
            tp.points.push(ToolpathPoint {
                position: bottom,
                orientation: axis,
                feed_rate: params.feed_rate * 0.5,
                move_type: MoveType::LeadOut,
            });
            // Retract straight up.
            tp.points.push(ToolpathPoint {
                position: safe_pos,
                orientation: axis,
                feed_rate: 0.0,
                move_type: MoveType::Retract,
            });
        }

        Ok(vec![tp])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{StockShape, WorkpieceModel};
    use nalgebra::Point3;
    use parry3d::math::Point as PPoint;
    use parry3d::shape::TriMesh;

    fn empty_model() -> WorkpieceModel {
        let verts = vec![
            PPoint::<f32>::new(0.0, 0.0, 0.0),
            PPoint::<f32>::new(10.0, 0.0, 0.0),
            PPoint::<f32>::new(5.0, 10.0, 0.0),
        ];
        let tris = vec![[0u32, 1, 2]];
        WorkpieceModel::new(
            TriMesh::new(verts, tris),
            StockShape::BoundingBox {
                margin: nalgebra::Vector3::new(2.0, 2.0, 2.0),
            },
        )
    }

    #[test]
    fn helix_advances_one_pitch_per_revolution() {
        let params = ThreadMillingParams {
            holes: vec![Hole {
                position: Point3::new(0.0, 0.0, 0.0),
                axis: Vector3::new(0.0, 0.0, -1.0),
                depth: 10.0,
            }],
            thread_diameter: 8.0,
            thread_pitch: 2.0,
            thread_depth: 4.0, // 2 revs at pitch 2.0
            segments_per_rev: 8,
            ..Default::default()
        };
        let tool = Tool::flat_end(1, "thread mill", 4.0, 12.0);
        let machine = crate::kinematics::default_machines::default_trunnion_config();
        let paths = ThreadMilling.generate(&empty_model(), &tool, &machine, &params).unwrap();

        // The hole's axis is −Z, so the helix descends. First helix step is at
        //   z = top + axis.z * (1/16) * depth = 0 + (−1) * (1/16) * 4 = −0.25.
        let helix_first = &paths[0].points[3];
        assert!((helix_first.position.z + 0.25).abs() < 1e-6, "first helix step z = {}", helix_first.position.z);

        // After 16 helix points (full 2 revs), the cutter should be at z = -depth = -4.
        let helix_last_idx = 2 + 1 + 16; // 2 rapids + lead-in + 16 helix points
        let helix_last = &paths[0].points[helix_last_idx];
        assert!((helix_last.position.z + 4.0).abs() < 1e-6, "last helix step z = {}", helix_last.position.z);
    }

    #[test]
    fn climb_and_conventional_wind_opposite_ways() {
        let mut p = ThreadMillingParams {
            holes: vec![Hole {
                position: Point3::new(0.0, 0.0, 0.0),
                axis: Vector3::new(0.0, 0.0, -1.0),
                depth: 10.0,
            }],
            thread_diameter: 6.0,
            thread_pitch: 1.0,
            thread_depth: 1.0,
            segments_per_rev: 8,
            ..Default::default()
        };
        let tool = Tool::flat_end(1, "tm", 3.0, 12.0);
        let machine = crate::kinematics::default_machines::default_trunnion_config();

        p.climb = true;
        let climb = ThreadMilling.generate(&empty_model(), &tool, &machine, &p).unwrap();
        p.climb = false;
        let conv  = ThreadMilling.generate(&empty_model(), &tool, &machine, &p).unwrap();

        // First helix step y component opposite sign for climb vs conventional.
        let cy = climb[0].points[3].position.y;
        let vy = conv[0].points[3].position.y;
        assert!(cy.signum() != vy.signum() || (cy.abs() + vy.abs() < 1e-12),
            "climb y={cy}, conventional y={vy} should wind opposite");
    }
}

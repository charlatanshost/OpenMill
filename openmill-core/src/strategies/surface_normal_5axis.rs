use anyhow::Result;
use nalgebra::{Point3, Rotation3, Unit, Vector3};
use parry3d::query::{Ray, RayCast};
use serde::{Deserialize, Serialize};

use crate::kinematics::MachineConfig;
use crate::model::WorkpieceModel;
use crate::tool::Tool;
use crate::toolpath::{MoveType, OperationType, Toolpath, ToolpathPoint};

use super::traits::ToolpathStrategy;

/// Simultaneous 5-axis surface-normal finishing.
///
/// The tool axis is tilted to match (or offset from) the surface normal at
/// each contact point.  Best used with a ball-end mill on sculptured surfaces.
pub struct SurfaceNormal5Axis;

/// Drive-surface scan pattern.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DrivePattern {
    /// Parallel raster passes along the surface U parameter.
    Parallel,
    /// Constant-scallop step-over.
    Scallop,
    /// Inward spiral from boundary.
    Spiral,
    /// 360-degree rotary scan around the X axis.
    Rotary,
}

/// Tuning parameters for [`SurfaceNormal5Axis`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SurfaceNormal5AxisParams {
    /// Lead angle (tilt in direction of motion) [degrees].
    pub lead_angle: f64,
    /// Side tilt angle (tilt perpendicular to motion) [degrees].
    pub tilt_angle: f64,
    /// Step-over as a fraction of tool diameter [0.0 .. 1.0].
    pub step_over: f64,
    /// Cutting feed rate [mm/min].
    pub feed_rate: f64,
    /// Surface chord tolerance [mm].
    pub tolerance: f64,
    /// Scan pattern.
    pub pattern: DrivePattern,
}

impl Default for SurfaceNormal5AxisParams {
    fn default() -> Self {
        SurfaceNormal5AxisParams {
            lead_angle: 5.0,
            tilt_angle: 0.0,
            step_over: 0.1,
            feed_rate: 400.0,
            tolerance: 0.005,
            pattern: DrivePattern::Parallel,
        }
    }
}

impl ToolpathStrategy for SurfaceNormal5Axis {
    type Params = SurfaceNormal5AxisParams;

    fn name(&self) -> &str {
        "Surface Normal 5-Axis"
    }

    fn generate(
        &self,
        model: &WorkpieceModel,
        tool: &Tool,
        _machine: &MachineConfig,
        params: &SurfaceNormal5AxisParams,
    ) -> Result<Vec<Toolpath>> {
        let mut tp = Toolpath::new(
            tool.id,
            OperationType::Finishing,
            "Surface Normal 5-Axis Finishing",
        );

        match params.pattern {
            DrivePattern::Rotary => self.generate_rotary(model, tool, params, &mut tp)?,
            DrivePattern::Scallop => self.generate_scallop(model, tool, params, &mut tp)?,
            DrivePattern::Spiral => self.generate_spiral(model, tool, params, &mut tp)?,
            DrivePattern::Parallel => self.generate_parallel(model, tool, params, &mut tp)?,
        }

        Ok(vec![tp])
    }
}

impl SurfaceNormal5Axis {
    fn generate_scallop(
        &self,
        model: &WorkpieceModel,
        tool: &Tool,
        params: &SurfaceNormal5AxisParams,
        tp: &mut Toolpath,
    ) -> Result<()> {
        let aabb = &model.aabb;
        let tool_r = tool.shape.diameter() / 2.0;
        let step_over_mm = tool.shape.diameter() * params.step_over;
        let safe_z = aabb.maxs.z as f64 + 10.0;
        
        let center_x = (aabb.mins.x + aabb.maxs.x) as f64 * 0.5;
        let center_y = (aabb.mins.y + aabb.maxs.y) as f64 * 0.5;
        let max_r = (aabb.maxs.x - aabb.mins.x).max(aabb.maxs.y - aabb.mins.y) as f64 * 0.7; // Slightly more than half
        
        let mut r = max_r;
        while r > 0.0 {
            let mut pass_points = Vec::new();
            let steps = (2.0 * std::f64::consts::PI * r / params.tolerance.max(0.1)).ceil() as usize;
            
            for i in 0..=steps {
                let angle = (i as f64 / steps as f64) * 2.0 * std::f64::consts::PI;
                let dir = Vector3::new(angle.cos(), angle.sin(), 0.0);
                let start = Point3::new(center_x, center_y, safe_z) + dir * r;
                let ray = Ray::new(start.cast::<f32>(), Vector3::new(0.0, 0.0, -1.0f32));
                
                if let Some(inter) = model.mesh.cast_local_ray_and_get_normal(&ray, 1000.0, true) {
                    let hit_point = ray.point_at(inter.time_of_impact);
                    let mut normal = Vector3::new(inter.normal.x as f64, inter.normal.y as f64, inter.normal.z as f64);
                    if normal.z < 0.0 { normal = -normal; }
                    
                    let orientation = self.apply_lead_tilt(normal, true, params);
                    let pos = self.calculate_tool_pos(hit_point, normal, tool, tool_r);
                    
                    pass_points.push(ToolpathPoint {
                        position: pos,
                        orientation,
                        feed_rate: params.feed_rate,
                        move_type: MoveType::Linear,
                    });
                }
            }
            self.add_pass_to_toolpath(tp, pass_points, safe_z);
            r -= step_over_mm;
        }
        Ok(())
    }

    fn generate_spiral(
        &self,
        model: &WorkpieceModel,
        tool: &Tool,
        params: &SurfaceNormal5AxisParams,
        tp: &mut Toolpath,
    ) -> Result<()> {
        let aabb = &model.aabb;
        let tool_r = tool.shape.diameter() / 2.0;
        let step_over_mm = tool.shape.diameter() * params.step_over;
        let safe_z = aabb.maxs.z as f64 + 10.0;
        
        let center_x = (aabb.mins.x + aabb.maxs.x) as f64 * 0.5;
        let center_y = (aabb.mins.y + aabb.maxs.y) as f64 * 0.5;
        let max_r = (aabb.maxs.x - aabb.mins.x).max(aabb.maxs.y - aabb.mins.y) as f64 * 0.7;
        
        let mut r = max_r;
        let mut angle = 0.0f64;
        let ang_step = 2.0f64.to_radians(); // 2 degree steps
        
        let mut pass_points = Vec::new();
        
        while r > 0.0 {
            let dir = Vector3::new(angle.cos(), angle.sin(), 0.0);
            let start = Point3::new(center_x, center_y, safe_z) + dir * r;
            let ray = Ray::new(start.cast::<f32>(), Vector3::new(0.0, 0.0, -1.0f32));
            
            if let Some(inter) = model.mesh.cast_local_ray_and_get_normal(&ray, 1000.0, true) {
                let hit_point = ray.point_at(inter.time_of_impact);
                let mut normal = Vector3::new(inter.normal.x as f64, inter.normal.y as f64, inter.normal.z as f64);
                if normal.z < 0.0 { normal = -normal; }
                
                let orientation = self.apply_lead_tilt(normal, true, params);
                let pos = self.calculate_tool_pos(hit_point, normal, tool, tool_r);
                
                pass_points.push(ToolpathPoint {
                    position: pos,
                    orientation,
                    feed_rate: params.feed_rate,
                    move_type: MoveType::Linear,
                });
            }
            
            angle += ang_step;
            r -= step_over_mm * (ang_step / (2.0 * std::f64::consts::PI));
        }
        
        self.add_pass_to_toolpath(tp, pass_points, safe_z);
        Ok(())
    }

    fn generate_parallel(
        &self,
        model: &WorkpieceModel,
        tool: &Tool,
        params: &SurfaceNormal5AxisParams,
        tp: &mut Toolpath,
    ) -> Result<()> {
        let aabb = &model.aabb;
        let mins = aabb.mins;
        let maxs = aabb.maxs;
        let tool_r = tool.shape.diameter() / 2.0;
        let step_over_mm = tool.shape.diameter() * params.step_over;

        let x_min = mins.x as f64;
        let x_max = maxs.x as f64;
        let y_min = mins.y as f64;
        let y_max = maxs.y as f64;
        let safe_z = maxs.z as f64 + 10.0;

        let mut y = y_min;
        let mut forward = true;
        while y <= y_max {
            let (xa, xb) = if forward { (x_min, x_max) } else { (x_max, x_min) };
            let dist = (xb - xa).abs();
            let steps = (dist / params.tolerance.max(0.1)).ceil() as usize;
            let mut pass_points = Vec::new();

            for i in 0..=steps {
                let t = i as f64 / steps as f64;
                let curr_x = xa + (xb - xa) * t;
                let ray = Ray::new(
                    Point3::new(curr_x as f32, y as f32, safe_z as f32),
                    Vector3::new(0.0, 0.0, -1.0f32),
                );

                if let Some(inter) = model.mesh.cast_local_ray_and_get_normal(&ray, 1000.0, true) {
                    let hit_point = ray.point_at(inter.time_of_impact);
                    let mut normal = Vector3::new(inter.normal.x as f64, inter.normal.y as f64, inter.normal.z as f64);
                    if normal.z < 0.0 { normal = -normal; }

                    let tool_orientation = self.apply_lead_tilt(normal, forward, params);
                    let pos = self.calculate_tool_pos(hit_point, normal, tool, tool_r);

                    pass_points.push(ToolpathPoint {
                        position: pos,
                        orientation: tool_orientation,
                        feed_rate: params.feed_rate,
                        move_type: MoveType::Linear,
                    });
                }
            }
            self.add_pass_to_toolpath(tp, pass_points, safe_z);
            y += step_over_mm;
            forward = !forward;
        }
        Ok(())
    }

    fn generate_rotary(
        &self,
        model: &WorkpieceModel,
        tool: &Tool,
        params: &SurfaceNormal5AxisParams,
        tp: &mut Toolpath,
    ) -> Result<()> {
        let aabb = &model.aabb;
        let mins = aabb.mins;
        let maxs = aabb.maxs;
        let tool_r = tool.shape.diameter() / 2.0;
        let step_over_mm = tool.shape.diameter() * params.step_over;

        // Rotary scan around X axis
        let x_min = mins.x as f64;
        let x_max = maxs.x as f64;
        let r_max = (mins.y.abs().max(maxs.y.abs()).max(mins.z.abs()).max(maxs.z.abs())) as f64 + 10.0;
        let safe_r = r_max + 10.0;

        let mut x = x_min;
        let mut forward = true;
        while x <= x_max {
            let mut pass_points = Vec::new();
            let ang_steps = 360; // 1 degree steps

            for i in 0..=ang_steps {
                let angle_deg = if forward { i as f64 } else { (ang_steps - i) as f64 };
                let angle_rad = angle_deg.to_radians();
                
                // Direction from center to tool
                let dir = Vector3::new(0.0, angle_rad.cos(), angle_rad.sin());
                let start_pt = Point3::new(x, 0.0, 0.0) + dir * safe_r;
                
                let ray = Ray::new(
                    start_pt.cast::<f32>(),
                    (-dir).cast::<f32>(),
                );

                if let Some(inter) = model.mesh.cast_local_ray_and_get_normal(&ray, safe_r as f32 * 2.0, true) {
                    let hit_point = ray.point_at(inter.time_of_impact);
                    let mut normal = Vector3::new(inter.normal.x as f64, inter.normal.y as f64, inter.normal.z as f64);
                    if normal.dot(&dir) < 0.0 { normal = -normal; }
                    
                    let tool_orientation = self.apply_lead_tilt(normal, forward, params);
                    let pos = self.calculate_tool_pos(hit_point, normal, tool, tool_r);

                    pass_points.push(ToolpathPoint {
                        position: pos,
                        orientation: tool_orientation,
                        feed_rate: params.feed_rate,
                        move_type: MoveType::Linear,
                    });
                }
            }
            
            if !pass_points.is_empty() {
                let first_pos = pass_points[0].position;
                tp.points.push(ToolpathPoint {
                    position: first_pos + (pass_points[0].orientation.into_inner() * 10.0),
                    orientation: pass_points[0].orientation,
                    feed_rate: 0.0,
                    move_type: MoveType::Rapid,
                });
                for (idx, mut pt) in pass_points.into_iter().enumerate() {
                    if idx == 0 { pt.move_type = MoveType::LeadIn; }
                    tp.points.push(pt);
                }
            }
            
            x += step_over_mm;
            forward = !forward;
        }
        Ok(())
    }

    fn apply_lead_tilt(&self, normal: Vector3<f64>, forward: bool, params: &SurfaceNormal5AxisParams) -> Unit<Vector3<f64>> {
        let mut tool_orientation = Unit::new_normalize(normal);
        if params.lead_angle != 0.0 || params.tilt_angle != 0.0 {
            let direction = if forward { 1.0 } else { -1.0 };
            // Simple forward vector (can be improved by tracking actual path direction)
            let forward_vec = Vector3::new(direction, 0.0, 0.0);
            let side_vec = if forward_vec.cross(&normal).norm() > 0.001 {
                forward_vec.cross(&normal).normalize()
            } else {
                Vector3::new(0.0, 1.0, 0.0).cross(&normal).normalize()
            };
            
            let lead_rot = Rotation3::from_axis_angle(&Unit::new_normalize(side_vec), -params.lead_angle.to_radians() * direction);
            let tilt_rot = Rotation3::from_axis_angle(&Unit::new_normalize(forward_vec), params.tilt_angle.to_radians());
            tool_orientation = Unit::new_normalize(tilt_rot * lead_rot * normal);
        }
        tool_orientation
    }

    fn calculate_tool_pos(&self, hit_point: Point3<f32>, normal: Vector3<f64>, tool: &Tool, tool_r: f64) -> Point3<f64> {
        if tool.shape.is_ball() {
            Point3::new(
                hit_point.x as f64 + normal.x * tool_r,
                hit_point.y as f64 + normal.y * tool_r,
                hit_point.z as f64 + normal.z * tool_r,
            )
        } else {
            Point3::new(hit_point.x as f64, hit_point.y as f64, hit_point.z as f64)
        }
    }

    fn add_pass_to_toolpath(&self, tp: &mut Toolpath, pass_points: Vec<ToolpathPoint>, safe_z: f64) {
        if !pass_points.is_empty() {
            let first = pass_points[0].position;
            tp.points.push(ToolpathPoint {
                position: Point3::new(first.x, first.y, safe_z),
                orientation: pass_points[0].orientation,
                feed_rate: 0.0,
                move_type: MoveType::Rapid,
            });
            for (idx, mut pt) in pass_points.into_iter().enumerate() {
                if idx == 0 { pt.move_type = MoveType::LeadIn; }
                tp.points.push(pt);
            }
            let last = tp.points.last().unwrap().position;
            tp.points.push(ToolpathPoint {
                position: Point3::new(last.x, last.y, safe_z),
                orientation: tp.points.last().unwrap().orientation,
                feed_rate: 0.0,
                move_type: MoveType::Retract,
            });
        }
    }
}

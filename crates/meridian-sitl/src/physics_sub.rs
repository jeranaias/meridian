//! Submarine (AUV/ROV) physics model.
//!
//! 6-DOF underwater vehicle with buoyancy, water drag, and thruster model.
//! Based on ArduSub's simplified dynamics.

use meridian_math::{Vec3, Quaternion};
use meridian_math::frames::{NED, Body};
use crate::physics::{PhysicsState, GRAVITY};

/// Submarine physics parameters.
#[derive(Debug, Clone)]
pub struct SubParams {
    pub mass: f32,               // kg (in air)
    pub buoyancy_force: f32,     // N (upward force from displacement - slightly > weight for positive buoyancy)
    pub inertia: [f32; 3],       // Ixx, Iyy, Izz
    pub drag_linear: [f32; 3],   // linear drag coefficients (N·s/m) per body axis
    pub drag_angular: [f32; 3],  // angular drag (N·m·s/rad) per body axis
    pub max_thrust: f32,         // max thrust per thruster (N)
    pub depth_max: f32,          // max operating depth (m, positive down)
}

impl SubParams {
    /// Default BlueROV2-like submarine.
    pub fn default_sub() -> Self {
        Self {
            mass: 11.0,           // ~11 kg
            buoyancy_force: 11.0 * GRAVITY + 1.0, // slightly positively buoyant (+1N)
            inertia: [0.2, 0.3, 0.3],
            drag_linear: [20.0, 25.0, 30.0],  // much higher than air (water!)
            drag_angular: [2.0, 2.0, 1.5],
            max_thrust: 30.0,     // N per thruster
            depth_max: 100.0,     // 100m
        }
    }
}

/// Submarine control inputs (6-DOF thruster allocation).
#[derive(Debug, Clone, Copy, Default)]
pub struct SubControls {
    pub forward: f32,    // -1..1 forward/backward
    pub lateral: f32,    // -1..1 left/right
    pub vertical: f32,   // -1..1 up/down (positive = up)
    pub yaw: f32,        // -1..1 yaw rate command
    pub pitch: f32,      // -1..1 pitch command
    pub roll: f32,       // -1..1 roll command
}

/// Run one submarine physics step.
pub fn step_sub(
    state: &mut PhysicsState,
    controls: &SubControls,
    params: &SubParams,
    dt: f32,
) {
    let body_to_ned = meridian_math::Rotation::<Body, NED>::from_quaternion(state.attitude);
    let ned_to_body = body_to_ned.inverse();

    // ── Thruster forces in body frame ──
    let thrust_body = Vec3::<Body>::new(
        controls.forward * params.max_thrust,
        controls.lateral * params.max_thrust,
        -controls.vertical * params.max_thrust, // up command → negative body Z (upward)
    );

    // ── Water drag (linear, proportional to velocity — simplified) ──
    let vel_body = ned_to_body.rotate(state.velocity);
    let drag_body = Vec3::<Body>::new(
        -params.drag_linear[0] * vel_body.x,
        -params.drag_linear[1] * vel_body.y,
        -params.drag_linear[2] * vel_body.z,
    );

    // ── Angular drag ──
    let angular_drag = Vec3::<Body>::new(
        -params.drag_angular[0] * state.gyro.x,
        -params.drag_angular[1] * state.gyro.y,
        -params.drag_angular[2] * state.gyro.z,
    );

    // ── Thruster torques (yaw/pitch/roll) ──
    let torque = Vec3::<Body>::new(
        controls.roll * params.max_thrust * 0.15,   // roll moment arm ~0.15m
        controls.pitch * params.max_thrust * 0.15,
        controls.yaw * params.max_thrust * 0.2,     // yaw moment arm ~0.2m
    );

    // ── Net force in body frame ──
    let total_force_body = thrust_body + drag_body;
    let force_ned = body_to_ned.rotate(total_force_body);

    // ── Gravity and buoyancy in NED ──
    let weight = Vec3::<NED>::new(0.0, 0.0, params.mass * GRAVITY);       // down
    let buoyancy = Vec3::<NED>::new(0.0, 0.0, -params.buoyancy_force);    // up
    let total_force_ned = force_ned + weight + buoyancy;

    // ── Integration ──
    let accel_ned = total_force_ned * (1.0 / params.mass);
    let angular_accel = Vec3::<Body>::new(
        (torque.x + angular_drag.x) / params.inertia[0],
        (torque.y + angular_drag.y) / params.inertia[1],
        (torque.z + angular_drag.z) / params.inertia[2],
    );

    // Angular
    state.gyro = state.gyro + angular_accel * dt;
    let angle = state.gyro.length() * dt;
    if angle > 1e-10 {
        let axis = state.gyro.normalized();
        let dq = Quaternion::from_axis_angle(&[axis.x, axis.y, axis.z], angle);
        state.attitude = state.attitude * dq;
        state.attitude.normalize();
    }

    // Linear
    state.velocity = state.velocity + accel_ned * dt;
    state.position[0] += (state.velocity.x * dt) as f64;
    state.position[1] += (state.velocity.y * dt) as f64;
    state.position[2] += (state.velocity.z * dt) as f64;

    // Surface contact (can't go above water)
    if state.position[2] < 0.0 {
        state.position[2] = 0.0;
        if state.velocity.z < 0.0 {
            state.velocity = Vec3::new(state.velocity.x, state.velocity.y, 0.0);
        }
    }

    // Depth limit
    if state.position[2] > params.depth_max as f64 {
        state.position[2] = params.depth_max as f64;
        if state.velocity.z > 0.0 {
            state.velocity = Vec3::new(state.velocity.x, state.velocity.y, 0.0);
        }
    }

    state.on_ground = false;
    state.time_us += (dt * 1e6) as u64;
}

/// Simple depth-hold controller for submarine.
/// Returns vertical thrust command (-1..1).
pub fn depth_hold(target_depth: f32, current_depth: f32, current_vz: f32) -> f32 {
    let depth_err = target_depth - current_depth;
    let desired_vz = (depth_err * 0.5).clamp(-1.0, 1.0); // max 1 m/s
    let vz_err = desired_vz - current_vz;
    (vz_err * 0.3).clamp(-0.5, 0.5) // gentle thrust
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sub_sinks_without_thrust() {
        // Slightly negatively buoyant without vertical thrust
        let mut params = SubParams::default_sub();
        params.buoyancy_force = params.mass * GRAVITY - 2.0; // 2N negative buoyancy
        let mut state = PhysicsState::new();
        state.position[2] = -5.0; // 5m depth (NED: positive = deeper)
        // Wait, in NED positive Z is down. For underwater, deeper = more positive Z.
        // But PhysicsState starts at [0,0,0] which is the surface.
        // Let's start at surface and see if it sinks.
        state.position[2] = 0.0;

        let controls = SubControls::default();
        let dt = 1.0 / 50.0;

        for _ in 0..500 { // 10 seconds
            step_sub(&mut state, &controls, &params, dt);
        }

        // Should have sunk (positive Z = deeper)
        assert!(state.position[2] > 0.5, "Should sink: depth={:.2}", state.position[2]);
    }

    #[test]
    fn test_sub_depth_hold() {
        let params = SubParams::default_sub();
        let mut state = PhysicsState::new();
        state.position[2] = 0.0; // at surface

        let target_depth = 5.0; // 5m deep (NED positive = down)
        let dt = 1.0 / 50.0;

        for _ in 0..1000 { // 20 seconds
            let current_depth = state.position[2] as f32;
            let vert_cmd = depth_hold(target_depth, current_depth, state.velocity.z);
            let controls = SubControls {
                vertical: -vert_cmd, // positive depth_hold = descend = negative vertical cmd
                ..Default::default()
            };
            step_sub(&mut state, &controls, &params, dt);
        }

        let final_depth = state.position[2] as f32;
        assert!((final_depth - target_depth).abs() < 2.0,
            "Should hold depth 5m: got {:.1}m", final_depth);
    }

    #[test]
    fn test_sub_forward_drive() {
        let params = SubParams::default_sub();
        let mut state = PhysicsState::new();
        state.position[2] = 5.0; // 5m deep

        let controls = SubControls { forward: 0.5, ..Default::default() };
        let dt = 1.0 / 50.0;

        for _ in 0..500 {
            step_sub(&mut state, &controls, &params, dt);
        }

        assert!(state.position[0] > 2.0, "Should move forward: N={:.1}", state.position[0]);
        assert!(!state.velocity.is_nan());
    }

    #[test]
    fn test_sub_yaw_turn() {
        let params = SubParams::default_sub();
        let mut state = PhysicsState::new();
        state.position[2] = 5.0;

        let controls = SubControls { yaw: 0.5, forward: 0.3, ..Default::default() };
        let dt = 1.0 / 50.0;

        for _ in 0..500 {
            step_sub(&mut state, &controls, &params, dt);
        }

        let (_, _, yaw) = state.attitude.to_euler();
        assert!(yaw.abs() > 0.5, "Should have turned: yaw={:.2}", yaw);
    }

    #[test]
    fn test_sub_waypoint_mission() {
        // Drive to waypoint at 5m depth
        let params = SubParams::default_sub();
        let mut state = PhysicsState::new();
        state.position[2] = 5.0;

        let target = Vec3::<NED>::new(10.0, 5.0, 5.0);
        let dt = 1.0 / 50.0;

        for _ in 0..1500 { // 30 seconds
            let pos = Vec3::<NED>::new(
                state.position[0] as f32, state.position[1] as f32, state.position[2] as f32);
            let to_target = target - pos;
            let dist = libm::sqrtf(to_target.x*to_target.x + to_target.y*to_target.y);

            // Simple heading control
            let target_yaw = libm::atan2f(to_target.y, to_target.x);
            let (_, _, current_yaw) = state.attitude.to_euler();
            let mut yaw_err = target_yaw - current_yaw;
            while yaw_err > core::f32::consts::PI { yaw_err -= 2.0*core::f32::consts::PI; }
            while yaw_err < -core::f32::consts::PI { yaw_err += 2.0*core::f32::consts::PI; }

            let depth_cmd = depth_hold(5.0, state.position[2] as f32, state.velocity.z);

            let controls = SubControls {
                forward: if dist > 1.0 { 0.5 } else { 0.0 },
                yaw: (yaw_err * 0.5).clamp(-0.5, 0.5),
                vertical: -depth_cmd,
                ..Default::default()
            };
            step_sub(&mut state, &controls, &params, dt);
        }

        let final_dist = {
            let dx = state.position[0] as f32 - target.x;
            let dy = state.position[1] as f32 - target.y;
            libm::sqrtf(dx*dx + dy*dy)
        };
        eprintln!("  Sub WP mission: target_err={:.1}m depth={:.1}m", final_dist, state.position[2]);
        assert!(final_dist < 5.0, "Should reach waypoint: err={:.1}m", final_dist);
        assert!((state.position[2] as f32 - 5.0).abs() < 2.0, "Should hold depth");
    }
}

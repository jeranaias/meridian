//! Ground vehicle (rover/boat) physics model.
//!
//! Models 2D ground vehicle motion with steering and throttle.
//! Works for both wheeled rovers and differential-thrust boats.

use meridian_math::{Vec3, Quaternion};
use meridian_math::frames::{NED, Body};
use crate::physics::PhysicsState;

/// Rover/boat physics parameters.
#[derive(Debug, Clone)]
pub struct RoverParams {
    pub mass: f32,               // kg
    pub max_speed: f32,          // max forward speed (m/s)
    pub max_force: f32,          // max drive force (N)
    pub drag_coefficient: f32,   // velocity-proportional drag (N·s/m)
    pub max_steer_angle: f32,    // max steering angle (rad) — for Ackermann
    pub wheelbase: f32,          // distance between axles (m)
    pub is_boat: bool,           // if true, higher drag (water), no ground friction
    pub turn_rate_max: f32,      // max yaw rate for skid-steer (rad/s)
}

impl RoverParams {
    pub fn default_rover() -> Self {
        Self {
            mass: 5.0,
            max_speed: 5.0,
            max_force: 20.0,
            drag_coefficient: 4.0,   // moderate ground friction
            max_steer_angle: 30.0 * core::f32::consts::PI / 180.0,
            wheelbase: 0.3,
            is_boat: false,
            turn_rate_max: core::f32::consts::PI / 2.0, // 90°/s
        }
    }

    pub fn default_boat() -> Self {
        Self {
            mass: 10.0,
            max_speed: 3.0,
            max_force: 15.0,
            drag_coefficient: 8.0,   // water drag
            max_steer_angle: 0.0,    // differential thrust, no steering angle
            wheelbase: 0.5,
            is_boat: true,
            turn_rate_max: core::f32::consts::PI / 4.0, // 45°/s
        }
    }
}

/// Rover/boat control inputs.
#[derive(Debug, Clone, Copy, Default)]
pub struct RoverControls {
    pub steering: f32,   // -1..1 (left..right)
    pub throttle: f32,   // -1..1 (reverse..forward)
}

/// Run one rover/boat physics step.
///
/// The vehicle moves on the ground (Z=0) in 2D.
/// Heading is controlled by steering (yaw rate).
/// Speed is controlled by throttle (forward force).
pub fn step_rover(
    state: &mut PhysicsState,
    controls: &RoverControls,
    params: &RoverParams,
    dt: f32,
) {
    // Current heading from quaternion
    let (_, _, yaw) = state.attitude.to_euler();

    // ── Steering → yaw rate ──
    let yaw_rate = controls.steering * params.turn_rate_max;

    // ── Throttle → forward force ──
    let drive_force = controls.throttle * params.max_force;

    // ── Current speed (forward component) ──
    let cos_yaw = libm::cosf(yaw);
    let sin_yaw = libm::sinf(yaw);
    let speed_fwd = state.velocity.x * cos_yaw + state.velocity.y * sin_yaw;

    // ── Forces in NED ──
    // Drive force along heading
    let force_n = drive_force * cos_yaw;
    let force_e = drive_force * sin_yaw;

    // Drag (velocity-proportional, opposes motion)
    let drag_n = -params.drag_coefficient * state.velocity.x;
    let drag_e = -params.drag_coefficient * state.velocity.y;

    let accel_n = (force_n + drag_n) / params.mass;
    let accel_e = (force_e + drag_e) / params.mass;

    // ── Update velocity ──
    state.velocity = Vec3::<NED>::new(
        state.velocity.x + accel_n * dt,
        state.velocity.y + accel_e * dt,
        0.0, // stays on ground
    );

    // Clamp speed
    let speed = libm::sqrtf(state.velocity.x * state.velocity.x + state.velocity.y * state.velocity.y);
    if speed > params.max_speed {
        let scale = params.max_speed / speed;
        state.velocity = Vec3::new(state.velocity.x * scale, state.velocity.y * scale, 0.0);
    }

    // ── Update position ──
    state.position[0] += (state.velocity.x * dt) as f64;
    state.position[1] += (state.velocity.y * dt) as f64;
    state.position[2] = 0.0; // always on ground/water surface

    // ── Update heading (yaw) ──
    let new_yaw = yaw + yaw_rate * dt;
    state.attitude = Quaternion::from_euler(0.0, 0.0, new_yaw);

    // ── Gyro ──
    state.gyro = Vec3::<Body>::new(0.0, 0.0, yaw_rate);

    state.on_ground = true;
    state.time_us += (dt * 1e6) as u64;
}

/// Generate IMU reading for rover (on ground: accel = -gravity + drive force component).
pub fn rover_imu_accel(
    state: &PhysicsState,
    controls: &RoverControls,
    params: &RoverParams,
) -> Vec3<Body> {
    use crate::physics::GRAVITY;
    use meridian_math::Rotation;

    let ned_to_body: Rotation<NED, Body> = Rotation::from_quaternion(state.attitude).inverse();
    let gravity_ned = Vec3::<NED>::new(0.0, 0.0, GRAVITY);
    let gravity_body = ned_to_body.rotate(gravity_ned);

    // On ground: specific force = ground_reaction/mass = -gravity + drive_accel
    let drive_accel = controls.throttle * params.max_force / params.mass;
    Vec3::<Body>::new(drive_accel, 0.0, 0.0) - gravity_body
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rover_straight_drive() {
        let params = RoverParams::default_rover();
        let mut state = PhysicsState::new();
        state.attitude = Quaternion::from_euler(0.0, 0.0, 0.0); // facing north

        let controls = RoverControls { steering: 0.0, throttle: 0.5 };
        let dt = 1.0 / 50.0; // 50Hz

        // Drive for 5 seconds
        for _ in 0..250 {
            step_rover(&mut state, &controls, &params, dt);
        }

        // Should have moved north
        assert!(state.position[0] > 5.0, "Should move north: N={:.1}", state.position[0]);
        assert!((state.position[1] as f32).abs() < 0.1, "Should stay straight: E={:.1}", state.position[1]);
        // Should have reached some speed
        assert!(state.velocity.x > 1.0, "Should have speed: vx={:.1}", state.velocity.x);
    }

    #[test]
    fn test_rover_turn() {
        let params = RoverParams::default_rover();
        let mut state = PhysicsState::new();
        state.attitude = Quaternion::from_euler(0.0, 0.0, 0.0);

        let controls = RoverControls { steering: 0.5, throttle: 0.5 };
        let dt = 1.0 / 50.0;

        for _ in 0..250 {
            step_rover(&mut state, &controls, &params, dt);
        }

        // Should have turned (heading changed)
        let (_, _, yaw) = state.attitude.to_euler();
        assert!(yaw.abs() > 0.5, "Should have turned: yaw={:.2}", yaw);
        // Should have moved east (turning right)
        assert!(state.position[1] > 0.5, "Should drift east: E={:.1}", state.position[1]);
    }

    #[test]
    fn test_rover_stop() {
        let params = RoverParams::default_rover();
        let mut state = PhysicsState::new();
        state.velocity = Vec3::<NED>::new(3.0, 0.0, 0.0); // moving north

        let controls = RoverControls { steering: 0.0, throttle: 0.0 }; // no input
        let dt = 1.0 / 50.0;

        for _ in 0..500 { // 10 seconds
            step_rover(&mut state, &controls, &params, dt);
        }

        // Should have stopped (drag)
        assert!(state.velocity.x < 0.1, "Should stop: vx={:.2}", state.velocity.x);
    }

    #[test]
    fn test_boat_physics() {
        let params = RoverParams::default_boat();
        let mut state = PhysicsState::new();

        let controls = RoverControls { steering: 0.0, throttle: 0.8 };
        let dt = 1.0 / 50.0;

        for _ in 0..500 {
            step_rover(&mut state, &controls, &params, dt);
        }

        assert!(!state.velocity.is_nan());
        // Boat should move but slower than rover (more drag)
        assert!(state.velocity.length() < params.max_speed + 0.1,
            "Speed limited: v={:.1}", state.velocity.length());
    }
}

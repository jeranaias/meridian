//! Rigid body physics simulation.
//!
//! 6-DOF dynamics with motor forces, drag, rotational damping, and ground contact.
//! Integration at 1200 Hz matching ArduPilot SITL.
//!
//! Source: libraries/SITL/SIM_Aircraft.cpp, SIM_Frame.cpp

use meridian_math::{Vec3, Quaternion, Rotation};
use meridian_math::frames::{NED, Body};

/// Gravity in NED frame (m/s², positive down).
pub const GRAVITY: f32 = 9.80665;

/// Default physics rate (Hz). Source: SIM_Aircraft.h:283
pub const PHYSICS_HZ: u32 = 1200;

/// Complete state of the simulated vehicle.
#[derive(Debug, Clone)]
pub struct PhysicsState {
    /// Position in NED frame relative to home (meters, f64 for precision).
    pub position: [f64; 3],
    /// Velocity in NED frame (m/s).
    pub velocity: Vec3<NED>,
    /// Attitude quaternion (body-to-NED rotation).
    pub attitude: Quaternion,
    /// Angular velocity in body frame (rad/s).
    pub gyro: Vec3<Body>,
    /// Whether the vehicle is on the ground.
    pub on_ground: bool,
    /// Simulation time in microseconds.
    pub time_us: u64,
}

impl PhysicsState {
    /// Create initial state: on the ground, level, at home position.
    pub fn new() -> Self {
        Self {
            position: [0.0, 0.0, 0.0],
            velocity: Vec3::zero(),
            attitude: Quaternion::identity(),
            gyro: Vec3::zero(),
            on_ground: true,
            time_us: 0,
        }
    }

    /// Altitude above ground (positive up).
    pub fn altitude(&self) -> f32 {
        -self.position[2] as f32
    }
}

/// Per-motor state for physics calculation.
#[derive(Debug, Clone)]
pub struct MotorState {
    /// Position relative to CG in body frame (meters).
    pub position: Vec3<Body>,
    /// Yaw factor: +1 for CCW, -1 for CW (reaction torque sign).
    pub yaw_factor: f32,
    /// Maximum thrust (Newtons) for this motor.
    pub max_thrust: f32,
    /// Thrust curve expo (0=linear, 1=quadratic). Source: SIM_Frame.h:111
    pub expo: f32,
    /// Yaw torque coupling: 0.05 * diagonal. Source: SIM_Motor.cpp:70-72
    pub yaw_coupling: f32,
}

/// Vehicle physics parameters (frame-specific, loaded from config).
#[derive(Debug, Clone)]
pub struct VehicleParams {
    pub mass: f32,
    pub inertia: [f32; 3],  // Ixx, Iyy, Izz
    pub motors: Vec<MotorState>,
    pub drag_area_cd: f32,
    /// Rotational damping: 400/terminal_rate. Source: SIM_Frame.cpp:699-704
    pub rotational_damping: f32,
}

impl VehicleParams {
    /// Create from a VehiclePhysics config.
    pub fn from_vehicle(vp: &meridian_vehicle::VehiclePhysics) -> Self {
        let positions = vp.motor_positions();
        let max_thrust_per_motor = vp.max_thrust_per_motor();

        let motors: Vec<MotorState> = positions.iter().map(|(pos, yaw)| {
            MotorState {
                position: *pos,
                yaw_factor: *yaw,
                max_thrust: max_thrust_per_motor,
                expo: vp.motor_expo,
                yaw_coupling: 0.05 * vp.diagonal_m,
            }
        }).collect();

        let terminal_rad = vp.terminal_rotation_rate * core::f32::consts::PI / 180.0;
        let rot_damping = if terminal_rad > 0.0 {
            (400.0 * core::f32::consts::PI / 180.0) / terminal_rad
        } else {
            0.0
        };

        Self {
            mass: vp.mass_kg,
            inertia: vp.inertia,
            motors,
            drag_area_cd: vp.drag_coefficient,
            rotational_damping: rot_damping,
        }
    }
}

/// Advance the physics state by one timestep.
///
/// `motor_commands`: 0..1 throttle command per motor.
/// `dt`: timestep in seconds (typically 1/1200).
pub fn step(
    state: &mut PhysicsState,
    motor_commands: &[f32],
    params: &VehicleParams,
    dt: f32,
) {
    let n = params.motors.len();

    // ─── Compute motor forces and torques in body frame ───

    let mut total_force_body = Vec3::<Body>::zero();
    let mut total_torque_body = Vec3::<Body>::zero();

    for i in 0..n {
        let motor = &params.motors[i];
        let cmd = motor_commands.get(i).copied().unwrap_or(0.0).clamp(0.0, 1.0);

        // Thrust from expo curve: T = Tmax * ((1-expo)*cmd + expo*cmd²)
        let linearized = (1.0 - motor.expo) * cmd + motor.expo * cmd * cmd;
        let thrust = motor.max_thrust * linearized;

        // Thrust vector in body frame (upward = negative Z)
        let thrust_vec = Vec3::<Body>::new(0.0, 0.0, -thrust);
        total_force_body = total_force_body + thrust_vec;

        // Torque from thrust moment arm: τ = position × force
        let torque = motor.position.cross(&thrust_vec);
        total_torque_body = total_torque_body + torque;

        // Yaw reaction torque from motor spin
        // Source: SIM_Motor.cpp:70-72
        // DA1 fix: torque is proportional to thrust alone, not cmd*thrust.
        // cmd is already baked into thrust via the expo curve above.
        let yaw_torque = motor.yaw_factor * motor.yaw_coupling * thrust;
        total_torque_body = total_torque_body + Vec3::<Body>::new(0.0, 0.0, yaw_torque);
    }

    // ─── Drag in body frame ───
    // Source: SIM_Frame.cpp:706-725
    // F_drag = -areaCd * 0.5 * rho * v² * sign(v)
    let rho = 1.225f32; // air density at sea level (kg/m³)
    let body_to_ned: Rotation<Body, NED> = Rotation::from_quaternion(state.attitude);
    let ned_to_body: Rotation<NED, Body> = body_to_ned.inverse();
    let vel_body = ned_to_body.rotate(state.velocity);

    let drag_factor = params.drag_area_cd * 0.5 * rho;
    let drag_body = Vec3::<Body>::new(
        -drag_factor * vel_body.x * libm::fabsf(vel_body.x),
        -drag_factor * vel_body.y * libm::fabsf(vel_body.y),
        -drag_factor * vel_body.z * libm::fabsf(vel_body.z),
    );
    total_force_body = total_force_body + drag_body;

    // ─── Rotational damping ───
    // Source: SIM_Frame.cpp:699-704
    let damping = Vec3::<Body>::new(
        -state.gyro.x * params.rotational_damping,
        -state.gyro.y * params.rotational_damping,
        -state.gyro.z * params.rotational_damping,
    );
    total_torque_body = total_torque_body + damping;

    // ─── Convert force to NED and add gravity ───
    let force_ned = body_to_ned.rotate(total_force_body);
    let gravity_force = Vec3::<NED>::new(0.0, 0.0, params.mass * GRAVITY);
    let total_force_ned = force_ned + gravity_force;

    // ─── Linear dynamics ───
    let accel_ned = total_force_ned * (1.0 / params.mass);

    // ─── Angular dynamics ───
    // τ = I * α  →  α = τ / I  (diagonal inertia)
    let angular_accel = Vec3::<Body>::new(
        if params.inertia[0] > 0.0 { total_torque_body.x / params.inertia[0] } else { 0.0 },
        if params.inertia[1] > 0.0 { total_torque_body.y / params.inertia[1] } else { 0.0 },
        if params.inertia[2] > 0.0 { total_torque_body.z / params.inertia[2] } else { 0.0 },
    );

    // ─── Ground contact ───
    // Source: SIM_Aircraft.cpp:745-868
    if state.on_ground {
        // Can't accelerate downward through the ground
        if state.position[2] >= 0.0 {
            // On or below ground level
            state.position[2] = 0.0;
            if state.velocity.z > 0.0 {
                state.velocity = Vec3::new(state.velocity.x, state.velocity.y, 0.0);
            }
        }
    }

    // ─── Integration (semi-implicit Euler) ───

    // Angular: update gyro first, then attitude
    state.gyro = state.gyro + angular_accel * dt;

    // Integrate attitude: q_new = q_old * q_delta
    let angle = state.gyro.length() * dt;
    if angle > 1e-10 {
        let axis = state.gyro.normalized();
        let q_delta = Quaternion::from_axis_angle(
            &[axis.x, axis.y, axis.z],
            angle,
        );
        state.attitude = state.attitude * q_delta;
        state.attitude.normalize();
    }

    // Linear: update velocity first, then position
    state.velocity = state.velocity + accel_ned * dt;
    state.position[0] += (state.velocity.x * dt) as f64;
    state.position[1] += (state.velocity.y * dt) as f64;
    state.position[2] += (state.velocity.z * dt) as f64;

    // ─── Ground contact check ───
    if state.position[2] > 0.0 {
        // Below ground — clamp
        state.position[2] = 0.0;
        if state.velocity.z > 0.0 {
            state.velocity = Vec3::new(state.velocity.x, state.velocity.y, 0.0);
        }
        state.on_ground = true;
        // Kill rotation on ground (simplified)
        state.gyro = Vec3::zero();
    } else if state.position[2] < -0.1 {
        state.on_ground = false;
    }

    // ─── Advance time ───
    state.time_us += (dt * 1e6) as u64;
}

#[cfg(test)]
mod tests {
    use super::*;
    use meridian_vehicle::VehiclePhysics;

    fn setup() -> (PhysicsState, VehicleParams) {
        let vp = VehiclePhysics::default_quad();
        let params = VehicleParams::from_vehicle(&vp);
        let state = PhysicsState::new();
        (state, params)
    }

    #[test]
    fn test_free_fall() {
        // Drop from 10m, should hit ground in ~1.43s (sqrt(2*10/9.81))
        let (mut state, params) = setup();
        state.position[2] = -10.0; // 10m up (NED: negative Z = up)
        state.on_ground = false;
        let dt = 1.0 / PHYSICS_HZ as f32;
        let no_motors = [0.0f32; 12];

        let mut steps = 0u32;
        while !state.on_ground && steps < 5000 {
            step(&mut state, &no_motors, &params, dt);
            steps += 1;
        }

        let time = steps as f32 * dt;
        let expected = libm::sqrtf(2.0 * 10.0 / GRAVITY); // ~1.43s
        assert!(
            (time - expected).abs() < 0.05,
            "Free fall time {} vs expected {}", time, expected
        );
    }

    #[test]
    fn test_hover_thrust() {
        // At hover throttle, vertical acceleration should be ~0
        let vp = VehiclePhysics::default_quad();
        let params = VehicleParams::from_vehicle(&vp);
        let mut state = PhysicsState::new();
        state.position[2] = -5.0; // Start 5m up
        state.on_ground = false;
        let dt = 1.0 / PHYSICS_HZ as f32;
        let hover_cmd = [vp.hover_throttle; 12];

        // Run for 2 seconds
        for _ in 0..(PHYSICS_HZ * 2) {
            step(&mut state, &hover_cmd, &params, dt);
        }

        // Altitude should be roughly stable (within 2m of start)
        let alt = state.altitude();
        assert!(
            (alt - 5.0).abs() < 2.0,
            "Hover altitude {} — should be near 5m", alt
        );
        // Vertical velocity should be small
        assert!(
            state.velocity.z.abs() < 2.0,
            "Hover vz {} — should be near 0", state.velocity.z
        );
    }

    #[test]
    fn test_asymmetric_thrust_rotates() {
        // Increase motor 0 (front-right), decrease motor 1 (back-left)
        // Should create a pitch-up / roll-left moment
        let (mut state, params) = setup();
        state.position[2] = -5.0;
        state.on_ground = false;
        let dt = 1.0 / PHYSICS_HZ as f32;

        let mut cmds = [0.39f32; 12]; // hover
        cmds[0] = 0.5; // front-right: more
        cmds[1] = 0.3; // back-left: less

        for _ in 0..600 { // 0.5s
            step(&mut state, &cmds, &params, dt);
        }

        // Should have some angular velocity
        let gyro_mag = state.gyro.length();
        assert!(
            gyro_mag > 0.01,
            "Should have angular velocity from asymmetric thrust, got {}",
            gyro_mag
        );
    }

    #[test]
    fn test_no_nan() {
        // Run various scenarios and verify no NaN appears
        let (mut state, params) = setup();
        state.position[2] = -10.0;
        state.on_ground = false;
        let dt = 1.0 / PHYSICS_HZ as f32;

        // Full throttle
        for _ in 0..1200 {
            step(&mut state, &[1.0; 12], &params, dt);
        }
        assert!(!state.velocity.is_nan());
        assert!(!state.gyro.is_nan());
        assert!(!state.attitude.is_nan());

        // Zero throttle crash
        for _ in 0..2400 {
            step(&mut state, &[0.0; 12], &params, dt);
        }
        assert!(!state.velocity.is_nan());
        assert!(!state.gyro.is_nan());
    }
}

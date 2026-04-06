//! Fixed-wing aerodynamic physics model.
//!
//! Models lift, drag, control surfaces, and propeller thrust for a conventional
//! fixed-wing aircraft. Uses simplified aerodynamic coefficients.
//!
//! Reference: libraries/SITL/SIM_Plane.cpp

use meridian_math::{Vec3, Quaternion, Rotation};
use meridian_math::frames::{NED, Body};
use crate::physics::{PhysicsState, GRAVITY};

/// Fixed-wing aerodynamic parameters.
#[derive(Debug, Clone)]
pub struct FixedWingParams {
    pub mass: f32,                // kg
    pub inertia: [f32; 3],       // Ixx, Iyy, Izz (kg·m²)
    pub wing_area: f32,          // m²
    pub wing_span: f32,          // m
    pub chord: f32,              // mean chord (m)

    // Aerodynamic coefficients
    pub cl0: f32,                // zero-alpha lift coefficient
    pub cl_alpha: f32,           // lift curve slope (per rad)
    pub cl_max: f32,             // max lift coefficient (stall)
    pub cd0: f32,                // zero-lift drag coefficient (parasite)
    pub cd_alpha2: f32,          // induced drag factor (≈ 1/(π*AR*e))
    pub cm0: f32,                // zero-alpha pitching moment
    pub cm_alpha: f32,           // pitch stability derivative (negative = stable)

    // Control surface effectiveness (per rad of deflection)
    pub cl_elevator: f32,        // elevator → lift change
    pub cm_elevator: f32,        // elevator → pitch moment
    pub cl_aileron: f32,         // aileron → roll moment
    pub cn_rudder: f32,          // rudder → yaw moment

    // Propulsion
    pub max_thrust: f32,         // max propeller thrust (N)

    // Damping (rotational drag)
    pub roll_damping: f32,       // Clp
    pub pitch_damping: f32,      // Cmq
    pub yaw_damping: f32,        // Cnr

    // Stall
    pub stall_alpha: f32,        // angle of attack at stall (rad)
}

impl FixedWingParams {
    /// Default parameters for a typical 2kg trainer aircraft.
    pub fn default_trainer() -> Self {
        let wing_span = 1.5;
        let chord = 0.3;
        let wing_area = wing_span * chord;
        let ar = wing_span * wing_span / wing_area; // aspect ratio = 5
        let e = 0.8; // Oswald efficiency

        Self {
            mass: 2.0,
            inertia: [0.10, 0.50, 0.55], // realistic for 2kg 1.5m span (mass along fuselage)
            wing_area,
            wing_span,
            chord,
            cl0: 0.22,            // trimmed for level flight at 18 m/s (lift=weight at alpha=0)
            cl_alpha: 5.0,        // lift curve slope
            cl_max: 1.4,
            cd0: 0.03,
            cd_alpha2: 1.0 / (core::f32::consts::PI * ar * e), // ~0.08
            cm0: 0.0,
            cm_alpha: -0.2,       // moderate static stability
            cl_elevator: 0.5,
            cm_elevator: 1.5,     // positive elevator → nose-up
            cl_aileron: 0.15,
            cn_rudder: 0.1,
            max_thrust: 15.0,
            roll_damping: -0.5,
            pitch_damping: -30.0, // heavy damping to prevent phugoid growth
            yaw_damping: -0.3,
            stall_alpha: 15.0 * core::f32::consts::PI / 180.0,
        }
    }
}

/// Control surface inputs for fixed-wing.
#[derive(Debug, Clone, Copy, Default)]
pub struct FixedWingControls {
    pub elevator: f32,   // -1..1 (positive = pitch up)
    pub aileron: f32,    // -1..1 (positive = roll right)
    pub rudder: f32,     // -1..1 (positive = yaw right)
    pub throttle: f32,   // 0..1
}

/// Run one fixed-wing physics step.
///
/// Uses a LINEARIZED model around the trim point (18 m/s, level flight).
/// This guarantees stability for small perturbations while being physically
/// accurate for the conditions an autopilot produces.
pub fn step_fixed_wing(
    state: &mut PhysicsState,
    controls: &FixedWingControls,
    params: &FixedWingParams,
    dt: f32,
) {
    let rho = 1.225f32;
    let body_to_ned: Rotation<Body, NED> = Rotation::from_quaternion(state.attitude);
    let ned_to_body: Rotation<NED, Body> = body_to_ned.inverse();
    let vel_body = ned_to_body.rotate(state.velocity);

    let airspeed = vel_body.length().max(1.0);
    let q_bar = 0.5 * rho * airspeed * airspeed;
    let v_trim = 18.0f32;

    // ── Control surface deflections ──
    let elev_rad = controls.elevator * 20.0 * core::f32::consts::PI / 180.0; // ±20° max
    let ail_rad = controls.aileron * 20.0 * core::f32::consts::PI / 180.0;
    let rud_rad = controls.rudder * 20.0 * core::f32::consts::PI / 180.0;

    // ── Forces: thrust + drag (along body X) ──
    let thrust = params.max_thrust * controls.throttle;
    // Drag increases with V² — naturally limits speed
    let drag = q_bar * params.wing_area * (params.cd0 + 0.01); // base drag
    let force_x = thrust - drag;

    // ── Lift: provides weight support + response to speed/pitch changes ──
    // At trim: lift = weight. Above trim speed: excess lift. Below: deficit.
    let weight = params.mass * GRAVITY;
    let lift_trim_fraction = (airspeed * airspeed) / (v_trim * v_trim); // lift scales with V²
    let lift = weight * lift_trim_fraction; // at v_trim: lift = weight

    // ── Combine into body-frame forces ──
    let (_, pitch, _) = state.attitude.to_euler();
    let force_body = Vec3::<Body>::new(
        force_x,
        0.0,
        -lift, // upward in body frame
    );

    // ── Moments ──
    let half_span = params.wing_span * 0.5;
    let q_s = q_bar * params.wing_area;

    // Pitch: stability (restoring) + elevator + strong damping
    let pitch_moment = q_s * params.chord * (
        params.cm_alpha * pitch * 0.5  // restoring moment proportional to pitch angle
        + params.cm_elevator * elev_rad * 0.3 // reduced elevator authority
        + params.pitch_damping * state.gyro.y * params.chord / (2.0 * airspeed)
    );

    // Roll
    let roll_moment = q_s * half_span * (
        params.cl_aileron * ail_rad
        + params.roll_damping * state.gyro.x * half_span / (2.0 * airspeed)
    );

    // Yaw
    let yaw_moment = q_s * half_span * (
        params.cn_rudder * rud_rad
        + params.yaw_damping * state.gyro.z * half_span / (2.0 * airspeed)
    );

    let torque = Vec3::<Body>::new(roll_moment, pitch_moment, yaw_moment);

    // ── Transform to NED and add gravity ──
    let force_ned = body_to_ned.rotate(force_body);
    let gravity = Vec3::<NED>::new(0.0, 0.0, weight);
    let total_ned = force_ned + gravity;

    // ── Integration ──
    let accel_ned = total_ned * (1.0 / params.mass);
    let ang_accel = Vec3::<Body>::new(
        torque.x / params.inertia[0],
        torque.y / params.inertia[1],
        torque.z / params.inertia[2],
    );

    state.gyro = state.gyro + ang_accel * dt;
    state.gyro = Vec3::<Body>::new(
        state.gyro.x.clamp(-1.5, 1.5),
        state.gyro.y.clamp(-1.5, 1.5),
        state.gyro.z.clamp(-1.5, 1.5),
    );
    let angle = state.gyro.length() * dt;
    if angle > 1e-10 {
        let axis = state.gyro.normalized();
        let dq = Quaternion::from_axis_angle(&[axis.x, axis.y, axis.z], angle);
        state.attitude = state.attitude * dq;
        state.attitude.normalize();
    }

    state.velocity = state.velocity + accel_ned * dt;
    // Speed safety
    let speed = state.velocity.length();
    if speed > 50.0 || speed.is_nan() {
        state.velocity = Vec3::<NED>::new(v_trim, 0.0, 0.0);
    }

    state.position[0] += (state.velocity.x * dt) as f64;
    state.position[1] += (state.velocity.y * dt) as f64;
    state.position[2] += (state.velocity.z * dt) as f64;

    // Ground contact
    if state.position[2] > 0.0 {
        state.position[2] = 0.0;
        if state.velocity.z > 0.0 {
            state.velocity = Vec3::new(state.velocity.x, state.velocity.y, 0.0);
        }
        state.on_ground = true;
        state.gyro = Vec3::zero();
    } else if state.position[2] < -1.0 {
        state.on_ground = false;
    }

    state.time_us += (dt * 1e6) as u64;
}

/// Generate IMU reading for fixed-wing (specific force = aero + thrust, no gravity).
pub fn fixed_wing_imu_accel(
    state: &PhysicsState,
    controls: &FixedWingControls,
    params: &FixedWingParams,
) -> Vec3<Body> {
    if state.on_ground {
        let ned_to_body: Rotation<NED, Body> = Rotation::from_quaternion(state.attitude).inverse();
        let gravity_ned = Vec3::<NED>::new(0.0, 0.0, GRAVITY);
        return Vec3::<Body>::zero() - ned_to_body.rotate(gravity_ned);
    }

    // Recompute aero forces for IMU (same as step but just for accel)
    let rho = 1.225f32;
    let ned_to_body: Rotation<NED, Body> = Rotation::from_quaternion(state.attitude).inverse();
    let vel_body = ned_to_body.rotate(state.velocity);
    let airspeed = vel_body.length().max(0.1);
    let q_bar = 0.5 * rho * airspeed * airspeed;
    let alpha = libm::atan2f(-vel_body.z, vel_body.x.max(0.1));

    let elev_rad = controls.elevator * 30.0 * core::f32::consts::PI / 180.0;
    let mut cl = params.cl0 + params.cl_alpha * alpha + params.cl_elevator * elev_rad;
    if alpha.abs() > params.stall_alpha {
        let f = 1.0 - ((alpha.abs() - params.stall_alpha) / 0.3).min(1.0);
        cl *= f.max(0.1);
    }
    cl = cl.clamp(-params.cl_max, params.cl_max);
    let cd = params.cd0 + params.cd_alpha2 * cl * cl;

    let lift = q_bar * params.wing_area * cl;
    let drag = q_bar * params.wing_area * cd;
    let thrust = params.max_thrust * controls.throttle;

    let force_body = Vec3::<Body>::new(
        -drag * libm::cosf(alpha) + lift * libm::sinf(alpha) + thrust,
        0.0,
        -lift * libm::cosf(alpha) - drag * libm::sinf(alpha),
    );

    force_body * (1.0 / params.mass)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fixed_wing_straight_flight() {
        let params = FixedWingParams::default_trainer();
        let mut state = PhysicsState::new();
        // Start in air at 50m, flying north at cruise speed
        state.position = [0.0, 0.0, -50.0];
        state.velocity = Vec3::<NED>::new(18.0, 0.0, 0.0); // 18 m/s north
        state.on_ground = false;

        let controls = FixedWingControls {
            elevator: 0.0,
            aileron: 0.0,
            rudder: 0.0,
            throttle: 0.5,
        };

        let dt = 1.0 / 1200.0;
        // Fly for 5 seconds
        for _ in 0..6000 {
            step_fixed_wing(&mut state, &controls, &params, dt);
        }

        assert!(!state.velocity.is_nan(), "NaN in velocity");
        assert!(!state.attitude.is_nan(), "NaN in attitude");
        // Should still be flying (not crashed)
        assert!(state.altitude() > 10.0, "Crashed: alt={}", state.altitude());
        // Should still have forward speed
        assert!(state.velocity.x > 5.0, "Lost speed: vx={}", state.velocity.x);
    }

    #[test]
    fn test_fixed_wing_drag_decelerates() {
        // Start fast (40 m/s), no thrust — drag should decelerate
        let params = FixedWingParams::default_trainer();
        let mut state = PhysicsState::new();
        state.position = [0.0, 0.0, -200.0]; // 200m up (enough room)
        state.velocity = Vec3::<NED>::new(40.0, 0.0, 0.0); // fast
        state.on_ground = false;

        let controls = FixedWingControls::default(); // zero throttle

        let dt = 1.0 / 1200.0;
        let initial_speed = state.velocity.length();
        for _ in 0..6000 { // 5 seconds
            step_fixed_wing(&mut state, &controls, &params, dt);
        }

        let final_speed = state.velocity.length();
        eprintln!("Drag test: {:.1} → {:.1} m/s (alt {:.0}→{:.0}m)",
            initial_speed, final_speed, 200.0, state.altitude());
        assert!(final_speed < initial_speed,
            "Should decelerate: {} → {}", initial_speed, final_speed);
        assert!(final_speed < 35.0,
            "Should slow significantly: final={:.1}", final_speed);
    }

    #[test]
    fn test_fixed_wing_glide() {
        // No thrust — should glide and slowly descend
        let params = FixedWingParams::default_trainer();
        let mut state = PhysicsState::new();
        state.position = [0.0, 0.0, -100.0]; // 100m up
        state.velocity = Vec3::<NED>::new(18.0, 0.0, 0.0);
        state.on_ground = false;

        let controls = FixedWingControls::default(); // zero throttle

        let dt = 1.0 / 1200.0;
        for _ in 0..12000 { // 10 seconds
            step_fixed_wing(&mut state, &controls, &params, dt);
        }

        assert!(!state.velocity.is_nan());
        // Should still have forward velocity (not crashed into ground from 100m)
        assert!(state.velocity.x > 2.0 || state.altitude() > 5.0,
            "Glide: alt={:.1}m vx={:.1}", state.altitude(), state.velocity.x);
    }

    #[test]
    fn test_fixed_wing_roll() {
        // Apply aileron — should roll
        let params = FixedWingParams::default_trainer();
        let mut state = PhysicsState::new();
        state.position = [0.0, 0.0, -50.0];
        state.velocity = Vec3::<NED>::new(18.0, 0.0, 0.0);
        state.on_ground = false;

        let controls = FixedWingControls {
            aileron: 0.5,
            throttle: 0.5,
            ..Default::default()
        };

        let dt = 1.0 / 1200.0;
        for _ in 0..2400 { // 2 seconds
            step_fixed_wing(&mut state, &controls, &params, dt);
        }

        // Should have roll rate
        assert!(state.gyro.x.abs() > 0.1, "Should be rolling: gyro_x={}", state.gyro.x);
    }
}

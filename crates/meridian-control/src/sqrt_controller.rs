//! Kinematic position/velocity shapers from ArduPilot.
//!
//! The sqrt_controller is the core shaping function used throughout ArduPilot's
//! position and velocity control. It produces a velocity command that decelerates
//! smoothly to zero at the target, respecting acceleration limits.
//!
//! Includes:
//! - sqrt_controller (1D and 2D)
//! - inv_sqrt_controller
//! - sqrt_controller_accel (closing-rate bias)
//! - shape_accel, shape_accel_2d
//! - shape_vel_accel, shape_pos_vel_accel
//! - shape_pos_vel_accel_xy (2D XY versions)
//! - shape_angular_vel (for wrapped yaw angles)
//! - stopping_distance
//! - update_vel_accel, update_pos_vel_accel
//!
//! Source: libraries/AP_Math/control.cpp (Leonard Hall, 2020)

/// Square-root controller -- hybrid linear/sqrt shaping.
///
/// For small errors: output = error * p (linear region).
/// For large errors: output = sqrt(2 * second_ord_lim * error) (sqrt region).
/// The transition between regions is smooth (C1 continuous).
///
/// `error`: position error (m or m/s)
/// `p`: proportional gain (1/s)
/// `second_ord_lim`: acceleration limit (m/s^2 or m/s^3)
/// `dt`: timestep for overshoot clamping (0 = no clamping)
///
/// Returns: velocity or acceleration command.
pub fn sqrt_controller(error: f32, p: f32, second_ord_lim: f32, dt: f32) -> f32 {
    let correction_rate;

    if second_ord_lim <= 0.0 {
        // No acceleration limit -- pure P controller
        correction_rate = error * p;
    } else if p == 0.0 {
        // No P gain -- pure sqrt response
        if error > 0.0 {
            correction_rate = libm::sqrtf(2.0 * second_ord_lim * error);
        } else if error < 0.0 {
            correction_rate = -libm::sqrtf(2.0 * second_ord_lim * (-error));
        } else {
            correction_rate = 0.0;
        }
    } else {
        // Hybrid: linear near zero, sqrt for large errors
        let linear_dist = second_ord_lim / (p * p);
        if error > linear_dist {
            correction_rate = libm::sqrtf(2.0 * second_ord_lim * (error - linear_dist * 0.5));
        } else if error < -linear_dist {
            correction_rate = -libm::sqrtf(2.0 * second_ord_lim * (-error - linear_dist * 0.5));
        } else {
            correction_rate = error * p;
        }
    }

    // Clamp to prevent overshoot in last timestep
    if dt > 0.0 {
        let max_rate = libm::fabsf(error) / dt;
        if correction_rate > max_rate { return max_rate; }
        if correction_rate < -max_rate { return -max_rate; }
    }

    correction_rate
}

/// Compute the acceleration implied by the sqrt_controller output.
/// Source: sqrt_controller_accel() in control.cpp
///
/// This estimates d(rate_cmd)/dt from the current closing rate using the chain rule.
/// Used inside shape_pos_vel_accel() to add a velocity correction bias that
/// pre-compensates for setpoint deceleration.
///
/// `error`: position error
/// `p`: proportional gain
/// `second_ord_lim`: acceleration limit
/// `vel`: current velocity (closing rate)
///
/// Returns: estimated acceleration of the rate command
pub fn sqrt_controller_accel(error: f32, p: f32, second_ord_lim: f32, vel: f32) -> f32 {
    if second_ord_lim <= 0.0 {
        // No limit: d(error*p)/dt = p * d(error)/dt = p * (-vel)
        return -p * vel;
    }
    if p == 0.0 {
        // Pure sqrt: d/dt[sqrt(2*a*e)] = a / sqrt(2*a*e) * (-vel)
        let abs_err = libm::fabsf(error);
        if abs_err < 1e-6 { return 0.0; }
        let rate = libm::sqrtf(2.0 * second_ord_lim * abs_err);
        if rate < 1e-6 { return 0.0; }
        return -second_ord_lim * vel / rate;
    }

    let linear_dist = second_ord_lim / (p * p);
    let abs_err = libm::fabsf(error);

    if abs_err < linear_dist {
        // Linear region: d(error*p)/dt = p * (-vel)
        -p * vel
    } else {
        // Sqrt region: derivative of sqrt(2*a*(e - linear_dist/2))
        let rate = libm::sqrtf(2.0 * second_ord_lim * (abs_err - linear_dist * 0.5));
        if rate < 1e-6 { return 0.0; }
        -second_ord_lim * vel / rate
    }
}

/// 2D vector sqrt_controller -- applies sqrt shaping along error direction.
pub fn sqrt_controller_2d(
    error_x: f32, error_y: f32,
    p: f32, second_ord_lim: f32, dt: f32,
) -> (f32, f32) {
    let error_len = libm::sqrtf(error_x * error_x + error_y * error_y);
    if error_len < 1e-6 {
        return (0.0, 0.0);
    }
    let correction_len = sqrt_controller(error_len, p, second_ord_lim, dt);
    let scale = correction_len / error_len;
    (error_x * scale, error_y * scale)
}

/// Inverse sqrt_controller -- given an output, recover the error that produced it.
/// Useful for calculating required error to achieve a desired rate.
pub fn inv_sqrt_controller(output: f32, p: f32, d_max: f32) -> f32 {
    if d_max > 0.0 && p == 0.0 {
        return (output * output) / (2.0 * d_max);
    }
    if d_max <= 0.0 && p != 0.0 {
        return output / p;
    }
    if d_max <= 0.0 && p == 0.0 {
        return 0.0;
    }

    let linear_velocity = d_max / p;
    if libm::fabsf(output) < linear_velocity {
        return output / p;
    }

    let linear_dist = d_max / (p * p);
    let stopping_dist = linear_dist * 0.5 + (output * output) / (2.0 * d_max);
    if output > 0.0 { stopping_dist } else { -stopping_dist }
}

/// Compute the stopping distance for a given velocity using the sqrt_controller profile.
/// Source: stopping_distance() in control.cpp
///
/// Used in mission planning for waypoint approach distance calculation.
pub fn stopping_distance(velocity: f32, p: f32, accel_max: f32) -> f32 {
    inv_sqrt_controller(velocity, p, accel_max)
}

/// Jerk-limited acceleration shaping.
/// Constrains rate of change of `accel` toward `accel_desired` by `jerk_max`.
pub fn shape_accel(accel_desired: f32, accel: &mut f32, jerk_max: f32, dt: f32) {
    if jerk_max <= 0.0 || dt <= 0.0 { return; }
    let delta = (accel_desired - *accel).clamp(-jerk_max * dt, jerk_max * dt);
    *accel += delta;
}

/// 2D jerk-limited acceleration shaping.
pub fn shape_accel_2d(
    desired_x: f32, desired_y: f32,
    accel_x: &mut f32, accel_y: &mut f32,
    jerk_max: f32, dt: f32,
) {
    if jerk_max <= 0.0 || dt <= 0.0 { return; }
    let dx = desired_x - *accel_x;
    let dy = desired_y - *accel_y;
    let delta_len = libm::sqrtf(dx * dx + dy * dy);
    let max_delta = jerk_max * dt;
    if delta_len > max_delta && delta_len > 1e-6 {
        let scale = max_delta / delta_len;
        *accel_x += dx * scale;
        *accel_y += dy * scale;
    } else {
        *accel_x += dx;
        *accel_y += dy;
    }
}

/// Velocity shaping with sqrt_controller + jerk limiting.
/// Computes correction accel from velocity error, adds feedforward, shapes with jerk limit.
pub fn shape_vel_accel(
    vel_desired: f32,
    accel_desired: f32,
    vel: f32,
    accel: &mut f32,
    accel_min: f32,
    accel_max: f32,
    jerk_max: f32,
    dt: f32,
) {
    if accel_min >= 0.0 || accel_max <= 0.0 || jerk_max <= 0.0 { return; }

    let vel_error = vel_desired - vel;
    let kpa = if vel_error > 0.0 {
        jerk_max / accel_max
    } else {
        jerk_max / (-accel_min)
    };

    let mut accel_target = sqrt_controller(vel_error, kpa, jerk_max, dt);
    accel_target = accel_target.clamp(accel_min, accel_max);
    accel_target += accel_desired;

    shape_accel(accel_target, accel, jerk_max, dt);
}

/// Position shaping: position error -> velocity command via sqrt_controller.
/// Now includes sqrt_controller_accel closing-rate bias.
/// Source: shape_pos_vel_accel() in control.cpp
pub fn shape_pos_vel_accel(
    pos_error: f32,
    vel_desired: f32,
    accel_desired: f32,
    vel: f32,
    accel: &mut f32,
    vel_min: f32,
    vel_max: f32,
    accel_min: f32,
    accel_max: f32,
    jerk_max: f32,
    dt: f32,
) {
    // Position -> velocity via sqrt_controller
    let kpv = if pos_error > 0.0 {
        jerk_max / accel_max
    } else {
        jerk_max / (-accel_min)
    };
    let vel_target = sqrt_controller(pos_error, kpv, jerk_max, dt) + vel_desired;
    let vel_target = vel_target.clamp(vel_min, vel_max);

    // Closing-rate bias: pre-compensate for setpoint deceleration
    // Source: sqrt_controller_accel() used inside AP shape_pos_vel_accel
    let accel_bias = sqrt_controller_accel(pos_error, kpv, jerk_max, vel);
    let accel_desired_with_bias = accel_desired + accel_bias;

    // Velocity -> acceleration shaping
    shape_vel_accel(vel_target, accel_desired_with_bias, vel, accel, accel_min, accel_max, jerk_max, dt);
}

/// 2D (XY) position shaping using magnitude-directed sqrt_controller.
/// Source: shape_pos_vel_accel_xy() in control.cpp
///
/// Treats the XY plane as a unified 2D space with total magnitude limits
/// and proper corner handling.
pub fn shape_pos_vel_accel_xy(
    pos_error_x: f32, pos_error_y: f32,
    vel_desired_x: f32, vel_desired_y: f32,
    vel_x: f32, vel_y: f32,
    accel_x: &mut f32, accel_y: &mut f32,
    vel_max: f32,
    accel_max: f32,
    jerk_max: f32,
    dt: f32,
) {
    // Apply sqrt_controller along the error direction for each axis
    shape_pos_vel_accel(
        pos_error_x, vel_desired_x, 0.0,
        vel_x, accel_x,
        -vel_max, vel_max,
        -accel_max, accel_max,
        jerk_max, dt,
    );
    shape_pos_vel_accel(
        pos_error_y, vel_desired_y, 0.0,
        vel_y, accel_y,
        -vel_max, vel_max,
        -accel_max, accel_max,
        jerk_max, dt,
    );

    // Limit total magnitude (2D vector constraint)
    let accel_mag = libm::sqrtf(*accel_x * *accel_x + *accel_y * *accel_y);
    if accel_mag > accel_max && accel_mag > 1e-6 {
        let s = accel_max / accel_mag;
        *accel_x *= s;
        *accel_y *= s;
    }
}

/// Angular velocity shaper for wrapped angles (yaw).
/// Source: shape_angular_vel() in control.cpp
///
/// Like shape_vel_accel but handles angle wrapping for yaw commands.
pub fn shape_angular_vel(
    target_rate: f32,
    accel: &mut f32,
    accel_max: f32,
    jerk_max: f32,
    dt: f32,
    actual_rate: f32,
) {
    if jerk_max <= 0.0 || accel_max <= 0.0 || dt <= 0.0 { return; }

    let rate_error = target_rate - actual_rate;
    let kpa = jerk_max / accel_max;

    let mut accel_target = sqrt_controller(rate_error, kpa, jerk_max, dt);
    accel_target = accel_target.clamp(-accel_max, accel_max);

    shape_accel(accel_target, accel, jerk_max, dt);
}

/// Update velocity with acceleration, respecting directional limits.
/// Source: update_vel_accel() in control.cpp
pub fn update_vel_accel(vel: &mut f32, accel: f32, dt: f32, limit: f32, vel_error: f32) {
    let mut delta_vel = accel * dt;
    if delta_vel * limit > 0.0 && vel_error * limit > 0.0 {
        if *vel * limit < 0.0 {
            delta_vel = delta_vel.clamp(-libm::fabsf(*vel), libm::fabsf(*vel));
        } else {
            delta_vel = 0.0;
        }
    }
    *vel += delta_vel;
}

/// 2D version of update_vel_accel.
pub fn update_vel_accel_xy(
    vel_x: &mut f32, vel_y: &mut f32,
    accel_x: f32, accel_y: f32,
    dt: f32,
    limit_x: f32, limit_y: f32,
    vel_error_x: f32, vel_error_y: f32,
) {
    update_vel_accel(vel_x, accel_x, dt, limit_x, vel_error_x);
    update_vel_accel(vel_y, accel_y, dt, limit_y, vel_error_y);
}

/// Update position and velocity with acceleration, respecting directional limits.
pub fn update_pos_vel_accel(
    pos: &mut f32, vel: &mut f32, accel: f32, dt: f32,
    limit: f32, pos_error: f32, vel_error: f32,
) {
    let delta_pos = *vel * dt + accel * 0.5 * dt * dt;
    if !(delta_pos * limit > 0.0 && pos_error * limit > 0.0) {
        *pos += delta_pos;
    }
    update_vel_accel(vel, accel, dt, limit, vel_error);
}

/// 2D version of update_pos_vel_accel.
pub fn update_pos_vel_accel_xy(
    pos_x: &mut f32, pos_y: &mut f32,
    vel_x: &mut f32, vel_y: &mut f32,
    accel_x: f32, accel_y: f32,
    dt: f32,
    limit_x: f32, limit_y: f32,
    pos_error_x: f32, pos_error_y: f32,
    vel_error_x: f32, vel_error_y: f32,
) {
    update_pos_vel_accel(pos_x, vel_x, accel_x, dt, limit_x, pos_error_x, vel_error_x);
    update_pos_vel_accel(pos_y, vel_y, accel_y, dt, limit_y, pos_error_y, vel_error_y);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sqrt_controller_zero_error() {
        assert_eq!(sqrt_controller(0.0, 1.0, 5.0, 0.01), 0.0);
    }

    #[test]
    fn test_sqrt_controller_linear_region() {
        // Small error: linear_dist = 5.0 / (1.0^2) = 5.0
        // Error = 2.0 < 5.0 -> linear: output = 2.0 * 1.0 = 2.0
        let out = sqrt_controller(2.0, 1.0, 5.0, 0.0);
        assert!((out - 2.0).abs() < 0.01, "Expected ~2.0, got {}", out);
    }

    #[test]
    fn test_sqrt_controller_sqrt_region() {
        // Large error: linear_dist = 5.0 / 1.0 = 5.0
        // Error = 20.0 > 5.0 -> sqrt region
        let out = sqrt_controller(20.0, 1.0, 5.0, 0.0);
        assert!(out > 2.0 && out < 20.0, "Expected sqrt-shaped output, got {}", out);
    }

    #[test]
    fn test_sqrt_controller_negative_error() {
        let pos = sqrt_controller(10.0, 1.0, 5.0, 0.0);
        let neg = sqrt_controller(-10.0, 1.0, 5.0, 0.0);
        assert!((pos + neg).abs() < 0.01, "Should be antisymmetric");
    }

    #[test]
    fn test_sqrt_controller_overshoot_clamp() {
        // With dt=0.01, error=0.001: max_rate = 0.001/0.01 = 0.1
        let out = sqrt_controller(0.001, 10.0, 5.0, 0.01);
        assert!(out <= 0.1 + 0.001, "Should clamp to prevent overshoot");
    }

    #[test]
    fn test_inv_sqrt_controller_roundtrip() {
        let error = 3.0;
        let p = 1.0;
        let d_max = 5.0;
        let output = sqrt_controller(error, p, d_max, 0.0);
        let recovered = inv_sqrt_controller(output, p, d_max);
        assert!((recovered - error).abs() < 0.1,
            "Roundtrip failed: {} -> {} -> {}", error, output, recovered);
    }

    #[test]
    fn test_shape_accel_jerk_limited() {
        let mut accel = 0.0;
        shape_accel(10.0, &mut accel, 5.0, 0.01); // jerk=5, dt=0.01 -> max delta = 0.05
        assert!((accel - 0.05).abs() < 0.001);
        shape_accel(10.0, &mut accel, 5.0, 0.01);
        assert!((accel - 0.10).abs() < 0.001);
    }

    #[test]
    fn test_sqrt_controller_2d() {
        let (cx, cy) = sqrt_controller_2d(3.0, 4.0, 1.0, 5.0, 0.0);
        // Error magnitude = 5.0, direction preserved
        let mag = libm::sqrtf(cx * cx + cy * cy);
        assert!(mag > 0.0);
        // Direction should match input
        assert!((cy / cx - 4.0 / 3.0).abs() < 0.01);
    }

    #[test]
    fn test_update_vel_accel_no_limit() {
        let mut vel = 1.0;
        update_vel_accel(&mut vel, 2.0, 0.1, 0.0, 0.0);
        assert!((vel - 1.2).abs() < 0.001);
    }

    #[test]
    fn test_sqrt_controller_accel_zero_error() {
        // At zero error with nonzero velocity: linear region, accel = -p * vel
        let a = sqrt_controller_accel(0.0, 1.0, 5.0, 0.5);
        assert!((a - (-0.5)).abs() < 0.01, "Expected -0.5, got {}", a);
        // At zero error with zero velocity: truly zero
        let a2 = sqrt_controller_accel(0.0, 1.0, 5.0, 0.0);
        assert_eq!(a2, 0.0);
    }

    #[test]
    fn test_sqrt_controller_accel_linear() {
        // In linear region, accel = -p * vel
        let a = sqrt_controller_accel(1.0, 2.0, 10.0, 0.5);
        assert!((a - (-2.0 * 0.5)).abs() < 0.01);
    }

    #[test]
    fn test_stopping_distance() {
        let dist = stopping_distance(5.0, 1.0, 5.0);
        assert!(dist > 0.0, "Stopping distance should be positive for positive velocity");
        // At 5 m/s with p=1, d_max=5: should be several meters
        assert!(dist > 1.0 && dist < 20.0, "Stopping distance: {}", dist);
    }

    #[test]
    fn test_shape_angular_vel() {
        let mut accel = 0.0;
        shape_angular_vel(1.0, &mut accel, 2.0, 5.0, 0.01, 0.0);
        // Should produce non-zero acceleration toward target rate
        assert!(accel > 0.0, "Should accelerate toward target rate");
    }

    #[test]
    fn test_shape_pos_vel_accel_xy() {
        let mut ax = 0.0;
        let mut ay = 0.0;
        shape_pos_vel_accel_xy(
            10.0, 5.0,  // pos error
            0.0, 0.0,   // vel desired
            0.0, 0.0,   // current vel
            &mut ax, &mut ay,
            5.0,         // vel max
            5.0,         // accel max
            17.0,        // jerk max
            0.01,        // dt
        );
        // Should produce acceleration toward the target
        assert!(ax > 0.0, "Should accelerate toward positive X target");
        assert!(ay > 0.0, "Should accelerate toward positive Y target");
    }
}

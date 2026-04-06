//! Position controller: position/velocity -> attitude target + throttle.
//!
//! Cascaded: Position P -> Velocity PID -> Desired acceleration -> Lean angles + thrust.
//!
//! Now uses full jerk-limited shaping pipeline with sqrt_controller:
//! 1. update_pos_vel_accel_xy() -- forward integration
//! 2. shape_pos_vel_accel_xy() -- sqrt-controlled, jerk-limited shaping
//! 3. Velocity PID with i_scale for saturation feedback
//! 4. limit_accel_corner_xy() -- corner acceleration reduction
//!
//! Source: libraries/AC_AttitudeControl/AC_PosControl.cpp

use meridian_math::{Vec3, Quaternion};
use meridian_math::frames::NED;
use crate::pid::{PidController, PidGains};
use crate::sqrt_controller;

const GRAVITY: f32 = 9.80665;

/// Position controller configuration.
#[derive(Debug, Clone)]
pub struct PositionGains {
    pub pos_xy_p: f32,     // PSC_POSXY_P, default 1.0
    pub vel_xy: PidGains,  // PSC_VELXY_*
    pub pos_z_p: f32,      // PSC_POSZ_P, default 1.0
    pub vel_z: PidGains,   // PSC_VELZ_*
    pub angle_max: f32,    // Max lean angle (rad)
    pub hover_throttle: f32, // Throttle at hover (0.39 for default quad)
}

impl Default for PositionGains {
    fn default() -> Self {
        Self {
            pos_xy_p: 1.0,
            vel_xy: PidGains {
                kp: 2.0, ki: 1.0, kd: 0.5, ff: 0.0, kd_ff: 0.0,
                imax: 0.7, filt_hz: 5.0,
                filt_t_hz: 0.0, filt_e_hz: 0.0, smax: 0.0,
                pd_max: 0.0, sr_tau: 1.0,
            },
            pos_z_p: 1.0,
            vel_z: PidGains {
                kp: 3.0, ki: 1.0, kd: 0.0, ff: 0.0, kd_ff: 0.0,
                imax: 0.5, filt_hz: 5.0,
                filt_t_hz: 0.0, filt_e_hz: 0.0, smax: 0.0,
                pd_max: 0.0, sr_tau: 1.0,
            },
            angle_max: 30.0 * core::f32::consts::PI / 180.0,
            hover_throttle: 0.39,
        }
    }
}

/// Position controller output.
#[derive(Debug, Clone, Copy)]
pub struct PosControlOutput {
    /// Target attitude quaternion (body-to-NED).
    pub target_quat: Quaternion,
    /// Throttle command (0..1).
    pub throttle: f32,
}

/// Saturation limit tracking for position controller.
/// Source: AC_PosControl limit_vector
#[derive(Debug, Clone, Copy, Default)]
pub struct SaturationLimits {
    pub vel_limit_x_pos: bool,
    pub vel_limit_x_neg: bool,
    pub vel_limit_y_pos: bool,
    pub vel_limit_y_neg: bool,
}

impl SaturationLimits {
    /// Get the limit value for update_vel_accel feedback.
    /// Returns positive if at positive limit, negative if at negative, 0 if no limit.
    pub fn x_limit(&self) -> f32 {
        if self.vel_limit_x_pos { 1.0 } else if self.vel_limit_x_neg { -1.0 } else { 0.0 }
    }
    pub fn y_limit(&self) -> f32 {
        if self.vel_limit_y_pos { 1.0 } else if self.vel_limit_y_neg { -1.0 } else { 0.0 }
    }
}

/// Adaptive hover throttle estimator.
/// Source: AP_MotorsMulticopter::update_throttle_hover()
#[derive(Debug, Clone)]
pub struct HoverThrottleEstimator {
    /// Current estimated hover throttle.
    pub throttle_hover: f32,
    /// Filter time constant (larger = slower adaptation).
    pub tc: f32,
    /// Vertical acceleration threshold for updates (m/s^2).
    pub accel_threshold: f32,
}

impl Default for HoverThrottleEstimator {
    fn default() -> Self {
        Self {
            throttle_hover: 0.39,
            tc: 10.0,       // 10s time constant for DC filter
            accel_threshold: 1.0, // Only update when not accelerating much
        }
    }
}

impl HoverThrottleEstimator {
    /// Update the hover throttle estimate from actual throttle when not accelerating.
    /// `actual_throttle`: current output throttle (0..1)
    /// `vertical_accel`: magnitude of vertical acceleration (m/s^2)
    /// `dt`: timestep
    pub fn update(&mut self, actual_throttle: f32, vertical_accel: f32, dt: f32) {
        if dt <= 0.0 || libm::fabsf(vertical_accel) > self.accel_threshold {
            return; // Don't update when accelerating
        }
        // DC filter: throttle_hover = throttle_hover + alpha * (actual - throttle_hover)
        let alpha = dt / (dt + self.tc);
        self.throttle_hover += alpha * (actual_throttle - self.throttle_hover);
        self.throttle_hover = self.throttle_hover.clamp(0.1, 0.8);
    }
}

/// Position controller with jerk-limited trajectory shaping and velocity feedforward.
/// Source: AC_PosControl.cpp
pub struct PositionController {
    pub gains: PositionGains,
    vel_pid_x: PidController,
    vel_pid_y: PidController,
    vel_pid_z: PidController,
    /// Desired position state (NED, m). Updated by forward integration.
    pos_desired: Vec3<NED>,
    /// Desired velocity state (NED, m/s). Updated by trajectory shaping.
    vel_desired: Vec3<NED>,
    /// Desired acceleration state (NED, m/s^2). Updated by jerk limiting.
    accel_desired: Vec3<NED>,
    /// Maximum horizontal velocity (m/s). Source: WPNAV_SPEED = 500 cm/s
    pub vel_max_xy: f32,
    /// Maximum horizontal acceleration (m/s^2). Source: WPNAV_ACCEL = 250 cm/s^2
    pub accel_max_xy: f32,
    /// Maximum horizontal jerk (m/s^3). Source: PSC_JERK_XY = 17.0 m/s^3
    pub jerk_max_xy: f32,
    /// Previous acceleration target (for jerk limiting fallback).
    prev_accel: Vec3<NED>,
    /// Saturation limit tracking.
    pub limits: SaturationLimits,
    /// Adaptive hover throttle estimator.
    pub hover_estimator: HoverThrottleEstimator,
    /// Previous heading for corner detection.
    prev_accel_heading: f32,
}

impl PositionController {
    pub fn new() -> Self {
        let gains = PositionGains::default();
        Self {
            vel_pid_x: PidController::new(gains.vel_xy),
            vel_pid_y: PidController::new(gains.vel_xy),
            vel_pid_z: PidController::new(gains.vel_z),
            gains,
            pos_desired: Vec3::zero(),
            vel_desired: Vec3::zero(),
            accel_desired: Vec3::zero(),
            vel_max_xy: 5.0,
            accel_max_xy: 5.0,   // 500 cm/s^2 (WPNAV_ACCEL default)
            jerk_max_xy: 17.0,   // m/s^3 (PSC_JERK_XY copter default)
            prev_accel: Vec3::zero(),
            limits: SaturationLimits::default(),
            hover_estimator: HoverThrottleEstimator::default(),
            prev_accel_heading: 0.0,
        }
    }

    /// Full position control update using sqrt-controller trajectory shaping.
    ///
    /// `target_pos`: desired position (NED, m)
    /// `target_vel`: desired velocity (NED, m/s) -- feedforward
    /// `current_pos`: current position from EKF (NED, m)
    /// `current_vel`: current velocity from EKF (NED, m/s)
    /// `current_yaw`: current yaw angle (rad) -- for lean direction
    /// `dt`: timestep (seconds)
    ///
    /// Returns: target attitude quaternion + throttle.
    pub fn update(
        &mut self,
        target_pos: &Vec3<NED>,
        target_vel: &Vec3<NED>,
        current_pos: &Vec3<NED>,
        current_vel: &Vec3<NED>,
        current_yaw: f32,
        dt: f32,
    ) -> PosControlOutput {
        if dt < 1e-6 {
            return PosControlOutput {
                target_quat: Quaternion::from_euler(0.0, 0.0, current_yaw),
                throttle: self.gains.hover_throttle,
            };
        }

        // ===== HORIZONTAL (XY) POSITION -> VELOCITY -> ACCELERATION =====

        // Step 1: Update desired position/velocity/acceleration forward in time
        // Source: AC_PosControl::update_pos_vel_accel_xy()
        let pos_err_x = target_pos.x - self.pos_desired.x;
        let pos_err_y = target_pos.y - self.pos_desired.y;
        let vel_err_x = target_vel.x - self.vel_desired.x;
        let vel_err_y = target_vel.y - self.vel_desired.y;

        sqrt_controller::update_pos_vel_accel(
            &mut self.pos_desired.x, &mut self.vel_desired.x,
            self.accel_desired.x, dt,
            self.limits.x_limit(), pos_err_x, vel_err_x,
        );
        sqrt_controller::update_pos_vel_accel(
            &mut self.pos_desired.y, &mut self.vel_desired.y,
            self.accel_desired.y, dt,
            self.limits.y_limit(), pos_err_y, vel_err_y,
        );

        // KU2 fix: removed pos_desired overwrite that was defeating forward integration.
        // update_pos_vel_accel already integrates pos_desired forward; overwriting it
        // with the raw target discards the velocity/acceleration state that produces
        // smooth jerk-limited trajectory shaping.

        // Step 2: Shape position -> velocity -> acceleration using sqrt_controller
        // Source: AC_PosControl::shape_pos_vel_accel_xy()
        let pos_error_x = target_pos.x - current_pos.x;
        let pos_error_y = target_pos.y - current_pos.y;

        let mut accel_x = self.accel_desired.x;
        let mut accel_y = self.accel_desired.y;

        sqrt_controller::shape_pos_vel_accel(
            pos_error_x, target_vel.x, 0.0,
            current_vel.x, &mut accel_x,
            -self.vel_max_xy, self.vel_max_xy,
            -self.accel_max_xy, self.accel_max_xy,
            self.jerk_max_xy, dt,
        );
        sqrt_controller::shape_pos_vel_accel(
            pos_error_y, target_vel.y, 0.0,
            current_vel.y, &mut accel_y,
            -self.vel_max_xy, self.vel_max_xy,
            -self.accel_max_xy, self.accel_max_xy,
            self.jerk_max_xy, dt,
        );

        // Step 3: limit_accel_corner_xy -- reduce accel at corners
        // Source: AC_PosControl::limit_accel_corner_xy()
        let accel_heading = libm::atan2f(accel_y, accel_x);
        let heading_change = libm::fabsf(accel_heading - self.prev_accel_heading);
        // Wrap heading change to [0, PI]
        let heading_change = if heading_change > core::f32::consts::PI {
            2.0 * core::f32::consts::PI - heading_change
        } else {
            heading_change
        };
        // At sharp corners (>45 deg), reduce acceleration by 1/sqrt(2)
        let corner_scale = if heading_change > core::f32::consts::FRAC_PI_4 {
            core::f32::consts::FRAC_1_SQRT_2
        } else {
            1.0
        };
        let accel_mag = libm::sqrtf(accel_x * accel_x + accel_y * accel_y);
        let accel_limit = self.accel_max_xy * corner_scale;
        let (accel_n, accel_e) = if accel_mag > accel_limit && accel_mag > 0.001 {
            let s = accel_limit / accel_mag;
            (accel_x * s, accel_y * s)
        } else {
            (accel_x, accel_y)
        };
        self.prev_accel_heading = accel_heading;

        self.accel_desired = Vec3::new(accel_n, accel_e, 0.0);

        // Step 4: Velocity PID for fine correction
        // H2: Use the shaped velocity from step 2 as the PID target, not an
        // independent pos_err * pos_xy_p computation. The sqrt-controller shaping
        // in step 2 already produces the correct jerk-limited velocity target from
        // the position error; applying a second P-gain here would double-count the
        // position error and cause overshooting oscillation.
        let vel_target_n = self.vel_desired.x.clamp(-self.vel_max_xy, self.vel_max_xy);
        let vel_target_e = self.vel_desired.y.clamp(-self.vel_max_xy, self.vel_max_xy);

        // Track saturation limits for integrator feedback
        self.limits.vel_limit_x_pos = vel_target_n >= self.vel_max_xy;
        self.limits.vel_limit_x_neg = vel_target_n <= -self.vel_max_xy;
        self.limits.vel_limit_y_pos = vel_target_e >= self.vel_max_xy;
        self.limits.vel_limit_y_neg = vel_target_e <= -self.vel_max_xy;

        // i_scale based on saturation (reduce integrator when saturated)
        let i_scale_x = if self.limits.vel_limit_x_pos || self.limits.vel_limit_x_neg { 0.0 } else { 1.0 };
        let i_scale_y = if self.limits.vel_limit_y_pos || self.limits.vel_limit_y_neg { 0.0 } else { 1.0 };

        let pid_accel_n = self.vel_pid_x.update_full(vel_target_n, current_vel.x, dt, false, i_scale_x);
        let pid_accel_e = self.vel_pid_y.update_full(vel_target_e, current_vel.y, dt, false, i_scale_y);

        // Blend shaped acceleration with PID correction
        let final_accel_n = accel_n + pid_accel_n;
        let final_accel_e = accel_e + pid_accel_e;

        // Cap total acceleration magnitude
        let final_mag = libm::sqrtf(final_accel_n * final_accel_n + final_accel_e * final_accel_e);
        let (accel_out_n, accel_out_e) = if final_mag > self.accel_max_xy && final_mag > 0.001 {
            let s = self.accel_max_xy / final_mag;
            (final_accel_n * s, final_accel_e * s)
        } else {
            (final_accel_n, final_accel_e)
        };

        self.prev_accel = Vec3::new(accel_out_n, accel_out_e, 0.0);

        // ===== VERTICAL POSITION -> VELOCITY -> THROTTLE =====

        let pos_err_d = target_pos.z - current_pos.z;
        let vel_target_d_raw = pos_err_d * self.gains.pos_z_p + target_vel.z;
        let vel_target_d = vel_target_d_raw.clamp(-5.0, 1.5); // up=neg (5 m/s climb), down=pos (1.5 m/s descent)

        let accel_d = self.vel_pid_z.update(vel_target_d, current_vel.z, dt);

        // Use adaptive hover throttle
        let hover_thr = self.hover_estimator.throttle_hover;
        let throttle = (hover_thr * (GRAVITY - accel_d) / GRAVITY).clamp(0.0, 1.0);

        // ===== HORIZONTAL ACCELERATION -> LEAN ANGLES =====

        // Rotate to body frame using yaw
        let cos_yaw = libm::cosf(current_yaw);
        let sin_yaw = libm::sinf(current_yaw);
        let accel_fwd = accel_out_n * cos_yaw + accel_out_e * sin_yaw;   // body X
        let accel_right = -accel_out_n * sin_yaw + accel_out_e * cos_yaw; // body Y

        // Use throttle-adjusted thrust for lean angle calculation (not just gravity)
        // Source: AP lean_angles_to_accel_xy uses thrust-normalized conversion
        let thrust_ref = (throttle * GRAVITY / hover_thr).max(GRAVITY * 0.5);
        let pitch_target = libm::atan2f(-accel_fwd, thrust_ref)
            .clamp(-self.gains.angle_max, self.gains.angle_max);
        let roll_target = libm::atan2f(accel_right, thrust_ref)
            .clamp(-self.gains.angle_max, self.gains.angle_max);

        // Build target quaternion from euler angles
        let target_quat = Quaternion::from_euler(roll_target, pitch_target, current_yaw);

        PosControlOutput { target_quat, throttle }
    }

    /// Altitude-only control (for AltHold mode).
    ///
    /// Returns throttle command (0..1).
    pub fn update_altitude(
        &mut self,
        target_alt: f32,     // target altitude (positive up, meters)
        current_alt: f32,    // current altitude (positive up)
        current_vel_d: f32,  // current vertical velocity (NED down, m/s)
        dt: f32,
    ) -> f32 {
        // Convert to NED down
        let target_d = -target_alt;
        let current_d = -current_alt;

        let pos_err = target_d - current_d;
        let vel_target = (pos_err * self.gains.pos_z_p).clamp(-5.0, 1.5);
        let accel_d = self.vel_pid_z.update(vel_target, current_vel_d, dt);
        let hover_thr = self.hover_estimator.throttle_hover;
        (hover_thr * (GRAVITY - accel_d) / GRAVITY).clamp(0.0, 1.0)
    }

    /// Update hover throttle estimate from actual throttle.
    /// Call each loop iteration.
    pub fn update_hover_throttle(&mut self, actual_throttle: f32, vertical_accel: f32, dt: f32) {
        self.hover_estimator.update(actual_throttle, vertical_accel, dt);
        self.gains.hover_throttle = self.hover_estimator.throttle_hover;
    }

    pub fn reset(&mut self) {
        self.vel_pid_x.reset();
        self.vel_pid_y.reset();
        self.vel_pid_z.reset();
        self.vel_desired = Vec3::zero();
        self.accel_desired = Vec3::zero();
        self.prev_accel = Vec3::zero();
        self.limits = SaturationLimits::default();
        self.prev_accel_heading = 0.0;
    }
}

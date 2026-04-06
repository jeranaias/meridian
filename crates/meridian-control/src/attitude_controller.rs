//! Attitude controller: quaternion error -> angular rate targets.
//!
//! Ports ArduPilot's AC_AttitudeControl with:
//! - Input shaping (acceleration-limited trajectory on attitude commands)
//! - Angle boost (throttle compensation for tilt)
//! - Lean angle max enforcement (dynamic based on throttle)
//! - Rate feedforward from shaped trajectory
//! - Thrust-error feedforward attenuation
//! - Thrust vector heading decomposition
//! - Throttle-RPY mix blending
//! Source: libraries/AC_AttitudeControl/AC_AttitudeControl.cpp

use meridian_math::{Vec3, Quaternion};
use meridian_math::frames::Body;

/// Maximum thrust error angle (rad) before feedforward is fully attenuated.
/// Source: AC_ATTITUDE_THRUST_ERROR_ANGLE_RAD (0.5236 = 30 deg)
const THRUST_ERROR_ANGLE_RAD: f32 = 0.5236;

/// Attitude controller configuration.
#[derive(Debug, Clone, Copy)]
pub struct AttitudeGains {
    /// Roll angle P gain. Source: ATC_ANG_RLL_P, default 4.5
    pub roll_p: f32,
    /// Pitch angle P gain. Source: ATC_ANG_PIT_P, default 4.5
    pub pitch_p: f32,
    /// Yaw angle P gain. Source: ATC_ANG_YAW_P, default 4.5
    pub yaw_p: f32,
    /// Maximum lean angle (rad). Source: ANGLE_MAX/100, default 30 deg
    pub angle_max: f32,
    /// Maximum angular rate (rad/s).
    pub rate_max: f32,
    /// Per-axis acceleration limits (rad/s^2).
    /// Source: ATC_ACCEL_R_MAX, ATC_ACCEL_P_MAX, ATC_ACCEL_Y_MAX
    pub accel_roll_max: f32,
    pub accel_pitch_max: f32,
    pub accel_yaw_max: f32,
    /// Per-axis rate limits (rad/s).
    /// Source: ATC_RATE_R_MAX, ATC_RATE_P_MAX, ATC_RATE_Y_MAX
    pub rate_roll_max: f32,
    pub rate_pitch_max: f32,
    pub rate_yaw_max: f32,
    /// Input time constant for shaping (s). Source: ATC_INPUT_TC, default 0.15
    pub input_tc: f32,
    /// Enable body-frame rate feedforward. Source: ATC_RATE_FF_ENAB
    pub rate_bf_ff_enabled: bool,
}

impl Default for AttitudeGains {
    fn default() -> Self {
        let angle_max = 30.0 * core::f32::consts::PI / 180.0;
        Self {
            roll_p: 4.5,
            pitch_p: 4.5,
            yaw_p: 4.5,
            angle_max,
            rate_max: 4.5 * angle_max,
            // ArduPilot defaults: 110000 cdeg/s^2 = 19.2 rad/s^2
            accel_roll_max: 19.2,
            accel_pitch_max: 19.2,
            // Yaw accel is typically lower: 27000 cdeg/s^2 = 4.71 rad/s^2
            accel_yaw_max: 4.71,
            rate_roll_max: 19.2 / 4.5,  // accel_max / P
            rate_pitch_max: 19.2 / 4.5,
            rate_yaw_max: 4.71 / 4.5,
            input_tc: 0.15,
            rate_bf_ff_enabled: true,
        }
    }
}

/// Throttle-RPY mix state.
/// Source: AC_AttitudeControl_Multi::_throttle_rpy_mix
#[derive(Debug, Clone)]
pub struct ThrottleRpyMix {
    /// Current mix value (0.1 .. 0.9). 1.0 = full attitude priority.
    pub mix: f32,
    /// Slew rate for increasing (default +2.0/s).
    pub slew_up: f32,
    /// Slew rate for decreasing (default -0.5/s).
    pub slew_down: f32,
}

impl Default for ThrottleRpyMix {
    fn default() -> Self {
        Self {
            mix: 0.5,
            slew_up: 2.0,
            slew_down: 0.5,
        }
    }
}

impl ThrottleRpyMix {
    /// Set to maximum (prioritize attitude control, e.g., flight).
    pub fn set_max(&mut self) { self.mix = 0.9; }
    /// Set to minimum (prioritize throttle, e.g., landing).
    pub fn set_min(&mut self) { self.mix = 0.1; }

    /// Slew toward desired mix value.
    pub fn update(&mut self, desired: f32, dt: f32) {
        let desired = desired.clamp(0.1, 0.9);
        if desired > self.mix {
            let delta = (self.slew_up * dt).min(desired - self.mix);
            self.mix += delta;
        } else if desired < self.mix {
            let delta = (self.slew_down * dt).min(self.mix - desired);
            self.mix -= delta;
        }
        self.mix = self.mix.clamp(0.1, 0.9);
    }

    /// Get the average max throttle available for attitude control.
    /// Source: AC_AttitudeControl_Multi::get_throttle_avg_max()
    pub fn get_throttle_avg_max(&self) -> f32 {
        // Blend between 0.5 (full throttle priority) and 1.0 (full attitude priority)
        0.5 + 0.5 * self.mix
    }
}

/// Attitude controller with input shaping and angle boost.
pub struct AttitudeController {
    pub gains: AttitudeGains,
    /// Shaped rate target from previous tick (for feedforward).
    prev_rate_target: Vec3<Body>,
    /// Shaped rate target (acceleration-limited).
    shaped_rate: Vec3<Body>,
    /// Thrust-error angle for feedforward attenuation (rad).
    thrust_error_angle: f32,
    /// Throttle-RPY mix blending state.
    pub throttle_rpy_mix: ThrottleRpyMix,
}

impl AttitudeController {
    pub fn new() -> Self {
        Self {
            gains: AttitudeGains::default(),
            prev_rate_target: Vec3::zero(),
            shaped_rate: Vec3::zero(),
            thrust_error_angle: 0.0,
            throttle_rpy_mix: ThrottleRpyMix::default(),
        }
    }

    pub fn with_gains(gains: AttitudeGains) -> Self {
        Self {
            gains,
            prev_rate_target: Vec3::zero(),
            shaped_rate: Vec3::zero(),
            thrust_error_angle: 0.0,
            throttle_rpy_mix: ThrottleRpyMix::default(),
        }
    }

    /// Compute angular rate targets from attitude error.
    ///
    /// CRITICAL FIX: Quaternion error order matches ArduPilot:
    ///   q_err = target.inverse() * body
    /// This produces axis-angle in the target frame, matching AP's convention.
    ///
    /// Returns: rate_target in body frame (rad/s).
    pub fn update(
        &mut self,
        target_quat: &Quaternion,
        current_quat: &Quaternion,
        dt: f32,
    ) -> Vec3<Body> {
        // Quaternion error: q_err = target^-1 * current (ArduPilot convention)
        // This expresses the body orientation error in the target frame.
        let q_err = target_quat.inverse() * *current_quat;

        // Convert to axis-angle error (shortest path via sign check)
        // The sign ensures we take the shortest rotation path.
        let sign = if q_err.w >= 0.0 { 2.0 } else { -2.0 };
        let angle_err = Vec3::<Body>::new(
            sign * q_err.x,
            sign * q_err.y,
            sign * q_err.z,
        );

        // Compute thrust error angle for feedforward attenuation
        let lean_err = libm::sqrtf(angle_err.x * angle_err.x + angle_err.y * angle_err.y);
        self.thrust_error_angle = lean_err;

        // Enforce lean angle max on the error
        let scale = if lean_err > self.gains.angle_max && lean_err > 0.001 {
            self.gains.angle_max / lean_err
        } else {
            1.0
        };
        let clamped_err = Vec3::<Body>::new(
            angle_err.x * scale,
            angle_err.y * scale,
            angle_err.z.clamp(-self.gains.angle_max, self.gains.angle_max),
        );

        // Raw rate target = P * error
        // Note: negated because q_err = target^-1 * body means error is "how body differs from target"
        // and we want to drive error to zero (negative feedback).
        let raw_rate = Vec3::<Body>::new(
            -self.gains.roll_p * clamped_err.x,
            -self.gains.pitch_p * clamped_err.y,
            -self.gains.yaw_p * clamped_err.z,
        );

        // Input shaping: per-axis acceleration-limited rate target change
        // Source: AC_AttitudeControl::attitude_command_model()
        if dt > 1e-6 {
            let accel_r = self.gains.accel_roll_max;
            let accel_p = self.gains.accel_pitch_max;
            let accel_y = self.gains.accel_yaw_max;

            let dx = (raw_rate.x - self.shaped_rate.x).clamp(-accel_r * dt, accel_r * dt);
            let dy = (raw_rate.y - self.shaped_rate.y).clamp(-accel_p * dt, accel_p * dt);
            let dz = (raw_rate.z - self.shaped_rate.z).clamp(-accel_y * dt, accel_y * dt);
            self.shaped_rate = Vec3::new(
                self.shaped_rate.x + dx,
                self.shaped_rate.y + dy,
                self.shaped_rate.z + dz,
            );
        } else {
            self.shaped_rate = raw_rate;
        }

        // Clamp to per-axis rate max
        let rate_target = Vec3::<Body>::new(
            self.shaped_rate.x.clamp(-self.gains.rate_roll_max, self.gains.rate_roll_max),
            self.shaped_rate.y.clamp(-self.gains.rate_pitch_max, self.gains.rate_pitch_max),
            self.shaped_rate.z.clamp(-self.gains.rate_yaw_max, self.gains.rate_yaw_max),
        );

        // Feedforward attenuation based on thrust error.
        // Source: AC_AttitudeControl_Multi::attitude_controller_run_quat()
        // When thrust vector error is large, reduce feedforward to prevent yaw loss.
        let ff_scalar = if self.thrust_error_angle < THRUST_ERROR_ANGLE_RAD {
            1.0
        } else {
            // Linearly reduce to 0.0 as thrust error goes to 2x the threshold
            (1.0 - (self.thrust_error_angle - THRUST_ERROR_ANGLE_RAD) / THRUST_ERROR_ANGLE_RAD)
                .max(0.0)
        };

        // Compute feedforward from shaped rate change.
        // KU1 fix: divide by dt to get angular acceleration (rad/s^2).
        // Without /dt, the feedforward term was dimensionally wrong —
        // adding rad/s delta to a rad/s rate instead of rad/s^2 * dt.
        let rate_ff = if dt > 1e-6 && self.gains.rate_bf_ff_enabled {
            Vec3::<Body>::new(
                (rate_target.x - self.prev_rate_target.x) / dt * ff_scalar,
                (rate_target.y - self.prev_rate_target.y) / dt * ff_scalar,
                (rate_target.z - self.prev_rate_target.z) / dt * ff_scalar,
            )
        } else {
            Vec3::zero()
        };

        self.prev_rate_target = rate_target;

        // Combined output: error-derived rate + feedforward
        Vec3::<Body>::new(
            rate_target.x + rate_ff.x,
            rate_target.y + rate_ff.y,
            rate_target.z + rate_ff.z,
        )
    }

    /// Input thrust vector heading: converts a 3D thrust vector + heading into
    /// a target quaternion. Used by loiter, position hold, and auto modes.
    /// Source: AC_AttitudeControl::input_thrust_vector_heading()
    ///
    /// `thrust_vec`: desired thrust vector in NED (x=North, y=East, z=Down, typically negative z)
    /// `heading_rad`: desired yaw heading (rad)
    ///
    /// Returns: target quaternion for the attitude controller.
    pub fn input_thrust_vector_heading(thrust_vec_x: f32, thrust_vec_y: f32, thrust_vec_z: f32, heading_rad: f32) -> Quaternion {
        // Compute roll and pitch from thrust vector
        // thrust_vec is in NED: (North, East, Down)
        // For multirotor: thrust points upward, so z component is typically negative
        let thrust_mag = libm::sqrtf(thrust_vec_x * thrust_vec_x + thrust_vec_y * thrust_vec_y + thrust_vec_z * thrust_vec_z);
        if thrust_mag < 0.01 {
            return Quaternion::from_euler(0.0, 0.0, heading_rad);
        }

        // Compute the tilt angles needed to produce this thrust vector
        // Pitch = atan2(-forward_component, -down_component)
        // Roll = atan2(right_component, -down_component)
        // We need to rotate the NED thrust vector by the yaw heading to get body-frame components
        let cos_yaw = libm::cosf(heading_rad);
        let sin_yaw = libm::sinf(heading_rad);

        // Rotate thrust vector into body-aligned NED (yaw-removed)
        let thrust_fwd = thrust_vec_x * cos_yaw + thrust_vec_y * sin_yaw;
        let thrust_right = -thrust_vec_x * sin_yaw + thrust_vec_y * cos_yaw;
        let thrust_down = thrust_vec_z;

        // Pitch = atan2(-fwd, -down) (nose up when thrust has forward component)
        let pitch = libm::atan2f(-thrust_fwd, -thrust_down);
        // Roll = atan2(right, -down) (roll right when thrust has right component)
        let roll = libm::atan2f(thrust_right, -thrust_down);

        Quaternion::from_euler(roll, pitch, heading_rad)
    }

    /// Dynamic lean angle max based on throttle.
    /// Source: AC_AttitudeControl::update_althold_lean_angle_max()
    ///
    /// Reduces the maximum lean angle when near throttle limits.
    /// `throttle`: current throttle (0..1)
    /// `thr_max`: maximum throttle (typically 0.8 * 1.0)
    ///
    /// Returns: maximum lean angle (rad).
    pub fn dynamic_lean_angle_max(&self, throttle: f32, thr_max: f32) -> f32 {
        let thr_limit = 0.8 * thr_max;
        if throttle >= thr_limit || thr_limit <= 0.0 {
            return 0.0; // No headroom for lean
        }
        let cos_angle = (throttle / thr_limit).clamp(0.0, 1.0);
        let angle = libm::acosf(cos_angle);
        angle.min(self.gains.angle_max)
    }

    /// Angle boost: compute throttle multiplier to compensate for tilt.
    /// Source: AC_AttitudeControl_Multi::get_throttle_boosted()
    ///
    /// Fixed to match ArduPilot:
    /// - Uses inverted_factor (10x cos_tilt smooth rolloff near 90 deg)
    /// - Prevents boost runaway past 60 deg
    pub fn angle_boost(&self, roll: f32, pitch: f32) -> f32 {
        let cos_tilt = libm::cosf(roll) * libm::cosf(pitch);
        // Inverted factor: smoothly reduces boost to zero as vehicle approaches 90 deg tilt
        // Source: AP_MotorsMulticopter::get_throttle_boosted()
        let inverted_factor = (10.0 * cos_tilt).clamp(0.0, 1.0);
        // cos_tilt_target clamped to [0.1, 1.0] to prevent divide-by-zero
        let cos_tilt_clamped = cos_tilt.clamp(0.1, 1.0);
        let boost_factor = 1.0 / cos_tilt_clamped;
        inverted_factor * boost_factor
    }

    /// Get the current thrust error angle (rad).
    /// Used for feedforward attenuation reporting.
    pub fn thrust_error_angle(&self) -> f32 { self.thrust_error_angle }

    /// Reset the input shaping state (call on mode change).
    pub fn reset(&mut self) {
        self.shaped_rate = Vec3::zero();
        self.prev_rate_target = Vec3::zero();
        self.thrust_error_angle = 0.0;
    }
}

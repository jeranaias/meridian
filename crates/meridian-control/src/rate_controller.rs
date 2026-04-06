//! Rate controller: 3-axis angular rate PID.
//!
//! Input: desired rates (rad/s) from attitude controller + measured rates from gyro.
//! Output: axis commands (-1 to 1) for the mixer.
//!
//! Source: libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp

use meridian_math::Vec3;
use meridian_math::frames::Body;
use crate::pid::{PidController, PidGains};

/// Default rate PID gains for a quad. Source: AC_AttitudeControl_Multi.h
/// ArduPilot: ATC_RAT_RLL_FLTT = 20 Hz
pub fn default_rate_roll() -> PidGains {
    PidGains {
        kp: 0.135, ki: 0.135, kd: 0.0036, ff: 0.0, kd_ff: 0.0,
        imax: 0.5, filt_hz: 20.0, filt_t_hz: 20.0, filt_e_hz: 0.0,
        smax: 0.0, pd_max: 0.0, sr_tau: 1.0,
    }
}

pub fn default_rate_pitch() -> PidGains {
    PidGains {
        kp: 0.135, ki: 0.135, kd: 0.0036, ff: 0.0, kd_ff: 0.0,
        imax: 0.5, filt_hz: 20.0, filt_t_hz: 20.0, filt_e_hz: 0.0,
        smax: 0.0, pd_max: 0.0, sr_tau: 1.0,
    }
}

/// ArduPilot: ATC_RAT_YAW_FLTE = 2.5 Hz (not 20!)
pub fn default_rate_yaw() -> PidGains {
    PidGains {
        kp: 0.180, ki: 0.018, kd: 0.0, ff: 0.0, kd_ff: 0.0,
        imax: 0.5, filt_hz: 20.0, filt_t_hz: 20.0, filt_e_hz: 2.5,
        smax: 0.0, pd_max: 0.0, sr_tau: 1.0,
    }
}

/// Throttle gain boost configuration.
/// Source: AC_AttitudeControl_Multi::update_throttle_gain_boost()
const THR_G_BOOST_THRESH: f32 = 0.5; // throttle slew rate threshold (per second)
const THR_G_BOOST_P_MAX: f32 = 2.0;  // max P/D boost factor
const THR_G_BOOST_ANG_MAX: f32 = 4.0; // max angle P boost factor

/// Landed gain multiplier configuration.
/// Source: ATC_LAND_R_MULT, ATC_LAND_P_MULT, ATC_LAND_Y_MULT
#[derive(Debug, Clone, Copy)]
pub struct LandedGainMultipliers {
    pub roll: f32,   // default 0.5
    pub pitch: f32,  // default 0.5
    pub yaw: f32,    // default 0.5
}

impl Default for LandedGainMultipliers {
    fn default() -> Self {
        Self { roll: 1.0, pitch: 1.0, yaw: 1.0 } // ArduPilot default: no reduction (1.0)
    }
}

/// Three-axis rate controller.
pub struct RateController {
    pub roll: PidController,
    pub pitch: PidController,
    pub yaw: PidController,
    pub landed_multipliers: LandedGainMultipliers,
    /// Throttle gain boost state.
    prev_throttle: f32,
    throttle_boost_scale: f32,
}

impl RateController {
    pub fn new() -> Self {
        let mut rc = Self {
            roll: PidController::new(default_rate_roll()),
            pitch: PidController::new(default_rate_pitch()),
            yaw: PidController::new(default_rate_yaw()),
            landed_multipliers: LandedGainMultipliers::default(),
            prev_throttle: 0.0,
            throttle_boost_scale: 1.0,
        };
        // Activate 2nd-order Butterworth on D-term for all axes (Hall Round 2 fix)
        use crate::notch_filter::LowPassFilter2p;
        let sample_rate = 400.0; // 400Hz main loop
        let mut lpf_r = LowPassFilter2p::new(); lpf_r.init(rc.roll.gains.filt_hz, sample_rate);
        let mut lpf_p = LowPassFilter2p::new(); lpf_p.init(rc.pitch.gains.filt_hz, sample_rate);
        let mut lpf_y = LowPassFilter2p::new(); lpf_y.init(rc.yaw.gains.filt_hz, sample_rate);
        rc.roll.set_d_lpf2p(lpf_r);
        rc.pitch.set_d_lpf2p(lpf_p);
        rc.yaw.set_d_lpf2p(lpf_y);
        rc
    }

    pub fn with_gains(roll: PidGains, pitch: PidGains, yaw: PidGains) -> Self {
        Self {
            roll: PidController::new(roll),
            pitch: PidController::new(pitch),
            yaw: PidController::new(yaw),
            landed_multipliers: LandedGainMultipliers::default(),
            prev_throttle: 0.0,
            throttle_boost_scale: 1.0,
        }
    }

    /// Compute axis commands from rate targets and measured rates.
    ///
    /// `target`: desired angular rates (rad/s) in body frame
    /// `measured`: measured angular rates from gyro (rad/s) in body frame
    /// `dt`: timestep (seconds)
    /// `limit_roll`: true if mixer reports roll saturation
    /// `limit_pitch`: true if mixer reports pitch saturation
    /// `limit_yaw`: true if mixer reports yaw saturation
    ///
    /// Returns: (roll_cmd, pitch_cmd, yaw_cmd) each in [-1, 1]
    pub fn update(
        &mut self,
        target: &Vec3<Body>,
        measured: &Vec3<Body>,
        dt: f32,
        limit_roll: bool,
        limit_pitch: bool,
        limit_yaw: bool,
    ) -> (f32, f32, f32) {
        let roll = self.roll.update_full(target.x, measured.x, dt, limit_roll, 1.0).clamp(-1.0, 1.0);
        let pitch = self.pitch.update_full(target.y, measured.y, dt, limit_pitch, 1.0).clamp(-1.0, 1.0);
        let yaw = self.yaw.update_full(target.z, measured.z, dt, limit_yaw, 1.0).clamp(-1.0, 1.0);
        (roll, pitch, yaw)
    }

    /// Simplified update without limit flags (backwards compatible).
    pub fn update_simple(
        &mut self,
        target: &Vec3<Body>,
        measured: &Vec3<Body>,
        dt: f32,
    ) -> (f32, f32, f32) {
        self.update(target, measured, dt, false, false, false)
    }

    /// Apply landed gain multipliers. Call this before update when landed.
    /// Source: AC_AttitudeControl_Multi landed gain multipliers LAND_R/P/Y_MULT.
    /// Temporarily scales the PID gains for ground stability.
    pub fn apply_landed_gains(&mut self, is_landed: bool) {
        if is_landed {
            // Scale P and D gains down when landed to prevent ground oscillation
            // We do this by modifying the gains temporarily; caller should restore after
            self.roll.gains.kp *= self.landed_multipliers.roll;
            self.roll.gains.kd *= self.landed_multipliers.roll;
            self.pitch.gains.kp *= self.landed_multipliers.pitch;
            self.pitch.gains.kd *= self.landed_multipliers.pitch;
            self.yaw.gains.kp *= self.landed_multipliers.yaw;
            self.yaw.gains.kd *= self.landed_multipliers.yaw;
        }
    }

    /// Restore gains after landed multiplier was applied.
    pub fn restore_landed_gains(&mut self, is_landed: bool) {
        if is_landed {
            let lm = &self.landed_multipliers;
            if lm.roll > 0.0 {
                self.roll.gains.kp /= lm.roll;
                self.roll.gains.kd /= lm.roll;
            }
            if lm.pitch > 0.0 {
                self.pitch.gains.kp /= lm.pitch;
                self.pitch.gains.kd /= lm.pitch;
            }
            if lm.yaw > 0.0 {
                self.yaw.gains.kp /= lm.yaw;
                self.yaw.gains.kd /= lm.yaw;
            }
        }
    }

    /// Update throttle gain boost factor.
    /// During rapid throttle slew, temporarily boosts PD gains.
    /// Source: AC_AttitudeControl_Multi::update_throttle_gain_boost()
    ///
    /// `throttle`: current throttle (0..1)
    /// `dt`: timestep
    ///
    /// Returns: (rate_pd_boost, angle_p_boost) multipliers.
    pub fn update_throttle_gain_boost(&mut self, throttle: f32, dt: f32) -> (f32, f32) {
        if dt <= 0.0 { return (1.0, 1.0); }
        let slew_rate = libm::fabsf(throttle - self.prev_throttle) / dt;
        self.prev_throttle = throttle;

        // Boost when slew rate exceeds threshold
        let rate_pd_boost;
        let angle_p_boost;
        if slew_rate > THR_G_BOOST_THRESH {
            let boost = (slew_rate / THR_G_BOOST_THRESH).min(THR_G_BOOST_P_MAX);
            rate_pd_boost = boost;
            angle_p_boost = (boost * 2.0).min(THR_G_BOOST_ANG_MAX);
        } else {
            rate_pd_boost = 1.0;
            angle_p_boost = 1.0;
        }
        self.throttle_boost_scale = rate_pd_boost;
        (rate_pd_boost, angle_p_boost)
    }

    pub fn reset(&mut self) {
        self.roll.reset();
        self.pitch.reset();
        self.yaw.reset();
        self.prev_throttle = 0.0;
        self.throttle_boost_scale = 1.0;
    }
}

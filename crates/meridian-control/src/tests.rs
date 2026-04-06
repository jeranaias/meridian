//! Unit tests for the control system -- PID, rate, attitude, position controllers.

#[cfg(test)]
mod pid_tests {
    use crate::pid::{PidController, PidGains};

    fn basic_gains() -> PidGains {
        PidGains {
            kp: 1.0, ki: 0.5, kd: 0.1, ff: 0.0, kd_ff: 0.0,
            imax: 1.0, filt_hz: 20.0, filt_t_hz: 0.0, filt_e_hz: 0.0,
            smax: 0.0, pd_max: 0.0, sr_tau: 1.0,
        }
    }

    #[test]
    fn test_p_only() {
        let gains = PidGains {
            kp: 2.0, ki: 0.0, kd: 0.0, ff: 0.0, kd_ff: 0.0,
            imax: 1.0, filt_hz: 0.0, filt_t_hz: 0.0, filt_e_hz: 0.0,
            smax: 0.0, pd_max: 0.0, sr_tau: 1.0,
        };
        let mut pid = PidController::new(gains);
        let out = pid.update(10.0, 8.0, 0.01); // error = 2.0
        assert!((out - 4.0).abs() < 0.01, "P-only: expected 4.0, got {}", out);
    }

    #[test]
    fn test_pi_steady_state() {
        let mut pid = PidController::new(basic_gains());
        // Drive with constant error for many steps -- integrator should accumulate
        for _ in 0..100 {
            pid.update(1.0, 0.0, 0.01); // error = 1.0
        }
        let out = pid.update(1.0, 0.0, 0.01);
        // Should have P (1.0) + I (accumulated) + D (small)
        assert!(out > 1.0, "PI should exceed P-only: {}", out);
    }

    #[test]
    fn test_integrator_clamp() {
        let gains = PidGains {
            kp: 0.0, ki: 100.0, kd: 0.0, ff: 0.0, kd_ff: 0.0,
            imax: 0.5, filt_hz: 0.0, filt_t_hz: 0.0, filt_e_hz: 0.0,
            smax: 0.0, pd_max: 0.0, sr_tau: 1.0,
        };
        let mut pid = PidController::new(gains);
        for _ in 0..1000 {
            pid.update(1.0, 0.0, 0.01);
        }
        let out = pid.update(1.0, 0.0, 0.01);
        assert!(out <= 0.5 + 0.01, "Integrator should be clamped to imax: {}", out);
    }

    #[test]
    fn test_zero_dt() {
        let mut pid = PidController::new(basic_gains());
        let out = pid.update(10.0, 0.0, 0.0);
        assert_eq!(out, 0.0, "Zero dt should return 0");
    }

    #[test]
    fn test_negative_dt() {
        let mut pid = PidController::new(basic_gains());
        let out = pid.update(10.0, 0.0, -0.01);
        assert_eq!(out, 0.0, "Negative dt should return 0");
    }

    #[test]
    fn test_reset() {
        let mut pid = PidController::new(basic_gains());
        for _ in 0..50 { pid.update(1.0, 0.0, 0.01); }
        pid.reset();
        let out = pid.update(1.0, 0.0, 0.01);
        // After reset, only P term contributes (no accumulated I)
        assert!((out - 1.0).abs() < 0.2, "After reset, output should be near P-only: {}", out);
    }

    #[test]
    fn test_feedforward() {
        let gains = PidGains {
            kp: 0.0, ki: 0.0, kd: 0.0, ff: 2.0, kd_ff: 0.0,
            imax: 1.0, filt_hz: 0.0, filt_t_hz: 0.0, filt_e_hz: 0.0,
            smax: 0.0, pd_max: 0.0, sr_tau: 1.0,
        };
        let mut pid = PidController::new(gains);
        let out = pid.update(5.0, 5.0, 0.01); // zero error, but ff on target
        assert!((out - 10.0).abs() < 0.01, "FF should be 2.0 * 5.0 = 10.0: {}", out);
    }

    #[test]
    fn test_integrator_limit_flag() {
        // When limit=true, integrator should not grow (but can shrink)
        let gains = PidGains {
            kp: 0.0, ki: 1.0, kd: 0.0, ff: 0.0, kd_ff: 0.0,
            imax: 10.0, filt_hz: 0.0, filt_t_hz: 0.0, filt_e_hz: 0.0,
            smax: 0.0, pd_max: 0.0, sr_tau: 1.0,
        };
        let mut pid = PidController::new(gains);

        // Build up integrator without limit
        for _ in 0..100 {
            pid.update_full(1.0, 0.0, 0.01, false, 1.0);
        }
        let i_before = pid.get_integrator();
        assert!(i_before > 0.5, "Should have accumulated positive integrator: {}", i_before);

        // Now with limit=true and positive error, integrator should NOT grow
        pid.update_full(1.0, 0.0, 0.01, true, 1.0);
        let i_after_limited = pid.get_integrator();
        assert!((i_after_limited - i_before).abs() < 0.001,
            "Integrator should not grow when limited: before={} after={}", i_before, i_after_limited);

        // With limit=true and negative error, integrator SHOULD shrink (decay toward zero)
        pid.update_full(0.0, 1.0, 0.01, true, 1.0); // negative error
        let i_after_decay = pid.get_integrator();
        assert!(i_after_decay < i_before,
            "Integrator should decay when limited with opposite error: before={} after={}", i_before, i_after_decay);
    }

    #[test]
    fn test_pd_max() {
        let gains = PidGains {
            kp: 10.0, ki: 0.0, kd: 0.0, ff: 0.0, kd_ff: 0.0,
            imax: 1.0, filt_hz: 0.0, filt_t_hz: 0.0, filt_e_hz: 0.0,
            smax: 0.0, pd_max: 5.0, sr_tau: 1.0,
        };
        let mut pid = PidController::new(gains);
        let out = pid.update(10.0, 0.0, 0.01); // P = 10*10 = 100, but PDMX=5
        assert!(libm::fabsf(out) <= 5.1, "PDMX should limit PD sum to 5.0: {}", out);
    }

    #[test]
    fn test_get_pid_info() {
        let mut pid = PidController::new(basic_gains());
        pid.update(1.0, 0.5, 0.01);
        let info = pid.get_pid_info();
        assert_eq!(info.target, 1.0);
        assert_eq!(info.actual, 0.5);
        assert!(info.p != 0.0, "P term should be nonzero");
    }

    #[test]
    fn test_relax_integrator() {
        let mut pid = PidController::new(basic_gains());
        pid.set_integrator(0.5);
        // Relax toward 0.0 with small time constant
        for _ in 0..100 {
            pid.relax_integrator(0.0, 0.01, 0.1);
        }
        assert!(libm::fabsf(pid.get_integrator()) < 0.05,
            "Integrator should relax toward 0: {}", pid.get_integrator());
    }

    #[test]
    fn test_d_ff() {
        let gains = PidGains {
            kp: 0.0, ki: 0.0, kd: 0.0, ff: 0.0, kd_ff: 1.0,
            imax: 1.0, filt_hz: 0.0, filt_t_hz: 0.0, filt_e_hz: 0.0,
            smax: 0.0, pd_max: 0.0, sr_tau: 1.0,
        };
        let mut pid = PidController::new(gains);
        // First call initializes, no D_FF yet
        pid.update(0.0, 0.0, 0.01);
        // Second call with changing target should produce D_FF
        let out = pid.update(1.0, 1.0, 0.01); // error=0, but target changed
        // D_FF = kd_ff * (1.0 - 0.0) / 0.01 = 100.0
        assert!(out > 10.0, "D_FF should produce output when target changes: {}", out);
    }

    #[test]
    fn test_smax_adaptive_limiter() {
        let gains = PidGains {
            kp: 10.0, ki: 0.0, kd: 0.0, ff: 0.0, kd_ff: 0.0,
            imax: 1.0, filt_hz: 0.0, filt_t_hz: 0.0, filt_e_hz: 0.0,
            smax: 50.0, pd_max: 0.0, sr_tau: 1.0,
        };
        let mut pid = PidController::new(gains);
        // Oscillate the error to trigger slew limiter
        for i in 0..100 {
            let target = if i % 2 == 0 { 5.0 } else { -5.0 };
            pid.update(target, 0.0, 0.0025);
        }
        let info = pid.get_pid_info();
        // After oscillation, d_mod should be reduced from 1.0
        assert!(info.d_mod < 1.0,
            "SMAX should reduce d_mod during oscillation: {}", info.d_mod);
        assert!(info.d_mod >= 0.1,
            "SMAX d_mod should have 0.1 floor: {}", info.d_mod);
    }
}

#[cfg(test)]
mod attitude_tests {
    use crate::attitude_controller::AttitudeController;
    use meridian_math::Quaternion;
    use meridian_math::Vec3;
    use meridian_math::frames::Body;

    #[test]
    fn test_zero_error() {
        let mut ctrl = AttitudeController::new();
        let q = Quaternion::from_euler(0.1, 0.05, 0.3);
        let rate = ctrl.update(&q, &q, 0.0025);
        assert!(rate.length() < 0.01, "Zero error should produce zero rate: {}", rate.length());
    }

    #[test]
    fn test_small_error() {
        let mut ctrl = AttitudeController::new();
        let target = Quaternion::from_euler(0.1, 0.0, 0.0);
        let current = Quaternion::identity();
        // Run a few ticks to let input shaping engage
        let mut rate = Vec3::<Body>::zero();
        for _ in 0..10 {
            rate = ctrl.update(&target, &current, 0.0025);
        }
        // Should command some rate to correct error
        assert!(rate.length() > 0.01, "Should command rate for error: {}", rate.length());
    }

    #[test]
    fn test_180_degree_error() {
        let mut ctrl = AttitudeController::new();
        let target = Quaternion::from_euler(core::f32::consts::PI, 0.0, 0.0); // 180 deg roll
        let current = Quaternion::identity();
        // Run multiple ticks so input shaping ramps up
        let mut rate = Vec3::<Body>::zero();
        for _ in 0..100 {
            rate = ctrl.update(&target, &current, 0.0025);
        }
        // After shaping ramp-up, should produce significant rate, not NaN
        assert!(!rate.is_nan(), "180 deg error should not produce NaN");
        assert!(rate.length() > 0.1, "180 deg error should produce significant rate: {}", rate.length());
    }

    #[test]
    fn test_gimbal_lock_pitch_90() {
        let mut ctrl = AttitudeController::new();
        let target = Quaternion::from_euler(0.0, 1.5, 0.0); // ~86 deg pitch
        let current = Quaternion::identity();
        let rate = ctrl.update(&target, &current, 0.0025);
        assert!(!rate.is_nan(), "Near-gimbal-lock should not produce NaN");
    }

    #[test]
    fn test_angle_boost_hover() {
        let ctrl = AttitudeController::new();
        let boost = ctrl.angle_boost(0.0, 0.0);
        assert!((boost - 1.0).abs() < 0.01, "Hover boost should be 1.0: {}", boost);
    }

    #[test]
    fn test_angle_boost_rolloff_near_90() {
        let ctrl = AttitudeController::new();
        let boost = ctrl.angle_boost(1.48, 0.0); // ~85 deg
        let raw = 1.0 / libm::cosf(1.48);
        assert!(boost < raw, "Boost with rolloff ({}) should be less than raw ({})", boost, raw);
    }

    #[test]
    fn test_input_thrust_vector_heading() {
        let q = AttitudeController::input_thrust_vector_heading(0.0, 0.0, -9.8, 0.0);
        let (r, p, _y) = q.to_euler();
        assert!(libm::fabsf(r) < 0.1, "Upward thrust should be level roll: {}", r);
        assert!(libm::fabsf(p) < 0.1, "Upward thrust should be level pitch: {}", p);
    }

    #[test]
    fn test_throttle_rpy_mix() {
        use crate::attitude_controller::ThrottleRpyMix;
        let mut mix = ThrottleRpyMix::default();
        assert!((mix.mix - 0.5).abs() < 0.01);

        mix.set_max();
        assert!((mix.mix - 0.9).abs() < 0.01);

        mix.set_min();
        assert!((mix.mix - 0.1).abs() < 0.01);

        for _ in 0..100 {
            mix.update(0.5, 0.01);
        }
        assert!((mix.mix - 0.5).abs() < 0.1, "Should slew toward 0.5: {}", mix.mix);
    }
}

#[cfg(test)]
mod rate_tests {
    use crate::rate_controller::{RateController, LandedGainMultipliers, default_rate_yaw};

    #[test]
    fn test_yaw_filt_e_hz_default() {
        let gains = default_rate_yaw();
        assert!((gains.filt_e_hz - 2.5).abs() < 0.01,
            "Yaw filt_E_hz should be 2.5: {}", gains.filt_e_hz);
    }

    #[test]
    fn test_rate_controller_with_limits() {
        use meridian_math::Vec3;
        use meridian_math::frames::Body;

        let mut rc = RateController::new();
        let target = Vec3::<Body>::new(1.0, 0.0, 0.0);
        let measured = Vec3::<Body>::new(0.0, 0.0, 0.0);
        let (roll, _pitch, _yaw) = rc.update(&target, &measured, 0.0025, false, false, false);
        assert!(roll > 0.0, "Should produce positive roll command: {}", roll);
    }

    #[test]
    fn test_filt_t_hz_defaults() {
        use crate::rate_controller::{default_rate_roll, default_rate_pitch};
        let roll = default_rate_roll();
        let pitch = default_rate_pitch();
        assert!((roll.filt_t_hz - 20.0).abs() < 0.01,
            "Roll filt_T_hz should be 20.0: {}", roll.filt_t_hz);
        assert!((pitch.filt_t_hz - 20.0).abs() < 0.01,
            "Pitch filt_T_hz should be 20.0: {}", pitch.filt_t_hz);
    }

    #[test]
    fn test_landed_gain_multipliers() {
        let mut rc = RateController::new();
        let orig_kp = rc.roll.gains.kp;
        rc.landed_multipliers = LandedGainMultipliers { roll: 0.5, pitch: 0.5, yaw: 0.5 };
        rc.apply_landed_gains(true);
        assert!((rc.roll.gains.kp - orig_kp * 0.5).abs() < 0.001,
            "Landed gains should halve kp when multiplier is 0.5: {}", rc.roll.gains.kp);
        rc.restore_landed_gains(true);
        assert!((rc.roll.gains.kp - orig_kp).abs() < 0.001,
            "Restored gains should match original: {}", rc.roll.gains.kp);
    }
}

#[cfg(test)]
mod position_tests {
    use crate::position_controller::PositionController;
    use meridian_math::Vec3;
    use meridian_math::frames::NED;

    #[test]
    fn test_at_target() {
        let mut ctrl = PositionController::new();
        let pos = Vec3::<NED>::new(10.0, 5.0, -20.0);
        let out = ctrl.update(&pos, &Vec3::zero(), &pos, &Vec3::zero(), 0.0, 0.01);
        // At target: throttle should be near hover
        assert!((out.throttle - ctrl.gains.hover_throttle).abs() < 0.2,
            "At target, throttle should be near hover: {}", out.throttle);
    }

    #[test]
    fn test_target_below() {
        let mut ctrl = PositionController::new();
        let target = Vec3::<NED>::new(0.0, 0.0, -10.0); // 10m up
        let current = Vec3::<NED>::new(0.0, 0.0, -5.0);  // 5m up
        let out = ctrl.update(&target, &Vec3::zero(), &current, &Vec3::zero(), 0.0, 0.01);
        // Need to climb: throttle > hover
        assert!(out.throttle > ctrl.gains.hover_throttle,
            "Need to climb: throttle {} should exceed hover {}", out.throttle, ctrl.gains.hover_throttle);
    }

    #[test]
    fn test_auto_zero_waypoints() {
        let mut ctrl = PositionController::new();
        let current = Vec3::<NED>::new(100.0, 50.0, -20.0);
        let target = Vec3::<NED>::zero(); // NED origin
        let out = ctrl.update(&target, &Vec3::zero(), &current, &Vec3::zero(), 0.0, 0.01);
        // Should produce a lean toward target, not NaN
        assert!(!out.target_quat.is_nan(), "Zero target should not produce NaN");
        assert!(!out.throttle.is_nan());
    }

    #[test]
    fn test_hover_throttle_estimator() {
        use crate::position_controller::HoverThrottleEstimator;
        let mut est = HoverThrottleEstimator::default();
        assert!((est.throttle_hover - 0.39).abs() < 0.01);

        // Simulate hover at 0.45 throttle with no vertical accel
        for _ in 0..1000 {
            est.update(0.45, 0.0, 0.01);
        }
        assert!((est.throttle_hover - 0.45).abs() < 0.05,
            "Should adapt toward 0.45: {}", est.throttle_hover);
    }

    #[test]
    fn test_hover_throttle_no_update_when_accelerating() {
        use crate::position_controller::HoverThrottleEstimator;
        let mut est = HoverThrottleEstimator::default();
        let initial = est.throttle_hover;

        // Feed high acceleration - should NOT update
        for _ in 0..100 {
            est.update(0.8, 5.0, 0.01); // high vertical accel
        }
        assert!((est.throttle_hover - initial).abs() < 0.01,
            "Should not update during acceleration: {}", est.throttle_hover);
    }
}

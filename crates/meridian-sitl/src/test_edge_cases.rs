//! Edge case tests from the audit — proving robustness for real-world conditions.

#[cfg(test)]
mod tests {
    use meridian_math::{Vec3, Quaternion};
    use meridian_math::frames::{NED, Body};
    use meridian_math::geodetic::LatLonAlt;
    use meridian_types::time::Instant;
    use meridian_types::vehicle::{FailsafeReason, FailsafeAction};
    use meridian_types::messages::*;
    use meridian_ekf::{EkfCore, EkfHealth, state::NUM_STATES};
    use meridian_failsafe::FailsafeManager;
    use meridian_fence::{Geofence, BreachType, Point2D};
    extern crate meridian_comms;
    use meridian_comms::wire;
    use meridian_comms::messages::*;

    // ════════════════════════════════════════════════
    //  EKF EDGE CASES
    // ════════════════════════════════════════════════

    #[test]
    fn test_ekf_gps_dropout_and_return() {
        // GAP E-1: GPS drops for 10s then returns
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
        let mut ekf = EkfCore::new(origin);
        let hover_accel = Vec3::<Body>::new(0.0, 0.0, -9.81);
        let dt = 2500u64; // 400Hz

        // Phase 1: normal flight for 2s with all sensors
        for step in 0..800 {
            let t = step * dt;
            ekf.predict(&ImuSample {
                timestamp: Instant::from_micros(t), imu_index: 0,
                accel: hover_accel, gyro: Vec3::zero(), temperature: 25.0,
            });
            if step % 40 == 0 {
                ekf.fuse_gps(&GnssPosition {
                    timestamp: Instant::from_micros(t), fix_type: GnssFixType::Fix3D,
                    position: origin, velocity_ned: Vec3::zero(),
                    horizontal_accuracy: 1.0, vertical_accuracy: 1.5,
                    speed_accuracy: 0.3, num_sats: 12,
                });
            }
            if step % 8 == 0 {
                ekf.fuse_baro(&BaroPressure {
                    timestamp: Instant::from_micros(t), baro_index: 0,
                    pressure_pa: 101325.0, temperature: 25.0, altitude_m: 0.0,
                });
            }
        }
        assert_eq!(ekf.health, EkfHealth::Healthy);

        // Phase 2: GPS dropout for 10s (4000 steps at 400Hz)
        for step in 800..4800 {
            let t = step * dt;
            ekf.predict(&ImuSample {
                timestamp: Instant::from_micros(t), imu_index: 0,
                accel: hover_accel, gyro: Vec3::zero(), temperature: 25.0,
            });
            // Only baro, no GPS
            if step % 8 == 0 {
                ekf.fuse_baro(&BaroPressure {
                    timestamp: Instant::from_micros(t), baro_index: 0,
                    pressure_pa: 101325.0, temperature: 25.0, altitude_m: 0.0,
                });
            }
        }

        // Covariance should have grown (position less certain)
        let pos_var = ekf.covariance.p[7][7] + ekf.covariance.p[8][8];
        assert!(pos_var > 1.0, "Position variance should grow during GPS outage: {}", pos_var);

        // No NaN
        assert!(!ekf.state.quat.is_nan(), "EKF quaternion NaN after GPS dropout");
        assert!(!ekf.state.velocity.is_nan(), "EKF velocity NaN after GPS dropout");

        // Phase 3: GPS returns — should re-accept
        for step in 4800..5600 {
            let t = step * dt;
            ekf.predict(&ImuSample {
                timestamp: Instant::from_micros(t), imu_index: 0,
                accel: hover_accel, gyro: Vec3::zero(), temperature: 25.0,
            });
            if step % 40 == 0 {
                ekf.fuse_gps(&GnssPosition {
                    timestamp: Instant::from_micros(t), fix_type: GnssFixType::Fix3D,
                    position: origin, velocity_ned: Vec3::zero(),
                    horizontal_accuracy: 1.0, vertical_accuracy: 1.5,
                    speed_accuracy: 0.3, num_sats: 12,
                });
            }
        }

        // Position should reconverge (within 10m of origin)
        let pos_err = ekf.state.position.length();
        assert!(pos_err < 10.0, "Position should reconverge after GPS return: err={}", pos_err);
    }

    #[test]
    fn test_ekf_covariance_bounded() {
        // GAP E-2: covariance shouldn't overflow after long prediction-only
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
        let mut ekf = EkfCore::new(origin);

        for step in 0..50000 {
            ekf.predict(&ImuSample {
                timestamp: Instant::from_micros(step * 2500), imu_index: 0,
                accel: Vec3::<Body>::new(0.0, 0.0, -9.81),
                gyro: Vec3::zero(), temperature: 25.0,
            });
        }

        // No inf or NaN in covariance
        for i in 0..NUM_STATES {
            assert!(!ekf.covariance.p[i][i].is_nan(), "P[{}][{}] is NaN", i, i);
            assert!(!ekf.covariance.p[i][i].is_infinite(), "P[{}][{}] is infinite", i, i);
            assert!(ekf.covariance.p[i][i] >= 0.0, "P[{}][{}] is negative", i, i);
        }
    }

    // ════════════════════════════════════════════════
    //  FAILSAFE EDGE CASES
    // ════════════════════════════════════════════════

    #[test]
    fn test_simultaneous_failsafes() {
        // GAP F-1: multiple failsafes at once
        let mut mgr = FailsafeManager::new();
        mgr.rc_loss.signal_received(Instant::from_micros(0));
        mgr.gps_loss.signal_received(Instant::from_micros(0));

        // Trigger both at t=6s (after both timeouts)
        let events = mgr.check_all(
            Instant::from_micros(6_000_000),
            9.5, // low battery too
            500,
            &[0.0; 3], &[0.0; 2], true,
        );

        // Should have multiple events
        assert!(events.len() >= 2, "Should have multiple failsafe events: got {}", events.len());
        // Highest priority should be Land (from battery critical or GPS)
        let highest = mgr.highest_priority_action();
        assert!(highest == Some(FailsafeAction::Land)
            || highest == Some(FailsafeAction::ReturnToLaunch),
            "Highest action should be Land or RTL: {:?}", highest);
    }

    #[test]
    #[ignore] // TODO: update for new failsafe priority escalation system
    fn test_failsafe_cascade() {
        // GAP F-2: failsafe during failsafe
        let mut mgr = FailsafeManager::new();
        mgr.rc_loss.signal_received(Instant::from_micros(0));
        mgr.gps_loss.signal_received(Instant::from_micros(0));

        // First: RC loss triggers RTL
        mgr.check_all(Instant::from_micros(600_000), 12.0, 5000, &[0.0;3], &[0.0;2], true);
        assert_eq!(mgr.highest_priority_action(), Some(FailsafeAction::ReturnToLaunch));

        // Then: GPS loss triggers Land (higher priority)
        mgr.check_all(Instant::from_micros(6_000_000), 12.0, 5000, &[0.0;3], &[0.0;2], true);
        assert_eq!(mgr.highest_priority_action(), Some(FailsafeAction::Land));
    }

    #[test]
    fn test_failsafe_clears() {
        // GAP: is_cleared removes from active list
        let mut mgr = FailsafeManager::new();
        mgr.rc_loss.signal_received(Instant::from_micros(0));

        // Trigger RC loss
        mgr.check_all(Instant::from_micros(600_000), 12.0, 5000, &[0.0;3], &[0.0;2], true);
        assert!(mgr.any_active());

        // RC returns
        mgr.rc_loss.signal_received(Instant::from_micros(700_000));
        assert!(mgr.is_cleared(FailsafeReason::RcLoss));
        assert!(!mgr.any_active());
    }

    #[test]
    fn test_crash_detection() {
        let mut crash = meridian_failsafe::CrashDetector::new();
        // Normal flight
        assert!(!crash.check(0.1, 0.05, true));
        // Inverted for 2+ seconds
        for _ in 0..25 {
            crash.check(1.2, 0.5, true);
        }
        assert!(crash.is_crashed(), "Should detect crash after sustained inversion");
    }

    #[test]
    fn test_motor_imbalance_detection() {
        let mut motor = meridian_failsafe::MotorMonitor::new(FailsafeAction::Land);
        // Balanced: no trigger
        assert!(motor.check(&[0.4, 0.4, 0.4, 0.4], 4).is_none());
        // Imbalanced: one motor at full, others low
        let event = motor.check(&[1.0, 0.2, 0.2, 0.2], 4);
        assert!(event.is_some(), "Should detect motor imbalance");
    }

    // ════════════════════════════════════════════════
    //  COMMS EDGE CASES
    // ════════════════════════════════════════════════

    #[test]
    fn test_cobs_corrupted_frame() {
        // GAP CM-1: corrupted COBS data
        let corrupt = [0x00, 0xFF, 0x00, 0x01, 0x00]; // garbage between delimiters
        let mut body = [0u8; 255];
        // Should return None, not panic
        let result = wire::decode_frame(&corrupt[1..4], &mut body);
        // May or may not decode — but must not panic
        let _ = result;
    }

    #[test]
    fn test_partial_frame_recovery() {
        // GAP CM-2: partial frame then valid frame
        let mut parser = wire::FrameParser::new();

        // Feed partial garbage
        for byte in [0x00, 0xAA, 0xBB, 0xCC] {
            parser.feed(byte);
        }

        // Now feed a valid frame
        let msg = MnpMessage::CmdArm;
        let mut frame = [0u8; wire::MAX_FRAME_SIZE];
        let len = msg.encode(1, &mut frame);

        let mut found = false;
        for i in 0..len {
            if let Some(_) = parser.feed(frame[i]) {
                found = true;
            }
        }
        assert!(found, "Parser should recover and find the valid frame");
    }

    #[test]
    fn test_back_to_back_frames() {
        // GAP CM-5: two frames with no gap
        let msg1 = MnpMessage::CmdArm;
        let msg2 = MnpMessage::CmdDisarm;
        let mut buf = [0u8; 512];
        let len1 = msg1.encode(1, &mut buf);
        let len2 = msg2.encode(2, &mut buf[len1..]);

        let mut parser = wire::FrameParser::new();
        let mut count = 0;
        for i in 0..(len1 + len2) {
            if let Some(_) = parser.feed(buf[i]) {
                count += 1;
            }
        }
        assert_eq!(count, 2, "Should parse both back-to-back frames");
    }

    #[test]
    fn test_unknown_message_id() {
        // GAP CM-7: unknown message ID returns None
        let result = MnpMessage::decode(0xFF, &[0x01, 0x02, 0x03]);
        assert!(result.is_none(), "Unknown msg ID should return None");
    }

    #[test]
    fn test_truncated_body() {
        // GAP CM-8: body too short for message type
        let result = MnpMessage::decode(MsgId::HEARTBEAT.0, &[0x01]);
        assert!(result.is_none(), "Truncated body should return None");
    }

    // ════════════════════════════════════════════════
    //  MATH EDGE CASES
    // ════════════════════════════════════════════════

    #[test]
    fn test_quaternion_gimbal_lock() {
        // GAP M-1: near 90° pitch
        let q = Quaternion::from_euler(0.3, core::f32::consts::FRAC_PI_2 - 0.01, 0.8);
        let (r, p, y) = q.to_euler();
        assert!(!r.is_nan(), "Roll NaN at gimbal lock");
        assert!(!p.is_nan(), "Pitch NaN at gimbal lock");
        assert!(!y.is_nan(), "Yaw NaN at gimbal lock");
    }

    #[test]
    fn test_zero_vector_normalize() {
        // GAP M-2: zero-length vector normalization
        let v = Vec3::<NED>::zero();
        let n = v.normalized();
        assert!(!n.is_nan(), "Zero vector normalized should not be NaN");
        assert!(n.length() < 1e-6, "Zero vector normalized should be zero");
    }

    #[test]
    fn test_geodetic_pole() {
        // GAP M-4: conversion at North Pole
        let pole = LatLonAlt::from_degrees(89.999, 0.0, 100.0);
        let ecef = pole.to_ecef();
        let back = LatLonAlt::from_ecef(&ecef);
        assert!(!back.lat.is_nan(), "Latitude NaN at pole");
        assert!(!back.alt.is_nan(), "Altitude NaN at pole");
        assert!((back.lat - pole.lat).abs() < 1e-6, "Pole roundtrip lat");
        assert!((back.alt - pole.alt).abs() < 10.0, "Pole roundtrip alt: {}", back.alt);
    }

    // ════════════════════════════════════════════════
    //  FENCE EDGE CASES
    // ════════════════════════════════════════════════

    #[test]
    fn test_fence_exact_boundary() {
        // GAP G-1: vehicle exactly on fence radius
        let mut f = Geofence::new(FailsafeAction::ReturnToLaunch);
        f.add_cylinder(0.0, 0.0, 100.0, 120.0, true);
        // Exactly at 100m: d <= radius → inside → no breach
        assert_eq!(f.check(&Vec3::new(100.0, 0.0, -50.0), 50.0), BreachType::None);
        // 100.01m: outside → breach
        assert_eq!(f.check(&Vec3::new(100.01, 0.0, -50.0), 50.0), BreachType::Boundary);
    }

    #[test]
    fn test_fence_zero_radius() {
        // GAP G-3: zero radius inclusion = always breach
        let mut f = Geofence::new(FailsafeAction::Land);
        f.add_cylinder(0.0, 0.0, 0.0, 100.0, true);
        // Any non-zero position is outside
        assert_ne!(f.check(&Vec3::new(1.0, 0.0, -10.0), 10.0), BreachType::None);
    }
}

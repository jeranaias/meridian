//! Unit tests for precision landing.

#[cfg(test)]
mod kalman_tests {
    use crate::kalman::PosVelKF;

    #[test]
    fn test_kf_initial_state() {
        let kf = PosVelKF::new(0.1, 0.5);
        assert_eq!(kf.pos, 0.0);
        assert_eq!(kf.vel, 0.0);
    }

    #[test]
    fn test_kf_predict_constant_velocity() {
        let mut kf = PosVelKF::new(0.1, 0.5);
        kf.vel = 2.0;
        kf.predict(0.1);
        assert!((kf.pos - 0.2).abs() < 1e-4);
    }

    #[test]
    fn test_kf_update_converges() {
        let mut kf = PosVelKF::new(0.01, 0.1);
        let true_pos = 5.0;
        for _ in 0..50 {
            kf.predict(0.01);
            kf.update(true_pos, 0.01);
        }
        assert!((kf.pos - true_pos).abs() < 0.1);
    }

    #[test]
    fn test_kf_nis_outlier_rejection() {
        let mut kf = PosVelKF::new(0.001, 0.01);
        // Converge to 0.0
        for _ in 0..50 {
            kf.predict(0.01);
            kf.update(0.0, 0.001);
        }

        // Large outlier should be rejected
        kf.predict(0.01);
        let accepted = kf.update(1000.0, 0.001);
        assert!(!accepted, "Large outlier should be rejected");
        assert!(kf.pos.abs() < 1.0, "Position should not jump to outlier");
    }

    #[test]
    fn test_kf_nis_forced_accept_after_3() {
        let mut kf = PosVelKF::new(0.001, 0.01);
        for _ in 0..50 {
            kf.predict(0.01);
            kf.update(0.0, 0.001);
        }

        // 3 consecutive outliers should force accept on the 3rd
        for i in 0..3 {
            kf.predict(0.01);
            let accepted = kf.update(1000.0, 0.001);
            if i < 2 {
                assert!(!accepted, "Outlier {} should be rejected", i);
            } else {
                assert!(accepted, "3rd consecutive outlier should be force-accepted");
            }
        }
    }

    #[test]
    fn test_kf_reset() {
        let mut kf = PosVelKF::new(0.1, 0.5);
        kf.pos = 10.0;
        kf.vel = 5.0;
        kf.reset();
        assert_eq!(kf.pos, 0.0);
        assert_eq!(kf.vel, 0.0);
    }
}

#[cfg(test)]
mod inertial_history_tests {
    use crate::inertial_history::{InertialHistory, InertialSample};

    #[test]
    fn test_displacement_constant_velocity() {
        let mut hist: InertialHistory<16> = InertialHistory::new();
        for i in 0..11 {
            hist.push(InertialSample { vel_x: 1.0, vel_y: 0.0, timestamp_ms: i * 100 });
        }
        let (dx, _) = hist.compute_displacement(0, 1000);
        assert!((dx - 1.0).abs() < 0.05);
    }
}

#[cfg(test)]
mod precland_tests {
    use crate::precland::{PrecisionLand, TargetState};
    use crate::backend::PrecLandTarget;

    #[test]
    fn test_precland_initial_state() {
        let pl = PrecisionLand::new();
        assert_eq!(pl.target_state, TargetState::NeverFound);
        assert!(pl.get_target_position().is_none());
    }

    #[test]
    fn test_precland_state_machine() {
        let mut pl = PrecisionLand::new();

        let target = PrecLandTarget {
            angle_x: 0.0, angle_y: 0.0, distance_m: 0.0, timestamp_ms: 1000,
        };

        // First detection -> Acquiring
        pl.handle_target(&target, 10.0, 1000);
        assert_eq!(pl.target_state, TargetState::Acquiring);

        // 4 more -> Tracking
        for i in 1..5 {
            pl.handle_target(&target, 10.0, 1000 + i * 10);
        }
        assert_eq!(pl.target_state, TargetState::Tracking);
        assert!(pl.target_acquired());
    }

    #[test]
    fn test_precland_target_lost_and_retry() {
        let mut pl = PrecisionLand::new();
        pl.target_timeout_ms = 100;
        pl.retry_timeout_ms = 200;

        let target = PrecLandTarget {
            angle_x: 0.0, angle_y: 0.0, distance_m: 0.0, timestamp_ms: 1000,
        };

        // Acquire target
        for i in 0..5 {
            pl.handle_target(&target, 10.0, 1000 + i * 10);
        }
        assert_eq!(pl.target_state, TargetState::Tracking);

        // No more detections -> lost
        pl.predict(0.01, 1200);
        assert_eq!(pl.target_state, TargetState::RecentlyLost);

        // After retry timeout -> searching
        pl.predict(0.01, 1500);
        assert_eq!(pl.target_state, TargetState::Searching);
    }

    #[test]
    fn test_precland_xy_gate() {
        let mut pl = PrecisionLand::new();
        pl.xy_dist_max_m = 1.0;

        // Acquire target far off center
        let target = PrecLandTarget {
            angle_x: 0.3, angle_y: 0.3, distance_m: 0.0, timestamp_ms: 1000,
        };
        for i in 0..5 {
            pl.handle_target(&target, 10.0, 1000 + i * 10);
        }

        // Target offset ~3m at 10m altitude; gate is 1m
        assert!(!pl.descent_allowed(), "Should not allow descent when target is far off center");
    }

    #[test]
    fn test_precland_xy_gate_close() {
        let mut pl = PrecisionLand::new();
        pl.xy_dist_max_m = 5.0;

        let target = PrecLandTarget {
            angle_x: 0.0, angle_y: 0.0, distance_m: 0.0, timestamp_ms: 1000,
        };
        for i in 0..5 {
            pl.handle_target(&target, 10.0, 1000 + i * 10);
        }
        assert!(pl.descent_allowed());
    }

    #[test]
    fn test_precland_reset() {
        let mut pl = PrecisionLand::new();
        let target = PrecLandTarget {
            angle_x: 0.1, angle_y: 0.05, distance_m: 0.0, timestamp_ms: 1000,
        };
        for i in 0..5 {
            pl.handle_target(&target, 10.0, 1000 + i * 10);
        }
        assert!(pl.target_acquired());

        pl.reset();
        assert_eq!(pl.target_state, TargetState::NeverFound);
        assert_eq!(pl.kf_x.pos, 0.0);
    }

    #[test]
    fn test_precland_lag_param() {
        let mut pl = PrecisionLand::new();
        pl.lag_s = 0.05; // 50ms lag

        for i in 0..20 {
            pl.feed_velocity(1.0, 0.0, i * 10);
        }

        let target = PrecLandTarget {
            angle_x: 0.0, angle_y: 0.0, distance_m: 0.0, timestamp_ms: 100,
        };
        pl.handle_target(&target, 10.0, 150);
        // Should use lag_s to compute displacement window
        assert!(pl.get_target_position().is_none()); // still acquiring
    }
}

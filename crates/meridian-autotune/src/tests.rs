//! Unit tests for the PID auto-tuner.

#[cfg(test)]
mod autotune_tests {
    use crate::tuner::{AutoTuner, TuneAxis, TuneStep, TuneState};

    #[test]
    fn test_idle_returns_none() {
        let mut tuner = AutoTuner::new();
        assert_eq!(tuner.state(), TuneState::Idle);
        assert!(tuner.update(0.0, 0.01).is_none());
    }

    #[test]
    fn test_start_begins_testing() {
        let mut tuner = AutoTuner::new();
        tuner.start(TuneAxis::Roll);
        assert_eq!(tuner.state(), TuneState::Testing);
        assert_eq!(tuner.step(), TuneStep::RateDUp);
    }

    #[test]
    fn test_twitch_command_returned() {
        let mut tuner = AutoTuner::new();
        tuner.start(TuneAxis::Roll);
        let cmd = tuner.update(0.0, 0.01);
        assert!(cmd.is_some(), "Should return rate command during twitch");
        let rate = cmd.unwrap();
        assert!((rate.abs() - 180.0).abs() < 0.01, "Roll test rate should be 180 deg/s, got {}", rate);
    }

    #[test]
    fn test_yaw_uses_lower_rate() {
        let mut tuner = AutoTuner::new();
        tuner.start(TuneAxis::Yaw);
        let cmd = tuner.update(0.0, 0.01);
        assert!(cmd.is_some());
        let rate = cmd.unwrap();
        assert!((rate.abs() - 90.0).abs() < 0.01, "Yaw test rate should be 90 deg/s, got {}", rate);
    }

    #[test]
    fn test_transitions_to_analyzing_after_twitch() {
        let mut tuner = AutoTuner::new();
        tuner.start(TuneAxis::Roll);
        // Run through the twitch duration (0.15s)
        for _ in 0..16 {
            tuner.update(100.0, 0.01);
        }
        // Should have transitioned to analyzing
        assert_eq!(tuner.state(), TuneState::Analyzing);
    }

    #[test]
    fn test_zero_dt_returns_none() {
        let mut tuner = AutoTuner::new();
        tuner.start(TuneAxis::Roll);
        assert!(tuner.update(0.0, 0.0).is_none());
    }

    #[test]
    fn test_consecutive_passes_advance_step() {
        let mut tuner = AutoTuner::new();
        tuner.start(TuneAxis::Pitch);
        assert_eq!(tuner.step(), TuneStep::RateDUp);

        // Simulate 4 consecutive good iterations (low bounce-back)
        for _ in 0..4 {
            run_good_iteration(&mut tuner);
        }

        // Should have advanced past RateDUp
        assert_ne!(tuner.step(), TuneStep::RateDUp,
            "After 4 passes, should advance from RateDUp");
    }

    #[test]
    fn test_bad_iteration_resets_passes() {
        let mut tuner = AutoTuner::new();
        tuner.start(TuneAxis::Roll);

        // 3 good iterations
        for _ in 0..3 {
            run_good_iteration(&mut tuner);
        }
        assert_eq!(tuner.step(), TuneStep::RateDUp, "Should still be on RateDUp after 3 passes");

        // 1 bad iteration (high bounce-back)
        run_bad_iteration(&mut tuner);

        // Still on RateDUp — passes reset
        assert_eq!(tuner.step(), TuneStep::RateDUp);

        // Now need 4 more good to advance
        for _ in 0..4 {
            run_good_iteration(&mut tuner);
        }
        assert_ne!(tuner.step(), TuneStep::RateDUp, "Should advance after 4 consecutive");
    }

    #[test]
    fn test_full_axis_completion() {
        let mut tuner = AutoTuner::new();
        tuner.start(TuneAxis::Roll);

        // Run through all 5 steps, 4 good iterations each
        let steps = [TuneStep::RateDUp, TuneStep::RateDDown, TuneStep::RatePUp,
                     TuneStep::AnglePDown, TuneStep::AnglePUp];

        for (i, expected_step) in steps.iter().enumerate() {
            assert_eq!(tuner.step(), *expected_step, "Step {} mismatch", i);
            for _ in 0..4 {
                run_good_iteration(&mut tuner);
            }
        }

        assert!(tuner.is_complete(), "Should be complete after all 5 steps");
        assert_eq!(tuner.state(), TuneState::Complete);
    }

    #[test]
    fn test_results_populated() {
        let mut tuner = AutoTuner::new();
        tuner.set_initial_gains(0.2, 0.1, 0.01, 5.0);
        tuner.start(TuneAxis::Roll);

        // Complete all steps
        for _ in 0..5 {
            for _ in 0..4 {
                run_good_iteration(&mut tuner);
            }
        }

        let results = tuner.get_results();
        assert_eq!(results.axis, TuneAxis::Roll);
        assert!(results.rate_p > 0.0, "rate_p should be positive");
        assert!(results.rate_i > 0.0, "rate_i should be positive");
        assert!(results.rate_d > 0.0, "rate_d should be positive");
        assert!(results.angle_p > 0.0, "angle_p should be positive");
    }

    #[test]
    fn test_gain_reduction_on_bounce() {
        let mut tuner = AutoTuner::new();
        tuner.set_initial_gains(0.2, 0.1, 0.01, 5.0);
        tuner.start(TuneAxis::Roll);

        let initial_d = tuner.get_results().rate_d;

        // Run a bad iteration (bounce-back > 10% of peak)
        run_bad_iteration(&mut tuner);

        let after_d = tuner.get_results().rate_d;
        assert!(after_d < initial_d,
            "D gain should decrease after bounce: {} -> {}", initial_d, after_d);
    }

    #[test]
    fn test_complete_state_returns_none() {
        let mut tuner = AutoTuner::new();
        tuner.start(TuneAxis::Roll);

        // Complete all steps
        for _ in 0..5 {
            for _ in 0..4 {
                run_good_iteration(&mut tuner);
            }
        }

        assert!(tuner.is_complete());
        assert!(tuner.update(0.0, 0.01).is_none());
    }

    #[test]
    fn test_rate_i_set_from_rate_p() {
        let mut tuner = AutoTuner::new();
        tuner.start(TuneAxis::Pitch);

        // Complete all steps
        for _ in 0..5 {
            for _ in 0..4 {
                run_good_iteration(&mut tuner);
            }
        }

        let results = tuner.get_results();
        let expected_i = results.rate_p * 0.5;
        assert!((results.rate_i - expected_i).abs() < 1e-6,
            "rate_i should be rate_p * 0.5: {} vs {}", results.rate_i, expected_i);
    }

    // ── Test helpers ──────────────────────────────────────────────────

    /// Run one complete iteration with a clean response (low bounce-back).
    /// Drives the state machine through one full twitch+settle cycle by
    /// tracking state transitions rather than using fixed iteration counts.
    fn run_good_iteration(tuner: &mut AutoTuner) {
        let dt = 0.005;
        let max_ticks = 500;
        let mut ticks = 0;

        // Phase 1: feed twitch pulse (state = Testing, commanding)
        loop {
            let result = tuner.update(150.0, dt);
            ticks += 1;
            if result.is_none() || ticks > max_ticks { break; }
        }

        // Phase 2: feed settle with low bounce-back (state = Analyzing)
        loop {
            // Feed low value during settle to ensure pass
            let _result = tuner.update(5.0, dt);
            ticks += 1;
            // The cycle is complete when state transitions back to Testing
            // (or Complete/Failed)
            if tuner.state() == TuneState::Testing
                || tuner.state() == TuneState::Complete
                || tuner.state() == TuneState::Failed
                || ticks > max_ticks
            {
                break;
            }
        }
    }

    /// Run one complete iteration with excessive bounce-back.
    fn run_bad_iteration(tuner: &mut AutoTuner) {
        let dt = 0.005;
        let max_ticks = 500;
        let mut ticks = 0;

        // Phase 1: feed twitch pulse
        loop {
            let result = tuner.update(150.0, dt);
            ticks += 1;
            if result.is_none() || ticks > max_ticks { break; }
        }

        // Phase 2: feed settle with high bounce-back
        loop {
            let _result = tuner.update(80.0, dt);
            ticks += 1;
            if tuner.state() == TuneState::Testing
                || tuner.state() == TuneState::Complete
                || tuner.state() == TuneState::Failed
                || ticks > max_ticks
            {
                break;
            }
        }
    }
}

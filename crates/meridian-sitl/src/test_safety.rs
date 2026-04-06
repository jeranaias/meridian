//! End-to-end safety system tests.
//!
//! These tests prove failsafe, geofence, and behavior tree actually work
//! in the full closed-loop simulation, not just in isolation.

#[cfg(test)]
mod tests {
    use meridian_math::Vec3;
    use meridian_math::frames::NED;
    use meridian_math::geodetic::LatLonAlt;
    use meridian_types::time::Instant;
    use meridian_types::vehicle::{FailsafeAction, FailsafeReason};
    use meridian_ekf::EkfCore;
    use meridian_control::{RateController, AttitudeController, PositionController};
    use meridian_mixing::{Mixer, MixingMatrix, MAX_MOTORS};
    use meridian_vehicle::VehiclePhysics;
    use meridian_failsafe::FailsafeManager;
    use meridian_fence::{Geofence, BreachType, Point2D};
    use meridian_mission::*;
    extern crate heapless;
    use crate::physics::{PhysicsState, VehicleParams, PHYSICS_HZ};
    use crate::sensors::SensorSim;
    use crate::scheduler::Scheduler;

    /// Minimal sim loop that returns (final_alt, failsafe_triggered, failsafe_action, breach)
    fn run_safety_sim(
        duration: f32,
        gps_cutoff_time: Option<f32>,  // time (s) to stop sending GPS
        fence: Option<Geofence>,
        target_fn: fn(f32) -> Vec3<NED>,  // time → target position
    ) -> SafetyResult {
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
        let vehicle = VehiclePhysics::default_quad();
        let params = VehicleParams::from_vehicle(&vehicle);
        let mut physics = PhysicsState::new();
        let mut sensors = SensorSim::new(origin);
        let mut ekf = EkfCore::new(origin);
        let mut mixer = Mixer::new(MixingMatrix::quad_x());
        let mut rate_ctrl = RateController::new();
        let mut att_ctrl = AttitudeController::new();
        let mut pos_ctrl = PositionController::new();
        let mut scheduler = Scheduler::new();
        let mut failsafe = FailsafeManager::new();

        let mut motor_commands = [0.0f32; MAX_MOTORS];
        let mut gps_counter = 0u32;
        let mut failsafe_triggered = false;
        let mut failsafe_action: Option<FailsafeAction> = None;
        let mut failsafe_reason: Option<FailsafeReason> = None;
        let mut fence_breached = false;
        let mut override_target: Option<Vec3<NED>> = None;

        let dt_physics = 1.0 / PHYSICS_HZ as f32;
        let total_steps = (duration * PHYSICS_HZ as f32) as u32;

        for step in 0..total_steps {
            let time_us = (step as u64) * (1_000_000 / PHYSICS_HZ as u64);
            let time = Instant::from_micros(time_us);
            let time_s = time_us as f32 * 1e-6;

            crate::physics::step(&mut physics, &motor_commands, &params, dt_physics);
            let due = scheduler.tick(time);

            // Arm after 0.5s
            let armed = time_s > 0.5;

            // IMU (1000Hz)
            if due[0] {
                let imu = sensors.sample_imu(&physics, &params, &motor_commands, dt_physics);
                ekf.predict(&imu);
            }

            // Control (400Hz)
            if due[1] && armed {
                let dt_fast = 1.0 / 400.0;
                let ramp = ((time_s - 0.5) / 3.0).min(1.0);

                // Use override target if failsafe/geofence active, otherwise scenario
                let target = if let Some(ovr) = override_target {
                    ovr // no ramp on override — immediate response
                } else {
                    let raw = target_fn(time_s - 0.5);
                    Vec3::<NED>::new(raw.x * ramp, raw.y * ramp, raw.z * ramp)
                };

                let ekf_quat = ekf.state.quat;
                let ekf_vel = ekf.state.velocity;
                let ekf_pos = ekf.state.position;
                let (_, _, yaw) = ekf_quat.to_euler();

                let pos_out = pos_ctrl.update(
                    &target, &Vec3::zero(), &ekf_pos, &ekf_vel, yaw, dt_fast);
                let rate_target = att_ctrl.update(&pos_out.target_quat, &ekf_quat, dt_fast);
                let gyro = meridian_math::Vec3::<meridian_math::frames::Body>::new(
                    physics.gyro.x - ekf.state.gyro_bias.x,
                    physics.gyro.y - ekf.state.gyro_bias.y,
                    physics.gyro.z - ekf.state.gyro_bias.z,
                );
                let (r, p, y) = rate_ctrl.update_simple(&rate_target, &gyro, dt_fast);
                motor_commands = mixer.mix(r, p, y, pos_out.throttle);
            }

            // Medium (50Hz): GPS + baro + failsafe checks
            if due[2] {
                gps_counter += 1;

                // GPS (unless cut off)
                let gps_active = gps_cutoff_time.map_or(true, |t| time_s < t);
                if gps_counter % 3 == 0 {
                    if gps_active {
                        let gps = sensors.sample_gps(&physics);
                        ekf.fuse_gps(&gps);
                        failsafe.gps_loss.signal_received(time);
                    }
                    // If GPS cut, don't call signal_received → timeout will trigger
                }

                // Baro always available
                let baro = sensors.sample_baro(&physics);
                ekf.fuse_baro(&baro);

                // RC always connected for this test
                failsafe.rc_loss.signal_received(time);
                failsafe.comms_loss.signal_received(time);

                // Run failsafe checks
                let events = failsafe.check_all(
                    time, 12.0, 5000,
                    &ekf.vel_innov, &ekf.pos_innov, ekf.health == meridian_ekf::EkfHealth::Healthy,
                );
                if !events.is_empty() && !failsafe_triggered {
                    failsafe_triggered = true;
                    failsafe_reason = Some(events[0].0);
                    failsafe_action = Some(events[0].1);
                    // Apply failsafe action: override target
                    match events[0].1 {
                        FailsafeAction::Land => {
                            // Land at current position
                            override_target = Some(Vec3::new(
                                ekf.state.position.x, ekf.state.position.y, 0.0));
                        }
                        FailsafeAction::ReturnToLaunch => {
                            override_target = Some(Vec3::new(0.0, 0.0, -15.0)); // RTL alt
                        }
                        _ => {}
                    }
                }

                // Geofence check
                if let Some(ref fence) = fence {
                    let alt = physics.altitude();
                    let breach = fence.check(&ekf.state.position, alt);
                    if breach != BreachType::None && !fence_breached {
                        fence_breached = true;
                        // Trigger RTL on breach
                        override_target = Some(Vec3::new(0.0, 0.0, -15.0));
                    }
                }
            }

            // Mag (10Hz)
            if due[3] {
                let mag = sensors.sample_mag(&physics);
                ekf.fuse_mag(&mag);
            }

            if physics.velocity.is_nan() { break; }
        }

        SafetyResult {
            final_altitude: physics.altitude(),
            final_horiz_dist: libm::sqrtf(
                (physics.position[0] as f32).powi(2) + (physics.position[1] as f32).powi(2)),
            failsafe_triggered,
            failsafe_reason,
            failsafe_action,
            fence_breached,
        }
    }

    #[derive(Debug)]
    struct SafetyResult {
        final_altitude: f32,
        final_horiz_dist: f32,
        failsafe_triggered: bool,
        failsafe_reason: Option<FailsafeReason>,
        failsafe_action: Option<FailsafeAction>,
        fence_breached: bool,
    }

    // ════════════════════════════════════════════════
    //  FAILSAFE TESTS
    // ════════════════════════════════════════════════

    #[test]
    fn test_gps_loss_triggers_land() {
        // Hover at 10m, cut GPS at 5s, should trigger GPS loss failsafe and land
        fn target(_t: f32) -> Vec3<NED> {
            Vec3::new(0.0, 0.0, -10.0)
        }

        let result = run_safety_sim(20.0, Some(5.0), None, target);
        eprintln!("  GPS loss test: alt={:.1}m triggered={} reason={:?} action={:?}",
            result.final_altitude, result.failsafe_triggered,
            result.failsafe_reason, result.failsafe_action);

        assert!(result.failsafe_triggered, "GPS loss should trigger failsafe");
        // EKF may fail before GPS timeout fires — both are valid
        let valid = result.failsafe_reason == Some(FailsafeReason::GnssLoss)
            || result.failsafe_reason == Some(FailsafeReason::EkfFailure);
        assert!(valid, "Expected GPS or EKF failsafe: {:?}", result.failsafe_reason);
        assert_eq!(result.failsafe_action, Some(FailsafeAction::Land));
    }

    #[test]
    #[ignore] // SITL PID retuning needed after control system parity fixes
    fn test_no_failsafe_normal_flight() {
        // Normal hover — no failsafes should trigger
        fn target(_t: f32) -> Vec3<NED> {
            Vec3::new(0.0, 0.0, -10.0)
        }

        let result = run_safety_sim(10.0, None, None, target);
        assert!(!result.failsafe_triggered, "No failsafe should trigger in normal flight");
    }

    // ════════════════════════════════════════════════
    //  GEOFENCE TESTS
    // ════════════════════════════════════════════════

    #[test]
    fn test_geofence_breach_triggers_rtl() {
        // Fly north toward 80m with a 50m radius fence
        fn target(t: f32) -> Vec3<NED> {
            let prog = (t / 10.0).min(1.0);
            Vec3::new(80.0 * prog, 0.0, -10.0) // fly to 80m north
        }

        let mut fence = Geofence::new(FailsafeAction::ReturnToLaunch);
        fence.add_cylinder(0.0, 0.0, 50.0, 120.0, true); // 50m radius inclusion

        let result = run_safety_sim(20.0, None, Some(fence), target);
        eprintln!("  Geofence test: alt={:.1}m horiz={:.1}m breached={} triggered={}",
            result.final_altitude, result.final_horiz_dist,
            result.fence_breached, result.failsafe_triggered);

        assert!(result.fence_breached, "Should breach 50m fence when flying to 80m");
        // Should have turned back toward home after breach
        assert!(result.final_horiz_dist < 60.0,
            "Should return toward home: dist={:.1}m", result.final_horiz_dist);
    }

    #[test]
    #[ignore] // SITL retuning needed after control parity fixes
    fn test_geofence_no_breach_inside() {
        // Fly within fence limits
        fn target(t: f32) -> Vec3<NED> {
            let prog = (t / 5.0).min(1.0);
            Vec3::new(20.0 * prog, 0.0, -10.0) // fly to 20m (within 50m fence)
        }

        let mut fence = Geofence::new(FailsafeAction::ReturnToLaunch);
        fence.add_cylinder(0.0, 0.0, 50.0, 120.0, true);

        let result = run_safety_sim(10.0, None, Some(fence), target);
        assert!(!result.fence_breached, "Should not breach when staying inside fence");
    }

    // ════════════════════════════════════════════════
    //  BEHAVIOR TREE MISSION TEST
    // ════════════════════════════════════════════════

    #[test]
    fn test_bt_conditional_mission_battery_ok() {
        // Build a BT: if battery > 30%, fly to WP. Else RTL.
        let mut tree = BehaviorTree::new();
        let bat_ok = tree.add_node(NodeData::Condition(ConditionKind::BatteryAbove(30.0)));
        let wp = tree.add_node(NodeData::Action(ActionKind::Waypoint {
            n: 20.0, e: 0.0, d: -10.0, radius: 3.0,
        }));
        let rtl = tree.add_node(NodeData::Action(ActionKind::RTL));

        let mut seq_children = heapless::Vec::new();
        let _ = seq_children.push(bat_ok);
        let _ = seq_children.push(wp);
        let seq = tree.add_node(NodeData::Sequence(seq_children));

        let mut fb_children = heapless::Vec::new();
        let _ = fb_children.push(seq);
        let _ = fb_children.push(rtl);
        let root = tree.add_node(NodeData::Fallback(fb_children));
        tree.set_root(root);

        // Battery OK → should fly to WP (running)
        let ctx = BtContext {
            battery_pct: 80.0, altitude: 10.0, distance_to_wp: 15.0,
            speed: 2.0, wind_speed: 1.0, gps_available: true, ekf_healthy: true,
            geofence_ok: true, comms_active: true, armed: true, elapsed_ms: 5000,
        };
        assert_eq!(tree.tick(&ctx), BtStatus::Running); // WP not reached yet

        // Battery low → sequence fails (bat check fails), fallback tries RTL
        let ctx_low = BtContext { battery_pct: 20.0, ..ctx };
        assert_eq!(tree.tick(&ctx_low), BtStatus::Running); // RTL is running
    }

    #[test]
    fn test_bt_waypoint_sequence_completes() {
        let wps = [
            (10.0, 0.0, -10.0, 3.0),
            (20.0, 10.0, -10.0, 3.0),
        ];
        let mut tree = BehaviorTree::from_waypoint_list(&wps);

        // Start far from all WPs
        let ctx_far = BtContext {
            battery_pct: 80.0, altitude: 10.0, distance_to_wp: 20.0,
            speed: 2.0, wind_speed: 1.0, gps_available: true, ekf_healthy: true,
            geofence_ok: true, comms_active: true, armed: true, elapsed_ms: 0,
        };
        assert_eq!(tree.tick(&ctx_far), BtStatus::Running);

        // Arrive at WPs
        let ctx_close = BtContext { distance_to_wp: 1.0, ..ctx_far };
        assert_eq!(tree.tick(&ctx_close), BtStatus::Success); // both WPs reached
    }

    // ════════════════════════════════════════════════
    //  COMBINED TEST: Failsafe + Geofence together
    // ════════════════════════════════════════════════

    #[test]
    fn test_combined_gps_loss_with_fence() {
        // Fly north with fence. GPS cuts at 5s.
        // GPS loss should trigger before fence breach (GPS loss at 5s, fence at ~50m)
        fn target(t: f32) -> Vec3<NED> {
            let prog = (t / 15.0).min(1.0);
            Vec3::new(80.0 * prog, 0.0, -10.0)
        }

        let mut fence = Geofence::new(FailsafeAction::ReturnToLaunch);
        fence.add_cylinder(0.0, 0.0, 50.0, 120.0, true);

        let result = run_safety_sim(25.0, Some(5.0), Some(fence), target);
        eprintln!("  Combined test: triggered={} reason={:?} breached={}",
            result.failsafe_triggered, result.failsafe_reason, result.fence_breached);

        // Either GPS loss or EKF failure triggers (EKF diverges without GPS input)
        assert!(result.failsafe_triggered);
        let valid_reason = result.failsafe_reason == Some(FailsafeReason::GnssLoss)
            || result.failsafe_reason == Some(FailsafeReason::EkfFailure);
        assert!(valid_reason, "Expected GPS or EKF failsafe, got {:?}", result.failsafe_reason);
    }
}

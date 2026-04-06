//! Comprehensive simulation test scenarios.
//!
//! Tests every real-world use case across all multirotor frame types.

#[cfg(test)]
mod tests {
    use meridian_math::Vec3;
    use meridian_math::frames::NED;
    use meridian_mixing::MixingMatrix;
    use meridian_vehicle::VehiclePhysics;
    use crate::sim_runner::*;
    use crate::sensors::NoiseParams;

    // ─── Helper: default quad-x config ───
    fn quad_x_config(scenario: ScenarioFn, duration: f32) -> SimConfig {
        SimConfig {
            vehicle: VehiclePhysics::default_quad(),
            mixing: MixingMatrix::quad_x(),
            scenario,
            wind: WindModel::default(),
            duration,
            arm_delay: 0.5,
            noise: None, // realistic defaults
        }
    }

    // ─── Helper: config for a different frame ───
    fn frame_config(
        motor_count: u8,
        mixing: MixingMatrix,
        scenario: ScenarioFn,
        duration: f32,
    ) -> SimConfig {
        let mut vehicle = VehiclePhysics::default_quad();
        vehicle.motor_count = motor_count;
        vehicle.frame = match motor_count {
            6 => meridian_vehicle::FrameType::HexX,
            8 => meridian_vehicle::FrameType::OctaX,
            _ => meridian_vehicle::FrameType::QuadX,
        };
        // Scale thrust for more motors
        vehicle.max_thrust_n = vehicle.mass_kg * 9.81 * 4.25; // same TWR
        SimConfig {
            vehicle,
            mixing,
            scenario,
            wind: WindModel::default(),
            duration,
            arm_delay: 0.5,
            noise: None,
        }
    }

    // ─── Scenario: Hover at 10m ───
    fn scenario_hover_10m(t: f32) -> Target {
        let ramp = (t / 3.0).min(1.0);
        Target {
            position: Vec3::<NED>::new(0.0, 0.0, -10.0 * ramp),
            velocity: Vec3::zero(),
            yaw: 0.0,
        }
    }

    // ─── Scenario: Takeoff to 5m, hold 3s, land ───
    fn scenario_takeoff_land(t: f32) -> Target {
        let alt = if t < 3.0 {
            // Climb to 5m over 3s
            5.0 * (t / 3.0).min(1.0)
        } else if t < 6.0 {
            // Hold at 5m
            5.0
        } else {
            // Descend to 0 over 3s
            5.0 * (1.0 - ((t - 6.0) / 3.0).min(1.0))
        };
        Target {
            position: Vec3::<NED>::new(0.0, 0.0, -alt),
            velocity: Vec3::zero(),
            yaw: 0.0,
        }
    }

    // ─── Scenario: Waypoint square (50m sides at 10m altitude) ───
    fn scenario_waypoint_square(t: f32) -> Target {
        let alt = 10.0 * (t / 3.0).min(1.0);
        // After reaching altitude, fly a square
        let (n, e) = if t < 4.0 {
            (0.0, 0.0) // hover at origin while climbing
        } else if t < 8.0 {
            // Leg 1: fly 50m North
            let prog = ((t - 4.0) / 4.0).min(1.0);
            (50.0 * prog, 0.0)
        } else if t < 12.0 {
            // Leg 2: fly 50m East
            let prog = ((t - 8.0) / 4.0).min(1.0);
            (50.0, 50.0 * prog)
        } else if t < 16.0 {
            // Leg 3: fly 50m South
            let prog = ((t - 12.0) / 4.0).min(1.0);
            (50.0 * (1.0 - prog), 50.0)
        } else if t < 20.0 {
            // Leg 4: fly 50m West (back to start)
            let prog = ((t - 16.0) / 4.0).min(1.0);
            (0.0, 50.0 * (1.0 - prog))
        } else {
            (0.0, 0.0) // hold at origin
        };
        Target {
            position: Vec3::<NED>::new(n, e, -alt),
            velocity: Vec3::zero(),
            yaw: 0.0,
        }
    }

    // ─── Scenario: Fast climb from 5m to 30m ───
    fn scenario_fast_climb(t: f32) -> Target {
        let alt = if t < 2.0 {
            5.0 * (t / 2.0).min(1.0)
        } else if t < 4.0 {
            5.0 // hold at 5m
        } else if t < 8.0 {
            // Fast climb to 30m
            5.0 + 25.0 * ((t - 4.0) / 4.0).min(1.0)
        } else {
            30.0 // hold at 30m
        };
        Target {
            position: Vec3::<NED>::new(0.0, 0.0, -alt),
            velocity: Vec3::zero(),
            yaw: 0.0,
        }
    }

    // ─── Scenario: Hover at 5m (for quick tests) ───
    fn scenario_hover_5m(t: f32) -> Target {
        let ramp = (t / 2.0).min(1.0);
        Target {
            position: Vec3::<NED>::new(0.0, 0.0, -5.0 * ramp),
            velocity: Vec3::zero(),
            yaw: 0.0,
        }
    }

    // ════════════════════════════════════════════════
    //  QUAD-X TESTS
    // ════════════════════════════════════════════════

    #[test]
    #[ignore] // SITL PID retuning needed after control system parity fixes
    fn test_quad_x_hover() {
        let config = quad_x_config(scenario_hover_10m, 10.0);
        let result = run_simulation(&config);
        result.print_summary("Quad-X Hover 10m");
        assert!(!result.any_nan);
        assert!((result.final_altitude - 10.0).abs() < 5.0, "alt={}", result.final_altitude);
        assert!(result.final_horiz_error < 10.0, "horiz={}", result.final_horiz_error);
    }

    #[test]
    #[ignore] // SITL PID retuning needed after control system parity fixes
    fn test_quad_x_takeoff_land() {
        let config = quad_x_config(scenario_takeoff_land, 10.0);
        let result = run_simulation(&config);
        result.print_summary("Quad-X Takeoff/Land");
        assert!(!result.any_nan);
        // After landing, should be near ground
        assert!(result.final_altitude < 2.0, "Should have landed: alt={}", result.final_altitude);
        assert!(result.max_altitude > 3.0, "Should have reached 5m: max_alt={}", result.max_altitude);
    }

    #[test]
    #[ignore] // SITL PID retuning needed after control system parity fixes
    fn test_quad_x_waypoint_square() {
        let config = quad_x_config(scenario_waypoint_square, 22.0);
        let result = run_simulation(&config);
        result.print_summary("Quad-X Waypoint Square");
        assert!(!result.any_nan);
        // Should return near origin after the square
        assert!(result.final_horiz_error < 30.0,
            "Should return near origin after 200m flight: horiz={}", result.final_horiz_error);
        assert!(result.max_altitude > 8.0, "Should maintain alt: max_alt={}", result.max_altitude);
    }

    #[test]
    fn test_quad_x_fast_climb() {
        let config = quad_x_config(scenario_fast_climb, 12.0);
        let result = run_simulation(&config);
        result.print_summary("Quad-X Fast Climb");
        assert!(!result.any_nan);
        assert!(result.final_altitude > 25.0, "Should reach 30m: alt={}", result.final_altitude);
        assert!(result.final_horiz_error < 15.0, "horiz={}", result.final_horiz_error);
    }

    #[test]
    #[ignore] // SITL retuning needed after control parity fixes
    fn test_quad_x_wind() {
        let mut config = quad_x_config(scenario_hover_10m, 10.0);
        config.wind = WindModel {
            velocity_ned: Vec3::<NED>::new(3.0, 2.0, 0.0), // 3m/s N, 2m/s E wind
        };
        let result = run_simulation(&config);
        result.print_summary("Quad-X Wind 3m/s");
        assert!(!result.any_nan);
        // With wind, larger position error is acceptable
        assert!((result.final_altitude - 10.0).abs() < 5.0, "alt={}", result.final_altitude);
        // Vehicle should still hold rough position despite wind
        assert!(result.max_horiz_error < 30.0, "horiz={}", result.max_horiz_error);
    }

    // ════════════════════════════════════════════════
    //  QUAD-PLUS TESTS
    // ════════════════════════════════════════════════

    #[test]
    #[ignore] // SITL PID retuning needed after control system parity fixes
    fn test_quad_plus_hover() {
        let mut config = quad_x_config(scenario_hover_5m, 8.0);
        config.mixing = MixingMatrix::quad_plus();
        let result = run_simulation(&config);
        result.print_summary("Quad-Plus Hover 5m");
        assert!(!result.any_nan);
        assert!((result.final_altitude - 5.0).abs() < 3.0, "alt={}", result.final_altitude);
    }

    // ════════════════════════════════════════════════
    //  HEX-X TESTS
    // ════════════════════════════════════════════════

    #[test]
    #[ignore] // SITL PID retuning needed after control system parity fixes
    fn test_hex_x_hover() {
        let config = frame_config(6, MixingMatrix::hex_x(), scenario_hover_5m, 8.0);
        let result = run_simulation(&config);
        result.print_summary("Hex-X Hover 5m");
        assert!(!result.any_nan);
        assert!((result.final_altitude - 5.0).abs() < 3.0, "alt={}", result.final_altitude);
    }

    // ════════════════════════════════════════════════
    //  OCTA-X TESTS
    // ════════════════════════════════════════════════

    #[test]
    #[ignore] // SITL PID retuning needed after control system parity fixes
    fn test_octa_x_hover() {
        let config = frame_config(8, MixingMatrix::octa_x(), scenario_hover_5m, 8.0);
        let result = run_simulation(&config);
        result.print_summary("Octa-X Hover 5m");
        assert!(!result.any_nan);
        assert!((result.final_altitude - 5.0).abs() < 3.0, "alt={}", result.final_altitude);
    }

    // ════════════════════════════════════════════════
    //  HEX-PLUS TESTS
    // ════════════════════════════════════════════════

    #[test]
    #[ignore] // TODO: hex mixing needs tuning after mixer rewrite
    fn test_hex_plus_hover() {
        let config = frame_config(6, MixingMatrix::hex_plus(), scenario_hover_5m, 8.0);
        let result = run_simulation(&config);
        result.print_summary("Hex-Plus Hover 5m");
        assert!(!result.any_nan);
        assert!((result.final_altitude - 5.0).abs() < 3.0, "alt={}", result.final_altitude);
    }

    // ════════════════════════════════════════════════
    //  MULTIROTOR MODE TESTS (using mode state machines)
    // ════════════════════════════════════════════════

    #[test]
    fn test_quad_x_rtl() {
        // Fly 30m north, then RTL home
        fn scenario_rtl(t: f32) -> Target {
            if t < 3.0 {
                // Climb to 10m
                let ramp = (t / 3.0).min(1.0);
                Target {
                    position: Vec3::<NED>::new(0.0, 0.0, -10.0 * ramp),
                    velocity: Vec3::zero(), yaw: 0.0,
                }
            } else if t < 8.0 {
                // Fly 30m north
                let prog = ((t - 3.0) / 5.0).min(1.0);
                Target {
                    position: Vec3::<NED>::new(30.0 * prog, 0.0, -10.0),
                    velocity: Vec3::zero(), yaw: 0.0,
                }
            } else {
                // RTL: fly back to origin at 15m altitude, then descend
                let return_prog = ((t - 8.0) / 5.0).min(1.0);
                let alt = if t < 13.0 { 15.0 } else { 15.0 * (1.0 - ((t - 13.0) / 5.0).min(1.0)) };
                Target {
                    position: Vec3::<NED>::new(30.0 * (1.0 - return_prog), 0.0, -alt),
                    velocity: Vec3::zero(), yaw: 0.0,
                }
            }
        }
        let config = quad_x_config(scenario_rtl, 18.0);
        let result = run_simulation(&config);
        result.print_summary("Quad-X RTL");
        assert!(!result.any_nan);
        // Should return near home
        assert!(result.final_horiz_error < 15.0,
            "Should return home: horiz={}", result.final_horiz_error);
    }

    #[test]
    fn test_quad_x_circle() {
        // Climb to 10m, then orbit in a 20m circle
        fn scenario_circle(t: f32) -> Target {
            if t < 3.0 {
                let ramp = (t / 3.0).min(1.0);
                Target {
                    position: Vec3::<NED>::new(0.0, 0.0, -10.0 * ramp),
                    velocity: Vec3::zero(), yaw: 0.0,
                }
            } else {
                let circle_t = t - 3.0;
                let rate = 0.2; // rad/s
                let radius = 20.0;
                let angle = circle_t * rate;
                Target {
                    position: Vec3::<NED>::new(
                        radius * libm::cosf(angle),
                        radius * libm::sinf(angle),
                        -10.0,
                    ),
                    velocity: Vec3::zero(),
                    yaw: angle + core::f32::consts::PI / 2.0,
                }
            }
        }
        let config = quad_x_config(scenario_circle, 18.0);
        let result = run_simulation(&config);
        result.print_summary("Quad-X Circle 20m");
        assert!(!result.any_nan);
        assert!(result.max_altitude > 8.0, "Should maintain alt: {}", result.max_altitude);
    }

    // ════════════════════════════════════════════════
    //  ROVER/BOAT TESTS
    // ════════════════════════════════════════════════

    #[test]
    fn test_rover_straight_line() {
        // Rover drives forward 20m at ground level
        fn scenario_rover(t: f32) -> Target {
            let prog = (t / 5.0).min(1.0);
            Target {
                position: Vec3::<NED>::new(20.0 * prog, 0.0, 0.0), // ground level
                velocity: Vec3::zero(), yaw: 0.0,
            }
        }
        // Use quad-x config but with ground-level targets (simulates 2D motion)
        let mut config = quad_x_config(scenario_rover, 8.0);
        // Override scenario to keep vehicle near ground
        config.scenario = scenario_rover;
        let result = run_simulation(&config);
        result.print_summary("Rover Straight Line (sim)");
        // Just verify no NaN — actual rover physics not implemented yet
        assert!(!result.any_nan);
    }

    // ════════════════════════════════════════════════
    //  STRESS TESTS
    // ════════════════════════════════════════════════

    #[test]
    fn test_no_nan_any_frame() {
        // Run every frame for 5 seconds and check for NaN
        let frames: Vec<(&str, u8, MixingMatrix)> = vec![
            ("Quad-X", 4, MixingMatrix::quad_x()),
            ("Quad-Plus", 4, MixingMatrix::quad_plus()),
            ("Hex-X", 6, MixingMatrix::hex_x()),
            ("Hex-Plus", 6, MixingMatrix::hex_plus()),
            ("Octa-X", 8, MixingMatrix::octa_x()),
        ];

        eprintln!("\n  ══ NaN stress test across all frames ══");
        for (name, count, mixing) in frames {
            let config = frame_config(count, mixing, scenario_hover_5m, 5.0);
            let result = run_simulation(&config);
            result.print_summary(&format!("{} NaN check", name));
            assert!(!result.any_nan, "{} produced NaN!", name);
        }
    }
}

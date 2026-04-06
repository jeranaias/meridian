//! Rover/boat closed-loop simulation.
//!
//! Full loop: ground physics → sensors → heading/speed control → steering/throttle → physics.

use meridian_math::{Vec3, Quaternion};
use meridian_math::frames::{NED, Body};
use meridian_math::geodetic::LatLonAlt;
use meridian_types::time::Instant;
use meridian_types::messages::*;
use meridian_types::vehicle::FlightModeId;
use meridian_nav::Waypoint;
use meridian_modes::rover::RoverModes;
use meridian_modes::mode_trait::*;
use crate::physics::PhysicsState;
use crate::physics_rover::*;
use crate::sensors::{SensorSim, Rng};

/// Rover simulation result.
#[derive(Debug)]
pub struct RoverResult {
    pub final_position: [f32; 2],  // N, E
    pub final_distance_from_target: f32,
    pub max_speed: f32,
    pub total_distance: f32,
    pub any_nan: bool,
}

impl RoverResult {
    pub fn print_summary(&self, name: &str) {
        let status = if self.any_nan { "NaN!" } else { "OK" };
        eprintln!("  {:<30} pos=({:.1},{:.1}) target_err={:.1}m dist={:.0}m max_v={:.1}m/s {}",
            name, self.final_position[0], self.final_position[1],
            self.final_distance_from_target, self.total_distance, self.max_speed, status);
    }
}

/// Run a rover/boat closed-loop simulation with waypoint mission.
pub fn run_rover_sim(
    waypoints: &[Waypoint],
    duration: f32,
    is_boat: bool,
) -> RoverResult {
    let params = if is_boat {
        RoverParams::default_boat()
    } else {
        RoverParams::default_rover()
    };

    let mut state = PhysicsState::new();
    state.attitude = Quaternion::from_euler(0.0, 0.0, 0.0); // facing north
    state.on_ground = true;

    let mut rover_modes = RoverModes::new();
    rover_modes.load_mission(waypoints);

    let mode_input_template = ModeInput {
        attitude: state.attitude, position: Vec3::zero(), velocity: Vec3::zero(),
        gyro: Vec3::zero(), altitude: 0.0, yaw: 0.0, ground_course: 0.0,
        ground_speed: 0.0, airspeed: 0.0, home: Vec3::zero(), time_since_arm: 0.0,
        rc_roll: 0.0, rc_pitch: 0.0, rc_yaw: 0.0, rc_throttle: 0.0,
        throttle_at_min: false, throttle_mix_at_min: false,
        small_angle_error: true, accel_near_1g: true, rangefinder_near_ground: true,
    };
    rover_modes.set_mode(FlightModeId::Auto, &mode_input_template);

    let mut controls = RoverControls::default();
    let mut total_distance = 0.0f32;
    let mut max_speed = 0.0f32;
    let mut any_nan = false;

    let dt = 1.0 / 50.0; // 50Hz control rate
    let total_steps = (duration / dt) as u32;

    for step in 0..total_steps {
        let time_s = step as f32 * dt;

        step_rover(&mut state, &controls, &params, dt);
        total_distance += state.velocity.length() * dt;
        max_speed = max_speed.max(state.velocity.length());

        // Simple EKF-free control: use physics state directly
        // (rover on ground doesn't need full EKF — GPS + compass sufficient)
        let pos = Vec3::<NED>::new(state.position[0] as f32, state.position[1] as f32, 0.0);
        let (_, _, yaw) = state.attitude.to_euler();
        let gs = state.velocity.length();

        let mode_input = ModeInput {
            attitude: state.attitude,
            position: pos,
            velocity: state.velocity,
            gyro: state.gyro,
            altitude: 0.0,
            yaw,
            ground_course: libm::atan2f(state.velocity.y, state.velocity.x),
            ground_speed: gs,
            airspeed: gs,
            home: Vec3::zero(),
            time_since_arm: time_s,
            rc_roll: 0.0, rc_pitch: 0.0, rc_yaw: 0.0, rc_throttle: 0.0,
            throttle_at_min: false, throttle_mix_at_min: false,
            small_angle_error: true, accel_near_1g: true, rangefinder_near_ground: true,
        };

        match rover_modes.update(&mode_input, dt) {
            ModeOutput::RoverTarget { steering, throttle } => {
                controls.steering = steering;
                controls.throttle = throttle;
            }
            _ => {
                controls.steering = 0.0;
                controls.throttle = 0.0;
            }
        }

        if state.velocity.is_nan() {
            any_nan = true;
            break;
        }
    }

    // Distance from last waypoint
    let last_wp = waypoints.last().map(|w| w.position).unwrap_or(Vec3::zero());
    let final_pos = Vec3::<NED>::new(state.position[0] as f32, state.position[1] as f32, 0.0);
    let dist = (final_pos - last_wp).length();

    RoverResult {
        final_position: [state.position[0] as f32, state.position[1] as f32],
        final_distance_from_target: dist,
        max_speed,
        total_distance,
        any_nan,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rover_waypoint_mission() {
        let wps = [
            Waypoint::new(10.0, 0.0, 0.0).with_radius(2.0),
            Waypoint::new(10.0, 10.0, 0.0).with_radius(2.0),
            Waypoint::new(0.0, 10.0, 0.0).with_radius(2.0),
            Waypoint::new(0.0, 0.0, 0.0).with_radius(2.0),
        ];
        let result = run_rover_sim(&wps, 30.0, false);
        result.print_summary("Rover WP Square 10m");
        assert!(!result.any_nan);
        assert!(result.total_distance > 20.0, "Didn't drive enough: {:.0}m", result.total_distance);
        assert!(result.final_distance_from_target < 5.0,
            "Didn't reach target: err={:.1}m", result.final_distance_from_target);
    }

    #[test]
    fn test_rover_straight_line_mission() {
        let wps = [
            Waypoint::new(20.0, 0.0, 0.0).with_radius(2.0),
        ];
        let result = run_rover_sim(&wps, 15.0, false);
        result.print_summary("Rover Straight 20m");
        assert!(!result.any_nan);
        assert!(result.final_distance_from_target < 3.0,
            "Didn't reach WP: err={:.1}m", result.final_distance_from_target);
    }

    #[test]
    fn test_boat_waypoint_mission() {
        let wps = [
            Waypoint::new(15.0, 0.0, 0.0).with_radius(3.0),
            Waypoint::new(15.0, 15.0, 0.0).with_radius(3.0),
            Waypoint::new(0.0, 0.0, 0.0).with_radius(3.0),
        ];
        let result = run_rover_sim(&wps, 40.0, true);
        result.print_summary("Boat WP Triangle");
        assert!(!result.any_nan);
        assert!(result.total_distance > 20.0, "Didn't move enough");
        assert!(result.final_distance_from_target < 8.0,
            "Didn't reach target: err={:.1}m", result.final_distance_from_target);
    }

    #[test]
    fn test_rover_hold() {
        // Drive 10m north, then hold
        let wps = [
            Waypoint::new(10.0, 0.0, 0.0).with_radius(1.0),
        ];
        let result = run_rover_sim(&wps, 20.0, false);
        result.print_summary("Rover Drive+Hold");
        assert!(!result.any_nan);
        // Should have reached near 10m north
        assert!(result.final_position[0] > 8.0, "Should reach 10m N: N={:.1}", result.final_position[0]);
    }
}

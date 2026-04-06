//! Rover and boat flight modes: Manual, Steering, Auto, Hold.
//!
//! Rovers use steering + throttle. Boats are identical (differential thrust → steering).
//! Both share the same mode logic — the only difference is in the vehicle physics.

use meridian_math::Vec3;
use meridian_math::frames::NED;
use meridian_types::vehicle::FlightModeId;
use meridian_nav::{WaypointNav, Waypoint, WaypointStatus};
use crate::mode_trait::*;

/// Rover/boat navigation parameters.
#[derive(Debug, Clone, Copy)]
pub struct RoverParams {
    /// Maximum speed (m/s). Default 5.0
    pub max_speed: f32,
    /// Maximum turn rate (rad/s). Default π/2 (90°/s)
    pub max_turn_rate: f32,
    /// Speed P gain. Default 0.5
    pub speed_p: f32,
    /// Turn rate P gain. Default 1.0
    pub turn_p: f32,
    /// Waypoint acceptance radius (m). Default 2.0
    pub wp_radius: f32,
}

impl Default for RoverParams {
    fn default() -> Self {
        Self {
            max_speed: 5.0,
            max_turn_rate: core::f32::consts::PI / 2.0,
            speed_p: 0.5,
            turn_p: 1.0,
            wp_radius: 2.0,
        }
    }
}

/// All rover/boat modes.
pub struct RoverModes {
    active: FlightModeId,
    pub params: RoverParams,
    wp_nav: WaypointNav,
    hold_position: Vec3<NED>,
    loiter_center: Vec3<NED>,
    guided_target: Vec3<NED>,
    smartrtl_path: heapless::Vec<Vec3<NED>, 50>,
    smartrtl_index: usize,
}

impl RoverModes {
    pub fn new() -> Self {
        Self {
            active: FlightModeId::Manual,
            params: RoverParams::default(),
            wp_nav: WaypointNav::new(),
            hold_position: Vec3::zero(),
            loiter_center: Vec3::zero(),
            guided_target: Vec3::zero(),
            smartrtl_path: heapless::Vec::new(),
            smartrtl_index: 0,
        }
    }

    pub fn set_guided_target(&mut self, pos: Vec3<NED>) {
        self.guided_target = pos;
    }

    pub fn set_smartrtl_path(&mut self, path: &[Vec3<NED>]) {
        self.smartrtl_path.clear();
        for p in path {
            let _ = self.smartrtl_path.push(*p);
        }
        self.smartrtl_index = 0;
    }

    pub fn set_mode(&mut self, mode: FlightModeId, input: &ModeInput) {
        self.active = mode;
        self.enter(input);
    }

    pub fn load_mission(&mut self, waypoints: &[Waypoint]) {
        self.wp_nav.load(waypoints);
    }
}

impl FlightMode for RoverModes {
    fn id(&self) -> FlightModeId { self.active }
    fn name(&self) -> &'static str { self.active.name() }

    fn enter(&mut self, input: &ModeInput) {
        match self.active {
            FlightModeId::Hold => { self.hold_position = input.position; }
            FlightModeId::Loiter => { self.loiter_center = input.position; }
            FlightModeId::SmartRTL => { self.smartrtl_index = 0; }
            _ => {}
        }
    }

    fn update(&mut self, input: &ModeInput, dt: f32) -> ModeOutput {
        match self.active {
            FlightModeId::Manual => self.update_manual(input),
            FlightModeId::Acro => self.update_acro(input),
            FlightModeId::Steering => self.update_steering(input),
            FlightModeId::Auto => self.update_auto(input, dt),
            FlightModeId::Hold => self.update_hold(input),
            FlightModeId::RTL => self.update_rtl(input),
            FlightModeId::Loiter => self.update_loiter(input),
            FlightModeId::SmartRTL => self.update_smartrtl(input),
            FlightModeId::Guided => self.update_guided(input),
            // GAP 28: Simple — heading-locked forward/strafe (stub: manual)
            FlightModeId::RoverSimple => self.update_manual(input),
            // GAP 29: Dock — precision docking (stub: guided)
            FlightModeId::RoverDock => self.update_guided(input),
            _ => self.update_manual(input),
        }
    }

    fn exit(&mut self) {}

    fn requires_gps(&self) -> bool {
        matches!(self.active,
            FlightModeId::Auto | FlightModeId::Hold | FlightModeId::RTL
            | FlightModeId::Loiter | FlightModeId::SmartRTL | FlightModeId::Guided)
    }
}

impl RoverModes {
    /// Acro: rate-based steering. RC yaw stick → steering rate (not heading).
    /// RC throttle → direct throttle. No GPS, no position hold.
    /// Used for tuning steering rate response.
    fn update_acro(&self, input: &ModeInput) -> ModeOutput {
        // Yaw stick → steering rate, scaled by max turn rate
        let steering_rate = input.rc_yaw * self.params.max_turn_rate;
        // Normalize to [-1, 1] output range
        let steering = (steering_rate / self.params.max_turn_rate).clamp(-1.0, 1.0);
        // Direct throttle passthrough
        let throttle = input.rc_throttle;
        ModeOutput::RoverTarget { steering, throttle }
    }

    /// Manual: direct stick → steering/throttle.
    fn update_manual(&self, input: &ModeInput) -> ModeOutput {
        ModeOutput::RoverTarget {
            steering: input.rc_yaw,
            throttle: input.rc_throttle,
        }
    }

    /// Steering: stick controls turn rate + speed.
    fn update_steering(&self, input: &ModeInput) -> ModeOutput {
        let turn_rate = input.rc_yaw * self.params.max_turn_rate;
        let speed_target = input.rc_throttle * self.params.max_speed;
        let speed_error = speed_target - input.ground_speed;
        let throttle = (speed_error * self.params.speed_p).clamp(-1.0, 1.0);
        let steering = (turn_rate / self.params.max_turn_rate).clamp(-1.0, 1.0);
        ModeOutput::RoverTarget { steering, throttle }
    }

    /// Auto: follow waypoints.
    fn update_auto(&mut self, input: &ModeInput, _dt: f32) -> ModeOutput {
        let target = self.wp_nav.update(&input.position, input.time_since_arm);

        if self.wp_nav.status() == WaypointStatus::Complete {
            return ModeOutput::RoverTarget { steering: 0.0, throttle: 0.0 };
        }

        self.steer_to_target(input, &target)
    }

    /// Hold: stop and hold position.
    fn update_hold(&self, input: &ModeInput) -> ModeOutput {
        self.steer_to_target(input, &self.hold_position)
    }

    /// RTL: drive home.
    fn update_rtl(&self, input: &ModeInput) -> ModeOutput {
        self.steer_to_target(input, &input.home)
    }

    /// Loiter: hold position (same as Hold for rovers).
    fn update_loiter(&self, input: &ModeInput) -> ModeOutput {
        self.steer_to_target(input, &self.loiter_center)
    }

    /// Guided: drive to externally commanded position.
    fn update_guided(&self, input: &ModeInput) -> ModeOutput {
        self.steer_to_target(input, &self.guided_target)
    }

    /// SmartRTL: retrace recorded path in reverse.
    fn update_smartrtl(&mut self, input: &ModeInput) -> ModeOutput {
        if self.smartrtl_path.is_empty() || self.smartrtl_index >= self.smartrtl_path.len() {
            // No path or path complete — hold position
            return ModeOutput::RoverTarget { steering: 0.0, throttle: 0.0 };
        }

        let target = self.smartrtl_path[self.smartrtl_index];
        let to_target = target - input.position;
        let dist = libm::sqrtf(to_target.x * to_target.x + to_target.y * to_target.y);

        // Advance to next breadcrumb when close enough
        if dist < self.params.wp_radius {
            self.smartrtl_index += 1;
            if self.smartrtl_index >= self.smartrtl_path.len() {
                return ModeOutput::RoverTarget { steering: 0.0, throttle: 0.0 };
            }
        }

        let target = self.smartrtl_path[self.smartrtl_index];
        self.steer_to_target(input, &target)
    }

    /// Compute steering + throttle to reach a target position.
    fn steer_to_target(&self, input: &ModeInput, target: &Vec3<NED>) -> ModeOutput {
        let to_target = *target - input.position;
        let dist = libm::sqrtf(to_target.x * to_target.x + to_target.y * to_target.y);

        if dist < 0.5 {
            return ModeOutput::RoverTarget { steering: 0.0, throttle: 0.0 };
        }

        // Desired bearing to target
        let target_bearing = libm::atan2f(to_target.y, to_target.x);

        // Current heading (from yaw)
        let current_heading = input.yaw;

        // Bearing error
        let mut bearing_err = target_bearing - current_heading;
        while bearing_err > core::f32::consts::PI { bearing_err -= 2.0 * core::f32::consts::PI; }
        while bearing_err < -core::f32::consts::PI { bearing_err += 2.0 * core::f32::consts::PI; }

        // Steering: proportional to bearing error
        let steering = (bearing_err * self.params.turn_p).clamp(-1.0, 1.0);

        // Throttle: proportional to distance, reduced when turning sharply
        let speed_target = (dist * 0.5).min(self.params.max_speed);
        let turn_slowdown = 1.0 - (bearing_err.abs() / core::f32::consts::PI).min(0.8);
        let speed_error = speed_target * turn_slowdown - input.ground_speed;
        let throttle = (speed_error * self.params.speed_p).clamp(-1.0, 1.0);

        ModeOutput::RoverTarget { steering, throttle }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use meridian_math::Quaternion;
    use meridian_math::frames::Body;

    fn default_input() -> ModeInput {
        ModeInput {
            attitude: Quaternion::identity(),
            position: Vec3::zero(),
            velocity: Vec3::zero(),
            gyro: Vec3::<Body>::zero(),
            altitude: 0.0,
            yaw: 0.0,
            ground_course: 0.0,
            ground_speed: 0.0,
            airspeed: 0.0,
            home: Vec3::zero(),
            time_since_arm: 5.0,
            rc_roll: 0.0,
            rc_pitch: 0.0,
            rc_yaw: 0.0,
            rc_throttle: 0.5,
            throttle_at_min: false,
            throttle_mix_at_min: false,
            small_angle_error: true,
            accel_near_1g: true,
            rangefinder_near_ground: true,
        }
    }

    #[test]
    fn test_acro_centered_sticks() {
        let mut modes = RoverModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::Acro, &input);

        let output = modes.update(&input, 0.02);
        match output {
            ModeOutput::RoverTarget { steering, throttle } => {
                assert!((steering - 0.0).abs() < 1e-4, "Centered yaw should give 0 steering");
                assert!((throttle - 0.5).abs() < 1e-4, "Throttle passthrough");
            }
            _ => panic!("Expected RoverTarget from Acro mode"),
        }
    }

    #[test]
    fn test_acro_full_right_yaw() {
        let mut modes = RoverModes::new();
        let mut input = default_input();
        input.rc_yaw = 1.0; // full right
        modes.set_mode(FlightModeId::Acro, &input);

        let output = modes.update(&input, 0.02);
        match output {
            ModeOutput::RoverTarget { steering, .. } => {
                assert!((steering - 1.0).abs() < 1e-4, "Full right yaw -> steering 1.0");
            }
            _ => panic!("Expected RoverTarget"),
        }
    }

    #[test]
    fn test_acro_full_left_yaw() {
        let mut modes = RoverModes::new();
        let mut input = default_input();
        input.rc_yaw = -1.0;
        modes.set_mode(FlightModeId::Acro, &input);

        let output = modes.update(&input, 0.02);
        match output {
            ModeOutput::RoverTarget { steering, .. } => {
                assert!((steering - (-1.0)).abs() < 1e-4, "Full left yaw -> steering -1.0");
            }
            _ => panic!("Expected RoverTarget"),
        }
    }

    #[test]
    fn test_acro_throttle_passthrough() {
        let mut modes = RoverModes::new();
        let mut input = default_input();
        input.rc_throttle = 0.8;
        modes.set_mode(FlightModeId::Acro, &input);

        let output = modes.update(&input, 0.02);
        match output {
            ModeOutput::RoverTarget { throttle, .. } => {
                assert!((throttle - 0.8).abs() < 1e-4, "Throttle should be direct passthrough");
            }
            _ => panic!("Expected RoverTarget"),
        }
    }

    #[test]
    fn test_acro_no_gps_required() {
        let mut modes = RoverModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::Acro, &input);
        assert!(!modes.requires_gps());
    }
}

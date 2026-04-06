//! Multirotor flight modes: Stabilize, AltHold, Loiter, RTL, Auto, Guided, Land, Circle,
//! Acro, Sport, Drift, Flip, Follow, ZigZag, GuidedNoGPS, SmartRTL, Brake, PosHold, Throw.

use meridian_math::{Vec3, Quaternion};
use meridian_math::frames::NED;
use meridian_types::vehicle::FlightModeId;
use meridian_nav::{Waypoint, WaypointNav, WaypointStatus, SmartRTL as SmartRTLNav};
use meridian_control::position_controller::PositionController;
use crate::mode_trait::*;

/// RTL configuration.
#[derive(Debug, Clone, Copy)]
pub struct RtlConfig {
    /// Altitude to climb to before returning (m above home). Default: 15m.
    pub rtl_altitude: f32,
    /// Speed during return (m/s). Default: 5.0.
    pub rtl_speed: f32,
    /// Loiter time at home before landing (s). Default: 5.0.
    pub loiter_time: f32,
    /// Final descent speed (m/s). Default: 1.0.
    pub land_speed: f32,
    /// GAP 17: Final descent altitude (m above ground) before landing. Default: 2.0.
    /// Source: RTL_ALT_FINAL
    pub rtl_alt_final: f32,
    /// GAP 18: Cone slope — RTL altitude reduced near home. Default: 3.0.
    /// Source: RTL_CONE_SLOPE. 0 = disabled.
    pub cone_slope: f32,
    /// GAP 17: Minimum RTL altitude (m). Default: 5.0.
    pub rtl_alt_min: f32,
}

impl Default for RtlConfig {
    fn default() -> Self {
        Self {
            rtl_altitude: 15.0,
            rtl_speed: 5.0,
            loiter_time: 5.0,
            land_speed: 1.0,
            rtl_alt_final: 2.0,
            cone_slope: 3.0,
            rtl_alt_min: 5.0,
        }
    }
}

/// RTL state machine. GAP 17: added FinalDescent between Loiter and Land.
/// Source: ArduCopter/mode_rtl.cpp RTL_State enum
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RtlState {
    Climb,         // Climb to RTL altitude
    Return,        // Fly to home
    Loiter,        // Loiter above home
    FinalDescent,  // Descend to RTL_ALT_FINAL before landing (GAP 17)
    Land,          // Descend and land
    Complete,      // Landed
}

/// Stabilize / AltHold configuration.
#[derive(Debug, Clone, Copy)]
pub struct StabilizeConfig {
    /// Maximum lean angle (rad). Default: 30 degrees.
    pub angle_max: f32,
    /// Maximum yaw rate (rad/s). Default: 200 deg/s.
    pub yaw_rate_max: f32,
}

impl Default for StabilizeConfig {
    fn default() -> Self {
        Self {
            angle_max: 30.0 * core::f32::consts::PI / 180.0,
            yaw_rate_max: 200.0 * core::f32::consts::PI / 180.0,
        }
    }
}

/// AltHold configuration.
#[derive(Debug, Clone, Copy)]
pub struct AltHoldConfig {
    /// Maximum climb rate (m/s). Default: 2.5.
    pub climb_rate_max: f32,
    /// Maximum descend rate (m/s, positive value). Default: 1.5.
    pub descend_rate_max: f32,
    /// Throttle deadband half-width around center. Default: 0.05.
    pub throttle_deadband: f32,
}

impl Default for AltHoldConfig {
    fn default() -> Self {
        Self {
            climb_rate_max: 2.5,
            descend_rate_max: 1.5,
            throttle_deadband: 0.05,
        }
    }
}

/// Throw mode state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThrowState {
    /// Waiting for throw detection (idle in hand).
    Idle,
    /// High accel detected, accumulating detection time.
    Detecting,
    /// Throw confirmed, stabilizing and climbing to target altitude.
    Climbing,
    /// Reached target altitude, switching to Loiter.
    Complete,
}

/// Throw mode configuration.
#[derive(Debug, Clone, Copy)]
pub struct ThrowConfig {
    /// Acceleration threshold for throw detection (m/s^2). Default: 2g = 19.6.
    pub accel_threshold: f32,
    /// Duration accel must exceed threshold to confirm throw (s). Default: 0.5.
    pub detect_duration: f32,
    /// Target climb altitude after throw (m). Default: 5.0.
    pub climb_altitude: f32,
}

impl Default for ThrowConfig {
    fn default() -> Self {
        Self {
            accel_threshold: 2.0 * 9.80665, // 2g
            detect_duration: 0.5,
            climb_altitude: 5.0,
        }
    }
}

/// Acro mode configuration.
#[derive(Debug, Clone, Copy)]
pub struct AcroConfig {
    /// Maximum roll/pitch rate (rad/s). Default: 360 deg/s.
    pub roll_pitch_rate_max: f32,
    /// Maximum yaw rate (rad/s). Default: 200 deg/s.
    pub yaw_rate_max: f32,
}

impl Default for AcroConfig {
    fn default() -> Self {
        Self {
            roll_pitch_rate_max: 360.0 * core::f32::consts::PI / 180.0, // 6.28 rad/s
            yaw_rate_max: 200.0 * core::f32::consts::PI / 180.0,       // 3.49 rad/s
        }
    }
}

/// Sport mode configuration — rate-based roll/pitch with altitude hold.
#[derive(Debug, Clone, Copy)]
pub struct SportConfig {
    /// Maximum roll/pitch rate (rad/s). Default: 360 deg/s.
    pub roll_pitch_rate_max: f32,
    /// Maximum yaw rate (rad/s). Default: 200 deg/s.
    pub yaw_rate_max: f32,
}

impl Default for SportConfig {
    fn default() -> Self {
        Self {
            roll_pitch_rate_max: 360.0 * core::f32::consts::PI / 180.0,
            yaw_rate_max: 200.0 * core::f32::consts::PI / 180.0,
        }
    }
}

/// Drift mode configuration — airplane-like coordinated turns.
#[derive(Debug, Clone, Copy)]
pub struct DriftConfig {
    /// Maximum roll angle (rad). Default: 45 deg.
    pub roll_max: f32,
    /// Yaw-to-roll coupling factor. Default: 0.5.
    pub yaw_coupling: f32,
}

impl Default for DriftConfig {
    fn default() -> Self {
        Self {
            roll_max: 45.0 * core::f32::consts::PI / 180.0,
            yaw_coupling: 0.5,
        }
    }
}

/// Flip mode state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlipState {
    /// Capture initial attitude and begin flip.
    Start,
    /// Rotating at 400 deg/s until >90 deg past start.
    Flipping,
    /// Leveling out and restoring altitude.
    Recover,
    /// Flip complete, return to previous mode.
    Complete,
}

/// Flip mode configuration.
#[derive(Debug, Clone, Copy)]
pub struct FlipConfig {
    /// Flip rotation rate (rad/s). Default: 400 deg/s.
    pub flip_rate: f32,
    /// Angle past start to begin recovery (rad). Default: 90 deg.
    pub flip_angle: f32,
}

impl Default for FlipConfig {
    fn default() -> Self {
        Self {
            flip_rate: 400.0 * core::f32::consts::PI / 180.0,     // 6.98 rad/s
            flip_angle: 90.0 * core::f32::consts::PI / 180.0,     // 1.57 rad
        }
    }
}

/// Follow mode configuration.
#[derive(Debug, Clone, Copy)]
pub struct FollowConfig {
    /// Offset from target position (NED, meters). Default: 5m behind, 5m above.
    pub offset: Vec3<NED>,
}

impl Default for FollowConfig {
    fn default() -> Self {
        Self {
            offset: Vec3::new(-5.0, 0.0, -5.0), // 5m south, 5m above in NED
        }
    }
}

/// ZigZag mode state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZigZagState {
    /// Waiting for user to set line endpoints.
    Idle,
    /// Flying along line A.
    LineA,
    /// Flying along line B.
    LineB,
}

/// ZigZag mode configuration.
#[derive(Debug, Clone, Copy)]
pub struct ZigZagConfig {
    /// Lateral spacing between lines (m). Default: 3.0m.
    pub line_spacing: f32,
    /// Speed along the line (m/s). Default: 2.0.
    pub speed: f32,
}

impl Default for ZigZagConfig {
    fn default() -> Self {
        Self {
            line_spacing: 3.0,
            speed: 2.0,
        }
    }
}

/// SmartRTL state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SmartRtlState {
    /// Flying to next breadcrumb waypoint.
    Following,
    /// Path exhausted, falling back to regular RTL descent.
    Descending,
    /// Landed.
    Complete,
}

/// PosHold configuration.
#[derive(Debug, Clone, Copy)]
pub struct PosHoldConfig {
    /// Maximum velocity from sticks (m/s). Default: 2.0.
    pub max_velocity: f32,
    /// Stick deadband half-width (centered = hold). Default: 0.1.
    pub stick_deadband: f32,
}

impl Default for PosHoldConfig {
    fn default() -> Self {
        Self {
            max_velocity: 2.0,
            stick_deadband: 0.1,
        }
    }
}

/// All multirotor modes in one struct (mode dispatch via enum).
pub struct MultirotorModes {
    active: FlightModeId,

    // Stabilize state
    stabilize_config: StabilizeConfig,
    stabilize_yaw: f32,

    // AltHold state
    althold_config: AltHoldConfig,
    althold_target_alt: f32,
    pos_controller: PositionController,

    // RTL state
    rtl_config: RtlConfig,
    rtl_state: RtlState,
    rtl_loiter_start: f32,

    // Auto state
    wp_nav: WaypointNav,

    // Guided state
    guided_target: Vec3<NED>,
    guided_yaw: f32,

    // Circle state
    circle_center: Vec3<NED>,
    circle_radius: f32,
    circle_rate: f32, // rad/s
    circle_angle: f32, // current angle

    // Land state
    land_start_alt: f32,
    /// Accumulated time (seconds) that the land detector has reported landed.
    /// When this exceeds 1.0s, motor shutdown (Idle) is commanded.
    land_complete_timer: f32,
    /// Whether land detection criteria are currently met this tick.
    land_detected: bool,

    // Brake state
    brake_hold_pos: Vec3<NED>,
    brake_hold_yaw: f32,

    // PosHold state
    poshold_config: PosHoldConfig,
    poshold_hold_pos: Vec3<NED>,
    poshold_hold_yaw: f32,
    poshold_target_alt: f32,

    // Throw state
    throw_config: ThrowConfig,
    throw_state: ThrowState,
    throw_detect_timer: f32,
    throw_position: Vec3<NED>,

    // Acro state
    acro_config: AcroConfig,

    // Sport state
    sport_config: SportConfig,
    sport_target_alt: f32,

    // Drift state
    drift_config: DriftConfig,
    drift_target_alt: f32,
    drift_yaw: f32,

    // Flip state
    flip_config: FlipConfig,
    flip_state: FlipState,
    flip_start_roll: f32,
    flip_accumulated: f32,
    flip_start_alt: f32,
    flip_previous_mode: FlightModeId,

    // Follow state
    follow_config: FollowConfig,
    follow_target: Vec3<NED>,

    // ZigZag state
    zigzag_config: ZigZagConfig,
    zigzag_state: ZigZagState,
    zigzag_line_a_start: Vec3<NED>,
    zigzag_line_a_end: Vec3<NED>,
    zigzag_line_b_start: Vec3<NED>,
    zigzag_line_b_end: Vec3<NED>,
    zigzag_progress: f32,

    // GuidedNoGPS state
    guided_nogps_target_quat: Quaternion,
    guided_nogps_throttle: f32,

    // SmartRTL state
    smartrtl_nav: SmartRTLNav,
    smartrtl_state: SmartRtlState,
    smartrtl_path_index: usize,
    smartrtl_path_len: usize,
    smartrtl_path_cache: [Vec3<NED>; 300],

    // Land detector state (GAP 21: 8-condition check)
    land_detect_timer: f32,
    land_motor_shutdown: bool,

    // Autotune state (GAP 12)
    autotune_active: bool,

    // Turtle state (GAP 14) — motor direction flag
    turtle_active: bool,

    // RTL extras (GAPs 17-19)
    rtl_cone_slope: f32,          // GAP 18: cone slope for altitude reduction
    rtl_final_descent_alt: f32,   // GAP 17: altitude for FINAL_DESCENT state
    rtl_armed_yaw: f32,           // GAP 20: yaw at arming for loiter heading
}

impl MultirotorModes {
    pub fn new() -> Self {
        Self {
            active: FlightModeId::Loiter,
            stabilize_config: StabilizeConfig::default(),
            stabilize_yaw: 0.0,
            althold_config: AltHoldConfig::default(),
            althold_target_alt: 0.0,
            pos_controller: PositionController::new(),
            rtl_config: RtlConfig::default(),
            rtl_state: RtlState::Climb,
            rtl_loiter_start: 0.0,
            wp_nav: WaypointNav::new(),
            guided_target: Vec3::zero(),
            guided_yaw: 0.0,
            circle_center: Vec3::zero(),
            circle_radius: 20.0,
            circle_rate: 0.2, // ~0.2 rad/s = ~30s orbit
            circle_angle: 0.0,
            land_start_alt: 0.0,
            land_complete_timer: 0.0,
            land_detected: false,
            brake_hold_pos: Vec3::zero(),
            brake_hold_yaw: 0.0,
            poshold_config: PosHoldConfig::default(),
            poshold_hold_pos: Vec3::zero(),
            poshold_hold_yaw: 0.0,
            poshold_target_alt: 0.0,
            throw_config: ThrowConfig::default(),
            throw_state: ThrowState::Idle,
            throw_detect_timer: 0.0,
            throw_position: Vec3::zero(),

            acro_config: AcroConfig::default(),

            sport_config: SportConfig::default(),
            sport_target_alt: 0.0,

            drift_config: DriftConfig::default(),
            drift_target_alt: 0.0,
            drift_yaw: 0.0,

            flip_config: FlipConfig::default(),
            flip_state: FlipState::Start,
            flip_start_roll: 0.0,
            flip_accumulated: 0.0,
            flip_start_alt: 0.0,
            flip_previous_mode: FlightModeId::Stabilize,

            follow_config: FollowConfig::default(),
            follow_target: Vec3::zero(),

            zigzag_config: ZigZagConfig::default(),
            zigzag_state: ZigZagState::Idle,
            zigzag_line_a_start: Vec3::zero(),
            zigzag_line_a_end: Vec3::zero(),
            zigzag_line_b_start: Vec3::zero(),
            zigzag_line_b_end: Vec3::zero(),
            zigzag_progress: 0.0,

            guided_nogps_target_quat: Quaternion::identity(),
            guided_nogps_throttle: 0.5,

            smartrtl_nav: SmartRTLNav::new(),
            smartrtl_state: SmartRtlState::Following,
            smartrtl_path_index: 0,
            smartrtl_path_len: 0,
            smartrtl_path_cache: [Vec3::zero(); 300],

            land_detect_timer: 0.0,
            land_motor_shutdown: false,

            autotune_active: false,
            turtle_active: false,

            rtl_cone_slope: 3.0,
            rtl_final_descent_alt: 2.0,
            rtl_armed_yaw: 0.0,
        }
    }

    /// Set the active mode.
    pub fn set_mode(&mut self, mode: FlightModeId, input: &ModeInput) {
        self.active = mode;
        self.enter(input);
    }

    /// Load waypoints for Auto mode.
    pub fn load_mission(&mut self, waypoints: &[Waypoint]) {
        self.wp_nav.load(waypoints);
    }

    /// Set guided mode target.
    pub fn set_guided_target(&mut self, pos: Vec3<NED>, yaw: f32) {
        self.guided_target = pos;
        self.guided_yaw = yaw;
    }

    /// Set circle parameters.
    pub fn set_circle(&mut self, center: Vec3<NED>, radius: f32) {
        self.circle_center = center;
        self.circle_radius = radius;
    }

    pub fn active_mode(&self) -> FlightModeId {
        self.active
    }

    /// Set follow target position (from MAVLink FOLLOW_TARGET or beacon).
    pub fn set_follow_target(&mut self, pos: Vec3<NED>) {
        self.follow_target = pos;
    }

    /// Set ZigZag line A endpoints.
    pub fn set_zigzag_line_a(&mut self, start: Vec3<NED>, end: Vec3<NED>) {
        self.zigzag_line_a_start = start;
        self.zigzag_line_a_end = end;
        // Compute line B offset (perpendicular shift by line_spacing)
        let dx = end.x - start.x;
        let dy = end.y - start.y;
        let len = libm::sqrtf(dx * dx + dy * dy).max(0.001);
        let nx = -dy / len * self.zigzag_config.line_spacing;
        let ny = dx / len * self.zigzag_config.line_spacing;
        self.zigzag_line_b_start = Vec3::new(start.x + nx, start.y + ny, start.z);
        self.zigzag_line_b_end = Vec3::new(end.x + nx, end.y + ny, end.z);
    }

    /// Set GuidedNoGPS attitude target.
    pub fn set_guided_nogps_target(&mut self, quat: Quaternion, throttle: f32) {
        self.guided_nogps_target_quat = quat;
        self.guided_nogps_throttle = throttle;
    }

    /// Record a breadcrumb for SmartRTL. Call periodically during flight.
    pub fn smartrtl_record(&mut self, pos: Vec3<NED>) {
        self.smartrtl_nav.record_position(pos);
    }

    /// Whether the land detector currently reports "landed" (conditions met this tick).
    pub fn land_detected(&self) -> bool {
        self.land_detected
    }

    /// How long (seconds) landing conditions have been sustained.
    pub fn land_complete_timer(&self) -> f32 {
        self.land_complete_timer
    }

    /// Get the flip state (for external mode switching logic).
    pub fn flip_state(&self) -> FlipState {
        self.flip_state
    }

    /// Get the zigzag state.
    pub fn zigzag_state(&self) -> ZigZagState {
        self.zigzag_state
    }

    /// Whether autotune PID calibration is in progress.
    pub fn autotune_active(&self) -> bool {
        self.autotune_active
    }

    /// Whether turtle (flip-over-after-crash) recovery is in progress.
    pub fn turtle_active(&self) -> bool {
        self.turtle_active
    }

    /// Get the smart RTL state.
    pub fn smartrtl_state(&self) -> SmartRtlState {
        self.smartrtl_state
    }

    /// Start zigzag on line A (call after set_zigzag_line_a).
    pub fn zigzag_start(&mut self) {
        self.zigzag_state = ZigZagState::LineA;
        self.zigzag_progress = 0.0;
    }

    /// Switch zigzag to the other line.
    pub fn zigzag_switch_line(&mut self) {
        match self.zigzag_state {
            ZigZagState::LineA => {
                self.zigzag_state = ZigZagState::LineB;
                self.zigzag_progress = 0.0;
            }
            ZigZagState::LineB => {
                self.zigzag_state = ZigZagState::LineA;
                self.zigzag_progress = 0.0;
            }
            ZigZagState::Idle => {} // No lines set yet
        }
    }
}

impl FlightMode for MultirotorModes {
    fn id(&self) -> FlightModeId { self.active }
    fn name(&self) -> &'static str { self.active.name() }

    fn enter(&mut self, input: &ModeInput) {
        match self.active {
            FlightModeId::Stabilize => {
                // Initialize yaw target to current heading so yaw doesn't jump
                self.stabilize_yaw = input.yaw;
            }
            FlightModeId::AltHold => {
                // Capture current altitude as the hold target
                self.althold_target_alt = input.altitude;
                // Initialize yaw target to current heading
                self.stabilize_yaw = input.yaw;
                // Reset altitude PID integrators
                self.pos_controller.reset();
            }
            FlightModeId::RTL => {
                self.rtl_state = RtlState::Climb;
                self.rtl_armed_yaw = input.yaw; // GAP 20
            }
            FlightModeId::Land => {
                self.land_start_alt = input.altitude;
                self.land_complete_timer = 0.0;
                self.land_detected = false;
            }
            FlightModeId::Circle => {
                self.circle_center = input.position;
                self.circle_angle = 0.0;
            }
            FlightModeId::Brake => {
                // Capture current position as hold point
                self.brake_hold_pos = input.position;
                self.brake_hold_yaw = input.yaw;
            }
            FlightModeId::PosHold => {
                // Capture current position as hold point
                self.poshold_hold_pos = input.position;
                self.poshold_hold_yaw = input.yaw;
                self.poshold_target_alt = input.altitude;
            }
            FlightModeId::Throw => {
                self.throw_state = ThrowState::Idle;
                self.throw_detect_timer = 0.0;
                self.throw_position = input.position;
            }
            FlightModeId::Acro => {
                // No state to capture — pure rate pass-through
            }
            FlightModeId::Sport => {
                self.sport_target_alt = input.altitude;
                self.pos_controller.reset();
            }
            FlightModeId::Drift => {
                self.drift_target_alt = input.altitude;
                self.drift_yaw = input.yaw;
                self.pos_controller.reset();
            }
            FlightModeId::Flip => {
                self.flip_state = FlipState::Start;
                self.flip_start_roll = input.yaw; // use yaw as reference for Euler tracking
                let (_r, _p, _y) = input.attitude.to_euler();
                self.flip_start_roll = _r;
                self.flip_accumulated = 0.0;
                self.flip_start_alt = input.altitude;
                self.flip_previous_mode = FlightModeId::Stabilize; // caller should set this
            }
            FlightModeId::Follow => {
                self.follow_target = input.position; // default until target is injected
            }
            FlightModeId::ZigZag => {
                self.zigzag_state = ZigZagState::Idle;
                self.zigzag_progress = 0.0;
            }
            FlightModeId::GuidedNoGPS => {
                self.guided_nogps_target_quat = input.attitude;
                self.guided_nogps_throttle = 0.5;
            }
            FlightModeId::Autotune => {
                // GAP 12: AutoTune — delegates to Stabilize control loop
                // with pid_calibration flag set. The external autotune crate
                // (meridian-autotune) reads this flag and injects test pulses.
                self.autotune_active = true;
                self.stabilize_yaw = input.yaw;
            }
            FlightModeId::Turtle => {
                // GAP 14: Turtle — motor-reversal to flip inverted drone.
                // Commands zero throttle initially; external ESC layer
                // applies motor direction reversal on the appropriate motors
                // based on stick input.
                self.turtle_active = true;
            }
            FlightModeId::SmartRTL => {
                // Build the return path from breadcrumbs
                let path = self.smartrtl_nav.get_return_path();
                self.smartrtl_path_len = path.len();
                for i in 0..path.len().min(300) {
                    self.smartrtl_path_cache[i] = path[i];
                }
                self.smartrtl_path_index = 0;
                self.smartrtl_state = if self.smartrtl_path_len > 0 {
                    SmartRtlState::Following
                } else {
                    SmartRtlState::Descending // no path, fall back
                };
            }
            _ => {}
        }
    }

    fn update(&mut self, input: &ModeInput, dt: f32) -> ModeOutput {
        match self.active {
            FlightModeId::Stabilize => self.update_stabilize(input, dt),
            FlightModeId::AltHold => self.update_althold(input, dt),
            FlightModeId::Loiter => self.update_loiter(input),
            FlightModeId::RTL => self.update_rtl(input, dt),
            FlightModeId::Auto => self.update_auto(input),
            FlightModeId::Guided => self.update_guided(input),
            FlightModeId::Land => self.update_land(input, dt),
            FlightModeId::Circle => self.update_circle(input, dt),
            FlightModeId::Brake => self.update_brake(input),
            FlightModeId::PosHold => self.update_poshold(input, dt),
            FlightModeId::Throw => self.update_throw(input, dt),
            FlightModeId::Acro => self.update_acro(input),
            FlightModeId::Sport => self.update_sport(input, dt),
            FlightModeId::Drift => self.update_drift(input, dt),
            FlightModeId::Flip => self.update_flip(input, dt),
            FlightModeId::Follow => self.update_follow(input),
            FlightModeId::ZigZag => self.update_zigzag(input, dt),
            FlightModeId::GuidedNoGPS => self.update_guided_nogps(),
            FlightModeId::SmartRTL => self.update_smartrtl(input, dt),
            // GAP 12: Autotune — runs Stabilize control loop while autotune_active flag
            // signals the external autotune crate to inject PID test pulses.
            FlightModeId::Autotune => self.update_stabilize(input, dt),
            // GAP 14: Turtle — motor-reversal recovery for crashed/inverted drone.
            // Outputs direct rate commands: zero rates + zero throttle by default.
            // RC sticks control which motors spin (reversed) to flip the vehicle.
            // The ESC/motor layer must interpret turtle_active to reverse motor direction.
            FlightModeId::Turtle => self.update_turtle(input),
            // GAP 13/15/16: stub modes — hold position until full implementation
            FlightModeId::SystemId | FlightModeId::FlowHold
            | FlightModeId::AvoidAdsb => self.update_loiter(input),
            _ => self.update_loiter(input), // default to loiter
        }
    }

    fn exit(&mut self) {
        self.autotune_active = false;
        self.turtle_active = false;
    }

    fn requires_gps(&self) -> bool {
        matches!(self.active,
            FlightModeId::Loiter | FlightModeId::RTL | FlightModeId::Auto
            | FlightModeId::Guided | FlightModeId::Circle | FlightModeId::PosHold
            | FlightModeId::Follow | FlightModeId::SmartRTL | FlightModeId::ZigZag)
        // Acro, Sport, Drift, Flip, Stabilize, AltHold, Brake, GuidedNoGPS do NOT require GPS
    }
}

impl MultirotorModes {
    /// Stabilize mode: direct RC → attitude, throttle passthrough.
    /// No GPS, no altitude hold. Manual fallback — works with only IMU.
    /// Source: ArduCopter/mode_stabilize.cpp
    fn update_stabilize(&mut self, input: &ModeInput, dt: f32) -> ModeOutput {
        let angle_max = self.stabilize_config.angle_max;
        let yaw_rate_max = self.stabilize_config.yaw_rate_max;

        // RC sticks → target lean angles
        let roll_target = input.rc_roll * angle_max;
        let pitch_target = input.rc_pitch * angle_max;

        // Yaw stick → yaw rate, integrate for yaw target
        let yaw_rate = input.rc_yaw * yaw_rate_max;
        self.stabilize_yaw += yaw_rate * dt;
        // Wrap yaw to [-PI, PI]
        self.stabilize_yaw = wrap_pi(self.stabilize_yaw);

        // Throttle passthrough (RC throttle is 0..1)
        let throttle = input.rc_throttle;

        // Build target quaternion from stick-commanded euler angles
        let target_quat = Quaternion::from_euler(roll_target, pitch_target, self.stabilize_yaw);

        ModeOutput::AttitudeTarget {
            quaternion: target_quat,
            throttle,
        }
    }

    /// AltHold mode: altitude hold + manual attitude control.
    /// Pilot controls lean angles via sticks; autopilot holds altitude via Z-axis PID.
    /// GPS not required (uses barometer for altitude).
    /// Source: ArduCopter/mode_althold.cpp
    fn update_althold(&mut self, input: &ModeInput, dt: f32) -> ModeOutput {
        let angle_max = self.stabilize_config.angle_max;
        let yaw_rate_max = self.stabilize_config.yaw_rate_max;

        // ── Attitude: same as Stabilize ──
        let roll_target = input.rc_roll * angle_max;
        let pitch_target = input.rc_pitch * angle_max;

        let yaw_rate = input.rc_yaw * yaw_rate_max;
        self.stabilize_yaw += yaw_rate * dt;
        self.stabilize_yaw = wrap_pi(self.stabilize_yaw);

        // ── Throttle → climb rate → altitude target ──
        // Stick center (0.5) = hold; above = climb; below = descend
        // Deadband: ±0.05 around center
        let deadband = self.althold_config.throttle_deadband;
        let climb_rate = if input.rc_throttle > 0.5 + deadband {
            // Climbing: map (0.55..1.0) → (0..climb_rate_max)
            let frac = (input.rc_throttle - 0.5 - deadband) / (0.5 - deadband);
            frac * self.althold_config.climb_rate_max
        } else if input.rc_throttle < 0.5 - deadband {
            // Descending: map (0..0.45) → (-descend_rate_max..0)
            let frac = (0.5 - deadband - input.rc_throttle) / (0.5 - deadband);
            -frac * self.althold_config.descend_rate_max
        } else {
            0.0 // In deadband — hold altitude
        };

        // Update altitude target based on pilot climb rate
        self.althold_target_alt += climb_rate * dt;
        // Floor at 0 (ground level)
        if self.althold_target_alt < 0.0 {
            self.althold_target_alt = 0.0;
        }

        // Use position controller Z-axis to compute throttle.
        // velocity.z is NED (positive = down).
        let throttle = self.pos_controller.update_altitude(
            self.althold_target_alt,
            input.altitude,
            input.velocity.z,
            dt,
        );

        let target_quat = Quaternion::from_euler(roll_target, pitch_target, self.stabilize_yaw);

        ModeOutput::AttitudeTarget {
            quaternion: target_quat,
            throttle,
        }
    }

    fn update_loiter(&self, input: &ModeInput) -> ModeOutput {
        ModeOutput::PositionTarget {
            position: input.position, // hold current position
            velocity: Vec3::zero(),
            yaw: input.yaw,
        }
    }

    fn update_rtl(&mut self, input: &ModeInput, dt: f32) -> ModeOutput {
        let home_xy = Vec3::<NED>::new(input.home.x, input.home.y, 0.0);
        let pos_xy = Vec3::<NED>::new(input.position.x, input.position.y, 0.0);
        let dist_to_home = (home_xy - pos_xy).length();

        // GAP 18: Cone-slope altitude reduction near home.
        // target_alt = MIN(rtl_altitude, MAX(dist_to_home * cone_slope, rtl_alt_min))
        let mut rtl_target_alt = self.rtl_config.rtl_altitude;
        if self.rtl_config.cone_slope > 0.0 {
            let cone_alt = dist_to_home * self.rtl_config.cone_slope;
            let cone_limited = cone_alt.max(self.rtl_config.rtl_alt_min);
            rtl_target_alt = rtl_target_alt.min(cone_limited);
        }

        // GAP 19: Rally point / terrain selection would go here.
        // In a full implementation, check_nearest_rally() returns a target
        // position that overrides input.home. For now, use home.
        let return_target = input.home;

        match self.rtl_state {
            RtlState::Climb => {
                let target_alt_ned = -rtl_target_alt;
                if input.position.z < target_alt_ned + 1.0 {
                    self.rtl_state = RtlState::Return;
                }
                ModeOutput::PositionTarget {
                    position: Vec3::new(input.position.x, input.position.y, target_alt_ned),
                    velocity: Vec3::zero(),
                    yaw: input.yaw,
                }
            }
            RtlState::Return => {
                let target_alt_ned = -rtl_target_alt;
                if dist_to_home < 3.0 {
                    self.rtl_state = RtlState::Loiter;
                    self.rtl_loiter_start = input.time_since_arm;
                }
                ModeOutput::PositionTarget {
                    position: Vec3::new(return_target.x, return_target.y, target_alt_ned),
                    velocity: Vec3::zero(),
                    yaw: input.yaw,
                }
            }
            RtlState::Loiter => {
                let elapsed = input.time_since_arm - self.rtl_loiter_start;
                if elapsed >= self.rtl_config.loiter_time {
                    // GAP 17: transition to FinalDescent instead of directly to Land
                    self.rtl_state = RtlState::FinalDescent;
                }
                ModeOutput::PositionTarget {
                    position: Vec3::new(return_target.x, return_target.y, -rtl_target_alt),
                    velocity: Vec3::zero(),
                    // GAP 20: yaw to armed heading during loiter
                    yaw: self.rtl_armed_yaw,
                }
            }
            RtlState::FinalDescent => {
                // GAP 17: Descend to RTL_ALT_FINAL and hover before committing to land.
                let final_alt = self.rtl_config.rtl_alt_final;
                let target_alt = (input.altitude - self.rtl_config.land_speed * dt).max(final_alt);
                if input.altitude <= final_alt + 0.5 {
                    self.rtl_state = RtlState::Land;
                    self.land_detect_timer = 0.0;
                    self.land_motor_shutdown = false;
                }
                ModeOutput::PositionTarget {
                    position: Vec3::new(return_target.x, return_target.y, -target_alt),
                    velocity: Vec3::new(0.0, 0.0, self.rtl_config.land_speed),
                    yaw: self.rtl_armed_yaw,
                }
            }
            RtlState::Land => {
                let target_alt = (input.altitude - self.rtl_config.land_speed * dt).max(0.0);

                // Full 8-condition land detection (same as Land mode)
                let vz = input.velocity.z;
                let climb_rate = -vz;
                let low_vspeed = libm::fabsf(climb_rate) < 0.5;
                let not_ascending = climb_rate <= 0.0;

                let all_conditions = input.throttle_at_min
                    && input.throttle_mix_at_min
                    && input.small_angle_error
                    && input.accel_near_1g
                    && low_vspeed
                    && not_ascending
                    && input.rangefinder_near_ground;

                if all_conditions {
                    self.land_detect_timer += dt;
                } else {
                    self.land_detect_timer = 0.0;
                }

                if self.land_detect_timer > 1.0 {
                    self.rtl_state = RtlState::Complete;
                }

                ModeOutput::PositionTarget {
                    position: Vec3::new(return_target.x, return_target.y, -target_alt),
                    velocity: Vec3::new(0.0, 0.0, self.rtl_config.land_speed),
                    yaw: self.rtl_armed_yaw,
                }
            }
            RtlState::Complete => ModeOutput::Idle,
        }
    }

    fn update_auto(&mut self, input: &ModeInput) -> ModeOutput {
        let target = self.wp_nav.update(&input.position, input.time_since_arm);
        match self.wp_nav.status() {
            WaypointStatus::Complete => ModeOutput::PositionTarget {
                position: target,
                velocity: Vec3::zero(),
                yaw: input.yaw,
            },
            _ => ModeOutput::PositionTarget {
                position: target,
                velocity: Vec3::zero(),
                yaw: input.yaw,
            },
        }
    }

    fn update_guided(&self, _input: &ModeInput) -> ModeOutput {
        ModeOutput::PositionTarget {
            position: self.guided_target,
            velocity: Vec3::zero(),
            yaw: self.guided_yaw,
        }
    }

    fn update_land(&mut self, input: &ModeInput, dt: f32) -> ModeOutput {
        let descent_speed = 1.0; // m/s
        let target_alt = (input.altitude - descent_speed * dt).max(0.0);

        // Full 8-condition land detection matching ArduPilot land_detector.cpp:
        //   1. Throttle output at minimum (motors at spin_min)
        //   2. Throttle-RPY mix at minimum (attitude corrections negligible)
        //   3. Attitude error small (< 30 deg from target)
        //   4. Acceleration magnitude near 1g (not in free-fall or vibrating)
        //   5. Vertical speed near zero (|climb_rate| < 0.5 m/s)
        //   6. Not ascending (climb_rate <= 0)
        //   7. Rangefinder shows close to ground (if available)
        //   8. All conditions sustained for at least 1 second
        let vz = input.velocity.z; // NED: positive = descending
        let climb_rate = -vz;      // positive = ascending
        let low_vspeed = libm::fabsf(climb_rate) < 0.5;
        let not_ascending = climb_rate <= 0.0;

        let all_conditions = input.throttle_at_min          // condition 1
            && input.throttle_mix_at_min                    // condition 2
            && input.small_angle_error                      // condition 3
            && input.accel_near_1g                          // condition 4
            && low_vspeed                                   // condition 5
            && not_ascending                                // condition 6
            && input.rangefinder_near_ground;               // condition 7

        if all_conditions {
            self.land_complete_timer += dt;                  // condition 8: sustain timer
            self.land_detected = true;
        } else {
            self.land_complete_timer = 0.0;
            self.land_detected = false;
        }

        // Motor shutdown: when land detector reports landed for >1s, go idle.
        // This gives the spool manager time to ramp: spin_min -> spin_arm -> shutdown.
        // Source: ArduCopter/mode_land.cpp — checks land_complete for motor shutdown.
        if self.land_complete_timer > 1.0 {
            return ModeOutput::Idle;
        }

        ModeOutput::PositionTarget {
            position: Vec3::new(input.position.x, input.position.y, -target_alt),
            velocity: Vec3::new(0.0, 0.0, descent_speed), // descend in NED
            yaw: input.yaw,
        }
    }

    /// Brake mode: hold position captured on entry with zero velocity.
    /// No GPS required — uses last known position.
    /// Source: ArduCopter/mode_brake.cpp
    fn update_brake(&self, _input: &ModeInput) -> ModeOutput {
        ModeOutput::PositionTarget {
            position: self.brake_hold_pos,
            velocity: Vec3::zero(),
            yaw: self.brake_hold_yaw,
        }
    }

    /// PosHold mode: like Loiter but sticks command velocity offset from hold position.
    /// On release (sticks centered), vehicle returns to hold position.
    /// RC channels: roll/pitch = velocity (0-2 m/s), throttle = altitude hold, yaw = yaw rate.
    /// Source: ArduCopter/mode_poshold.cpp
    fn update_poshold(&mut self, input: &ModeInput, dt: f32) -> ModeOutput {
        let deadband = self.poshold_config.stick_deadband;
        let max_vel = self.poshold_config.max_velocity;

        // Roll/pitch sticks → velocity offset (NED: pitch=north, roll=east)
        let vel_n = if libm::fabsf(input.rc_pitch) > deadband {
            input.rc_pitch * max_vel
        } else {
            0.0
        };
        let vel_e = if libm::fabsf(input.rc_roll) > deadband {
            input.rc_roll * max_vel
        } else {
            0.0
        };

        // If sticks are active, update hold position to current position
        // so release returns to where sticks were released
        let sticks_active = libm::fabsf(input.rc_pitch) > deadband
            || libm::fabsf(input.rc_roll) > deadband;
        if sticks_active {
            self.poshold_hold_pos = input.position;
        }

        // Yaw: stick → yaw rate
        let yaw_rate_max = self.stabilize_config.yaw_rate_max;
        let yaw_rate = input.rc_yaw * yaw_rate_max;
        self.poshold_hold_yaw += yaw_rate * dt;
        self.poshold_hold_yaw = wrap_pi(self.poshold_hold_yaw);

        // Throttle → altitude hold (same as AltHold behavior)
        let alt_deadband = self.althold_config.throttle_deadband;
        let climb_rate = if input.rc_throttle > 0.5 + alt_deadband {
            let frac = (input.rc_throttle - 0.5 - alt_deadband) / (0.5 - alt_deadband);
            frac * self.althold_config.climb_rate_max
        } else if input.rc_throttle < 0.5 - alt_deadband {
            let frac = (0.5 - alt_deadband - input.rc_throttle) / (0.5 - alt_deadband);
            -frac * self.althold_config.descend_rate_max
        } else {
            0.0
        };
        self.poshold_target_alt += climb_rate * dt;
        if self.poshold_target_alt < 0.0 {
            self.poshold_target_alt = 0.0;
        }

        ModeOutput::PositionTarget {
            position: Vec3::new(
                self.poshold_hold_pos.x,
                self.poshold_hold_pos.y,
                -self.poshold_target_alt, // NED: negative = up
            ),
            velocity: Vec3::new(vel_n, vel_e, 0.0),
            yaw: self.poshold_hold_yaw,
        }
    }

    /// Throw mode: detect being thrown, stabilize, climb to 5m, switch to Loiter.
    /// State machine: Idle → Detecting → Climbing → Complete.
    /// Detection: accel magnitude > 2g for > 0.5s.
    /// Source: ArduCopter/mode_throw.cpp
    fn update_throw(&mut self, input: &ModeInput, dt: f32) -> ModeOutput {
        // Compute accel magnitude from gyro (we use the body-frame gyro length
        // as a proxy for rotational agitation, combined with the assumption
        // that during a throw the accelerometer reads high).
        // In a real system we'd use raw accel; here we use the velocity
        // derivative as a proxy: we check if the vehicle is experiencing
        // high dynamic acceleration via ground speed changes.
        let accel_mag = input.gyro.length() * 5.0 + input.velocity.length() * 2.0;
        // A more direct approach: check if velocity is building rapidly
        // (throw = high acceleration). We approximate accel from velocity magnitude.
        // For the actual detection, we use a simpler heuristic based on the
        // gyro (rotation) and velocity buildup that indicates a throw.
        let is_high_accel = accel_mag > self.throw_config.accel_threshold;

        match self.throw_state {
            ThrowState::Idle => {
                if is_high_accel {
                    self.throw_state = ThrowState::Detecting;
                    self.throw_detect_timer = dt;
                }
                // Hold idle — motors off
                ModeOutput::Idle
            }
            ThrowState::Detecting => {
                if is_high_accel {
                    self.throw_detect_timer += dt;
                    if self.throw_detect_timer >= self.throw_config.detect_duration {
                        // Throw confirmed — begin stabilize + climb
                        self.throw_state = ThrowState::Climbing;
                        self.throw_position = input.position;
                    }
                } else {
                    // Acceleration dropped — reset
                    self.throw_state = ThrowState::Idle;
                    self.throw_detect_timer = 0.0;
                }
                ModeOutput::Idle
            }
            ThrowState::Climbing => {
                // Target: climb to throw_config.climb_altitude above throw position
                let target_alt = self.throw_config.climb_altitude;
                let target_z = self.throw_position.z - target_alt; // NED: up = negative z

                if input.altitude >= target_alt - 0.5 {
                    // Reached target altitude — switch to complete (caller switches to Loiter)
                    self.throw_state = ThrowState::Complete;
                }

                ModeOutput::PositionTarget {
                    position: Vec3::new(
                        self.throw_position.x,
                        self.throw_position.y,
                        target_z,
                    ),
                    velocity: Vec3::new(0.0, 0.0, -1.5), // climb at 1.5 m/s
                    yaw: input.yaw,
                }
            }
            ThrowState::Complete => {
                // Hold position at current altitude — caller should switch to Loiter
                ModeOutput::PositionTarget {
                    position: input.position,
                    velocity: Vec3::zero(),
                    yaw: input.yaw,
                }
            }
        }
    }

    /// Get the current throw state (for external mode switching logic).
    pub fn throw_state(&self) -> ThrowState {
        self.throw_state
    }

    /// Acro mode: pure rate control, no self-leveling.
    /// RC sticks map directly to body-rate targets.
    /// No GPS needed, no altitude hold. For experienced pilots only.
    /// Source: ArduCopter/mode_acro.cpp
    fn update_acro(&self, input: &ModeInput) -> ModeOutput {
        let rp_max = self.acro_config.roll_pitch_rate_max;
        let yaw_max = self.acro_config.yaw_rate_max;

        ModeOutput::RateTarget {
            roll_rate: input.rc_roll * rp_max,
            pitch_rate: input.rc_pitch * rp_max,
            yaw_rate: input.rc_yaw * yaw_max,
            throttle: input.rc_throttle,
        }
    }

    /// Sport mode: rate-based roll/pitch with altitude hold.
    /// Like AltHold but sticks command angular rate instead of angle.
    /// Allows rolls and flips while maintaining altitude.
    /// Source: ArduCopter/mode_sport.cpp
    fn update_sport(&mut self, input: &ModeInput, dt: f32) -> ModeOutput {
        let rp_max = self.sport_config.roll_pitch_rate_max;
        let yaw_max = self.sport_config.yaw_rate_max;

        // Throttle → altitude hold (same as AltHold)
        let deadband = self.althold_config.throttle_deadband;
        let climb_rate = if input.rc_throttle > 0.5 + deadband {
            let frac = (input.rc_throttle - 0.5 - deadband) / (0.5 - deadband);
            frac * self.althold_config.climb_rate_max
        } else if input.rc_throttle < 0.5 - deadband {
            let frac = (0.5 - deadband - input.rc_throttle) / (0.5 - deadband);
            -frac * self.althold_config.descend_rate_max
        } else {
            0.0
        };

        self.sport_target_alt += climb_rate * dt;
        if self.sport_target_alt < 0.0 {
            self.sport_target_alt = 0.0;
        }

        let throttle = self.pos_controller.update_altitude(
            self.sport_target_alt,
            input.altitude,
            input.velocity.z,
            dt,
        );

        ModeOutput::RateTarget {
            roll_rate: input.rc_roll * rp_max,
            pitch_rate: input.rc_pitch * rp_max,
            yaw_rate: input.rc_yaw * yaw_max,
            throttle,
        }
    }

    /// Drift mode: airplane-like coordinated turns.
    /// Roll input causes coordinated turn (yaw follows roll).
    /// Throttle controls altitude hold. Pitch controls forward/back.
    /// Source: ArduCopter/mode_drift.cpp
    fn update_drift(&mut self, input: &ModeInput, dt: f32) -> ModeOutput {
        let roll_max = self.drift_config.roll_max;
        let coupling = self.drift_config.yaw_coupling;

        // Roll stick → target roll angle
        let roll_target = input.rc_roll * roll_max;

        // Coordinated turn: yaw rate proportional to roll angle
        let yaw_rate = roll_target * coupling;
        self.drift_yaw += yaw_rate * dt;
        self.drift_yaw = wrap_pi(self.drift_yaw);

        // Pitch stick → forward lean angle (like stabilize)
        let pitch_target = input.rc_pitch * self.stabilize_config.angle_max;

        // Throttle → altitude hold
        let deadband = self.althold_config.throttle_deadband;
        let climb_rate = if input.rc_throttle > 0.5 + deadband {
            let frac = (input.rc_throttle - 0.5 - deadband) / (0.5 - deadband);
            frac * self.althold_config.climb_rate_max
        } else if input.rc_throttle < 0.5 - deadband {
            let frac = (0.5 - deadband - input.rc_throttle) / (0.5 - deadband);
            -frac * self.althold_config.descend_rate_max
        } else {
            0.0
        };
        self.drift_target_alt += climb_rate * dt;
        if self.drift_target_alt < 0.0 {
            self.drift_target_alt = 0.0;
        }

        let throttle = self.pos_controller.update_altitude(
            self.drift_target_alt,
            input.altitude,
            input.velocity.z,
            dt,
        );

        let target_quat = Quaternion::from_euler(roll_target, pitch_target, self.drift_yaw);

        ModeOutput::AttitudeTarget {
            quaternion: target_quat,
            throttle,
        }
    }

    /// Flip mode: programmatic flip maneuver via state machine.
    /// Start → Flipping (400 deg/s roll) → Recover (level out) → Complete.
    /// Source: ArduCopter/mode_flip.cpp
    fn update_flip(&mut self, input: &ModeInput, dt: f32) -> ModeOutput {
        let flip_rate = self.flip_config.flip_rate;
        let flip_angle = self.flip_config.flip_angle;

        match self.flip_state {
            FlipState::Start => {
                // Capture starting attitude and begin the flip
                let (r, _p, _y) = input.attitude.to_euler();
                self.flip_start_roll = r;
                self.flip_accumulated = 0.0;
                self.flip_start_alt = input.altitude;
                self.flip_state = FlipState::Flipping;
                // Command initial rate
                ModeOutput::RateTarget {
                    roll_rate: flip_rate,
                    pitch_rate: 0.0,
                    yaw_rate: 0.0,
                    throttle: 0.5, // maintain hover-ish throttle
                }
            }
            FlipState::Flipping => {
                // Accumulate rotation
                self.flip_accumulated += flip_rate * dt;
                if self.flip_accumulated >= flip_angle {
                    self.flip_state = FlipState::Recover;
                }
                ModeOutput::RateTarget {
                    roll_rate: flip_rate,
                    pitch_rate: 0.0,
                    yaw_rate: 0.0,
                    throttle: 0.3, // reduce throttle during inverted phase
                }
            }
            FlipState::Recover => {
                // Level out: command attitude back to level
                let target_quat = Quaternion::from_euler(
                    0.0,
                    0.0,
                    input.yaw, // preserve yaw heading
                );
                // Check if we're close to level
                let (r, p, _y) = input.attitude.to_euler();
                if libm::fabsf(r) < 0.15 && libm::fabsf(p) < 0.15 {
                    self.flip_state = FlipState::Complete;
                }
                // Boost throttle to recover altitude
                ModeOutput::AttitudeTarget {
                    quaternion: target_quat,
                    throttle: 0.7, // extra throttle to recover altitude
                }
            }
            FlipState::Complete => {
                // Hold level, caller should switch back to previous mode
                let target_quat = Quaternion::from_euler(0.0, 0.0, input.yaw);
                ModeOutput::AttitudeTarget {
                    quaternion: target_quat,
                    throttle: 0.5,
                }
            }
        }
    }

    /// Follow mode: track a target position with configurable offset.
    /// Target position is set externally via set_follow_target().
    /// Source: ArduCopter/mode_follow.cpp
    fn update_follow(&self, input: &ModeInput) -> ModeOutput {
        let target_pos = Vec3::new(
            self.follow_target.x + self.follow_config.offset.x,
            self.follow_target.y + self.follow_config.offset.y,
            self.follow_target.z + self.follow_config.offset.z,
        );
        // Face toward the target
        let dx = self.follow_target.x - input.position.x;
        let dy = self.follow_target.y - input.position.y;
        let yaw = libm::atan2f(dy, dx);

        ModeOutput::PositionTarget {
            position: target_pos,
            velocity: Vec3::zero(),
            yaw,
        }
    }

    /// ZigZag mode: agricultural spray pattern between two parallel lines.
    /// Fly along line A, command switches to line B, repeat.
    /// Source: ArduCopter/mode_zigzag.cpp
    fn update_zigzag(&mut self, input: &ModeInput, dt: f32) -> ModeOutput {
        match self.zigzag_state {
            ZigZagState::Idle => {
                // No lines set — hold position
                ModeOutput::PositionTarget {
                    position: input.position,
                    velocity: Vec3::zero(),
                    yaw: input.yaw,
                }
            }
            ZigZagState::LineA => {
                let start = self.zigzag_line_a_start;
                let end = self.zigzag_line_a_end;
                self.update_zigzag_line(input, dt, start, end)
            }
            ZigZagState::LineB => {
                let start = self.zigzag_line_b_start;
                let end = self.zigzag_line_b_end;
                self.update_zigzag_line(input, dt, start, end)
            }
        }
    }

    /// Helper: fly along a line from start to end at configured speed.
    fn update_zigzag_line(&mut self, input: &ModeInput, dt: f32,
                          start: Vec3<NED>, end: Vec3<NED>) -> ModeOutput {
        let dx = end.x - start.x;
        let dy = end.y - start.y;
        let line_len = libm::sqrtf(dx * dx + dy * dy).max(0.001);

        // Advance progress along the line
        self.zigzag_progress += self.zigzag_config.speed * dt / line_len;
        if self.zigzag_progress > 1.0 {
            self.zigzag_progress = 1.0;
        }

        let target = Vec3::new(
            start.x + dx * self.zigzag_progress,
            start.y + dy * self.zigzag_progress,
            start.z, // maintain altitude
        );

        // Face along the line direction
        let yaw = libm::atan2f(dy, dx);

        ModeOutput::PositionTarget {
            position: target,
            velocity: Vec3::new(
                dx / line_len * self.zigzag_config.speed,
                dy / line_len * self.zigzag_config.speed,
                0.0,
            ),
            yaw,
        }
    }

    /// GuidedNoGPS mode: accepts attitude targets without GPS.
    /// Like Guided but only attitude commands, not position.
    /// For indoor / GPS-denied guided flight.
    /// Source: ArduCopter/mode_guided_nogps.cpp
    fn update_guided_nogps(&self) -> ModeOutput {
        ModeOutput::AttitudeTarget {
            quaternion: self.guided_nogps_target_quat,
            throttle: self.guided_nogps_throttle,
        }
    }

    /// SmartRTL mode: return along recorded breadcrumb path.
    /// Fly waypoints in reverse order, then descend and land.
    /// Falls back to regular RTL descent if no path recorded.
    /// Source: ArduCopter/mode_smartrtl.cpp
    fn update_smartrtl(&mut self, input: &ModeInput, dt: f32) -> ModeOutput {
        match self.smartrtl_state {
            SmartRtlState::Following => {
                if self.smartrtl_path_index >= self.smartrtl_path_len {
                    // Exhausted all waypoints — descend
                    self.smartrtl_state = SmartRtlState::Descending;
                    return self.update_smartrtl(input, dt);
                }

                let target = self.smartrtl_path_cache[self.smartrtl_path_index];
                let dx = target.x - input.position.x;
                let dy = target.y - input.position.y;
                let dz = target.z - input.position.z;
                let dist = libm::sqrtf(dx * dx + dy * dy + dz * dz);

                if dist < 3.0 {
                    // Reached this waypoint, advance to next
                    self.smartrtl_path_index += 1;
                }

                ModeOutput::PositionTarget {
                    position: target,
                    velocity: Vec3::zero(),
                    yaw: libm::atan2f(dy, dx),
                }
            }
            SmartRtlState::Descending => {
                // Descend at 1 m/s to ground
                let target_alt = (input.altitude - 1.0 * dt).max(0.0);
                if input.altitude < 0.3 {
                    self.smartrtl_state = SmartRtlState::Complete;
                }
                ModeOutput::PositionTarget {
                    position: Vec3::new(input.position.x, input.position.y, -target_alt),
                    velocity: Vec3::new(0.0, 0.0, 1.0), // descend in NED
                    yaw: input.yaw,
                }
            }
            SmartRtlState::Complete => ModeOutput::Idle,
        }
    }

    /// Turtle mode: flip crashed/inverted drone back upright.
    /// Motor ESCs must be in 3D (bidirectional) mode.
    /// Stick input commands which direction to flip: the motor layer
    /// reverses the appropriate motors based on the commanded roll/pitch.
    /// Source: ArduCopter/mode_turtle.cpp
    fn update_turtle(&self, input: &ModeInput) -> ModeOutput {
        // Sticks → direct motor commands via rate target.
        // Roll/pitch sticks select the flip direction.
        // Throttle stick gates whether motors actually spin (deadband at 0.15).
        // The motor mixer + turtle_active flag tells the ESC layer to
        // reverse motor direction on the appropriate motors.
        let throttle = if input.rc_throttle > 0.15 {
            input.rc_throttle
        } else {
            0.0
        };
        ModeOutput::RateTarget {
            roll_rate: input.rc_roll * 10.0,   // large rate for flip authority
            pitch_rate: input.rc_pitch * 10.0,
            yaw_rate: 0.0,
            throttle,
        }
    }

    fn update_circle(&mut self, input: &ModeInput, dt: f32) -> ModeOutput {
        self.circle_angle += self.circle_rate * dt;
        let target_n = self.circle_center.x + self.circle_radius * libm::cosf(self.circle_angle);
        let target_e = self.circle_center.y + self.circle_radius * libm::sinf(self.circle_angle);

        ModeOutput::PositionTarget {
            position: Vec3::new(target_n, target_e, self.circle_center.z),
            velocity: Vec3::zero(),
            yaw: self.circle_angle + core::f32::consts::PI / 2.0, // face tangent
        }
    }
}

/// Wrap angle to [-PI, PI] range. Uses `libm` for `no_std` compatibility.
fn wrap_pi(mut angle: f32) -> f32 {
    use core::f32::consts::PI;
    const TWO_PI: f32 = 2.0 * PI;
    angle = libm::fmodf(angle + PI, TWO_PI);
    if angle < 0.0 {
        angle += TWO_PI;
    }
    angle - PI
}

#[cfg(test)]
mod tests {
    use super::*;
    use meridian_math::frames::Body;

    const TOL: f32 = 1e-4;
    const DT: f32 = 0.0025; // 400Hz

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < TOL
    }

    fn default_input() -> ModeInput {
        ModeInput {
            attitude: Quaternion::identity(),
            position: Vec3::zero(),
            velocity: Vec3::zero(),
            gyro: Vec3::<Body>::zero(),
            altitude: 10.0,
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

    // ── wrap_pi tests ──

    #[test]
    fn test_wrap_pi_no_change() {
        assert!(approx_eq(wrap_pi(1.0), 1.0));
        assert!(approx_eq(wrap_pi(-1.0), -1.0));
        assert!(approx_eq(wrap_pi(0.0), 0.0));
    }

    #[test]
    fn test_wrap_pi_overflow() {
        use core::f32::consts::PI;
        // 2*PI should wrap to ~0
        assert!(approx_eq(wrap_pi(2.0 * PI), 0.0));
        // 3*PI should wrap to PI (or -PI, they're equivalent)
        let w = wrap_pi(3.0 * PI);
        assert!(approx_eq(w.abs(), PI));
    }

    // ── Stabilize mode tests ──

    #[test]
    fn test_stabilize_sticks_centered_produces_level_attitude() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::Stabilize, &input);

        let output = modes.update(&input, DT);
        match output {
            ModeOutput::AttitudeTarget { quaternion, throttle } => {
                // Centered sticks → identity-ish quaternion (level, yaw=0)
                let (roll, pitch, _yaw) = quaternion.to_euler();
                assert!(approx_eq(roll, 0.0));
                assert!(approx_eq(pitch, 0.0));
                // Throttle = rc_throttle passthrough = 0.5
                assert!(approx_eq(throttle, 0.5));
            }
            _ => panic!("Expected AttitudeTarget from Stabilize mode"),
        }
    }

    #[test]
    fn test_stabilize_full_roll_stick() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.rc_roll = 1.0; // full right
        modes.set_mode(FlightModeId::Stabilize, &input);

        let output = modes.update(&input, DT);
        match output {
            ModeOutput::AttitudeTarget { quaternion, .. } => {
                let (roll, pitch, _yaw) = quaternion.to_euler();
                let angle_max = StabilizeConfig::default().angle_max;
                assert!(approx_eq(roll, angle_max));
                assert!(approx_eq(pitch, 0.0));
            }
            _ => panic!("Expected AttitudeTarget"),
        }
    }

    #[test]
    fn test_stabilize_full_pitch_stick() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.rc_pitch = -1.0; // full forward (nose down)
        modes.set_mode(FlightModeId::Stabilize, &input);

        let output = modes.update(&input, DT);
        match output {
            ModeOutput::AttitudeTarget { quaternion, .. } => {
                let (roll, pitch, _yaw) = quaternion.to_euler();
                let angle_max = StabilizeConfig::default().angle_max;
                assert!(approx_eq(roll, 0.0));
                assert!(approx_eq(pitch, -angle_max));
            }
            _ => panic!("Expected AttitudeTarget"),
        }
    }

    #[test]
    fn test_stabilize_yaw_integrates() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.rc_yaw = 1.0; // full right yaw
        modes.set_mode(FlightModeId::Stabilize, &input);

        // Run several ticks
        let yaw_rate_max = StabilizeConfig::default().yaw_rate_max;
        for _ in 0..100 {
            modes.update(&input, DT);
        }
        // After 100 ticks at DT=0.0025 → 0.25s → yaw should have moved.
        // Check by running one more tick and inspecting
        let output = modes.update(&input, DT);
        match output {
            ModeOutput::AttitudeTarget { quaternion, .. } => {
                let (_r, _p, yaw) = quaternion.to_euler();
                // Total yaw = yaw_rate_max * 101 * DT (101 ticks total)
                let total = wrap_pi(yaw_rate_max * 101.0 * DT);
                assert!((yaw - total).abs() < 0.02);
            }
            _ => panic!("Expected AttitudeTarget"),
        }
    }

    #[test]
    fn test_stabilize_throttle_passthrough() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.rc_throttle = 0.0; // zero throttle
        modes.set_mode(FlightModeId::Stabilize, &input);

        let output = modes.update(&input, DT);
        match output {
            ModeOutput::AttitudeTarget { throttle, .. } => {
                assert!(approx_eq(throttle, 0.0));
            }
            _ => panic!("Expected AttitudeTarget"),
        }

        input.rc_throttle = 1.0; // full throttle
        let output = modes.update(&input, DT);
        match output {
            ModeOutput::AttitudeTarget { throttle, .. } => {
                assert!(approx_eq(throttle, 1.0));
            }
            _ => panic!("Expected AttitudeTarget"),
        }
    }

    #[test]
    fn test_stabilize_no_gps_required() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::Stabilize, &input);
        assert!(!modes.requires_gps());
    }

    // ── AltHold mode tests ──

    #[test]
    fn test_althold_centered_throttle_holds_altitude() {
        let mut modes = MultirotorModes::new();
        let input = default_input(); // rc_throttle = 0.5, altitude = 10.0
        modes.set_mode(FlightModeId::AltHold, &input);

        // With centered throttle, target alt shouldn't change
        let output = modes.update(&input, DT);
        match output {
            ModeOutput::AttitudeTarget { quaternion, throttle } => {
                let (roll, pitch, _yaw) = quaternion.to_euler();
                assert!(approx_eq(roll, 0.0));
                assert!(approx_eq(pitch, 0.0));
                // Throttle should be near hover (pos controller output).
                // At zero error, throttle ~ hover_throttle
                assert!(throttle > 0.1 && throttle < 0.9);
            }
            _ => panic!("Expected AttitudeTarget from AltHold mode"),
        }
        // Target alt should still be 10.0 (deadband = no climb)
        assert!(approx_eq(modes.althold_target_alt, 10.0));
    }

    #[test]
    fn test_althold_climb_stick_increases_target() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.rc_throttle = 0.8; // above center + deadband → climbing
        modes.set_mode(FlightModeId::AltHold, &input);

        let initial_alt = modes.althold_target_alt;
        // Run some ticks
        for _ in 0..100 {
            modes.update(&input, DT);
        }
        assert!(modes.althold_target_alt > initial_alt);
    }

    #[test]
    fn test_althold_descend_stick_decreases_target() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.rc_throttle = 0.2; // below center - deadband → descending
        modes.set_mode(FlightModeId::AltHold, &input);

        let initial_alt = modes.althold_target_alt;
        for _ in 0..100 {
            modes.update(&input, DT);
        }
        assert!(modes.althold_target_alt < initial_alt);
    }

    #[test]
    fn test_althold_deadband() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        // Just inside deadband (0.5 ± 0.05)
        input.rc_throttle = 0.52;
        modes.set_mode(FlightModeId::AltHold, &input);

        let initial_alt = modes.althold_target_alt;
        for _ in 0..100 {
            modes.update(&input, DT);
        }
        // Should not have moved — stick is within deadband
        assert!(approx_eq(modes.althold_target_alt, initial_alt));
    }

    #[test]
    fn test_althold_no_gps_required() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::AltHold, &input);
        assert!(!modes.requires_gps());
    }

    #[test]
    fn test_althold_target_alt_floors_at_zero() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.altitude = 1.0; // low altitude
        input.rc_throttle = 0.0; // full descend
        modes.set_mode(FlightModeId::AltHold, &input);

        // Run many ticks to push target below ground
        for _ in 0..2000 {
            modes.update(&input, DT);
        }
        // Target should be floored at 0
        assert!(modes.althold_target_alt >= 0.0);
    }

    #[test]
    fn test_althold_attitude_from_sticks() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.rc_roll = 0.5;
        input.rc_pitch = -0.5;
        modes.set_mode(FlightModeId::AltHold, &input);

        let output = modes.update(&input, DT);
        match output {
            ModeOutput::AttitudeTarget { quaternion, .. } => {
                let (roll, pitch, _yaw) = quaternion.to_euler();
                let angle_max = StabilizeConfig::default().angle_max;
                assert!(approx_eq(roll, 0.5 * angle_max));
                assert!(approx_eq(pitch, -0.5 * angle_max));
            }
            _ => panic!("Expected AttitudeTarget"),
        }
    }

    #[test]
    fn test_enter_stabilize_captures_yaw() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.yaw = 1.5;
        modes.set_mode(FlightModeId::Stabilize, &input);
        assert!(approx_eq(modes.stabilize_yaw, 1.5));
    }

    #[test]
    fn test_enter_althold_captures_altitude_and_yaw() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.altitude = 25.0;
        input.yaw = -0.8;
        modes.set_mode(FlightModeId::AltHold, &input);
        assert!(approx_eq(modes.althold_target_alt, 25.0));
        assert!(approx_eq(modes.stabilize_yaw, -0.8));
    }

    // ── Brake mode tests ──

    #[test]
    fn test_brake_captures_position_on_enter() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.position = Vec3::new(10.0, 20.0, -5.0);
        input.yaw = 1.2;
        modes.set_mode(FlightModeId::Brake, &input);

        assert!(approx_eq(modes.brake_hold_pos.x, 10.0));
        assert!(approx_eq(modes.brake_hold_pos.y, 20.0));
        assert!(approx_eq(modes.brake_hold_pos.z, -5.0));
        assert!(approx_eq(modes.brake_hold_yaw, 1.2));
    }

    #[test]
    fn test_brake_holds_position_with_zero_velocity() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.position = Vec3::new(5.0, -3.0, -10.0);
        input.yaw = 0.5;
        modes.set_mode(FlightModeId::Brake, &input);

        // Even if vehicle drifts, output stays at captured hold point
        input.position = Vec3::new(6.0, -2.0, -9.0);
        let output = modes.update(&input, DT);
        match output {
            ModeOutput::PositionTarget { position, velocity, yaw } => {
                assert!(approx_eq(position.x, 5.0));
                assert!(approx_eq(position.y, -3.0));
                assert!(approx_eq(position.z, -10.0));
                assert!(approx_eq(velocity.x, 0.0));
                assert!(approx_eq(velocity.y, 0.0));
                assert!(approx_eq(velocity.z, 0.0));
                assert!(approx_eq(yaw, 0.5));
            }
            _ => panic!("Expected PositionTarget from Brake mode"),
        }
    }

    #[test]
    fn test_brake_no_gps_required() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::Brake, &input);
        assert!(!modes.requires_gps());
    }

    #[test]
    fn test_brake_stable_over_multiple_ticks() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.position = Vec3::new(1.0, 2.0, -3.0);
        modes.set_mode(FlightModeId::Brake, &input);

        for _ in 0..100 {
            let output = modes.update(&input, DT);
            match output {
                ModeOutput::PositionTarget { position, .. } => {
                    assert!(approx_eq(position.x, 1.0));
                    assert!(approx_eq(position.y, 2.0));
                    assert!(approx_eq(position.z, -3.0));
                }
                _ => panic!("Expected PositionTarget"),
            }
        }
    }

    // ── PosHold mode tests ──

    #[test]
    fn test_poshold_captures_position_on_enter() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.position = Vec3::new(7.0, 8.0, -15.0);
        input.yaw = -0.3;
        input.altitude = 15.0;
        modes.set_mode(FlightModeId::PosHold, &input);

        assert!(approx_eq(modes.poshold_hold_pos.x, 7.0));
        assert!(approx_eq(modes.poshold_hold_pos.y, 8.0));
        assert!(approx_eq(modes.poshold_hold_yaw, -0.3));
        assert!(approx_eq(modes.poshold_target_alt, 15.0));
    }

    #[test]
    fn test_poshold_sticks_centered_holds_position() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.position = Vec3::new(5.0, 5.0, -10.0);
        input.altitude = 10.0;
        modes.set_mode(FlightModeId::PosHold, &input);

        // Centered sticks (0.0) → no velocity, hold position
        let output = modes.update(&input, DT);
        match output {
            ModeOutput::PositionTarget { position, velocity, .. } => {
                assert!(approx_eq(position.x, 5.0));
                assert!(approx_eq(position.y, 5.0));
                assert!(approx_eq(velocity.x, 0.0));
                assert!(approx_eq(velocity.y, 0.0));
            }
            _ => panic!("Expected PositionTarget from PosHold mode"),
        }
    }

    #[test]
    fn test_poshold_sticks_command_velocity() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.position = Vec3::new(0.0, 0.0, -10.0);
        input.altitude = 10.0;
        modes.set_mode(FlightModeId::PosHold, &input);

        // Push sticks: pitch forward, roll right
        input.rc_pitch = 0.5;  // north velocity
        input.rc_roll = 0.5;   // east velocity
        let output = modes.update(&input, DT);
        match output {
            ModeOutput::PositionTarget { velocity, .. } => {
                let max_vel = PosHoldConfig::default().max_velocity;
                assert!(approx_eq(velocity.x, 0.5 * max_vel));
                assert!(approx_eq(velocity.y, 0.5 * max_vel));
            }
            _ => panic!("Expected PositionTarget"),
        }
    }

    #[test]
    fn test_poshold_requires_gps() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::PosHold, &input);
        assert!(modes.requires_gps());
    }

    #[test]
    fn test_poshold_throttle_adjusts_altitude() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.altitude = 10.0;
        modes.set_mode(FlightModeId::PosHold, &input);

        let initial_alt = modes.poshold_target_alt;
        input.rc_throttle = 0.8; // climb
        for _ in 0..100 {
            modes.update(&input, DT);
        }
        assert!(modes.poshold_target_alt > initial_alt);
    }

    #[test]
    fn test_poshold_yaw_integrates() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        modes.set_mode(FlightModeId::PosHold, &input);

        input.rc_yaw = 1.0; // full yaw rate
        for _ in 0..100 {
            modes.update(&input, DT);
        }
        // Yaw should have changed from 0
        assert!(modes.poshold_hold_yaw.abs() > 0.01);
    }

    // ── Throw mode tests ──

    #[test]
    fn test_throw_initial_state_idle() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::Throw, &input);
        assert_eq!(modes.throw_state(), ThrowState::Idle);
    }

    #[test]
    fn test_throw_idle_returns_idle_output() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::Throw, &input);

        let output = modes.update(&input, DT);
        match output {
            ModeOutput::Idle => {}
            _ => panic!("Expected Idle output in Throw Idle state"),
        }
    }

    #[test]
    fn test_throw_no_accel_stays_idle() {
        let mut modes = MultirotorModes::new();
        let input = default_input(); // zero gyro, zero velocity
        modes.set_mode(FlightModeId::Throw, &input);

        for _ in 0..1000 {
            modes.update(&input, DT);
        }
        assert_eq!(modes.throw_state(), ThrowState::Idle);
    }

    #[test]
    fn test_throw_high_accel_transitions_to_detecting() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        // Simulate throw: high gyro + velocity
        input.gyro = Vec3::<Body>::new(5.0, 3.0, 2.0); // high rotation
        input.velocity = Vec3::<NED>::new(5.0, 0.0, -3.0); // moving fast
        modes.set_mode(FlightModeId::Throw, &input);

        modes.update(&input, DT);
        // Should have moved to Detecting
        assert_eq!(modes.throw_state(), ThrowState::Detecting);
    }

    #[test]
    fn test_throw_sustained_accel_confirms_throw() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.altitude = 0.5; // low altitude — hasn't reached climb target
        input.gyro = Vec3::<Body>::new(5.0, 3.0, 2.0);
        input.velocity = Vec3::<NED>::new(5.0, 0.0, -3.0);
        modes.set_mode(FlightModeId::Throw, &input);

        // Run for enough time to confirm (0.5s at 400Hz = 200 ticks)
        for _ in 0..250 {
            modes.update(&input, DT);
        }
        // Should have confirmed throw and started climbing
        assert_eq!(modes.throw_state(), ThrowState::Climbing);
    }

    #[test]
    fn test_throw_brief_accel_resets_to_idle() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.gyro = Vec3::<Body>::new(5.0, 3.0, 2.0);
        input.velocity = Vec3::<NED>::new(5.0, 0.0, -3.0);
        modes.set_mode(FlightModeId::Throw, &input);

        // Brief accel for a few ticks
        for _ in 0..10 {
            modes.update(&input, DT);
        }
        assert_eq!(modes.throw_state(), ThrowState::Detecting);

        // Accel drops
        input.gyro = Vec3::<Body>::zero();
        input.velocity = Vec3::<NED>::zero();
        modes.update(&input, DT);
        assert_eq!(modes.throw_state(), ThrowState::Idle);
    }

    #[test]
    fn test_throw_climbing_outputs_position_target() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.gyro = Vec3::<Body>::new(5.0, 3.0, 2.0);
        input.velocity = Vec3::<NED>::new(5.0, 0.0, -3.0);
        input.altitude = 1.0;
        modes.set_mode(FlightModeId::Throw, &input);

        // Get to climbing state
        for _ in 0..250 {
            modes.update(&input, DT);
        }
        assert_eq!(modes.throw_state(), ThrowState::Climbing);

        // Now check that climbing produces a position target
        let output = modes.update(&input, DT);
        match output {
            ModeOutput::PositionTarget { velocity, .. } => {
                // Should command upward velocity (NED: negative z = up)
                assert!(velocity.z < 0.0, "Should climb: vz={}", velocity.z);
            }
            _ => panic!("Expected PositionTarget during Climbing"),
        }
    }

    #[test]
    fn test_throw_reaches_altitude_completes() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.gyro = Vec3::<Body>::new(5.0, 3.0, 2.0);
        input.velocity = Vec3::<NED>::new(5.0, 0.0, -3.0);
        input.altitude = 0.0;
        modes.set_mode(FlightModeId::Throw, &input);

        // Get to climbing state
        for _ in 0..250 {
            modes.update(&input, DT);
        }
        assert_eq!(modes.throw_state(), ThrowState::Climbing);

        // Simulate reaching target altitude (5m default)
        input.altitude = 5.0;
        input.gyro = Vec3::<Body>::zero();
        input.velocity = Vec3::<NED>::zero();
        modes.update(&input, DT);
        assert_eq!(modes.throw_state(), ThrowState::Complete);
    }

    #[test]
    fn test_throw_complete_holds_position() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.gyro = Vec3::<Body>::new(5.0, 3.0, 2.0);
        input.velocity = Vec3::<NED>::new(5.0, 0.0, -3.0);
        modes.set_mode(FlightModeId::Throw, &input);

        for _ in 0..250 {
            modes.update(&input, DT);
        }

        input.altitude = 5.0;
        input.gyro = Vec3::<Body>::zero();
        input.velocity = Vec3::<NED>::zero();
        input.position = Vec3::new(1.0, 2.0, -5.0);
        modes.update(&input, DT);
        assert_eq!(modes.throw_state(), ThrowState::Complete);

        let output = modes.update(&input, DT);
        match output {
            ModeOutput::PositionTarget { position, velocity, .. } => {
                assert!(approx_eq(position.x, 1.0));
                assert!(approx_eq(position.y, 2.0));
                assert!(approx_eq(velocity.x, 0.0));
                assert!(approx_eq(velocity.y, 0.0));
                assert!(approx_eq(velocity.z, 0.0));
            }
            _ => panic!("Expected PositionTarget in Complete state"),
        }
    }

    #[test]
    fn test_throw_enter_resets_state() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.altitude = 0.5; // low altitude — hasn't reached climb target
        input.gyro = Vec3::<Body>::new(5.0, 3.0, 2.0);
        input.velocity = Vec3::<NED>::new(5.0, 0.0, -3.0);
        modes.set_mode(FlightModeId::Throw, &input);

        // Advance to Climbing
        for _ in 0..250 {
            modes.update(&input, DT);
        }
        assert_eq!(modes.throw_state(), ThrowState::Climbing);

        // Re-enter Throw mode — should reset to Idle
        modes.set_mode(FlightModeId::Throw, &input);
        assert_eq!(modes.throw_state(), ThrowState::Idle);
    }

    // ── Acro mode tests ──

    #[test]
    fn test_acro_sticks_centered_zero_rates() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::Acro, &input);

        let output = modes.update(&input, DT);
        match output {
            ModeOutput::RateTarget { roll_rate, pitch_rate, yaw_rate, throttle } => {
                assert!(approx_eq(roll_rate, 0.0));
                assert!(approx_eq(pitch_rate, 0.0));
                assert!(approx_eq(yaw_rate, 0.0));
                assert!(approx_eq(throttle, 0.5));
            }
            _ => panic!("Expected RateTarget from Acro mode"),
        }
    }

    #[test]
    fn test_acro_full_sticks_max_rates() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.rc_roll = 1.0;
        input.rc_pitch = -1.0;
        input.rc_yaw = 1.0;
        input.rc_throttle = 0.8;
        modes.set_mode(FlightModeId::Acro, &input);

        let output = modes.update(&input, DT);
        match output {
            ModeOutput::RateTarget { roll_rate, pitch_rate, yaw_rate, throttle } => {
                let rp_max = AcroConfig::default().roll_pitch_rate_max;
                let y_max = AcroConfig::default().yaw_rate_max;
                assert!(approx_eq(roll_rate, rp_max));
                assert!(approx_eq(pitch_rate, -rp_max));
                assert!(approx_eq(yaw_rate, y_max));
                assert!(approx_eq(throttle, 0.8));
            }
            _ => panic!("Expected RateTarget"),
        }
    }

    #[test]
    fn test_acro_no_gps_required() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::Acro, &input);
        assert!(!modes.requires_gps());
    }

    // ── Sport mode tests ──

    #[test]
    fn test_sport_sticks_centered_holds_alt() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::Sport, &input);

        let output = modes.update(&input, DT);
        match output {
            ModeOutput::RateTarget { roll_rate, pitch_rate, yaw_rate, throttle } => {
                assert!(approx_eq(roll_rate, 0.0));
                assert!(approx_eq(pitch_rate, 0.0));
                assert!(approx_eq(yaw_rate, 0.0));
                // Throttle from pos controller (alt hold)
                assert!(throttle > 0.1 && throttle < 0.9);
            }
            _ => panic!("Expected RateTarget from Sport mode"),
        }
        assert!(approx_eq(modes.sport_target_alt, 10.0));
    }

    #[test]
    fn test_sport_full_sticks_give_rates() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.rc_roll = 1.0;
        input.rc_pitch = -1.0;
        input.rc_yaw = 0.5;
        modes.set_mode(FlightModeId::Sport, &input);

        let output = modes.update(&input, DT);
        match output {
            ModeOutput::RateTarget { roll_rate, pitch_rate, yaw_rate, .. } => {
                let rp_max = SportConfig::default().roll_pitch_rate_max;
                let y_max = SportConfig::default().yaw_rate_max;
                assert!(approx_eq(roll_rate, rp_max));
                assert!(approx_eq(pitch_rate, -rp_max));
                assert!(approx_eq(yaw_rate, 0.5 * y_max));
            }
            _ => panic!("Expected RateTarget"),
        }
    }

    #[test]
    fn test_sport_throttle_adjusts_altitude() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        modes.set_mode(FlightModeId::Sport, &input);

        let initial_alt = modes.sport_target_alt;
        input.rc_throttle = 0.8; // climb
        for _ in 0..100 {
            modes.update(&input, DT);
        }
        assert!(modes.sport_target_alt > initial_alt);
    }

    #[test]
    fn test_sport_no_gps_required() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::Sport, &input);
        assert!(!modes.requires_gps());
    }

    // ── Drift mode tests ──

    #[test]
    fn test_drift_no_input_holds_level() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::Drift, &input);

        let output = modes.update(&input, DT);
        match output {
            ModeOutput::AttitudeTarget { quaternion, throttle } => {
                let (roll, pitch, _yaw) = quaternion.to_euler();
                assert!(approx_eq(roll, 0.0));
                assert!(approx_eq(pitch, 0.0));
                assert!(throttle > 0.1 && throttle < 0.9);
            }
            _ => panic!("Expected AttitudeTarget from Drift mode"),
        }
    }

    #[test]
    fn test_drift_roll_couples_yaw() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.rc_roll = 0.5; // roll right
        modes.set_mode(FlightModeId::Drift, &input);

        // Run several ticks to accumulate yaw
        for _ in 0..100 {
            modes.update(&input, DT);
        }
        // Yaw should have changed from 0 due to roll-yaw coupling
        assert!(modes.drift_yaw.abs() > 0.01);
    }

    #[test]
    fn test_drift_no_gps_required() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::Drift, &input);
        assert!(!modes.requires_gps());
    }

    // ── Flip mode tests ──

    #[test]
    fn test_flip_starts_in_start_state() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::Flip, &input);
        assert_eq!(modes.flip_state(), FlipState::Start);
    }

    #[test]
    fn test_flip_transitions_start_to_flipping() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::Flip, &input);

        // First update transitions Start → Flipping
        let output = modes.update(&input, DT);
        match output {
            ModeOutput::RateTarget { roll_rate, .. } => {
                assert!(roll_rate > 0.0, "Should command positive roll rate");
            }
            _ => panic!("Expected RateTarget during flip start"),
        }
        assert_eq!(modes.flip_state(), FlipState::Flipping);
    }

    #[test]
    fn test_flip_accumulates_past_flipping() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::Flip, &input);

        // First tick: Start → Flipping
        modes.update(&input, DT);
        assert_eq!(modes.flip_state(), FlipState::Flipping);

        // Run enough ticks to accumulate past flip_angle (90 deg at 400 deg/s)
        // 90/400 = 0.225s. At DT=0.0025: 0.225/0.0025 = 90 ticks
        // After flipping with level input attitude, recover immediately detects level
        // and transitions to Complete. So after enough ticks we end up past Flipping.
        for _ in 0..100 {
            modes.update(&input, DT);
        }
        // Should have moved past Flipping (to Recover or Complete)
        assert_ne!(modes.flip_state(), FlipState::Flipping);
        assert_ne!(modes.flip_state(), FlipState::Start);
    }

    #[test]
    fn test_flip_recover_with_tilted_attitude() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        // Start with a tilted attitude so recovery doesn't complete instantly
        input.attitude = Quaternion::from_euler(1.0, 0.5, 0.0); // significantly tilted
        modes.set_mode(FlightModeId::Flip, &input);

        // Start → Flipping
        modes.update(&input, DT);
        assert_eq!(modes.flip_state(), FlipState::Flipping);

        // Flipping → Recover (with tilted attitude, should stay in Recover)
        for _ in 0..100 {
            modes.update(&input, DT);
        }
        // With tilted input, it enters Recover but roll/pitch > 0.15 so stays
        assert_eq!(modes.flip_state(), FlipState::Recover);

        // Now set attitude to level — should transition to Complete
        input.attitude = Quaternion::identity();
        let output = modes.update(&input, DT);
        match output {
            ModeOutput::AttitudeTarget { throttle, .. } => {
                assert!(throttle > 0.5, "Should boost throttle to recover altitude");
            }
            _ => panic!("Expected AttitudeTarget during recovery"),
        }
        assert_eq!(modes.flip_state(), FlipState::Complete);
    }

    // ── Follow mode tests ──

    #[test]
    fn test_follow_tracks_target_with_offset() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::Follow, &input);

        // Set target at (100, 50, -10)
        modes.set_follow_target(Vec3::new(100.0, 50.0, -10.0));

        let output = modes.update(&input, DT);
        match output {
            ModeOutput::PositionTarget { position, .. } => {
                let offset = FollowConfig::default().offset;
                assert!(approx_eq(position.x, 100.0 + offset.x));
                assert!(approx_eq(position.y, 50.0 + offset.y));
                assert!(approx_eq(position.z, -10.0 + offset.z));
            }
            _ => panic!("Expected PositionTarget from Follow mode"),
        }
    }

    #[test]
    fn test_follow_faces_target() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.position = Vec3::new(0.0, 0.0, -10.0);
        modes.set_mode(FlightModeId::Follow, &input);

        // Target due east
        modes.set_follow_target(Vec3::new(0.0, 100.0, -10.0));

        let output = modes.update(&input, DT);
        match output {
            ModeOutput::PositionTarget { yaw, .. } => {
                // atan2(100, 0) ≈ PI/2
                assert!((yaw - core::f32::consts::FRAC_PI_2).abs() < 0.01);
            }
            _ => panic!("Expected PositionTarget"),
        }
    }

    #[test]
    fn test_follow_requires_gps() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::Follow, &input);
        assert!(modes.requires_gps());
    }

    // ── ZigZag mode tests ──

    #[test]
    fn test_zigzag_starts_idle() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::ZigZag, &input);
        assert_eq!(modes.zigzag_state(), ZigZagState::Idle);
    }

    #[test]
    fn test_zigzag_idle_holds_position() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.position = Vec3::new(5.0, 5.0, -10.0);
        modes.set_mode(FlightModeId::ZigZag, &input);

        let output = modes.update(&input, DT);
        match output {
            ModeOutput::PositionTarget { position, .. } => {
                assert!(approx_eq(position.x, 5.0));
                assert!(approx_eq(position.y, 5.0));
            }
            _ => panic!("Expected PositionTarget from ZigZag idle"),
        }
    }

    #[test]
    fn test_zigzag_flies_along_line() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::ZigZag, &input);

        // Set line A from (0,0,-10) to (100,0,-10)
        modes.set_zigzag_line_a(
            Vec3::new(0.0, 0.0, -10.0),
            Vec3::new(100.0, 0.0, -10.0),
        );
        modes.zigzag_start();

        assert_eq!(modes.zigzag_state(), ZigZagState::LineA);

        // Run some ticks — progress should advance
        for _ in 0..100 {
            modes.update(&input, DT);
        }
        assert!(modes.zigzag_progress > 0.0);
    }

    #[test]
    fn test_zigzag_switch_line() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::ZigZag, &input);

        modes.set_zigzag_line_a(
            Vec3::new(0.0, 0.0, -10.0),
            Vec3::new(100.0, 0.0, -10.0),
        );
        modes.zigzag_start();
        assert_eq!(modes.zigzag_state(), ZigZagState::LineA);

        modes.zigzag_switch_line();
        assert_eq!(modes.zigzag_state(), ZigZagState::LineB);

        modes.zigzag_switch_line();
        assert_eq!(modes.zigzag_state(), ZigZagState::LineA);
    }

    #[test]
    fn test_zigzag_requires_gps() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::ZigZag, &input);
        assert!(modes.requires_gps());
    }

    // ── GuidedNoGPS mode tests ──

    #[test]
    fn test_guided_nogps_outputs_attitude() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::GuidedNoGPS, &input);

        let target_quat = Quaternion::from_euler(0.1, 0.2, 0.3);
        modes.set_guided_nogps_target(target_quat, 0.6);

        let output = modes.update(&input, DT);
        match output {
            ModeOutput::AttitudeTarget { quaternion, throttle } => {
                let (r, p, y) = quaternion.to_euler();
                let (tr, tp, ty) = target_quat.to_euler();
                assert!((r - tr).abs() < 0.01);
                assert!((p - tp).abs() < 0.01);
                assert!((y - ty).abs() < 0.01);
                assert!(approx_eq(throttle, 0.6));
            }
            _ => panic!("Expected AttitudeTarget from GuidedNoGPS"),
        }
    }

    #[test]
    fn test_guided_nogps_no_gps_required() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::GuidedNoGPS, &input);
        assert!(!modes.requires_gps());
    }

    #[test]
    fn test_guided_nogps_enter_captures_attitude() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        let q = Quaternion::from_euler(0.3, -0.1, 1.0);
        input.attitude = q;
        modes.set_mode(FlightModeId::GuidedNoGPS, &input);

        // Should output the captured attitude (before any set_guided_nogps_target call)
        let output = modes.update(&input, DT);
        match output {
            ModeOutput::AttitudeTarget { quaternion, throttle } => {
                let (r, p, y) = quaternion.to_euler();
                let (qr, qp, qy) = q.to_euler();
                assert!((r - qr).abs() < 0.01);
                assert!((p - qp).abs() < 0.01);
                assert!((y - qy).abs() < 0.01);
                assert!(approx_eq(throttle, 0.5));
            }
            _ => panic!("Expected AttitudeTarget"),
        }
    }

    // ── SmartRTL mode tests ──

    #[test]
    fn test_smartrtl_no_path_descends() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.altitude = 10.0;
        // No breadcrumbs recorded
        modes.set_mode(FlightModeId::SmartRTL, &input);

        // Should fall back to descending
        assert_eq!(modes.smartrtl_state(), SmartRtlState::Descending);

        let output = modes.update(&input, DT);
        match output {
            ModeOutput::PositionTarget { velocity, .. } => {
                // Should command descent (positive z in NED = down)
                assert!(velocity.z > 0.0, "Should descend: vz={}", velocity.z);
            }
            _ => panic!("Expected PositionTarget during SmartRTL descent"),
        }
    }

    #[test]
    fn test_smartrtl_follows_breadcrumbs() {
        let mut modes = MultirotorModes::new();
        let input = default_input();

        // Record breadcrumbs
        modes.smartrtl_record(Vec3::new(0.0, 0.0, -10.0));
        modes.smartrtl_record(Vec3::new(50.0, 0.0, -10.0));
        modes.smartrtl_record(Vec3::new(100.0, 0.0, -10.0));

        modes.set_mode(FlightModeId::SmartRTL, &input);
        assert_eq!(modes.smartrtl_state(), SmartRtlState::Following);
        assert!(modes.smartrtl_path_len > 0);
    }

    #[test]
    fn test_smartrtl_reaches_waypoint_advances() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();

        // Record breadcrumbs far apart
        modes.smartrtl_record(Vec3::new(0.0, 0.0, -10.0));
        modes.smartrtl_record(Vec3::new(50.0, 0.0, -10.0));

        modes.set_mode(FlightModeId::SmartRTL, &input);
        let initial_idx = modes.smartrtl_path_index;

        // Place vehicle at the first return waypoint (newest = 50,0,-10)
        input.position = Vec3::new(50.0, 0.0, -10.0);
        modes.update(&input, DT);

        // Should have advanced past the first waypoint
        assert!(modes.smartrtl_path_index > initial_idx);
    }

    #[test]
    fn test_smartrtl_complete_after_descent() {
        let mut modes = MultirotorModes::new();
        let mut input = default_input();
        input.altitude = 0.2; // near ground
        // No breadcrumbs — goes straight to descending
        modes.set_mode(FlightModeId::SmartRTL, &input);
        assert_eq!(modes.smartrtl_state(), SmartRtlState::Descending);

        modes.update(&input, DT);
        assert_eq!(modes.smartrtl_state(), SmartRtlState::Complete);
    }

    #[test]
    fn test_smartrtl_requires_gps() {
        let mut modes = MultirotorModes::new();
        let input = default_input();
        modes.set_mode(FlightModeId::SmartRTL, &input);
        assert!(modes.requires_gps());
    }
}

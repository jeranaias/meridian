#![no_std]

//! Helicopter support: swashplate mixing, rotor speed control, and autorotation.
//!
//! Source: ArduPilot AP_MotorsHeli, AP_MotorsHeli_Swash, RSC_Governor.
//! Swashplate geometry converts (roll, pitch, collective) into 3-4 servo outputs.
//! RSC governor maintains constant headspeed via PID on RPM error.
//! Autorotation controller manages collective during power loss.

use core::f32::consts::PI;

// =================================================================
//  Swashplate Types (6 variants -- all ArduPilot types)
//  Source: AP_MotorsHeli_Swash.h
// =================================================================

/// Swashplate geometry types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SwashplateType {
    /// H3 Generic: configurable servo phase angles via parameters.
    /// Source: SWASHPLATE_TYPE_H3 (index 0)
    H3Generic,
    /// H1: non-CCPM, direct pitch/roll to individual servos.
    /// Source: SWASHPLATE_TYPE_H1 (index 1)
    H1,
    /// 3-servo, 140 deg spacing. Some scale helis.
    H3_140,
    /// 3-servo, 120 deg spacing. Most common for small/medium helis.
    H3_120,
    /// 4-servo, 90 deg spacing. Large helis, redundant.
    H4_90,
    /// 4-servo, 45 deg offset. Rotated 4-point swash.
    H4_45,
}

impl SwashplateType {
    /// Number of servos for this swashplate type.
    pub fn servo_count(&self) -> usize {
        match self {
            SwashplateType::H3Generic | SwashplateType::H3_120 | SwashplateType::H3_140 => 3,
            SwashplateType::H1 | SwashplateType::H4_90 | SwashplateType::H4_45 => 4,
        }
    }
}

/// Swashplate mixer output: up to 4 servo commands.
#[derive(Debug, Clone, Copy)]
pub struct SwashplateOutput {
    /// Servo values in [-1.0, 1.0] range. Only first `count` are valid.
    pub servos: [f32; 4],
    /// Number of valid servo outputs.
    pub count: usize,
}

/// Swashplate mixer converts (roll, pitch, collective) to servo commands.
///
/// Source: AP_MotorsHeli_Swash.cpp
pub struct SwashplateMixer {
    swash_type: SwashplateType,
    /// Phase angles for each servo (radians from front, CW positive).
    phase_angles: [f32; 4],
    /// Collective mixing weight per servo.
    collective_weight: [f32; 4],
}

impl SwashplateMixer {
    /// Create a mixer for the given swashplate geometry.
    pub fn new(swash_type: SwashplateType) -> Self {
        let (phase_angles, collective_weight) = match swash_type {
            SwashplateType::H3Generic => {
                // Default generic: same as H3_120 until configured
                let angles = [0.0, 2.0 * PI / 3.0, 4.0 * PI / 3.0, 0.0];
                let coll = [1.0, 1.0, 1.0, 0.0];
                (angles, coll)
            }
            SwashplateType::H1 => {
                // H1: non-CCPM. 3 servos: pitch, roll, collective (direct, no phase mixing).
                // Pitch servo at 0 deg, roll servo at 90 deg, collective all-up.
                // The 4th is unused but we keep it for the API.
                let angles = [0.0, PI / 2.0, 0.0, 0.0];
                let coll = [0.0, 0.0, 1.0, 0.0];
                (angles, coll)
            }
            SwashplateType::H3_120 => {
                let angles = [0.0, 2.0 * PI / 3.0, 4.0 * PI / 3.0, 0.0];
                let coll = [1.0, 1.0, 1.0, 0.0];
                (angles, coll)
            }
            SwashplateType::H3_140 => {
                let angles = [0.0, 140.0 * PI / 180.0, 220.0 * PI / 180.0, 0.0];
                let coll = [1.0, 1.0, 1.0, 0.0];
                (angles, coll)
            }
            SwashplateType::H4_90 => {
                let angles = [0.0, PI / 2.0, PI, 3.0 * PI / 2.0];
                let coll = [1.0, 1.0, 1.0, 1.0];
                (angles, coll)
            }
            SwashplateType::H4_45 => {
                let angles = [PI / 4.0, 3.0 * PI / 4.0, 5.0 * PI / 4.0, 7.0 * PI / 4.0];
                let coll = [1.0, 1.0, 1.0, 1.0];
                (angles, coll)
            }
        };

        Self { swash_type, phase_angles, collective_weight }
    }

    /// Create an H3 Generic mixer with custom servo phase angles (degrees).
    /// Source: AP_MotorsHeli_Swash.cpp SWASHPLATE_TYPE_H3
    pub fn new_h3_generic(servo1_deg: f32, servo2_deg: f32, servo3_deg: f32) -> Self {
        let angles = [
            servo1_deg * PI / 180.0,
            servo2_deg * PI / 180.0,
            servo3_deg * PI / 180.0,
            0.0,
        ];
        Self {
            swash_type: SwashplateType::H3Generic,
            phase_angles: angles,
            collective_weight: [1.0, 1.0, 1.0, 0.0],
        }
    }

    /// Swashplate type this mixer is configured for.
    pub fn swash_type(&self) -> SwashplateType {
        self.swash_type
    }

    /// Mix roll, pitch, and collective inputs into servo commands.
    ///
    /// Inputs are normalized: roll/pitch in [-1.0, 1.0], collective in [0.0, 1.0].
    /// Outputs are servo positions in [-1.0, 1.0].
    pub fn mix(&self, roll: f32, pitch: f32, collective: f32) -> SwashplateOutput {
        let count = self.swash_type.servo_count();
        let mut servos = [0.0f32; 4];

        if self.swash_type == SwashplateType::H1 {
            // H1: non-CCPM, direct mixing.
            // Servo 0: pitch only
            // Servo 1: roll only
            // Servo 2: collective only
            let coll_centered = collective * 2.0 - 1.0;
            servos[0] = pitch.clamp(-1.0, 1.0);
            servos[1] = roll.clamp(-1.0, 1.0);
            servos[2] = coll_centered.clamp(-1.0, 1.0);
            // H1 uses 3 servos, not 4
            return SwashplateOutput { servos, count: 3 };
        }

        // CCPM mixing for H3/H4 types
        let coll_centered = collective * 2.0 - 1.0;

        for i in 0..count {
            let angle = self.phase_angles[i];
            let pitch_factor = libm::cosf(angle);
            let roll_factor = libm::sinf(angle);

            servos[i] = pitch * pitch_factor
                + roll * roll_factor
                + coll_centered * self.collective_weight[i];

            servos[i] = servos[i].clamp(-1.0, 1.0);
        }

        SwashplateOutput { servos, count }
    }
}

// =================================================================
//  Rotor Speed Controller (RSC / Governor)
//  Source: AP_MotorsHeli_RSC.cpp
// =================================================================

/// RSC operating modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RscMode {
    /// Engine off, zero throttle output.
    Idle,
    /// Ramping throttle from idle to governed speed.
    Ramp,
    /// Governor active, PID on RPM error.
    Active,
    /// External governor mode: RSC outputs a constant setpoint, external governor handles PID.
    /// Source: RSC_MODE=3
    ExternalGovernor,
}

/// Rotor Speed Controller configuration.
#[derive(Debug, Clone, Copy)]
pub struct RscConfig {
    /// Target headspeed (RPM).
    pub target_rpm: f32,
    /// Idle throttle output [0.0, 1.0].
    pub idle_throttle: f32,
    /// Ramp time from idle to governed throttle (seconds).
    pub ramp_time_s: f32,
    /// Governor PID gains.
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    /// Maximum integrator value.
    pub imax: f32,
    /// Throttle output limits.
    pub throttle_min: f32,
    pub throttle_max: f32,
    /// External governor setpoint [0.0, 1.0]. Used in ExternalGovernor mode.
    pub ext_governor_setpoint: f32,
    /// Collective feedforward gain. Scales collective input into throttle.
    /// Source: RSC_GOV_FF (passive governor assist).
    pub collective_ff: f32,
}

impl RscConfig {
    pub fn default_config() -> Self {
        Self {
            target_rpm: 1500.0,
            idle_throttle: 0.15,
            ramp_time_s: 5.0,
            kp: 0.002,
            ki: 0.001,
            kd: 0.0001,
            imax: 0.2,
            throttle_min: 0.0,
            throttle_max: 1.0,
            ext_governor_setpoint: 0.6,
            collective_ff: 0.0,
        }
    }
}

/// Rotor Speed Controller -- governor for maintaining constant headspeed.
///
/// Source: AP_MotorsHeli_RSC.cpp
pub struct RotorSpeedController {
    config: RscConfig,
    pub mode: RscMode,
    /// Current throttle output [0.0, 1.0].
    throttle_output: f32,
    /// Ramp progress [0.0, 1.0].
    ramp_progress: f32,
    /// PID state.
    pub integrator: f32,
    prev_error: f32,
    initialized: bool,
    /// Autorotation interlock: when true, RSC outputs zero throttle.
    autorotation_active: bool,
}

impl RotorSpeedController {
    pub fn new(config: RscConfig) -> Self {
        Self {
            config,
            mode: RscMode::Idle,
            throttle_output: 0.0,
            ramp_progress: 0.0,
            integrator: 0.0,
            prev_error: 0.0,
            initialized: false,
            autorotation_active: false,
        }
    }

    /// Current operating mode.
    pub fn mode(&self) -> RscMode {
        self.mode
    }

    /// Current throttle output [0.0, 1.0].
    pub fn throttle_output(&self) -> f32 {
        self.throttle_output
    }

    /// Set autorotation interlock. When active, RSC outputs zero throttle.
    /// Source: AP_MotorsHeli_RSC autorotation interlock
    pub fn set_autorotation(&mut self, active: bool) {
        self.autorotation_active = active;
    }

    /// Whether autorotation interlock is active.
    pub fn autorotation_active(&self) -> bool {
        self.autorotation_active
    }

    /// Transition to idle mode.
    pub fn set_idle(&mut self) {
        self.mode = RscMode::Idle;
        self.ramp_progress = 0.0;
        self.integrator = 0.0;
        self.initialized = false;
        self.throttle_output = self.config.idle_throttle;
    }

    /// Start ramp-up to governed speed.
    pub fn start_ramp(&mut self) {
        if self.mode == RscMode::Idle {
            self.mode = RscMode::Ramp;
            self.ramp_progress = 0.0;
        }
    }

    /// Set external governor mode.
    pub fn set_external_governor(&mut self) {
        self.mode = RscMode::ExternalGovernor;
    }

    /// Update the RSC. Call at regular intervals.
    ///
    /// `measured_rpm`: current rotor RPM from sensor.
    /// `collective`: current collective input [0.0, 1.0] (for feedforward).
    /// `dt`: time step in seconds.
    pub fn update_with_collective(&mut self, measured_rpm: f32, collective: f32, dt: f32) {
        // Autorotation interlock: zero throttle
        if self.autorotation_active {
            self.throttle_output = 0.0;
            return;
        }

        match self.mode {
            RscMode::Idle => {
                self.throttle_output = self.config.idle_throttle;
            }
            RscMode::Ramp => {
                if self.config.ramp_time_s > 0.0 {
                    self.ramp_progress += dt / self.config.ramp_time_s;
                } else {
                    self.ramp_progress = 1.0;
                }

                if self.ramp_progress >= 1.0 {
                    self.ramp_progress = 1.0;
                    self.mode = RscMode::Active;
                    self.integrator = self.throttle_output.clamp(0.0, self.config.imax);
                }

                let ramp_target = 0.5;
                self.throttle_output = self.config.idle_throttle
                    + self.ramp_progress * (ramp_target - self.config.idle_throttle);
            }
            RscMode::Active => {
                let error = self.config.target_rpm - measured_rpm;

                // Proportional
                let p_term = self.config.kp * error;

                // Integral with anti-windup
                self.integrator += self.config.ki * error * dt;
                self.integrator = self.integrator.clamp(-self.config.imax, self.config.imax);

                // Derivative
                let d_term = if self.initialized {
                    self.config.kd * (error - self.prev_error) / dt.max(0.001)
                } else {
                    self.initialized = true;
                    0.0
                };

                self.prev_error = error;

                // Collective feedforward
                let ff_term = self.config.collective_ff * collective;

                self.throttle_output = (p_term + self.integrator + d_term + ff_term)
                    .clamp(self.config.throttle_min, self.config.throttle_max);
            }
            RscMode::ExternalGovernor => {
                // Just output the constant setpoint; external governor handles PID
                self.throttle_output = self.config.ext_governor_setpoint
                    .clamp(self.config.throttle_min, self.config.throttle_max);
            }
        }
    }

    /// Update without collective feedforward (backward-compatible).
    pub fn update(&mut self, measured_rpm: f32, dt: f32) {
        self.update_with_collective(measured_rpm, 0.0, dt);
    }
}

// =================================================================
//  Collective Mapper
// =================================================================

/// Maps throttle stick position to collective pitch with configurable curves.
///
/// Source: AP_MotorsHeli::set_collective
pub struct CollectiveMapper {
    /// Curve control points: input [0.0, 1.0] -> collective [min, max].
    points: [(f32, f32); 6],
    num_points: usize,
}

impl CollectiveMapper {
    /// Create a linear mapper from `col_min` to `col_max`.
    pub fn linear(col_min: f32, col_max: f32) -> Self {
        let mut points = [(0.0, 0.0); 6];
        points[0] = (0.0, col_min);
        points[1] = (1.0, col_max);
        Self { points, num_points: 2 }
    }

    /// Create a mapper with a custom curve (up to 6 control points).
    pub fn from_curve(curve_points: &[(f32, f32)]) -> Self {
        let mut points = [(0.0, 0.0); 6];
        let count = curve_points.len().min(6);
        for i in 0..count {
            points[i] = curve_points[i];
        }
        Self { points, num_points: count }
    }

    /// Map a stick position [0.0, 1.0] to collective output.
    pub fn map(&self, stick: f32) -> f32 {
        let stick = stick.clamp(0.0, 1.0);

        if self.num_points == 0 { return stick; }
        if self.num_points == 1 { return self.points[0].1; }

        if stick <= self.points[0].0 { return self.points[0].1; }
        if stick >= self.points[self.num_points - 1].0 {
            return self.points[self.num_points - 1].1;
        }

        for i in 1..self.num_points {
            if stick <= self.points[i].0 {
                let (x0, y0) = self.points[i - 1];
                let (x1, y1) = self.points[i];
                let t = if (x1 - x0).abs() > 1e-6 {
                    (stick - x0) / (x1 - x0)
                } else {
                    0.0
                };
                return y0 + t * (y1 - y0);
            }
        }

        self.points[self.num_points - 1].1
    }
}

// =================================================================
//  Autorotation Controller
//  Source: AC_Autorotation.cpp
// =================================================================

/// Autorotation phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutorotationPhase {
    /// Normal powered flight.
    Off,
    /// Power loss detected -- reducing collective to maintain rotor RPM.
    Entry,
    /// Steady-state autorotation descent.
    Glide,
    /// Flare -- increasing collective to arrest descent rate.
    Flare,
    /// Touch-down -- final collective reduction for landing.
    /// Source: AC_Autorotation.cpp (present in AP, was missing in Meridian)
    TouchDown,
}

/// Autorotation controller configuration.
#[derive(Debug, Clone, Copy)]
pub struct AutorotationConfig {
    /// Minimum RPM before declaring power loss (as fraction of target).
    pub rpm_threshold_frac: f32,
    /// Collective value during entry/glide phase [0.0, 1.0].
    pub min_collective: f32,
    /// Collective value during flare [0.0, 1.0].
    pub flare_collective: f32,
    /// Altitude AGL to begin flare (meters).
    pub flare_altitude_m: f32,
    /// Time to hold entry phase before transitioning to glide (seconds).
    pub entry_time_s: f32,
    /// Altitude AGL to begin touch-down phase (meters).
    pub touchdown_altitude_m: f32,
    /// Collective during touch-down [0.0, 1.0].
    pub touchdown_collective: f32,
    /// Headspeed target fraction during glide [0.0, 1.0] of target RPM.
    /// Source: AC_Autorotation headspeed controller
    pub glide_headspeed_target: f32,
    /// Headspeed P gain for collective adjustment during glide.
    pub headspeed_kp: f32,
}

impl AutorotationConfig {
    pub fn default_config() -> Self {
        Self {
            rpm_threshold_frac: 0.85,
            min_collective: 0.1,
            flare_collective: 0.8,
            flare_altitude_m: 10.0,
            entry_time_s: 1.0,
            touchdown_altitude_m: 2.0,
            touchdown_collective: 0.3,
            glide_headspeed_target: 0.95,
            headspeed_kp: 0.5,
        }
    }
}

/// Autorotation controller -- manages collective during power loss.
///
/// Source: AC_Autorotation.cpp
///
/// Sequence: detect RPM drop -> Entry -> Glide (with headspeed control) -> Flare -> TouchDown
pub struct AutorotationController {
    config: AutorotationConfig,
    phase: AutorotationPhase,
    /// Target RPM for rotor speed monitoring.
    target_rpm: f32,
    /// Time spent in current phase (seconds).
    phase_timer_s: f32,
    /// Collective output [0.0, 1.0].
    collective_output: f32,
    /// RSC interlock: set to true when autorotation is active to zero engine throttle.
    rsc_interlock: bool,
}

impl AutorotationController {
    pub fn new(config: AutorotationConfig, target_rpm: f32) -> Self {
        Self {
            config,
            phase: AutorotationPhase::Off,
            target_rpm,
            phase_timer_s: 0.0,
            collective_output: 0.5,
            rsc_interlock: false,
        }
    }

    /// Current autorotation phase.
    pub fn phase(&self) -> AutorotationPhase {
        self.phase
    }

    /// Current collective output [0.0, 1.0].
    pub fn collective_output(&self) -> f32 {
        self.collective_output
    }

    /// Whether RSC should be interlocked (zero throttle).
    pub fn rsc_interlock(&self) -> bool {
        self.rsc_interlock
    }

    /// Check if autorotation should be activated based on RPM.
    pub fn detect_power_loss(&self, measured_rpm: f32) -> bool {
        measured_rpm < self.target_rpm * self.config.rpm_threshold_frac
    }

    /// Update the autorotation controller.
    ///
    /// `measured_rpm`: current rotor RPM.
    /// `altitude_agl_m`: altitude above ground (meters).
    /// `dt`: time step (seconds).
    ///
    /// Returns the commanded collective [0.0, 1.0].
    pub fn update(&mut self, measured_rpm: f32, altitude_agl_m: f32, dt: f32) -> f32 {
        match self.phase {
            AutorotationPhase::Off => {
                if self.detect_power_loss(measured_rpm) {
                    self.phase = AutorotationPhase::Entry;
                    self.phase_timer_s = 0.0;
                    self.collective_output = self.config.min_collective;
                    self.rsc_interlock = true;
                }
            }
            AutorotationPhase::Entry => {
                self.phase_timer_s += dt;
                self.collective_output = self.config.min_collective;

                if self.phase_timer_s >= self.config.entry_time_s {
                    self.phase = AutorotationPhase::Glide;
                    self.phase_timer_s = 0.0;
                }
            }
            AutorotationPhase::Glide => {
                self.phase_timer_s += dt;

                // Headspeed control: adjust collective to maintain target RPM fraction.
                // Source: AC_Autorotation headspeed controller
                let target_rpm = self.target_rpm * self.config.glide_headspeed_target;
                let rpm_error = target_rpm - measured_rpm;
                // Negative error (RPM too high) -> increase collective (more drag)
                // Positive error (RPM too low) -> decrease collective (less drag)
                let coll_adj = -self.config.headspeed_kp * rpm_error / self.target_rpm.max(1.0);
                self.collective_output = (self.config.min_collective + coll_adj).clamp(0.0, 0.6);

                // Transition to flare when close to ground
                if altitude_agl_m <= self.config.flare_altitude_m {
                    self.phase = AutorotationPhase::Flare;
                    self.phase_timer_s = 0.0;
                    self.collective_output = self.config.flare_collective;
                }
            }
            AutorotationPhase::Flare => {
                self.phase_timer_s += dt;
                self.collective_output = self.config.flare_collective;

                // Transition to touch-down at low altitude
                if altitude_agl_m <= self.config.touchdown_altitude_m {
                    self.phase = AutorotationPhase::TouchDown;
                    self.phase_timer_s = 0.0;
                    self.collective_output = self.config.touchdown_collective;
                }
            }
            AutorotationPhase::TouchDown => {
                self.phase_timer_s += dt;
                // Reduce collective for soft landing
                self.collective_output = self.config.touchdown_collective;
            }
        }

        self.collective_output
    }

    /// Reset to normal powered flight.
    pub fn reset(&mut self) {
        self.phase = AutorotationPhase::Off;
        self.phase_timer_s = 0.0;
        self.collective_output = 0.5;
        self.rsc_interlock = false;
    }
}

// =================================================================
//  AP_MotorsHeli_Dual stub (tandem/coaxial)
//  Source: AP_MotorsHeli_Dual.cpp/h
// =================================================================

/// Dual helicopter configuration (tandem or coaxial).
/// Source: AP_MotorsHeli_Dual
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DualHeliMode {
    /// Tandem: two rotors fore and aft (e.g., CH-47 Chinook).
    Tandem,
    /// Coaxial: two rotors on same mast, counter-rotating.
    Coaxial,
}

/// AP_MotorsHeli_Dual stub: dual-rotor helicopter mixing.
///
/// Two swashplates, differential collective for yaw (tandem)
/// or differential collective for yaw (coaxial).
pub struct HeliDual {
    pub mode: DualHeliMode,
    pub swash_front: SwashplateMixer,
    pub swash_rear: SwashplateMixer,
    /// Yaw mixing gain: how much differential collective is used for yaw.
    pub yaw_gain: f32,
}

impl HeliDual {
    pub fn new(mode: DualHeliMode, swash_type: SwashplateType) -> Self {
        Self {
            mode,
            swash_front: SwashplateMixer::new(swash_type),
            swash_rear: SwashplateMixer::new(swash_type),
            yaw_gain: 0.5,
        }
    }

    /// Mix inputs into dual swashplate servo commands.
    ///
    /// `roll`, `pitch`, `yaw`: [-1.0, 1.0]
    /// `collective`: [0.0, 1.0]
    ///
    /// Returns (front_output, rear_output).
    pub fn mix(
        &self,
        roll: f32,
        pitch: f32,
        yaw: f32,
        collective: f32,
    ) -> (SwashplateOutput, SwashplateOutput) {
        match self.mode {
            DualHeliMode::Tandem => {
                // Tandem: yaw via differential collective.
                // Pitch via differential collective (front up, rear down or vice versa).
                let yaw_diff = yaw * self.yaw_gain;
                let coll_front = (collective + yaw_diff).clamp(0.0, 1.0);
                let coll_rear = (collective - yaw_diff).clamp(0.0, 1.0);

                let front = self.swash_front.mix(roll, pitch, coll_front);
                let rear = self.swash_rear.mix(roll, -pitch, coll_rear);
                (front, rear)
            }
            DualHeliMode::Coaxial => {
                // Coaxial: yaw via differential collective between upper and lower rotor.
                let yaw_diff = yaw * self.yaw_gain;
                let coll_upper = (collective + yaw_diff).clamp(0.0, 1.0);
                let coll_lower = (collective - yaw_diff).clamp(0.0, 1.0);

                let upper = self.swash_front.mix(roll, pitch, coll_upper);
                let lower = self.swash_rear.mix(roll, pitch, coll_lower);
                (upper, lower)
            }
        }
    }
}

// =================================================================
//  AP_MotorsHeli_Quad stub (quad-rotor helicopter)
//  Source: AP_MotorsHeli_Quad.cpp/h
// =================================================================

/// AP_MotorsHeli_Quad stub: quad-rotor helicopter with collective pitch props.
///
/// Four rotors each with individual collective pitch control.
/// Roll/pitch via differential collective, yaw via differential RPM or collective.
pub struct HeliQuad {
    /// Swashplate mixers for each rotor (typically H1 or simple collective).
    pub swash: [SwashplateMixer; 4],
    /// Motor angles from front (degrees, CW positive), same convention as multirotor.
    pub angles_deg: [f32; 4],
}

impl HeliQuad {
    /// Create a quad heli in X configuration.
    pub fn new_x() -> Self {
        Self {
            swash: [
                SwashplateMixer::new(SwashplateType::H1),
                SwashplateMixer::new(SwashplateType::H1),
                SwashplateMixer::new(SwashplateType::H1),
                SwashplateMixer::new(SwashplateType::H1),
            ],
            angles_deg: [45.0, -135.0, -45.0, 135.0],
        }
    }

    /// Mix inputs into per-rotor collective commands.
    ///
    /// `roll`, `pitch`, `yaw`: [-1.0, 1.0]
    /// `collective`: [0.0, 1.0]
    ///
    /// Returns collective for each rotor [0.0, 1.0].
    pub fn mix(&self, roll: f32, pitch: f32, yaw: f32, collective: f32) -> [f32; 4] {
        let mut collectives = [0.0f32; 4];
        for i in 0..4 {
            let angle_rad = self.angles_deg[i] * PI / 180.0;
            let roll_factor = libm::cosf(angle_rad + PI / 2.0);
            let pitch_factor = libm::cosf(angle_rad);

            // Each rotor's collective = base collective + RP differential
            let diff = roll * roll_factor * 0.5 + pitch * pitch_factor * 0.5;
            // Yaw via collective differential (alternating CW/CCW)
            let yaw_sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            let yaw_diff = yaw * yaw_sign * 0.25;

            collectives[i] = (collective + diff + yaw_diff).clamp(0.0, 1.0);
        }
        collectives
    }
}

// =================================================================
//  Bidirectional DShot support flag
//  Source: AP_HAL/RCOutput.h
// =================================================================

/// Bidirectional DShot (BDSHOT) configuration.
/// Source: AP_HAL/RCOutput.h BDSHOT support
///
/// Bidirectional DShot allows ESCs to report RPM telemetry on the same wire
/// as commands, eliminating the need for separate telemetry wires.
/// Requires SRAM4 (uncached) on STM32H7 for reliable DMA operation.
#[derive(Debug, Clone, Copy)]
pub struct BidirectionalDshotConfig {
    /// Whether BDSHOT is enabled.
    pub enabled: bool,
    /// Number of motor pole pairs (for RPM calculation from eRPM).
    pub motor_poles: u8,
}

impl Default for BidirectionalDshotConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            motor_poles: 7,
        }
    }
}

impl BidirectionalDshotConfig {
    /// Convert electrical RPM (from BDSHOT telemetry) to mechanical RPM.
    pub fn erpm_to_rpm(&self, erpm: u32) -> u32 {
        if self.motor_poles == 0 { return erpm; }
        erpm / self.motor_poles as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- Swashplate Tests ---

    #[test]
    fn test_h3_120_servo_count() {
        let mixer = SwashplateMixer::new(SwashplateType::H3_120);
        let output = mixer.mix(0.0, 0.0, 0.5);
        assert_eq!(output.count, 3);
    }

    #[test]
    fn test_h3_140_servo_count() {
        let mixer = SwashplateMixer::new(SwashplateType::H3_140);
        let output = mixer.mix(0.0, 0.0, 0.5);
        assert_eq!(output.count, 3);
    }

    #[test]
    fn test_h4_90_servo_count() {
        let mixer = SwashplateMixer::new(SwashplateType::H4_90);
        let output = mixer.mix(0.0, 0.0, 0.5);
        assert_eq!(output.count, 4);
    }

    #[test]
    fn test_h4_45_servo_count() {
        let mixer = SwashplateMixer::new(SwashplateType::H4_45);
        let output = mixer.mix(0.0, 0.0, 0.5);
        assert_eq!(output.count, 4);
    }

    #[test]
    fn test_h3_120_neutral_collective() {
        let mixer = SwashplateMixer::new(SwashplateType::H3_120);
        let output = mixer.mix(0.0, 0.0, 0.5);
        for i in 0..output.count {
            assert!(output.servos[i].abs() < 0.01,
                "Servo {} should be ~0 at neutral, got {}", i, output.servos[i]);
        }
    }

    #[test]
    fn test_h3_120_full_collective() {
        let mixer = SwashplateMixer::new(SwashplateType::H3_120);
        let output = mixer.mix(0.0, 0.0, 1.0);
        for i in 0..output.count {
            assert!(output.servos[i] > 0.5,
                "Servo {} should be >0.5 at full collective, got {}", i, output.servos[i]);
        }
    }

    #[test]
    fn test_h3_120_pitch_forward() {
        let mixer = SwashplateMixer::new(SwashplateType::H3_120);
        let output = mixer.mix(0.0, 1.0, 0.5);
        assert!(output.servos[0] > 0.5,
            "Front servo should move positive for forward pitch, got {}", output.servos[0]);
    }

    #[test]
    fn test_h4_90_symmetry() {
        let mixer = SwashplateMixer::new(SwashplateType::H4_90);
        let output = mixer.mix(1.0, 0.0, 0.5);
        assert!(output.servos[1] > 0.0, "90 deg servo should be positive for right roll");
        assert!(output.servos[3] < 0.0, "270 deg servo should be negative for right roll");
        assert!((output.servos[1] + output.servos[3]).abs() < 0.1,
            "Left/right servos should be symmetric");
    }

    // --- H1 swashplate tests ---

    #[test]
    fn test_h1_servo_count() {
        let mixer = SwashplateMixer::new(SwashplateType::H1);
        let output = mixer.mix(0.0, 0.0, 0.5);
        assert_eq!(output.count, 3);
    }

    #[test]
    fn test_h1_direct_mixing() {
        let mixer = SwashplateMixer::new(SwashplateType::H1);
        // Pure pitch -> servo 0 only
        let output = mixer.mix(0.0, 0.8, 0.5);
        assert!((output.servos[0] - 0.8).abs() < 0.01, "H1 pitch servo");
        assert!((output.servos[1]).abs() < 0.01, "H1 roll servo should be 0");
    }

    #[test]
    fn test_h1_roll() {
        let mixer = SwashplateMixer::new(SwashplateType::H1);
        let output = mixer.mix(0.7, 0.0, 0.5);
        assert!((output.servos[0]).abs() < 0.01, "H1 pitch servo should be 0 for roll");
        assert!((output.servos[1] - 0.7).abs() < 0.01, "H1 roll servo");
    }

    // --- H3 Generic tests ---

    #[test]
    fn test_h3_generic_custom_angles() {
        // Create H3 generic with 0, 120, 240 (same as H3_120)
        let mixer = SwashplateMixer::new_h3_generic(0.0, 120.0, 240.0);
        assert_eq!(mixer.swash_type(), SwashplateType::H3Generic);
        let output = mixer.mix(0.0, 0.0, 0.5);
        assert_eq!(output.count, 3);
        // At neutral collective, servos should be ~0
        for i in 0..3 {
            assert!(output.servos[i].abs() < 0.01);
        }
    }

    #[test]
    fn test_h3_generic_asymmetric() {
        // 0, 90, 180 -- non-standard spacing
        let mixer = SwashplateMixer::new_h3_generic(0.0, 90.0, 180.0);
        let output = mixer.mix(0.0, 1.0, 0.5);
        // Servo at 0 deg: cos(0)=1.0, should respond strongly to pitch
        assert!(output.servos[0] > 0.5);
        // Servo at 90 deg: cos(pi/2)~0, minimal pitch response
        assert!(output.servos[1].abs() < 0.2);
        // Servo at 180 deg: cos(pi)=-1, negative pitch response
        assert!(output.servos[2] < -0.5);
    }

    // --- RSC Tests ---

    #[test]
    fn test_rsc_idle_mode() {
        let config = RscConfig::default_config();
        let mut rsc = RotorSpeedController::new(config);
        rsc.set_idle();
        rsc.update(0.0, 0.01);
        assert_eq!(rsc.mode(), RscMode::Idle);
        assert!((rsc.throttle_output() - config.idle_throttle).abs() < 0.01);
    }

    #[test]
    fn test_rsc_ramp_timing() {
        let mut config = RscConfig::default_config();
        config.ramp_time_s = 2.0;
        let mut rsc = RotorSpeedController::new(config);

        rsc.set_idle();
        rsc.start_ramp();
        assert_eq!(rsc.mode(), RscMode::Ramp);

        for _ in 0..100 {
            rsc.update(0.0, 0.01);
        }
        assert_eq!(rsc.mode(), RscMode::Ramp, "Should still be ramping at 1s");

        for _ in 0..150 {
            rsc.update(1500.0, 0.01);
        }
        assert_eq!(rsc.mode(), RscMode::Active, "Should be active after ramp");
    }

    #[test]
    fn test_rsc_governor_corrects_underspeed() {
        let mut config = RscConfig::default_config();
        config.target_rpm = 1500.0;
        config.kp = 0.01;
        config.ki = 0.001;
        let mut rsc = RotorSpeedController::new(config);

        rsc.set_idle();
        rsc.start_ramp();
        rsc.mode = RscMode::Active;
        rsc.integrator = 0.3;

        rsc.update(1400.0, 0.01);
        let output = rsc.throttle_output();
        assert!(output > 0.3, "Should increase throttle for underspeed, got {}", output);
    }

    // --- External governor test ---

    #[test]
    fn test_rsc_external_governor() {
        let mut config = RscConfig::default_config();
        config.ext_governor_setpoint = 0.65;
        let mut rsc = RotorSpeedController::new(config);
        rsc.set_external_governor();
        rsc.update(1500.0, 0.01);
        assert_eq!(rsc.mode(), RscMode::ExternalGovernor);
        assert!((rsc.throttle_output() - 0.65).abs() < 0.01);
    }

    // --- Collective feedforward test ---

    #[test]
    fn test_rsc_collective_feedforward() {
        let mut config = RscConfig::default_config();
        config.target_rpm = 1500.0;
        config.kp = 0.01;
        config.ki = 0.0;
        config.collective_ff = 0.5;
        let mut rsc = RotorSpeedController::new(config);
        rsc.mode = RscMode::Active;
        rsc.initialized = true;

        // At target RPM with collective=0.8, feedforward adds 0.4 to output
        rsc.update_with_collective(1500.0, 0.8, 0.01);
        let out_ff = rsc.throttle_output();

        rsc.integrator = 0.0;
        rsc.prev_error = 0.0;
        rsc.update_with_collective(1500.0, 0.0, 0.01);
        let out_no_ff = rsc.throttle_output();

        assert!(out_ff > out_no_ff, "Feedforward should increase output: ff={}, no_ff={}", out_ff, out_no_ff);
    }

    // --- Autorotation interlock test ---

    #[test]
    fn test_rsc_autorotation_interlock() {
        let config = RscConfig::default_config();
        let mut rsc = RotorSpeedController::new(config);
        rsc.mode = RscMode::Active;
        rsc.integrator = 0.3;

        rsc.set_autorotation(true);
        rsc.update(1500.0, 0.01);
        assert!(rsc.throttle_output() < 0.01, "Autorotation interlock should zero throttle");

        rsc.set_autorotation(false);
        rsc.update(1500.0, 0.01);
        assert!(rsc.throttle_output() > 0.01, "Should resume after interlock released");
    }

    // --- Collective Mapper Tests ---

    #[test]
    fn test_linear_collective_map() {
        let mapper = CollectiveMapper::linear(-0.1, 0.9);
        assert!((mapper.map(0.0) - (-0.1)).abs() < 0.01);
        assert!((mapper.map(1.0) - 0.9).abs() < 0.01);
        assert!((mapper.map(0.5) - 0.4).abs() < 0.01);
    }

    #[test]
    fn test_curve_collective_map() {
        let mapper = CollectiveMapper::from_curve(&[
            (0.0, 0.0),
            (0.5, 0.3),
            (1.0, 1.0),
        ]);
        assert!((mapper.map(0.0)).abs() < 0.01);
        assert!((mapper.map(0.5) - 0.3).abs() < 0.01);
        assert!((mapper.map(1.0) - 1.0).abs() < 0.01);
        assert!((mapper.map(0.25) - 0.15).abs() < 0.01);
    }

    // --- Autorotation Tests ---

    #[test]
    fn test_autorotation_detection() {
        let config = AutorotationConfig::default_config();
        let ctrl = AutorotationController::new(config, 1500.0);
        assert!(!ctrl.detect_power_loss(1500.0));
        assert!(!ctrl.detect_power_loss(1300.0));
        assert!(ctrl.detect_power_loss(1200.0));
    }

    #[test]
    fn test_autorotation_entry_on_rpm_drop() {
        let config = AutorotationConfig::default_config();
        let mut ctrl = AutorotationController::new(config, 1500.0);

        assert_eq!(ctrl.phase(), AutorotationPhase::Off);
        ctrl.update(1000.0, 100.0, 0.01);
        assert_eq!(ctrl.phase(), AutorotationPhase::Entry);
        assert!((ctrl.collective_output() - config.min_collective).abs() < 0.01);
        assert!(ctrl.rsc_interlock(), "RSC interlock should be set on autorotation entry");
    }

    #[test]
    fn test_autorotation_entry_to_glide() {
        let mut config = AutorotationConfig::default_config();
        config.entry_time_s = 0.5;
        let mut ctrl = AutorotationController::new(config, 1500.0);

        ctrl.update(1000.0, 100.0, 0.01);
        assert_eq!(ctrl.phase(), AutorotationPhase::Entry);

        for _ in 0..55 {
            ctrl.update(1200.0, 100.0, 0.01);
        }
        assert_eq!(ctrl.phase(), AutorotationPhase::Glide);
    }

    #[test]
    fn test_autorotation_glide_to_flare() {
        let mut config = AutorotationConfig::default_config();
        config.entry_time_s = 0.1;
        config.flare_altitude_m = 10.0;
        let mut ctrl = AutorotationController::new(config, 1500.0);

        ctrl.update(1000.0, 100.0, 0.01);
        for _ in 0..15 {
            ctrl.update(1200.0, 100.0, 0.01);
        }
        assert_eq!(ctrl.phase(), AutorotationPhase::Glide);

        ctrl.update(1200.0, 8.0, 0.01);
        assert_eq!(ctrl.phase(), AutorotationPhase::Flare);
        assert!((ctrl.collective_output() - config.flare_collective).abs() < 0.01);
    }

    #[test]
    fn test_autorotation_flare_to_touchdown() {
        let mut config = AutorotationConfig::default_config();
        config.entry_time_s = 0.1;
        config.flare_altitude_m = 10.0;
        config.touchdown_altitude_m = 2.0;
        config.touchdown_collective = 0.3;
        let mut ctrl = AutorotationController::new(config, 1500.0);

        // Go through Entry -> Glide -> Flare
        ctrl.update(1000.0, 100.0, 0.01);
        for _ in 0..15 { ctrl.update(1200.0, 100.0, 0.01); }
        ctrl.update(1200.0, 8.0, 0.01);
        assert_eq!(ctrl.phase(), AutorotationPhase::Flare);

        // Drop to touchdown altitude
        ctrl.update(1200.0, 1.5, 0.01);
        assert_eq!(ctrl.phase(), AutorotationPhase::TouchDown);
        assert!((ctrl.collective_output() - 0.3).abs() < 0.01);
    }

    #[test]
    fn test_autorotation_headspeed_control() {
        let mut config = AutorotationConfig::default_config();
        config.entry_time_s = 0.1;
        config.glide_headspeed_target = 0.95;
        config.headspeed_kp = 0.5;
        let mut ctrl = AutorotationController::new(config, 1500.0);

        // Enter autorotation and reach glide phase
        ctrl.update(1000.0, 100.0, 0.01);
        for _ in 0..15 { ctrl.update(1200.0, 100.0, 0.01); }
        assert_eq!(ctrl.phase(), AutorotationPhase::Glide);

        // If RPM is above target (e.g., diving), collective should increase
        let _ = ctrl.update(1500.0, 100.0, 0.01);
        let high_rpm_coll = ctrl.collective_output();

        // If RPM is below target, collective should decrease
        ctrl.phase = AutorotationPhase::Glide; // reset
        let _ = ctrl.update(1200.0, 100.0, 0.01);
        let low_rpm_coll = ctrl.collective_output();

        assert!(high_rpm_coll > low_rpm_coll,
            "Higher RPM should produce higher collective: high={}, low={}", high_rpm_coll, low_rpm_coll);
    }

    #[test]
    fn test_autorotation_reset() {
        let config = AutorotationConfig::default_config();
        let mut ctrl = AutorotationController::new(config, 1500.0);

        ctrl.update(1000.0, 100.0, 0.01);
        assert_eq!(ctrl.phase(), AutorotationPhase::Entry);

        ctrl.reset();
        assert_eq!(ctrl.phase(), AutorotationPhase::Off);
        assert!(!ctrl.rsc_interlock());
    }

    // --- Dual Heli Tests ---

    #[test]
    fn test_heli_dual_tandem() {
        let dual = HeliDual::new(DualHeliMode::Tandem, SwashplateType::H3_120);
        let (front, rear) = dual.mix(0.0, 0.0, 0.0, 0.5);
        assert_eq!(front.count, 3);
        assert_eq!(rear.count, 3);
    }

    #[test]
    fn test_heli_dual_yaw() {
        let dual = HeliDual::new(DualHeliMode::Tandem, SwashplateType::H3_120);
        // Yaw right: front collective should differ from rear
        let (front, rear) = dual.mix(0.0, 0.0, 0.5, 0.5);
        // With positive yaw, front and rear get different collectives
        let front_avg: f32 = front.servos.iter().take(front.count).sum::<f32>() / front.count as f32;
        let rear_avg: f32 = rear.servos.iter().take(rear.count).sum::<f32>() / rear.count as f32;
        assert!((front_avg - rear_avg).abs() > 0.01,
            "Yaw should create differential: front={}, rear={}", front_avg, rear_avg);
    }

    // --- Quad Heli Tests ---

    #[test]
    fn test_heli_quad_mix() {
        let quad = HeliQuad::new_x();
        let collectives = quad.mix(0.0, 0.0, 0.0, 0.5);
        for i in 0..4 {
            assert!((collectives[i] - 0.5).abs() < 0.1,
                "Neutral should give ~0.5 collective, got {}", collectives[i]);
        }
    }

    #[test]
    fn test_heli_quad_roll() {
        let quad = HeliQuad::new_x();
        let collectives = quad.mix(0.5, 0.0, 0.0, 0.5);
        // Roll should create differential between left and right
        let left_avg = (collectives[1] + collectives[2]) / 2.0;
        let right_avg = (collectives[0] + collectives[3]) / 2.0;
        assert!((left_avg - right_avg).abs() > 0.05,
            "Roll should create differential: left={}, right={}", left_avg, right_avg);
    }

    // --- Bidirectional DShot ---

    #[test]
    fn test_bdshot_erpm_conversion() {
        let cfg = BidirectionalDshotConfig { enabled: true, motor_poles: 7 };
        assert_eq!(cfg.erpm_to_rpm(21000), 3000);
        assert_eq!(cfg.erpm_to_rpm(7000), 1000);
    }
}

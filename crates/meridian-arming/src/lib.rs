#![no_std]

//! Pre-arm check system — validates vehicle readiness before arming.
//!
//! Source: ArduPilot AP_Arming.cpp, AP_Arming_Copter.cpp
//! Every check must pass before motors can spin.
//! Checks can be individually disabled via bitmask (ARMING_CHECK parameter).

use heapless::String;

/// Maximum number of pre-arm check results.
pub const MAX_CHECKS: usize = 32;

/// Result of a single pre-arm check.
#[derive(Debug, Clone)]
pub struct CheckResult {
    pub name: String<20>,
    pub passed: bool,
    pub message: String<50>,
}

/// Pre-arm check categories (bitmask for ARMING_CHECK parameter).
/// Source: ArduPilot AP_Arming.h ArmingChecks enum
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArmingCheck {
    All = 0,
    BaroHealth = 1,
    CompassHealth = 2,
    GpsLock = 3,
    InsHealth = 4,    // IMU health
    Parameters = 5,
    RcChannels = 6,
    BoardVoltage = 7,
    BatteryLevel = 8,
    Logging = 9,
    SafetySwitch = 10,
    GpsConfig = 11,
    SystemHealth = 12,
    Mission = 13,
    RangeFinder = 14,
    FenceGeofence = 15,
    EkfHealth = 16,
}

/// Input state for pre-arm checking.
#[derive(Debug, Clone)]
pub struct ArmingState {
    pub baro_healthy: bool,
    pub baro_calibrated: bool,
    pub compass_healthy: bool,
    pub compass_configured: bool,
    pub gps_fix_type: u8,     // 0=none, 2=2D, 3=3D
    pub gps_num_sats: u8,
    pub gps_hdop: f32,
    pub imu_healthy: [bool; 3],
    pub imu_count: u8,
    pub imu_consistent: bool,  // multi-IMU consistency (0.75 m/s², 5 deg/s, 10s)
    pub ekf_healthy: bool,
    pub ekf_velocity_variance: f32,
    pub ekf_position_variance: f32,
    pub ekf_compass_variance: f32,
    pub battery_voltage: f32,
    pub battery_remaining_pct: u8,
    pub rc_calibrated: bool,
    pub rc_channels_valid: u8,
    pub board_voltage: f32,
    pub safety_switch_enabled: bool,
    pub safety_switch_state: bool, // true = safe (block arming)
    pub logging_available: bool,
    pub fence_loaded: bool,
    pub fence_enabled: bool,
    pub lean_angle_deg: f32,   // current lean angle (must be < threshold to arm)
    pub motor_interlock: bool, // true = motors can spin
    pub is_flying: bool,       // can't arm if already flying
    /// GAP 41: true if motor emergency stop switch is active
    pub motor_estop: bool,
    /// GAP 42: RC throttle value for failsafe check
    pub rc_throttle_value: u16,
    /// GAP 42: RC throttle failsafe threshold value
    pub rc_throttle_fs_value: u16,
    /// GAP 43: Airspeed sensor healthy (if installed)
    pub airspeed_healthy: bool,
    /// GAP 43: Whether airspeed sensor is required
    pub airspeed_required: bool,
    /// GAP 43: Obstacle avoidance system healthy (if installed)
    pub oa_healthy: bool,
    /// GAP 43: Whether OA system is required
    pub oa_required: bool,
    /// GAP 43: Whether altitude estimate is valid
    pub alt_valid: bool,
    /// GAP 44: Accelerometer readings for IMU consistency (m/s^2) [imu0, imu1, imu2]
    pub accel_readings: [[f32; 3]; 3],
    /// GAP 44: Gyro readings for IMU consistency (rad/s) [imu0, imu1, imu2]
    pub gyro_readings: [[f32; 3]; 3],
    /// L1: Current flight mode for mode-based arming checks.
    pub flight_mode: u8,
    /// L1: Whether the AHRS has a valid attitude estimate.
    pub attitude_valid: bool,
    /// L2: Whether a mission is loaded (for Auto mode arm check).
    pub mission_loaded: bool,
    /// L2: Number of mission items loaded.
    pub mission_item_count: u16,
}

/// Pre-arm check results.
#[derive(Debug)]
pub struct PreArmResult {
    pub checks: heapless::Vec<CheckResult, MAX_CHECKS>,
    pub all_passed: bool,
}

/// Configuration for arming checks.
#[derive(Debug, Clone, Copy)]
pub struct ArmingConfig {
    /// Bitmask of disabled checks. 0 = all enabled.
    pub disabled_mask: u32,
    /// Minimum GPS satellites for arming. Default: 6
    pub min_gps_sats: u8,
    /// Minimum GPS fix type (3 = 3D fix). Default: 3
    pub min_gps_fix: u8,
    /// Maximum HDOP for arming. Default: 2.0
    pub max_hdop: f32,
    /// Minimum battery voltage (V). Default: 10.0
    pub min_battery_voltage: f32,
    /// Minimum battery percentage. Default: 0 (disabled)
    pub min_battery_pct: u8,
    /// Maximum lean angle at arming (degrees). Default: 25.0
    pub max_lean_angle: f32,
    /// Maximum board voltage deviation from 5V (V). Default: 0.5
    pub board_voltage_tolerance: f32,
    /// EKF variance threshold. Default: 0.8
    pub ekf_variance_threshold: f32,
}

impl Default for ArmingConfig {
    fn default() -> Self {
        Self {
            disabled_mask: 0,
            min_gps_sats: 6,
            min_gps_fix: 3,
            max_hdop: 2.0,
            min_battery_voltage: 10.0,
            min_battery_pct: 0,
            max_lean_angle: 25.0,
            board_voltage_tolerance: 0.5,
            ekf_variance_threshold: 0.8,
        }
    }
}

/// The flight mode being armed into. Used to skip GPS checks for non-GPS modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ArmingMode {
    /// Mode requires GPS (Loiter, RTL, Auto, Guided, etc.)
    RequiresGps,
    /// Mode does not require GPS (Stabilize, AltHold, Acro, Sport, etc.)
    NoGpsRequired,
}

/// Run all pre-arm checks and return results.
/// Uses `ArmingMode::RequiresGps` by default (legacy behavior).
pub fn check_prearm(state: &ArmingState, config: &ArmingConfig) -> PreArmResult {
    check_prearm_for_mode(state, config, ArmingMode::RequiresGps)
}

/// Run all pre-arm checks, skipping GPS check for non-GPS modes.
///
/// M4 fix: ArduPilot skips GPS checks when arming into Stabilize, AltHold,
/// or Acro because those modes do not use GPS position. Without this, a vehicle
/// with a cold/broken GPS cannot arm into manual modes.
pub fn check_prearm_for_mode(state: &ArmingState, config: &ArmingConfig, arming_mode: ArmingMode) -> PreArmResult {
    let mut result = PreArmResult {
        checks: heapless::Vec::new(),
        all_passed: true,
    };

    let is_enabled = |check: ArmingCheck| -> bool {
        config.disabled_mask & (1 << check as u32) == 0
    };

    // Already flying — can't arm
    if state.is_flying {
        add_check(&mut result, "Vehicle", false, "Already flying");
    }

    // Barometer health
    if is_enabled(ArmingCheck::BaroHealth) {
        if !state.baro_healthy {
            add_check(&mut result, "Baro", false, "Not healthy");
        } else if !state.baro_calibrated {
            add_check(&mut result, "Baro", false, "Not calibrated");
        } else {
            add_check(&mut result, "Baro", true, "OK");
        }
    }

    // Compass
    if is_enabled(ArmingCheck::CompassHealth) {
        if !state.compass_healthy {
            add_check(&mut result, "Compass", false, "Not healthy");
        } else if !state.compass_configured {
            add_check(&mut result, "Compass", false, "Not calibrated");
        } else {
            add_check(&mut result, "Compass", true, "OK");
        }
    }

    // GPS — M4: skip GPS check for non-GPS modes (Stabilize, AltHold, Acro)
    // Source: ArduPilot AP_Arming.cpp — gps_checks() skipped when mode doesn't require GPS
    if is_enabled(ArmingCheck::GpsLock) && arming_mode == ArmingMode::RequiresGps {
        if state.gps_fix_type < config.min_gps_fix {
            add_check(&mut result, "GPS", false, "No 3D fix");
        } else if state.gps_num_sats < config.min_gps_sats {
            add_check(&mut result, "GPS", false, "Not enough sats");
        } else if state.gps_hdop > config.max_hdop {
            add_check(&mut result, "GPS", false, "HDOP too high");
        } else {
            add_check(&mut result, "GPS", true, "OK");
        }
    }

    // IMU
    if is_enabled(ArmingCheck::InsHealth) {
        let any_healthy = state.imu_healthy.iter().take(state.imu_count as usize).any(|h| *h);
        if !any_healthy {
            add_check(&mut result, "IMU", false, "No healthy IMU");
        } else if state.imu_count > 1 && !state.imu_consistent {
            add_check(&mut result, "IMU", false, "IMUs inconsistent");
        } else {
            add_check(&mut result, "IMU", true, "OK");
        }
    }

    // EKF
    if is_enabled(ArmingCheck::EkfHealth) {
        if !state.ekf_healthy {
            add_check(&mut result, "EKF", false, "Not healthy");
        } else if state.ekf_velocity_variance > config.ekf_variance_threshold {
            add_check(&mut result, "EKF", false, "Velocity variance");
        } else if state.ekf_position_variance > config.ekf_variance_threshold {
            add_check(&mut result, "EKF", false, "Position variance");
        } else {
            add_check(&mut result, "EKF", true, "OK");
        }
    }

    // Battery
    if is_enabled(ArmingCheck::BatteryLevel) {
        if state.battery_voltage < config.min_battery_voltage && config.min_battery_voltage > 0.0 {
            add_check(&mut result, "Battery", false, "Voltage low");
        } else if config.min_battery_pct > 0 && state.battery_remaining_pct < config.min_battery_pct {
            add_check(&mut result, "Battery", false, "Level low");
        } else {
            add_check(&mut result, "Battery", true, "OK");
        }
    }

    // RC
    if is_enabled(ArmingCheck::RcChannels) {
        if !state.rc_calibrated {
            add_check(&mut result, "RC", false, "Not calibrated");
        } else if state.rc_channels_valid < 4 {
            add_check(&mut result, "RC", false, "Missing channels");
        } else {
            add_check(&mut result, "RC", true, "OK");
        }
    }

    // Board voltage
    if is_enabled(ArmingCheck::BoardVoltage) {
        if (state.board_voltage - 5.0).abs() > config.board_voltage_tolerance {
            add_check(&mut result, "Board", false, "Voltage out of range");
        } else {
            add_check(&mut result, "Board", true, "OK");
        }
    }

    // Safety switch
    if is_enabled(ArmingCheck::SafetySwitch) && state.safety_switch_enabled {
        if state.safety_switch_state {
            add_check(&mut result, "Safety", false, "Switch engaged");
        } else {
            add_check(&mut result, "Safety", true, "OK");
        }
    }

    // Lean angle
    if state.lean_angle_deg > config.max_lean_angle {
        add_check(&mut result, "Lean", false, "Too tilted to arm");
    }

    // Logging
    if is_enabled(ArmingCheck::Logging) {
        if !state.logging_available {
            add_check(&mut result, "Logging", false, "No SD card");
        } else {
            add_check(&mut result, "Logging", true, "OK");
        }
    }

    // Fence
    if is_enabled(ArmingCheck::FenceGeofence) && state.fence_enabled {
        if !state.fence_loaded {
            add_check(&mut result, "Fence", false, "Not loaded");
        } else {
            add_check(&mut result, "Fence", true, "OK");
        }
    }

    // L1: Mode check at arm — block dangerous modes without attitude estimation.
    // Acro (1) and Sport (13) require direct rate control; if the AHRS has no valid
    // attitude estimate, the pilot cannot recover from upset.
    // Flip (14) is never safe to arm into.
    {
        let mode = state.flight_mode;
        // Acro = 1, Sport = 13: require attitude valid
        if (mode == 1 || mode == 13) && !state.attitude_valid {
            add_check(&mut result, "Mode", false, "No attitude for Acro");
        }
        // Flip = 14: never arm directly into Flip
        if mode == 14 {
            add_check(&mut result, "Mode", false, "Cannot arm in Flip");
        }
    }

    // L2: Mission check at arm — Auto mode (3) must have a mission loaded.
    if state.flight_mode == 3 && !state.mission_loaded {
        add_check(&mut result, "Mission", false, "No mission for Auto");
    }

    // GAP 41: Motor interlock / E-Stop conflict check
    if state.motor_interlock && state.motor_estop {
        add_check(&mut result, "Motors", false, "Interlock+EStop conflict");
    }

    // GAP 42: RC throttle failsafe pre-arm check
    if is_enabled(ArmingCheck::RcChannels) && state.rc_throttle_fs_value > 0 {
        // Throttle must be above the failsafe value (not in failsafe)
        if state.rc_throttle_value <= state.rc_throttle_fs_value {
            add_check(&mut result, "Throttle", false, "In failsafe range");
        }
    }

    // GAP 43: Airspeed sensor check
    if state.airspeed_required && !state.airspeed_healthy {
        add_check(&mut result, "Airspeed", false, "Not healthy");
    }

    // GAP 43: Obstacle avoidance check
    if state.oa_required && !state.oa_healthy {
        add_check(&mut result, "OA", false, "Not healthy");
    }

    // GAP 43: Altitude validity check
    if !state.alt_valid {
        add_check(&mut result, "Altitude", false, "Not valid");
    }

    // GAP 44: IMU consistency check (0.75 m/s², 5 deg/s)
    // Compute internally rather than relying on caller's imu_consistent bool
    if is_enabled(ArmingCheck::InsHealth) && state.imu_count > 1 {
        let consistent = check_imu_consistency(
            &state.accel_readings, &state.gyro_readings, state.imu_count,
        );
        if !consistent {
            add_check(&mut result, "IMU", false, "IMUs inconsistent");
        }
    }

    // Update all_passed
    result.all_passed = result.checks.iter().all(|c| c.passed);
    result
}

/// GAP 44: Compute IMU consistency check with ArduPilot's thresholds.
/// Returns true if all IMU pairs are consistent.
///
/// Source: AP_Arming::ins_accels_consistent(), AP_Arming::ins_gyros_consistent()
/// Thresholds:
///   - Accel: 0.75 m/s² (AP_ARMING_ACCEL_ERROR_THRESHOLD)
///   - Gyro: 5 deg/s (0.0873 rad/s)
///
/// Note: ArduPilot also requires 10s sustain time. This function does the
/// instantaneous check; the caller should track sustain time externally.
pub fn check_imu_consistency(
    accel_readings: &[[f32; 3]; 3],
    gyro_readings: &[[f32; 3]; 3],
    imu_count: u8,
) -> bool {
    const ACCEL_THRESHOLD: f32 = 0.75;  // m/s²
    const GYRO_THRESHOLD: f32 = 0.0873; // 5 deg/s in rad/s

    let count = imu_count.min(3) as usize;
    if count < 2 {
        return true;
    }

    // Check all pairs of IMUs
    for i in 0..count {
        for j in (i + 1)..count {
            // Accel consistency: squared magnitude of difference vector
            // (compare squared to avoid needing sqrt/libm)
            let dx = accel_readings[i][0] - accel_readings[j][0];
            let dy = accel_readings[i][1] - accel_readings[j][1];
            let dz = accel_readings[i][2] - accel_readings[j][2];
            let accel_diff_sq = dx * dx + dy * dy + dz * dz;
            if accel_diff_sq > ACCEL_THRESHOLD * ACCEL_THRESHOLD {
                return false;
            }

            // Gyro consistency: squared magnitude of difference vector
            let gx = gyro_readings[i][0] - gyro_readings[j][0];
            let gy = gyro_readings[i][1] - gyro_readings[j][1];
            let gz = gyro_readings[i][2] - gyro_readings[j][2];
            let gyro_diff_sq = gx * gx + gy * gy + gz * gz;
            if gyro_diff_sq > GYRO_THRESHOLD * GYRO_THRESHOLD {
                return false;
            }
        }
    }

    true
}

fn add_check(result: &mut PreArmResult, name: &str, passed: bool, msg: &str) {
    let mut n = String::new();
    let _ = n.push_str(name);
    let mut m = String::new();
    let _ = m.push_str(msg);
    let _ = result.checks.push(CheckResult { name: n, passed, message: m });
    if !passed {
        result.all_passed = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn healthy_state() -> ArmingState {
        ArmingState {
            baro_healthy: true,
            baro_calibrated: true,
            compass_healthy: true,
            compass_configured: true,
            gps_fix_type: 3,
            gps_num_sats: 12,
            gps_hdop: 0.9,
            imu_healthy: [true, true, false],
            imu_count: 2,
            imu_consistent: true,
            ekf_healthy: true,
            ekf_velocity_variance: 0.1,
            ekf_position_variance: 0.1,
            ekf_compass_variance: 0.1,
            battery_voltage: 12.6,
            battery_remaining_pct: 85,
            rc_calibrated: true,
            rc_channels_valid: 8,
            board_voltage: 5.0,
            safety_switch_enabled: false,
            safety_switch_state: false,
            logging_available: true,
            fence_loaded: true,
            fence_enabled: false,
            lean_angle_deg: 2.0,
            motor_interlock: false,
            is_flying: false,
            motor_estop: false,
            rc_throttle_value: 1100,
            rc_throttle_fs_value: 975,
            airspeed_healthy: true,
            airspeed_required: false,
            oa_healthy: true,
            oa_required: false,
            alt_valid: true,
            accel_readings: [[0.0, 0.0, -9.81], [0.0, 0.0, -9.81], [0.0; 3]],
            gyro_readings: [[0.0; 3]; 3],
            flight_mode: 5,       // Loiter — safe default
            attitude_valid: true,
            mission_loaded: true,
            mission_item_count: 3,
        }
    }

    #[test]
    fn test_all_healthy_passes() {
        let result = check_prearm(&healthy_state(), &ArmingConfig::default());
        assert!(result.all_passed, "All healthy should pass. Failed: {:?}",
            result.checks.iter().filter(|c| !c.passed).collect::<heapless::Vec<_, 32>>());
    }

    #[test]
    fn test_no_gps_fix_fails() {
        let mut state = healthy_state();
        state.gps_fix_type = 0;
        let result = check_prearm(&state, &ArmingConfig::default());
        assert!(!result.all_passed);
        assert!(result.checks.iter().any(|c| c.name.as_str() == "GPS" && !c.passed));
    }

    #[test]
    fn test_low_battery_fails() {
        let mut state = healthy_state();
        state.battery_voltage = 9.0;
        let result = check_prearm(&state, &ArmingConfig::default());
        assert!(!result.all_passed);
    }

    #[test]
    fn test_ekf_unhealthy_fails() {
        let mut state = healthy_state();
        state.ekf_healthy = false;
        let result = check_prearm(&state, &ArmingConfig::default());
        assert!(!result.all_passed);
    }

    #[test]
    fn test_imu_inconsistent_fails() {
        let mut state = healthy_state();
        state.imu_consistent = false;
        let result = check_prearm(&state, &ArmingConfig::default());
        assert!(!result.all_passed);
    }

    #[test]
    fn test_disabled_check_ignored() {
        let mut state = healthy_state();
        state.gps_fix_type = 0; // would fail GPS check
        let mut config = ArmingConfig::default();
        config.disabled_mask = 1 << ArmingCheck::GpsLock as u32; // disable GPS check
        let result = check_prearm(&state, &config);
        assert!(result.all_passed, "Disabled GPS check should be ignored");
    }

    #[test]
    fn test_too_tilted_fails() {
        let mut state = healthy_state();
        state.lean_angle_deg = 30.0; // > 25° default max
        let result = check_prearm(&state, &ArmingConfig::default());
        assert!(!result.all_passed);
    }

    #[test]
    fn test_safety_switch_blocks() {
        let mut state = healthy_state();
        state.safety_switch_enabled = true;
        state.safety_switch_state = true; // safety ON = block arming
        let result = check_prearm(&state, &ArmingConfig::default());
        assert!(!result.all_passed);
    }

    #[test]
    fn test_compass_not_calibrated_fails() {
        let mut state = healthy_state();
        state.compass_configured = false;
        let result = check_prearm(&state, &ArmingConfig::default());
        assert!(!result.all_passed);
    }

    #[test]
    fn test_already_flying_fails() {
        let mut state = healthy_state();
        state.is_flying = true;
        let result = check_prearm(&state, &ArmingConfig::default());
        assert!(!result.all_passed);
    }

    /// M4: GPS check should not block non-GPS modes (Stabilize, AltHold, Acro).
    #[test]
    fn test_no_gps_mode_arms_without_gps_fix() {
        let mut state = healthy_state();
        state.gps_fix_type = 0; // no GPS fix
        state.gps_num_sats = 0;
        // Should FAIL when arming into a GPS-required mode
        let result = check_prearm_for_mode(&state, &ArmingConfig::default(), ArmingMode::RequiresGps);
        assert!(!result.all_passed, "GPS-required mode should fail without GPS");
        // Should PASS when arming into a non-GPS mode
        let result = check_prearm_for_mode(&state, &ArmingConfig::default(), ArmingMode::NoGpsRequired);
        assert!(result.all_passed, "Non-GPS mode should pass without GPS. Failed: {:?}",
            result.checks.iter().filter(|c| !c.passed).collect::<heapless::Vec<_, 32>>());
    }

    /// L1: Acro without attitude estimation blocks arming.
    #[test]
    fn test_acro_without_attitude_fails() {
        let mut state = healthy_state();
        state.flight_mode = 1; // Acro
        state.attitude_valid = false;
        let result = check_prearm(&state, &ArmingConfig::default());
        assert!(!result.all_passed);
        assert!(result.checks.iter().any(|c| c.name.as_str() == "Mode" && !c.passed));
    }

    /// L1: Acro with valid attitude passes.
    #[test]
    fn test_acro_with_attitude_passes() {
        let mut state = healthy_state();
        state.flight_mode = 1; // Acro
        state.attitude_valid = true;
        let result = check_prearm(&state, &ArmingConfig::default());
        assert!(result.all_passed, "Acro with attitude should pass. Failed: {:?}",
            result.checks.iter().filter(|c| !c.passed).collect::<heapless::Vec<_, 32>>());
    }

    /// L1: Flip mode blocks arming directly.
    #[test]
    fn test_flip_arm_blocked() {
        let mut state = healthy_state();
        state.flight_mode = 14; // Flip
        let result = check_prearm(&state, &ArmingConfig::default());
        assert!(!result.all_passed);
        assert!(result.checks.iter().any(|c| c.name.as_str() == "Mode" && !c.passed));
    }

    /// L2: Auto mode without mission blocks arming.
    #[test]
    fn test_auto_no_mission_fails() {
        let mut state = healthy_state();
        state.flight_mode = 3; // Auto
        state.mission_loaded = false;
        state.mission_item_count = 0;
        let result = check_prearm(&state, &ArmingConfig::default());
        assert!(!result.all_passed);
        assert!(result.checks.iter().any(|c| c.name.as_str() == "Mission" && !c.passed));
    }

    /// L2: Auto mode with mission passes.
    #[test]
    fn test_auto_with_mission_passes() {
        let mut state = healthy_state();
        state.flight_mode = 3; // Auto
        state.mission_loaded = true;
        state.mission_item_count = 5;
        let result = check_prearm(&state, &ArmingConfig::default());
        assert!(result.all_passed, "Auto with mission should pass. Failed: {:?}",
            result.checks.iter().filter(|c| !c.passed).collect::<heapless::Vec<_, 32>>());
    }
}

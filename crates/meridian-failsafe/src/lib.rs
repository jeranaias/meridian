#![no_std]

//! Failsafe monitors and pre-arm checks.
//!
//! Every monitor is a concrete type — no dynamic dispatch, no heap.
//! Source: ArduCopter/failsafe.cpp, libraries/AP_Arming/AP_Arming.cpp

use meridian_types::time::{Instant, Duration};
use meridian_types::vehicle::{FailsafeReason, FailsafeAction};

// ─── Timeout Monitor ───

/// Generic timeout-based failsafe for RC, GPS, and comms loss.
#[derive(Debug, Clone)]
pub struct TimeoutMonitor {
    pub reason: FailsafeReason,
    pub action: FailsafeAction,
    pub timeout: Duration,
    last_seen: Instant,
    initialized: bool,
    triggered: bool,
}

impl TimeoutMonitor {
    pub fn new(reason: FailsafeReason, action: FailsafeAction, timeout_ms: u32) -> Self {
        Self {
            reason,
            action,
            timeout: Duration::from_millis(timeout_ms as u64),
            last_seen: Instant::ZERO,
            initialized: false,
            triggered: false,
        }
    }

    /// Call when the monitored signal is received (RC packet, GPS fix, GCS heartbeat).
    pub fn signal_received(&mut self, now: Instant) {
        self.last_seen = now;
        self.initialized = true;
        self.triggered = false;
    }

    /// Check if the timeout has elapsed. Returns action if triggered.
    pub fn check(&mut self, now: Instant) -> Option<(FailsafeReason, FailsafeAction)> {
        if !self.initialized {
            return None;
        }
        let elapsed = now.elapsed_since(self.last_seen);
        if elapsed >= self.timeout && !self.triggered {
            self.triggered = true;
            Some((self.reason, self.action))
        } else {
            None
        }
    }

    pub fn is_triggered(&self) -> bool {
        self.triggered
    }

    pub fn clear(&mut self) {
        self.triggered = false;
    }
}

// ─── Battery Monitor ───

/// Battery failsafe with voltage and capacity thresholds.
#[derive(Debug, Clone)]
pub struct BatteryMonitor {
    pub low_voltage: f32,
    pub critical_voltage: f32,
    pub low_mah_remaining: u32,
    pub low_action: FailsafeAction,
    pub critical_action: FailsafeAction,
    low_triggered: bool,
    critical_triggered: bool,
}

impl BatteryMonitor {
    pub fn new(
        low_v: f32, critical_v: f32, low_mah: u32,
        low_action: FailsafeAction, critical_action: FailsafeAction,
    ) -> Self {
        Self {
            low_voltage: low_v,
            critical_voltage: critical_v,
            low_mah_remaining: low_mah,
            low_action,
            critical_action,
            low_triggered: false,
            critical_triggered: false,
        }
    }

    /// Check battery state. Returns highest-priority triggered action.
    pub fn check(&mut self, voltage: f32, mah_remaining: u32)
        -> Option<(FailsafeReason, FailsafeAction)>
    {
        if voltage <= self.critical_voltage && !self.critical_triggered {
            self.critical_triggered = true;
            return Some((FailsafeReason::BatteryCritical, self.critical_action));
        }
        if (voltage <= self.low_voltage || mah_remaining <= self.low_mah_remaining)
            && !self.low_triggered
        {
            self.low_triggered = true;
            return Some((FailsafeReason::BatteryLow, self.low_action));
        }
        None
    }

    pub fn is_low(&self) -> bool { self.low_triggered }
    pub fn is_critical(&self) -> bool { self.critical_triggered }
}

// ─── EKF Health Monitor ───

/// EKF innovation-based health monitor.
#[derive(Debug, Clone)]
pub struct EkfHealthMonitor {
    pub action: FailsafeAction,
    pub max_vel_innov: f32,    // max acceptable velocity innovation (m/s)
    pub max_pos_innov: f32,    // max acceptable position innovation (m)
    unhealthy_count: u32,
    threshold_count: u32,      // how many consecutive unhealthy before triggering
    triggered: bool,
}

impl EkfHealthMonitor {
    pub fn new(action: FailsafeAction) -> Self {
        Self {
            action,
            max_vel_innov: 3.0,   // m/s — generous to avoid false triggers
            max_pos_innov: 10.0,  // m — allows normal GPS noise + drift
            unhealthy_count: 0,
            threshold_count: 50,  // 50 consecutive failures at 10Hz = 5s of bad EKF
            triggered: false,
        }
    }

    /// Check EKF innovations. Returns action if diverged.
    pub fn check(
        &mut self, vel_innov: &[f32; 3], pos_innov: &[f32; 2], healthy: bool,
    ) -> Option<(FailsafeReason, FailsafeAction)> {
        let vel_bad = vel_innov.iter().any(|v| v.abs() > self.max_vel_innov);
        let pos_bad = pos_innov.iter().any(|p| p.abs() > self.max_pos_innov);

        if !healthy || vel_bad || pos_bad {
            self.unhealthy_count += 1;
        } else {
            self.unhealthy_count = 0;
        }

        if self.unhealthy_count >= self.threshold_count && !self.triggered {
            self.triggered = true;
            Some((FailsafeReason::EkfFailure, self.action))
        } else {
            None
        }
    }

    pub fn is_triggered(&self) -> bool { self.triggered }
}

// ─── Geofence Monitor ───

/// Monitors geofence breach from meridian-fence.
#[derive(Debug, Clone)]
pub struct GeofenceMonitor {
    pub action: FailsafeAction,
    triggered: bool,
}

impl GeofenceMonitor {
    pub fn new(action: FailsafeAction) -> Self {
        Self { action, triggered: false }
    }

    /// Check if geofence is breached. `breached` comes from Geofence::check().
    pub fn check(&mut self, breached: bool) -> Option<(FailsafeReason, FailsafeAction)> {
        if breached && !self.triggered {
            self.triggered = true;
            Some((FailsafeReason::GeofenceBreach, self.action))
        } else {
            None
        }
    }

    pub fn clear(&mut self) { self.triggered = false; }
    pub fn is_triggered(&self) -> bool { self.triggered }
}

// ─── Motor Monitor ───

/// Monitors motor health via output imbalance detection.
/// Source: AP_MotorsMatrix::check_for_failed_motor()
#[derive(Debug, Clone)]
pub struct MotorMonitor {
    pub action: FailsafeAction,
    pub imbalance_threshold: f32, // ratio (1.5 = 50% imbalance)
    triggered: bool,
}

impl MotorMonitor {
    pub fn new(action: FailsafeAction) -> Self {
        Self { action, imbalance_threshold: 1.5, triggered: false }
    }

    /// Check motor outputs for imbalance.
    /// `outputs`: motor commands (0..1), `count`: number of active motors.
    pub fn check(&mut self, outputs: &[f32], count: usize) -> Option<(FailsafeReason, FailsafeAction)> {
        if count < 2 { return None; }
        let sum: f32 = outputs[..count].iter().sum();
        if sum < 0.01 { return None; } // motors not running
        let avg = sum / count as f32;
        let max = outputs[..count].iter().cloned().fold(0.0f32, f32::max);
        let ratio = max * count as f32 / sum;
        if ratio > self.imbalance_threshold && !self.triggered {
            self.triggered = true;
            Some((FailsafeReason::MotorFailure, self.action))
        } else {
            None
        }
    }

    pub fn is_triggered(&self) -> bool { self.triggered }
}

// ─── Crash Detector ───

/// Detects crashes and inverted flight.
/// Source: ArduCopter/crash_check.cpp
#[derive(Debug, Clone)]
pub struct CrashDetector {
    /// Consecutive inverted ticks.
    inverted_count: u32,
    /// Threshold before declaring crash.
    threshold: u32,
    triggered: bool,
}

impl CrashDetector {
    pub fn new() -> Self {
        Self { inverted_count: 0, threshold: 20, triggered: false } // 20 ticks at 10Hz = 2s
    }

    /// Check attitude for crash indicators.
    /// `roll`, `pitch` in radians. Armed state required.
    pub fn check(&mut self, roll: f32, pitch: f32, armed: bool) -> bool {
        if !armed {
            self.inverted_count = 0;
            self.triggered = false;
            return false;
        }
        // Inverted: roll or pitch > 60 degrees
        if roll.abs() > 1.05 || pitch.abs() > 1.05 {
            self.inverted_count += 1;
        } else {
            self.inverted_count = 0;
        }
        if self.inverted_count >= self.threshold && !self.triggered {
            self.triggered = true;
        }
        self.triggered
    }

    pub fn is_crashed(&self) -> bool { self.triggered }
}

// ─── Vibration Monitor ───

/// Monitors IMU vibration levels.
#[derive(Debug, Clone)]
pub struct VibrationMonitor {
    pub max_accel_clip: f32, // m/s² threshold for "clipping"
    clip_count: u32,
    total_samples: u32,
}

impl VibrationMonitor {
    pub fn new() -> Self {
        Self { max_accel_clip: 29.4, clip_count: 0, total_samples: 0 } // 3G default
    }

    /// Feed an accel sample magnitude. Returns true if vibration is concerning.
    pub fn check(&mut self, accel_magnitude: f32) -> bool {
        self.total_samples += 1;
        if accel_magnitude > self.max_accel_clip {
            self.clip_count += 1;
        }
        // Concerning if >1% of samples are clipping
        if self.total_samples > 100 {
            let ratio = self.clip_count as f32 / self.total_samples as f32;
            ratio > 0.01
        } else {
            false
        }
    }

    pub fn clip_count(&self) -> u32 { self.clip_count }
}

// ─── Dead Reckoning Monitor ───

/// Monitors dead reckoning state — EKF navigating without GPS.
/// Distinct from EkfHealthMonitor: EKF can report "healthy" while dead reckoning
/// on IMU-only, which degrades rapidly. Triggers after a configurable timeout.
#[derive(Debug, Clone)]
pub struct DeadReckoningMonitor {
    pub action: FailsafeAction,
    /// Timeout before triggering (default 20s).
    pub timeout: Duration,
    /// Timestamp of last valid GPS position.
    last_gps_time: Instant,
    /// Whether the monitor has been initialized with a valid GPS fix.
    initialized: bool,
    /// Whether dead reckoning mode is currently active.
    dead_reckoning: bool,
    triggered: bool,
}

impl DeadReckoningMonitor {
    pub fn new(action: FailsafeAction) -> Self {
        Self {
            action,
            timeout: Duration::from_secs(20),
            last_gps_time: Instant::ZERO,
            initialized: false,
            dead_reckoning: false,
            triggered: false,
        }
    }

    /// Call when a valid GPS position is received.
    pub fn gps_fix_received(&mut self, now: Instant) {
        self.last_gps_time = now;
        self.initialized = true;
        self.dead_reckoning = false;
        self.triggered = false;
    }

    /// Check dead reckoning state.
    /// `ekf_dead_reckoning`: true if EKF is navigating without GPS.
    pub fn check(&mut self, now: Instant, ekf_dead_reckoning: bool)
        -> Option<(FailsafeReason, FailsafeAction)>
    {
        if !self.initialized {
            return None;
        }
        self.dead_reckoning = ekf_dead_reckoning;
        if !ekf_dead_reckoning {
            self.last_gps_time = now;
            self.triggered = false;
            return None;
        }
        let elapsed = now.elapsed_since(self.last_gps_time);
        if elapsed >= self.timeout && !self.triggered {
            self.triggered = true;
            Some((FailsafeReason::DeadReckoning, self.action))
        } else {
            None
        }
    }

    pub fn is_triggered(&self) -> bool { self.triggered }
    pub fn is_dead_reckoning(&self) -> bool { self.dead_reckoning }
}

// ─── Terrain Monitor ───

/// Monitors terrain data availability.
/// If terrain data is required but stale (not updated within threshold), triggers RTL.
#[derive(Debug, Clone)]
pub struct TerrainMonitor {
    pub action: FailsafeAction,
    /// Maximum age of terrain data before declaring stale (default 10s).
    pub max_age: Duration,
    /// Timestamp of last valid terrain update.
    last_update: Instant,
    initialized: bool,
    triggered: bool,
}

impl TerrainMonitor {
    pub fn new(action: FailsafeAction) -> Self {
        Self {
            action,
            max_age: Duration::from_secs(10),
            last_update: Instant::ZERO,
            initialized: false,
            triggered: false,
        }
    }

    /// Call when terrain data is received/updated.
    pub fn terrain_updated(&mut self, now: Instant) {
        self.last_update = now;
        self.initialized = true;
        self.triggered = false;
    }

    /// Check terrain data freshness. `terrain_required`: whether current mode needs terrain.
    pub fn check(&mut self, now: Instant, terrain_required: bool)
        -> Option<(FailsafeReason, FailsafeAction)>
    {
        if !self.initialized || !terrain_required {
            return None;
        }
        let age = now.elapsed_since(self.last_update);
        if age >= self.max_age && !self.triggered {
            self.triggered = true;
            Some((FailsafeReason::TerrainStale, self.action))
        } else {
            None
        }
    }

    pub fn is_triggered(&self) -> bool { self.triggered }

    pub fn clear(&mut self) {
        self.triggered = false;
    }
}

// ─── Watchdog Monitor ───

/// Hardware watchdog: detects main loop stalls.
/// If `feed()` is not called within the timeout, motors are cut.
#[derive(Debug, Clone)]
pub struct WatchdogMonitor {
    last_feed: Instant,
    initialized: bool,
}

impl WatchdogMonitor {
    /// Motor-min threshold (seconds).
    pub const MOTOR_MIN_TIMEOUT: f32 = 2.0;
    /// Disarm threshold (seconds).
    pub const DISARM_TIMEOUT: f32 = 3.0;

    pub fn new() -> Self {
        Self {
            last_feed: Instant::ZERO,
            initialized: false,
        }
    }

    /// Feed the watchdog. Call every main loop tick.
    pub fn feed(&mut self, now: Instant) {
        self.last_feed = now;
        self.initialized = true;
    }

    /// Elapsed time since last feed (seconds). Returns 0 if not initialized.
    pub fn elapsed_since_feed(&self, now: Instant) -> f32 {
        if !self.initialized {
            return 0.0;
        }
        now.elapsed_since(self.last_feed).as_secs_f32()
    }

    /// Check watchdog state. Returns the action to take.
    pub fn check(&self, now: Instant) -> WatchdogState {
        if !self.initialized {
            return WatchdogState::Normal;
        }
        let elapsed = self.elapsed_since_feed(now);
        if elapsed >= Self::DISARM_TIMEOUT {
            WatchdogState::Disarm
        } else if elapsed >= Self::MOTOR_MIN_TIMEOUT {
            WatchdogState::MotorMin
        } else {
            WatchdogState::Normal
        }
    }
}

/// Watchdog state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WatchdogState {
    /// Normal operation.
    Normal,
    /// Motors to minimum — loop stall detected.
    MotorMin,
    /// Disarm — prolonged stall.
    Disarm,
}

// ─── Compass Multi-Consistency Check ───

/// Compares heading from multiple compasses and flags inconsistency
/// if heading differs by more than a threshold sustained for a duration.
#[derive(Debug, Clone)]
pub struct CompassConsistency {
    /// Maximum heading difference (radians) before flagging. Default: 30 degrees.
    pub max_diff_rad: f32,
    /// Sustained time (seconds) before flagging inconsistency. Default: 5.0s.
    pub sustain_time: f32,
    /// Accumulated time of inconsistency (seconds).
    inconsistent_time: f32,
    /// Whether inconsistency has been flagged.
    flagged: bool,
}

impl CompassConsistency {
    pub fn new() -> Self {
        Self {
            max_diff_rad: 30.0 * core::f32::consts::PI / 180.0, // 30 degrees
            sustain_time: 5.0,
            inconsistent_time: 0.0,
            flagged: false,
        }
    }

    /// Update with headings from two compasses (radians) and timestep.
    /// Returns true if inconsistency is flagged.
    pub fn update(&mut self, heading_1: f32, heading_2: f32, dt: f32) -> bool {
        // Compute shortest angular difference, wrapped to [-PI, PI]
        let pi = core::f32::consts::PI;
        let two_pi = 2.0 * pi;
        let mut diff = heading_1 - heading_2;
        // Wrap to [-PI, PI] using modular arithmetic without libm
        diff = diff % two_pi;
        if diff > pi { diff -= two_pi; }
        if diff < -pi { diff += two_pi; }
        let abs_diff = if diff < 0.0 { -diff } else { diff };

        if abs_diff > self.max_diff_rad {
            self.inconsistent_time += dt;
        } else {
            self.inconsistent_time = 0.0;
            self.flagged = false;
        }

        if self.inconsistent_time >= self.sustain_time - 0.001 {
            self.flagged = true;
        }

        self.flagged
    }

    /// Check if compass inconsistency is currently flagged.
    pub fn is_flagged(&self) -> bool {
        self.flagged
    }

    /// Reset the monitor.
    pub fn reset(&mut self) {
        self.inconsistent_time = 0.0;
        self.flagged = false;
    }
}

// ─── GPS Glitch Monitor ───

/// Monitors GPS innovation ratios from EKF to detect GPS glitches in flight.
/// Sustained high innovation ratio indicates GPS is providing bad data.
#[derive(Debug, Clone)]
pub struct GpsGlitchMonitor {
    pub action: FailsafeAction,
    /// Innovation ratio threshold (default 1.0).
    pub innov_threshold: f32,
    /// Sustained duration before declaring glitch (default 5s, as tick count at 10Hz = 50).
    pub threshold_count: u32,
    /// Consecutive ticks above threshold.
    high_innov_count: u32,
    triggered: bool,
}

impl GpsGlitchMonitor {
    pub fn new(action: FailsafeAction) -> Self {
        Self {
            action,
            innov_threshold: 1.0,
            threshold_count: 50, // 5s at 10Hz
            high_innov_count: 0,
            triggered: false,
        }
    }

    /// Check GPS innovation ratio from EKF.
    /// `gps_innov_ratio`: the GPS position innovation ratio (>1 = problematic).
    pub fn check(&mut self, gps_innov_ratio: f32) -> Option<(FailsafeReason, FailsafeAction)> {
        if gps_innov_ratio > self.innov_threshold {
            self.high_innov_count += 1;
        } else {
            self.high_innov_count = 0;
            self.triggered = false;
        }

        if self.high_innov_count >= self.threshold_count && !self.triggered {
            self.triggered = true;
            Some((FailsafeReason::GpsGlitch, self.action))
        } else {
            None
        }
    }

    pub fn is_triggered(&self) -> bool { self.triggered }
}

// ─── Failsafe Options ───

/// FS_OPTIONS-style continuation flags.
/// Source: ArduCopter parameters FS_OPTIONS bitmask.
/// Controls whether certain failsafes are suppressed when in autonomous modes
/// and provides alternative failsafe actions.
///
/// ArduPilot's 7 relevant flags:
///   Bit 0: RC_CONTINUE_IF_AUTO — don't RTL on RC loss if in Auto
///   Bit 1: GCS_CONTINUE_IF_AUTO — don't RTL on GCS loss if in Auto
///   Bit 2: RC_CONTINUE_IF_GUIDED — don't RTL on RC loss if in Guided
///   Bit 3: GCS_CONTINUE_IF_GUIDED — don't RTL on GCS loss if in Guided (custom)
///   Bit 4: SmartRTL→Land action override
///   Bit 5: Brake→Land action override
///   Bit 6: DO_LAND_START action override
///   Bit 7: CONTINUE_IF_LANDING — don't interrupt landing for RC/GCS loss
///   Bit 8: RC_CONTINUE_IF_PILOT_CONTROLLED — don't RTL on RC loss if in pilot mode
///   Bit 9: GCS_CONTINUE_IF_PILOT_CONTROL — don't RTL on GCS loss if in pilot mode
///   Bit 10: BATTERY_CONTINUE_IF_NOT_LANDED — battery failsafe only fires when landed
#[derive(Debug, Clone, Copy)]
pub struct FailsafeOptions {
    /// Bit 0: Continue mission if in Auto mode when RC is lost.
    pub continue_auto_on_rc_loss: bool,
    /// Bit 1: Continue mission if in Auto mode when GCS link is lost.
    pub continue_auto_on_gcs_loss: bool,
    /// Bit 2: Continue mission if in Guided mode when RC is lost.
    pub continue_guided_on_rc_loss: bool,
    /// Bit 3: Continue mission if in Guided mode when GCS link is lost.
    pub continue_guided_on_gcs_loss: bool,
    /// Bit 4: Use SmartRTL->Land as the failsafe action (try SmartRTL, fall back to Land).
    pub smartrtl_land: bool,
    /// Bit 5: Use Brake->Land as the failsafe action (try Brake, fall back to Land).
    pub brake_land: bool,
    /// Bit 6: Jump to mission's DO_LAND_START item on failsafe.
    pub do_land_start: bool,
    /// Bit 7: GAP 38 — Don't interrupt an active landing sequence for RC/GCS loss.
    pub continue_if_landing: bool,
    /// Bit 8: GAP 38 — Don't RTL on RC loss if in a pilot-controlled mode
    /// (Stabilize, AltHold, Acro, Sport, Drift).
    pub continue_pilot_on_rc_loss: bool,
    /// Bit 9: GAP 38 — Don't RTL on GCS loss if in a pilot-controlled mode.
    pub continue_pilot_on_gcs_loss: bool,
    /// Bit 10: GAP 38 — Battery failsafe only triggers when landed (not in flight).
    pub battery_continue_if_not_landed: bool,
}

impl FailsafeOptions {
    /// Construct from a raw bitmask (matches ArduPilot FS_OPTIONS parameter).
    pub fn from_bits(bits: u16) -> Self {
        Self {
            continue_auto_on_rc_loss: bits & (1 << 0) != 0,
            continue_auto_on_gcs_loss: bits & (1 << 1) != 0,
            continue_guided_on_rc_loss: bits & (1 << 2) != 0,
            continue_guided_on_gcs_loss: bits & (1 << 3) != 0,
            smartrtl_land: bits & (1 << 4) != 0,
            brake_land: bits & (1 << 5) != 0,
            do_land_start: bits & (1 << 6) != 0,
            continue_if_landing: bits & (1 << 7) != 0,
            continue_pilot_on_rc_loss: bits & (1 << 8) != 0,
            continue_pilot_on_gcs_loss: bits & (1 << 9) != 0,
            battery_continue_if_not_landed: bits & (1 << 10) != 0,
        }
    }

    /// Convert back to raw bitmask.
    pub fn to_bits(&self) -> u16 {
        let mut bits = 0u16;
        if self.continue_auto_on_rc_loss { bits |= 1 << 0; }
        if self.continue_auto_on_gcs_loss { bits |= 1 << 1; }
        if self.continue_guided_on_rc_loss { bits |= 1 << 2; }
        if self.continue_guided_on_gcs_loss { bits |= 1 << 3; }
        if self.smartrtl_land { bits |= 1 << 4; }
        if self.brake_land { bits |= 1 << 5; }
        if self.do_land_start { bits |= 1 << 6; }
        if self.continue_if_landing { bits |= 1 << 7; }
        if self.continue_pilot_on_rc_loss { bits |= 1 << 8; }
        if self.continue_pilot_on_gcs_loss { bits |= 1 << 9; }
        if self.battery_continue_if_not_landed { bits |= 1 << 10; }
        bits
    }

    /// Check if the given failsafe should be suppressed based on the current mode.
    /// Returns true if the failsafe should be suppressed (continue current mission).
    ///
    /// `in_auto`: vehicle is in Auto mode.
    /// `in_guided`: vehicle is in Guided mode.
    /// `in_landing`: vehicle is currently executing a landing sequence.
    /// `in_pilot_mode`: vehicle is in a pilot-controlled mode (Stabilize, AltHold, etc.).
    /// `is_landed`: vehicle has landed (for battery_continue_if_not_landed).
    pub fn should_suppress_ext(
        &self, reason: FailsafeReason,
        in_auto: bool, in_guided: bool,
        in_landing: bool, in_pilot_mode: bool,
        is_landed: bool,
    ) -> bool {
        match reason {
            FailsafeReason::RcLoss => {
                (in_auto && self.continue_auto_on_rc_loss)
                    || (in_guided && self.continue_guided_on_rc_loss)
                    || (in_landing && self.continue_if_landing)
                    || (in_pilot_mode && self.continue_pilot_on_rc_loss)
            }
            FailsafeReason::CommsLoss => {
                (in_auto && self.continue_auto_on_gcs_loss)
                    || (in_guided && self.continue_guided_on_gcs_loss)
                    || (in_landing && self.continue_if_landing)
                    || (in_pilot_mode && self.continue_pilot_on_gcs_loss)
            }
            FailsafeReason::BatteryLow | FailsafeReason::BatteryCritical => {
                // Suppress battery failsafe while LANDED (not while airborne).
                // The flag means "continue without battery failsafe when on the ground."
                self.battery_continue_if_not_landed && is_landed
            }
            _ => false,
        }
    }

    /// Backwards-compatible suppression check (no landing/pilot/landed context).
    pub fn should_suppress(
        &self, reason: FailsafeReason, in_auto: bool, in_guided: bool,
    ) -> bool {
        self.should_suppress_ext(reason, in_auto, in_guided, false, false, false)
    }

    /// Resolve the failsafe action, applying FS_OPTIONS action overrides (bits 4-6).
    /// Bit 6 (DO_LAND_START) takes precedence over bit 5 (BrakeLand) over bit 4 (SmartRtlLand).
    /// Only applies to actions that are RTL-class — does not override Land or Terminate.
    pub fn resolve_action(&self, base_action: FailsafeAction) -> FailsafeAction {
        match base_action {
            // Only override RTL-class actions
            FailsafeAction::ReturnToLaunch | FailsafeAction::SmartReturnToLaunch => {
                if self.do_land_start {
                    FailsafeAction::DoLandStart
                } else if self.brake_land {
                    FailsafeAction::BrakeLand
                } else if self.smartrtl_land {
                    FailsafeAction::SmartRtlLand
                } else {
                    base_action
                }
            }
            // Land, Terminate, Warn — no override
            _ => base_action,
        }
    }
}

impl Default for FailsafeOptions {
    fn default() -> Self {
        Self {
            continue_auto_on_rc_loss: false,
            continue_auto_on_gcs_loss: false,
            continue_guided_on_rc_loss: false,
            continue_guided_on_gcs_loss: false,
            smartrtl_land: false,
            brake_land: false,
            do_land_start: false,
            continue_if_landing: false,
            continue_pilot_on_rc_loss: false,
            continue_pilot_on_gcs_loss: false,
            battery_continue_if_not_landed: false,
        }
    }
}

// ─── Failsafe Reason Priority ───

/// Priority of failsafe reasons, from highest (Crash=10) to lowest (Geofence=1).
/// ArduPilot enforces strict priority: a higher-priority failsafe cannot be
/// downgraded by a lower-priority one.
/// Source: ArduCopter/failsafe.cpp priority order.
fn reason_priority(reason: FailsafeReason) -> u8 {
    match reason {
        FailsafeReason::Crash          => 12,
        FailsafeReason::ThrustLoss     => 11,
        FailsafeReason::MotorFailure   => 10,
        FailsafeReason::YawImbalance   => 9,
        FailsafeReason::BatteryCritical => 8,
        FailsafeReason::EkfFailure     => 7,
        FailsafeReason::CommsLoss      => 6,
        FailsafeReason::RcLoss         => 5,
        FailsafeReason::BatteryLow     => 4,
        FailsafeReason::TerrainStale   => 3,
        FailsafeReason::DeadReckoning  => 2,
        FailsafeReason::GeofenceBreach => 1,
        // Everything else (GpsGlitch, GnssLoss, WatchdogTimeout, etc.) at baseline
        _ => 0,
    }
}

// ─── Complete Failsafe Manager ───

/// All failsafe monitors in one struct.
/// Implements ArduPilot's priority escalation: once a high-priority failsafe
/// fires, lower-priority ones cannot downgrade the response. When the
/// highest-priority failsafe clears, the manager drops to the next-highest
/// active failsafe.
pub struct FailsafeManager {
    pub rc_loss: TimeoutMonitor,
    pub gps_loss: TimeoutMonitor,
    pub comms_loss: TimeoutMonitor,
    pub battery: BatteryMonitor,
    pub ekf: EkfHealthMonitor,
    pub geofence: GeofenceMonitor,
    pub motor: MotorMonitor,
    pub crash: CrashDetector,
    pub vibration: VibrationMonitor,
    pub dead_reckoning: DeadReckoningMonitor,
    pub terrain: TerrainMonitor,
    pub watchdog: WatchdogMonitor,
    pub gps_glitch: GpsGlitchMonitor,
    pub options: FailsafeOptions,
    /// Active failsafe events.
    active: heapless::Vec<(FailsafeReason, FailsafeAction), 16>,
    /// Current priority level — new failsafes must meet or exceed this to apply.
    current_priority: u8,
}

impl FailsafeManager {
    pub fn new() -> Self {
        Self {
            rc_loss: TimeoutMonitor::new(
                FailsafeReason::RcLoss, FailsafeAction::ReturnToLaunch, 500),
            gps_loss: TimeoutMonitor::new(
                FailsafeReason::GnssLoss, FailsafeAction::Land, 5000),
            comms_loss: TimeoutMonitor::new(
                FailsafeReason::CommsLoss, FailsafeAction::ReturnToLaunch, 15000),
            battery: BatteryMonitor::new(
                10.5, 10.0, 1000,
                FailsafeAction::ReturnToLaunch, FailsafeAction::Land),
            ekf: EkfHealthMonitor::new(FailsafeAction::Land),
            geofence: GeofenceMonitor::new(FailsafeAction::ReturnToLaunch),
            motor: MotorMonitor::new(FailsafeAction::Land),
            crash: CrashDetector::new(),
            vibration: VibrationMonitor::new(),
            dead_reckoning: DeadReckoningMonitor::new(FailsafeAction::Land),
            terrain: TerrainMonitor::new(FailsafeAction::ReturnToLaunch),
            watchdog: WatchdogMonitor::new(),
            gps_glitch: GpsGlitchMonitor::new(FailsafeAction::Land),
            options: FailsafeOptions::default(),
            active: heapless::Vec::new(),
            current_priority: 0,
        }
    }

    /// Check if a failsafe has cleared. Removes from active list and
    /// recalculates the current priority to the next-highest active failsafe.
    pub fn is_cleared(&mut self, reason: FailsafeReason) -> bool {
        let cleared = match reason {
            FailsafeReason::RcLoss => !self.rc_loss.is_triggered(),
            FailsafeReason::GnssLoss => !self.gps_loss.is_triggered(),
            FailsafeReason::CommsLoss => !self.comms_loss.is_triggered(),
            FailsafeReason::BatteryLow | FailsafeReason::BatteryCritical => false, // never clears
            FailsafeReason::EkfFailure => !self.ekf.is_triggered(),
            FailsafeReason::GeofenceBreach => !self.geofence.is_triggered(),
            FailsafeReason::MotorFailure => !self.motor.is_triggered(),
            FailsafeReason::DeadReckoning => !self.dead_reckoning.is_triggered(),
            FailsafeReason::TerrainStale => !self.terrain.is_triggered(),
            FailsafeReason::GpsGlitch => !self.gps_glitch.is_triggered(),
            FailsafeReason::Crash => !self.crash.is_crashed(),
            _ => false,
        };
        if cleared {
            self.active.retain(|(r, _)| *r != reason);
            // Recalculate current_priority to next-highest active failsafe
            self.current_priority = self.active.iter()
                .map(|(r, _)| reason_priority(*r))
                .fold(0u8, |max, p| if p > max { p } else { max });
        }
        cleared
    }

    /// Run all failsafe checks. Returns newly triggered events.
    ///
    /// `in_auto_mode`: true if the vehicle is currently in Auto mode (used by FS_OPTIONS).
    /// `ekf_dead_reckoning`: true if EKF is navigating without GPS.
    /// `terrain_required`: true if current mode requires terrain data.
    /// `gps_innov_ratio`: GPS position innovation ratio from EKF.
    pub fn check_all(
        &mut self,
        now: Instant,
        battery_voltage: f32,
        battery_mah: u32,
        ekf_vel_innov: &[f32; 3],
        ekf_pos_innov: &[f32; 2],
        ekf_healthy: bool,
    ) -> heapless::Vec<(FailsafeReason, FailsafeAction), 16> {
        self.check_all_extended(
            now, battery_voltage, battery_mah,
            ekf_vel_innov, ekf_pos_innov, ekf_healthy,
            false, false, false, 0.0,
        )
    }

    /// Extended check with all new monitor inputs.
    ///
    /// `in_auto_mode`: vehicle is in Auto mode.
    /// `in_guided_mode`: vehicle is in Guided mode.
    /// `ekf_dead_reckoning`: EKF is navigating without GPS.
    /// `terrain_required`: current mode requires terrain data.
    /// `gps_innov_ratio`: GPS position innovation ratio from EKF.
    pub fn check_all_extended(
        &mut self,
        now: Instant,
        battery_voltage: f32,
        battery_mah: u32,
        ekf_vel_innov: &[f32; 3],
        ekf_pos_innov: &[f32; 2],
        ekf_healthy: bool,
        in_auto_mode: bool,
        ekf_dead_reckoning: bool,
        terrain_required: bool,
        gps_innov_ratio: f32,
    ) -> heapless::Vec<(FailsafeReason, FailsafeAction), 16> {
        self.check_all_full(
            now, battery_voltage, battery_mah,
            ekf_vel_innov, ekf_pos_innov, ekf_healthy,
            in_auto_mode, false, ekf_dead_reckoning,
            terrain_required, gps_innov_ratio,
        )
    }

    /// Full check with Auto+Guided mode awareness for FS_OPTIONS bits 2-3.
    pub fn check_all_full(
        &mut self,
        now: Instant,
        battery_voltage: f32,
        battery_mah: u32,
        ekf_vel_innov: &[f32; 3],
        ekf_pos_innov: &[f32; 2],
        ekf_healthy: bool,
        in_auto_mode: bool,
        in_guided_mode: bool,
        ekf_dead_reckoning: bool,
        terrain_required: bool,
        gps_innov_ratio: f32,
    ) -> heapless::Vec<(FailsafeReason, FailsafeAction), 16> {
        let mut events = heapless::Vec::new();

        // Collect candidate events from all monitors
        let mut candidates: heapless::Vec<(FailsafeReason, FailsafeAction), 16> = heapless::Vec::new();

        // RC loss
        if let Some(e) = self.rc_loss.check(now) {
            if !self.options.should_suppress(e.0, in_auto_mode, in_guided_mode) {
                let _ = candidates.push(e);
            }
        }
        if let Some(e) = self.gps_loss.check(now) { let _ = candidates.push(e); }
        // GCS/Comms loss
        if let Some(e) = self.comms_loss.check(now) {
            if !self.options.should_suppress(e.0, in_auto_mode, in_guided_mode) {
                let _ = candidates.push(e);
            }
        }
        if let Some(e) = self.battery.check(battery_voltage, battery_mah) { let _ = candidates.push(e); }
        if let Some(e) = self.ekf.check(ekf_vel_innov, ekf_pos_innov, ekf_healthy) { let _ = candidates.push(e); }

        // Additional monitors
        if let Some(e) = self.dead_reckoning.check(now, ekf_dead_reckoning) { let _ = candidates.push(e); }
        if let Some(e) = self.terrain.check(now, terrain_required) { let _ = candidates.push(e); }
        if let Some(e) = self.gps_glitch.check(gps_innov_ratio) { let _ = candidates.push(e); }
        // Watchdog is checked separately via check_watchdog()
        // Geofence and motor are checked externally via their check() methods

        // Priority escalation: only apply events whose priority >= current_priority.
        // Apply FS_OPTIONS action overrides (bits 4-6).
        for &(reason, action) in candidates.iter() {
            let priority = reason_priority(reason);
            if priority >= self.current_priority {
                let resolved_action = self.options.resolve_action(action);
                let event = (reason, resolved_action);
                let _ = events.push(event);
                // Track active and escalate priority
                if !self.active.iter().any(|(r, _)| *r == reason) {
                    let _ = self.active.push(event);
                }
                if priority > self.current_priority {
                    self.current_priority = priority;
                }
            }
        }

        events
    }

    /// Check watchdog state. Call from main loop.
    pub fn check_watchdog(&self, now: Instant) -> WatchdogState {
        self.watchdog.check(now)
    }

    /// Feed the watchdog timer. Call every main loop tick.
    pub fn feed_watchdog(&mut self, now: Instant) {
        self.watchdog.feed(now);
    }

    /// Check geofence externally and feed result.
    /// Respects priority escalation: only applies if Geofence priority >= current.
    pub fn check_geofence(&mut self, breached: bool) -> Option<(FailsafeReason, FailsafeAction)> {
        let event = self.geofence.check(breached);
        if let Some((reason, action)) = event {
            let priority = reason_priority(reason);
            if priority >= self.current_priority {
                let resolved = self.options.resolve_action(action);
                let e = (reason, resolved);
                if !self.active.iter().any(|(r, _)| *r == reason) {
                    let _ = self.active.push(e);
                }
                if priority > self.current_priority {
                    self.current_priority = priority;
                }
                return Some(e);
            }
            // Lower priority than current — suppressed
            return None;
        }
        None
    }

    /// Check motor outputs externally.
    /// Respects priority escalation: only applies if MotorFailure priority >= current.
    pub fn check_motors(&mut self, outputs: &[f32], count: usize) -> Option<(FailsafeReason, FailsafeAction)> {
        let event = self.motor.check(outputs, count);
        if let Some((reason, action)) = event {
            let priority = reason_priority(reason);
            if priority >= self.current_priority {
                let resolved = self.options.resolve_action(action);
                let e = (reason, resolved);
                if !self.active.iter().any(|(r, _)| *r == reason) {
                    let _ = self.active.push(e);
                }
                if priority > self.current_priority {
                    self.current_priority = priority;
                }
                return Some(e);
            }
            return None;
        }
        None
    }

    /// Check crash detector externally and feed result.
    /// CrashDetector is the highest-priority failsafe — always escalates.
    pub fn check_crash(&mut self, roll: f32, pitch: f32, armed: bool) -> Option<(FailsafeReason, FailsafeAction)> {
        let crashed = self.crash.check(roll, pitch, armed);
        if crashed {
            let reason = FailsafeReason::Crash;
            let action = FailsafeAction::Terminate;
            let priority = reason_priority(reason);
            if priority >= self.current_priority {
                let e = (reason, action);
                if !self.active.iter().any(|(r, _)| *r == reason) {
                    let _ = self.active.push(e);
                }
                self.current_priority = priority;
                return Some(e);
            }
        }
        None
    }

    /// Get the current priority level.
    pub fn current_priority(&self) -> u8 {
        self.current_priority
    }

    /// Get highest-priority active failsafe action (Land > RTL > Warn).
    pub fn highest_priority_action(&self) -> Option<FailsafeAction> {
        let mut highest: Option<FailsafeAction> = None;
        for (_, action) in self.active.iter() {
            let priority = match action {
                FailsafeAction::Terminate => 6,
                FailsafeAction::Land => 5,
                FailsafeAction::SmartRtlLand => 4,
                FailsafeAction::BrakeLand => 4,
                FailsafeAction::SmartReturnToLaunch => 3,
                FailsafeAction::ReturnToLaunch => 2,
                FailsafeAction::DoLandStart => 2,
                FailsafeAction::Warn => 1,
            };
            let current_priority = highest.map(|a| match a {
                FailsafeAction::Terminate => 6,
                FailsafeAction::Land => 5,
                FailsafeAction::SmartRtlLand => 4,
                FailsafeAction::BrakeLand => 4,
                FailsafeAction::SmartReturnToLaunch => 3,
                FailsafeAction::ReturnToLaunch => 2,
                FailsafeAction::DoLandStart => 2,
                FailsafeAction::Warn => 1,
            }).unwrap_or(0);
            if priority > current_priority {
                highest = Some(*action);
            }
        }
        highest
    }

    pub fn any_active(&self) -> bool {
        !self.active.is_empty()
    }

    pub fn active_reasons(&self) -> &[(FailsafeReason, FailsafeAction)] {
        &self.active
    }
}

// ─── Pre-Arm Checks ───

/// Pre-arm check result.
#[derive(Debug, Clone)]
pub struct PreArmResult {
    pub passed: bool,
    pub failures: heapless::Vec<heapless::String<64>, 8>,
}

/// Run pre-arm checks. Returns pass/fail with reasons.
pub fn check_pre_arm(
    ekf_healthy: bool,
    gps_fix_3d: bool,
    gps_num_sats: u8,
    battery_voltage: f32,
    min_voltage: f32,
    rc_connected: bool,
    gyro_calibrated: bool,
    accel_calibrated: bool,
) -> PreArmResult {
    let mut failures: heapless::Vec<heapless::String<64>, 8> = heapless::Vec::new();

    if !ekf_healthy {
        let mut s = heapless::String::new();
        let _ = s.push_str("EKF not healthy");
        let _ = failures.push(s);
    }
    if !gps_fix_3d {
        let mut s = heapless::String::new();
        let _ = s.push_str("No GPS 3D fix");
        let _ = failures.push(s);
    }
    if gps_num_sats < 6 {
        let mut s = heapless::String::new();
        let _ = s.push_str("GPS sats < 6");
        let _ = failures.push(s);
    }
    if battery_voltage < min_voltage {
        let mut s = heapless::String::new();
        let _ = s.push_str("Battery voltage low");
        let _ = failures.push(s);
    }
    if !rc_connected {
        let mut s = heapless::String::new();
        let _ = s.push_str("RC not connected");
        let _ = failures.push(s);
    }
    if !gyro_calibrated {
        let mut s = heapless::String::new();
        let _ = s.push_str("Gyro not calibrated");
        let _ = failures.push(s);
    }
    if !accel_calibrated {
        let mut s = heapless::String::new();
        let _ = s.push_str("Accel not calibrated");
        let _ = failures.push(s);
    }

    PreArmResult {
        passed: failures.is_empty(),
        failures,
    }
}

// ─── Motor Interlock ───

/// Prevents motor spin during arm/disarm transitions.
/// Source: ArduCopter/motors.cpp lines 93-101
#[derive(Debug, Clone)]
pub struct MotorInterlock {
    enabled: bool,
    arm_delay_ms: u32,
    arm_time: Instant,
}

impl MotorInterlock {
    pub fn new(arm_delay_ms: u32) -> Self {
        Self { enabled: false, arm_delay_ms, arm_time: Instant::ZERO }
    }

    /// Call when vehicle arms.
    pub fn on_arm(&mut self, now: Instant) {
        self.arm_time = now;
    }

    /// Call when vehicle disarms.
    pub fn on_disarm(&mut self) {
        self.enabled = false;
    }

    /// Check if motors are allowed to spin. False during arm delay.
    pub fn motors_allowed(&self, now: Instant) -> bool {
        if !self.enabled { return false; }
        let elapsed = now.elapsed_since(self.arm_time);
        elapsed.as_millis() >= self.arm_delay_ms as u64
    }

    /// Enable the interlock (arm sequence complete).
    pub fn enable(&mut self) { self.enabled = true; }
    pub fn is_enabled(&self) -> bool { self.enabled }
}

// ─── Land Detector ───

/// Multi-factor landing detection with auto-disarm.
/// Source: ArduCopter/land_detector.cpp
///
/// ArduPilot checks 8 conditions before declaring "landed":
/// 1. Throttle output at minimum (motors at spin_min)
/// 2. Throttle-RPY mix at minimum (no significant attitude corrections)
/// 3. Attitude error small (< 30 deg from target)
/// 4. Acceleration magnitude near 1g (low vibration, not in free-fall)
/// 5. Vertical speed near zero (|climb_rate| < 0.5 m/s)
/// 6. Not ascending (climb_rate <= 0)
/// 7. Rangefinder shows close to ground (if available)
/// 8. All conditions sustained for at least 1 second
#[derive(Debug, Clone)]
pub struct LandDetector {
    /// Consecutive ticks where all landing conditions are met.
    landed_count: u32,
    /// Ticks required before declaring landed (1s at 10Hz = 10 ticks).
    threshold: u32,
    landed: bool,
    /// Auto-disarm delay after landing (seconds).
    pub disarm_delay_s: f32,
    land_detected_time: Instant,
}

/// Input to the land detector with all 8 ArduPilot conditions.
/// Source: ArduCopter/land_detector.cpp
#[derive(Debug, Clone, Copy)]
pub struct LandDetectorInput {
    /// Condition 1: Throttle output at minimum (motors at spin_min).
    pub throttle_at_min: bool,
    /// Condition 2: Throttle-RPY mix at minimum (attitude corrections negligible).
    pub throttle_mix_at_min: bool,
    /// Condition 3: Attitude error small (< 30 deg from target).
    pub small_angle_error: bool,
    /// Condition 4: Acceleration magnitude near 1g (9.81 +/- 3 m/s^2).
    pub accel_near_1g: bool,
    /// Condition 5: Vertical speed near zero (|climb_rate| < 0.5 m/s).
    pub low_climb_rate: bool,
    /// Condition 6: Not ascending (climb_rate <= 0).
    pub not_ascending: bool,
    /// Condition 7: Rangefinder shows close to ground, or rangefinder not available.
    /// Pass `true` when no rangefinder is installed (condition is satisfied by default).
    pub rangefinder_near_ground: bool,
    /// Vehicle is armed.
    pub armed: bool,
}

impl Default for LandDetectorInput {
    fn default() -> Self {
        Self {
            throttle_at_min: false,
            throttle_mix_at_min: false,
            small_angle_error: false,
            accel_near_1g: false,
            low_climb_rate: false,
            not_ascending: false,
            rangefinder_near_ground: true, // default: no rangefinder → always satisfied
            armed: false,
        }
    }
}

impl LandDetector {
    pub fn new() -> Self {
        Self {
            landed_count: 0, threshold: 10, // 1s at 10Hz
            landed: false, disarm_delay_s: 5.0,
            land_detected_time: Instant::ZERO,
        }
    }

    /// Check all 8 landing conditions. Call at 10Hz.
    /// Source: ArduCopter/land_detector.cpp
    ///
    /// All 8 conditions must be true simultaneously for `threshold` consecutive
    /// ticks (condition 8: sustained for at least 1 second) before `landed` is set.
    pub fn update_full(&mut self, input: &LandDetectorInput) -> bool {
        if !input.armed {
            self.landed = false;
            self.landed_count = 0;
            return false;
        }

        let all_conditions = input.throttle_at_min
            && input.throttle_mix_at_min
            && input.small_angle_error
            && input.accel_near_1g
            && input.low_climb_rate
            && input.not_ascending
            && input.rangefinder_near_ground;

        if all_conditions {
            self.landed_count += 1;
        } else {
            self.landed_count = 0;
        }

        if self.landed_count >= self.threshold && !self.landed {
            self.landed = true;
        }

        self.landed
    }

    /// Legacy 4-condition check for backward compatibility.
    /// Wraps the full 8-condition check with sensible defaults for the new fields.
    /// Source: ArduCopter/land_detector.cpp criteria
    pub fn update(
        &mut self,
        throttle_at_min: bool,   // motors at minimum output
        low_accel: bool,         // vertical accel < 3 m/s² above gravity
        small_angle_error: bool, // attitude error < 30°
        low_climb_rate: bool,    // |climb_rate| < 0.5 m/s
        armed: bool,
    ) -> bool {
        let input = LandDetectorInput {
            throttle_at_min,
            throttle_mix_at_min: throttle_at_min, // assume mix tracks throttle
            small_angle_error,
            accel_near_1g: low_accel,
            low_climb_rate,
            not_ascending: low_climb_rate,        // if low climb rate, also not ascending
            rangefinder_near_ground: true,         // no rangefinder → always satisfied
            armed,
        };
        self.update_full(&input)
    }

    /// Should the vehicle auto-disarm?
    pub fn should_disarm(&self, now: Instant) -> bool {
        if !self.landed { return false; }
        if self.land_detected_time == Instant::ZERO { return false; }
        let elapsed = now.elapsed_since(self.land_detected_time);
        elapsed.as_secs_f32() >= self.disarm_delay_s
    }

    /// Record the landing time (call once when landed first becomes true).
    pub fn set_land_time(&mut self, now: Instant) {
        if self.landed && self.land_detected_time == Instant::ZERO {
            self.land_detected_time = now;
        }
    }

    pub fn is_landed(&self) -> bool { self.landed }

    /// How long landing conditions have been sustained (in ticks).
    pub fn landed_duration_ticks(&self) -> u32 { self.landed_count }

    pub fn reset(&mut self) {
        self.landed = false;
        self.landed_count = 0;
        self.land_detected_time = Instant::ZERO;
    }
}

// ─── Thrust Loss Detector ───

/// Detects motor/ESC failure mid-flight.
/// Source: ArduCopter/crash_check.cpp lines 99-181
#[derive(Debug, Clone)]
pub struct ThrustLossDetector {
    loss_count: u32,
    threshold: u32,
    detected: bool,
}

impl ThrustLossDetector {
    pub fn new() -> Self {
        Self { loss_count: 0, threshold: 10, detected: false } // 1s at 10Hz
    }

    /// Check for thrust loss condition.
    /// Criteria: high throttle + level attitude + descending
    pub fn check(
        &mut self,
        throttle_pct: f32,   // 0-100
        lean_angle: f32,     // radians (total lean from vertical)
        climb_rate: f32,     // m/s (positive = climbing)
        attitude_error: f32, // radians (error from target)
    ) -> bool {
        let high_throttle = throttle_pct > 90.0;
        let level = lean_angle < 0.26; // <15°
        let descending = climb_rate < -0.5; // falling >0.5 m/s
        let small_error = attitude_error < 0.26; // attitude close to target

        if high_throttle && level && descending && small_error {
            self.loss_count += 1;
        } else {
            self.loss_count = 0;
        }

        if self.loss_count >= self.threshold && !self.detected {
            self.detected = true;
        }
        self.detected
    }

    pub fn is_detected(&self) -> bool { self.detected }
}

// ─── Yaw Imbalance Detector (GAP 36) ───

/// Detects excessive yaw correction requirement indicating motor/prop damage.
/// Source: ArduCopter/crash_check.cpp yaw_imbalance_check()
///
/// If the yaw error remains high (> threshold) while the controller is
/// commanding full yaw correction for a sustained duration, a motor is
/// likely damaged or producing asymmetric thrust.
#[derive(Debug, Clone)]
pub struct YawImbalanceDetector {
    imbalance_count: u32,
    threshold: u32,
    detected: bool,
}

impl YawImbalanceDetector {
    pub fn new() -> Self {
        Self { imbalance_count: 0, threshold: 30, detected: false } // 3s at 10Hz
    }

    /// Check for yaw imbalance.
    /// `yaw_error_rad`: difference between target and actual yaw (radians).
    /// `yaw_output_pct`: yaw motor output as percent of max (0-100).
    pub fn check(&mut self, yaw_error_rad: f32, yaw_output_pct: f32) -> bool {
        // Imbalance: large yaw error AND controller is commanding near-max yaw correction
        let large_error = yaw_error_rad.abs() > 0.52; // > 30 degrees
        let high_output = yaw_output_pct > 80.0;

        if large_error && high_output {
            self.imbalance_count += 1;
        } else {
            self.imbalance_count = 0;
        }

        if self.imbalance_count >= self.threshold && !self.detected {
            self.detected = true;
        }
        self.detected
    }

    pub fn is_detected(&self) -> bool { self.detected }
}

// ─── Enhanced Pre-Arm Checks ───

/// Extended pre-arm check including lean angle and parameter validation.
pub fn check_pre_arm_extended(
    ekf_healthy: bool,
    gps_fix_3d: bool,
    gps_num_sats: u8,
    battery_voltage: f32,
    min_voltage: f32,
    rc_connected: bool,
    gyro_calibrated: bool,
    accel_calibrated: bool,
    // New checks from ArduPilot scan:
    lean_angle_deg: f32,     // current lean from level
    max_lean_for_arm: f32,   // max allowed lean at arm (default 15°)
    logging_ok: bool,
    compass_consistent: bool,
) -> PreArmResult {
    let mut failures: heapless::Vec<heapless::String<64>, 8> = heapless::Vec::new();

    let mut fail = |msg: &str| {
        let mut s = heapless::String::new();
        let _ = s.push_str(msg);
        let _ = failures.push(s);
    };

    if !ekf_healthy { fail("EKF not healthy"); }
    if !gps_fix_3d { fail("No GPS 3D fix"); }
    if gps_num_sats < 6 { fail("GPS sats < 6"); }
    if battery_voltage < min_voltage { fail("Battery voltage low"); }
    if !rc_connected { fail("RC not connected"); }
    if !gyro_calibrated { fail("Gyro not calibrated"); }
    if !accel_calibrated { fail("Accel not calibrated"); }
    if lean_angle_deg > max_lean_for_arm { fail("Vehicle not level"); }
    if !logging_ok { fail("Logging not available"); }
    if !compass_consistent { fail("Compass inconsistent"); }

    PreArmResult {
        passed: failures.is_empty(),
        failures,
    }
}

// ─── L3: GPS-aware failsafe action resolution ───

/// Resolve a failsafe action considering GPS availability.
///
/// L3 fix: If the failsafe requests RTL but GPS is not valid, fall back to Land.
/// Without this, RC loss triggers RTL without checking GPS validity, which causes
/// the EKF to navigate blind and the vehicle to fly away.
///
/// `action`: the requested failsafe action.
/// `gps_valid`: whether the GPS has a valid 3D fix.
///
/// Returns: the resolved action (may be downgraded to Land).
pub fn resolve_failsafe_action(action: FailsafeAction, gps_valid: bool) -> FailsafeAction {
    if !gps_valid {
        match action {
            FailsafeAction::ReturnToLaunch => FailsafeAction::Land,
            FailsafeAction::SmartReturnToLaunch => FailsafeAction::Land,
            FailsafeAction::SmartRtlLand => FailsafeAction::Land,
            other => other,
        }
    } else {
        action
    }
}

// ─── L4: Ground idle disarm timer (dead man's switch) ───

/// Monitors ground idle state and auto-disarms if armed on ground with low
/// throttle for too long. Prevents a forgotten armed vehicle from sitting
/// with motors spinning indefinitely.
///
/// Source: ArduCopter's DISARM_DELAY parameter / motors_output.cpp
#[derive(Debug, Clone)]
pub struct GroundIdleDisarm {
    /// Time (seconds) of continuous idle on ground before auto-disarm.
    pub timeout_s: f32,
    /// Accumulated idle time (seconds).
    idle_time: f32,
    /// Whether auto-disarm has been triggered.
    triggered: bool,
}

impl GroundIdleDisarm {
    /// Default: 15 seconds of ground idle → auto-disarm.
    pub fn new() -> Self {
        Self {
            timeout_s: 15.0,
            idle_time: 0.0,
            triggered: false,
        }
    }

    /// Create with a custom timeout.
    pub fn with_timeout(timeout_s: f32) -> Self {
        Self {
            timeout_s,
            idle_time: 0.0,
            triggered: false,
        }
    }

    /// Call every control loop tick.
    ///
    /// `on_ground`: true if the vehicle is on the ground (land detector).
    /// `throttle_low`: true if throttle is below idle threshold (~5%).
    /// `armed`: true if the vehicle is armed.
    /// `dt`: timestep (seconds).
    ///
    /// Returns true if auto-disarm should happen.
    pub fn update(&mut self, on_ground: bool, throttle_low: bool, armed: bool, dt: f32) -> bool {
        if !armed {
            self.idle_time = 0.0;
            self.triggered = false;
            return false;
        }

        if on_ground && throttle_low {
            self.idle_time += dt;
        } else {
            self.idle_time = 0.0;
            self.triggered = false;
        }

        if self.idle_time >= self.timeout_s && !self.triggered {
            self.triggered = true;
            return true;
        }

        false
    }

    pub fn is_triggered(&self) -> bool {
        self.triggered
    }

    pub fn reset(&mut self) {
        self.idle_time = 0.0;
        self.triggered = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_timeout_triggers() {
        let mut mon = TimeoutMonitor::new(
            FailsafeReason::RcLoss, FailsafeAction::ReturnToLaunch, 500);

        let t0 = Instant::from_micros(0);
        mon.signal_received(t0);

        // Before timeout: no trigger
        assert!(mon.check(Instant::from_micros(400_000)).is_none());

        // After timeout: trigger
        let event = mon.check(Instant::from_micros(600_000));
        assert!(event.is_some());
        assert_eq!(event.unwrap().0, FailsafeReason::RcLoss);

        // Doesn't re-trigger
        assert!(mon.check(Instant::from_micros(700_000)).is_none());
    }

    #[test]
    fn test_timeout_clears_on_signal() {
        let mut mon = TimeoutMonitor::new(
            FailsafeReason::GnssLoss, FailsafeAction::Land, 1000);

        mon.signal_received(Instant::from_micros(0));
        // Trigger
        let _ = mon.check(Instant::from_micros(1_100_000));
        assert!(mon.is_triggered());

        // Signal received again — clear
        mon.signal_received(Instant::from_micros(1_200_000));
        assert!(!mon.is_triggered());
    }

    #[test]
    fn test_battery_low() {
        let mut bat = BatteryMonitor::new(10.5, 10.0, 1000,
            FailsafeAction::ReturnToLaunch, FailsafeAction::Land);

        // Normal voltage
        assert!(bat.check(11.0, 2000).is_none());

        // Low voltage
        let event = bat.check(10.3, 2000);
        assert!(event.is_some());
        assert_eq!(event.unwrap().0, FailsafeReason::BatteryLow);
    }

    #[test]
    fn test_battery_critical() {
        let mut bat = BatteryMonitor::new(10.5, 10.0, 1000,
            FailsafeAction::ReturnToLaunch, FailsafeAction::Land);

        // Skip low, go straight to critical
        let event = bat.check(9.8, 500);
        assert!(event.is_some());
        assert_eq!(event.unwrap().0, FailsafeReason::BatteryCritical);
        assert_eq!(event.unwrap().1, FailsafeAction::Land);
    }

    #[test]
    fn test_ekf_health() {
        let mut ekf = EkfHealthMonitor::new(FailsafeAction::Land);

        // Healthy
        for _ in 0..20 {
            assert!(ekf.check(&[0.1, 0.1, 0.1], &[0.5, 0.5], true).is_none());
        }

        // Unhealthy for 50 consecutive checks (threshold_count=50)
        for i in 0..55 {
            let result = ekf.check(&[5.0, 5.0, 5.0], &[20.0, 20.0], false);
            if i >= 49 {
                assert!(result.is_some() || ekf.is_triggered());
            }
        }
        assert!(ekf.is_triggered());
    }

    #[test]
    fn test_failsafe_manager_rc_loss() {
        let mut mgr = FailsafeManager::new();
        mgr.rc_loss.signal_received(Instant::from_micros(0));

        // No events before timeout
        let events = mgr.check_all(Instant::from_micros(300_000), 12.0, 5000, &[0.0;3], &[0.0;2], true);
        assert!(events.is_empty());

        // RC loss after 500ms
        let events = mgr.check_all(Instant::from_micros(600_000), 12.0, 5000, &[0.0;3], &[0.0;2], true);
        assert!(!events.is_empty());
        assert_eq!(mgr.highest_priority_action(), Some(FailsafeAction::ReturnToLaunch));
    }

    #[test]
    fn test_priority_ordering() {
        let mut mgr = FailsafeManager::new();
        mgr.rc_loss.signal_received(Instant::from_micros(0));

        // Trigger RC loss (RTL, priority 5) at 600ms
        mgr.check_all(Instant::from_micros(600_000), 12.0, 5000, &[0.0;3], &[0.0;2], true);
        assert_eq!(mgr.highest_priority_action(), Some(FailsafeAction::ReturnToLaunch));

        // Trigger battery critical (Land, priority 8) — escalates past RC loss
        mgr.check_all(Instant::from_micros(700_000), 9.8, 500, &[0.0;3], &[0.0;2], true);

        // Land should be highest priority (battery critical > RC loss)
        assert_eq!(mgr.highest_priority_action(), Some(FailsafeAction::Land));
    }

    #[test]
    fn test_pre_arm_all_pass() {
        let result = check_pre_arm(true, true, 12, 12.0, 10.0, true, true, true);
        assert!(result.passed);
        assert!(result.failures.is_empty());
    }

    #[test]
    fn test_pre_arm_failures() {
        let result = check_pre_arm(false, false, 3, 9.0, 10.0, false, false, false);
        assert!(!result.passed);
        assert!(result.failures.len() >= 5);
    }

    #[test]
    fn test_motor_interlock() {
        let mut interlock = MotorInterlock::new(500); // 500ms arm delay
        interlock.enable();
        interlock.on_arm(Instant::from_micros(1_000_000));
        // During delay: motors NOT allowed
        assert!(!interlock.motors_allowed(Instant::from_micros(1_200_000)));
        // After delay: motors allowed
        assert!(interlock.motors_allowed(Instant::from_micros(1_600_000)));
        // On disarm: motors NOT allowed
        interlock.on_disarm();
        assert!(!interlock.motors_allowed(Instant::from_micros(2_000_000)));
    }

    #[test]
    fn test_land_detector() {
        let mut det = LandDetector::new();
        // Not landed in flight
        for _ in 0..20 {
            assert!(!det.update(false, true, true, true, true));
        }
        // All conditions met for 1+ seconds
        for _ in 0..15 {
            det.update(true, true, true, true, true);
        }
        assert!(det.is_landed());
    }

    #[test]
    fn test_land_detector_resets_on_disarm() {
        let mut det = LandDetector::new();
        for _ in 0..15 { det.update(true, true, true, true, true); }
        assert!(det.is_landed());
        det.update(false, false, false, false, false); // disarmed
        assert!(!det.is_landed());
    }

    #[test]
    fn test_thrust_loss() {
        let mut tl = ThrustLossDetector::new();
        // Normal flight: no detection
        assert!(!tl.check(50.0, 0.1, 0.0, 0.1));
        // Thrust loss: full throttle, level, descending
        for _ in 0..15 {
            tl.check(95.0, 0.1, -1.5, 0.1);
        }
        assert!(tl.is_detected());
    }

    #[test]
    fn test_extended_pre_arm_lean_check() {
        // Tilted vehicle should fail
        let result = check_pre_arm_extended(
            true, true, 12, 12.0, 10.0, true, true, true,
            20.0, 15.0, true, true, // lean=20° > max=15°
        );
        assert!(!result.passed);

        // Level vehicle should pass
        let result = check_pre_arm_extended(
            true, true, 12, 12.0, 10.0, true, true, true,
            5.0, 15.0, true, true,
        );
        assert!(result.passed);
    }

    #[test]
    fn test_geofence_monitor() {
        let mut gm = GeofenceMonitor::new(FailsafeAction::ReturnToLaunch);
        assert!(gm.check(false).is_none()); // no breach
        let event = gm.check(true); // breach!
        assert!(event.is_some());
        assert_eq!(event.unwrap().0, FailsafeReason::GeofenceBreach);
        // Doesn't re-trigger
        assert!(gm.check(true).is_none());
        // Can clear
        gm.clear();
        assert!(!gm.is_triggered());
    }

    #[test]
    fn test_vibration_monitor() {
        let mut vm = VibrationMonitor::new();
        // Normal: no concern
        for _ in 0..200 {
            assert!(!vm.check(9.81)); // 1G = normal
        }
        // High vibration: concern
        let mut vm2 = VibrationMonitor::new();
        for _ in 0..50 { vm2.check(9.81); } // some normal
        for _ in 0..60 { vm2.check(35.0); } // clipping!
        assert!(vm2.check(35.0)); // >1% clipping
    }

    // ─── Dead Reckoning Tests ───

    #[test]
    fn test_dead_reckoning_triggers_after_timeout() {
        let mut dr = DeadReckoningMonitor::new(FailsafeAction::Land);
        let t0 = Instant::from_micros(0);
        dr.gps_fix_received(t0);

        // Not dead reckoning: no trigger
        assert!(dr.check(Instant::from_micros(1_000_000), false).is_none());

        // Enter dead reckoning — not yet timed out
        assert!(dr.check(Instant::from_micros(5_000_000), true).is_none());

        // Still dead reckoning at 19s — no trigger yet
        assert!(dr.check(Instant::from_micros(19_000_000), true).is_none());

        // At 21s — should trigger (20s timeout from last GPS at t0)
        let event = dr.check(Instant::from_micros(21_000_000), true);
        assert!(event.is_some());
        assert_eq!(event.unwrap().0, FailsafeReason::DeadReckoning);
    }

    #[test]
    fn test_dead_reckoning_clears_on_gps_fix() {
        let mut dr = DeadReckoningMonitor::new(FailsafeAction::Land);
        dr.gps_fix_received(Instant::from_micros(0));

        // Trigger
        let _ = dr.check(Instant::from_micros(21_000_000), true);
        assert!(dr.is_triggered());

        // GPS fix received — clears
        dr.gps_fix_received(Instant::from_micros(22_000_000));
        assert!(!dr.is_triggered());
    }

    #[test]
    fn test_dead_reckoning_not_triggered_without_init() {
        let mut dr = DeadReckoningMonitor::new(FailsafeAction::Land);
        // Never got a GPS fix
        assert!(dr.check(Instant::from_micros(30_000_000), true).is_none());
    }

    // ─── Terrain Monitor Tests ───

    #[test]
    fn test_terrain_stale_triggers() {
        let mut tm = TerrainMonitor::new(FailsafeAction::ReturnToLaunch);
        tm.terrain_updated(Instant::from_micros(0));

        // Not stale yet
        assert!(tm.check(Instant::from_micros(5_000_000), true).is_none());

        // Stale after 10s
        let event = tm.check(Instant::from_micros(11_000_000), true);
        assert!(event.is_some());
        assert_eq!(event.unwrap().0, FailsafeReason::TerrainStale);
        assert_eq!(event.unwrap().1, FailsafeAction::ReturnToLaunch);
    }

    #[test]
    fn test_terrain_not_required_no_trigger() {
        let mut tm = TerrainMonitor::new(FailsafeAction::ReturnToLaunch);
        tm.terrain_updated(Instant::from_micros(0));

        // Stale but not required — no trigger
        assert!(tm.check(Instant::from_micros(20_000_000), false).is_none());
    }

    #[test]
    fn test_terrain_clears_on_update() {
        let mut tm = TerrainMonitor::new(FailsafeAction::ReturnToLaunch);
        tm.terrain_updated(Instant::from_micros(0));
        let _ = tm.check(Instant::from_micros(11_000_000), true);
        assert!(tm.is_triggered());

        // New data arrives
        tm.terrain_updated(Instant::from_micros(12_000_000));
        assert!(!tm.is_triggered());
    }

    // ─── Watchdog Tests ───

    #[test]
    fn test_watchdog_normal() {
        let mut wd = WatchdogMonitor::new();
        wd.feed(Instant::from_micros(0));
        assert_eq!(wd.check(Instant::from_micros(500_000)), WatchdogState::Normal);
    }

    #[test]
    fn test_watchdog_motor_min() {
        let mut wd = WatchdogMonitor::new();
        wd.feed(Instant::from_micros(0));
        // 2.5s since feed — motor min
        assert_eq!(wd.check(Instant::from_micros(2_500_000)), WatchdogState::MotorMin);
    }

    #[test]
    fn test_watchdog_disarm() {
        let mut wd = WatchdogMonitor::new();
        wd.feed(Instant::from_micros(0));
        // 3.5s since feed — disarm
        assert_eq!(wd.check(Instant::from_micros(3_500_000)), WatchdogState::Disarm);
    }

    #[test]
    fn test_watchdog_feed_resets() {
        let mut wd = WatchdogMonitor::new();
        wd.feed(Instant::from_micros(0));
        assert_eq!(wd.check(Instant::from_micros(2_500_000)), WatchdogState::MotorMin);
        wd.feed(Instant::from_micros(2_500_000));
        assert_eq!(wd.check(Instant::from_micros(3_000_000)), WatchdogState::Normal);
    }

    #[test]
    fn test_watchdog_not_initialized() {
        let wd = WatchdogMonitor::new();
        // Not initialized — always normal
        assert_eq!(wd.check(Instant::from_micros(10_000_000)), WatchdogState::Normal);
    }

    // ─── GPS Glitch Tests ───

    #[test]
    fn test_gps_glitch_triggers_after_sustained() {
        let mut gm = GpsGlitchMonitor::new(FailsafeAction::Land);

        // Normal innovation
        for _ in 0..20 {
            assert!(gm.check(0.5).is_none());
        }

        // High innovation for 50 ticks (5s at 10Hz)
        for i in 0..55 {
            let result = gm.check(1.5);
            if i >= 49 {
                assert!(result.is_some() || gm.is_triggered());
            }
        }
        assert!(gm.is_triggered());
    }

    #[test]
    fn test_gps_glitch_clears_on_good_data() {
        let mut gm = GpsGlitchMonitor::new(FailsafeAction::Land);

        // Build up count
        for _ in 0..30 {
            gm.check(1.5);
        }
        assert!(!gm.is_triggered()); // not yet

        // Good data resets count
        gm.check(0.5);
        assert!(!gm.is_triggered());

        // Would need 50 more consecutive bad readings
        for _ in 0..30 {
            gm.check(1.5);
        }
        assert!(!gm.is_triggered()); // still not enough
    }

    #[test]
    fn test_gps_glitch_doesnt_retrigger() {
        let mut gm = GpsGlitchMonitor::new(FailsafeAction::Land);
        for _ in 0..55 {
            gm.check(1.5);
        }
        assert!(gm.is_triggered());
        // Further checks should not produce new events
        assert!(gm.check(1.5).is_none());
    }

    // ─── FailsafeOptions Tests ───

    #[test]
    fn test_fs_options_continue_auto_on_rc_loss() {
        let mut mgr = FailsafeManager::new();
        mgr.options.continue_auto_on_rc_loss = true;
        mgr.rc_loss.signal_received(Instant::from_micros(0));

        // In Auto mode with continue_auto_on_rc_loss — RC loss suppressed
        let events = mgr.check_all_extended(
            Instant::from_micros(600_000), 12.0, 5000,
            &[0.0; 3], &[0.0; 2], true,
            true, false, false, 0.0,
        );
        assert!(events.is_empty(), "RC loss should be suppressed in Auto mode");
    }

    #[test]
    fn test_fs_options_no_suppress_when_not_auto() {
        let mut mgr = FailsafeManager::new();
        mgr.options.continue_auto_on_rc_loss = true;
        mgr.rc_loss.signal_received(Instant::from_micros(0));

        // Not in Auto mode — RC loss should trigger normally
        let events = mgr.check_all_extended(
            Instant::from_micros(600_000), 12.0, 5000,
            &[0.0; 3], &[0.0; 2], true,
            false, false, false, 0.0,
        );
        assert!(!events.is_empty(), "RC loss should trigger when not in Auto mode");
    }

    #[test]
    fn test_fs_options_continue_auto_on_gcs_loss() {
        let mut mgr = FailsafeManager::new();
        mgr.options.continue_auto_on_gcs_loss = true;
        mgr.comms_loss.signal_received(Instant::from_micros(0));

        // In Auto mode with continue_auto_on_gcs_loss — comms loss suppressed
        let events = mgr.check_all_extended(
            Instant::from_micros(16_000_000), 12.0, 5000,
            &[0.0; 3], &[0.0; 2], true,
            true, false, false, 0.0,
        );
        // Only comms_loss would trigger; should be suppressed
        let has_comms = events.iter().any(|(r, _)| *r == FailsafeReason::CommsLoss);
        assert!(!has_comms, "Comms loss should be suppressed in Auto mode");
    }

    // ─── Dead Reckoning in FailsafeManager ───

    #[test]
    fn test_manager_dead_reckoning() {
        let mut mgr = FailsafeManager::new();
        mgr.dead_reckoning.gps_fix_received(Instant::from_micros(0));

        // Dead reckoning for > 20s
        let events = mgr.check_all_extended(
            Instant::from_micros(21_000_000), 12.0, 5000,
            &[0.0; 3], &[0.0; 2], true,
            false, true, false, 0.0,
        );
        let has_dr = events.iter().any(|(r, _)| *r == FailsafeReason::DeadReckoning);
        assert!(has_dr, "should trigger dead reckoning failsafe");
    }

    // ─── GPS Glitch in FailsafeManager ───

    #[test]
    fn test_manager_gps_glitch() {
        let mut mgr = FailsafeManager::new();
        // Initialize timeouts so they don't fire
        let t0 = Instant::from_micros(0);
        mgr.rc_loss.signal_received(t0);
        mgr.gps_loss.signal_received(t0);
        mgr.comms_loss.signal_received(t0);

        // Feed high innovation for 55 ticks
        for i in 0..55u64 {
            let now = Instant::from_micros(i * 100_000); // 10Hz ticks
            mgr.rc_loss.signal_received(now);
            mgr.gps_loss.signal_received(now);
            mgr.comms_loss.signal_received(now);
            let events = mgr.check_all_extended(
                now, 12.0, 5000,
                &[0.0; 3], &[0.0; 2], true,
                false, false, false, 1.5, // high innovation
            );
            if i >= 50 {
                let has_glitch = events.iter().any(|(r, _)| *r == FailsafeReason::GpsGlitch)
                    || mgr.gps_glitch.is_triggered();
                assert!(has_glitch, "should have GPS glitch by tick {}", i);
            }
        }
    }

    // ─── Watchdog in FailsafeManager ───

    #[test]
    fn test_manager_watchdog() {
        let mut mgr = FailsafeManager::new();
        mgr.feed_watchdog(Instant::from_micros(0));

        assert_eq!(mgr.check_watchdog(Instant::from_micros(500_000)), WatchdogState::Normal);
        assert_eq!(mgr.check_watchdog(Instant::from_micros(2_500_000)), WatchdogState::MotorMin);
        assert_eq!(mgr.check_watchdog(Instant::from_micros(3_500_000)), WatchdogState::Disarm);
    }

    // ── Compass Consistency Tests ──

    #[test]
    fn test_compass_consistent_no_flag() {
        let mut cc = CompassConsistency::new();
        // Same heading — no flag
        for _ in 0..100 {
            assert!(!cc.update(1.0, 1.0, 0.1));
        }
        assert!(!cc.is_flagged());
    }

    #[test]
    fn test_compass_small_diff_no_flag() {
        let mut cc = CompassConsistency::new();
        // 20 degree difference (< 30 degree threshold)
        let diff = 20.0 * core::f32::consts::PI / 180.0;
        for _ in 0..100 {
            assert!(!cc.update(1.0, 1.0 + diff, 0.1));
        }
        assert!(!cc.is_flagged());
    }

    #[test]
    fn test_compass_large_diff_flags_after_sustained() {
        let mut cc = CompassConsistency::new();
        // 45 degree difference (> 30 degree threshold)
        let diff = 45.0 * core::f32::consts::PI / 180.0;
        // Needs 5s sustained. Use dt=0.25 → 20 ticks = 5.0s exactly
        for _ in 0..19 {
            assert!(!cc.update(1.0, 1.0 + diff, 0.25));
        }
        // 20th tick → 5.0s → should flag
        assert!(cc.update(1.0, 1.0 + diff, 0.25));
        assert!(cc.is_flagged());
    }

    #[test]
    fn test_compass_diff_resets_on_consistent() {
        let mut cc = CompassConsistency::new();
        let diff = 45.0 * core::f32::consts::PI / 180.0;

        // Build up 3s of inconsistency (12 ticks * 0.25s = 3.0s)
        for _ in 0..12 {
            cc.update(1.0, 1.0 + diff, 0.25);
        }
        assert!(!cc.is_flagged());

        // One consistent reading resets the counter
        cc.update(1.0, 1.0, 0.25);
        assert!(!cc.is_flagged());

        // Need another 5s sustained to flag (20 ticks * 0.25s = 5.0s)
        for _ in 0..19 {
            cc.update(1.0, 1.0 + diff, 0.25);
        }
        assert!(!cc.is_flagged()); // only 4.75s
        assert!(cc.update(1.0, 1.0 + diff, 0.25)); // 5.0s → flagged
    }

    #[test]
    fn test_compass_wrapping_handling() {
        let mut cc = CompassConsistency::new();
        use core::f32::consts::PI;
        // Headings near +-PI should compute correct shortest path
        // heading_1 = PI - 0.1, heading_2 = -PI + 0.1 → actual diff = 0.2 rad (~11 deg)
        for _ in 0..100 {
            assert!(!cc.update(PI - 0.1, -PI + 0.1, 0.1));
        }
        assert!(!cc.is_flagged());
    }

    #[test]
    fn test_compass_reset() {
        let mut cc = CompassConsistency::new();
        let diff = 45.0 * core::f32::consts::PI / 180.0;
        for _ in 0..60 {
            cc.update(1.0, 1.0 + diff, 0.1);
        }
        assert!(cc.is_flagged());
        cc.reset();
        assert!(!cc.is_flagged());
    }

    // ─── Priority Escalation Tests ───

    #[test]
    fn test_priority_escalation_blocks_lower() {
        // Once a higher-priority failsafe fires, lower-priority ones are blocked.
        let mut mgr = FailsafeManager::new();
        let t0 = Instant::from_micros(0);
        mgr.rc_loss.signal_received(t0);
        mgr.gps_loss.signal_received(t0);
        mgr.comms_loss.signal_received(t0);

        // Trigger EKF failure (priority 7) via sustained bad innovation
        for i in 0..55u64 {
            let now = Instant::from_micros(i * 100_000);
            mgr.rc_loss.signal_received(now);
            mgr.gps_loss.signal_received(now);
            mgr.comms_loss.signal_received(now);
            mgr.check_all_extended(
                now, 12.0, 5000,
                &[5.0, 5.0, 5.0], &[20.0, 20.0], false,
                false, false, false, 0.0,
            );
        }
        assert!(mgr.ekf.is_triggered());
        assert_eq!(mgr.current_priority(), reason_priority(FailsafeReason::EkfFailure));

        // Now try to trigger RC loss (priority 5) - should be suppressed
        let now = Instant::from_micros(6_000_000);
        // Don't send RC signal so it times out
        let events = mgr.check_all_extended(
            now, 12.0, 5000,
            &[0.1, 0.1, 0.1], &[0.5, 0.5], true,
            false, false, false, 0.0,
        );
        // RC loss should NOT appear since it's lower priority than EKF
        let has_rc = events.iter().any(|(r, _)| *r == FailsafeReason::RcLoss);
        assert!(!has_rc, "RC loss (priority 5) should be blocked by active EKF failure (priority 7)");
    }

    #[test]
    fn test_priority_drops_on_clear() {
        let mut mgr = FailsafeManager::new();
        let t0 = Instant::from_micros(0);
        mgr.rc_loss.signal_received(t0);
        mgr.gps_loss.signal_received(t0);
        mgr.comms_loss.signal_received(t0);

        // Trigger geofence (priority 1)
        mgr.check_geofence(true);
        assert_eq!(mgr.current_priority(), reason_priority(FailsafeReason::GeofenceBreach));

        // Clear geofence
        mgr.geofence.clear();
        assert!(mgr.is_cleared(FailsafeReason::GeofenceBreach));
        // Priority should drop to 0 (no active failsafes)
        assert_eq!(mgr.current_priority(), 0);
    }

    #[test]
    fn test_priority_drops_to_next_highest() {
        let mut mgr = FailsafeManager::new();
        let t0 = Instant::from_micros(0);
        mgr.rc_loss.signal_received(t0);
        mgr.gps_loss.signal_received(t0);
        mgr.comms_loss.signal_received(t0);

        // Trigger geofence (priority 1)
        mgr.check_geofence(true);

        // Trigger RC loss (priority 5) — higher, so it applies
        let now = Instant::from_micros(600_000);
        mgr.check_all_extended(
            now, 12.0, 5000, &[0.0; 3], &[0.0; 2], true,
            false, false, false, 0.0,
        );
        assert_eq!(mgr.current_priority(), reason_priority(FailsafeReason::RcLoss));

        // Clear RC loss
        mgr.rc_loss.signal_received(Instant::from_micros(700_000));
        assert!(mgr.is_cleared(FailsafeReason::RcLoss));
        // Priority should drop to geofence level (1), not 0
        assert_eq!(mgr.current_priority(), reason_priority(FailsafeReason::GeofenceBreach));
    }

    #[test]
    fn test_crash_is_highest_priority() {
        let mut mgr = FailsafeManager::new();

        // Trigger crash (priority 12 — highest)
        for _ in 0..25 {
            mgr.check_crash(2.0, 0.0, true); // >60 deg roll, armed
        }
        assert!(mgr.crash.is_crashed());
        assert_eq!(mgr.current_priority(), reason_priority(FailsafeReason::Crash));
        assert_eq!(mgr.current_priority(), 12);
    }

    // ─── FS_OPTIONS Extended Tests ───

    #[test]
    fn test_fs_options_guided_rc_suppress() {
        let mut mgr = FailsafeManager::new();
        mgr.options.continue_guided_on_rc_loss = true;
        mgr.rc_loss.signal_received(Instant::from_micros(0));

        // In Guided mode — RC loss suppressed
        let events = mgr.check_all_full(
            Instant::from_micros(600_000), 12.0, 5000,
            &[0.0; 3], &[0.0; 2], true,
            false, true, false, false, 0.0,
        );
        assert!(events.is_empty(), "RC loss should be suppressed in Guided mode");
    }

    #[test]
    fn test_fs_options_guided_gcs_suppress() {
        let mut mgr = FailsafeManager::new();
        mgr.options.continue_guided_on_gcs_loss = true;
        mgr.comms_loss.signal_received(Instant::from_micros(0));

        // In Guided mode — GCS loss suppressed
        let events = mgr.check_all_full(
            Instant::from_micros(16_000_000), 12.0, 5000,
            &[0.0; 3], &[0.0; 2], true,
            false, true, false, false, 0.0,
        );
        let has_comms = events.iter().any(|(r, _)| *r == FailsafeReason::CommsLoss);
        assert!(!has_comms, "Comms loss should be suppressed in Guided mode");
    }

    #[test]
    fn test_fs_options_smartrtl_land_action() {
        let mut mgr = FailsafeManager::new();
        mgr.options.smartrtl_land = true;
        mgr.rc_loss.signal_received(Instant::from_micros(0));

        // RC loss triggers RTL normally; with bit 4, action becomes SmartRtlLand
        let events = mgr.check_all_extended(
            Instant::from_micros(600_000), 12.0, 5000,
            &[0.0; 3], &[0.0; 2], true,
            false, false, false, 0.0,
        );
        assert!(!events.is_empty());
        assert_eq!(events[0].1, FailsafeAction::SmartRtlLand);
    }

    #[test]
    fn test_fs_options_brake_land_action() {
        let mut mgr = FailsafeManager::new();
        mgr.options.brake_land = true;
        mgr.rc_loss.signal_received(Instant::from_micros(0));

        let events = mgr.check_all_extended(
            Instant::from_micros(600_000), 12.0, 5000,
            &[0.0; 3], &[0.0; 2], true,
            false, false, false, 0.0,
        );
        assert!(!events.is_empty());
        assert_eq!(events[0].1, FailsafeAction::BrakeLand);
    }

    #[test]
    fn test_fs_options_do_land_start_action() {
        let mut mgr = FailsafeManager::new();
        mgr.options.do_land_start = true;
        mgr.rc_loss.signal_received(Instant::from_micros(0));

        let events = mgr.check_all_extended(
            Instant::from_micros(600_000), 12.0, 5000,
            &[0.0; 3], &[0.0; 2], true,
            false, false, false, 0.0,
        );
        assert!(!events.is_empty());
        assert_eq!(events[0].1, FailsafeAction::DoLandStart);
    }

    #[test]
    fn test_fs_options_bit6_overrides_bit5_and_bit4() {
        // Bit 6 (DO_LAND_START) should take precedence when multiple bits set
        let mut mgr = FailsafeManager::new();
        mgr.options.smartrtl_land = true;
        mgr.options.brake_land = true;
        mgr.options.do_land_start = true;
        mgr.rc_loss.signal_received(Instant::from_micros(0));

        let events = mgr.check_all_extended(
            Instant::from_micros(600_000), 12.0, 5000,
            &[0.0; 3], &[0.0; 2], true,
            false, false, false, 0.0,
        );
        assert_eq!(events[0].1, FailsafeAction::DoLandStart);
    }

    #[test]
    fn test_fs_options_from_bits_roundtrip() {
        let opts = FailsafeOptions::from_bits(0b0111_1111);
        assert!(opts.continue_auto_on_rc_loss);
        assert!(opts.continue_auto_on_gcs_loss);
        assert!(opts.continue_guided_on_rc_loss);
        assert!(opts.continue_guided_on_gcs_loss);
        assert!(opts.smartrtl_land);
        assert!(opts.brake_land);
        assert!(opts.do_land_start);
        assert_eq!(opts.to_bits(), 0b0111_1111);

        let opts2 = FailsafeOptions::from_bits(0b0000_0101); // bits 0 and 2
        assert!(opts2.continue_auto_on_rc_loss);
        assert!(!opts2.continue_auto_on_gcs_loss);
        assert!(opts2.continue_guided_on_rc_loss);
        assert!(!opts2.continue_guided_on_gcs_loss);
        assert_eq!(opts2.to_bits(), 0b0000_0101);
    }

    #[test]
    fn test_fs_options_no_override_on_land() {
        // FS_OPTIONS action overrides should NOT affect Land actions
        let opts = FailsafeOptions::from_bits(0b0111_0000); // all action bits
        assert_eq!(opts.resolve_action(FailsafeAction::Land), FailsafeAction::Land);
        assert_eq!(opts.resolve_action(FailsafeAction::Terminate), FailsafeAction::Terminate);
    }

    // ─── GAP 38: Extended FS_OPTIONS flags tests ───

    #[test]
    fn test_fs_options_continue_if_landing() {
        let mut opts = FailsafeOptions::default();
        opts.continue_if_landing = true;
        // RC loss suppressed during landing
        assert!(opts.should_suppress_ext(
            FailsafeReason::RcLoss, false, false, true, false, false));
        // GCS loss suppressed during landing
        assert!(opts.should_suppress_ext(
            FailsafeReason::CommsLoss, false, false, true, false, false));
        // Not suppressed when NOT landing
        assert!(!opts.should_suppress_ext(
            FailsafeReason::RcLoss, false, false, false, false, false));
    }

    #[test]
    fn test_fs_options_pilot_mode_rc_suppress() {
        let mut opts = FailsafeOptions::default();
        opts.continue_pilot_on_rc_loss = true;
        // RC loss suppressed in pilot mode
        assert!(opts.should_suppress_ext(
            FailsafeReason::RcLoss, false, false, false, true, false));
        // Not suppressed in auto mode
        assert!(!opts.should_suppress_ext(
            FailsafeReason::RcLoss, false, false, false, false, false));
    }

    #[test]
    fn test_fs_options_pilot_mode_gcs_suppress() {
        let mut opts = FailsafeOptions::default();
        opts.continue_pilot_on_gcs_loss = true;
        assert!(opts.should_suppress_ext(
            FailsafeReason::CommsLoss, false, false, false, true, false));
        assert!(!opts.should_suppress_ext(
            FailsafeReason::CommsLoss, false, false, false, false, false));
    }

    #[test]
    fn test_fs_options_battery_continue_if_not_landed() {
        let mut opts = FailsafeOptions::default();
        opts.battery_continue_if_not_landed = true;
        // Battery low suppressed when LANDED (on the ground) — don't trigger failsafe on pad
        assert!(opts.should_suppress_ext(
            FailsafeReason::BatteryLow, false, false, false, false, true));
        // Battery low NOT suppressed when NOT landed (airborne) — failsafe fires in flight
        assert!(!opts.should_suppress_ext(
            FailsafeReason::BatteryLow, false, false, false, false, false));
        // Battery critical same behavior
        assert!(opts.should_suppress_ext(
            FailsafeReason::BatteryCritical, false, false, false, false, true));
        assert!(!opts.should_suppress_ext(
            FailsafeReason::BatteryCritical, false, false, false, false, false));
    }

    #[test]
    fn test_fs_options_extended_bits_roundtrip() {
        let opts = FailsafeOptions::from_bits(0b0111_1111_1111);
        assert!(opts.continue_auto_on_rc_loss);
        assert!(opts.continue_auto_on_gcs_loss);
        assert!(opts.continue_guided_on_rc_loss);
        assert!(opts.continue_guided_on_gcs_loss);
        assert!(opts.smartrtl_land);
        assert!(opts.brake_land);
        assert!(opts.do_land_start);
        assert!(opts.continue_if_landing);
        assert!(opts.continue_pilot_on_rc_loss);
        assert!(opts.continue_pilot_on_gcs_loss);
        assert!(opts.battery_continue_if_not_landed);
        assert_eq!(opts.to_bits(), 0b0111_1111_1111);
    }

    // ─── GAP 36: Yaw Imbalance Detector ───

    #[test]
    fn test_yaw_imbalance_detects() {
        let mut yid = YawImbalanceDetector::new();
        // Normal: no detection
        assert!(!yid.check(0.1, 30.0));
        // Large yaw error + high output for 3+ seconds
        for _ in 0..35 {
            yid.check(0.7, 85.0); // >30 deg error, >80% output
        }
        assert!(yid.is_detected());
    }

    #[test]
    fn test_yaw_imbalance_no_false_positive() {
        let mut yid = YawImbalanceDetector::new();
        // Large error but low output = not imbalance (vehicle just not commanded)
        for _ in 0..50 {
            yid.check(1.0, 20.0);
        }
        assert!(!yid.is_detected());
        // High output but small error = normal coordinated flight
        for _ in 0..50 {
            yid.check(0.1, 90.0);
        }
        assert!(!yid.is_detected());
    }

    // ─── Thrust loss + yaw imbalance priority ───

    #[test]
    fn test_thrust_loss_priority_above_motor_failure() {
        assert!(reason_priority(FailsafeReason::ThrustLoss) >
                reason_priority(FailsafeReason::MotorFailure));
    }

    #[test]
    fn test_yaw_imbalance_priority_above_battery_critical() {
        assert!(reason_priority(FailsafeReason::YawImbalance) >
                reason_priority(FailsafeReason::BatteryCritical));
    }

    // ─── L3: GPS-aware failsafe action resolution ───

    #[test]
    fn test_rtl_downgrades_to_land_without_gps() {
        let action = resolve_failsafe_action(FailsafeAction::ReturnToLaunch, false);
        assert_eq!(action, FailsafeAction::Land);
    }

    #[test]
    fn test_smart_rtl_downgrades_to_land_without_gps() {
        let action = resolve_failsafe_action(FailsafeAction::SmartReturnToLaunch, false);
        assert_eq!(action, FailsafeAction::Land);
    }

    #[test]
    fn test_rtl_preserved_with_gps() {
        let action = resolve_failsafe_action(FailsafeAction::ReturnToLaunch, true);
        assert_eq!(action, FailsafeAction::ReturnToLaunch);
    }

    #[test]
    fn test_land_unchanged_without_gps() {
        let action = resolve_failsafe_action(FailsafeAction::Land, false);
        assert_eq!(action, FailsafeAction::Land);
    }

    // ─── L4: Ground idle disarm (dead man's switch) ───

    #[test]
    fn test_ground_idle_disarm_triggers_after_timeout() {
        let mut gid = GroundIdleDisarm::with_timeout(15.0);
        let dt = 0.1; // 10 Hz
        // Simulate 14.9 seconds on ground with low throttle
        for _ in 0..149 {
            assert!(!gid.update(true, true, true, dt));
        }
        // 15.0 seconds — should trigger
        assert!(gid.update(true, true, true, dt));
        assert!(gid.is_triggered());
    }

    #[test]
    fn test_ground_idle_resets_on_throttle() {
        let mut gid = GroundIdleDisarm::with_timeout(15.0);
        let dt = 0.1;
        // 10 seconds idle
        for _ in 0..100 {
            gid.update(true, true, true, dt);
        }
        // Throttle up — resets
        gid.update(true, false, true, dt);
        // Another 14 seconds idle — should NOT trigger yet
        for _ in 0..140 {
            assert!(!gid.update(true, true, true, dt));
        }
    }

    #[test]
    fn test_ground_idle_no_trigger_when_disarmed() {
        let mut gid = GroundIdleDisarm::with_timeout(15.0);
        let dt = 0.1;
        for _ in 0..200 {
            assert!(!gid.update(true, true, false, dt));
        }
    }
}


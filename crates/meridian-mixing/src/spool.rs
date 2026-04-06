//! Motor spool state machine.
//!
//! Manages motor startup/shutdown sequence to prevent ESC issues.
//! Source: AP_MotorsMulticopter.cpp output_logic()
//!
//! States: SHUT_DOWN -> GROUND_IDLE -> SPOOLING_UP -> THROTTLE_UNLIMITED -> SPOOLING_DOWN

/// Spool state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpoolState {
    /// Motors off. Zero PWM output.
    ShutDown,
    /// Armed, motors spinning at idle (spin_arm). Waiting for throttle.
    GroundIdle,
    /// Ramping up from idle to full authority.
    SpoolingUp,
    /// Full authority. Normal flight.
    ThrottleUnlimited,
    /// Ramping down to idle.
    SpoolingDown,
}

/// Desired spool state (from flight controller).
/// Source: AP_MotorsMulticopter DesiredSpoolState
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DesiredSpoolState {
    ShutDown,
    GroundIdle,
    ThrottleUnlimited,
}

/// Spool state machine configuration.
#[derive(Debug, Clone, Copy)]
pub struct SpoolConfig {
    /// Time to ramp from idle to unlimited (seconds). Source: MOT_SPOOL_TIME, default 0.5
    pub spool_up_time: f32,
    /// Time to ramp from unlimited to idle (seconds). Default: 0.0 (uses spool_up_time).
    /// Source: MOT_SPOOL_TIM_DN, default 0 in AP (falls back to up time).
    pub spool_down_time: f32,
    /// ESC startup delay (seconds). Source: MOT_SAFE_TIME, default 0.0
    pub safe_time: f32,
    /// Idle motor output (fraction 0..1). Source: MOT_SPIN_ARM, default 0.10
    pub spin_arm: f32,
    /// Minimum in-flight motor output (fraction 0..1). Source: MOT_SPIN_MIN, default 0.15
    pub spin_min: f32,
    /// IDLE_TIME: hold at ground idle after reaching spin_arm before advancing (seconds).
    /// Source: AP_MotorsMulticopter idle_time_delay_s, default 0.0
    pub idle_time: f32,
}

/// Minimum spool time to prevent divide-by-zero.
/// Source: AP_MotorsMulticopter.cpp constexpr 0.05f
pub const MINIMUM_SPOOL_TIME: f32 = 0.05;

impl Default for SpoolConfig {
    fn default() -> Self {
        Self {
            spool_up_time: 0.5,
            spool_down_time: 0.0,  // 0 = use spool_up_time (matches AP default)
            safe_time: 0.0,
            spin_arm: 0.10,
            spin_min: 0.15,
            idle_time: 0.0,
        }
    }
}

impl SpoolConfig {
    /// Effective spool-up time (clamped to minimum).
    pub fn effective_spool_up(&self) -> f32 {
        self.spool_up_time.max(MINIMUM_SPOOL_TIME)
    }

    /// Effective spool-down time (falls back to spool_up if 0, clamped to minimum).
    pub fn effective_spool_down(&self) -> f32 {
        let t = if self.spool_down_time > 0.0 { self.spool_down_time } else { self.spool_up_time };
        t.max(MINIMUM_SPOOL_TIME)
    }
}

/// Spoolup block check callback type.
/// Returns `true` if spoolup should be blocked (ESC not ready, etc).
/// Source: AP_MotorsMulticopter::get_spoolup_block()
pub type SpoolupBlockFn = fn() -> bool;

/// Motor spool state machine.
pub struct SpoolManager {
    pub config: SpoolConfig,
    pub state: SpoolState,
    /// Ramp progress: 0.0 (idle) to 1.0 (unlimited).
    ramp: f32,
    /// Time spent in current state (seconds).
    state_time: f32,
    /// Idle time counter: tracks time at ground idle before advancing.
    idle_timer: f32,
    /// Optional spoolup block check.
    spoolup_block: Option<SpoolupBlockFn>,
}

impl SpoolManager {
    pub fn new(config: SpoolConfig) -> Self {
        Self {
            config,
            state: SpoolState::ShutDown,
            ramp: 0.0,
            state_time: 0.0,
            idle_timer: 0.0,
            spoolup_block: None,
        }
    }

    /// Set a spoolup block check function.
    /// Source: AP_MotorsMulticopter::get_spoolup_block()
    pub fn set_spoolup_block(&mut self, f: SpoolupBlockFn) {
        self.spoolup_block = Some(f);
    }

    /// Check if spoolup is blocked.
    pub fn get_spoolup_block(&self) -> bool {
        self.spoolup_block.map_or(false, |f| f())
    }

    /// Request arm (transition from ShutDown to GroundIdle).
    pub fn arm(&mut self) {
        if self.state == SpoolState::ShutDown {
            self.state = SpoolState::GroundIdle;
            self.state_time = 0.0;
            self.idle_timer = 0.0;
            self.ramp = 0.0;
        }
    }

    /// Request disarm (immediate transition to ShutDown).
    pub fn disarm(&mut self) {
        self.state = SpoolState::ShutDown;
        self.ramp = 0.0;
        self.state_time = 0.0;
        self.idle_timer = 0.0;
    }

    /// Request spool up (transition from GroundIdle to SpoolingUp).
    /// Respects safe_time, idle_time, and spoolup_block.
    pub fn request_spool_up(&mut self) {
        if self.state == SpoolState::GroundIdle {
            // Check safe_time gate
            if self.state_time < self.config.safe_time {
                return;
            }
            // Check idle_time gate
            if self.idle_timer < self.config.idle_time {
                return;
            }
            // Check spoolup block (ESC telemetry, RPM, CPU)
            if self.get_spoolup_block() {
                return;
            }
            self.state = SpoolState::SpoolingUp;
            self.state_time = 0.0;
        }
    }

    /// Request spool down (transition from ThrottleUnlimited to SpoolingDown).
    pub fn request_spool_down(&mut self) {
        if self.state == SpoolState::ThrottleUnlimited {
            self.state = SpoolState::SpoolingDown;
            self.state_time = 0.0;
        }
    }

    /// Update the state machine. Call at motor output rate (400Hz).
    pub fn update(&mut self, dt: f32) {
        self.state_time += dt;

        match self.state {
            SpoolState::ShutDown => {
                self.ramp = 0.0;
            }
            SpoolState::GroundIdle => {
                self.ramp = 0.0;
                // Track idle time for IDLE_TIME delay
                self.idle_timer += dt;
            }
            SpoolState::SpoolingUp => {
                let spool_time = self.config.effective_spool_up();
                self.ramp += dt / spool_time;
                if self.ramp >= 1.0 {
                    self.ramp = 1.0;
                    self.state = SpoolState::ThrottleUnlimited;
                    self.state_time = 0.0;
                }
            }
            SpoolState::ThrottleUnlimited => {
                self.ramp = 1.0;
            }
            SpoolState::SpoolingDown => {
                let spool_time = self.config.effective_spool_down();
                self.ramp -= dt / spool_time;
                if self.ramp <= 0.0 {
                    self.ramp = 0.0;
                    self.state = SpoolState::GroundIdle;
                    self.state_time = 0.0;
                    self.idle_timer = 0.0;
                }
            }
        }
    }

    /// Get the throttle ceiling for the current spool state.
    pub fn throttle_ceiling(&self) -> f32 {
        match self.state {
            SpoolState::ShutDown => 0.0,
            SpoolState::GroundIdle => self.config.spin_arm,
            SpoolState::SpoolingUp | SpoolState::SpoolingDown => {
                self.config.spin_arm + self.ramp * (1.0 - self.config.spin_arm)
            }
            SpoolState::ThrottleUnlimited => 1.0,
        }
    }

    /// Get the minimum motor output for the current state.
    pub fn motor_floor(&self) -> f32 {
        match self.state {
            SpoolState::ShutDown => 0.0,
            SpoolState::GroundIdle => self.config.spin_arm,
            SpoolState::SpoolingUp | SpoolState::SpoolingDown | SpoolState::ThrottleUnlimited => {
                self.config.spin_min
            }
        }
    }

    /// Should motors output any signal?
    pub fn motors_active(&self) -> bool {
        self.state != SpoolState::ShutDown
    }

    pub fn is_unlimited(&self) -> bool {
        self.state == SpoolState::ThrottleUnlimited
    }

    /// Current ramp value [0.0, 1.0].
    pub fn ramp(&self) -> f32 {
        self.ramp
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_spool_lifecycle() {
        let mut spool = SpoolManager::new(SpoolConfig::default());
        assert_eq!(spool.state, SpoolState::ShutDown);
        assert_eq!(spool.throttle_ceiling(), 0.0);

        // Arm
        spool.arm();
        assert_eq!(spool.state, SpoolState::GroundIdle);
        assert!((spool.throttle_ceiling() - 0.10).abs() < 0.01);

        // One tick to advance state_time past safe_time (0.0)
        spool.update(0.0025);

        // Spool up
        spool.request_spool_up();
        assert_eq!(spool.state, SpoolState::SpoolingUp);

        // Ramp over 0.5s at 400Hz (add extra ticks for float precision)
        for _ in 0..210 {
            spool.update(0.0025);
        }
        assert_eq!(spool.state, SpoolState::ThrottleUnlimited);
        assert!((spool.throttle_ceiling() - 1.0).abs() < 0.01);

        // Spool down (now uses effective_spool_down which defaults to spool_up_time)
        spool.request_spool_down();
        for _ in 0..210 {
            spool.update(0.0025);
        }
        assert_eq!(spool.state, SpoolState::GroundIdle);

        // Disarm
        spool.disarm();
        assert_eq!(spool.state, SpoolState::ShutDown);
        assert_eq!(spool.throttle_ceiling(), 0.0);
    }

    #[test]
    fn test_safe_time_delay() {
        let config = SpoolConfig { safe_time: 0.5, ..SpoolConfig::default() };
        let mut spool = SpoolManager::new(config);
        spool.arm();

        // Try to spool up immediately -- should stay in GroundIdle
        spool.request_spool_up();
        assert_eq!(spool.state, SpoolState::GroundIdle);

        // Wait 0.6s (past 0.5s safe_time)
        for _ in 0..240 {
            spool.update(0.0025);
        }
        spool.request_spool_up();
        assert_eq!(spool.state, SpoolState::SpoolingUp);
    }

    #[test]
    fn test_disarm_immediate() {
        let mut spool = SpoolManager::new(SpoolConfig::default());
        spool.arm();
        spool.update(0.0025);
        spool.request_spool_up();
        for _ in 0..100 { spool.update(0.0025); } // mid-spool
        assert_eq!(spool.state, SpoolState::SpoolingUp);

        spool.disarm(); // immediate cut
        assert_eq!(spool.state, SpoolState::ShutDown);
        assert_eq!(spool.throttle_ceiling(), 0.0);
    }

    #[test]
    fn test_minimum_spool_time() {
        // Setting spool_up_time to 0 should be clamped to MINIMUM_SPOOL_TIME
        let config = SpoolConfig { spool_up_time: 0.0, ..SpoolConfig::default() };
        assert!((config.effective_spool_up() - MINIMUM_SPOOL_TIME).abs() < 0.001);
    }

    #[test]
    fn test_spool_down_defaults_to_up() {
        // spool_down_time = 0 should fall back to spool_up_time
        let config = SpoolConfig {
            spool_up_time: 0.8,
            spool_down_time: 0.0,
            ..SpoolConfig::default()
        };
        assert!((config.effective_spool_down() - 0.8).abs() < 0.001);
    }

    #[test]
    fn test_idle_time_delay() {
        let config = SpoolConfig { idle_time: 1.0, ..SpoolConfig::default() };
        let mut spool = SpoolManager::new(config);
        spool.arm();
        spool.update(0.0025); // small tick

        // Try spool up -- should fail because idle_timer < idle_time
        spool.request_spool_up();
        assert_eq!(spool.state, SpoolState::GroundIdle);

        // Wait >1.0s
        for _ in 0..500 {
            spool.update(0.0025);
        }
        spool.request_spool_up();
        assert_eq!(spool.state, SpoolState::SpoolingUp);
    }

    #[test]
    fn test_spoolup_block() {
        static mut BLOCK: bool = true;

        fn block_fn() -> bool {
            unsafe { BLOCK }
        }

        let mut spool = SpoolManager::new(SpoolConfig::default());
        spool.set_spoolup_block(block_fn);
        spool.arm();
        spool.update(0.0025);

        // Should be blocked
        spool.request_spool_up();
        assert_eq!(spool.state, SpoolState::GroundIdle);

        // Unblock
        unsafe { BLOCK = false; }
        spool.request_spool_up();
        assert_eq!(spool.state, SpoolState::SpoolingUp);
    }
}

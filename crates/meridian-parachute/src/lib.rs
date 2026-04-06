#![no_std]

//! Parachute deploy subsystem.
//!
//! Source: ArduPilot `libraries/AP_Parachute/`
//!
//! Sequence:
//! 1. `release()` disarms motors immediately
//! 2. Configurable delay (default 500ms) to let props clear
//! 3. Servo/relay pulse (2000ms) to fire chute
//! 4. Servo returns to off position

/// Parachute deployment state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParachuteState {
    /// Not deployed, ready for release.
    Idle,
    /// Motors shutdown, waiting for delay before chute fires.
    MotorShutdown,
    /// Chute servo/relay is energized.
    Releasing,
    /// Release pulse complete — chute deployed.
    Deployed,
}

/// Parachute release output type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParachuteOutput {
    Servo,
    Relay,
}

/// Parachute subsystem.
pub struct Parachute {
    /// Current state.
    pub state: ParachuteState,
    /// Output type (servo or relay).
    pub output_type: ParachuteOutput,
    /// Servo channel index (0-based) when output is Servo.
    pub servo_channel: u8,
    /// Relay pin number when output is Relay.
    pub relay_pin: u16,
    /// PWM when chute servo is idle.
    pub servo_off_pwm: u16,
    /// PWM when chute servo is firing.
    pub servo_on_pwm: u16,
    /// Delay between motor shutdown and chute deploy (ms). Default 500.
    pub delay_ms: u32,
    /// Duration of the release pulse (ms). Default 2000.
    pub release_duration_ms: u32,
    /// Minimum altitude (meters) below which auto-trigger is inhibited.
    pub alt_min_m: f32,
    /// Sink rate threshold for auto-trigger (m/s, positive = sinking).
    pub sink_rate_trigger_ms: f32,
    /// Whether the parachute is enabled.
    pub enabled: bool,

    // Internal timing state
    state_start_ms: u32,
    /// Whether motors-off request has been issued to the caller.
    pub motors_off_requested: bool,
    /// Current servo PWM output (caller reads).
    pub current_servo_pwm: u16,
    /// Current relay state (caller reads).
    pub current_relay_active: bool,
}

impl Parachute {
    pub fn new() -> Self {
        Self {
            state: ParachuteState::Idle,
            output_type: ParachuteOutput::Servo,
            servo_channel: 0,
            relay_pin: 0,
            servo_off_pwm: 1100,
            servo_on_pwm: 1300,
            delay_ms: 500,
            release_duration_ms: 2000,
            alt_min_m: 10.0,
            sink_rate_trigger_ms: 0.0,
            enabled: true,
            state_start_ms: 0,
            motors_off_requested: false,
            current_servo_pwm: 1100,
            current_relay_active: false,
        }
    }

    /// Initiate parachute release sequence.
    /// Returns `true` if release was initiated, `false` if already deployed or disabled.
    pub fn release(&mut self, now_ms: u32) -> bool {
        if !self.enabled || self.state != ParachuteState::Idle {
            return false;
        }
        self.state = ParachuteState::MotorShutdown;
        self.state_start_ms = now_ms;
        self.motors_off_requested = true;
        true
    }

    /// Check whether auto-trigger conditions are met (sink rate + altitude).
    /// Caller should call `release()` if this returns `true`.
    pub fn should_auto_trigger(&self, altitude_m: f32, sink_rate_ms: f32) -> bool {
        if !self.enabled || self.state != ParachuteState::Idle {
            return false;
        }
        if self.sink_rate_trigger_ms <= 0.0 {
            return false;
        }
        // Altitude must be above minimum
        if altitude_m < self.alt_min_m {
            return false;
        }
        // Sink rate must exceed threshold (positive = sinking)
        sink_rate_ms >= self.sink_rate_trigger_ms
    }

    /// Main update loop. Call every control cycle.
    /// `now_ms`: current timestamp.
    pub fn update(&mut self, now_ms: u32) {
        match self.state {
            ParachuteState::Idle => {
                // Idle: servo off, relay off
                self.current_servo_pwm = self.servo_off_pwm;
                self.current_relay_active = false;
            }
            ParachuteState::MotorShutdown => {
                // Waiting for delay before deploying chute
                let elapsed = now_ms.wrapping_sub(self.state_start_ms);
                if elapsed >= self.delay_ms {
                    // Transition to releasing
                    self.state = ParachuteState::Releasing;
                    self.state_start_ms = now_ms;
                    // Energize servo/relay
                    self.current_servo_pwm = self.servo_on_pwm;
                    self.current_relay_active = true;
                }
            }
            ParachuteState::Releasing => {
                let elapsed = now_ms.wrapping_sub(self.state_start_ms);
                self.current_servo_pwm = self.servo_on_pwm;
                self.current_relay_active = true;
                if elapsed >= self.release_duration_ms {
                    // Release complete
                    self.state = ParachuteState::Deployed;
                    self.current_servo_pwm = self.servo_off_pwm;
                    self.current_relay_active = false;
                }
            }
            ParachuteState::Deployed => {
                // Terminal state
                self.current_servo_pwm = self.servo_off_pwm;
                self.current_relay_active = false;
            }
        }
    }

    /// Whether the parachute has been released (any non-idle state).
    pub fn is_released(&self) -> bool {
        self.state != ParachuteState::Idle
    }

    /// Whether the full deployment sequence is complete.
    pub fn is_deployed(&self) -> bool {
        self.state == ParachuteState::Deployed
    }

    /// Reset to idle (for testing or rearm after aborted deploy).
    pub fn reset(&mut self) {
        self.state = ParachuteState::Idle;
        self.state_start_ms = 0;
        self.motors_off_requested = false;
        self.current_servo_pwm = self.servo_off_pwm;
        self.current_relay_active = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_idle_state() {
        let p = Parachute::new();
        assert_eq!(p.state, ParachuteState::Idle);
        assert!(!p.is_released());
        assert!(!p.is_deployed());
    }

    #[test]
    fn test_release_sequence() {
        let mut p = Parachute::new();
        p.delay_ms = 500;
        p.release_duration_ms = 2000;

        assert!(p.release(1000));
        assert!(p.motors_off_requested);
        assert_eq!(p.state, ParachuteState::MotorShutdown);

        // 400ms: still in motor shutdown
        p.update(1400);
        assert_eq!(p.state, ParachuteState::MotorShutdown);
        assert_eq!(p.current_servo_pwm, p.servo_off_pwm);

        // 500ms: transition to releasing
        p.update(1500);
        assert_eq!(p.state, ParachuteState::Releasing);
        assert_eq!(p.current_servo_pwm, p.servo_on_pwm);
        assert!(p.current_relay_active);

        // 2000ms into releasing: still releasing
        p.update(3000);
        assert_eq!(p.state, ParachuteState::Releasing);

        // 2500ms total: deploy complete
        p.update(3500);
        assert_eq!(p.state, ParachuteState::Deployed);
        assert_eq!(p.current_servo_pwm, p.servo_off_pwm);
        assert!(!p.current_relay_active);
        assert!(p.is_deployed());
    }

    #[test]
    fn test_double_release_fails() {
        let mut p = Parachute::new();
        assert!(p.release(1000));
        assert!(!p.release(1100));
    }

    #[test]
    fn test_disabled_release_fails() {
        let mut p = Parachute::new();
        p.enabled = false;
        assert!(!p.release(1000));
    }

    #[test]
    fn test_auto_trigger_sink_rate() {
        let p = Parachute {
            sink_rate_trigger_ms: 5.0,
            alt_min_m: 10.0,
            enabled: true,
            ..Parachute::new()
        };

        // Above alt, sinking fast
        assert!(p.should_auto_trigger(50.0, 6.0));
        // Below alt minimum
        assert!(!p.should_auto_trigger(5.0, 6.0));
        // Not sinking fast enough
        assert!(!p.should_auto_trigger(50.0, 3.0));
    }

    #[test]
    fn test_auto_trigger_disabled_when_no_threshold() {
        let p = Parachute::new();
        assert!(!p.should_auto_trigger(100.0, 10.0));
    }

    #[test]
    fn test_reset() {
        let mut p = Parachute::new();
        p.release(1000);
        p.update(2000);
        p.reset();
        assert_eq!(p.state, ParachuteState::Idle);
        assert!(!p.motors_off_requested);
    }
}

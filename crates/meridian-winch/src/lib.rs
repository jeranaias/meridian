#![no_std]

//! Winch subsystem — position/rate/RC modes.
//!
//! Source: ArduPilot `libraries/AP_Winch/`

/// Winch operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WinchMode {
    /// Relaxed — no tension on the line.
    Relaxed,
    /// Move to an absolute line length position (meters).
    Position,
    /// Move at a given rate (m/s, positive = deploy).
    Rate,
    /// Rate controlled from RC input.
    RateFromRc,
}

/// Winch subsystem.
pub struct Winch {
    pub mode: WinchMode,
    /// Current line length (meters deployed).
    pub line_length_m: f32,
    /// Target line length for Position mode (meters).
    pub target_length_m: f32,
    /// Target deploy rate for Rate/RateFromRc modes (m/s).
    pub target_rate_ms: f32,
    /// Maximum deploy rate (m/s).
    pub rate_max_ms: f32,
    /// Servo channel for winch motor.
    pub servo_channel: u8,
    /// PWM output — caller reads this.
    pub output_pwm: u16,
    /// Healthy / connected flag.
    pub healthy: bool,
}

impl Winch {
    pub fn new() -> Self {
        Self {
            mode: WinchMode::Relaxed,
            line_length_m: 0.0,
            target_length_m: 0.0,
            target_rate_ms: 0.0,
            rate_max_ms: 2.0,
            servo_channel: 0,
            output_pwm: 1500,
            healthy: true,
        }
    }

    /// Set winch mode.
    pub fn set_mode(&mut self, mode: WinchMode) {
        self.mode = mode;
    }

    /// Set target position (meters).
    pub fn set_target_position(&mut self, length_m: f32) {
        self.mode = WinchMode::Position;
        self.target_length_m = length_m;
    }

    /// Set target rate (m/s, positive = deploy).
    pub fn set_target_rate(&mut self, rate_ms: f32) {
        self.target_rate_ms = rate_ms.clamp(-self.rate_max_ms, self.rate_max_ms);
    }

    /// Feed RC input for RateFromRc mode. `rc_norm` is -1.0 to +1.0.
    pub fn set_rc_input(&mut self, rc_norm: f32) {
        if self.mode == WinchMode::RateFromRc {
            self.target_rate_ms = rc_norm * self.rate_max_ms;
        }
    }

    /// Feed actual line length from sensor/telemetry (for position mode feedback).
    pub fn set_measured_length(&mut self, length_m: f32) {
        self.line_length_m = length_m;
    }

    /// Update. Call each control loop. `dt` in seconds.
    pub fn update(&mut self, dt: f32) {
        if dt <= 0.0 {
            return;
        }

        match self.mode {
            WinchMode::Relaxed => {
                self.output_pwm = 1500;
            }
            WinchMode::Position => {
                let error = self.target_length_m - self.line_length_m;
                let rate = error.clamp(-self.rate_max_ms, self.rate_max_ms);
                self.output_pwm = rate_to_pwm(rate, self.rate_max_ms);
                self.line_length_m += rate * dt;
            }
            WinchMode::Rate | WinchMode::RateFromRc => {
                let rate = self.target_rate_ms.clamp(-self.rate_max_ms, self.rate_max_ms);
                self.output_pwm = rate_to_pwm(rate, self.rate_max_ms);
                self.line_length_m += rate * dt;
            }
        }
    }

    /// Get WINCH_STATUS fields.
    pub fn status(&self) -> WinchStatus {
        WinchStatus {
            line_length_m: self.line_length_m,
            rate_ms: self.target_rate_ms,
            healthy: self.healthy,
        }
    }
}

/// Winch status report (maps to MAVLink WINCH_STATUS).
pub struct WinchStatus {
    pub line_length_m: f32,
    pub rate_ms: f32,
    pub healthy: bool,
}

fn rate_to_pwm(rate: f32, max_rate: f32) -> u16 {
    if max_rate <= 0.0 {
        return 1500;
    }
    let frac = rate / max_rate;
    (1500.0 + frac * 500.0) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_relaxed_neutral() {
        let mut w = Winch::new();
        w.set_mode(WinchMode::Relaxed);
        w.update(0.01);
        assert_eq!(w.output_pwm, 1500);
    }

    #[test]
    fn test_rate_mode() {
        let mut w = Winch::new();
        w.mode = WinchMode::Rate;
        w.set_target_rate(1.0);
        w.update(1.0);
        assert!(w.line_length_m > 0.9);
        assert!(w.output_pwm > 1500);
    }

    #[test]
    fn test_position_mode() {
        let mut w = Winch::new();
        w.set_target_position(5.0);
        for _ in 0..500 {
            w.update(0.01);
        }
        assert!((w.line_length_m - 5.0).abs() < 1.0);
    }

    #[test]
    fn test_rc_mode() {
        let mut w = Winch::new();
        w.set_mode(WinchMode::RateFromRc);
        w.set_rc_input(0.5);
        assert!((w.target_rate_ms - 1.0).abs() < 0.01);
    }
}

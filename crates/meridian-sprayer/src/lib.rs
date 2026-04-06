#![no_std]

//! Sprayer subsystem — pump + spinner PWM, ground-speed-proportional flow.
//!
//! Source: ArduPilot `libraries/AC_Sprayer/`

/// Sprayer subsystem.
pub struct Sprayer {
    /// Whether the sprayer is enabled.
    pub enabled: bool,
    /// Whether the sprayer is actively spraying.
    pub running: bool,
    /// Pump servo channel.
    pub pump_channel: u8,
    /// Spinner servo channel.
    pub spinner_channel: u8,
    /// Pump PWM output (caller reads).
    pub pump_pwm: u16,
    /// Spinner PWM output (caller reads).
    pub spinner_pwm: u16,
    /// Pump speed at full rate (PWM).
    pub pump_max_pwm: u16,
    /// Spinner speed when active (PWM).
    pub spinner_active_pwm: u16,
    /// Ground speed for full pump output (m/s).
    pub speed_max_ms: f32,
    /// Minimum ground speed to enable spraying (m/s).
    pub speed_min_ms: f32,
}

impl Sprayer {
    pub fn new() -> Self {
        Self {
            enabled: false,
            running: false,
            pump_channel: 0,
            spinner_channel: 1,
            pump_pwm: 1000,
            spinner_pwm: 1000,
            pump_max_pwm: 1900,
            spinner_active_pwm: 1700,
            speed_max_ms: 10.0,
            speed_min_ms: 1.0,
        }
    }

    /// Enable/disable sprayer.
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
        if !enabled {
            self.running = false;
            self.pump_pwm = 1000;
            self.spinner_pwm = 1000;
        }
    }

    /// Activate spraying (start pump and spinner).
    pub fn start(&mut self) {
        if self.enabled {
            self.running = true;
        }
    }

    /// Deactivate spraying.
    pub fn stop(&mut self) {
        self.running = false;
        self.pump_pwm = 1000;
        self.spinner_pwm = 1000;
    }

    /// Update pump/spinner outputs based on ground speed.
    /// `ground_speed_ms`: current vehicle ground speed in m/s.
    pub fn update(&mut self, ground_speed_ms: f32) {
        if !self.enabled || !self.running {
            self.pump_pwm = 1000;
            self.spinner_pwm = 1000;
            return;
        }

        if ground_speed_ms < self.speed_min_ms {
            // Too slow — stop pump but keep spinner
            self.pump_pwm = 1000;
            self.spinner_pwm = self.spinner_active_pwm;
            return;
        }

        // Proportional pump speed
        let frac = if self.speed_max_ms > 0.0 {
            (ground_speed_ms / self.speed_max_ms).min(1.0)
        } else {
            1.0
        };

        self.pump_pwm = 1000 + ((self.pump_max_pwm - 1000) as f32 * frac) as u16;
        self.spinner_pwm = self.spinner_active_pwm;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_initial_off() {
        let s = Sprayer::new();
        assert!(!s.enabled);
        assert!(!s.running);
        assert_eq!(s.pump_pwm, 1000);
    }

    #[test]
    fn test_proportional_pump() {
        let mut s = Sprayer::new();
        s.set_enabled(true);
        s.start();
        s.speed_max_ms = 10.0;
        s.pump_max_pwm = 1900;

        // Half speed
        s.update(5.0);
        assert!(s.pump_pwm > 1400 && s.pump_pwm < 1500);
        assert_eq!(s.spinner_pwm, s.spinner_active_pwm);

        // Full speed
        s.update(10.0);
        assert_eq!(s.pump_pwm, s.pump_max_pwm);
    }

    #[test]
    fn test_below_min_speed() {
        let mut s = Sprayer::new();
        s.set_enabled(true);
        s.start();
        s.speed_min_ms = 1.0;

        s.update(0.5);
        assert_eq!(s.pump_pwm, 1000);
        assert_eq!(s.spinner_pwm, s.spinner_active_pwm);
    }

    #[test]
    fn test_disabled_no_output() {
        let mut s = Sprayer::new();
        s.start();
        s.update(5.0);
        assert_eq!(s.pump_pwm, 1000);
    }
}

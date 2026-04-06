#![no_std]

//! Landing gear subsystem — deploy/retract with altitude triggers.
//!
//! Source: ArduPilot `libraries/AP_LandingGear/`

/// Landing gear state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GearState {
    Deployed,
    Deploying,
    Retracted,
    Retracting,
}

/// Landing gear subsystem.
pub struct LandingGear {
    pub state: GearState,
    /// Servo channel.
    pub servo_channel: u8,
    /// PWM for deployed position.
    pub deploy_pwm: u16,
    /// PWM for retracted position.
    pub retract_pwm: u16,
    /// Current servo PWM output.
    pub current_pwm: u16,
    /// Transition time (ms).
    pub transition_ms: u32,
    /// Altitude below which gear auto-deploys (meters, 0 = disabled).
    pub auto_deploy_alt_m: f32,
    /// Altitude above which gear auto-retracts (meters, 0 = disabled).
    pub auto_retract_alt_m: f32,
    /// Weight-on-wheels sensor state (true = on ground).
    pub wow_detected: bool,

    transition_start_ms: u32,
}

impl LandingGear {
    pub fn new() -> Self {
        Self {
            state: GearState::Deployed,
            servo_channel: 0,
            deploy_pwm: 1100,
            retract_pwm: 1900,
            current_pwm: 1100,
            transition_ms: 1000,
            auto_deploy_alt_m: 0.0,
            auto_retract_alt_m: 0.0,
            wow_detected: false,
            transition_start_ms: 0,
        }
    }

    /// Command deploy.
    pub fn deploy(&mut self, now_ms: u32) {
        if self.state == GearState::Deployed {
            return;
        }
        self.state = GearState::Deploying;
        self.transition_start_ms = now_ms;
        self.current_pwm = self.deploy_pwm;
    }

    /// Command retract.
    pub fn retract(&mut self, now_ms: u32) {
        if self.state == GearState::Retracted {
            return;
        }
        self.state = GearState::Retracting;
        self.transition_start_ms = now_ms;
        self.current_pwm = self.retract_pwm;
    }

    /// Update with current altitude. Handles auto-deploy/retract and transition timing.
    pub fn update(&mut self, altitude_m: f32, now_ms: u32) {
        // Handle transition completion
        match self.state {
            GearState::Deploying => {
                if now_ms.wrapping_sub(self.transition_start_ms) >= self.transition_ms {
                    self.state = GearState::Deployed;
                }
            }
            GearState::Retracting => {
                if now_ms.wrapping_sub(self.transition_start_ms) >= self.transition_ms {
                    self.state = GearState::Retracted;
                }
            }
            _ => {}
        }

        // Auto-deploy: if below threshold and retracted
        if self.auto_deploy_alt_m > 0.0
            && altitude_m < self.auto_deploy_alt_m
            && (self.state == GearState::Retracted || self.state == GearState::Retracting)
        {
            self.deploy(now_ms);
        }

        // Auto-retract: if above threshold and deployed
        if self.auto_retract_alt_m > 0.0
            && altitude_m > self.auto_retract_alt_m
            && (self.state == GearState::Deployed || self.state == GearState::Deploying)
            && !self.wow_detected
        {
            self.retract(now_ms);
        }
    }

    pub fn is_deployed(&self) -> bool {
        self.state == GearState::Deployed
    }

    pub fn is_retracted(&self) -> bool {
        self.state == GearState::Retracted
    }

    /// Set weight-on-wheels sensor state.
    pub fn set_wow(&mut self, on_ground: bool) {
        self.wow_detected = on_ground;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_initial_deployed() {
        let g = LandingGear::new();
        assert!(g.is_deployed());
        assert_eq!(g.current_pwm, g.deploy_pwm);
    }

    #[test]
    fn test_retract_deploy_cycle() {
        let mut g = LandingGear::new();
        g.transition_ms = 1000;

        g.retract(1000);
        assert_eq!(g.state, GearState::Retracting);
        assert_eq!(g.current_pwm, g.retract_pwm);

        g.update(50.0, 2000);
        assert!(g.is_retracted());

        g.deploy(3000);
        assert_eq!(g.state, GearState::Deploying);
        g.update(10.0, 4000);
        assert!(g.is_deployed());
    }

    #[test]
    fn test_auto_deploy() {
        let mut g = LandingGear::new();
        g.auto_deploy_alt_m = 5.0;
        g.transition_ms = 100;

        // Start retracted
        g.retract(0);
        g.update(50.0, 200);
        assert!(g.is_retracted());

        // Drop below auto-deploy altitude
        g.update(3.0, 300);
        assert_eq!(g.state, GearState::Deploying);
        g.update(3.0, 500);
        assert!(g.is_deployed());
    }

    #[test]
    fn test_auto_retract() {
        let mut g = LandingGear::new();
        g.auto_retract_alt_m = 10.0;
        g.transition_ms = 100;

        g.update(20.0, 100);
        assert_eq!(g.state, GearState::Retracting);
        g.update(20.0, 300);
        assert!(g.is_retracted());
    }

    #[test]
    fn test_wow_prevents_retract() {
        let mut g = LandingGear::new();
        g.auto_retract_alt_m = 10.0;
        g.set_wow(true);

        g.update(20.0, 100);
        assert!(g.is_deployed(), "WoW should prevent retract");
    }
}

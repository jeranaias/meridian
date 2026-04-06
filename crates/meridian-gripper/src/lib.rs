#![no_std]

//! Gripper subsystem — servo and EPM (electropermanent magnet) backends.
//!
//! Source: ArduPilot `libraries/AP_Gripper/`

/// Gripper state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GripperState {
    Released,
    Grabbing,
    Grabbed,
    Releasing,
}

/// Gripper backend type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GripperType {
    Servo,
    Epm,
}

/// Gripper subsystem with grab/release state machine.
pub struct Gripper {
    pub state: GripperState,
    pub gripper_type: GripperType,
    /// Servo channel (0-based).
    pub servo_channel: u8,
    /// PWM for grabbed position.
    pub grab_pwm: u16,
    /// PWM for released position.
    pub release_pwm: u16,
    /// Time to complete grab/release action (ms).
    pub action_time_ms: u32,
    /// EPM pulse duration for grab (ms).
    pub epm_grab_ms: u32,
    /// EPM pulse duration for release (ms).
    pub epm_release_ms: u32,
    /// Whether grip has been confirmed (from sensor or timer).
    pub grip_confirmed: bool,
    /// Current servo PWM output.
    pub current_pwm: u16,
    /// Current EPM state (true = energized).
    pub epm_active: bool,

    action_start_ms: u32,
}

impl Gripper {
    pub fn new(gripper_type: GripperType) -> Self {
        Self {
            state: GripperState::Released,
            gripper_type,
            servo_channel: 0,
            grab_pwm: 1900,
            release_pwm: 1100,
            action_time_ms: 1000,
            epm_grab_ms: 500,
            epm_release_ms: 500,
            grip_confirmed: false,
            current_pwm: 1100,
            epm_active: false,
            action_start_ms: 0,
        }
    }

    /// Command grab.
    pub fn grab(&mut self, now_ms: u32) {
        if self.state == GripperState::Grabbed || self.state == GripperState::Grabbing {
            return;
        }
        self.state = GripperState::Grabbing;
        self.action_start_ms = now_ms;
        self.grip_confirmed = false;
        match self.gripper_type {
            GripperType::Servo => self.current_pwm = self.grab_pwm,
            GripperType::Epm => self.epm_active = true,
        }
    }

    /// Command release.
    pub fn release(&mut self, now_ms: u32) {
        if self.state == GripperState::Released || self.state == GripperState::Releasing {
            return;
        }
        self.state = GripperState::Releasing;
        self.action_start_ms = now_ms;
        match self.gripper_type {
            GripperType::Servo => self.current_pwm = self.release_pwm,
            GripperType::Epm => self.epm_active = true,
        }
    }

    /// Update state machine.
    pub fn update(&mut self, now_ms: u32) {
        let elapsed = now_ms.wrapping_sub(self.action_start_ms);
        match self.state {
            GripperState::Grabbing => {
                let timeout = match self.gripper_type {
                    GripperType::Servo => self.action_time_ms,
                    GripperType::Epm => self.epm_grab_ms,
                };
                if elapsed >= timeout {
                    self.state = GripperState::Grabbed;
                    self.grip_confirmed = true;
                    self.epm_active = false;
                }
            }
            GripperState::Releasing => {
                let timeout = match self.gripper_type {
                    GripperType::Servo => self.action_time_ms,
                    GripperType::Epm => self.epm_release_ms,
                };
                if elapsed >= timeout {
                    self.state = GripperState::Released;
                    self.grip_confirmed = false;
                    self.epm_active = false;
                }
            }
            _ => {}
        }
    }

    /// Whether the gripper has confirmed grip.
    pub fn is_grabbed(&self) -> bool {
        self.state == GripperState::Grabbed && self.grip_confirmed
    }

    /// Whether the gripper is released.
    pub fn is_released(&self) -> bool {
        self.state == GripperState::Released
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_initial_released() {
        let g = Gripper::new(GripperType::Servo);
        assert!(g.is_released());
        assert!(!g.is_grabbed());
    }

    #[test]
    fn test_servo_grab_release() {
        let mut g = Gripper::new(GripperType::Servo);
        g.action_time_ms = 1000;

        g.grab(1000);
        assert_eq!(g.state, GripperState::Grabbing);
        assert_eq!(g.current_pwm, g.grab_pwm);

        g.update(1500);
        assert_eq!(g.state, GripperState::Grabbing);

        g.update(2000);
        assert!(g.is_grabbed());

        g.release(3000);
        assert_eq!(g.state, GripperState::Releasing);
        assert_eq!(g.current_pwm, g.release_pwm);

        g.update(4000);
        assert!(g.is_released());
    }

    #[test]
    fn test_epm_grab_release() {
        let mut g = Gripper::new(GripperType::Epm);
        g.epm_grab_ms = 500;
        g.epm_release_ms = 500;

        g.grab(1000);
        assert!(g.epm_active);

        g.update(1500);
        assert!(g.is_grabbed());
        assert!(!g.epm_active);

        g.release(2000);
        assert!(g.epm_active);

        g.update(2500);
        assert!(g.is_released());
        assert!(!g.epm_active);
    }
}

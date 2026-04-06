//! Motor test via MAVLink (MAV_CMD_DO_MOTOR_TEST, command 209).
//!
//! Source: ArduCopter/motor_test.cpp
//!
//! Allows individual motor testing from GCS. The motor test overrides
//! normal mixer output when active.

use crate::MAX_MOTORS;

/// Motor test command ordering.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotorTestOrder {
    /// Test motors by their output index (0-based).
    ByIndex,
    /// Test motors in sequence (front-right, back-left, ...).
    BySequence,
}

/// A motor test command.
#[derive(Debug, Clone, Copy)]
pub struct MotorTestCmd {
    pub motor_index: u8,
    pub throttle_pct: f32,
    pub duration_s: f32,
}

/// Throttle type for motor test.
/// Source: ArduCopter/motor_test.cpp
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThrottleType {
    /// Throttle as percentage (0.0 to 1.0).
    Percent,
    /// Raw PWM value (1000-2000).
    Pwm,
    /// Pass through from pilot throttle stick.
    PilotPassthrough,
}

/// Motor test state machine.
///
/// Usage: when MAV_CMD_DO_MOTOR_TEST is received, call `start()`.
/// Each loop iteration, call `get_output()` to get motor outputs if active.
/// Call `update()` with elapsed time to handle timeout.
pub struct MotorTest {
    /// Which motor is being tested (0-indexed).
    motor_index: u8,
    /// Throttle type.
    throttle_type: ThrottleType,
    /// Throttle value (0.0-1.0 for Percent, raw for PWM).
    throttle_value: f32,
    /// Timeout in seconds.
    timeout_s: f32,
    /// Elapsed time since test started.
    elapsed_s: f32,
    /// Whether a test is currently active.
    active: bool,
}

impl MotorTest {
    pub fn new() -> Self {
        Self {
            motor_index: 0,
            throttle_type: ThrottleType::Percent,
            throttle_value: 0.0,
            timeout_s: 0.0,
            elapsed_s: 0.0,
            active: false,
        }
    }

    /// Start a motor test.
    ///
    /// `motor_index`: 0-based motor number.
    /// `throttle_type`: how to interpret the value.
    /// `throttle_value`: throttle command (meaning depends on type).
    /// `timeout_s`: how long to run the test (seconds). 0 = no timeout.
    pub fn start(
        &mut self,
        motor_index: u8,
        throttle_type: ThrottleType,
        throttle_value: f32,
        timeout_s: f32,
    ) {
        self.motor_index = motor_index;
        self.throttle_type = throttle_type;
        self.throttle_value = throttle_value;
        self.timeout_s = timeout_s;
        self.elapsed_s = 0.0;
        self.active = true;
    }

    /// Stop the current test.
    pub fn stop(&mut self) {
        self.active = false;
    }

    /// Whether a test is active.
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Update elapsed time. Call every loop iteration with dt.
    /// Automatically stops the test when timeout expires.
    pub fn update(&mut self, dt: f32) {
        if !self.active { return; }
        self.elapsed_s += dt;
        if self.timeout_s > 0.0 && self.elapsed_s >= self.timeout_s {
            self.active = false;
        }
    }

    /// Get motor outputs for the test. Returns array of motor outputs [0.0, 1.0].
    /// Only the tested motor gets a non-zero output.
    ///
    /// `motor_count`: total number of motors on the vehicle.
    pub fn get_output(&self, motor_count: usize) -> [f32; MAX_MOTORS] {
        let mut output = [0.0f32; MAX_MOTORS];
        if !self.active { return output; }

        let idx = self.motor_index as usize;
        if idx >= motor_count || idx >= MAX_MOTORS { return output; }

        let value = match self.throttle_type {
            ThrottleType::Percent => self.throttle_value.clamp(0.0, 1.0),
            ThrottleType::Pwm => {
                // Convert PWM 1000-2000 to 0.0-1.0
                ((self.throttle_value - 1000.0) / 1000.0).clamp(0.0, 1.0)
            }
            ThrottleType::PilotPassthrough => {
                self.throttle_value.clamp(0.0, 1.0)
            }
        };

        output[idx] = value;
        output
    }

    /// Current motor index being tested.
    pub fn motor_index(&self) -> u8 {
        self.motor_index
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_motor_test_lifecycle() {
        let mut mt = MotorTest::new();
        assert!(!mt.is_active());

        mt.start(2, ThrottleType::Percent, 0.75, 3.0);
        assert!(mt.is_active());
        assert_eq!(mt.motor_index(), 2);

        let out = mt.get_output(4);
        assert!((out[2] - 0.75).abs() < 0.01);
        assert!(out[0] < 0.01);
        assert!(out[1] < 0.01);
        assert!(out[3] < 0.01);

        // Advance time but not past timeout
        mt.update(1.0);
        assert!(mt.is_active());

        // Past timeout
        mt.update(2.5);
        assert!(!mt.is_active());

        // Output should be zero when inactive
        let out2 = mt.get_output(4);
        for i in 0..4 {
            assert!(out2[i] < 0.01);
        }
    }

    #[test]
    fn test_motor_test_pwm_conversion() {
        let mut mt = MotorTest::new();
        mt.start(0, ThrottleType::Pwm, 1500.0, 1.0);
        let out = mt.get_output(4);
        assert!((out[0] - 0.5).abs() < 0.01);
    }

    #[test]
    fn test_motor_test_manual_stop() {
        let mut mt = MotorTest::new();
        mt.start(1, ThrottleType::Percent, 1.0, 0.0); // no timeout
        assert!(mt.is_active());
        mt.update(100.0);
        assert!(mt.is_active()); // no timeout = stays active
        mt.stop();
        assert!(!mt.is_active());
    }
}

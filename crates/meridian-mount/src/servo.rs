//! Servo-based mount backend — direct PWM output on 2-3 channels.
//!
//! Source: ArduPilot `AP_Mount_Servo.cpp`
//! Maps desired angles to PWM values (1000-2000us range).

use crate::backend::{MountBackend, MountTarget};

/// Servo channel assignment.
#[derive(Debug, Clone, Copy)]
pub struct ServoChannel {
    /// Output channel index (0-based).
    pub channel: u8,
    /// PWM value for minimum angle.
    pub pwm_min: u16,
    /// PWM value for maximum angle.
    pub pwm_max: u16,
    /// Angle at minimum PWM (radians).
    pub angle_min: f32,
    /// Angle at maximum PWM (radians).
    pub angle_max: f32,
    /// Whether this axis is reversed.
    pub reversed: bool,
}

impl ServoChannel {
    /// Map an angle (radians) to a PWM value, clamped to [pwm_min, pwm_max].
    pub fn angle_to_pwm(&self, angle_rad: f32) -> u16 {
        let clamped = clamp(angle_rad, self.angle_min, self.angle_max);
        let range = self.angle_max - self.angle_min;
        if range.abs() < 1e-6 {
            return (self.pwm_min + self.pwm_max) / 2;
        }
        let frac = (clamped - self.angle_min) / range;
        let frac = if self.reversed { 1.0 - frac } else { frac };
        let pwm = self.pwm_min as f32 + frac * (self.pwm_max - self.pwm_min) as f32;
        clamp(pwm, self.pwm_min as f32, self.pwm_max as f32) as u16
    }
}

/// Servo-based mount with up to 3 axes.
pub struct ServoMount {
    /// Roll servo channel (optional — some gimbals are 2-axis).
    pub roll: Option<ServoChannel>,
    /// Pitch servo channel.
    pub pitch: ServoChannel,
    /// Yaw servo channel (optional).
    pub yaw: Option<ServoChannel>,
    /// Current angles (radians).
    current_roll: f32,
    current_pitch: f32,
    current_yaw: f32,
    /// Output PWM buffer: [roll, pitch, yaw]. Read by the caller to apply via RcOutput.
    pub output_pwm: [u16; 3],
    initialized: bool,
}

impl ServoMount {
    pub fn new(pitch: ServoChannel, roll: Option<ServoChannel>, yaw: Option<ServoChannel>) -> Self {
        Self {
            roll,
            pitch,
            yaw,
            current_roll: 0.0,
            current_pitch: 0.0,
            current_yaw: 0.0,
            output_pwm: [1500; 3],
            initialized: false,
        }
    }
}

impl MountBackend for ServoMount {
    fn init(&mut self) -> bool {
        self.current_roll = 0.0;
        self.current_pitch = 0.0;
        self.current_yaw = 0.0;
        self.output_pwm = [1500; 3];
        self.initialized = true;
        true
    }

    fn update(&mut self, target: &MountTarget, dt: f32) {
        if !self.initialized || dt <= 0.0 {
            return;
        }

        match target {
            MountTarget::Angle { roll, pitch, yaw } => {
                self.current_roll = *roll;
                self.current_pitch = *pitch;
                self.current_yaw = *yaw;
            }
            MountTarget::Rate { roll_rate, pitch_rate, yaw_rate } => {
                self.current_roll += roll_rate * dt;
                self.current_pitch += pitch_rate * dt;
                self.current_yaw += yaw_rate * dt;
            }
            MountTarget::Retract => {
                self.current_roll = 0.0;
                self.current_pitch = 0.0;
                self.current_yaw = 0.0;
            }
            MountTarget::Neutral => {
                self.current_roll = 0.0;
                self.current_pitch = 0.0;
                self.current_yaw = 0.0;
            }
            MountTarget::RoiLocation { .. } => {
                // ROI requires vehicle position — handled by MountManager.
                // ServoMount just applies the resulting angle target.
            }
        }

        // Convert angles to PWM
        if let Some(ref ch) = self.roll {
            self.output_pwm[0] = ch.angle_to_pwm(self.current_roll);
        }
        self.output_pwm[1] = self.pitch.angle_to_pwm(self.current_pitch);
        if let Some(ref ch) = self.yaw {
            self.output_pwm[2] = ch.angle_to_pwm(self.current_yaw);
        }
    }

    fn get_attitude(&self) -> (f32, f32, f32) {
        (self.current_roll, self.current_pitch, self.current_yaw)
    }

    fn has_pan_control(&self) -> bool {
        self.yaw.is_some()
    }
}

#[inline]
fn clamp(val: f32, min: f32, max: f32) -> f32 {
    if val < min { min } else if val > max { max } else { val }
}

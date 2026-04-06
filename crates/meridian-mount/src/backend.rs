//! Mount backend trait and target types.
//!
//! Every gimbal implementation (servo, MAVLink, Siyi) implements this trait.

/// Target command for the mount.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MountTarget {
    /// Direct angle command (radians).
    Angle { roll: f32, pitch: f32, yaw: f32 },
    /// Rate command (radians/s).
    Rate { roll_rate: f32, pitch_rate: f32, yaw_rate: f32 },
    /// Point at GPS location (Region-Of-Interest).
    RoiLocation { lat: f64, lon: f64, alt: f32 },
    /// Retract mount to stowed position.
    Retract,
    /// Move mount to neutral (forward-facing) position.
    Neutral,
}

impl Default for MountTarget {
    fn default() -> Self {
        MountTarget::Neutral
    }
}

/// Backend trait for gimbal/mount hardware.
pub trait MountBackend {
    /// Initialize the backend. Returns true on success.
    fn init(&mut self) -> bool;

    /// Update mount towards the target. Called each control loop iteration.
    /// `dt` is seconds since last call.
    fn update(&mut self, target: &MountTarget, dt: f32);

    /// Get current attitude in radians: (roll, pitch, yaw).
    fn get_attitude(&self) -> (f32, f32, f32);

    /// Whether this backend supports yaw/pan control.
    fn has_pan_control(&self) -> bool;
}

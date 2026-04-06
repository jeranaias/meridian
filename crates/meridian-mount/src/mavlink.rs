//! MAVLink mount backend — control via DO_MOUNT_CONTROL messages to external gimbal.
//!
//! Source: ArduPilot `AP_Mount_MAVLink.cpp`
//! Sends MAV_CMD_DO_MOUNT_CONTROL commands to an external gimbal controller.

use crate::backend::{MountBackend, MountTarget};

/// MAVLink DO_MOUNT_CONTROL message fields (outbound).
/// The caller reads this and serializes it into a MAVLink packet.
#[derive(Debug, Clone, Copy, Default)]
pub struct MountControlMsg {
    /// Target system ID of the gimbal.
    pub target_sysid: u8,
    /// Target component ID of the gimbal.
    pub target_compid: u8,
    /// Roll angle in centidegrees.
    pub roll_cd: i32,
    /// Pitch angle in centidegrees.
    pub pitch_cd: i32,
    /// Yaw angle in centidegrees.
    pub yaw_cd: i32,
    /// True if a new message is pending transmission.
    pub pending: bool,
}

/// MAVLink-based mount backend.
pub struct MavlinkMount {
    /// Target gimbal system/component IDs.
    target_sysid: u8,
    target_compid: u8,
    /// Current known attitude from gimbal feedback (radians).
    current_roll: f32,
    current_pitch: f32,
    current_yaw: f32,
    /// Outbound message buffer.
    pub out_msg: MountControlMsg,
    initialized: bool,
}

impl MavlinkMount {
    pub fn new(target_sysid: u8, target_compid: u8) -> Self {
        Self {
            target_sysid,
            target_compid,
            current_roll: 0.0,
            current_pitch: 0.0,
            current_yaw: 0.0,
            out_msg: MountControlMsg::default(),
            initialized: false,
        }
    }

    /// Feed attitude feedback from the gimbal (e.g., MOUNT_ORIENTATION message).
    pub fn set_attitude_feedback(&mut self, roll: f32, pitch: f32, yaw: f32) {
        self.current_roll = roll;
        self.current_pitch = pitch;
        self.current_yaw = yaw;
    }
}

impl MountBackend for MavlinkMount {
    fn init(&mut self) -> bool {
        self.out_msg = MountControlMsg {
            target_sysid: self.target_sysid,
            target_compid: self.target_compid,
            ..MountControlMsg::default()
        };
        self.initialized = true;
        true
    }

    fn update(&mut self, target: &MountTarget, _dt: f32) {
        if !self.initialized {
            return;
        }

        match target {
            MountTarget::Angle { roll, pitch, yaw } => {
                self.out_msg.roll_cd = rad_to_centideg(*roll);
                self.out_msg.pitch_cd = rad_to_centideg(*pitch);
                self.out_msg.yaw_cd = rad_to_centideg(*yaw);
                self.out_msg.pending = true;
            }
            MountTarget::Rate { .. } => {
                // MAVLink DO_MOUNT_CONTROL doesn't natively support rates.
                // A higher-level integrator converts rates to angles before here.
            }
            MountTarget::Retract | MountTarget::Neutral => {
                self.out_msg.roll_cd = 0;
                self.out_msg.pitch_cd = 0;
                self.out_msg.yaw_cd = 0;
                self.out_msg.pending = true;
            }
            MountTarget::RoiLocation { .. } => {
                // ROI resolved to angle by MountManager before reaching backend.
            }
        }
    }

    fn get_attitude(&self) -> (f32, f32, f32) {
        (self.current_roll, self.current_pitch, self.current_yaw)
    }

    fn has_pan_control(&self) -> bool {
        true
    }
}

/// Convert radians to centidegrees.
#[inline]
fn rad_to_centideg(rad: f32) -> i32 {
    (rad * 18000.0 / core::f32::consts::PI) as i32
}

/// Convert centidegrees to radians.
#[inline]
#[allow(dead_code)]
fn centideg_to_rad(cd: i32) -> f32 {
    cd as f32 * core::f32::consts::PI / 18000.0
}

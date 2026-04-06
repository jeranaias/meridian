//! Siyi gimbal binary protocol backend.
//!
//! Source: ArduPilot `AP_Mount_Siyi.cpp`
//!
//! CRITICAL: Siyi gimbals do NOT accept direct angle commands. The flight
//! controller must run a P-controller (P=1.5, max 90 deg/s) to convert desired
//! angles to -100..+100 rate scalars, which is what the gimbal protocol accepts.
//!
//! Protocol: 0x5566 magic, little-endian length/sequence, CRC16-CCITT checksum.

use crate::backend::{MountBackend, MountTarget};

/// Siyi protocol constants.
const SIYI_MAGIC: u16 = 0x5566;
const SIYI_CMD_GIMBAL_ROTATION: u8 = 0x07;
const SIYI_CMD_GET_ATTITUDE: u8 = 0x0D;
const SIYI_CMD_EXTERNAL_ATTITUDE: u8 = 0x22;
#[allow(dead_code)]
const SIYI_CMD_POSITION_DATA: u8 = 0x3E;
#[allow(dead_code)]
const SIYI_HEADER_LEN: usize = 10;

/// Maximum output rate sent to gimbal (degrees/s).
const MAX_RATE_DEG_S: f32 = 90.0;

/// P-controller gain for angle-to-rate conversion.
const ANGLE_P_GAIN: f32 = 1.5;

/// Attitude poll interval (ms) — 20Hz = 50ms.
const ATTITUDE_POLL_INTERVAL_MS: u32 = 50;

/// Outbound serial packet buffer. Caller reads and transmits via UART.
pub struct SiyiPacket {
    pub data: [u8; 32],
    pub len: usize,
    pub pending: bool,
}

impl Default for SiyiPacket {
    fn default() -> Self {
        Self { data: [0u8; 32], len: 0, pending: false }
    }
}

/// Siyi gimbal backend.
pub struct SiyiMount {
    /// Sequence counter for the binary protocol.
    seq: u16,
    /// Current gimbal attitude from feedback (radians).
    current_roll: f32,
    current_pitch: f32,
    current_yaw: f32,
    /// Desired angles for P-controller tracking (radians).
    desired_pitch: f32,
    desired_yaw: f32,
    /// Outbound packet buffer.
    pub out_packet: SiyiPacket,
    /// Attitude request packet (separate for 20Hz polling).
    pub attitude_request_packet: SiyiPacket,
    /// EXTERNAL_ATTITUDE command packet.
    pub external_attitude_packet: SiyiPacket,
    /// Whether mounting is upside-down (inverts pitch/yaw transforms).
    pub upside_down: bool,
    /// Timestamp of last attitude poll request.
    last_attitude_poll_ms: u32,
    initialized: bool,
}

impl SiyiMount {
    pub fn new() -> Self {
        Self {
            seq: 0,
            current_roll: 0.0,
            current_pitch: 0.0,
            current_yaw: 0.0,
            desired_pitch: 0.0,
            desired_yaw: 0.0,
            out_packet: SiyiPacket::default(),
            attitude_request_packet: SiyiPacket::default(),
            external_attitude_packet: SiyiPacket::default(),
            upside_down: false,
            last_attitude_poll_ms: u32::MAX,
            initialized: false,
        }
    }

    /// Feed attitude feedback from gimbal (parsed from 0x0D response).
    pub fn set_attitude_feedback(&mut self, roll: f32, pitch: f32, yaw: f32) {
        if self.upside_down {
            // Upside-down mounting transform:
            // pitch_transformed = -(current_pitch + PI)
            // yaw_transformed = -current_yaw
            self.current_roll = roll;
            self.current_pitch = -(pitch + core::f32::consts::PI);
            self.current_yaw = -yaw;
        } else {
            self.current_roll = roll;
            self.current_pitch = pitch;
            self.current_yaw = yaw;
        }
    }

    /// P-controller: converts angle error to rate scalar in -100..+100.
    fn angle_to_rate_scalar(desired: f32, current: f32) -> i8 {
        let error_rad = desired - current;
        let error_deg = error_rad * 180.0 / core::f32::consts::PI;

        let rate_deg_s = error_deg * ANGLE_P_GAIN;
        let rate_clamped = clamp(rate_deg_s, -MAX_RATE_DEG_S, MAX_RATE_DEG_S);

        let scalar = rate_clamped * 100.0 / MAX_RATE_DEG_S;
        clamp(scalar, -100.0, 100.0) as i8
    }

    /// Build a Siyi rotation command packet.
    fn build_rotation_packet(&mut self, yaw_rate: i8, pitch_rate: i8) {
        let buf = &mut self.out_packet.data;
        let data_len: u16 = 2;

        buf[0] = (SIYI_MAGIC & 0xFF) as u8;
        buf[1] = (SIYI_MAGIC >> 8) as u8;
        buf[2] = 0x00;
        buf[3] = (data_len & 0xFF) as u8;
        buf[4] = (data_len >> 8) as u8;
        buf[5] = (self.seq & 0xFF) as u8;
        buf[6] = (self.seq >> 8) as u8;
        buf[7] = SIYI_CMD_GIMBAL_ROTATION;
        buf[8] = yaw_rate as u8;
        buf[9] = pitch_rate as u8;

        let crc = crc16_ccitt(&buf[0..10]);
        buf[10] = (crc & 0xFF) as u8;
        buf[11] = (crc >> 8) as u8;

        self.out_packet.len = 12;
        self.out_packet.pending = true;
        self.seq = self.seq.wrapping_add(1);
    }

    /// Build a request for gimbal attitude (0x0D command, no payload).
    pub fn build_attitude_request(&mut self) {
        let buf = &mut self.attitude_request_packet.data;
        let data_len: u16 = 0;

        buf[0] = (SIYI_MAGIC & 0xFF) as u8;
        buf[1] = (SIYI_MAGIC >> 8) as u8;
        buf[2] = 0x00;
        buf[3] = (data_len & 0xFF) as u8;
        buf[4] = (data_len >> 8) as u8;
        buf[5] = (self.seq & 0xFF) as u8;
        buf[6] = (self.seq >> 8) as u8;
        buf[7] = SIYI_CMD_GET_ATTITUDE;

        let crc = crc16_ccitt(&buf[0..8]);
        buf[8] = (crc & 0xFF) as u8;
        buf[9] = (crc >> 8) as u8;

        self.attitude_request_packet.len = 10;
        self.attitude_request_packet.pending = true;
        self.seq = self.seq.wrapping_add(1);
    }

    /// Build EXTERNAL_ATTITUDE (0x22) command with AHRS attitude data.
    /// This sends the FC's orientation to the gimbal for stabilization.
    pub fn build_external_attitude(&mut self, roll_rad: f32, pitch_rad: f32, yaw_rad: f32) {
        let buf = &mut self.external_attitude_packet.data;
        let data_len: u16 = 12; // 3 floats

        buf[0] = (SIYI_MAGIC & 0xFF) as u8;
        buf[1] = (SIYI_MAGIC >> 8) as u8;
        buf[2] = 0x00;
        buf[3] = (data_len & 0xFF) as u8;
        buf[4] = (data_len >> 8) as u8;
        buf[5] = (self.seq & 0xFF) as u8;
        buf[6] = (self.seq >> 8) as u8;
        buf[7] = SIYI_CMD_EXTERNAL_ATTITUDE;

        let r = roll_rad.to_le_bytes();
        let p = pitch_rad.to_le_bytes();
        let y = yaw_rad.to_le_bytes();
        buf[8..12].copy_from_slice(&r);
        buf[12..16].copy_from_slice(&p);
        buf[16..20].copy_from_slice(&y);

        let crc = crc16_ccitt(&buf[0..20]);
        buf[20] = (crc & 0xFF) as u8;
        buf[21] = (crc >> 8) as u8;

        self.external_attitude_packet.len = 22;
        self.external_attitude_packet.pending = true;
        self.seq = self.seq.wrapping_add(1);
    }

    /// Check if an attitude poll is due (20Hz) and build the request if so.
    /// Returns true if a request was built.
    pub fn poll_attitude_if_due(&mut self, now_ms: u32) -> bool {
        let elapsed = now_ms.wrapping_sub(self.last_attitude_poll_ms);
        if elapsed >= ATTITUDE_POLL_INTERVAL_MS || self.last_attitude_poll_ms == u32::MAX {
            self.build_attitude_request();
            self.last_attitude_poll_ms = now_ms;
            true
        } else {
            false
        }
    }
}

impl MountBackend for SiyiMount {
    fn init(&mut self) -> bool {
        self.seq = 0;
        self.desired_pitch = 0.0;
        self.desired_yaw = 0.0;
        self.out_packet = SiyiPacket::default();
        self.attitude_request_packet = SiyiPacket::default();
        self.external_attitude_packet = SiyiPacket::default();
        self.initialized = true;
        true
    }

    fn update(&mut self, target: &MountTarget, dt: f32) {
        if !self.initialized || dt <= 0.0 {
            return;
        }

        match target {
            MountTarget::Angle { roll: _, pitch, yaw } => {
                self.desired_pitch = *pitch;
                self.desired_yaw = *yaw;
            }
            MountTarget::Rate { roll_rate: _, pitch_rate, yaw_rate } => {
                self.desired_pitch += pitch_rate * dt;
                self.desired_yaw += yaw_rate * dt;
            }
            MountTarget::Retract | MountTarget::Neutral => {
                self.desired_pitch = 0.0;
                self.desired_yaw = 0.0;
            }
            MountTarget::RoiLocation { .. } => {
                // ROI resolved to angle by MountManager.
            }
        }

        let yaw_scalar = Self::angle_to_rate_scalar(self.desired_yaw, self.current_yaw);
        let pitch_scalar = Self::angle_to_rate_scalar(self.desired_pitch, self.current_pitch);

        self.build_rotation_packet(yaw_scalar, pitch_scalar);
    }

    fn get_attitude(&self) -> (f32, f32, f32) {
        (self.current_roll, self.current_pitch, self.current_yaw)
    }

    fn has_pan_control(&self) -> bool {
        true
    }
}

/// CRC16-CCITT (polynomial 0x1021, init 0x0000).
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0x0000;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[inline]
fn clamp(val: f32, min: f32, max: f32) -> f32 {
    if val < min { min } else if val > max { max } else { val }
}

//! VectorNav VN-100/200/300 External AHRS binary protocol driver.
//!
//! ArduPilot reference: `AP_ExternalAHRS_VectorNav.cpp`
//!
//! Binary protocol: sync byte 0xFA, group fields byte(s), payload, CRC16.
//!
//! Two operating modes:
//! - VN_AHRS (VN-100): 2 packets — IMU data + EKF attitude.
//! - VN_INS  (VN-200/300): 3 packets — IMU + EKF + GNSS position.
//!
//! Configuration via ASCII $VNWRG commands to set binary output registers.

use meridian_hal::UartDriver;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const VN_SYNC_BYTE: u8 = 0xFA;

/// CRC16 polynomial for VectorNav binary protocol.
const VN_CRC_POLY: u16 = 0x8005;

// Group field masks for binary output configuration.
const GROUP_COMMON: u8 = 0x01;
const GROUP_IMU: u8 = 0x04;
const GROUP_ATTITUDE: u8 = 0x08;
const GROUP_INS: u8 = 0x10;
const GROUP_GNSS: u8 = 0x20;

// ---------------------------------------------------------------------------
// CRC
// ---------------------------------------------------------------------------

/// VectorNav CRC16 over a byte slice.
fn vn_crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        crc = (crc >> 8) | (crc << 8);
        crc ^= byte as u16;
        crc ^= (crc & 0xFF) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0x00FF) << 5;
    }
    crc
}

// ---------------------------------------------------------------------------
// Data types
// ---------------------------------------------------------------------------

/// IMU data from VectorNav (high-rate packet).
#[derive(Clone, Default)]
pub struct VnImuData {
    /// Angular rate (rad/s) in body frame.
    pub gyro: [f32; 3],
    /// Acceleration (m/s^2) in body frame.
    pub accel: [f32; 3],
    /// Uncompensated angular rate (rad/s).
    pub uncomp_gyro: [f32; 3],
    /// Uncompensated acceleration (m/s^2).
    pub uncomp_accel: [f32; 3],
    /// Magnetometer (gauss) in body frame.
    pub mag: [f32; 3],
    /// Temperature (degrees C).
    pub temperature: f32,
    /// Barometric pressure (kPa).
    pub pressure: f32,
    /// Timestamp from device startup (nanoseconds).
    pub time_startup_ns: u64,
}

/// EKF attitude data from VectorNav.
#[derive(Clone, Default)]
pub struct VnAttitudeData {
    /// Yaw, pitch, roll (degrees).
    pub ypr: [f32; 3],
    /// Quaternion (w, x, y, z).
    pub quaternion: [f32; 4],
    /// Attitude uncertainty (yaw, pitch, roll in degrees).
    pub ypr_uncertainty: [f32; 3],
    pub time_startup_ns: u64,
}

/// INS position/velocity data from VectorNav VN-200/300.
#[derive(Clone, Default)]
pub struct VnInsData {
    /// INS status word.
    pub ins_status: u16,
    /// Position (latitude, longitude in degrees, altitude in m WGS84).
    pub position_lla: [f64; 3],
    /// Velocity NED (m/s).
    pub velocity_ned: [f32; 3],
    /// Position uncertainty (m).
    pub pos_uncertainty: f32,
    /// Velocity uncertainty (m/s).
    pub vel_uncertainty: f32,
    pub time_startup_ns: u64,
}

/// GNSS data from VectorNav VN-200/300.
#[derive(Clone, Default)]
pub struct VnGnssData {
    pub time_gps_ns: u64,
    pub num_sats: u8,
    pub fix_type: u8,
    pub position_lla: [f64; 3],
    pub velocity_ned: [f32; 3],
    pub pos_uncertainty: f32,
    pub vel_uncertainty: f32,
    pub dop: f32,
}

/// Complete state from VectorNav.
pub struct VnState {
    pub imu: VnImuData,
    pub attitude: VnAttitudeData,
    pub ins: Option<VnInsData>,
    pub gnss: Option<VnGnssData>,
    pub healthy: bool,
    pub last_imu_ms: u32,
    pub last_att_ms: u32,
    pub last_ins_ms: u32,
}

impl VnState {
    pub fn new() -> Self {
        Self {
            imu: VnImuData::default(),
            attitude: VnAttitudeData::default(),
            ins: None,
            gnss: None,
            healthy: false,
            last_imu_ms: 0,
            last_att_ms: 0,
            last_ins_ms: 0,
        }
    }
}

// ---------------------------------------------------------------------------
// Operating mode
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VnMode {
    /// VN-100: attitude + IMU only (2 binary output packets).
    Ahrs,
    /// VN-200/300: full INS with position (3 binary output packets).
    Ins,
}

// ---------------------------------------------------------------------------
// Parser
// ---------------------------------------------------------------------------

const MAX_PACKET: usize = 256;

#[derive(Clone, Copy, PartialEq)]
enum ParseState {
    WaitSync,
    Groups,
    Payload,
    CrcLow,
    CrcHigh,
}

pub struct VnParser {
    state: ParseState,
    buf: [u8; MAX_PACKET],
    buf_pos: usize,
    groups: u8,
    payload_len: usize,
    crc_low: u8,
}

impl VnParser {
    pub fn new() -> Self {
        Self {
            state: ParseState::WaitSync,
            buf: [0u8; MAX_PACKET],
            buf_pos: 0,
            groups: 0,
            payload_len: 0,
            crc_low: 0,
        }
    }

    /// Feed a byte. Returns the raw packet (groups byte + payload) when complete.
    pub fn process_byte(&mut self, byte: u8) -> Option<(u8, &[u8])> {
        match self.state {
            ParseState::WaitSync => {
                if byte == VN_SYNC_BYTE {
                    self.buf_pos = 0;
                    self.state = ParseState::Groups;
                }
                None
            }
            ParseState::Groups => {
                self.groups = byte;
                self.buf[0] = byte;
                self.buf_pos = 1;
                // Estimate payload size from group mask (simplified — each group has known field sizes).
                self.payload_len = estimate_payload_len(byte);
                if self.payload_len == 0 || self.payload_len > MAX_PACKET - 4 {
                    self.state = ParseState::WaitSync;
                    return None;
                }
                self.state = ParseState::Payload;
                None
            }
            ParseState::Payload => {
                if self.buf_pos < self.payload_len + 1 {
                    self.buf[self.buf_pos] = byte;
                    self.buf_pos += 1;
                }
                if self.buf_pos >= self.payload_len + 1 {
                    self.state = ParseState::CrcLow;
                }
                None
            }
            ParseState::CrcLow => {
                self.crc_low = byte;
                self.state = ParseState::CrcHigh;
                None
            }
            ParseState::CrcHigh => {
                self.state = ParseState::WaitSync;
                let received = (self.crc_low as u16) | ((byte as u16) << 8);
                let computed = vn_crc16(&self.buf[..self.buf_pos]);
                if received != computed {
                    return None;
                }
                Some((self.groups, &self.buf[1..self.buf_pos]))
            }
        }
    }
}

/// Estimate payload length from group mask (simplified).
fn estimate_payload_len(groups: u8) -> usize {
    let mut len = 0;
    if groups & GROUP_COMMON != 0 { len += 8; }   // TimeStartup (8 bytes)
    if groups & GROUP_IMU != 0 { len += 48; }      // gyro(12) + accel(12) + uncomp_gyro(12) + mag(12)
    if groups & GROUP_ATTITUDE != 0 { len += 28; }  // YPR(12) + Quat(16)
    if groups & GROUP_INS != 0 { len += 40; }       // status(2) + PosLla(24) + VelNed(12) + PosU(4)
    if groups & GROUP_GNSS != 0 { len += 50; }      // various GNSS fields
    len
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

pub struct VectorNav {
    mode: VnMode,
    parser: VnParser,
    pub state: VnState,
    configured: bool,
}

impl VectorNav {
    pub fn new(mode: VnMode) -> Self {
        Self {
            mode,
            parser: VnParser::new(),
            state: VnState::new(),
            configured: false,
        }
    }

    /// Send ASCII configuration commands to set up binary outputs.
    pub fn configure(&mut self, uart: &mut dyn UartDriver) -> bool {
        // Stop existing outputs.
        let stop = b"$VNWRG,75,0*XX\r\n";
        let _ = uart.write(stop);

        // Configure binary output register 1 (IMU + attitude data).
        let cmd1: &[u8] = match self.mode {
            VnMode::Ahrs => {
                // Groups: Common + IMU + Attitude.
                b"$VNWRG,75,2,16,01,0628,0001*XX\r\n"
            }
            VnMode::Ins => {
                // Groups: Common + IMU + Attitude + INS.
                b"$VNWRG,75,2,16,11,0628,0001,0007*XX\r\n"
            }
        };
        let _ = uart.write(cmd1);

        // For INS mode, configure GNSS output on register 2.
        if self.mode == VnMode::Ins {
            let cmd2 = b"$VNWRG,76,1,16,20,0001,0128*XX\r\n";
            let _ = uart.write(cmd2);
        }

        self.configured = true;
        true
    }

    /// Process available UART bytes and update state.
    pub fn update(&mut self, uart: &mut dyn UartDriver) -> bool {
        let mut got_data = false;
        let mut byte_buf = [0u8; 1];

        while uart.read(&mut byte_buf) > 0 {
            if let Some((_groups, payload)) = self.parser.process_byte(byte_buf[0]) {
                // Copy payload to local buffer to avoid borrow conflict.
                let mut local_buf = [0u8; MAX_PACKET];
                let plen = payload.len().min(MAX_PACKET);
                local_buf[..plen].copy_from_slice(&payload[..plen]);
                self.parse_packet(&local_buf[..plen]);
                got_data = true;
            }
        }

        got_data
    }

    /// Parse a validated binary packet payload.
    fn parse_packet(&mut self, data: &[u8]) {
        if data.len() < 8 { return; }

        // For now, extract time + IMU + attitude from flat payload.
        // Real implementation would parse based on group mask and field masks.
        let mut offset = 0;

        // TimeStartup (8 bytes, nanoseconds).
        if data.len() >= offset + 8 {
            let time_ns = u64::from_le_bytes([
                data[offset], data[offset+1], data[offset+2], data[offset+3],
                data[offset+4], data[offset+5], data[offset+6], data[offset+7],
            ]);
            self.state.imu.time_startup_ns = time_ns;
            offset += 8;
        }

        // Angular rate (3 x f32 = 12 bytes).
        if data.len() >= offset + 12 {
            for i in 0..3 {
                let bytes = [data[offset+i*4], data[offset+i*4+1], data[offset+i*4+2], data[offset+i*4+3]];
                self.state.imu.gyro[i] = f32::from_le_bytes(bytes);
            }
            offset += 12;
        }

        // Acceleration (3 x f32 = 12 bytes).
        if data.len() >= offset + 12 {
            for i in 0..3 {
                let bytes = [data[offset+i*4], data[offset+i*4+1], data[offset+i*4+2], data[offset+i*4+3]];
                self.state.imu.accel[i] = f32::from_le_bytes(bytes);
            }
            offset += 12;
        }

        // Quaternion (4 x f32 = 16 bytes) — if attitude group present.
        if data.len() >= offset + 16 {
            for i in 0..4 {
                let bytes = [data[offset+i*4], data[offset+i*4+1], data[offset+i*4+2], data[offset+i*4+3]];
                self.state.attitude.quaternion[i] = f32::from_le_bytes(bytes);
            }
            offset += 16;
        }

        // INS position (3 x f64 = 24 bytes) — if INS group present.
        if self.mode == VnMode::Ins && data.len() >= offset + 24 {
            let mut ins = VnInsData::default();
            for i in 0..3 {
                let mut bytes = [0u8; 8];
                bytes.copy_from_slice(&data[offset+i*8..offset+i*8+8]);
                ins.position_lla[i] = f64::from_le_bytes(bytes);
            }
            self.state.ins = Some(ins);
        }

        self.state.healthy = true;
    }

    pub fn is_healthy(&self) -> bool {
        self.state.healthy
    }

    pub fn is_configured(&self) -> bool {
        self.configured
    }

    pub fn mode(&self) -> VnMode {
        self.mode
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc16() {
        let data = [0x01, 0x02, 0x03, 0x04];
        let crc = vn_crc16(&data);
        assert!(crc != 0, "CRC should be nonzero for real data");

        // Same data should produce same CRC.
        let crc2 = vn_crc16(&data);
        assert_eq!(crc, crc2);

        // Different data should produce different CRC.
        let crc3 = vn_crc16(&[0x04, 0x03, 0x02, 0x01]);
        assert_ne!(crc, crc3);
    }

    #[test]
    fn test_payload_estimation() {
        // Common group only: 8 bytes.
        assert_eq!(estimate_payload_len(GROUP_COMMON), 8);

        // IMU group: 48 bytes.
        assert_eq!(estimate_payload_len(GROUP_IMU), 48);

        // Common + IMU + Attitude: 8 + 48 + 28 = 84.
        assert_eq!(estimate_payload_len(GROUP_COMMON | GROUP_IMU | GROUP_ATTITUDE), 84);
    }

    #[test]
    fn test_parser_sync() {
        let mut parser = VnParser::new();
        // Garbage bytes should not produce output.
        for &byte in &[0x00, 0x12, 0xFF, 0x34] {
            assert!(parser.process_byte(byte).is_none());
        }
    }

    #[test]
    fn test_state_init() {
        let state = VnState::new();
        assert!(!state.healthy);
        assert!(state.ins.is_none());
        assert!(state.gnss.is_none());
    }

    #[test]
    fn test_mode_selection() {
        let vn = VectorNav::new(VnMode::Ahrs);
        assert_eq!(vn.mode(), VnMode::Ahrs);

        let vn_ins = VectorNav::new(VnMode::Ins);
        assert_eq!(vn_ins.mode(), VnMode::Ins);
    }
}

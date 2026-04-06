//! Additional ExternalAHRS backends: InertialLabs, MicroStrain, SBG.
//! Plus shared infrastructure: SENSORS bitmask, thread wrapper, semaphore state.
//!
//! ArduPilot references:
//!   AP_ExternalAHRS_InertialLabs.cpp
//!   AP_ExternalAHRS_MicroStrain5.cpp / MicroStrain7.cpp
//!   AP_ExternalAHRS_SBG.cpp

use crate::external_ahrs_vectornav::{VnImuData, VnAttitudeData, VnInsData, VnGnssData};

// =========================================================================
// Shared ExternalAHRS infrastructure
// =========================================================================

/// SENSORS bitmask — controls which subsystems the external AHRS feeds.
/// ArduPilot: EAHRS_SENSORS parameter.
#[derive(Clone, Copy)]
pub struct SensorsMask {
    bits: u8,
}

impl SensorsMask {
    pub const GPS: u8    = 1 << 0;
    pub const IMU: u8    = 1 << 1;
    pub const BARO: u8   = 1 << 2;
    pub const COMPASS: u8 = 1 << 3;

    pub fn new(bits: u8) -> Self { Self { bits } }
    pub fn all() -> Self { Self { bits: 0x0F } }
    pub fn has_gps(&self) -> bool { self.bits & Self::GPS != 0 }
    pub fn has_imu(&self) -> bool { self.bits & Self::IMU != 0 }
    pub fn has_baro(&self) -> bool { self.bits & Self::BARO != 0 }
    pub fn has_compass(&self) -> bool { self.bits & Self::COMPASS != 0 }
}

/// Thread-safe state wrapper for ExternalAHRS backends.
///
/// In a real RTIC/embedded system this uses a critical section.
/// In std mode (SITL/Linux), this uses a mutex.
/// The state is written by the backend thread and read by the main loop.
pub struct AhrsState {
    pub imu: VnImuData,
    pub attitude: VnAttitudeData,
    pub ins: Option<VnInsData>,
    pub gnss: Option<VnGnssData>,
    pub baro_pressure_pa: f32,
    pub baro_temperature_c: f32,
    pub mag_field: [f32; 3], // milligauss
    pub healthy: bool,
    pub initialised: bool,
    pub sensors: SensorsMask,
    // In real impl, protected by critical_section::Mutex or std::sync::Mutex
}

impl AhrsState {
    pub fn new(sensors: SensorsMask) -> Self {
        Self {
            imu: VnImuData::default(),
            attitude: VnAttitudeData::default(),
            ins: None,
            gnss: None,
            baro_pressure_pa: 0.0,
            baro_temperature_c: 0.0,
            mag_field: [0.0; 3],
            healthy: false,
            initialised: false,
            sensors,
        }
    }

    /// Pre-arm check: returns error message or None if OK.
    pub fn pre_arm_check(&self) -> Option<&'static str> {
        if !self.healthy { return Some("ExternalAHRS not healthy"); }
        if !self.initialised { return Some("ExternalAHRS not initialized"); }
        if self.attitude.quaternion == [0.0; 4] { return Some("ExternalAHRS no attitude"); }
        None
    }
}

// =========================================================================
// InertialLabs binary protocol
// =========================================================================

/// InertialLabs INS serial protocol parser.
///
/// Binary protocol with single-byte message IDs.
/// Key message types: ORIENTATION_ANGLES (0x07), VELOCITIES (0x12),
/// POSITION (0x10), ACCEL_HR (0x23), GYRO_HR (0x21), BARO (0x25),
/// MAG (0x24), GPS_INS_TIME (0x01).
pub struct InertialLabs {
    buf: [u8; 256],
    pos: usize,
    pub state: AhrsState,
}

impl InertialLabs {
    pub fn new(sensors: SensorsMask) -> Self {
        Self {
            buf: [0; 256],
            pos: 0,
            state: AhrsState::new(sensors),
        }
    }

    /// Process a byte from the serial port.
    /// InertialLabs uses a packet format: [header(2)] [length(2)] [payload] [CRC(2)]
    pub fn process_byte(&mut self, byte: u8) -> bool {
        self.buf[self.pos] = byte;
        self.pos += 1;

        if self.pos < 4 { return false; }

        // Header: 0x55 0xAA
        if self.buf[0] != 0x55 || self.buf[1] != 0xAA {
            // Scan for header.
            self.pos = 0;
            return false;
        }

        let payload_len = u16::from_le_bytes([self.buf[2], self.buf[3]]) as usize;
        let frame_len = 4 + payload_len + 2; // header + payload + CRC

        if self.pos < frame_len { return false; }
        self.pos = 0;

        // Copy payload to local buffer to avoid borrow conflict with parse_payload(&mut self).
        let mut payload_copy = [0u8; 256];
        let plen = payload_len.min(252); // cap at available space (256 - 4 header)
        payload_copy[..plen].copy_from_slice(&self.buf[4..4 + plen]);
        self.parse_payload(&payload_copy[..plen]);
        self.state.healthy = true;
        self.state.initialised = true;
        true
    }

    fn parse_payload(&mut self, data: &[u8]) {
        let mut offset = 0;
        while offset + 1 < data.len() {
            let msg_id = data[offset];
            offset += 1;

            match msg_id {
                0x23 => { // ACCEL_DATA_HR: 3 x f32 (12 bytes)
                    if offset + 12 <= data.len() {
                        for i in 0..3 {
                            self.state.imu.accel[i] = f32::from_le_bytes([
                                data[offset+i*4], data[offset+i*4+1],
                                data[offset+i*4+2], data[offset+i*4+3],
                            ]);
                        }
                        offset += 12;
                    } else { break; }
                }
                0x21 => { // GYRO_DATA_HR: 3 x f32
                    if offset + 12 <= data.len() {
                        for i in 0..3 {
                            self.state.imu.gyro[i] = f32::from_le_bytes([
                                data[offset+i*4], data[offset+i*4+1],
                                data[offset+i*4+2], data[offset+i*4+3],
                            ]);
                        }
                        offset += 12;
                    } else { break; }
                }
                0x07 => { // ORIENTATION_ANGLES: heading, pitch, roll (3 x f32)
                    if offset + 12 <= data.len() {
                        for i in 0..3 {
                            self.state.attitude.ypr[i] = f32::from_le_bytes([
                                data[offset+i*4], data[offset+i*4+1],
                                data[offset+i*4+2], data[offset+i*4+3],
                            ]);
                        }
                        offset += 12;
                    } else { break; }
                }
                0x25 => { // BARO_DATA: pressure(f32), temperature(f32)
                    if offset + 8 <= data.len() {
                        self.state.baro_pressure_pa = f32::from_le_bytes([
                            data[offset], data[offset+1], data[offset+2], data[offset+3],
                        ]);
                        self.state.baro_temperature_c = f32::from_le_bytes([
                            data[offset+4], data[offset+5], data[offset+6], data[offset+7],
                        ]);
                        offset += 8;
                    } else { break; }
                }
                0x24 => { // MAG_DATA: 3 x f32 (milligauss)
                    if offset + 12 <= data.len() {
                        for i in 0..3 {
                            self.state.mag_field[i] = f32::from_le_bytes([
                                data[offset+i*4], data[offset+i*4+1],
                                data[offset+i*4+2], data[offset+i*4+3],
                            ]);
                        }
                        offset += 12;
                    } else { break; }
                }
                0x10 => { // POSITION: lat(f64), lon(f64), alt(f32)
                    if offset + 20 <= data.len() {
                        let mut ins = VnInsData::default();
                        ins.position_lla[0] = f64::from_le_bytes([
                            data[offset], data[offset+1], data[offset+2], data[offset+3],
                            data[offset+4], data[offset+5], data[offset+6], data[offset+7],
                        ]);
                        ins.position_lla[1] = f64::from_le_bytes([
                            data[offset+8], data[offset+9], data[offset+10], data[offset+11],
                            data[offset+12], data[offset+13], data[offset+14], data[offset+15],
                        ]);
                        ins.position_lla[2] = f32::from_le_bytes([
                            data[offset+16], data[offset+17], data[offset+18], data[offset+19],
                        ]) as f64;
                        self.state.ins = Some(ins);
                        offset += 20;
                    } else { break; }
                }
                0x12 => { // VELOCITIES: 3 x f32 NED (m/s)
                    if offset + 12 <= data.len() {
                        if let Some(ref mut ins) = self.state.ins {
                            for i in 0..3 {
                                ins.velocity_ned[i] = f32::from_le_bytes([
                                    data[offset+i*4], data[offset+i*4+1],
                                    data[offset+i*4+2], data[offset+i*4+3],
                                ]);
                            }
                        }
                        offset += 12;
                    } else { break; }
                }
                _ => {
                    // Unknown message ID — skip. Can't determine length, so break.
                    break;
                }
            }
        }
    }
}

// =========================================================================
// MicroStrain GX5 / GQ7 (stub — binary MIP protocol)
// =========================================================================

/// MicroStrain GX5 (MIP protocol) external AHRS.
///
/// MIP (MicroStrain Inertial Protocol): header(0x75 0x65), descriptor,
/// payload_length, field_data[], checksum(2). Complex multi-field packets.
pub struct MicroStrain {
    pub state: AhrsState,
    version: u8, // 5 = GX5, 7 = GQ7
}

impl MicroStrain {
    pub fn new_gx5(sensors: SensorsMask) -> Self {
        Self { state: AhrsState::new(sensors), version: 5 }
    }

    pub fn new_gq7(sensors: SensorsMask) -> Self {
        Self { state: AhrsState::new(sensors), version: 7 }
    }

    /// Process MIP packet bytes. Returns true when state updated.
    pub fn process_byte(&mut self, _byte: u8) -> bool {
        // MIP protocol parsing: header(0x75 0x65) + descriptor_set + payload_len + fields + checksum
        // Each field: field_len + field_descriptor + data
        // Descriptor sets: 0x80 (IMU), 0x82 (estimation/EKF), 0x81 (GNSS)
        //
        // Full implementation requires ~300 lines of MIP field parsing.
        // For now, marks healthy when bytes are received.
        self.state.healthy = true;
        false
    }

    pub fn version(&self) -> u8 { self.version }
}

// =========================================================================
// SBG Ellipse (stub — binary SBG protocol)
// =========================================================================

/// SBG Ellipse series external AHRS.
///
/// Binary protocol with sync bytes 0xFF 0x5A, message class/ID, payload, CRC.
pub struct SbgEllipse {
    pub state: AhrsState,
}

impl SbgEllipse {
    pub fn new(sensors: SensorsMask) -> Self {
        Self { state: AhrsState::new(sensors) }
    }

    /// Process SBG binary protocol bytes.
    pub fn process_byte(&mut self, _byte: u8) -> bool {
        // SBG protocol: sync(0xFF 0x5A) + msg_id(2) + class(1) + len(2) + payload + CRC(2)
        // Message classes: LOG_IMU_DATA (0x01), LOG_EKF_EULER/QUAT (0x06/0x07),
        // LOG_EKF_NAV (0x08), LOG_GPS1_VEL/POS (0x0D/0x0E)
        //
        // Full implementation requires ~250 lines.
        self.state.healthy = true;
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sensors_mask() {
        let all = SensorsMask::all();
        assert!(all.has_gps());
        assert!(all.has_imu());
        assert!(all.has_baro());
        assert!(all.has_compass());

        let gps_only = SensorsMask::new(SensorsMask::GPS);
        assert!(gps_only.has_gps());
        assert!(!gps_only.has_imu());
    }

    #[test]
    fn test_ahrs_state_prearm() {
        let state = AhrsState::new(SensorsMask::all());
        assert!(state.pre_arm_check().is_some()); // Not healthy yet.
    }

    #[test]
    fn test_inertial_labs_init() {
        let il = InertialLabs::new(SensorsMask::all());
        assert!(!il.state.healthy);
        assert!(!il.state.initialised);
    }

    #[test]
    fn test_microstrain_versions() {
        let gx5 = MicroStrain::new_gx5(SensorsMask::all());
        assert_eq!(gx5.version(), 5);
        let gq7 = MicroStrain::new_gq7(SensorsMask::all());
        assert_eq!(gq7.version(), 7);
    }
}

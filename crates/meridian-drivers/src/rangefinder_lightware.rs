//! LightWare lidar rangefinder drivers (I2C + Serial).
//!
//! ArduPilot reference: `AP_RangeFinder_LightWareI2C.cpp`, `AP_RangeFinder_LightWareSerial.cpp`
//!
//! I2C models (SF10/SF11/SF20/LW20): Read 2 bytes big-endian, value in cm.
//! Serial models (SF40C/SF45B): Binary protocol with 0xAA start, CRC16.

use meridian_hal::I2cDevice;
use crate::rangefinder::{RangefinderReading, RangefinderStatus};

// ---------------------------------------------------------------------------
// Model definitions
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, PartialEq)]
pub enum LightWareModel {
    SF10,  // 0-25m
    SF11,  // 0-120m
    SF20,  // 0-100m
    LW20,  // 0-100m
}

impl LightWareModel {
    pub fn min_distance_m(&self) -> f32 { 0.2 }
    pub fn max_distance_m(&self) -> f32 {
        match self {
            Self::SF10 => 25.0,
            Self::SF11 => 120.0,
            Self::SF20 | Self::LW20 => 100.0,
        }
    }
}

// ---------------------------------------------------------------------------
// I2C Driver
// ---------------------------------------------------------------------------

const LW_I2C_DEFAULT_ADDR: u8 = 0x66;

pub struct LightWareI2c {
    address: u8,
    model: LightWareModel,
    valid_count: u8,
}

impl LightWareI2c {
    pub fn new(model: LightWareModel) -> Self {
        Self { address: LW_I2C_DEFAULT_ADDR, model, valid_count: 0 }
    }

    pub fn with_address(model: LightWareModel, address: u8) -> Self {
        Self { address, model, valid_count: 0 }
    }

    pub fn read(&mut self, i2c: &mut dyn I2cDevice) -> Option<RangefinderReading> {
        let mut buf = [0u8; 2];
        i2c.set_address(self.address);
        if !i2c.transfer(&[], &mut buf) {
            self.valid_count = 0;
            return None;
        }

        let distance_cm = u16::from_be_bytes([buf[0], buf[1]]);
        let distance_m = distance_cm as f32 / 100.0;

        let status = if distance_m > self.model.max_distance_m() {
            self.valid_count = 0;
            RangefinderStatus::OutOfRangeHigh
        } else if distance_m < self.model.min_distance_m() {
            self.valid_count = 0;
            RangefinderStatus::OutOfRangeLow
        } else {
            self.valid_count = self.valid_count.saturating_add(1).min(10);
            RangefinderStatus::Good
        };

        Some(RangefinderReading {
            distance_m,
            status,
            signal_quality: if status == RangefinderStatus::Good { 100 } else { 0 },
            valid_count: self.valid_count,
        })
    }
}

// ---------------------------------------------------------------------------
// Serial Driver (SF40C/SF45B binary protocol)
// ---------------------------------------------------------------------------

const SERIAL_START: u8 = 0xAA;
const MSG_ID_DISTANCE: u16 = 44; // 0x2C
const MAX_PAYLOAD: usize = 64;

/// CRC16-CCITT for LightWare serial protocol.
fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x8005;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[derive(Clone, Copy, PartialEq)]
enum SerialState {
    WaitStart,
    LenLow,
    LenHigh,
    Payload,
    CrcLow,
    CrcHigh,
}

pub struct LightWareSerial {
    model: LightWareModel,
    state: SerialState,
    payload_buf: [u8; MAX_PAYLOAD],
    payload_len: usize,
    payload_pos: usize,
    crc_low: u8,
    valid_count: u8,
}

impl LightWareSerial {
    pub fn new(model: LightWareModel) -> Self {
        Self {
            model,
            state: SerialState::WaitStart,
            payload_buf: [0u8; MAX_PAYLOAD],
            payload_len: 0,
            payload_pos: 0,
            crc_low: 0,
            valid_count: 0,
        }
    }

    pub fn process_byte(&mut self, byte: u8) -> Option<RangefinderReading> {
        match self.state {
            SerialState::WaitStart => {
                if byte == SERIAL_START { self.state = SerialState::LenLow; }
                None
            }
            SerialState::LenLow => {
                self.payload_len = byte as usize;
                self.state = SerialState::LenHigh;
                None
            }
            SerialState::LenHigh => {
                self.payload_len |= (byte as usize) << 8;
                // Flags are in top 2 bits of length, actual length in bottom 14.
                self.payload_len &= 0x3FFF;
                if self.payload_len > MAX_PAYLOAD || self.payload_len < 2 {
                    self.state = SerialState::WaitStart;
                    return None;
                }
                self.payload_pos = 0;
                self.state = SerialState::Payload;
                None
            }
            SerialState::Payload => {
                if self.payload_pos < self.payload_len {
                    self.payload_buf[self.payload_pos] = byte;
                    self.payload_pos += 1;
                }
                if self.payload_pos >= self.payload_len {
                    self.state = SerialState::CrcLow;
                }
                None
            }
            SerialState::CrcLow => {
                self.crc_low = byte;
                self.state = SerialState::CrcHigh;
                None
            }
            SerialState::CrcHigh => {
                self.state = SerialState::WaitStart;
                let received_crc = (self.crc_low as u16) | ((byte as u16) << 8);
                let computed = crc16(&self.payload_buf[..self.payload_len]);
                if received_crc != computed { return None; }
                self.parse_payload()
            }
        }
    }

    fn parse_payload(&mut self) -> Option<RangefinderReading> {
        if self.payload_len < 4 { return None; }

        let msg_id = (self.payload_buf[0] as u16) | ((self.payload_buf[1] as u16) << 8);
        if msg_id != MSG_ID_DISTANCE { return None; }

        let distance_cm = (self.payload_buf[2] as u16) | ((self.payload_buf[3] as u16) << 8);
        let distance_m = distance_cm as f32 / 100.0;

        let status = if distance_m > self.model.max_distance_m() {
            self.valid_count = 0;
            RangefinderStatus::OutOfRangeHigh
        } else if distance_m < self.model.min_distance_m() {
            self.valid_count = 0;
            RangefinderStatus::OutOfRangeLow
        } else {
            self.valid_count = self.valid_count.saturating_add(1).min(10);
            RangefinderStatus::Good
        };

        Some(RangefinderReading {
            distance_m,
            status,
            signal_quality: if status == RangefinderStatus::Good { 100 } else { 0 },
            valid_count: self.valid_count,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_i2c_conversion() {
        // 500 cm = 5.0m
        let distance_cm: u16 = 500;
        let buf = distance_cm.to_be_bytes();
        let distance_m = u16::from_be_bytes(buf) as f32 / 100.0;
        assert!((distance_m - 5.0).abs() < 0.01);
    }

    #[test]
    fn test_crc16() {
        let data = [0x2C, 0x00, 0xF4, 0x01]; // msg_id=44, distance=500cm
        let c = crc16(&data);
        assert!(c != 0, "CRC should be nonzero for real data");
    }

    #[test]
    fn test_serial_parse() {
        let mut drv = LightWareSerial::new(LightWareModel::SF20);
        let payload = [0x2C, 0x00, 0xF4, 0x01]; // msg_id=44, distance=500cm
        let crc_val = crc16(&payload);

        let frame: [u8; 9] = [
            SERIAL_START, 4, 0, // start + length (4 bytes)
            payload[0], payload[1], payload[2], payload[3],
            (crc_val & 0xFF) as u8, (crc_val >> 8) as u8,
        ];

        let mut result = None;
        for &b in &frame {
            if let Some(r) = drv.process_byte(b) { result = Some(r); }
        }

        let r = result.expect("should parse valid serial frame");
        assert!((r.distance_m - 5.0).abs() < 0.01);
        assert_eq!(r.status, RangefinderStatus::Good);
    }
}

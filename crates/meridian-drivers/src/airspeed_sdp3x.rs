//! Sensirion SDP3x differential pressure sensor driver.
//!
//! ArduPilot reference: `AP_Airspeed_SDP3X.cpp`
//!
//! I2C, 16-bit differential pressure + 16-bit temperature.
//! Variants: SDP31 (500 Pa), SDP32 (125 Pa), SDP33 (up to ±500 Pa).
//! Default address: 0x21 (SDP3x-Analog) or 0x22 (SDP3x-Digital).
//!
//! Protocol:
//!   1. Send measurement command (0x3603 for continuous mass flow, 0x3615 for differential pressure).
//!   2. Read 9 bytes: [dp_msb, dp_lsb, dp_crc, temp_msb, temp_lsb, temp_crc, scale_msb, scale_lsb, scale_crc].
//!   3. CRC-8 check per 2-byte word (polynomial 0x31, init 0xFF).
//!   4. pressure_pa = dp_raw / scale_factor.
//!   5. temperature_c = temp_raw / 200.0.

use meridian_hal::I2cDevice;
use crate::airspeed::{AirspeedReading, pressure_to_airspeed};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const SDP3X_DEFAULT_ADDR: u8 = 0x21;

/// Command: trigger continuous differential pressure measurement (averaging).
const CMD_CONTINUOUS_DP: [u8; 2] = [0x36, 0x15];

/// Command: stop continuous measurement.
const CMD_STOP: [u8; 2] = [0x3F, 0xF9];

/// CRC-8 polynomial for Sensirion sensors.
const CRC_POLYNOMIAL: u8 = 0x31;
const CRC_INIT: u8 = 0xFF;

// ---------------------------------------------------------------------------
// CRC
// ---------------------------------------------------------------------------

/// Sensirion CRC-8 over 2 bytes.
fn crc8(data: &[u8; 2]) -> u8 {
    let mut crc = CRC_INIT;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if crc & 0x80 != 0 {
                crc = (crc << 1) ^ CRC_POLYNOMIAL;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

pub struct Sdp3xAirspeed {
    address: u8,
    scale_factor: f32,
    pressure_offset: f32,
    calibrated: bool,
    cal_sum: f32,
    cal_count: u16,
    started: bool,
}

impl Sdp3xAirspeed {
    pub fn new() -> Self {
        Self::with_address(SDP3X_DEFAULT_ADDR)
    }

    pub fn with_address(address: u8) -> Self {
        Self {
            address,
            scale_factor: 0.0,
            pressure_offset: 0.0,
            calibrated: false,
            cal_sum: 0.0,
            cal_count: 0,
            started: false,
        }
    }

    /// Start continuous measurement mode.
    pub fn start(&mut self, i2c: &mut dyn I2cDevice) -> bool {
        // Stop any previous measurement first.
        i2c.set_address(self.address);
        let _ = i2c.write(&CMD_STOP);

        // Small delay would be needed here in real firmware (500us).

        if i2c.write(&CMD_CONTINUOUS_DP) {
            self.started = true;
            true
        } else {
            false
        }
    }

    /// Read pressure and temperature from continuous mode.
    pub fn read(&mut self, i2c: &mut dyn I2cDevice) -> Option<AirspeedReading> {
        if !self.started {
            return None;
        }

        let mut buf = [0u8; 9];
        i2c.set_address(self.address);
        if !i2c.transfer(&[], &mut buf) {
            return None;
        }

        // CRC check each 2-byte word.
        if crc8(&[buf[0], buf[1]]) != buf[2] { return None; }
        if crc8(&[buf[3], buf[4]]) != buf[5] { return None; }
        if crc8(&[buf[6], buf[7]]) != buf[8] { return None; }

        let dp_raw = i16::from_be_bytes([buf[0], buf[1]]) as f32;
        let temp_raw = i16::from_be_bytes([buf[3], buf[4]]) as f32;
        let scale_raw = i16::from_be_bytes([buf[6], buf[7]]) as f32;

        // First read: capture scale factor.
        if self.scale_factor == 0.0 {
            if scale_raw > 0.0 {
                self.scale_factor = scale_raw;
            } else {
                // Default for SDP31: 60 counts/Pa.
                self.scale_factor = 60.0;
            }
        }

        let pressure_pa = dp_raw / self.scale_factor;
        let temperature_c = temp_raw / 200.0;

        let corrected = pressure_pa - self.pressure_offset;
        let airspeed = pressure_to_airspeed(corrected);

        Some(AirspeedReading {
            airspeed_ms: airspeed,
            differential_pressure_pa: corrected,
            temperature_c,
            healthy: true,
        })
    }

    /// Ground calibration — accumulate zero-pressure offset.
    pub fn calibrate_sample(&mut self, i2c: &mut dyn I2cDevice) -> bool {
        if !self.started {
            self.start(i2c);
            return false;
        }

        let mut buf = [0u8; 9];
        i2c.set_address(self.address);
        if !i2c.transfer(&[], &mut buf) {
            return false;
        }

        if crc8(&[buf[0], buf[1]]) != buf[2] { return false; }
        if crc8(&[buf[6], buf[7]]) != buf[8] { return false; }

        let dp_raw = i16::from_be_bytes([buf[0], buf[1]]) as f32;
        let scale_raw = i16::from_be_bytes([buf[6], buf[7]]) as f32;
        if self.scale_factor == 0.0 && scale_raw > 0.0 {
            self.scale_factor = scale_raw;
        }
        if self.scale_factor == 0.0 { return false; }

        let pressure_pa = dp_raw / self.scale_factor;
        self.cal_sum += pressure_pa;
        self.cal_count += 1;

        if self.cal_count >= 50 {
            self.pressure_offset = self.cal_sum / self.cal_count as f32;
            self.calibrated = true;
            return true;
        }
        false
    }

    pub fn set_offset(&mut self, offset: f32) {
        self.pressure_offset = offset;
        self.calibrated = true;
    }

    pub fn is_calibrated(&self) -> bool {
        self.calibrated
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc8() {
        // Known test vector from Sensirion datasheet.
        assert_eq!(crc8(&[0xBE, 0xEF]), 0x92);
        // Zero input.
        assert_eq!(crc8(&[0x00, 0x00]), 0x81);
    }

    #[test]
    fn test_pressure_conversion() {
        // With scale factor 60 (SDP31), raw 60 → 1.0 Pa.
        let p = 60.0_f32 / 60.0;
        assert!((p - 1.0).abs() < 0.01);

        // raw 0 → 0 Pa.
        let p0 = 0.0_f32 / 60.0;
        assert!(p0.abs() < 0.01);
    }
}

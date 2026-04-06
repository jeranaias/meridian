//! DLVR digital differential pressure sensor driver (All Sensors).
//!
//! ArduPilot reference: `AP_Airspeed_DLVR.cpp`
//!
//! I2C-based digital pressure sensor. Available in multiple pressure ranges
//! (DLVR-L01D, DLVR-L02D, etc.). Output is 14-bit digital with built-in
//! temperature compensation.

use meridian_hal::I2cDevice;
use crate::airspeed::{AirspeedReading, pressure_to_airspeed};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const DLVR_DEFAULT_ADDR: u8 = 0x28;

/// Pressure range in inches of water (for DLVR-L01D).
/// Other variants use different ranges.
const DLVR_RANGE_INH2O: f32 = 1.0;

/// 1 inch of water = 248.84 Pa.
const INH2O_TO_PA: f32 = 248.84;

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

/// DLVR airspeed sensor.
pub struct DlvrAirspeed {
    address: u8,
    range_pa: f32,
    pressure_offset: f32,
    calibrated: bool,
    cal_sum: f32,
    cal_count: u16,
}

impl DlvrAirspeed {
    /// Create a new DLVR driver with the given I2C address and pressure range.
    ///
    /// `range_inh2o`: Full-scale pressure range in inches of water (e.g., 1.0 for L01D).
    pub fn new(address: u8, range_inh2o: f32) -> Self {
        Self {
            address,
            range_pa: range_inh2o * INH2O_TO_PA,
            pressure_offset: 0.0,
            calibrated: false,
            cal_sum: 0.0,
            cal_count: 0,
        }
    }

    /// Create with default address and 1" H2O range.
    pub fn default_config() -> Self {
        Self::new(DLVR_DEFAULT_ADDR, DLVR_RANGE_INH2O)
    }

    /// Read and process a measurement.
    pub fn read(&mut self, i2c: &mut dyn I2cDevice) -> Option<AirspeedReading> {
        i2c.set_address(self.address);

        let mut buf = [0u8; 4];
        if !i2c.transfer(&[], &mut buf) {
            return None;
        }

        let status = (buf[0] >> 6) & 0x03;
        if status != 0 {
            return None; // not valid data
        }

        // Pressure: 14-bit, bytes 0-1 (status bits masked out)
        let press_raw = (((buf[0] & 0x3F) as u16) << 8) | (buf[1] as u16);
        // Temperature: 11-bit, bytes 2-3
        let temp_raw = ((buf[2] as u16) << 3) | ((buf[3] as u16) >> 5);

        // Convert pressure: centered at 8192, range maps to +-range_pa
        let dp_pa = (press_raw as f32 - 8192.0) / 8192.0 * self.range_pa;

        // Convert temperature: 0 → -50°C, 2047 → 150°C
        let temp_c = temp_raw as f32 * (200.0 / 2047.0) - 50.0;

        // Ground calibration
        if !self.calibrated {
            self.cal_sum += dp_pa;
            self.cal_count += 1;
            if self.cal_count >= 50 {
                self.pressure_offset = self.cal_sum / self.cal_count as f32;
                self.calibrated = true;
            }
            return Some(AirspeedReading {
                differential_pressure_pa: 0.0,
                temperature_c: temp_c,
                airspeed_ms: 0.0,
                healthy: false,
            });
        }

        let corrected_dp = dp_pa - self.pressure_offset;
        let airspeed = pressure_to_airspeed(corrected_dp);

        Some(AirspeedReading {
            differential_pressure_pa: corrected_dp,
            temperature_c: temp_c,
            airspeed_ms: airspeed,
            healthy: true,
        })
    }

    pub fn is_calibrated(&self) -> bool {
        self.calibrated
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pressure_conversion() {
        // Center value (8192) should give ~0 Pa
        let press_raw = 8192u16;
        let dp = (press_raw as f32 - 8192.0) / 8192.0 * 248.84;
        assert!(dp.abs() < 0.01);
    }
}

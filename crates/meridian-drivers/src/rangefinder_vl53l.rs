//! STMicroelectronics VL53L0X / VL53L1X time-of-flight I2C rangefinders.
//!
//! ArduPilot reference: `AP_RangeFinder_VL53L0X.cpp`, `AP_RangeFinder_VL53L1X.cpp`
//!
//! VL53L0X: 30-1200mm range, I2C address 0x29.
//! VL53L1X: 40-4000mm range (short mode) or 40-1300mm (long mode), I2C address 0x29.
//!
//! Both use complex multi-register init sequences. The VL53L0X requires a
//! ~40-register boot sequence; VL53L1X uses a different register map.
//! After init, reading is simple: poll a status register, then read the
//! 16-bit range in mm.

use meridian_hal::I2cDevice;
use crate::rangefinder::{RangefinderReading, RangefinderStatus};

// ---------------------------------------------------------------------------
// VL53L0X
// ---------------------------------------------------------------------------

const VL53L0X_ADDR: u8 = 0x29;

// Key registers.
const REG_SYSRANGE_START: u8 = 0x00;
const REG_RESULT_RANGE_STATUS: u8 = 0x14;
const REG_RESULT_RANGE_VAL: u8 = 0x1E; // 16-bit, mm

const VL53L0X_MAX_MM: u16 = 1200;
const VL53L0X_MIN_MM: u16 = 30;

/// VL53L0X init tuning sequence (simplified from ST API).
/// Each pair is (register, value). ArduPilot uses ~40 writes.
const VL53L0X_INIT_SEQ: &[(u8, u8)] = &[
    (0x88, 0x00), (0x80, 0x01), (0xFF, 0x01), (0x00, 0x00),
    (0x00, 0x01), (0xFF, 0x00), (0x80, 0x00),
    // Timing budget: ~33ms measurement period.
    (0x01, 0xFF), // SYSTEM_SEQUENCE_CONFIG: range + DSS
    (REG_SYSRANGE_START, 0x01), // single-shot start (first trigger)
];

pub struct Vl53l0x {
    address: u8,
    initialized: bool,
    valid_count: u8,
}

impl Vl53l0x {
    pub fn new() -> Self {
        Self { address: VL53L0X_ADDR, initialized: false, valid_count: 0 }
    }

    pub fn with_address(address: u8) -> Self {
        Self { address, initialized: false, valid_count: 0 }
    }

    /// Run the init sequence. Call once after power-on.
    pub fn init(&mut self, i2c: &mut dyn I2cDevice) -> bool {
        i2c.set_address(self.address);
        // Verify device ID.
        let mut id_buf = [0u8; 1];
        if !i2c.transfer(&[0xC0], &mut id_buf) {
            return false;
        }
        if id_buf[0] != 0xEE {
            return false; // Not a VL53L0X.
        }

        // Write init sequence.
        for &(reg, val) in VL53L0X_INIT_SEQ {
            if !i2c.write_register(reg, val) {
                return false;
            }
        }

        self.initialized = true;
        true
    }

    /// Start a single-shot measurement.
    pub fn start_measurement(&self, i2c: &mut dyn I2cDevice) -> bool {
        i2c.write_register(REG_SYSRANGE_START, 0x01)
    }

    /// Read the range result. Returns None if not ready or failed.
    pub fn read(&mut self, i2c: &mut dyn I2cDevice) -> Option<RangefinderReading> {
        if !self.initialized { return None; }
        i2c.set_address(self.address);

        // Check if measurement complete (bit 0 of status register).
        let mut status = [0u8; 1];
        if !i2c.read_registers(REG_RESULT_RANGE_STATUS, &mut status) {
            return None;
        }
        if status[0] & 0x01 == 0 {
            return None; // Not ready.
        }

        // Read 16-bit range.
        let mut range_buf = [0u8; 2];
        if !i2c.read_registers(REG_RESULT_RANGE_VAL, &mut range_buf) {
            return None;
        }

        let range_mm = u16::from_be_bytes([range_buf[0], range_buf[1]]);

        // Start next measurement.
        let _ = self.start_measurement(i2c);

        let distance_m = range_mm as f32 / 1000.0;

        let range_status = if range_mm > VL53L0X_MAX_MM || range_mm == 8190 {
            // 8190 = "phase out of range" sentinel
            self.valid_count = 0;
            RangefinderStatus::OutOfRangeHigh
        } else if range_mm < VL53L0X_MIN_MM {
            self.valid_count = 0;
            RangefinderStatus::OutOfRangeLow
        } else {
            self.valid_count = self.valid_count.saturating_add(1).min(10);
            RangefinderStatus::Good
        };

        // Signal quality from status register (bits 3:1 = range error code).
        let error_code = (status[0] >> 1) & 0x07;
        let quality = if error_code == 0 { 100 } else if error_code <= 2 { 50 } else { 10 };

        Some(RangefinderReading {
            distance_m,
            status: range_status,
            signal_quality: quality,
            valid_count: self.valid_count,
        })
    }
}

// ---------------------------------------------------------------------------
// VL53L1X
// ---------------------------------------------------------------------------

const VL53L1X_ADDR: u8 = 0x29;

// VL53L1X has a different register layout (16-bit register addresses).
const REG_L1_MODEL_ID: u16 = 0x010F;
const REG_L1_SYSTEM_START: u16 = 0x0087;
const REG_L1_GPIO_HV_MUX_CTRL: u16 = 0x0030;
const REG_L1_RESULT_RANGE_STATUS: u16 = 0x0089;
const REG_L1_RESULT_DISTANCE: u16 = 0x0096;

const VL53L1X_MAX_MM_SHORT: u16 = 1300;
const VL53L1X_MAX_MM_LONG: u16 = 4000;
const VL53L1X_MIN_MM: u16 = 40;

#[derive(Clone, Copy, PartialEq)]
pub enum Vl53l1xMode {
    Short, // 1.3m max, better ambient immunity
    Long,  // 4.0m max, lower accuracy
}

pub struct Vl53l1x {
    address: u8,
    mode: Vl53l1xMode,
    initialized: bool,
    valid_count: u8,
}

impl Vl53l1x {
    pub fn new(mode: Vl53l1xMode) -> Self {
        Self { address: VL53L1X_ADDR, mode, initialized: false, valid_count: 0 }
    }

    fn write_reg16(&self, i2c: &mut dyn I2cDevice, reg: u16, val: u8) -> bool {
        i2c.set_address(self.address);
        let buf = [(reg >> 8) as u8, (reg & 0xFF) as u8, val];
        i2c.write(&buf)
    }

    fn read_reg16(&self, i2c: &mut dyn I2cDevice, reg: u16, out: &mut [u8]) -> bool {
        i2c.set_address(self.address);
        let reg_buf = [(reg >> 8) as u8, (reg & 0xFF) as u8];
        i2c.transfer(&reg_buf, out)
    }

    pub fn init(&mut self, i2c: &mut dyn I2cDevice) -> bool {
        // Check model ID (should be 0xEA for VL53L1X).
        let mut id = [0u8; 1];
        if !self.read_reg16(i2c, REG_L1_MODEL_ID, &mut id) { return false; }
        if id[0] != 0xEA { return false; }

        // Set distance mode via timing budget register (simplified).
        let timing_budget = match self.mode {
            Vl53l1xMode::Short => 0x14, // ~20ms timing budget
            Vl53l1xMode::Long => 0x0A,  // ~50ms timing budget
        };
        if !self.write_reg16(i2c, 0x0060, timing_budget) { return false; }

        // Start continuous ranging.
        if !self.write_reg16(i2c, REG_L1_SYSTEM_START, 0x40) { return false; }

        self.initialized = true;
        true
    }

    pub fn read(&mut self, i2c: &mut dyn I2cDevice) -> Option<RangefinderReading> {
        if !self.initialized { return None; }

        // Check data ready.
        let mut gpio = [0u8; 1];
        if !self.read_reg16(i2c, REG_L1_GPIO_HV_MUX_CTRL, &mut gpio) { return None; }

        // Read range result (2 bytes).
        let mut range_buf = [0u8; 2];
        if !self.read_reg16(i2c, REG_L1_RESULT_DISTANCE, &mut range_buf) { return None; }

        let range_mm = u16::from_be_bytes([range_buf[0], range_buf[1]]);

        // Read range status.
        let mut status_buf = [0u8; 1];
        self.read_reg16(i2c, REG_L1_RESULT_RANGE_STATUS, &mut status_buf);
        let range_error = (status_buf[0] >> 3) & 0x1F;

        // Clear interrupt.
        let _ = self.write_reg16(i2c, 0x0086, 0x01);

        let distance_m = range_mm as f32 / 1000.0;
        let max_mm = match self.mode {
            Vl53l1xMode::Short => VL53L1X_MAX_MM_SHORT,
            Vl53l1xMode::Long => VL53L1X_MAX_MM_LONG,
        };

        let range_status = if range_mm > max_mm || range_error >= 4 {
            self.valid_count = 0;
            RangefinderStatus::OutOfRangeHigh
        } else if range_mm < VL53L1X_MIN_MM {
            self.valid_count = 0;
            RangefinderStatus::OutOfRangeLow
        } else {
            self.valid_count = self.valid_count.saturating_add(1).min(10);
            RangefinderStatus::Good
        };

        let quality = match range_error {
            0..=1 => 100,
            2..=3 => 60,
            _ => 10,
        };

        Some(RangefinderReading {
            distance_m,
            status: range_status,
            signal_quality: quality,
            valid_count: self.valid_count,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_vl53l0x_range_conversion() {
        let range_mm: u16 = 750;
        let distance_m = range_mm as f32 / 1000.0;
        assert!((distance_m - 0.75).abs() < 0.001);
    }

    #[test]
    fn test_vl53l0x_sentinel() {
        // 8190mm is the "phase out of range" sentinel.
        assert!(8190 > VL53L0X_MAX_MM);
    }

    #[test]
    fn test_vl53l1x_modes() {
        assert_eq!(VL53L1X_MAX_MM_SHORT, 1300);
        assert_eq!(VL53L1X_MAX_MM_LONG, 4000);
    }
}

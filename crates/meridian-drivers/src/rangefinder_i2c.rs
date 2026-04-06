//! I2C rangefinder drivers — Garmin LIDAR-Lite, MaxBotix, VL53L0X, etc.
//!
//! ArduPilot has 47 rangefinder types. This module adds I2C-based sensors
//! that are the most commonly used in the ecosystem.

use meridian_hal::I2cDevice;
use crate::rangefinder::{RangefinderConfig, RangefinderReading, RangefinderStatus, validate_distance};

// ---------------------------------------------------------------------------
// Garmin LIDAR-Lite v3/v4 (I2C address 0x62)
// ---------------------------------------------------------------------------

const GARMIN_DEFAULT_ADDR: u8 = 0x62;
const GARMIN_REG_ACQ_COMMAND: u8 = 0x00;
const GARMIN_REG_STATUS: u8 = 0x01;
const GARMIN_REG_FULL_DELAY_HIGH: u8 = 0x0F;
const GARMIN_REG_FULL_DELAY_LOW: u8 = 0x10;

/// Garmin LIDAR-Lite I2C driver.
///
/// ArduPilot reference: `AP_RangeFinder_LidarLightI2C.cpp`
pub struct GarminLiteI2c {
    address: u8,
    config: RangefinderConfig,
    state: GarminState,
    last_distance_m: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum GarminState {
    Idle,
    WaitingForMeasurement,
}

impl GarminLiteI2c {
    pub fn new(config: RangefinderConfig) -> Self {
        Self {
            address: GARMIN_DEFAULT_ADDR,
            config,
            state: GarminState::Idle,
            last_distance_m: 0.0,
        }
    }

    pub fn with_address(mut self, addr: u8) -> Self {
        self.address = addr;
        self
    }

    /// Start a measurement. Call `read_result()` after ~20 ms.
    pub fn trigger_measurement(&mut self, i2c: &mut dyn I2cDevice) -> bool {
        i2c.set_address(self.address);
        // Write 0x04 to ACQ_COMMAND to take a measurement with bias correction
        if i2c.write_register(GARMIN_REG_ACQ_COMMAND, 0x04) {
            self.state = GarminState::WaitingForMeasurement;
            true
        } else {
            false
        }
    }

    /// Read the measurement result. Returns None if not ready.
    pub fn read_result(&mut self, i2c: &mut dyn I2cDevice) -> Option<RangefinderReading> {
        if self.state != GarminState::WaitingForMeasurement {
            return None;
        }
        i2c.set_address(self.address);

        // Check status register — bit 0 = busy
        let status = i2c.read_register(GARMIN_REG_STATUS)?;
        if status & 0x01 != 0 {
            return None; // still measuring
        }

        // Read 2-byte distance (cm)
        let high = i2c.read_register(GARMIN_REG_FULL_DELAY_HIGH)?;
        let low = i2c.read_register(GARMIN_REG_FULL_DELAY_LOW)?;
        let distance_cm = ((high as u16) << 8) | (low as u16);
        self.last_distance_m = distance_cm as f32 / 100.0;
        self.state = GarminState::Idle;

        let status = validate_distance(self.last_distance_m, &self.config);
        Some(RangefinderReading {
            distance_m: self.last_distance_m,
            status,
            signal_quality: -1,
            valid_count: 0,
        })
    }
}

// ---------------------------------------------------------------------------
// MaxBotix MaxSonar I2C XL (I2C address 0x70)
// ---------------------------------------------------------------------------

const MAXBOTIX_DEFAULT_ADDR: u8 = 0x70;
const MAXBOTIX_CMD_RANGE: u8 = 0x51;

/// MaxBotix MaxSonar I2C XL ultrasonic rangefinder.
///
/// ArduPilot reference: `AP_RangeFinder_MaxsonarI2CXL.cpp`
pub struct MaxBotixI2c {
    address: u8,
    config: RangefinderConfig,
    last_distance_m: f32,
}

impl MaxBotixI2c {
    pub fn new(config: RangefinderConfig) -> Self {
        Self {
            address: MAXBOTIX_DEFAULT_ADDR,
            config,
            last_distance_m: 0.0,
        }
    }

    pub fn with_address(mut self, addr: u8) -> Self {
        self.address = addr;
        self
    }

    /// Trigger a range reading. Call `read_result()` after ~100 ms.
    pub fn trigger(&self, i2c: &mut dyn I2cDevice) -> bool {
        i2c.set_address(self.address);
        i2c.write(&[MAXBOTIX_CMD_RANGE])
    }

    /// Read the range result. Returns distance in cm as 2-byte BE.
    pub fn read_result(&mut self, i2c: &mut dyn I2cDevice) -> Option<RangefinderReading> {
        i2c.set_address(self.address);
        let mut buf = [0u8; 2];
        if !i2c.transfer(&[], &mut buf) {
            return None;
        }
        let distance_cm = ((buf[0] as u16) << 8) | (buf[1] as u16);
        self.last_distance_m = distance_cm as f32 / 100.0;

        let status = validate_distance(self.last_distance_m, &self.config);
        Some(RangefinderReading {
            distance_m: self.last_distance_m,
            status,
            signal_quality: -1,
            valid_count: 0,
        })
    }
}

// ---------------------------------------------------------------------------
// VL53L0X / VL53L1X (ST ToF sensor, I2C address 0x29)
// ---------------------------------------------------------------------------

const VL53L0X_DEFAULT_ADDR: u8 = 0x29;

/// VL53L0X/VL53L1X time-of-flight rangefinder stub.
///
/// ArduPilot reference: `AP_RangeFinder_VL53L0X.cpp`, `AP_RangeFinder_VL53L1X.cpp`
///
/// These sensors require a complex multi-step init sequence (firmware loading,
/// timing calibration, etc.). This is a structural stub that handles the
/// I2C transaction format.
pub struct Vl53l0xI2c {
    address: u8,
    config: RangefinderConfig,
    last_distance_m: f32,
    initialised: bool,
}

impl Vl53l0xI2c {
    pub fn new(config: RangefinderConfig) -> Self {
        Self {
            address: VL53L0X_DEFAULT_ADDR,
            config,
            last_distance_m: 0.0,
            initialised: false,
        }
    }

    /// Probe: check if the device responds.
    pub fn probe(&self, i2c: &mut dyn I2cDevice) -> bool {
        i2c.set_address(self.address);
        i2c.probe()
    }

    /// Initialise the sensor. Stub — full init requires firmware tables.
    pub fn init(&mut self, i2c: &mut dyn I2cDevice) -> bool {
        i2c.set_address(self.address);
        // Check model ID register (0xC0) — should read 0xEE for VL53L0X
        if let Some(id) = i2c.read_register(0xC0) {
            if id == 0xEE || id == 0xEA { // VL53L0X or VL53L1X
                self.initialised = true;
                return true;
            }
        }
        false
    }

    /// Read distance. Stub — returns from data-ready register.
    pub fn read(&mut self, i2c: &mut dyn I2cDevice) -> Option<RangefinderReading> {
        if !self.initialised {
            return None;
        }
        i2c.set_address(self.address);
        // Read result register (simplified — real driver uses a state machine)
        let mut buf = [0u8; 2];
        if !i2c.read_registers(0x14, &mut buf) { // RESULT_RANGE_VALUE for VL53L0X
            return None;
        }
        let distance_mm = u16::from_be_bytes(buf);
        self.last_distance_m = distance_mm as f32 / 1000.0;

        let status = validate_distance(self.last_distance_m, &self.config);
        Some(RangefinderReading {
            distance_m: self.last_distance_m,
            status,
            signal_quality: -1,
            valid_count: 0,
        })
    }
}

// ---------------------------------------------------------------------------
// TeraRanger I2C (address 0x30)
// ---------------------------------------------------------------------------

const TERARANGER_ADDR: u8 = 0x30;

/// TeraRanger I2C rangefinder stub.
pub struct TeraRangerI2c {
    config: RangefinderConfig,
    last_distance_m: f32,
}

impl TeraRangerI2c {
    pub fn new(config: RangefinderConfig) -> Self {
        Self { config, last_distance_m: 0.0 }
    }

    pub fn read(&mut self, i2c: &mut dyn I2cDevice) -> Option<RangefinderReading> {
        i2c.set_address(TERARANGER_ADDR);
        let mut buf = [0u8; 3]; // 2 bytes distance + 1 CRC
        if !i2c.transfer(&[], &mut buf) {
            return None;
        }
        let distance_mm = u16::from_be_bytes([buf[0], buf[1]]);
        self.last_distance_m = distance_mm as f32 / 1000.0;

        let status = validate_distance(self.last_distance_m, &self.config);
        Some(RangefinderReading {
            distance_m: self.last_distance_m,
            status,
            signal_quality: -1,
            valid_count: 0,
        })
    }
}

// ---------------------------------------------------------------------------
// TOFSenseF I2C (address 0x08)
// ---------------------------------------------------------------------------

/// TOFSenseF I2C rangefinder stub.
pub struct TofSenseFI2c {
    config: RangefinderConfig,
}

impl TofSenseFI2c {
    pub fn new(config: RangefinderConfig) -> Self {
        Self { config }
    }

    pub fn read(&mut self, i2c: &mut dyn I2cDevice) -> Option<RangefinderReading> {
        i2c.set_address(0x08);
        let mut buf = [0u8; 4];
        if !i2c.transfer(&[0x00], &mut buf) {
            return None;
        }
        let distance_mm = u16::from_le_bytes([buf[0], buf[1]]);
        let distance_m = distance_mm as f32 / 1000.0;

        let status = validate_distance(distance_m, &self.config);
        Some(RangefinderReading {
            distance_m,
            status,
            signal_quality: -1,
            valid_count: 0,
        })
    }
}

// ---------------------------------------------------------------------------
// Serial rangefinder stubs (top 10 missing, per the report)
// ---------------------------------------------------------------------------

/// LeddarOne serial rangefinder stub.
pub struct LeddarOneParser {
    config: RangefinderConfig,
}

impl LeddarOneParser {
    pub fn new(config: RangefinderConfig) -> Self {
        Self { config }
    }
}

/// USD1 serial/CAN rangefinder stub.
pub struct Usd1Parser {
    config: RangefinderConfig,
}

impl Usd1Parser {
    pub fn new(config: RangefinderConfig) -> Self {
        Self { config }
    }
}

/// Ainstein LR-D1 serial rangefinder stub.
pub struct AinsteinLrd1Parser {
    config: RangefinderConfig,
}

impl AinsteinLrd1Parser {
    pub fn new(config: RangefinderConfig) -> Self {
        Self { config }
    }
}

/// NRA24 CAN radar rangefinder stub.
pub struct Nra24Parser {
    config: RangefinderConfig,
}

impl Nra24Parser {
    pub fn new(config: RangefinderConfig) -> Self {
        Self { config }
    }
}

/// Analog (voltage-based) rangefinder.
pub struct AnalogRangefinder {
    config: RangefinderConfig,
    voltage_scale: f32, // V per meter
}

impl AnalogRangefinder {
    pub fn new(config: RangefinderConfig, voltage_scale: f32) -> Self {
        Self { config, voltage_scale }
    }

    pub fn process_voltage(&self, voltage: f32) -> RangefinderReading {
        let distance_m = voltage / self.voltage_scale;
        RangefinderReading {
            distance_m,
            status: validate_distance(distance_m, &self.config),
            signal_quality: -1,
            valid_count: 0,
        }
    }
}

/// PWM (pulse-width) rangefinder.
pub struct PwmRangefinder {
    config: RangefinderConfig,
    us_per_meter: f32,
}

impl PwmRangefinder {
    pub fn new(config: RangefinderConfig, us_per_meter: f32) -> Self {
        Self { config, us_per_meter }
    }

    pub fn process_pulse(&self, pulse_width_us: u32) -> RangefinderReading {
        let distance_m = pulse_width_us as f32 / self.us_per_meter;
        RangefinderReading {
            distance_m,
            status: validate_distance(distance_m, &self.config),
            signal_quality: -1,
            valid_count: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_analog_rangefinder() {
        let rf = AnalogRangefinder::new(RangefinderConfig::default(), 0.1); // 0.1V/m
        let reading = rf.process_voltage(0.5);
        assert!((reading.distance_m - 5.0).abs() < 0.01);
        assert_eq!(reading.status, RangefinderStatus::Good);
    }

    #[test]
    fn test_pwm_rangefinder() {
        // HC-SR04: 58 us per cm = 5800 us per meter
        let rf = PwmRangefinder::new(RangefinderConfig::default(), 5800.0);
        let reading = rf.process_pulse(5800);
        assert!((reading.distance_m - 1.0).abs() < 0.01, "distance = {}", reading.distance_m);
    }
}

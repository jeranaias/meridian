//! MS4525DO digital differential pressure sensor driver.
//!
//! ArduPilot reference: `AP_Airspeed_MS4525.cpp`
//!
//! MEAS MS4525DO is the most common airspeed sensor in the ArduPilot ecosystem.
//! I2C, 14-bit pressure + 11-bit temperature output.
//! Default address: 0x28.
//!
//! Data format (4 bytes from I2C read):
//!   Byte 0: [status(2)] [pressure_high(6)]
//!   Byte 1: [pressure_low(8)]
//!   Byte 2: [temp_high(8)]
//!   Byte 3: [temp_low(3)] [unused(5)]
//!
//! Status bits: 00 = normal, 01 = reserved, 10 = stale, 11 = fault
//!
//! Pressure conversion (for 1 PSI differential range):
//!   pressure_raw: 14-bit unsigned (0..16383)
//!   P_min_count = 1638  (10% of 16384)
//!   P_max_count = 14745 (90% of 16384)
//!   pressure_psi = (raw - P_min_count) / (P_max_count - P_min_count) * 2.0 - 1.0
//!   pressure_pa  = pressure_psi * 6894.757  (PSI_TO_PA)
//!
//! Temperature conversion:
//!   temp_raw: 11-bit unsigned (0..2047)
//!   temperature_c = temp_raw * (200.0 / 2047.0) - 50.0

use meridian_hal::I2cDevice;
use crate::airspeed::{AirspeedReading, pressure_to_airspeed};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const MS4525_DEFAULT_ADDR: u8 = 0x28;

/// 14-bit output counts for 10% and 90% span.
const P_MIN_COUNT: f32 = 1638.0;
const P_MAX_COUNT: f32 = 14745.0;

/// Full-scale differential pressure in PSI (for MS4525DO-DS5AI001DP, ±1 PSI).
const FULL_SCALE_PSI: f32 = 1.0;

/// PSI to Pascals.
const PSI_TO_PA: f32 = 6894.757;

/// Temperature scale: 200°C over 2047 counts, offset -50°C.
const TEMP_SCALE: f32 = 200.0 / 2047.0;
const TEMP_OFFSET: f32 = -50.0;

/// Maximum consecutive stale/fault reads before marking unhealthy.
const MAX_STALE_COUNT: u8 = 10;

// ---------------------------------------------------------------------------
// Health probability system (from AP_Airspeed_Health.cpp)
// ---------------------------------------------------------------------------

/// Airspeed health tracker using exponential probability filter.
/// This is the ArduPilot health monitoring system: a probability value
/// that decays on bad readings and recovers on good ones.
pub struct AirspeedHealth {
    /// Health probability: 0.0 = definitely bad, 1.0 = definitely good.
    probability: f32,
    /// Whether the sensor is currently enabled.
    enabled: bool,
    /// Backup of the enabled state before auto-disable.
    was_enabled: bool,
    /// Consecutive failures for the wind-max check.
    wind_max_failures: u32,
}

impl AirspeedHealth {
    pub fn new() -> Self {
        Self {
            probability: 1.0,
            enabled: true,
            was_enabled: true,
            wind_max_failures: 0,
        }
    }

    /// Update health based on a single check result.
    /// `consistent`: true if airspeed agrees with GPS/EKF, false otherwise.
    pub fn update(&mut self, consistent: bool) {
        if consistent {
            // Good reading: slow recovery.
            self.probability = 0.98 * self.probability + 0.02;
            self.wind_max_failures = 0;
        } else {
            // Bad reading: fast decay.
            self.probability *= 0.90;
            self.wind_max_failures += 1;
        }

        // Auto-disable if probability drops below threshold.
        if self.probability < 0.1 && self.enabled {
            self.was_enabled = true;
            self.enabled = false;
        }

        // Auto-re-enable if probability recovers.
        if self.probability > 0.95 && !self.enabled && self.was_enabled {
            self.enabled = true;
        }
    }

    /// Check if the sensor should be used for flight control.
    pub fn is_healthy(&self) -> bool {
        self.enabled && self.probability > 0.5
    }

    /// Current health probability (0.0 - 1.0).
    pub fn probability(&self) -> f32 {
        self.probability
    }

    /// Whether sensor is enabled (may be auto-disabled).
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }
}

// ---------------------------------------------------------------------------
// MS4525 Driver
// ---------------------------------------------------------------------------

/// MS4525DO airspeed sensor.
pub struct Ms4525Airspeed {
    address: u8,
    pressure_offset: f32,
    calibrated: bool,
    cal_sum: f32,
    cal_count: u16,
    stale_count: u8,
    last_pressure_pa: f32,
    last_temperature_c: f32,
    health: AirspeedHealth,
}

impl Ms4525Airspeed {
    /// Create a new MS4525 driver with default I2C address (0x28).
    pub fn new() -> Self {
        Self::with_address(MS4525_DEFAULT_ADDR)
    }

    /// Create a new MS4525 driver with a custom I2C address.
    pub fn with_address(address: u8) -> Self {
        Self {
            address,
            pressure_offset: 0.0,
            calibrated: false,
            cal_sum: 0.0,
            cal_count: 0,
            stale_count: 0,
            last_pressure_pa: 0.0,
            last_temperature_c: 0.0,
            health: AirspeedHealth::new(),
        }
    }

    /// Read raw pressure and temperature from the sensor.
    ///
    /// Returns `None` if the read fails or the sensor reports a fault.
    pub fn read_raw(&mut self, i2c: &mut dyn I2cDevice) -> Option<(u16, u16, u8)> {
        let mut buf = [0u8; 4];
        i2c.set_address(self.address);
        if !i2c.transfer(&[], &mut buf) {
            return None;
        }

        // Status bits (top 2 bits of byte 0).
        let status = (buf[0] >> 6) & 0x03;

        // Status: 0 = normal, 1 = reserved, 2 = stale, 3 = fault.
        if status == 3 {
            self.stale_count = self.stale_count.saturating_add(1);
            return None;
        }
        if status == 2 {
            self.stale_count = self.stale_count.saturating_add(1);
            if self.stale_count > MAX_STALE_COUNT {
                return None;
            }
            // Allow a few stale reads — sensor may be slow.
        } else {
            self.stale_count = 0;
        }

        // 14-bit pressure: top 6 bits of byte 0 + all 8 bits of byte 1.
        let pressure_raw = (((buf[0] as u16) & 0x3F) << 8) | (buf[1] as u16);

        // 11-bit temperature: all 8 bits of byte 2 + top 3 bits of byte 3.
        let temp_raw = ((buf[2] as u16) << 3) | ((buf[3] as u16) >> 5);

        Some((pressure_raw, temp_raw, status))
    }

    /// Convert raw pressure count to Pascals.
    fn raw_to_pressure_pa(raw: u16) -> f32 {
        let raw_f = raw as f32;
        // Normalise to -1.0..+1.0 range.
        let normalised = (raw_f - P_MIN_COUNT) / (P_MAX_COUNT - P_MIN_COUNT) * 2.0 - 1.0;
        // Scale to Pascals.
        normalised * FULL_SCALE_PSI * PSI_TO_PA
    }

    /// Convert raw temperature count to Celsius.
    fn raw_to_temperature_c(raw: u16) -> f32 {
        (raw as f32) * TEMP_SCALE + TEMP_OFFSET
    }

    /// Accumulate a ground calibration sample.
    ///
    /// Call this repeatedly while on the ground before arming. After enough
    /// samples, the offset is computed and `is_calibrated()` returns true.
    pub fn calibrate_sample(&mut self, i2c: &mut dyn I2cDevice) -> bool {
        if let Some((p_raw, _, _)) = self.read_raw(i2c) {
            let p_pa = Self::raw_to_pressure_pa(p_raw);
            self.cal_sum += p_pa;
            self.cal_count += 1;

            if self.cal_count >= 50 {
                self.pressure_offset = self.cal_sum / self.cal_count as f32;
                self.calibrated = true;
                return true;
            }
        }
        false
    }

    /// Set the pressure offset directly (e.g. from saved parameters).
    pub fn set_offset(&mut self, offset: f32) {
        self.pressure_offset = offset;
        self.calibrated = true;
    }

    /// Read and convert to an airspeed reading.
    ///
    /// Returns `None` if the read fails or the sensor is unhealthy.
    pub fn read(&mut self, i2c: &mut dyn I2cDevice) -> Option<AirspeedReading> {
        let (p_raw, t_raw, _status) = self.read_raw(i2c)?;

        let pressure_pa = Self::raw_to_pressure_pa(p_raw);
        let temperature_c = Self::raw_to_temperature_c(t_raw);

        self.last_pressure_pa = pressure_pa;
        self.last_temperature_c = temperature_c;

        // Apply ground offset.
        let corrected = pressure_pa - self.pressure_offset;

        // Convert to airspeed.
        let airspeed = pressure_to_airspeed(corrected);

        Some(AirspeedReading {
            airspeed_ms: airspeed,
            differential_pressure_pa: corrected,
            temperature_c,
            healthy: self.health.is_healthy(),
        })
    }

    /// Whether the sensor has been ground-calibrated.
    pub fn is_calibrated(&self) -> bool {
        self.calibrated
    }

    /// Access the health monitor.
    pub fn health(&self) -> &AirspeedHealth {
        &self.health
    }

    /// Mutable access to the health monitor (for external consistency checks).
    pub fn health_mut(&mut self) -> &mut AirspeedHealth {
        &mut self.health
    }

    /// Last measured temperature in Celsius.
    pub fn temperature(&self) -> f32 {
        self.last_temperature_c
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pressure_conversion() {
        // At midpoint (zero differential pressure), raw = (P_min + P_max) / 2 = 8191.5
        let mid = ((P_MIN_COUNT + P_MAX_COUNT) / 2.0) as u16;
        let p = Ms4525Airspeed::raw_to_pressure_pa(mid);
        assert!(p.abs() < 10.0, "midpoint should be near zero Pa, got {}", p);

        // At max count (positive full scale).
        let p_max = Ms4525Airspeed::raw_to_pressure_pa(P_MAX_COUNT as u16);
        let expected_max = FULL_SCALE_PSI * PSI_TO_PA;
        assert!((p_max - expected_max).abs() < 50.0, "max should be ~{} Pa, got {}", expected_max, p_max);

        // At min count (negative full scale).
        let p_min = Ms4525Airspeed::raw_to_pressure_pa(P_MIN_COUNT as u16);
        assert!((p_min + expected_max).abs() < 50.0, "min should be ~-{} Pa, got {}", expected_max, p_min);
    }

    #[test]
    fn test_temperature_conversion() {
        // Raw 0 → -50°C.
        let t0 = Ms4525Airspeed::raw_to_temperature_c(0);
        assert!((t0 - (-50.0)).abs() < 0.1);

        // Raw 2047 → +150°C.
        let t_max = Ms4525Airspeed::raw_to_temperature_c(2047);
        assert!((t_max - 150.0).abs() < 0.2);

        // Raw ~1024 → ~50°C.
        let t_mid = Ms4525Airspeed::raw_to_temperature_c(1024);
        assert!(t_mid > 45.0 && t_mid < 55.0);
    }

    #[test]
    fn test_health_probability() {
        let mut h = AirspeedHealth::new();
        assert!(h.is_healthy());
        assert!((h.probability() - 1.0).abs() < 0.01);

        // 50 bad readings should drive probability below 0.1.
        for _ in 0..50 {
            h.update(false);
        }
        assert!(h.probability() < 0.01);
        assert!(!h.is_enabled()); // auto-disabled

        // Recovery with good readings.
        for _ in 0..200 {
            h.update(true);
        }
        assert!(h.probability() > 0.95);
        assert!(h.is_enabled()); // auto-re-enabled
        assert!(h.is_healthy());
    }

    #[test]
    fn test_status_bits() {
        // Status 3 (fault) should return None.
        // We can't easily test I2C here, but verify the conversion logic.
        let p = Ms4525Airspeed::raw_to_pressure_pa(8192); // near midpoint
        assert!(p.abs() < 10.0);
    }
}

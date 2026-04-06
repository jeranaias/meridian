//! Airspeed sensor drivers — differential pressure → indicated airspeed.
//!
//! Source: ArduPilot AP_Airspeed + docs/AUDIT_SENSOR_DRIVERS.md
//! 19 sensor types in ArduPilot. We implement the most common: MS4525DO, analog, synthetic.

/// Airspeed reading.
#[derive(Debug, Clone, Copy)]
pub struct AirspeedReading {
    /// Differential pressure in Pascals.
    pub differential_pressure_pa: f32,
    /// Temperature in Celsius.
    pub temperature_c: f32,
    /// Indicated airspeed in m/s.
    pub airspeed_ms: f32,
    /// Whether the reading is healthy.
    pub healthy: bool,
}

/// Convert differential pressure to airspeed (Bernoulli equation).
/// IAS = sqrt(2 * |dP| / rho_0) where rho_0 = 1.225 kg/m³ (sea level density)
pub fn pressure_to_airspeed(dp_pa: f32) -> f32 {
    const RHO_0: f32 = 1.225;
    let abs_dp = libm::fabsf(dp_pa);
    let speed = libm::sqrtf(2.0 * abs_dp / RHO_0);
    if dp_pa < 0.0 { -speed } else { speed }
}

/// MS4525DO differential pressure sensor (I2C).
/// Most common airspeed sensor in the ArduPilot ecosystem.
pub struct Ms4525do {
    address: u8,
    pressure_offset: f32,
    calibrated: bool,
    cal_sum: f32,
    cal_count: u16,
}

impl Ms4525do {
    pub fn new(address: u8) -> Self {
        Self {
            address,
            pressure_offset: 0.0,
            calibrated: false,
            cal_sum: 0.0,
            cal_count: 0,
        }
    }

    /// Parse a 4-byte I2C read from the MS4525DO.
    /// Returns (status, pressure_raw, temperature_raw).
    /// Status: 0=normal, 1=command mode, 2=stale, 3=diagnostic
    pub fn parse_raw(data: &[u8; 4]) -> (u8, u16, u16) {
        let status = (data[0] >> 6) & 0x03;
        let pressure_raw = ((data[0] as u16 & 0x3F) << 8) | (data[1] as u16);
        let temperature_raw = ((data[2] as u16) << 3) | ((data[3] as u16) >> 5);
        (status, pressure_raw, temperature_raw)
    }

    /// Convert raw values to engineering units.
    /// Pressure: 14-bit, range 0-16383, maps to -1 to +1 PSI (6894.76 Pa/PSI)
    /// Temperature: 11-bit, range 0-2047, maps to -50 to +150°C
    pub fn convert(pressure_raw: u16, temperature_raw: u16) -> (f32, f32) {
        // Pressure: centered at 8192, 1 PSI = 6894.76 Pa
        let dp_psi = (pressure_raw as f32 - 8192.0) / 8192.0; // -1..+1 PSI
        let dp_pa = dp_psi * 6894.76;

        // Temperature: 0 → -50°C, 2047 → 150°C
        let temp_c = temperature_raw as f32 * (200.0 / 2047.0) - 50.0;

        (dp_pa, temp_c)
    }

    /// Process a reading. Handles ground zero calibration.
    pub fn process(&mut self, data: &[u8; 4]) -> Option<AirspeedReading> {
        let (status, praw, traw) = Self::parse_raw(data);
        if status >= 2 { return None; } // stale or diagnostic

        let (dp_pa, temp_c) = Self::convert(praw, traw);

        // Ground calibration: average first 50 samples to find zero offset
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

    pub fn is_calibrated(&self) -> bool { self.calibrated }
}

/// Analog airspeed sensor — reads voltage from ADC.
pub struct AnalogAirspeed {
    pin: u8,
    voltage_offset: f32,
    voltage_scale: f32, // V → Pa conversion factor
    calibrated: bool,
    cal_sum: f32,
    cal_count: u16,
}

impl AnalogAirspeed {
    pub fn new(pin: u8) -> Self {
        Self {
            pin,
            voltage_offset: 0.0,
            voltage_scale: 1000.0, // typical: 1V = 1000 Pa
            calibrated: false,
            cal_sum: 0.0,
            cal_count: 0,
        }
    }

    pub fn process(&mut self, voltage: f32) -> AirspeedReading {
        if !self.calibrated {
            self.cal_sum += voltage;
            self.cal_count += 1;
            if self.cal_count >= 50 {
                self.voltage_offset = self.cal_sum / self.cal_count as f32;
                self.calibrated = true;
            }
            return AirspeedReading {
                differential_pressure_pa: 0.0, temperature_c: 25.0,
                airspeed_ms: 0.0, healthy: false,
            };
        }

        let dp_pa = (voltage - self.voltage_offset) * self.voltage_scale;
        let airspeed = pressure_to_airspeed(dp_pa);
        AirspeedReading {
            differential_pressure_pa: dp_pa, temperature_c: 25.0,
            airspeed_ms: airspeed, healthy: true,
        }
    }
}

/// Synthetic airspeed — estimated from GPS groundspeed + wind estimate.
/// No sensor needed. Used as fallback when no airspeed sensor is installed.
pub fn synthetic_airspeed(groundspeed: f32, wind_speed: f32, wind_dir_rad: f32, heading_rad: f32) -> f32 {
    // Airspeed ≈ groundspeed - wind component along heading
    let headwind = wind_speed * libm::cosf(wind_dir_rad - heading_rad);
    (groundspeed + headwind).max(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pressure_to_airspeed() {
        // At 100 Pa differential: sqrt(2*100/1.225) ≈ 12.78 m/s
        let speed = pressure_to_airspeed(100.0);
        assert!((speed - 12.78).abs() < 0.1, "100Pa → {} m/s", speed);
    }

    #[test]
    fn test_pressure_to_airspeed_zero() {
        assert_eq!(pressure_to_airspeed(0.0), 0.0);
    }

    #[test]
    fn test_ms4525_parse_raw() {
        // Status=0 (normal), pressure=8192 (zero), temp=1024 (mid)
        let data = [0x20, 0x00, 0x80, 0x00]; // 0b00_100000, 0x00, 0b10000000, 0x00
        let (status, praw, traw) = Ms4525do::parse_raw(&data);
        assert_eq!(status, 0);
        assert_eq!(praw, 0x2000);
        assert_eq!(traw, 0x0400);
    }

    #[test]
    fn test_ms4525_convert_zero_pressure() {
        let (dp, _temp) = Ms4525do::convert(8192, 1024);
        assert!((dp - 0.0).abs() < 1.0, "Center value should be ~0 Pa: {}", dp);
    }

    #[test]
    fn test_ms4525_calibration() {
        let mut sensor = Ms4525do::new(0x28);
        assert!(!sensor.is_calibrated());

        // Feed 50 samples at zero pressure (8192 raw)
        for _ in 0..50 {
            let data = [0x20, 0x00, 0x80, 0x00];
            sensor.process(&data);
        }
        assert!(sensor.is_calibrated());
    }

    #[test]
    fn test_synthetic_airspeed_no_wind() {
        let as_ms = synthetic_airspeed(20.0, 0.0, 0.0, 0.0);
        assert!((as_ms - 20.0).abs() < 0.1);
    }

    #[test]
    fn test_synthetic_airspeed_headwind() {
        // 20 m/s groundspeed, 5 m/s headwind (wind from same direction as heading)
        let as_ms = synthetic_airspeed(20.0, 5.0, 0.0, 0.0);
        assert!((as_ms - 25.0).abs() < 0.1, "Headwind should increase airspeed: {}", as_ms);
    }

    #[test]
    fn test_synthetic_airspeed_tailwind() {
        // 20 m/s groundspeed, 5 m/s tailwind (wind from behind)
        let as_ms = synthetic_airspeed(20.0, 5.0, core::f32::consts::PI, 0.0);
        assert!((as_ms - 15.0).abs() < 0.1, "Tailwind should decrease airspeed: {}", as_ms);
    }
}

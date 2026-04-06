//! DPS310 barometer driver — high-accuracy pressure/temperature sensor.
//!
//! Source: ArduPilot AP_Baro_DPS310.cpp + Infineon DPS310 datasheet
//! Interface: I2C (0x76 or 0x77) or SPI
//! Used on: 50+ boards, most popular baro on modern H7 FCs

use meridian_math::Vec3;
use meridian_math::frames::Body;

/// DPS310 registers.
const REG_PRS_B2: u8 = 0x00;
const REG_PRS_B1: u8 = 0x01;
const REG_PRS_B0: u8 = 0x02;
const REG_TMP_B2: u8 = 0x03;
const REG_TMP_B1: u8 = 0x04;
const REG_TMP_B0: u8 = 0x05;
const REG_PRS_CFG: u8 = 0x06;
const REG_TMP_CFG: u8 = 0x07;
const REG_MEAS_CFG: u8 = 0x08;
const REG_CFG_REG: u8 = 0x09;
const REG_PRODUCT_ID: u8 = 0x0D;
const REG_COEF: u8 = 0x10;
const REG_COEF_SRCE: u8 = 0x28;

const DPS310_PRODUCT_ID: u8 = 0x10;

/// CFG_REG value for 16x oversampling: P_SHIFT (bit 2) + T_SHIFT (bit 3) = 0x0C.
/// Must be written after PRS_CFG and TMP_CFG when OSR > 8x.
/// Source: DPS310 datasheet section 7, "Result bit-shifting"
const DPS310_CFG_REG_SHIFT: u8 = 0x0C;

/// Barometer reading.
#[derive(Debug, Clone, Copy)]
pub struct BaroReading {
    pub pressure_pa: f32,
    pub temperature_c: f32,
    pub altitude_m: f32,
}

/// DPS310 calibration coefficients (11 values, float).
#[derive(Debug, Clone, Copy, Default)]
struct Dps310Cal {
    c0: f32,
    c1: f32,
    c00: f32,
    c10: f32,
    c01: f32,
    c11: f32,
    c20: f32,
    c21: f32,
    c30: f32,
}

/// Oversampling rate to scale factor.
fn scale_factor(osr: u8) -> f32 {
    match osr {
        0 => 524288.0,
        1 => 1572864.0,
        2 => 3670016.0,
        3 => 7864320.0,
        4 => 253952.0,
        5 => 516096.0,
        6 => 1040384.0,
        7 => 2088960.0,
        _ => 524288.0,
    }
}

/// DPS310 barometer driver.
pub struct Dps310 {
    cal: Dps310Cal,
    ground_pressure: f32,
    calibrated: bool,
    cal_sum: f32,
    cal_count: u16,
    last_pressure: f32,
    last_temperature: f32,
    prs_scale: f32,
    tmp_scale: f32,
}

impl Dps310 {
    pub fn new() -> Self {
        Self {
            cal: Dps310Cal::default(),
            ground_pressure: 101325.0,
            calibrated: false,
            cal_sum: 0.0,
            cal_count: 0,
            last_pressure: 101325.0,
            last_temperature: 25.0,
            prs_scale: scale_factor(4), // 16x oversampling to match ArduPilot's default config
            tmp_scale: scale_factor(4),
        }
    }

    /// Check if a device at the given address is a DPS310.
    /// Product ID register (0x0D): upper nibble [7:4] = product ID (0x01 for DPS310),
    /// lower nibble [3:0] = revision ID.
    pub fn probe_i2c(i2c: &mut dyn I2cProbe, addr: u8) -> bool {
        if let Some(id) = i2c.read_register_at(addr, REG_PRODUCT_ID) {
            (id >> 4) == 0x01
        } else {
            false
        }
    }

    /// Return the register writes needed to configure the DPS310 for 16x OSR
    /// continuous pressure + temperature measurement. The caller must write each
    /// (register, value) pair via I2C/SPI after probe succeeds.
    ///
    /// Sequence:
    ///   1. PRS_CFG  (0x06) = 0x24  — 4 meas/s, 16x oversampling
    ///   2. TMP_CFG  (0x07) = 0xA4  — 4 meas/s, 16x oversampling, ext sensor
    ///   3. CFG_REG  (0x09) = 0x0C  — P_SHIFT + T_SHIFT (required at >8x OSR)
    ///   4. MEAS_CFG (0x08) = 0x07  — continuous pressure + temperature
    pub fn init_register_writes() -> [(u8, u8); 4] {
        [
            (REG_PRS_CFG,  0x24), // rate=4, osr=16x
            (REG_TMP_CFG,  0xA4), // rate=4, osr=16x, external sensor
            (REG_CFG_REG,  DPS310_CFG_REG_SHIFT), // P_SHIFT | T_SHIFT
            (REG_MEAS_CFG, 0x07), // continuous P+T
        ]
    }

    /// Parse calibration coefficients from 18 raw bytes (registers 0x10-0x21).
    pub fn parse_calibration(&mut self, coef: &[u8; 18], coef_src: u8) {
        // c0: bits [19:12] from coef[0], [11:4] from coef[1]
        let c0_raw = ((coef[0] as i16) << 4) | ((coef[1] as i16) >> 4);
        let c0_raw = if c0_raw & 0x800 != 0 { c0_raw | !0xFFF_i16 } else { c0_raw };

        // c1: bits [11:8] from coef[1], [7:0] from coef[2]
        let c1_raw = (((coef[1] & 0x0F) as i16) << 8) | (coef[2] as i16);
        let c1_raw = if c1_raw & 0x800 != 0 { c1_raw | !0xFFF_i16 } else { c1_raw };

        // c00: 20-bit from coef[3..6]
        let c00_raw = ((coef[3] as i32) << 12) | ((coef[4] as i32) << 4) | ((coef[5] as i32) >> 4);
        let c00_raw = if c00_raw & 0x80000 != 0 { c00_raw | !0xFFFFF_i32 } else { c00_raw };

        // c10: 20-bit from coef[5..8]
        let c10_raw = (((coef[5] & 0x0F) as i32) << 16) | ((coef[6] as i32) << 8) | (coef[7] as i32);
        let c10_raw = if c10_raw & 0x80000 != 0 { c10_raw | !0xFFFFF_i32 } else { c10_raw };

        // c01: 16-bit from coef[8..10]
        let c01_raw = ((coef[8] as i16) << 8) | (coef[9] as i16);

        // c11: 16-bit from coef[10..12]
        let c11_raw = ((coef[10] as i16) << 8) | (coef[11] as i16);

        // c20: 16-bit from coef[12..14]
        let c20_raw = ((coef[12] as i16) << 8) | (coef[13] as i16);

        // c21: 16-bit from coef[14..16]
        let c21_raw = ((coef[14] as i16) << 8) | (coef[15] as i16);

        // c30: 16-bit from coef[16..18]
        let c30_raw = ((coef[16] as i16) << 8) | (coef[17] as i16);

        self.cal = Dps310Cal {
            c0: c0_raw as f32,
            c1: c1_raw as f32,
            c00: c00_raw as f32,
            c10: c10_raw as f32,
            c01: c01_raw as f32,
            c11: c11_raw as f32,
            c20: c20_raw as f32,
            c21: c21_raw as f32,
            c30: c30_raw as f32,
        };
    }

    /// Compensate raw pressure and temperature readings.
    /// Returns (pressure_pa, temperature_c).
    pub fn compensate(&self, raw_prs: i32, raw_tmp: i32) -> (f32, f32) {
        let t_scaled = raw_tmp as f32 / self.tmp_scale;
        let p_scaled = raw_prs as f32 / self.prs_scale;

        let temperature = self.cal.c0 * 0.5 + self.cal.c1 * t_scaled;

        let pressure = self.cal.c00
            + p_scaled * (self.cal.c10 + p_scaled * (self.cal.c20 + p_scaled * self.cal.c30))
            + t_scaled * (self.cal.c01 + p_scaled * (self.cal.c11 + p_scaled * self.cal.c21));

        (pressure, temperature)
    }

    /// Convert pressure to altitude using barometric formula.
    pub fn altitude_from_pressure(pressure_pa: f32, ground_pressure_pa: f32) -> f32 {
        if ground_pressure_pa <= 0.0 || pressure_pa <= 0.0 { return 0.0; }
        44330.0 * (1.0 - libm::powf(pressure_pa / ground_pressure_pa, 0.190295))
    }

    /// Process a reading. Handles ground calibration automatically.
    pub fn process_reading(&mut self, raw_prs: i32, raw_tmp: i32) -> BaroReading {
        let (pressure, temperature) = self.compensate(raw_prs, raw_tmp);
        self.last_pressure = pressure;
        self.last_temperature = temperature;

        // Ground calibration: average first 10 samples
        if !self.calibrated {
            self.cal_sum += pressure;
            self.cal_count += 1;
            if self.cal_count >= 10 {
                self.ground_pressure = self.cal_sum / self.cal_count as f32;
                self.calibrated = true;
            }
        }

        let altitude = Self::altitude_from_pressure(pressure, self.ground_pressure);

        BaroReading { pressure_pa: pressure, temperature_c: temperature, altitude_m: altitude }
    }

    pub fn is_calibrated(&self) -> bool { self.calibrated }
}

/// Trait for I2C probing (read a register at a given address).
pub trait I2cProbe {
    fn read_register_at(&mut self, addr: u8, reg: u8) -> Option<u8>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dps310_altitude_formula() {
        // At sea level pressure, altitude should be ~0
        let alt = Dps310::altitude_from_pressure(101325.0, 101325.0);
        assert!((alt - 0.0).abs() < 0.1, "Sea level: {}", alt);

        // At ~900 hPa (~1000m elevation)
        let alt = Dps310::altitude_from_pressure(90000.0, 101325.0);
        assert!((alt - 988.0).abs() < 50.0, "~1000m: {}", alt);
    }

    #[test]
    fn test_dps310_compensate_identity() {
        let mut dps = Dps310::new();
        // With zero calibration coefficients, everything should be 0
        dps.cal = Dps310Cal::default();
        let (p, t) = dps.compensate(0, 0);
        assert_eq!(p, 0.0);
        assert_eq!(t, 0.0);
    }

    #[test]
    fn test_dps310_compensate_temperature() {
        let mut dps = Dps310::new();
        dps.cal.c0 = 50.0; // temperature offset
        dps.cal.c1 = 1.0;  // temperature slope
        let (_, t) = dps.compensate(0, 1000000);
        // t = c0*0.5 + c1 * (1000000 / scale_factor)
        assert!(t > 25.0, "Temperature should be positive: {}", t);
    }

    #[test]
    fn test_dps310_ground_calibration() {
        let mut dps = Dps310::new();
        dps.cal.c00 = 101325.0; // make compensation return ~101325 Pa
        assert!(!dps.is_calibrated());

        // Feed 10 samples
        for _ in 0..10 {
            dps.process_reading(0, 0);
        }
        assert!(dps.is_calibrated());

        // After calibration, altitude should be near 0
        let reading = dps.process_reading(0, 0);
        assert!((reading.altitude_m - 0.0).abs() < 1.0,
            "Ground altitude should be ~0: {}", reading.altitude_m);
    }

    #[test]
    fn test_dps310_parse_calibration() {
        let mut dps = Dps310::new();
        // Fabricated coefficient bytes
        let coef = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
                    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA];
        dps.parse_calibration(&coef, 0);
        // Just verify it doesn't crash and produces non-zero values
        assert!(dps.cal.c0 != 0.0 || dps.cal.c1 != 0.0);
    }

    #[test]
    fn test_scale_factor() {
        assert_eq!(scale_factor(0), 524288.0);
        assert_eq!(scale_factor(3), 7864320.0);
    }
}

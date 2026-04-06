//! MS5611 barometer driver — the Pixhawk standard pressure sensor.
//!
//! Source: ArduPilot AP_Baro_MS5611.cpp + TE Connectivity MS5611 datasheet
//! Interface: I2C (0x77 or 0x76) or SPI
//! Used on: all Pixhawk boards, many older FCs

/// MS5611 commands.
const CMD_RESET: u8 = 0x1E;
const CMD_CONV_D1_4096: u8 = 0x48; // pressure, OSR=4096
const CMD_CONV_D2_4096: u8 = 0x58; // temperature, OSR=4096
const CMD_ADC_READ: u8 = 0x00;
const CMD_PROM_READ_BASE: u8 = 0xA0; // + 2*index for PROM addresses 0-7

/// MS5611 calibration coefficients (from PROM).
#[derive(Debug, Clone, Copy, Default)]
pub struct Ms5611Cal {
    pub c1: u16, // pressure sensitivity
    pub c2: u16, // pressure offset
    pub c3: u16, // temp coeff of pressure sensitivity
    pub c4: u16, // temp coeff of pressure offset
    pub c5: u16, // reference temperature
    pub c6: u16, // temp coeff of temperature
}

/// Barometer reading.
#[derive(Debug, Clone, Copy)]
pub struct BaroReading {
    pub pressure_pa: f32,
    pub temperature_c: f32,
    pub altitude_m: f32,
}

/// MS5611 driver state.
pub struct Ms5611 {
    cal: Ms5611Cal,
    ground_pressure: f32,
    calibrated: bool,
    cal_sum: f32,
    cal_count: u16,
    last_d1: u32, // raw pressure ADC
    last_d2: u32, // raw temperature ADC
}

impl Ms5611 {
    pub fn new() -> Self {
        Self {
            cal: Ms5611Cal::default(),
            ground_pressure: 101325.0,
            calibrated: false,
            cal_sum: 0.0,
            cal_count: 0,
            last_d1: 0,
            last_d2: 0,
        }
    }

    /// Parse PROM calibration data (7 words, C1-C6 at indices 1-6).
    pub fn parse_prom(&mut self, prom: &[u16; 8]) -> bool {
        // Verify CRC (PROM word 7 contains CRC4 in bits [3:0])
        let crc_read = (prom[7] & 0x000F) as u8;
        let crc_calc = Self::crc4(prom);
        if crc_read != crc_calc {
            return false;
        }

        self.cal = Ms5611Cal {
            c1: prom[1],
            c2: prom[2],
            c3: prom[3],
            c4: prom[4],
            c5: prom[5],
            c6: prom[6],
        };
        true
    }

    /// CRC4 over PROM data per TE Connectivity AN520.
    /// Processes each PROM word exactly once: high byte then low byte.
    fn crc4(prom: &[u16; 8]) -> u8 {
        let mut n_rem: u16 = 0;

        // Working copy with CRC bits zeroed
        let mut data = *prom;
        data[7] &= 0xFF00; // zero CRC nibble in word 7

        for i in 0..16 {
            // Process high byte for even i, low byte for odd i
            // Each word is processed once: word[i/2] high byte, then low byte
            if i % 2 == 0 {
                n_rem ^= data[i >> 1] >> 8;
            } else {
                n_rem ^= data[i >> 1] & 0x00FF;
            }
            for _ in 0..8 {
                if n_rem & 0x8000 != 0 {
                    n_rem = (n_rem << 1) ^ 0x3000;
                } else {
                    n_rem <<= 1;
                }
            }
        }
        ((n_rem >> 12) & 0x0F) as u8
    }

    /// Compensate raw ADC values to pressure (Pa) and temperature (°C).
    /// Source: MS5611 datasheet, second-order compensation.
    pub fn compensate(&self, d1: u32, d2: u32) -> (f32, f32) {
        // First-order compensation
        let dt = d2 as i64 - (self.cal.c5 as i64) * 256;
        let mut temp = 2000 + (dt * self.cal.c6 as i64) / 8388608; // 2^23

        let mut off = (self.cal.c2 as i64) * 65536 + (self.cal.c4 as i64 * dt) / 128;
        let mut sens = (self.cal.c1 as i64) * 32768 + (self.cal.c3 as i64 * dt) / 256;

        // Second-order compensation (for temperatures < 20°C)
        if temp < 2000 {
            let t2 = (dt * dt) / 2147483648; // 2^31
            let temp_diff = temp - 2000;
            let mut off2 = 5 * temp_diff * temp_diff / 2;
            let mut sens2 = 5 * temp_diff * temp_diff / 4;

            if temp < -1500 {
                let temp_diff2 = temp + 1500;
                off2 += 7 * temp_diff2 * temp_diff2;
                sens2 += 11 * temp_diff2 * temp_diff2 / 2;
            }

            temp -= t2;
            off -= off2;
            sens -= sens2;
        }

        let pressure = ((d1 as i64 * sens / 2097152 - off) / 32768) as f32; // Pa * 100? No, units depend
        let temperature = temp as f32 / 100.0; // °C

        // MS5611 formula produces pressure in Pa (1 Pa = 0.01 mbar)
        (pressure, temperature)
    }

    /// Process a reading pair. Call after both D1 and D2 conversions complete.
    pub fn process_reading(&mut self, d1: u32, d2: u32) -> BaroReading {
        self.last_d1 = d1;
        self.last_d2 = d2;

        let (pressure, temperature) = self.compensate(d1, d2);

        // Ground calibration
        if !self.calibrated {
            self.cal_sum += pressure;
            self.cal_count += 1;
            if self.cal_count >= 10 {
                self.ground_pressure = self.cal_sum / self.cal_count as f32;
                self.calibrated = true;
            }
        }

        let altitude = altitude_from_pressure(pressure, self.ground_pressure);
        BaroReading { pressure_pa: pressure, temperature_c: temperature, altitude_m: altitude }
    }

    pub fn is_calibrated(&self) -> bool { self.calibrated }
}

/// Barometric altitude formula.
pub fn altitude_from_pressure(pressure_pa: f32, ground_pressure_pa: f32) -> f32 {
    if ground_pressure_pa <= 0.0 || pressure_pa <= 0.0 { return 0.0; }
    44330.0 * (1.0 - libm::powf(pressure_pa / ground_pressure_pa, 0.190295))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ms5611_compensation_typical() {
        let mut ms = Ms5611::new();
        // Typical calibration values from datasheet example
        ms.cal = Ms5611Cal {
            c1: 40127, c2: 36924, c3: 23317,
            c4: 23282, c5: 33464, c6: 28312,
        };
        // D1=9085466, D2=8569150 → expected P≈100009 Pa, T≈20.07°C (datasheet example)
        let (p, t) = ms.compensate(9085466, 8569150);
        assert!((t - 20.07).abs() < 1.0, "Temperature: {}", t);
        assert!((p - 100009.0).abs() < 500.0, "Pressure: {}", p);
    }

    #[test]
    fn test_ms5611_altitude() {
        let alt = altitude_from_pressure(101325.0, 101325.0);
        assert!((alt - 0.0).abs() < 0.5, "Sea level: {}", alt);

        let alt = altitude_from_pressure(89876.0, 101325.0);
        assert!((alt - 1000.0).abs() < 50.0, "~1000m: {}", alt);
    }

    #[test]
    fn test_ms5611_ground_cal() {
        let mut ms = Ms5611::new();
        ms.cal = Ms5611Cal { c1: 40127, c2: 36924, c3: 23317, c4: 23282, c5: 33464, c6: 28312 };
        assert!(!ms.is_calibrated());

        for _ in 0..10 {
            ms.process_reading(9085466, 8569150);
        }
        assert!(ms.is_calibrated());

        let reading = ms.process_reading(9085466, 8569150);
        assert!((reading.altitude_m - 0.0).abs() < 1.0, "Ground alt: {}", reading.altitude_m);
    }

    #[test]
    fn test_ms5611_crc4() {
        // Test with known PROM values where CRC is valid
        let mut prom = [0u16; 8];
        prom[0] = 0x0000;
        prom[1] = 40127;
        prom[2] = 36924;
        prom[3] = 23317;
        prom[4] = 23282;
        prom[5] = 33464;
        prom[6] = 28312;
        // Compute CRC and store it
        let crc = Ms5611::crc4(&prom);
        prom[7] = (prom[7] & 0xFFF0) | (crc as u16);

        let mut ms = Ms5611::new();
        assert!(ms.parse_prom(&prom), "CRC should match");
    }
}

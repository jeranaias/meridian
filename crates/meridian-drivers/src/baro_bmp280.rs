//! BMP280 / BME280 barometric pressure sensor driver over I2C.
//!
//! Implements the Bosch integer compensation algorithm from the BMP280 datasheet.
//! Reads temperature and pressure, computes altitude from the hypsometric formula.
//!
//! Datasheet: BST-BMP280-DS001-19
//! Registers:
//!   0xD0 = chip ID (0x58 for BMP280, 0x60 for BME280)
//!   0x88..0x9F = calibration data (26 bytes: T1-T3, P1-P9)
//!   0xF4 = ctrl_meas (osrs_t, osrs_p, mode)
//!   0xF7..0xFC = raw ADC (press[19:12], press[11:4], press[3:0]<<4, temp same)

use meridian_hal::i2c::I2cDevice;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Default I2C address (SDO to GND). SDO to VCC = 0x77.
pub const BMP280_ADDR: u8 = 0x76;

const REG_CHIP_ID: u8 = 0xD0;
const REG_CALIB_START: u8 = 0x88;
const CALIB_LEN: usize = 26;
const REG_CTRL_MEAS: u8 = 0xF4;
const REG_DATA_START: u8 = 0xF7;
const DATA_LEN: usize = 6;

const CHIP_ID_BMP280: u8 = 0x58;
const CHIP_ID_BME280: u8 = 0x60;

/// Standard sea-level pressure in Pa.
pub const SEA_LEVEL_PA: f32 = 101_325.0;

// ---------------------------------------------------------------------------
// Calibration data
// ---------------------------------------------------------------------------

/// Trimming parameters read from the sensor's NVM.
#[derive(Debug, Clone, Copy)]
pub struct Bmp280Calibration {
    pub dig_t1: u16,
    pub dig_t2: i16,
    pub dig_t3: i16,
    pub dig_p1: u16,
    pub dig_p2: i16,
    pub dig_p3: i16,
    pub dig_p4: i16,
    pub dig_p5: i16,
    pub dig_p6: i16,
    pub dig_p7: i16,
    pub dig_p8: i16,
    pub dig_p9: i16,
}

impl Bmp280Calibration {
    /// Parse 26 bytes of calibration data from registers 0x88..0x9F.
    pub fn from_bytes(b: &[u8; CALIB_LEN]) -> Self {
        Self {
            dig_t1: u16::from_le_bytes([b[0], b[1]]),
            dig_t2: i16::from_le_bytes([b[2], b[3]]),
            dig_t3: i16::from_le_bytes([b[4], b[5]]),
            dig_p1: u16::from_le_bytes([b[6], b[7]]),
            dig_p2: i16::from_le_bytes([b[8], b[9]]),
            dig_p3: i16::from_le_bytes([b[10], b[11]]),
            dig_p4: i16::from_le_bytes([b[12], b[13]]),
            dig_p5: i16::from_le_bytes([b[14], b[15]]),
            dig_p6: i16::from_le_bytes([b[16], b[17]]),
            dig_p7: i16::from_le_bytes([b[18], b[19]]),
            dig_p8: i16::from_le_bytes([b[20], b[21]]),
            dig_p9: i16::from_le_bytes([b[22], b[23]]),
            // bytes 24-25 are reserved / dig_t related padding
        }
    }
}

// ---------------------------------------------------------------------------
// Bosch integer compensation (from datasheet section 4.2.3)
// ---------------------------------------------------------------------------

/// Compensate raw temperature ADC. Returns (temperature in 0.01 degC, t_fine).
pub fn compensate_temperature(raw: i32, cal: &Bmp280Calibration) -> (i32, i32) {
    let var1 = ((((raw >> 3) - ((cal.dig_t1 as i32) << 1))) * (cal.dig_t2 as i32)) >> 11;
    let var2 = (((((raw >> 4) - (cal.dig_t1 as i32))
        * ((raw >> 4) - (cal.dig_t1 as i32)))
        >> 12)
        * (cal.dig_t3 as i32))
        >> 14;
    let t_fine = var1 + var2;
    let temperature = (t_fine * 5 + 128) >> 8;
    (temperature, t_fine)
}

/// Compensate raw pressure ADC. Returns pressure in Pa as u32 (Q24.8 format).
/// The returned value divided by 256 gives Pa.
pub fn compensate_pressure(raw: i32, t_fine: i32, cal: &Bmp280Calibration) -> u32 {
    let mut var1 = (t_fine as i64) - 128000;
    let mut var2 = var1 * var1 * (cal.dig_p6 as i64);
    var2 = var2 + ((var1 * (cal.dig_p5 as i64)) << 17);
    var2 = var2 + ((cal.dig_p4 as i64) << 35);
    var1 = ((var1 * var1 * (cal.dig_p3 as i64)) >> 8) + ((var1 * (cal.dig_p2 as i64)) << 12);
    var1 = (((1i64 << 47) + var1) * (cal.dig_p1 as i64)) >> 33;

    if var1 == 0 {
        return 0; // avoid division by zero
    }

    let mut p: i64 = 1_048_576 - raw as i64;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((cal.dig_p9 as i64) * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((cal.dig_p8 as i64) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((cal.dig_p7 as i64) << 4);

    p as u32
}

// ---------------------------------------------------------------------------
// Altitude from pressure (hypsometric formula)
// ---------------------------------------------------------------------------

/// Compute altitude in meters from pressure and reference sea-level pressure.
///
/// Formula: `44330.0 * (1.0 - (P / P0)^0.190295)`
pub fn altitude_from_pressure(pressure_pa: f32, p0_pa: f32) -> f32 {
    let ratio = pressure_pa / p0_pa;
    44330.0 * (1.0 - libm::powf(ratio, 0.190295))
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

/// Initialization error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bmp280Error {
    /// Chip did not respond on I2C.
    NotFound,
    /// Chip ID was not BMP280 (0x58) or BME280 (0x60).
    WrongChipId(u8),
    /// Failed to read calibration data.
    CalibReadFailed,
    /// Failed to write configuration.
    ConfigWriteFailed,
}

/// Reading from the sensor.
#[derive(Debug, Clone, Copy)]
pub struct Bmp280Reading {
    /// Temperature in degrees Celsius.
    pub temperature_c: f32,
    /// Pressure in Pascals.
    pub pressure_pa: f32,
    /// Altitude in meters above reference pressure (sea level default).
    pub altitude_m: f32,
}

/// BMP280 driver state.
#[derive(Debug)]
pub struct Bmp280 {
    pub calibration: Bmp280Calibration,
    pub chip_id: u8,
    pub sea_level_pa: f32,
}

impl Bmp280 {
    /// Initialize the BMP280: probe chip ID, read calibration, configure.
    ///
    /// Sets oversampling: temperature x2, pressure x16, normal mode.
    /// ctrl_meas register = osrs_t(010) | osrs_p(101) | mode(11) = 0b0101_0111 = 0x57
    pub fn init(i2c: &mut dyn I2cDevice) -> Result<Self, Bmp280Error> {
        i2c.set_address(BMP280_ADDR);

        // Read chip ID
        let chip_id = i2c.read_register(REG_CHIP_ID).ok_or(Bmp280Error::NotFound)?;
        if chip_id != CHIP_ID_BMP280 && chip_id != CHIP_ID_BME280 {
            return Err(Bmp280Error::WrongChipId(chip_id));
        }

        // Read calibration
        let mut cal_buf = [0u8; CALIB_LEN];
        if !i2c.read_registers(REG_CALIB_START, &mut cal_buf) {
            return Err(Bmp280Error::CalibReadFailed);
        }
        let calibration = Bmp280Calibration::from_bytes(&cal_buf);

        // Configure: osrs_t=x2(010), osrs_p=x16(101), mode=normal(11)
        // ctrl_meas = (010 << 5) | (101 << 2) | 11 = 0x57
        if !i2c.write_register(REG_CTRL_MEAS, 0x57) {
            return Err(Bmp280Error::ConfigWriteFailed);
        }

        Ok(Self {
            calibration,
            chip_id,
            sea_level_pa: SEA_LEVEL_PA,
        })
    }

    /// Read temperature and pressure, compute altitude.
    pub fn read(&self, i2c: &mut dyn I2cDevice) -> Option<Bmp280Reading> {
        i2c.set_address(BMP280_ADDR);

        let mut buf = [0u8; DATA_LEN];
        if !i2c.read_registers(REG_DATA_START, &mut buf) {
            return None;
        }

        // Raw pressure: 20-bit unsigned, MSB first
        let raw_press =
            ((buf[0] as i32) << 12) | ((buf[1] as i32) << 4) | ((buf[2] as i32) >> 4);
        // Raw temperature: 20-bit unsigned, MSB first
        let raw_temp =
            ((buf[3] as i32) << 12) | ((buf[4] as i32) << 4) | ((buf[5] as i32) >> 4);

        let (temp_raw, t_fine) = compensate_temperature(raw_temp, &self.calibration);
        let press_q24_8 = compensate_pressure(raw_press, t_fine, &self.calibration);

        let temperature_c = temp_raw as f32 / 100.0;
        let pressure_pa = press_q24_8 as f32 / 256.0;
        let altitude_m = altitude_from_pressure(pressure_pa, self.sea_level_pa);

        Some(Bmp280Reading {
            temperature_c,
            pressure_pa,
            altitude_m,
        })
    }

    /// Set reference sea-level pressure for altitude computation.
    pub fn set_sea_level_pressure(&mut self, pa: f32) {
        self.sea_level_pa = pa;
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use meridian_hal::i2c::I2cDevice;
    use meridian_hal::util::Semaphore;

    // ── Mock I2C device ──────────────────────────────────────────────────

    struct DummySemaphore;
    impl Semaphore for DummySemaphore {
        fn take_blocking(&self) {}
        fn take(&self, _timeout_us: u32) -> bool { true }
        fn give(&self) {}
    }

    static DUMMY_SEM: DummySemaphore = DummySemaphore;

    /// A mock I2C device that simulates BMP280 register reads.
    struct MockI2c {
        chip_id: u8,
        cal_bytes: [u8; CALIB_LEN],
        data_bytes: [u8; DATA_LEN],
        address: u8,
        config_written: Option<u8>,
    }

    impl MockI2c {
        fn bmp280(cal: [u8; CALIB_LEN], data: [u8; DATA_LEN]) -> Self {
            Self {
                chip_id: CHIP_ID_BMP280,
                cal_bytes: cal,
                data_bytes: data,
                address: 0,
                config_written: None,
            }
        }
    }

    impl I2cDevice for MockI2c {
        fn set_address(&mut self, addr: u8) {
            self.address = addr;
        }

        fn transfer(&mut self, send: &[u8], recv: &mut [u8]) -> bool {
            if send.is_empty() && recv.is_empty() {
                return true;
            }
            if send.len() == 1 && !recv.is_empty() {
                // Register read
                let reg = send[0];
                match reg {
                    REG_CHIP_ID => {
                        if !recv.is_empty() {
                            recv[0] = self.chip_id;
                        }
                    }
                    REG_CALIB_START => {
                        let n = recv.len().min(CALIB_LEN);
                        recv[..n].copy_from_slice(&self.cal_bytes[..n]);
                    }
                    REG_DATA_START => {
                        let n = recv.len().min(DATA_LEN);
                        recv[..n].copy_from_slice(&self.data_bytes[..n]);
                    }
                    _ => return false,
                }
                return true;
            }
            if send.len() == 2 && recv.is_empty() {
                // Register write
                self.config_written = Some(send[1]);
                return true;
            }
            false
        }

        fn get_semaphore(&self) -> &dyn Semaphore {
            &DUMMY_SEM
        }

        fn set_speed(&mut self, _speed_hz: u32) {}

        fn device_id(&self) -> u32 {
            0
        }
    }

    // ── Helper: Bosch reference calibration values ──────────────────────

    /// Build calibration bytes for known reference values.
    /// Using the Bosch datasheet example calibration (section 8.1):
    ///   dig_T1 = 27504, dig_T2 = 26435, dig_T3 = -1000
    ///   dig_P1 = 36477, dig_P2 = -10685, dig_P3 = 3024
    ///   dig_P4 = 2855, dig_P5 = 140, dig_P6 = -7, dig_P7 = 15500
    ///   dig_P8 = -14600, dig_P9 = 6000
    fn reference_cal_bytes() -> [u8; CALIB_LEN] {
        let mut b = [0u8; CALIB_LEN];
        b[0..2].copy_from_slice(&27504u16.to_le_bytes());
        b[2..4].copy_from_slice(&26435i16.to_le_bytes());
        b[4..6].copy_from_slice(&(-1000i16).to_le_bytes());
        b[6..8].copy_from_slice(&36477u16.to_le_bytes());
        b[8..10].copy_from_slice(&(-10685i16).to_le_bytes());
        b[10..12].copy_from_slice(&3024i16.to_le_bytes());
        b[12..14].copy_from_slice(&2855i16.to_le_bytes());
        b[14..16].copy_from_slice(&140i16.to_le_bytes());
        b[16..18].copy_from_slice(&(-7i16).to_le_bytes());
        b[18..20].copy_from_slice(&15500i16.to_le_bytes());
        b[20..22].copy_from_slice(&(-14600i16).to_le_bytes());
        b[22..24].copy_from_slice(&6000i16.to_le_bytes());
        b
    }

    fn reference_cal() -> Bmp280Calibration {
        Bmp280Calibration::from_bytes(&reference_cal_bytes())
    }

    #[test]
    fn test_chip_id_probe() {
        let cal = reference_cal_bytes();
        let data = [0u8; DATA_LEN];
        let mut i2c = MockI2c::bmp280(cal, data);
        let baro = Bmp280::init(&mut i2c);
        assert!(baro.is_ok());
        let baro = baro.unwrap();
        assert_eq!(baro.chip_id, CHIP_ID_BMP280);
    }

    #[test]
    fn test_chip_id_bme280() {
        let cal = reference_cal_bytes();
        let data = [0u8; DATA_LEN];
        let mut i2c = MockI2c::bmp280(cal, data);
        i2c.chip_id = CHIP_ID_BME280;
        let baro = Bmp280::init(&mut i2c);
        assert!(baro.is_ok());
        assert_eq!(baro.unwrap().chip_id, CHIP_ID_BME280);
    }

    #[test]
    fn test_chip_id_wrong() {
        let cal = reference_cal_bytes();
        let data = [0u8; DATA_LEN];
        let mut i2c = MockI2c::bmp280(cal, data);
        i2c.chip_id = 0xFF;
        let baro = Bmp280::init(&mut i2c);
        assert_eq!(baro.unwrap_err(), Bmp280Error::WrongChipId(0xFF));
    }

    #[test]
    fn test_temperature_compensation() {
        let cal = reference_cal();
        // Raw temperature ADC = 519888 (from Bosch datasheet example)
        let (temp_c100, t_fine) = compensate_temperature(519888, &cal);
        // Expected: ~25.08 degC => 2508 in 0.01C units
        // t_fine ~ 128422
        // Accept range due to integer truncation
        assert!(
            (temp_c100 - 2508).unsigned_abs() < 5,
            "temperature {temp_c100} expected ~2508"
        );
        assert!(t_fine > 100_000 && t_fine < 200_000, "t_fine={t_fine}");
    }

    #[test]
    fn test_pressure_compensation() {
        let cal = reference_cal();
        // Use the t_fine from temperature compensation
        let (_, t_fine) = compensate_temperature(519888, &cal);
        // Raw pressure ADC = 415148
        let press_q24_8 = compensate_pressure(415148, t_fine, &cal);
        let press_pa = press_q24_8 as f32 / 256.0;
        // Expected: ~100653 Pa (within 500 Pa of sea level)
        assert!(
            press_pa > 95000.0 && press_pa < 110000.0,
            "pressure {press_pa} expected ~100653 Pa"
        );
    }

    #[test]
    fn test_altitude_calculation() {
        // At sea level pressure, altitude should be 0
        let alt = altitude_from_pressure(101_325.0, 101_325.0);
        assert!(alt.abs() < 0.1, "sea level altitude={alt}");

        // At ~90000 Pa with sea level 101325, altitude should be ~1000m
        let alt = altitude_from_pressure(89874.6, 101_325.0);
        assert!(
            (alt - 1000.0).abs() < 50.0,
            "1000m altitude={alt}"
        );

        // At half sea level, altitude should be ~5500m
        let alt = altitude_from_pressure(50_662.5, 101_325.0);
        assert!(
            (alt - 5500.0).abs() < 200.0,
            "5500m altitude={alt}"
        );
    }

    #[test]
    fn test_calibration_parsing() {
        let cal = reference_cal();
        assert_eq!(cal.dig_t1, 27504);
        assert_eq!(cal.dig_t2, 26435);
        assert_eq!(cal.dig_t3, -1000);
        assert_eq!(cal.dig_p1, 36477);
        assert_eq!(cal.dig_p2, -10685);
        assert_eq!(cal.dig_p9, 6000);
    }
}

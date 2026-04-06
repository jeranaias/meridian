//! SPL06-001 barometric pressure sensor driver (Goertek).
//!
//! ArduPilot reference: `AP_Baro_SPL06.cpp`
//!
//! Common budget barometer found on many low-cost flight controllers.
//! I2C address: 0x76 (default) or 0x77.
//!
//! The compensation algorithm uses a two-step process similar to DPS310:
//! 1. Read calibration coefficients from NVM
//! 2. Apply temperature-compensated pressure calculation

use meridian_hal::I2cDevice;

// ---------------------------------------------------------------------------
// Register addresses
// ---------------------------------------------------------------------------

const REG_PRS_B2: u8 = 0x00;
const REG_TMP_B2: u8 = 0x03;
const REG_PRS_CFG: u8 = 0x06;
const REG_TMP_CFG: u8 = 0x07;
const REG_MEAS_CFG: u8 = 0x08;
const REG_CFG_REG: u8 = 0x09;
const REG_ID: u8 = 0x0D;
const REG_COEF: u8 = 0x10;

const SPL06_CHIP_ID: u8 = 0x10;

// ---------------------------------------------------------------------------
// Calibration coefficients
// ---------------------------------------------------------------------------

/// SPL06 calibration data (18 bytes from NVM).
#[derive(Debug, Clone, Copy)]
pub struct Spl06Cal {
    pub c0: i16,
    pub c1: i16,
    pub c00: i32,
    pub c10: i32,
    pub c01: i16,
    pub c11: i16,
    pub c20: i16,
    pub c21: i16,
    pub c30: i16,
}

/// Scale factors for oversampling rate.
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

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Driver errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Spl06Error {
    I2cFailed,
    BadChipId(u8),
    NotReady,
}

/// A barometer reading.
#[derive(Debug, Clone, Copy)]
pub struct BaroReading {
    pub pressure_pa: f32,
    pub temperature_c: f32,
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

/// SPL06-001 I2C driver.
pub struct Spl06 {
    address: u8,
    cal: Option<Spl06Cal>,
    pressure_osr: u8,
    temperature_osr: u8,
    initialised: bool,
}

impl Spl06 {
    pub fn new(address: u8) -> Self {
        Self {
            address,
            cal: None,
            pressure_osr: 3,    // 8x oversampling
            temperature_osr: 3, // 8x oversampling
            initialised: false,
        }
    }

    /// Probe: check chip ID.
    pub fn probe(&self, i2c: &mut dyn I2cDevice) -> Result<(), Spl06Error> {
        i2c.set_address(self.address);
        let id = i2c.read_register(REG_ID).ok_or(Spl06Error::I2cFailed)?;
        if id == SPL06_CHIP_ID {
            Ok(())
        } else {
            Err(Spl06Error::BadChipId(id))
        }
    }

    /// Initialise: read calibration, configure measurement mode.
    pub fn init(&mut self, i2c: &mut dyn I2cDevice) -> Result<(), Spl06Error> {
        i2c.set_address(self.address);

        // Read 18 calibration bytes
        let mut coef = [0u8; 18];
        if !i2c.read_registers(REG_COEF, &mut coef) {
            return Err(Spl06Error::I2cFailed);
        }
        self.cal = Some(Self::parse_calibration(&coef));

        // Configure pressure: 8x oversampling, 32 measurements/sec
        if !i2c.write_register(REG_PRS_CFG, 0x33) {
            return Err(Spl06Error::I2cFailed);
        }

        // Configure temperature: 8x oversampling, external sensor
        if !i2c.write_register(REG_TMP_CFG, 0xB3) { // bit 7 = use external temp sensor
            return Err(Spl06Error::I2cFailed);
        }

        // P_SHIFT and T_SHIFT for >8x oversampling
        if !i2c.write_register(REG_CFG_REG, 0x0C) {
            return Err(Spl06Error::I2cFailed);
        }

        // Continuous pressure and temperature measurement
        if !i2c.write_register(REG_MEAS_CFG, 0x07) {
            return Err(Spl06Error::I2cFailed);
        }

        self.initialised = true;
        Ok(())
    }

    /// Parse 18 calibration bytes into coefficients.
    pub fn parse_calibration(coef: &[u8; 18]) -> Spl06Cal {
        let c0 = ((coef[0] as i16) << 4) | ((coef[1] as i16) >> 4);
        let c0 = if c0 & 0x0800 != 0 { c0 | !0x0FFF } else { c0 };

        let c1 = (((coef[1] & 0x0F) as i16) << 8) | (coef[2] as i16);
        let c1 = if c1 & 0x0800 != 0 { c1 | !0x0FFF } else { c1 };

        let c00 = ((coef[3] as i32) << 12) | ((coef[4] as i32) << 4) | ((coef[5] as i32) >> 4);
        let c00 = if c00 & 0x80000 != 0 { c00 | !0xFFFFF } else { c00 };

        let c10 = (((coef[5] & 0x0F) as i32) << 16) | ((coef[6] as i32) << 8) | (coef[7] as i32);
        let c10 = if c10 & 0x80000 != 0 { c10 | !0xFFFFF } else { c10 };

        let c01 = i16::from_be_bytes([coef[8], coef[9]]);
        let c11 = i16::from_be_bytes([coef[10], coef[11]]);
        let c20 = i16::from_be_bytes([coef[12], coef[13]]);
        let c21 = i16::from_be_bytes([coef[14], coef[15]]);
        let c30 = i16::from_be_bytes([coef[16], coef[17]]);

        Spl06Cal { c0, c1, c00, c10, c01, c11, c20, c21, c30 }
    }

    /// Read raw pressure and temperature, compute compensated values.
    pub fn read(&self, i2c: &mut dyn I2cDevice) -> Result<BaroReading, Spl06Error> {
        if !self.initialised {
            return Err(Spl06Error::NotReady);
        }
        let cal = self.cal.as_ref().ok_or(Spl06Error::NotReady)?;
        i2c.set_address(self.address);

        // Read raw pressure (24-bit signed, 2's complement)
        let mut pbuf = [0u8; 3];
        if !i2c.read_registers(REG_PRS_B2, &mut pbuf) {
            return Err(Spl06Error::I2cFailed);
        }
        let praw = ((pbuf[0] as i32) << 16) | ((pbuf[1] as i32) << 8) | (pbuf[2] as i32);
        let praw = if praw & 0x800000 != 0 { praw | !0xFFFFFF } else { praw };

        // Read raw temperature (24-bit signed)
        let mut tbuf = [0u8; 3];
        if !i2c.read_registers(REG_TMP_B2, &mut tbuf) {
            return Err(Spl06Error::I2cFailed);
        }
        let traw = ((tbuf[0] as i32) << 16) | ((tbuf[1] as i32) << 8) | (tbuf[2] as i32);
        let traw = if traw & 0x800000 != 0 { traw | !0xFFFFFF } else { traw };

        let reading = Self::compensate(cal, praw, traw, self.pressure_osr, self.temperature_osr);
        Ok(reading)
    }

    /// Compensate raw values using calibration coefficients.
    pub fn compensate(cal: &Spl06Cal, praw: i32, traw: i32, p_osr: u8, t_osr: u8) -> BaroReading {
        let kt = scale_factor(t_osr);
        let kp = scale_factor(p_osr);

        let traw_sc = traw as f32 / kt;
        let praw_sc = praw as f32 / kp;

        let temperature_c = cal.c0 as f32 * 0.5 + cal.c1 as f32 * traw_sc;

        let pressure_pa = cal.c00 as f32
            + praw_sc * (cal.c10 as f32 + praw_sc * (cal.c20 as f32 + praw_sc * cal.c30 as f32))
            + traw_sc * cal.c01 as f32
            + traw_sc * praw_sc * (cal.c11 as f32 + praw_sc * cal.c21 as f32);

        BaroReading { pressure_pa, temperature_c }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_calibration() {
        // Synthetic calibration bytes
        let coef = [
            0x12, 0x34, 0x56, // c0, c1
            0x78, 0x9A, 0xBC, // c00 high
            0xDE, 0xF0, // c10 low part from coef[5..8]
            0x01, 0x02, // c01
            0x03, 0x04, // c11
            0x05, 0x06, // c20
            0x07, 0x08, // c21
            0x09, 0x0A, // c30
        ];
        let cal = Spl06::parse_calibration(&coef);
        // c0 = (0x12 << 4) | (0x34 >> 4) = 0x123 = 291 (positive, bit 11 = 0)
        assert_eq!(cal.c0, 0x123);
    }

    #[test]
    fn test_scale_factors() {
        assert_eq!(scale_factor(0), 524288.0);
        assert_eq!(scale_factor(7), 2088960.0);
    }
}

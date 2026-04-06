//! QMC5883L compass driver (I2C).
//!
//! Common HMC5883L replacement / clone found on many budget boards.
//! I2C address 0x0D, Chip ID register 0x0D = 0xFF.
//! 16-bit XYZ output, continuous mode, 200 Hz ODR, +/-800 uT range.

use meridian_hal::I2cDevice;
use meridian_math::{frames::Body, Vec3};

// ---------------------------------------------------------------------------
// Registers
// ---------------------------------------------------------------------------

const QMC5883L_ADDR: u8 = 0x0D;

const REG_DATA_X_LSB: u8 = 0x00;
const REG_STATUS: u8 = 0x06;
const REG_CTRL1: u8 = 0x09;
const REG_CTRL2: u8 = 0x0A;
const REG_SET_RESET: u8 = 0x0B;
const REG_CHIP_ID: u8 = 0x0D;

const CHIP_ID_QMC5883L: u8 = 0xFF;

const DATA_LEN: usize = 6;

// ---------------------------------------------------------------------------
// Scale factor
// ---------------------------------------------------------------------------

/// +/-800 uT range (field range 01 = 8 Gauss).
/// Sensitivity: 3000 LSB/Gauss at +/-8 Ga.
/// We return milliGauss: 1 Gauss = 1000 mGa.
/// mGa/LSB = 1000.0 / 3000.0 = 0.3333...
const SCALE_MGA_PER_LSB: f32 = 1000.0 / 3000.0;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
pub struct Qmc5883lReading {
    pub field: Vec3<Body>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Qmc5883lError {
    I2cFailed,
    WrongChipId(u8),
    InitFailed,
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

pub struct Qmc5883l {
    initialised: bool,
}

impl Qmc5883l {
    pub const fn new() -> Self {
        Self { initialised: false }
    }

    /// Probe: check chip ID register.
    pub fn probe(i2c: &mut dyn I2cDevice) -> Result<(), Qmc5883lError> {
        i2c.set_address(QMC5883L_ADDR);
        let id = i2c.read_register(REG_CHIP_ID).ok_or(Qmc5883lError::I2cFailed)?;
        if id != CHIP_ID_QMC5883L {
            return Err(Qmc5883lError::WrongChipId(id));
        }
        Ok(())
    }

    /// Initialize: soft reset, set/reset period, continuous mode at 200 Hz.
    pub fn init(&mut self, i2c: &mut dyn I2cDevice) -> Result<(), Qmc5883lError> {
        i2c.set_address(QMC5883L_ADDR);

        let writes: [(u8, u8); 3] = [
            // Soft reset
            (REG_CTRL2, 0x80),
            // Set/Reset period (recommended 0x01)
            (REG_SET_RESET, 0x01),
            // CTRL1: continuous mode, 200 Hz ODR, +/-8 Ga range, 512x oversample
            // mode=01 (continuous), ODR=11 (200Hz), range=01 (8Ga), OSR=00 (512)
            // = 0b00_01_11_01 = 0x1D
            (REG_CTRL1, 0x1D),
        ];
        for &(reg, val) in &writes {
            if !i2c.write_register(reg, val) {
                return Err(Qmc5883lError::InitFailed);
            }
        }
        self.initialised = true;
        Ok(())
    }

    /// Read XYZ magnetic field.
    pub fn read(&self, i2c: &mut dyn I2cDevice) -> Result<Qmc5883lReading, Qmc5883lError> {
        if !self.initialised {
            return Err(Qmc5883lError::InitFailed);
        }
        i2c.set_address(QMC5883L_ADDR);

        let mut buf = [0u8; DATA_LEN];
        if !i2c.read_registers(REG_DATA_X_LSB, &mut buf) {
            return Err(Qmc5883lError::I2cFailed);
        }

        Ok(Self::parse_data(&buf))
    }

    /// Parse 6-byte data buffer into calibrated reading.
    pub fn parse_data(buf: &[u8; DATA_LEN]) -> Qmc5883lReading {
        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);

        Qmc5883lReading {
            field: Vec3::<Body>::new(
                x as f32 * SCALE_MGA_PER_LSB,
                y as f32 * SCALE_MGA_PER_LSB,
                z as f32 * SCALE_MGA_PER_LSB,
            ),
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use meridian_hal::util::Semaphore;

    struct DummySem;
    impl Semaphore for DummySem {
        fn take_blocking(&self) {}
        fn take(&self, _: u32) -> bool { true }
        fn give(&self) {}
    }
    static DSEM: DummySem = DummySem;

    struct MockI2c {
        address: u8,
        regs: heapless::Vec<(u8, u8), 16>,
        writes: heapless::Vec<(u8, u8), 16>,
        data_bytes: [u8; DATA_LEN],
    }

    impl MockI2c {
        fn new(chip_id: u8) -> Self {
            let mut s = Self {
                address: 0,
                regs: heapless::Vec::new(),
                writes: heapless::Vec::new(),
                data_bytes: [0u8; DATA_LEN],
            };
            let _ = s.regs.push((REG_CHIP_ID, chip_id));
            s
        }
    }

    impl I2cDevice for MockI2c {
        fn set_address(&mut self, addr: u8) { self.address = addr; }
        fn transfer(&mut self, send: &[u8], recv: &mut [u8]) -> bool {
            if send.len() == 1 && !recv.is_empty() {
                let reg = send[0];
                if reg == REG_DATA_X_LSB && recv.len() == DATA_LEN {
                    recv.copy_from_slice(&self.data_bytes);
                    return true;
                }
                if let Some((_, v)) = self.regs.iter().find(|(r,_)| *r == reg) {
                    recv[0] = *v;
                    return true;
                }
                return false;
            }
            if send.len() == 2 && recv.is_empty() {
                let _ = self.writes.push((send[0], send[1]));
                return true;
            }
            !send.is_empty() || !recv.is_empty()
        }
        fn get_semaphore(&self) -> &dyn Semaphore { &DSEM }
        fn set_speed(&mut self, _: u32) {}
        fn device_id(&self) -> u32 { 0 }
    }

    #[test]
    fn test_probe_success() {
        let mut i2c = MockI2c::new(CHIP_ID_QMC5883L);
        assert!(Qmc5883l::probe(&mut i2c).is_ok());
    }

    #[test]
    fn test_probe_wrong_id() {
        let mut i2c = MockI2c::new(0x00);
        assert_eq!(Qmc5883l::probe(&mut i2c).unwrap_err(), Qmc5883lError::WrongChipId(0x00));
    }

    #[test]
    fn test_parse_data_known_values() {
        // X=3000 LSB -> 1000 mGa (1 Gauss), Y=-3000 -> -1000 mGa, Z=0
        let mut buf = [0u8; DATA_LEN];
        buf[0..2].copy_from_slice(&3000i16.to_le_bytes());
        buf[2..4].copy_from_slice(&(-3000i16).to_le_bytes());
        buf[4..6].copy_from_slice(&0i16.to_le_bytes());
        let reading = Qmc5883l::parse_data(&buf);
        assert!((reading.field.x - 1000.0).abs() < 0.5);
        assert!((reading.field.y - (-1000.0)).abs() < 0.5);
        assert!(reading.field.z.abs() < 0.001);
    }

    #[test]
    fn test_init_sets_continuous_mode() {
        let mut i2c = MockI2c::new(CHIP_ID_QMC5883L);
        let mut drv = Qmc5883l::new();
        drv.init(&mut i2c).unwrap();
        assert!(drv.initialised);
        let ctrl1 = i2c.writes.iter().find(|(r,_)| *r == REG_CTRL1);
        assert!(ctrl1.is_some());
        assert_eq!(ctrl1.unwrap().1, 0x1D);
    }

    #[test]
    fn test_read_rejects_uninitialised() {
        let mut i2c = MockI2c::new(CHIP_ID_QMC5883L);
        let drv = Qmc5883l::new();
        assert_eq!(drv.read(&mut i2c).unwrap_err(), Qmc5883lError::InitFailed);
    }
}

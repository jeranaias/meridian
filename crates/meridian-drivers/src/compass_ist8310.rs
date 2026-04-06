//! IST8310 compass driver (I2C).
//!
//! Most common internal compass on Pixhawk-class boards.
//! I2C address 0x0E, WHO_AM_I register 0x00 = 0x10.
//! 16-bit XYZ output, +/-1600 uT range.
//! Continuous measurement mode at 200 Hz.

use meridian_hal::I2cDevice;
use meridian_math::{frames::Body, Vec3};

// ---------------------------------------------------------------------------
// Registers
// ---------------------------------------------------------------------------

const IST8310_ADDR: u8 = 0x0E;

const REG_WHO_AM_I: u8 = 0x00;
const REG_CTRL1: u8 = 0x0A;
const REG_CTRL2: u8 = 0x0B;
const REG_AVG_CTRL: u8 = 0x41;
const REG_PDCNTL: u8 = 0x42;
const REG_DATA_X_LSB: u8 = 0x03;

const WHOAMI_IST8310: u8 = 0x10;

/// Data length: 6 bytes (XYZ, 16-bit LE each).
const DATA_LEN: usize = 6;

// ---------------------------------------------------------------------------
// Scale factor
// ---------------------------------------------------------------------------

/// IST8310: +/-1600 uT full scale, 16-bit signed.
/// Scale: 0.3 uT/LSB (from datasheet). Convert to Gauss: 1 Gauss = 100 uT.
/// We return milliGauss (mGa) for compatibility with ArduPilot compass pipeline.
/// 0.3 uT/LSB * 10 mGa/uT = 3.0 mGa/LSB.
const SCALE_MGA_PER_LSB: f32 = 3.0;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Magnetometer reading in milliGauss, body frame.
#[derive(Debug, Clone, Copy)]
pub struct Ist8310Reading {
    pub field: Vec3<Body>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Ist8310Error {
    I2cFailed,
    WrongChipId(u8),
    InitFailed,
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

pub struct Ist8310 {
    initialised: bool,
}

impl Ist8310 {
    pub const fn new() -> Self {
        Self { initialised: false }
    }

    /// Probe: set address and check WHO_AM_I.
    pub fn probe(i2c: &mut dyn I2cDevice) -> Result<(), Ist8310Error> {
        i2c.set_address(IST8310_ADDR);
        let id = i2c.read_register(REG_WHO_AM_I).ok_or(Ist8310Error::I2cFailed)?;
        if id != WHOAMI_IST8310 {
            return Err(Ist8310Error::WrongChipId(id));
        }
        Ok(())
    }

    /// Initialize: configure continuous mode at 200 Hz.
    pub fn init(&mut self, i2c: &mut dyn I2cDevice) -> Result<(), Ist8310Error> {
        i2c.set_address(IST8310_ADDR);

        let writes: [(u8, u8); 3] = [
            // Average control: 16x averaging for noise reduction
            (REG_AVG_CTRL, 0x24),
            // Pulse duration control: normal
            (REG_PDCNTL, 0xC0),
            // CTRL1: continuous measurement mode, 200 Hz ODR
            (REG_CTRL1, 0x0B),
        ];
        for &(reg, val) in &writes {
            if !i2c.write_register(reg, val) {
                return Err(Ist8310Error::InitFailed);
            }
        }
        self.initialised = true;
        Ok(())
    }

    /// Read XYZ magnetic field.
    pub fn read(&self, i2c: &mut dyn I2cDevice) -> Result<Ist8310Reading, Ist8310Error> {
        if !self.initialised {
            return Err(Ist8310Error::InitFailed);
        }
        i2c.set_address(IST8310_ADDR);

        let mut buf = [0u8; DATA_LEN];
        if !i2c.read_registers(REG_DATA_X_LSB, &mut buf) {
            return Err(Ist8310Error::I2cFailed);
        }

        Ok(Self::parse_data(&buf))
    }

    /// Parse 6-byte data buffer into a reading.
    pub fn parse_data(buf: &[u8; DATA_LEN]) -> Ist8310Reading {
        let x = i16::from_le_bytes([buf[0], buf[1]]);
        let y = i16::from_le_bytes([buf[2], buf[3]]);
        let z = i16::from_le_bytes([buf[4], buf[5]]);

        Ist8310Reading {
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
            let _ = s.regs.push((REG_WHO_AM_I, chip_id));
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
        let mut i2c = MockI2c::new(WHOAMI_IST8310);
        assert!(Ist8310::probe(&mut i2c).is_ok());
    }

    #[test]
    fn test_probe_wrong_id() {
        let mut i2c = MockI2c::new(0xAB);
        assert_eq!(Ist8310::probe(&mut i2c).unwrap_err(), Ist8310Error::WrongChipId(0xAB));
    }

    #[test]
    fn test_parse_data_known_values() {
        // X = 100, Y = -200, Z = 300 LSB
        let mut buf = [0u8; DATA_LEN];
        buf[0..2].copy_from_slice(&100i16.to_le_bytes());
        buf[2..4].copy_from_slice(&(-200i16).to_le_bytes());
        buf[4..6].copy_from_slice(&300i16.to_le_bytes());
        let reading = Ist8310::parse_data(&buf);
        assert!((reading.field.x - 300.0).abs() < 0.01); // 100 * 3.0
        assert!((reading.field.y - (-600.0)).abs() < 0.01); // -200 * 3.0
        assert!((reading.field.z - 900.0).abs() < 0.01); // 300 * 3.0
    }

    #[test]
    fn test_init_writes_config() {
        let mut i2c = MockI2c::new(WHOAMI_IST8310);
        let mut drv = Ist8310::new();
        drv.init(&mut i2c).unwrap();
        assert!(drv.initialised);
        // Verify CTRL1 was written
        let ctrl1 = i2c.writes.iter().find(|(r,_)| *r == REG_CTRL1);
        assert!(ctrl1.is_some());
        assert_eq!(ctrl1.unwrap().1, 0x0B);
    }

    #[test]
    fn test_read_rejects_uninitialised() {
        let mut i2c = MockI2c::new(WHOAMI_IST8310);
        let drv = Ist8310::new();
        assert_eq!(drv.read(&mut i2c).unwrap_err(), Ist8310Error::InitFailed);
    }
}

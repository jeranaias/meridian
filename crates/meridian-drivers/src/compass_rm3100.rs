//! PNI RM3100 high-resolution compass driver (I2C/SPI).
//!
//! I2C addresses 0x20-0x23 (configurable via SA0/SA1 pins).
//! 24-bit output per axis -- highest resolution of any compass.
//! Cycle count registers configure the measurement time vs resolution tradeoff.
//! Higher cycle count = longer measurement but lower noise.

use meridian_hal::I2cDevice;
use meridian_math::{frames::Body, Vec3};
// Registers
const RM3100_DEFAULT_ADDR: u8 = 0x20;

const REG_REVID: u8 = 0x36;       // Revision ID (acts as WHO_AM_I)
const REG_CMM: u8 = 0x01;         // Continuous measurement mode
const REG_CCX_MSB: u8 = 0x04;     // Cycle count X MSB
const REG_CCX_LSB: u8 = 0x05;     // Cycle count X LSB
const REG_CCY_MSB: u8 = 0x06;
const REG_CCY_LSB: u8 = 0x07;
const REG_CCZ_MSB: u8 = 0x08;
const REG_CCZ_LSB: u8 = 0x09;
const REG_TMRC: u8 = 0x0B;        // Timer / continuous mode rate
const REG_MX: u8 = 0x24;          // Measurement result X (3 bytes)
const REG_STATUS: u8 = 0x34;

const REVID_RM3100: u8 = 0x22;

/// Data length: 9 bytes (3 axes x 24-bit each).
const DATA_LEN: usize = 9;
// Scale factor
/// At default cycle count 200:
/// Sensitivity = 75 LSB/uT (from datasheet).
/// 1 Gauss = 100 uT. 1 Gauss = 1000 milliGauss.
/// mGa/LSB = (1.0 / 75.0) * (1000.0 / 100.0) = 0.1333...
/// = 10.0 / 75.0
const SCALE_MGA_PER_LSB_CC200: f32 = 10.0 / 75.0;

/// Default cycle count for all axes.
const DEFAULT_CYCLE_COUNT: u16 = 200;
// Types
#[derive(Debug, Clone, Copy)]
pub struct Rm3100Reading {
    pub field: Vec3<Body>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rm3100Error {
    I2cFailed,
    WrongRevId(u8),
    InitFailed,
}
// Driver
pub struct Rm3100 {
    addr: u8,
    initialised: bool,
    scale: f32,
}

impl Rm3100 {
    pub const fn new() -> Self {
        Self {
            addr: RM3100_DEFAULT_ADDR,
            initialised: false,
            scale: SCALE_MGA_PER_LSB_CC200,
        }
    }

    /// Create with a specific I2C address (0x20-0x23).
    pub const fn with_address(addr: u8) -> Self {
        Self {
            addr,
            initialised: false,
            scale: SCALE_MGA_PER_LSB_CC200,
        }
    }

    /// Probe: check REVID register.
    pub fn probe(&self, i2c: &mut dyn I2cDevice) -> Result<(), Rm3100Error> {
        i2c.set_address(self.addr);
        let id = i2c.read_register(REG_REVID).ok_or(Rm3100Error::I2cFailed)?;
        if id != REVID_RM3100 {
            return Err(Rm3100Error::WrongRevId(id));
        }
        Ok(())
    }

    /// Initialize: set cycle counts and start continuous measurement.
    pub fn init(&mut self, i2c: &mut dyn I2cDevice) -> Result<(), Rm3100Error> {
        i2c.set_address(self.addr);

        let cc_bytes = DEFAULT_CYCLE_COUNT.to_be_bytes();

        let writes: [(u8, u8); 8] = [
            // Set cycle count for all three axes
            (REG_CCX_MSB, cc_bytes[0]),
            (REG_CCX_LSB, cc_bytes[1]),
            (REG_CCY_MSB, cc_bytes[0]),
            (REG_CCY_LSB, cc_bytes[1]),
            (REG_CCZ_MSB, cc_bytes[0]),
            (REG_CCZ_LSB, cc_bytes[1]),
            // TMRC: ~150 Hz continuous measurement rate (0x04)
            (REG_TMRC, 0x04),
            // CMM: continuous mode, all 3 axes enabled
            // Bit 0 = start, bits 4-6 = XYZ enable = 0x71
            (REG_CMM, 0x71),
        ];

        for &(reg, val) in &writes {
            if !i2c.write_register(reg, val) {
                return Err(Rm3100Error::InitFailed);
            }
        }

        self.initialised = true;
        Ok(())
    }

    /// Read 24-bit XYZ measurement.
    pub fn read(&self, i2c: &mut dyn I2cDevice) -> Result<Rm3100Reading, Rm3100Error> {
        if !self.initialised {
            return Err(Rm3100Error::InitFailed);
        }
        i2c.set_address(self.addr);

        let mut buf = [0u8; DATA_LEN];
        if !i2c.read_registers(REG_MX, &mut buf) {
            return Err(Rm3100Error::I2cFailed);
        }

        Ok(Self::parse_data(&buf, self.scale))
    }

    /// Parse 9-byte measurement buffer (3 x 24-bit big-endian signed).
    pub fn parse_data(buf: &[u8; DATA_LEN], scale: f32) -> Rm3100Reading {
        let x = Self::sign_extend_24(buf[0], buf[1], buf[2]);
        let y = Self::sign_extend_24(buf[3], buf[4], buf[5]);
        let z = Self::sign_extend_24(buf[6], buf[7], buf[8]);

        Rm3100Reading {
            field: Vec3::<Body>::new(
                x as f32 * scale,
                y as f32 * scale,
                z as f32 * scale,
            ),
        }
    }

    /// Sign-extend a 24-bit big-endian value to i32.
    fn sign_extend_24(msb: u8, mid: u8, lsb: u8) -> i32 {
        let raw = ((msb as u32) << 16) | ((mid as u32) << 8) | (lsb as u32);
        // Sign-extend from 24 bits
        if raw & 0x80_0000 != 0 {
            (raw | 0xFF00_0000) as i32
        } else {
            raw as i32
        }
    }
}
// Tests
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
        writes: heapless::Vec<(u8, u8), 32>,
        data_bytes: [u8; DATA_LEN],
    }

    impl MockI2c {
        fn new(rev_id: u8) -> Self {
            let mut s = Self {
                address: 0,
                regs: heapless::Vec::new(),
                writes: heapless::Vec::new(),
                data_bytes: [0u8; DATA_LEN],
            };
            let _ = s.regs.push((REG_REVID, rev_id));
            s
        }
    }

    impl I2cDevice for MockI2c {
        fn set_address(&mut self, addr: u8) { self.address = addr; }
        fn transfer(&mut self, send: &[u8], recv: &mut [u8]) -> bool {
            if send.len() == 1 && !recv.is_empty() {
                let reg = send[0];
                if reg == REG_MX && recv.len() == DATA_LEN {
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
        let mut i2c = MockI2c::new(REVID_RM3100);
        let drv = Rm3100::new();
        assert!(drv.probe(&mut i2c).is_ok());
    }

    #[test]
    fn test_probe_wrong_id() {
        let mut i2c = MockI2c::new(0x00);
        let drv = Rm3100::new();
        assert_eq!(drv.probe(&mut i2c).unwrap_err(), Rm3100Error::WrongRevId(0x00));
    }

    #[test]
    fn test_sign_extend_24_positive() {
        // 0x001234 = 4660
        assert_eq!(Rm3100::sign_extend_24(0x00, 0x12, 0x34), 4660);
    }

    #[test]
    fn test_sign_extend_24_negative() {
        // 0xFF0000 = -65536 (sign extended)
        assert_eq!(Rm3100::sign_extend_24(0xFF, 0x00, 0x00), -65536);
        // 0xFFFFFE = -2
        assert_eq!(Rm3100::sign_extend_24(0xFF, 0xFF, 0xFE), -2);
    }

    #[test]
    fn test_parse_data_known_values() {
        // X = 7500 LSB (positive), Y = -7500 (negative), Z = 0
        // 7500 = 0x001D4C
        // -7500 in 24-bit two's complement: 0xFFE2B4
        let buf: [u8; DATA_LEN] = [
            0x00, 0x1D, 0x4C, // X = 7500
            0xFF, 0xE2, 0xB4, // Y = -7500
            0x00, 0x00, 0x00, // Z = 0
        ];
        let reading = Rm3100::parse_data(&buf, SCALE_MGA_PER_LSB_CC200);
        let expected_x = 7500.0 * SCALE_MGA_PER_LSB_CC200;
        let expected_y = -7500.0 * SCALE_MGA_PER_LSB_CC200;
        assert!((reading.field.x - expected_x).abs() < 0.1);
        assert!((reading.field.y - expected_y).abs() < 0.1);
        assert!(reading.field.z.abs() < 0.001);
    }

    #[test]
    fn test_init_sets_cycle_count_and_cmm() {
        let mut i2c = MockI2c::new(REVID_RM3100);
        let mut drv = Rm3100::new();
        drv.init(&mut i2c).unwrap();
        assert!(drv.initialised);
        // Verify CMM register was set
        let cmm = i2c.writes.iter().find(|(r,_)| *r == REG_CMM);
        assert!(cmm.is_some());
        assert_eq!(cmm.unwrap().1, 0x71);
    }

    #[test]
    fn test_configurable_address() { assert_eq!(Rm3100::with_address(0x23).addr, 0x23); }

    #[test]
    fn test_read_rejects_uninitialised() {
        let mut i2c = MockI2c::new(REVID_RM3100);
        let drv = Rm3100::new();
        assert_eq!(drv.read(&mut i2c).unwrap_err(), Rm3100Error::InitFailed);
    }
}

//! BMI088 IMU driver (industrial, vibration-tolerant).
//!
//! The BMI088 uses **two separate SPI devices**: one for the accelerometer
//! and one for the gyroscope, each with its own chip-select.
//!
//! **Critical**: The accel SPI interface has a dummy read byte -- the first
//! byte read after the address is garbage and must be discarded.
//!
//! Accel WHO_AM_I 0x00 = 0x1E, Gyro WHO_AM_I 0x00 = 0x0F.
//! Popular on Pixhawk 6X, CubeOrange+.

use meridian_hal::SpiDevice;
use meridian_math::{frames::Body, Vec3};
// Registers -- Accelerometer
const ACC_CHIP_ID: u8 = 0x00;
const ACC_CONF: u8 = 0x40;
const ACC_RANGE: u8 = 0x41;
const ACC_PWR_CONF: u8 = 0x7C;
const ACC_PWR_CTRL: u8 = 0x7D;
const ACC_SOFTRESET: u8 = 0x7E;
const ACC_DATA_X_LSB: u8 = 0x12;

const ACC_WHOAMI_VAL: u8 = 0x1E;
const ACC_RESET_CMD: u8 = 0xB6;
// Registers -- Gyroscope
const GYR_CHIP_ID: u8 = 0x00;
const GYR_RANGE_REG: u8 = 0x0F;
const GYR_BW: u8 = 0x10;
const GYR_LPM1: u8 = 0x11;
const GYR_SOFTRESET: u8 = 0x14;
const GYR_DATA_X_LSB: u8 = 0x02;

const GYR_WHOAMI_VAL: u8 = 0x0F;
const GYR_RESET_CMD: u8 = 0xB6;
// Scale factors
/// Accel +/-24 g (BMI088 default): 1365 LSB/g -> m/s^2
/// We configure +/-6 g: 5460 LSB/g for better resolution.
const ACCEL_SCALE_6G: f32 = (1.0 / 5460.0) * 9.80665;

/// Gyro +/-2000 dps: 16.384 LSB/(deg/s)
const GYRO_SCALE: f32 = (1.0 / 16.384) * (core::f32::consts::PI / 180.0);
// Types
#[derive(Debug, Clone, Copy)]
pub struct Bmi088Sample {
    pub accel: Vec3<Body>,
    pub gyro: Vec3<Body>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bmi088Error {
    AccelSpiFailed,
    GyroSpiFailed,
    AccelWrongId(u8),
    GyroWrongId(u8),
    InitFailed,
}
// Driver
pub struct Bmi088 {
    initialised: bool,
}

impl Bmi088 {
    pub const fn new() -> Self {
        Self { initialised: false }
    }

    /// Probe both accel and gyro WHO_AM_I registers.
    pub fn probe(
        accel_spi: &mut dyn SpiDevice,
        gyro_spi: &mut dyn SpiDevice,
    ) -> Result<(), Bmi088Error> {
        // Accel has dummy byte: read 2 bytes, discard first
        let acc_id = Self::accel_read_register(accel_spi, ACC_CHIP_ID)
            .ok_or(Bmi088Error::AccelSpiFailed)?;
        if acc_id != ACC_WHOAMI_VAL {
            return Err(Bmi088Error::AccelWrongId(acc_id));
        }

        let gyr_id = gyro_spi.read_register(GYR_CHIP_ID)
            .ok_or(Bmi088Error::GyroSpiFailed)?;
        if gyr_id != GYR_WHOAMI_VAL {
            return Err(Bmi088Error::GyroWrongId(gyr_id));
        }

        Ok(())
    }

    /// Read a single register from the accel, handling the dummy byte.
    /// BMI088 accel SPI reads return a garbage byte before the real data.
    fn accel_read_register(spi: &mut dyn SpiDevice, reg: u8) -> Option<u8> {
        let tx = [reg | 0x80, 0x00, 0x00];
        let mut rx = [0u8; 3];
        if spi.transfer(&tx, &mut rx) {
            Some(rx[2]) // skip rx[1] (dummy byte)
        } else {
            None
        }
    }

    /// Read multiple accel registers, skipping dummy byte.
    fn accel_read_registers(spi: &mut dyn SpiDevice, reg: u8, buf: &mut [u8]) -> bool {
        let len = buf.len() + 2; // addr + dummy + data
        let mut tx = [0u8; 16];
        let mut rx = [0u8; 16];
        if len > 16 { return false; }
        tx[0] = reg | 0x80;
        if spi.transfer(&tx[..len], &mut rx[..len]) {
            buf.copy_from_slice(&rx[2..len]); // skip byte 1 (dummy)
            true
        } else {
            false
        }
    }

    /// Initialize both accel and gyro sub-devices.
    pub fn init(
        &mut self,
        accel_spi: &mut dyn SpiDevice,
        gyro_spi: &mut dyn SpiDevice,
    ) -> Result<(), Bmi088Error> {
        // Accel: reset -> active mode -> configure ODR + range
        let accel_writes: [(u8, u8); 4] = [
            (ACC_SOFTRESET, ACC_RESET_CMD),
            (ACC_PWR_CONF, 0x00),       // active mode
            (ACC_PWR_CTRL, 0x04),       // enable accel
            (ACC_CONF, 0xAC),           // ODR 1600Hz, normal BW
        ];
        for &(reg, val) in &accel_writes {
            if !accel_spi.write_register(reg, val) {
                return Err(Bmi088Error::InitFailed);
            }
        }
        // Range: +/-6 g (0x01)
        if !accel_spi.write_register(ACC_RANGE, 0x01) {
            return Err(Bmi088Error::InitFailed);
        }

        // Gyro: reset -> configure range + BW + mode
        let gyro_writes: [(u8, u8); 3] = [
            (GYR_SOFTRESET, GYR_RESET_CMD),
            (GYR_RANGE_REG, 0x00),  // +/-2000 dps
            (GYR_BW, 0x01),         // ODR 2000Hz, BW 230Hz
        ];
        for &(reg, val) in &gyro_writes {
            if !gyro_spi.write_register(reg, val) {
                return Err(Bmi088Error::InitFailed);
            }
        }
        // Normal mode
        if !gyro_spi.write_register(GYR_LPM1, 0x00) {
            return Err(Bmi088Error::InitFailed);
        }

        self.initialised = true;
        Ok(())
    }

    /// Read accel and gyro from data registers.
    pub fn read(
        &self,
        accel_spi: &mut dyn SpiDevice,
        gyro_spi: &mut dyn SpiDevice,
    ) -> Result<Bmi088Sample, Bmi088Error> {
        if !self.initialised {
            return Err(Bmi088Error::InitFailed);
        }

        // Accel: 6 bytes from ACC_DATA_X_LSB (with dummy byte handling)
        let mut abuf = [0u8; 6];
        if !Self::accel_read_registers(accel_spi, ACC_DATA_X_LSB, &mut abuf) {
            return Err(Bmi088Error::AccelSpiFailed);
        }
        let ax = i16::from_le_bytes([abuf[0], abuf[1]]);
        let ay = i16::from_le_bytes([abuf[2], abuf[3]]);
        let az = i16::from_le_bytes([abuf[4], abuf[5]]);

        // Gyro: 6 bytes from GYR_DATA_X_LSB (normal SPI, no dummy)
        let mut gbuf = [0u8; 6];
        if !gyro_spi.read_registers(GYR_DATA_X_LSB, &mut gbuf) {
            return Err(Bmi088Error::GyroSpiFailed);
        }
        let gx = i16::from_le_bytes([gbuf[0], gbuf[1]]);
        let gy = i16::from_le_bytes([gbuf[2], gbuf[3]]);
        let gz = i16::from_le_bytes([gbuf[4], gbuf[5]]);

        Ok(Bmi088Sample {
            accel: Vec3::<Body>::new(
                ax as f32 * ACCEL_SCALE_6G,
                ay as f32 * ACCEL_SCALE_6G,
                az as f32 * ACCEL_SCALE_6G,
            ),
            gyro: Vec3::<Body>::new(
                gx as f32 * GYRO_SCALE,
                gy as f32 * GYRO_SCALE,
                gz as f32 * GYRO_SCALE,
            ),
        })
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

    /// Mock SPI. `dummy_byte` = true simulates BMI088 accel (garbage byte on read).
    struct MockSpi { regs: heapless::Vec<(u8, u8), 16>, dummy_byte: bool }
    impl MockSpi {
        fn accel() -> Self { Self { regs: heapless::Vec::new(), dummy_byte: true } }
        fn gyro() -> Self { Self { regs: heapless::Vec::new(), dummy_byte: false } }
        fn set_reg(&mut self, reg: u8, val: u8) {
            for e in self.regs.iter_mut() { if e.0 == reg { e.1 = val; return; } }
            let _ = self.regs.push((reg, val));
        }
    }
    impl SpiDevice for MockSpi {
        fn transfer(&mut self, tx: &[u8], rx: &mut [u8]) -> bool {
            if !tx.is_empty() && (tx[0] & 0x80) != 0 {
                let reg = tx[0] & 0x7F;
                let skip = if self.dummy_byte { 2 } else { 1 }; // data starts after skip
                for b in rx.iter_mut() { *b = 0; }
                for i in skip..rx.len() {
                    let r = reg + (i as u8 - skip as u8);
                    rx[i] = self.regs.iter().find(|(a,_)| *a == r).map(|(_,v)| *v).unwrap_or(0);
                }
            }
            true
        }
        fn write(&mut self, data: &[u8]) -> bool { true }
        fn read(&mut self, data: &mut [u8]) -> bool { data.fill(0); true }
        fn set_speed(&mut self, _: u32) {}
        fn get_semaphore(&self) -> &dyn Semaphore { &DSEM }
        fn device_id(&self) -> u32 { 0 }
    }

    #[test]
    fn test_probe_success() {
        let mut acc = MockSpi::accel();
        let mut gyr = MockSpi::gyro();
        acc.set_reg(ACC_CHIP_ID, ACC_WHOAMI_VAL);
        gyr.set_reg(GYR_CHIP_ID, GYR_WHOAMI_VAL);
        assert!(Bmi088::probe(&mut acc, &mut gyr).is_ok());
    }

    #[test]
    fn test_probe_wrong_accel_id() {
        let mut acc = MockSpi::accel();
        let mut gyr = MockSpi::gyro();
        acc.set_reg(ACC_CHIP_ID, 0xAA);
        gyr.set_reg(GYR_CHIP_ID, GYR_WHOAMI_VAL);
        assert_eq!(Bmi088::probe(&mut acc, &mut gyr).unwrap_err(), Bmi088Error::AccelWrongId(0xAA));
    }

    #[test]
    fn test_probe_wrong_gyro_id() {
        let mut acc = MockSpi::accel();
        let mut gyr = MockSpi::gyro();
        acc.set_reg(ACC_CHIP_ID, ACC_WHOAMI_VAL);
        gyr.set_reg(GYR_CHIP_ID, 0xBB);
        assert_eq!(Bmi088::probe(&mut acc, &mut gyr).unwrap_err(), Bmi088Error::GyroWrongId(0xBB));
    }

    #[test]
    fn test_accel_scale_1g() {
        let val = 5460.0 * ACCEL_SCALE_6G;
        assert!((val - 9.80665).abs() < 0.01);
    }

    #[test]
    fn test_read_rejects_uninitialised() {
        let mut acc = MockSpi::accel();
        let mut gyr = MockSpi::gyro();
        let drv = Bmi088::new();
        assert_eq!(drv.read(&mut acc, &mut gyr).unwrap_err(), Bmi088Error::InitFailed);
    }
}

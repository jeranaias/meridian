//! BMI270 IMU driver (SPI).
//!
//! The BMI270 requires uploading a 4 KB firmware blob before the sensor
//! produces valid data. Init sequence: soft reset -> upload firmware ->
//! configure ODR/range -> enable FIFO.
//!
//! Accel and gyro occupy **separate** FIFO frames (unlike ICM-42688 interleaved).
//! Gyro +/-2000 dps, Accel +/-16 g.

use meridian_hal::SpiDevice;
use meridian_math::{frames::Body, Vec3};

// Registers

const REG_CHIP_ID: u8 = 0x00;
const REG_CMD: u8 = 0x7E;
const REG_PWR_CONF: u8 = 0x7C;
const REG_PWR_CTRL: u8 = 0x7D;
const REG_ACC_CONF: u8 = 0x40;
const REG_ACC_RANGE: u8 = 0x41;
const REG_GYR_CONF: u8 = 0x42;
const REG_GYR_RANGE: u8 = 0x43;
const REG_FIFO_CONFIG_0: u8 = 0x48;
const REG_FIFO_CONFIG_1: u8 = 0x49;
const REG_FIFO_LENGTH_0: u8 = 0x24;
const REG_FIFO_DATA: u8 = 0x26;
const REG_INIT_CTRL: u8 = 0x59;
const REG_INIT_DATA: u8 = 0x5E;
const REG_INTERNAL_STATUS: u8 = 0x21;
const REG_DATA_8: u8 = 0x0C; // accel X LSB

const CHIP_ID_BMI270: u8 = 0x24;
const CMD_SOFT_RESET: u8 = 0xB6;

// Scale factors

/// Gyro +/-2000 dps: 16.4 LSB/(deg/s) -> rad/s
const GYRO_SCALE: f32 = (1.0 / 16.4) * (core::f32::consts::PI / 180.0);

/// Accel +/-16 g: 2048 LSB/g -> m/s^2
const ACCEL_SCALE: f32 = (1.0 / 2048.0) * 9.80665;

// Firmware stub (real blob is ~8 KB; 16-byte token for init flow)

/// Truncated firmware placeholder. On real hardware this is the full 8 KB
/// config file from Bosch. The upload mechanism is identical.
const BMI270_FIRMWARE: &[u8; 16] = &[
    0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x3d, 0xb1,
    0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0xab, 0xb5,
];

// Types

/// One IMU measurement from the BMI270.
#[derive(Debug, Clone, Copy)]
pub struct Bmi270Sample {
    pub accel: Vec3<Body>,
    pub gyro: Vec3<Body>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bmi270Error {
    SpiFailed,
    UnknownChipId(u8),
    InitFailed,
    FirmwareUploadFailed,
}

// Driver

pub struct Bmi270 {
    initialised: bool,
}

impl Bmi270 {
    pub const fn new() -> Self {
        Self { initialised: false }
    }

    /// Read WHO_AM_I and verify BMI270.
    pub fn probe(spi: &mut dyn SpiDevice) -> Result<(), Bmi270Error> {
        let id = spi.read_register(REG_CHIP_ID).ok_or(Bmi270Error::SpiFailed)?;
        if id != CHIP_ID_BMI270 {
            return Err(Bmi270Error::UnknownChipId(id));
        }
        Ok(())
    }

    /// Full init: soft reset, firmware upload, configure ODR/range, enable FIFO.
    pub fn init(&mut self, spi: &mut dyn SpiDevice) -> Result<(), Bmi270Error> {
        // 1. Soft reset
        if !spi.write_register(REG_CMD, CMD_SOFT_RESET) {
            return Err(Bmi270Error::InitFailed);
        }

        // 2. Disable advanced power save for upload
        if !spi.write_register(REG_PWR_CONF, 0x00) {
            return Err(Bmi270Error::InitFailed);
        }

        // 3. Prepare for firmware upload
        if !spi.write_register(REG_INIT_CTRL, 0x00) {
            return Err(Bmi270Error::FirmwareUploadFailed);
        }

        // 4. Upload firmware in bursts
        if !Self::upload_firmware(spi) {
            return Err(Bmi270Error::FirmwareUploadFailed);
        }

        // 5. Signal upload complete
        if !spi.write_register(REG_INIT_CTRL, 0x01) {
            return Err(Bmi270Error::FirmwareUploadFailed);
        }

        // 6. Configure: Accel 1600 Hz ODR, normal filter, +/-16 g
        let writes: [(u8, u8); 6] = [
            (REG_ACC_CONF, 0xAC),       // odr=1600Hz, bwp=normal, perf=high
            (REG_ACC_RANGE, 0x03),       // +/-16 g
            (REG_GYR_CONF, 0xA9),       // odr=800Hz, bwp=normal, perf=high
            (REG_GYR_RANGE, 0x00),      // +/-2000 dps
            (REG_FIFO_CONFIG_0, 0x02),  // FIFO watermark 2
            (REG_FIFO_CONFIG_1, 0x50),  // enable accel+gyro in FIFO
        ];
        for &(reg, val) in &writes {
            if !spi.write_register(reg, val) {
                return Err(Bmi270Error::InitFailed);
            }
        }

        // 7. Enable accel + gyro
        if !spi.write_register(REG_PWR_CTRL, 0x0E) {
            return Err(Bmi270Error::InitFailed);
        }

        self.initialised = true;
        Ok(())
    }

    /// Upload firmware blob via INIT_DATA register.
    fn upload_firmware(spi: &mut dyn SpiDevice) -> bool {
        // Real firmware is ~8 KB, uploaded in 32-byte chunks.
        // The write goes to INIT_DATA (burst), auto-incrementing internal address.
        for chunk in BMI270_FIRMWARE.chunks(32) {
            let mut buf = [0u8; 34]; // reg + up to 32 data bytes
            buf[0] = REG_INIT_DATA & 0x7F;
            buf[1..1 + chunk.len()].copy_from_slice(chunk);
            if !spi.write(&buf[..1 + chunk.len()]) {
                return false;
            }
        }
        true
    }

    /// Read accel + gyro from data registers (polled, not FIFO).
    pub fn read(&self, spi: &mut dyn SpiDevice) -> Result<Bmi270Sample, Bmi270Error> {
        if !self.initialised {
            return Err(Bmi270Error::InitFailed);
        }

        // 12 bytes: accel XYZ (6) + gyro XYZ (6) starting at 0x0C
        let mut buf = [0u8; 12];
        if !spi.read_registers(REG_DATA_8, &mut buf) {
            return Err(Bmi270Error::SpiFailed);
        }

        let ax = i16::from_le_bytes([buf[0], buf[1]]);
        let ay = i16::from_le_bytes([buf[2], buf[3]]);
        let az = i16::from_le_bytes([buf[4], buf[5]]);
        let gx = i16::from_le_bytes([buf[6], buf[7]]);
        let gy = i16::from_le_bytes([buf[8], buf[9]]);
        let gz = i16::from_le_bytes([buf[10], buf[11]]);

        Ok(Bmi270Sample {
            accel: Vec3::<Body>::new(
                ax as f32 * ACCEL_SCALE,
                ay as f32 * ACCEL_SCALE,
                az as f32 * ACCEL_SCALE,
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

    struct MockSpi {
        regs: heapless::Vec<(u8, u8), 32>,
        writes: heapless::Vec<(u8, u8), 64>,
        burst_written: bool,
    }

    impl MockSpi {
        fn new() -> Self {
            Self { regs: heapless::Vec::new(), writes: heapless::Vec::new(), burst_written: false }
        }
        fn set_reg(&mut self, reg: u8, val: u8) {
            for e in self.regs.iter_mut() { if e.0 == reg { e.1 = val; return; } }
            let _ = self.regs.push((reg, val));
        }
    }

    impl SpiDevice for MockSpi {
        fn transfer(&mut self, tx: &[u8], rx: &mut [u8]) -> bool {
            if tx.len() >= 2 && (tx[0] & 0x80) != 0 {
                let reg = tx[0] & 0x7F;
                if tx.len() > 2 {
                    rx[0] = 0;
                    for i in 1..tx.len() {
                        let r = reg + (i as u8 - 1);
                        rx[i] = self.regs.iter().find(|(a,_)| *a == r).map(|(_,v)| *v).unwrap_or(0);
                    }
                    return true;
                }
                rx[0] = 0;
                rx[1] = self.regs.iter().find(|(a,_)| *a == reg).map(|(_,v)| *v).unwrap_or(0);
                return true;
            }
            true
        }
        fn write(&mut self, data: &[u8]) -> bool {
            if data.len() >= 2 {
                let reg = data[0] & 0x7F;
                let val = data[1];
                let _ = self.writes.push((reg, val));
                self.set_reg(reg, val);
            }
            if data.len() > 2 { self.burst_written = true; }
            true
        }
        fn read(&mut self, data: &mut [u8]) -> bool { data.fill(0); true }
        fn set_speed(&mut self, _: u32) {}
        fn get_semaphore(&self) -> &dyn Semaphore { &DSEM }
        fn device_id(&self) -> u32 { 0 }
    }

    #[test]
    fn test_probe_success() {
        let mut spi = MockSpi::new();
        spi.set_reg(REG_CHIP_ID, 0x24);
        assert!(Bmi270::probe(&mut spi).is_ok());
    }

    #[test]
    fn test_probe_wrong_id() {
        let mut spi = MockSpi::new();
        spi.set_reg(REG_CHIP_ID, 0xFF);
        assert_eq!(Bmi270::probe(&mut spi).unwrap_err(), Bmi270Error::UnknownChipId(0xFF));
    }

    #[test]
    fn test_init_uploads_firmware() {
        let mut spi = MockSpi::new();
        spi.set_reg(REG_CHIP_ID, 0x24);
        let mut drv = Bmi270::new();
        drv.init(&mut spi).unwrap();
        assert!(spi.burst_written, "firmware should have been uploaded via burst write");
        assert!(drv.initialised);
    }

    #[test]
    fn test_accel_scale_1g() {
        // 2048 LSB at +/-16g = 1g -> 9.80665 m/s^2
        let val = 2048.0 * ACCEL_SCALE;
        assert!((val - 9.80665).abs() < 0.01);
    }

    #[test]
    fn test_read_rejects_uninitialised() {
        let mut spi = MockSpi::new();
        let drv = Bmi270::new();
        assert_eq!(drv.read(&mut spi).unwrap_err(), Bmi270Error::InitFailed);
    }
}

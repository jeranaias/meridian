//! Legacy Invensense v1 IMU driver: MPU6000, MPU9250, ICM-20608.
//!
//! WHO_AM_I register 0x75: MPU6000=0x68, MPU9250=0x71, ICM-20608=0xAF.
//! 14-byte burst read: accel(6) + temp(2) + gyro(6).
//! MPU9250 has AK8963 compass on auxiliary I2C bus (shared SPI semaphore).
//!
//! Still found on many F405-based flight controllers.

use meridian_hal::SpiDevice;
use meridian_math::{frames::Body, Vec3};
// Registers
const REG_WHO_AM_I: u8 = 0x75;
const REG_PWR_MGMT_1: u8 = 0x6B;
const REG_PWR_MGMT_2: u8 = 0x6C;
const REG_SIGNAL_PATH_RESET: u8 = 0x68;
const REG_CONFIG: u8 = 0x1A;
const REG_GYRO_CONFIG: u8 = 0x1B;
const REG_ACCEL_CONFIG: u8 = 0x1C;
const REG_SMPLRT_DIV: u8 = 0x19;
const REG_INT_PIN_CFG: u8 = 0x37;
const REG_ACCEL_XOUT_H: u8 = 0x3B;

/// 14 bytes: accel XYZ (6) + temp (2) + gyro XYZ (6).
const BURST_LEN: usize = 14;
// WHO_AM_I values
const WHOAMI_MPU6000: u8 = 0x68;
const WHOAMI_MPU9250: u8 = 0x71;
const WHOAMI_ICM20608: u8 = 0xAF;
// Scale factors (configured for +/-2000 dps, +/-16 g)
/// Gyro +/-2000 dps: 16.4 LSB/(deg/s)
const GYRO_SCALE: f32 = (1.0 / 16.4) * (core::f32::consts::PI / 180.0);

/// Accel +/-16 g: 2048 LSB/g
const ACCEL_SCALE: f32 = (1.0 / 2048.0) * 9.80665;

/// Temperature: deg C = raw / 340.0 + 36.53 (MPU6000/9250)
const TEMP_SENSITIVITY: f32 = 340.0;
const TEMP_OFFSET: f32 = 36.53;
// Types
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InvensenseVariant {
    Mpu6000,
    Mpu9250,
    Icm20608,
}

#[derive(Debug, Clone, Copy)]
pub struct InvensenseSample {
    pub accel: Vec3<Body>,
    pub gyro: Vec3<Body>,
    pub temperature: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InvensenseError {
    SpiFailed,
    UnknownDevice(u8),
    InitFailed,
}
// Driver
pub struct InvensenseV1 {
    variant: Option<InvensenseVariant>,
    initialised: bool,
}

impl InvensenseV1 {
    pub const fn new() -> Self {
        Self { variant: None, initialised: false }
    }

    /// Read WHO_AM_I and detect variant.
    pub fn probe(&mut self, spi: &mut dyn SpiDevice) -> Result<InvensenseVariant, InvensenseError> {
        let id = spi.read_register(REG_WHO_AM_I).ok_or(InvensenseError::SpiFailed)?;
        let variant = match id {
            WHOAMI_MPU6000 => InvensenseVariant::Mpu6000,
            WHOAMI_MPU9250 => InvensenseVariant::Mpu9250,
            WHOAMI_ICM20608 => InvensenseVariant::Icm20608,
            other => return Err(InvensenseError::UnknownDevice(other)),
        };
        self.variant = Some(variant);
        Ok(variant)
    }

    /// Full initialization sequence.
    pub fn init(&mut self, spi: &mut dyn SpiDevice) -> Result<(), InvensenseError> {
        let writes: [(u8, u8); 8] = [
            // 1. Reset
            (REG_PWR_MGMT_1, 0x80),
            // 2. Wake, use PLL with gyro X reference
            (REG_PWR_MGMT_1, 0x01),
            // 3. Enable all accel+gyro axes
            (REG_PWR_MGMT_2, 0x00),
            // 4. Signal path reset
            (REG_SIGNAL_PATH_RESET, 0x07),
            // 5. DLPF config: BW=41Hz (for 1kHz sample rate)
            (REG_CONFIG, 0x03),
            // 6. Gyro: +/-2000 dps
            (REG_GYRO_CONFIG, 0x18),
            // 7. Accel: +/-16 g
            (REG_ACCEL_CONFIG, 0x18),
            // 8. Sample rate divider = 0 -> 1 kHz
            (REG_SMPLRT_DIV, 0x00),
        ];

        for &(reg, val) in &writes {
            if !spi.write_register(reg, val) {
                return Err(InvensenseError::InitFailed);
            }
        }

        // For MPU9250: enable I2C master bypass so external host can access AK8963
        if self.variant == Some(InvensenseVariant::Mpu9250) {
            if !spi.write_register(REG_INT_PIN_CFG, 0x02) {
                return Err(InvensenseError::InitFailed);
            }
        }

        self.initialised = true;
        Ok(())
    }

    /// 14-byte burst read: accel(6) + temp(2) + gyro(6).
    pub fn read(&self, spi: &mut dyn SpiDevice) -> Result<InvensenseSample, InvensenseError> {
        if !self.initialised {
            return Err(InvensenseError::InitFailed);
        }

        let mut buf = [0u8; BURST_LEN];
        if !spi.read_registers(REG_ACCEL_XOUT_H, &mut buf) {
            return Err(InvensenseError::SpiFailed);
        }

        Ok(Self::parse_burst(&buf))
    }

    /// Parse a 14-byte burst buffer into a calibrated sample.
    pub fn parse_burst(buf: &[u8; BURST_LEN]) -> InvensenseSample {
        let ax = i16::from_be_bytes([buf[0], buf[1]]);
        let ay = i16::from_be_bytes([buf[2], buf[3]]);
        let az = i16::from_be_bytes([buf[4], buf[5]]);
        let temp_raw = i16::from_be_bytes([buf[6], buf[7]]);
        let gx = i16::from_be_bytes([buf[8], buf[9]]);
        let gy = i16::from_be_bytes([buf[10], buf[11]]);
        let gz = i16::from_be_bytes([buf[12], buf[13]]);

        InvensenseSample {
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
            temperature: temp_raw as f32 / TEMP_SENSITIVITY + TEMP_OFFSET,
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

    struct MockSpi {
        regs: heapless::Vec<(u8, u8), 32>,
        writes: heapless::Vec<(u8, u8), 32>,
    }
    impl MockSpi {
        fn new() -> Self { Self { regs: heapless::Vec::new(), writes: heapless::Vec::new() } }
        fn set_reg(&mut self, reg: u8, val: u8) {
            for e in self.regs.iter_mut() { if e.0 == reg { e.1 = val; return; } }
            let _ = self.regs.push((reg, val));
        }
    }
    impl SpiDevice for MockSpi {
        fn transfer(&mut self, tx: &[u8], rx: &mut [u8]) -> bool {
            if !tx.is_empty() && (tx[0] & 0x80) != 0 {
                let reg = tx[0] & 0x7F;
                rx[0] = 0;
                for i in 1..tx.len() {
                    let r = reg + (i as u8 - 1);
                    rx[i] = self.regs.iter().find(|(a,_)| *a == r).map(|(_,v)| *v).unwrap_or(0);
                }
            }
            true
        }
        fn write(&mut self, data: &[u8]) -> bool {
            if data.len() >= 2 { let _ = self.writes.push((data[0] & 0x7F, data[1])); }
            true
        }
        fn read(&mut self, data: &mut [u8]) -> bool { data.fill(0); true }
        fn set_speed(&mut self, _: u32) {}
        fn get_semaphore(&self) -> &dyn Semaphore { &DSEM }
        fn device_id(&self) -> u32 { 0 }
    }

    #[test]
    fn test_probe_mpu6000() {
        let mut spi = MockSpi::new();
        spi.set_reg(REG_WHO_AM_I, WHOAMI_MPU6000);
        let mut drv = InvensenseV1::new();
        assert_eq!(drv.probe(&mut spi).unwrap(), InvensenseVariant::Mpu6000);
    }

    #[test]
    fn test_probe_mpu9250() {
        let mut spi = MockSpi::new();
        spi.set_reg(REG_WHO_AM_I, WHOAMI_MPU9250);
        let mut drv = InvensenseV1::new();
        assert_eq!(drv.probe(&mut spi).unwrap(), InvensenseVariant::Mpu9250);
    }

    #[test]
    fn test_probe_icm20608() {
        let mut spi = MockSpi::new();
        spi.set_reg(REG_WHO_AM_I, WHOAMI_ICM20608);
        let mut drv = InvensenseV1::new();
        assert_eq!(drv.probe(&mut spi).unwrap(), InvensenseVariant::Icm20608);
    }

    #[test]
    fn test_probe_unknown() {
        let mut spi = MockSpi::new();
        spi.set_reg(REG_WHO_AM_I, 0x00);
        let mut drv = InvensenseV1::new();
        assert_eq!(drv.probe(&mut spi).unwrap_err(), InvensenseError::UnknownDevice(0x00));
    }

    #[test]
    fn test_parse_burst_1g_z() {
        // Accel Z = 2048 (big-endian at bytes 4-5), rest zero
        let mut buf = [0u8; BURST_LEN];
        let az_be = 2048i16.to_be_bytes();
        buf[4] = az_be[0];
        buf[5] = az_be[1];

        let sample = InvensenseV1::parse_burst(&buf);
        assert!((sample.accel.z - 9.80665).abs() < 0.01);
        assert!(sample.accel.x.abs() < 0.001);
        assert!(sample.gyro.x.abs() < 0.001);
    }

    #[test]
    fn test_parse_burst_temperature() {
        // temp raw = 0 -> 0/340 + 36.53 = 36.53
        let buf = [0u8; BURST_LEN];
        let sample = InvensenseV1::parse_burst(&buf);
        assert!((sample.temperature - TEMP_OFFSET).abs() < 0.01);
    }

    #[test]
    fn test_mpu9250_enables_bypass() {
        let mut spi = MockSpi::new();
        spi.set_reg(REG_WHO_AM_I, WHOAMI_MPU9250);
        let mut drv = InvensenseV1::new();
        drv.probe(&mut spi).unwrap();
        drv.init(&mut spi).unwrap();
        // INT_PIN_CFG should have bypass bit
        let bypass = spi.writes.iter().find(|(r,_)| *r == REG_INT_PIN_CFG);
        assert!(bypass.is_some(), "MPU9250 should write INT_PIN_CFG for I2C bypass");
        assert_eq!(bypass.unwrap().1, 0x02);
    }

    #[test]
    fn test_read_rejects_uninitialised() {
        let mut spi = MockSpi::new();
        let drv = InvensenseV1::new();
        assert_eq!(drv.read(&mut spi).unwrap_err(), InvensenseError::InitFailed);
    }
}

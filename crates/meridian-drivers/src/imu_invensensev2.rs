//! ICM-20948 / ICM-20649 / ICM-20648 IMU driver (Invensensev2 family).
//!
//! Stub driver for the second-generation Invensense chips. The ICM-20948 is
//! found on many H7-based flight controllers and includes an AK09916 compass
//! on an auxiliary I2C bus.
//!
//! ArduPilot reference: `AP_InertialSensor_Invensensev2.cpp`

use meridian_hal::SpiDevice;
use meridian_math::{frames::Body, Vec3};

// ---------------------------------------------------------------------------
// Register addresses (Bank 0)
// ---------------------------------------------------------------------------

const REG_WHO_AM_I: u8 = 0x00;
const REG_USER_CTRL: u8 = 0x03;
const REG_LP_CONFIG: u8 = 0x05;
const REG_PWR_MGMT_1: u8 = 0x06;
const REG_PWR_MGMT_2: u8 = 0x07;
const REG_INT_PIN_CFG: u8 = 0x0F;
const REG_INT_ENABLE_1: u8 = 0x11;
const REG_FIFO_EN_2: u8 = 0x67;
const REG_FIFO_RST: u8 = 0x68;
const REG_FIFO_MODE: u8 = 0x69;
const REG_FIFO_COUNTH: u8 = 0x70;
const REG_FIFO_R_W: u8 = 0x72;
const REG_BANK_SEL: u8 = 0x7F;

// Bank 2 registers
const REG_GYRO_SMPLRT_DIV: u8 = 0x00;
const REG_GYRO_CONFIG_1: u8 = 0x01;
const REG_ACCEL_SMPLRT_DIV_2: u8 = 0x11;
const REG_ACCEL_CONFIG: u8 = 0x14;

// ---------------------------------------------------------------------------
// WHO_AM_I values
// ---------------------------------------------------------------------------

const WHOAMI_ICM20948: u8 = 0xEA;
const WHOAMI_ICM20649: u8 = 0xE1;
const WHOAMI_ICM20648: u8 = 0xE0;

// ---------------------------------------------------------------------------
// FIFO packet constants
// ---------------------------------------------------------------------------

/// FIFO packet: 6 accel + 6 gyro + 2 temp = 14 bytes (no header byte).
const FIFO_PACKET_SIZE: usize = 14;
const MAX_FIFO_PACKETS: usize = 32;

// ---------------------------------------------------------------------------
// Scale factors
// ---------------------------------------------------------------------------

/// Gyro full-scale +-2000 dps: 16.4 LSB/(deg/s).
const GYRO_SCALE: f32 = (1.0 / 16.4) * (core::f32::consts::PI / 180.0);

/// Accel full-scale +-16 g: 2048 LSB/g.
const ACCEL_SCALE: f32 = (1.0 / 2048.0) * 9.80665;

/// Temperature: deg C = (raw - 21) / 333.87 + 21.0
const TEMP_SENSITIVITY: f32 = 333.87;
const TEMP_OFFSET: f32 = 21.0;
const TEMP_RAW_OFFSET: f32 = 21.0;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// One IMU measurement from the Invensensev2 family.
#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    pub accel: Vec3<Body>,
    pub gyro: Vec3<Body>,
    pub temperature: f32,
}

/// Detected device variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Icm209xxVariant {
    Icm20948,
    Icm20649,
    Icm20648,
}

/// Driver errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Icm209xxError {
    SpiFailed,
    UnknownDevice(u8),
    InitFailed,
    BankSwitchFailed,
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

/// ICM-20948 / ICM-20649 / ICM-20648 SPI driver.
///
/// Implements bank-switching for the multi-bank register layout used by
/// this chip family. FIFO reads accel + gyro data at 1 kHz.
pub struct Icm209xx {
    variant: Option<Icm209xxVariant>,
    initialised: bool,
    current_bank: u8,
}

impl Icm209xx {
    pub const fn new() -> Self {
        Self {
            variant: None,
            initialised: false,
            current_bank: 0,
        }
    }

    /// Switch to the specified register bank (0-3).
    fn select_bank(&mut self, spi: &mut dyn SpiDevice, bank: u8) -> Result<(), Icm209xxError> {
        if self.current_bank == bank {
            return Ok(());
        }
        if !spi.write_register(REG_BANK_SEL, bank << 4) {
            return Err(Icm209xxError::BankSwitchFailed);
        }
        self.current_bank = bank;
        Ok(())
    }

    /// Read WHO_AM_I and identify the variant.
    pub fn probe(&mut self, spi: &mut dyn SpiDevice) -> Result<Icm209xxVariant, Icm209xxError> {
        self.select_bank(spi, 0)?;
        let whoami = spi.read_register(REG_WHO_AM_I).ok_or(Icm209xxError::SpiFailed)?;
        let variant = match whoami {
            WHOAMI_ICM20948 => Icm209xxVariant::Icm20948,
            WHOAMI_ICM20649 => Icm209xxVariant::Icm20649,
            WHOAMI_ICM20648 => Icm209xxVariant::Icm20648,
            other => return Err(Icm209xxError::UnknownDevice(other)),
        };
        self.variant = Some(variant);
        Ok(variant)
    }

    /// Full initialisation sequence matching ArduPilot's Invensensev2 backend.
    ///
    /// The caller must delay 10 ms after this returns for the soft reset to
    /// complete, then call `configure()` to set up FIFO and ODR.
    pub fn init(&mut self, spi: &mut dyn SpiDevice) -> Result<(), Icm209xxError> {
        self.select_bank(spi, 0)?;

        // 1. Soft reset
        self.write_checked(spi, REG_PWR_MGMT_1, 0x80)?; // DEVICE_RESET
        // Caller must delay 10 ms here

        // 2. Wake up — auto-select best clock
        self.write_checked(spi, REG_PWR_MGMT_1, 0x01)?; // CLKSEL = auto

        // 3. Enable all accel + gyro axes
        self.write_checked(spi, REG_PWR_MGMT_2, 0x00)?;

        // 4. Configure gyro (Bank 2)
        self.select_bank(spi, 2)?;
        // Gyro: +-2000 dps, DLPF enabled, BW ~197 Hz (GYRO_DLPFCFG=0)
        self.write_checked(spi, REG_GYRO_CONFIG_1, 0x07)?; // FS=3 (2000dps), DLPF_EN=1
        self.write_checked(spi, REG_GYRO_SMPLRT_DIV, 0x00)?; // 1125 / (1+0) = 1125 Hz

        // 5. Configure accel (Bank 2)
        // Accel: +-16g, DLPF enabled, BW ~246 Hz
        self.write_checked(spi, REG_ACCEL_CONFIG, 0x07)?; // FS=3 (16g), DLPF_EN=1
        self.write_checked(spi, REG_ACCEL_SMPLRT_DIV_2, 0x00)?; // 1125 / (1+0) = 1125 Hz

        // 6. Back to Bank 0 for FIFO config
        self.select_bank(spi, 0)?;

        // 7. Reset FIFO
        self.write_checked(spi, REG_FIFO_RST, 0x1F)?;
        self.write_checked(spi, REG_FIFO_RST, 0x00)?;

        // 8. FIFO mode: stream (overwrite oldest on overflow)
        self.write_checked(spi, REG_FIFO_MODE, 0x00)?;

        // 9. Enable accel + gyro into FIFO
        self.write_checked(spi, REG_FIFO_EN_2, 0x1E)?; // ACCEL_FIFO_EN + GYRO_XYZ_FIFO_EN

        // 10. Enable FIFO in user control
        self.write_checked(spi, REG_USER_CTRL, 0x40)?; // FIFO_EN

        // 11. Configure interrupt (data ready → INT1, push-pull, active-high)
        self.write_checked(spi, REG_INT_PIN_CFG, 0x10)?;
        self.write_checked(spi, REG_INT_ENABLE_1, 0x01)?; // RAW_DATA_0_RDY_EN

        self.initialised = true;
        Ok(())
    }

    fn write_checked(&mut self, spi: &mut dyn SpiDevice, reg: u8, val: u8) -> Result<(), Icm209xxError> {
        if !spi.write_register(reg, val) {
            return Err(Icm209xxError::InitFailed);
        }
        Ok(())
    }

    /// Drain FIFO packets and return parsed IMU samples.
    pub fn read_fifo(
        &self,
        spi: &mut dyn SpiDevice,
        out: &mut [ImuSample],
    ) -> Result<usize, Icm209xxError> {
        if !self.initialised {
            return Err(Icm209xxError::InitFailed);
        }

        let mut cnt_buf = [0u8; 2];
        if !spi.read_registers(REG_FIFO_COUNTH, &mut cnt_buf) {
            return Err(Icm209xxError::SpiFailed);
        }
        let byte_count = u16::from_be_bytes(cnt_buf) as usize;
        let packet_count = byte_count / FIFO_PACKET_SIZE;
        let to_read = packet_count.min(out.len()).min(MAX_FIFO_PACKETS);

        if to_read == 0 {
            return Ok(0);
        }

        let total_bytes = to_read * FIFO_PACKET_SIZE;
        let mut fifo_buf = [0u8; MAX_FIFO_PACKETS * FIFO_PACKET_SIZE];
        if !spi.read_registers(REG_FIFO_R_W, &mut fifo_buf[..total_bytes]) {
            return Err(Icm209xxError::SpiFailed);
        }

        let mut written = 0;
        for i in 0..to_read {
            let offset = i * FIFO_PACKET_SIZE;
            let pkt = &fifo_buf[offset..offset + FIFO_PACKET_SIZE];
            out[written] = Self::parse_fifo_packet(pkt);
            written += 1;
        }

        Ok(written)
    }

    /// Parse a 14-byte FIFO packet: [accel_xyz(6 BE)] [gyro_xyz(6 BE)] [temp(2 BE)]
    pub fn parse_fifo_packet(pkt: &[u8]) -> ImuSample {
        let ax = i16::from_be_bytes([pkt[0], pkt[1]]);
        let ay = i16::from_be_bytes([pkt[2], pkt[3]]);
        let az = i16::from_be_bytes([pkt[4], pkt[5]]);
        let gx = i16::from_be_bytes([pkt[6], pkt[7]]);
        let gy = i16::from_be_bytes([pkt[8], pkt[9]]);
        let gz = i16::from_be_bytes([pkt[10], pkt[11]]);
        let temp_raw = i16::from_be_bytes([pkt[12], pkt[13]]);

        let temperature = (temp_raw as f32 - TEMP_RAW_OFFSET) / TEMP_SENSITIVITY + TEMP_OFFSET;

        ImuSample {
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
            temperature,
        }
    }

    /// Whether the driver has been initialised.
    pub fn is_initialised(&self) -> bool {
        self.initialised
    }

    /// Get the detected variant.
    pub fn variant(&self) -> Option<Icm209xxVariant> {
        self.variant
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_fifo_packet_known_values() {
        // 1g on Z accel = 2048 LSB
        let mut pkt = [0u8; 14];
        pkt[4..6].copy_from_slice(&2048i16.to_be_bytes()); // az = +1g
        pkt[6..8].copy_from_slice(&1640i16.to_be_bytes()); // gx = ~100 dps
        pkt[12..14].copy_from_slice(&21i16.to_be_bytes()); // temp raw = 21 -> 21.0 C

        let sample = Icm209xx::parse_fifo_packet(&pkt);
        assert!((sample.accel.z - 9.80665).abs() < 0.01);
        let expected_gx = 100.0 * core::f32::consts::PI / 180.0;
        assert!((sample.gyro.x - expected_gx).abs() < 0.1);
    }
}

//! LSM6DSV IMU driver — new ST 6-axis MEMS appearing on modern FCs.
//!
//! ArduPilot reference: `AP_InertialSensor_LSM6DSV.cpp`
//!
//! The LSM6DSV supports SPI up to 10 MHz and provides a hardware FIFO.
//! This driver reads accel + gyro from the FIFO in continuous mode.

use meridian_hal::SpiDevice;
use meridian_math::{frames::Body, Vec3};

// ---------------------------------------------------------------------------
// Register addresses
// ---------------------------------------------------------------------------

const REG_WHO_AM_I: u8 = 0x0F;
const REG_CTRL1: u8 = 0x10; // Accel ODR + FS
const REG_CTRL2: u8 = 0x11; // Gyro ODR + FS
const REG_CTRL3: u8 = 0x12; // BDU, IF_INC, SW_RESET
const REG_CTRL6: u8 = 0x15; // Accel high-perf mode
const REG_CTRL7: u8 = 0x16; // Gyro high-perf mode
const REG_FIFO_CTRL1: u8 = 0x07; // FIFO watermark low
const REG_FIFO_CTRL2: u8 = 0x08; // FIFO watermark high
const REG_FIFO_CTRL3: u8 = 0x09; // Accel/Gyro batch decimation
const REG_FIFO_CTRL4: u8 = 0x0A; // FIFO mode
const REG_FIFO_STATUS1: u8 = 0x1B;
const REG_FIFO_DATA_OUT_TAG: u8 = 0x78;

const WHOAMI_LSM6DSV: u8 = 0x70;

// ---------------------------------------------------------------------------
// FIFO tag values
// ---------------------------------------------------------------------------

const FIFO_TAG_GYRO: u8 = 0x01;
const FIFO_TAG_ACCEL: u8 = 0x02;
const FIFO_TAG_TEMP: u8 = 0x03;

/// FIFO word: 1 tag byte + 6 data bytes = 7 bytes.
const FIFO_WORD_SIZE: usize = 7;
const MAX_FIFO_WORDS: usize = 64;

// ---------------------------------------------------------------------------
// Scale factors
// ---------------------------------------------------------------------------

/// Gyro +-2000 dps: 70.0 mdps/LSB
const GYRO_SCALE: f32 = 70.0e-3 * (core::f32::consts::PI / 180.0);

/// Accel +-16g: 0.488 mg/LSB
const ACCEL_SCALE: f32 = 0.488e-3 * 9.80665;

/// Temperature: 256 LSB/°C, offset 25°C
const TEMP_SCALE: f32 = 1.0 / 256.0;
const TEMP_OFFSET: f32 = 25.0;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// One IMU measurement.
#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    pub accel: Vec3<Body>,
    pub gyro: Vec3<Body>,
    pub temperature: f32,
}

/// Driver errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Lsm6dsvError {
    SpiFailed,
    UnknownDevice(u8),
    InitFailed,
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

/// LSM6DSV SPI driver.
pub struct Lsm6dsv {
    initialised: bool,
}

impl Lsm6dsv {
    pub const fn new() -> Self {
        Self { initialised: false }
    }

    /// Probe: read WHO_AM_I.
    pub fn probe(&self, spi: &mut dyn SpiDevice) -> Result<(), Lsm6dsvError> {
        let whoami = spi.read_register(REG_WHO_AM_I).ok_or(Lsm6dsvError::SpiFailed)?;
        if whoami == WHOAMI_LSM6DSV {
            Ok(())
        } else {
            Err(Lsm6dsvError::UnknownDevice(whoami))
        }
    }

    /// Full initialisation: soft reset, configure ODR/FS, enable FIFO.
    pub fn init(&mut self, spi: &mut dyn SpiDevice) -> Result<(), Lsm6dsvError> {
        // 1. Software reset
        self.write_checked(spi, REG_CTRL3, 0x05)?; // SW_RESET + IF_INC
        // Caller should delay 10 ms after this

        // 2. BDU enable + IF_INC
        self.write_checked(spi, REG_CTRL3, 0x44)?; // BDU + IF_INC

        // 3. Accel: ODR 1.92 kHz, +-16g
        self.write_checked(spi, REG_CTRL1, 0x7C)?; // ODR=0111, FS=11

        // 4. Gyro: ODR 1.92 kHz, +-2000 dps
        self.write_checked(spi, REG_CTRL2, 0x7C)?;

        // 5. High-performance modes
        self.write_checked(spi, REG_CTRL6, 0x00)?;
        self.write_checked(spi, REG_CTRL7, 0x00)?;

        // 6. FIFO config: continuous mode, accel+gyro batch rate = ODR
        self.write_checked(spi, REG_FIFO_CTRL3, 0x77)?; // Accel+Gyro BDR = ODR
        self.write_checked(spi, REG_FIFO_CTRL4, 0x06)?; // Continuous mode
        self.write_checked(spi, REG_FIFO_CTRL1, 0x01)?; // Watermark = 1
        self.write_checked(spi, REG_FIFO_CTRL2, 0x00)?;

        self.initialised = true;
        Ok(())
    }

    fn write_checked(&self, spi: &mut dyn SpiDevice, reg: u8, val: u8) -> Result<(), Lsm6dsvError> {
        if !spi.write_register(reg, val) {
            return Err(Lsm6dsvError::InitFailed);
        }
        Ok(())
    }

    /// Read FIFO and return paired accel+gyro samples.
    pub fn read_fifo(
        &self,
        spi: &mut dyn SpiDevice,
        out: &mut [ImuSample],
    ) -> Result<usize, Lsm6dsvError> {
        if !self.initialised {
            return Err(Lsm6dsvError::InitFailed);
        }

        // Read FIFO status
        let mut status_buf = [0u8; 2];
        if !spi.read_registers(REG_FIFO_STATUS1, &mut status_buf) {
            return Err(Lsm6dsvError::SpiFailed);
        }
        let fifo_count = (u16::from_le_bytes(status_buf) & 0x01FF) as usize;
        let to_read = fifo_count.min(MAX_FIFO_WORDS);

        let mut written = 0;
        let mut pending_accel: Option<Vec3<Body>> = None;
        let mut pending_gyro: Option<Vec3<Body>> = None;
        let mut last_temp: f32 = TEMP_OFFSET;

        for _ in 0..to_read {
            let mut word = [0u8; FIFO_WORD_SIZE];
            if !spi.read_registers(REG_FIFO_DATA_OUT_TAG, &mut word) {
                break;
            }
            let tag = (word[0] >> 3) & 0x1F;
            let data = &word[1..7];

            match tag {
                FIFO_TAG_ACCEL => {
                    let ax = i16::from_le_bytes([data[0], data[1]]);
                    let ay = i16::from_le_bytes([data[2], data[3]]);
                    let az = i16::from_le_bytes([data[4], data[5]]);
                    pending_accel = Some(Vec3::<Body>::new(
                        ax as f32 * ACCEL_SCALE,
                        ay as f32 * ACCEL_SCALE,
                        az as f32 * ACCEL_SCALE,
                    ));
                }
                FIFO_TAG_GYRO => {
                    let gx = i16::from_le_bytes([data[0], data[1]]);
                    let gy = i16::from_le_bytes([data[2], data[3]]);
                    let gz = i16::from_le_bytes([data[4], data[5]]);
                    pending_gyro = Some(Vec3::<Body>::new(
                        gx as f32 * GYRO_SCALE,
                        gy as f32 * GYRO_SCALE,
                        gz as f32 * GYRO_SCALE,
                    ));
                }
                FIFO_TAG_TEMP => {
                    let raw = i16::from_le_bytes([data[0], data[1]]);
                    last_temp = raw as f32 * TEMP_SCALE + TEMP_OFFSET;
                }
                _ => {} // skip unknown tags
            }

            // Emit a sample when we have both accel and gyro
            if let (Some(accel), Some(gyro)) = (pending_accel, pending_gyro) {
                if written < out.len() {
                    out[written] = ImuSample { accel, gyro, temperature: last_temp };
                    written += 1;
                }
                pending_accel = None;
                pending_gyro = None;
            }
        }

        Ok(written)
    }

    pub fn is_initialised(&self) -> bool {
        self.initialised
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_accel_scale() {
        // 1g at +-16g full scale: raw ≈ 2048
        let raw = 2048i16;
        let ms2 = raw as f32 * ACCEL_SCALE;
        assert!((ms2 - 9.80665).abs() < 0.1, "1g = {} m/s^2", ms2);
    }

    #[test]
    fn test_gyro_scale() {
        // 100 dps at +-2000 dps: raw = 100 / 0.070 ≈ 1429
        let raw = 1429i16;
        let rads = raw as f32 * GYRO_SCALE;
        let expected = 100.0 * core::f32::consts::PI / 180.0;
        assert!((rads - expected).abs() < 0.1, "100 dps = {} rad/s", rads);
    }
}

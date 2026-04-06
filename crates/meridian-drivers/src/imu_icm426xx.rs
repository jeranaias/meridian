//! ICM-42688-P / ICM-42688-V / ICM-42605 / ICM-42670 IMU driver.
//!
//! Invensensev3 family SPI driver. Reads accelerometer and gyroscope data
//! from the on-chip FIFO in 16-byte packets. The driver is `no_std`
//! compatible and borrows the SPI bus rather than owning it.
//!
//! **Critical**: AFSR (Anti-Flicker Suppression Resonance) must be disabled
//! at init by writing INTF_CONFIG1[6:5] = 0b10, otherwise the gyro stalls
//! for ~2 ms near 100 deg/s crossing.

use meridian_hal::SpiDevice;
use meridian_math::{frames::Body, Vec3};

// ---------------------------------------------------------------------------
// Register addresses
// ---------------------------------------------------------------------------

const REG_PWR_MGMT0: u8 = 0x4E;
const REG_GYRO_CONFIG0: u8 = 0x4F;
const REG_ACCEL_CONFIG0: u8 = 0x50;
const REG_GYRO_CONFIG1: u8 = 0x51;
const REG_GYRO_ACCEL_CONFIG0: u8 = 0x52;
const REG_FIFO_CONFIG: u8 = 0x16;
const REG_FIFO_CONFIG1: u8 = 0x5F;
const REG_FIFO_CONFIG2: u8 = 0x60;
const REG_FIFO_CONFIG3: u8 = 0x61;
const REG_INTF_CONFIG0: u8 = 0x4C;
const REG_INTF_CONFIG1: u8 = 0x4D;
const REG_INT_CONFIG: u8 = 0x14;
const REG_WHO_AM_I: u8 = 0x75;
const REG_FIFO_COUNTH: u8 = 0x2E;
const REG_FIFO_DATA: u8 = 0x30;
const REG_SIGNAL_PATH_RESET: u8 = 0x4B;
const REG_DEVICE_CONFIG: u8 = 0x11;
const REG_ACCEL_CONFIG1: u8 = 0x53;
const REG_BANK_SEL: u8 = 0x76;

// Bank 1 registers (AAF configuration)
const REG_GYRO_CONFIG_STATIC2: u8 = 0x0B;
const REG_GYRO_CONFIG_STATIC3: u8 = 0x0C;
const REG_GYRO_CONFIG_STATIC4: u8 = 0x0D;
const REG_GYRO_CONFIG_STATIC5: u8 = 0x0E;

// Bank 2 registers (Accel AAF)
const REG_ACCEL_CONFIG_STATIC2: u8 = 0x03;
const REG_ACCEL_CONFIG_STATIC3: u8 = 0x04;
const REG_ACCEL_CONFIG_STATIC4: u8 = 0x05;

// ---------------------------------------------------------------------------
// WHO_AM_I values for supported parts
// ---------------------------------------------------------------------------

/// ICM-42688-P
const WHOAMI_ICM42688: u8 = 0x47;
/// ICM-42688-V (low-noise variant)
const WHOAMI_ICM42688V: u8 = 0xDB;
/// ICM-42605
const WHOAMI_ICM42605: u8 = 0x42;
/// ICM-42670
const WHOAMI_ICM42670: u8 = 0x67;

// ---------------------------------------------------------------------------
// FIFO packet constants
// ---------------------------------------------------------------------------

/// Expected FIFO packet header byte (accel + gyro + temp + timestamp).
const FIFO_HEADER: u8 = 0x68;
/// Total bytes per FIFO packet (header + 6 accel + 6 gyro + 1 temp + 2 ts).
const FIFO_PACKET_SIZE: usize = 16;
/// Maximum number of FIFO packets we will drain per call.
const MAX_FIFO_PACKETS: usize = 32;

// ---------------------------------------------------------------------------
// Scale factors
// ---------------------------------------------------------------------------

/// Gyro full-scale +-2000 dps: 16.4 LSB/(deg/s).
/// Multiply raw value by this to get rad/s.
const GYRO_SCALE: f32 = (1.0 / 16.4) * (core::f32::consts::PI / 180.0);

/// Accel full-scale +-16 g: 2048 LSB/g.
/// Multiply raw value by this to get m/s^2.
const ACCEL_SCALE: f32 = (1.0 / 2048.0) * 9.80665;

/// Temperature: degrees C = (raw / 2.07) + 25.0.
/// Raw is a signed 8-bit value from the FIFO packet.
const TEMP_SENSITIVITY: f32 = 2.07;
const TEMP_OFFSET: f32 = 25.0;

// ---------------------------------------------------------------------------
// AFSR disable mask
// ---------------------------------------------------------------------------

/// INTF_CONFIG1 bits [6:5] = 0b10 disables AFSR.
const AFSR_DISABLE_MASK: u8 = 0b0100_0000;
/// Bits [6:5] cleared before OR.
const INTF_CONFIG1_AFSR_CLEAR: u8 = 0b1001_1111;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// One IMU measurement: accelerometer, gyroscope, and die temperature.
#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    /// Acceleration in body frame [m/s^2].
    pub accel: Vec3<Body>,
    /// Angular rate in body frame [rad/s].
    pub gyro: Vec3<Body>,
    /// Die temperature [deg C].
    pub temperature: f32,
    /// FIFO timestamp tick (raw 16-bit, wraps).
    pub timestamp_raw: u16,
}

/// Detected device variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Icm426xxVariant {
    Icm42688,
    Icm42688V,
    Icm42605,
    Icm42670,
}

/// Errors the driver can return.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Icm426xxError {
    /// SPI transfer failed.
    SpiFailed,
    /// WHO_AM_I read returned an unrecognised value.
    UnknownDevice(u8),
    /// A register write failed during init.
    InitFailed,
}

// ---------------------------------------------------------------------------
// Driver struct
// ---------------------------------------------------------------------------

/// ICM-426xx family SPI driver.
pub struct Icm426xx {
    variant: Option<Icm426xxVariant>,
    initialised: bool,
}

impl Icm426xx {
    /// Create a new, uninitialised driver instance.
    pub const fn new() -> Self {
        Self {
            variant: None,
            initialised: false,
        }
    }

    // -- probe ---------------------------------------------------------------

    /// Read WHO_AM_I and determine the device variant.
    pub fn probe(
        &mut self,
        spi: &mut dyn SpiDevice,
    ) -> Result<Icm426xxVariant, Icm426xxError> {
        let whoami = spi.read_register(REG_WHO_AM_I).ok_or(Icm426xxError::SpiFailed)?;
        let variant = match whoami {
            WHOAMI_ICM42688 => Icm426xxVariant::Icm42688,
            WHOAMI_ICM42688V => Icm426xxVariant::Icm42688V,
            WHOAMI_ICM42605 => Icm426xxVariant::Icm42605,
            WHOAMI_ICM42670 => Icm426xxVariant::Icm42670,
            other => return Err(Icm426xxError::UnknownDevice(other)),
        };
        self.variant = Some(variant);
        Ok(variant)
    }

    // -- init ----------------------------------------------------------------

    /// Full initialisation sequence matching ArduPilot's Invensensev3 backend.
    ///
    /// The caller must have previously called [`probe`] successfully.
    /// After this returns `Ok(())` the FIFO is streaming accel+gyro
    /// at 1 kHz / +-2000 dps / +-16 g.
    ///
    /// A 300 us delay is needed after writing PWR_MGMT0 (final step).
    /// In a `no_std` environment the caller should `delay_us(300)` after
    /// this function returns (we cannot block here without a HAL timer).
    ///
    /// Changes from original 12-step init:
    /// - Added SIGNAL_PATH_RESET to flush stale FIFO data (ArduPilot step 1)
    /// - Added DEVICE_CONFIG soft reset (ArduPilot step 2)
    /// - Added Bank 1/2 AAF configuration (ArduPilot steps 6-8)
    /// - Added accel UI filter order (ArduPilot step 13)
    pub fn init(
        &mut self,
        spi: &mut dyn SpiDevice,
    ) -> Result<(), Icm426xxError> {
        // WH2 fix: Soft reset must happen BEFORE signal path reset.
        // ArduPilot order: DEVICE_CONFIG reset → 10ms delay → SIGNAL_PATH_RESET
        // A soft reset clears all registers to defaults; doing it after config
        // writes would undo them. After reset, flush the FIFO.

        // Step 1: DEVICE_CONFIG soft reset (ArduPilot writes 0x11 = 0x01)
        // Caller must delay 10 ms after this write.
        if !spi.write_register(REG_DEVICE_CONFIG, 0x01) {
            return Err(Icm426xxError::InitFailed);
        }

        // Step 2: SIGNAL_PATH_RESET — flush FIFO after reset
        if !spi.write_register(REG_SIGNAL_PATH_RESET, 0x02) {
            return Err(Icm426xxError::InitFailed);
        }

        // Step 3: INTF_CONFIG0 — BE endian + count-by-records
        // 0xC0 = bit 7 (fifo count in records) + bit 6 (big-endian fifo count).
        // Sensor data is big-endian (default). Parsers use from_be_bytes.
        // Matches ArduPilot: register_write(INV3REG_INTF_CONFIG0, 0xC0)
        if !spi.write_register(REG_INTF_CONFIG0, 0xC0) {
            return Err(Icm426xxError::InitFailed);
        }

        // Step 4: INTF_CONFIG1 — AFSR disable (read-modify-write)
        let intf1 = spi
            .read_register(REG_INTF_CONFIG1)
            .ok_or(Icm426xxError::SpiFailed)?;
        let intf1_new = (intf1 & INTF_CONFIG1_AFSR_CLEAR) | AFSR_DISABLE_MASK;
        if !spi.write_register(REG_INTF_CONFIG1, intf1_new) {
            return Err(Icm426xxError::InitFailed);
        }

        // Steps 5-7: Bank 1 — Gyro AAF configuration
        // AAF DELT=6, DELTSQR=36, BITSHIFT=10 → ~536 Hz 3dB BW at 1 kHz ODR
        if !Self::write_bank(spi, 1, REG_GYRO_CONFIG_STATIC2, 0x00) { // AAF_DIS = 0 (enabled)
            return Err(Icm426xxError::InitFailed);
        }
        if !Self::write_bank(spi, 1, REG_GYRO_CONFIG_STATIC3, 6) { // DELT = 6
            return Err(Icm426xxError::InitFailed);
        }
        if !Self::write_bank(spi, 1, REG_GYRO_CONFIG_STATIC4, 36) { // DELTSQR low
            return Err(Icm426xxError::InitFailed);
        }
        if !Self::write_bank(spi, 1, REG_GYRO_CONFIG_STATIC5, 10) { // BITSHIFT = 10
            return Err(Icm426xxError::InitFailed);
        }

        // Step 8: Bank 2 — Accel AAF configuration (same coefficients)
        if !Self::write_bank(spi, 2, REG_ACCEL_CONFIG_STATIC2, 6) { // DELT = 6
            return Err(Icm426xxError::InitFailed);
        }
        if !Self::write_bank(spi, 2, REG_ACCEL_CONFIG_STATIC3, 36) { // DELTSQR
            return Err(Icm426xxError::InitFailed);
        }
        if !Self::write_bank(spi, 2, REG_ACCEL_CONFIG_STATIC4, 10) { // BITSHIFT
            return Err(Icm426xxError::InitFailed);
        }

        // Return to bank 0
        let _ = spi.write_register(REG_BANK_SEL, 0x00);

        // Step 9: GYRO_CONFIG0 — 2000 dps + 1 kHz ODR
        if !spi.write_register(REG_GYRO_CONFIG0, 0x06) {
            return Err(Icm426xxError::InitFailed);
        }

        // Step 10: ACCEL_CONFIG0 — 16g + 1 kHz ODR
        if !spi.write_register(REG_ACCEL_CONFIG0, 0x06) {
            return Err(Icm426xxError::InitFailed);
        }

        // Step 11: GYRO_CONFIG1 — UI filter order (1st order)
        if !spi.write_register(REG_GYRO_CONFIG1, 0x00) {
            return Err(Icm426xxError::InitFailed);
        }

        // Step 12: GYRO_ACCEL_CONFIG0 — UI filter BW for accel+gyro = ODR/4
        if !spi.write_register(REG_GYRO_ACCEL_CONFIG0, 0x11) {
            return Err(Icm426xxError::InitFailed);
        }

        // Step 13: ACCEL_CONFIG1 — accel UI filter order (1st order)
        if !spi.write_register(REG_ACCEL_CONFIG1, 0x00) {
            return Err(Icm426xxError::InitFailed);
        }

        // Step 14: FIFO_CONFIG1 — enable accel + gyro + timestamp in FIFO
        // 0x0F = accel(bit0) + gyro(bit1) + temp(bit2) + timestamp(bit3)
        // Produces 16-byte packets matching FIFO_PACKET_SIZE.
        if !spi.write_register(REG_FIFO_CONFIG1, 0x0F) {
            return Err(Icm426xxError::InitFailed);
        }

        // Step 15: FIFO watermark
        if !spi.write_register(REG_FIFO_CONFIG2, 0x00) {
            return Err(Icm426xxError::InitFailed);
        }
        if !spi.write_register(REG_FIFO_CONFIG3, 0x02) {
            return Err(Icm426xxError::InitFailed);
        }

        // Step 16: INT_CONFIG — push-pull, active-high, latched
        if !spi.write_register(REG_INT_CONFIG, 0x36) {
            return Err(Icm426xxError::InitFailed);
        }

        // Step 17: FIFO_CONFIG — stream-to-FIFO
        if !spi.write_register(REG_FIFO_CONFIG, 0x80) {
            return Err(Icm426xxError::InitFailed);
        }

        // Step 18: PWR_MGMT0 — accel + gyro in low-noise mode
        // Caller must delay 300 us after this write.
        if !spi.write_register(REG_PWR_MGMT0, 0x0F) {
            return Err(Icm426xxError::InitFailed);
        }

        self.initialised = true;
        Ok(())
    }

    /// Write a register on a specific bank and return to bank 0.
    fn write_bank(spi: &mut dyn SpiDevice, bank: u8, reg: u8, val: u8) -> bool {
        if !spi.write_register(REG_BANK_SEL, bank) {
            return false;
        }
        if !spi.write_register(reg, val) {
            let _ = spi.write_register(REG_BANK_SEL, 0x00);
            return false;
        }
        spi.write_register(REG_BANK_SEL, 0x00)
    }

    // -- FIFO read -----------------------------------------------------------

    /// Drain available FIFO packets and return parsed IMU samples.
    ///
    /// Returns up to `out.len()` samples. Returns `Ok(n)` where `n` is the
    /// number of valid samples written into `out[..n]`.
    pub fn read_fifo(
        &self,
        spi: &mut dyn SpiDevice,
        out: &mut [ImuSample],
    ) -> Result<usize, Icm426xxError> {
        if !self.initialised {
            return Err(Icm426xxError::InitFailed);
        }

        // Read FIFO count (16-bit, big-endian, in records — INTF_CONFIG0 bit 6 = 1).
        let mut cnt_buf = [0u8; 2];
        if !spi.read_registers(REG_FIFO_COUNTH, &mut cnt_buf) {
            return Err(Icm426xxError::SpiFailed);
        }
        let count = u16::from_be_bytes(cnt_buf) as usize;
        let to_read = count.min(out.len()).min(MAX_FIFO_PACKETS);

        if to_read == 0 {
            return Ok(0);
        }

        // Burst-read FIFO data.
        let total_bytes = to_read * FIFO_PACKET_SIZE;
        let mut fifo_buf = [0u8; MAX_FIFO_PACKETS * FIFO_PACKET_SIZE];
        if !spi.read_registers(REG_FIFO_DATA, &mut fifo_buf[..total_bytes]) {
            return Err(Icm426xxError::SpiFailed);
        }

        let mut written = 0;
        for i in 0..to_read {
            let offset = i * FIFO_PACKET_SIZE;
            let pkt = &fifo_buf[offset..offset + FIFO_PACKET_SIZE];
            if let Some(sample) = Self::parse_fifo_packet(pkt) {
                out[written] = sample;
                written += 1;
            }
        }

        Ok(written)
    }

    // -- FIFO packet parser --------------------------------------------------

    /// Parse a single 16-byte FIFO packet. Returns `None` if the header
    /// byte is wrong (invalid / corrupted packet).
    pub fn parse_fifo_packet(pkt: &[u8]) -> Option<ImuSample> {
        if pkt.len() < FIFO_PACKET_SIZE {
            return None;
        }
        if pkt[0] != FIFO_HEADER {
            return None;
        }

        // Accel XYZ — big-endian i16 at bytes 1..7 (INTF_CONFIG0=0xC0, default BE)
        let ax = i16::from_be_bytes([pkt[1], pkt[2]]);
        let ay = i16::from_be_bytes([pkt[3], pkt[4]]);
        let az = i16::from_be_bytes([pkt[5], pkt[6]]);

        // Gyro XYZ — big-endian i16 at bytes 7..13
        let gx = i16::from_be_bytes([pkt[7], pkt[8]]);
        let gy = i16::from_be_bytes([pkt[9], pkt[10]]);
        let gz = i16::from_be_bytes([pkt[11], pkt[12]]);

        // Temperature — signed 8-bit at byte 13
        let temp_raw = pkt[13] as i8;
        let temperature = (temp_raw as f32) / TEMP_SENSITIVITY + TEMP_OFFSET;

        // Timestamp — big-endian u16 at bytes 14..16
        let timestamp_raw = u16::from_be_bytes([pkt[14], pkt[15]]);

        let accel = Vec3::<Body>::new(
            ax as f32 * ACCEL_SCALE,
            ay as f32 * ACCEL_SCALE,
            az as f32 * ACCEL_SCALE,
        );
        let gyro = Vec3::<Body>::new(
            gx as f32 * GYRO_SCALE,
            gy as f32 * GYRO_SCALE,
            gz as f32 * GYRO_SCALE,
        );

        Some(ImuSample {
            accel,
            gyro,
            temperature,
            timestamp_raw,
        })
    }

    /// Convert a raw temperature byte to degrees C (public for testing).
    #[inline]
    pub fn temperature_from_raw(raw: i8) -> f32 {
        (raw as f32) / TEMP_SENSITIVITY + TEMP_OFFSET
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use meridian_hal::util::Semaphore;

    // -----------------------------------------------------------------------
    // Mock SPI device for testing
    // -----------------------------------------------------------------------

    struct MockSpi {
        /// Canned register values: (address, value).
        registers: heapless::Vec<(u8, u8), 64>,
        /// Recorded writes: (address, value).
        writes: heapless::Vec<(u8, u8), 64>,
        /// If set, read_registers for FIFO_DATA returns this blob.
        fifo_data: Option<heapless::Vec<u8, 512>>,
        /// Canned FIFO count (little-endian u16, record count).
        fifo_count: u16,
    }

    impl MockSpi {
        fn new() -> Self {
            Self {
                registers: heapless::Vec::new(),
                writes: heapless::Vec::new(),
                fifo_data: None,
                fifo_count: 0,
            }
        }

        fn set_register(&mut self, reg: u8, val: u8) {
            // Replace existing or push new.
            for entry in self.registers.iter_mut() {
                if entry.0 == reg {
                    entry.1 = val;
                    return;
                }
            }
            let _ = self.registers.push((reg, val));
        }

        fn get_written(&self, reg: u8) -> Option<u8> {
            self.writes.iter().rev().find(|(r, _)| *r == reg).map(|(_, v)| *v)
        }
    }

    // Dummy semaphore for the mock.
    struct DummySemaphore;
    impl Semaphore for DummySemaphore {
        fn take_blocking(&self) {}
        fn take(&self, _timeout_us: u32) -> bool { true }
        fn give(&self) {}
    }

    static DUMMY_SEM: DummySemaphore = DummySemaphore;

    impl SpiDevice for MockSpi {
        fn transfer(&mut self, tx: &[u8], rx: &mut [u8]) -> bool {
            // Simple mock: handle single register reads.
            if tx.len() >= 2 && (tx[0] & 0x80) != 0 {
                let reg = tx[0] & 0x7F;

                // Multi-byte read_registers path.
                if tx.len() > 2 {
                    // FIFO_COUNTH — big-endian (INTF_CONFIG0 bit 6 = 1)
                    if reg == REG_FIFO_COUNTH && tx.len() == 3 {
                        let bytes = self.fifo_count.to_be_bytes();
                        rx[0] = 0;
                        rx[1] = bytes[0];
                        rx[2] = bytes[1];
                        return true;
                    }
                    // FIFO_DATA burst read
                    if reg == REG_FIFO_DATA {
                        if let Some(ref data) = self.fifo_data {
                            rx[0] = 0;
                            let len = (tx.len() - 1).min(data.len());
                            rx[1..1 + len].copy_from_slice(&data[..len]);
                            return true;
                        }
                    }
                    // Generic multi-byte: return register values sequentially.
                    rx[0] = 0;
                    for i in 1..tx.len() {
                        let r = reg + (i as u8 - 1);
                        rx[i] = self.registers.iter().find(|(a, _)| *a == r).map(|(_, v)| *v).unwrap_or(0);
                    }
                    return true;
                }

                // Single register read.
                rx[0] = 0;
                rx[1] = self
                    .registers
                    .iter()
                    .find(|(a, _)| *a == reg)
                    .map(|(_, v)| *v)
                    .unwrap_or(0);
                return true;
            }
            true
        }

        fn write(&mut self, data: &[u8]) -> bool {
            if data.len() >= 2 {
                let reg = data[0] & 0x7F;
                let val = data[1];
                let _ = self.writes.push((reg, val));
                // Also update our canned register store so subsequent
                // reads see the written value.
                self.set_register(reg, val);
            }
            true
        }

        fn read(&mut self, data: &mut [u8]) -> bool {
            for b in data.iter_mut() {
                *b = 0;
            }
            true
        }

        fn set_speed(&mut self, _speed_hz: u32) {}

        fn get_semaphore(&self) -> &dyn Semaphore {
            &DUMMY_SEM
        }

        fn device_id(&self) -> u32 {
            0x4200_0001
        }
    }

    // -----------------------------------------------------------------------
    // WHO_AM_I detection
    // -----------------------------------------------------------------------

    #[test]
    fn test_probe_icm42688() {
        let mut spi = MockSpi::new();
        spi.set_register(REG_WHO_AM_I, 0x47);
        let mut drv = Icm426xx::new();
        assert_eq!(drv.probe(&mut spi).unwrap(), Icm426xxVariant::Icm42688);
    }

    #[test]
    fn test_probe_icm42688v() {
        let mut spi = MockSpi::new();
        spi.set_register(REG_WHO_AM_I, 0xDB);
        let mut drv = Icm426xx::new();
        assert_eq!(drv.probe(&mut spi).unwrap(), Icm426xxVariant::Icm42688V);
    }

    #[test]
    fn test_probe_icm42605() {
        let mut spi = MockSpi::new();
        spi.set_register(REG_WHO_AM_I, 0x42);
        let mut drv = Icm426xx::new();
        assert_eq!(drv.probe(&mut spi).unwrap(), Icm426xxVariant::Icm42605);
    }

    #[test]
    fn test_probe_icm42670() {
        let mut spi = MockSpi::new();
        spi.set_register(REG_WHO_AM_I, 0x67);
        let mut drv = Icm426xx::new();
        assert_eq!(drv.probe(&mut spi).unwrap(), Icm426xxVariant::Icm42670);
    }

    #[test]
    fn test_probe_unknown_device() {
        let mut spi = MockSpi::new();
        spi.set_register(REG_WHO_AM_I, 0xFF);
        let mut drv = Icm426xx::new();
        assert_eq!(drv.probe(&mut spi).unwrap_err(), Icm426xxError::UnknownDevice(0xFF));
    }

    // -----------------------------------------------------------------------
    // AFSR disable verification
    // -----------------------------------------------------------------------

    #[test]
    fn test_init_disables_afsr() {
        let mut spi = MockSpi::new();
        // Pre-set INTF_CONFIG1 with AFSR enabled (bits [6:5] = 0b00) and
        // some other bits set.
        spi.set_register(REG_INTF_CONFIG1, 0b0001_0001);
        spi.set_register(REG_WHO_AM_I, 0x47);

        let mut drv = Icm426xx::new();
        drv.probe(&mut spi).unwrap();
        drv.init(&mut spi).unwrap();

        let written = spi.get_written(REG_INTF_CONFIG1).unwrap();
        // Bits [6:5] must be 0b10, other bits preserved.
        assert_eq!(written & 0b0110_0000, 0b0100_0000, "AFSR disable bits [6:5] must be 0b10");
        // Lower bits preserved.
        assert_eq!(written & 0b0001_1111, 0b0001_0001, "non-AFSR bits must be preserved");
    }

    #[test]
    fn test_init_writes_pwr_mgmt0_last() {
        let mut spi = MockSpi::new();
        spi.set_register(REG_INTF_CONFIG1, 0x00);
        spi.set_register(REG_WHO_AM_I, 0x47);

        let mut drv = Icm426xx::new();
        drv.probe(&mut spi).unwrap();
        drv.init(&mut spi).unwrap();

        // PWR_MGMT0 should be written with value 0x0F (the last meaningful init register).
        // After the full init sequence, the last write is PWR_MGMT0.
        let pwr = spi.get_written(REG_PWR_MGMT0).unwrap();
        assert_eq!(pwr, 0x0F, "PWR_MGMT0 should be 0x0F (accel+gyro low-noise)");

        // SIGNAL_PATH_RESET should have been written (new step 1)
        let spr = spi.get_written(REG_SIGNAL_PATH_RESET).unwrap();
        assert_eq!(spr, 0x02, "SIGNAL_PATH_RESET should flush FIFO");

        // DEVICE_CONFIG soft reset should have been written (new step 2)
        let dc = spi.get_written(REG_DEVICE_CONFIG).unwrap();
        assert_eq!(dc, 0x01, "DEVICE_CONFIG should soft-reset");
    }

    // -----------------------------------------------------------------------
    // FIFO packet parsing
    // -----------------------------------------------------------------------

    /// Build a known 16-byte FIFO packet (big-endian, matching INTF_CONFIG0=0xC0).
    fn make_fifo_packet(
        ax: i16, ay: i16, az: i16,
        gx: i16, gy: i16, gz: i16,
        temp: i8,
        ts: u16,
    ) -> [u8; 16] {
        let mut pkt = [0u8; 16];
        pkt[0] = FIFO_HEADER;
        pkt[1..3].copy_from_slice(&ax.to_be_bytes());
        pkt[3..5].copy_from_slice(&ay.to_be_bytes());
        pkt[5..7].copy_from_slice(&az.to_be_bytes());
        pkt[7..9].copy_from_slice(&gx.to_be_bytes());
        pkt[9..11].copy_from_slice(&gy.to_be_bytes());
        pkt[11..13].copy_from_slice(&gz.to_be_bytes());
        pkt[13] = temp as u8;
        pkt[14..16].copy_from_slice(&ts.to_be_bytes());
        pkt
    }

    #[test]
    fn test_parse_fifo_packet_known_values() {
        // 1 g on Z accel  = 2048 LSB → should produce ~9.80665 m/s^2
        // 100 deg/s on X gyro = 100 * 16.4 = 1640 LSB
        let pkt = make_fifo_packet(
            0, 0, 2048,    // accel: 0, 0, +1g
            1640, 0, 0,    // gyro: ~100 dps on X
            0,             // temp raw = 0 → 25.0 C
            0x1234,        // timestamp
        );
        let sample = Icm426xx::parse_fifo_packet(&pkt).unwrap();

        // Accel Z ≈ 9.80665
        assert!((sample.accel.z - 9.80665).abs() < 0.01,
            "accel z = {}, expected ~9.80665", sample.accel.z);
        assert!((sample.accel.x).abs() < 0.001);
        assert!((sample.accel.y).abs() < 0.001);

        // Gyro X ≈ 100 deg/s in rad/s = 100 * pi/180 ≈ 1.7453
        let expected_gyro_x = 100.0 * core::f32::consts::PI / 180.0;
        assert!((sample.gyro.x - expected_gyro_x).abs() < 0.01,
            "gyro x = {}, expected ~{}", sample.gyro.x, expected_gyro_x);
        assert!((sample.gyro.y).abs() < 0.01);
        assert!((sample.gyro.z).abs() < 0.01);

        assert_eq!(sample.timestamp_raw, 0x1234);
    }

    #[test]
    fn test_parse_fifo_packet_negative_values() {
        // -2048 on accel X → -1g → ~-9.80665 m/s^2
        // -1640 on gyro Z → ~-100 deg/s
        let pkt = make_fifo_packet(
            -2048, 0, 0,
            0, 0, -1640,
            -20,          // temp raw = -20 → -20/2.07 + 25 ≈ 15.34
            0xABCD,
        );
        let sample = Icm426xx::parse_fifo_packet(&pkt).unwrap();

        assert!((sample.accel.x + 9.80665).abs() < 0.01);
        let expected_gz = -100.0 * core::f32::consts::PI / 180.0;
        assert!((sample.gyro.z - expected_gz).abs() < 0.01);
        assert_eq!(sample.timestamp_raw, 0xABCD);
    }

    // -----------------------------------------------------------------------
    // Invalid header rejection
    // -----------------------------------------------------------------------

    #[test]
    fn test_invalid_header_rejected() {
        let mut pkt = make_fifo_packet(0, 0, 0, 0, 0, 0, 0, 0);
        pkt[0] = 0x00; // wrong header
        assert!(Icm426xx::parse_fifo_packet(&pkt).is_none());
    }

    #[test]
    fn test_short_packet_rejected() {
        let pkt = [FIFO_HEADER; 10]; // too short
        assert!(Icm426xx::parse_fifo_packet(&pkt).is_none());
    }

    // -----------------------------------------------------------------------
    // Temperature conversion
    // -----------------------------------------------------------------------

    #[test]
    fn test_temperature_conversion_zero() {
        // raw = 0 → 0/2.07 + 25 = 25.0
        let t = Icm426xx::temperature_from_raw(0);
        assert!((t - 25.0).abs() < 0.001, "temp = {}, expected 25.0", t);
    }

    #[test]
    fn test_temperature_conversion_positive() {
        // raw = 52 → 52/2.07 + 25 ≈ 50.12
        let t = Icm426xx::temperature_from_raw(52);
        let expected = 52.0 / 2.07 + 25.0;
        assert!((t - expected).abs() < 0.01, "temp = {}, expected {}", t, expected);
    }

    #[test]
    fn test_temperature_conversion_negative() {
        // raw = -52 → -52/2.07 + 25 ≈ -0.12
        let t = Icm426xx::temperature_from_raw(-52);
        let expected = -52.0 / 2.07 + 25.0;
        assert!((t - expected).abs() < 0.01, "temp = {}, expected {}", t, expected);
    }

    #[test]
    fn test_temperature_in_parsed_packet() {
        // raw = 41 → 41/2.07 + 25 ≈ 44.80
        let pkt = make_fifo_packet(0, 0, 0, 0, 0, 0, 41, 0);
        let sample = Icm426xx::parse_fifo_packet(&pkt).unwrap();
        let expected = 41.0 / 2.07 + 25.0;
        assert!((sample.temperature - expected).abs() < 0.01);
    }

    // -----------------------------------------------------------------------
    // FIFO read integration (through mock SPI)
    // -----------------------------------------------------------------------

    #[test]
    fn test_read_fifo_one_packet() {
        let mut spi = MockSpi::new();
        spi.set_register(REG_INTF_CONFIG1, 0x00);
        spi.set_register(REG_WHO_AM_I, 0x47);

        let mut drv = Icm426xx::new();
        drv.probe(&mut spi).unwrap();
        drv.init(&mut spi).unwrap();

        // Stage one FIFO packet.
        let pkt = make_fifo_packet(2048, 0, 0, 0, 0, 0, 25, 0x0001);
        spi.fifo_count = 1;
        let mut fifo_data: heapless::Vec<u8, 512> = heapless::Vec::new();
        for &b in pkt.iter() {
            let _ = fifo_data.push(b);
        }
        spi.fifo_data = Some(fifo_data);

        let mut samples = [ImuSample {
            accel: Vec3::zero(),
            gyro: Vec3::zero(),
            temperature: 0.0,
            timestamp_raw: 0,
        }; 4];
        let n = drv.read_fifo(&mut spi, &mut samples).unwrap();
        assert_eq!(n, 1);
        assert!((samples[0].accel.x - 2048.0 * ACCEL_SCALE).abs() < 0.01);
    }

    #[test]
    fn test_read_fifo_rejects_if_not_initialised() {
        let mut spi = MockSpi::new();
        let drv = Icm426xx::new();
        let mut samples = [ImuSample {
            accel: Vec3::zero(),
            gyro: Vec3::zero(),
            temperature: 0.0,
            timestamp_raw: 0,
        }; 4];
        assert_eq!(drv.read_fifo(&mut spi, &mut samples).unwrap_err(), Icm426xxError::InitFailed);
    }
}

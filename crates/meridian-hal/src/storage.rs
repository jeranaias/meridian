//! Non-volatile storage trait (parameters, calibration data).
//!
//! Source: ArduPilot `AP_HAL/Storage.h`
//! STM32: flash pages 14-15 on H743 (last 256KB). Wear leveling via A/B sectors.
//! Linux: file-backed (eeprom.bin).

/// Virtual EEPROM size (matches ArduPilot's 16KB default).
pub const STORAGE_SIZE: usize = 16384;

/// Non-volatile storage driver.
pub trait Storage {
    /// Read a block of bytes from storage.
    fn read_block(&self, offset: u16, buf: &mut [u8]) -> bool;

    /// Write a block of bytes to storage.
    fn write_block(&mut self, offset: u16, data: &[u8]) -> bool;

    /// Flush any pending writes to physical storage.
    fn flush(&mut self);

    /// Check if storage is healthy (flash not corrupted).
    fn healthy(&self) -> bool;

    /// Total storage capacity in bytes.
    fn size(&self) -> u16 { STORAGE_SIZE as u16 }
}

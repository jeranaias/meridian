//! SITL utility functions — memory info, hardware ID, persistent data.

use meridian_hal::util::{ResetReason, Util};

/// Persistent data backing store size (matches HAL spec: 16 bytes max).
const PERSISTENT_SIZE: usize = 16;

/// SITL utility driver.
pub struct SitlUtil {
    /// Fixed hardware ID for SITL (12 bytes, matches STM32 UID size).
    hw_id: [u8; 12],
    /// Persistent data across simulated resets.
    persistent_data: [u8; PERSISTENT_SIZE],
    /// Whether persistent data has been written at least once.
    persistent_valid: bool,
}

impl SitlUtil {
    pub fn new() -> Self {
        // Deterministic but recognizable SITL hardware ID.
        // Spells "SITL-MERIDN" in ASCII + trailing null.
        Self {
            hw_id: [
                0x53, 0x49, 0x54, 0x4C, // S I T L
                0x2D, 0x4D, 0x45, 0x52, // - M E R
                0x49, 0x44, 0x4E, 0x00, // I D N \0
            ],
            persistent_data: [0u8; PERSISTENT_SIZE],
            persistent_valid: false,
        }
    }

    /// Set a custom hardware ID (for multi-instance SITL).
    pub fn set_hw_id(&mut self, id: [u8; 12]) {
        self.hw_id = id;
    }
}

impl Util for SitlUtil {
    fn available_memory(&self) -> u32 {
        // Report 1 GB — SITL runs on a desktop with plenty of RAM.
        // This prevents low-memory guards from triggering.
        1024 * 1024 * 1024
    }

    fn get_hw_id(&self, id: &mut [u8; 12]) {
        *id = self.hw_id;
    }

    fn last_reset_reason(&self) -> ResetReason {
        ResetReason::PowerOn
    }

    fn get_persistent_data(&self, buf: &mut [u8]) -> bool {
        if !self.persistent_valid {
            return false;
        }
        let n = buf.len().min(PERSISTENT_SIZE);
        buf[..n].copy_from_slice(&self.persistent_data[..n]);
        true
    }

    fn set_persistent_data(&mut self, data: &[u8]) -> bool {
        if data.len() > PERSISTENT_SIZE {
            return false;
        }
        self.persistent_data[..data.len()].copy_from_slice(data);
        // Zero-fill remainder.
        for b in &mut self.persistent_data[data.len()..] {
            *b = 0;
        }
        self.persistent_valid = true;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_available_memory() {
        let util = SitlUtil::new();
        assert_eq!(util.available_memory(), 1024 * 1024 * 1024);
    }

    #[test]
    fn test_hw_id() {
        let util = SitlUtil::new();
        let mut id = [0u8; 12];
        util.get_hw_id(&mut id);
        assert_eq!(&id[..4], b"SITL");
    }

    #[test]
    fn test_custom_hw_id() {
        let mut util = SitlUtil::new();
        let custom = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12];
        util.set_hw_id(custom);
        let mut id = [0u8; 12];
        util.get_hw_id(&mut id);
        assert_eq!(id, custom);
    }

    #[test]
    fn test_reset_reason() {
        let util = SitlUtil::new();
        assert_eq!(util.last_reset_reason(), ResetReason::PowerOn);
    }

    #[test]
    fn test_persistent_data_not_valid_initially() {
        let util = SitlUtil::new();
        let mut buf = [0u8; 16];
        assert!(!util.get_persistent_data(&mut buf));
    }

    #[test]
    fn test_persistent_data_roundtrip() {
        let mut util = SitlUtil::new();
        assert!(util.set_persistent_data(&[0xDE, 0xAD, 0xBE, 0xEF]));

        let mut buf = [0u8; 4];
        assert!(util.get_persistent_data(&mut buf));
        assert_eq!(&buf, &[0xDE, 0xAD, 0xBE, 0xEF]);
    }

    #[test]
    fn test_persistent_data_too_large() {
        let mut util = SitlUtil::new();
        let big = [0u8; 32];
        assert!(!util.set_persistent_data(&big));
    }

    #[test]
    fn test_persistent_data_zero_fills_remainder() {
        let mut util = SitlUtil::new();
        assert!(util.set_persistent_data(&[0xFF, 0xFF]));

        let mut buf = [0u8; 16];
        assert!(util.get_persistent_data(&mut buf));
        assert_eq!(buf[0], 0xFF);
        assert_eq!(buf[1], 0xFF);
        assert_eq!(buf[2], 0x00); // zero-filled
        assert_eq!(buf[15], 0x00);
    }
}

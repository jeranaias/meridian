//! SITL storage implementation — file-backed virtual EEPROM.
//!
//! Persists parameters to `eeprom.bin` in the working directory.

use std::fs;

use meridian_hal::storage::{Storage, STORAGE_SIZE};

/// SITL storage driver backed by a local `eeprom.bin` file.
///
/// On construction, attempts to load existing data from the file.
/// Falls back to all-zeros if the file does not exist or is the wrong size.
/// `flush()` writes the entire buffer to disk when dirty.
pub struct SitlStorage {
    data: [u8; STORAGE_SIZE],
    dirty: bool,
}

impl SitlStorage {
    const FILE_NAME: &'static str = "eeprom.bin";

    pub fn new() -> Self {
        let data = match fs::read(Self::FILE_NAME) {
            Ok(bytes) if bytes.len() == STORAGE_SIZE => {
                let mut buf = [0u8; STORAGE_SIZE];
                buf.copy_from_slice(&bytes);
                buf
            }
            _ => [0u8; STORAGE_SIZE],
        };

        Self {
            data,
            dirty: false,
        }
    }
}

impl Storage for SitlStorage {
    fn read_block(&self, offset: u16, buf: &mut [u8]) -> bool {
        let start = offset as usize;
        let end = start + buf.len();
        if end > STORAGE_SIZE {
            return false;
        }
        buf.copy_from_slice(&self.data[start..end]);
        true
    }

    fn write_block(&mut self, offset: u16, data: &[u8]) -> bool {
        let start = offset as usize;
        let end = start + data.len();
        if end > STORAGE_SIZE {
            return false;
        }
        self.data[start..end].copy_from_slice(data);
        self.dirty = true;
        true
    }

    fn flush(&mut self) {
        if self.dirty {
            let _ = fs::write(Self::FILE_NAME, &self.data);
            self.dirty = false;
        }
    }

    fn healthy(&self) -> bool {
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_storage_read_write() {
        let mut storage = SitlStorage::new();

        let data = [0xDE, 0xAD, 0xBE, 0xEF];
        assert!(storage.write_block(100, &data));

        let mut buf = [0u8; 4];
        assert!(storage.read_block(100, &mut buf));
        assert_eq!(buf, [0xDE, 0xAD, 0xBE, 0xEF]);
    }

    #[test]
    fn test_storage_bounds_check() {
        let mut storage = SitlStorage::new();

        // Write past end should fail.
        let data = [0u8; 4];
        assert!(!storage.write_block(STORAGE_SIZE as u16 - 2, &data));

        // Read past end should fail.
        let mut buf = [0u8; 4];
        assert!(!storage.read_block(STORAGE_SIZE as u16 - 2, &mut buf));
    }

    #[test]
    fn test_storage_dirty_flag() {
        let mut storage = SitlStorage::new();
        assert!(!storage.dirty);

        storage.write_block(0, &[1, 2, 3]);
        assert!(storage.dirty);

        // Flush clears dirty (file write may fail in test env, but flag clears).
        storage.flush();
        assert!(!storage.dirty);
    }
}

//! Parameter storage on STM32H743 internal flash.
//!
//! H743 flash layout: 2 MB total, 16 pages of 128 KB each, dual-bank.
//! MatekH743 uses pages 14 and 15 (last 256 KB of bank 2) for parameter storage.
//!
//! # Flash Addresses
//!
//!   Page 14: 0x081C_0000 (128 KB) -- sector A
//!   Page 15: 0x081E_0000 (128 KB) -- sector B
//!
//! # Wear Leveling (AP_FlashStorage algorithm)
//!
//! Two sectors form an A/B pair. Each sector is divided into 32-byte "lines".
//! Writes are log-structured: dirty lines are appended sequentially.
//! When a sector fills up, live data is compacted into the other sector,
//! and the full sector is erased. This gives ~50K erase cycles per sector
//! spread across normal parameter writes.
//!
//! # Dual-Bank Flash Unlock
//!
//! H743 has separate unlock registers per bank:
//!   Bank 1: FLASH_KEYR1 (pages 0-7)
//!   Bank 2: FLASH_KEYR2 (pages 8-15) <-- our storage pages
//!
//! Flash cache must be disabled before erase/write and re-enabled after
//! (H7 errata -- cache can serve stale pre-write data).
//!
//! # Safety Warning
//!
//! Writing to wrong flash pages will brick the firmware. The page addresses
//! are verified against the expected range before any erase or write.
//!
//! Source: ArduPilot `AP_HAL_ChibiOS/Storage.cpp`, `hwdef/common/flash.c`

use meridian_hal::storage::{Storage, STORAGE_SIZE};

// ---------------------------------------------------------------------------
// Flash layout constants
// ---------------------------------------------------------------------------

/// Flash base address.
const FLASH_BASE: u32 = 0x0800_0000;

/// Flash page size on H743 (128 KB).
const PAGE_SIZE: usize = 128 * 1024;

/// Storage flash page number (first of the A/B pair).
/// Pages 14 and 15 are the last 256 KB of bank 2.
const STORAGE_PAGE_A: u8 = 14;
const STORAGE_PAGE_B: u8 = 15;

/// Flash address of page A.
const PAGE_A_ADDR: u32 = FLASH_BASE + (STORAGE_PAGE_A as u32) * (PAGE_SIZE as u32);
/// Flash address of page B.
const PAGE_B_ADDR: u32 = FLASH_BASE + (STORAGE_PAGE_B as u32) * (PAGE_SIZE as u32);

/// Line size for log-structured writes (bytes).
/// Each write unit is one line. Must match ArduPilot's CH_STORAGE_LINE_SIZE.
const LINE_SIZE: usize = 32;

/// Number of lines in the virtual EEPROM.
const NUM_LINES: usize = STORAGE_SIZE / LINE_SIZE;

/// Flash unlock key sequence (same for all STM32).
const FLASH_KEY1: u32 = 0x4567_0123;
const FLASH_KEY2: u32 = 0xCDEF_89AB;

/// Maximum write retries before declaring storage unhealthy.
const MAX_WRITE_RETRIES: u8 = 5;

// ---------------------------------------------------------------------------
// Dirty mask -- tracks which lines need to be written to flash
// ---------------------------------------------------------------------------

/// Bitmap tracking dirty lines (1 bit per line).
/// 16384 / 32 = 512 lines, needing 512 bits = 16 u32 words.
struct DirtyMask {
    bits: [u32; (NUM_LINES + 31) / 32],
}

impl DirtyMask {
    const fn new() -> Self {
        Self {
            bits: [0u32; (NUM_LINES + 31) / 32],
        }
    }

    fn set(&mut self, line: usize) {
        if line < NUM_LINES {
            self.bits[line / 32] |= 1 << (line % 32);
        }
    }

    fn clear(&mut self, line: usize) {
        if line < NUM_LINES {
            self.bits[line / 32] &= !(1 << (line % 32));
        }
    }

    #[allow(dead_code)]
    fn get(&self, line: usize) -> bool {
        if line < NUM_LINES {
            (self.bits[line / 32] >> (line % 32)) & 1 == 1
        } else {
            false
        }
    }

    /// Find the first set bit (first dirty line). Returns None if all clean.
    fn first_set(&self) -> Option<usize> {
        for (word_idx, &word) in self.bits.iter().enumerate() {
            if word != 0 {
                let bit = word.trailing_zeros() as usize;
                let line = word_idx * 32 + bit;
                if line < NUM_LINES {
                    return Some(line);
                }
            }
        }
        None
    }

    /// Check if any bits are set.
    fn any_set(&self) -> bool {
        self.bits.iter().any(|&w| w != 0)
    }
}

// ---------------------------------------------------------------------------
// FlashStorage -- implements meridian_hal::Storage
// ---------------------------------------------------------------------------

/// STM32H743 flash-backed parameter storage with A/B sector wear leveling.
///
/// The in-RAM buffer is the authoritative copy. Dirty lines are written
/// to flash one at a time from the storage thread (1 kHz tick, one line
/// per tick to minimize latency).
///
/// On boot, the active sector is identified by its header, and the contents
/// are loaded into the RAM buffer.
pub struct FlashStorage {
    /// In-RAM copy of the entire parameter store.
    buffer: [u8; STORAGE_SIZE],
    /// Which lines have been modified since last flash write.
    dirty: DirtyMask,
    /// Which sector is currently active (0 = page A, 1 = page B).
    active_sector: u8,
    /// Write offset within the active sector (next free line slot).
    write_offset: u32,
    /// Whether flash storage is healthy (no write failures).
    is_healthy: bool,
    /// Whether initial load from flash has completed.
    loaded: bool,
}

impl FlashStorage {
    /// Create a new flash storage instance. Call `load()` to read from flash.
    pub const fn new() -> Self {
        Self {
            buffer: [0xFF; STORAGE_SIZE],
            dirty: DirtyMask::new(),
            active_sector: 0,
            write_offset: 0,
            is_healthy: true,
            loaded: false,
        }
    }

    /// Load parameter data from flash into the RAM buffer.
    ///
    /// Reads both sectors, identifies the active one by header/sequence,
    /// and populates the buffer. Called once at boot.
    pub fn load(&mut self) {
        // Read sector headers to determine which is active.
        // The active sector has a valid header and the highest sequence number.
        //
        // Each sector starts with a 16-byte header:
        //   [0..3]   magic: 0x4D455249 ("MERI")
        //   [4..7]   sequence number (incremented on each compaction)
        //   [8..11]  number of valid lines written
        //   [12..15] CRC32 of the header

        // TODO: actual flash read
        // For each sector:
        //   1. Read header from flash address
        //   2. Validate magic and CRC
        //   3. Select sector with highest valid sequence number
        //   4. Read all valid lines into buffer

        self.active_sector = 0; // Default to sector A
        self.loaded = true;
    }

    /// Write one dirty line to flash. Called from storage thread at 1 kHz.
    ///
    /// Only writes one line per call to minimize latency impact.
    /// The ArduPilot convention: find first dirty line, copy under semaphore,
    /// write to flash, verify buffer unchanged, clear dirty bit.
    pub fn tick(&mut self) {
        if !self.loaded || !self.is_healthy {
            return;
        }

        // Find the first dirty line.
        let line_idx = match self.dirty.first_set() {
            Some(idx) => idx,
            None => return, // Nothing to write
        };

        // Copy the line data under protection (in RTIC, this runs in the
        // storage task which has exclusive access to self).
        let mut line_data = [0u8; LINE_SIZE];
        let offset = line_idx * LINE_SIZE;
        line_data.copy_from_slice(&self.buffer[offset..offset + LINE_SIZE]);

        // Write to flash with retry.
        let mut success = false;
        for _attempt in 0..MAX_WRITE_RETRIES {
            if self.flash_write_line(line_idx as u16, &line_data) {
                success = true;
                break;
            }
        }

        if !success {
            self.is_healthy = false;
            return;
        }

        // Only clear dirty bit if buffer hasn't changed during the write.
        // (Another task might have modified the same line concurrently.)
        if self.buffer[offset..offset + LINE_SIZE] == line_data {
            self.dirty.clear(line_idx);
        }
    }

    /// Check if a compaction is needed (active sector is nearly full).
    fn needs_compaction(&self) -> bool {
        // Compaction threshold: 90% of sector capacity.
        let sector_lines = PAGE_SIZE / (LINE_SIZE + 2); // +2 for line index header
        self.write_offset as usize > (sector_lines * 9 / 10)
    }

    /// Compact: copy all live data to the inactive sector, erase the full one.
    fn compact(&mut self) {
        let inactive = 1 - self.active_sector;

        // 1. Erase the inactive sector
        if !self.flash_erase_sector(inactive) {
            self.is_healthy = false;
            return;
        }

        // 2. Write all buffer lines to the inactive sector sequentially
        // TODO: actual flash writes -- iterate buffer lines, write each to flash

        // 3. Switch active sector
        self.active_sector = inactive;
        self.write_offset = NUM_LINES as u32; // All lines written
    }

    // -----------------------------------------------------------------------
    // Low-level flash operations
    // -----------------------------------------------------------------------

    /// Unlock flash bank 2 for write/erase.
    ///
    /// H743 dual-bank: pages 14-15 are in bank 2, requiring KEYR2 unlock.
    fn flash_unlock_bank2(&self) {
        // TODO: actual register access
        //
        // unsafe {
        //     let flash = &*stm32h7xx_hal::pac::FLASH::ptr();
        //
        //     // Check if already unlocked
        //     if flash.cr2.read().lock().bit_is_set() {
        //         flash.keyr2.write(|w| w.bits(FLASH_KEY1));
        //         flash.keyr2.write(|w| w.bits(FLASH_KEY2));
        //     }
        //
        //     // Disable flash cache before write (H7 errata)
        //     flash.acr.modify(|_, w| w.dcen().clear_bit());
        // }
        let _ = (FLASH_KEY1, FLASH_KEY2);
    }

    /// Lock flash bank 2 after write/erase.
    fn flash_lock_bank2(&self) {
        // TODO: actual register access
        //
        // unsafe {
        //     let flash = &*stm32h7xx_hal::pac::FLASH::ptr();
        //     flash.cr2.modify(|_, w| w.lock().set_bit());
        //
        //     // Re-enable flash cache (H7 errata)
        //     flash.acr.modify(|_, w| w.dcen().set_bit());
        // }
    }

    /// Erase a flash sector (page 14 or 15).
    fn flash_erase_sector(&self, sector: u8) -> bool {
        if sector > 1 {
            return false;
        }

        let page = if sector == 0 { STORAGE_PAGE_A } else { STORAGE_PAGE_B };

        // Verify page is in the expected range (safety check).
        if page < 14 || page > 15 {
            return false;
        }

        self.flash_unlock_bank2();

        // TODO: actual register access
        //
        // unsafe {
        //     let flash = &*stm32h7xx_hal::pac::FLASH::ptr();
        //
        //     // Wait for ready
        //     // while flash.sr2.read().bsy().bit_is_set() {}
        //
        //     // Set sector erase: SNB2 field = page - 8 (bank 2 sector numbering)
        //     // let snb = (page - 8) as u32;
        //     // flash.cr2.modify(|_, w| w.ser().set_bit().snb().bits(snb as u8));
        //
        //     // Start erase
        //     // flash.cr2.modify(|_, w| w.start().set_bit());
        //
        //     // Wait for completion
        //     // while flash.sr2.read().bsy().bit_is_set() {}
        //
        //     // Clear SER bit
        //     // flash.cr2.modify(|_, w| w.ser().clear_bit());
        // }

        self.flash_lock_bank2();

        let _ = page;
        true
    }

    /// Write a single storage line to flash.
    ///
    /// H743 flash writes are in 256-bit (32-byte) words -- conveniently
    /// matching our LINE_SIZE of 32 bytes. Each write is atomic at the
    /// flash controller level.
    fn flash_write_line(&mut self, line_index: u16, data: &[u8; LINE_SIZE]) -> bool {
        if line_index as usize >= NUM_LINES {
            return false;
        }

        let sector_addr = if self.active_sector == 0 {
            PAGE_A_ADDR
        } else {
            PAGE_B_ADDR
        };

        // Each log entry: 32-byte data at the next write slot.
        // H743 programs in 256-bit (32-byte) words, matching LINE_SIZE exactly.
        let write_addr = sector_addr + self.write_offset * (LINE_SIZE as u32);

        // Verify address is within the expected flash range.
        if write_addr < PAGE_A_ADDR || write_addr >= PAGE_B_ADDR + PAGE_SIZE as u32 {
            return false;
        }

        self.flash_unlock_bank2();

        // TODO: actual register access
        //
        // unsafe {
        //     let flash = &*stm32h7xx_hal::pac::FLASH::ptr();
        //
        //     // Wait for ready
        //     // while flash.sr2.read().bsy().bit_is_set() {}
        //
        //     // Set programming mode
        //     // flash.cr2.modify(|_, w| w.pg().set_bit());
        //
        //     // Write 32 bytes (256 bits) -- must be written as 8 aligned u32 words
        //     // let dest = write_addr as *mut u32;
        //     // let src = data.as_ptr() as *const u32;
        //     // for i in 0..8 {
        //     //     core::ptr::write_volatile(dest.add(i), core::ptr::read(src.add(i)));
        //     // }
        //
        //     // Wait for completion
        //     // while flash.sr2.read().bsy().bit_is_set() {}
        //
        //     // Clear PG bit
        //     // flash.cr2.modify(|_, w| w.pg().clear_bit());
        // }

        self.flash_lock_bank2();

        self.write_offset += 1;

        // Check if compaction is needed after this write.
        if self.needs_compaction() {
            self.compact();
        }

        let _ = (write_addr, data);
        true
    }
}

impl Storage for FlashStorage {
    fn read_block(&self, offset: u16, buf: &mut [u8]) -> bool {
        let start = offset as usize;
        let end = start + buf.len();

        if end > STORAGE_SIZE {
            return false;
        }

        buf.copy_from_slice(&self.buffer[start..end]);
        true
    }

    fn write_block(&mut self, offset: u16, data: &[u8]) -> bool {
        let start = offset as usize;
        let end = start + data.len();

        if end > STORAGE_SIZE {
            return false;
        }

        // Write to RAM buffer.
        self.buffer[start..end].copy_from_slice(data);

        // Mark affected lines as dirty.
        let first_line = start / LINE_SIZE;
        let last_line = (end.saturating_sub(1)) / LINE_SIZE;
        for line in first_line..=last_line {
            self.dirty.set(line);
        }

        true
    }

    fn flush(&mut self) {
        // Write all dirty lines immediately (blocking).
        // Called during parameter save or before shutdown.
        while self.dirty.any_set() {
            self.tick();
            if !self.is_healthy {
                break;
            }
        }
    }

    fn healthy(&self) -> bool {
        self.is_healthy
    }
}

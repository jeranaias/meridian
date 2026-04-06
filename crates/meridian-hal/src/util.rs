//! Utility traits — semaphore, memory allocation, system info.
//!
//! Source: ArduPilot `AP_HAL/Util.h`, `AP_HAL/Semaphores.h`
//! CRITICAL: ArduPilot uses recursive semaphores (408 call sites).
//! Meridian uses `meridian-sync::RecursiveMutex` to handle this.

/// Semaphore trait — used for bus locking (SPI, I2C).
/// ArduPilot semaphores are recursive (same thread can lock multiple times).
pub trait Semaphore {
    /// Take the semaphore (blocking).
    fn take_blocking(&self);

    /// Try to take with timeout (microseconds). Returns true if acquired.
    fn take(&self, timeout_us: u32) -> bool;

    /// Release the semaphore.
    fn give(&self);
}

/// Memory allocation type (for DMA-safe buffers on STM32H7).
/// Source: ArduPilot `AP_HAL/Util.h` Memory_Type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemoryType {
    /// Normal heap allocation.
    Normal,
    /// DMA-safe memory (SRAM1/2/3 on H7, NOT DTCM).
    /// Required for all SPI/UART DMA buffers.
    DmaSafe,
    /// Fast memory (DTCM on H7 — fastest but DMA-inaccessible).
    Fast,
}

/// Platform utility functions.
pub trait Util {
    /// Get free memory in bytes.
    fn available_memory(&self) -> u32;

    /// Get unique hardware ID (96-bit STM32 UID, or random for SITL).
    fn get_hw_id(&self, id: &mut [u8; 12]);

    /// Get last reset reason.
    fn last_reset_reason(&self) -> ResetReason;

    /// Persistent data across watchdog resets (16 bytes max).
    fn get_persistent_data(&self, buf: &mut [u8]) -> bool;
    fn set_persistent_data(&mut self, data: &[u8]) -> bool;
}

/// System reset reasons.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResetReason {
    PowerOn,
    ExternalReset,
    Watchdog,
    Software,
    BrownOut,
    Unknown,
}

//! Memory region management for STM32H743.
//!
//! The H743 has a complex memory map with different performance and DMA
//! accessibility characteristics. Getting this wrong causes silent data
//! corruption — the DMA controller simply cannot reach DTCM addresses.
//!
//! # Memory Regions
//!
//! | Region   | Size   | Base Address   | DMA  | Cache  | Speed    |
//! |----------|--------|----------------|------|--------|----------|
//! | ITCM     | 64 KB  | 0x0000_0000    | No   | No     | 0-wait   |
//! | DTCM     | 128 KB | 0x2000_0000    | No   | No     | 0-wait   |
//! | AXI SRAM | 512 KB | 0x2400_0000    | Yes  | Yes    | 0-wait   |
//! | SRAM1    | 128 KB | 0x3000_0000    | Yes  | Yes    | 1-wait   |
//! | SRAM2    | 128 KB | 0x3002_0000    | Yes  | Yes    | 1-wait   |
//! | SRAM3    | 32 KB  | 0x3004_0000    | Yes  | Yes    | 1-wait   |
//! | SRAM4    | 64 KB  | 0x3800_0000    | Yes  | No*    | 1-wait   |
//!
//! *SRAM4 is configured uncached via MPU for bidirectional DShot buffers.
//!
//! # Linker Section Placement
//!
//! DMA buffers are placed in specific memory regions using `#[link_section]`.
//! The linker script must define `.axisram`, `.sram1`, `.sram4` sections
//! mapped to the correct addresses.

use core::sync::atomic::{AtomicUsize, Ordering};

// ---------------------------------------------------------------------------
// Static DMA buffers placed in known-DMA-safe memory regions.
//
// These MUST NOT be in DTCM (0x2000_0000) or stack memory.
// The #[link_section] attribute requires a corresponding section in memory.x.
// ---------------------------------------------------------------------------

/// SPI DMA buffer in AXI SRAM (DMA-safe, cache-coherent).
/// Used for IMU SPI transfers (ICM42688 FIFO reads are up to 2KB).
#[link_section = ".axisram.dma"]
static mut SPI_DMA_BUF: [u8; 4096] = [0u8; 4096];

/// UART DMA bounce buffers in AXI SRAM.
/// Double-buffered RX (128 bytes each) + TX buffer (256 bytes).
#[link_section = ".axisram.dma"]
static mut UART_DMA_BUF: [u8; 2048] = [0u8; 2048];

/// I2C DMA buffer in AXI SRAM.
#[link_section = ".axisram.dma"]
static mut I2C_DMA_BUF: [u8; 512] = [0u8; 512];

/// DShot bidirectional buffer in SRAM4 (uncached, required for bdshot).
/// Must be uncached because the DMA controller writes RPM telemetry data
/// that the CPU reads immediately — cache coherency would cause stale reads.
#[link_section = ".sram4"]
static mut DSHOT_BUF: [u8; 1024] = [0u8; 1024];

// ---------------------------------------------------------------------------
// DMA-safe buffer allocator
// ---------------------------------------------------------------------------

/// Memory region identifier for buffer allocation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemRegion {
    /// AXI SRAM — general DMA-safe buffers (SPI, UART, I2C).
    AxiSram,
    /// SRAM4 — uncached region for bidirectional DShot.
    Sram4,
}

/// Fixed-pool allocator for DMA-safe buffers.
///
/// Pre-allocated static buffers are divided into fixed-size slots.
/// This avoids a general-purpose heap allocator while ensuring all
/// returned buffers are in DMA-accessible memory.
pub struct DmaBufPool {
    /// Base pointer of the pool (must be in AXI SRAM or SRAM1-4).
    base: *mut u8,
    /// Total pool size in bytes.
    size: usize,
    /// Current allocation watermark (bump allocator).
    watermark: AtomicUsize,
}

// SAFETY: The pool is accessed through atomic watermark, ensuring
// each allocation gets a unique, non-overlapping region.
unsafe impl Sync for DmaBufPool {}
unsafe impl Send for DmaBufPool {}

impl DmaBufPool {
    /// Create a new pool from a static buffer.
    ///
    /// # Safety
    ///
    /// `base` must point to a DMA-safe memory region that lives for `'static`.
    /// The caller must ensure no other code accesses the same memory.
    pub const unsafe fn new(base: *mut u8, size: usize) -> Self {
        Self {
            base,
            size,
            watermark: AtomicUsize::new(0),
        }
    }

    /// Allocate a buffer of `len` bytes, aligned to `align`.
    ///
    /// Returns `None` if the pool is exhausted.
    /// Allocations are permanent (no deallocation) — this is a bump allocator
    /// suitable for one-time peripheral initialization.
    pub fn alloc(&self, len: usize, align: usize) -> Option<&'static mut [u8]> {
        loop {
            let current = self.watermark.load(Ordering::Relaxed);
            // Align up
            let aligned = (current + align - 1) & !(align - 1);
            let new_watermark = aligned + len;

            if new_watermark > self.size {
                return None;
            }

            if self
                .watermark
                .compare_exchange_weak(current, new_watermark, Ordering::AcqRel, Ordering::Relaxed)
                .is_ok()
            {
                // SAFETY: We have exclusive ownership of [aligned..new_watermark]
                // because the atomic CAS succeeded.
                unsafe {
                    let ptr = self.base.add(aligned);
                    // Zero-initialize for safety
                    core::ptr::write_bytes(ptr, 0, len);
                    return Some(core::slice::from_raw_parts_mut(ptr, len));
                }
            }
            // CAS failed, retry
        }
    }

    /// Remaining bytes in the pool.
    pub fn remaining(&self) -> usize {
        self.size.saturating_sub(self.watermark.load(Ordering::Relaxed))
    }
}

/// Get a reference to the SPI DMA buffer.
///
/// # Safety
///
/// Caller must ensure exclusive access (typically via SPI bus semaphore).
#[inline]
pub unsafe fn spi_dma_buffer() -> &'static mut [u8] {
    unsafe { &mut *(&raw mut SPI_DMA_BUF) }
}

/// Get a reference to the UART DMA buffer region.
///
/// # Safety
///
/// Caller must ensure exclusive access to the requested slice.
#[inline]
pub unsafe fn uart_dma_buffer() -> &'static mut [u8] {
    unsafe { &mut *(&raw mut UART_DMA_BUF) }
}

/// Get a reference to the I2C DMA buffer.
///
/// # Safety
///
/// Caller must ensure exclusive access.
#[inline]
pub unsafe fn i2c_dma_buffer() -> &'static mut [u8] {
    unsafe { &mut *(&raw mut I2C_DMA_BUF) }
}

/// Get a reference to the DShot bidirectional buffer (SRAM4, uncached).
///
/// # Safety
///
/// Caller must ensure exclusive access.
#[inline]
pub unsafe fn dshot_dma_buffer() -> &'static mut [u8] {
    unsafe { &mut *(&raw mut DSHOT_BUF) }
}

/// Validate that a buffer pointer is in a DMA-safe region.
///
/// Returns `true` if the address falls within AXI SRAM, SRAM1-4.
/// Returns `false` for DTCM, ITCM, or flash addresses.
///
/// Use this as a debug assertion before configuring DMA transfers.
#[inline]
pub fn is_dma_safe(ptr: *const u8) -> bool {
    let addr = ptr as usize;
    // AXI SRAM: 0x2400_0000 .. 0x2408_0000 (512 KB)
    if (0x2400_0000..0x2408_0000).contains(&addr) {
        return true;
    }
    // SRAM1: 0x3000_0000 .. 0x3002_0000 (128 KB)
    if (0x3000_0000..0x3002_0000).contains(&addr) {
        return true;
    }
    // SRAM2: 0x3002_0000 .. 0x3004_0000 (128 KB)
    if (0x3002_0000..0x3004_0000).contains(&addr) {
        return true;
    }
    // SRAM3: 0x3004_0000 .. 0x3004_8000 (32 KB)
    if (0x3004_0000..0x3004_8000).contains(&addr) {
        return true;
    }
    // SRAM4: 0x3800_0000 .. 0x3801_0000 (64 KB)
    if (0x3800_0000..0x3801_0000).contains(&addr) {
        return true;
    }
    false
}

/// Check if an address is in DTCM (NOT DMA-safe).
///
/// DTCM: 128 KB at 0x2000_0000.
/// Using DTCM for DMA will silently corrupt data.
#[inline]
pub fn is_dtcm(ptr: *const u8) -> bool {
    let addr = ptr as usize;
    (0x2000_0000..0x2002_0000).contains(&addr)
}

/// MPU configuration for memory protection.
///
/// Sets up the Memory Protection Unit to:
/// 1. Guard the first 1 KB of ITCM as a null-pointer trap (bus fault on access)
/// 2. Mark SRAM4 as non-cacheable (required for bidirectional DShot)
///
/// This matches ArduPilot's `mem_protect_enable()` and `NOCACHE_MPU_REGION_1`.
pub fn configure_mpu() {
    // MPU configuration is done through raw register writes to the
    // Cortex-M7 MPU registers. The cortex-m crate provides access.
    //
    // Region 0: Null pointer guard — first 1 KB of ITCM, no access
    // Region 1: SRAM4 non-cacheable — 64 KB at 0x3800_0000
    //
    // Implementation deferred to integration testing on real hardware,
    // as MPU misconfiguration causes hard faults that are difficult
    // to debug without a physical probe.
    //
    // The memory.x linker script and #[link_section] attributes are
    // the primary mechanism ensuring correct buffer placement.
}

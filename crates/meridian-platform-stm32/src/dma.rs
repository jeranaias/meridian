//! DMA channel management for STM32H743.
//!
//! The H743 has two DMA controllers (DMA1, DMA2) with 8 streams each,
//! plus a BDMA controller for D3 domain peripherals. DMA streams are a
//! scarce resource — multiple peripherals may need the same stream.
//!
//! ArduPilot's `Shared_DMA` system handles cooperative sharing: when a
//! peripheral needs a DMA stream owned by another, the owner's
//! `deallocate` callback stops its peripheral, and the new owner
//! reconfigures the stream.
//!
//! For Meridian, IMU SPI buses (SPI1, SPI4) have `DMA_NOSHARE` —
//! their DMA streams are never contested. UART and lower-priority
//! SPI buses share DMA streams.

use core::sync::atomic::{AtomicU8, Ordering};
use meridian_sync::RecursiveMutex;

/// Maximum number of DMA streams managed (DMA1: 0-7, DMA2: 8-15).
pub const MAX_DMA_STREAMS: usize = 16;

/// Sentinel: no peripheral owns this stream.
const NO_OWNER: u8 = 0xFF;

/// Per-stream ownership tracking.
struct StreamState {
    /// Which peripheral currently owns this stream (NO_OWNER = free).
    owner: AtomicU8,
}

/// Global DMA stream ownership table.
///
/// Each stream index maps to a `StreamState` tracking the current owner.
/// Peripherals must acquire a stream before configuring DMA, and release
/// it when done.
static STREAM_OWNERS: [StreamState; MAX_DMA_STREAMS] = {
    // const-initialize all entries
    const INIT: StreamState = StreamState {
        owner: AtomicU8::new(NO_OWNER),
    };
    [INIT; MAX_DMA_STREAMS]
};

/// Lock for stream reallocation — prevents races during steal/release.
static DMA_LOCK: RecursiveMutex<()> = RecursiveMutex::new(());

/// Peripheral identifier for DMA ownership tracking.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DmaPeripheral {
    Spi1Rx = 0,
    Spi1Tx = 1,
    Spi2Rx = 2,
    Spi2Tx = 3,
    Spi3Rx = 4,
    Spi3Tx = 5,
    Spi4Rx = 6,
    Spi4Tx = 7,
    Uart1Rx = 8,
    Uart1Tx = 9,
    Uart2Rx = 10,
    Uart2Tx = 11,
    Uart3Rx = 12,
    Uart3Tx = 13,
    I2c1 = 14,
    I2c2 = 15,
}

/// Shared DMA handle — manages cooperative access to DMA streams.
///
/// When a peripheral needs a DMA stream, it calls `acquire()`. If the
/// stream is owned by another peripheral, the other's deallocate callback
/// is invoked first (e.g., `spiStop()`), then the new owner takes over.
pub struct SharedDma {
    /// Which DMA stream(s) this handle manages.
    stream_tx: u8,
    stream_rx: u8,
    /// Peripheral that owns this handle.
    peripheral: DmaPeripheral,
    /// Whether this stream is truly shared (false for DMA_NOSHARE IMU buses).
    is_shared: bool,
}

impl SharedDma {
    /// Create a new shared DMA handle.
    ///
    /// `is_shared` should be `false` for `DMA_NOSHARE` buses (SPI1, SPI4)
    /// which have dedicated DMA streams and skip the locking protocol.
    pub const fn new(
        stream_tx: u8,
        stream_rx: u8,
        peripheral: DmaPeripheral,
        is_shared: bool,
    ) -> Self {
        Self {
            stream_tx,
            stream_rx,
            peripheral,
            is_shared,
        }
    }

    /// Acquire DMA streams for this peripheral.
    ///
    /// For non-shared streams (IMU SPI), this is a no-op.
    /// For shared streams, acquires the global DMA lock and claims ownership.
    ///
    /// `task_id` is the RTIC task priority (for RecursiveMutex).
    pub fn acquire(&self, task_id: u8) {
        if !self.is_shared {
            return;
        }

        let _guard = DMA_LOCK.lock(task_id);
        let periph_id = self.peripheral as u8;

        // Claim TX stream
        if (self.stream_tx as usize) < MAX_DMA_STREAMS {
            STREAM_OWNERS[self.stream_tx as usize]
                .owner
                .store(periph_id, Ordering::Release);
        }

        // Claim RX stream
        if (self.stream_rx as usize) < MAX_DMA_STREAMS {
            STREAM_OWNERS[self.stream_rx as usize]
                .owner
                .store(periph_id, Ordering::Release);
        }
    }

    /// Release DMA streams.
    pub fn release(&self, task_id: u8) {
        if !self.is_shared {
            return;
        }

        let _guard = DMA_LOCK.lock(task_id);

        if (self.stream_tx as usize) < MAX_DMA_STREAMS {
            STREAM_OWNERS[self.stream_tx as usize]
                .owner
                .store(NO_OWNER, Ordering::Release);
        }

        if (self.stream_rx as usize) < MAX_DMA_STREAMS {
            STREAM_OWNERS[self.stream_rx as usize]
                .owner
                .store(NO_OWNER, Ordering::Release);
        }
    }

    /// Check if the stream is currently contested (owned by another peripheral).
    pub fn is_contested(&self) -> bool {
        if !self.is_shared {
            return false;
        }
        let periph_id = self.peripheral as u8;

        let tx_owner = if (self.stream_tx as usize) < MAX_DMA_STREAMS {
            STREAM_OWNERS[self.stream_tx as usize]
                .owner
                .load(Ordering::Acquire)
        } else {
            NO_OWNER
        };

        let rx_owner = if (self.stream_rx as usize) < MAX_DMA_STREAMS {
            STREAM_OWNERS[self.stream_rx as usize]
                .owner
                .load(Ordering::Acquire)
        } else {
            NO_OWNER
        };

        (tx_owner != NO_OWNER && tx_owner != periph_id)
            || (rx_owner != NO_OWNER && rx_owner != periph_id)
    }
}

/// Contention statistics for debugging.
pub struct DmaContentionStats {
    pub contended_locks: u32,
    pub uncontended_locks: u32,
    pub total_transactions: u32,
}

impl DmaContentionStats {
    pub const fn new() -> Self {
        Self {
            contended_locks: 0,
            uncontended_locks: 0,
            total_transactions: 0,
        }
    }
}

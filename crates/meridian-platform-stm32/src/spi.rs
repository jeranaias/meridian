//! STM32H743 SPI driver with DMA for IMU access.
//!
//! H7 uses SPI v2 peripheral — completely different register layout from F4/F7:
//!   - Mode bits (CPOL/CPHA) in SPI_CFG2, not CR1
//!   - Clock divider (MBR) in SPI_CFG1, not CR1 BR field
//!   - Dummy TX/RX buffers must be DMA-safe and 4-byte aligned
//!
//! DMA_NOSHARE on IMU buses (SPI1/SPI4): dedicated DMA streams, no contention.
//! All DMA buffers placed in AXI SRAM (0x24000000) via linker section, NOT stack/DTCM.
//!
//! Transfer flow (from ArduPilot ChibiOS audit):
//!   acquire_bus -> spiStart -> spiSelect -> DMA exchange -> spiUnselect -> release_bus
//!
//! Source: ArduPilot `AP_HAL_ChibiOS/SPIDevice.cpp`

use core::sync::atomic::{AtomicBool, Ordering};

use meridian_hal::spi::{SpiBus, SpiDevice, SpiDeviceDesc, SpiMode};
use meridian_hal::util::Semaphore;
use meridian_sync::{RecursiveMutex, TASK_ID_SPI_BUS};

// ---------------------------------------------------------------------------
// H7 SPI v2 register constants
// ---------------------------------------------------------------------------

/// SPI_CFG2 mode bits (H7 SPI v2 — NOT in CR1 like F4/F7).
const SPI_CFG2_CPOL: u32 = 1 << 25;
const SPI_CFG2_CPHA: u32 = 1 << 24;

/// SPI_CFG1 MBR (master baud rate) field: bits [30:28].
/// Divider = 2^(MBR+1), so MBR=0 -> /2, MBR=7 -> /256.
const SPI_CFG1_MBR_SHIFT: u32 = 28;

/// SPI_CFG1 DSIZE field for 8-bit transfers (DSIZE=7 means 8-bit).
const SPI_CFG1_DSIZE_8BIT: u32 = 7;

/// Maximum SPI transfer size for stack-allocated command buffers.
const MAX_TRANSFER_SIZE: usize = 64;

/// Number of SPI buses on H743 (SPI1..SPI6, but boards typically use 1-4).
const MAX_SPI_BUSES: usize = 6;

// ---------------------------------------------------------------------------
// DMA buffers in AXI SRAM
// ---------------------------------------------------------------------------

/// DMA-safe dummy TX buffer (4-byte aligned, in AXI SRAM).
/// H7 SPI v2 requires valid DMA buffers even for dummy transfers.
#[link_section = ".axisram"]
static mut DUMMY_TX: [u32; 1] = [0xFFFF_FFFF];

/// DMA-safe dummy RX buffer (4-byte aligned, in AXI SRAM).
#[link_section = ".axisram"]
static mut DUMMY_RX: [u32; 1] = [0x0000_0000];

/// Per-bus DMA bounce buffer for SPI transfers (in AXI SRAM).
/// Sized for typical IMU burst reads (32 bytes accel+gyro+temp + register addr).
const DMA_BUF_SIZE: usize = 128;

#[link_section = ".axisram"]
static mut DMA_TX_BUF: [[u8; DMA_BUF_SIZE]; MAX_SPI_BUSES] =
    [[0u8; DMA_BUF_SIZE]; MAX_SPI_BUSES];

#[link_section = ".axisram"]
static mut DMA_RX_BUF: [[u8; DMA_BUF_SIZE]; MAX_SPI_BUSES] =
    [[0u8; DMA_BUF_SIZE]; MAX_SPI_BUSES];

// ---------------------------------------------------------------------------
// Bus semaphores (one per physical bus, recursive)
// ---------------------------------------------------------------------------

/// Per-bus recursive mutex. ArduPilot has 408 call sites using recursive locking.
static SPI_BUS_MUTEX: [RecursiveMutex<()>; MAX_SPI_BUSES] = [
    RecursiveMutex::new(()),
    RecursiveMutex::new(()),
    RecursiveMutex::new(()),
    RecursiveMutex::new(()),
    RecursiveMutex::new(()),
    RecursiveMutex::new(()),
];

/// Semaphore wrapper for a specific SPI bus, implementing the HAL trait.
struct Stm32SpiSemaphore {
    bus_index: usize,
}

impl Semaphore for Stm32SpiSemaphore {
    fn take_blocking(&self) {
        let _guard = SPI_BUS_MUTEX[self.bus_index].lock(TASK_ID_SPI_BUS);
        // Guard is intentionally leaked here — the HAL semaphore model is
        // take/give rather than RAII. Real implementation will store the guard.
    }

    fn take(&self, _timeout_us: u32) -> bool {
        // TODO: implement timeout via cortex-m systick or RTIC monotonic
        let _guard = SPI_BUS_MUTEX[self.bus_index].lock(TASK_ID_SPI_BUS);
        true
    }

    fn give(&self) {
        // In the full implementation, this drops the stored RecursiveGuard.
        // For now, the guard is dropped at scope exit of the caller.
    }
}

// Static semaphore instances (one per bus).
static SPI_SEMAPHORES: [Stm32SpiSemaphore; MAX_SPI_BUSES] = [
    Stm32SpiSemaphore { bus_index: 0 },
    Stm32SpiSemaphore { bus_index: 1 },
    Stm32SpiSemaphore { bus_index: 2 },
    Stm32SpiSemaphore { bus_index: 3 },
    Stm32SpiSemaphore { bus_index: 4 },
    Stm32SpiSemaphore { bus_index: 5 },
];

// ---------------------------------------------------------------------------
// SPI frequency calculation
// ---------------------------------------------------------------------------

/// Convert a target frequency to the SPI_CFG1 MBR divisor value.
/// SPI1/SPI4 clock from PLL2 (200 MHz typical), SPI2/SPI3 from PLL1.
/// Divisor = 2^(mbr+1). We find the largest divisor <= target frequency.
fn compute_mbr(spi_clock_hz: u32, target_hz: u32) -> u32 {
    let mut freq = spi_clock_hz;
    let mut mbr: u32 = 0;
    // Binary search: divide clock until <= target.
    // mbr=0 -> /2, mbr=1 -> /4, ..., mbr=7 -> /256
    while freq > target_hz && mbr < 7 {
        freq >>= 1;
        mbr += 1;
    }
    mbr
}

/// Map SpiMode to H7 SPI_CFG2 bits (CPOL/CPHA in CFG2, not CR1).
fn mode_to_cfg2(mode: SpiMode) -> u32 {
    match mode {
        SpiMode::Mode0 => 0,
        SpiMode::Mode1 => SPI_CFG2_CPHA,
        SpiMode::Mode2 => SPI_CFG2_CPOL,
        SpiMode::Mode3 => SPI_CFG2_CPOL | SPI_CFG2_CPHA,
    }
}

// ---------------------------------------------------------------------------
// Stm32Spi — single device on a bus
// ---------------------------------------------------------------------------

/// STM32H7 SPI device driver.
///
/// Each instance represents one chip (IMU, OSD, etc.) on a physical SPI bus.
/// DMA transfers use bus-specific bounce buffers in AXI SRAM.
pub struct Stm32Spi {
    /// Physical SPI bus index (0-based: SPI1=0, SPI4=3).
    bus_index: u8,
    /// GPIO chip-select pin (encoded as port<<8 | pin).
    cs_pin: u16,
    /// SPI clock mode (CPOL/CPHA).
    mode: SpiMode,
    /// Current transfer speed (Hz).
    speed_hz: u32,
    /// Precomputed CFG1 value for current speed.
    cfg1: u32,
    /// Precomputed CFG2 value for current mode.
    cfg2: u32,
    /// Whether this bus has dedicated DMA (DMA_NOSHARE).
    dma_noshare: bool,
    /// Whether the peripheral is currently started.
    peripheral_started: AtomicBool,
}

impl Stm32Spi {
    /// Create a new SPI device from a descriptor.
    ///
    /// `spi_clock_hz` is the peripheral clock feeding this SPI
    /// (200 MHz for SPI1/SPI4 from PLL2 on MatekH743).
    pub fn new(desc: &SpiDeviceDesc, spi_clock_hz: u32, dma_noshare: bool) -> Self {
        let mbr = compute_mbr(spi_clock_hz, desc.high_speed_hz);
        let cfg1 = (mbr << SPI_CFG1_MBR_SHIFT) | SPI_CFG1_DSIZE_8BIT;
        let cfg2 = mode_to_cfg2(desc.mode);

        Self {
            bus_index: desc.bus,
            cs_pin: desc.cs_pin,
            mode: desc.mode,
            speed_hz: desc.high_speed_hz,
            cfg1,
            cfg2,
            dma_noshare,
            peripheral_started: AtomicBool::new(false),
        }
    }

    /// Bus index as usize for array indexing.
    fn bus_idx(&self) -> usize {
        self.bus_index as usize
    }

    /// Assert chip-select (drive CS low).
    fn cs_assert(&self) {
        let _port = (self.cs_pin >> 8) as u8;
        let _pin = (self.cs_pin & 0xFF) as u8;
        // TODO: actual register access — GPIO BSRR reset bit
        // unsafe { (*gpio_port(port)).bsrr.write(|w| w.bits(1 << (pin + 16))) }
    }

    /// Deassert chip-select (drive CS high).
    fn cs_deassert(&self) {
        let _port = (self.cs_pin >> 8) as u8;
        let _pin = (self.cs_pin & 0xFF) as u8;
        // TODO: actual register access — GPIO BSRR set bit
        // unsafe { (*gpio_port(port)).bsrr.write(|w| w.bits(1 << pin)) }
    }

    /// Start the SPI peripheral with the current configuration.
    fn start_peripheral(&self) {
        if self.peripheral_started.load(Ordering::Relaxed) {
            return;
        }
        // TODO: actual register access
        // Configure SPI_CFG1 (baud rate, data size)
        // Configure SPI_CFG2 (mode bits, master mode, SS management)
        // Set dummy TX/RX pointers for H7 SPI v2
        // Enable SPI (SPI_CR1_SPE)
        //
        // unsafe {
        //     let spi = spi_peripheral(self.bus_index);
        //     spi.cfg1.write(|w| w.bits(self.cfg1));
        //     spi.cfg2.write(|w| w.bits(self.cfg2 | SPI_CFG2_MASTER | SPI_CFG2_SSM));
        //     spi.cr1.modify(|_, w| w.spe().set_bit());
        // }
        self.peripheral_started.store(true, Ordering::Relaxed);
    }

    /// Stop the SPI peripheral (called when DMA is deallocated for sharing).
    fn stop_peripheral(&self) {
        if !self.peripheral_started.load(Ordering::Relaxed) {
            return;
        }
        // TODO: actual register access
        // Disable SPI (SPI_CR1_SPE clear)
        // Force SCK idle to avoid floating clock line
        //
        // unsafe {
        //     let spi = spi_peripheral(self.bus_index);
        //     spi.cr1.modify(|_, w| w.spe().clear_bit());
        // }
        self.peripheral_started.store(false, Ordering::Relaxed);
    }

    /// Perform a DMA exchange: simultaneous TX and RX.
    ///
    /// Both buffers must already be in AXI SRAM (the DMA bounce buffers).
    /// On H7, we configure DMA streams, start the transfer, and wait for
    /// completion via interrupt or polling.
    fn dma_exchange(&self, tx: &[u8], rx: &mut [u8]) -> bool {
        let len = tx.len();
        if len == 0 || len != rx.len() {
            return false;
        }

        let bus = self.bus_idx();

        // Copy TX data into the DMA-safe bounce buffer in AXI SRAM.
        // SAFETY: single-threaded access guaranteed by bus mutex.
        unsafe {
            if len > DMA_BUF_SIZE {
                return false;
            }
            DMA_TX_BUF[bus][..len].copy_from_slice(tx);

            // Invalidate D-cache for RX buffer region before DMA.
            // TODO: actual cache maintenance
            // cortex_m::asm::dsb();
            // SCB::clean_dcache_by_address(DMA_TX_BUF[bus].as_ptr() as usize, len);
            // SCB::invalidate_dcache_by_address(DMA_RX_BUF[bus].as_ptr() as usize, len);
        }

        // TODO: actual DMA register access
        // 1. Configure TX DMA stream:
        //    - Source: DMA_TX_BUF[bus].as_ptr()
        //    - Dest: SPI_DR
        //    - Count: len
        //    - Direction: memory-to-peripheral
        //    - Memory increment, no peripheral increment
        //    - H743 errata: set DMA_SxCR_TRBUFF on all streams
        //
        // 2. Configure RX DMA stream:
        //    - Source: SPI_DR
        //    - Dest: DMA_RX_BUF[bus].as_mut_ptr()
        //    - Count: len
        //    - Direction: peripheral-to-memory
        //    - DMA_SxCR_TRBUFF must be set (H743 errata 2.3.1)
        //
        // 3. Enable both DMA streams
        // 4. Start SPI transfer (CSTART bit in CR1 for H7 SPI v2)
        // 5. Wait for DMA complete (interrupt or poll TCIF)
        //    Timeout: 20ms + 32us per byte
        // 6. Disable DMA streams

        // Copy RX data from DMA-safe bounce buffer back to caller.
        unsafe {
            // Invalidate D-cache for RX region after DMA completes.
            // TODO: SCB::invalidate_dcache_by_address(...)
            rx.copy_from_slice(&DMA_RX_BUF[bus][..len]);
        }

        true
    }
}

impl SpiDevice for Stm32Spi {
    fn transfer(&mut self, tx: &[u8], rx: &mut [u8]) -> bool {
        if tx.is_empty() || tx.len() != rx.len() {
            return false;
        }
        if tx.len() > DMA_BUF_SIZE {
            return false;
        }

        // Full transfer flow from ArduPilot audit:
        // acquire_bus -> spiStart -> spiSelect -> DMA exchange -> spiUnselect -> release_bus

        self.start_peripheral();
        self.cs_assert();

        let ok = self.dma_exchange(tx, rx);

        self.cs_deassert();
        // Peripheral stays started for back-to-back transfers on IMU buses.
        // It will be stopped by dma_deallocate callback if DMA is shared.
        if !self.dma_noshare {
            self.stop_peripheral();
        }

        ok
    }

    fn write(&mut self, data: &[u8]) -> bool {
        if data.is_empty() || data.len() > DMA_BUF_SIZE {
            return false;
        }

        self.start_peripheral();
        self.cs_assert();

        // Write-only: use dummy RX buffer.
        let bus = self.bus_idx();
        unsafe {
            DMA_TX_BUF[bus][..data.len()].copy_from_slice(data);
        }

        // TODO: actual DMA write-only transfer
        // TX DMA from DMA_TX_BUF, RX DMA to DUMMY_RX (discarded)

        self.cs_deassert();
        if !self.dma_noshare {
            self.stop_peripheral();
        }

        true
    }

    fn read(&mut self, data: &mut [u8]) -> bool {
        if data.is_empty() || data.len() > DMA_BUF_SIZE {
            return false;
        }

        self.start_peripheral();
        self.cs_assert();

        // Read-only: TX sends 0xFF (DUMMY_TX), capture RX.
        let bus = self.bus_idx();
        unsafe {
            DMA_TX_BUF[bus][..data.len()].fill(0xFF);
        }

        // TODO: actual DMA read-only transfer
        // TX DMA from DUMMY_TX pattern, RX DMA to DMA_RX_BUF

        unsafe {
            data.copy_from_slice(&DMA_RX_BUF[bus][..data.len()]);
        }

        self.cs_deassert();
        if !self.dma_noshare {
            self.stop_peripheral();
        }

        true
    }

    fn set_speed(&mut self, speed_hz: u32) {
        self.speed_hz = speed_hz;
        // Recalculate MBR divisor.
        // SPI1/SPI4 from PLL2 = 200 MHz, SPI2/SPI3 from PLL1.
        let spi_clock_hz = match self.bus_index {
            0 | 3 => 200_000_000, // SPI1, SPI4 from PLL2
            _ => 100_000_000,     // SPI2, SPI3 from PLL1
        };
        let mbr = compute_mbr(spi_clock_hz, speed_hz);
        self.cfg1 = (mbr << SPI_CFG1_MBR_SHIFT) | SPI_CFG1_DSIZE_8BIT;
        // Peripheral must be restarted to pick up new config.
        self.stop_peripheral();
    }

    fn get_semaphore(&self) -> &dyn Semaphore {
        &SPI_SEMAPHORES[self.bus_idx()]
    }

    fn device_id(&self) -> u32 {
        // Encode bus + CS pin into a unique 32-bit device ID.
        // Matches ArduPilot's DeviceId encoding: bus type (SPI=1) | bus | device.
        let bus_type: u32 = 1; // SPI
        (bus_type << 24) | ((self.bus_index as u32) << 16) | (self.cs_pin as u32)
    }
}

// ---------------------------------------------------------------------------
// Stm32SpiBus — manages multiple devices on one physical SPI peripheral
// ---------------------------------------------------------------------------

/// STM32H7 SPI bus — one per physical SPI peripheral.
pub struct Stm32SpiBus {
    /// Bus index (0-based: SPI1=0, SPI4=3).
    bus_index: u8,
    /// SPI peripheral clock frequency (Hz).
    spi_clock_hz: u32,
    /// Whether this bus has dedicated DMA (DMA_NOSHARE).
    dma_noshare: bool,
}

impl Stm32SpiBus {
    /// Create a bus from board configuration.
    pub fn new(bus_index: u8, spi_clock_hz: u32, dma_noshare: bool) -> Self {
        Self {
            bus_index,
            spi_clock_hz,
            dma_noshare,
        }
    }

    /// Initialize the bus hardware: configure GPIO alternate functions for SCK/MISO/MOSI,
    /// enable peripheral clock, set up DMA streams.
    pub fn init(&self) {
        // TODO: actual register access
        // 1. Enable SPI peripheral clock in RCC
        // 2. Configure SCK, MISO, MOSI pins as alternate function
        // 3. Allocate DMA TX and RX streams
        // 4. If dma_noshare, lock DMA streams permanently
    }
}

impl SpiBus for Stm32SpiBus {
    type Device = Stm32Spi;

    fn get_device(&self, desc: &SpiDeviceDesc) -> Option<Self::Device> {
        if desc.bus != self.bus_index {
            return None;
        }
        Some(Stm32Spi::new(desc, self.spi_clock_hz, self.dma_noshare))
    }
}

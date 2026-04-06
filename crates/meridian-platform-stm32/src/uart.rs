//! STM32H743 UART driver with DMA TX/RX.
//!
//! H7 UART architecture:
//!   - Clock: 100 MHz from PLL3
//!   - 16x oversampling by default; switches to 8x above 6.25 Mbps
//!   - FIFO enabled by default (USART_CR1_FIFOEN) for high-baudrate performance
//!   - DMA RX: double-buffer bounce with IDLE interrupt flush
//!   - DMA TX: single bounce buffer, shared DMA possible
//!
//! H743 errata 2.3.1: DMA_SxCR_TRBUFF must be set on every RX DMA stream.
//!
//! Each UART has its own TX thread and shares one global RX thread.
//! IDLE interrupt (USART_CR1_IDLEIE) triggers early DMA flush when the bus
//! goes idle — critical for variable-length protocols (SBUS, CRSF, GPS NMEA).
//!
//! Source: ArduPilot `AP_HAL_ChibiOS/UARTDriver.cpp`

use heapless::Deque;

use meridian_hal::uart::{FlowControl, UartDriver};

// ---------------------------------------------------------------------------
// H7 UART constants
// ---------------------------------------------------------------------------

/// H7 UART peripheral clock (from PLL3).
const UART_CLOCK_HZ: u32 = 100_000_000;

/// Threshold above which we switch from 16x to 8x oversampling.
/// At 100 MHz UART clock: 100_000_000 / 16 = 6_250_000.
const OVERSAMPLING_8X_THRESHOLD: u32 = UART_CLOCK_HZ / 16;

/// USART_CR1 register bits.
const USART_CR1_FIFOEN: u32 = 1 << 29;  // FIFO enable (H7)
const USART_CR1_OVER8: u32 = 1 << 15;   // 8x oversampling
const USART_CR1_IDLEIE: u32 = 1 << 4;   // IDLE interrupt enable

/// DMA_SxCR_TRBUFF — H743 errata 2.3.1 requirement.
/// Must be set on every RX DMA stream configuration.
const DMA_SXCR_TRBUFF: u32 = 1 << 20;

/// Default bounce buffer size for DMA RX (bytes per buffer, double-buffered).
const RX_BOUNCE_BUFSIZE: usize = 128;

/// Default bounce buffer size for DMA TX.
const TX_BOUNCE_BUFSIZE: usize = 64;

/// Maximum number of UART ports on the platform.
const MAX_UARTS: usize = 10;

/// Default ring buffer capacity if caller specifies 0.
const DEFAULT_BUF_CAPACITY: usize = 512;

// ---------------------------------------------------------------------------
// DMA bounce buffers in AXI SRAM (NOT stack, NOT DTCM)
// ---------------------------------------------------------------------------

/// Double-buffered RX bounce buffers for each UART (in AXI SRAM).
/// DMA alternates between [0] and [1]; IDLE interrupt flushes partial data.
#[link_section = ".axisram"]
static mut RX_BOUNCE: [[u8; RX_BOUNCE_BUFSIZE]; MAX_UARTS * 2] =
    [[0u8; RX_BOUNCE_BUFSIZE]; MAX_UARTS * 2];

/// TX bounce buffer for each UART (in AXI SRAM).
#[link_section = ".axisram"]
static mut TX_BOUNCE: [[u8; TX_BOUNCE_BUFSIZE]; MAX_UARTS] =
    [[0u8; TX_BOUNCE_BUFSIZE]; MAX_UARTS];

// ---------------------------------------------------------------------------
// Baud rate configuration
// ---------------------------------------------------------------------------

/// Compute USART_BRR and CR1 oversampling bits for a target baud rate.
///
/// Returns (brr_value, cr1_over8_bit).
/// At 100 MHz clock:
///   - 16x oversampling for baud <= 6.25 Mbps: BRR = clock / baud
///   - 8x oversampling for baud > 6.25 Mbps: BRR = 2 * clock / baud
fn compute_baud_config(baud: u32) -> (u32, bool) {
    if baud == 0 {
        return (0, false);
    }
    if baud > OVERSAMPLING_8X_THRESHOLD {
        // 8x oversampling — max achievable: 12.5 Mbps at 100 MHz.
        let brr = (2 * UART_CLOCK_HZ + baud / 2) / baud;
        (brr, true)
    } else {
        // 16x oversampling — normal path.
        let brr = (UART_CLOCK_HZ + baud / 2) / baud;
        (brr, false)
    }
}

// ---------------------------------------------------------------------------
// Stm32Uart — one instance per physical UART
// ---------------------------------------------------------------------------

/// STM32H7 UART driver with DMA double-buffer RX and single-buffer TX.
///
/// Ring buffers provide the application-facing read/write API.
/// DMA bounce buffers in AXI SRAM handle the hardware-facing transfers.
/// IDLE interrupt triggers early flush for variable-length protocols.
pub struct Stm32Uart {
    /// UART peripheral index (USART1=1, UART4=4, etc.).
    uart_index: u8,
    /// Application-facing RX ring buffer.
    rx_ring: Deque<u8, DEFAULT_BUF_CAPACITY>,
    /// Application-facing TX ring buffer.
    tx_ring: Deque<u8, DEFAULT_BUF_CAPACITY>,
    /// Current baud rate.
    baud: u32,
    /// Current oversampling mode (true = 8x, false = 16x).
    over8: bool,
    /// Flow control setting.
    flow_control: FlowControl,
    /// Whether this is a USB CDC port.
    is_usb_cdc: bool,
    /// Whether the port is initialized and running.
    initialized: bool,
    /// Half-duplex mode (single-wire bidirectional).
    half_duplex: bool,
    /// Signal inversion (for SBUS RX).
    inverted: bool,
    /// Which RX bounce buffer is currently being filled by DMA (0 or 1).
    rx_bounce_idx: u8,
    /// Whether DMA TX is currently in progress.
    tx_dma_active: bool,
}

impl Stm32Uart {
    /// Create a new uninitialized UART driver.
    pub fn new(uart_index: u8, is_usb: bool) -> Self {
        Self {
            uart_index,
            rx_ring: Deque::new(),
            tx_ring: Deque::new(),
            baud: 0,
            over8: false,
            flow_control: FlowControl::Disable,
            is_usb_cdc: is_usb,
            initialized: false,
            half_duplex: false,
            inverted: false,
            rx_bounce_idx: 0,
            tx_dma_active: false,
        }
    }

    /// Bounce buffer index for this UART's RX double-buffer scheme.
    fn rx_bounce_base(&self) -> usize {
        (self.uart_index as usize) * 2
    }

    /// Configure the UART peripheral hardware.
    fn configure_peripheral(&mut self) {
        let (brr, over8) = compute_baud_config(self.baud);
        self.over8 = over8;

        // TODO: actual register access
        //
        // unsafe {
        //     let usart = usart_peripheral(self.uart_index);
        //
        //     // Disable UART during configuration
        //     usart.cr1.modify(|_, w| w.ue().clear_bit());
        //
        //     // Set baud rate
        //     usart.brr.write(|w| w.bits(brr));
        //
        //     // CR1: FIFO enable, oversampling, IDLE interrupt
        //     let mut cr1 = USART_CR1_FIFOEN | USART_CR1_IDLEIE;
        //     if over8 {
        //         cr1 |= USART_CR1_OVER8;
        //     }
        //     usart.cr1.write(|w| w.bits(cr1));
        //
        //     // CR2: inversion if needed (for SBUS)
        //     if self.inverted {
        //         usart.cr2.modify(|_, w| w.rxinv().set_bit().txinv().set_bit());
        //     }
        //
        //     // CR3: half-duplex mode
        //     if self.half_duplex {
        //         usart.cr3.modify(|_, w| w.hdsel().set_bit());
        //     }
        //
        //     // Enable UART
        //     usart.cr1.modify(|_, w| w.ue().set_bit().te().set_bit().re().set_bit());
        // }
        let _ = brr;
    }

    /// Set up DMA RX double-buffer for this UART.
    fn setup_dma_rx(&mut self) {
        let base = self.rx_bounce_base();
        self.rx_bounce_idx = 0;

        // TODO: actual DMA register access
        //
        // 1. Select the RX DMA stream for this UART
        // 2. Configure:
        //    - Peripheral address: USART_RDR
        //    - Memory address: RX_BOUNCE[base + rx_bounce_idx]
        //    - Transfer size: RX_BOUNCE_BUFSIZE
        //    - Direction: peripheral-to-memory
        //    - Memory increment, no peripheral increment
        //    - Transfer complete interrupt enable
        //    - H743 ERRATA: DMA_SxCR_TRBUFF MUST be set
        //
        // unsafe {
        //     let dma_stream = dma_rx_stream(self.uart_index);
        //     dma_stream.cr.modify(|r, w| w.bits(r.bits() | DMA_SXCR_TRBUFF));
        //     dma_stream.m0ar.write(|w| w.bits(
        //         RX_BOUNCE[base].as_ptr() as u32
        //     ));
        //     dma_stream.ndtr.write(|w| w.bits(RX_BOUNCE_BUFSIZE as u32));
        //     dma_stream.cr.modify(|_, w| w.en().set_bit());
        // }
        let _ = base;
    }

    /// Set up DMA TX for this UART.
    fn setup_dma_tx(&self) {
        // TODO: actual DMA register access
        //
        // TX DMA is configured per-transfer (not continuous like RX).
        // The TX bounce buffer is filled from the ring buffer and
        // the DMA transfer is started.
        //
        // TX DMA auto-disabled for:
        // - Half-duplex mode
        // - Low baud rates on shared DMA streams (<=115200)
        // - After 1000 contention events at <=460800
    }

    /// IDLE interrupt handler — flush partial DMA RX data into ring buffer.
    ///
    /// Called when the UART bus goes idle (gap between bytes).
    /// Critical for variable-length protocols: SBUS, CRSF, GPS NMEA.
    pub fn idle_irq_handler(&mut self) {
        // TODO: actual register access
        //
        // 1. Read USART_ISR to check IDLE flag
        // 2. Clear IDLE flag in USART_ICR
        // 3. Disable RX DMA (forces TC interrupt on H7)
        //    uart.rxdma.stream.CR &= ~DMA_CR_EN;
        // 4. This triggers the DMA TC handler which flips bounce buffers
        //    and copies received bytes into the ring buffer.
    }

    /// DMA RX transfer complete handler.
    /// Copies received bytes from the current bounce buffer into the ring buffer,
    /// then flips to the other bounce buffer for the next DMA transfer.
    pub fn dma_rx_complete_handler(&mut self) {
        let base = self.rx_bounce_base();
        let current = self.rx_bounce_idx as usize;
        let _bounce = base + current;

        // TODO: actual register access
        //
        // 1. Calculate bytes received: RX_BOUNCE_BUFSIZE - DMA_NDTR
        // 2. Invalidate D-cache for the bounce buffer region
        //    SCB::invalidate_dcache_by_address(...)
        // 3. Copy received bytes into rx_ring
        //    for i in 0..bytes_received {
        //        let _ = self.rx_ring.push_back(unsafe { RX_BOUNCE[bounce][i] });
        //    }
        // 4. Flip to other bounce buffer
        //    self.rx_bounce_idx ^= 1;
        // 5. Reconfigure DMA for the new bounce buffer and restart
        self.rx_bounce_idx ^= 1;
    }

    /// DMA TX complete handler.
    /// Check if more data is pending in the TX ring buffer; if so, start
    /// another DMA transfer.
    pub fn dma_tx_complete_handler(&mut self) {
        self.tx_dma_active = false;

        // If there's more data in the TX ring, start another transfer.
        if !self.tx_ring.is_empty() {
            self.start_tx_dma();
        }
    }

    /// Start a DMA TX transfer from the TX ring buffer.
    fn start_tx_dma(&mut self) {
        if self.tx_dma_active || self.tx_ring.is_empty() {
            return;
        }

        let uart_idx = self.uart_index as usize;
        let mut count = 0usize;

        // Copy from ring buffer into DMA-safe TX bounce buffer.
        unsafe {
            while count < TX_BOUNCE_BUFSIZE {
                if let Some(byte) = self.tx_ring.pop_front() {
                    TX_BOUNCE[uart_idx][count] = byte;
                    count += 1;
                } else {
                    break;
                }
            }
        }

        if count == 0 {
            return;
        }

        self.tx_dma_active = true;

        // TODO: actual DMA register access
        // 1. Clean D-cache for TX bounce buffer region
        //    SCB::clean_dcache_by_address(TX_BOUNCE[uart_idx].as_ptr() as usize, count)
        // 2. Configure TX DMA stream:
        //    - Source: TX_BOUNCE[uart_idx].as_ptr()
        //    - Dest: USART_TDR
        //    - Count: count
        //    - Direction: memory-to-peripheral
        //    - Transfer complete interrupt enable
        // 3. Enable DMA stream
    }
}

impl UartDriver for Stm32Uart {
    fn begin(&mut self, baud: u32, rx_buf: u16, tx_buf: u16) {
        let _ = (rx_buf, tx_buf);
        // The heapless Deque has a fixed compile-time capacity (DEFAULT_BUF_CAPACITY).
        // In a production implementation, we'd use a dynamically-sized ring buffer
        // allocated from DMA-safe memory. For now, we clear and reinitialize.

        self.baud = baud;
        self.rx_ring = Deque::new();
        self.tx_ring = Deque::new();

        // TODO: actual register access
        // 1. Enable UART peripheral clock in RCC
        // 2. Configure TX/RX pins as alternate function
        // 3. Configure the UART peripheral (baud, FIFO, oversampling)
        self.configure_peripheral();
        // 4. Set up DMA RX with double-buffer bounce
        self.setup_dma_rx();
        // 5. Set up DMA TX
        self.setup_dma_tx();

        self.initialized = true;
    }

    fn end(&mut self) {
        if !self.initialized {
            return;
        }

        // TODO: actual register access
        // 1. Disable DMA RX and TX streams
        // 2. Disable UART peripheral
        // 3. Disable UART peripheral clock

        self.rx_ring = Deque::new();
        self.tx_ring = Deque::new();
        self.initialized = false;
        self.tx_dma_active = false;
    }

    fn available(&self) -> u16 {
        self.rx_ring.len() as u16
    }

    fn txspace(&self) -> u16 {
        (self.tx_ring.capacity() - self.tx_ring.len()) as u16
    }

    fn read(&mut self, buf: &mut [u8]) -> u16 {
        let mut count = 0u16;
        for b in buf.iter_mut() {
            if let Some(byte) = self.rx_ring.pop_front() {
                *b = byte;
                count += 1;
            } else {
                break;
            }
        }
        count
    }

    fn read_byte(&mut self) -> Option<u8> {
        self.rx_ring.pop_front()
    }

    fn write(&mut self, data: &[u8]) -> u16 {
        let mut count = 0u16;
        for &byte in data {
            if self.tx_ring.push_back(byte).is_ok() {
                count += 1;
            } else {
                break; // TX ring full.
            }
        }

        // Kick off DMA TX if not already running.
        if count > 0 && !self.tx_dma_active {
            self.start_tx_dma();
        }

        count
    }

    fn write_byte(&mut self, byte: u8) -> bool {
        if self.tx_ring.push_back(byte).is_ok() {
            if !self.tx_dma_active {
                self.start_tx_dma();
            }
            true
        } else {
            false
        }
    }

    fn flush(&mut self) {
        // Block until TX ring and DMA are both drained.
        // TODO: actual implementation — wait for DMA TC and TXE + TC flags in USART_ISR
        while !self.tx_ring.is_empty() || self.tx_dma_active {
            // In real implementation: WFI or yield to scheduler.
            cortex_m::asm::nop();
        }
    }

    fn set_flow_control(&mut self, flow: FlowControl) {
        self.flow_control = flow;
        // TODO: actual register access — USART_CR3 CTSE/RTSE bits
    }

    fn get_baud_rate(&self) -> u32 {
        self.baud
    }

    fn is_usb(&self) -> bool {
        self.is_usb_cdc
    }

    fn set_half_duplex(&mut self, enable: bool) {
        self.half_duplex = enable;
        // Must reconfigure peripheral for HDSEL bit.
        if self.initialized {
            self.configure_peripheral();
        }
    }

    fn set_inverted(&mut self, enable: bool) {
        self.inverted = enable;
        // Must reconfigure peripheral for RXINV/TXINV bits (SBUS).
        if self.initialized {
            self.configure_peripheral();
        }
    }

    fn discard_input(&mut self) {
        self.rx_ring = Deque::new();
        // Also flush any in-progress DMA RX data.
        // TODO: disable and re-enable DMA RX stream to reset bounce buffers.
    }
}

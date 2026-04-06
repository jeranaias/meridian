//! UART/Serial driver trait.
//!
//! Source: ArduPilot `AP_HAL/UARTDriver.h`
//! ChibiOS implementation: DMA TX/RX with double-buffer bounce, IDLE interrupt flush.

/// Flow control mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlowControl {
    Disable,
    Enable,
    Auto,
}

/// UART driver trait — one per physical serial port.
pub trait UartDriver {
    /// Initialize the UART at the given baud rate with TX/RX buffer sizes.
    fn begin(&mut self, baud: u32, rx_buf: u16, tx_buf: u16);

    /// Shut down the UART.
    fn end(&mut self);

    /// Bytes available to read.
    fn available(&self) -> u16;

    /// Space available in TX buffer.
    fn txspace(&self) -> u16;

    /// Read up to `buf.len()` bytes. Returns number of bytes actually read.
    fn read(&mut self, buf: &mut [u8]) -> u16;

    /// Read a single byte. Returns None if no data available.
    fn read_byte(&mut self) -> Option<u8>;

    /// Write bytes. Returns number of bytes actually written.
    fn write(&mut self, data: &[u8]) -> u16;

    /// Write a single byte. Returns true if written.
    fn write_byte(&mut self, byte: u8) -> bool;

    /// Flush pending TX data (block until sent).
    fn flush(&mut self);

    /// Set flow control mode.
    fn set_flow_control(&mut self, flow: FlowControl);

    /// Get current baud rate.
    fn get_baud_rate(&self) -> u32;

    /// Whether this is a USB CDC port (affects some protocol behavior).
    fn is_usb(&self) -> bool { false }

    /// Set half-duplex mode (single-wire bidirectional).
    fn set_half_duplex(&mut self, _enable: bool) {}

    /// Set inverted signal (for SBUS RX).
    fn set_inverted(&mut self, _enable: bool) {}

    /// Discard all buffered input.
    fn discard_input(&mut self);
}

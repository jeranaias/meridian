//! SITL UART implementation — ring-buffer backed, no socket I/O.
//!
//! The main loop bridges these buffers to TCP/UDP connections.

use std::collections::VecDeque;

use meridian_hal::uart::{FlowControl, UartDriver};

const DEFAULT_BUF_SIZE: usize = 4096;

/// SITL UART driver backed by in-memory ring buffers.
///
/// No actual socket I/O happens here. The simulation main loop reads from
/// `tx_buf` and writes into `rx_buf` to bridge to TCP/UDP transports.
pub struct SitlUart {
    rx_buf: VecDeque<u8>,
    tx_buf: VecDeque<u8>,
    rx_capacity: usize,
    tx_capacity: usize,
    baud: u32,
    initialized: bool,
    flow_control: FlowControl,
}

impl SitlUart {
    pub fn new() -> Self {
        Self {
            rx_buf: VecDeque::new(),
            tx_buf: VecDeque::new(),
            rx_capacity: DEFAULT_BUF_SIZE,
            tx_capacity: DEFAULT_BUF_SIZE,
            baud: 0,
            initialized: false,
            flow_control: FlowControl::Disable,
        }
    }

    /// Inject bytes into the RX buffer (called by the main loop / network bridge).
    pub fn inject_rx(&mut self, data: &[u8]) {
        for &b in data {
            if self.rx_buf.len() < self.rx_capacity {
                self.rx_buf.push_back(b);
            }
        }
    }

    /// Drain bytes from the TX buffer (called by the main loop / network bridge).
    pub fn drain_tx(&mut self, buf: &mut [u8]) -> usize {
        let count = buf.len().min(self.tx_buf.len());
        for b in buf.iter_mut().take(count) {
            *b = self.tx_buf.pop_front().unwrap();
        }
        count
    }
}

impl UartDriver for SitlUart {
    fn begin(&mut self, baud: u32, rx_buf: u16, tx_buf: u16) {
        let rx_size = if rx_buf == 0 { DEFAULT_BUF_SIZE } else { rx_buf as usize };
        let tx_size = if tx_buf == 0 { DEFAULT_BUF_SIZE } else { tx_buf as usize };

        self.baud = baud;
        self.rx_buf = VecDeque::with_capacity(rx_size);
        self.tx_buf = VecDeque::with_capacity(tx_size);
        self.rx_capacity = rx_size;
        self.tx_capacity = tx_size;
        self.initialized = true;
    }

    fn end(&mut self) {
        self.initialized = false;
        self.rx_buf.clear();
        self.tx_buf.clear();
    }

    fn available(&self) -> u16 {
        self.rx_buf.len().min(u16::MAX as usize) as u16
    }

    fn txspace(&self) -> u16 {
        let space = self.tx_capacity.saturating_sub(self.tx_buf.len());
        space.min(u16::MAX as usize) as u16
    }

    fn read(&mut self, buf: &mut [u8]) -> u16 {
        let count = buf.len().min(self.rx_buf.len());
        for b in buf.iter_mut().take(count) {
            *b = self.rx_buf.pop_front().unwrap();
        }
        count as u16
    }

    fn read_byte(&mut self) -> Option<u8> {
        self.rx_buf.pop_front()
    }

    fn write(&mut self, data: &[u8]) -> u16 {
        let space = self.tx_capacity.saturating_sub(self.tx_buf.len());
        let count = data.len().min(space);
        for &b in data.iter().take(count) {
            self.tx_buf.push_back(b);
        }
        count as u16
    }

    fn write_byte(&mut self, byte: u8) -> bool {
        if self.tx_buf.len() < self.tx_capacity {
            self.tx_buf.push_back(byte);
            true
        } else {
            false
        }
    }

    fn flush(&mut self) {
        // No-op in SITL. The main loop drains TX on its own schedule.
    }

    fn set_flow_control(&mut self, flow: FlowControl) {
        self.flow_control = flow;
    }

    fn get_baud_rate(&self) -> u32 {
        self.baud
    }

    fn discard_input(&mut self) {
        self.rx_buf.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_uart_basic_rw() {
        let mut uart = SitlUart::new();
        uart.begin(115200, 0, 0);

        // Write some bytes.
        let written = uart.write(b"hello");
        assert_eq!(written, 5);
        assert_eq!(uart.txspace(), DEFAULT_BUF_SIZE as u16 - 5);

        // Inject into RX (simulating network bridge).
        uart.inject_rx(b"world");
        assert_eq!(uart.available(), 5);

        let mut buf = [0u8; 5];
        let read = uart.read(&mut buf);
        assert_eq!(read, 5);
        assert_eq!(&buf, b"world");
    }

    #[test]
    fn test_uart_drain_tx() {
        let mut uart = SitlUart::new();
        uart.begin(115200, 0, 0);

        uart.write(b"drain_me");
        let mut buf = [0u8; 16];
        let n = uart.drain_tx(&mut buf);
        assert_eq!(n, 8);
        assert_eq!(&buf[..8], b"drain_me");
    }

    #[test]
    fn test_uart_tx_full() {
        let mut uart = SitlUart::new();
        uart.begin(115200, 16, 16);

        // Fill TX buffer exactly.
        let data = [0xAA; 16];
        let written = uart.write(&data);
        assert_eq!(written, 16);
        assert_eq!(uart.txspace(), 0);

        // Further writes should return 0.
        assert_eq!(uart.write(&[0xBB]), 0);
        assert!(!uart.write_byte(0xCC));
    }

    #[test]
    fn test_uart_rx_overflow() {
        let mut uart = SitlUart::new();
        uart.begin(115200, 8, 8);

        // Inject more than capacity.
        uart.inject_rx(&[0x11; 12]);

        // Only 8 should be in the buffer.
        assert_eq!(uart.available(), 8);
    }
}

//! Transport trait — matches your ATAK plugin's MavlinkTransport pattern.
//!
//! Any byte stream (UART, USB, TCP, UDP, WebSocket) implements this.
//! The protocol layer doesn't know or care what's underneath.

/// Transport trait for any bidirectional byte stream.
/// Mirrors com.thornveil.dronemobile.MavlinkTransport exactly.
pub trait Transport {
    /// Send raw bytes.
    fn write(&mut self, data: &[u8]) -> Result<usize, TransportError>;

    /// Read available bytes into buffer. Non-blocking: returns 0 if no data.
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, TransportError>;

    /// Is the transport currently connected/open?
    fn is_connected(&self) -> bool;

    /// Human-readable description (e.g., "UART /dev/ttyS1 57600", "TCP 192.168.1.1:5762")
    fn description(&self) -> &str;
}

/// Transport errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportError {
    NotConnected,
    Timeout,
    IoError,
    BufferFull,
}

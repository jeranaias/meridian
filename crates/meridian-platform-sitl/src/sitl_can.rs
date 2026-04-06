//! SITL CAN bus — loopback for DroneCAN testing.
//!
//! Frames sent via `send()` are queued and returned by `receive()`.
//! This lets DroneCAN protocol code be tested without real CAN hardware.

use std::collections::VecDeque;

use meridian_hal::can::{CanFrame, CanIface};

/// Maximum loopback queue depth before oldest frames are dropped.
const MAX_QUEUE_DEPTH: usize = 256;

/// SITL CAN interface — loopback mode.
pub struct SitlCan {
    /// Bus index (0 or 1).
    bus_index: u8,
    /// Loopback queue: sent frames appear here for receive.
    queue: VecDeque<CanFrame>,
    /// Whether the interface has been initialized.
    initialized: bool,
    /// Configured bitrate (stored but unused in SITL).
    bitrate: u32,
    /// CAN FD mode flag.
    fd_mode: bool,
}

impl SitlCan {
    pub fn new() -> Self {
        Self {
            bus_index: 0,
            queue: VecDeque::new(),
            initialized: false,
            bitrate: 1_000_000,
            fd_mode: false,
        }
    }

    /// Create with a specific bus index.
    pub fn with_index(index: u8) -> Self {
        Self {
            bus_index: index,
            ..Self::new()
        }
    }

    /// Peek at the next frame without removing it.
    pub fn peek(&self) -> Option<&CanFrame> {
        self.queue.front()
    }

    /// Inject a frame into the receive queue from an external source
    /// (e.g. another SitlCan instance for bus-to-bus testing).
    pub fn inject_frame(&mut self, frame: CanFrame) {
        if self.queue.len() >= MAX_QUEUE_DEPTH {
            self.queue.pop_front(); // drop oldest
        }
        self.queue.push_back(frame);
    }
}

impl CanIface for SitlCan {
    fn init(&mut self, bitrate: u32) -> bool {
        self.bitrate = bitrate;
        self.initialized = true;
        true
    }

    fn send(&mut self, frame: &CanFrame, _timeout_us: u32) -> bool {
        if !self.initialized {
            return false;
        }
        // Loopback: sent frames go directly into the receive queue.
        if self.queue.len() >= MAX_QUEUE_DEPTH {
            self.queue.pop_front(); // drop oldest on overflow
        }
        self.queue.push_back(*frame);
        true
    }

    fn receive(&mut self) -> Option<CanFrame> {
        self.queue.pop_front()
    }

    fn available(&self) -> u16 {
        self.queue.len() as u16
    }

    fn is_healthy(&self) -> bool {
        true // loopback is always healthy
    }

    fn get_error_counts(&self) -> (u16, u16) {
        (0, 0) // no errors in loopback
    }

    fn set_filter(&mut self, _id: u32, _mask: u32, _extended: bool) -> bool {
        // Accept all frames in SITL loopback.
        true
    }

    fn set_fd_mode(&mut self, enable: bool) {
        self.fd_mode = enable;
    }

    fn bus_index(&self) -> u8 {
        self.bus_index
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_loopback() {
        let mut can = SitlCan::new();
        can.init(1_000_000);

        let frame = CanFrame::new_standard(0x123, &[0xDE, 0xAD]);
        assert!(can.send(&frame, 1000));
        assert_eq!(can.available(), 1);

        let rx = can.receive().unwrap();
        assert_eq!(rx.id, 0x123);
        assert_eq!(rx.dlc, 2);
        assert_eq!(&rx.data[..2], &[0xDE, 0xAD]);
        assert_eq!(can.available(), 0);
    }

    #[test]
    fn test_not_initialized() {
        let mut can = SitlCan::new();
        let frame = CanFrame::new_standard(0x100, &[0x01]);
        assert!(!can.send(&frame, 1000)); // should fail before init
    }

    #[test]
    fn test_multiple_frames_fifo() {
        let mut can = SitlCan::new();
        can.init(1_000_000);

        for i in 0..5u16 {
            let frame = CanFrame::new_standard(i, &[i as u8]);
            can.send(&frame, 1000);
        }
        assert_eq!(can.available(), 5);

        // Receive in FIFO order.
        for i in 0..5u16 {
            let rx = can.receive().unwrap();
            assert_eq!(rx.id, i as u32);
        }
        assert!(can.receive().is_none());
    }

    #[test]
    fn test_extended_frame() {
        let mut can = SitlCan::new();
        can.init(1_000_000);

        let frame = CanFrame::new_extended(0x1ABCDEF0, &[1, 2, 3, 4, 5, 6, 7, 8]);
        can.send(&frame, 1000);

        let rx = can.receive().unwrap();
        assert!(rx.extended);
        assert_eq!(rx.id, 0x1ABCDEF0);
        assert_eq!(rx.dlc, 8);
    }

    #[test]
    fn test_overflow_drops_oldest() {
        let mut can = SitlCan::new();
        can.init(1_000_000);

        // Fill beyond max capacity.
        for i in 0..=(MAX_QUEUE_DEPTH as u16) {
            let frame = CanFrame::new_standard(i, &[i as u8]);
            can.send(&frame, 1000);
        }
        // Queue capped at MAX_QUEUE_DEPTH; oldest (id=0) was dropped.
        assert_eq!(can.available(), MAX_QUEUE_DEPTH as u16);
        let first = can.receive().unwrap();
        assert_eq!(first.id, 1);
    }

    #[test]
    fn test_inject_frame() {
        let mut can = SitlCan::new();
        let frame = CanFrame::new_standard(0x42, &[0xFF]);
        can.inject_frame(frame);
        assert_eq!(can.available(), 1);
        let rx = can.receive().unwrap();
        assert_eq!(rx.id, 0x42);
    }

    #[test]
    fn test_healthy_and_errors() {
        let can = SitlCan::new();
        assert!(can.is_healthy());
        assert_eq!(can.get_error_counts(), (0, 0));
    }

    #[test]
    fn test_bus_index() {
        let can = SitlCan::with_index(1);
        assert_eq!(can.bus_index(), 1);
    }
}

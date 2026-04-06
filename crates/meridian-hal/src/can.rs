//! CAN bus interface trait.
//!
//! Source: ArduPilot `AP_HAL/CANIface.h`
//! H7: FDCAN peripheral (Bosch MCAN). Up to 3 CAN interfaces.
//! Default 1 Mbit/s, FDCAN data phase up to 8 Mbit/s.

/// CAN frame (standard or extended ID).
#[derive(Debug, Clone, Copy)]
pub struct CanFrame {
    pub id: u32,
    pub extended: bool,
    pub data: [u8; 64], // up to 64 bytes for CAN FD
    pub dlc: u8,
}

impl CanFrame {
    pub fn new_standard(id: u16, data: &[u8]) -> Self {
        let mut frame = Self {
            id: id as u32,
            extended: false,
            data: [0u8; 64],
            dlc: data.len() as u8,
        };
        frame.data[..data.len()].copy_from_slice(data);
        frame
    }

    pub fn new_extended(id: u32, data: &[u8]) -> Self {
        let mut frame = Self {
            id: id & 0x1FFFFFFF,
            extended: true,
            data: [0u8; 64],
            dlc: data.len() as u8,
        };
        frame.data[..data.len()].copy_from_slice(data);
        frame
    }

    pub fn data_slice(&self) -> &[u8] {
        &self.data[..self.dlc as usize]
    }
}

/// CAN bus interface.
pub trait CanIface {
    /// Initialize the CAN interface at the given bitrate.
    fn init(&mut self, bitrate: u32) -> bool;

    /// Send a CAN frame. Returns true if queued successfully.
    fn send(&mut self, frame: &CanFrame, timeout_us: u32) -> bool;

    /// Receive a CAN frame. Returns None if no frame available.
    fn receive(&mut self) -> Option<CanFrame>;

    /// Number of available frames in RX buffer.
    fn available(&self) -> u16;

    /// Whether the bus is healthy (no bus-off, no error passive).
    fn is_healthy(&self) -> bool;

    /// Get bus error counters.
    fn get_error_counts(&self) -> (u16, u16); // (tx_errors, rx_errors)

    /// Set acceptance filter (hardware filtering by ID).
    fn set_filter(&mut self, id: u32, mask: u32, extended: bool) -> bool;

    /// Enable/disable CAN FD mode.
    fn set_fd_mode(&mut self, enable: bool);

    /// Bus index (0, 1, or 2).
    fn bus_index(&self) -> u8;
}

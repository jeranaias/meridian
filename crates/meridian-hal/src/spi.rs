//! SPI bus and device traits.
//!
//! Source: ArduPilot `AP_HAL/SPIDevice.h`
//! H7 uses SPI v2 peripheral — completely different register layout from F4/F7.
//! DMA_NOSHARE on IMU buses gives dedicated DMA streams.

use crate::util::Semaphore;

/// SPI clock polarity and phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiMode {
    Mode0, // CPOL=0, CPHA=0
    Mode1, // CPOL=0, CPHA=1
    Mode2, // CPOL=1, CPHA=0
    Mode3, // CPOL=1, CPHA=1
}

/// SPI device descriptor — identifies a specific chip on a bus.
#[derive(Debug, Clone, Copy)]
pub struct SpiDeviceDesc {
    pub bus: u8,
    pub cs_pin: u16,
    pub mode: SpiMode,
    pub low_speed_hz: u32,
    pub high_speed_hz: u32,
}

/// A single SPI device (chip on a bus).
pub trait SpiDevice {
    /// Full-duplex transfer: simultaneous TX and RX.
    /// `tx` and `rx` must be the same length.
    fn transfer(&mut self, tx: &[u8], rx: &mut [u8]) -> bool;

    /// Write-only transfer (discard RX).
    fn write(&mut self, data: &[u8]) -> bool;

    /// Read-only transfer (TX zeros).
    fn read(&mut self, data: &mut [u8]) -> bool;

    /// Set clock speed for next transfer.
    fn set_speed(&mut self, speed_hz: u32);

    /// Get the bus semaphore for exclusive access.
    fn get_semaphore(&self) -> &dyn Semaphore;

    /// Read a single register (write address, read value).
    fn read_register(&mut self, reg: u8) -> Option<u8> {
        let tx = [reg | 0x80, 0x00];
        let mut rx = [0u8; 2];
        if self.transfer(&tx, &mut rx) {
            Some(rx[1])
        } else {
            None
        }
    }

    /// Write a single register.
    fn write_register(&mut self, reg: u8, value: u8) -> bool {
        let tx = [reg & 0x7F, value];
        self.write(&tx)
    }

    /// Read multiple registers starting at `reg`.
    ///
    /// MU2 fix: Supports reads up to 255 bytes by chunking into 63-byte
    /// transfers when the requested length exceeds the 64-byte stack buffer
    /// (1 byte for register address + 63 bytes of data = 64 total).
    fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> bool {
        if buf.is_empty() {
            return true;
        }

        let mut offset = 0usize;
        while offset < buf.len() {
            // Each transfer: 1 byte register address + up to 63 bytes data = 64 total
            let chunk_len = (buf.len() - offset).min(63);
            let xfer_len = chunk_len + 1;
            let mut tx = [0u8; 64];
            let mut rx = [0u8; 64];
            // Auto-increment register address for multi-byte reads
            tx[0] = (reg.wrapping_add(offset as u8)) | 0x80;
            if !self.transfer(&tx[..xfer_len], &mut rx[..xfer_len]) {
                return false;
            }
            buf[offset..offset + chunk_len].copy_from_slice(&rx[1..xfer_len]);
            offset += chunk_len;
        }
        true
    }

    /// Device ID for this chip (for parameter persistence).
    fn device_id(&self) -> u32;
}

/// SPI bus — manages multiple devices on one physical SPI peripheral.
pub trait SpiBus {
    type Device: SpiDevice;

    /// Get a device handle by its descriptor.
    fn get_device(&self, desc: &SpiDeviceDesc) -> Option<Self::Device>;
}

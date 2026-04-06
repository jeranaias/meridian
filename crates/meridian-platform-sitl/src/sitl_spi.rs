//! SITL SPI bus and device implementation.
//!
//! Simulates SPI register read/write using in-memory HashMaps.
//! Bit 7 of tx[0] distinguishes reads from writes (matching real IMU conventions).

use std::collections::HashMap;

use meridian_hal::spi::{SpiBus, SpiDevice, SpiDeviceDesc};
use meridian_hal::util::Semaphore;
use meridian_sync::{RecursiveMutex, TASK_ID_SPI_BUS};

/// Static semaphore for SPI bus locking using RecursiveMutex.
static SPI_SEM: RecursiveMutex<()> = RecursiveMutex::new(());

struct SitlSpiSemaphore;

impl Semaphore for SitlSpiSemaphore {
    fn take_blocking(&self) {
        let _guard = SPI_SEM.lock(TASK_ID_SPI_BUS);
    }

    fn take(&self, _timeout_us: u32) -> bool {
        let _guard = SPI_SEM.lock(TASK_ID_SPI_BUS);
        true
    }

    fn give(&self) {
        // Guard dropped automatically at scope exit.
    }
}

static SPI_SEMAPHORE: SitlSpiSemaphore = SitlSpiSemaphore;

/// Simulated SPI device holding per-register data.
///
/// Register convention (matching ICM-42688, BMI270, etc.):
///  - tx[0] bit 7 set  => READ from register (address = tx[0] & 0x7F)
///  - tx[0] bit 7 clear => WRITE to register (address = tx[0])
pub struct SitlSpiDevice {
    /// Register map: address -> data bytes.
    registers: HashMap<u8, Vec<u8>>,
    /// Current clock speed (Hz).
    speed_hz: u32,
    /// Device descriptor fields for device_id encoding.
    bus: u8,
    cs_pin: u16,
}

impl SitlSpiDevice {
    pub fn new(bus: u8, cs_pin: u16) -> Self {
        Self {
            registers: HashMap::new(),
            speed_hz: 1_000_000,
            bus,
            cs_pin,
        }
    }

    /// Pre-load a register value for test injection.
    pub fn set_register(&mut self, addr: u8, data: &[u8]) {
        self.registers.insert(addr & 0x7F, data.to_vec());
    }
}

impl SpiDevice for SitlSpiDevice {
    fn transfer(&mut self, tx: &[u8], rx: &mut [u8]) -> bool {
        if tx.is_empty() || tx.len() != rx.len() {
            return false;
        }

        // Zero the receive buffer.
        rx.fill(0);

        let addr = tx[0] & 0x7F;
        let is_read = (tx[0] & 0x80) != 0;

        if is_read {
            // Read: return register data starting at addr.
            if let Some(data) = self.registers.get(&addr) {
                let copy_len = (rx.len() - 1).min(data.len());
                rx[1..1 + copy_len].copy_from_slice(&data[..copy_len]);
            }
            // If register not found, rx stays zeroed (device returns 0x00
            // for uninitialised registers).
        } else {
            // Write: store bytes into register at addr.
            let payload = &tx[1..];
            if !payload.is_empty() {
                self.registers.insert(addr, payload.to_vec());
            }
        }

        true
    }

    fn write(&mut self, data: &[u8]) -> bool {
        if data.is_empty() {
            return false;
        }
        let addr = data[0] & 0x7F;
        let payload = &data[1..];
        if !payload.is_empty() {
            self.registers.insert(addr, payload.to_vec());
        }
        true
    }

    fn read(&mut self, data: &mut [u8]) -> bool {
        // Read-only transfer (TX zeros). Returns zeros.
        data.fill(0);
        true
    }

    fn set_speed(&mut self, speed_hz: u32) {
        self.speed_hz = speed_hz;
    }

    fn get_semaphore(&self) -> &dyn Semaphore {
        &SPI_SEMAPHORE
    }

    fn device_id(&self) -> u32 {
        // Encode bus + cs_pin into a unique 32-bit ID.
        ((self.bus as u32) << 16) | (self.cs_pin as u32)
    }
}

/// SITL SPI bus -- manages simulated devices.
pub struct SitlSpiBus {
    /// Pre-registered devices keyed by CS pin.
    devices: HashMap<u16, SitlSpiDevice>,
}

impl SitlSpiBus {
    pub fn new() -> Self {
        Self {
            devices: HashMap::new(),
        }
    }

    /// Register a simulated device on this bus (for test setup).
    pub fn add_device(&mut self, cs_pin: u16, device: SitlSpiDevice) {
        self.devices.insert(cs_pin, device);
    }
}

impl SpiBus for SitlSpiBus {
    type Device = SitlSpiDevice;

    fn get_device(&self, desc: &SpiDeviceDesc) -> Option<Self::Device> {
        // Return a fresh device configured per descriptor.
        let mut dev = SitlSpiDevice::new(desc.bus, desc.cs_pin);
        dev.speed_hz = desc.high_speed_hz;

        // Copy register state from pre-registered device if it exists.
        if let Some(existing) = self.devices.get(&desc.cs_pin) {
            dev.registers = existing.registers.clone();
        }

        Some(dev)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use meridian_hal::spi::SpiMode;

    #[test]
    fn test_spi_register_read_write() {
        let mut dev = SitlSpiDevice::new(0, 1);

        // Write register 0x10 with value 0xAB.
        let tx_w = [0x10, 0xAB];
        assert!(dev.write(&tx_w));

        // Read register 0x10 -- bit 7 set.
        let tx_r = [0x10 | 0x80, 0x00];
        let mut rx = [0u8; 2];
        assert!(dev.transfer(&tx_r, &mut rx));
        assert_eq!(rx[1], 0xAB);
    }

    #[test]
    fn test_spi_read_unset_register_returns_zero() {
        let mut dev = SitlSpiDevice::new(0, 1);
        let tx = [0x20 | 0x80, 0x00];
        let mut rx = [0xFFu8; 2];
        assert!(dev.transfer(&tx, &mut rx));
        assert_eq!(rx[1], 0x00);
    }

    #[test]
    fn test_spi_bus_get_device() {
        let bus = SitlSpiBus::new();
        let desc = SpiDeviceDesc {
            bus: 0,
            cs_pin: 5,
            mode: SpiMode::Mode0,
            low_speed_hz: 1_000_000,
            high_speed_hz: 8_000_000,
        };
        let dev = bus.get_device(&desc);
        assert!(dev.is_some());
    }

    #[test]
    fn test_spi_multi_byte_read() {
        let mut dev = SitlSpiDevice::new(0, 1);
        dev.set_register(0x3B, &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);

        // Read 6 bytes from register 0x3B (accelerometer-style burst read).
        let tx = [0x3B | 0x80, 0, 0, 0, 0, 0, 0];
        let mut rx = [0u8; 7];
        assert!(dev.transfer(&tx, &mut rx));
        assert_eq!(&rx[1..], &[0x01, 0x02, 0x03, 0x04, 0x05, 0x06]);
    }
}

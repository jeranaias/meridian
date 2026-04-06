//! SITL I2C device implementation.
//!
//! Simulates I2C register read/write using in-memory HashMaps.
//! Each I2C address maps to its own register bank, matching real
//! multi-device bus topology (e.g. compass at 0x1E, baro at 0x76).

use std::collections::HashMap;

use meridian_hal::i2c::I2cDevice;
use meridian_hal::util::Semaphore;
use meridian_sync::{RecursiveMutex, TASK_ID_I2C_BUS};

/// Static semaphore for I2C bus locking using RecursiveMutex.
static I2C_SEM: RecursiveMutex<()> = RecursiveMutex::new(());

struct SitlI2cSemaphore;

impl Semaphore for SitlI2cSemaphore {
    fn take_blocking(&self) {
        let _guard = I2C_SEM.lock(TASK_ID_I2C_BUS);
    }

    fn take(&self, _timeout_us: u32) -> bool {
        let _guard = I2C_SEM.lock(TASK_ID_I2C_BUS);
        true
    }

    fn give(&self) {}
}

static I2C_SEMAPHORE: SitlI2cSemaphore = SitlI2cSemaphore;

/// Per-address simulated device register bank.
struct DeviceRegisters {
    /// Register address -> data bytes.
    registers: HashMap<u8, Vec<u8>>,
}

impl DeviceRegisters {
    fn new() -> Self {
        Self {
            registers: HashMap::new(),
        }
    }
}

/// SITL I2C device -- simulates a full I2C bus with multiple addresses.
pub struct SitlI2c {
    /// Current target address.
    address: u8,
    /// Per-address device register maps.
    devices: HashMap<u8, DeviceRegisters>,
    /// Bus speed (Hz).
    speed_hz: u32,
    /// Bus index for device_id encoding.
    bus_index: u8,
}

impl SitlI2c {
    pub fn new() -> Self {
        Self {
            address: 0,
            devices: HashMap::new(),
            speed_hz: 400_000,
            bus_index: 0,
        }
    }

    /// Register a simulated device at the given I2C address.
    /// Must be called before transfer() can succeed for that address.
    pub fn add_device(&mut self, addr: u8) {
        self.devices.entry(addr).or_insert_with(DeviceRegisters::new);
    }

    /// Pre-load a register value on a simulated device (for test injection).
    pub fn set_register(&mut self, addr: u8, reg: u8, data: &[u8]) {
        let dev = self.devices.entry(addr).or_insert_with(DeviceRegisters::new);
        dev.registers.insert(reg, data.to_vec());
    }

    /// Set the bus index (used for device_id encoding).
    pub fn set_bus_index(&mut self, index: u8) {
        self.bus_index = index;
    }
}

impl I2cDevice for SitlI2c {
    fn set_address(&mut self, addr: u8) {
        self.address = addr;
    }

    fn transfer(&mut self, send: &[u8], recv: &mut [u8]) -> bool {
        let dev = match self.devices.get_mut(&self.address) {
            Some(d) => d,
            None => return false, // No device at this address.
        };

        if send.is_empty() && recv.is_empty() {
            // Empty transfer -- used for probe(), succeeds if device exists.
            return true;
        }

        if send.is_empty() {
            // Read-only: fill recv with zeros (no register specified).
            recv.fill(0);
            return true;
        }

        let reg = send[0];

        if send.len() > 1 {
            // Write: bytes after the register address are data.
            let payload = &send[1..];
            dev.registers.insert(reg, payload.to_vec());
        }

        if !recv.is_empty() {
            // Read: return register data.
            if let Some(data) = dev.registers.get(&reg) {
                let copy_len = recv.len().min(data.len());
                recv[..copy_len].copy_from_slice(&data[..copy_len]);
                // Zero any remaining bytes.
                for b in recv[copy_len..].iter_mut() {
                    *b = 0;
                }
            } else {
                // Register not set -- return zeros.
                recv.fill(0);
            }
        }

        true
    }

    fn get_semaphore(&self) -> &dyn Semaphore {
        &I2C_SEMAPHORE
    }

    fn set_speed(&mut self, speed_hz: u32) {
        self.speed_hz = speed_hz;
    }

    fn probe(&mut self) -> bool {
        self.devices.contains_key(&self.address)
    }

    fn device_id(&self) -> u32 {
        // Encode bus index + address into a unique 32-bit ID.
        ((self.bus_index as u32) << 16) | (self.address as u32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_i2c_probe_unregistered() {
        let mut i2c = SitlI2c::new();
        i2c.set_address(0x68);
        assert!(!i2c.probe());
    }

    #[test]
    fn test_i2c_probe_registered() {
        let mut i2c = SitlI2c::new();
        i2c.add_device(0x68);
        i2c.set_address(0x68);
        assert!(i2c.probe());
    }

    #[test]
    fn test_i2c_register_read_write() {
        let mut i2c = SitlI2c::new();
        i2c.add_device(0x76);
        i2c.set_address(0x76);

        // Write register 0x10 = [0xAB].
        assert!(i2c.transfer(&[0x10, 0xAB], &mut []));

        // Read register 0x10.
        let mut buf = [0u8; 1];
        assert!(i2c.transfer(&[0x10], &mut buf));
        assert_eq!(buf[0], 0xAB);
    }

    #[test]
    fn test_i2c_set_register_injection() {
        let mut i2c = SitlI2c::new();
        i2c.set_register(0x1E, 0x0A, &[0xDE, 0xAD]);
        i2c.set_address(0x1E);

        let mut buf = [0u8; 2];
        assert!(i2c.transfer(&[0x0A], &mut buf));
        assert_eq!(buf, [0xDE, 0xAD]);
    }

    #[test]
    fn test_i2c_transfer_nonexistent_device() {
        let mut i2c = SitlI2c::new();
        i2c.set_address(0x50);
        let mut buf = [0u8; 1];
        assert!(!i2c.transfer(&[0x00], &mut buf));
    }
}

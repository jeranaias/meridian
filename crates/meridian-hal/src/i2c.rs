//! I2C bus driver trait.
//!
//! Source: ArduPilot `AP_HAL/I2CDevice.h`
//! H7 uses TIMINGR register (0x00707CBB for 100kHz, 0x00300F38 for 400kHz).
//! Bus clear: 20 SCL pulses at boot to unstick devices.

use crate::util::Semaphore;

/// I2C device — one per chip on an I2C bus.
pub trait I2cDevice {
    /// Set the I2C address for subsequent operations.
    fn set_address(&mut self, addr: u8);

    /// Write then read (combined transfer). Standard I2C repeated-start.
    fn transfer(&mut self, send: &[u8], recv: &mut [u8]) -> bool;

    /// Write only.
    fn write(&mut self, data: &[u8]) -> bool {
        self.transfer(data, &mut [])
    }

    /// Read a single register.
    fn read_register(&mut self, reg: u8) -> Option<u8> {
        let mut val = [0u8; 1];
        if self.transfer(&[reg], &mut val) {
            Some(val[0])
        } else {
            None
        }
    }

    /// Read multiple registers starting at `reg`.
    fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> bool {
        self.transfer(&[reg], buf)
    }

    /// Write a single register.
    fn write_register(&mut self, reg: u8, value: u8) -> bool {
        self.transfer(&[reg, value], &mut [])
    }

    /// Get bus semaphore for exclusive access.
    fn get_semaphore(&self) -> &dyn Semaphore;

    /// Set bus clock speed (Hz). Drops permanently if lower than current.
    fn set_speed(&mut self, speed_hz: u32);

    /// Probe: check if a device responds at the current address.
    fn probe(&mut self) -> bool {
        let mut dummy = [0u8; 1];
        self.transfer(&[], &mut dummy)
    }

    /// Device ID for this chip (bus + address encoding).
    fn device_id(&self) -> u32;
}

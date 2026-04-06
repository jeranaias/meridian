//! SITL (Software-In-The-Loop) HAL implementation.
//!
//! Implements all meridian-hal traits for desktop Linux/Windows/macOS.
//! Used for testing, development, and running ArduPilot's Python test suite.
//!
//! Architecture:
//! - Physics bridge: UDP to `meridian-sitl` physics engine (port 5501)
//! - MAVLink server: TCP port 5760 for QGC/Mission Planner
//! - Simulated sensors: generated from physics state + noise
//! - File-backed storage: `eeprom.bin` for parameter persistence
//! - Real OS threads with std::time for scheduling

pub mod sitl_uart;
pub mod sitl_spi;
pub mod sitl_i2c;
pub mod sitl_gpio;
pub mod sitl_rc_output;
pub mod sitl_rc_input;
pub mod sitl_storage;
pub mod sitl_scheduler;
pub mod sitl_analog;
pub mod sitl_can;
pub mod sitl_util;

pub use sitl_uart::SitlUart;
pub use sitl_spi::{SitlSpiBus, SitlSpiDevice};
pub use sitl_i2c::SitlI2c;
pub use sitl_gpio::SitlGpio;
pub use sitl_rc_output::SitlRcOutput;
pub use sitl_rc_input::SitlRcInput;
pub use sitl_storage::SitlStorage;
pub use sitl_scheduler::SitlScheduler;
pub use sitl_analog::SitlAnalog;
pub use sitl_can::SitlCan;
pub use sitl_util::SitlUtil;

use meridian_hal::Hal;

/// The SITL HAL — complete platform implementation for desktop simulation.
pub struct SitlHal {
    pub uarts: [SitlUart; 10],
    pub spi_buses: [SitlSpiBus; 4],
    pub i2c_buses: [SitlI2c; 2],
    pub gpio: SitlGpio,
    pub rc_out: SitlRcOutput,
    pub rc_in: SitlRcInput,
    pub storage: SitlStorage,
    pub scheduler: SitlScheduler,
    pub analog: SitlAnalog,
    pub can_ifaces: [SitlCan; 2],
    pub util: SitlUtil,
}

impl SitlHal {
    pub fn new() -> Self {
        Self {
            uarts: core::array::from_fn(|_| SitlUart::new()),
            spi_buses: core::array::from_fn(|_| SitlSpiBus::new()),
            i2c_buses: core::array::from_fn(|_| SitlI2c::new()),
            gpio: SitlGpio::new(),
            rc_out: SitlRcOutput::new(),
            rc_in: SitlRcInput::new(),
            storage: SitlStorage::new(),
            scheduler: SitlScheduler::new(),
            analog: SitlAnalog::new(),
            can_ifaces: core::array::from_fn(|_| SitlCan::new()),
            util: SitlUtil::new(),
        }
    }
}

impl Hal for SitlHal {
    type Uart = SitlUart;
    type Spi = SitlSpiBus;
    type I2c = SitlI2c;
    type Gpio = SitlGpio;
    type RcOut = SitlRcOutput;
    type RcIn = SitlRcInput;
    type Store = SitlStorage;
    type Sched = SitlScheduler;
    type Adc = SitlAnalog;
    type Can = SitlCan;
    type Util = SitlUtil;

    fn serial(&self, index: u8) -> Option<&Self::Uart> {
        self.uarts.get(index as usize)
    }
    fn spi(&self, bus: u8) -> Option<&Self::Spi> {
        self.spi_buses.get(bus as usize)
    }
    fn i2c(&self, bus: u8) -> Option<&Self::I2c> {
        self.i2c_buses.get(bus as usize)
    }
    fn gpio(&self) -> &Self::Gpio { &self.gpio }
    fn rcout(&self) -> &Self::RcOut { &self.rc_out }
    fn rcin(&self) -> &Self::RcIn { &self.rc_in }
    fn storage(&self) -> &Self::Store { &self.storage }
    fn scheduler(&self) -> &Self::Sched { &self.scheduler }
    fn analogin(&self) -> &Self::Adc { &self.analog }
    fn can(&self, index: u8) -> Option<&Self::Can> {
        self.can_ifaces.get(index as usize)
    }
    fn util(&self) -> &Self::Util { &self.util }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sitl_hal_creates() {
        let hal = SitlHal::new();
        assert!(hal.serial(0).is_some());
        assert!(hal.serial(9).is_some());
        assert!(hal.serial(10).is_none());
        assert!(hal.spi(0).is_some());
        assert!(hal.i2c(0).is_some());
        assert!(hal.can(0).is_some());
        assert!(hal.can(2).is_none());
    }
}

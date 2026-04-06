#![no_std]

//! Hardware Abstraction Layer traits for Meridian.
//!
//! Every platform (STM32/RTIC, Linux, SITL) implements these traits.
//! All sensor drivers and subsystems program against these traits, never
//! against hardware registers directly.
//!
//! Source: ArduPilot `libraries/AP_HAL/AP_HAL.h` — every interface mapped to Rust.

pub mod uart;
pub mod spi;
pub mod i2c;
pub mod gpio;
pub mod rc_output;
pub mod rc_input;
pub mod storage;
pub mod scheduler;
pub mod analog;
pub mod can;
pub mod util;
pub mod relay;
pub mod rtc;

pub use uart::UartDriver;
pub use spi::{SpiDevice, SpiBus};
pub use i2c::I2cDevice;
pub use gpio::{GpioPin, PinMode};
pub use rc_output::RcOutput;
pub use rc_input::RcInput;
pub use storage::Storage;
pub use scheduler::{Scheduler, TimerProc, IoProc};
pub use analog::{AnalogIn, AnalogSource};
pub use can::CanIface;
pub use util::Util;

/// The complete HAL — one struct holding all platform abstractions.
/// Each platform provides a concrete implementation.
pub trait Hal {
    type Uart: UartDriver;
    type Spi: SpiBus;
    type I2c: I2cDevice;
    type Gpio: GpioPin;
    type RcOut: RcOutput;
    type RcIn: RcInput;
    type Store: Storage;
    type Sched: Scheduler;
    type Adc: AnalogIn;
    type Can: CanIface;
    type Util: Util;

    fn serial(&self, index: u8) -> Option<&Self::Uart>;
    fn spi(&self, bus: u8) -> Option<&Self::Spi>;
    fn i2c(&self, bus: u8) -> Option<&Self::I2c>;
    fn gpio(&self) -> &Self::Gpio;
    fn rcout(&self) -> &Self::RcOut;
    fn rcin(&self) -> &Self::RcIn;
    fn storage(&self) -> &Self::Store;
    fn scheduler(&self) -> &Self::Sched;
    fn analogin(&self) -> &Self::Adc;
    fn can(&self, index: u8) -> Option<&Self::Can>;
    fn util(&self) -> &Self::Util;
}

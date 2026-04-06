//! GPIO pin control trait.
//!
//! Source: ArduPilot `AP_HAL/GPIO.h`

/// Pin mode configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinMode {
    Input,
    InputPullUp,
    InputPullDown,
    Output,
    OutputOpenDrain,
    Alternate(u8), // alternate function number (0-15)
}

/// Interrupt trigger edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Edge {
    Rising,
    Falling,
    Both,
}

/// GPIO pin abstraction.
pub trait GpioPin {
    /// Configure pin mode.
    fn set_mode(&mut self, pin: u16, mode: PinMode);

    /// Read pin state (true = high).
    fn read(&self, pin: u16) -> bool;

    /// Write pin state.
    fn write(&mut self, pin: u16, value: bool);

    /// Toggle pin.
    fn toggle(&mut self, pin: u16);

    /// Attach interrupt handler to a pin.
    /// The callback receives the pin number and current state.
    fn attach_interrupt(&mut self, pin: u16, edge: Edge, callback: fn(u16, bool)) -> bool;

    /// Detach interrupt from a pin.
    fn detach_interrupt(&mut self, pin: u16);

    /// Check if a pin number is valid for this platform.
    fn pin_valid(&self, pin: u16) -> bool;
}

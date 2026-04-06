//! SITL GPIO — simulated pins backed by a flat array.

use meridian_hal::gpio::{Edge, GpioPin, PinMode};

/// Maximum simulated GPIO pins.
const MAX_PINS: usize = 64;

/// SITL GPIO driver.
pub struct SitlGpio {
    pin_state: [bool; MAX_PINS],
    pin_mode: [PinMode; MAX_PINS],
}

impl SitlGpio {
    pub fn new() -> Self {
        Self {
            pin_state: [false; MAX_PINS],
            pin_mode: [PinMode::Input; MAX_PINS],
        }
    }
}

impl GpioPin for SitlGpio {
    fn set_mode(&mut self, pin: u16, mode: PinMode) {
        if (pin as usize) < MAX_PINS {
            self.pin_mode[pin as usize] = mode;
        }
    }

    fn read(&self, pin: u16) -> bool {
        if (pin as usize) < MAX_PINS {
            self.pin_state[pin as usize]
        } else {
            false
        }
    }

    fn write(&mut self, pin: u16, value: bool) {
        if (pin as usize) < MAX_PINS {
            self.pin_state[pin as usize] = value;
        }
    }

    fn toggle(&mut self, pin: u16) {
        if (pin as usize) < MAX_PINS {
            self.pin_state[pin as usize] = !self.pin_state[pin as usize];
        }
    }

    fn attach_interrupt(&mut self, _pin: u16, _edge: Edge, _callback: fn(u16, bool)) -> bool {
        // No interrupt simulation in SITL.
        false
    }

    fn detach_interrupt(&mut self, _pin: u16) {}

    fn pin_valid(&self, pin: u16) -> bool {
        (pin as usize) < MAX_PINS
    }
}

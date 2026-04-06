//! SITL analog input — simulated ADC channels.

use meridian_hal::analog::{AnalogIn, AnalogSource};

/// SITL analog source — returns configurable voltage.
pub struct SitlAnalogSource {
    voltage: f32,
    pin: u8,
}

impl SitlAnalogSource {
    fn new(pin: u8, voltage: f32) -> Self {
        Self { voltage, pin }
    }

    /// Set the simulated voltage for this source (test injection).
    pub fn set_voltage(&mut self, voltage: f32) {
        self.voltage = voltage;
    }
}

impl AnalogSource for SitlAnalogSource {
    fn voltage_average(&self) -> f32 {
        self.voltage
    }

    fn voltage_latest(&self) -> f32 {
        self.voltage
    }

    fn set_pin(&mut self, pin: u8) {
        self.pin = pin;
    }

    fn set_stop_pin(&mut self, _pin: u8) {}
}

/// Maximum analog channels.
const MAX_CHANNELS: usize = 16;

/// SITL analog input driver.
pub struct SitlAnalog {
    channels: Vec<SitlAnalogSource>,
    board_voltage: f32,
    servorail_voltage: f32,
}

impl SitlAnalog {
    pub fn new() -> Self {
        let channels = (0..MAX_CHANNELS as u8)
            .map(|i| SitlAnalogSource::new(i, 0.0))
            .collect();
        Self {
            channels,
            board_voltage: 5.0,
            servorail_voltage: 5.0,
        }
    }

    /// Get a mutable reference to a channel for test injection.
    pub fn channel_mut(&mut self, channel: u8) -> Option<&mut SitlAnalogSource> {
        self.channels.get_mut(channel as usize)
    }
}

impl AnalogIn for SitlAnalog {
    fn init(&mut self) {}

    fn channel(&self, channel: u8) -> Option<&dyn AnalogSource> {
        self.channels.get(channel as usize).map(|s| s as &dyn AnalogSource)
    }

    fn board_voltage(&self) -> f32 {
        self.board_voltage
    }

    fn servorail_voltage(&self) -> f32 {
        self.servorail_voltage
    }
}

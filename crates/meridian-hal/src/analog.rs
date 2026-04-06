//! Analog input (ADC) trait — battery voltage, current, servo rail.
//!
//! Source: ArduPilot `AP_HAL/AnalogIn.h`
//! H7: ADC with DMA for continuous background sampling.

/// A single analog input source (channel).
pub trait AnalogSource {
    /// Read voltage (V). Returns filtered average.
    fn voltage_average(&self) -> f32;

    /// Read latest raw voltage (V), unfiltered.
    fn voltage_latest(&self) -> f32;

    /// Set pin for this source.
    fn set_pin(&mut self, pin: u8);

    /// Stop reading this source.
    fn set_stop_pin(&mut self, pin: u8);
}

/// Analog input driver — manages all ADC channels.
pub trait AnalogIn {
    /// Initialize all ADC channels.
    fn init(&mut self);

    /// Get an analog source for the given channel.
    fn channel(&self, channel: u8) -> Option<&dyn AnalogSource>;

    /// Board supply voltage (V). Typically 5.0V regulated.
    fn board_voltage(&self) -> f32;

    /// Servo rail voltage (V). May differ from board voltage.
    fn servorail_voltage(&self) -> f32;
}

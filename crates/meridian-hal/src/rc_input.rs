//! RC receiver input trait.
//!
//! Source: ArduPilot `AP_HAL/RCInput.h`
//! Input protocols: CRSF, SBUS, SRXL2, DSM, PPM, IBUS, SUMD, FPort, ST24.

/// Maximum RC channels.
pub const MAX_RC_CHANNELS: usize = 18;

/// RC input driver.
pub trait RcInput {
    /// Check if new input is available since last read.
    fn new_input(&self) -> bool;

    /// Number of valid channels in last frame.
    fn num_channels(&self) -> u8;

    /// Read channel value (1000-2000 us range, center 1500).
    fn read(&self, channel: u8) -> u16;

    /// Read all channels at once.
    fn read_all(&self, values: &mut [u16]) -> u8;

    /// Set override value for a channel (GCS override).
    fn set_override(&mut self, channel: u8, value: u16);

    /// Clear all overrides.
    fn clear_overrides(&mut self);

    /// Whether the RC link is in failsafe state.
    fn in_failsafe(&self) -> bool;

    /// Time since last valid RC frame (microseconds).
    fn last_input_us(&self) -> u64;

    /// RSSI (0-255, 255=full signal) if available.
    fn rssi(&self) -> Option<u8>;

    /// Link quality percentage (0-100) if available (CRSF/ELRS).
    fn link_quality(&self) -> Option<u8>;
}

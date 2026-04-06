//! RC/Motor output trait — PWM, DShot, OneShot.
//!
//! Source: ArduPilot `AP_HAL/RCOutput.h`
//! DShot uses timer DMAR burst — DMA writes CCR registers for all channels simultaneously.
//! Bidirectional DShot requires SRAM4 (uncached) on H7.

/// DShot zero-throttle constant. Values 0-47 are command slots; 48 = zero throttle.
/// Source: AP_HAL/RCOutput.h line 276: `static constexpr uint8_t DSHOT_ZERO_THROTTLE = 48`
pub const DSHOT_ZERO_THROTTLE: u16 = 48;

/// Output protocol type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputProtocol {
    Pwm,
    OneShot125,
    DShot150,
    DShot300,
    DShot600,
    DShot1200,
    /// Bidirectional DShot: ESC reports RPM telemetry on the same wire.
    /// Requires SRAM4 (uncached) on STM32H7 for reliable DMA operation.
    /// Source: AP_HAL/RCOutput.h BDSHOT
    BidirectionalDShot,
}

/// DShot command IDs (sent as throttle values 0-47).
/// Source: AP_HAL/RCOutput.h DshotCommandType
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DshotCommand {
    MotorStop = 0,
    Beep1 = 1,
    Beep2 = 2,
    Beep3 = 3,
    Beep4 = 4,
    Beep5 = 5,
    EscInfo = 6,
    SpinDirection1 = 7,
    SpinDirection2 = 8,
    ThreeDModeOff = 9,
    ThreeDModeOn = 10,
    SaveSettings = 12,
    SpinDirectionNormal = 20,
    SpinDirectionReversed = 21,
    Led0On = 22,
    Led1On = 23,
    Led2On = 24,
    Led3On = 25,
    Led0Off = 26,
    Led1Off = 27,
    Led2Off = 28,
    Led3Off = 29,
}

/// Maximum number of output channels.
pub const MAX_OUTPUT_CHANNELS: usize = 16;

/// RC/Motor output driver.
pub trait RcOutput {
    /// Set PWM frequency for a group of channels (Hz).
    fn set_freq(&mut self, channel_mask: u16, freq_hz: u16);

    /// Get PWM frequency for a channel.
    fn get_freq(&self, channel: u8) -> u16;

    /// Set output value for a channel (PWM: 1000-2000us, DShot: 0-2047).
    fn write(&mut self, channel: u8, value: u16);

    /// Set output for multiple channels at once.
    fn write_multi(&mut self, values: &[u16]);

    /// Read back the last written value for a channel.
    fn read(&self, channel: u8) -> u16;

    /// Push all pending output values to hardware.
    /// Called once per loop iteration after all channels are set.
    fn push(&mut self);

    /// Cork: buffer writes until push() is called.
    /// Prevents partial updates from reaching ESCs.
    fn cork(&mut self);

    /// Set the output protocol for a channel group.
    fn set_protocol(&mut self, channel_mask: u16, protocol: OutputProtocol);

    /// Set DShot rate multiplier (1x = once per loop, 2x = twice, etc.)
    fn set_dshot_rate(&mut self, rate: u8);

    /// Send a DShot command to specific channels.
    fn send_dshot_command(&mut self, command: DshotCommand, channel_mask: u16, repeat: u8);

    /// Number of output channels available on this platform.
    fn num_channels(&self) -> u8;

    /// Enable/disable output for a channel (safety).
    fn enable_ch(&mut self, channel: u8);
    fn disable_ch(&mut self, channel: u8);

    /// Get ESC telemetry data (RPM, voltage, current, temperature) if available.
    fn get_esc_telemetry(&self, channel: u8) -> Option<EscTelemetry>;
}

/// ESC telemetry data from bidirectional DShot or BLHeli telemetry.
#[derive(Debug, Clone, Copy, Default)]
pub struct EscTelemetry {
    pub rpm: u32,
    pub voltage_mv: u16,
    pub current_ca: u16,
    pub temperature_c: u8,
}

//! SITL RC/Motor output — stores channel values in memory.
//!
//! No real PWM/DShot hardware. Values are readable by the physics bridge
//! to compute motor thrust.

use meridian_hal::rc_output::{
    DshotCommand, EscTelemetry, OutputProtocol, RcOutput, MAX_OUTPUT_CHANNELS,
};

/// SITL RC output driver.
pub struct SitlRcOutput {
    /// Per-channel output values (PWM us or DShot 0-2047).
    channels: [u16; MAX_OUTPUT_CHANNELS],
    /// Per-channel frequency (Hz). Grouped by timer on real HW, flat here.
    freq_hz: [u16; MAX_OUTPUT_CHANNELS],
    /// Per-channel protocol.
    protocol: [OutputProtocol; MAX_OUTPUT_CHANNELS],
    /// Per-channel enabled flag.
    enabled: [bool; MAX_OUTPUT_CHANNELS],
    /// Cork flag — when true, writes are buffered until push().
    corked: bool,
    /// DShot rate multiplier.
    dshot_rate: u8,
}

impl SitlRcOutput {
    pub fn new() -> Self {
        Self {
            channels: [1000; MAX_OUTPUT_CHANNELS],
            freq_hz: [50; MAX_OUTPUT_CHANNELS],
            protocol: [OutputProtocol::Pwm; MAX_OUTPUT_CHANNELS],
            enabled: [false; MAX_OUTPUT_CHANNELS],
            corked: false,
            dshot_rate: 1,
        }
    }

    /// Read all channel values (used by physics bridge to compute thrust).
    pub fn get_all_channels(&self) -> &[u16; MAX_OUTPUT_CHANNELS] {
        &self.channels
    }

    /// Check whether a channel is enabled.
    pub fn is_enabled(&self, channel: u8) -> bool {
        self.enabled
            .get(channel as usize)
            .copied()
            .unwrap_or(false)
    }

    /// Check whether output is currently corked.
    pub fn is_corked(&self) -> bool {
        self.corked
    }
}

impl RcOutput for SitlRcOutput {
    fn set_freq(&mut self, channel_mask: u16, freq_hz: u16) {
        for ch in 0..MAX_OUTPUT_CHANNELS {
            if (channel_mask >> ch) & 1 == 1 {
                self.freq_hz[ch] = freq_hz;
            }
        }
    }

    fn get_freq(&self, channel: u8) -> u16 {
        self.freq_hz
            .get(channel as usize)
            .copied()
            .unwrap_or(0)
    }

    fn write(&mut self, channel: u8, value: u16) {
        if let Some(v) = self.channels.get_mut(channel as usize) {
            *v = value;
        }
    }

    fn write_multi(&mut self, values: &[u16]) {
        let n = values.len().min(MAX_OUTPUT_CHANNELS);
        self.channels[..n].copy_from_slice(&values[..n]);
    }

    fn read(&self, channel: u8) -> u16 {
        self.channels
            .get(channel as usize)
            .copied()
            .unwrap_or(0)
    }

    fn push(&mut self) {
        // No-op in SITL — no hardware to push to.
        // The physics bridge reads channel values directly via get_all_channels().
        self.corked = false;
    }

    fn cork(&mut self) {
        self.corked = true;
    }

    fn set_protocol(&mut self, channel_mask: u16, protocol: OutputProtocol) {
        for ch in 0..MAX_OUTPUT_CHANNELS {
            if (channel_mask >> ch) & 1 == 1 {
                self.protocol[ch] = protocol;
            }
        }
    }

    fn set_dshot_rate(&mut self, rate: u8) {
        self.dshot_rate = rate;
    }

    fn send_dshot_command(
        &mut self,
        _command: DshotCommand,
        _channel_mask: u16,
        _repeat: u8,
    ) {
        // No-op in SITL — no ESC to receive DShot commands.
    }

    fn num_channels(&self) -> u8 {
        MAX_OUTPUT_CHANNELS as u8
    }

    fn enable_ch(&mut self, channel: u8) {
        if let Some(e) = self.enabled.get_mut(channel as usize) {
            *e = true;
        }
    }

    fn disable_ch(&mut self, channel: u8) {
        if let Some(e) = self.enabled.get_mut(channel as usize) {
            *e = false;
        }
    }

    fn get_esc_telemetry(&self, _channel: u8) -> Option<EscTelemetry> {
        None // No ESC telemetry in SITL.
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_write_read() {
        let mut rc = SitlRcOutput::new();
        rc.write(0, 1500);
        rc.write(3, 1800);
        assert_eq!(rc.read(0), 1500);
        assert_eq!(rc.read(3), 1800);
        assert_eq!(rc.read(1), 1000); // default
    }

    #[test]
    fn test_write_multi() {
        let mut rc = SitlRcOutput::new();
        rc.write_multi(&[1100, 1200, 1300, 1400]);
        assert_eq!(rc.read(0), 1100);
        assert_eq!(rc.read(3), 1400);
        assert_eq!(rc.read(4), 1000); // untouched
    }

    #[test]
    fn test_num_channels() {
        let rc = SitlRcOutput::new();
        assert_eq!(rc.num_channels(), 16);
    }

    #[test]
    fn test_set_freq() {
        let mut rc = SitlRcOutput::new();
        rc.set_freq(0b1111, 400);
        assert_eq!(rc.get_freq(0), 400);
        assert_eq!(rc.get_freq(3), 400);
        assert_eq!(rc.get_freq(4), 50); // unchanged
    }

    #[test]
    fn test_cork_push() {
        let mut rc = SitlRcOutput::new();
        rc.cork();
        assert!(rc.is_corked());
        rc.push();
        assert!(!rc.is_corked());
    }

    #[test]
    fn test_enable_disable() {
        let mut rc = SitlRcOutput::new();
        assert!(!rc.is_enabled(0));
        rc.enable_ch(0);
        assert!(rc.is_enabled(0));
        rc.disable_ch(0);
        assert!(!rc.is_enabled(0));
    }

    #[test]
    fn test_set_protocol() {
        let mut rc = SitlRcOutput::new();
        rc.set_protocol(0b0011, OutputProtocol::DShot600);
        assert_eq!(rc.protocol[0], OutputProtocol::DShot600);
        assert_eq!(rc.protocol[1], OutputProtocol::DShot600);
        assert_eq!(rc.protocol[2], OutputProtocol::Pwm); // unchanged
    }

    #[test]
    fn test_out_of_bounds() {
        let mut rc = SitlRcOutput::new();
        rc.write(20, 1500); // should not panic
        assert_eq!(rc.read(20), 0);
    }
}

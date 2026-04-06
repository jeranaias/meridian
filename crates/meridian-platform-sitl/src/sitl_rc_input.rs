//! SITL RC receiver input — channels injected by the physics bridge.
//!
//! The SITL physics bridge (or a joystick adapter) calls `inject_channels()`
//! to push RC data. The flight code reads via the `RcInput` trait as usual.

use meridian_hal::rc_input::{RcInput, MAX_RC_CHANNELS};

/// SITL RC input driver.
pub struct SitlRcInput {
    /// Channel values (1000-2000 us, center 1500).
    channels: [u16; MAX_RC_CHANNELS],
    /// Number of valid channels in the last injected frame.
    num_channels: u8,
    /// True if new data has been injected since last read.
    new_input: bool,
    /// Timestamp of last injection (microseconds since boot).
    last_input_us: u64,
    /// Failsafe flag — set by the physics bridge when simulating link loss.
    failsafe: bool,
    /// Per-channel GCS overrides (0 = no override).
    overrides: [u16; MAX_RC_CHANNELS],
    /// Simulated RSSI (0-255).
    rssi: Option<u8>,
    /// Simulated link quality (0-100).
    link_quality: Option<u8>,
}

impl SitlRcInput {
    pub fn new() -> Self {
        Self {
            channels: [1500; MAX_RC_CHANNELS],
            num_channels: 0,
            new_input: false,
            last_input_us: 0,
            failsafe: false,
            overrides: [0; MAX_RC_CHANNELS],
            rssi: Some(255),
            link_quality: Some(100),
        }
    }

    /// Inject RC channel data from the physics bridge or joystick adapter.
    ///
    /// `values` contains channel values in PWM microseconds (1000-2000).
    /// `timestamp_us` is the microsecond timestamp of the frame.
    pub fn inject_channels(&mut self, values: &[u16], timestamp_us: u64) {
        let n = values.len().min(MAX_RC_CHANNELS);
        self.channels[..n].copy_from_slice(&values[..n]);
        self.num_channels = n as u8;
        self.new_input = true;
        self.last_input_us = timestamp_us;
    }

    /// Set the failsafe flag (for simulating RC link loss).
    pub fn set_failsafe(&mut self, failsafe: bool) {
        self.failsafe = failsafe;
    }

    /// Set simulated RSSI value.
    pub fn set_rssi(&mut self, rssi: Option<u8>) {
        self.rssi = rssi;
    }

    /// Set simulated link quality.
    pub fn set_link_quality(&mut self, lq: Option<u8>) {
        self.link_quality = lq;
    }

    /// Get the effective value for a channel, considering overrides.
    fn effective_value(&self, channel: u8) -> u16 {
        let idx = channel as usize;
        if idx < MAX_RC_CHANNELS && self.overrides[idx] != 0 {
            self.overrides[idx]
        } else {
            self.channels.get(idx).copied().unwrap_or(0)
        }
    }
}

impl RcInput for SitlRcInput {
    fn new_input(&self) -> bool {
        self.new_input
    }

    fn num_channels(&self) -> u8 {
        self.num_channels
    }

    fn read(&self, channel: u8) -> u16 {
        self.effective_value(channel)
    }

    fn read_all(&self, values: &mut [u16]) -> u8 {
        let n = values.len().min(self.num_channels as usize);
        for i in 0..n {
            values[i] = self.effective_value(i as u8);
        }
        n as u8
    }

    fn set_override(&mut self, channel: u8, value: u16) {
        if let Some(v) = self.overrides.get_mut(channel as usize) {
            *v = value;
        }
    }

    fn clear_overrides(&mut self) {
        self.overrides = [0; MAX_RC_CHANNELS];
    }

    fn in_failsafe(&self) -> bool {
        self.failsafe
    }

    fn last_input_us(&self) -> u64 {
        self.last_input_us
    }

    fn rssi(&self) -> Option<u8> {
        self.rssi
    }

    fn link_quality(&self) -> Option<u8> {
        self.link_quality
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_inject_and_read() {
        let mut rc = SitlRcInput::new();
        assert!(!rc.new_input());
        assert_eq!(rc.num_channels(), 0);

        rc.inject_channels(&[1000, 1500, 1200, 1800], 100_000);
        assert!(rc.new_input());
        assert_eq!(rc.num_channels(), 4);
        assert_eq!(rc.read(0), 1000);
        assert_eq!(rc.read(1), 1500);
        assert_eq!(rc.read(3), 1800);
        assert_eq!(rc.last_input_us(), 100_000);
    }

    #[test]
    fn test_overrides() {
        let mut rc = SitlRcInput::new();
        rc.inject_channels(&[1000, 1500, 1200, 1800], 100_000);

        rc.set_override(1, 1700);
        assert_eq!(rc.read(1), 1700); // override wins
        assert_eq!(rc.read(0), 1000); // no override

        rc.clear_overrides();
        assert_eq!(rc.read(1), 1500); // back to injected value
    }

    #[test]
    fn test_failsafe() {
        let mut rc = SitlRcInput::new();
        assert!(!rc.in_failsafe());

        rc.set_failsafe(true);
        assert!(rc.in_failsafe());
    }

    #[test]
    fn test_read_all() {
        let mut rc = SitlRcInput::new();
        rc.inject_channels(&[1100, 1200, 1300], 50_000);

        let mut buf = [0u16; 8];
        let n = rc.read_all(&mut buf);
        assert_eq!(n, 3);
        assert_eq!(buf[0], 1100);
        assert_eq!(buf[1], 1200);
        assert_eq!(buf[2], 1300);
    }

    #[test]
    fn test_read_all_with_override() {
        let mut rc = SitlRcInput::new();
        rc.inject_channels(&[1100, 1200, 1300], 50_000);
        rc.set_override(1, 1900);

        let mut buf = [0u16; 8];
        let n = rc.read_all(&mut buf);
        assert_eq!(n, 3);
        assert_eq!(buf[0], 1100);
        assert_eq!(buf[1], 1900); // override
        assert_eq!(buf[2], 1300);
    }

    #[test]
    fn test_rssi_and_link_quality() {
        let mut rc = SitlRcInput::new();
        assert_eq!(rc.rssi(), Some(255));
        assert_eq!(rc.link_quality(), Some(100));

        rc.set_rssi(Some(128));
        rc.set_link_quality(Some(75));
        assert_eq!(rc.rssi(), Some(128));
        assert_eq!(rc.link_quality(), Some(75));

        rc.set_rssi(None);
        assert_eq!(rc.rssi(), None);
    }
}

//! PPM (Pulse Position Modulation) decoder.
//!
//! Pulse timing based: 1ms-2ms channel pulses with >3ms sync gap.
//! Up to 12 channels.
//! Decoded from timer input capture interrupts.
//! In SITL: simulated with known timing values.
//! Failsafe: no pulses for >100ms.

use crate::{RcFrame, MAX_CHANNELS};

const PPM_MAX_CHANNELS: usize = 12;
const PPM_SYNC_GAP_US: u16 = 3000;   // >3ms sync gap
const PPM_MIN_PULSE_US: u16 = 800;   // minimum valid pulse
const PPM_MAX_PULSE_US: u16 = 2200;  // maximum valid pulse
const PPM_FAILSAFE_US: u32 = 100_000; // 100ms timeout

/// PPM decoder state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PpmState {
    /// Waiting for sync gap.
    WaitingSync,
    /// Collecting channel pulses.
    Collecting,
}

/// PPM pulse decoder.
///
/// PPM works by measuring time between rising edges. A gap >3ms indicates
/// the start of a new frame. Each subsequent pulse width encodes a channel
/// value (1000-2000us).
pub struct PpmDecoder {
    channels: [u16; MAX_CHANNELS],
    channel_idx: usize,
    state: PpmState,
    last_edge_us: u32,
    last_frame_us: u32,
    /// True if no valid frame received within timeout.
    pub failsafe: bool,
    /// Number of successfully decoded frames.
    pub frames_decoded: u32,
}

impl PpmDecoder {
    pub const fn new() -> Self {
        Self {
            channels: [1500; MAX_CHANNELS],
            channel_idx: 0,
            state: PpmState::WaitingSync,
            last_edge_us: 0,
            last_frame_us: 0,
            failsafe: true,
            frames_decoded: 0,
        }
    }

    /// Process a rising-edge capture event.
    ///
    /// `timestamp_us` is the microsecond timestamp of the rising edge.
    /// Returns `Some(RcFrame)` when a complete PPM frame is decoded
    /// (i.e., when a sync gap is detected after collecting channels).
    pub fn feed_edge(&mut self, timestamp_us: u32) -> Option<RcFrame> {
        let elapsed = timestamp_us.wrapping_sub(self.last_edge_us);
        self.last_edge_us = timestamp_us;

        // Check for failsafe timeout
        if timestamp_us.wrapping_sub(self.last_frame_us) > PPM_FAILSAFE_US {
            self.failsafe = true;
        }

        if elapsed as u16 >= PPM_SYNC_GAP_US || elapsed >= PPM_SYNC_GAP_US as u32 {
            // Sync gap detected -- finish previous frame if we have channels
            let result = if self.state == PpmState::Collecting && self.channel_idx > 0 {
                let count = self.channel_idx.min(PPM_MAX_CHANNELS);
                self.last_frame_us = timestamp_us;
                self.failsafe = false;
                self.frames_decoded += 1;
                Some(RcFrame {
                    channels: self.channels,
                    count: count as u8,
                    failsafe: false,
                    rssi: 100,
                })
            } else {
                None
            };

            // Start new frame
            self.state = PpmState::Collecting;
            self.channel_idx = 0;
            // Reset channels for new frame
            self.channels = [1500; MAX_CHANNELS];

            return result;
        }

        if self.state == PpmState::Collecting {
            let pulse_us = elapsed as u16;
            if pulse_us >= PPM_MIN_PULSE_US
                && pulse_us <= PPM_MAX_PULSE_US
                && self.channel_idx < PPM_MAX_CHANNELS
            {
                self.channels[self.channel_idx] = pulse_us;
                self.channel_idx += 1;
            } else if pulse_us < PPM_MIN_PULSE_US || pulse_us > PPM_MAX_PULSE_US {
                // Invalid pulse, corrupt frame -- wait for next sync
                self.state = PpmState::WaitingSync;
                self.channel_idx = 0;
            }
        }

        None
    }

    /// Feed a complete set of pulse widths (for SITL simulation).
    ///
    /// `pulse_widths_us` is a slice of channel pulse widths in microseconds.
    /// This simulates a full PPM frame without needing real timer captures.
    pub fn feed_frame(&mut self, pulse_widths_us: &[u16]) -> Option<RcFrame> {
        let count = pulse_widths_us.len().min(PPM_MAX_CHANNELS);
        let mut channels = [1500u16; MAX_CHANNELS];

        for (i, &pw) in pulse_widths_us.iter().enumerate().take(count) {
            if pw >= PPM_MIN_PULSE_US && pw <= PPM_MAX_PULSE_US {
                channels[i] = pw;
            } else {
                return None; // invalid pulse width
            }
        }

        self.failsafe = false;
        self.frames_decoded += 1;

        Some(RcFrame {
            channels,
            count: count as u8,
            failsafe: false,
            rssi: 100,
        })
    }

    /// Check if failsafe should be triggered based on elapsed time.
    pub fn check_timeout(&mut self, current_us: u32) {
        if current_us.wrapping_sub(self.last_frame_us) > PPM_FAILSAFE_US {
            self.failsafe = true;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ppm_basic_frame() {
        let mut decoder = PpmDecoder::new();
        assert!(decoder.failsafe);

        // Simulate PPM timing: sync gap, then 6 channel pulses, then next sync gap
        let mut time: u32 = 0;

        // First sync gap (>3ms) -- starts frame collection
        time += 5000;
        assert!(decoder.feed_edge(time).is_none());

        // 6 channels at center (1500us each)
        for _ in 0..6 {
            time += 1500;
            assert!(decoder.feed_edge(time).is_none());
        }

        // Next sync gap -- completes the frame
        time += 5000;
        let result = decoder.feed_edge(time);
        assert!(result.is_some());

        let rc = result.unwrap();
        assert_eq!(rc.count, 6);
        assert!(!rc.failsafe);
        for i in 0..6 {
            assert_eq!(rc.channels[i], 1500, "Channel {} = {}", i, rc.channels[i]);
        }
        assert!(!decoder.failsafe);
    }

    #[test]
    fn test_ppm_varying_channels() {
        let mut decoder = PpmDecoder::new();
        let mut time: u32 = 0;

        // Sync gap
        time += 5000;
        decoder.feed_edge(time);

        // 4 channels with different values
        let pulses = [1000u16, 1250, 1750, 2000];
        for &pw in &pulses {
            time += pw as u32;
            decoder.feed_edge(time);
        }

        // Sync gap to complete frame
        time += 5000;
        let result = decoder.feed_edge(time).unwrap();

        assert_eq!(result.count, 4);
        assert_eq!(result.channels[0], 1000);
        assert_eq!(result.channels[1], 1250);
        assert_eq!(result.channels[2], 1750);
        assert_eq!(result.channels[3], 2000);
    }

    #[test]
    fn test_ppm_failsafe_timeout() {
        let mut decoder = PpmDecoder::new();
        decoder.failsafe = false;
        decoder.last_frame_us = 0;

        // Check timeout at 50ms -- no failsafe yet (within 100ms)
        decoder.check_timeout(50_000);
        assert!(!decoder.failsafe);

        // Check timeout at 150ms -- failsafe triggered
        decoder.check_timeout(150_000);
        assert!(decoder.failsafe);
    }

    #[test]
    fn test_ppm_feed_frame_sitl() {
        let mut decoder = PpmDecoder::new();
        let pulses = [1100u16, 1300, 1500, 1700, 1900, 1500];
        let result = decoder.feed_frame(&pulses).unwrap();

        assert_eq!(result.count, 6);
        assert_eq!(result.channels[0], 1100);
        assert_eq!(result.channels[4], 1900);
        assert!(!decoder.failsafe);
    }

    #[test]
    fn test_ppm_feed_frame_invalid_pulse() {
        let mut decoder = PpmDecoder::new();
        // Pulse too low
        let bad_pulses = [500u16, 1500, 1500];
        assert!(decoder.feed_frame(&bad_pulses).is_none());

        // Pulse too high
        let bad_pulses2 = [1500u16, 1500, 2500];
        assert!(decoder.feed_frame(&bad_pulses2).is_none());
    }

    #[test]
    fn test_ppm_max_channels() {
        let mut decoder = PpmDecoder::new();
        let mut time: u32 = 0;

        // Sync
        time += 5000;
        decoder.feed_edge(time);

        // 12 channels (max)
        for _ in 0..12 {
            time += 1500;
            decoder.feed_edge(time);
        }

        // Sync to complete
        time += 5000;
        let result = decoder.feed_edge(time).unwrap();
        assert_eq!(result.count, 12);
    }
}

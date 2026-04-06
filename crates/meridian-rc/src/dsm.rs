//! Spektrum DSM/DSM2/DSMX protocol parser.
//!
//! 16-byte frames. First 2 bytes: fades(u8) + system(u8).
//! Remaining 14 bytes: 7 channels x 2 bytes each.
//! DSM2 = 10-bit resolution, DSMX = 11-bit resolution.
//! Channel IDs are used directly as array indices (no reorder).
//! Failsafe: detected by timeout (no explicit flag).

use crate::{RcFrame, MAX_CHANNELS};

const DSM_FRAME_LEN: usize = 16;
const DSM_CHANNELS_PER_FRAME: usize = 7;

/// DSM protocol variant, determined from system byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DsmVariant {
    /// DSM2 with 10-bit channel resolution.
    Dsm2,
    /// DSMX with 11-bit channel resolution.
    Dsmx,
    /// Unknown variant.
    Unknown,
}

/// Spektrum DSM/DSM2/DSMX parser.
pub struct DsmParser {
    buf: [u8; DSM_FRAME_LEN],
    pos: usize,
    /// Number of frames received (for timeout-based failsafe detection).
    pub frames_received: u32,
    /// Detected protocol variant.
    pub variant: DsmVariant,
    /// True if no frame has been received within the expected interval.
    pub failsafe: bool,
}

impl DsmParser {
    pub const fn new() -> Self {
        Self {
            buf: [0; DSM_FRAME_LEN],
            pos: 0,
            frames_received: 0,
            variant: DsmVariant::Unknown,
            failsafe: true, // failsafe until first frame
        }
    }

    /// Feed one byte. Returns `Some(RcFrame)` when a complete 16-byte frame is assembled.
    ///
    /// DSM has no sync byte, so the caller must ensure bytes are aligned to frame
    /// boundaries (typically via UART idle-line detection). This parser simply
    /// collects 16 bytes and parses.
    pub fn feed_byte(&mut self, byte: u8) -> Option<RcFrame> {
        self.buf[self.pos] = byte;
        self.pos += 1;

        if self.pos >= DSM_FRAME_LEN {
            let result = self.parse_frame();
            self.pos = 0;
            return result;
        }

        None
    }

    /// Feed a complete 16-byte frame at once.
    pub fn feed_frame(&mut self, frame: &[u8]) -> Option<RcFrame> {
        if frame.len() != DSM_FRAME_LEN {
            return None;
        }
        self.buf.copy_from_slice(frame);
        self.pos = 0;
        self.parse_frame()
    }

    /// Mark failsafe (called externally when timeout is detected, >22ms no data).
    pub fn set_failsafe(&mut self) {
        self.failsafe = true;
    }

    fn detect_variant(&mut self, system_byte: u8) {
        // System byte encodes protocol info.
        // Common values: 0x01/0x02 = DSM2 1024(10-bit), 0x12 = DSM2 2048(11-bit),
        // 0xA2/0xB2 = DSMX, etc.
        // Bit 4 set typically means 11-bit (2048) resolution.
        if system_byte & 0x10 != 0 {
            self.variant = DsmVariant::Dsmx;
        } else {
            self.variant = DsmVariant::Dsm2;
        }
    }

    fn parse_frame(&mut self) -> Option<RcFrame> {
        let _fades = self.buf[0];
        let system = self.buf[1];

        self.detect_variant(system);

        let is_11bit = self.variant == DsmVariant::Dsmx;
        let mut channels = [1500u16; MAX_CHANNELS];
        let mut max_chan = 0u8;

        for i in 0..DSM_CHANNELS_PER_FRAME {
            let offset = 2 + i * 2;
            let raw16 = ((self.buf[offset] as u16) << 8) | (self.buf[offset + 1] as u16);

            let (chan_id, value) = if is_11bit {
                // 11-bit: top 4 bits = channel ID, bottom 11 bits = value
                let id = (raw16 >> 11) & 0x0F;
                let val = raw16 & 0x07FF;
                (id, val)
            } else {
                // 10-bit: top 6 bits = channel ID (really top 2 for phase, next 4 for channel),
                // bottom 10 bits = value
                let id = (raw16 >> 10) & 0x3F;
                let chan = id & 0x0F;
                let val = raw16 & 0x03FF;
                (chan, val)
            };

            if chan_id < MAX_CHANNELS as u16 {
                // Convert to standard 1000-2000 range
                let pwm = if is_11bit {
                    // 11-bit: 0-2047, center at 1024 -> map to 1000-2000
                    ((value as u32 * 1000) / 2047 + 1000).min(2000) as u16
                } else {
                    // 10-bit: 0-1023, center at 512 -> map to 1000-2000
                    ((value as u32 * 1000) / 1023 + 1000).min(2000) as u16
                };

                // DSM channel IDs are used directly — no reorder needed.
                // ArduPilot does NOT reorder DSM channels.
                let idx = chan_id as usize;
                if idx < MAX_CHANNELS {
                    channels[idx] = pwm;
                    if idx as u8 + 1 > max_chan {
                        max_chan = idx as u8 + 1;
                    }
                }
            }
        }

        self.frames_received += 1;
        self.failsafe = false;

        Some(RcFrame {
            channels,
            count: max_chan.max(7),
            failsafe: false,
            rssi: 100,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a DSM frame with known channel values.
    fn build_dsm_frame(system: u8, channels: &[(u16, u16)], is_11bit: bool) -> [u8; 16] {
        let mut frame = [0u8; 16];
        frame[0] = 0; // fades = 0
        frame[1] = system;
        for (i, &(chan_id, value)) in channels.iter().enumerate().take(7) {
            let raw = if is_11bit {
                ((chan_id & 0x0F) << 11) | (value & 0x07FF)
            } else {
                ((chan_id & 0x3F) << 10) | (value & 0x03FF)
            };
            frame[2 + i * 2] = (raw >> 8) as u8;
            frame[2 + i * 2 + 1] = (raw & 0xFF) as u8;
        }
        frame
    }

    #[test]
    fn test_dsm2_center_sticks() {
        let frame = build_dsm_frame(
            0x01, // DSM2
            &[
                (0, 512), // ch0 center
                (1, 512), // ch1 center
                (2, 512), // ch2 center
                (3, 512),
                (4, 512),
                (5, 512),
                (6, 512),
            ],
            false,
        );
        let mut parser = DsmParser::new();
        let result = parser.feed_frame(&frame).unwrap();

        assert_eq!(parser.variant, DsmVariant::Dsm2);
        assert!(!result.failsafe);
        // 10-bit: 512/1023*1000 + 1000 ~ 1501
        // Channel IDs are used directly — no reorder.
        assert!(result.channels[0] >= 1490 && result.channels[0] <= 1510,
            "ch0 = {}", result.channels[0]);
        assert!(result.channels[1] >= 1490 && result.channels[1] <= 1510,
            "ch1 = {}", result.channels[1]);
        assert!(result.channels[2] >= 1490 && result.channels[2] <= 1510,
            "ch2 = {}", result.channels[2]);
    }

    #[test]
    fn test_dsmx_full_range() {
        let frame = build_dsm_frame(
            0x12, // DSMX (bit 4 set)
            &[
                (0, 0),    // ch0 minimum
                (1, 1024), // ch1 center
                (2, 2047), // ch2 maximum
                (3, 512),
                (4, 1500),
                (5, 100),
                (6, 1900),
            ],
            true,
        );
        let mut parser = DsmParser::new();
        let result = parser.feed_frame(&frame).unwrap();

        assert_eq!(parser.variant, DsmVariant::Dsmx);
        // Channel IDs used directly — no reorder.
        // ch0: value=0 -> 1000
        assert_eq!(result.channels[0], 1000);
        // ch1: value=1024 -> ~1500
        assert!(result.channels[1] >= 1490 && result.channels[1] <= 1510,
            "ch1 = {}", result.channels[1]);
        // ch2: value=2047 -> 2000
        assert_eq!(result.channels[2], 2000);
    }

    #[test]
    fn test_dsm_failsafe_initial_and_recovery() {
        let mut parser = DsmParser::new();
        assert!(parser.failsafe);

        let frame = build_dsm_frame(0x12, &[
            (0, 1024), (1, 1024), (2, 1024),
            (3, 1024), (4, 1024), (5, 1024), (6, 1024),
        ], true);

        let _ = parser.feed_frame(&frame);
        assert!(!parser.failsafe);

        // External timeout triggers failsafe
        parser.set_failsafe();
        assert!(parser.failsafe);
    }

    #[test]
    fn test_dsm_byte_by_byte() {
        let frame = build_dsm_frame(0x12, &[
            (0, 1024), (1, 1024), (2, 1024),
            (3, 1024), (4, 1024), (5, 1024), (6, 1024),
        ], true);

        let mut parser = DsmParser::new();
        let mut result = None;
        for &byte in &frame {
            if let Some(r) = parser.feed_byte(byte) {
                result = Some(r);
            }
        }
        assert!(result.is_some());
        assert!(!result.unwrap().failsafe);
    }

    #[test]
    fn test_dsm_wrong_length_rejected() {
        let mut parser = DsmParser::new();
        let short = [0u8; 8];
        assert!(parser.feed_frame(&short).is_none());
    }
}

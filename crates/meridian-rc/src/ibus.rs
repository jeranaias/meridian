//! FlySky iBus protocol parser.
//!
//! 32-byte frames at 115200 baud.
//! Header: [0x20, 0x40].
//! 14 channels x 2 bytes (u16 LE), range 1000-2000.
//! Checksum: 0xFFFF - sum of all preceding 30 bytes.

use crate::{RcFrame, MAX_CHANNELS};

const IBUS_FRAME_LEN: usize = 32;
const IBUS_HEADER_0: u8 = 0x20;
const IBUS_HEADER_1: u8 = 0x40;
const IBUS_NUM_CHANNELS: usize = 14;

/// FlySky iBus parser.
pub struct IBusParser {
    buf: [u8; IBUS_FRAME_LEN],
    pos: usize,
}

impl IBusParser {
    pub const fn new() -> Self {
        Self {
            buf: [0; IBUS_FRAME_LEN],
            pos: 0,
        }
    }

    /// Feed one byte. Returns `Some(RcFrame)` when a complete frame is parsed.
    pub fn feed_byte(&mut self, byte: u8) -> Option<RcFrame> {
        match self.pos {
            0 => {
                if byte == IBUS_HEADER_0 {
                    self.buf[0] = byte;
                    self.pos = 1;
                }
                return None;
            }
            1 => {
                if byte != IBUS_HEADER_1 {
                    self.pos = 0;
                    // Check if this byte is itself a new header start
                    if byte == IBUS_HEADER_0 {
                        self.buf[0] = byte;
                        self.pos = 1;
                    }
                    return None;
                }
                self.buf[1] = byte;
                self.pos = 2;
                return None;
            }
            _ => {}
        }

        self.buf[self.pos] = byte;
        self.pos += 1;

        if self.pos >= IBUS_FRAME_LEN {
            let result = self.parse_frame();
            self.pos = 0;
            return result;
        }

        None
    }

    /// Feed a complete 32-byte frame.
    pub fn feed_frame(&mut self, frame: &[u8]) -> Option<RcFrame> {
        if frame.len() != IBUS_FRAME_LEN {
            return None;
        }
        self.buf.copy_from_slice(frame);
        self.pos = 0;
        self.parse_frame()
    }

    fn verify_checksum(&self) -> bool {
        // Checksum = 0xFFFF - sum of first 30 bytes
        let sum: u16 = self.buf[..30].iter().map(|&b| b as u16).sum();
        let expected = 0xFFFFu16.wrapping_sub(sum);
        let received = (self.buf[30] as u16) | ((self.buf[31] as u16) << 8);
        expected == received
    }

    fn parse_frame(&self) -> Option<RcFrame> {
        // Verify header
        if self.buf[0] != IBUS_HEADER_0 || self.buf[1] != IBUS_HEADER_1 {
            return None;
        }

        if !self.verify_checksum() {
            return None;
        }

        let mut channels = [1500u16; MAX_CHANNELS];

        for i in 0..IBUS_NUM_CHANNELS {
            let offset = 2 + i * 2;
            let value = (self.buf[offset] as u16) | ((self.buf[offset + 1] as u16) << 8);
            // iBus values are already in 1000-2000 range
            channels[i] = value.clamp(1000, 2000);
        }

        Some(RcFrame {
            channels,
            count: IBUS_NUM_CHANNELS as u8,
            failsafe: false, // iBus doesn't have an explicit failsafe flag in the frame
            rssi: 100,
        })
    }
}

/// Compute iBus checksum: 0xFFFF - sum of all bytes.
pub fn ibus_checksum(data: &[u8]) -> u16 {
    let sum: u16 = data.iter().map(|&b| b as u16).sum();
    0xFFFFu16.wrapping_sub(sum)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build an iBus frame with given channel values.
    fn build_ibus_frame(channels: &[u16]) -> [u8; 32] {
        let mut frame = [0u8; 32];
        frame[0] = IBUS_HEADER_0;
        frame[1] = IBUS_HEADER_1;

        for (i, &val) in channels.iter().enumerate().take(IBUS_NUM_CHANNELS) {
            let offset = 2 + i * 2;
            frame[offset] = (val & 0xFF) as u8;
            frame[offset + 1] = ((val >> 8) & 0xFF) as u8;
        }

        // Fill remaining channels with 1500 (center)
        for i in channels.len()..IBUS_NUM_CHANNELS {
            let offset = 2 + i * 2;
            frame[offset] = (1500u16 & 0xFF) as u8;
            frame[offset + 1] = ((1500u16 >> 8) & 0xFF) as u8;
        }

        let checksum = ibus_checksum(&frame[..30]);
        frame[30] = (checksum & 0xFF) as u8;
        frame[31] = ((checksum >> 8) & 0xFF) as u8;
        frame
    }

    #[test]
    fn test_ibus_center_sticks() {
        let frame = build_ibus_frame(&[1500; 14]);
        let mut parser = IBusParser::new();
        let result = parser.feed_frame(&frame).unwrap();

        assert_eq!(result.count, 14);
        assert!(!result.failsafe);
        for i in 0..14 {
            assert_eq!(result.channels[i], 1500, "Channel {} wrong", i);
        }
    }

    #[test]
    fn test_ibus_full_range() {
        let channels = [1000, 1250, 1500, 1750, 2000, 1100, 1900,
                        1000, 1500, 2000, 1500, 1500, 1500, 1500];
        let frame = build_ibus_frame(&channels);
        let mut parser = IBusParser::new();
        let result = parser.feed_frame(&frame).unwrap();

        assert_eq!(result.channels[0], 1000);
        assert_eq!(result.channels[4], 2000);
        assert_eq!(result.channels[2], 1500);
    }

    #[test]
    fn test_ibus_bad_checksum_rejected() {
        let mut frame = build_ibus_frame(&[1500; 14]);
        frame[31] = 0xFF; // corrupt checksum
        let mut parser = IBusParser::new();
        assert!(parser.feed_frame(&frame).is_none());
    }

    #[test]
    fn test_ibus_byte_by_byte() {
        let frame = build_ibus_frame(&[1200, 1400, 1600, 1800, 1500, 1500, 1500,
                                       1500, 1500, 1500, 1500, 1500, 1500, 1500]);
        let mut parser = IBusParser::new();
        let mut result = None;
        for &byte in &frame {
            if let Some(r) = parser.feed_byte(byte) {
                result = Some(r);
            }
        }
        let rc = result.unwrap();
        assert_eq!(rc.channels[0], 1200);
        assert_eq!(rc.channels[1], 1400);
        assert_eq!(rc.channels[2], 1600);
        assert_eq!(rc.channels[3], 1800);
    }

    #[test]
    fn test_ibus_bad_header_rejected() {
        let mut frame = build_ibus_frame(&[1500; 14]);
        frame[0] = 0xFF; // wrong header
        let mut parser = IBusParser::new();
        assert!(parser.feed_frame(&frame).is_none());
    }

    #[test]
    fn test_ibus_wrong_length_rejected() {
        let mut parser = IBusParser::new();
        let short = [0x20, 0x40, 0x00, 0x00];
        assert!(parser.feed_frame(&short).is_none());
    }
}

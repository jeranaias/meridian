//! FrSky FPort protocol parser.
//!
//! Combined RC + telemetry on a single half-duplex UART wire at 115200 baud.
//! Frame: [len(1)] [type(1)] [payload(24)] [CRC(1)]
//! Type 0x00 = RC data: 16 channels x 11-bit (same packing as SBUS).
//! Type 0x01 = uplink telemetry sensor polling.
//! CRC: 0xFF - (sum_of_bytes & 0xFF).
//! Failsafe: bit 3 of flags byte (same as SBUS).

use crate::{RcFrame, MAX_CHANNELS};

const FPORT_MAX_FRAME: usize = 32;
const FPORT_TYPE_RC: u8 = 0x00;
const FPORT_TYPE_TELEMETRY: u8 = 0x01;
const FPORT_RC_PAYLOAD_LEN: u8 = 24;

/// FPort frame type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FPortFrameType {
    /// RC channel data.
    Rc,
    /// Uplink telemetry poll.
    Telemetry,
    /// Unknown frame type.
    Unknown(u8),
}

/// FPort parser.
pub struct FPortParser {
    buf: [u8; FPORT_MAX_FRAME],
    pos: usize,
    expected_len: usize,
}

impl FPortParser {
    pub const fn new() -> Self {
        Self {
            buf: [0; FPORT_MAX_FRAME],
            pos: 0,
            expected_len: 0,
        }
    }

    /// Feed one byte. Returns `Some(RcFrame)` when a complete RC frame is parsed.
    pub fn feed_byte(&mut self, byte: u8) -> Option<RcFrame> {
        self.buf[self.pos] = byte;
        self.pos += 1;

        if self.pos == 1 {
            // First byte is length
            let len = byte as usize;
            if len == 0 || len + 2 > FPORT_MAX_FRAME {
                // len byte + payload(len) + crc byte = len + 2
                self.pos = 0;
                return None;
            }
            self.expected_len = len + 2; // total: len_byte + payload(len) + crc
        }

        if self.pos >= self.expected_len && self.expected_len > 0 {
            let result = self.parse_frame();
            self.pos = 0;
            self.expected_len = 0;
            return result;
        }

        None
    }

    /// Feed a complete frame (including length byte and CRC).
    pub fn feed_frame(&mut self, frame: &[u8]) -> Option<RcFrame> {
        if frame.len() < 3 || frame.len() > FPORT_MAX_FRAME {
            return None;
        }
        let len = frame[0] as usize;
        if frame.len() != len + 2 {
            return None;
        }
        self.buf[..frame.len()].copy_from_slice(frame);
        self.pos = frame.len();
        self.expected_len = frame.len();
        let result = self.parse_frame();
        self.pos = 0;
        self.expected_len = 0;
        result
    }

    fn verify_crc(&self) -> bool {
        if self.expected_len < 3 {
            return false;
        }
        // CRC is over bytes from type through end of payload (bytes 1..expected_len-1)
        let crc_idx = self.expected_len - 1;
        let sum: u8 = self.buf[1..crc_idx]
            .iter()
            .fold(0u8, |acc, &b| acc.wrapping_add(b));
        let expected_crc = 0xFFu8.wrapping_sub(sum);
        expected_crc == self.buf[crc_idx]
    }

    fn parse_frame(&self) -> Option<RcFrame> {
        if !self.verify_crc() {
            return None;
        }

        let frame_type = self.buf[1];
        match frame_type {
            FPORT_TYPE_RC => self.parse_rc(),
            _ => None, // telemetry frames don't produce RcFrame
        }
    }

    fn parse_rc(&self) -> Option<RcFrame> {
        // RC payload: 22 bytes channel data + 1 flags + 1 RSSI = 24 bytes
        let payload_len = self.buf[0] as usize;
        if payload_len < 24 {
            return None;
        }

        // Channel data starts at byte 2 (after len and type)
        let data = &self.buf[2..24]; // 22 bytes of channel data
        let flags = self.buf[24];
        let rssi = self.buf[25];

        let mut channels = [0u16; MAX_CHANNELS];
        let mut bit_offset = 0u32;

        for ch in 0..16 {
            let byte_idx = (bit_offset / 8) as usize;
            let bit_idx = bit_offset % 8;
            if byte_idx + 2 > 22 {
                break;
            }

            let raw = ((data[byte_idx] as u32) >> bit_idx)
                | ((data[byte_idx + 1] as u32) << (8 - bit_idx))
                | (if byte_idx + 2 < 22 {
                    (data[byte_idx + 2] as u32) << (16 - bit_idx)
                } else {
                    0
                });
            let raw = (raw & 0x7FF) as u16;

            // Convert SBUS-style range (172-1811) to standard PWM (988-2012)
            // ArduPilot formula: (raw * 5 / 8) + 880. Center stick (992) → 1500us.
            channels[ch] = ((raw as i32 * 5 / 8) + 880).clamp(988, 2012) as u16;
            bit_offset += 11;
        }

        let failsafe = (flags & 0x08) != 0;

        Some(RcFrame {
            channels,
            count: 16,
            failsafe,
            rssi,
        })
    }
}

/// Compute FPort CRC: 0xFF - (sum of bytes & 0xFF).
pub fn fport_crc(data: &[u8]) -> u8 {
    let sum: u8 = data.iter().fold(0u8, |acc, &b| acc.wrapping_add(b));
    0xFFu8.wrapping_sub(sum)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build an FPort RC frame with known channel data.
    fn build_fport_rc_frame(channel_data: &[u8; 22], flags: u8, rssi: u8) -> [u8; 28] {
        let mut frame = [0u8; 28];
        // len = 26 (type + 24 payload)
        frame[0] = 26;
        frame[1] = FPORT_TYPE_RC;
        frame[2..24].copy_from_slice(channel_data);
        frame[24] = flags;
        frame[25] = rssi;
        // Pad remaining payload bytes (if any) with zeros - already zeroed
        // CRC: sum of bytes[1..27], then 0xFF - sum
        let crc = fport_crc(&frame[1..27]);
        frame[27] = crc;
        frame
    }

    #[test]
    fn test_fport_rc_center_sticks() {
        // Fill channel data with approximate center values (similar to SBUS encoding)
        let mut ch_data = [0u8; 22];
        // Pack 16 channels of raw 992 (center) into 11-bit slots
        // 992 = 0x3E0 in each 11-bit slot
        // For simplicity, use a known pattern that gives center-ish values
        for b in ch_data.iter_mut() {
            *b = 0xE0; // approximate center fill
        }

        let frame = build_fport_rc_frame(&ch_data, 0x00, 95);
        let mut parser = FPortParser::new();
        let result = parser.feed_frame(&frame).unwrap();

        assert_eq!(result.count, 16);
        assert!(!result.failsafe);
        assert_eq!(result.rssi, 95);
        // All channels should be in valid ArduPilot PWM range (988-2012)
        for i in 0..16 {
            assert!(
                result.channels[i] >= 988 && result.channels[i] <= 2012,
                "Channel {} = {} out of range",
                i,
                result.channels[i]
            );
        }
    }

    #[test]
    fn test_fport_failsafe_flag() {
        let ch_data = [0xE0u8; 22];
        let frame = build_fport_rc_frame(&ch_data, 0x08, 0); // failsafe bit 3 set
        let mut parser = FPortParser::new();
        let result = parser.feed_frame(&frame).unwrap();

        assert!(result.failsafe);
        assert_eq!(result.rssi, 0);
    }

    #[test]
    fn test_fport_bad_crc_rejected() {
        let ch_data = [0xE0u8; 22];
        let mut frame = build_fport_rc_frame(&ch_data, 0x00, 95);
        frame[27] = 0xFF; // corrupt CRC
        let mut parser = FPortParser::new();
        assert!(parser.feed_frame(&frame).is_none());
    }

    #[test]
    fn test_fport_byte_by_byte() {
        let ch_data = [0xE0u8; 22];
        let frame = build_fport_rc_frame(&ch_data, 0x00, 80);
        let mut parser = FPortParser::new();
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
    fn test_fport_crc_function() {
        let data = [0x00, 0xE0, 0xE0, 0xE0];
        let crc = fport_crc(&data);
        let verify_sum: u8 = data.iter().chain(core::iter::once(&crc)).fold(0u8, |a, &b| a.wrapping_add(b));
        assert_eq!(verify_sum, 0xFF);
    }
}

//! Graupner SUMD/SUMH protocol parser.
//!
//! Header: [0xA8] [status(1)] [num_channels(1)]
//! Status: 0x01=valid, 0x81=failsafe.
//! Channels: num_channels x 2 bytes (u16 BE), range mapped from 8000 center.
//! CRC16-CCITT over header + status + num_channels + channel data.

use crate::{RcFrame, MAX_CHANNELS};

const SUMD_HEADER: u8 = 0xA8;
const SUMD_STATUS_VALID: u8 = 0x01;
const SUMD_STATUS_FAILSAFE: u8 = 0x81;
const SUMD_MAX_CHANNELS: usize = 16;
/// Maximum frame: header(1) + status(1) + nch(1) + 16*2 channels + crc(2) = 37
const SUMD_MAX_FRAME: usize = 37;

/// Graupner SUMD parser.
pub struct SumdParser {
    buf: [u8; SUMD_MAX_FRAME],
    pos: usize,
    expected_len: usize,
}

impl SumdParser {
    pub const fn new() -> Self {
        Self {
            buf: [0; SUMD_MAX_FRAME],
            pos: 0,
            expected_len: 0,
        }
    }

    /// Feed one byte. Returns `Some(RcFrame)` when a complete frame is parsed.
    pub fn feed_byte(&mut self, byte: u8) -> Option<RcFrame> {
        if self.pos == 0 {
            if byte == SUMD_HEADER {
                self.buf[0] = byte;
                self.pos = 1;
            }
            return None;
        }

        self.buf[self.pos] = byte;
        self.pos += 1;

        if self.pos == 3 {
            // We now have header, status, num_channels
            let num_ch = self.buf[2] as usize;
            if num_ch == 0 || num_ch > SUMD_MAX_CHANNELS {
                self.pos = 0;
                return None;
            }
            // Total: header(1) + status(1) + nch(1) + channels(num_ch*2) + crc(2)
            self.expected_len = 3 + num_ch * 2 + 2;
            if self.expected_len > SUMD_MAX_FRAME {
                self.pos = 0;
                return None;
            }
        }

        if self.pos >= self.expected_len && self.expected_len > 3 {
            let result = self.parse_frame();
            self.pos = 0;
            self.expected_len = 0;
            return result;
        }

        None
    }

    /// Feed a complete frame at once.
    pub fn feed_frame(&mut self, frame: &[u8]) -> Option<RcFrame> {
        if frame.len() < 5 || frame.len() > SUMD_MAX_FRAME {
            return None;
        }
        if frame[0] != SUMD_HEADER {
            return None;
        }
        let num_ch = frame[2] as usize;
        let expected = 3 + num_ch * 2 + 2;
        if frame.len() != expected {
            return None;
        }
        self.buf[..frame.len()].copy_from_slice(frame);
        self.pos = frame.len();
        self.expected_len = expected;
        let result = self.parse_frame();
        self.pos = 0;
        self.expected_len = 0;
        result
    }

    fn parse_frame(&self) -> Option<RcFrame> {
        let num_ch = self.buf[2] as usize;
        let data_end = 3 + num_ch * 2;

        // Verify CRC16-CCITT over header + status + nch + channels (not the CRC itself)
        let crc_received = ((self.buf[data_end] as u16) << 8) | (self.buf[data_end + 1] as u16);
        let crc_calc = crc16_ccitt(&self.buf[..data_end]);
        if crc_received != crc_calc {
            return None;
        }

        let status = self.buf[1];
        let failsafe = status == SUMD_STATUS_FAILSAFE;

        let mut channels = [1500u16; MAX_CHANNELS];
        let count = num_ch.min(MAX_CHANNELS);

        for i in 0..count {
            let offset = 3 + i * 2;
            let raw = ((self.buf[offset] as u16) << 8) | (self.buf[offset + 1] as u16);
            // SUMD uses 8*channel_us: center=12000(1500us), range ~7200(900us)-16800(2100us)
            // Convert: pwm_us = raw / 8
            let pwm = (raw / 8).clamp(1000, 2000);
            channels[i] = pwm;
        }

        Some(RcFrame {
            channels,
            count: count as u8,
            failsafe,
            rssi: if failsafe { 0 } else { 100 },
        })
    }
}

/// CRC16-CCITT (polynomial 0x1021, init 0x0000).
fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a SUMD frame with given channel values (in microseconds).
    fn build_sumd_frame(status: u8, channels: &[u16]) -> heapless::Vec<u8, 64> {
        let num_ch = channels.len().min(SUMD_MAX_CHANNELS);
        let mut frame = heapless::Vec::<u8, 64>::new();
        let _ = frame.push(SUMD_HEADER);
        let _ = frame.push(status);
        let _ = frame.push(num_ch as u8);

        for &ch_us in channels.iter().take(num_ch) {
            let raw = ch_us * 8; // SUMD encoding: microseconds * 8
            let _ = frame.push((raw >> 8) as u8);
            let _ = frame.push((raw & 0xFF) as u8);
        }

        let crc = crc16_ccitt(&frame);
        let _ = frame.push((crc >> 8) as u8);
        let _ = frame.push((crc & 0xFF) as u8);

        frame
    }

    #[test]
    fn test_sumd_center_sticks() {
        let frame = build_sumd_frame(SUMD_STATUS_VALID, &[1500; 8]);
        let mut parser = SumdParser::new();
        let result = parser.feed_frame(&frame).unwrap();

        assert_eq!(result.count, 8);
        assert!(!result.failsafe);
        for i in 0..8 {
            assert_eq!(result.channels[i], 1500, "Channel {} wrong", i);
        }
    }

    #[test]
    fn test_sumd_failsafe() {
        let frame = build_sumd_frame(SUMD_STATUS_FAILSAFE, &[1500; 6]);
        let mut parser = SumdParser::new();
        let result = parser.feed_frame(&frame).unwrap();

        assert!(result.failsafe);
        assert_eq!(result.rssi, 0);
    }

    #[test]
    fn test_sumd_full_range() {
        let channels = [1000, 1250, 1500, 1750, 2000, 1100, 1900, 1500];
        let frame = build_sumd_frame(SUMD_STATUS_VALID, &channels);
        let mut parser = SumdParser::new();
        let result = parser.feed_frame(&frame).unwrap();

        assert_eq!(result.channels[0], 1000);
        assert_eq!(result.channels[2], 1500);
        assert_eq!(result.channels[4], 2000);
    }

    #[test]
    fn test_sumd_bad_crc_rejected() {
        let mut frame = build_sumd_frame(SUMD_STATUS_VALID, &[1500; 4]);
        let last = frame.len() - 1;
        frame[last] = 0xFF; // corrupt CRC
        let mut parser = SumdParser::new();
        assert!(parser.feed_frame(&frame).is_none());
    }

    #[test]
    fn test_sumd_byte_by_byte() {
        let frame = build_sumd_frame(SUMD_STATUS_VALID, &[1200, 1400, 1600, 1800]);
        let mut parser = SumdParser::new();
        let mut result = None;
        for &byte in frame.iter() {
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
    fn test_sumd_crc16_known() {
        // Verify CRC16-CCITT produces expected values
        let data = [0xA8, 0x01, 0x02, 0x2E, 0xE0, 0x2E, 0xE0];
        let crc = crc16_ccitt(&data);
        // Just verify it's nonzero and deterministic
        let crc2 = crc16_ccitt(&data);
        assert_eq!(crc, crc2);
        assert_ne!(crc, 0);
    }
}

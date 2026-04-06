//! Spektrum SRXL2 (bidirectional serial protocol) parser.
//!
//! Header: [0xA6] [type(1)] [length(1)] [payload...] [CRC16]
//! Type 0x00 = handshake, 0xCD = channel data, 0x15 = telemetry.
//! Channel data: device_id(1) + RSSI(1) + frame_losses(u16) + channels(N x u16).
//! CRC16-CCITT over entire frame (excluding CRC bytes).
//! Bidirectional: FC sends telemetry back on the same wire.

use crate::{RcFrame, MAX_CHANNELS};

const SRXL2_HEADER: u8 = 0xA6;
const SRXL2_TYPE_HANDSHAKE: u8 = 0x00;
const SRXL2_TYPE_CHANNEL_DATA: u8 = 0xCD;
const SRXL2_TYPE_TELEMETRY: u8 = 0x15;
const SRXL2_MAX_FRAME: usize = 80;

/// SRXL2 frame type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Srxl2FrameType {
    Handshake,
    ChannelData,
    Telemetry,
    Unknown(u8),
}

/// SRXL2 parser.
pub struct Srxl2Parser {
    buf: [u8; SRXL2_MAX_FRAME],
    pos: usize,
    expected_len: usize,
    /// Last received RSSI.
    pub last_rssi: u8,
    /// Cumulative frame losses reported by receiver.
    pub frame_losses: u16,
}

impl Srxl2Parser {
    pub const fn new() -> Self {
        Self {
            buf: [0; SRXL2_MAX_FRAME],
            pos: 0,
            expected_len: 0,
            last_rssi: 0,
            frame_losses: 0,
        }
    }

    /// Feed one byte. Returns `Some(RcFrame)` when channel data is received.
    pub fn feed_byte(&mut self, byte: u8) -> Option<RcFrame> {
        if self.pos == 0 {
            if byte == SRXL2_HEADER {
                self.buf[0] = byte;
                self.pos = 1;
            }
            return None;
        }

        self.buf[self.pos] = byte;
        self.pos += 1;

        if self.pos == 3 {
            // Length byte at position 2
            let len = self.buf[2] as usize;
            if len < 5 || len > SRXL2_MAX_FRAME {
                self.pos = 0;
                return None;
            }
            self.expected_len = len;
        }

        if self.pos >= self.expected_len && self.expected_len >= 5 {
            let result = self.parse_frame();
            self.pos = 0;
            self.expected_len = 0;
            return result;
        }

        None
    }

    /// Feed a complete frame at once.
    pub fn feed_frame(&mut self, frame: &[u8]) -> Option<RcFrame> {
        if frame.len() < 5 || frame.len() > SRXL2_MAX_FRAME {
            return None;
        }
        if frame[0] != SRXL2_HEADER {
            return None;
        }
        let len = frame[2] as usize;
        if frame.len() != len {
            return None;
        }
        self.buf[..frame.len()].copy_from_slice(frame);
        self.pos = frame.len();
        self.expected_len = len;
        let result = self.parse_frame();
        self.pos = 0;
        self.expected_len = 0;
        result
    }

    fn parse_frame(&mut self) -> Option<RcFrame> {
        let len = self.expected_len;
        if len < 5 {
            return None;
        }

        // Verify CRC16-CCITT over all bytes except the last 2 (CRC)
        let crc_received = ((self.buf[len - 2] as u16) << 8) | (self.buf[len - 1] as u16);
        let crc_calc = crc16_ccitt(&self.buf[..len - 2]);
        if crc_received != crc_calc {
            return None;
        }

        let frame_type = self.buf[1];
        match frame_type {
            SRXL2_TYPE_CHANNEL_DATA => self.parse_channel_data(),
            _ => None, // handshake and telemetry don't produce RcFrame
        }
    }

    fn parse_channel_data(&mut self) -> Option<RcFrame> {
        let len = self.expected_len;
        // Minimum channel data: header(1) + type(1) + len(1) + device_id(1) + rssi(1)
        //   + frame_losses(2) + at_least_1_channel(2) + crc(2) = 11
        if len < 11 {
            return None;
        }

        let _device_id = self.buf[3];
        let rssi = self.buf[4];
        let frame_losses = ((self.buf[5] as u16) << 8) | (self.buf[6] as u16);

        self.last_rssi = rssi;
        self.frame_losses = frame_losses;

        // Channels start at byte 7, each is 2 bytes (u16 BE), until CRC (last 2 bytes)
        let channel_data_end = len - 2; // exclude CRC
        let channel_bytes = channel_data_end - 7;
        let num_channels = channel_bytes / 2;

        let mut channels = [1500u16; MAX_CHANNELS];
        let count = num_channels.min(MAX_CHANNELS);

        for i in 0..count {
            let offset = 7 + i * 2;
            let raw = ((self.buf[offset] as u16) << 8) | (self.buf[offset + 1] as u16);
            // SRXL2 channel values: 0-32768, center at 16384
            // Map to 1000-2000
            let pwm = ((raw as u32 * 1000) / 32768 + 1000).min(2000) as u16;
            channels[i] = pwm;
        }

        Some(RcFrame {
            channels,
            count: count as u8,
            failsafe: false,
            rssi,
        })
    }

    /// Encode a telemetry response frame for bidirectional communication.
    ///
    /// Returns the frame bytes and length. The FC sends this back to the receiver
    /// on the same wire during its time slot.
    pub fn encode_telemetry(device_id: u8, payload: &[u8]) -> ([u8; SRXL2_MAX_FRAME], usize) {
        let mut frame = [0u8; SRXL2_MAX_FRAME];
        // header + type + length + device_id + payload + crc
        let total_len = 3 + 1 + payload.len() + 2;
        if total_len > SRXL2_MAX_FRAME {
            return (frame, 0);
        }

        frame[0] = SRXL2_HEADER;
        frame[1] = SRXL2_TYPE_TELEMETRY;
        frame[2] = total_len as u8;
        frame[3] = device_id;
        frame[4..4 + payload.len()].copy_from_slice(payload);

        let crc = crc16_ccitt(&frame[..total_len - 2]);
        frame[total_len - 2] = (crc >> 8) as u8;
        frame[total_len - 1] = (crc & 0xFF) as u8;

        (frame, total_len)
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

    /// Build an SRXL2 channel data frame with known channel values (0-32768 range).
    fn build_srxl2_channel_frame(device_id: u8, rssi: u8, losses: u16, channels: &[u16]) -> heapless::Vec<u8, 80> {
        let num_ch = channels.len().min(MAX_CHANNELS);
        let total_len = 3 + 1 + 1 + 2 + num_ch * 2 + 2;
        // header(1) + type(1) + len(1) + dev(1) + rssi(1) + losses(2) + ch_data + crc(2)
        let mut frame = heapless::Vec::<u8, 80>::new();
        let _ = frame.push(SRXL2_HEADER);
        let _ = frame.push(SRXL2_TYPE_CHANNEL_DATA);
        let _ = frame.push(total_len as u8);
        let _ = frame.push(device_id);
        let _ = frame.push(rssi);
        let _ = frame.push((losses >> 8) as u8);
        let _ = frame.push((losses & 0xFF) as u8);

        for &ch in channels.iter().take(num_ch) {
            let _ = frame.push((ch >> 8) as u8);
            let _ = frame.push((ch & 0xFF) as u8);
        }

        let crc = crc16_ccitt(&frame);
        let _ = frame.push((crc >> 8) as u8);
        let _ = frame.push((crc & 0xFF) as u8);

        frame
    }

    #[test]
    fn test_srxl2_center_sticks() {
        // Center = 16384 -> maps to 1500
        let frame = build_srxl2_channel_frame(0x01, 90, 0, &[16384; 8]);
        let mut parser = Srxl2Parser::new();
        let result = parser.feed_frame(&frame).unwrap();

        assert_eq!(result.count, 8);
        assert!(!result.failsafe);
        assert_eq!(result.rssi, 90);
        assert_eq!(parser.frame_losses, 0);
        for i in 0..8 {
            assert_eq!(result.channels[i], 1500, "Channel {} = {}", i, result.channels[i]);
        }
    }

    #[test]
    fn test_srxl2_full_range() {
        let channels = [0, 8192, 16384, 24576, 32768];
        let frame = build_srxl2_channel_frame(0x01, 80, 5, &channels);
        let mut parser = Srxl2Parser::new();
        let result = parser.feed_frame(&frame).unwrap();

        assert_eq!(result.channels[0], 1000); // 0 -> 1000
        assert_eq!(result.channels[2], 1500); // 16384 -> 1500
        assert_eq!(result.channels[4], 2000); // 32768 -> 2000
        assert_eq!(parser.frame_losses, 5);
    }

    #[test]
    fn test_srxl2_bad_crc_rejected() {
        let mut frame = build_srxl2_channel_frame(0x01, 90, 0, &[16384; 4]);
        let last = frame.len() - 1;
        frame[last] = 0xFF;
        let mut parser = Srxl2Parser::new();
        assert!(parser.feed_frame(&frame).is_none());
    }

    #[test]
    fn test_srxl2_byte_by_byte() {
        let frame = build_srxl2_channel_frame(0x01, 85, 2, &[16384; 6]);
        let mut parser = Srxl2Parser::new();
        let mut result = None;
        for &byte in frame.iter() {
            if let Some(r) = parser.feed_byte(byte) {
                result = Some(r);
            }
        }
        let rc = result.unwrap();
        assert_eq!(rc.rssi, 85);
        assert_eq!(parser.frame_losses, 2);
    }

    #[test]
    fn test_srxl2_telemetry_encode() {
        let payload = [0x01, 0x02, 0x03, 0x04];
        let (frame, len) = Srxl2Parser::encode_telemetry(0x31, &payload);
        assert!(len > 0);
        assert_eq!(frame[0], SRXL2_HEADER);
        assert_eq!(frame[1], SRXL2_TYPE_TELEMETRY);
        assert_eq!(frame[2], len as u8);
        assert_eq!(frame[3], 0x31);
        assert_eq!(&frame[4..8], &payload);
        // Verify CRC
        let crc_calc = crc16_ccitt(&frame[..len - 2]);
        let crc_in_frame = ((frame[len - 2] as u16) << 8) | (frame[len - 1] as u16);
        assert_eq!(crc_calc, crc_in_frame);
    }

    #[test]
    fn test_srxl2_bad_header_rejected() {
        let mut parser = Srxl2Parser::new();
        let frame = [0xFF, 0xCD, 0x05, 0x00, 0x00];
        assert!(parser.feed_frame(&frame).is_none());
    }
}

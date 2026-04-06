//! uBlox GPS driver — UBX binary protocol parser.
//!
//! Implements a byte-at-a-time state machine for the UBX protocol.
//! Primary message: NAV-PVT (0x01, 0x07) — 92-byte position/velocity/time.
//!
//! UBX frame format:
//!   [0xB5, 0x62] class(u8) id(u8) length(u16 LE) payload[length] ckA(u8) ckB(u8)
//! Checksum: Fletcher-16 over class + id + length(2) + payload.

use meridian_math::geodetic::LatLonAlt;
use meridian_math::{Vec3, NED};
use meridian_types::messages::{GnssFixType, GnssPosition};
use meridian_types::time::Instant;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const SYNC1: u8 = 0xB5;
const SYNC2: u8 = 0x62;

const NAV_CLASS: u8 = 0x01;
const NAV_PVT_ID: u8 = 0x07;
const NAV_PVT_LEN: u16 = 92;

/// Maximum payload we accept (NAV-PVT is 92 bytes; cap at 256 for safety).
const MAX_PAYLOAD: usize = 256;

// ---------------------------------------------------------------------------
// Parsed UBX message
// ---------------------------------------------------------------------------

/// A successfully parsed UBX message.
#[derive(Debug, Clone)]
pub struct UbxMessage {
    pub class: u8,
    pub id: u8,
    pub payload: [u8; MAX_PAYLOAD],
    pub length: u16,
}

// ---------------------------------------------------------------------------
// UBX parser state machine
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ParserState {
    WaitSync1,
    WaitSync2,
    Header,
    Payload,
    CkA,
    CkB,
}

/// Byte-at-a-time UBX parser.
pub struct UbxParser {
    state: ParserState,
    class: u8,
    id: u8,
    length: u16,
    payload: [u8; MAX_PAYLOAD],
    payload_idx: u16,
    header_buf: [u8; 4], // class, id, len_lo, len_hi
    header_idx: u8,
    ck_a: u8,
    ck_b: u8,
    rx_ck_a: u8,
}

impl UbxParser {
    pub fn new() -> Self {
        Self {
            state: ParserState::WaitSync1,
            class: 0,
            id: 0,
            length: 0,
            payload: [0u8; MAX_PAYLOAD],
            payload_idx: 0,
            header_buf: [0u8; 4],
            header_idx: 0,
            ck_a: 0,
            ck_b: 0,
            rx_ck_a: 0,
        }
    }

    /// Reset the parser to idle state.
    fn reset(&mut self) {
        self.state = ParserState::WaitSync1;
        self.header_idx = 0;
        self.payload_idx = 0;
        self.ck_a = 0;
        self.ck_b = 0;
    }

    /// Feed one byte. Returns Some(UbxMessage) when a complete, valid frame is decoded.
    pub fn feed(&mut self, byte: u8) -> Option<UbxMessage> {
        match self.state {
            ParserState::WaitSync1 => {
                if byte == SYNC1 {
                    self.state = ParserState::WaitSync2;
                }
                None
            }
            ParserState::WaitSync2 => {
                if byte == SYNC2 {
                    self.state = ParserState::Header;
                    self.header_idx = 0;
                    self.ck_a = 0;
                    self.ck_b = 0;
                } else {
                    self.reset();
                }
                None
            }
            ParserState::Header => {
                self.header_buf[self.header_idx as usize] = byte;
                self.ck_a = self.ck_a.wrapping_add(byte);
                self.ck_b = self.ck_b.wrapping_add(self.ck_a);
                self.header_idx += 1;
                if self.header_idx == 4 {
                    self.class = self.header_buf[0];
                    self.id = self.header_buf[1];
                    self.length =
                        u16::from_le_bytes([self.header_buf[2], self.header_buf[3]]);
                    if self.length as usize > MAX_PAYLOAD {
                        self.reset();
                        return None;
                    }
                    self.payload_idx = 0;
                    if self.length == 0 {
                        self.state = ParserState::CkA;
                    } else {
                        self.state = ParserState::Payload;
                    }
                }
                None
            }
            ParserState::Payload => {
                self.payload[self.payload_idx as usize] = byte;
                self.ck_a = self.ck_a.wrapping_add(byte);
                self.ck_b = self.ck_b.wrapping_add(self.ck_a);
                self.payload_idx += 1;
                if self.payload_idx == self.length {
                    self.state = ParserState::CkA;
                }
                None
            }
            ParserState::CkA => {
                self.rx_ck_a = byte;
                self.state = ParserState::CkB;
                None
            }
            ParserState::CkB => {
                let valid = self.rx_ck_a == self.ck_a && byte == self.ck_b;
                let result = if valid {
                    Some(UbxMessage {
                        class: self.class,
                        id: self.id,
                        payload: self.payload,
                        length: self.length,
                    })
                } else {
                    None
                };
                self.reset();
                result
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Checksum utility
// ---------------------------------------------------------------------------

/// Compute Fletcher-16 checksum over class + id + length(LE) + payload.
pub fn ubx_checksum(class: u8, id: u8, payload: &[u8]) -> (u8, u8) {
    let len = payload.len() as u16;
    let len_bytes = len.to_le_bytes();
    let mut ck_a: u8 = 0;
    let mut ck_b: u8 = 0;
    for &b in &[class, id, len_bytes[0], len_bytes[1]] {
        ck_a = ck_a.wrapping_add(b);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    for &b in payload {
        ck_a = ck_a.wrapping_add(b);
        ck_b = ck_b.wrapping_add(ck_a);
    }
    (ck_a, ck_b)
}

/// Build a complete UBX frame (sync + header + payload + checksum).
pub fn ubx_frame(class: u8, id: u8, payload: &[u8]) -> heapless::Vec<u8, 264> {
    let (ck_a, ck_b) = ubx_checksum(class, id, payload);
    let len = payload.len() as u16;
    let len_bytes = len.to_le_bytes();
    let mut buf = heapless::Vec::new();
    let _ = buf.push(SYNC1);
    let _ = buf.push(SYNC2);
    let _ = buf.push(class);
    let _ = buf.push(id);
    let _ = buf.push(len_bytes[0]);
    let _ = buf.push(len_bytes[1]);
    for &b in payload {
        let _ = buf.push(b);
    }
    let _ = buf.push(ck_a);
    let _ = buf.push(ck_b);
    buf
}

// ---------------------------------------------------------------------------
// NAV-PVT extraction
// ---------------------------------------------------------------------------

/// Extract a GnssPosition from a NAV-PVT payload.
fn parse_nav_pvt(payload: &[u8], timestamp: Instant) -> Option<GnssPosition> {
    if payload.len() < NAV_PVT_LEN as usize {
        return None;
    }

    let fix_raw = payload[20];
    let num_sv = payload[23];

    let lon_1e7 = i32::from_le_bytes([payload[24], payload[25], payload[26], payload[27]]);
    let lat_1e7 = i32::from_le_bytes([payload[28], payload[29], payload[30], payload[31]]);
    let h_msl_mm = i32::from_le_bytes([payload[36], payload[37], payload[38], payload[39]]);
    let h_acc_mm = u32::from_le_bytes([payload[40], payload[41], payload[42], payload[43]]);
    let v_acc_mm = u32::from_le_bytes([payload[44], payload[45], payload[46], payload[47]]);
    let vel_n_mm = i32::from_le_bytes([payload[48], payload[49], payload[50], payload[51]]);
    let vel_e_mm = i32::from_le_bytes([payload[52], payload[53], payload[54], payload[55]]);
    let vel_d_mm = i32::from_le_bytes([payload[56], payload[57], payload[58], payload[59]]);
    let s_acc_mm = u32::from_le_bytes([payload[68], payload[69], payload[70], payload[71]]);

    let fix_type = match fix_raw {
        0 => GnssFixType::NoFix,
        1 => GnssFixType::NoFix, // dead-reckoning only
        2 => GnssFixType::Fix2D,
        3 => GnssFixType::Fix3D,
        4 => GnssFixType::Fix3D, // GNSS + DR combined
        5 => GnssFixType::NoFix, // time-only
        _ => GnssFixType::NoFix,
    };

    let lat_deg = lat_1e7 as f64 * 1e-7;
    let lon_deg = lon_1e7 as f64 * 1e-7;
    let alt_m = h_msl_mm as f64 * 0.001;

    Some(GnssPosition {
        timestamp,
        fix_type,
        position: LatLonAlt::from_degrees(lat_deg, lon_deg, alt_m),
        velocity_ned: Vec3::<NED>::new(
            vel_n_mm as f32 * 0.001,
            vel_e_mm as f32 * 0.001,
            vel_d_mm as f32 * 0.001,
        ),
        horizontal_accuracy: h_acc_mm as f32 * 0.001,
        vertical_accuracy: v_acc_mm as f32 * 0.001,
        speed_accuracy: s_acc_mm as f32 * 0.001,
        num_sats: num_sv,
    })
}

// ---------------------------------------------------------------------------
// High-level driver
// ---------------------------------------------------------------------------

/// uBlox GPS driver. Feed bytes from UART; query latest fix.
pub struct UbloxGps {
    parser: UbxParser,
    latest_fix: Option<GnssPosition>,
}

impl UbloxGps {
    pub fn new() -> Self {
        Self {
            parser: UbxParser::new(),
            latest_fix: None,
        }
    }

    /// Feed one byte from the UART. Returns `Some(GnssPosition)` when a
    /// NAV-PVT message is successfully decoded.
    pub fn feed_byte(&mut self, byte: u8, timestamp: Instant) -> Option<GnssPosition> {
        if let Some(msg) = self.parser.feed(byte) {
            if msg.class == NAV_CLASS && msg.id == NAV_PVT_ID {
                if let Some(pos) = parse_nav_pvt(&msg.payload[..msg.length as usize], timestamp) {
                    self.latest_fix = Some(pos);
                    return Some(pos);
                }
            }
        }
        None
    }

    /// Get the most recent fix, if any.
    pub fn get_fix(&self) -> Option<&GnssPosition> {
        self.latest_fix.as_ref()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a valid NAV-PVT frame with known fields for testing.
    fn make_nav_pvt_frame() -> heapless::Vec<u8, 264> {
        let mut payload = [0u8; 92];

        // fixType = 3 (3D fix) at offset 20
        payload[20] = 3;
        // numSV = 12 at offset 23
        payload[23] = 12;

        // lon = 11.5000000 deg => 115_000_000 * 1e-7
        let lon: i32 = 115_000_000;
        payload[24..28].copy_from_slice(&lon.to_le_bytes());

        // lat = 47.3000000 deg => 473_000_000 * 1e-7
        let lat: i32 = 473_000_000;
        payload[28..32].copy_from_slice(&lat.to_le_bytes());

        // hMSL = 540000 mm = 540 m
        let h_msl: i32 = 540_000;
        payload[36..40].copy_from_slice(&h_msl.to_le_bytes());

        // hAcc = 1500 mm = 1.5 m
        let h_acc: u32 = 1500;
        payload[40..44].copy_from_slice(&h_acc.to_le_bytes());

        // vAcc = 2500 mm = 2.5 m
        let v_acc: u32 = 2500;
        payload[44..48].copy_from_slice(&v_acc.to_le_bytes());

        // velN = 500 mm/s = 0.5 m/s
        let vel_n: i32 = 500;
        payload[48..52].copy_from_slice(&vel_n.to_le_bytes());

        // velE = -300 mm/s = -0.3 m/s
        let vel_e: i32 = -300;
        payload[52..56].copy_from_slice(&vel_e.to_le_bytes());

        // velD = 100 mm/s = 0.1 m/s
        let vel_d: i32 = 100;
        payload[56..60].copy_from_slice(&vel_d.to_le_bytes());

        // sAcc = 200 mm/s = 0.2 m/s
        let s_acc: u32 = 200;
        payload[68..72].copy_from_slice(&s_acc.to_le_bytes());

        ubx_frame(NAV_CLASS, NAV_PVT_ID, &payload)
    }

    #[test]
    fn test_frame_parsing() {
        let frame = make_nav_pvt_frame();
        let mut parser = UbxParser::new();
        let mut result = None;
        for &b in frame.as_slice() {
            if let Some(msg) = parser.feed(b) {
                result = Some(msg);
            }
        }
        let msg = result.expect("should parse a complete frame");
        assert_eq!(msg.class, NAV_CLASS);
        assert_eq!(msg.id, NAV_PVT_ID);
        assert_eq!(msg.length, 92);
    }

    #[test]
    fn test_checksum_validation() {
        let frame = make_nav_pvt_frame();
        let (ck_a, ck_b) = ubx_checksum(NAV_CLASS, NAV_PVT_ID, &frame[6..98]);
        assert_eq!(ck_a, frame[98]);
        assert_eq!(ck_b, frame[99]);
    }

    #[test]
    fn test_checksum_rejection() {
        let mut frame = make_nav_pvt_frame();
        // Corrupt the checksum
        let last = frame.len() - 1;
        frame[last] ^= 0xFF;

        let mut parser = UbxParser::new();
        let mut result = None;
        for &b in frame.as_slice() {
            if let Some(msg) = parser.feed(b) {
                result = Some(msg);
            }
        }
        assert!(result.is_none(), "corrupted checksum should be rejected");
    }

    #[test]
    fn test_nav_pvt_extraction() {
        let frame = make_nav_pvt_frame();
        let mut gps = UbloxGps::new();
        let ts = Instant::from_micros(1_000_000);

        let mut fix = None;
        for &b in frame.as_slice() {
            if let Some(f) = gps.feed_byte(b, ts) {
                fix = Some(f);
            }
        }
        let fix = fix.expect("should produce a GnssPosition");

        assert_eq!(fix.fix_type, GnssFixType::Fix3D);
        assert_eq!(fix.num_sats, 12);

        // Latitude: 47.3 deg (1e-7 tolerance on radians)
        let expected_lat_rad = 47.3_f64.to_radians();
        assert!((fix.position.lat - expected_lat_rad).abs() < 1e-7);

        // Longitude: 11.5 deg
        let expected_lon_rad = 11.5_f64.to_radians();
        assert!((fix.position.lon - expected_lon_rad).abs() < 1e-7);

        // Altitude: 540.0 m
        assert!((fix.position.alt - 540.0).abs() < 0.01);

        // Velocity NED
        assert!((fix.velocity_ned.x - 0.5).abs() < 0.001);
        assert!((fix.velocity_ned.y - (-0.3)).abs() < 0.001);
        assert!((fix.velocity_ned.z - 0.1).abs() < 0.001);

        // Accuracies
        assert!((fix.horizontal_accuracy - 1.5).abs() < 0.001);
        assert!((fix.vertical_accuracy - 2.5).abs() < 0.001);
        assert!((fix.speed_accuracy - 0.2).abs() < 0.001);

        // get_fix should return the same
        assert!(gps.get_fix().is_some());
    }

    #[test]
    fn test_parser_recovery_after_garbage() {
        let mut parser = UbxParser::new();
        // Feed garbage bytes
        for b in &[0x00, 0xFF, 0xB5, 0x00, 0xAA] {
            assert!(parser.feed(*b).is_none());
        }
        // Now feed a valid frame
        let frame = make_nav_pvt_frame();
        let mut result = None;
        for &b in frame.as_slice() {
            if let Some(msg) = parser.feed(b) {
                result = Some(msg);
            }
        }
        assert!(result.is_some(), "parser should recover from garbage");
    }

    #[test]
    fn test_ubx_checksum_function() {
        // Verify standalone checksum matches what the frame builder produces
        let payload = [0x01, 0x02, 0x03];
        let (a, b) = ubx_checksum(0x06, 0x01, &payload);
        // Manual: class=0x06, id=0x01, len_lo=0x03, len_hi=0x00
        // ck_a: 06 -> 07 -> 0A -> 0A -> 0B -> 0D -> 10
        // ck_b: 06 -> 0D -> 17 -> 21 -> 2C -> 39 -> 49
        let mut ck_a: u8 = 0;
        let mut ck_b: u8 = 0;
        for &byte in &[0x06u8, 0x01, 0x03, 0x00, 0x01, 0x02, 0x03] {
            ck_a = ck_a.wrapping_add(byte);
            ck_b = ck_b.wrapping_add(ck_a);
        }
        assert_eq!(a, ck_a);
        assert_eq!(b, ck_b);
    }
}

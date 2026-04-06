//! MAVLink v2 wire format: framing, CRC, parsing.
//!
//! Implements just enough of MAVLink v2 to communicate with QGC/MP/ATAK.
//! No external crate dependency — pure Rust, no_std.

/// MAVLink v2 start byte.
pub const MAVLINK_STX: u8 = 0xFD;

/// MAVLink v2 header size (10 bytes).
pub const HEADER_SIZE: usize = 10;

/// Maximum payload size.
pub const MAX_PAYLOAD: usize = 255;

/// Full frame max: header + payload + CRC.
pub const MAX_FRAME_SIZE: usize = HEADER_SIZE + MAX_PAYLOAD + 2;

// ─── Common MAVLink message IDs ───
pub const MSG_HEARTBEAT: u32 = 0;
pub const MSG_SYS_STATUS: u32 = 1;
pub const MSG_GPS_RAW_INT: u32 = 24;
pub const MSG_ATTITUDE: u32 = 30;
pub const MSG_GLOBAL_POSITION_INT: u32 = 33;
pub const MSG_RC_CHANNELS: u32 = 65;
pub const MSG_VFR_HUD: u32 = 74;
pub const MSG_COMMAND_LONG: u32 = 76;
pub const MSG_COMMAND_ACK: u32 = 77;
pub const MSG_BATTERY_STATUS: u32 = 147;
pub const MSG_STATUSTEXT: u32 = 253;
pub const MSG_PARAM_VALUE: u32 = 22;
pub const MSG_PARAM_SET: u32 = 23;
pub const MSG_PARAM_REQUEST_LIST: u32 = 21;
pub const MSG_PARAM_REQUEST_READ: u32 = 20;
pub const MSG_REQUEST_DATA_STREAM: u32 = 66;
pub const MSG_MISSION_REQUEST_LIST: u32 = 43;
pub const MSG_MISSION_COUNT: u32 = 44;
pub const MSG_MISSION_ITEM_INT: u32 = 73;
pub const MSG_MISSION_REQUEST_INT: u32 = 51;
pub const MSG_MISSION_ACK: u32 = 47;
pub const MSG_MISSION_CLEAR_ALL: u32 = 45;
pub const MSG_AUTOPILOT_VERSION: u32 = 148;
pub const MSG_HOME_POSITION: u32 = 242;
pub const MSG_COMMAND_INT: u32 = 75;
pub const MSG_TIMESYNC: u32 = 111;
pub const MSG_MISSION_CURRENT: u32 = 42;

// MAVLink CRC extra bytes (per message ID, for integrity checking).
// Only the messages we actually send/receive.
pub fn crc_extra(msg_id: u32) -> u8 {
    match msg_id {
        MSG_HEARTBEAT => 50,
        MSG_SYS_STATUS => 124,
        MSG_PARAM_VALUE => 220,
        MSG_PARAM_SET => 168,
        MSG_GPS_RAW_INT => 24,
        MSG_ATTITUDE => 39,
        MSG_GLOBAL_POSITION_INT => 104,
        MSG_RC_CHANNELS => 118,
        MSG_VFR_HUD => 20,
        MSG_COMMAND_LONG => 152,
        MSG_COMMAND_ACK => 143,
        MSG_BATTERY_STATUS => 154,
        MSG_STATUSTEXT => 83,
        MSG_REQUEST_DATA_STREAM => 148,
        MSG_MISSION_REQUEST_LIST => 132,
        MSG_MISSION_COUNT => 221,
        MSG_MISSION_ITEM_INT => 38,
        MSG_MISSION_REQUEST_INT => 196,
        MSG_MISSION_ACK => 153,
        MSG_MISSION_CLEAR_ALL => 232,
        MSG_AUTOPILOT_VERSION => 178,
        MSG_HOME_POSITION => 104,
        MSG_COMMAND_INT => 158,
        MSG_PARAM_REQUEST_LIST => 159,
        MSG_PARAM_REQUEST_READ => 214,
        MSG_TIMESYNC => 34,
        MSG_MISSION_CURRENT => 28,
        _ => 0,
    }
}

/// MAVLink v2 frame header.
#[derive(Debug, Clone, Copy)]
pub struct MavHeader {
    pub payload_len: u8,
    pub incompat_flags: u8,
    pub compat_flags: u8,
    pub seq: u8,
    pub system_id: u8,
    pub component_id: u8,
    pub msg_id: u32, // 24-bit
}

/// Encode a MAVLink v2 frame. Returns total frame length.
pub fn encode_frame(
    header: &MavHeader,
    payload: &[u8],
    frame_buf: &mut [u8],
) -> usize {
    if payload.len() > MAX_PAYLOAD || frame_buf.len() < MAX_FRAME_SIZE {
        return 0;
    }

    let len = payload.len() as u8;
    frame_buf[0] = MAVLINK_STX;
    frame_buf[1] = len;
    frame_buf[2] = header.incompat_flags;
    frame_buf[3] = header.compat_flags;
    frame_buf[4] = header.seq;
    frame_buf[5] = header.system_id;
    frame_buf[6] = header.component_id;
    frame_buf[7] = (header.msg_id & 0xFF) as u8;
    frame_buf[8] = ((header.msg_id >> 8) & 0xFF) as u8;
    frame_buf[9] = ((header.msg_id >> 16) & 0xFF) as u8;

    frame_buf[HEADER_SIZE..HEADER_SIZE + payload.len()].copy_from_slice(payload);

    // CRC: X.25 over bytes [1..header+payload], then append crc_extra
    let crc_start = 1;
    let crc_end = HEADER_SIZE + payload.len();
    let mut crc = crc_x25(&frame_buf[crc_start..crc_end]);
    crc_accumulate(&mut crc, crc_extra(header.msg_id));

    let crc_offset = HEADER_SIZE + payload.len();
    frame_buf[crc_offset] = (crc & 0xFF) as u8;
    frame_buf[crc_offset + 1] = ((crc >> 8) & 0xFF) as u8;

    crc_offset + 2
}

/// Parse a MAVLink v2 frame. Returns (header, payload_slice) or None.
pub fn parse_frame(data: &[u8]) -> Option<(MavHeader, &[u8])> {
    if data.len() < HEADER_SIZE + 2 { return None; }
    if data[0] != MAVLINK_STX { return None; }

    let payload_len = data[1] as usize;
    let total_len = HEADER_SIZE + payload_len + 2;
    if data.len() < total_len { return None; }

    let header = MavHeader {
        payload_len: data[1],
        incompat_flags: data[2],
        compat_flags: data[3],
        seq: data[4],
        system_id: data[5],
        component_id: data[6],
        msg_id: data[7] as u32 | ((data[8] as u32) << 8) | ((data[9] as u32) << 16),
    };

    // Verify CRC
    let mut crc = crc_x25(&data[1..HEADER_SIZE + payload_len]);
    crc_accumulate(&mut crc, crc_extra(header.msg_id));
    let expected = (data[HEADER_SIZE + payload_len] as u16)
        | ((data[HEADER_SIZE + payload_len + 1] as u16) << 8);
    if crc != expected { return None; }

    Some((header, &data[HEADER_SIZE..HEADER_SIZE + payload_len]))
}

/// X.25 CRC (used by MAVLink).
fn crc_x25(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc_accumulate(&mut crc, byte);
    }
    crc
}

fn crc_accumulate(crc: &mut u16, byte: u8) {
    let mut tmp = byte as u16 ^ (*crc & 0xFF);
    tmp ^= (tmp << 4) & 0xFF;
    *crc = (*crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

/// Stream parser for MAVLink v2 frames.
pub struct MavParser {
    buf: [u8; MAX_FRAME_SIZE],
    pos: usize,
    state: MavParseState,
    expected_len: usize,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum MavParseState {
    WaitingStx,
    ReadingHeader,
    ReadingPayload,
    ReadingCrc,
}

impl MavParser {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; MAX_FRAME_SIZE],
            pos: 0,
            state: MavParseState::WaitingStx,
            expected_len: 0,
        }
    }

    /// Feed one byte. Returns Some((header, payload)) when a frame is complete.
    pub fn feed(&mut self, byte: u8) -> Option<(MavHeader, &[u8])> {
        match self.state {
            MavParseState::WaitingStx => {
                if byte == MAVLINK_STX {
                    self.buf[0] = byte;
                    self.pos = 1;
                    self.state = MavParseState::ReadingHeader;
                }
                None
            }
            MavParseState::ReadingHeader => {
                self.buf[self.pos] = byte;
                self.pos += 1;
                if self.pos >= HEADER_SIZE {
                    let payload_len = self.buf[1] as usize;
                    self.expected_len = HEADER_SIZE + payload_len + 2;
                    if self.expected_len > MAX_FRAME_SIZE {
                        self.state = MavParseState::WaitingStx;
                        return None;
                    }
                    self.state = MavParseState::ReadingPayload;
                }
                None
            }
            MavParseState::ReadingPayload | MavParseState::ReadingCrc => {
                self.buf[self.pos] = byte;
                self.pos += 1;
                if self.pos >= self.expected_len {
                    self.state = MavParseState::WaitingStx;
                    return parse_frame(&self.buf[..self.pos]);
                }
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_heartbeat_encode_parse() {
        // MAVLink HEARTBEAT payload: 9 bytes
        // type(4B) + autopilot(1B) + base_mode(1B) + custom_mode(4B) ... wait
        // Actually HEARTBEAT is: custom_mode(u32) + type(u8) + autopilot(u8) + base_mode(u8) + system_status(u8) + mavlink_version(u8) = 9 bytes
        let payload = [
            0x00, 0x00, 0x00, 0x00, // custom_mode = 0
            0x02,                   // type = MAV_TYPE_QUADROTOR
            0x03,                   // autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA
            0x81,                   // base_mode = ARMED | CUSTOM
            0x04,                   // system_status = ACTIVE
            0x03,                   // mavlink_version = 3
        ];

        let header = MavHeader {
            payload_len: payload.len() as u8,
            incompat_flags: 0,
            compat_flags: 0,
            seq: 42,
            system_id: 1,
            component_id: 1,
            msg_id: MSG_HEARTBEAT,
        };

        let mut frame = [0u8; MAX_FRAME_SIZE];
        let len = encode_frame(&header, &payload, &mut frame);
        assert!(len > 0);
        assert_eq!(frame[0], MAVLINK_STX);

        // Parse it back
        let (parsed_hdr, parsed_payload) = parse_frame(&frame[..len]).unwrap();
        assert_eq!(parsed_hdr.msg_id, MSG_HEARTBEAT);
        assert_eq!(parsed_hdr.seq, 42);
        assert_eq!(parsed_hdr.system_id, 1);
        assert_eq!(parsed_payload, &payload);
    }

    #[test]
    fn test_stream_parser() {
        let payload = [0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x81, 0x04, 0x03];
        let header = MavHeader {
            payload_len: 9, incompat_flags: 0, compat_flags: 0,
            seq: 1, system_id: 1, component_id: 1, msg_id: MSG_HEARTBEAT,
        };
        let mut frame = [0u8; MAX_FRAME_SIZE];
        let len = encode_frame(&header, &payload, &mut frame);

        let mut parser = MavParser::new();
        let mut found = false;
        for i in 0..len {
            if let Some((hdr, pay)) = parser.feed(frame[i]) {
                assert_eq!(hdr.msg_id, MSG_HEARTBEAT);
                assert_eq!(pay.len(), 9);
                found = true;
            }
        }
        assert!(found);
    }

    #[test]
    fn test_crc_rejects_corruption() {
        let payload = [0x02, 0x03, 0x81, 0x04, 0x03, 0x00, 0x00, 0x00, 0x00];
        let header = MavHeader {
            payload_len: 9, incompat_flags: 0, compat_flags: 0,
            seq: 0, system_id: 1, component_id: 1, msg_id: MSG_HEARTBEAT,
        };
        let mut frame = [0u8; MAX_FRAME_SIZE];
        let len = encode_frame(&header, &payload, &mut frame);

        // Corrupt one byte
        frame[12] ^= 0xFF;
        assert!(parse_frame(&frame[..len]).is_none());
    }
}

//! Wire framing: COBS encoding over byte streams.
//!
//! COBS (Consistent Overhead Byte Stuffing) provides:
//! - Unambiguous frame boundaries (0x00 delimiter)
//! - At most 1 byte overhead per 254 data bytes
//! - No escape sequences — fixed overhead, streaming-friendly
//! - Perfect for UART where 0x00 is the frame delimiter

/// Maximum payload before COBS encoding (msg_id + seq + body).
pub const MAX_PAYLOAD_SIZE: usize = 255;
/// Maximum frame size after COBS encoding + delimiters.
pub const MAX_FRAME_SIZE: usize = MAX_PAYLOAD_SIZE + MAX_PAYLOAD_SIZE / 254 + 3;

/// COBS-encode `data` into `out`. Returns number of bytes written.
/// `out` must be at least data.len() + data.len()/254 + 1 bytes.
pub fn cobs_encode(data: &[u8], out: &mut [u8]) -> usize {
    let mut write_idx = 1; // leave room for first code byte
    let mut code_idx = 0;
    let mut code: u8 = 1;

    for &byte in data {
        if byte == 0 {
            out[code_idx] = code;
            code_idx = write_idx;
            write_idx += 1;
            code = 1;
        } else {
            out[write_idx] = byte;
            write_idx += 1;
            code += 1;
            if code == 255 {
                out[code_idx] = code;
                code_idx = write_idx;
                write_idx += 1;
                code = 1;
            }
        }
    }
    out[code_idx] = code;
    write_idx
}

/// COBS-decode `data` into `out`. Returns number of bytes written, or None on error.
pub fn cobs_decode(data: &[u8], out: &mut [u8]) -> Option<usize> {
    if data.is_empty() { return Some(0); }
    let mut read_idx = 0;
    let mut write_idx = 0;

    while read_idx < data.len() {
        let code = data[read_idx] as usize;
        if code == 0 { return None; }
        read_idx += 1;

        let data_bytes = code - 1;
        for _ in 0..data_bytes {
            if read_idx >= data.len() { return None; }
            if write_idx >= out.len() { return None; }
            out[write_idx] = data[read_idx];
            write_idx += 1;
            read_idx += 1;
        }

        // If code < 255, insert a zero (unless we're at the end of data)
        if code < 255 && read_idx < data.len() {
            if write_idx >= out.len() { return None; }
            out[write_idx] = 0;
            write_idx += 1;
        }
    }

    Some(write_idx)
}

/// Encode a complete MNP frame: [0x00] [COBS(payload)] [0x00]
/// `payload` = [msg_id, seq_lo, seq_hi, ...body...]
/// Returns number of bytes written to `frame_buf`.
pub fn encode_frame(msg_id: u8, seq: u16, body: &[u8], frame_buf: &mut [u8]) -> usize {
    // Build payload: msg_id + seq_le + body
    let mut payload = [0u8; MAX_PAYLOAD_SIZE];
    let payload_len = 3 + body.len();
    if payload_len > MAX_PAYLOAD_SIZE || frame_buf.len() < MAX_FRAME_SIZE {
        return 0;
    }
    payload[0] = msg_id;
    payload[1] = seq as u8;
    payload[2] = (seq >> 8) as u8;
    payload[3..3 + body.len()].copy_from_slice(body);

    // Frame: [0x00] [COBS] [0x00]
    frame_buf[0] = 0x00;
    let cobs_len = cobs_encode(&payload[..payload_len], &mut frame_buf[1..]);
    frame_buf[1 + cobs_len] = 0x00;
    2 + cobs_len
}

/// Decode a frame. Input should be the bytes BETWEEN the 0x00 delimiters.
/// Returns (msg_id, seq, body_slice_len) and writes body into `body_buf`.
pub fn decode_frame(cobs_data: &[u8], body_buf: &mut [u8]) -> Option<(u8, u16, usize)> {
    let mut payload = [0u8; MAX_PAYLOAD_SIZE];
    let payload_len = cobs_decode(cobs_data, &mut payload)?;
    if payload_len < 3 { return None; }

    let msg_id = payload[0];
    let seq = payload[1] as u16 | ((payload[2] as u16) << 8);
    let body_len = payload_len - 3;
    if body_len > body_buf.len() { return None; }
    body_buf[..body_len].copy_from_slice(&payload[3..3 + body_len]);
    Some((msg_id, seq, body_len))
}

/// Stream parser: accumulates bytes and yields complete frames.
pub struct FrameParser {
    buf: [u8; MAX_FRAME_SIZE],
    pos: usize,
    in_frame: bool,
}

impl FrameParser {
    pub const fn new() -> Self {
        Self { buf: [0u8; MAX_FRAME_SIZE], pos: 0, in_frame: false }
    }

    /// Feed bytes one at a time. Returns Some(slice) when a complete frame is found.
    /// The returned slice is the COBS-encoded payload (between delimiters).
    pub fn feed(&mut self, byte: u8) -> Option<&[u8]> {
        if byte == 0x00 {
            if self.in_frame && self.pos > 0 {
                // End of frame — return accumulated data
                self.in_frame = false;
                let len = self.pos;
                self.pos = 0;
                return Some(&self.buf[..len]);
            }
            // Start of new frame
            self.in_frame = true;
            self.pos = 0;
            return None;
        }

        if self.in_frame {
            if self.pos < MAX_FRAME_SIZE {
                self.buf[self.pos] = byte;
                self.pos += 1;
            } else {
                // Frame too long — discard
                self.in_frame = false;
                self.pos = 0;
            }
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cobs_roundtrip() {
        let data = [0x01, 0x00, 0x03, 0x00, 0x05];
        let mut encoded = [0u8; 64];
        let enc_len = cobs_encode(&data, &mut encoded);

        let mut decoded = [0u8; 64];
        let dec_len = cobs_decode(&encoded[..enc_len], &mut decoded).unwrap();
        assert_eq!(&decoded[..dec_len], &data);
    }

    #[test]
    fn test_cobs_no_zeros() {
        let data = [0x01, 0x02, 0x03, 0x04];
        let mut encoded = [0u8; 64];
        let enc_len = cobs_encode(&data, &mut encoded);

        // No zeros in encoded data
        for i in 0..enc_len {
            assert_ne!(encoded[i], 0, "COBS output should have no zeros");
        }

        let mut decoded = [0u8; 64];
        let dec_len = cobs_decode(&encoded[..enc_len], &mut decoded).unwrap();
        assert_eq!(&decoded[..dec_len], &data);
    }

    #[test]
    fn test_frame_encode_decode() {
        let body = [0x10, 0x20, 0x30, 0x00, 0x50]; // includes a zero byte
        let mut frame = [0u8; MAX_FRAME_SIZE];
        let frame_len = encode_frame(0x01, 42, &body, &mut frame);
        assert!(frame_len > 0);
        assert_eq!(frame[0], 0x00); // start delimiter
        assert_eq!(frame[frame_len - 1], 0x00); // end delimiter

        // Decode the COBS payload (between delimiters)
        let mut body_out = [0u8; 255];
        let (msg_id, seq, body_len) = decode_frame(
            &frame[1..frame_len - 1], &mut body_out
        ).unwrap();
        assert_eq!(msg_id, 0x01);
        assert_eq!(seq, 42);
        assert_eq!(&body_out[..body_len], &body);
    }

    #[test]
    fn test_frame_parser_stream() {
        let body = [0xAA, 0xBB];
        let mut frame = [0u8; MAX_FRAME_SIZE];
        let frame_len = encode_frame(0x05, 100, &body, &mut frame);

        let mut parser = FrameParser::new();
        let mut found = false;
        for i in 0..frame_len {
            if let Some(cobs_data) = parser.feed(frame[i]) {
                let mut body_out = [0u8; 255];
                let (msg_id, seq, blen) = decode_frame(cobs_data, &mut body_out).unwrap();
                assert_eq!(msg_id, 0x05);
                assert_eq!(seq, 100);
                assert_eq!(&body_out[..blen], &body);
                found = true;
            }
        }
        assert!(found, "Parser should have yielded a frame");
    }

    #[test]
    fn test_bandwidth_budget() {
        // Verify attitude message fits 57600 baud budget
        // Attitude: 6 floats (roll,pitch,yaw,rollrate,pitchrate,yawrate) = 24 bytes
        // + 3 header (msg_id, seq) = 27 bytes payload
        // + COBS overhead (~1 byte) = 28 bytes
        // + 2 delimiters = 30 bytes on wire
        // At 10Hz: 300 bytes/s
        // 57600 baud = 5760 bytes/s → 5.2% of link. Plenty of room.
        let attitude_body = [0u8; 24]; // 6 floats
        let mut frame = [0u8; MAX_FRAME_SIZE];
        let frame_len = encode_frame(0x02, 0, &attitude_body, &mut frame);
        assert!(frame_len <= 32, "Attitude frame should be <=32 bytes, got {}", frame_len);

        // Budget check: 16 msgs/s * 32 bytes = 512 bytes/s = 8.9% of 5760
        let msgs_per_sec = 16;
        let bytes_per_sec = msgs_per_sec * frame_len;
        let link_capacity = 5760; // 57600 baud / 10 bits per byte
        let utilization = bytes_per_sec as f32 / link_capacity as f32;
        assert!(utilization < 0.20,
            "Link utilization {:.1}% should be <20%", utilization * 100.0);
    }
}

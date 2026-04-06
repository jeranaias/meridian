//! DroneCAN (UAVCAN v0) frame layer.
//!
//! Implements the 29-bit extended CAN ID encoding used by UAVCAN v0:
//!
//!   Bits 28..24  (5 bits) — priority (0 = highest)
//!   Bits 23..8   (16 bits) — data type ID (message or service)
//!   Bit  7       (1 bit) — service_not_message (0 = message, 1 = service)
//!   Bits 6..0    (7 bits) — source node ID
//!
//! Payload in each CAN frame is up to 7 bytes of data + 1 tail byte.
//! Multi-frame transfers use CRC16-CCITT over the full payload and tail
//! byte toggle/end-of-transfer/transfer_id fields for reassembly.

use meridian_hal::can::CanFrame;

// ---------------------------------------------------------------------------
// Priority levels (matching libcanard)
// ---------------------------------------------------------------------------

/// Transfer priority levels (lower value = higher priority).
pub const PRIORITY_HIGHEST: u8 = 0;
pub const PRIORITY_HIGH: u8 = 8;
pub const PRIORITY_MEDIUM: u8 = 16;
pub const PRIORITY_LOW: u8 = 24;
pub const PRIORITY_LOWEST: u8 = 31;

// ---------------------------------------------------------------------------
// CAN ID field encoding / decoding
// ---------------------------------------------------------------------------

/// Encode a 29-bit extended CAN ID for a DroneCAN message broadcast.
///
/// For messages (not services): service_not_message = 0.
#[inline]
pub fn encode_can_id(priority: u8, type_id: u16, service_not_message: bool, source_node: u8) -> u32 {
    let p = (priority as u32 & 0x1F) << 24;
    let t = (type_id as u32 & 0xFFFF) << 8;
    let s = if service_not_message { 1u32 << 7 } else { 0 };
    let n = (source_node as u32) & 0x7F;
    p | t | s | n
}

/// Decode a 29-bit CAN ID into its DroneCAN fields.
#[inline]
pub fn decode_can_id(id: u32) -> (u8, u16, bool, u8) {
    let priority = ((id >> 24) & 0x1F) as u8;
    let type_id = ((id >> 8) & 0xFFFF) as u16;
    let service_not_message = (id >> 7) & 1 != 0;
    let source_node = (id & 0x7F) as u8;
    (priority, type_id, service_not_message, source_node)
}

// ---------------------------------------------------------------------------
// Tail byte
// ---------------------------------------------------------------------------

/// UAVCAN v0 tail byte, appended to each CAN frame of a transfer.
///
/// Bit layout: `[start_of_transfer | end_of_transfer | toggle | transfer_id(5)]`
///   - Bit 7: start of transfer
///   - Bit 6: end of transfer
///   - Bit 5: toggle (alternates per frame within a multi-frame transfer)
///   - Bits 4..0: transfer_id (wraps at 32)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TailByte(pub u8);

impl TailByte {
    /// Create a tail byte for a single-frame transfer.
    pub fn single_frame(transfer_id: u8) -> Self {
        // SOT=1, EOT=1, toggle=0
        TailByte(0b1100_0000 | (transfer_id & 0x1F))
    }

    /// Create a tail byte for the first frame of a multi-frame transfer.
    pub fn first_frame(transfer_id: u8) -> Self {
        // SOT=1, EOT=0, toggle=0
        TailByte(0b1000_0000 | (transfer_id & 0x1F))
    }

    /// Create a tail byte for a middle frame of a multi-frame transfer.
    pub fn middle_frame(transfer_id: u8, toggle: bool) -> Self {
        // SOT=0, EOT=0
        let t = if toggle { 1u8 << 5 } else { 0 };
        TailByte(t | (transfer_id & 0x1F))
    }

    /// Create a tail byte for the last frame of a multi-frame transfer.
    pub fn last_frame(transfer_id: u8, toggle: bool) -> Self {
        // SOT=0, EOT=1
        let t = if toggle { 1u8 << 5 } else { 0 };
        TailByte(0b0100_0000 | t | (transfer_id & 0x1F))
    }

    #[inline]
    pub fn start_of_transfer(self) -> bool { self.0 & 0x80 != 0 }

    #[inline]
    pub fn end_of_transfer(self) -> bool { self.0 & 0x40 != 0 }

    #[inline]
    pub fn toggle(self) -> bool { self.0 & 0x20 != 0 }

    #[inline]
    pub fn transfer_id(self) -> u8 { self.0 & 0x1F }
}

// ---------------------------------------------------------------------------
// Frame encode / decode
// ---------------------------------------------------------------------------

/// Maximum data bytes per CAN frame (classic CAN = 8 bytes total, 7 data + 1 tail).
const MAX_DATA_PER_FRAME: usize = 7;

/// Encode a single-frame DroneCAN transfer into a CAN frame.
///
/// For payloads up to 7 bytes. Returns a single `CanFrame` with the tail byte
/// appended. For multi-frame transfers, use `encode_multi_frame`.
pub fn encode_frame(
    priority: u8,
    type_id: u16,
    source_node: u8,
    payload: &[u8],
    transfer_id: u8,
) -> CanFrame {
    let id = encode_can_id(priority, type_id, false, source_node);
    let tail = TailByte::single_frame(transfer_id);

    let mut data = [0u8; 64];
    let len = payload.len().min(MAX_DATA_PER_FRAME);
    data[..len].copy_from_slice(&payload[..len]);
    data[len] = tail.0;

    CanFrame {
        id,
        extended: true,
        data,
        dlc: (len + 1) as u8, // payload + tail byte
    }
}

/// Decode a single CAN frame, extracting the DroneCAN fields and payload.
///
/// Returns `(priority, type_id, source_node, payload_slice, tail_byte)`.
pub fn decode_frame(frame: &CanFrame) -> (u8, u16, u8, &[u8], TailByte) {
    let (priority, type_id, _service, source_node) = decode_can_id(frame.id);
    let data = frame.data_slice();
    if data.is_empty() {
        return (priority, type_id, source_node, &[], TailByte(0));
    }
    let tail = TailByte(data[data.len() - 1]);
    let payload = &data[..data.len() - 1];
    (priority, type_id, source_node, payload, tail)
}

// ---------------------------------------------------------------------------
// Multi-frame transfer encoding
// ---------------------------------------------------------------------------

/// Encode a multi-frame DroneCAN transfer.
///
/// Splits a payload larger than 7 bytes into multiple CAN frames.
/// The first two bytes of the first frame carry the CRC16-CCITT of the full
/// payload (UAVCAN v0 multi-frame convention). Remaining bytes are split
/// into 7-byte chunks, each followed by a tail byte.
///
/// Returns the number of frames written to `out`.
pub fn encode_multi_frame(
    priority: u8,
    type_id: u16,
    source_node: u8,
    payload: &[u8],
    transfer_id: u8,
    out: &mut [CanFrame],
) -> usize {
    if payload.len() <= MAX_DATA_PER_FRAME {
        // Single frame — caller should use encode_frame, but handle gracefully
        if out.is_empty() {
            return 0;
        }
        out[0] = encode_frame(priority, type_id, source_node, payload, transfer_id);
        return 1;
    }

    let can_id = encode_can_id(priority, type_id, false, source_node);
    let crc = crc16_ccitt(payload);
    let crc_bytes = crc.to_le_bytes();

    // Build the full transfer payload: CRC(2) + original payload
    // We iterate through this virtual buffer in 7-byte chunks.
    let total_len = 2 + payload.len();
    let mut frame_idx = 0;
    let mut byte_offset = 0;
    let mut toggle = false;

    while byte_offset < total_len {
        if frame_idx >= out.len() {
            break;
        }

        let remaining = total_len - byte_offset;
        let chunk_len = remaining.min(MAX_DATA_PER_FRAME);
        let is_first = byte_offset == 0;
        let is_last = byte_offset + chunk_len >= total_len;

        // Build the tail byte
        let tail = if is_first {
            TailByte::first_frame(transfer_id)
        } else if is_last {
            TailByte::last_frame(transfer_id, toggle)
        } else {
            TailByte::middle_frame(transfer_id, toggle)
        };

        // Copy chunk data from the virtual CRC+payload buffer
        let mut data = [0u8; 64];
        for i in 0..chunk_len {
            let src_idx = byte_offset + i;
            data[i] = if src_idx < 2 {
                crc_bytes[src_idx]
            } else {
                payload[src_idx - 2]
            };
        }
        data[chunk_len] = tail.0;

        out[frame_idx] = CanFrame {
            id: can_id,
            extended: true,
            data,
            dlc: (chunk_len + 1) as u8,
        };

        byte_offset += chunk_len;
        frame_idx += 1;
        // After first frame, toggle starts at true and alternates
        toggle = !toggle;
    }

    frame_idx
}

// ---------------------------------------------------------------------------
// Multi-frame transfer reassembly
// ---------------------------------------------------------------------------

/// Maximum multi-frame reassembly buffer size.
///
/// ArduPilot's libcanard uses an 8 KB dynamic memory pool. Meridian uses a
/// fixed buffer to remain `no_std`/alloc-free. 8192 bytes accommodates all
/// standard DroneCAN transfers including firmware updates and file transfers.
const TRANSFER_BUF_SIZE: usize = 8192;

/// Reassembles multi-frame DroneCAN transfers.
///
/// Feed CAN frames one at a time via `push_frame`. When a complete transfer
/// is assembled, `push_frame` returns `Some(payload_length)` and the payload
/// is available in the internal buffer.
pub struct TransferAssembler {
    buf: [u8; TRANSFER_BUF_SIZE],
    len: usize,
    expected_toggle: bool,
    transfer_id: u8,
    active: bool,
}

impl TransferAssembler {
    pub const fn new() -> Self {
        Self {
            buf: [0u8; TRANSFER_BUF_SIZE],
            len: 0,
            expected_toggle: false,
            transfer_id: 0,
            active: false,
        }
    }

    /// Feed a decoded payload + tail byte. Returns `Some(payload_len)` when
    /// a complete multi-frame transfer has been assembled.
    pub fn push_frame(&mut self, payload: &[u8], tail: TailByte) -> Option<usize> {
        if tail.start_of_transfer() {
            // Start new transfer
            self.buf[..payload.len()].copy_from_slice(payload);
            self.len = payload.len();
            self.transfer_id = tail.transfer_id();
            self.expected_toggle = true; // next frame expects toggle=1
            self.active = true;

            if tail.end_of_transfer() {
                // Single frame transfer
                self.active = false;
                return Some(self.len);
            }
            return None;
        }

        if !self.active {
            return None;
        }

        // Check transfer ID matches
        if tail.transfer_id() != self.transfer_id {
            self.active = false;
            return None;
        }

        // Check toggle bit
        if tail.toggle() != self.expected_toggle {
            self.active = false;
            return None;
        }
        self.expected_toggle = !self.expected_toggle;

        // Append data
        let end = self.len + payload.len();
        if end > self.buf.len() {
            self.active = false;
            return None; // overflow
        }
        self.buf[self.len..end].copy_from_slice(payload);
        self.len = end;

        if tail.end_of_transfer() {
            self.active = false;
            // Validate CRC: first 2 bytes are CRC of the rest
            if self.len < 2 {
                return None;
            }
            let received_crc = u16::from_le_bytes([self.buf[0], self.buf[1]]);
            let computed_crc = crc16_ccitt(&self.buf[2..self.len]);
            if received_crc != computed_crc {
                return None; // CRC mismatch
            }
            // Shift payload down to remove CRC prefix
            let payload_len = self.len - 2;
            self.buf.copy_within(2..self.len, 0);
            self.len = payload_len;
            return Some(payload_len);
        }

        None
    }

    /// Get the assembled payload after a successful `push_frame` return.
    pub fn payload(&self) -> &[u8] {
        &self.buf[..self.len]
    }

    /// Reset the assembler state.
    pub fn reset(&mut self) {
        self.active = false;
        self.len = 0;
    }
}

// ---------------------------------------------------------------------------
// CRC16-CCITT (0x1021 polynomial, 0xFFFF initial)
// ---------------------------------------------------------------------------

/// CRC16-CCITT as used by UAVCAN v0 for multi-frame transfer integrity.
///
/// Polynomial: 0x1021, initial value: 0xFFFF, no final XOR.
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
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

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_can_id_roundtrip() {
        let priority = 16u8;
        let type_id = 341u16; // NodeStatus
        let source_node = 10u8;

        let id = encode_can_id(priority, type_id, false, source_node);
        let (p, t, s, n) = decode_can_id(id);

        assert_eq!(p, priority);
        assert_eq!(t, type_id);
        assert!(!s);
        assert_eq!(n, source_node);
    }

    #[test]
    fn test_can_id_service_flag() {
        let id = encode_can_id(0, 1, true, 42);
        let (p, t, s, n) = decode_can_id(id);
        assert_eq!(p, 0);
        assert_eq!(t, 1);
        assert!(s);
        assert_eq!(n, 42);
    }

    #[test]
    fn test_single_frame_roundtrip() {
        let payload = [1u8, 2, 3, 4];
        let frame = encode_frame(PRIORITY_MEDIUM, 341, 10, &payload, 5);

        assert!(frame.extended);
        assert_eq!(frame.dlc, 5); // 4 data + 1 tail

        let (priority, type_id, source_node, decoded_payload, tail) = decode_frame(&frame);
        assert_eq!(priority, PRIORITY_MEDIUM);
        assert_eq!(type_id, 341);
        assert_eq!(source_node, 10);
        assert_eq!(decoded_payload, &payload);
        assert!(tail.start_of_transfer());
        assert!(tail.end_of_transfer());
        assert_eq!(tail.transfer_id(), 5);
    }

    #[test]
    fn test_tail_byte_single() {
        let tb = TailByte::single_frame(7);
        assert!(tb.start_of_transfer());
        assert!(tb.end_of_transfer());
        assert!(!tb.toggle());
        assert_eq!(tb.transfer_id(), 7);
    }

    #[test]
    fn test_tail_byte_multi() {
        let first = TailByte::first_frame(3);
        assert!(first.start_of_transfer());
        assert!(!first.end_of_transfer());
        assert!(!first.toggle());
        assert_eq!(first.transfer_id(), 3);

        let mid = TailByte::middle_frame(3, true);
        assert!(!mid.start_of_transfer());
        assert!(!mid.end_of_transfer());
        assert!(mid.toggle());
        assert_eq!(mid.transfer_id(), 3);

        let last = TailByte::last_frame(3, false);
        assert!(!last.start_of_transfer());
        assert!(last.end_of_transfer());
        assert!(!last.toggle());
        assert_eq!(last.transfer_id(), 3);
    }

    #[test]
    fn test_crc16_ccitt_known() {
        // Well-known test vector: "123456789" -> 0x29B1
        let data = b"123456789";
        let crc = crc16_ccitt(data);
        assert_eq!(crc, 0x29B1);
    }

    #[test]
    fn test_crc16_empty() {
        assert_eq!(crc16_ccitt(&[]), 0xFFFF);
    }

    #[test]
    fn test_multi_frame_encode_decode() {
        // Create a payload larger than 7 bytes to force multi-frame
        let payload: [u8; 20] = [
            0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80,
            0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0, 0x01,
            0x02, 0x03, 0x04, 0x05,
        ];

        let mut frames = [CanFrame::new_extended(0, &[]); 8];
        let n = encode_multi_frame(PRIORITY_LOW, 1063, 10, &payload, 2, &mut frames);

        // 2 CRC bytes + 20 payload = 22 bytes, at 7 bytes per frame = ceil(22/7) = 4 frames
        assert_eq!(n, 4);

        // Reassemble
        let mut asm = TransferAssembler::new();
        let mut result = None;
        for i in 0..n {
            let (_, _, _, data, tail) = decode_frame(&frames[i]);
            result = asm.push_frame(data, tail);
        }

        assert!(result.is_some());
        assert_eq!(result.unwrap(), 20);
        assert_eq!(&asm.payload()[..20], &payload);
    }

    #[test]
    fn test_multi_frame_crc_mismatch() {
        let payload = [0xAAu8; 15];
        let mut frames = [CanFrame::new_extended(0, &[]); 8];
        let n = encode_multi_frame(PRIORITY_LOW, 100, 10, &payload, 0, &mut frames);
        assert!(n > 1);

        // Corrupt a byte in the second frame
        frames[1].data[0] ^= 0xFF;

        let mut asm = TransferAssembler::new();
        let mut result = None;
        for i in 0..n {
            let (_, _, _, data, tail) = decode_frame(&frames[i]);
            result = asm.push_frame(data, tail);
        }

        // Should fail CRC check
        assert!(result.is_none());
    }
}

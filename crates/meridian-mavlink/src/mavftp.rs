//! MAVFTP — MAVLink file transfer protocol stub.
//!
//! ArduPilot reference: `GCS_FTP.cpp`
//!
//! MAVFTP uses the FILE_TRANSFER_PROTOCOL (MSG_ID=110) message to implement
//! a file transfer protocol over MAVLink. QGroundControl uses this to:
//! - Download flight logs
//! - Upload parameter files
//! - Browse the filesystem
//!
//! The protocol is a request/response state machine with opcodes for:
//! open, read, write, create, remove, list directory, etc.

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// MAVLink message ID for FILE_TRANSFER_PROTOCOL.
pub const MSG_FILE_TRANSFER_PROTOCOL: u32 = 110;

/// MAVFTP opcodes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FtpOpcode {
    None = 0,
    TerminateSession = 1,
    ResetSessions = 2,
    ListDirectory = 3,
    OpenFileRO = 4,
    ReadFile = 5,
    CreateFile = 6,
    WriteFile = 7,
    RemoveFile = 8,
    CreateDirectory = 9,
    RemoveDirectory = 10,
    OpenFileWO = 11,
    TruncateFile = 12,
    Rename = 13,
    CalcFileCRC32 = 14,
    BurstReadFile = 15,
    Ack = 128,
    Nak = 129,
}

/// MAVFTP error codes for NAK responses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FtpError {
    None = 0,
    Fail = 1,
    FailErrno = 2,
    InvalidDataSize = 3,
    InvalidSession = 4,
    NoSessionsAvailable = 5,
    EndOfFile = 6,
    UnknownCommand = 7,
    FileExists = 8,
    FileProtected = 9,
    FileNotFound = 10,
}

/// MAVFTP payload header (12 bytes).
#[derive(Debug, Clone, Copy)]
pub struct FtpHeader {
    /// Sequence number (increments per request).
    pub seq_number: u16,
    /// Session ID.
    pub session: u8,
    /// Request/response opcode.
    pub opcode: u8,
    /// Size of data in this payload.
    pub size: u8,
    /// Request-specific opcode (for responses).
    pub req_opcode: u8,
    /// Burst flag (for burst reads).
    pub burst_complete: u8,
    /// Padding.
    pub padding: u8,
    /// Offset into file.
    pub offset: u32,
}

impl FtpHeader {
    /// Decode from a 12-byte buffer.
    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 12 {
            return None;
        }
        Some(FtpHeader {
            seq_number: u16::from_le_bytes([data[0], data[1]]),
            session: data[2],
            opcode: data[3],
            size: data[4],
            req_opcode: data[5],
            burst_complete: data[6],
            padding: data[7],
            offset: u32::from_le_bytes([data[8], data[9], data[10], data[11]]),
        })
    }

    /// Encode into a 12-byte buffer.
    pub fn encode(&self, buf: &mut [u8]) {
        if buf.len() < 12 {
            return;
        }
        let seq = self.seq_number.to_le_bytes();
        buf[0] = seq[0];
        buf[1] = seq[1];
        buf[2] = self.session;
        buf[3] = self.opcode;
        buf[4] = self.size;
        buf[5] = self.req_opcode;
        buf[6] = self.burst_complete;
        buf[7] = self.padding;
        let off = self.offset.to_le_bytes();
        buf[8..12].copy_from_slice(&off);
    }
}

/// MAVFTP session state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionState {
    Idle,
    Open { session_id: u8 },
}

/// MAVFTP server stub.
///
/// Handles FILE_TRANSFER_PROTOCOL messages and dispatches to a filesystem
/// backend. The actual filesystem operations are deferred to a callback
/// trait that the platform must implement.
pub struct MavFtpServer {
    /// Current session state.
    pub state: SessionState,
    /// Sequence counter.
    pub seq: u16,
    /// Maximum data payload per FTP message (239 bytes max).
    pub max_data_len: u8,
}

impl MavFtpServer {
    pub fn new() -> Self {
        Self {
            state: SessionState::Idle,
            seq: 0,
            max_data_len: 239,
        }
    }

    /// Handle an incoming FTP request.
    ///
    /// `payload`: the payload from FILE_TRANSFER_PROTOCOL message (after the
    ///            MAVLink header). First 12 bytes are the FTP header, rest is data.
    ///
    /// Returns a response payload to send back, or None if no response needed.
    ///
    /// TODO: Connect to actual filesystem backend when platform storage is available.
    pub fn handle_request(&mut self, payload: &[u8]) -> Option<([u8; 251], usize)> {
        let header = FtpHeader::decode(payload)?;
        let _data = if payload.len() > 12 { &payload[12..] } else { &[] as &[u8] };

        let mut response = [0u8; 251];
        let mut resp_header = FtpHeader {
            seq_number: header.seq_number + 1,
            session: header.session,
            opcode: FtpOpcode::Nak as u8,
            size: 1,
            req_opcode: header.opcode,
            burst_complete: 0,
            padding: 0,
            offset: 0,
        };

        match header.opcode {
            op if op == FtpOpcode::ListDirectory as u8 => {
                // Stub: return empty directory
                resp_header.opcode = FtpOpcode::Nak as u8;
                resp_header.size = 1;
                resp_header.encode(&mut response);
                response[12] = FtpError::EndOfFile as u8;
                Some((response, 13))
            }
            op if op == FtpOpcode::OpenFileRO as u8 => {
                // Stub: NAK with file not found
                resp_header.opcode = FtpOpcode::Nak as u8;
                resp_header.size = 1;
                resp_header.encode(&mut response);
                response[12] = FtpError::FileNotFound as u8;
                Some((response, 13))
            }
            op if op == FtpOpcode::ResetSessions as u8 => {
                self.state = SessionState::Idle;
                resp_header.opcode = FtpOpcode::Ack as u8;
                resp_header.size = 0;
                resp_header.encode(&mut response);
                Some((response, 12))
            }
            op if op == FtpOpcode::TerminateSession as u8 => {
                self.state = SessionState::Idle;
                resp_header.opcode = FtpOpcode::Ack as u8;
                resp_header.size = 0;
                resp_header.encode(&mut response);
                Some((response, 12))
            }
            _ => {
                // Unknown command
                resp_header.opcode = FtpOpcode::Nak as u8;
                resp_header.size = 1;
                resp_header.encode(&mut response);
                response[12] = FtpError::UnknownCommand as u8;
                Some((response, 13))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ftp_header_roundtrip() {
        let header = FtpHeader {
            seq_number: 42,
            session: 1,
            opcode: FtpOpcode::ListDirectory as u8,
            size: 10,
            req_opcode: 0,
            burst_complete: 0,
            padding: 0,
            offset: 1024,
        };
        let mut buf = [0u8; 12];
        header.encode(&mut buf);
        let decoded = FtpHeader::decode(&buf).unwrap();
        assert_eq!(decoded.seq_number, 42);
        assert_eq!(decoded.session, 1);
        assert_eq!(decoded.opcode, FtpOpcode::ListDirectory as u8);
        assert_eq!(decoded.offset, 1024);
    }

    #[test]
    fn test_reset_sessions() {
        let mut server = MavFtpServer::new();
        let mut payload = [0u8; 12];
        let header = FtpHeader {
            seq_number: 1,
            session: 0,
            opcode: FtpOpcode::ResetSessions as u8,
            size: 0,
            req_opcode: 0,
            burst_complete: 0,
            padding: 0,
            offset: 0,
        };
        header.encode(&mut payload);
        let result = server.handle_request(&payload);
        assert!(result.is_some());
    }
}

//! MAVLink log download protocol stub.
//!
//! ArduPilot reference: `GCS_MAVLink/GCS_FTP.cpp`, log download handlers
//!
//! Implements the LOG_REQUEST_LIST, LOG_REQUEST_DATA, LOG_ENTRY, LOG_DATA
//! message handlers that allow QGC to browse and download flight logs.

// ---------------------------------------------------------------------------
// MAVLink message IDs for log protocol
// ---------------------------------------------------------------------------

/// LOG_REQUEST_LIST (117): GCS requests list of available logs.
pub const MSG_LOG_REQUEST_LIST: u32 = 117;

/// LOG_REQUEST_DATA (119): GCS requests a portion of a log.
pub const MSG_LOG_REQUEST_DATA: u32 = 119;

/// LOG_ENTRY (118): FC sends log metadata to GCS.
pub const MSG_LOG_ENTRY: u32 = 118;

/// LOG_DATA (120): FC sends log data chunk to GCS.
pub const MSG_LOG_DATA: u32 = 120;

/// LOG_REQUEST_END (122): GCS tells FC to stop sending.
pub const MSG_LOG_REQUEST_END: u32 = 122;

/// LOG_ERASE (121): GCS requests FC to erase all logs.
pub const MSG_LOG_ERASE: u32 = 121;

// ---------------------------------------------------------------------------
// Log entry metadata
// ---------------------------------------------------------------------------

/// Metadata for a single log file.
#[derive(Debug, Clone, Copy)]
pub struct LogEntry {
    /// Log number (unique ID).
    pub id: u16,
    /// Total number of logs.
    pub num_logs: u16,
    /// Last log number.
    pub last_log_num: u16,
    /// UTC timestamp of log creation (seconds since epoch).
    pub time_utc: u32,
    /// Size in bytes.
    pub size: u32,
}

impl LogEntry {
    /// Encode a LOG_ENTRY payload (14 bytes).
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 14 {
            return 0;
        }
        buf[0..2].copy_from_slice(&self.id.to_le_bytes());
        buf[2..4].copy_from_slice(&self.num_logs.to_le_bytes());
        buf[4..6].copy_from_slice(&self.last_log_num.to_le_bytes());
        buf[6..10].copy_from_slice(&self.time_utc.to_le_bytes());
        buf[10..14].copy_from_slice(&self.size.to_le_bytes());
        14
    }
}

/// Log data chunk (90 bytes payload max).
#[derive(Debug, Clone)]
pub struct LogDataChunk {
    /// Log number.
    pub id: u16,
    /// Offset in bytes from the start of the log.
    pub offset: u32,
    /// Number of valid bytes in data (max 90).
    pub count: u8,
    /// Log data bytes.
    pub data: [u8; 90],
}

impl LogDataChunk {
    /// Encode a LOG_DATA payload (97 bytes).
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 97 {
            return 0;
        }
        buf[0..2].copy_from_slice(&self.id.to_le_bytes());
        buf[2..6].copy_from_slice(&self.offset.to_le_bytes());
        buf[6] = self.count;
        let n = self.count.min(90) as usize;
        buf[7..7 + n].copy_from_slice(&self.data[..n]);
        // Zero-pad remaining
        for i in (7 + n)..97 {
            buf[i] = 0;
        }
        97
    }
}

// ---------------------------------------------------------------------------
// Log download server
// ---------------------------------------------------------------------------

/// State of the log download protocol.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LogDownloadState {
    /// No active download.
    Idle,
    /// Sending log list entries.
    SendingList { start: u16, end: u16, current: u16 },
    /// Sending log data.
    SendingData { log_id: u16, offset: u32, remaining: u32 },
}

/// Log download protocol handler.
///
/// Manages the state machine for responding to GCS log download requests.
/// The actual log storage/retrieval is delegated to a platform callback.
pub struct LogDownloadServer {
    /// Current state.
    pub state: LogDownloadState,
}

impl LogDownloadServer {
    pub fn new() -> Self {
        Self {
            state: LogDownloadState::Idle,
        }
    }

    /// Handle LOG_REQUEST_LIST: start sending log entries.
    ///
    /// `start`: first log ID requested.
    /// `end`: last log ID requested (0xFFFF = all).
    pub fn handle_request_list(&mut self, start: u16, end: u16) {
        self.state = LogDownloadState::SendingList { start, end, current: start };
    }

    /// Handle LOG_REQUEST_DATA: start sending log data.
    ///
    /// `log_id`: which log to download.
    /// `offset`: byte offset to start from.
    /// `count`: number of bytes requested.
    pub fn handle_request_data(&mut self, log_id: u16, offset: u32, count: u32) {
        self.state = LogDownloadState::SendingData { log_id, offset, remaining: count };
    }

    /// Handle LOG_REQUEST_END: stop sending.
    pub fn handle_request_end(&mut self) {
        self.state = LogDownloadState::Idle;
    }

    /// Handle LOG_ERASE: erase all logs.
    ///
    /// TODO: Connect to platform storage to actually erase logs.
    pub fn handle_erase(&mut self) {
        self.state = LogDownloadState::Idle;
    }

    /// Check if there is pending data to send.
    pub fn has_pending(&self) -> bool {
        !matches!(self.state, LogDownloadState::Idle)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_log_entry_encode() {
        let entry = LogEntry {
            id: 1,
            num_logs: 5,
            last_log_num: 5,
            time_utc: 1234567890,
            size: 102400,
        };
        let mut buf = [0u8; 14];
        let len = entry.encode(&mut buf);
        assert_eq!(len, 14);
        assert_eq!(u16::from_le_bytes([buf[0], buf[1]]), 1);
    }

    #[test]
    fn test_log_download_state_machine() {
        let mut server = LogDownloadServer::new();
        assert!(!server.has_pending());

        server.handle_request_list(1, 10);
        assert!(server.has_pending());

        server.handle_request_end();
        assert!(!server.has_pending());
    }
}

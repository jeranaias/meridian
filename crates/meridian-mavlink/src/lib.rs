#![no_std]

//! MAVLink v2 adapter — translates Meridian bus messages to/from MAVLink.
//!
//! This is NOT a bridge (separate process). It runs ON the FC as a protocol
//! adapter, converting the internal bus to MAVLink wire format so QGC,
//! Mission Planner, and ATAK plugins can connect directly.
//!
//! MAVLink v2 frame format:
//!   [0xFD] [len] [incompat] [compat] [seq] [sysid] [compid] [msgid:3] [payload] [crc:2]

pub mod v2;
pub mod adapter;
pub mod server;

/// MAVFTP file transfer protocol stub.
pub mod mavftp;

/// MAVLink v2 signing (HMAC-SHA256 authentication) stub.
pub mod signing;

/// Log download protocol stub (LOG_REQUEST_LIST/DATA).
pub mod log_download;

pub use adapter::MavlinkAdapter;
pub use server::MavlinkServer;

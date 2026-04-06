#![no_std]

//! Meridian Native Protocol (MNP) — compact binary wire protocol.
//!
//! Design goals:
//! - Fits 57600 baud serial (5.7 KB/s) — typical RFD 900x link
//! - Self-framed (COBS encoding) — no byte stuffing ambiguity
//! - Zero-alloc serialization via postcard (serde)
//! - Works over UART, USB, UDP, WebSocket — transport agnostic
//! - No bridges — FC speaks this directly
//!
//! Frame format: `[0x00] [COBS-encoded payload] [0x00]`
//! Payload (before COBS): `[msg_id: 1B] [seq_le: 2B] [postcard body: NB]`
//!
//! At 57600 baud with 10Hz attitude + 5Hz position + 1Hz heartbeat:
//! ~40 bytes/msg * 16 msgs/s = 640 bytes/s = 11% of link capacity.

pub mod wire;
pub mod messages;
pub mod transport;

pub use wire::{encode_frame, decode_frame, MAX_FRAME_SIZE};
pub use messages::{MsgId, MnpMessage};
pub use transport::Transport;

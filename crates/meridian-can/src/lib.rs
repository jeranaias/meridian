#![no_std]
//! DroneCAN (UAVCAN v0) protocol implementation for Meridian.
//!
//! Pure Rust implementation of:
//! - CAN frame encoding/decoding with 29-bit extended IDs
//! - Multi-frame transfer protocol with CRC16-CCITT
//! - Standard DroneCAN message types (NodeStatus, Fix2, RawCommand, etc.)
//! - Node management with Dynamic Node Allocation (DNA) server
//! - Sensor integration adapters producing Meridian message bus types

pub mod frames;
pub mod node;
pub mod messages;
pub mod sensors;

pub use frames::{encode_frame, decode_frame, TailByte, TransferAssembler, crc16_ccitt};
pub use node::CanardNode;
pub use messages::*;
pub use sensors::{CanGps, CanCompass, CanBaro, CanBattery};

//! Meridian Native Protocol message definitions.
//!
//! Each message has a unique ID and a serde-serializable body.
//! Serialized with postcard (zero-alloc, varint encoding).

use serde::{Serialize, Deserialize};

/// Message ID (single byte — max 256 message types).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MsgId(pub u8);

// ─── Message IDs ───
// Grouped by function. IDs 0x00-0x0F: system, 0x10-0x2F: telemetry,
// 0x30-0x4F: commands, 0x50-0x6F: params, 0x70-0x7F: mission.

impl MsgId {
    // System
    pub const HEARTBEAT: MsgId = MsgId(0x01);
    pub const SYSTEM_STATUS: MsgId = MsgId(0x02);
    pub const STATUSTEXT: MsgId = MsgId(0x03);

    // Telemetry (vehicle → GCS)
    pub const ATTITUDE: MsgId = MsgId(0x10);
    pub const POSITION: MsgId = MsgId(0x11);
    pub const GPS_RAW: MsgId = MsgId(0x12);
    pub const BATTERY: MsgId = MsgId(0x13);
    pub const RC_CHANNELS: MsgId = MsgId(0x14);
    pub const EKF_STATUS: MsgId = MsgId(0x15);
    pub const MOTOR_OUTPUT: MsgId = MsgId(0x16);
    pub const SENSOR_HEALTH: MsgId = MsgId(0x17);
    pub const VFR_HUD: MsgId = MsgId(0x18);

    // Commands (GCS → vehicle)
    pub const CMD_ARM: MsgId = MsgId(0x30);
    pub const CMD_DISARM: MsgId = MsgId(0x31);
    pub const CMD_TAKEOFF: MsgId = MsgId(0x32);
    pub const CMD_LAND: MsgId = MsgId(0x33);
    pub const CMD_RTL: MsgId = MsgId(0x34);
    pub const CMD_SET_MODE: MsgId = MsgId(0x35);
    pub const CMD_GOTO: MsgId = MsgId(0x36);
    pub const CMD_SET_SPEED: MsgId = MsgId(0x37);
    pub const CMD_ACK: MsgId = MsgId(0x38);

    // Parameters
    pub const PARAM_REQUEST: MsgId = MsgId(0x50);
    pub const PARAM_VALUE: MsgId = MsgId(0x51);
    pub const PARAM_SET: MsgId = MsgId(0x52);

    // Mission
    pub const MISSION_COUNT: MsgId = MsgId(0x70);
    pub const MISSION_ITEM: MsgId = MsgId(0x71);
    pub const MISSION_REQUEST: MsgId = MsgId(0x72);
    pub const MISSION_ACK: MsgId = MsgId(0x73);
    pub const MISSION_CURRENT: MsgId = MsgId(0x74);
}

// ─── Message Bodies ───

/// Heartbeat: sent by FC at 1Hz. This is the "I'm alive" signal.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Heartbeat {
    pub vehicle_type: u8,    // 0=unknown, 1=quad, 2=plane, 3=rover, 4=boat, 5=sub
    pub armed: bool,
    pub mode: u8,            // FlightModeId as u8
    pub system_status: u8,   // 0=uninit, 1=boot, 2=calibrating, 3=standby, 4=active, 5=critical
}

/// Attitude: roll/pitch/yaw + rates. Sent at 10-20Hz.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Attitude {
    pub roll: f32,          // rad
    pub pitch: f32,         // rad
    pub yaw: f32,           // rad
    pub rollspeed: f32,     // rad/s
    pub pitchspeed: f32,    // rad/s
    pub yawspeed: f32,      // rad/s
}

/// Position: lat/lon/alt + velocity. Sent at 5-10Hz.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Position {
    pub lat_e7: i32,        // latitude * 1e7 (degrees)
    pub lon_e7: i32,        // longitude * 1e7 (degrees)
    pub alt_mm: i32,        // altitude MSL (millimeters)
    pub relative_alt_mm: i32, // altitude above home (mm)
    pub vx: i16,            // velocity N (cm/s)
    pub vy: i16,            // velocity E (cm/s)
    pub vz: i16,            // velocity D (cm/s)
    pub heading: u16,       // heading (cdeg, 0-35999)
}

/// GPS raw data.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GpsRaw {
    pub fix_type: u8,       // 0=none, 2=2D, 3=3D, 4=DGPS, 5=RTK_FLOAT, 6=RTK_FIXED
    pub num_sats: u8,
    pub hdop: u16,          // HDOP * 100
    pub lat_e7: i32,
    pub lon_e7: i32,
    pub alt_mm: i32,
}

/// Battery status.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Battery {
    pub voltage_mv: u16,    // millivolts
    pub current_ca: i16,    // centi-amps (100 = 1A)
    pub remaining_pct: u8,  // 0-100
    pub consumed_mah: u16,
}

/// VFR HUD (pilot-oriented display data).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VfrHud {
    pub airspeed: f32,      // m/s
    pub groundspeed: f32,   // m/s
    pub heading: i16,       // degrees 0-359
    pub throttle: u8,       // percent 0-100
    pub alt: f32,           // m MSL
    pub climb: f32,         // m/s
}

/// EKF health status.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EkfStatus {
    pub healthy: bool,
    pub vel_innov: [f32; 3],
    pub pos_innov: [f32; 2],
    pub hgt_innov: f32,
}

/// Status text message.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StatusText {
    pub severity: u8,       // 0=emergency..7=debug
    pub text: heapless::String<50>,
}

/// Command acknowledgment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CmdAck {
    pub command_id: u8,     // MsgId of the command being acknowledged
    pub result: u8,         // 0=accepted, 1=rejected, 2=in_progress, 3=failed
}

/// Go-to command: fly/drive to position.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CmdGoto {
    pub lat_e7: i32,
    pub lon_e7: i32,
    pub alt_mm: i32,        // relative altitude (mm)
}

/// Set mode command.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CmdSetMode {
    pub mode: u8,           // FlightModeId as u8
}

/// Takeoff command.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CmdTakeoff {
    pub altitude_m: f32,
}

/// Parameter value (sent in response to request, or broadcast on change).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParamValue {
    pub index: u16,
    pub count: u16,         // total param count (for enumeration)
    pub name: heapless::String<16>,
    pub value: f32,
    pub param_type: u8,     // 0=f32, 1=i32, 2=u16, 3=u8
}

/// Parameter set request.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ParamSet {
    pub name: heapless::String<16>,
    pub value: f32,
    pub param_type: u8,
}

/// Unified message enum for dispatch.
#[derive(Debug, Clone)]
pub enum MnpMessage {
    Heartbeat(Heartbeat),
    Attitude(Attitude),
    Position(Position),
    GpsRaw(GpsRaw),
    Battery(Battery),
    VfrHud(VfrHud),
    EkfStatus(EkfStatus),
    StatusText(StatusText),
    CmdAck(CmdAck),
    CmdArm,
    CmdDisarm,
    CmdTakeoff(CmdTakeoff),
    CmdLand,
    CmdRtl,
    CmdSetMode(CmdSetMode),
    CmdGoto(CmdGoto),
    ParamValue(ParamValue),
    ParamSet(ParamSet),
}

impl MnpMessage {
    pub fn msg_id(&self) -> MsgId {
        match self {
            Self::Heartbeat(_) => MsgId::HEARTBEAT,
            Self::Attitude(_) => MsgId::ATTITUDE,
            Self::Position(_) => MsgId::POSITION,
            Self::GpsRaw(_) => MsgId::GPS_RAW,
            Self::Battery(_) => MsgId::BATTERY,
            Self::VfrHud(_) => MsgId::VFR_HUD,
            Self::EkfStatus(_) => MsgId::EKF_STATUS,
            Self::StatusText(_) => MsgId::STATUSTEXT,
            Self::CmdAck(_) => MsgId::CMD_ACK,
            Self::CmdArm => MsgId::CMD_ARM,
            Self::CmdDisarm => MsgId::CMD_DISARM,
            Self::CmdTakeoff(_) => MsgId::CMD_TAKEOFF,
            Self::CmdLand => MsgId::CMD_LAND,
            Self::CmdRtl => MsgId::CMD_RTL,
            Self::CmdSetMode(_) => MsgId::CMD_SET_MODE,
            Self::CmdGoto(_) => MsgId::CMD_GOTO,
            Self::ParamValue(_) => MsgId::PARAM_VALUE,
            Self::ParamSet(_) => MsgId::PARAM_SET,
        }
    }

    /// Serialize to a frame buffer. Returns frame length.
    pub fn encode(&self, seq: u16, frame_buf: &mut [u8]) -> usize {
        let mut body = [0u8; 200];
        let body_len = match self {
            Self::Heartbeat(m) => postcard::to_slice(m, &mut body).map(|s| s.len()),
            Self::Attitude(m) => postcard::to_slice(m, &mut body).map(|s| s.len()),
            Self::Position(m) => postcard::to_slice(m, &mut body).map(|s| s.len()),
            Self::GpsRaw(m) => postcard::to_slice(m, &mut body).map(|s| s.len()),
            Self::Battery(m) => postcard::to_slice(m, &mut body).map(|s| s.len()),
            Self::VfrHud(m) => postcard::to_slice(m, &mut body).map(|s| s.len()),
            Self::EkfStatus(m) => postcard::to_slice(m, &mut body).map(|s| s.len()),
            Self::StatusText(m) => postcard::to_slice(m, &mut body).map(|s| s.len()),
            Self::CmdAck(m) => postcard::to_slice(m, &mut body).map(|s| s.len()),
            Self::CmdArm | Self::CmdDisarm | Self::CmdLand | Self::CmdRtl => Ok(0),
            Self::CmdTakeoff(m) => postcard::to_slice(m, &mut body).map(|s| s.len()),
            Self::CmdSetMode(m) => postcard::to_slice(m, &mut body).map(|s| s.len()),
            Self::CmdGoto(m) => postcard::to_slice(m, &mut body).map(|s| s.len()),
            Self::ParamValue(m) => postcard::to_slice(m, &mut body).map(|s| s.len()),
            Self::ParamSet(m) => postcard::to_slice(m, &mut body).map(|s| s.len()),
        };

        match body_len {
            Ok(len) => crate::wire::encode_frame(self.msg_id().0, seq, &body[..len], frame_buf),
            Err(_) => 0,
        }
    }

    /// Decode from a COBS-decoded frame payload.
    pub fn decode(msg_id: u8, body: &[u8]) -> Option<Self> {
        let id = MsgId(msg_id);
        match id {
            MsgId::HEARTBEAT => postcard::from_bytes(body).ok().map(Self::Heartbeat),
            MsgId::ATTITUDE => postcard::from_bytes(body).ok().map(Self::Attitude),
            MsgId::POSITION => postcard::from_bytes(body).ok().map(Self::Position),
            MsgId::GPS_RAW => postcard::from_bytes(body).ok().map(Self::GpsRaw),
            MsgId::BATTERY => postcard::from_bytes(body).ok().map(Self::Battery),
            MsgId::VFR_HUD => postcard::from_bytes(body).ok().map(Self::VfrHud),
            MsgId::EKF_STATUS => postcard::from_bytes(body).ok().map(Self::EkfStatus),
            MsgId::STATUSTEXT => postcard::from_bytes(body).ok().map(Self::StatusText),
            MsgId::CMD_ACK => postcard::from_bytes(body).ok().map(Self::CmdAck),
            MsgId::CMD_ARM => Some(Self::CmdArm),
            MsgId::CMD_DISARM => Some(Self::CmdDisarm),
            MsgId::CMD_TAKEOFF => postcard::from_bytes(body).ok().map(Self::CmdTakeoff),
            MsgId::CMD_LAND => Some(Self::CmdLand),
            MsgId::CMD_RTL => Some(Self::CmdRtl),
            MsgId::CMD_SET_MODE => postcard::from_bytes(body).ok().map(Self::CmdSetMode),
            MsgId::CMD_GOTO => postcard::from_bytes(body).ok().map(Self::CmdGoto),
            MsgId::PARAM_VALUE => postcard::from_bytes(body).ok().map(Self::ParamValue),
            MsgId::PARAM_SET => postcard::from_bytes(body).ok().map(Self::ParamSet),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_heartbeat_roundtrip() {
        let msg = MnpMessage::Heartbeat(Heartbeat {
            vehicle_type: 1, armed: true, mode: 3, system_status: 4,
        });
        let mut frame = [0u8; crate::wire::MAX_FRAME_SIZE];
        let len = msg.encode(1, &mut frame);
        assert!(len > 0);

        // Decode the COBS payload
        let mut body = [0u8; 255];
        let (id, seq, blen) = crate::wire::decode_frame(&frame[1..len-1], &mut body).unwrap();
        assert_eq!(id, MsgId::HEARTBEAT.0);
        assert_eq!(seq, 1);

        let decoded = MnpMessage::decode(id, &body[..blen]).unwrap();
        match decoded {
            MnpMessage::Heartbeat(h) => {
                assert_eq!(h.vehicle_type, 1);
                assert!(h.armed);
                assert_eq!(h.mode, 3);
            }
            _ => panic!("Wrong message type"),
        }
    }

    #[test]
    fn test_attitude_size() {
        let msg = MnpMessage::Attitude(Attitude {
            roll: 0.1, pitch: -0.05, yaw: 1.5,
            rollspeed: 0.01, pitchspeed: -0.02, yawspeed: 0.0,
        });
        let mut frame = [0u8; crate::wire::MAX_FRAME_SIZE];
        let len = msg.encode(0, &mut frame);
        // 6 f32s = 24 bytes body + 3 header + COBS overhead + 2 delimiters ≈ 31 bytes
        assert!(len <= 35, "Attitude frame should be compact: {} bytes", len);
    }

    #[test]
    fn test_position_size() {
        let msg = MnpMessage::Position(Position {
            lat_e7: 350000000, lon_e7: -1200000000,
            alt_mm: 50000, relative_alt_mm: 10000,
            vx: 100, vy: -50, vz: 20, heading: 27000,
        });
        let mut frame = [0u8; crate::wire::MAX_FRAME_SIZE];
        let len = msg.encode(0, &mut frame);
        // Postcard uses varint — large ints compress less than small ones
        assert!(len <= 40, "Position frame: {} bytes", len);
    }

    #[test]
    fn test_command_arm_minimal() {
        let msg = MnpMessage::CmdArm;
        let mut frame = [0u8; crate::wire::MAX_FRAME_SIZE];
        let len = msg.encode(5, &mut frame);
        // Zero-body command: 3 header + COBS + 2 delimiters ≈ 7 bytes
        assert!(len <= 8, "Arm command should be tiny: {} bytes", len);
    }

    #[test]
    fn test_all_messages_encode_decode() {
        let messages: [MnpMessage; 10] = [
            MnpMessage::Heartbeat(Heartbeat { vehicle_type: 1, armed: false, mode: 0, system_status: 3 }),
            MnpMessage::Attitude(Attitude { roll: 0.1, pitch: -0.05, yaw: 1.5, rollspeed: 0.01, pitchspeed: -0.02, yawspeed: 0.03 }),
            MnpMessage::Position(Position { lat_e7: 350000000, lon_e7: -1200000000, alt_mm: 50000, relative_alt_mm: 10000, vx: 100, vy: -50, vz: 20, heading: 27000 }),
            MnpMessage::Battery(Battery { voltage_mv: 12600, current_ca: 500, remaining_pct: 85, consumed_mah: 200 }),
            MnpMessage::CmdArm,
            MnpMessage::CmdDisarm,
            MnpMessage::CmdLand,
            MnpMessage::CmdRtl,
            MnpMessage::CmdTakeoff(CmdTakeoff { altitude_m: 10.0 }),
            MnpMessage::CmdSetMode(CmdSetMode { mode: 5 }),
        ];

        for (i, msg) in messages.iter().enumerate() {
            let mut frame = [0u8; crate::wire::MAX_FRAME_SIZE];
            let len = msg.encode(i as u16, &mut frame);
            assert!(len > 0, "Message {:?} failed to encode", msg.msg_id());

            let mut body = [0u8; 255];
            let (id, seq, blen) = crate::wire::decode_frame(&frame[1..len-1], &mut body)
                .unwrap_or_else(|| panic!("Frame decode failed for msg {} (frame_len={}, cobs_len={})",
                    i, len, len-2));
            assert_eq!(seq, i as u16);

            let decoded = MnpMessage::decode(id, &body[..blen]);
            assert!(decoded.is_some(), "Message ID 0x{:02x} failed to decode", id);
        }
    }
}

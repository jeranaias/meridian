//! MAVLink adapter: converts between Meridian bus and MAVLink wire format.
//!
//! Runs on the FC. Reads bus messages, sends MAVLink. Receives MAVLink, posts to bus.
//! No separate process. No bridge.

use meridian_comms::transport::{Transport, TransportError};
use meridian_comms::messages as mnp;
use crate::v2::*;

/// MAVLink system/component IDs.
#[derive(Debug, Clone, Copy)]
pub struct MavIds {
    pub system_id: u8,
    pub component_id: u8,
}

impl Default for MavIds {
    fn default() -> Self {
        Self { system_id: 1, component_id: 1 }
    }
}

// MAV_TYPE constants
pub const MAV_TYPE_QUADROTOR: u8 = 2;
pub const MAV_TYPE_FIXED_WING: u8 = 1;
pub const MAV_TYPE_GROUND_ROVER: u8 = 10;
pub const MAV_TYPE_SURFACE_BOAT: u8 = 11;
pub const MAV_TYPE_SUBMARINE: u8 = 12;

// MAV_AUTOPILOT
pub const MAV_AUTOPILOT_ARDUPILOTMEGA: u8 = 3;

// MAV_MODE_FLAG bits
pub const MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: u8 = 0x01;
pub const MAV_MODE_FLAG_STABILIZE_ENABLED: u8 = 0x10;
pub const MAV_MODE_FLAG_GUIDED_ENABLED: u8 = 0x08;
pub const MAV_MODE_FLAG_AUTO_ENABLED: u8 = 0x04;
pub const MAV_MODE_FLAG_SAFETY_ARMED: u8 = 0x80;

// MAV_STATE
pub const MAV_STATE_UNINIT: u8 = 0;
pub const MAV_STATE_STANDBY: u8 = 3;
pub const MAV_STATE_ACTIVE: u8 = 4;
pub const MAV_STATE_CRITICAL: u8 = 5;
pub const MAV_STATE_EMERGENCY: u8 = 6;

// MAV_RESULT
pub const MAV_RESULT_ACCEPTED: u8 = 0;
pub const MAV_RESULT_DENIED: u8 = 1;
pub const MAV_RESULT_UNSUPPORTED: u8 = 3;
pub const MAV_RESULT_FAILED: u8 = 4;

// Sensor bits for SYS_STATUS
pub const MAV_SYS_STATUS_SENSOR_3D_GYRO: u32 = 0x01;
pub const MAV_SYS_STATUS_SENSOR_3D_ACCEL: u32 = 0x02;
pub const MAV_SYS_STATUS_SENSOR_3D_MAG: u32 = 0x04;
pub const MAV_SYS_STATUS_SENSOR_GPS: u32 = 0x08;
pub const MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE: u32 = 0x10;
pub const MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE: u32 = 0x20;
pub const MAV_SYS_STATUS_AHRS: u32 = 0x200000;
pub const MAV_SYS_STATUS_PREARM_CHECK: u32 = 0x4000000;

/// Stream rate configuration (Hz per stream group).
#[derive(Debug, Clone, Copy)]
pub struct StreamRates {
    pub raw_sensors: u8,    // SRx_RAW_SENS
    pub ext_status: u8,     // SRx_EXT_STAT
    pub rc_channels: u8,    // SRx_RC_CHAN
    pub position: u8,       // SRx_POSITION
    pub extra1: u8,         // SRx_EXTRA1 (attitude)
    pub extra2: u8,         // SRx_EXTRA2 (VFR_HUD)
    pub extra3: u8,         // SRx_EXTRA3
}

impl Default for StreamRates {
    fn default() -> Self {
        Self {
            raw_sensors: 2,
            ext_status: 2,
            rc_channels: 5,
            position: 3,
            extra1: 10,  // attitude at 10Hz
            extra2: 4,   // VFR_HUD at 4Hz
            extra3: 1,
        }
    }
}

/// Sensor health state for SYS_STATUS reporting.
#[derive(Debug, Clone, Copy, Default)]
pub struct SensorHealth {
    pub gyro_present: bool,
    pub gyro_healthy: bool,
    pub accel_present: bool,
    pub accel_healthy: bool,
    pub mag_present: bool,
    pub mag_healthy: bool,
    pub gps_present: bool,
    pub gps_healthy: bool,
    pub baro_present: bool,
    pub baro_healthy: bool,
    pub ahrs_healthy: bool,
    pub prearm_ok: bool,
}

impl SensorHealth {
    pub fn all_healthy() -> Self {
        Self {
            gyro_present: true, gyro_healthy: true,
            accel_present: true, accel_healthy: true,
            mag_present: true, mag_healthy: true,
            gps_present: true, gps_healthy: true,
            baro_present: true, baro_healthy: true,
            ahrs_healthy: true, prearm_ok: true,
        }
    }

    fn present_mask(&self) -> u32 {
        let mut m = 0u32;
        if self.gyro_present { m |= MAV_SYS_STATUS_SENSOR_3D_GYRO; }
        if self.accel_present { m |= MAV_SYS_STATUS_SENSOR_3D_ACCEL; }
        if self.mag_present { m |= MAV_SYS_STATUS_SENSOR_3D_MAG; }
        if self.gps_present { m |= MAV_SYS_STATUS_SENSOR_GPS; }
        if self.baro_present { m |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE; }
        m |= MAV_SYS_STATUS_AHRS;
        m |= MAV_SYS_STATUS_PREARM_CHECK;
        m
    }

    fn health_mask(&self) -> u32 {
        let mut m = 0u32;
        if self.gyro_healthy { m |= MAV_SYS_STATUS_SENSOR_3D_GYRO; }
        if self.accel_healthy { m |= MAV_SYS_STATUS_SENSOR_3D_ACCEL; }
        if self.mag_healthy { m |= MAV_SYS_STATUS_SENSOR_3D_MAG; }
        if self.gps_healthy { m |= MAV_SYS_STATUS_SENSOR_GPS; }
        if self.baro_healthy { m |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE; }
        if self.ahrs_healthy { m |= MAV_SYS_STATUS_AHRS; }
        if self.prearm_ok { m |= MAV_SYS_STATUS_PREARM_CHECK; }
        m
    }
}

/// The MAVLink adapter: translates Meridian messages to/from MAVLink.
pub struct MavlinkAdapter {
    pub ids: MavIds,
    pub stream_rates: StreamRates,
    seq: u8,
    parser: MavParser,
}

impl MavlinkAdapter {
    pub fn new(ids: MavIds) -> Self {
        Self { ids, stream_rates: StreamRates::default(), seq: 0, parser: MavParser::new() }
    }

    fn next_seq(&mut self) -> u8 {
        let s = self.seq;
        self.seq = self.seq.wrapping_add(1);
        s
    }

    /// Build a MAVLink v2 header with auto-incrementing sequence number.
    /// Public so the server module can encode messages the adapter doesn't have
    /// a dedicated method for (e.g. MISSION_REQUEST_INT outbound).
    pub fn make_header_pub(&mut self, msg_id: u32, payload_len: u8) -> MavHeader {
        self.make_header(msg_id, payload_len)
    }

    fn make_header(&mut self, msg_id: u32, payload_len: u8) -> MavHeader {
        MavHeader {
            payload_len,
            incompat_flags: 0,
            compat_flags: 0,
            seq: self.next_seq(),
            system_id: self.ids.system_id,
            component_id: self.ids.component_id,
            msg_id,
        }
    }

    // ─── Outbound: Meridian → MAVLink ───

    /// Encode a heartbeat with correct base_mode and custom_mode.
    /// Source: GCS_Common.cpp send_heartbeat()
    ///
    /// base_mode encodes: CUSTOM_MODE_ENABLED | flags for stabilize/guided/auto/armed
    /// custom_mode: ArduPilot mode number (0=Stabilize, 2=AltHold, 5=Loiter, 6=RTL, etc.)
    /// system_status: STANDBY when disarmed, ACTIVE when armed, CRITICAL/EMERGENCY on failsafe
    pub fn encode_heartbeat(
        &mut self,
        armed: bool,
        custom_mode: u32,
        mav_type: u8,
        system_status: u8,
        buf: &mut [u8],
    ) -> usize {
        let mut base_mode: u8 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        if armed { base_mode |= MAV_MODE_FLAG_SAFETY_ARMED; }
        // Set mode flags based on custom_mode
        match custom_mode {
            0 | 2 => { base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED; } // Stabilize, AltHold
            4 | 5 | 7 => { base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED; } // Guided, Loiter, Circle
            3 | 6 | 9 => { base_mode |= MAV_MODE_FLAG_AUTO_ENABLED; } // Auto, RTL, Land
            _ => {}
        }

        let mut payload = [0u8; 9];
        payload[0..4].copy_from_slice(&custom_mode.to_le_bytes());
        payload[4] = mav_type;
        payload[5] = MAV_AUTOPILOT_ARDUPILOTMEGA;
        payload[6] = base_mode;
        payload[7] = system_status;
        payload[8] = 3; // mavlink_version
        let header = self.make_header(MSG_HEARTBEAT, 9);
        encode_frame(&header, &payload, buf)
    }

    /// Encode attitude into MAVLink ATTITUDE message (msg 30).
    pub fn encode_attitude(
        &mut self, roll: f32, pitch: f32, yaw: f32,
        rollspeed: f32, pitchspeed: f32, yawspeed: f32,
        time_boot_ms: u32, buf: &mut [u8],
    ) -> usize {
        let mut payload = [0u8; 28]; // 4+6*4 = 28 bytes
        payload[0..4].copy_from_slice(&time_boot_ms.to_le_bytes());
        payload[4..8].copy_from_slice(&roll.to_le_bytes());
        payload[8..12].copy_from_slice(&pitch.to_le_bytes());
        payload[12..16].copy_from_slice(&yaw.to_le_bytes());
        payload[16..20].copy_from_slice(&rollspeed.to_le_bytes());
        payload[20..24].copy_from_slice(&pitchspeed.to_le_bytes());
        payload[24..28].copy_from_slice(&yawspeed.to_le_bytes());
        let header = self.make_header(MSG_ATTITUDE, 28);
        encode_frame(&header, &payload, buf)
    }

    /// Encode position into MAVLink GLOBAL_POSITION_INT (msg 33).
    pub fn encode_position(
        &mut self, lat_e7: i32, lon_e7: i32, alt_mm: i32,
        relative_alt_mm: i32, vx: i16, vy: i16, vz: i16, heading: u16,
        time_boot_ms: u32, buf: &mut [u8],
    ) -> usize {
        let mut payload = [0u8; 28];
        payload[0..4].copy_from_slice(&time_boot_ms.to_le_bytes());
        payload[4..8].copy_from_slice(&lat_e7.to_le_bytes());
        payload[8..12].copy_from_slice(&lon_e7.to_le_bytes());
        payload[12..16].copy_from_slice(&alt_mm.to_le_bytes());
        payload[16..20].copy_from_slice(&relative_alt_mm.to_le_bytes());
        payload[20..22].copy_from_slice(&vx.to_le_bytes());
        payload[22..24].copy_from_slice(&vy.to_le_bytes());
        payload[24..26].copy_from_slice(&vz.to_le_bytes());
        payload[26..28].copy_from_slice(&heading.to_le_bytes());
        let header = self.make_header(MSG_GLOBAL_POSITION_INT, 28);
        encode_frame(&header, &payload, buf)
    }

    /// Encode SYS_STATUS with sensor health bitmask + battery.
    /// Source: GCS_Common.cpp send_sys_status()
    pub fn encode_sys_status(
        &mut self,
        sensors: &SensorHealth,
        voltage_mv: u16,
        current_ca: i16,
        remaining_pct: i8,
        load_pct: u16,
        buf: &mut [u8],
    ) -> usize {
        let mut payload = [0u8; 31];
        let present = sensors.present_mask();
        let enabled = present; // enabled = present for now
        let health = sensors.health_mask();
        payload[0..4].copy_from_slice(&present.to_le_bytes());
        payload[4..8].copy_from_slice(&enabled.to_le_bytes());
        payload[8..12].copy_from_slice(&health.to_le_bytes());
        payload[12..14].copy_from_slice(&load_pct.to_le_bytes());
        payload[14..16].copy_from_slice(&voltage_mv.to_le_bytes());
        payload[16..18].copy_from_slice(&current_ca.to_le_bytes());
        payload[30] = remaining_pct as u8;
        let header = self.make_header(MSG_SYS_STATUS, 31);
        encode_frame(&header, &payload, buf)
    }

    /// Encode AUTOPILOT_VERSION response.
    /// Source: GCS_Common.cpp send_autopilot_version()
    /// QGC requires this to confirm MAVLink v2 capability.
    pub fn encode_autopilot_version(&mut self, buf: &mut [u8]) -> usize {
        let mut payload = [0u8; 78];
        // capabilities: u64 — set MAV_PROTOCOL_CAPABILITY_MAVLINK2 (bit 8)
        //   + PARAM_FLOAT (bit 1) + MISSION_INT (bit 13) + COMMAND_INT (bit 7)
        let capabilities: u64 = (1 << 1) | (1 << 7) | (1 << 8) | (1 << 13);
        payload[0..8].copy_from_slice(&capabilities.to_le_bytes());
        // flight_sw_version: u32 — encode as major.minor.patch.type
        let sw_version: u32 = (0 << 24) | (1 << 16) | (0 << 8) | 0; // 0.1.0-dev
        payload[8..12].copy_from_slice(&sw_version.to_le_bytes());
        // middleware_sw_version, os_sw_version, board_version: leave 0
        // uid: 8 bytes at offset 36
        let header = self.make_header(MSG_AUTOPILOT_VERSION, 78);
        encode_frame(&header, &payload, buf)
    }

    /// Encode PARAM_VALUE message.
    /// Source: GCS_Param.cpp handle_param_request_read/list
    pub fn encode_param_value(
        &mut self,
        name: &str,
        value: f32,
        param_type: u8,
        param_count: u16,
        param_index: u16,
        buf: &mut [u8],
    ) -> usize {
        let mut payload = [0u8; 25];
        payload[0..4].copy_from_slice(&value.to_le_bytes());
        payload[4..6].copy_from_slice(&param_count.to_le_bytes());
        payload[6..8].copy_from_slice(&param_index.to_le_bytes());
        // param_id: 16 bytes null-terminated
        let bytes = name.as_bytes();
        let len = bytes.len().min(16);
        payload[8..8+len].copy_from_slice(&bytes[..len]);
        payload[24] = param_type; // MAV_PARAM_TYPE_REAL32 = 9
        let header = self.make_header(MSG_PARAM_VALUE, 25);
        encode_frame(&header, &payload, buf)
    }

    /// Encode STATUSTEXT message (severity + 50-char text).
    pub fn encode_statustext(
        &mut self,
        severity: u8, // MAV_SEVERITY: 0=EMERGENCY..7=DEBUG
        text: &str,
        buf: &mut [u8],
    ) -> usize {
        let mut payload = [0u8; 54]; // severity(1) + text(50) + id(2) + chunk_seq(1)
        payload[0] = severity;
        let bytes = text.as_bytes();
        let len = bytes.len().min(50);
        payload[1..1+len].copy_from_slice(&bytes[..len]);
        let header = self.make_header(MSG_STATUSTEXT, 54);
        encode_frame(&header, &payload, buf)
    }

    /// Encode HOME_POSITION message.
    pub fn encode_home_position(
        &mut self,
        lat_e7: i32, lon_e7: i32, alt_mm: i32,
        buf: &mut [u8],
    ) -> usize {
        let mut payload = [0u8; 60];
        payload[0..4].copy_from_slice(&lat_e7.to_le_bytes());
        payload[4..8].copy_from_slice(&lon_e7.to_le_bytes());
        payload[8..12].copy_from_slice(&alt_mm.to_le_bytes());
        // local x/y/z = 0, quaternion = identity [1,0,0,0]
        payload[24..28].copy_from_slice(&1.0f32.to_le_bytes());
        let header = self.make_header(MSG_HOME_POSITION, 60);
        encode_frame(&header, &payload, buf)
    }

    /// Encode MISSION_COUNT for mission download protocol.
    pub fn encode_mission_count(&mut self, count: u16, buf: &mut [u8]) -> usize {
        let mut payload = [0u8; 4];
        payload[0..2].copy_from_slice(&count.to_le_bytes());
        payload[2] = 0; // target_system (GCS)
        payload[3] = 0; // target_component
        let header = self.make_header(MSG_MISSION_COUNT, 4);
        encode_frame(&header, &payload, buf)
    }

    /// Encode MISSION_ITEM_INT.
    pub fn encode_mission_item_int(
        &mut self,
        seq: u16,
        frame: u8,
        command: u16,
        current: u8,
        autocontinue: u8,
        params: [f32; 4],
        x: i32, y: i32, z: f32,
        buf: &mut [u8],
    ) -> usize {
        let mut payload = [0u8; 37];
        payload[0..4].copy_from_slice(&params[0].to_le_bytes());
        payload[4..8].copy_from_slice(&params[1].to_le_bytes());
        payload[8..12].copy_from_slice(&params[2].to_le_bytes());
        payload[12..16].copy_from_slice(&params[3].to_le_bytes());
        payload[16..20].copy_from_slice(&x.to_le_bytes());
        payload[20..24].copy_from_slice(&y.to_le_bytes());
        payload[24..28].copy_from_slice(&z.to_le_bytes());
        payload[28..30].copy_from_slice(&seq.to_le_bytes());
        payload[30..32].copy_from_slice(&command.to_le_bytes());
        payload[32] = 0; // target_system
        payload[33] = 0; // target_component
        payload[34] = frame;
        payload[35] = current;
        payload[36] = autocontinue;
        let header = self.make_header(MSG_MISSION_ITEM_INT, 37);
        encode_frame(&header, &payload, buf)
    }

    /// Encode MISSION_ACK.
    pub fn encode_mission_ack(&mut self, ack_type: u8, buf: &mut [u8]) -> usize {
        let mut payload = [0u8; 3];
        payload[0] = 0; // target_system
        payload[1] = 0; // target_component
        payload[2] = ack_type; // MAV_MISSION_RESULT
        let header = self.make_header(MSG_MISSION_ACK, 3);
        encode_frame(&header, &payload, buf)
    }

    /// Encode VFR HUD.
    pub fn encode_vfr_hud(
        &mut self, airspeed: f32, groundspeed: f32, heading: i16,
        throttle: u16, alt: f32, climb: f32, buf: &mut [u8],
    ) -> usize {
        let mut payload = [0u8; 20];
        payload[0..4].copy_from_slice(&airspeed.to_le_bytes());
        payload[4..8].copy_from_slice(&groundspeed.to_le_bytes());
        payload[8..12].copy_from_slice(&alt.to_le_bytes());
        payload[12..16].copy_from_slice(&climb.to_le_bytes());
        payload[16..18].copy_from_slice(&heading.to_le_bytes());
        payload[18..20].copy_from_slice(&throttle.to_le_bytes());
        let header = self.make_header(MSG_VFR_HUD, 20);
        encode_frame(&header, &payload, buf)
    }

    /// Encode COMMAND_ACK.
    pub fn encode_command_ack(
        &mut self, command: u16, result: u8, buf: &mut [u8],
    ) -> usize {
        let mut payload = [0u8; 3];
        payload[0..2].copy_from_slice(&command.to_le_bytes());
        payload[2] = result;
        let header = self.make_header(MSG_COMMAND_ACK, 3);
        encode_frame(&header, &payload, buf)
    }

    /// Encode TIMESYNC response (msg 111).
    /// Echoes back tc1=ts1=server_time_ns so the GCS can compute round-trip time.
    /// Source: MAVLink TIMESYNC protocol — GCS sends tc1=0, ts1=remote_time.
    /// We respond with tc1=their_ts1, ts1=our_time_ns.
    pub fn encode_timesync(
        &mut self,
        tc1: i64,
        ts1: i64,
        buf: &mut [u8],
    ) -> usize {
        let mut payload = [0u8; 16];
        payload[0..8].copy_from_slice(&tc1.to_le_bytes());
        payload[8..16].copy_from_slice(&ts1.to_le_bytes());
        let header = self.make_header(MSG_TIMESYNC, 16);
        encode_frame(&header, &payload, buf)
    }

    /// Encode MISSION_CURRENT (msg 42).
    /// Reports the current active waypoint sequence number.
    pub fn encode_mission_current(
        &mut self,
        seq: u16,
        buf: &mut [u8],
    ) -> usize {
        let mut payload = [0u8; 2];
        payload[0..2].copy_from_slice(&seq.to_le_bytes());
        let header = self.make_header(MSG_MISSION_CURRENT, 2);
        encode_frame(&header, &payload, buf)
    }

    // ─── Inbound: MAVLink → Meridian ───

    /// Feed received bytes through the parser. Returns parsed commands.
    pub fn feed_bytes(&mut self, data: &[u8]) -> heapless::Vec<InboundCommand, 4> {
        let mut cmds = heapless::Vec::new();
        for &byte in data {
            if let Some((header, payload)) = self.parser.feed(byte) {
                let msg_id = header.msg_id;
                let mut pay_copy = [0u8; MAX_PAYLOAD];
                let plen = payload.len().min(MAX_PAYLOAD);
                pay_copy[..plen].copy_from_slice(&payload[..plen]);
                if let Some(cmd) = self.parse_inbound(msg_id, &pay_copy[..plen]) {
                    let _ = cmds.push(cmd);
                }
            }
        }
        cmds
    }

    fn parse_inbound(&self, msg_id: u32, payload: &[u8]) -> Option<InboundCommand> {
        match msg_id {
            MSG_COMMAND_LONG => self.parse_command_long(payload),
            MSG_COMMAND_INT => self.parse_command_int(payload),
            MSG_PARAM_SET => self.parse_param_set(payload),
            MSG_PARAM_REQUEST_LIST => Some(InboundCommand::ParamRequestList),
            MSG_PARAM_REQUEST_READ => self.parse_param_request_read(payload),
            MSG_REQUEST_DATA_STREAM => self.parse_stream_request(payload),
            MSG_MISSION_REQUEST_LIST => Some(InboundCommand::MissionRequestList),
            MSG_MISSION_COUNT => self.parse_mission_count(payload),
            MSG_MISSION_ITEM_INT => self.parse_mission_item_int(payload),
            MSG_MISSION_REQUEST_INT => self.parse_mission_request_int(payload),
            MSG_MISSION_CLEAR_ALL => Some(InboundCommand::MissionClearAll),
            MSG_HEARTBEAT => Some(InboundCommand::GcsHeartbeat),
            MSG_TIMESYNC => self.parse_timesync(payload),
            _ => None,
        }
    }

    fn parse_command_long(&self, payload: &[u8]) -> Option<InboundCommand> {
        if payload.len() < 33 { return None; }
        let command = u16::from_le_bytes([payload[28], payload[29]]);
        match command {
            400 => { // MAV_CMD_COMPONENT_ARM_DISARM
                let arm = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
                Some(InboundCommand::Command {
                    command,
                    result: if arm > 0.5 {
                        CommandAction::Arm
                    } else {
                        CommandAction::Disarm
                    },
                })
            }
            22 => { // MAV_CMD_NAV_TAKEOFF
                let alt = f32::from_le_bytes([payload[24], payload[25], payload[26], payload[27]]);
                Some(InboundCommand::Command {
                    command,
                    result: CommandAction::Takeoff(alt),
                })
            }
            20 => Some(InboundCommand::Command { command, result: CommandAction::Rtl }),
            21 => Some(InboundCommand::Command { command, result: CommandAction::Land }),
            176 => { // MAV_CMD_DO_SET_MODE
                let mode = f32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]) as u32;
                Some(InboundCommand::Command { command, result: CommandAction::SetMode(mode) })
            }
            512 => { // MAV_CMD_REQUEST_MESSAGE
                let msg_id = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]) as u32;
                Some(InboundCommand::Command { command, result: CommandAction::RequestMessage(msg_id) })
            }
            519 => { // MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
                Some(InboundCommand::Command { command, result: CommandAction::RequestAutopilotVersion })
            }
            511 => { // MAV_CMD_SET_MESSAGE_INTERVAL
                let msg_id = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]) as u32;
                let interval_us = f32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]) as i64;
                Some(InboundCommand::Command {
                    command,
                    result: CommandAction::SetMessageInterval { msg_id, interval_us },
                })
            }
            209 => { // MAV_CMD_DO_MOTOR_TEST
                let motor_index = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]) as u8;
                let throttle_type = f32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]) as u8;
                let throttle_value = f32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]);
                let timeout_s = f32::from_le_bytes([payload[12], payload[13], payload[14], payload[15]]);
                let motor_count = f32::from_le_bytes([payload[16], payload[17], payload[18], payload[19]]) as u8;
                Some(InboundCommand::Command {
                    command,
                    result: CommandAction::MotorTest {
                        motor_index,
                        throttle_type,
                        throttle_value,
                        timeout_s,
                        motor_count,
                    },
                })
            }
            _ => Some(InboundCommand::Command { command, result: CommandAction::Unknown }),
        }
    }

    fn parse_command_int(&self, payload: &[u8]) -> Option<InboundCommand> {
        if payload.len() < 35 { return None; }
        let command = u16::from_le_bytes([payload[28], payload[29]]);
        // Reuse same command handling
        match command {
            400 => {
                let arm = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
                Some(InboundCommand::Command {
                    command,
                    result: if arm > 0.5 { CommandAction::Arm } else { CommandAction::Disarm },
                })
            }
            176 => {
                let mode = f32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]) as u32;
                Some(InboundCommand::Command { command, result: CommandAction::SetMode(mode) })
            }
            _ => Some(InboundCommand::Command { command, result: CommandAction::Unknown }),
        }
    }

    fn parse_param_set(&self, payload: &[u8]) -> Option<InboundCommand> {
        if payload.len() < 23 { return None; }
        let value = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
        let mut name = [0u8; 16];
        name.copy_from_slice(&payload[6..22]);
        let name_len = name.iter().position(|&b| b == 0).unwrap_or(16);
        let mut name_str = heapless::String::<16>::new();
        for &b in &name[..name_len] {
            let _ = name_str.push(b as char);
        }
        Some(InboundCommand::ParamSet { name: name_str, value })
    }

    fn parse_param_request_read(&self, payload: &[u8]) -> Option<InboundCommand> {
        if payload.len() < 20 { return None; }
        let index = i16::from_le_bytes([payload[0], payload[1]]);
        let mut name = [0u8; 16];
        name.copy_from_slice(&payload[4..20]);
        let name_len = name.iter().position(|&b| b == 0).unwrap_or(16);
        let mut name_str = heapless::String::<16>::new();
        for &b in &name[..name_len] {
            let _ = name_str.push(b as char);
        }
        Some(InboundCommand::ParamRequestRead { name: name_str, index })
    }

    fn parse_stream_request(&self, payload: &[u8]) -> Option<InboundCommand> {
        if payload.len() < 6 { return None; }
        let rate = u16::from_le_bytes([payload[0], payload[1]]);
        let stream_id = payload[4];
        let start_stop = payload[5];
        Some(InboundCommand::StreamRequest { stream_id, rate, start: start_stop != 0 })
    }

    fn parse_mission_count(&self, payload: &[u8]) -> Option<InboundCommand> {
        if payload.len() < 4 { return None; }
        let count = u16::from_le_bytes([payload[0], payload[1]]);
        Some(InboundCommand::MissionCount(count))
    }

    fn parse_mission_item_int(&self, payload: &[u8]) -> Option<InboundCommand> {
        if payload.len() < 37 { return None; }
        let seq = u16::from_le_bytes([payload[28], payload[29]]);
        let command = u16::from_le_bytes([payload[30], payload[31]]);
        let frame = payload[34];
        let params = [
            f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]),
            f32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]),
            f32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]),
            f32::from_le_bytes([payload[12], payload[13], payload[14], payload[15]]),
        ];
        let x = i32::from_le_bytes([payload[16], payload[17], payload[18], payload[19]]);
        let y = i32::from_le_bytes([payload[20], payload[21], payload[22], payload[23]]);
        let z = f32::from_le_bytes([payload[24], payload[25], payload[26], payload[27]]);
        Some(InboundCommand::MissionItemInt {
            seq, frame, command, params, x, y, z,
        })
    }

    fn parse_mission_request_int(&self, payload: &[u8]) -> Option<InboundCommand> {
        if payload.len() < 4 { return None; }
        let seq = u16::from_le_bytes([payload[0], payload[1]]);
        Some(InboundCommand::MissionRequestInt(seq))
    }

    fn parse_timesync(&self, payload: &[u8]) -> Option<InboundCommand> {
        if payload.len() < 16 { return None; }
        let tc1 = i64::from_le_bytes([
            payload[0], payload[1], payload[2], payload[3],
            payload[4], payload[5], payload[6], payload[7],
        ]);
        let ts1 = i64::from_le_bytes([
            payload[8], payload[9], payload[10], payload[11],
            payload[12], payload[13], payload[14], payload[15],
        ]);
        Some(InboundCommand::Timesync { tc1, ts1 })
    }
}

/// Action extracted from a MAVLink command.
#[derive(Debug, Clone, Copy)]
pub enum CommandAction {
    Arm,
    Disarm,
    Takeoff(f32),
    Land,
    Rtl,
    SetMode(u32),
    RequestMessage(u32),
    RequestAutopilotVersion,
    /// MAV_CMD_DO_MOTOR_TEST (209): motor_index, throttle_type, throttle_value, timeout, motor_count
    MotorTest {
        motor_index: u8,
        throttle_type: u8,
        throttle_value: f32,
        timeout_s: f32,
        motor_count: u8,
    },
    /// MAV_CMD_SET_MESSAGE_INTERVAL (511): configure per-message streaming rate.
    SetMessageInterval {
        msg_id: u32,
        interval_us: i64,
    },
    Unknown,
}

/// Commands received from a MAVLink GCS.
#[derive(Debug, Clone)]
pub enum InboundCommand {
    /// A command (COMMAND_LONG or COMMAND_INT) — always ACK back.
    Command { command: u16, result: CommandAction },
    ParamSet { name: heapless::String<16>, value: f32 },
    ParamRequestList,
    ParamRequestRead { name: heapless::String<16>, index: i16 },
    StreamRequest { stream_id: u8, rate: u16, start: bool },
    MissionRequestList,
    MissionCount(u16),
    MissionItemInt {
        seq: u16, frame: u8, command: u16,
        params: [f32; 4], x: i32, y: i32, z: f32,
    },
    MissionRequestInt(u16),
    MissionClearAll,
    GcsHeartbeat,
    /// TIMESYNC (msg 111): tc1 and ts1 timestamps for round-trip time computation.
    Timesync { tc1: i64, ts1: i64 },
}

impl InboundCommand {
    /// Get the command ID if this is a Command variant (for ACK).
    pub fn command_id(&self) -> Option<u16> {
        match self {
            InboundCommand::Command { command, .. } => Some(*command),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_heartbeat_roundtrip() {
        let mut adapter = MavlinkAdapter::new(MavIds::default());
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let len = adapter.encode_heartbeat(
            true, 0, MAV_TYPE_QUADROTOR, MAV_STATE_ACTIVE, &mut buf);
        assert!(len > 0);

        let (hdr, payload) = parse_frame(&buf[..len]).unwrap();
        assert_eq!(hdr.msg_id, MSG_HEARTBEAT);
        assert_eq!(hdr.system_id, 1);
        assert_eq!(payload.len(), 9);
        assert_eq!(payload[6] & 0x80, 0x80); // ARMED bit
        assert_eq!(payload[4], MAV_TYPE_QUADROTOR);
        assert_eq!(payload[5], MAV_AUTOPILOT_ARDUPILOTMEGA);
        assert_eq!(payload[7], MAV_STATE_ACTIVE);
    }

    #[test]
    fn test_heartbeat_base_mode_flags() {
        let mut adapter = MavlinkAdapter::new(MavIds::default());
        let mut buf = [0u8; MAX_FRAME_SIZE];
        // Auto mode (3) should set AUTO flag
        adapter.encode_heartbeat(true, 3, MAV_TYPE_QUADROTOR, MAV_STATE_ACTIVE, &mut buf);
        let (_, payload) = parse_frame(&buf[..]).unwrap();
        let base_mode = payload[6];
        assert!(base_mode & MAV_MODE_FLAG_AUTO_ENABLED != 0, "Auto mode should set AUTO flag");
        assert!(base_mode & MAV_MODE_FLAG_SAFETY_ARMED != 0, "Should be armed");

        // Stabilize mode (0) should set STABILIZE flag
        let len = adapter.encode_heartbeat(false, 0, MAV_TYPE_QUADROTOR, MAV_STATE_STANDBY, &mut buf);
        let (_, payload) = parse_frame(&buf[..len]).unwrap();
        assert!(payload[6] & MAV_MODE_FLAG_STABILIZE_ENABLED != 0);
        assert!(payload[6] & MAV_MODE_FLAG_SAFETY_ARMED == 0, "Should be disarmed");
    }

    #[test]
    fn test_attitude_encoding() {
        let mut adapter = MavlinkAdapter::new(MavIds::default());
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let len = adapter.encode_attitude(
            0.1, -0.05, 1.5, 0.01, -0.02, 0.0, 5000, &mut buf);
        assert!(len > 0);

        let (hdr, payload) = parse_frame(&buf[..len]).unwrap();
        assert_eq!(hdr.msg_id, MSG_ATTITUDE);
        let roll = f32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]);
        assert!((roll - 0.1).abs() < 1e-6);
    }

    #[test]
    fn test_command_long_arm_with_ack() {
        let mut adapter = MavlinkAdapter::new(MavIds::default());
        let mut payload = [0u8; 33];
        payload[0..4].copy_from_slice(&1.0f32.to_le_bytes());
        payload[28..30].copy_from_slice(&400u16.to_le_bytes());
        payload[30] = 1;
        payload[31] = 1;

        let header = MavHeader {
            payload_len: 33, incompat_flags: 0, compat_flags: 0,
            seq: 0, system_id: 255, component_id: 190,
            msg_id: MSG_COMMAND_LONG,
        };
        let mut frame = [0u8; MAX_FRAME_SIZE];
        let len = encode_frame(&header, &payload, &mut frame);

        let cmds = adapter.feed_bytes(&frame[..len]);
        assert_eq!(cmds.len(), 1);
        // Should be a Command with Arm action
        match &cmds[0] {
            InboundCommand::Command { command, result: CommandAction::Arm } => {
                assert_eq!(*command, 400);
                // Verify we can generate ACK
                let mut ack_buf = [0u8; MAX_FRAME_SIZE];
                let ack_len = adapter.encode_command_ack(400, MAV_RESULT_ACCEPTED, &mut ack_buf);
                assert!(ack_len > 0);
            }
            other => panic!("Expected Command/Arm, got {:?}", other),
        }
    }

    #[test]
    fn test_autopilot_version() {
        let mut adapter = MavlinkAdapter::new(MavIds::default());
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let len = adapter.encode_autopilot_version(&mut buf);
        assert!(len > 0);
        let (hdr, payload) = parse_frame(&buf[..len]).unwrap();
        assert_eq!(hdr.msg_id, MSG_AUTOPILOT_VERSION);
        // Check MAVLINK2 capability bit
        let capabilities = u64::from_le_bytes([
            payload[0], payload[1], payload[2], payload[3],
            payload[4], payload[5], payload[6], payload[7],
        ]);
        assert!(capabilities & (1 << 8) != 0, "Should have MAVLINK2 capability");
    }

    #[test]
    fn test_sys_status_sensor_health() {
        let mut adapter = MavlinkAdapter::new(MavIds::default());
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let sensors = SensorHealth::all_healthy();
        let len = adapter.encode_sys_status(&sensors, 12600, 500, 85, 200, &mut buf);
        assert!(len > 0);
        let (hdr, payload) = parse_frame(&buf[..len]).unwrap();
        assert_eq!(hdr.msg_id, MSG_SYS_STATUS);
        let present = u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
        let health = u32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]);
        assert!(present & MAV_SYS_STATUS_SENSOR_3D_GYRO != 0);
        assert!(health & MAV_SYS_STATUS_SENSOR_3D_GYRO != 0);
    }

    #[test]
    fn test_param_value_encoding() {
        let mut adapter = MavlinkAdapter::new(MavIds::default());
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let len = adapter.encode_param_value("ATC_RAT_RLL_P", 0.135, 9, 128, 0, &mut buf);
        assert!(len > 0);
        let (hdr, payload) = parse_frame(&buf[..len]).unwrap();
        assert_eq!(hdr.msg_id, MSG_PARAM_VALUE);
        let val = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
        assert!((val - 0.135).abs() < 1e-6);
    }

    #[test]
    fn test_statustext_encoding() {
        let mut adapter = MavlinkAdapter::new(MavIds::default());
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let len = adapter.encode_statustext(4, "PreArm: GPS not healthy", &mut buf);
        assert!(len > 0);
        let (hdr, payload) = parse_frame(&buf[..len]).unwrap();
        assert_eq!(hdr.msg_id, MSG_STATUSTEXT);
        assert_eq!(payload[0], 4); // WARNING severity
    }

    #[test]
    fn test_position_encoding() {
        let mut adapter = MavlinkAdapter::new(MavIds::default());
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let len = adapter.encode_position(
            350000000, -1200000000, 50000, 10000,
            100, -50, 20, 27000, 10000, &mut buf);
        assert!(len > 0);
        let (hdr, _) = parse_frame(&buf[..len]).unwrap();
        assert_eq!(hdr.msg_id, MSG_GLOBAL_POSITION_INT);
    }

    #[test]
    fn test_mission_protocol_messages() {
        let mut adapter = MavlinkAdapter::new(MavIds::default());
        let mut buf = [0u8; MAX_FRAME_SIZE];

        assert!(adapter.encode_mission_count(5, &mut buf) > 0);
        assert!(adapter.encode_mission_item_int(
            0, 3, 16, 1, 1, [0.0; 4], 350000000, -1200000000, 50.0, &mut buf) > 0);
        assert!(adapter.encode_mission_ack(0, &mut buf) > 0);
    }

    #[test]
    fn test_all_telemetry_encode() {
        let mut adapter = MavlinkAdapter::new(MavIds::default());
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let sensors = SensorHealth::all_healthy();

        assert!(adapter.encode_heartbeat(true, 5, MAV_TYPE_QUADROTOR, MAV_STATE_ACTIVE, &mut buf) > 0);
        assert!(adapter.encode_attitude(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, &mut buf) > 0);
        assert!(adapter.encode_position(0, 0, 0, 0, 0, 0, 0, 0, 0, &mut buf) > 0);
        assert!(adapter.encode_sys_status(&sensors, 12600, 500, 85, 200, &mut buf) > 0);
        assert!(adapter.encode_vfr_hud(18.0, 17.5, 270, 50, 45.0, 0.5, &mut buf) > 0);
        assert!(adapter.encode_command_ack(400, 0, &mut buf) > 0);
        assert!(adapter.encode_autopilot_version(&mut buf) > 0);
        assert!(adapter.encode_param_value("TEST_P", 1.0, 9, 1, 0, &mut buf) > 0);
        assert!(adapter.encode_statustext(6, "All systems nominal", &mut buf) > 0);
        assert!(adapter.encode_home_position(350000000, -1200000000, 50000, &mut buf) > 0);
    }

    #[test]
    fn test_unknown_command_still_parsed() {
        let mut adapter = MavlinkAdapter::new(MavIds::default());
        let mut payload = [0u8; 33];
        payload[28..30].copy_from_slice(&999u16.to_le_bytes()); // unknown command
        let header = MavHeader {
            payload_len: 33, incompat_flags: 0, compat_flags: 0,
            seq: 0, system_id: 255, component_id: 190,
            msg_id: MSG_COMMAND_LONG,
        };
        let mut frame = [0u8; MAX_FRAME_SIZE];
        let len = encode_frame(&header, &payload, &mut frame);
        let cmds = adapter.feed_bytes(&frame[..len]);
        assert_eq!(cmds.len(), 1);
        // Unknown commands should still be parsed so we can ACK them
        match &cmds[0] {
            InboundCommand::Command { command, result: CommandAction::Unknown } => {
                assert_eq!(*command, 999);
            }
            other => panic!("Expected Command/Unknown, got {:?}", other),
        }
    }
}

//! MAVLink GCS server — full QGroundControl connection state machine.
//!
//! Manages stream rates, parameter streaming, mission upload/download,
//! GCS heartbeat tracking, and statustext queuing. Runs on the FC alongside
//! the MavlinkAdapter (which handles raw encode/decode).

use crate::adapter::*;
use crate::v2::*;

/// Number of stream groups (ArduPilot SRx_ system).
const NUM_STREAM_GROUPS: usize = 7;

/// Default stream intervals in milliseconds (0 = disabled).
/// Index: 0=RAW_SENSORS, 1=EXT_STATUS, 2=RC_CHANNELS, 3=POSITION,
///        4=EXTRA1(attitude), 5=EXTRA2(VFR_HUD), 6=EXTRA3
const DEFAULT_STREAM_INTERVALS_MS: [u32; NUM_STREAM_GROUPS] = [
    500,  // 0: RAW_SENSORS  — 2 Hz
    500,  // 1: EXT_STATUS   — 2 Hz
    200,  // 2: RC_CHANNELS  — 5 Hz (unused for now)
    333,  // 3: POSITION     — 3 Hz
    100,  // 4: EXTRA1       — 10 Hz
    250,  // 5: EXTRA2       — 4 Hz
    1000, // 6: EXTRA3       — 1 Hz
];

/// Heartbeat is always 1 Hz.
const HEARTBEAT_INTERVAL_MS: u32 = 1000;

/// GCS heartbeat timeout (3.5 seconds).
const GCS_HEARTBEAT_TIMEOUT_MS: u64 = 3500;

/// Sentinel for "never sent" timestamps — ensures first update always fires.
const NEVER_SENT: u64 = u64::MAX;

/// Mission upload timeout (2 seconds without new item).
const MISSION_UPLOAD_TIMEOUT_MS: u64 = 2000;

/// Minimum interval between statustext messages (200 ms).
const STATUSTEXT_MIN_INTERVAL_MS: u64 = 200;

/// Maximum parameter sends per update (bandwidth limiting: ~30% of link).
const MAX_PARAMS_PER_UPDATE: u16 = 5;

/// Maximum mission items we can store.
const MAX_MISSION_ITEMS: usize = 128;

// ─── Vehicle State ───

/// Vehicle state snapshot — the server reads this to generate telemetry.
#[derive(Debug, Clone)]
pub struct VehicleState {
    pub armed: bool,
    pub custom_mode: u32,
    pub mav_type: u8,
    pub system_status: u8,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub rollspeed: f32,
    pub pitchspeed: f32,
    pub yawspeed: f32,
    pub lat_e7: i32,
    pub lon_e7: i32,
    pub alt_mm: i32,
    pub relative_alt_mm: i32,
    pub vx: i16,
    pub vy: i16,
    pub vz: i16,
    pub heading: u16,
    pub airspeed: f32,
    pub groundspeed: f32,
    pub alt: f32,
    pub climb: f32,
    pub throttle: u16,
    pub voltage_mv: u16,
    pub current_ca: i16,
    pub remaining_pct: i8,
    pub sensor_health: SensorHealth,
    pub boot_time_ms: u32,
    pub home_lat_e7: i32,
    pub home_lon_e7: i32,
    pub home_alt_mm: i32,
    /// Current active waypoint sequence number (for MISSION_CURRENT).
    pub mission_current_seq: u16,
}

impl Default for VehicleState {
    fn default() -> Self {
        Self {
            armed: false,
            custom_mode: 0,
            mav_type: MAV_TYPE_QUADROTOR,
            system_status: MAV_STATE_STANDBY,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            rollspeed: 0.0,
            pitchspeed: 0.0,
            yawspeed: 0.0,
            lat_e7: 0,
            lon_e7: 0,
            alt_mm: 0,
            relative_alt_mm: 0,
            vx: 0,
            vy: 0,
            vz: 0,
            heading: 0,
            airspeed: 0.0,
            groundspeed: 0.0,
            alt: 0.0,
            climb: 0.0,
            throttle: 0,
            voltage_mv: 0,
            current_ca: 0,
            remaining_pct: -1,
            sensor_health: SensorHealth::all_healthy(),
            boot_time_ms: 0,
            home_lat_e7: 0,
            home_lon_e7: 0,
            home_alt_mm: 0,
            mission_current_seq: 0,
        }
    }
}

// ─── Server Action ───

/// Actions the caller must execute after handling a GCS message.
#[derive(Debug, Clone)]
pub enum ServerAction {
    Arm,
    Disarm,
    Takeoff(f32),
    Land,
    Rtl,
    SetMode(u32),
    SetParam { name: heapless::String<16>, value: f32 },
    RequestAutopilotVersion,
    RequestMessage(u32),
    MissionUploaded,
    MissionCleared,
    /// MAV_CMD_DO_MOTOR_TEST (209)
    MotorTest {
        motor_index: u8,
        throttle_type: u8,
        throttle_value: f32,
        timeout_s: f32,
        motor_count: u8,
    },
    /// MAV_CMD_SET_MESSAGE_INTERVAL (511): configure per-message rate.
    SetMessageInterval { msg_id: u32, interval_us: i64 },
}

// ─── Stored Mission Item ───

/// A mission item received during upload.
#[derive(Debug, Clone, Copy, Default)]
pub struct StoredMissionItem {
    pub seq: u16,
    pub frame: u8,
    pub command: u16,
    pub params: [f32; 4],
    pub x: i32,
    pub y: i32,
    pub z: f32,
}

// ─── MavlinkServer ───

/// MAVLink GCS server — manages the full QGC connection protocol.
///
/// Handles stream rate scheduling, parameter enumeration, mission
/// upload/download, GCS heartbeat tracking, and statustext queuing.
pub struct MavlinkServer {
    adapter: MavlinkAdapter,

    // Stream rate tracking
    stream_intervals_ms: [u32; NUM_STREAM_GROUPS],
    stream_last_send_ms: [u64; NUM_STREAM_GROUPS],

    // Heartbeat tracking (our outbound 1 Hz heartbeat)
    last_heartbeat_send_ms: u64,

    // Parameter streaming state
    param_send_index: u16,
    param_send_count: u16,
    param_streaming: bool,

    // Mission upload/download state
    mission_upload_count: u16,
    mission_upload_received: u16,
    mission_upload_timeout_ms: u64,
    mission_items: [StoredMissionItem; MAX_MISSION_ITEMS],
    mission_item_count: u16,
    mission_download_seq: u16,

    // GCS heartbeat tracking
    last_gcs_heartbeat_ms: u64,
    gcs_sysid: u8,

    // Statustext queue
    statustext_queue: heapless::Deque<heapless::String<50>, 30>,
    last_statustext_ms: u64,

    // MISSION_CURRENT at 1 Hz
    last_mission_current_ms: u64,

    // Boot time in nanoseconds for TIMESYNC (approximated from ms)
    boot_time_ns: u64,
}

impl MavlinkServer {
    /// Create a new MAVLink GCS server.
    pub fn new(system_id: u8, component_id: u8) -> Self {
        Self {
            adapter: MavlinkAdapter::new(MavIds { system_id, component_id }),
            stream_intervals_ms: DEFAULT_STREAM_INTERVALS_MS,
            stream_last_send_ms: [NEVER_SENT; NUM_STREAM_GROUPS],
            last_heartbeat_send_ms: NEVER_SENT,
            param_send_index: 0,
            param_send_count: 0,
            param_streaming: false,
            mission_upload_count: 0,
            mission_upload_received: 0,
            mission_upload_timeout_ms: 0,
            mission_items: [StoredMissionItem::default(); MAX_MISSION_ITEMS],
            mission_item_count: 0,
            mission_download_seq: 0,
            last_gcs_heartbeat_ms: NEVER_SENT,
            gcs_sysid: 0,
            statustext_queue: heapless::Deque::new(),
            last_statustext_ms: NEVER_SENT,
            last_mission_current_ms: NEVER_SENT,
            boot_time_ns: 0,
        }
    }

    /// Periodic update — send due telemetry, stream parameters, drain statustexts.
    ///
    /// Returns total bytes written to `tx_buf`.
    pub fn update(
        &mut self,
        now_ms: u64,
        vehicle_state: &VehicleState,
        tx_buf: &mut [u8],
    ) -> usize {
        let mut offset = 0;

        // ─── Heartbeat (always 1 Hz) ───
        if self.last_heartbeat_send_ms == NEVER_SENT
            || now_ms.wrapping_sub(self.last_heartbeat_send_ms) >= HEARTBEAT_INTERVAL_MS as u64
        {
            let n = self.send_heartbeat(vehicle_state, &mut tx_buf[offset..]);
            offset += n;
            self.last_heartbeat_send_ms = now_ms;
        }

        // ─── Stream groups ───
        for group in 0..NUM_STREAM_GROUPS {
            let interval = self.stream_intervals_ms[group];
            if interval == 0 {
                continue;
            }
            if self.stream_last_send_ms[group] == NEVER_SENT
                || now_ms.wrapping_sub(self.stream_last_send_ms[group]) >= interval as u64
            {
                let remaining = &mut tx_buf[offset..];
                if remaining.len() < MAX_FRAME_SIZE {
                    break;
                }
                let n = self.send_stream_group(group, now_ms, vehicle_state, remaining);
                offset += n;
                self.stream_last_send_ms[group] = now_ms;
            }
        }

        // ─── Parameter streaming (bandwidth-limited) ───
        if self.param_streaming {
            let mut params_sent = 0u16;
            while self.param_send_index < self.param_send_count
                && params_sent < MAX_PARAMS_PER_UPDATE
            {
                let remaining = &mut tx_buf[offset..];
                if remaining.len() < MAX_FRAME_SIZE {
                    break;
                }
                let n = self.send_param_at_index(self.param_send_index, remaining);
                offset += n;
                self.param_send_index += 1;
                params_sent += 1;
            }
            if self.param_send_index >= self.param_send_count {
                self.param_streaming = false;
            }
        }

        // ─── Statustext queue (max 1 per 200ms) ───
        if !self.statustext_queue.is_empty()
            && (self.last_statustext_ms == NEVER_SENT
                || now_ms.wrapping_sub(self.last_statustext_ms) >= STATUSTEXT_MIN_INTERVAL_MS)
        {
            if let Some(text) = self.statustext_queue.pop_front() {
                let remaining = &mut tx_buf[offset..];
                if remaining.len() >= MAX_FRAME_SIZE {
                    // Severity is packed in first byte as ASCII digit, or default INFO
                    let (severity, msg) = parse_severity_prefix(&text);
                    let n = self.adapter.encode_statustext(severity, msg, remaining);
                    offset += n;
                    self.last_statustext_ms = now_ms;
                }
            }
        }

        // ─── MISSION_CURRENT at 1 Hz ───
        if self.last_mission_current_ms == NEVER_SENT
            || now_ms.wrapping_sub(self.last_mission_current_ms) >= 1000
        {
            let remaining = &mut tx_buf[offset..];
            if remaining.len() >= MAX_FRAME_SIZE {
                let n = self.adapter.encode_mission_current(
                    vehicle_state.mission_current_seq,
                    remaining,
                );
                offset += n;
                self.last_mission_current_ms = now_ms;
            }
        }

        // ─── Mission upload timeout ───
        if self.mission_upload_count > 0
            && self.mission_upload_received < self.mission_upload_count
            && self.mission_upload_timeout_ms > 0
            && now_ms > self.mission_upload_timeout_ms
        {
            // Timed out — reset upload state
            self.mission_upload_count = 0;
            self.mission_upload_received = 0;
        }

        offset
    }

    /// Handle an inbound MAVLink command/request from the GCS.
    ///
    /// Returns (bytes_written, optional action for the caller).
    pub fn handle_message(
        &mut self,
        msg: &InboundCommand,
        now_ms: u64,
        vehicle_state: &VehicleState,
        tx_buf: &mut [u8],
    ) -> (usize, Option<ServerAction>) {
        let _ = vehicle_state; // available for future use
        let mut offset = 0;

        match msg {
            InboundCommand::Command { command, result } => {
                let (ack_result, action) = self.dispatch_command(*command, result);
                let n = self.adapter.encode_command_ack(*command, ack_result, &mut tx_buf[offset..]);
                offset += n;
                (offset, action)
            }

            InboundCommand::ParamRequestList => {
                self.param_send_index = 0;
                self.param_send_count = param_table_count();
                self.param_streaming = true;
                (0, None)
            }

            InboundCommand::ParamRequestRead { name, index } => {
                let n = self.send_param_by_name_or_index(name, *index, &mut tx_buf[offset..]);
                offset += n;
                (offset, None)
            }

            InboundCommand::ParamSet { name, value } => {
                // Send confirmation PARAM_VALUE back
                let n = self.send_param_value_for_set(name, *value, &mut tx_buf[offset..]);
                offset += n;
                (offset, Some(ServerAction::SetParam {
                    name: name.clone(),
                    value: *value,
                }))
            }

            InboundCommand::StreamRequest { stream_id, rate, start } => {
                let group = *stream_id as usize;
                if group < NUM_STREAM_GROUPS {
                    if *start && *rate > 0 {
                        self.stream_intervals_ms[group] = 1000 / (*rate as u32);
                    } else {
                        self.stream_intervals_ms[group] = 0;
                    }
                }
                (0, None)
            }

            InboundCommand::MissionRequestList => {
                // Start mission download — send MISSION_COUNT
                let count = self.mission_item_count;
                let n = self.adapter.encode_mission_count(count, &mut tx_buf[offset..]);
                offset += n;
                self.mission_download_seq = 0;
                (offset, None)
            }

            InboundCommand::MissionCount(count) => {
                // GCS is starting a mission upload
                let count = *count;
                if count == 0 || count as usize > MAX_MISSION_ITEMS {
                    // Send NACK
                    let n = self.adapter.encode_mission_ack(1, &mut tx_buf[offset..]); // 1 = error
                    offset += n;
                    return (offset, None);
                }
                self.mission_upload_count = count;
                self.mission_upload_received = 0;
                self.mission_upload_timeout_ms = now_ms + MISSION_UPLOAD_TIMEOUT_MS;
                // Request first item
                let n = encode_mission_request_int(
                    &mut self.adapter,
                    0,
                    &mut tx_buf[offset..],
                );
                offset += n;
                (offset, None)
            }

            InboundCommand::MissionItemInt { seq, frame, command, params, x, y, z } => {
                let seq = *seq;
                if seq as usize >= MAX_MISSION_ITEMS || seq != self.mission_upload_received {
                    // Out of sequence or overflow — send NACK
                    let n = self.adapter.encode_mission_ack(1, &mut tx_buf[offset..]);
                    offset += n;
                    return (offset, None);
                }

                // Store the item
                self.mission_items[seq as usize] = StoredMissionItem {
                    seq,
                    frame: *frame,
                    command: *command,
                    params: *params,
                    x: *x,
                    y: *y,
                    z: *z,
                };
                self.mission_upload_received = seq + 1;
                self.mission_upload_timeout_ms = now_ms + MISSION_UPLOAD_TIMEOUT_MS;

                if self.mission_upload_received >= self.mission_upload_count {
                    // Upload complete
                    self.mission_item_count = self.mission_upload_count;
                    self.mission_upload_count = 0;
                    let n = self.adapter.encode_mission_ack(0, &mut tx_buf[offset..]); // 0 = accepted
                    offset += n;
                    (offset, Some(ServerAction::MissionUploaded))
                } else {
                    // Request next item
                    let n = encode_mission_request_int(
                        &mut self.adapter,
                        self.mission_upload_received,
                        &mut tx_buf[offset..],
                    );
                    offset += n;
                    (offset, None)
                }
            }

            InboundCommand::MissionRequestInt(seq) => {
                let seq = *seq;
                if (seq as usize) < self.mission_item_count as usize {
                    let item = &self.mission_items[seq as usize];
                    let current = if seq == 0 { 1 } else { 0 };
                    let n = self.adapter.encode_mission_item_int(
                        item.seq, item.frame, item.command,
                        current, 1, // autocontinue
                        item.params, item.x, item.y, item.z,
                        &mut tx_buf[offset..],
                    );
                    offset += n;
                }
                (offset, None)
            }

            InboundCommand::MissionClearAll => {
                self.mission_item_count = 0;
                self.mission_upload_count = 0;
                self.mission_upload_received = 0;
                let n = self.adapter.encode_mission_ack(0, &mut tx_buf[offset..]);
                offset += n;
                (offset, Some(ServerAction::MissionCleared))
            }

            InboundCommand::GcsHeartbeat => {
                self.last_gcs_heartbeat_ms = now_ms;
                (0, None)
            }

            InboundCommand::Timesync { tc1, ts1 } => {
                // TIMESYNC protocol: GCS sends tc1=0, ts1=gcs_time_ns.
                // We respond with tc1=their_ts1 (echo), ts1=our_time_ns.
                // This lets the GCS compute round-trip latency.
                if *tc1 == 0 {
                    // Response: tc1 = their ts1, ts1 = our time
                    let our_time_ns = (now_ms as i64) * 1_000_000;
                    let remaining = &mut tx_buf[offset..];
                    if remaining.len() >= MAX_FRAME_SIZE {
                        let n = self.adapter.encode_timesync(*ts1, our_time_ns, remaining);
                        offset += n;
                    }
                }
                (offset, None)
            }
        }
    }

    /// Queue a statustext message. Severity is stored as a prefix digit.
    pub fn queue_statustext(&mut self, severity: u8, text: &str) {
        let mut s = heapless::String::<50>::new();
        // Pack severity as first char (ASCII digit 0-7)
        let _ = s.push((b'0' + severity.min(7)) as char);
        // Then the message text (remaining capacity)
        for ch in text.chars() {
            if s.push(ch).is_err() {
                break;
            }
        }
        let _ = self.statustext_queue.push_back(s);
    }

    /// Returns true if we have received a GCS heartbeat within the timeout window.
    pub fn is_gcs_connected(&self, now_ms: u64) -> bool {
        self.last_gcs_heartbeat_ms != NEVER_SENT
            && now_ms.wrapping_sub(self.last_gcs_heartbeat_ms) < GCS_HEARTBEAT_TIMEOUT_MS
    }

    /// Get the stored mission items (read-only).
    pub fn mission_items(&self) -> &[StoredMissionItem] {
        &self.mission_items[..self.mission_item_count as usize]
    }

    /// Get the GCS system ID (learned from heartbeat).
    pub fn gcs_sysid(&self) -> u8 {
        self.gcs_sysid
    }

    /// Check if parameter streaming is in progress.
    pub fn is_param_streaming(&self) -> bool {
        self.param_streaming
    }

    /// Get mutable ref to adapter (for direct access if needed).
    pub fn adapter_mut(&mut self) -> &mut MavlinkAdapter {
        &mut self.adapter
    }

    // ─── Private helpers ───

    fn send_heartbeat(&mut self, vs: &VehicleState, buf: &mut [u8]) -> usize {
        self.adapter.encode_heartbeat(
            vs.armed,
            vs.custom_mode,
            vs.mav_type,
            vs.system_status,
            buf,
        )
    }

    fn send_stream_group(
        &mut self,
        group: usize,
        _now_ms: u64,
        vs: &VehicleState,
        buf: &mut [u8],
    ) -> usize {
        match group {
            // Group 0: RAW_SENSORS — unused for now
            0 => 0,

            // Group 1: EXT_STATUS — SYS_STATUS
            1 => {
                self.adapter.encode_sys_status(
                    &vs.sensor_health,
                    vs.voltage_mv,
                    vs.current_ca,
                    vs.remaining_pct,
                    0, // load_pct
                    buf,
                )
            }

            // Group 2: RC_CHANNELS — unused for now
            2 => 0,

            // Group 3: POSITION — GLOBAL_POSITION_INT
            3 => {
                self.adapter.encode_position(
                    vs.lat_e7, vs.lon_e7, vs.alt_mm, vs.relative_alt_mm,
                    vs.vx, vs.vy, vs.vz, vs.heading,
                    vs.boot_time_ms,
                    buf,
                )
            }

            // Group 4: EXTRA1 — ATTITUDE
            4 => {
                self.adapter.encode_attitude(
                    vs.roll, vs.pitch, vs.yaw,
                    vs.rollspeed, vs.pitchspeed, vs.yawspeed,
                    vs.boot_time_ms,
                    buf,
                )
            }

            // Group 5: EXTRA2 — VFR_HUD
            5 => {
                self.adapter.encode_vfr_hud(
                    vs.airspeed,
                    vs.groundspeed,
                    vs.heading as i16,
                    vs.throttle,
                    vs.alt,
                    vs.climb,
                    buf,
                )
            }

            // Group 6: EXTRA3 — battery via SYS_STATUS (same as group 1)
            6 => {
                self.adapter.encode_sys_status(
                    &vs.sensor_health,
                    vs.voltage_mv,
                    vs.current_ca,
                    vs.remaining_pct,
                    0,
                    buf,
                )
            }

            _ => 0,
        }
    }

    fn dispatch_command(
        &self,
        command: u16,
        action: &CommandAction,
    ) -> (u8, Option<ServerAction>) {
        match action {
            CommandAction::Arm => (MAV_RESULT_ACCEPTED, Some(ServerAction::Arm)),
            CommandAction::Disarm => (MAV_RESULT_ACCEPTED, Some(ServerAction::Disarm)),
            CommandAction::Takeoff(alt) => {
                (MAV_RESULT_ACCEPTED, Some(ServerAction::Takeoff(*alt)))
            }
            CommandAction::Land => (MAV_RESULT_ACCEPTED, Some(ServerAction::Land)),
            CommandAction::Rtl => (MAV_RESULT_ACCEPTED, Some(ServerAction::Rtl)),
            CommandAction::SetMode(mode) => {
                (MAV_RESULT_ACCEPTED, Some(ServerAction::SetMode(*mode)))
            }
            CommandAction::RequestAutopilotVersion => {
                (MAV_RESULT_ACCEPTED, Some(ServerAction::RequestAutopilotVersion))
            }
            CommandAction::RequestMessage(msg_id) => {
                (MAV_RESULT_ACCEPTED, Some(ServerAction::RequestMessage(*msg_id)))
            }
            CommandAction::MotorTest { motor_index, throttle_type, throttle_value, timeout_s, motor_count } => {
                let _ = (motor_index, throttle_type, throttle_value, timeout_s, motor_count);
                (MAV_RESULT_ACCEPTED, Some(ServerAction::MotorTest {
                    motor_index: *motor_index,
                    throttle_type: *throttle_type,
                    throttle_value: *throttle_value,
                    timeout_s: *timeout_s,
                    motor_count: *motor_count,
                }))
            }
            CommandAction::SetMessageInterval { msg_id, interval_us } => {
                let _ = (msg_id, interval_us);
                // Accept the command — the caller can use the action to reconfigure
                // per-message streaming rates. For now, acknowledge unconditionally.
                (MAV_RESULT_ACCEPTED, Some(ServerAction::SetMessageInterval {
                    msg_id: *msg_id,
                    interval_us: *interval_us,
                }))
            }
            CommandAction::Unknown => {
                let _ = command; // suppress warning
                (MAV_RESULT_UNSUPPORTED, None)
            }
        }
    }

    fn send_param_at_index(&mut self, index: u16, buf: &mut [u8]) -> usize {
        if let Some((name, value)) = param_at_index(index) {
            self.adapter.encode_param_value(
                name,
                value,
                9, // MAV_PARAM_TYPE_REAL32
                self.param_send_count,
                index,
                buf,
            )
        } else {
            0
        }
    }

    fn send_param_by_name_or_index(
        &mut self,
        name: &heapless::String<16>,
        index: i16,
        buf: &mut [u8],
    ) -> usize {
        // If index >= 0, look up by index. Otherwise look up by name.
        if index >= 0 {
            let idx = index as u16;
            if let Some((pname, value)) = param_at_index(idx) {
                return self.adapter.encode_param_value(
                    pname, value, 9, param_table_count(), idx, buf,
                );
            }
        }
        // Look up by name
        if let Some((idx, value)) = param_by_name(name.as_str()) {
            return self.adapter.encode_param_value(
                name.as_str(), value, 9, param_table_count(), idx, buf,
            );
        }
        0
    }

    fn send_param_value_for_set(
        &mut self,
        name: &heapless::String<16>,
        value: f32,
        buf: &mut [u8],
    ) -> usize {
        let idx = param_by_name(name.as_str())
            .map(|(i, _)| i)
            .unwrap_or(0);
        self.adapter.encode_param_value(
            name.as_str(), value, 9, param_table_count(), idx, buf,
        )
    }
}

// ─── MISSION_REQUEST_INT encoder (adapter doesn't have one) ───

/// Encode a MISSION_REQUEST_INT message asking for a specific sequence number.
fn encode_mission_request_int(
    adapter: &mut MavlinkAdapter,
    seq: u16,
    buf: &mut [u8],
) -> usize {
    let mut payload = [0u8; 4];
    payload[0..2].copy_from_slice(&seq.to_le_bytes());
    payload[2] = 0; // target_system (GCS)
    payload[3] = 0; // target_component
    let header = adapter.make_header_pub(MSG_MISSION_REQUEST_INT, 4);
    encode_frame(&header, &payload, buf)
}

// ─── Severity prefix helper ───

fn parse_severity_prefix(s: &str) -> (u8, &str) {
    let bytes = s.as_bytes();
    if !bytes.is_empty() && bytes[0] >= b'0' && bytes[0] <= b'7' {
        (bytes[0] - b'0', &s[1..])
    } else {
        (6, s) // default: INFO
    }
}

// ─── Minimal parameter table ───
//
// In a real system this would be backed by the params crate. For the server
// module we provide a small default table that QGC needs to connect.

struct ParamEntry {
    name: &'static str,
    default_value: f32,
}

const PARAM_TABLE: &[ParamEntry] = &[
    ParamEntry { name: "SYSID_THISMAV", default_value: 1.0 },
    ParamEntry { name: "SYSID_MYGCS", default_value: 255.0 },
    ParamEntry { name: "ARMING_CHECK", default_value: 1.0 },
    ParamEntry { name: "ATC_RAT_RLL_P", default_value: 0.135 },
    ParamEntry { name: "ATC_RAT_RLL_I", default_value: 0.135 },
    ParamEntry { name: "ATC_RAT_RLL_D", default_value: 0.0036 },
    ParamEntry { name: "ATC_RAT_PIT_P", default_value: 0.135 },
    ParamEntry { name: "ATC_RAT_PIT_I", default_value: 0.135 },
    ParamEntry { name: "ATC_RAT_PIT_D", default_value: 0.0036 },
    ParamEntry { name: "ATC_RAT_YAW_P", default_value: 0.18 },
    ParamEntry { name: "ATC_RAT_YAW_I", default_value: 0.018 },
    ParamEntry { name: "ATC_RAT_YAW_D", default_value: 0.0 },
    ParamEntry { name: "PILOT_THR_BHV", default_value: 0.0 },
    ParamEntry { name: "WPNAV_SPEED", default_value: 500.0 },
    ParamEntry { name: "WPNAV_SPEED_UP", default_value: 250.0 },
    ParamEntry { name: "WPNAV_SPEED_DN", default_value: 150.0 },
    ParamEntry { name: "RTL_ALT", default_value: 1500.0 },
    ParamEntry { name: "FLTMODE1", default_value: 0.0 },
    ParamEntry { name: "FLTMODE2", default_value: 2.0 },
    ParamEntry { name: "FLTMODE3", default_value: 5.0 },
    ParamEntry { name: "FLTMODE4", default_value: 4.0 },
    ParamEntry { name: "FLTMODE5", default_value: 6.0 },
    ParamEntry { name: "FLTMODE6", default_value: 9.0 },
    ParamEntry { name: "FRAME_CLASS", default_value: 1.0 },
    ParamEntry { name: "FRAME_TYPE", default_value: 0.0 },
    ParamEntry { name: "SR0_RAW_SENS", default_value: 2.0 },
    ParamEntry { name: "SR0_EXT_STAT", default_value: 2.0 },
    ParamEntry { name: "SR0_RC_CHAN", default_value: 5.0 },
    ParamEntry { name: "SR0_POSITION", default_value: 3.0 },
    ParamEntry { name: "SR0_EXTRA1", default_value: 10.0 },
    ParamEntry { name: "SR0_EXTRA2", default_value: 4.0 },
    ParamEntry { name: "SR0_EXTRA3", default_value: 1.0 },
];

fn param_table_count() -> u16 {
    PARAM_TABLE.len() as u16
}

fn param_at_index(index: u16) -> Option<(&'static str, f32)> {
    PARAM_TABLE.get(index as usize).map(|e| (e.name, e.default_value))
}

fn param_by_name(name: &str) -> Option<(u16, f32)> {
    PARAM_TABLE.iter().enumerate().find_map(|(i, e)| {
        if e.name == name {
            Some((i as u16, e.default_value))
        } else {
            None
        }
    })
}

// ─── Tests ───

#[cfg(test)]
mod tests {
    use super::*;

    fn make_vehicle_state() -> VehicleState {
        VehicleState {
            armed: true,
            custom_mode: 5, // Loiter
            mav_type: MAV_TYPE_QUADROTOR,
            system_status: MAV_STATE_ACTIVE,
            roll: 0.05,
            pitch: -0.02,
            yaw: 1.57,
            rollspeed: 0.01,
            pitchspeed: -0.005,
            yawspeed: 0.0,
            lat_e7: 350000000,
            lon_e7: -1200000000,
            alt_mm: 50000,
            relative_alt_mm: 10000,
            vx: 100,
            vy: -50,
            vz: 20,
            heading: 27000,
            airspeed: 18.0,
            groundspeed: 17.5,
            alt: 50.0,
            climb: 0.5,
            throttle: 500,
            voltage_mv: 12600,
            current_ca: 500,
            remaining_pct: 85,
            sensor_health: SensorHealth::all_healthy(),
            boot_time_ms: 5000,
            home_lat_e7: 350000000,
            home_lon_e7: -1200000000,
            home_alt_mm: 40000,
            mission_current_seq: 0,
        }
    }

    #[test]
    fn test_server_new() {
        let server = MavlinkServer::new(1, 1);
        assert!(!server.is_gcs_connected(0));
        assert!(!server.is_param_streaming());
        assert_eq!(server.mission_items().len(), 0);
    }

    #[test]
    fn test_heartbeat_sent_on_first_update() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 4096];

        let n = server.update(0, &vs, &mut buf);
        assert!(n > 0, "Should have sent at least heartbeat");

        // Parse the first frame — should be a heartbeat
        let (hdr, payload) = parse_frame(&buf[..]).unwrap();
        assert_eq!(hdr.msg_id, MSG_HEARTBEAT);
        assert_eq!(payload[4], MAV_TYPE_QUADROTOR);
        // Armed flag
        assert!(payload[6] & MAV_MODE_FLAG_SAFETY_ARMED != 0);
    }

    #[test]
    fn test_heartbeat_not_repeated_within_1s() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 4096];

        // First call at t=0 sends heartbeat
        let n1 = server.update(0, &vs, &mut buf);
        assert!(n1 > 0);

        // Second call at t=500ms should not send another heartbeat
        // (stream groups may fire, but heartbeat should not repeat)
        let mut buf2 = [0u8; 4096];
        let _n2 = server.update(500, &vs, &mut buf2);

        // Parse all frames in buf2 — none should be heartbeat
        let mut pos = 0;
        while pos + HEADER_SIZE + 2 <= _n2 {
            if let Some((hdr, _payload)) = parse_frame(&buf2[pos..]) {
                assert_ne!(hdr.msg_id, MSG_HEARTBEAT, "Should not repeat heartbeat at 500ms");
                let frame_len = HEADER_SIZE + hdr.payload_len as usize + 2;
                pos += frame_len;
            } else {
                break;
            }
        }
    }

    #[test]
    fn test_heartbeat_repeats_after_1s() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 4096];

        server.update(0, &vs, &mut buf);

        let mut buf2 = [0u8; 4096];
        let n2 = server.update(1000, &vs, &mut buf2);
        assert!(n2 > 0);

        // Should contain a heartbeat
        let mut found_hb = false;
        let mut pos = 0;
        while pos + HEADER_SIZE + 2 <= n2 {
            if let Some((hdr, _)) = parse_frame(&buf2[pos..]) {
                if hdr.msg_id == MSG_HEARTBEAT {
                    found_hb = true;
                }
                let frame_len = HEADER_SIZE + hdr.payload_len as usize + 2;
                pos += frame_len;
            } else {
                break;
            }
        }
        assert!(found_hb, "Should have heartbeat at 1000ms");
    }

    #[test]
    fn test_stream_groups_fire() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 8192];

        // At t=0, everything fires (all last_send = 0, all intervals elapsed)
        let n = server.update(0, &vs, &mut buf);
        assert!(n > 0);

        // Collect all message IDs
        let msg_ids = collect_msg_ids(&buf[..n]);
        assert!(msg_ids.contains(&MSG_HEARTBEAT), "Should have heartbeat");
        assert!(msg_ids.contains(&MSG_ATTITUDE), "Should have attitude (group 4)");
        assert!(msg_ids.contains(&MSG_VFR_HUD), "Should have VFR_HUD (group 5)");
        assert!(msg_ids.contains(&MSG_GLOBAL_POSITION_INT), "Should have position (group 3)");
        assert!(msg_ids.contains(&MSG_SYS_STATUS), "Should have sys_status (group 1)");
    }

    #[test]
    fn test_attitude_at_10hz() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 8192];

        // Fire at t=0 to initialize
        server.update(0, &vs, &mut buf);

        // At t=100ms, only attitude (100ms interval) should fire
        let mut buf2 = [0u8; 8192];
        let n = server.update(100, &vs, &mut buf2);
        let ids = collect_msg_ids(&buf2[..n]);
        assert!(ids.contains(&MSG_ATTITUDE), "Attitude should fire at 100ms");
        // VFR_HUD has 250ms interval, should not fire yet
        assert!(!ids.contains(&MSG_VFR_HUD), "VFR_HUD should NOT fire at 100ms");
    }

    #[test]
    fn test_gcs_heartbeat_tracking() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 1024];

        assert!(!server.is_gcs_connected(0));

        // Receive GCS heartbeat
        let (_, action) = server.handle_message(
            &InboundCommand::GcsHeartbeat, 1000, &vs, &mut buf,
        );
        assert!(action.is_none());
        assert!(server.is_gcs_connected(1000));
        assert!(server.is_gcs_connected(4400)); // 3400ms later = still OK (< 3500)
        assert!(!server.is_gcs_connected(4600)); // 3600ms later = timed out
    }

    #[test]
    fn test_command_arm_produces_ack_and_action() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 1024];

        let msg = InboundCommand::Command {
            command: 400,
            result: CommandAction::Arm,
        };
        let (n, action) = server.handle_message(&msg, 0, &vs, &mut buf);

        // Should produce COMMAND_ACK
        assert!(n > 0);
        let (hdr, payload) = parse_frame(&buf[..n]).unwrap();
        assert_eq!(hdr.msg_id, MSG_COMMAND_ACK);
        let acked_cmd = u16::from_le_bytes([payload[0], payload[1]]);
        assert_eq!(acked_cmd, 400);
        assert_eq!(payload[2], MAV_RESULT_ACCEPTED);

        // Should produce Arm action
        match action {
            Some(ServerAction::Arm) => {}
            other => panic!("Expected Arm, got {:?}", other),
        }
    }

    #[test]
    fn test_command_disarm() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 1024];

        let msg = InboundCommand::Command {
            command: 400,
            result: CommandAction::Disarm,
        };
        let (n, action) = server.handle_message(&msg, 0, &vs, &mut buf);
        assert!(n > 0);
        match action {
            Some(ServerAction::Disarm) => {}
            other => panic!("Expected Disarm, got {:?}", other),
        }
    }

    #[test]
    fn test_command_takeoff() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 1024];

        let msg = InboundCommand::Command {
            command: 22,
            result: CommandAction::Takeoff(10.0),
        };
        let (_, action) = server.handle_message(&msg, 0, &vs, &mut buf);
        match action {
            Some(ServerAction::Takeoff(alt)) => assert!((alt - 10.0).abs() < 1e-6),
            other => panic!("Expected Takeoff(10.0), got {:?}", other),
        }
    }

    #[test]
    fn test_command_land_rtl_setmode() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 1024];

        // Land
        let (_, action) = server.handle_message(
            &InboundCommand::Command { command: 21, result: CommandAction::Land },
            0, &vs, &mut buf,
        );
        assert!(matches!(action, Some(ServerAction::Land)));

        // RTL
        let (_, action) = server.handle_message(
            &InboundCommand::Command { command: 20, result: CommandAction::Rtl },
            0, &vs, &mut buf,
        );
        assert!(matches!(action, Some(ServerAction::Rtl)));

        // SetMode
        let (_, action) = server.handle_message(
            &InboundCommand::Command { command: 176, result: CommandAction::SetMode(5) },
            0, &vs, &mut buf,
        );
        match action {
            Some(ServerAction::SetMode(5)) => {}
            other => panic!("Expected SetMode(5), got {:?}", other),
        }
    }

    #[test]
    fn test_unknown_command_nacks() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 1024];

        let msg = InboundCommand::Command {
            command: 999,
            result: CommandAction::Unknown,
        };
        let (n, action) = server.handle_message(&msg, 0, &vs, &mut buf);
        assert!(n > 0);
        let (_, payload) = parse_frame(&buf[..n]).unwrap();
        assert_eq!(payload[2], MAV_RESULT_UNSUPPORTED);
        assert!(action.is_none());
    }

    #[test]
    fn test_param_request_list_starts_streaming() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 1024];

        let (n, _) = server.handle_message(
            &InboundCommand::ParamRequestList, 0, &vs, &mut buf,
        );
        assert_eq!(n, 0, "ParamRequestList itself doesn't send anything");
        assert!(server.is_param_streaming());

        // Now update should stream parameters
        let mut tx_buf = [0u8; 16384];
        let n = server.update(0, &vs, &mut tx_buf);
        assert!(n > 0);

        let ids = collect_msg_ids(&tx_buf[..n]);
        assert!(ids.contains(&MSG_PARAM_VALUE), "Should be streaming params");
    }

    #[test]
    fn test_param_streaming_completes() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 1024];

        server.handle_message(&InboundCommand::ParamRequestList, 0, &vs, &mut buf);

        // Stream until done
        let total_params = param_table_count();
        let mut tx_buf = [0u8; 65536];
        let mut total_param_values = 0u16;

        for tick in 0..100u64 {
            let n = server.update(tick * 10, &vs, &mut tx_buf);
            let ids = collect_msg_ids(&tx_buf[..n]);
            total_param_values += ids.iter().filter(|&&id| id == MSG_PARAM_VALUE).count() as u16;
            if !server.is_param_streaming() {
                break;
            }
        }

        assert!(!server.is_param_streaming(), "Streaming should have finished");
        assert_eq!(total_param_values, total_params, "Should have sent all params");
    }

    #[test]
    fn test_param_request_read_by_name() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 1024];

        let mut name = heapless::String::<16>::new();
        let _ = name.push_str("ATC_RAT_RLL_P");

        let (n, _) = server.handle_message(
            &InboundCommand::ParamRequestRead { name, index: -1 },
            0, &vs, &mut buf,
        );
        assert!(n > 0);
        let (hdr, payload) = parse_frame(&buf[..n]).unwrap();
        assert_eq!(hdr.msg_id, MSG_PARAM_VALUE);
        let val = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
        assert!((val - 0.135).abs() < 1e-6);
    }

    #[test]
    fn test_param_request_read_by_index() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 1024];

        let name = heapless::String::<16>::new(); // empty

        let (n, _) = server.handle_message(
            &InboundCommand::ParamRequestRead { name, index: 0 },
            0, &vs, &mut buf,
        );
        assert!(n > 0);
        let (hdr, _) = parse_frame(&buf[..n]).unwrap();
        assert_eq!(hdr.msg_id, MSG_PARAM_VALUE);
    }

    #[test]
    fn test_param_set_returns_confirmation_and_action() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 1024];

        let mut name = heapless::String::<16>::new();
        let _ = name.push_str("ATC_RAT_RLL_P");

        let (n, action) = server.handle_message(
            &InboundCommand::ParamSet { name: name.clone(), value: 0.2 },
            0, &vs, &mut buf,
        );
        assert!(n > 0);
        let (hdr, payload) = parse_frame(&buf[..n]).unwrap();
        assert_eq!(hdr.msg_id, MSG_PARAM_VALUE);
        let val = f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]);
        assert!((val - 0.2).abs() < 1e-6);

        match action {
            Some(ServerAction::SetParam { name: n, value: v }) => {
                assert_eq!(n.as_str(), "ATC_RAT_RLL_P");
                assert!((v - 0.2).abs() < 1e-6);
            }
            other => panic!("Expected SetParam, got {:?}", other),
        }
    }

    #[test]
    fn test_stream_request_updates_rate() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 1024];

        // Set attitude (group 4) to 20 Hz
        server.handle_message(
            &InboundCommand::StreamRequest { stream_id: 4, rate: 20, start: true },
            0, &vs, &mut buf,
        );
        assert_eq!(server.stream_intervals_ms[4], 50); // 1000/20 = 50ms

        // Disable group 4
        server.handle_message(
            &InboundCommand::StreamRequest { stream_id: 4, rate: 0, start: false },
            0, &vs, &mut buf,
        );
        assert_eq!(server.stream_intervals_ms[4], 0);
    }

    #[test]
    fn test_mission_upload_flow() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 2048];

        // GCS sends MISSION_COUNT(3)
        let (n, action) = server.handle_message(
            &InboundCommand::MissionCount(3), 0, &vs, &mut buf,
        );
        assert!(action.is_none());
        // Server should request item 0
        assert!(n > 0);
        let (hdr, payload) = parse_frame(&buf[..n]).unwrap();
        assert_eq!(hdr.msg_id, MSG_MISSION_REQUEST_INT);
        let req_seq = u16::from_le_bytes([payload[0], payload[1]]);
        assert_eq!(req_seq, 0);

        // GCS sends item 0
        let (n, action) = server.handle_message(
            &InboundCommand::MissionItemInt {
                seq: 0, frame: 3, command: 16,
                params: [0.0; 4], x: 350000000, y: -1200000000, z: 50.0,
            },
            100, &vs, &mut buf,
        );
        assert!(action.is_none());
        let (hdr, payload) = parse_frame(&buf[..n]).unwrap();
        assert_eq!(hdr.msg_id, MSG_MISSION_REQUEST_INT);
        let req_seq = u16::from_le_bytes([payload[0], payload[1]]);
        assert_eq!(req_seq, 1);

        // GCS sends item 1
        let (n, _) = server.handle_message(
            &InboundCommand::MissionItemInt {
                seq: 1, frame: 3, command: 16,
                params: [0.0; 4], x: 350001000, y: -1200001000, z: 60.0,
            },
            200, &vs, &mut buf,
        );
        let (hdr, payload) = parse_frame(&buf[..n]).unwrap();
        assert_eq!(hdr.msg_id, MSG_MISSION_REQUEST_INT);
        let req_seq = u16::from_le_bytes([payload[0], payload[1]]);
        assert_eq!(req_seq, 2);

        // GCS sends item 2 (last)
        let (n, action) = server.handle_message(
            &InboundCommand::MissionItemInt {
                seq: 2, frame: 3, command: 16,
                params: [0.0; 4], x: 350002000, y: -1200002000, z: 70.0,
            },
            300, &vs, &mut buf,
        );
        // Should send MISSION_ACK (accepted)
        let (hdr, payload) = parse_frame(&buf[..n]).unwrap();
        assert_eq!(hdr.msg_id, MSG_MISSION_ACK);
        assert_eq!(payload[2], 0); // MAV_MISSION_ACCEPTED

        match action {
            Some(ServerAction::MissionUploaded) => {}
            other => panic!("Expected MissionUploaded, got {:?}", other),
        }

        // Verify stored items
        assert_eq!(server.mission_items().len(), 3);
        assert_eq!(server.mission_items()[0].x, 350000000);
        assert_eq!(server.mission_items()[2].z, 70.0);
    }

    #[test]
    fn test_mission_download_flow() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 2048];

        // First upload a mission
        server.handle_message(&InboundCommand::MissionCount(1), 0, &vs, &mut buf);
        server.handle_message(
            &InboundCommand::MissionItemInt {
                seq: 0, frame: 3, command: 16,
                params: [1.0, 2.0, 3.0, 4.0], x: 350000000, y: -1200000000, z: 50.0,
            },
            100, &vs, &mut buf,
        );

        // Now request download
        let (n, _) = server.handle_message(
            &InboundCommand::MissionRequestList, 200, &vs, &mut buf,
        );
        let (hdr, payload) = parse_frame(&buf[..n]).unwrap();
        assert_eq!(hdr.msg_id, MSG_MISSION_COUNT);
        let count = u16::from_le_bytes([payload[0], payload[1]]);
        assert_eq!(count, 1);

        // GCS requests item 0
        let (n, _) = server.handle_message(
            &InboundCommand::MissionRequestInt(0), 300, &vs, &mut buf,
        );
        let (hdr, _) = parse_frame(&buf[..n]).unwrap();
        assert_eq!(hdr.msg_id, MSG_MISSION_ITEM_INT);
    }

    #[test]
    fn test_mission_clear_all() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 2048];

        // Upload a mission first
        server.handle_message(&InboundCommand::MissionCount(1), 0, &vs, &mut buf);
        server.handle_message(
            &InboundCommand::MissionItemInt {
                seq: 0, frame: 3, command: 16,
                params: [0.0; 4], x: 0, y: 0, z: 50.0,
            },
            100, &vs, &mut buf,
        );
        assert_eq!(server.mission_items().len(), 1);

        // Clear
        let (n, action) = server.handle_message(
            &InboundCommand::MissionClearAll, 200, &vs, &mut buf,
        );
        assert!(n > 0);
        let (hdr, _) = parse_frame(&buf[..n]).unwrap();
        assert_eq!(hdr.msg_id, MSG_MISSION_ACK);
        assert!(matches!(action, Some(ServerAction::MissionCleared)));
        assert_eq!(server.mission_items().len(), 0);
    }

    #[test]
    fn test_mission_upload_out_of_sequence_nacks() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 2048];

        server.handle_message(&InboundCommand::MissionCount(3), 0, &vs, &mut buf);

        // Send item 1 instead of 0 (out of sequence)
        let (n, action) = server.handle_message(
            &InboundCommand::MissionItemInt {
                seq: 1, frame: 3, command: 16,
                params: [0.0; 4], x: 0, y: 0, z: 50.0,
            },
            100, &vs, &mut buf,
        );
        let (hdr, payload) = parse_frame(&buf[..n]).unwrap();
        assert_eq!(hdr.msg_id, MSG_MISSION_ACK);
        assert_eq!(payload[2], 1); // error
        assert!(action.is_none());
    }

    #[test]
    fn test_statustext_queue_and_drain() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();

        server.queue_statustext(4, "PreArm: GPS not healthy");
        server.queue_statustext(6, "All systems nominal");

        // First update drains one message
        let mut buf = [0u8; 8192];
        let n = server.update(0, &vs, &mut buf);
        let ids = collect_msg_ids(&buf[..n]);
        let statustext_count = ids.iter().filter(|&&id| id == MSG_STATUSTEXT).count();
        assert_eq!(statustext_count, 1, "Should drain exactly 1 statustext per update");

        // Second update too soon — rate limited (200ms)
        let n = server.update(100, &vs, &mut buf);
        let ids = collect_msg_ids(&buf[..n]);
        let statustext_count = ids.iter().filter(|&&id| id == MSG_STATUSTEXT).count();
        assert_eq!(statustext_count, 0, "Rate limited: no statustext at 100ms");

        // Third update at 200ms — should drain the second message
        let n = server.update(200, &vs, &mut buf);
        let ids = collect_msg_ids(&buf[..n]);
        let statustext_count = ids.iter().filter(|&&id| id == MSG_STATUSTEXT).count();
        assert_eq!(statustext_count, 1, "Should drain second statustext at 200ms");
    }

    #[test]
    fn test_statustext_severity_encoding() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();

        server.queue_statustext(2, "Critical error");

        let mut buf = [0u8; 8192];
        let n = server.update(0, &vs, &mut buf);

        // Find the STATUSTEXT frame
        let mut pos = 0;
        while pos + HEADER_SIZE + 2 <= n {
            if let Some((hdr, payload)) = parse_frame(&buf[pos..]) {
                if hdr.msg_id == MSG_STATUSTEXT {
                    assert_eq!(payload[0], 2, "Severity should be 2 (CRITICAL)");
                    // Text should start with 'C' from "Critical error"
                    assert_eq!(payload[1], b'C');
                    return;
                }
                pos += HEADER_SIZE + hdr.payload_len as usize + 2;
            } else {
                break;
            }
        }
        panic!("STATUSTEXT frame not found");
    }

    #[test]
    fn test_request_autopilot_version_action() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 1024];

        let (n, action) = server.handle_message(
            &InboundCommand::Command {
                command: 519,
                result: CommandAction::RequestAutopilotVersion,
            },
            0, &vs, &mut buf,
        );
        assert!(n > 0); // ACK
        match action {
            Some(ServerAction::RequestAutopilotVersion) => {}
            other => panic!("Expected RequestAutopilotVersion, got {:?}", other),
        }
    }

    #[test]
    fn test_request_message_action() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 1024];

        let (_, action) = server.handle_message(
            &InboundCommand::Command {
                command: 520,
                result: CommandAction::RequestMessage(148),
            },
            0, &vs, &mut buf,
        );
        match action {
            Some(ServerAction::RequestMessage(148)) => {}
            other => panic!("Expected RequestMessage(148), got {:?}", other),
        }
    }

    #[test]
    fn test_mission_count_zero_nacks() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 1024];

        let (n, _) = server.handle_message(
            &InboundCommand::MissionCount(0), 0, &vs, &mut buf,
        );
        let (hdr, payload) = parse_frame(&buf[..n]).unwrap();
        assert_eq!(hdr.msg_id, MSG_MISSION_ACK);
        assert_eq!(payload[2], 1); // error
    }

    #[test]
    fn test_mission_upload_timeout() {
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 2048];

        // Start upload
        server.handle_message(&InboundCommand::MissionCount(3), 0, &vs, &mut buf);

        // Don't send any items — wait for timeout
        let mut tx = [0u8; 8192];
        server.update(3000, &vs, &mut tx); // 3s > 2s timeout

        // Upload state should be reset
        assert_eq!(server.mission_upload_count, 0);
        assert_eq!(server.mission_upload_received, 0);
    }

    #[test]
    fn test_vehicle_state_default() {
        let vs = VehicleState::default();
        assert!(!vs.armed);
        assert_eq!(vs.custom_mode, 0);
        assert_eq!(vs.mav_type, MAV_TYPE_QUADROTOR);
        assert_eq!(vs.system_status, MAV_STATE_STANDBY);
    }

    #[test]
    fn test_full_connection_sequence() {
        // Simulate what QGC does on connect:
        // 1. Send heartbeat
        // 2. Request data streams
        // 3. Request param list
        // 4. Request autopilot version
        let mut server = MavlinkServer::new(1, 1);
        let vs = make_vehicle_state();
        let mut buf = [0u8; 4096];

        // Step 1: GCS heartbeat
        server.handle_message(&InboundCommand::GcsHeartbeat, 0, &vs, &mut buf);
        assert!(server.is_gcs_connected(0));

        // Step 2: Request streams
        server.handle_message(
            &InboundCommand::StreamRequest { stream_id: 1, rate: 2, start: true },
            10, &vs, &mut buf,
        );
        server.handle_message(
            &InboundCommand::StreamRequest { stream_id: 3, rate: 3, start: true },
            20, &vs, &mut buf,
        );
        server.handle_message(
            &InboundCommand::StreamRequest { stream_id: 4, rate: 10, start: true },
            30, &vs, &mut buf,
        );

        // Step 3: Param list
        server.handle_message(&InboundCommand::ParamRequestList, 40, &vs, &mut buf);
        assert!(server.is_param_streaming());

        // Step 4: Autopilot version
        let (_, action) = server.handle_message(
            &InboundCommand::Command {
                command: 519,
                result: CommandAction::RequestAutopilotVersion,
            },
            50, &vs, &mut buf,
        );
        assert!(matches!(action, Some(ServerAction::RequestAutopilotVersion)));

        // Now run a few update cycles to make sure everything works
        let mut total_bytes = 0;
        for t in (100..2000).step_by(100) {
            let n = server.update(t as u64, &vs, &mut buf);
            total_bytes += n;
        }
        assert!(total_bytes > 0, "Should have sent telemetry");
    }

    // ─── Helper: collect all message IDs from a buffer of MAVLink frames ───

    fn collect_msg_ids(buf: &[u8]) -> heapless::Vec<u32, 64> {
        let mut ids = heapless::Vec::new();
        let mut pos = 0;
        while pos + HEADER_SIZE + 2 <= buf.len() {
            if let Some((hdr, _)) = parse_frame(&buf[pos..]) {
                let _ = ids.push(hdr.msg_id);
                let frame_len = HEADER_SIZE + hdr.payload_len as usize + 2;
                pos += frame_len;
            } else {
                pos += 1; // skip byte and try again
            }
        }
        ids
    }
}

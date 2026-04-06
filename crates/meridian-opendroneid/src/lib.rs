#![no_std]

//! OpenDroneID / Remote ID — FAA and EU regulatory compliance.
//!
//! Source: ArduPilot AP_OpenDroneID + ASTM F3411-22a Remote ID standard
//! 5 message types broadcast at required intervals.
//! Pre-arm requires arm_status OK handshake from transponder module.

use heapless::String;

/// UAS (drone) ID type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IdType {
    None = 0,
    SerialNumber = 1, // ANSI/CTA-2063-A format
    CaaRegistration = 2,
    UtmAssigned = 3,
    SpecificSession = 4,
}

/// UA (aircraft) type classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UaType {
    None = 0,
    Aeroplane = 1,
    HelicopterOrMultirotor = 2,
    Gyroplane = 3,
    HybridLift = 4,
    Ornithopter = 5,
    Glider = 6,
    Kite = 7,
    FreeBalloon = 8,
    CaptiveBalloon = 9,
    Airship = 10,
    FreeFallParachute = 11,
    Rocket = 12,
    TetheredPowered = 13,
    GroundObstacle = 14,
    Other = 15,
}

/// Operator location type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OperatorLocationType {
    Takeoff = 0,
    Dynamic = 1,
    Fixed = 2,
}

// ─── Message Types ───

/// Basic ID message — identifies the UAS.
#[derive(Debug, Clone)]
pub struct BasicIdMessage {
    pub id_type: IdType,
    pub ua_type: UaType,
    pub uas_id: String<20>, // serial number or registration
}

/// Location/Vector message — current position and velocity.
#[derive(Debug, Clone, Copy)]
pub struct LocationMessage {
    pub status: u8,         // 0=undeclared, 1=ground, 2=airborne, 3=emergency
    pub direction: u16,     // heading in 0.01° (0-35999)
    pub speed_horizontal: u16, // cm/s
    pub speed_vertical: i16,   // cm/s (positive = up)
    pub latitude: f64,      // degrees
    pub longitude: f64,
    pub altitude_baro: f32, // meters (pressure altitude)
    pub altitude_geo: f32,  // meters (geometric/GPS altitude)
    pub height_agl: f32,    // meters above ground/takeoff
    pub accuracy_h: u8,     // horizontal accuracy category (0-15)
    pub accuracy_v: u8,     // vertical accuracy category
    pub accuracy_speed: u8,
    pub timestamp: f32,     // seconds since hour (0-3599.9)
}

/// System message — operator location + area info.
#[derive(Debug, Clone, Copy)]
pub struct SystemMessage {
    pub operator_location_type: OperatorLocationType,
    pub operator_latitude: f64,
    pub operator_longitude: f64,
    pub operator_altitude_geo: f32,
    pub area_count: u16,
    pub area_radius: u16,   // meters
    pub area_ceiling: f32,
    pub area_floor: f32,
    pub classification_type: u8,
    pub timestamp: u32,     // Unix timestamp
}

/// Self-ID message — free-text description.
#[derive(Debug, Clone)]
pub struct SelfIdMessage {
    pub description_type: u8,
    pub description: String<23>,
}

/// Operator ID message.
#[derive(Debug, Clone)]
pub struct OperatorIdMessage {
    pub operator_id_type: u8,
    pub operator_id: String<20>,
}

// ─── D1: Authentication message (ASTM F3411-22a mandatory) ───

/// Authentication type (ASTM F3411-22a Table 2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuthType {
    None = 0,
    UasIdSignature = 1,
    OperatorIdSignature = 2,
    MessageSetSignature = 3,
    NetworkRemoteId = 4,
    SpecificAuthentication = 5,
}

/// D1: Authentication message — ASTM F3411-22a mandatory.
///
/// Carries a cryptographic signature proving the authenticity of
/// the other ODID messages. The full authentication data can span
/// multiple pages (up to 5 pages x 23 bytes = 115 bytes).
#[derive(Debug, Clone)]
pub struct AuthenticationMessage {
    /// Authentication type.
    pub auth_type: AuthType,
    /// Page number (0-4). Multi-page auth data.
    pub page_number: u8,
    /// Length of auth data in last page (0-23).
    pub last_page_index: u8,
    /// Timestamp: seconds since 2019-01-01 00:00:00.
    pub timestamp: u32,
    /// Authentication data (up to 23 bytes per page).
    pub auth_data: [u8; 23],
    /// Valid bytes in auth_data.
    pub auth_data_len: u8,
}

impl AuthenticationMessage {
    pub fn new() -> Self {
        Self {
            auth_type: AuthType::None,
            page_number: 0,
            last_page_index: 0,
            timestamp: 0,
            auth_data: [0u8; 23],
            auth_data_len: 0,
        }
    }
}

// ─── D2: ASTM 25-byte wire format encoding stubs ───

/// D2: Encode a BasicID message into the 25-byte ASTM wire format.
///
/// ASTM F3411-22a message format: 1-byte header + 24 bytes payload.
/// Header: bits [7:4] = message type (0x0), bits [3:0] = protocol version (0x2).
pub fn encode_basic_id(msg: &BasicIdMessage, buf: &mut [u8; 25]) {
    buf.fill(0);
    buf[0] = 0x02; // type=0 (BasicID), version=2
    buf[1] = (msg.id_type as u8) << 4 | (msg.ua_type as u8);
    // UAS ID: 20 bytes, ASCII, padded with nulls
    let id_bytes = msg.uas_id.as_bytes();
    let len = id_bytes.len().min(20);
    buf[2..2 + len].copy_from_slice(&id_bytes[..len]);
}

/// D2: Encode a LocationMessage into 25-byte ASTM wire format.
pub fn encode_location(msg: &LocationMessage, buf: &mut [u8; 25]) {
    buf.fill(0);
    buf[0] = 0x12; // type=1 (Location), version=2
    buf[1] = msg.status & 0x0F;
    // Direction: 0-360 in 0.01 deg → 0-35999 → encoded as u16
    buf[2] = (msg.direction & 0xFF) as u8;
    buf[3] = ((msg.direction >> 8) & 0xFF) as u8;
    // Speed horizontal: cm/s as u16
    buf[4] = (msg.speed_horizontal & 0xFF) as u8;
    buf[5] = ((msg.speed_horizontal >> 8) & 0xFF) as u8;
    // Speed vertical: cm/s as i16
    let sv = msg.speed_vertical as u16;
    buf[6] = (sv & 0xFF) as u8;
    buf[7] = ((sv >> 8) & 0xFF) as u8;
    // Lat/lon: degrees * 1e7 as i32
    let lat_i = (msg.latitude * 1e7) as i32;
    let lon_i = (msg.longitude * 1e7) as i32;
    buf[8..12].copy_from_slice(&lat_i.to_le_bytes());
    buf[12..16].copy_from_slice(&lon_i.to_le_bytes());
    // Altitude baro: (meters + 1000) * 2 as u16 (half-meter resolution)
    let alt_b = ((msg.altitude_baro + 1000.0) * 2.0) as u16;
    buf[16] = (alt_b & 0xFF) as u8;
    buf[17] = ((alt_b >> 8) & 0xFF) as u8;
    // Altitude geo
    let alt_g = ((msg.altitude_geo + 1000.0) * 2.0) as u16;
    buf[18] = (alt_g & 0xFF) as u8;
    buf[19] = ((alt_g >> 8) & 0xFF) as u8;
    // Height AGL
    let height = ((msg.height_agl + 1000.0) * 2.0) as u16;
    buf[20] = (height & 0xFF) as u8;
    buf[21] = ((height >> 8) & 0xFF) as u8;
    // Accuracy + timestamp packed into remaining bytes
    buf[22] = (msg.accuracy_h & 0x0F) | ((msg.accuracy_v & 0x0F) << 4);
    // Timestamp: seconds since hour * 10 (0.1s resolution)
    let ts = (msg.timestamp * 10.0) as u16;
    buf[23] = (ts & 0xFF) as u8;
    buf[24] = ((ts >> 8) & 0xFF) as u8;
}

/// D2: Encode a SystemMessage into 25-byte ASTM wire format.
pub fn encode_system(msg: &SystemMessage, buf: &mut [u8; 25]) {
    buf.fill(0);
    buf[0] = 0x42; // type=4 (System), version=2
    buf[1] = msg.operator_location_type as u8;
    let lat = (msg.operator_latitude * 1e7) as i32;
    let lon = (msg.operator_longitude * 1e7) as i32;
    buf[2..6].copy_from_slice(&lat.to_le_bytes());
    buf[6..10].copy_from_slice(&lon.to_le_bytes());
    buf[10..12].copy_from_slice(&msg.area_count.to_le_bytes());
    buf[12..14].copy_from_slice(&msg.area_radius.to_le_bytes());
    let alt = ((msg.operator_altitude_geo + 1000.0) * 2.0) as u16;
    buf[14] = (alt & 0xFF) as u8;
    buf[15] = ((alt >> 8) & 0xFF) as u8;
    buf[16..20].copy_from_slice(&msg.timestamp.to_le_bytes());
}

/// D2: Encode an Authentication message into 25-byte ASTM wire format.
pub fn encode_authentication(msg: &AuthenticationMessage, buf: &mut [u8; 25]) {
    buf.fill(0);
    buf[0] = 0x22; // type=2 (Auth), version=2
    buf[1] = (msg.auth_type as u8) << 4 | (msg.page_number & 0x0F);
    let len = msg.auth_data_len.min(23) as usize;
    buf[2..2 + len].copy_from_slice(&msg.auth_data[..len]);
}

// ─── D3: Per-type broadcast timers ───

/// D3: Per-message-type broadcast timers instead of shared round-robin.
///
/// ASTM F3411-22a requires:
/// - Location: 1 Hz (1000 ms)
/// - BasicID:  3 Hz (333 ms)  or 1/3 Hz (3000 ms) depending on transport
/// - System:   1/3 Hz (3000 ms)
/// - Authentication: 1/3 Hz (3000 ms)
#[derive(Debug, Clone, Copy)]
pub struct OdidTimers {
    pub last_location_ms: u32,
    pub last_basic_id_ms: u32,
    pub last_self_id_ms: u32,
    pub last_system_ms: u32,
    pub last_operator_id_ms: u32,
    pub last_auth_ms: u32,
    pub location_interval_ms: u32,
    pub basic_id_interval_ms: u32,
    pub self_id_interval_ms: u32,
    pub system_interval_ms: u32,
    pub operator_id_interval_ms: u32,
    pub auth_interval_ms: u32,
}

impl OdidTimers {
    pub const fn new() -> Self {
        Self {
            last_location_ms: u32::MAX,
            last_basic_id_ms: u32::MAX,
            last_self_id_ms: u32::MAX,
            last_system_ms: u32::MAX,
            last_operator_id_ms: u32::MAX,
            last_auth_ms: u32::MAX,
            location_interval_ms: 1000,      // 1 Hz required
            basic_id_interval_ms: 3000,       // every 3s
            self_id_interval_ms: 3000,
            system_interval_ms: 3000,
            operator_id_interval_ms: 3000,
            auth_interval_ms: 3000,
        }
    }

    /// Check which message type is due next. Returns the highest-priority
    /// due message, or None if nothing is due.
    pub fn next_due(&self, now_ms: u32) -> BroadcastAction {
        // Location has highest priority (1 Hz required)
        if self.is_due(self.last_location_ms, self.location_interval_ms, now_ms) {
            return BroadcastAction::Location;
        }
        if self.is_due(self.last_basic_id_ms, self.basic_id_interval_ms, now_ms) {
            return BroadcastAction::BasicId;
        }
        if self.is_due(self.last_self_id_ms, self.self_id_interval_ms, now_ms) {
            return BroadcastAction::SelfId;
        }
        if self.is_due(self.last_system_ms, self.system_interval_ms, now_ms) {
            return BroadcastAction::System;
        }
        if self.is_due(self.last_operator_id_ms, self.operator_id_interval_ms, now_ms) {
            return BroadcastAction::OperatorId;
        }
        if self.is_due(self.last_auth_ms, self.auth_interval_ms, now_ms) {
            return BroadcastAction::Authentication;
        }
        BroadcastAction::None
    }

    /// Mark a message type as sent.
    pub fn mark_sent(&mut self, action: BroadcastAction, now_ms: u32) {
        match action {
            BroadcastAction::Location => self.last_location_ms = now_ms,
            BroadcastAction::BasicId => self.last_basic_id_ms = now_ms,
            BroadcastAction::SelfId => self.last_self_id_ms = now_ms,
            BroadcastAction::System => self.last_system_ms = now_ms,
            BroadcastAction::OperatorId => self.last_operator_id_ms = now_ms,
            BroadcastAction::Authentication => self.last_auth_ms = now_ms,
            BroadcastAction::None => {}
        }
    }

    fn is_due(&self, last: u32, interval: u32, now: u32) -> bool {
        last == u32::MAX || now.wrapping_sub(last) >= interval
    }
}

// ─── D5: UTC timestamp source dependency ───

/// D5: UTC timestamp source for ODID messages.
///
/// OpenDroneID requires UTC timestamps. This tracks whether a valid
/// UTC source is available (GPS time, MAVLink SYSTEM_TIME, or RTC).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UtcSource {
    /// No UTC source available — timestamps are invalid.
    None,
    /// UTC from GPS time-of-week.
    Gps,
    /// UTC from MAVLink SYSTEM_TIME.
    Mavlink,
    /// UTC from hardware RTC.
    Rtc,
}

/// D5: UTC timestamp state for ODID.
#[derive(Debug, Clone, Copy)]
pub struct OdidUtcState {
    pub source: UtcSource,
    /// Current Unix timestamp in seconds (0 = invalid).
    pub unix_secs: u32,
    /// Seconds since start of current hour (for Location message timestamp field).
    pub secs_since_hour: f32,
}

impl OdidUtcState {
    pub const fn new() -> Self {
        Self {
            source: UtcSource::None,
            unix_secs: 0,
            secs_since_hour: 0.0,
        }
    }

    /// Update from a Unix timestamp.
    pub fn set_unix(&mut self, unix_secs: u32, source: UtcSource) {
        self.source = source;
        self.unix_secs = unix_secs;
        // Compute seconds since hour: unix_secs mod 3600
        self.secs_since_hour = (unix_secs % 3600) as f32;
    }

    /// Whether a valid UTC source is available.
    pub fn is_valid(&self) -> bool {
        self.source != UtcSource::None && self.unix_secs > 0
    }
}

// ─── Option Bits ───

/// Option bits controlling ODID behavior.
#[derive(Debug, Clone, Copy)]
pub struct OdidOptions {
    /// Whether pre-arm transponder check is enforced (hard block on arming).
    pub enforce_prearm_checks: bool,
    /// Allow broadcasting location without a GPS fix.
    pub allow_non_gps_position: bool,
    /// Lock UAS ID on first BasicID reception from transponder.
    pub lock_uas_id_on_first_rx: bool,
}

impl Default for OdidOptions {
    fn default() -> Self {
        Self {
            enforce_prearm_checks: true,
            allow_non_gps_position: false,
            lock_uas_id_on_first_rx: false,
        }
    }
}

/// Broadcast transport type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OdidTransport {
    /// MAVLink serial relay to connected transponder module.
    MavlinkSerial,
    /// DroneCAN broadcast.
    DroneCan,
}

// ─── Broadcast Manager ───

/// OpenDroneID broadcast state machine.
pub struct OpenDroneId {
    pub basic_id: BasicIdMessage,
    pub self_id: SelfIdMessage,
    pub operator_id: OperatorIdMessage,
    pub system: SystemMessage,
    /// D1: Authentication message (ASTM F3411-22a mandatory).
    pub authentication: AuthenticationMessage,

    // Broadcast timing (legacy round-robin, kept for backward compat)
    location_interval_ms: u32,  // 1000ms (1Hz required)
    static_interval_ms: u32,    // 3000ms round-robin for BasicID/SelfID/System/OperatorID
    last_location_ms: u32,
    last_static_ms: u32,
    static_msg_index: u8,

    /// D3: Per-type broadcast timers (preferred over round-robin).
    pub timers: OdidTimers,

    /// D5: UTC timestamp state.
    pub utc: OdidUtcState,

    // Transponder handshake
    transponder_connected: bool,
    transponder_arm_status_ok: bool,

    /// Option bits.
    pub options: OdidOptions,
    /// Active transport(s).
    pub transport: OdidTransport,
    /// Whether UAS ID has been locked (persistent across power cycles via param system).
    pub uas_id_locked: bool,
    /// Persistent UAS ID storage (survives reboot — caller saves/loads from param system).
    pub persistent_uas_id: Option<String<20>>,
}

/// What message to broadcast next.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BroadcastAction {
    None,
    Location,
    BasicId,
    SelfId,
    System,
    OperatorId,
    Authentication,
}

impl OpenDroneId {
    pub fn new() -> Self {
        Self {
            basic_id: BasicIdMessage {
                id_type: IdType::SerialNumber,
                ua_type: UaType::HelicopterOrMultirotor,
                uas_id: String::new(),
            },
            self_id: SelfIdMessage {
                description_type: 0,
                description: String::new(),
            },
            operator_id: OperatorIdMessage {
                operator_id_type: 0,
                operator_id: String::new(),
            },
            system: SystemMessage {
                operator_location_type: OperatorLocationType::Takeoff,
                operator_latitude: 0.0, operator_longitude: 0.0,
                operator_altitude_geo: 0.0,
                area_count: 1, area_radius: 0, area_ceiling: 0.0, area_floor: 0.0,
                classification_type: 0, timestamp: 0,
            },
            authentication: AuthenticationMessage::new(),
            location_interval_ms: 1000,
            static_interval_ms: 3000,
            last_location_ms: u32::MAX, // NEVER_SENT — triggers on first call
            last_static_ms: u32::MAX,
            static_msg_index: 0,
            timers: OdidTimers::new(),
            utc: OdidUtcState::new(),
            transponder_connected: false,
            transponder_arm_status_ok: false,
            options: OdidOptions::default(),
            transport: OdidTransport::MavlinkSerial,
            uas_id_locked: false,
            persistent_uas_id: None,
        }
    }

    /// Set the UAS serial number / registration ID.
    pub fn set_uas_id(&mut self, id: &str) {
        self.basic_id.uas_id.clear();
        let _ = self.basic_id.uas_id.push_str(id);
    }

    /// Set operator location (captured at takeoff or from GCS).
    pub fn set_operator_location(&mut self, lat: f64, lon: f64, alt: f32) {
        self.system.operator_latitude = lat;
        self.system.operator_longitude = lon;
        self.system.operator_altitude_geo = alt;
    }

    /// Update transponder handshake status.
    pub fn set_transponder_status(&mut self, connected: bool, arm_ok: bool) {
        self.transponder_connected = connected;
        self.transponder_arm_status_ok = arm_ok;
    }

    /// Pre-arm check: transponder must be connected and report arm_status OK.
    /// If `enforce_prearm_checks` option is false, always passes.
    pub fn prearm_ok(&self) -> bool {
        if !self.options.enforce_prearm_checks {
            return true;
        }
        self.transponder_connected && self.transponder_arm_status_ok
    }

    /// Load persistent UAS ID from param storage. Call at startup.
    pub fn load_persistent_id(&mut self, id: &str) {
        let mut s = String::new();
        let _ = s.push_str(id);
        self.persistent_uas_id = Some(s.clone());
        if !self.uas_id_locked {
            self.basic_id.uas_id = s;
        }
    }

    /// Lock the UAS ID (prevents further changes).
    pub fn lock_uas_id(&mut self) {
        self.uas_id_locked = true;
        self.persistent_uas_id = Some(self.basic_id.uas_id.clone());
    }

    /// Determine what to broadcast next based on timing.
    pub fn update(&mut self, now_ms: u32) -> BroadcastAction {
        // Use per-type independent timers (Doll Round 2 fix — F3411-22a compliant)
        let action = self.timers.next_due(now_ms);
        match action {
            BroadcastAction::Location => { self.timers.last_location_ms = now_ms; }
            BroadcastAction::BasicId => { self.timers.last_basic_id_ms = now_ms; }
            BroadcastAction::SelfId => { self.timers.last_self_id_ms = now_ms; }
            BroadcastAction::System => { self.timers.last_system_ms = now_ms; }
            BroadcastAction::OperatorId => { self.timers.last_operator_id_ms = now_ms; }
            BroadcastAction::Authentication => { self.timers.last_auth_ms = now_ms; }
            BroadcastAction::None => {}
        }
        action
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_broadcast_location_at_1hz() {
        let mut odid = OpenDroneId::new();
        let action = odid.update(0);
        assert_eq!(action, BroadcastAction::Location);

        let action = odid.update(500);
        assert_ne!(action, BroadcastAction::Location, "Should not send at 500ms");

        let action = odid.update(1000);
        assert_eq!(action, BroadcastAction::Location);
    }

    #[test]
    fn test_static_roundrobin() {
        let mut odid = OpenDroneId::new();
        odid.update(0); // location
        odid.update(1000); // location again

        // At 3000ms, static should fire
        let action = odid.update(3000);
        // Either location (1Hz) or static — location wins at exactly 3000
        // At 3001ms:
        let action = odid.update(3001);
        assert!(matches!(action, BroadcastAction::BasicId | BroadcastAction::None));
    }

    #[test]
    fn test_prearm_requires_transponder() {
        let mut odid = OpenDroneId::new();
        assert!(!odid.prearm_ok(), "No transponder → fail");

        odid.set_transponder_status(true, false);
        assert!(!odid.prearm_ok(), "Connected but not arm_ok → fail");

        odid.set_transponder_status(true, true);
        assert!(odid.prearm_ok(), "Connected + arm_ok → pass");
    }

    #[test]
    fn test_set_uas_id() {
        let mut odid = OpenDroneId::new();
        odid.set_uas_id("MERIDIAN-001-TEST");
        assert_eq!(odid.basic_id.uas_id.as_str(), "MERIDIAN-001-TEST");
    }

    #[test]
    fn test_operator_location() {
        let mut odid = OpenDroneId::new();
        odid.set_operator_location(35.0, -120.0, 50.0);
        assert!((odid.system.operator_latitude - 35.0).abs() < 0.001);
        assert!((odid.system.operator_longitude - (-120.0)).abs() < 0.001);
    }

    #[test]
    fn test_location_message_fields() {
        let loc = LocationMessage {
            status: 2, // airborne
            direction: 27000, // 270.00°
            speed_horizontal: 1500, // 15.00 m/s
            speed_vertical: -200, // -2.00 m/s (descending)
            latitude: 35.0, longitude: -120.0,
            altitude_baro: 100.0, altitude_geo: 102.0,
            height_agl: 50.0,
            accuracy_h: 7, accuracy_v: 5, accuracy_speed: 3,
            timestamp: 1234.5,
        };
        assert_eq!(loc.status, 2);
        assert_eq!(loc.direction, 27000);
    }
}

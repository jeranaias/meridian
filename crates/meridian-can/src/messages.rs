//! Standard DroneCAN (UAVCAN v0) message type encoding/decoding.
//!
//! Each message type has a fixed data type ID and a known wire format.
//! We implement encode/decode for the most important messages used in
//! the ArduPilot/DroneCAN ecosystem.

// ---------------------------------------------------------------------------
// Data type IDs
// ---------------------------------------------------------------------------

/// `uavcan.protocol.NodeStatus` — broadcast at 1 Hz by every node.
pub const NODESTATUS_DTID: u16 = 341;

/// `uavcan.protocol.GetNodeInfo` — service request/response.
pub const GETNODEINFO_DTID: u16 = 1;

/// `uavcan.equipment.gnss.Fix2` — GPS position fix.
pub const FIX2_DTID: u16 = 1063;

/// `uavcan.equipment.ahrs.MagneticFieldStrength2` — magnetometer.
pub const MAGNETIC_FIELD_DTID: u16 = 1002;

/// `uavcan.equipment.air_data.RawAirData` — barometric / airspeed.
pub const RAW_AIR_DATA_DTID: u16 = 1027;

/// `uavcan.equipment.esc.RawCommand` — ESC throttle commands.
pub const RAW_COMMAND_DTID: u16 = 1030;

/// `uavcan.equipment.power.BatteryInfo` — battery state.
pub const BATTERY_INFO_DTID: u16 = 1092;

/// `uavcan.equipment.indication.LightsCommand` — LED commands.
pub const LIGHTS_COMMAND_DTID: u16 = 1081;

/// `ardupilot.indication.SafetyState` — safety switch state.
pub const SAFETY_STATE_DTID: u16 = 20000;

/// `uavcan.equipment.safety.ArmingStatus` — arm/disarm state.
pub const ARMING_STATUS_DTID: u16 = 1100;

/// `uavcan.protocol.dynamic_node_id.Allocation` — DNA.
pub const DNA_ALLOCATION_DTID: u16 = 1;

/// `ardupilot.gnss.Status` — ArduPilot GNSS health.
pub const AP_GNSS_STATUS_DTID: u16 = 20003;

// ---------------------------------------------------------------------------
// NodeStatus (341) — 7 bytes
// ---------------------------------------------------------------------------

/// Node health status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NodeHealth {
    Ok = 0,
    Warning = 1,
    Error = 2,
    Critical = 3,
}

/// Node operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum NodeMode {
    Operational = 0,
    Initialization = 1,
    Maintenance = 2,
    SoftwareUpdate = 3,
    Offline = 7,
}

/// `uavcan.protocol.NodeStatus` — 7 bytes on the wire.
///
/// Wire format (little-endian):
///   - uptime_sec: u32 (4 bytes)
///   - health: u2 + mode: u3 + sub_mode: u3 → packed into 1 byte
///   - vendor_specific_status_code: u16 (2 bytes)
#[derive(Debug, Clone, Copy)]
pub struct NodeStatus {
    pub uptime_sec: u32,
    pub health: NodeHealth,
    pub mode: NodeMode,
    pub sub_mode: u8,
    pub vendor_status: u16,
}

impl NodeStatus {
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 7 {
            return 0;
        }
        let uptime = self.uptime_sec.to_le_bytes();
        buf[0] = uptime[0];
        buf[1] = uptime[1];
        buf[2] = uptime[2];
        buf[3] = uptime[3];
        buf[4] = ((self.health as u8) & 0x03) << 6
            | ((self.mode as u8) & 0x07) << 3
            | (self.sub_mode & 0x07);
        let vs = self.vendor_status.to_le_bytes();
        buf[5] = vs[0];
        buf[6] = vs[1];
        7
    }

    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 7 {
            return None;
        }
        let uptime_sec = u32::from_le_bytes([data[0], data[1], data[2], data[3]]);
        let health_raw = (data[4] >> 6) & 0x03;
        let mode_raw = (data[4] >> 3) & 0x07;
        let sub_mode = data[4] & 0x07;
        let vendor_status = u16::from_le_bytes([data[5], data[6]]);

        let health = match health_raw {
            0 => NodeHealth::Ok,
            1 => NodeHealth::Warning,
            2 => NodeHealth::Error,
            _ => NodeHealth::Critical,
        };
        let mode = match mode_raw {
            0 => NodeMode::Operational,
            1 => NodeMode::Initialization,
            2 => NodeMode::Maintenance,
            3 => NodeMode::SoftwareUpdate,
            _ => NodeMode::Offline,
        };

        Some(NodeStatus {
            uptime_sec,
            health,
            mode,
            sub_mode,
            vendor_status,
        })
    }
}

// ---------------------------------------------------------------------------
// GetNodeInfo (service 1) — response
// ---------------------------------------------------------------------------

/// Software version information from `GetNodeInfo` response.
#[derive(Debug, Clone, Copy)]
pub struct SoftwareVersion {
    pub major: u8,
    pub minor: u8,
    pub vcs_commit: u32,
    pub image_crc: u64,
}

/// Hardware version information from `GetNodeInfo` response.
#[derive(Debug, Clone, Copy)]
pub struct HardwareVersion {
    pub major: u8,
    pub minor: u8,
    pub unique_id: [u8; 16],
}

/// `uavcan.protocol.GetNodeInfo` response.
#[derive(Debug, Clone, Copy)]
pub struct GetNodeInfoResponse {
    pub status: NodeStatus,
    pub software_version: SoftwareVersion,
    pub hardware_version: HardwareVersion,
    pub name_len: u8,
    pub name: [u8; 80],
}

impl GetNodeInfoResponse {
    /// Encode into a buffer. Returns number of bytes written.
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 41 {
            return 0;
        }
        let mut pos = 0;

        // NodeStatus (7 bytes)
        pos += self.status.encode(&mut buf[pos..]);

        // Software version (15 bytes)
        buf[pos] = self.software_version.major; pos += 1;
        buf[pos] = self.software_version.minor; pos += 1;
        buf[pos] = 1; pos += 1; // optional field flags: vcs_commit present
        let vcs = self.software_version.vcs_commit.to_le_bytes();
        buf[pos..pos + 4].copy_from_slice(&vcs); pos += 4;
        let crc = self.software_version.image_crc.to_le_bytes();
        buf[pos..pos + 8].copy_from_slice(&crc); pos += 8;

        // Hardware version (18 bytes)
        buf[pos] = self.hardware_version.major; pos += 1;
        buf[pos] = self.hardware_version.minor; pos += 1;
        buf[pos..pos + 16].copy_from_slice(&self.hardware_version.unique_id); pos += 16;

        // Name (dynamic length)
        let name_len = self.name_len.min(80) as usize;
        if pos + name_len <= buf.len() {
            buf[pos..pos + name_len].copy_from_slice(&self.name[..name_len]);
            pos += name_len;
        }

        pos
    }
}

// ---------------------------------------------------------------------------
// Fix2 (1063) — GPS position fix
// ---------------------------------------------------------------------------

/// GPS fix status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GnssFixStatus {
    NoFix = 0,
    Fix2D = 1,
    Fix3D = 2,
}

/// GPS fix mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GnssFixMode {
    Single = 0,
    Dgps = 1,
    Rtk = 2,
}

/// GPS sub-mode for RTK.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GnssSubMode {
    DgpsOther = 0,
    RtkFloat = 1,
    RtkFixed = 2,
}

/// `uavcan.equipment.gnss.Fix2` — GPS fix data.
///
/// Simplified encoding focusing on fields Meridian needs. Wire format (little-endian):
///   - timestamp_usec: u64 (8 bytes)
///   - longitude_deg_1e8: i64 (8 bytes)
///   - latitude_deg_1e8: i64 (8 bytes)
///   - height_ellipsoid_mm: i32 (4 bytes)
///   - height_msl_mm: i32 (4 bytes)
///   - ned_velocity: [f32; 3] (12 bytes)
///   - sats_used: u8 (1 byte)
///   - status: u8 (1 byte)
///   - mode: u8 (1 byte)
///   - sub_mode: u8 (1 byte)
///   - covariance: [f32; 6] (24 bytes) — position XYZ + velocity XYZ diagonal
/// Total: 72 bytes (multi-frame transfer)
#[derive(Debug, Clone, Copy)]
pub struct Fix2 {
    pub timestamp_usec: u64,
    pub longitude_deg_1e8: i64,
    pub latitude_deg_1e8: i64,
    pub height_ellipsoid_mm: i32,
    pub height_msl_mm: i32,
    pub ned_velocity: [f32; 3],
    pub sats_used: u8,
    pub status: GnssFixStatus,
    pub mode: GnssFixMode,
    pub sub_mode: GnssSubMode,
    pub covariance: [f32; 6],
}

impl Fix2 {
    /// Encode into a buffer. Returns number of bytes written.
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 72 {
            return 0;
        }
        let mut pos = 0;

        buf[pos..pos + 8].copy_from_slice(&self.timestamp_usec.to_le_bytes()); pos += 8;
        buf[pos..pos + 8].copy_from_slice(&self.longitude_deg_1e8.to_le_bytes()); pos += 8;
        buf[pos..pos + 8].copy_from_slice(&self.latitude_deg_1e8.to_le_bytes()); pos += 8;
        buf[pos..pos + 4].copy_from_slice(&self.height_ellipsoid_mm.to_le_bytes()); pos += 4;
        buf[pos..pos + 4].copy_from_slice(&self.height_msl_mm.to_le_bytes()); pos += 4;

        for v in &self.ned_velocity {
            buf[pos..pos + 4].copy_from_slice(&v.to_le_bytes()); pos += 4;
        }

        buf[pos] = self.sats_used; pos += 1;
        buf[pos] = self.status as u8; pos += 1;
        buf[pos] = self.mode as u8; pos += 1;
        buf[pos] = self.sub_mode as u8; pos += 1;

        for c in &self.covariance {
            buf[pos..pos + 4].copy_from_slice(&c.to_le_bytes()); pos += 4;
        }

        pos
    }

    /// Decode from a buffer.
    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 72 {
            return None;
        }
        let mut pos = 0;

        let timestamp_usec = u64::from_le_bytes(data[pos..pos + 8].try_into().ok()?); pos += 8;
        let longitude_deg_1e8 = i64::from_le_bytes(data[pos..pos + 8].try_into().ok()?); pos += 8;
        let latitude_deg_1e8 = i64::from_le_bytes(data[pos..pos + 8].try_into().ok()?); pos += 8;
        let height_ellipsoid_mm = i32::from_le_bytes(data[pos..pos + 4].try_into().ok()?); pos += 4;
        let height_msl_mm = i32::from_le_bytes(data[pos..pos + 4].try_into().ok()?); pos += 4;

        let mut ned_velocity = [0.0f32; 3];
        for v in &mut ned_velocity {
            *v = f32::from_le_bytes(data[pos..pos + 4].try_into().ok()?); pos += 4;
        }

        let sats_used = data[pos]; pos += 1;
        let status = match data[pos] {
            0 => GnssFixStatus::NoFix,
            1 => GnssFixStatus::Fix2D,
            _ => GnssFixStatus::Fix3D,
        }; pos += 1;
        let mode = match data[pos] {
            0 => GnssFixMode::Single,
            1 => GnssFixMode::Dgps,
            _ => GnssFixMode::Rtk,
        }; pos += 1;
        let sub_mode = match data[pos] {
            0 => GnssSubMode::DgpsOther,
            1 => GnssSubMode::RtkFloat,
            _ => GnssSubMode::RtkFixed,
        }; pos += 1;

        let mut covariance = [0.0f32; 6];
        for c in &mut covariance {
            *c = f32::from_le_bytes(data[pos..pos + 4].try_into().ok()?); pos += 4;
        }

        Some(Fix2 {
            timestamp_usec,
            longitude_deg_1e8,
            latitude_deg_1e8,
            height_ellipsoid_mm,
            height_msl_mm,
            ned_velocity,
            sats_used,
            status,
            mode,
            sub_mode,
            covariance,
        })
    }
}

// ---------------------------------------------------------------------------
// MagneticFieldStrength2 (1002) — 3-axis magnetometer
// ---------------------------------------------------------------------------

/// `uavcan.equipment.ahrs.MagneticFieldStrength2` — compass data.
///
/// Wire format (little-endian):
///   - sensor_id: u8 (1 byte)
///   - magnetic_field_ga: [f16; 3] (6 bytes) — Gauss, float16
/// Total: 7 bytes (single frame)
///
/// We store as f32 internally and convert to/from f16 on the wire.
#[derive(Debug, Clone, Copy)]
pub struct MagneticFieldStrength2 {
    pub sensor_id: u8,
    pub magnetic_field_ga: [f32; 3],
}

impl MagneticFieldStrength2 {
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 7 {
            return 0;
        }
        buf[0] = self.sensor_id;
        for i in 0..3 {
            let h = f16_from_f32(self.magnetic_field_ga[i]);
            let bytes = h.to_le_bytes();
            buf[1 + i * 2] = bytes[0];
            buf[2 + i * 2] = bytes[1];
        }
        7
    }

    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 7 {
            return None;
        }
        let sensor_id = data[0];
        let mut mag = [0.0f32; 3];
        for i in 0..3 {
            let h = u16::from_le_bytes([data[1 + i * 2], data[2 + i * 2]]);
            mag[i] = f16_to_f32(h);
        }
        Some(MagneticFieldStrength2 {
            sensor_id,
            magnetic_field_ga: mag,
        })
    }
}

// ---------------------------------------------------------------------------
// RawAirData (1027) — barometric pressure + temperature
// ---------------------------------------------------------------------------

/// `uavcan.equipment.air_data.RawAirData` — barometric/airspeed data.
///
/// Simplified wire format (little-endian):
///   - static_pressure: f32 (4 bytes) — Pa
///   - differential_pressure: f32 (4 bytes) — Pa
///   - static_air_temperature: f32 (4 bytes) — Kelvin
///   - pitot_temperature: f32 (4 bytes) — Kelvin
///   - static_pressure_sensor_temp: f32 (4 bytes)
///   - differential_pressure_sensor_temp: f32 (4 bytes)
/// Total: 24 bytes (multi-frame)
#[derive(Debug, Clone, Copy)]
pub struct RawAirData {
    pub static_pressure: f32,
    pub differential_pressure: f32,
    pub static_air_temperature: f32,
    pub pitot_temperature: f32,
}

impl RawAirData {
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 24 {
            return 0;
        }
        let mut pos = 0;
        buf[pos..pos + 4].copy_from_slice(&self.static_pressure.to_le_bytes()); pos += 4;
        buf[pos..pos + 4].copy_from_slice(&self.differential_pressure.to_le_bytes()); pos += 4;
        buf[pos..pos + 4].copy_from_slice(&self.static_air_temperature.to_le_bytes()); pos += 4;
        buf[pos..pos + 4].copy_from_slice(&self.pitot_temperature.to_le_bytes()); pos += 4;
        // sensor temperatures — set to NaN (unused)
        let nan_bytes = f32::NAN.to_le_bytes();
        buf[pos..pos + 4].copy_from_slice(&nan_bytes); pos += 4;
        buf[pos..pos + 4].copy_from_slice(&nan_bytes); pos += 4;
        pos
    }

    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 16 {
            return None;
        }
        let static_pressure = f32::from_le_bytes(data[0..4].try_into().ok()?);
        let differential_pressure = f32::from_le_bytes(data[4..8].try_into().ok()?);
        let static_air_temperature = f32::from_le_bytes(data[8..12].try_into().ok()?);
        let pitot_temperature = f32::from_le_bytes(data[12..16].try_into().ok()?);

        Some(RawAirData {
            static_pressure,
            differential_pressure,
            static_air_temperature,
            pitot_temperature,
        })
    }
}

// ---------------------------------------------------------------------------
// RawCommand (1030) — ESC throttle commands
// ---------------------------------------------------------------------------

/// `uavcan.equipment.esc.RawCommand` — ESC throttle array.
///
/// Wire format: array of int14 values packed sequentially.
/// Each value is 14 bits, range -8192 to +8191.
/// The values are packed bit-by-bit (DSDL dynamic array of int14).
///
/// For simplicity, we encode up to 8 ESCs. The packing is little-endian
/// bit order within bytes.
#[derive(Debug, Clone, Copy)]
pub struct RawCommand {
    pub commands: [i16; 8],
    pub num_commands: u8,
}

impl RawCommand {
    /// Encode ESC commands into a buffer. Returns bytes written.
    ///
    /// Each command is 14 bits packed sequentially in little-endian bit order.
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        let n = self.num_commands.min(8) as usize;
        let total_bits = n * 14;
        let total_bytes = (total_bits + 7) / 8;
        if buf.len() < total_bytes {
            return 0;
        }

        // Zero the output buffer
        for b in buf[..total_bytes].iter_mut() {
            *b = 0;
        }

        let mut bit_offset = 0usize;
        for i in 0..n {
            // Convert i16 to 14-bit two's complement
            let val = self.commands[i].max(-8192).min(8191);
            let raw = (val as u16) & 0x3FFF; // mask to 14 bits

            // Pack 14 bits at bit_offset in little-endian bit order
            for bit in 0..14 {
                if raw & (1 << bit) != 0 {
                    let target_bit = bit_offset + bit;
                    let byte_idx = target_bit / 8;
                    let bit_idx = target_bit % 8;
                    buf[byte_idx] |= 1 << bit_idx;
                }
            }
            bit_offset += 14;
        }

        total_bytes
    }

    /// Decode ESC commands from a buffer.
    pub fn decode(data: &[u8], num_commands: u8) -> Option<Self> {
        let n = num_commands.min(8) as usize;
        let total_bits = n * 14;
        let total_bytes = (total_bits + 7) / 8;
        if data.len() < total_bytes {
            return None;
        }

        let mut commands = [0i16; 8];
        let mut bit_offset = 0usize;

        for cmd in commands.iter_mut().take(n) {
            let mut raw: u16 = 0;
            for bit in 0..14 {
                let target_bit = bit_offset + bit;
                let byte_idx = target_bit / 8;
                let bit_idx = target_bit % 8;
                if data[byte_idx] & (1 << bit_idx) != 0 {
                    raw |= 1 << bit;
                }
            }
            // Sign-extend from 14 bits
            if raw & 0x2000 != 0 {
                *cmd = (raw | 0xC000) as i16;
            } else {
                *cmd = raw as i16;
            }
            bit_offset += 14;
        }

        Some(RawCommand {
            commands,
            num_commands,
        })
    }
}

// ---------------------------------------------------------------------------
// BatteryInfo (1092) — battery state
// ---------------------------------------------------------------------------

/// `uavcan.equipment.power.BatteryInfo` — battery data.
///
/// Simplified wire format (little-endian):
///   - temperature: f32 (4 bytes) — Kelvin
///   - voltage: f32 (4 bytes) — V
///   - current: f32 (4 bytes) — A (positive = charging)
///   - full_charge_capacity_wh: f32 (4 bytes)
///   - remaining_capacity_wh: f32 (4 bytes)
///   - state_of_charge_pct: u8 (1 byte) — 0-100
///   - state_of_health_pct: u8 (1 byte) — 0-100 or 127=unknown
///   - status_flags: u16 (2 bytes)
///   - battery_id: u8 (1 byte)
///   - model_instance_id: u8 (1 byte)
/// Total: 26 bytes (multi-frame)
#[derive(Debug, Clone, Copy)]
pub struct BatteryInfo {
    pub temperature_k: f32,
    pub voltage: f32,
    pub current: f32,
    pub full_charge_capacity_wh: f32,
    pub remaining_capacity_wh: f32,
    pub state_of_charge_pct: u8,
    pub state_of_health_pct: u8,
    pub battery_id: u8,
    pub model_instance_id: u8,
}

impl BatteryInfo {
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 26 {
            return 0;
        }
        let mut pos = 0;
        buf[pos..pos + 4].copy_from_slice(&self.temperature_k.to_le_bytes()); pos += 4;
        buf[pos..pos + 4].copy_from_slice(&self.voltage.to_le_bytes()); pos += 4;
        buf[pos..pos + 4].copy_from_slice(&self.current.to_le_bytes()); pos += 4;
        buf[pos..pos + 4].copy_from_slice(&self.full_charge_capacity_wh.to_le_bytes()); pos += 4;
        buf[pos..pos + 4].copy_from_slice(&self.remaining_capacity_wh.to_le_bytes()); pos += 4;
        buf[pos] = self.state_of_charge_pct; pos += 1;
        buf[pos] = self.state_of_health_pct; pos += 1;
        buf[pos..pos + 2].copy_from_slice(&0u16.to_le_bytes()); pos += 2; // status_flags
        buf[pos] = self.battery_id; pos += 1;
        buf[pos] = self.model_instance_id; pos += 1;
        pos
    }

    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 26 {
            return None;
        }
        let mut pos = 0;
        let temperature_k = f32::from_le_bytes(data[pos..pos + 4].try_into().ok()?); pos += 4;
        let voltage = f32::from_le_bytes(data[pos..pos + 4].try_into().ok()?); pos += 4;
        let current = f32::from_le_bytes(data[pos..pos + 4].try_into().ok()?); pos += 4;
        let full_charge_capacity_wh = f32::from_le_bytes(data[pos..pos + 4].try_into().ok()?); pos += 4;
        let remaining_capacity_wh = f32::from_le_bytes(data[pos..pos + 4].try_into().ok()?); pos += 4;
        let state_of_charge_pct = data[pos]; pos += 1;
        let state_of_health_pct = data[pos]; pos += 1;
        pos += 2; // status_flags
        let battery_id = data[pos]; pos += 1;
        let model_instance_id = data[pos];

        Some(BatteryInfo {
            temperature_k,
            voltage,
            current,
            full_charge_capacity_wh,
            remaining_capacity_wh,
            state_of_charge_pct,
            state_of_health_pct,
            battery_id,
            model_instance_id,
        })
    }
}

// ---------------------------------------------------------------------------
// LightsCommand (1081) — LED color commands
// ---------------------------------------------------------------------------

/// Single light command entry.
#[derive(Debug, Clone, Copy)]
pub struct SingleLightCommand {
    pub light_id: u8,
    pub red: u8,    // 0-31 (5-bit)
    pub green: u8,  // 0-31
    pub blue: u8,   // 0-31
}

/// `uavcan.equipment.indication.LightsCommand` — LED array.
///
/// Wire format: array of (light_id: u8, color: packed RGB555).
/// Each entry = 3 bytes: light_id(1) + color(2) where color packs R5:G5:B5 + 1 unused bit.
#[derive(Debug, Clone, Copy)]
pub struct LightsCommand {
    pub commands: [SingleLightCommand; 4],
    pub num_commands: u8,
}

impl LightsCommand {
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        let n = self.num_commands.min(4) as usize;
        let needed = n * 3;
        if buf.len() < needed {
            return 0;
        }
        for i in 0..n {
            let c = &self.commands[i];
            buf[i * 3] = c.light_id;
            let color: u16 = ((c.red as u16) & 0x1F)
                | (((c.green as u16) & 0x1F) << 5)
                | (((c.blue as u16) & 0x1F) << 10);
            let cb = color.to_le_bytes();
            buf[i * 3 + 1] = cb[0];
            buf[i * 3 + 2] = cb[1];
        }
        needed
    }

    pub fn decode(data: &[u8]) -> Option<Self> {
        let n = data.len() / 3;
        if n == 0 {
            return None;
        }
        let n = n.min(4);
        let mut commands = [SingleLightCommand { light_id: 0, red: 0, green: 0, blue: 0 }; 4];
        for i in 0..n {
            commands[i].light_id = data[i * 3];
            let color = u16::from_le_bytes([data[i * 3 + 1], data[i * 3 + 2]]);
            commands[i].red = (color & 0x1F) as u8;
            commands[i].green = ((color >> 5) & 0x1F) as u8;
            commands[i].blue = ((color >> 10) & 0x1F) as u8;
        }
        Some(LightsCommand {
            commands,
            num_commands: n as u8,
        })
    }
}

// ---------------------------------------------------------------------------
// SafetyState — ardupilot.indication.SafetyState
// ---------------------------------------------------------------------------

/// Safety switch status.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SafetyStatus {
    /// Safety on — motors locked.
    SafetyOn = 0,
    /// Safety off — motors can spin.
    SafetyOff = 255,
}

/// `ardupilot.indication.SafetyState` — 1 byte.
#[derive(Debug, Clone, Copy)]
pub struct SafetyState {
    pub status: SafetyStatus,
}

impl SafetyState {
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        if buf.is_empty() { return 0; }
        buf[0] = self.status as u8;
        1
    }

    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.is_empty() { return None; }
        let status = if data[0] == 0 { SafetyStatus::SafetyOn } else { SafetyStatus::SafetyOff };
        Some(SafetyState { status })
    }
}

// ---------------------------------------------------------------------------
// ArmingStatus (1100)
// ---------------------------------------------------------------------------

/// Arming status values.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ArmingState {
    Disarmed = 0,
    Armed = 1,
}

/// `uavcan.equipment.safety.ArmingStatus` — 1 byte.
#[derive(Debug, Clone, Copy)]
pub struct ArmingStatus {
    pub status: ArmingState,
}

impl ArmingStatus {
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        if buf.is_empty() { return 0; }
        buf[0] = self.status as u8;
        1
    }

    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.is_empty() { return None; }
        let status = if data[0] == 0 { ArmingState::Disarmed } else { ArmingState::Armed };
        Some(ArmingStatus { status })
    }
}

// ---------------------------------------------------------------------------
// ESC Status (1034) — ESC telemetry feedback
// ---------------------------------------------------------------------------

/// `uavcan.equipment.esc.Status` — ESC telemetry.
pub const ESC_STATUS_DTID: u16 = 1034;

#[derive(Debug, Clone, Copy)]
pub struct EscStatus {
    pub error_count: u32,
    pub voltage: f32,
    pub current: f32,
    pub temperature: f32,
    pub rpm: i32,
    pub power_rating_pct: u8,
    pub esc_index: u8,
}

impl EscStatus {
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        if buf.len() < 22 { return 0; }
        let mut pos = 0;
        buf[pos..pos + 4].copy_from_slice(&self.error_count.to_le_bytes()); pos += 4;
        buf[pos..pos + 4].copy_from_slice(&self.voltage.to_le_bytes()); pos += 4;
        buf[pos..pos + 4].copy_from_slice(&self.current.to_le_bytes()); pos += 4;
        buf[pos..pos + 4].copy_from_slice(&self.temperature.to_le_bytes()); pos += 4;
        buf[pos..pos + 4].copy_from_slice(&self.rpm.to_le_bytes()); pos += 4;
        buf[pos] = self.power_rating_pct; pos += 1;
        buf[pos] = self.esc_index; pos += 1;
        pos
    }

    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 22 { return None; }
        let mut pos = 0;
        let error_count = u32::from_le_bytes(data[pos..pos + 4].try_into().ok()?); pos += 4;
        let voltage = f32::from_le_bytes(data[pos..pos + 4].try_into().ok()?); pos += 4;
        let current = f32::from_le_bytes(data[pos..pos + 4].try_into().ok()?); pos += 4;
        let temperature = f32::from_le_bytes(data[pos..pos + 4].try_into().ok()?); pos += 4;
        let rpm = i32::from_le_bytes(data[pos..pos + 4].try_into().ok()?); pos += 4;
        let power_rating_pct = data[pos]; pos += 1;
        let esc_index = data[pos];
        Some(EscStatus { error_count, voltage, current, temperature, rpm, power_rating_pct, esc_index })
    }
}

// ---------------------------------------------------------------------------
// ActuatorArrayCommand (1010) — generic actuator commands
// ---------------------------------------------------------------------------

/// `uavcan.equipment.actuator.ArrayCommand`
pub const ACTUATOR_ARRAY_COMMAND_DTID: u16 = 1010;

#[derive(Debug, Clone, Copy)]
pub struct ActuatorCommand {
    pub actuator_id: u8,
    pub command_type: u8,
    pub command_value: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct ActuatorArrayCommand {
    pub commands: [ActuatorCommand; 4],
    pub num_commands: u8,
}

impl ActuatorArrayCommand {
    pub fn encode(&self, buf: &mut [u8]) -> usize {
        let n = self.num_commands.min(4) as usize;
        let needed = n * 6;
        if buf.len() < needed { return 0; }
        let mut pos = 0;
        for i in 0..n {
            buf[pos] = self.commands[i].actuator_id; pos += 1;
            buf[pos] = self.commands[i].command_type; pos += 1;
            buf[pos..pos + 4].copy_from_slice(&self.commands[i].command_value.to_le_bytes()); pos += 4;
        }
        pos
    }

    pub fn decode(data: &[u8]) -> Option<Self> {
        let n = (data.len() / 6).min(4);
        if n == 0 { return None; }
        let mut commands = [ActuatorCommand { actuator_id: 0, command_type: 0, command_value: 0.0 }; 4];
        let mut pos = 0;
        for cmd in commands.iter_mut().take(n) {
            if pos + 6 > data.len() { break; }
            cmd.actuator_id = data[pos]; pos += 1;
            cmd.command_type = data[pos]; pos += 1;
            cmd.command_value = f32::from_le_bytes(data[pos..pos + 4].try_into().ok()?); pos += 4;
        }
        Some(ActuatorArrayCommand { commands, num_commands: n as u8 })
    }
}

// ---------------------------------------------------------------------------
// ActuatorStatus (1011) — actuator feedback
// ---------------------------------------------------------------------------

/// `uavcan.equipment.actuator.Status`
pub const ACTUATOR_STATUS_DTID: u16 = 1011;

#[derive(Debug, Clone, Copy)]
pub struct ActuatorStatus {
    pub actuator_id: u8,
    pub position: f32,
    pub force: f32,
    pub speed: f32,
    pub power_rating_pct: u8,
}

impl ActuatorStatus {
    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 14 { return None; }
        let mut pos = 0;
        let actuator_id = data[pos]; pos += 1;
        let position = f32::from_le_bytes(data[pos..pos + 4].try_into().ok()?); pos += 4;
        let force = f32::from_le_bytes(data[pos..pos + 4].try_into().ok()?); pos += 4;
        let speed = f32::from_le_bytes(data[pos..pos + 4].try_into().ok()?); pos += 4;
        let power_rating_pct = data[pos];
        Some(ActuatorStatus { actuator_id, position, force, speed, power_rating_pct })
    }
}

// ---------------------------------------------------------------------------
// RangeSensorMeasurement (1050) — CAN rangefinder
// ---------------------------------------------------------------------------

/// `uavcan.equipment.range_sensor.Measurement`
pub const RANGE_SENSOR_MEASUREMENT_DTID: u16 = 1050;

#[derive(Debug, Clone, Copy)]
pub struct RangeSensorMeasurement {
    pub sensor_id: u8,
    pub range: f32,
    pub field_of_view: f32,
    pub sensor_type: u8, // 0=undefined, 1=sonar, 2=lidar, 3=radar
    pub reading_type: u8, // 0=undefined, 1=valid, 2=too_close, 3=too_far
}

impl RangeSensorMeasurement {
    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 11 { return None; }
        let sensor_id = data[0];
        let range = f32::from_le_bytes(data[1..5].try_into().ok()?);
        let field_of_view = f32::from_le_bytes(data[5..9].try_into().ok()?);
        let sensor_type = data[9];
        let reading_type = data[10];
        Some(RangeSensorMeasurement { sensor_id, range, field_of_view, sensor_type, reading_type })
    }
}

// ---------------------------------------------------------------------------
// CircuitStatus (1091) — power circuit monitoring
// ---------------------------------------------------------------------------

/// `uavcan.equipment.power.CircuitStatus`
pub const CIRCUIT_STATUS_DTID: u16 = 1091;

#[derive(Debug, Clone, Copy)]
pub struct CircuitStatus {
    pub circuit_id: u16,
    pub voltage: f32,
    pub current: f32,
    pub error_flags: u8,
}

impl CircuitStatus {
    pub fn decode(data: &[u8]) -> Option<Self> {
        if data.len() < 11 { return None; }
        let circuit_id = u16::from_le_bytes([data[0], data[1]]);
        let voltage = f32::from_le_bytes(data[2..6].try_into().ok()?);
        let current = f32::from_le_bytes(data[6..10].try_into().ok()?);
        let error_flags = data[10];
        Some(CircuitStatus { circuit_id, voltage, current, error_flags })
    }
}

// ---------------------------------------------------------------------------
// GetSet (service 11) — parameter protocol
// ---------------------------------------------------------------------------

/// `uavcan.protocol.param.GetSet` — parameter read/write service.
pub const PARAM_GETSET_DTID: u16 = 11;

/// `uavcan.protocol.RestartNode` — restart a node.
pub const RESTART_NODE_DTID: u16 = 5;

// ---------------------------------------------------------------------------
// float16 helpers (IEEE 754 half-precision)
// ---------------------------------------------------------------------------

/// Convert f32 to IEEE 754 float16 (u16 representation).
fn f16_from_f32(value: f32) -> u16 {
    let bits = value.to_bits();
    let sign = ((bits >> 16) & 0x8000) as u16;
    let exp = ((bits >> 23) & 0xFF) as i32;
    let frac = bits & 0x007FFFFF;

    if exp == 255 {
        // Inf or NaN
        if frac != 0 {
            return sign | 0x7E00; // NaN
        }
        return sign | 0x7C00; // Inf
    }

    let new_exp = exp - 127 + 15;
    if new_exp >= 31 {
        return sign | 0x7C00; // overflow to Inf
    }
    if new_exp <= 0 {
        // Denormal or zero
        if new_exp < -10 {
            return sign; // too small, round to zero
        }
        let mant = (frac | 0x00800000) >> (1 - new_exp + 13);
        return sign | mant as u16;
    }

    sign | ((new_exp as u16) << 10) | ((frac >> 13) as u16)
}

/// Convert IEEE 754 float16 (u16) to f32.
fn f16_to_f32(h: u16) -> f32 {
    let sign = ((h & 0x8000) as u32) << 16;
    let exp = ((h >> 10) & 0x1F) as u32;
    let frac = (h & 0x03FF) as u32;

    if exp == 0 {
        if frac == 0 {
            return f32::from_bits(sign); // +/- 0
        }
        // Denormal
        let mut e = 1u32;
        let mut f = frac;
        while f & 0x0400 == 0 {
            f <<= 1;
            e += 1;
        }
        f &= 0x03FF;
        let new_exp = (127u32 - 15 + 1).wrapping_sub(e);
        return f32::from_bits(sign | (new_exp << 23) | (f << 13));
    }

    if exp == 31 {
        if frac == 0 {
            return f32::from_bits(sign | 0x7F800000); // Inf
        }
        return f32::from_bits(sign | 0x7FC00000); // NaN
    }

    let new_exp = exp + 127 - 15;
    f32::from_bits(sign | (new_exp << 23) | (frac << 13))
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_node_status_roundtrip() {
        let status = NodeStatus {
            uptime_sec: 12345,
            health: NodeHealth::Ok,
            mode: NodeMode::Operational,
            sub_mode: 0,
            vendor_status: 0,
        };

        let mut buf = [0u8; 7];
        let len = status.encode(&mut buf);
        assert_eq!(len, 7);

        let decoded = NodeStatus::decode(&buf).unwrap();
        assert_eq!(decoded.uptime_sec, 12345);
        assert_eq!(decoded.health, NodeHealth::Ok);
        assert_eq!(decoded.mode, NodeMode::Operational);
        assert_eq!(decoded.sub_mode, 0);
        assert_eq!(decoded.vendor_status, 0);
    }

    #[test]
    fn test_node_status_all_fields() {
        let status = NodeStatus {
            uptime_sec: 0xDEADBEEF,
            health: NodeHealth::Critical,
            mode: NodeMode::SoftwareUpdate,
            sub_mode: 5,
            vendor_status: 0x1234,
        };

        let mut buf = [0u8; 7];
        status.encode(&mut buf);
        let decoded = NodeStatus::decode(&buf).unwrap();
        assert_eq!(decoded.uptime_sec, 0xDEADBEEF);
        assert_eq!(decoded.health, NodeHealth::Critical);
        assert_eq!(decoded.mode, NodeMode::SoftwareUpdate);
        assert_eq!(decoded.sub_mode, 5);
        assert_eq!(decoded.vendor_status, 0x1234);
    }

    #[test]
    fn test_fix2_roundtrip() {
        let fix = Fix2 {
            timestamp_usec: 1_000_000_000,
            longitude_deg_1e8: -11739816200, // ~-117.398162 deg
            latitude_deg_1e8: 3386668700,    // ~33.866687 deg
            height_ellipsoid_mm: 150_000,
            height_msl_mm: 148_500,
            ned_velocity: [0.5, -0.3, 0.1],
            sats_used: 12,
            status: GnssFixStatus::Fix3D,
            mode: GnssFixMode::Rtk,
            sub_mode: GnssSubMode::RtkFixed,
            covariance: [1.0, 1.0, 4.0, 0.01, 0.01, 0.04],
        };

        let mut buf = [0u8; 128];
        let len = fix.encode(&mut buf);
        assert_eq!(len, 72);

        let decoded = Fix2::decode(&buf[..len]).unwrap();
        assert_eq!(decoded.timestamp_usec, fix.timestamp_usec);
        assert_eq!(decoded.longitude_deg_1e8, fix.longitude_deg_1e8);
        assert_eq!(decoded.latitude_deg_1e8, fix.latitude_deg_1e8);
        assert_eq!(decoded.height_ellipsoid_mm, fix.height_ellipsoid_mm);
        assert_eq!(decoded.height_msl_mm, fix.height_msl_mm);
        assert_eq!(decoded.sats_used, 12);
        assert_eq!(decoded.status, GnssFixStatus::Fix3D);
        assert_eq!(decoded.mode, GnssFixMode::Rtk);
        assert_eq!(decoded.sub_mode, GnssSubMode::RtkFixed);

        for i in 0..3 {
            assert!((decoded.ned_velocity[i] - fix.ned_velocity[i]).abs() < 1e-6);
        }
    }

    #[test]
    fn test_raw_command_roundtrip() {
        let cmd = RawCommand {
            commands: [0, 4000, 8191, -8192, 100, -100, 0, 0],
            num_commands: 6,
        };

        let mut buf = [0u8; 16];
        let len = cmd.encode(&mut buf);
        // 6 commands * 14 bits = 84 bits = 11 bytes (ceil)
        assert_eq!(len, 11);

        let decoded = RawCommand::decode(&buf[..len], 6).unwrap();
        assert_eq!(decoded.commands[0], 0);
        assert_eq!(decoded.commands[1], 4000);
        assert_eq!(decoded.commands[2], 8191);
        assert_eq!(decoded.commands[3], -8192);
        assert_eq!(decoded.commands[4], 100);
        assert_eq!(decoded.commands[5], -100);
    }

    #[test]
    fn test_raw_command_single_esc() {
        let cmd = RawCommand {
            commands: [4096, 0, 0, 0, 0, 0, 0, 0],
            num_commands: 1,
        };

        let mut buf = [0u8; 4];
        let len = cmd.encode(&mut buf);
        assert_eq!(len, 2); // 14 bits = 2 bytes

        let decoded = RawCommand::decode(&buf[..len], 1).unwrap();
        assert_eq!(decoded.commands[0], 4096);
    }

    #[test]
    fn test_battery_info_roundtrip() {
        let batt = BatteryInfo {
            temperature_k: 298.15,
            voltage: 22.2,
            current: -5.5,
            full_charge_capacity_wh: 100.0,
            remaining_capacity_wh: 75.0,
            state_of_charge_pct: 75,
            state_of_health_pct: 95,
            battery_id: 0,
            model_instance_id: 0,
        };

        let mut buf = [0u8; 32];
        let len = batt.encode(&mut buf);
        assert_eq!(len, 26);

        let decoded = BatteryInfo::decode(&buf[..len]).unwrap();
        assert!((decoded.voltage - 22.2).abs() < 1e-4);
        assert!((decoded.current - (-5.5)).abs() < 1e-4);
        assert_eq!(decoded.state_of_charge_pct, 75);
        assert_eq!(decoded.state_of_health_pct, 95);
    }

    #[test]
    fn test_lights_command_roundtrip() {
        let cmd = LightsCommand {
            commands: [
                SingleLightCommand { light_id: 0, red: 31, green: 0, blue: 0 },
                SingleLightCommand { light_id: 1, red: 0, green: 31, blue: 0 },
                SingleLightCommand { light_id: 0, red: 0, green: 0, blue: 0 },
                SingleLightCommand { light_id: 0, red: 0, green: 0, blue: 0 },
            ],
            num_commands: 2,
        };

        let mut buf = [0u8; 16];
        let len = cmd.encode(&mut buf);
        assert_eq!(len, 6); // 2 * 3 bytes

        let decoded = LightsCommand::decode(&buf[..len]).unwrap();
        assert_eq!(decoded.num_commands, 2);
        assert_eq!(decoded.commands[0].red, 31);
        assert_eq!(decoded.commands[0].green, 0);
        assert_eq!(decoded.commands[1].green, 31);
        assert_eq!(decoded.commands[1].blue, 0);
    }

    #[test]
    fn test_safety_state_roundtrip() {
        let state = SafetyState { status: SafetyStatus::SafetyOff };
        let mut buf = [0u8; 1];
        state.encode(&mut buf);
        let decoded = SafetyState::decode(&buf).unwrap();
        assert_eq!(decoded.status, SafetyStatus::SafetyOff);
    }

    #[test]
    fn test_arming_status_roundtrip() {
        let status = ArmingStatus { status: ArmingState::Armed };
        let mut buf = [0u8; 1];
        status.encode(&mut buf);
        let decoded = ArmingStatus::decode(&buf).unwrap();
        assert_eq!(decoded.status, ArmingState::Armed);
    }

    #[test]
    fn test_f16_roundtrip() {
        // Test some representative values
        let values = [0.0f32, 1.0, -1.0, 0.5, 100.0, -0.001, 65504.0];
        for &v in &values {
            let h = f16_from_f32(v);
            let back = f16_to_f32(h);
            // f16 has limited precision, check relative error
            if v == 0.0 {
                assert_eq!(back, 0.0);
            } else {
                let rel_err = ((back - v) / v).abs();
                assert!(rel_err < 0.01, "f16 roundtrip failed for {}: got {}, err {}", v, back, rel_err);
            }
        }
    }

    #[test]
    fn test_mag_roundtrip() {
        let mag = MagneticFieldStrength2 {
            sensor_id: 1,
            magnetic_field_ga: [0.25, -0.13, 0.45],
        };

        let mut buf = [0u8; 7];
        let len = mag.encode(&mut buf);
        assert_eq!(len, 7);

        let decoded = MagneticFieldStrength2::decode(&buf).unwrap();
        assert_eq!(decoded.sensor_id, 1);
        // f16 precision: ~0.1% for these values
        for i in 0..3 {
            let err = (decoded.magnetic_field_ga[i] - mag.magnetic_field_ga[i]).abs();
            assert!(err < 0.002, "mag axis {} error: {}", i, err);
        }
    }
}

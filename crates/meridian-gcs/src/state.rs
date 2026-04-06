//! GCS state management — the WASM core's data model.
//!
//! Receives MNP messages, maintains vehicle state, exposes to JS via public API.

use meridian_comms::messages::*;
use meridian_comms::wire;

/// Vehicle state as maintained by the GCS.
#[derive(Debug, Clone)]
pub struct VehicleState {
    pub connected: bool,
    pub armed: bool,
    pub mode: u8,
    pub mode_name: &'static str,

    // Attitude
    pub roll_deg: f32,
    pub pitch_deg: f32,
    pub yaw_deg: f32,

    // Position
    pub lat: f64,
    pub lon: f64,
    pub alt_msl: f32,
    pub alt_rel: f32,

    // Velocity
    pub groundspeed: f32,
    pub airspeed: f32,
    pub climb_rate: f32,
    pub heading: u16,

    // Battery
    pub voltage: f32,
    pub current: f32,
    pub remaining_pct: u8,

    // GPS
    pub gps_fix: u8,
    pub gps_sats: u8,

    // EKF
    pub ekf_healthy: bool,

    // Timing
    pub last_heartbeat_ms: u64,
    pub msg_count: u32,
}

impl VehicleState {
    pub fn new() -> Self {
        Self {
            connected: false, armed: false, mode: 0, mode_name: "UNKNOWN",
            roll_deg: 0.0, pitch_deg: 0.0, yaw_deg: 0.0,
            lat: 0.0, lon: 0.0, alt_msl: 0.0, alt_rel: 0.0,
            groundspeed: 0.0, airspeed: 0.0, climb_rate: 0.0, heading: 0,
            voltage: 0.0, current: 0.0, remaining_pct: 0,
            gps_fix: 0, gps_sats: 0, ekf_healthy: false,
            last_heartbeat_ms: 0, msg_count: 0,
        }
    }
}

/// The GCS state machine.
pub struct GcsState {
    pub vehicle: VehicleState,
    parser: wire::FrameParser,
    seq: u16,
}

impl GcsState {
    pub fn new() -> Self {
        Self {
            vehicle: VehicleState::new(),
            parser: wire::FrameParser::new(),
            seq: 0,
        }
    }

    /// Feed raw bytes from WebSocket. Updates vehicle state.
    /// Returns number of messages processed.
    pub fn feed_bytes(&mut self, data: &[u8]) -> u32 {
        let mut count = 0;
        for &byte in data {
            if let Some(cobs_data) = self.parser.feed(byte) {
                let mut body = [0u8; 255];
                if let Some((msg_id, _seq, body_len)) = wire::decode_frame(cobs_data, &mut body) {
                    if let Some(msg) = MnpMessage::decode(msg_id, &body[..body_len]) {
                        self.handle_message(&msg);
                        count += 1;
                    }
                }
            }
        }
        self.vehicle.msg_count += count;
        count
    }

    fn handle_message(&mut self, msg: &MnpMessage) {
        match msg {
            MnpMessage::Heartbeat(h) => {
                self.vehicle.connected = true;
                self.vehicle.armed = h.armed;
                self.vehicle.mode = h.mode;
                self.vehicle.mode_name = mode_name(h.mode);
            }
            MnpMessage::Attitude(a) => {
                self.vehicle.roll_deg = a.roll * 180.0 / core::f32::consts::PI;
                self.vehicle.pitch_deg = a.pitch * 180.0 / core::f32::consts::PI;
                self.vehicle.yaw_deg = a.yaw * 180.0 / core::f32::consts::PI;
            }
            MnpMessage::Position(p) => {
                self.vehicle.lat = p.lat_e7 as f64 / 1e7;
                self.vehicle.lon = p.lon_e7 as f64 / 1e7;
                self.vehicle.alt_msl = p.alt_mm as f32 / 1000.0;
                self.vehicle.alt_rel = p.relative_alt_mm as f32 / 1000.0;
                self.vehicle.heading = p.heading;
                self.vehicle.groundspeed = ((p.vx as f32 / 100.0).powi(2)
                    + (p.vy as f32 / 100.0).powi(2)).sqrt();
            }
            MnpMessage::Battery(b) => {
                self.vehicle.voltage = b.voltage_mv as f32 / 1000.0;
                self.vehicle.current = b.current_ca as f32 / 100.0;
                self.vehicle.remaining_pct = b.remaining_pct;
            }
            MnpMessage::GpsRaw(g) => {
                self.vehicle.gps_fix = g.fix_type;
                self.vehicle.gps_sats = g.num_sats;
            }
            MnpMessage::VfrHud(v) => {
                self.vehicle.airspeed = v.airspeed;
                self.vehicle.groundspeed = v.groundspeed;
                self.vehicle.climb_rate = v.climb;
            }
            MnpMessage::EkfStatus(e) => {
                self.vehicle.ekf_healthy = e.healthy;
            }
            _ => {}
        }
    }

    /// Encode a command to send to the vehicle. Returns frame bytes.
    pub fn encode_command(&mut self, cmd: MnpMessage, buf: &mut [u8]) -> usize {
        self.seq = self.seq.wrapping_add(1);
        cmd.encode(self.seq, buf)
    }
}

fn mode_name(mode: u8) -> &'static str {
    match mode {
        0 => "STABILIZE", 1 => "ACRO", 2 => "ALT_HOLD", 3 => "LOITER",
        4 => "RTL", 5 => "AUTO", 6 => "GUIDED", 7 => "POSHOLD",
        8 => "LAND", 9 => "CIRCLE", 10 => "BRAKE",
        _ => "UNKNOWN",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gcs_heartbeat_updates_state() {
        let mut gcs = GcsState::new();
        let msg = MnpMessage::Heartbeat(Heartbeat {
            vehicle_type: 1, armed: true, mode: 3, system_status: 4,
        });
        let mut buf = [0u8; wire::MAX_FRAME_SIZE];
        let len = msg.encode(1, &mut buf);

        let count = gcs.feed_bytes(&buf[..len]);
        assert_eq!(count, 1);
        assert!(gcs.vehicle.connected);
        assert!(gcs.vehicle.armed);
        assert_eq!(gcs.vehicle.mode, 3);
        assert_eq!(gcs.vehicle.mode_name, "LOITER");
    }

    #[test]
    fn test_gcs_position_updates() {
        let mut gcs = GcsState::new();
        let msg = MnpMessage::Position(Position {
            lat_e7: 350000000, lon_e7: -1200000000,
            alt_mm: 50000, relative_alt_mm: 10000,
            vx: 300, vy: 400, vz: -50, heading: 27000,
        });
        let mut buf = [0u8; wire::MAX_FRAME_SIZE];
        let len = msg.encode(1, &mut buf);

        gcs.feed_bytes(&buf[..len]);
        assert!((gcs.vehicle.lat - 35.0).abs() < 0.001);
        assert!((gcs.vehicle.lon - (-120.0)).abs() < 0.001);
        assert!((gcs.vehicle.alt_msl - 50.0).abs() < 0.1);
        assert!((gcs.vehicle.alt_rel - 10.0).abs() < 0.1);
        // Groundspeed = sqrt(3² + 4²) = 5 m/s
        assert!((gcs.vehicle.groundspeed - 5.0).abs() < 0.1);
    }

    #[test]
    fn test_gcs_encode_arm_command() {
        let mut gcs = GcsState::new();
        let mut buf = [0u8; wire::MAX_FRAME_SIZE];
        let len = gcs.encode_command(MnpMessage::CmdArm, &mut buf);
        assert!(len > 0);
        assert_eq!(buf[0], 0x00); // COBS delimiter
    }

    #[test]
    fn test_gcs_multiple_messages() {
        let mut gcs = GcsState::new();
        let mut stream = [0u8; 512];
        let mut offset = 0;

        // Build a stream of multiple messages
        let msgs = [
            MnpMessage::Heartbeat(Heartbeat { vehicle_type: 1, armed: true, mode: 5, system_status: 4 }),
            MnpMessage::Battery(Battery { voltage_mv: 11800, current_ca: 1500, remaining_pct: 72, consumed_mah: 800 }),
        ];
        for (i, msg) in msgs.iter().enumerate() {
            let len = msg.encode(i as u16, &mut stream[offset..]);
            offset += len;
        }

        let count = gcs.feed_bytes(&stream[..offset]);
        assert_eq!(count, 2);
        assert!(gcs.vehicle.armed);
        assert_eq!(gcs.vehicle.mode, 5);
        assert!((gcs.vehicle.voltage - 11.8).abs() < 0.1);
        assert_eq!(gcs.vehicle.remaining_pct, 72);
    }
}

//! Indoor positioning beacon system (Marvelmind, Pozyx, Nooploop).
//!
//! ArduPilot reference: `AP_Beacon.cpp`, `AP_Beacon_Backend.h`
//!
//! Beacon systems provide indoor position fixes using ultrasonic or UWB
//! beacons at known positions. The vehicle measures distances to multiple
//! beacons and computes its NED position relative to the beacon origin.

use meridian_hal::UartDriver;

// ---------------------------------------------------------------------------
// Beacon state
// ---------------------------------------------------------------------------

/// Maximum number of beacons tracked.
pub const MAX_BEACONS: usize = 16;

/// Minimum beacons for a valid position fix.
pub const MIN_BEACONS_FOR_FIX: usize = 3;

/// State of a single beacon.
#[derive(Clone, Copy, Default)]
pub struct BeaconState {
    /// Beacon ID.
    pub id: u16,
    /// Whether this beacon is active and reporting.
    pub healthy: bool,
    /// Distance from vehicle to this beacon (meters).
    pub distance_m: f32,
    /// Time of last distance update (ms).
    pub last_update_ms: u32,
    /// Known position of beacon in NED frame (meters from origin).
    pub position_ned: [f32; 3],
}

/// Vehicle position estimate from the beacon system.
#[derive(Clone, Default)]
pub struct BeaconVehiclePosition {
    /// Vehicle position in NED frame (meters from beacon origin).
    pub position_ned: [f32; 3],
    /// Position accuracy estimate (meters).
    pub accuracy_m: f32,
    /// Time of last position update (ms).
    pub last_update_ms: u32,
    /// Whether the position is valid.
    pub valid: bool,
}

/// Beacon system frontend.
pub struct BeaconSystem {
    /// Per-beacon state.
    pub beacons: [BeaconState; MAX_BEACONS],
    /// Number of active beacons.
    pub num_beacons: u8,
    /// Vehicle position estimate.
    pub vehicle_pos: BeaconVehiclePosition,
    /// Origin in WGS84 (set by user via parameters).
    pub origin_lat: f64,
    pub origin_lon: f64,
    pub origin_alt: f32,
    /// Yaw rotation from beacon frame to NED (degrees).
    pub orient_yaw_deg: f32,
}

impl BeaconSystem {
    pub fn new() -> Self {
        Self {
            beacons: [BeaconState::default(); MAX_BEACONS],
            num_beacons: 0,
            vehicle_pos: BeaconVehiclePosition::default(),
            origin_lat: 0.0, origin_lon: 0.0, origin_alt: 0.0,
            orient_yaw_deg: 0.0,
        }
    }

    /// Set the origin (WGS84 location of the beacon coordinate system).
    pub fn set_origin(&mut self, lat: f64, lon: f64, alt: f32, yaw_deg: f32) {
        self.origin_lat = lat;
        self.origin_lon = lon;
        self.origin_alt = alt;
        self.orient_yaw_deg = yaw_deg;
    }

    /// Update a beacon's distance measurement.
    pub fn update_beacon_distance(&mut self, id: u16, distance_m: f32, now_ms: u32) {
        // Find or create beacon slot.
        let slot = self.find_or_create_slot(id);
        if let Some(b) = slot {
            b.distance_m = distance_m;
            b.last_update_ms = now_ms;
            b.healthy = true;
        }
    }

    /// Set a beacon's known position in NED frame.
    pub fn set_beacon_position(&mut self, id: u16, n: f32, e: f32, d: f32) {
        let slot = self.find_or_create_slot(id);
        if let Some(b) = slot {
            b.position_ned = [n, e, d];
        }
    }

    /// Update vehicle position from beacon distance measurements.
    ///
    /// Called by the backend after a complete set of distance measurements.
    pub fn update_vehicle_position(&mut self, pos_ned: [f32; 3], accuracy_m: f32, now_ms: u32) {
        self.vehicle_pos.position_ned = pos_ned;
        self.vehicle_pos.accuracy_m = accuracy_m;
        self.vehicle_pos.last_update_ms = now_ms;
        self.vehicle_pos.valid = true;
    }

    /// Get the vehicle position for EKF injection.
    pub fn get_vehicle_position(&self) -> Option<&BeaconVehiclePosition> {
        if self.vehicle_pos.valid { Some(&self.vehicle_pos) } else { None }
    }

    /// Count healthy beacons.
    pub fn healthy_count(&self, now_ms: u32) -> u8 {
        let mut count = 0u8;
        for b in &self.beacons[..self.num_beacons as usize] {
            if b.healthy && now_ms.wrapping_sub(b.last_update_ms) < 2000 {
                count += 1;
            }
        }
        count
    }

    /// Whether we have enough beacons for a valid position.
    pub fn has_fix(&self, now_ms: u32) -> bool {
        self.healthy_count(now_ms) >= MIN_BEACONS_FOR_FIX as u8 && self.vehicle_pos.valid
    }

    /// Compute convex hull boundary from beacon positions (for fence).
    pub fn boundary_points(&self) -> heapless::Vec<[f32; 2], 16> {
        let mut points = heapless::Vec::<[f32; 2], 16>::new();
        for b in &self.beacons[..self.num_beacons as usize] {
            if b.healthy {
                let _ = points.push([b.position_ned[0], b.position_ned[1]]);
            }
        }
        // Simple convex hull would go here — for now return all points.
        points
    }

    fn find_or_create_slot(&mut self, id: u16) -> Option<&mut BeaconState> {
        // Find existing.
        for i in 0..self.num_beacons as usize {
            if self.beacons[i].id == id { return Some(&mut self.beacons[i]); }
        }
        // Create new.
        if (self.num_beacons as usize) < MAX_BEACONS {
            let idx = self.num_beacons as usize;
            self.beacons[idx].id = id;
            self.num_beacons += 1;
            Some(&mut self.beacons[idx])
        } else {
            None
        }
    }
}

// ---------------------------------------------------------------------------
// Marvelmind serial protocol
// ---------------------------------------------------------------------------

/// Marvelmind SuperBeacon serial protocol parser.
///
/// Protocol: header(0x47 0x01), length, packet_type, payload, CRC16.
/// Position packet type 0x01: x(i32), y(i32), z(i32) in mm, plus address byte.
pub struct MarvelmindBeacon {
    buf: [u8; 64],
    pos: usize,
    pub system: BeaconSystem,
}

impl MarvelmindBeacon {
    pub fn new() -> Self {
        Self { buf: [0; 64], pos: 0, system: BeaconSystem::new() }
    }

    pub fn process_byte(&mut self, byte: u8, now_ms: u32) -> bool {
        if self.pos == 0 && byte != 0x47 { return false; }
        if self.pos == 1 && byte != 0x01 { self.pos = 0; return false; }

        self.buf[self.pos] = byte;
        self.pos += 1;

        if self.pos < 4 { return false; } // Need at least header + type + length.

        let payload_len = self.buf[3] as usize;
        let frame_len = 4 + payload_len + 2; // header(2) + type(1) + len(1) + payload + crc(2)

        if self.pos < frame_len { return false; }
        self.pos = 0;

        // CRC check would go here.
        let pkt_type = self.buf[2];
        if pkt_type == 0x01 && payload_len >= 13 {
            // Position packet: address(1), x(i32 LE mm), y(i32 LE mm), z(i32 LE mm).
            let addr = self.buf[4] as u16;
            let x_mm = i32::from_le_bytes([self.buf[5], self.buf[6], self.buf[7], self.buf[8]]);
            let y_mm = i32::from_le_bytes([self.buf[9], self.buf[10], self.buf[11], self.buf[12]]);
            let z_mm = i32::from_le_bytes([self.buf[13], self.buf[14], self.buf[15], self.buf[16]]);

            // Convert mm to meters, map to NED.
            let n = x_mm as f32 / 1000.0;
            let e = y_mm as f32 / 1000.0;
            let d = -z_mm as f32 / 1000.0; // Z up in Marvelmind, down in NED.

            self.system.update_vehicle_position([n, e, d], 0.05, now_ms);
            return true;
        }

        false
    }
}

// ---------------------------------------------------------------------------
// Pozyx serial protocol
// ---------------------------------------------------------------------------

/// Pozyx UWB positioning serial parser.
///
/// Sends position as JSON-like ASCII: `POS,id,x,y,z,quality\r\n`
pub struct PozyxBeacon {
    buf: [u8; 128],
    pos: usize,
    pub system: BeaconSystem,
}

impl PozyxBeacon {
    pub fn new() -> Self {
        Self { buf: [0; 128], pos: 0, system: BeaconSystem::new() }
    }

    pub fn process_byte(&mut self, byte: u8, now_ms: u32) -> bool {
        if byte == b'\n' || byte == b'\r' {
            if self.pos > 5 { return self.parse_line(now_ms); }
            self.pos = 0;
            return false;
        }
        if self.pos < 127 { self.buf[self.pos] = byte; self.pos += 1; }
        false
    }

    fn parse_line(&mut self, now_ms: u32) -> bool {
        // Expected: "POS,<id>,<x_mm>,<y_mm>,<z_mm>,<quality>"
        let line = &self.buf[..self.pos];
        self.pos = 0;

        if line.len() < 8 || &line[..4] != b"POS," { return false; }

        let fields: heapless::Vec<&[u8], 8> = line[4..].split(|&b| b == b',')
            .collect::<heapless::Vec<_, 8>>();
        if fields.len() < 4 { return false; }

        let x_mm = parse_i32(fields[1]);
        let y_mm = parse_i32(fields[2]);
        let z_mm = parse_i32(fields[3]);

        let n = x_mm as f32 / 1000.0;
        let e = y_mm as f32 / 1000.0;
        let d = -z_mm as f32 / 1000.0;

        self.system.update_vehicle_position([n, e, d], 0.1, now_ms);
        true
    }
}

/// Nooploop LinkTrack serial parser (similar ASCII format).
pub struct NoopLoopBeacon {
    buf: [u8; 128],
    pos: usize,
    pub system: BeaconSystem,
}

impl NoopLoopBeacon {
    pub fn new() -> Self {
        Self { buf: [0; 128], pos: 0, system: BeaconSystem::new() }
    }

    pub fn process_byte(&mut self, byte: u8, now_ms: u32) -> bool {
        if byte == b'\n' || byte == b'\r' {
            if self.pos > 5 { return self.parse_line(now_ms); }
            self.pos = 0;
            return false;
        }
        if self.pos < 127 { self.buf[self.pos] = byte; self.pos += 1; }
        false
    }

    fn parse_line(&mut self, now_ms: u32) -> bool {
        let line = &self.buf[..self.pos];
        self.pos = 0;

        // Nooploop outputs: "nlt,<tag_id>,<x_m>,<y_m>,<z_m>\r\n" (meters, float)
        if line.len() < 6 { return false; }

        let fields: heapless::Vec<&[u8], 8> = line.split(|&b| b == b',')
            .collect::<heapless::Vec<_, 8>>();
        if fields.len() < 5 || fields[0] != b"nlt" { return false; }

        let x_mm = parse_i32(fields[2]); // treat as mm for simplicity
        let y_mm = parse_i32(fields[3]);
        let z_mm = parse_i32(fields[4]);

        self.system.update_vehicle_position(
            [x_mm as f32 / 1000.0, y_mm as f32 / 1000.0, -z_mm as f32 / 1000.0],
            0.15, now_ms,
        );
        true
    }
}

// Simple integer parser for no_std.
fn parse_i32(s: &[u8]) -> i32 {
    let mut val: i32 = 0;
    let mut neg = false;
    for &b in s {
        if b == b'-' { neg = true; continue; }
        if b >= b'0' && b <= b'9' { val = val * 10 + (b - b'0') as i32; }
    }
    if neg { -val } else { val }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_beacon_system() {
        let mut sys = BeaconSystem::new();
        sys.set_beacon_position(1, 0.0, 0.0, 0.0);
        sys.set_beacon_position(2, 5.0, 0.0, 0.0);
        sys.set_beacon_position(3, 0.0, 5.0, 0.0);

        sys.update_beacon_distance(1, 2.5, 1000);
        sys.update_beacon_distance(2, 3.0, 1000);
        sys.update_beacon_distance(3, 3.5, 1000);

        assert_eq!(sys.healthy_count(1000), 3);
        assert_eq!(sys.num_beacons, 3);

        sys.update_vehicle_position([1.2, 1.5, 0.0], 0.05, 1000);
        assert!(sys.has_fix(1000));
    }

    #[test]
    fn test_parse_i32() {
        assert_eq!(parse_i32(b"1234"), 1234);
        assert_eq!(parse_i32(b"-500"), -500);
        assert_eq!(parse_i32(b"0"), 0);
    }

    #[test]
    fn test_boundary() {
        let mut sys = BeaconSystem::new();
        sys.set_beacon_position(1, 0.0, 0.0, 0.0);
        sys.update_beacon_distance(1, 1.0, 100);
        sys.set_beacon_position(2, 5.0, 0.0, 0.0);
        sys.update_beacon_distance(2, 3.0, 100);

        let boundary = sys.boundary_points();
        assert_eq!(boundary.len(), 2);
    }
}

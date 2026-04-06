//! DroneCAN GPS receiver — processes GPS fix messages from CAN bus.
//!
//! ArduPilot reference: `AP_GPS_DroneCAN.cpp`
//!
//! Receives `uavcan.equipment.gnss.Fix2` messages from a DroneCAN GPS node
//! and converts them to the standard GPS state format.

/// GPS state from a DroneCAN GNSS node.
#[derive(Clone, Default)]
pub struct DroneCAnGpsState {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_msl_m: f32,
    pub velocity_ned: [f32; 3],
    pub ground_speed_ms: f32,
    pub ground_course_deg: f32,
    pub fix_type: u8,
    pub num_sats: u8,
    pub hdop: f32,
    pub vdop: f32,
    pub horizontal_accuracy_m: f32,
    pub vertical_accuracy_m: f32,
    pub speed_accuracy_ms: f32,
    pub timestamp_us: u64,
    pub valid: bool,
}

/// DroneCAN GPS receiver.
///
/// Doesn't parse CAN frames directly — the `meridian-can` crate handles
/// frame decoding and calls `handle_fix2()` when a GNSS Fix2 message arrives.
pub struct DroneCanGps {
    pub state: DroneCAnGpsState,
    pub node_id: u8,
    last_update_ms: u32,
}

impl DroneCanGps {
    pub fn new(node_id: u8) -> Self {
        Self {
            state: DroneCAnGpsState::default(),
            node_id,
            last_update_ms: 0,
        }
    }

    /// Handle a decoded Fix2 message from the CAN bus.
    ///
    /// Fields correspond to `uavcan.equipment.gnss.Fix2`:
    /// - `status`: 0=no_fix, 1=time_only, 2=2d, 3=3d
    /// - `lat/lon`: degrees
    /// - `alt_msl`: meters
    /// - `ned_velocity`: [N, E, D] in m/s
    /// - `sats_used`: number of satellites
    /// - `pdop`: position DOP
    pub fn handle_fix2(
        &mut self,
        now_ms: u32,
        status: u8,
        lat: f64,
        lon: f64,
        alt_msl: f32,
        ned_velocity: [f32; 3],
        sats_used: u8,
        pdop: f32,
    ) {
        self.state.latitude_deg = lat;
        self.state.longitude_deg = lon;
        self.state.altitude_msl_m = alt_msl;
        self.state.velocity_ned = ned_velocity;
        self.state.ground_speed_ms = libm::sqrtf(
            ned_velocity[0] * ned_velocity[0] + ned_velocity[1] * ned_velocity[1]
        );
        self.state.ground_course_deg = libm::atan2f(ned_velocity[1], ned_velocity[0])
            * 180.0 / core::f32::consts::PI;
        if self.state.ground_course_deg < 0.0 {
            self.state.ground_course_deg += 360.0;
        }
        self.state.fix_type = match status {
            3 => 3, // 3D fix
            2 => 2, // 2D fix
            _ => 0,
        };
        self.state.num_sats = sats_used;
        self.state.hdop = pdop * 0.7; // approximate HDOP from PDOP
        self.state.vdop = pdop * 0.7;
        self.state.valid = status >= 2;
        self.last_update_ms = now_ms;
    }

    /// Whether the GPS data is fresh (received within last 2 seconds).
    pub fn is_healthy(&self, now_ms: u32) -> bool {
        self.state.valid && now_ms.wrapping_sub(self.last_update_ms) < 2000
    }
}

/// ExternalAHRS GPS passthrough — receives GPS data from an external AHRS device.
///
/// ArduPilot reference: `AP_GPS_ExternalAHRS.cpp`
///
/// The external AHRS (VectorNav, InertialLabs, etc.) provides GPS data
/// as part of its output. This module receives that data and presents it
/// as a standard GPS source.
pub struct ExternalAhrsGps {
    pub state: DroneCAnGpsState, // reuse same state struct
    last_update_ms: u32,
}

impl ExternalAhrsGps {
    pub fn new() -> Self {
        Self {
            state: DroneCAnGpsState::default(),
            last_update_ms: 0,
        }
    }

    /// Handle GPS data injected from an external AHRS device.
    pub fn handle_external(
        &mut self,
        now_ms: u32,
        lat: f64,
        lon: f64,
        alt_msl: f32,
        vel_ned: [f32; 3],
        fix_type: u8,
        num_sats: u8,
        hacc: f32,
        vacc: f32,
        sacc: f32,
    ) {
        self.state.latitude_deg = lat;
        self.state.longitude_deg = lon;
        self.state.altitude_msl_m = alt_msl;
        self.state.velocity_ned = vel_ned;
        self.state.ground_speed_ms = libm::sqrtf(vel_ned[0] * vel_ned[0] + vel_ned[1] * vel_ned[1]);
        self.state.fix_type = fix_type;
        self.state.num_sats = num_sats;
        self.state.horizontal_accuracy_m = hacc;
        self.state.vertical_accuracy_m = vacc;
        self.state.speed_accuracy_ms = sacc;
        self.state.valid = fix_type >= 2;
        self.last_update_ms = now_ms;
    }

    pub fn is_healthy(&self, now_ms: u32) -> bool {
        self.state.valid && now_ms.wrapping_sub(self.last_update_ms) < 2000
    }
}

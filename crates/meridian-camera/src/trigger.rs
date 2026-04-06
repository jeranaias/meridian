//! Camera trigger manager — distance/time auto-triggering and manual shutter.
//!
//! Source: ArduPilot `AP_Camera.cpp`

use crate::backend::{CameraBackend, ServoCameraBackend, RelayCameraBackend, MountCameraBackend, RunCamBackend, MavlinkCamV2Backend};
use crate::geotag::{GeotagEntry, GeotagLog};

/// Auto-trigger mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriggerMode {
    /// Manual trigger only.
    Manual,
    /// Auto-trigger when distance from last photo exceeds threshold.
    Distance,
    /// Auto-trigger every N milliseconds.
    Time,
}

impl Default for TriggerMode {
    fn default() -> Self {
        TriggerMode::Manual
    }
}

/// Camera backend type — avoids dynamic dispatch for embedded.
pub enum CameraBackendType {
    Servo(ServoCameraBackend),
    Relay(RelayCameraBackend),
    Mount(MountCameraBackend),
    RunCam(RunCamBackend),
    MavlinkCamV2(MavlinkCamV2Backend),
}

/// Camera manager: handles triggering logic and geotagging.
pub struct CameraManager {
    /// Active backend.
    pub backend: CameraBackendType,
    /// Auto-trigger mode.
    pub mode: TriggerMode,
    /// Distance threshold for distance triggering (meters).
    pub distance_threshold_m: f32,
    /// Time interval for time triggering (milliseconds).
    pub time_interval_ms: u32,
    /// Geotag log (stores last 64 entries).
    pub geotags: GeotagLog<64>,
    /// Total photo count.
    pub photo_count: u32,

    // ── New parity fields ──

    /// Total number of photos for burst mode (0 = infinite).
    pub burst_count: u32,
    /// Minimum interval between triggers (ms). 0 = no guard.
    pub min_interval_ms: u32,
    /// Whether distance triggering only works in AUTO mode.
    pub auto_mode_only: bool,
    /// Maximum roll (radians) for distance/time triggering. 0 = disabled.
    pub max_roll_rad: f32,
    /// Whether vehicle is currently in AUTO mode (set by caller).
    pub is_auto_mode: bool,

    // Internal state
    last_trigger_ms: u32,
    last_trigger_lat: f64,
    last_trigger_lon: f64,
    /// Number of photos taken in the current burst sequence.
    burst_taken: u32,
}

impl CameraManager {
    pub fn new(backend: CameraBackendType) -> Self {
        Self {
            backend,
            mode: TriggerMode::Manual,
            distance_threshold_m: 0.0,
            time_interval_ms: 0,
            geotags: GeotagLog::new(),
            photo_count: 0,
            burst_count: 0,
            min_interval_ms: 0,
            auto_mode_only: false,
            max_roll_rad: 0.0,
            is_auto_mode: false,
            last_trigger_ms: 0,
            last_trigger_lat: 0.0,
            last_trigger_lon: 0.0,
            burst_taken: 0,
        }
    }

    /// Start a burst sequence: take `count` photos at `interval_ms`.
    /// count = 0 means infinite (keep shooting until mode changes).
    pub fn start_burst(&mut self, interval_ms: u32, count: u32) {
        self.mode = TriggerMode::Time;
        self.time_interval_ms = interval_ms;
        self.burst_count = count;
        self.burst_taken = 0;
    }

    /// Manually trigger the shutter and record a geotag.
    pub fn trigger_shutter(
        &mut self,
        lat: f64,
        lon: f64,
        alt: f32,
        alt_agl: f32,
        roll: f32,
        pitch: f32,
        yaw: f32,
        now_ms: u32,
    ) {
        match &mut self.backend {
            CameraBackendType::Servo(b) => b.trigger_shutter(),
            CameraBackendType::Relay(b) => b.trigger_shutter(),
            CameraBackendType::Mount(b) => b.trigger_shutter(),
            CameraBackendType::RunCam(b) => b.trigger_shutter(),
            CameraBackendType::MavlinkCamV2(b) => b.trigger_shutter(),
        }

        self.photo_count += 1;

        self.geotags.push(GeotagEntry {
            lat,
            lon,
            alt,
            alt_agl,
            roll,
            pitch,
            yaw,
            timestamp_ms: now_ms,
            image_index: self.photo_count,
        });

        self.last_trigger_ms = now_ms;
        self.last_trigger_lat = lat;
        self.last_trigger_lon = lon;
        self.burst_taken += 1;
    }

    /// Start video recording.
    pub fn start_video(&mut self) {
        match &mut self.backend {
            CameraBackendType::Servo(b) => b.start_video(),
            CameraBackendType::Relay(b) => b.start_video(),
            CameraBackendType::Mount(b) => b.start_video(),
            CameraBackendType::RunCam(b) => b.start_video(),
            CameraBackendType::MavlinkCamV2(b) => b.start_video(),
        }
    }

    /// Stop video recording.
    pub fn stop_video(&mut self) {
        match &mut self.backend {
            CameraBackendType::Servo(b) => b.stop_video(),
            CameraBackendType::Relay(b) => b.stop_video(),
            CameraBackendType::Mount(b) => b.stop_video(),
            CameraBackendType::RunCam(b) => b.stop_video(),
            CameraBackendType::MavlinkCamV2(b) => b.stop_video(),
        }
    }

    /// Check the minimum interval guard.
    fn min_interval_ok(&self, now_ms: u32) -> bool {
        if self.min_interval_ms == 0 || self.photo_count == 0 {
            return true;
        }
        now_ms.wrapping_sub(self.last_trigger_ms) >= self.min_interval_ms
    }

    /// Check the roll limit guard.
    fn roll_ok(&self, roll: f32) -> bool {
        if self.max_roll_rad <= 0.0 {
            return true;
        }
        libm::fabsf(roll) <= self.max_roll_rad
    }

    /// Check burst count limit.
    fn burst_ok(&self) -> bool {
        if self.burst_count == 0 {
            return true; // infinite
        }
        self.burst_taken < self.burst_count
    }

    /// Main update — call each control loop. Handles auto-triggering and backend pulse timing.
    ///
    /// `lat`, `lon`: current vehicle position in degrees.
    /// `alt`: altitude above home in meters (MSL).
    /// `alt_agl`: altitude above ground in meters.
    /// `roll`, `pitch`, `yaw`: current vehicle attitude in radians.
    /// `now_ms`: current timestamp in milliseconds.
    pub fn update(
        &mut self,
        lat: f64,
        lon: f64,
        alt: f32,
        alt_agl: f32,
        roll: f32,
        pitch: f32,
        yaw: f32,
        now_ms: u32,
    ) {
        // Update backend pulse timing
        match &mut self.backend {
            CameraBackendType::Servo(b) => b.update(now_ms),
            CameraBackendType::Relay(b) => b.update(now_ms),
            CameraBackendType::Mount(b) => b.update(now_ms),
            CameraBackendType::RunCam(b) => b.update(now_ms),
            CameraBackendType::MavlinkCamV2(b) => b.update(now_ms),
        }

        match self.mode {
            TriggerMode::Manual => {}
            TriggerMode::Distance => {
                // AUTO mode only guard
                if self.auto_mode_only && !self.is_auto_mode {
                    return;
                }
                // Roll limit guard
                if !self.roll_ok(roll) {
                    return;
                }
                // Minimum interval guard
                if !self.min_interval_ok(now_ms) {
                    return;
                }

                if self.distance_threshold_m > 0.0 && self.photo_count > 0 {
                    let dist = approx_distance_m(
                        self.last_trigger_lat,
                        self.last_trigger_lon,
                        lat,
                        lon,
                    );
                    if dist >= self.distance_threshold_m {
                        self.trigger_shutter(lat, lon, alt, alt_agl, roll, pitch, yaw, now_ms);
                    }
                } else if self.photo_count == 0 {
                    // Take first photo immediately
                    self.trigger_shutter(lat, lon, alt, alt_agl, roll, pitch, yaw, now_ms);
                }
            }
            TriggerMode::Time => {
                // Roll limit guard
                if !self.roll_ok(roll) {
                    return;
                }
                // Minimum interval guard
                if !self.min_interval_ok(now_ms) {
                    return;
                }
                // Burst count guard
                if !self.burst_ok() {
                    return;
                }

                if self.time_interval_ms > 0 {
                    let elapsed = now_ms.wrapping_sub(self.last_trigger_ms);
                    if elapsed >= self.time_interval_ms || self.photo_count == 0 {
                        self.trigger_shutter(lat, lon, alt, alt_agl, roll, pitch, yaw, now_ms);
                    }
                }
            }
        }
    }
}

/// Approximate distance between two lat/lon positions in meters.
/// Uses equirectangular approximation — sufficient for short distances in survey missions.
fn approx_distance_m(lat1_deg: f64, lon1_deg: f64, lat2_deg: f64, lon2_deg: f64) -> f32 {
    const DEG_TO_RAD: f64 = core::f64::consts::PI / 180.0;
    const EARTH_RADIUS_M: f64 = 6371000.0;

    let dlat = (lat2_deg - lat1_deg) * DEG_TO_RAD;
    let dlon = (lon2_deg - lon1_deg) * DEG_TO_RAD;
    let avg_lat = (lat1_deg + lat2_deg) * 0.5 * DEG_TO_RAD;

    let x = dlon * libm::cos(avg_lat);
    let y = dlat;

    let dist = EARTH_RADIUS_M * libm::sqrt(x * x + y * y);
    dist as f32
}

//! Mount manager — routes targets to the active backend.
//!
//! Source: ArduPilot `AP_Mount.cpp`

use crate::backend::{MountTarget, MountBackend};
use crate::servo::ServoMount;
use crate::mavlink::MavlinkMount;
use crate::siyi::SiyiMount;

/// Mount operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MountMode {
    /// Mount is retracted / stowed.
    Retract,
    /// Mount is at neutral (forward-facing) position.
    Neutral,
    /// Mount angles driven by RC stick input.
    RcTargeting,
    /// Mount angles driven by MAVLink commands from GCS.
    MavlinkTargeting,
    /// Mount points at a GPS location (Region-Of-Interest).
    GpsTargeting,
    /// Mount tracks a specific MAVLink system ID.
    SysidTargeting,
}

impl Default for MountMode {
    fn default() -> Self {
        MountMode::Neutral
    }
}

/// Backend type — avoids dynamic dispatch for embedded.
pub enum MountBackendType {
    Servo(ServoMount),
    Mavlink(MavlinkMount),
    Siyi(SiyiMount),
}

/// Mount manager: owns the backend and routes targeting modes.
pub struct MountManager {
    /// Active backend.
    pub backend: MountBackendType,
    /// Current operating mode.
    pub mode: MountMode,
    /// Current target (set by mode logic).
    pub target: MountTarget,
    /// RC rate scale factor (radians/s per unit RC deflection).
    pub rc_rate_scale: f32,
    /// Yaw frame-lock: true = earth-frame (lock), false = body-frame (follow).
    pub yaw_lock: bool,
    /// Pitch frame-lock.
    pub pitch_lock: bool,
    /// Roll frame-lock.
    pub roll_lock: bool,
    /// Whether RC targeting uses angle mode (false) or rate mode (true).
    pub rc_rate_mode: bool,
    /// Vehicle position — needed for GPS ROI calculations.
    vehicle_lat: f64,
    vehicle_lon: f64,
    vehicle_alt: f32,
}

impl MountManager {
    pub fn new(backend: MountBackendType) -> Self {
        Self {
            backend,
            mode: MountMode::Neutral,
            target: MountTarget::Neutral,
            rc_rate_scale: 1.0,
            yaw_lock: false,
            pitch_lock: false,
            roll_lock: false,
            rc_rate_mode: true,
            vehicle_lat: 0.0,
            vehicle_lon: 0.0,
            vehicle_alt: 0.0,
        }
    }

    /// Initialize the backend.
    pub fn init(&mut self) -> bool {
        match &mut self.backend {
            MountBackendType::Servo(b) => b.init(),
            MountBackendType::Mavlink(b) => b.init(),
            MountBackendType::Siyi(b) => b.init(),
        }
    }

    /// Set the mount mode.
    pub fn set_mode(&mut self, mode: MountMode) {
        self.mode = mode;
        match mode {
            MountMode::Retract => self.target = MountTarget::Retract,
            MountMode::Neutral => self.target = MountTarget::Neutral,
            _ => {}
        }
    }

    /// Set angle target (used by MavlinkTargeting and GpsTargeting modes).
    pub fn set_angle_target(&mut self, roll: f32, pitch: f32, yaw: f32) {
        self.target = MountTarget::Angle { roll, pitch, yaw };
    }

    /// Set rate target (used by RcTargeting mode).
    pub fn set_rate_target(&mut self, roll_rate: f32, pitch_rate: f32, yaw_rate: f32) {
        self.target = MountTarget::Rate { roll_rate, pitch_rate, yaw_rate };
    }

    /// Set ROI target (used by GpsTargeting mode).
    pub fn set_roi_target(&mut self, lat: f64, lon: f64, alt: f32) {
        self.target = MountTarget::RoiLocation { lat, lon, alt };
    }

    /// Feed current vehicle position for ROI computation.
    pub fn set_vehicle_position(&mut self, lat: f64, lon: f64, alt: f32) {
        self.vehicle_lat = lat;
        self.vehicle_lon = lon;
        self.vehicle_alt = alt;
    }

    /// Resolve an ROI location to angle target using atan2 bearing/pitch calculation.
    ///
    /// Returns (roll, pitch, yaw) in radians aimed at the ROI from current vehicle position.
    pub fn resolve_roi(&self, roi_lat: f64, roi_lon: f64, roi_alt: f32) -> (f32, f32, f32) {
        const DEG_TO_RAD: f64 = core::f64::consts::PI / 180.0;
        const EARTH_RADIUS_M: f64 = 6371000.0;

        let dlat = (roi_lat - self.vehicle_lat) * DEG_TO_RAD;
        let dlon = (roi_lon - self.vehicle_lon) * DEG_TO_RAD;
        let avg_lat = (self.vehicle_lat + roi_lat) * 0.5 * DEG_TO_RAD;

        let dx_m = dlon * EARTH_RADIUS_M * libm::cos(avg_lat);
        let dy_m = dlat * EARTH_RADIUS_M;

        // Bearing: atan2(east, north)
        let yaw = libm::atan2(dx_m, dy_m) as f32;

        // Horizontal distance
        let horiz_dist = libm::sqrt(dx_m * dx_m + dy_m * dy_m);

        // Pitch: atan2(height difference, horizontal distance)
        let dalt = roi_alt as f64 - self.vehicle_alt as f64;
        let pitch = libm::atan2(-dalt, horiz_dist) as f32; // negative because looking down

        let roll = 0.0;

        (roll, pitch, yaw)
    }

    /// Update RC targeting: convert RC channel deflection to rate or angle commands.
    /// `rc_pitch` and `rc_yaw` are normalized RC inputs in [-1.0, +1.0].
    pub fn update_rc_targeting(&mut self, rc_pitch: f32, rc_yaw: f32) {
        if self.mode != MountMode::RcTargeting {
            return;
        }

        if self.rc_rate_mode {
            // Rate mode: RC deflection maps to angular rate
            self.target = MountTarget::Rate {
                roll_rate: 0.0,
                pitch_rate: rc_pitch * self.rc_rate_scale,
                yaw_rate: rc_yaw * self.rc_rate_scale,
            };
        } else {
            // Angle mode: RC deflection maps directly to angle
            let pitch_angle = rc_pitch * core::f32::consts::FRAC_PI_2;
            let yaw_angle = rc_yaw * core::f32::consts::PI;
            self.target = MountTarget::Angle {
                roll: 0.0,
                pitch: pitch_angle,
                yaw: yaw_angle,
            };
        }
    }

    /// Main update — call once per control loop.
    pub fn update(&mut self, dt: f32) {
        // If in GPS targeting mode, resolve ROI to angle
        if self.mode == MountMode::GpsTargeting {
            if let MountTarget::RoiLocation { lat, lon, alt } = self.target {
                let (roll, pitch, yaw) = self.resolve_roi(lat, lon, alt);
                // Store the resolved angle but keep the ROI as the logical target
                let angle_target = MountTarget::Angle { roll, pitch, yaw };
                match &mut self.backend {
                    MountBackendType::Servo(b) => b.update(&angle_target, dt),
                    MountBackendType::Mavlink(b) => b.update(&angle_target, dt),
                    MountBackendType::Siyi(b) => b.update(&angle_target, dt),
                }
                return;
            }
        }

        match &mut self.backend {
            MountBackendType::Servo(b) => b.update(&self.target, dt),
            MountBackendType::Mavlink(b) => b.update(&self.target, dt),
            MountBackendType::Siyi(b) => b.update(&self.target, dt),
        }
    }

    /// Get current mount attitude (roll, pitch, yaw) in radians.
    pub fn get_attitude(&self) -> (f32, f32, f32) {
        use crate::backend::MountBackend;
        match &self.backend {
            MountBackendType::Servo(b) => b.get_attitude(),
            MountBackendType::Mavlink(b) => b.get_attitude(),
            MountBackendType::Siyi(b) => b.get_attitude(),
        }
    }

    /// Whether the active backend supports pan/yaw control.
    pub fn has_pan_control(&self) -> bool {
        use crate::backend::MountBackend;
        match &self.backend {
            MountBackendType::Servo(b) => b.has_pan_control(),
            MountBackendType::Mavlink(b) => b.has_pan_control(),
            MountBackendType::Siyi(b) => b.has_pan_control(),
        }
    }
}

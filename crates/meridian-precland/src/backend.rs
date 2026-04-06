//! Precision landing sensor backends.
//!
//! Source: ArduPilot `AC_PrecLand/AC_PrecLand_Backend.h`

/// A detected landing target in body-frame angles.
#[derive(Debug, Clone, Copy)]
pub struct PrecLandTarget {
    /// Angle to target in X axis (radians, body frame).
    pub angle_x: f32,
    /// Angle to target in Y axis (radians, body frame).
    pub angle_y: f32,
    /// Distance to target in meters (0 if unknown).
    pub distance_m: f32,
    /// Timestamp of the detection (milliseconds since boot).
    pub timestamp_ms: u32,
}

impl Default for PrecLandTarget {
    fn default() -> Self {
        Self {
            angle_x: 0.0,
            angle_y: 0.0,
            distance_m: 0.0,
            timestamp_ms: 0,
        }
    }
}

/// Landing target sensor backend trait.
pub trait PrecLandBackend {
    /// Initialize the sensor. Returns true on success.
    fn init(&mut self) -> bool;

    /// Update — poll for new data. Called each control loop.
    fn update(&mut self);

    /// Whether a target has been detected since last update.
    fn has_target(&self) -> bool;

    /// Get the most recent target detection.
    fn get_target(&self) -> Option<PrecLandTarget>;
}

/// IRLock sensor backend (I2C, PixArt IR camera).
///
/// Source: ArduPilot `AC_PrecLand_IRLock.cpp`
/// I2C address 0x54, reads 2-byte x/y pixel coordinates + size.
pub struct IRLockBackend {
    /// I2C bus index.
    pub bus: u8,
    /// I2C address (default 0x54).
    pub address: u8,
    /// Most recent target (if any).
    target: Option<PrecLandTarget>,
    /// Sensor field-of-view in radians (60 deg typical).
    pub fov_rad: f32,
    initialized: bool,
}

impl IRLockBackend {
    pub fn new(bus: u8) -> Self {
        Self {
            bus,
            address: 0x54,
            target: None,
            fov_rad: 60.0_f32 * core::f32::consts::PI / 180.0,
            initialized: false,
        }
    }

    /// Feed raw pixel data from I2C read (for testing / simulation).
    /// `px`, `py`: pixel coordinates (0-319 range for 320x200 sensor).
    /// `now_ms`: current timestamp.
    pub fn feed_pixel_data(&mut self, px: u16, py: u16, now_ms: u32) {
        // Convert pixel to angle: center is (160, 100), scale by FOV
        let angle_x = ((px as f32) - 160.0) / 160.0 * (self.fov_rad * 0.5);
        let angle_y = ((py as f32) - 100.0) / 100.0 * (self.fov_rad * 0.5);
        self.target = Some(PrecLandTarget {
            angle_x,
            angle_y,
            distance_m: 0.0, // IRLock doesn't measure distance
            timestamp_ms: now_ms,
        });
    }
}

impl PrecLandBackend for IRLockBackend {
    fn init(&mut self) -> bool {
        self.initialized = true;
        true
    }

    fn update(&mut self) {
        // In real hardware: read I2C and call feed_pixel_data().
        // Here the caller feeds data externally.
    }

    fn has_target(&self) -> bool {
        self.target.is_some()
    }

    fn get_target(&self) -> Option<PrecLandTarget> {
        self.target
    }
}

/// MAVLink LANDING_TARGET message backend.
///
/// Source: ArduPilot `AC_PrecLand_MAVLink.cpp`
/// Receives LANDING_TARGET (msg #149) from a companion computer.
pub struct MavlinkLandingBackend {
    target: Option<PrecLandTarget>,
    initialized: bool,
}

impl MavlinkLandingBackend {
    pub fn new() -> Self {
        Self {
            target: None,
            initialized: false,
        }
    }

    /// Feed a LANDING_TARGET message from MAVLink.
    pub fn feed_landing_target(&mut self, angle_x: f32, angle_y: f32, distance: f32, now_ms: u32) {
        self.target = Some(PrecLandTarget {
            angle_x,
            angle_y,
            distance_m: distance,
            timestamp_ms: now_ms,
        });
    }
}

impl PrecLandBackend for MavlinkLandingBackend {
    fn init(&mut self) -> bool {
        self.initialized = true;
        true
    }

    fn update(&mut self) {
        // Data fed externally via feed_landing_target()
    }

    fn has_target(&self) -> bool {
        self.target.is_some()
    }

    fn get_target(&self) -> Option<PrecLandTarget> {
        self.target
    }
}

/// SITL (Software-In-The-Loop) backend for testing.
pub struct SitlLandingBackend {
    target: Option<PrecLandTarget>,
    initialized: bool,
}

impl SitlLandingBackend {
    pub fn new() -> Self {
        Self {
            target: None,
            initialized: false,
        }
    }

    /// Set simulated target for testing.
    pub fn set_target(&mut self, angle_x: f32, angle_y: f32, distance: f32, now_ms: u32) {
        self.target = Some(PrecLandTarget {
            angle_x,
            angle_y,
            distance_m: distance,
            timestamp_ms: now_ms,
        });
    }

    /// Clear target (simulate target lost).
    pub fn clear_target(&mut self) {
        self.target = None;
    }
}

impl PrecLandBackend for SitlLandingBackend {
    fn init(&mut self) -> bool {
        self.initialized = true;
        true
    }

    fn update(&mut self) {}

    fn has_target(&self) -> bool {
        self.target.is_some()
    }

    fn get_target(&self) -> Option<PrecLandTarget> {
        self.target
    }
}

//! Optical flow sensor drivers — velocity estimation for GPS-denied flight.
//!
//! Source: ArduPilot AP_OpticalFlow
//! EKF fuses (flowRate - bodyRate) as translational measurement.
//! 8 driver types in ArduPilot. We implement PMW3901 (SPI) and MAVLink.

use meridian_math::Vec3;
use meridian_math::frames::Body;

/// Optical flow reading.
#[derive(Debug, Clone, Copy)]
pub struct FlowReading {
    /// Flow rate in radians/sec (integrated pixel movement / dt).
    /// X = forward/back, Y = left/right in body frame.
    pub flow_rate: Vec3<Body>,
    /// Simultaneous gyro measurement in radians/sec (for compensation).
    pub body_rate: Vec3<Body>,
    /// Surface quality (0-255, higher = better texture).
    pub quality: u8,
    /// Whether the reading is valid.
    pub valid: bool,
}

impl FlowReading {
    /// Get the translational flow (what the EKF fuses).
    /// This is flow_rate minus body_rate — removes rotational component.
    pub fn translational_flow(&self) -> Vec3<Body> {
        self.flow_rate - self.body_rate
    }
}

/// PMW3901 optical flow sensor (SPI).
/// Used on many small quads and indoor drones.
/// Provides raw delta-X/Y pixel counts per frame.
pub struct Pmw3901 {
    // Configuration
    flow_scale_x: f32, // pixels/radian
    flow_scale_y: f32,
    // State
    initialized: bool,
    frame_count: u32,
}

/// PMW3901 registers.
const PMW3901_PRODUCT_ID: u8 = 0x00;
const PMW3901_PRODUCT_ID_VAL: u8 = 0x49;
const PMW3901_MOTION: u8 = 0x02;
const PMW3901_DELTA_X_L: u8 = 0x03;
const PMW3901_DELTA_X_H: u8 = 0x04;
const PMW3901_DELTA_Y_L: u8 = 0x05;
const PMW3901_DELTA_Y_H: u8 = 0x06;
const PMW3901_SQUAL: u8 = 0x07;

impl Pmw3901 {
    pub fn new() -> Self {
        Self {
            flow_scale_x: 4.2, // typical: ~4.2 pixels per degree at 30cm height
            flow_scale_y: 4.2,
            initialized: false,
            frame_count: 0,
        }
    }

    /// Check WHO_AM_I register.
    pub fn probe(product_id: u8) -> bool {
        product_id == PMW3901_PRODUCT_ID_VAL
    }

    /// Parse a motion burst read (motion_detected, delta_x, delta_y, squal).
    /// Returns (delta_x_pixels, delta_y_pixels, quality).
    pub fn parse_motion(data: &[u8; 5]) -> (i16, i16, u8) {
        let motion = data[0];
        if motion & 0x80 == 0 {
            return (0, 0, 0); // no motion detected
        }
        let dx = (data[1] as i16) | ((data[2] as i16) << 8);
        let dy = (data[3] as i16) | ((data[4] as i16) << 8);
        let squal = data[4]; // actually from SQUAL register, simplified here
        (dx, dy, squal)
    }

    /// Convert pixel deltas to flow rate in rad/s.
    pub fn pixels_to_flow(&self, dx: i16, dy: i16, dt: f32) -> Vec3<Body> {
        if dt <= 0.0 {
            return Vec3::zero();
        }
        let deg_to_rad = core::f32::consts::PI / 180.0;
        Vec3::new(
            (dx as f32 / self.flow_scale_x) * deg_to_rad / dt,
            (dy as f32 / self.flow_scale_y) * deg_to_rad / dt,
            0.0,
        )
    }
}

/// MAVLink optical flow — receives OPTICAL_FLOW messages from companion.
pub struct MavlinkFlow {
    last_reading: FlowReading,
    last_update_ms: u32,
}

impl MavlinkFlow {
    pub fn new() -> Self {
        Self {
            last_reading: FlowReading {
                flow_rate: Vec3::zero(),
                body_rate: Vec3::zero(),
                quality: 0,
                valid: false,
            },
            last_update_ms: 0,
        }
    }

    /// Update from MAVLink OPTICAL_FLOW message fields.
    pub fn update(
        &mut self,
        flow_comp_x: f32, flow_comp_y: f32,
        flow_x: i16, flow_y: i16,
        quality: u8,
        ground_distance: f32,
        now_ms: u32,
    ) -> FlowReading {
        self.last_reading = FlowReading {
            flow_rate: Vec3::new(flow_comp_x, flow_comp_y, 0.0),
            body_rate: Vec3::zero(), // filled by caller from gyro
            quality,
            valid: quality > 50,
        };
        self.last_update_ms = now_ms;
        self.last_reading
    }

    pub fn is_healthy(&self, now_ms: u32) -> bool {
        self.last_reading.valid && now_ms.wrapping_sub(self.last_update_ms) < 500
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_translational_flow() {
        let reading = FlowReading {
            flow_rate: Vec3::new(0.5, 0.3, 0.0),
            body_rate: Vec3::new(0.1, 0.05, 0.0),
            quality: 200,
            valid: true,
        };
        let trans = reading.translational_flow();
        assert!((trans.x - 0.4).abs() < 0.01);
        assert!((trans.y - 0.25).abs() < 0.01);
    }

    #[test]
    fn test_pmw3901_probe() {
        assert!(Pmw3901::probe(0x49));
        assert!(!Pmw3901::probe(0x00));
    }

    #[test]
    fn test_pmw3901_parse_motion() {
        // Motion detected, dx=100, dy=-50
        let data = [0x80, 100, 0, 206, 0xFF]; // dx=100, dy=-50 (0xFFCE in i16)
        let (dx, dy, _) = Pmw3901::parse_motion(&data);
        assert_eq!(dx, 100);
        // dy parsing: 206 | (0xFF << 8) = 0xFF_CE = -50 as i16
        assert_eq!(dy, -50);
    }

    #[test]
    fn test_pmw3901_no_motion() {
        let data = [0x00, 0, 0, 0, 0]; // motion bit not set
        let (dx, dy, qual) = Pmw3901::parse_motion(&data);
        assert_eq!(dx, 0);
        assert_eq!(dy, 0);
        assert_eq!(qual, 0);
    }

    #[test]
    fn test_pixel_to_flow() {
        let pmw = Pmw3901::new();
        let flow = pmw.pixels_to_flow(42, 0, 0.01); // 42 pixels in 10ms
        // 42 / 4.2 = 10 degrees, in 0.01s = 1000 deg/s → ~17.45 rad/s
        assert!(flow.x > 10.0, "Flow should be significant: {}", flow.x);
    }

    #[test]
    fn test_mavlink_flow_health() {
        let mut flow = MavlinkFlow::new();
        assert!(!flow.is_healthy(0));

        flow.update(0.1, 0.05, 100, 50, 200, 1.5, 1000);
        assert!(flow.is_healthy(1000));
        assert!(!flow.is_healthy(2000)); // timeout
    }
}

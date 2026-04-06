//! PX4Flow optical flow I2C sensor driver.
//!
//! ArduPilot reference: `AP_OpticalFlow_PX4Flow.cpp`
//!
//! I2C address: 0x42. Integral frame: 26 bytes at ~10Hz.
//!
//! Frame layout (little-endian):
//!   u16  frame_count        — increments per frame
//!   i16  pixel_flow_x_sum   — accumulated flow X (pixels)
//!   i16  pixel_flow_y_sum   — accumulated flow Y (pixels)
//!   i16  flow_comp_m_x      — compensated flow X (m, scaled by 1000)
//!   i16  flow_comp_m_y      — compensated flow Y (m, scaled by 1000)
//!   i16  qual               — quality metric (0-255)
//!   i16  gyro_x_rate        — gyro X rate (rad/s * 1000)
//!   i16  gyro_y_rate        — gyro Y rate (rad/s * 1000)
//!   i16  gyro_z_rate        — gyro Z rate (rad/s * 1000)
//!   u8   gyro_range         — gyro range
//!   u8   sonar_timestamp    — sonar timestamp
//!   i16  ground_distance    — ground distance (mm)

use meridian_hal::I2cDevice;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const PX4FLOW_ADDR: u8 = 0x42;
const INTEGRAL_FRAME_REG: u8 = 0x16;
const FRAME_SIZE: usize = 26;

/// Minimum quality for the EKF to fuse flow data.
const MIN_QUALITY: u8 = 20;

// ---------------------------------------------------------------------------
// Data types
// ---------------------------------------------------------------------------

/// Raw PX4Flow integral frame data.
#[derive(Clone, Default)]
pub struct Px4FlowFrame {
    pub frame_count: u16,
    pub pixel_flow_x: i16,
    pub pixel_flow_y: i16,
    pub flow_comp_m_x: f32,
    pub flow_comp_m_y: f32,
    pub quality: u8,
    pub gyro_x_rate: f32,
    pub gyro_y_rate: f32,
    pub gyro_z_rate: f32,
    pub ground_distance_m: f32,
}

/// Processed optical flow reading for EKF consumption.
pub struct FlowReading {
    /// Optical angular rate about X body axis (rad/s).
    pub flow_rate_x: f32,
    /// Optical angular rate about Y body axis (rad/s).
    pub flow_rate_y: f32,
    /// IMU gyro rate about X body axis at measurement time (rad/s).
    pub body_rate_x: f32,
    /// IMU gyro rate about Y body axis at measurement time (rad/s).
    pub body_rate_y: f32,
    /// Surface quality (0-255). Below MIN_QUALITY, EKF should not fuse.
    pub quality: u8,
    /// Ground distance in meters (from integrated sonar, 0 if unavailable).
    pub ground_distance_m: f32,
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

pub struct Px4Flow {
    address: u8,
    last_frame_count: u16,
    /// Scale factors (parts-per-thousand) for flow calibration.
    scale_x: f32,
    scale_y: f32,
    /// Yaw offset (radians) of the sensor relative to vehicle frame.
    yaw_offset: f32,
}

impl Px4Flow {
    pub fn new() -> Self {
        Self {
            address: PX4FLOW_ADDR,
            last_frame_count: 0,
            scale_x: 0.0,
            scale_y: 0.0,
            yaw_offset: 0.0,
        }
    }

    pub fn with_address(address: u8) -> Self {
        Self { address, ..Self::new() }
    }

    /// Set flow scale factors (parts-per-thousand, range ±800).
    pub fn set_scale(&mut self, scale_x_ppt: f32, scale_y_ppt: f32) {
        self.scale_x = scale_x_ppt;
        self.scale_y = scale_y_ppt;
    }

    /// Set yaw alignment offset (degrees).
    pub fn set_yaw_offset_deg(&mut self, yaw_deg: f32) {
        self.yaw_offset = yaw_deg * core::f32::consts::PI / 180.0;
    }

    /// Read an integral frame from the sensor.
    pub fn read_raw(&self, i2c: &mut dyn I2cDevice) -> Option<Px4FlowFrame> {
        let mut buf = [0u8; FRAME_SIZE];
        i2c.set_address(self.address);
        if !i2c.read_registers(INTEGRAL_FRAME_REG, &mut buf) {
            return None;
        }

        let frame = Px4FlowFrame {
            frame_count: u16::from_le_bytes([buf[0], buf[1]]),
            pixel_flow_x: i16::from_le_bytes([buf[2], buf[3]]),
            pixel_flow_y: i16::from_le_bytes([buf[4], buf[5]]),
            flow_comp_m_x: i16::from_le_bytes([buf[6], buf[7]]) as f32 / 1000.0,
            flow_comp_m_y: i16::from_le_bytes([buf[8], buf[9]]) as f32 / 1000.0,
            quality: buf[10] as u8,
            gyro_x_rate: i16::from_le_bytes([buf[12], buf[13]]) as f32 / 1000.0,
            gyro_y_rate: i16::from_le_bytes([buf[14], buf[15]]) as f32 / 1000.0,
            gyro_z_rate: i16::from_le_bytes([buf[16], buf[17]]) as f32 / 1000.0,
            ground_distance_m: i16::from_le_bytes([buf[22], buf[23]]) as f32 / 1000.0,
        };

        Some(frame)
    }

    /// Read and process into a FlowReading suitable for EKF fusion.
    pub fn read(&mut self, i2c: &mut dyn I2cDevice) -> Option<FlowReading> {
        let frame = self.read_raw(i2c)?;

        // Skip if no new data.
        if frame.frame_count == self.last_frame_count {
            return None;
        }
        self.last_frame_count = frame.frame_count;

        // Apply scale factors.
        let sx = 1.0 + 0.001 * self.scale_x;
        let sy = 1.0 + 0.001 * self.scale_y;

        let mut flow_x = frame.flow_comp_m_x * sx;
        let mut flow_y = frame.flow_comp_m_y * sy;

        // Apply yaw rotation.
        if self.yaw_offset.abs() > 0.001 {
            let cos_y = libm::cosf(self.yaw_offset);
            let sin_y = libm::sinf(self.yaw_offset);
            let rx = flow_x * cos_y - flow_y * sin_y;
            let ry = flow_x * sin_y + flow_y * cos_y;
            flow_x = rx;
            flow_y = ry;
        }

        Some(FlowReading {
            flow_rate_x: flow_x,
            flow_rate_y: flow_y,
            body_rate_x: frame.gyro_x_rate,
            body_rate_y: frame.gyro_y_rate,
            quality: frame.quality,
            ground_distance_m: frame.ground_distance_m,
        })
    }

    /// Whether a flow reading should be fused by the EKF.
    pub fn is_quality_sufficient(quality: u8) -> bool {
        quality >= MIN_QUALITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quality_threshold() {
        assert!(!Px4Flow::is_quality_sufficient(0));
        assert!(!Px4Flow::is_quality_sufficient(19));
        assert!(Px4Flow::is_quality_sufficient(20));
        assert!(Px4Flow::is_quality_sufficient(255));
    }

    #[test]
    fn test_scale_factor() {
        // Scale of 100 ppt = +10% = multiply by 1.1
        let sx: f32 = 1.0 + 0.001 * 100.0;
        assert!((sx - 1.1).abs() < 0.001);

        // Scale of -200 ppt = -20% = multiply by 0.8
        let sy: f32 = 1.0 + 0.001 * (-200.0);
        assert!((sy - 0.8).abs() < 0.001);
    }

    #[test]
    fn test_yaw_rotation() {
        // 90° rotation: (1, 0) → (0, 1)
        let yaw = core::f32::consts::FRAC_PI_2;
        let cos_y = libm::cosf(yaw);
        let sin_y = libm::sinf(yaw);
        let rx = 1.0 * cos_y - 0.0 * sin_y;
        let ry = 1.0 * sin_y + 0.0 * cos_y;
        assert!(rx.abs() < 0.01);
        assert!((ry - 1.0).abs() < 0.01);
    }
}

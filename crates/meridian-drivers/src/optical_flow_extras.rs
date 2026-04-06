//! Additional optical flow drivers: PMW3901 SPI, MAVLink, CXOF serial.

use crate::optical_flow_px4flow::FlowReading;

// ---------------------------------------------------------------------------
// PMW3901 SPI optical flow (PixArt sensor)
// ---------------------------------------------------------------------------

/// PMW3901 SPI optical flow sensor.
///
/// ArduPilot reference: `AP_OpticalFlow_Pixart.cpp`
///
/// SPI interface. Scans for "pixartflow" or "pixartPC15" during init.
/// Outputs delta_x/delta_y pixel counts per frame at configurable rate.
/// Typical: 80x80 pixel array, up to ~7.4 rad/s tracking.
pub struct Pmw3901 {
    /// Accumulated pixel delta X since last read.
    delta_x: i16,
    /// Accumulated pixel delta Y since last read.
    delta_y: i16,
    /// Surface quality (0-255).
    quality: u8,
    /// Conversion factor: radians per pixel count.
    rad_per_pixel: f32,
    /// Gyro body rate at time of measurement (injected from IMU).
    body_rate_x: f32,
    body_rate_y: f32,
}

impl Pmw3901 {
    pub fn new() -> Self {
        Self {
            delta_x: 0, delta_y: 0, quality: 0,
            // Default: ~4.2 degrees FOV per pixel for 35-degree lens / 80 pixels.
            rad_per_pixel: (35.0 * core::f32::consts::PI / 180.0) / 80.0,
            body_rate_x: 0.0, body_rate_y: 0.0,
        }
    }

    /// Set the body rate from the IMU at the time of flow measurement.
    pub fn set_body_rate(&mut self, rate_x: f32, rate_y: f32) {
        self.body_rate_x = rate_x;
        self.body_rate_y = rate_y;
    }

    /// Handle raw SPI motion data from PMW3901 motion registers.
    ///
    /// Registers 0x02-0x06: Motion, Delta_X_L, Delta_X_H, Delta_Y_L, Delta_Y_H.
    pub fn handle_motion(&mut self, motion: u8, dx_l: u8, dx_h: u8, dy_l: u8, dy_h: u8) -> Option<FlowReading> {
        if motion & 0x80 == 0 { return None; } // No motion detected.

        self.delta_x = i16::from_le_bytes([dx_l, dx_h]);
        self.delta_y = i16::from_le_bytes([dy_l, dy_h]);
        self.quality = if motion & 0x80 != 0 { 128 } else { 0 };

        let flow_x = self.delta_x as f32 * self.rad_per_pixel;
        let flow_y = self.delta_y as f32 * self.rad_per_pixel;

        Some(FlowReading {
            flow_rate_x: flow_x,
            flow_rate_y: flow_y,
            body_rate_x: self.body_rate_x,
            body_rate_y: self.body_rate_y,
            quality: self.quality,
            ground_distance_m: 0.0, // PMW3901 has no sonar.
        })
    }
}

// ---------------------------------------------------------------------------
// MAVLink OPTICAL_FLOW receiver
// ---------------------------------------------------------------------------

/// Receives MAVLink OPTICAL_FLOW (msg 100) messages.
pub struct MavlinkOpticalFlow {
    last_update_ms: u32,
}

impl MavlinkOpticalFlow {
    pub fn new() -> Self {
        Self { last_update_ms: 0 }
    }

    /// Handle a MAVLINK OPTICAL_FLOW message.
    ///
    /// `flow_comp_m_x/y`: Flow in meters, compensated (already angular rate * dt).
    /// `quality`: Image quality (0-255).
    /// `ground_distance`: Distance to ground (m), -1 if unavailable.
    pub fn handle_message(
        &mut self,
        now_ms: u32,
        flow_comp_m_x: f32,
        flow_comp_m_y: f32,
        quality: u8,
        ground_distance: f32,
    ) -> FlowReading {
        self.last_update_ms = now_ms;
        FlowReading {
            flow_rate_x: flow_comp_m_x,
            flow_rate_y: flow_comp_m_y,
            body_rate_x: 0.0, // MAVLink flow is already compensated.
            body_rate_y: 0.0,
            quality,
            ground_distance_m: if ground_distance > 0.0 { ground_distance } else { 0.0 },
        }
    }

    pub fn is_healthy(&self, now_ms: u32) -> bool {
        now_ms.wrapping_sub(self.last_update_ms) < 500
    }
}

// ---------------------------------------------------------------------------
// CXOF serial optical flow (Cheerson CX-OF)
// ---------------------------------------------------------------------------

/// Cheerson CX-OF serial optical flow sensor.
///
/// ArduPilot reference: `AP_OpticalFlow_CXOF.cpp`
///
/// 9-byte serial frame at 25Hz:
///   Header: 0xFE
///   Byte 1: quality (0-255)
///   Byte 2-3: delta_x (i16 LE, 0.1 pixels)
///   Byte 4-5: delta_y (i16 LE, 0.1 pixels)
///   Byte 6-8: reserved
///   Checksum: sum of bytes 0..8 & 0xFF
pub struct CxofFlow {
    buf: [u8; 9],
    pos: usize,
    body_rate_x: f32,
    body_rate_y: f32,
    /// FOV conversion: radians per 0.1-pixel count.
    rad_per_tenth_pixel: f32,
}

impl CxofFlow {
    pub fn new() -> Self {
        Self {
            buf: [0; 9], pos: 0,
            body_rate_x: 0.0, body_rate_y: 0.0,
            // Approximate: 42 degree FOV, ~35 pixels → ~0.021 rad/pixel, /10 for 0.1-pixel units.
            rad_per_tenth_pixel: (42.0 * core::f32::consts::PI / 180.0) / 35.0 / 10.0,
        }
    }

    pub fn set_body_rate(&mut self, rate_x: f32, rate_y: f32) {
        self.body_rate_x = rate_x;
        self.body_rate_y = rate_y;
    }

    pub fn process_byte(&mut self, byte: u8) -> Option<FlowReading> {
        if self.pos == 0 && byte != 0xFE { return None; }

        self.buf[self.pos] = byte;
        self.pos += 1;

        if self.pos < 9 { return None; }
        self.pos = 0;

        // Checksum: sum of all 9 bytes should have low byte == 0 (or validate bytes 0..8 vs byte 9).
        // ArduPilot checks: sum of bytes 0..7 == byte 8.
        let sum: u8 = self.buf[..8].iter().fold(0u8, |a, &b| a.wrapping_add(b));
        if sum != self.buf[8] { return None; }

        let quality = self.buf[1];
        let delta_x = i16::from_le_bytes([self.buf[2], self.buf[3]]);
        let delta_y = i16::from_le_bytes([self.buf[4], self.buf[5]]);

        let flow_x = delta_x as f32 * self.rad_per_tenth_pixel;
        let flow_y = delta_y as f32 * self.rad_per_tenth_pixel;

        Some(FlowReading {
            flow_rate_x: flow_x,
            flow_rate_y: flow_y,
            body_rate_x: self.body_rate_x,
            body_rate_y: self.body_rate_y,
            quality,
            ground_distance_m: 0.0,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pmw3901_no_motion() {
        let mut drv = Pmw3901::new();
        assert!(drv.handle_motion(0x00, 0, 0, 0, 0).is_none());
    }

    #[test]
    fn test_pmw3901_motion() {
        let mut drv = Pmw3901::new();
        drv.set_body_rate(0.1, 0.2);
        let r = drv.handle_motion(0x80, 10, 0, 20, 0).expect("should detect motion");
        assert!(r.flow_rate_x > 0.0);
        assert!(r.flow_rate_y > 0.0);
        assert!((r.body_rate_x - 0.1).abs() < 0.001);
    }

    #[test]
    fn test_cxof_parse() {
        let mut drv = CxofFlow::new();
        let mut frame = [0xFE, 128, 50, 0, 100, 0, 0, 0, 0]; // quality=128, dx=50, dy=100
        frame[8] = frame[..8].iter().fold(0u8, |a, &b| a.wrapping_add(b));

        let mut result = None;
        for &b in &frame {
            if let Some(r) = drv.process_byte(b) { result = Some(r); }
        }
        let r = result.expect("should parse");
        assert_eq!(r.quality, 128);
        assert!(r.flow_rate_x > 0.0);
    }
}

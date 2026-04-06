//! Benewake TFMini / TF02 / TF03 serial rangefinder drivers.
//!
//! ArduPilot reference: `AP_RangeFinder_Benewake*.cpp`
//!
//! All Benewake lidars use the same 9-byte serial protocol:
//!   Byte 0: 0x59 (header 1)
//!   Byte 1: 0x59 (header 2)
//!   Byte 2: Dist_L (distance low byte, cm)
//!   Byte 3: Dist_H (distance high byte, cm)
//!   Byte 4: Strength_L (signal strength low)
//!   Byte 5: Strength_H (signal strength high)
//!   Byte 6: Temperature low (TFMini-Plus only) or reserved
//!   Byte 7: Temperature high (TFMini-Plus only) or reserved
//!   Byte 8: Checksum (lower 8 bits of sum of bytes 0..7)
//!
//! Variants:
//!   TFMini:  0.3m - 12m, 115200 baud
//!   TF02:    0.4m - 22m, 115200 baud
//!   TF03:    0.1m - 180m, 115200 baud
//!   TFMini-Plus: 0.1m - 12m, 115200 baud, has temperature byte

use meridian_hal::UartDriver;
use crate::rangefinder::{RangefinderReading, RangefinderStatus};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const HEADER_BYTE: u8 = 0x59;
const FRAME_SIZE: usize = 9;

/// Minimum signal strength to consider a valid reading.
/// Below this, the target is not reflective enough or too far.
const MIN_STRENGTH_TFMINI: u16 = 100;
const MIN_STRENGTH_TF02: u16 = 60;
const MIN_STRENGTH_TF03: u16 = 100;

/// Strength value indicating sensor saturation (too close).
const STRENGTH_SATURATION: u16 = 65535;

// ---------------------------------------------------------------------------
// Sensor variant
// ---------------------------------------------------------------------------

/// Which Benewake sensor variant.
#[derive(Clone, Copy, PartialEq)]
pub enum BenewakeModel {
    TFMini,
    TFMiniPlus,
    TF02,
    TF03,
}

impl BenewakeModel {
    /// Minimum valid distance in meters.
    pub fn min_distance_m(&self) -> f32 {
        match self {
            Self::TFMini => 0.3,
            Self::TFMiniPlus => 0.1,
            Self::TF02 => 0.4,
            Self::TF03 => 0.1,
        }
    }

    /// Maximum valid distance in meters.
    pub fn max_distance_m(&self) -> f32 {
        match self {
            Self::TFMini => 12.0,
            Self::TFMiniPlus => 12.0,
            Self::TF02 => 22.0,
            Self::TF03 => 180.0,
        }
    }

    /// Minimum signal strength for a valid reading.
    fn min_strength(&self) -> u16 {
        match self {
            Self::TFMini | Self::TFMiniPlus => MIN_STRENGTH_TFMINI,
            Self::TF02 => MIN_STRENGTH_TF02,
            Self::TF03 => MIN_STRENGTH_TF03,
        }
    }
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

pub struct BenewakeRangefinder {
    model: BenewakeModel,
    buf: [u8; FRAME_SIZE],
    buf_pos: usize,
    last_distance_m: f32,
    last_strength: u16,
    last_status: RangefinderStatus,
    valid_count: u8,
}

impl BenewakeRangefinder {
    pub fn new(model: BenewakeModel) -> Self {
        Self {
            model,
            buf: [0u8; FRAME_SIZE],
            buf_pos: 0,
            last_distance_m: 0.0,
            last_strength: 0,
            last_status: RangefinderStatus::NoData,
            valid_count: 0,
        }
    }

    /// Feed bytes from UART and return a reading when a complete frame is parsed.
    ///
    /// Call this in a loop with available serial bytes. Returns `Some` when a
    /// valid frame has been decoded.
    pub fn process_byte(&mut self, byte: u8) -> Option<RangefinderReading> {
        // State machine: look for header pair 0x59 0x59.
        match self.buf_pos {
            0 => {
                if byte == HEADER_BYTE {
                    self.buf[0] = byte;
                    self.buf_pos = 1;
                }
                return None;
            }
            1 => {
                if byte == HEADER_BYTE {
                    self.buf[1] = byte;
                    self.buf_pos = 2;
                } else {
                    self.buf_pos = 0;
                }
                return None;
            }
            _ => {
                self.buf[self.buf_pos] = byte;
                self.buf_pos += 1;

                if self.buf_pos < FRAME_SIZE {
                    return None;
                }
            }
        }

        // Full frame received — validate and parse.
        self.buf_pos = 0;
        self.parse_frame()
    }

    /// Parse a complete 9-byte frame.
    fn parse_frame(&mut self) -> Option<RangefinderReading> {
        // Checksum: lower 8 bits of sum of bytes 0..7.
        let checksum: u8 = self.buf[..8].iter().fold(0u8, |acc, &b| acc.wrapping_add(b));
        if checksum != self.buf[8] {
            return None;
        }

        let distance_cm = (self.buf[2] as u16) | ((self.buf[3] as u16) << 8);
        let strength = (self.buf[4] as u16) | ((self.buf[5] as u16) << 8);
        let distance_m = distance_cm as f32 / 100.0;

        self.last_strength = strength;

        // Determine status.
        let status = if strength < self.model.min_strength() || strength == STRENGTH_SATURATION {
            self.valid_count = 0;
            RangefinderStatus::NoData
        } else if distance_m > self.model.max_distance_m() {
            self.valid_count = 0;
            RangefinderStatus::OutOfRangeHigh
        } else if distance_m < self.model.min_distance_m() {
            self.valid_count = 0;
            RangefinderStatus::OutOfRangeLow
        } else {
            self.valid_count = self.valid_count.saturating_add(1).min(10);
            RangefinderStatus::Good
        };

        self.last_distance_m = distance_m;
        self.last_status = status;

        // Signal quality: map strength to 0-100%.
        let quality = if strength > 0 && strength < STRENGTH_SATURATION {
            ((strength as f32 / 1000.0).min(1.0) * 100.0) as u8
        } else {
            0
        };

        Some(RangefinderReading {
            distance_m,
            status,
            signal_quality: quality as i8,
            valid_count: self.valid_count,
        })
    }

    /// Convenience: read all available bytes from a UART and return the latest reading.
    pub fn update(&mut self, uart: &mut dyn UartDriver) -> Option<RangefinderReading> {
        let mut last_reading = None;
        let mut byte_buf = [0u8; 1];

        // Drain available bytes.
        while uart.read(&mut byte_buf) > 0 {
            if let Some(reading) = self.process_byte(byte_buf[0]) {
                last_reading = Some(reading);
            }
        }

        last_reading
    }

    pub fn model(&self) -> BenewakeModel {
        self.model
    }

    pub fn last_status(&self) -> RangefinderStatus {
        self.last_status
    }

    pub fn last_strength(&self) -> u16 {
        self.last_strength
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_frame(dist_cm: u16, strength: u16) -> [u8; 9] {
        let mut frame = [0u8; 9];
        frame[0] = 0x59;
        frame[1] = 0x59;
        frame[2] = (dist_cm & 0xFF) as u8;
        frame[3] = (dist_cm >> 8) as u8;
        frame[4] = (strength & 0xFF) as u8;
        frame[5] = (strength >> 8) as u8;
        frame[6] = 0;
        frame[7] = 0;
        frame[8] = frame[..8].iter().fold(0u8, |a, &b| a.wrapping_add(b));
        frame
    }

    #[test]
    fn test_valid_frame() {
        let mut drv = BenewakeRangefinder::new(BenewakeModel::TFMini);
        let frame = make_frame(350, 500); // 3.50m, strength 500

        let mut result = None;
        for &byte in &frame {
            if let Some(r) = drv.process_byte(byte) {
                result = Some(r);
            }
        }

        let r = result.expect("should parse valid frame");
        assert!((r.distance_m - 3.5).abs() < 0.01);
        assert_eq!(r.status, RangefinderStatus::Good);
        assert!(r.signal_quality > 0);
    }

    #[test]
    fn test_out_of_range() {
        let mut drv = BenewakeRangefinder::new(BenewakeModel::TFMini);
        let frame = make_frame(1500, 500); // 15.0m — beyond TFMini max of 12m

        let mut result = None;
        for &byte in &frame {
            if let Some(r) = drv.process_byte(byte) {
                result = Some(r);
            }
        }

        let r = result.expect("should parse frame");
        assert_eq!(r.status, RangefinderStatus::OutOfRangeHigh);
    }

    #[test]
    fn test_low_signal() {
        let mut drv = BenewakeRangefinder::new(BenewakeModel::TFMini);
        let frame = make_frame(200, 50); // strength below MIN_STRENGTH_TFMINI

        let mut result = None;
        for &byte in &frame {
            if let Some(r) = drv.process_byte(byte) {
                result = Some(r);
            }
        }

        let r = result.expect("should parse frame");
        assert_eq!(r.status, RangefinderStatus::NoData);
    }

    #[test]
    fn test_bad_checksum() {
        let mut drv = BenewakeRangefinder::new(BenewakeModel::TFMini);
        let mut frame = make_frame(350, 500);
        frame[8] = 0x00; // corrupt checksum

        let mut result = None;
        for &byte in &frame {
            if let Some(r) = drv.process_byte(byte) {
                result = Some(r);
            }
        }

        assert!(result.is_none(), "bad checksum should not produce reading");
    }

    #[test]
    fn test_tf03_long_range() {
        let mut drv = BenewakeRangefinder::new(BenewakeModel::TF03);
        let frame = make_frame(15000, 300); // 150.0m — within TF03 range

        let mut result = None;
        for &byte in &frame {
            if let Some(r) = drv.process_byte(byte) {
                result = Some(r);
            }
        }

        let r = result.expect("should parse frame");
        assert!((r.distance_m - 150.0).abs() < 0.01);
        assert_eq!(r.status, RangefinderStatus::Good);
    }

    #[test]
    fn test_header_sync() {
        let mut drv = BenewakeRangefinder::new(BenewakeModel::TFMini);
        let frame = make_frame(200, 500);

        // Feed garbage before the frame.
        for &byte in &[0x00, 0x12, 0x59, 0x34, 0xFF] {
            assert!(drv.process_byte(byte).is_none());
        }

        // Now feed the real frame.
        let mut result = None;
        for &byte in &frame {
            if let Some(r) = drv.process_byte(byte) {
                result = Some(r);
            }
        }

        assert!(result.is_some(), "should sync on valid frame after garbage");
    }
}

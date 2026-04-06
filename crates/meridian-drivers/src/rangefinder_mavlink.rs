//! MAVLink DISTANCE_SENSOR and DroneCAN rangefinder receivers.
//!
//! These aren't hardware drivers — they receive distance data from
//! MAVLink messages or DroneCAN nodes and present it as rangefinder readings.

use crate::rangefinder::{RangefinderReading, RangefinderStatus};

// ---------------------------------------------------------------------------
// MAVLink DISTANCE_SENSOR receiver
// ---------------------------------------------------------------------------

/// Receives MAVLink DISTANCE_SENSOR (msg 132) and provides rangefinder data.
pub struct MavlinkRangefinder {
    min_distance_m: f32,
    max_distance_m: f32,
    orientation: u8,
    valid_count: u8,
    last_update_ms: u32,
}

impl MavlinkRangefinder {
    pub fn new(orientation: u8, min_m: f32, max_m: f32) -> Self {
        Self {
            min_distance_m: min_m,
            max_distance_m: max_m,
            orientation,
            valid_count: 0,
            last_update_ms: 0,
        }
    }

    /// Handle a DISTANCE_SENSOR MAVLink message.
    ///
    /// `distance_cm`: measured distance in centimeters.
    /// `orientation`: MAV_SENSOR_ORIENTATION (25 = downward).
    /// `covariance`: measurement covariance (255 = unknown).
    /// `signal_quality`: 0-100 (0 = unknown).
    pub fn handle_message(
        &mut self,
        now_ms: u32,
        distance_cm: u16,
        orientation: u8,
        signal_quality: u8,
    ) -> Option<RangefinderReading> {
        if orientation != self.orientation { return None; }

        let distance_m = distance_cm as f32 / 100.0;
        self.last_update_ms = now_ms;

        let status = if distance_m > self.max_distance_m {
            self.valid_count = 0;
            RangefinderStatus::OutOfRangeHigh
        } else if distance_m < self.min_distance_m {
            self.valid_count = 0;
            RangefinderStatus::OutOfRangeLow
        } else {
            self.valid_count = self.valid_count.saturating_add(1).min(10);
            RangefinderStatus::Good
        };

        Some(RangefinderReading {
            distance_m,
            status,
            signal_quality: if signal_quality > 0 { signal_quality as i8 } else { if status == RangefinderStatus::Good { 100 } else { 0 } },
            valid_count: self.valid_count,
        })
    }

    pub fn is_healthy(&self, now_ms: u32) -> bool {
        now_ms.wrapping_sub(self.last_update_ms) < 1000
    }
}

// ---------------------------------------------------------------------------
// DroneCAN rangefinder receiver
// ---------------------------------------------------------------------------

/// Receives range data from a DroneCAN rangefinder node.
pub struct DroneCanRangefinder {
    node_id: u8,
    orientation: u8,
    valid_count: u8,
    last_update_ms: u32,
}

impl DroneCanRangefinder {
    pub fn new(node_id: u8, orientation: u8) -> Self {
        Self { node_id, orientation, valid_count: 0, last_update_ms: 0 }
    }

    /// Handle a decoded `uavcan.equipment.range_sensor.Measurement` message.
    pub fn handle_measurement(
        &mut self,
        now_ms: u32,
        range_m: f32,
        reading_type: u8, // 0=undefined, 1=valid, 2=too_close, 3=too_far
    ) -> Option<RangefinderReading> {
        self.last_update_ms = now_ms;

        let status = match reading_type {
            1 => {
                self.valid_count = self.valid_count.saturating_add(1).min(10);
                RangefinderStatus::Good
            }
            2 => { self.valid_count = 0; RangefinderStatus::OutOfRangeLow }
            3 => { self.valid_count = 0; RangefinderStatus::OutOfRangeHigh }
            _ => { self.valid_count = 0; RangefinderStatus::NoData }
        };

        Some(RangefinderReading {
            distance_m: range_m,
            status,
            signal_quality: if status == RangefinderStatus::Good { 80 } else { 0 },
            valid_count: self.valid_count,
        })
    }

    pub fn is_healthy(&self, now_ms: u32) -> bool {
        now_ms.wrapping_sub(self.last_update_ms) < 1000
    }
}

// ---------------------------------------------------------------------------
// Maxbotix serial rangefinder
// ---------------------------------------------------------------------------

/// Maxbotix serial rangefinder (ASCII protocol).
///
/// Output: "Rxxxx\r" where xxxx is range in mm (or cm depending on model).
/// Baud: 9600 (LV-MaxSonar) or 57600 (HRLV-MaxSonar).
pub struct MaxbotixSerial {
    buf: [u8; 8],
    pos: usize,
    scale_mm: bool, // true = mm output, false = cm output
    valid_count: u8,
}

impl MaxbotixSerial {
    /// Create for mm-output models (HRLV-MaxSonar).
    pub fn new_mm() -> Self {
        Self { buf: [0; 8], pos: 0, scale_mm: true, valid_count: 0 }
    }

    /// Create for cm-output models (LV-MaxSonar).
    pub fn new_cm() -> Self {
        Self { buf: [0; 8], pos: 0, scale_mm: false, valid_count: 0 }
    }

    pub fn process_byte(&mut self, byte: u8) -> Option<RangefinderReading> {
        if byte == b'R' {
            self.pos = 0;
            return None;
        }

        if byte == b'\r' {
            if self.pos == 0 { return None; }
            let val = self.parse_value();
            self.pos = 0;
            return val;
        }

        if self.pos < 7 && byte >= b'0' && byte <= b'9' {
            self.buf[self.pos] = byte;
            self.pos += 1;
        }
        None
    }

    fn parse_value(&mut self) -> Option<RangefinderReading> {
        let mut val: u32 = 0;
        for i in 0..self.pos {
            val = val * 10 + (self.buf[i] - b'0') as u32;
        }

        let distance_m = if self.scale_mm {
            val as f32 / 1000.0
        } else {
            val as f32 / 100.0
        };

        let status = if distance_m < 0.02 || distance_m > 10.0 {
            self.valid_count = 0;
            if distance_m > 10.0 { RangefinderStatus::OutOfRangeHigh } else { RangefinderStatus::OutOfRangeLow }
        } else {
            self.valid_count = self.valid_count.saturating_add(1).min(10);
            RangefinderStatus::Good
        };

        Some(RangefinderReading {
            distance_m,
            status,
            signal_quality: if status == RangefinderStatus::Good { 80 } else { 0 },
            valid_count: self.valid_count,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_maxbotix_parse() {
        let mut drv = MaxbotixSerial::new_cm();
        // "R0150\r" = 150 cm = 1.50 m
        let msg = b"R0150\r";
        let mut result = None;
        for &b in msg { if let Some(r) = drv.process_byte(b) { result = Some(r); } }
        let r = result.expect("should parse");
        assert!((r.distance_m - 1.5).abs() < 0.01);
        assert_eq!(r.status, RangefinderStatus::Good);
    }

    #[test]
    fn test_maxbotix_mm() {
        let mut drv = MaxbotixSerial::new_mm();
        let msg = b"R1500\r"; // 1500 mm = 1.50 m
        let mut result = None;
        for &b in msg { if let Some(r) = drv.process_byte(b) { result = Some(r); } }
        let r = result.expect("should parse");
        assert!((r.distance_m - 1.5).abs() < 0.01);
    }

    #[test]
    fn test_mavlink_rangefinder() {
        let mut drv = MavlinkRangefinder::new(25, 0.2, 50.0); // downward, 0.2-50m
        let r = drv.handle_message(1000, 350, 25, 80).expect("should handle");
        assert!((r.distance_m - 3.5).abs() < 0.01);
        assert_eq!(r.status, RangefinderStatus::Good);
        assert_eq!(r.signal_quality, 80);
    }
}

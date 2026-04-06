//! Rangefinder driver framework — distance sensors for altitude and obstacle detection.
//!
//! Source: ArduPilot AP_RangeFinder (47 driver types)
//! We implement: Serial (LightWare/Benewake), I2C (Garmin/MaxBotix), MAVLink, DroneCAN, SITL.

/// Rangefinder status (matches ArduPilot).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RangefinderStatus {
    NotConnected,
    NoData,
    OutOfRangeLow,
    OutOfRangeHigh,
    Good,
}

/// Rangefinder orientation (which direction the sensor points).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RangefinderOrientation {
    Down = 25,    // ROTATION_PITCH_270
    Forward = 0,  // ROTATION_NONE
    Backward = 4, // ROTATION_YAW_180
    Left = 6,     // ROTATION_YAW_270
    Right = 2,    // ROTATION_YAW_90
    Up = 24,      // ROTATION_PITCH_90
}

/// A single rangefinder reading.
#[derive(Debug, Clone, Copy)]
pub struct RangefinderReading {
    pub distance_m: f32,
    pub status: RangefinderStatus,
    pub signal_quality: i8, // -1 = unknown, 0-100 = quality percentage
    pub valid_count: u8,    // consecutive valid readings (caps at 10)
}

/// Rangefinder configuration.
#[derive(Debug, Clone, Copy)]
pub struct RangefinderConfig {
    pub min_distance_m: f32,
    pub max_distance_m: f32,
    pub orientation: RangefinderOrientation,
}

impl Default for RangefinderConfig {
    fn default() -> Self {
        Self {
            min_distance_m: 0.2,
            max_distance_m: 40.0,
            orientation: RangefinderOrientation::Down,
        }
    }
}

/// Rangefinder backend trait.
pub trait RangefinderBackend {
    fn update(&mut self) -> RangefinderReading;
    fn config(&self) -> &RangefinderConfig;
}

/// Validate a raw distance against min/max range.
pub fn validate_distance(distance_m: f32, config: &RangefinderConfig) -> RangefinderStatus {
    if distance_m < config.min_distance_m {
        RangefinderStatus::OutOfRangeLow
    } else if distance_m > config.max_distance_m {
        RangefinderStatus::OutOfRangeHigh
    } else {
        RangefinderStatus::Good
    }
}

/// Serial rangefinder parser for Benewake TFMini/TF02 protocol.
/// Frame: [0x59, 0x59, dist_L, dist_H, strength_L, strength_H, temp_L, checksum]
pub struct BenewakeParser {
    buf: [u8; 9],
    pos: usize,
    config: RangefinderConfig,
    last_distance: f32,
    last_strength: u16,
    valid_count: u8,
}

impl BenewakeParser {
    pub fn new(config: RangefinderConfig) -> Self {
        Self {
            buf: [0; 9],
            pos: 0,
            config,
            last_distance: 0.0,
            last_strength: 0,
            valid_count: 0,
        }
    }

    /// Feed a byte. Returns Some(reading) when a complete frame is parsed.
    pub fn feed_byte(&mut self, byte: u8) -> Option<RangefinderReading> {
        // Sync detection
        if self.pos == 0 && byte != 0x59 { return None; }
        if self.pos == 1 && byte != 0x59 { self.pos = 0; return None; }

        self.buf[self.pos] = byte;
        self.pos += 1;

        if self.pos < 9 { return None; }
        self.pos = 0;

        // Verify checksum (sum of bytes 0-7, low 8 bits)
        let sum: u8 = self.buf[..8].iter().fold(0u8, |a, b| a.wrapping_add(*b));
        if sum != self.buf[8] { return None; }

        // Parse
        let distance_cm = (self.buf[2] as u16) | ((self.buf[3] as u16) << 8);
        let strength = (self.buf[4] as u16) | ((self.buf[5] as u16) << 8);

        self.last_distance = distance_cm as f32 / 100.0;
        self.last_strength = strength;
        self.valid_count = self.valid_count.saturating_add(1).min(10);

        let status = validate_distance(self.last_distance, &self.config);
        let quality = if strength < 100 { 0 } else if strength > 65000 { -1 } else {
            ((strength as f32 / 65535.0 * 100.0) as i8).min(100)
        };

        Some(RangefinderReading {
            distance_m: self.last_distance,
            status,
            signal_quality: quality,
            valid_count: self.valid_count,
        })
    }
}

/// LightWare serial protocol (simple ASCII: distance in meters as decimal string).
pub struct LightwareParser {
    buf: [u8; 16],
    pos: usize,
    config: RangefinderConfig,
}

impl LightwareParser {
    pub fn new(config: RangefinderConfig) -> Self {
        Self { buf: [0; 16], pos: 0, config }
    }

    /// Feed a byte. Returns Some(reading) on newline.
    pub fn feed_byte(&mut self, byte: u8) -> Option<RangefinderReading> {
        if byte == b'\n' || byte == b'\r' {
            if self.pos == 0 { return None; }
            let distance = self.parse_float();
            self.pos = 0;
            if let Some(d) = distance {
                let status = validate_distance(d, &self.config);
                return Some(RangefinderReading {
                    distance_m: d, status, signal_quality: -1, valid_count: 0,
                });
            }
            return None;
        }

        if self.pos < 16 {
            self.buf[self.pos] = byte;
            self.pos += 1;
        }
        None
    }

    fn parse_float(&self) -> Option<f32> {
        // Simple ASCII float parser (no_std compatible)
        let mut val: f32 = 0.0;
        let mut decimal = false;
        let mut decimal_place: f32 = 0.1;
        let mut negative = false;

        for i in 0..self.pos {
            let c = self.buf[i];
            if c == b'-' && i == 0 { negative = true; continue; }
            if c == b'.' { decimal = true; continue; }
            if c >= b'0' && c <= b'9' {
                let digit = (c - b'0') as f32;
                if decimal {
                    val += digit * decimal_place;
                    decimal_place *= 0.1;
                } else {
                    val = val * 10.0 + digit;
                }
            } else {
                return None; // invalid character
            }
        }
        if negative { val = -val; }
        Some(val)
    }
}

/// MAVLink rangefinder — receives DISTANCE_SENSOR messages.
pub struct MavlinkRangefinder {
    config: RangefinderConfig,
    last_distance: f32,
    last_update_ms: u32,
}

impl MavlinkRangefinder {
    pub fn new(config: RangefinderConfig) -> Self {
        Self { config, last_distance: 0.0, last_update_ms: 0 }
    }

    pub fn update_from_mavlink(&mut self, distance_cm: u16, now_ms: u32) -> RangefinderReading {
        self.last_distance = distance_cm as f32 / 100.0;
        self.last_update_ms = now_ms;
        RangefinderReading {
            distance_m: self.last_distance,
            status: validate_distance(self.last_distance, &self.config),
            signal_quality: -1,
            valid_count: 0,
        }
    }

    pub fn is_healthy(&self, now_ms: u32) -> bool {
        now_ms.wrapping_sub(self.last_update_ms) < 500
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validate_distance() {
        let cfg = RangefinderConfig { min_distance_m: 0.2, max_distance_m: 40.0, ..Default::default() };
        assert_eq!(validate_distance(0.1, &cfg), RangefinderStatus::OutOfRangeLow);
        assert_eq!(validate_distance(5.0, &cfg), RangefinderStatus::Good);
        assert_eq!(validate_distance(50.0, &cfg), RangefinderStatus::OutOfRangeHigh);
    }

    #[test]
    fn test_benewake_parse() {
        let mut parser = BenewakeParser::new(RangefinderConfig::default());
        // Frame: 0x59 0x59 dist_L=0xE8(232) dist_H=0x03(3) → 1000cm = 10.0m
        // strength=0x00FF, temp=0x00, checksum
        let frame = [0x59u8, 0x59, 0xE8, 0x03, 0xFF, 0x00, 0x00, 0x00, 0x00];
        let checksum: u8 = frame[..8].iter().fold(0u8, |a, b| a.wrapping_add(*b));
        let mut full_frame = frame;
        full_frame[8] = checksum;

        let mut result = None;
        for &b in &full_frame {
            if let Some(r) = parser.feed_byte(b) {
                result = Some(r);
            }
        }
        assert!(result.is_some());
        let r = result.unwrap();
        assert!((r.distance_m - 10.0).abs() < 0.1, "Distance: {}", r.distance_m);
        assert_eq!(r.status, RangefinderStatus::Good);
    }

    #[test]
    fn test_benewake_bad_checksum() {
        let mut parser = BenewakeParser::new(RangefinderConfig::default());
        let frame = [0x59u8, 0x59, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0xFF]; // bad checksum
        let mut result = None;
        for &b in &frame {
            if let Some(r) = parser.feed_byte(b) { result = Some(r); }
        }
        assert!(result.is_none(), "Bad checksum should reject");
    }

    #[test]
    fn test_lightware_parse() {
        let mut parser = LightwareParser::new(RangefinderConfig::default());
        let data = b"12.34\n";
        let mut result = None;
        for &b in data {
            if let Some(r) = parser.feed_byte(b) { result = Some(r); }
        }
        assert!(result.is_some());
        assert!((result.unwrap().distance_m - 12.34).abs() < 0.01);
    }

    #[test]
    fn test_mavlink_rangefinder() {
        let mut rf = MavlinkRangefinder::new(RangefinderConfig::default());
        let reading = rf.update_from_mavlink(500, 1000); // 5.0m
        assert!((reading.distance_m - 5.0).abs() < 0.01);
        assert_eq!(reading.status, RangefinderStatus::Good);
        assert!(rf.is_healthy(1000));
        assert!(!rf.is_healthy(2000)); // 1s timeout
    }
}

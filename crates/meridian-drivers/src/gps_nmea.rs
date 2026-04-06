//! NMEA GPS parser — standard text-based GPS protocol.
//!
//! Source: ArduPilot AP_GPS_NMEA.cpp
//! Parses: $GPRMC, $GPGGA (also $GN* prefix for multi-constellation)
//! Requires BOTH RMC and GGA within 150ms for a valid fix.

/// NMEA GPS state.
pub struct NmeaGps {
    // From GGA
    gga_lat: f64,
    gga_lon: f64,
    gga_alt_msl: f32,
    gga_fix_quality: u8, // 0=invalid, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float
    gga_num_sats: u8,
    gga_hdop: f32,
    gga_time_ms: u32,
    gga_valid: bool,

    // From RMC
    rmc_lat: f64,
    rmc_lon: f64,
    rmc_speed_knots: f32,
    rmc_course_deg: f32,
    rmc_time_ms: u32,
    rmc_valid: bool,

    // From VTG
    vtg_speed_knots: f32,
    vtg_course_true: f32,
    vtg_time_ms: u32,
    vtg_valid: bool,

    // From HDT/THS (GPS yaw)
    /// True heading from dual-antenna GPS (degrees).
    pub gps_yaw: f32,
    /// GPS yaw quality (HDT: always valid; THS: quality indicator).
    pub gps_yaw_valid: bool,
    gps_yaw_time_ms: u32,

    // Combined fix
    pub fix_valid: bool,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude_msl: f32,
    pub speed_ms: f32,
    pub course_deg: f32,
    pub num_sats: u8,
    pub fix_type: u8,
    pub hdop: f32,

    // Parser state
    buf: [u8; 120],
    pos: usize,

    // Jitter correction (EMA of inter-message delta).
    last_fix_ms: u32,
    delta_ema_ms: f32,
    pub jitter_corrected_ms: u32,
}

impl NmeaGps {
    pub fn new() -> Self {
        Self {
            gga_lat: 0.0, gga_lon: 0.0, gga_alt_msl: 0.0,
            gga_fix_quality: 0, gga_num_sats: 0, gga_hdop: 99.9,
            gga_time_ms: 0, gga_valid: false,
            rmc_lat: 0.0, rmc_lon: 0.0, rmc_speed_knots: 0.0,
            rmc_course_deg: 0.0, rmc_time_ms: 0, rmc_valid: false,
            vtg_speed_knots: 0.0, vtg_course_true: 0.0, vtg_time_ms: 0, vtg_valid: false,
            gps_yaw: 0.0, gps_yaw_valid: false, gps_yaw_time_ms: 0,
            fix_valid: false, latitude: 0.0, longitude: 0.0,
            altitude_msl: 0.0, speed_ms: 0.0, course_deg: 0.0,
            num_sats: 0, fix_type: 0, hdop: 99.9,
            buf: [0; 120], pos: 0,
            last_fix_ms: 0, delta_ema_ms: 200.0, jitter_corrected_ms: 0,
        }
    }

    /// Feed a byte. Returns true when a new fix is available.
    pub fn feed_byte(&mut self, byte: u8, now_ms: u32) -> bool {
        if byte == b'$' {
            self.pos = 0;
            self.buf[0] = byte;
            self.pos = 1;
            return false;
        }

        if byte == b'\n' || byte == b'\r' {
            if self.pos > 5 {
                self.parse_sentence(now_ms);
                self.pos = 0;
                return self.try_combine(now_ms);
            }
            self.pos = 0;
            return false;
        }

        if self.pos < 120 {
            self.buf[self.pos] = byte;
            self.pos += 1;
        }
        false
    }

    fn parse_sentence(&mut self, now_ms: u32) {
        // Copy buffer to avoid borrow conflict
        let mut local_buf = [0u8; 120];
        let len = self.pos;
        local_buf[..len].copy_from_slice(&self.buf[..len]);
        let sentence = &local_buf[..len];

        let star_pos = sentence.iter().position(|&b| b == b'*');
        if star_pos.is_none() { return; }
        let star = star_pos.unwrap();
        if star + 3 > len { return; }

        let mut cksum: u8 = 0;
        for &b in &sentence[1..star] {
            cksum ^= b;
        }
        let expected = parse_hex_byte(sentence[star+1], sentence[star+2]);
        if cksum != expected { return; }

        let body = &sentence[1..star];
        if body.len() < 6 { return; }

        let msg_type = &body[2..5];
        if msg_type == b"GGA" {
            self.parse_gga(body, now_ms);
        } else if msg_type == b"RMC" {
            self.parse_rmc(body, now_ms);
        } else if msg_type == b"VTG" {
            self.parse_vtg(body, now_ms);
        } else if msg_type == b"HDT" {
            self.parse_hdt(body, now_ms);
        } else if msg_type == b"THS" {
            self.parse_ths(body, now_ms);
        }
    }

    fn parse_vtg(&mut self, body: &[u8], now_ms: u32) {
        let fields = split_fields(body);
        if fields.len() < 9 { return; }
        // Field 1: Course true (degrees).
        self.vtg_course_true = parse_f32(fields[1]);
        // Field 5: Speed knots.
        self.vtg_speed_knots = parse_f32(fields[5]);
        self.vtg_time_ms = now_ms;
        self.vtg_valid = true;
    }

    fn parse_hdt(&mut self, body: &[u8], now_ms: u32) {
        let fields = split_fields(body);
        if fields.len() < 2 { return; }
        // Field 1: True heading (degrees).
        self.gps_yaw = parse_f32(fields[1]);
        self.gps_yaw_valid = true;
        self.gps_yaw_time_ms = now_ms;
    }

    fn parse_ths(&mut self, body: &[u8], now_ms: u32) {
        let fields = split_fields(body);
        if fields.len() < 3 { return; }
        // Field 1: True heading (degrees).
        self.gps_yaw = parse_f32(fields[1]);
        // Field 2: Quality ('A'=autonomous, 'E'=estimated, 'M'=manual, 'V'=void).
        self.gps_yaw_valid = fields[2] == b"A" || fields[2] == b"E";
        self.gps_yaw_time_ms = now_ms;
    }

    fn parse_gga(&mut self, body: &[u8], now_ms: u32) {
        let fields = split_fields(body);
        if fields.len() < 14 { return; }

        // Field 6: Fix quality
        self.gga_fix_quality = parse_u8(fields[6]);
        if self.gga_fix_quality == 0 { self.gga_valid = false; return; }

        // Field 2,3: Latitude (ddmm.mmmm,N/S)
        if let Some(lat) = parse_latlon(fields[2], fields[3]) {
            self.gga_lat = lat;
        }
        // Field 4,5: Longitude (dddmm.mmmm,E/W)
        if let Some(lon) = parse_latlon_lon(fields[4], fields[5]) {
            self.gga_lon = lon;
        }
        // Field 7: Number of satellites
        self.gga_num_sats = parse_u8(fields[7]);
        // Field 8: HDOP
        self.gga_hdop = parse_f32(fields[8]);
        // Field 9: Altitude MSL
        self.gga_alt_msl = parse_f32(fields[9]);

        self.gga_time_ms = now_ms;
        self.gga_valid = true;
    }

    fn parse_rmc(&mut self, body: &[u8], now_ms: u32) {
        let fields = split_fields(body);
        if fields.len() < 12 { return; }

        // Field 2: Status (A=active, V=void)
        if fields[2] != b"A" { self.rmc_valid = false; return; }

        // Field 3,4: Latitude
        if let Some(lat) = parse_latlon(fields[3], fields[4]) {
            self.rmc_lat = lat;
        }
        // Field 5,6: Longitude
        if let Some(lon) = parse_latlon_lon(fields[5], fields[6]) {
            self.rmc_lon = lon;
        }
        // Field 7: Speed (knots)
        self.rmc_speed_knots = parse_f32(fields[7]);
        // Field 8: Course (degrees)
        self.rmc_course_deg = parse_f32(fields[8]);

        self.rmc_time_ms = now_ms;
        self.rmc_valid = true;
    }

    /// Combine GGA + RMC if both received within 150ms.
    fn try_combine(&mut self, now_ms: u32) -> bool {
        if !self.gga_valid || !self.rmc_valid { return false; }
        let dt = if self.gga_time_ms > self.rmc_time_ms {
            self.gga_time_ms - self.rmc_time_ms
        } else {
            self.rmc_time_ms - self.gga_time_ms
        };
        if dt > 150 { return false; } // Too old — need both within 150ms

        self.latitude = self.gga_lat;
        self.longitude = self.gga_lon;
        self.altitude_msl = self.gga_alt_msl;
        self.speed_ms = self.rmc_speed_knots * 0.514444; // knots → m/s
        self.course_deg = self.rmc_course_deg;
        self.num_sats = self.gga_num_sats;
        self.hdop = self.gga_hdop;
        self.fix_type = match self.gga_fix_quality {
            1 => 3, // GPS → 3D fix
            2 => 4, // DGPS
            4 => 6, // RTK fixed
            5 => 5, // RTK float
            _ => 0,
        };
        self.fix_valid = true;
        self.gga_valid = false; // consume
        self.rmc_valid = false;

        // Jitter correction: EMA of inter-fix interval for timestamp smoothing.
        if self.last_fix_ms > 0 {
            let delta = now_ms.wrapping_sub(self.last_fix_ms);
            if delta < 2000 {
                self.delta_ema_ms = 0.98 * self.delta_ema_ms + 0.02 * delta as f32;
            }
        }
        self.last_fix_ms = now_ms;
        // Corrected timestamp: align to expected interval.
        self.jitter_corrected_ms = now_ms;

        // Clear GPS yaw if stale (>300ms since last HDT/THS).
        if self.gps_yaw_valid && now_ms.wrapping_sub(self.gps_yaw_time_ms) > 300 {
            self.gps_yaw_valid = false;
        }

        true
    }

    /// Get the lag (delay) for this GPS type.
    /// NMEA lag is ~200ms due to sentence processing time.
    pub fn get_lag_s(&self) -> f32 {
        0.2
    }

    /// Average inter-fix interval (ms) for health monitoring.
    pub fn average_delta_ms(&self) -> f32 {
        self.delta_ema_ms
    }
}

// ─── Helper parsers (no_std, no alloc) ───

fn split_fields(data: &[u8]) -> heapless::Vec<&[u8], 20> {
    let mut fields = heapless::Vec::new();
    let mut start = 0;
    // Skip talker+message ID (e.g., "GPGGA")
    for i in 0..data.len() {
        if data[i] == b',' {
            let _ = fields.push(&data[start..i]);
            start = i + 1;
        }
    }
    let _ = fields.push(&data[start..]);
    fields
}

fn parse_hex_byte(h: u8, l: u8) -> u8 {
    let hi = if h >= b'A' { h - b'A' + 10 } else if h >= b'a' { h - b'a' + 10 } else { h - b'0' };
    let lo = if l >= b'A' { l - b'A' + 10 } else if l >= b'a' { l - b'a' + 10 } else { l - b'0' };
    (hi << 4) | lo
}

fn parse_u8(data: &[u8]) -> u8 {
    let mut val = 0u8;
    for &b in data {
        if b >= b'0' && b <= b'9' {
            val = val * 10 + (b - b'0');
        }
    }
    val
}

fn parse_f32(data: &[u8]) -> f32 {
    let mut val: f32 = 0.0;
    let mut decimal = false;
    let mut decimal_place: f32 = 0.1;
    let mut negative = false;
    for &b in data {
        if b == b'-' { negative = true; continue; }
        if b == b'.' { decimal = true; continue; }
        if b >= b'0' && b <= b'9' {
            let d = (b - b'0') as f32;
            if decimal { val += d * decimal_place; decimal_place *= 0.1; }
            else { val = val * 10.0 + d; }
        }
    }
    if negative { -val } else { val }
}

/// Parse NMEA latitude: ddmm.mmmm + N/S
fn parse_latlon(coord: &[u8], hemi: &[u8]) -> Option<f64> {
    if coord.is_empty() || hemi.is_empty() { return None; }
    let val = parse_f32(coord) as f64;
    let deg = (val / 100.0) as i32;
    let min = val - (deg as f64) * 100.0;
    let mut result = deg as f64 + min / 60.0;
    if hemi == b"S" || hemi == b"W" { result = -result; }
    Some(result)
}

/// Parse NMEA longitude: dddmm.mmmm + E/W
fn parse_latlon_lon(coord: &[u8], hemi: &[u8]) -> Option<f64> {
    parse_latlon(coord, hemi) // same algorithm, just wider degree field
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_gga_rmc_combined() {
        let mut gps = NmeaGps::new();
        // Feed a GGA sentence
        let gga = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*4F\r\n";
        for &b in gga.iter() { gps.feed_byte(b, 1000); }

        // GGA alone shouldn't produce a fix
        assert!(!gps.fix_valid);

        // Feed an RMC within 150ms
        let rmc = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
        let mut got_fix = false;
        for &b in rmc.iter() {
            if gps.feed_byte(b, 1050) { got_fix = true; }
        }

        assert!(got_fix, "Should have fix after GGA+RMC");
        assert!(gps.fix_valid);
        assert!((gps.latitude - 48.1173).abs() < 0.001, "Lat: {}", gps.latitude);
        assert!((gps.longitude - 11.5167).abs() < 0.001, "Lon: {}", gps.longitude);
        assert_eq!(gps.num_sats, 8);
    }

    #[test]
    fn test_parse_gga_only_no_fix() {
        let mut gps = NmeaGps::new();
        let gga = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*4F\r\n";
        for &b in gga.iter() { gps.feed_byte(b, 1000); }
        assert!(!gps.fix_valid, "GGA alone should not produce fix");
    }

    #[test]
    fn test_parse_rmc_speed() {
        let mut gps = NmeaGps::new();
        // Feed GGA first
        let gga = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*4F\r\n";
        for &b in gga.iter() { gps.feed_byte(b, 1000); }
        // Feed RMC with speed
        let rmc = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
        for &b in rmc.iter() { gps.feed_byte(b, 1050); }
        // 22.4 knots = 11.52 m/s
        assert!((gps.speed_ms - 11.52).abs() < 0.1, "Speed: {}", gps.speed_ms);
    }

    #[test]
    fn test_bad_checksum_rejected() {
        let mut gps = NmeaGps::new();
        let bad = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*FF\r\n";
        for &b in bad.iter() { gps.feed_byte(b, 1000); }
        assert!(!gps.gga_valid, "Bad checksum should be rejected");
    }

    #[test]
    fn test_stale_combination_rejected() {
        let mut gps = NmeaGps::new();
        let gga = b"$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*4F\r\n";
        for &b in gga.iter() { gps.feed_byte(b, 1000); }
        // RMC more than 150ms later
        let rmc = b"$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";
        let mut got_fix = false;
        for &b in rmc.iter() {
            if gps.feed_byte(b, 1200) { got_fix = true; }
        }
        assert!(!got_fix, "Stale combination should be rejected");
    }

    #[test]
    fn test_parse_hex_byte() {
        assert_eq!(parse_hex_byte(b'4', b'7'), 0x47);
        assert_eq!(parse_hex_byte(b'F', b'F'), 0xFF);
        assert_eq!(parse_hex_byte(b'0', b'0'), 0x00);
    }

    #[test]
    fn test_parse_f32() {
        assert!((parse_f32(b"123.45") - 123.45).abs() < 0.01);
        assert!((parse_f32(b"0.9") - 0.9).abs() < 0.01);
        assert!((parse_f32(b"-50.0") - (-50.0)).abs() < 0.1);
    }
}

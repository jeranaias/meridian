#![no_std]

//! ADS-B traffic awareness and threat assessment.
//!
//! Tracks up to 25 aircraft via ADS-B transponder messages.
//! Computes closest traffic and time-to-closest-approach (TTCA)
//! threat levels for collision avoidance advisories.

use heapless::Vec;

/// Maximum number of simultaneously tracked aircraft.
pub const MAX_AIRCRAFT: usize = 25;

/// Timeout for stale entries (milliseconds).
const STALE_TIMEOUT_MS: u32 = 5000;

/// Earth radius in meters (mean, for simplified 2D distance).
const EARTH_RADIUS_M: f64 = 6371000.0;

/// Threat level based on time-to-closest-approach.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThreatLevel {
    /// No threat — traffic is diverging or very far.
    None,
    /// Advisory — traffic within 5 NM, closing but not imminent.
    Advisory,
    /// Warning — traffic within 2 NM, closing, TTCA < 60s.
    Warning,
    /// Alert — traffic within 1 NM, TTCA < 30s, immediate action needed.
    Alert,
}

/// State of a single tracked aircraft from ADS-B messages.
#[derive(Debug, Clone)]
pub struct AircraftState {
    /// ICAO 24-bit transponder address.
    pub icao_address: u32,
    /// Latitude (degrees).
    pub lat: f64,
    /// Longitude (degrees).
    pub lon: f64,
    /// Altitude MSL (meters).
    pub altitude: f32,
    /// Heading (degrees, 0-359).
    pub heading: u16,
    /// Ground speed (m/s).
    pub speed: f32,
    /// Squawk code.
    pub squawk: u16,
    /// Callsign (up to 8 chars + null terminator).
    pub callsign: [u8; 9],
    /// Timestamp of last update (ms since boot).
    pub last_seen_ms: u32,
}

impl AircraftState {
    /// Create a new aircraft state with the given ICAO address.
    pub fn new(icao_address: u32) -> Self {
        Self {
            icao_address,
            lat: 0.0,
            lon: 0.0,
            altitude: 0.0,
            heading: 0,
            speed: 0.0,
            squawk: 0,
            callsign: [0u8; 9],
            last_seen_ms: 0,
        }
    }
}

/// Spatial pre-filter configuration.
#[derive(Debug, Clone, Copy)]
pub struct AdsbFilter {
    /// Maximum horizontal radius for tracking (meters). 0 = disabled.
    pub list_radius_m: f64,
    /// Maximum altitude difference for tracking (meters). 0 = disabled.
    pub list_alt_m: f32,
    /// Special ICAO address to bypass filters. 0 = disabled.
    pub icao_special: u32,
}

impl Default for AdsbFilter {
    fn default() -> Self {
        Self {
            list_radius_m: 0.0,
            list_alt_m: 0.0,
            icao_special: 0,
        }
    }
}

/// ADSB-Out (transponder) configuration stub.
#[derive(Debug, Clone)]
pub struct AdsbOutConfig {
    /// Own ICAO address.
    pub icao_address: u32,
    /// Callsign (up to 8 chars).
    pub callsign: [u8; 9],
    /// Emitter type (0=No info, 1=Light, 2=Small, etc.).
    pub emitter_type: u8,
    /// Squawk code.
    pub squawk: u16,
    /// Whether ADSB-Out is enabled.
    pub enabled: bool,
}

impl Default for AdsbOutConfig {
    fn default() -> Self {
        Self {
            icao_address: 0,
            callsign: *b"MERIDN\0\0\0",
            emitter_type: 0,
            squawk: 1200,
            enabled: false,
        }
    }
}

/// TRAFFIC_REPORT MAVLink emission request.
/// Caller reads this and serializes to MAVLink.
#[derive(Debug, Clone)]
pub struct TrafficReport {
    pub icao_address: u32,
    pub lat: f64,
    pub lon: f64,
    pub altitude: f32,
    pub heading: u16,
    pub speed: f32,
    pub callsign: [u8; 9],
    pub threat_level: ThreatLevel,
}

/// uAvionix MAVLink receiver backend stub.
pub struct UavionixReceiver {
    /// Whether this receiver is connected.
    pub connected: bool,
}

impl UavionixReceiver {
    pub fn new() -> Self {
        Self { connected: false }
    }

    /// Feed a received ADSB_VEHICLE MAVLink message.
    /// Returns parsed AircraftState if valid.
    pub fn parse_adsb_vehicle(&self, _payload: &[u8]) -> Option<AircraftState> {
        // Stub: real implementation would parse MAVLink msg #246
        None
    }
}

/// Sagetech receiver backend stub.
pub struct SagetechReceiver {
    pub connected: bool,
}

impl SagetechReceiver {
    pub fn new() -> Self {
        Self { connected: false }
    }

    /// Feed raw serial data from Sagetech receiver.
    pub fn parse_serial(&self, _data: &[u8]) -> Option<AircraftState> {
        // Stub: real implementation would parse Sagetech MXS protocol
        None
    }
}

/// Database of tracked ADS-B aircraft.
pub struct AircraftDatabase {
    aircraft: Vec<AircraftState, MAX_AIRCRAFT>,
    /// Spatial pre-filter configuration.
    pub filter: AdsbFilter,
    /// ADSB-Out transponder configuration.
    pub adsb_out: AdsbOutConfig,
}

impl AircraftDatabase {
    pub fn new() -> Self {
        Self {
            aircraft: Vec::new(),
            filter: AdsbFilter::default(),
            adsb_out: AdsbOutConfig::default(),
        }
    }

    /// Number of currently tracked aircraft.
    pub fn count(&self) -> usize {
        self.aircraft.len()
    }

    /// Add or update a tracked aircraft with spatial pre-filtering.
    /// `my_lat`, `my_lon`, `my_alt`: own aircraft position for filter evaluation.
    /// Pass (0,0,0) to skip filtering.
    pub fn update_aircraft_filtered(&mut self, state: AircraftState, my_lat: f64, my_lon: f64, my_alt: f32) {
        // Bypass filter for special ICAO
        if self.filter.icao_special != 0 && state.icao_address == self.filter.icao_special {
            self.update_aircraft(state);
            return;
        }

        // Spatial radius filter
        if self.filter.list_radius_m > 0.0 && (my_lat != 0.0 || my_lon != 0.0) {
            let dist = haversine_distance_m(my_lat, my_lon, state.lat, state.lon);
            if dist > self.filter.list_radius_m {
                return;
            }
        }

        // Altitude filter
        if self.filter.list_alt_m > 0.0 && my_alt != 0.0 {
            let alt_diff = (state.altitude - my_alt).abs();
            if alt_diff > self.filter.list_alt_m {
                return;
            }
        }

        self.update_aircraft(state);
    }

    /// Add or update a tracked aircraft (no pre-filtering).
    /// If the ICAO address is already tracked, updates in place.
    /// If new and the database is full, replaces the oldest entry.
    pub fn update_aircraft(&mut self, state: AircraftState) {
        // Look for existing entry by ICAO address
        for existing in self.aircraft.iter_mut() {
            if existing.icao_address == state.icao_address {
                *existing = state;
                return;
            }
        }

        // Try to add new entry
        if self.aircraft.push(state.clone()).is_err() {
            // Database full — replace oldest entry
            let mut oldest_idx = 0;
            let mut oldest_time = u32::MAX;
            for (i, ac) in self.aircraft.iter().enumerate() {
                if ac.last_seen_ms < oldest_time {
                    oldest_time = ac.last_seen_ms;
                    oldest_idx = i;
                }
            }
            self.aircraft[oldest_idx] = state;
        }
    }

    /// Get the closest aircraft to the given position by 2D great-circle distance.
    pub fn get_closest(&self, my_lat: f64, my_lon: f64) -> Option<&AircraftState> {
        let mut closest: Option<(usize, f64)> = None;

        for (i, ac) in self.aircraft.iter().enumerate() {
            let dist = haversine_distance_m(my_lat, my_lon, ac.lat, ac.lon);
            if closest.is_none() || dist < closest.unwrap().1 {
                closest = Some((i, dist));
            }
        }

        closest.map(|(i, _)| &self.aircraft[i])
    }

    /// Assess threat level of an aircraft relative to own position and velocity.
    ///
    /// Uses 2D distance and time-to-closest-approach (TTCA) estimation.
    /// `my_lat`, `my_lon`: own position (degrees).
    /// `my_heading`: own heading (degrees).
    /// `my_speed`: own ground speed (m/s).
    pub fn threat_level(
        &self,
        aircraft: &AircraftState,
        my_lat: f64,
        my_lon: f64,
        my_heading: f32,
        my_speed: f32,
    ) -> ThreatLevel {
        let distance_m = haversine_distance_m(my_lat, my_lon, aircraft.lat, aircraft.lon);
        let distance_nm = distance_m / 1852.0;

        // If beyond 5 NM, no threat
        if distance_nm > 5.0 {
            return ThreatLevel::None;
        }

        // Compute relative velocity components to estimate TTCA
        let my_hdg_rad = (my_heading as f64) * core::f64::consts::PI / 180.0;
        let ac_hdg_rad = (aircraft.heading as f64) * core::f64::consts::PI / 180.0;

        let my_vn = (my_speed as f64) * libm::cos(my_hdg_rad);
        let my_ve = (my_speed as f64) * libm::sin(my_hdg_rad);
        let ac_vn = (aircraft.speed as f64) * libm::cos(ac_hdg_rad);
        let ac_ve = (aircraft.speed as f64) * libm::sin(ac_hdg_rad);

        // Relative position (aircraft relative to self) in local NE frame
        let dlat_rad = (aircraft.lat - my_lat) * core::f64::consts::PI / 180.0;
        let dlon_rad = (aircraft.lon - my_lon) * core::f64::consts::PI / 180.0;
        let mean_lat_rad = ((my_lat + aircraft.lat) / 2.0) * core::f64::consts::PI / 180.0;

        let rel_n = dlat_rad * EARTH_RADIUS_M;
        let rel_e = dlon_rad * EARTH_RADIUS_M * libm::cos(mean_lat_rad);

        // Relative velocity
        let rel_vn = ac_vn - my_vn;
        let rel_ve = ac_ve - my_ve;

        let rel_v_sq = rel_vn * rel_vn + rel_ve * rel_ve;

        // Time to closest approach: t = -(r . v) / |v|^2
        let ttca = if rel_v_sq > 0.01 {
            let dot = rel_n * rel_vn + rel_e * rel_ve;
            let t = -dot / rel_v_sq;
            if t < 0.0 { f64::MAX } else { t } // negative = diverging
        } else {
            f64::MAX // no relative motion
        };

        // Classify threat
        if distance_nm <= 1.0 && ttca < 30.0 {
            ThreatLevel::Alert
        } else if distance_nm <= 2.0 && ttca < 60.0 {
            ThreatLevel::Warning
        } else if distance_nm <= 5.0 && ttca < 120.0 {
            ThreatLevel::Advisory
        } else {
            ThreatLevel::None
        }
    }

    /// Remove aircraft not updated within the stale timeout.
    pub fn prune(&mut self, now_ms: u32) {
        self.aircraft.retain(|ac| {
            now_ms.wrapping_sub(ac.last_seen_ms) < STALE_TIMEOUT_MS
        });
    }

    /// Get a reference to all tracked aircraft.
    pub fn aircraft(&self) -> &[AircraftState] {
        &self.aircraft
    }

    /// Generate TRAFFIC_REPORT MAVLink messages for all tracked aircraft.
    /// `my_lat`, `my_lon`: own position for threat assessment.
    /// `my_heading`, `my_speed`: own velocity for TTCA computation.
    pub fn generate_traffic_reports(
        &self,
        my_lat: f64,
        my_lon: f64,
        my_heading: f32,
        my_speed: f32,
    ) -> Vec<TrafficReport, MAX_AIRCRAFT> {
        let mut reports = Vec::new();
        for ac in self.aircraft.iter() {
            let threat = self.threat_level(ac, my_lat, my_lon, my_heading, my_speed);
            let _ = reports.push(TrafficReport {
                icao_address: ac.icao_address,
                lat: ac.lat,
                lon: ac.lon,
                altitude: ac.altitude,
                heading: ac.heading,
                speed: ac.speed,
                callsign: ac.callsign,
                threat_level: threat,
            });
        }
        reports
    }
}

/// Haversine distance between two points in meters.
/// Inputs in degrees.
fn haversine_distance_m(lat1_deg: f64, lon1_deg: f64, lat2_deg: f64, lon2_deg: f64) -> f64 {
    let to_rad = core::f64::consts::PI / 180.0;
    let lat1 = lat1_deg * to_rad;
    let lat2 = lat2_deg * to_rad;
    let dlat = (lat2_deg - lat1_deg) * to_rad;
    let dlon = (lon2_deg - lon1_deg) * to_rad;

    let a = libm::sin(dlat / 2.0) * libm::sin(dlat / 2.0)
        + libm::cos(lat1) * libm::cos(lat2)
            * libm::sin(dlon / 2.0) * libm::sin(dlon / 2.0);
    let c = 2.0 * libm::atan2(libm::sqrt(a), libm::sqrt(1.0 - a));

    EARTH_RADIUS_M * c
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_aircraft(icao: u32, lat: f64, lon: f64, alt: f32, heading: u16, speed: f32, now_ms: u32) -> AircraftState {
        AircraftState {
            icao_address: icao,
            lat,
            lon,
            altitude: alt,
            heading,
            speed,
            squawk: 1200,
            callsign: *b"TEST\0\0\0\0\0",
            last_seen_ms: now_ms,
        }
    }

    #[test]
    fn test_add_and_update_aircraft() {
        let mut db = AircraftDatabase::new();
        let ac = make_aircraft(0xABCDEF, 35.0, -120.0, 3000.0, 90, 100.0, 1000);
        db.update_aircraft(ac);
        assert_eq!(db.count(), 1);

        // Update same ICAO — should not increase count
        let ac2 = make_aircraft(0xABCDEF, 35.001, -120.0, 3050.0, 91, 101.0, 2000);
        db.update_aircraft(ac2);
        assert_eq!(db.count(), 1);
        assert!((db.aircraft()[0].lat - 35.001).abs() < 1e-6);

        // Add different ICAO
        let ac3 = make_aircraft(0x123456, 36.0, -121.0, 5000.0, 180, 200.0, 2000);
        db.update_aircraft(ac3);
        assert_eq!(db.count(), 2);
    }

    #[test]
    fn test_timeout_pruning() {
        let mut db = AircraftDatabase::new();
        db.update_aircraft(make_aircraft(0x01, 35.0, -120.0, 3000.0, 0, 100.0, 1000));
        db.update_aircraft(make_aircraft(0x02, 36.0, -121.0, 5000.0, 90, 200.0, 3000));

        // At t=4000, both should still be alive (0x01 seen 3s ago, 0x02 seen 1s ago)
        db.prune(4000);
        assert_eq!(db.count(), 2);

        // At t=7000, 0x01 should be pruned (6s stale), 0x02 still alive (4s)
        db.prune(7000);
        assert_eq!(db.count(), 1);
        assert_eq!(db.aircraft()[0].icao_address, 0x02);

        // At t=9000, 0x02 is now 6s stale → pruned
        db.prune(9000);
        assert_eq!(db.count(), 0);
    }

    #[test]
    fn test_closest_aircraft() {
        let mut db = AircraftDatabase::new();
        // Aircraft A: ~111 km north
        db.update_aircraft(make_aircraft(0x01, 36.0, -120.0, 3000.0, 0, 100.0, 1000));
        // Aircraft B: ~11 km north (closest)
        db.update_aircraft(make_aircraft(0x02, 35.1, -120.0, 5000.0, 90, 200.0, 1000));
        // Aircraft C: ~55 km north
        db.update_aircraft(make_aircraft(0x03, 35.5, -120.0, 4000.0, 180, 150.0, 1000));

        let closest = db.get_closest(35.0, -120.0).unwrap();
        assert_eq!(closest.icao_address, 0x02);
    }

    #[test]
    fn test_closest_empty_db() {
        let db = AircraftDatabase::new();
        assert!(db.get_closest(35.0, -120.0).is_none());
    }

    #[test]
    fn test_threat_level_none_far_away() {
        let db = AircraftDatabase::new();
        // Aircraft 100 NM away — no threat
        let ac = make_aircraft(0x01, 36.5, -120.0, 3000.0, 180, 100.0, 1000);
        let level = db.threat_level(&ac, 35.0, -120.0, 0.0, 50.0);
        assert_eq!(level, ThreatLevel::None);
    }

    #[test]
    fn test_threat_level_diverging() {
        let db = AircraftDatabase::new();
        // Aircraft 3 NM north, heading north (away) at 100 m/s
        let nm3_deg = 3.0 * 1852.0 / 111_320.0; // ~3 NM in degrees lat
        let ac = make_aircraft(0x01, 35.0 + nm3_deg, -120.0, 3000.0, 0, 100.0, 1000);
        // We are heading south
        let level = db.threat_level(&ac, 35.0, -120.0, 180.0, 50.0);
        // Both diverging — should be None despite distance < 5 NM
        assert_eq!(level, ThreatLevel::None);
    }

    #[test]
    fn test_threat_level_head_on_close() {
        let db = AircraftDatabase::new();
        // Aircraft 0.5 NM north, heading south (toward us) at 100 m/s
        let nm_half_deg = 0.5 * 1852.0 / 111_320.0;
        let ac = make_aircraft(0x01, 35.0 + nm_half_deg, -120.0, 3000.0, 180, 100.0, 1000);
        // We are heading north at 100 m/s
        let level = db.threat_level(&ac, 35.0, -120.0, 0.0, 100.0);
        // < 1 NM, head-on, closing fast → Alert
        assert_eq!(level, ThreatLevel::Alert);
    }

    #[test]
    fn test_threat_level_advisory() {
        let db = AircraftDatabase::new();
        // Aircraft 4 NM north, heading south at 150 m/s (~300 kts)
        let nm4_deg = 4.0 * 1852.0 / 111_320.0;
        let ac = make_aircraft(0x01, 35.0 + nm4_deg, -120.0, 3000.0, 180, 150.0, 1000);
        // We are heading north at 50 m/s — closing at ~200 m/s, TTCA ~37s
        let level = db.threat_level(&ac, 35.0, -120.0, 0.0, 50.0);
        // Within 5 NM, closing, TTCA < 120s → Advisory
        assert_eq!(level, ThreatLevel::Advisory);
    }

    #[test]
    fn test_full_database_replaces_oldest() {
        let mut db = AircraftDatabase::new();
        // Fill database
        for i in 0..MAX_AIRCRAFT {
            db.update_aircraft(make_aircraft(
                i as u32,
                35.0 + (i as f64) * 0.01,
                -120.0,
                3000.0,
                0,
                100.0,
                1000 + (i as u32) * 100,
            ));
        }
        assert_eq!(db.count(), MAX_AIRCRAFT);

        // Add one more — should replace the oldest (icao=0, last_seen=1000)
        db.update_aircraft(make_aircraft(0xFF, 40.0, -119.0, 6000.0, 270, 200.0, 5000));
        assert_eq!(db.count(), MAX_AIRCRAFT);

        // Verify icao=0 is gone and 0xFF is present
        assert!(db.aircraft().iter().all(|ac| ac.icao_address != 0));
        assert!(db.aircraft().iter().any(|ac| ac.icao_address == 0xFF));
    }

    #[test]
    fn test_haversine_known_distance() {
        // New York to Los Angeles: ~3944 km
        let dist = haversine_distance_m(40.7128, -74.0060, 34.0522, -118.2437);
        let dist_km = dist / 1000.0;
        assert!(dist_km > 3900.0 && dist_km < 4000.0,
            "NYC to LA should be ~3944 km, got {:.0} km", dist_km);
    }
}

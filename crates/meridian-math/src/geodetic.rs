use crate::vec3::Vec3;
use crate::frames::NED;

/// WGS84 ellipsoid constants
pub const WGS84_A: f64 = 6378137.0;              // Semi-major axis (m)
pub const WGS84_F: f64 = 1.0 / 298.257223563;    // Flattening
pub const WGS84_B: f64 = WGS84_A * (1.0 - WGS84_F); // Semi-minor axis
pub const WGS84_E2: f64 = 2.0 * WGS84_F - WGS84_F * WGS84_F; // First eccentricity squared

/// Geodetic position (latitude, longitude, altitude)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LatLonAlt {
    /// Latitude in radians
    pub lat: f64,
    /// Longitude in radians
    pub lon: f64,
    /// Altitude above WGS84 ellipsoid in meters
    pub alt: f64,
}

impl LatLonAlt {
    pub fn new(lat_rad: f64, lon_rad: f64, alt_m: f64) -> Self {
        Self { lat: lat_rad, lon: lon_rad, alt: alt_m }
    }

    /// Create from degrees
    pub fn from_degrees(lat_deg: f64, lon_deg: f64, alt_m: f64) -> Self {
        Self::new(lat_deg.to_radians(), lon_deg.to_radians(), alt_m)
    }

    /// Convert to ECEF coordinates (double precision for accuracy)
    pub fn to_ecef(&self) -> [f64; 3] {
        let sin_lat = libm::sin(self.lat);
        let cos_lat = libm::cos(self.lat);
        let sin_lon = libm::sin(self.lon);
        let cos_lon = libm::cos(self.lon);

        let n = WGS84_A / libm::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);

        [
            (n + self.alt) * cos_lat * cos_lon,
            (n + self.alt) * cos_lat * sin_lon,
            (n * (1.0 - WGS84_E2) + self.alt) * sin_lat,
        ]
    }

    /// Create from ECEF coordinates using iterative method
    pub fn from_ecef(ecef: &[f64; 3]) -> Self {
        let x = ecef[0];
        let y = ecef[1];
        let z = ecef[2];

        let lon = libm::atan2(y, x);
        let p = libm::sqrt(x * x + y * y);

        // Iterative latitude/altitude (Bowring's method, 2-3 iterations converges)
        let mut lat = libm::atan2(z, p * (1.0 - WGS84_E2));
        let mut alt = 0.0;

        for _ in 0..5 {
            let sin_lat = libm::sin(lat);
            let cos_lat = libm::cos(lat);
            let n = WGS84_A / libm::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
            // Guard against division by zero at poles (cos(lat) ≈ 0)
            if cos_lat.abs() > 1e-10 {
                alt = p / cos_lat - n;
            } else {
                // At poles: altitude from Z component directly
                alt = z.abs() / sin_lat.abs() - n * (1.0 - WGS84_E2);
            }
            lat = libm::atan2(z, p * (1.0 - WGS84_E2 * n / (n + alt)));
        }

        Self { lat, lon, alt }
    }

    /// Convert a point to NED coordinates relative to this position as origin.
    /// Returns f32 Vec3<NED> for the control system (sufficient precision within ~50km).
    pub fn to_ned_from(&self, point: &LatLonAlt) -> Vec3<NED> {
        let origin_ecef = self.to_ecef();
        let point_ecef = point.to_ecef();

        // Difference in ECEF
        let dx = point_ecef[0] - origin_ecef[0];
        let dy = point_ecef[1] - origin_ecef[1];
        let dz = point_ecef[2] - origin_ecef[2];

        // Rotation matrix from ECEF to NED at origin
        let sin_lat = libm::sin(self.lat);
        let cos_lat = libm::cos(self.lat);
        let sin_lon = libm::sin(self.lon);
        let cos_lon = libm::cos(self.lon);

        let north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
        let east = -sin_lon * dx + cos_lon * dy;
        let down = -cos_lat * cos_lon * dx - cos_lat * sin_lon * dy - sin_lat * dz;

        Vec3::new(north as f32, east as f32, down as f32)
    }

    /// Convert NED offset back to geodetic position relative to this origin.
    pub fn ned_to_lla(&self, ned: &Vec3<NED>) -> LatLonAlt {
        let origin_ecef = self.to_ecef();

        let sin_lat = libm::sin(self.lat);
        let cos_lat = libm::cos(self.lat);
        let sin_lon = libm::sin(self.lon);
        let cos_lon = libm::cos(self.lon);

        let n = ned.x as f64;
        let e = ned.y as f64;
        let d = ned.z as f64;

        // NED to ECEF rotation (transpose of ECEF-to-NED)
        let dx = -sin_lat * cos_lon * n - sin_lon * e - cos_lat * cos_lon * d;
        let dy = -sin_lat * sin_lon * n + cos_lon * e - cos_lat * sin_lon * d;
        let dz = cos_lat * n - sin_lat * d;

        let ecef = [
            origin_ecef[0] + dx,
            origin_ecef[1] + dy,
            origin_ecef[2] + dz,
        ];

        LatLonAlt::from_ecef(&ecef)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ecef_roundtrip() {
        let lla = LatLonAlt::from_degrees(35.0, -120.0, 100.0);
        let ecef = lla.to_ecef();
        let lla2 = LatLonAlt::from_ecef(&ecef);
        assert!((lla.lat - lla2.lat).abs() < 1e-10);
        assert!((lla.lon - lla2.lon).abs() < 1e-10);
        assert!((lla.alt - lla2.alt).abs() < 1e-3);
    }

    #[test]
    fn test_ned_conversion() {
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
        // Point ~111m north (1/1000 of a degree latitude)
        let point = LatLonAlt::from_degrees(35.001, -120.0, 0.0);
        let ned = origin.to_ned_from(&point);
        // Should be approximately 111m north, 0 east, 0 down
        assert!(ned.x > 100.0 && ned.x < 120.0); // ~111m north
        assert!(ned.y.abs() < 1.0);               // ~0 east
        assert!(ned.z.abs() < 1.0);               // ~0 down
    }

    #[test]
    fn test_ned_roundtrip() {
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 100.0);
        let point = LatLonAlt::from_degrees(35.01, -119.99, 150.0);
        let ned = origin.to_ned_from(&point);
        let point2 = origin.ned_to_lla(&ned);
        assert!((point.lat - point2.lat).abs() < 1e-8);
        assert!((point.lon - point2.lon).abs() < 1e-8);
        assert!((point.alt - point2.alt).abs() < 0.1);
    }
}

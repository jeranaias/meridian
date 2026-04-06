//! GPS blending — weighted merge of multiple GPS receivers.
//!
//! ArduPilot reference: `AP_GPS_Blended.cpp`
//!
//! Weights each GPS by the inverse square of its horizontal accuracy
//! (hacc), producing a blended position/velocity that is more accurate
//! than either receiver alone. Also handles velocity divergence detection.

use meridian_math::geodetic::LatLonAlt;
use meridian_math::{Vec3, NED};
use meridian_types::messages::{GnssFixType, GnssPosition};
use meridian_types::time::Instant;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Maximum number of GPS receivers to blend.
pub const MAX_GPS_RECEIVERS: usize = 2;

/// Minimum hacc (meters) to avoid divide-by-zero.
const MIN_HACC: f32 = 0.01;

/// Maximum velocity divergence before we stop blending (m/s).
const MAX_VEL_DIVERGENCE: f32 = 1.0;

// ---------------------------------------------------------------------------
// GPS blending
// ---------------------------------------------------------------------------

/// Blended GPS output.
pub struct GpsBlender {
    /// Latest fix from each receiver.
    receivers: [Option<GnssPosition>; MAX_GPS_RECEIVERS],
    /// Blended output.
    blended: Option<GnssPosition>,
    /// Whether blending is active (vs using primary only).
    blending_active: bool,
}

impl GpsBlender {
    pub fn new() -> Self {
        Self {
            receivers: [None; MAX_GPS_RECEIVERS],
            blended: None,
            blending_active: false,
        }
    }

    /// Update receiver data. `index` = 0 or 1.
    pub fn update_receiver(&mut self, index: usize, fix: GnssPosition) {
        if index < MAX_GPS_RECEIVERS {
            self.receivers[index] = Some(fix);
        }
    }

    /// Compute the blended GPS solution.
    ///
    /// Returns the blended position, or the best single receiver if only
    /// one is available or if velocity divergence is too high.
    pub fn blend(&mut self, now: Instant) -> Option<&GnssPosition> {
        let (has_0, has_1) = (self.receivers[0].is_some(), self.receivers[1].is_some());

        if has_0 && has_1 {
            let r0 = self.receivers[0].as_ref().unwrap();
            let r1 = self.receivers[1].as_ref().unwrap();

            // Check fix quality
            if !Self::has_3d_fix(r0) && !Self::has_3d_fix(r1) {
                self.blending_active = false;
                return None;
            }
            if !Self::has_3d_fix(r0) {
                self.blending_active = false;
                self.blended = Some(*r1);
                return self.blended.as_ref();
            }
            if !Self::has_3d_fix(r1) {
                self.blending_active = false;
                self.blended = Some(*r0);
                return self.blended.as_ref();
            }

            // Check velocity divergence
            let dx = r0.velocity_ned.x - r1.velocity_ned.x;
            let dy = r0.velocity_ned.y - r1.velocity_ned.y;
            let dz = r0.velocity_ned.z - r1.velocity_ned.z;
            let vel_diff = libm::sqrtf(dx * dx + dy * dy + dz * dz);
            if vel_diff > MAX_VEL_DIVERGENCE {
                // Use the receiver with better hacc
                self.blending_active = false;
                if r0.horizontal_accuracy <= r1.horizontal_accuracy {
                    self.blended = Some(*r0);
                } else {
                    self.blended = Some(*r1);
                }
                return self.blended.as_ref();
            }

            // Compute weights: w_i = 1 / hacc_i^2
            let hacc0 = r0.horizontal_accuracy.max(MIN_HACC);
            let hacc1 = r1.horizontal_accuracy.max(MIN_HACC);
            let w0 = 1.0 / (hacc0 * hacc0);
            let w1 = 1.0 / (hacc1 * hacc1);
            let wsum = w0 + w1;
            let n0 = w0 / wsum;
            let n1 = w1 / wsum;

            // Blend position (linear interp in lat/lon/alt)
            let blended_lat = r0.position.lat * n0 as f64 + r1.position.lat * n1 as f64;
            let blended_lon = r0.position.lon * n0 as f64 + r1.position.lon * n1 as f64;
            let blended_alt = r0.position.alt * n0 as f64 + r1.position.alt * n1 as f64;

            // Blend velocity
            let blended_vel = Vec3::<NED>::new(
                r0.velocity_ned.x * n0 + r1.velocity_ned.x * n1,
                r0.velocity_ned.y * n0 + r1.velocity_ned.y * n1,
                r0.velocity_ned.z * n0 + r1.velocity_ned.z * n1,
            );

            // Blend accuracies (weighted)
            let blended_hacc = r0.horizontal_accuracy * n0 + r1.horizontal_accuracy * n1;
            let blended_vacc = r0.vertical_accuracy * n0 + r1.vertical_accuracy * n1;
            let blended_sacc = r0.speed_accuracy * n0 + r1.speed_accuracy * n1;

            self.blending_active = true;
            self.blended = Some(GnssPosition {
                timestamp: now,
                fix_type: GnssFixType::Fix3D,
                position: LatLonAlt { lat: blended_lat, lon: blended_lon, alt: blended_alt },
                velocity_ned: blended_vel,
                horizontal_accuracy: blended_hacc,
                vertical_accuracy: blended_vacc,
                speed_accuracy: blended_sacc,
                num_sats: r0.num_sats.max(r1.num_sats),
            });
            self.blended.as_ref()
        } else if has_0 {
            self.blending_active = false;
            self.blended = self.receivers[0];
            self.blended.as_ref()
        } else if has_1 {
            self.blending_active = false;
            self.blended = self.receivers[1];
            self.blended.as_ref()
        } else {
            None
        }
    }

    fn has_3d_fix(fix: &GnssPosition) -> bool {
        matches!(fix.fix_type, GnssFixType::Fix3D | GnssFixType::DGps | GnssFixType::RtkFloat | GnssFixType::RtkFixed)
    }

    pub fn is_blending(&self) -> bool {
        self.blending_active
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_fix(hacc: f32, lat: f64) -> GnssPosition {
        GnssPosition {
            timestamp: Instant::from_micros(0),
            fix_type: GnssFixType::Fix3D,
            position: LatLonAlt { lat, lon: 0.0, alt: 100.0 },
            velocity_ned: Vec3::<NED>::new(1.0, 0.0, 0.0),
            horizontal_accuracy: hacc,
            vertical_accuracy: 2.0,
            speed_accuracy: 0.5,
            num_sats: 10,
        }
    }

    #[test]
    fn test_single_receiver() {
        let mut blender = GpsBlender::new();
        blender.update_receiver(0, make_fix(1.0, 0.5));
        let result = blender.blend(Instant::from_micros(0));
        assert!(result.is_some());
        assert!(!blender.is_blending());
    }

    #[test]
    fn test_dual_receiver_blending() {
        let mut blender = GpsBlender::new();
        blender.update_receiver(0, make_fix(1.0, 0.5));
        blender.update_receiver(1, make_fix(2.0, 0.6));
        blender.blend(Instant::from_micros(0));
        assert!(blender.is_blending());
        // Better hacc (1.0) should have higher weight → blended lat closer to 0.5
        let blended = blender.blended.as_ref().unwrap();
        assert!(blended.position.lat > 0.5 && blended.position.lat < 0.6);
        // Weight ratio: w0 = 1/1 = 1, w1 = 1/4 = 0.25, n0 = 0.8, n1 = 0.2
        // expected lat ≈ 0.5*0.8 + 0.6*0.2 = 0.52
        assert!((blended.position.lat - 0.52).abs() < 0.01);
    }
}

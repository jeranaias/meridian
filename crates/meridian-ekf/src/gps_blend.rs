//! GPS blending for dual-GPS configurations.
//!
//! Blends position and velocity from two GPS receivers, weighting
//! by horizontal accuracy (hacc). Falls back to single GPS when
//! one receiver is unhealthy.
//!
//! Source: ArduPilot AP_GPS_Blended

use meridian_math::Vec3;
use meridian_math::frames::NED;

/// A GPS measurement for blending.
#[derive(Debug, Clone, Copy)]
pub struct GpsMeasurement {
    /// Position in NED frame (m).
    pub position: Vec3<NED>,
    /// Velocity in NED frame (m/s).
    pub velocity: Vec3<NED>,
    /// Horizontal accuracy (m). Lower = better.
    pub hacc: f32,
    /// Whether this receiver is healthy (has fix, recent data).
    pub healthy: bool,
}

impl Default for GpsMeasurement {
    fn default() -> Self {
        Self {
            position: Vec3::zero(),
            velocity: Vec3::zero(),
            hacc: 100.0,
            healthy: false,
        }
    }
}

/// Blended GPS output.
#[derive(Debug, Clone, Copy)]
pub struct BlendedGps {
    /// Blended position in NED (m).
    pub position: Vec3<NED>,
    /// Blended velocity in NED (m/s).
    pub velocity: Vec3<NED>,
    /// Effective horizontal accuracy (m).
    pub hacc: f32,
    /// Number of GPS receivers used in the blend.
    pub sources: u8,
}

/// Dual-GPS blender.
pub struct GpsBlender {
    /// Minimum hacc to consider a GPS valid for blending (m).
    pub max_hacc: f32,
}

impl GpsBlender {
    pub fn new() -> Self {
        Self {
            max_hacc: 50.0,
        }
    }

    /// Blend two GPS measurements. Returns None if both are unhealthy.
    pub fn blend(&self, gps1: &GpsMeasurement, gps2: &GpsMeasurement) -> Option<BlendedGps> {
        let g1_valid = gps1.healthy && gps1.hacc < self.max_hacc && gps1.hacc > 0.0;
        let g2_valid = gps2.healthy && gps2.hacc < self.max_hacc && gps2.hacc > 0.0;

        match (g1_valid, g2_valid) {
            (false, false) => None,
            (true, false) => Some(BlendedGps {
                position: gps1.position,
                velocity: gps1.velocity,
                hacc: gps1.hacc,
                sources: 1,
            }),
            (false, true) => Some(BlendedGps {
                position: gps2.position,
                velocity: gps2.velocity,
                hacc: gps2.hacc,
                sources: 1,
            }),
            (true, true) => {
                // Weight inversely proportional to hacc squared
                // w1 = 1/hacc1^2, w2 = 1/hacc2^2, then normalize
                let inv_h1_sq = 1.0 / (gps1.hacc * gps1.hacc);
                let inv_h2_sq = 1.0 / (gps2.hacc * gps2.hacc);
                let total = inv_h1_sq + inv_h2_sq;
                let w1 = inv_h1_sq / total;
                let w2 = inv_h2_sq / total;

                let position = gps1.position.scale(w1) + gps2.position.scale(w2);
                let velocity = gps1.velocity.scale(w1) + gps2.velocity.scale(w2);

                // Blended hacc: weighted harmonic-ish mean
                let hacc = 1.0 / libm::sqrtf(total);

                Some(BlendedGps {
                    position,
                    velocity,
                    hacc,
                    sources: 2,
                })
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f32 = 1e-4;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < TOL
    }

    #[test]
    fn test_both_unhealthy_returns_none() {
        let blender = GpsBlender::new();
        let g1 = GpsMeasurement::default(); // healthy = false
        let g2 = GpsMeasurement::default();
        assert!(blender.blend(&g1, &g2).is_none());
    }

    #[test]
    fn test_single_gps_fallback_gps1() {
        let blender = GpsBlender::new();
        let g1 = GpsMeasurement {
            position: Vec3::new(10.0, 20.0, -5.0),
            velocity: Vec3::new(1.0, 0.0, 0.0),
            hacc: 1.5,
            healthy: true,
        };
        let g2 = GpsMeasurement::default(); // unhealthy

        let result = blender.blend(&g1, &g2).unwrap();
        assert_eq!(result.sources, 1);
        assert!(approx_eq(result.position.x, 10.0));
        assert!(approx_eq(result.position.y, 20.0));
        assert!(approx_eq(result.hacc, 1.5));
    }

    #[test]
    fn test_single_gps_fallback_gps2() {
        let blender = GpsBlender::new();
        let g1 = GpsMeasurement::default(); // unhealthy
        let g2 = GpsMeasurement {
            position: Vec3::new(30.0, 40.0, -10.0),
            velocity: Vec3::new(0.0, 2.0, 0.0),
            hacc: 2.0,
            healthy: true,
        };

        let result = blender.blend(&g1, &g2).unwrap();
        assert_eq!(result.sources, 1);
        assert!(approx_eq(result.position.x, 30.0));
        assert!(approx_eq(result.velocity.y, 2.0));
    }

    #[test]
    fn test_equal_accuracy_blends_50_50() {
        let blender = GpsBlender::new();
        let g1 = GpsMeasurement {
            position: Vec3::new(10.0, 0.0, 0.0),
            velocity: Vec3::new(2.0, 0.0, 0.0),
            hacc: 2.0,
            healthy: true,
        };
        let g2 = GpsMeasurement {
            position: Vec3::new(20.0, 0.0, 0.0),
            velocity: Vec3::new(4.0, 0.0, 0.0),
            hacc: 2.0,
            healthy: true,
        };

        let result = blender.blend(&g1, &g2).unwrap();
        assert_eq!(result.sources, 2);
        // Equal weights → average
        assert!(approx_eq(result.position.x, 15.0));
        assert!(approx_eq(result.velocity.x, 3.0));
    }

    #[test]
    fn test_unequal_accuracy_favors_better() {
        let blender = GpsBlender::new();
        let g1 = GpsMeasurement {
            position: Vec3::new(10.0, 0.0, 0.0),
            velocity: Vec3::zero(),
            hacc: 1.0, // much better
            healthy: true,
        };
        let g2 = GpsMeasurement {
            position: Vec3::new(20.0, 0.0, 0.0),
            velocity: Vec3::zero(),
            hacc: 10.0, // much worse
            healthy: true,
        };

        let result = blender.blend(&g1, &g2).unwrap();
        assert_eq!(result.sources, 2);
        // GPS1 should dominate (weight ~0.99 vs ~0.01)
        assert!(result.position.x > 9.5 && result.position.x < 11.0,
            "Expected position near 10.0, got {}", result.position.x);
    }

    #[test]
    fn test_hacc_too_high_treated_as_unhealthy() {
        let blender = GpsBlender::new(); // max_hacc = 50.0
        let g1 = GpsMeasurement {
            position: Vec3::new(10.0, 0.0, 0.0),
            velocity: Vec3::zero(),
            hacc: 100.0, // above max
            healthy: true,
        };
        let g2 = GpsMeasurement {
            position: Vec3::new(20.0, 0.0, 0.0),
            velocity: Vec3::zero(),
            hacc: 2.0,
            healthy: true,
        };

        let result = blender.blend(&g1, &g2).unwrap();
        assert_eq!(result.sources, 1);
        assert!(approx_eq(result.position.x, 20.0));
    }

    #[test]
    fn test_blended_hacc_improves_with_two_sources() {
        let blender = GpsBlender::new();
        let g1 = GpsMeasurement {
            position: Vec3::zero(),
            velocity: Vec3::zero(),
            hacc: 2.0,
            healthy: true,
        };
        let g2 = GpsMeasurement {
            position: Vec3::zero(),
            velocity: Vec3::zero(),
            hacc: 2.0,
            healthy: true,
        };

        let result = blender.blend(&g1, &g2).unwrap();
        // Blended hacc should be better than either individual
        assert!(result.hacc < 2.0,
            "Blended hacc {} should be better than individual 2.0", result.hacc);
    }

    #[test]
    fn test_zero_hacc_treated_as_unhealthy() {
        let blender = GpsBlender::new();
        let g1 = GpsMeasurement {
            position: Vec3::new(10.0, 0.0, 0.0),
            velocity: Vec3::zero(),
            hacc: 0.0, // invalid
            healthy: true,
        };
        let g2 = GpsMeasurement {
            position: Vec3::new(20.0, 0.0, 0.0),
            velocity: Vec3::zero(),
            hacc: 2.0,
            healthy: true,
        };

        let result = blender.blend(&g1, &g2).unwrap();
        assert_eq!(result.sources, 1);
        assert!(approx_eq(result.position.x, 20.0));
    }
}

//! Air density compensation for altitude effects on motor thrust.
//!
//! Uses the ISA (International Standard Atmosphere) troposphere model
//! to compute density ratio at altitude. Motor thrust scales linearly
//! with air density, so this factor is used to increase throttle at
//! higher altitudes.

/// Compute the air density ratio (rho / rho_0) at a given altitude.
///
/// Uses the ISA troposphere formula:
///   rho/rho0 = (1 - L*h / T0)^(g/(R*L) - 1)
///
/// Simplified to: (1 - 0.0000226 * h)^4.256
///
/// - `altitude_m`: altitude above sea level in meters (clamped to 0..11000)
/// - Returns: density ratio (1.0 at sea level, ~0.91 at 1000m, ~0.74 at 3000m)
pub fn air_density_ratio(altitude_m: f32) -> f32 {
    // Clamp to troposphere (0..11000m). Above 11km, use tropopause value.
    let h = altitude_m.clamp(0.0, 11000.0);
    let base = 1.0 - 0.0000226 * h;
    // base^4.256 via exp(4.256 * ln(base))
    if base <= 0.0 {
        // At or above ~44km (shouldn't happen with clamp, but safety)
        0.0
    } else {
        libm::expf(4.256 * libm::logf(base))
    }
}

/// Compute a throttle compensation factor for altitude.
///
/// At higher altitudes, motors produce less thrust per unit throttle.
/// Multiply throttle by this factor to compensate.
///
/// - `altitude_m`: altitude above sea level in meters
/// - Returns: compensation factor (1.0 at sea level, >1.0 at altitude)
pub fn throttle_altitude_compensation(altitude_m: f32) -> f32 {
    let ratio = air_density_ratio(altitude_m);
    if ratio > 0.1 {
        1.0 / ratio
    } else {
        // Safety cap: don't amplify throttle more than 10x
        10.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f32 = 0.02;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < TOL
    }

    #[test]
    fn test_sea_level_density_is_one() {
        assert!(approx_eq(air_density_ratio(0.0), 1.0));
    }

    #[test]
    fn test_1000m_density() {
        // ISA: ~0.9075 at 1000m
        let ratio = air_density_ratio(1000.0);
        assert!(ratio > 0.89 && ratio < 0.93,
            "Expected ~0.91 at 1000m, got {}", ratio);
    }

    #[test]
    fn test_3000m_density() {
        // ISA: ~0.742 at 3000m
        let ratio = air_density_ratio(3000.0);
        assert!(ratio > 0.72 && ratio < 0.76,
            "Expected ~0.74 at 3000m, got {}", ratio);
    }

    #[test]
    fn test_5000m_density() {
        // ISA: ~0.601 at 5000m
        let ratio = air_density_ratio(5000.0);
        assert!(ratio > 0.57 && ratio < 0.63,
            "Expected ~0.60 at 5000m, got {}", ratio);
    }

    #[test]
    fn test_negative_altitude_clamps_to_zero() {
        assert!(approx_eq(air_density_ratio(-500.0), 1.0));
    }

    #[test]
    fn test_very_high_altitude_clamps() {
        // Above 11000m, should use tropopause value
        let a = air_density_ratio(11000.0);
        let b = air_density_ratio(20000.0);
        assert!(approx_eq(a, b));
    }

    #[test]
    fn test_density_monotonically_decreasing() {
        let mut prev = air_density_ratio(0.0);
        for h in (500..=10000).step_by(500) {
            let curr = air_density_ratio(h as f32);
            assert!(curr < prev,
                "Density should decrease: at {}m got {}, prev was {}", h, curr, prev);
            prev = curr;
        }
    }

    #[test]
    fn test_throttle_compensation_sea_level() {
        assert!(approx_eq(throttle_altitude_compensation(0.0), 1.0));
    }

    #[test]
    fn test_throttle_compensation_at_altitude() {
        let comp = throttle_altitude_compensation(3000.0);
        // At 3000m, ratio ~0.74, so compensation ~1.35
        assert!(comp > 1.3 && comp < 1.4,
            "Expected ~1.35 at 3000m, got {}", comp);
    }

    #[test]
    fn test_throttle_compensation_inverse_of_density() {
        for h in [0.0, 1000.0, 2000.0, 5000.0, 8000.0] {
            let ratio = air_density_ratio(h);
            let comp = throttle_altitude_compensation(h);
            let product = ratio * comp;
            assert!(approx_eq(product, 1.0),
                "ratio * compensation should be ~1.0 at {}m: {} * {} = {}",
                h, ratio, comp, product);
        }
    }
}

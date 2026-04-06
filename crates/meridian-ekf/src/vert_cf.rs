//! Vertical complementary filter — lightweight altitude/climb rate estimation.
//!
//! Blends barometer altitude (low frequency, no drift) with accelerometer
//! vertical acceleration (high frequency, drifts). Used as:
//! - Quick altitude during EKF initialization
//! - Fallback when full EKF is unhealthy
//!
//! Source: ArduPilot uses this in AP_InertialNav for early altitude before EKF converges.

/// Vertical complementary filter state.
///
/// P3: 3rd-order filter with accel bias estimation (k3 term).
/// The bias state slowly corrects persistent accelerometer offset.
#[derive(Debug, Clone)]
pub struct VerticalCF {
    /// Estimated altitude (m, positive up).
    pub altitude: f32,
    /// Estimated climb rate (m/s, positive up).
    pub climb_rate: f32,
    /// P3: Estimated vertical accelerometer bias (m/s^2).
    /// Slowly converges to true accel bias via k3 integration.
    pub accel_bias: f32,
    /// Filter time constant for baro correction (s). Higher = trust accel more.
    pub time_constant: f32,
    /// Baro ground offset (set during calibration).
    ground_alt: f32,
    /// Whether ground calibration is complete.
    calibrated: bool,
    /// Calibration sample accumulator.
    cal_sum: f32,
    cal_count: u16,
    cal_target: u16,
}

impl VerticalCF {
    pub fn new() -> Self {
        Self {
            altitude: 0.0,
            climb_rate: 0.0,
            accel_bias: 0.0,
            time_constant: 3.0, // 3 second time constant
            ground_alt: 0.0,
            calibrated: false,
            cal_sum: 0.0,
            cal_count: 0,
            cal_target: 5, // average 5 samples for ground cal
        }
    }

    /// Feed a barometer altitude sample. Call at baro rate (50 Hz).
    pub fn update_baro(&mut self, baro_alt: f32, dt: f32) {
        // Ground calibration
        if !self.calibrated {
            self.cal_sum += baro_alt;
            self.cal_count += 1;
            if self.cal_count >= self.cal_target {
                self.ground_alt = self.cal_sum / self.cal_count as f32;
                self.calibrated = true;
                self.altitude = 0.0;
            }
            return;
        }

        let baro_relative = baro_alt - self.ground_alt;

        // 3rd-order complementary filter correction (P3: added k3 for bias)
        let alt_err = baro_relative - self.altitude;
        let tc = self.time_constant;
        let k1 = 3.0 / tc;               // position correction gain
        let k2 = 3.0 / (tc * tc);         // velocity correction gain
        let k3 = 1.0 / (tc * tc * tc);    // P3: accel bias correction gain

        self.altitude += alt_err * k1 * dt;
        self.climb_rate += alt_err * k2 * dt;
        self.accel_bias += alt_err * k3 * dt;
        // Clamp bias to reasonable range (+/- 1 m/s^2)
        self.accel_bias = self.accel_bias.clamp(-1.0, 1.0);
    }

    /// Feed a vertical acceleration sample (body Z, corrected for tilt).
    /// `accel_z_up`: vertical acceleration in earth frame, positive up, gravity removed.
    /// Call at IMU rate.
    pub fn update_accel(&mut self, accel_z_up: f32, dt: f32) {
        if !self.calibrated { return; }
        // P3: subtract estimated accel bias before integrating
        let corrected_accel = accel_z_up - self.accel_bias;
        self.climb_rate += corrected_accel * dt;
        self.altitude += self.climb_rate * dt;
    }

    pub fn is_calibrated(&self) -> bool { self.calibrated }

    pub fn reset(&mut self) {
        self.altitude = 0.0;
        self.climb_rate = 0.0;
        self.accel_bias = 0.0;
        self.calibrated = false;
        self.cal_sum = 0.0;
        self.cal_count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ground_calibration() {
        let mut cf = VerticalCF::new();
        assert!(!cf.is_calibrated());

        // Feed 5 baro samples at ~100m altitude
        for _ in 0..5 {
            cf.update_baro(100.0, 0.02);
        }
        assert!(cf.is_calibrated());
        assert!((cf.altitude - 0.0).abs() < 0.1, "Should be 0 after ground cal");
    }

    #[test]
    fn test_altitude_tracking() {
        let mut cf = VerticalCF::new();
        // Calibrate at ground
        for _ in 0..5 { cf.update_baro(0.0, 0.02); }

        // Feed 10m baro altitude for 10 seconds (500 samples at 50Hz)
        for _ in 0..500 {
            cf.update_baro(10.0, 0.02);
        }
        assert!((cf.altitude - 10.0).abs() < 1.0,
            "Should converge to 10m, got {}", cf.altitude);
    }

    #[test]
    fn test_accel_integration() {
        let mut cf = VerticalCF::new();
        for _ in 0..5 { cf.update_baro(0.0, 0.02); }

        // Apply 1 m/s² upward for 1 second
        for _ in 0..400 {
            cf.update_accel(1.0, 0.0025);
        }
        // Should have climbed: altitude > 0 and climb_rate > 0
        assert!(cf.climb_rate > 0.5, "Climb rate should increase: {}", cf.climb_rate);
        assert!(cf.altitude > 0.0, "Altitude should increase: {}", cf.altitude);
    }
}

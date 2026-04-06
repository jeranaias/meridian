//! Per-motor compass compensation — advanced motor interference removal.
//!
//! ArduPilot reference: `Compass_PerMotor.cpp`
//!
//! Three compensation modes:
//! 1. **Throttle linear**: `mag += comp * throttle` (existing in compass_cal.rs)
//! 2. **Current linear**: `mag += comp * current` (existing in compass_cal.rs)
//! 3. **Per-motor power^0.65**: `mag += sum(coeff[i] * throttle[i]^0.65)`
//!
//! Mode 3 is the most accurate because motor magnetic interference is
//! proportional to power (~throttle^0.65 due to motor efficiency curves),
//! and each motor contributes differently based on its distance/orientation
//! to the compass.

use meridian_math::{frames::Body, Vec3};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Maximum number of motors supported for per-motor compensation.
pub const MAX_MOTORS: usize = 8;

/// Power-law exponent: interference ∝ throttle^0.65.
const POWER_EXPONENT: f32 = 0.65;

// ---------------------------------------------------------------------------
// Per-motor compensation
// ---------------------------------------------------------------------------

/// Per-motor compass compensation coefficients.
///
/// Each motor has a 3-axis coefficient vector. The compensation for motor `i`
/// is `coeff[i] * throttle[i]^0.65`.
#[derive(Debug, Clone, Copy)]
pub struct PerMotorCompensation {
    /// Per-motor coefficients [motor_idx][axis]. Units: mGauss per unit power.
    pub coefficients: [Vec3<Body>; MAX_MOTORS],
    /// Number of active motors.
    pub num_motors: u8,
    /// Whether compensation is enabled and calibrated.
    pub enabled: bool,
}

impl Default for PerMotorCompensation {
    fn default() -> Self {
        Self {
            coefficients: [Vec3::<Body>::zero(); MAX_MOTORS],
            num_motors: 4,
            enabled: false,
        }
    }
}

impl PerMotorCompensation {
    /// Create compensation for the given number of motors.
    pub fn new(num_motors: u8) -> Self {
        Self {
            num_motors: num_motors.min(MAX_MOTORS as u8),
            ..Default::default()
        }
    }

    /// Apply per-motor power^0.65 compensation.
    ///
    /// `throttles`: per-motor throttle values (0.0 to 1.0).
    /// `mag`: raw magnetometer reading to correct.
    ///
    /// Returns the corrected magnetometer reading.
    pub fn apply(&self, mag: &Vec3<Body>, throttles: &[f32]) -> Vec3<Body> {
        if !self.enabled {
            return *mag;
        }

        let mut correction = Vec3::<Body>::zero();
        let n = (self.num_motors as usize).min(throttles.len()).min(MAX_MOTORS);

        for i in 0..n {
            let t = throttles[i].max(0.0).min(1.0);
            // power^0.65 — use powf approximation via exp/ln
            let power_factor = if t > 0.001 {
                libm::powf(t, POWER_EXPONENT)
            } else {
                0.0
            };
            correction.x += self.coefficients[i].x * power_factor;
            correction.y += self.coefficients[i].y * power_factor;
            correction.z += self.coefficients[i].z * power_factor;
        }

        Vec3::<Body>::new(
            mag.x + correction.x,
            mag.y + correction.y,
            mag.z + correction.z,
        )
    }
}

// ---------------------------------------------------------------------------
// Calibration learner (online)
// ---------------------------------------------------------------------------

/// Online learner for per-motor compensation coefficients.
///
/// Collects samples with known throttle states and solves for the
/// per-motor coefficients using least-squares accumulation.
pub struct PerMotorCalLearner {
    /// Number of motors.
    num_motors: u8,
    /// Accumulated throttle^0.65 * throttle^0.65 (AtA diagonal approximation).
    sum_power_sq: [f32; MAX_MOTORS],
    /// Accumulated throttle^0.65 * mag_residual (Atb).
    sum_power_mag: [Vec3<Body>; MAX_MOTORS],
    /// Number of samples collected.
    sample_count: u32,
}

impl PerMotorCalLearner {
    pub fn new(num_motors: u8) -> Self {
        Self {
            num_motors: num_motors.min(MAX_MOTORS as u8),
            sum_power_sq: [0.0; MAX_MOTORS],
            sum_power_mag: [Vec3::<Body>::zero(); MAX_MOTORS],
            sample_count: 0,
        }
    }

    /// Add a sample: magnetometer residual (after hard/soft-iron removal)
    /// paired with current throttle state.
    pub fn add_sample(&mut self, mag_residual: &Vec3<Body>, throttles: &[f32]) {
        let n = (self.num_motors as usize).min(throttles.len()).min(MAX_MOTORS);
        for i in 0..n {
            let t = throttles[i].max(0.0).min(1.0);
            let p = if t > 0.001 { libm::powf(t, POWER_EXPONENT) } else { 0.0 };
            self.sum_power_sq[i] += p * p;
            self.sum_power_mag[i].x += p * mag_residual.x;
            self.sum_power_mag[i].y += p * mag_residual.y;
            self.sum_power_mag[i].z += p * mag_residual.z;
        }
        self.sample_count += 1;
    }

    /// Solve for coefficients. Returns None if insufficient data.
    pub fn solve(&self) -> Option<PerMotorCompensation> {
        if self.sample_count < 100 {
            return None;
        }
        let n = self.num_motors as usize;
        let mut comp = PerMotorCompensation::new(self.num_motors);

        for i in 0..n {
            if self.sum_power_sq[i] > 1e-6 {
                let inv = 1.0 / self.sum_power_sq[i];
                comp.coefficients[i] = Vec3::<Body>::new(
                    self.sum_power_mag[i].x * inv,
                    self.sum_power_mag[i].y * inv,
                    self.sum_power_mag[i].z * inv,
                );
            }
        }
        comp.enabled = true;
        Some(comp)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_no_compensation_when_disabled() {
        let comp = PerMotorCompensation::default();
        let mag = Vec3::<Body>::new(100.0, 200.0, 300.0);
        let result = comp.apply(&mag, &[0.5, 0.5, 0.5, 0.5]);
        assert_eq!(result.x, 100.0);
    }

    #[test]
    fn test_compensation_at_full_throttle() {
        let mut comp = PerMotorCompensation::new(4);
        comp.enabled = true;
        comp.coefficients[0] = Vec3::<Body>::new(10.0, 0.0, 0.0);

        let mag = Vec3::<Body>::new(100.0, 200.0, 300.0);
        let result = comp.apply(&mag, &[1.0, 0.0, 0.0, 0.0]);
        // At throttle=1.0, power^0.65 = 1.0, so correction = 10.0
        assert!((result.x - 110.0).abs() < 0.01);
    }

    #[test]
    fn test_power_law_exponent() {
        // throttle=0.5, power = 0.5^0.65 ≈ 0.637
        let p = libm::powf(0.5, POWER_EXPONENT);
        assert!((p - 0.637).abs() < 0.01, "0.5^0.65 = {}", p);
    }
}

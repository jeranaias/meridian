//! IMU temperature calibration — 3rd-order polynomial correction.
//!
//! ArduPilot reference: `AP_InertialSensor_TCal`
//!
//! Evaluates a 3rd-order polynomial per axis to correct gyro and accel
//! bias drift as a function of temperature:
//!
//!   correction[axis] = c0[axis] + c1[axis] * dT + c2[axis] * dT^2 + c3[axis] * dT^3
//!
//! where dT = (current_temp - cal_temp).
//!
//! The polynomial coefficients are learned during a thermal sweep calibration
//! (requires heating/cooling the flight controller through ~40°C range while
//! stationary) and persisted to parameters.

use meridian_math::{frames::Body, Vec3};

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Polynomial coefficients for one sensor (accel or gyro).
/// `coeff[order][axis]` where order=0..3 and axis=x,y,z.
#[derive(Debug, Clone, Copy)]
pub struct TempCalCoefficients {
    /// Coefficient c0 (offset at cal_temp).
    pub c0: [f32; 3],
    /// Coefficient c1 (linear).
    pub c1: [f32; 3],
    /// Coefficient c2 (quadratic).
    pub c2: [f32; 3],
    /// Coefficient c3 (cubic).
    pub c3: [f32; 3],
}

impl Default for TempCalCoefficients {
    fn default() -> Self {
        Self {
            c0: [0.0; 3],
            c1: [0.0; 3],
            c2: [0.0; 3],
            c3: [0.0; 3],
        }
    }
}

/// Temperature calibration for one IMU instance.
#[derive(Debug, Clone, Copy)]
pub struct ImuTempCal {
    /// Reference temperature at which calibration was performed (deg C).
    pub cal_temp: f32,
    /// Gyro polynomial coefficients.
    pub gyro_coeff: TempCalCoefficients,
    /// Accel polynomial coefficients.
    pub accel_coeff: TempCalCoefficients,
    /// Whether calibration data has been loaded.
    pub enabled: bool,
}

impl Default for ImuTempCal {
    fn default() -> Self {
        Self {
            cal_temp: 25.0,
            gyro_coeff: TempCalCoefficients::default(),
            accel_coeff: TempCalCoefficients::default(),
            enabled: false,
        }
    }
}

impl ImuTempCal {
    /// Evaluate the 3rd-order polynomial correction for a single axis.
    ///
    /// correction = c0 + c1*dT + c2*dT^2 + c3*dT^3
    #[inline]
    fn eval_poly(coeff: &TempCalCoefficients, axis: usize, dt: f32) -> f32 {
        let dt2 = dt * dt;
        let dt3 = dt2 * dt;
        coeff.c0[axis] + coeff.c1[axis] * dt + coeff.c2[axis] * dt2 + coeff.c3[axis] * dt3
    }

    /// Apply temperature correction to a gyro reading.
    pub fn correct_gyro(&self, gyro: &Vec3<Body>, temperature: f32) -> Vec3<Body> {
        if !self.enabled {
            return *gyro;
        }
        let dt = temperature - self.cal_temp;
        Vec3::<Body>::new(
            gyro.x - Self::eval_poly(&self.gyro_coeff, 0, dt),
            gyro.y - Self::eval_poly(&self.gyro_coeff, 1, dt),
            gyro.z - Self::eval_poly(&self.gyro_coeff, 2, dt),
        )
    }

    /// Apply temperature correction to an accel reading.
    pub fn correct_accel(&self, accel: &Vec3<Body>, temperature: f32) -> Vec3<Body> {
        if !self.enabled {
            return *accel;
        }
        let dt = temperature - self.cal_temp;
        Vec3::<Body>::new(
            accel.x - Self::eval_poly(&self.accel_coeff, 0, dt),
            accel.y - Self::eval_poly(&self.accel_coeff, 1, dt),
            accel.z - Self::eval_poly(&self.accel_coeff, 2, dt),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_no_correction_when_disabled() {
        let cal = ImuTempCal::default();
        let gyro = Vec3::<Body>::new(1.0, 2.0, 3.0);
        let corrected = cal.correct_gyro(&gyro, 50.0);
        assert_eq!(corrected.x, 1.0);
        assert_eq!(corrected.y, 2.0);
        assert_eq!(corrected.z, 3.0);
    }

    #[test]
    fn test_linear_correction() {
        let mut cal = ImuTempCal {
            cal_temp: 25.0,
            enabled: true,
            ..Default::default()
        };
        // c1 = [0.01, 0.02, 0.03] rad/s per degree
        cal.gyro_coeff.c1 = [0.01, 0.02, 0.03];

        let gyro = Vec3::<Body>::new(0.0, 0.0, 0.0);
        let corrected = cal.correct_gyro(&gyro, 35.0); // dT = 10

        // correction = c1 * dT = [0.1, 0.2, 0.3]
        // corrected = gyro - correction = [-0.1, -0.2, -0.3]
        assert!((corrected.x - (-0.1)).abs() < 1e-6);
        assert!((corrected.y - (-0.2)).abs() < 1e-6);
        assert!((corrected.z - (-0.3)).abs() < 1e-6);
    }

    #[test]
    fn test_cubic_correction() {
        let mut cal = ImuTempCal {
            cal_temp: 25.0,
            enabled: true,
            ..Default::default()
        };
        cal.accel_coeff.c0 = [1.0, 0.0, 0.0];
        cal.accel_coeff.c1 = [0.1, 0.0, 0.0];
        cal.accel_coeff.c2 = [0.01, 0.0, 0.0];
        cal.accel_coeff.c3 = [0.001, 0.0, 0.0];

        let accel = Vec3::<Body>::new(0.0, 0.0, -9.81);
        let corrected = cal.correct_accel(&accel, 30.0); // dT = 5

        // correction_x = 1.0 + 0.1*5 + 0.01*25 + 0.001*125 = 1.0 + 0.5 + 0.25 + 0.125 = 1.875
        let expected_x = 0.0 - 1.875;
        assert!((corrected.x - expected_x).abs() < 1e-4, "x = {}", corrected.x);
    }

    #[test]
    fn test_zero_dt_uses_c0_only() {
        let mut cal = ImuTempCal {
            cal_temp: 25.0,
            enabled: true,
            ..Default::default()
        };
        cal.gyro_coeff.c0 = [0.5, 0.0, 0.0];
        cal.gyro_coeff.c1 = [999.0, 0.0, 0.0]; // should not matter at dT=0

        let gyro = Vec3::<Body>::new(1.0, 0.0, 0.0);
        let corrected = cal.correct_gyro(&gyro, 25.0); // dT = 0
        assert!((corrected.x - 0.5).abs() < 1e-6);
    }
}

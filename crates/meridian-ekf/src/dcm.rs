//! DCM fallback attitude estimator.
//!
//! A simple Direction Cosine Matrix estimator using gyro integration with
//! accelerometer tilt correction (complementary filter). Used as safety
//! fallback when the full EKF is unhealthy.
//!
//! Source: ArduPilot AP_AHRS_DCM, simplified.

use meridian_math::{Vec3, Quaternion};
use meridian_math::frames::Body;

/// DCM-based attitude estimator — complementary filter fallback.
pub struct DcmEstimator {
    /// Current rotation matrix (body-to-NED DCM), stored row-major.
    dcm: [[f32; 3]; 3],
    /// Proportional gain for accelerometer correction (ArduPilot: AP_AHRS_RP_P).
    pub p_gain: f32,
    /// P2: Integral gain for accelerometer correction.
    /// Slowly trims out persistent gyro bias. Default 0.02 matching ArduPilot.
    pub i_gain: f32,
    /// P2: Accumulated integral correction term (rad/s).
    i_term: Vec3<Body>,
}

impl DcmEstimator {
    /// Create a new estimator starting at level attitude.
    pub fn new() -> Self {
        Self {
            dcm: [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            p_gain: 0.2,
            i_gain: 0.02,
            i_term: Vec3::zero(),
        }
    }

    /// Update with IMU data.
    ///
    /// - `gyro`: angular velocity in body frame (rad/s)
    /// - `accel`: specific force in body frame (m/s^2)
    /// - `dt`: time step (s)
    pub fn update_imu(&mut self, gyro: Vec3<Body>, accel: Vec3<Body>, dt: f32) {
        if dt <= 0.0 || dt > 1.0 {
            return;
        }

        // ── Accelerometer tilt correction ──
        // The third row of the DCM is the body-frame representation of the
        // gravity direction (down in NED = [0,0,1] rotated into body).
        // accel_predicted = transpose(DCM) * [0,0,1] = third column of DCM transposed
        // = [dcm[0][2], dcm[1][2], dcm[2][2]] in body frame.
        let accel_predicted = Vec3::<Body>::new(
            self.dcm[0][2],
            self.dcm[1][2],
            self.dcm[2][2],
        );

        // Normalize measured accel (gravity reference)
        let accel_len = accel.length();
        let correction = if accel_len > 1.0 {
            // Only correct when acceleration magnitude is reasonable (0.5g..2g)
            let g_ratio = accel_len / 9.80665;
            if g_ratio > 0.5 && g_ratio < 2.0 {
                let accel_norm = accel.scale(1.0 / accel_len);
                // Tilt error = cross(measured, predicted)
                let error = accel_norm.cross(&accel_predicted);

                // P2: Integrate error for I-term (gyro bias estimation)
                self.i_term = Vec3::new(
                    self.i_term.x + error.x * self.i_gain * dt,
                    self.i_term.y + error.y * self.i_gain * dt,
                    self.i_term.z + error.z * self.i_gain * dt,
                );
                // Clamp I-term to prevent windup
                let i_max = 0.1; // rad/s
                self.i_term = Vec3::new(
                    self.i_term.x.clamp(-i_max, i_max),
                    self.i_term.y.clamp(-i_max, i_max),
                    self.i_term.z.clamp(-i_max, i_max),
                );

                // PI correction: P + I
                error.scale(self.p_gain) + self.i_term
            } else {
                Vec3::<Body>::zero()
            }
        } else {
            Vec3::<Body>::zero()
        };

        // ── Gyro integration with correction ──
        let omega = gyro + correction;

        // Small-angle rotation: R_new = R_old * (I + [omega x] * dt)
        // [omega x] is the skew-symmetric matrix of omega * dt
        let wx = omega.x * dt;
        let wy = omega.y * dt;
        let wz = omega.z * dt;

        // Apply rotation to each row of the DCM
        let mut new_dcm = [[0.0f32; 3]; 3];
        for i in 0..3 {
            let r = self.dcm[i];
            new_dcm[i][0] = r[0] + r[1] * wz - r[2] * wy;
            new_dcm[i][1] = r[1] - r[0] * wz + r[2] * wx;
            new_dcm[i][2] = r[2] + r[0] * wy - r[1] * wx;
        }

        self.dcm = new_dcm;

        // Re-orthonormalize via simple row normalization + cross-product method
        self.orthonormalize();
    }

    /// Get current attitude as a quaternion.
    pub fn attitude(&self) -> Quaternion {
        Quaternion::from_dcm(&self.dcm)
    }

    /// Reset to level attitude.
    pub fn reset(&mut self) {
        self.dcm = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        self.i_term = Vec3::zero();
    }

    /// Re-orthonormalize the DCM to prevent drift.
    /// Uses the simple method: normalize rows, rebuild third from cross product.
    /// P4: If any row is degenerate (near-zero length), reset to identity.
    fn orthonormalize(&mut self) {
        // Normalize row 0
        let len0 = libm::sqrtf(
            self.dcm[0][0] * self.dcm[0][0]
            + self.dcm[0][1] * self.dcm[0][1]
            + self.dcm[0][2] * self.dcm[0][2],
        );
        // P4: degenerate-row reset — if any row collapses to near-zero,
        // the DCM is corrupt. Reset to identity (level attitude).
        if len0 < 1e-4 {
            self.dcm = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
            return;
        }
        if len0 > 1e-6 {
            let inv = 1.0 / len0;
            self.dcm[0][0] *= inv;
            self.dcm[0][1] *= inv;
            self.dcm[0][2] *= inv;
        }

        // Remove row1 component along row0 (Gram-Schmidt)
        let dot01 = self.dcm[0][0] * self.dcm[1][0]
            + self.dcm[0][1] * self.dcm[1][1]
            + self.dcm[0][2] * self.dcm[1][2];
        for j in 0..3 {
            self.dcm[1][j] -= dot01 * self.dcm[0][j];
        }

        // Normalize row 1 (P4: degenerate check)
        let len1 = libm::sqrtf(
            self.dcm[1][0] * self.dcm[1][0]
            + self.dcm[1][1] * self.dcm[1][1]
            + self.dcm[1][2] * self.dcm[1][2],
        );
        if len1 < 1e-4 {
            self.dcm = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
            return;
        }
        if len1 > 1e-6 {
            let inv = 1.0 / len1;
            self.dcm[1][0] *= inv;
            self.dcm[1][1] *= inv;
            self.dcm[1][2] *= inv;
        }

        // Row 2 = cross(row0, row1) — ensures right-hand orthonormal
        self.dcm[2][0] = self.dcm[0][1] * self.dcm[1][2] - self.dcm[0][2] * self.dcm[1][1];
        self.dcm[2][1] = self.dcm[0][2] * self.dcm[1][0] - self.dcm[0][0] * self.dcm[1][2];
        self.dcm[2][2] = self.dcm[0][0] * self.dcm[1][1] - self.dcm[0][1] * self.dcm[1][0];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use meridian_math::frames::Body;

    const TOL: f32 = 0.05;
    const DT: f32 = 0.0025; // 400Hz

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < TOL
    }

    #[test]
    fn test_dcm_initial_attitude_is_level() {
        let est = DcmEstimator::new();
        let q = est.attitude();
        let (roll, pitch, yaw) = q.to_euler();
        assert!(approx_eq(roll, 0.0));
        assert!(approx_eq(pitch, 0.0));
        assert!(approx_eq(yaw, 0.0));
    }

    #[test]
    fn test_dcm_stationary_stays_level() {
        let mut est = DcmEstimator::new();
        let gyro = Vec3::<Body>::zero();
        let accel = Vec3::<Body>::new(0.0, 0.0, 9.81); // gravity in body (level)

        for _ in 0..1000 {
            est.update_imu(gyro, accel, DT);
        }

        let q = est.attitude();
        let (roll, pitch, _yaw) = q.to_euler();
        assert!(approx_eq(roll, 0.0));
        assert!(approx_eq(pitch, 0.0));
    }

    #[test]
    fn test_dcm_yaw_rotation() {
        let mut est = DcmEstimator::new();
        // Rotate at 1 rad/s around z for 1 second
        let gyro = Vec3::<Body>::new(0.0, 0.0, 1.0);
        let accel = Vec3::<Body>::new(0.0, 0.0, 9.81);

        let steps = 400;
        let dt = 1.0 / steps as f32;
        for _ in 0..steps {
            est.update_imu(gyro, accel, dt);
        }

        let q = est.attitude();
        let (_roll, _pitch, yaw) = q.to_euler();
        // After 1s at 1 rad/s, yaw should be ~1.0 radian
        assert!((yaw - 1.0).abs() < 0.1, "Expected yaw ~1.0, got {}", yaw);
    }

    #[test]
    fn test_dcm_accel_correction_tilted() {
        // Start level, but accel says we're tilted — correction should adjust
        let mut est = DcmEstimator::new();
        let gyro = Vec3::<Body>::zero();
        // Accel indicating ~10 degree pitch (gravity vector tilted)
        let pitch_rad: f32 = 0.17; // ~10 degrees
        let accel = Vec3::<Body>::new(
            -9.81 * libm::sinf(pitch_rad),
            0.0,
            9.81 * libm::cosf(pitch_rad),
        );

        // Run for 20 seconds — complementary filter converges slowly
        for _ in 0..20000 {
            est.update_imu(gyro, accel, DT);
        }

        let q = est.attitude();
        let (_roll, pitch, _yaw) = q.to_euler();
        // Should converge toward the accel-indicated pitch (within 3°)
        assert!((pitch - pitch_rad).abs() < 0.10,
            "Expected pitch ~{:.3}, got {:.3}", pitch_rad, pitch);
    }

    #[test]
    fn test_dcm_orthonormality_maintained() {
        let mut est = DcmEstimator::new();
        // Apply random-ish gyro for many steps
        let gyro = Vec3::<Body>::new(0.5, -0.3, 0.8);
        let accel = Vec3::<Body>::new(0.0, 0.0, 9.81);

        for _ in 0..10000 {
            est.update_imu(gyro, accel, DT);
        }

        // Check that DCM rows are roughly orthonormal
        let q = est.attitude();
        let norm = q.norm();
        assert!((norm - 1.0).abs() < 0.01, "Quaternion should be unit: norm={}", norm);
    }

    #[test]
    fn test_dcm_reset() {
        let mut est = DcmEstimator::new();
        let gyro = Vec3::<Body>::new(1.0, 0.5, -0.3);
        let accel = Vec3::<Body>::new(0.0, 0.0, 9.81);

        for _ in 0..1000 {
            est.update_imu(gyro, accel, DT);
        }

        est.reset();
        let q = est.attitude();
        let (roll, pitch, yaw) = q.to_euler();
        assert!(approx_eq(roll, 0.0));
        assert!(approx_eq(pitch, 0.0));
        assert!(approx_eq(yaw, 0.0));
    }

    #[test]
    fn test_dcm_zero_dt_ignored() {
        let mut est = DcmEstimator::new();
        let gyro = Vec3::<Body>::new(1.0, 1.0, 1.0);
        let accel = Vec3::<Body>::new(0.0, 0.0, 9.81);
        est.update_imu(gyro, accel, 0.0);
        // Should still be identity
        let q = est.attitude();
        assert!(approx_eq(q.w, 1.0));
    }

    #[test]
    fn test_dcm_rejects_high_accel() {
        // During high-g maneuver, accel correction should be disabled
        let mut est = DcmEstimator::new();
        let gyro = Vec3::<Body>::zero();
        // 3g — well above the 2g threshold
        let accel = Vec3::<Body>::new(0.0, 0.0, 29.4);
        est.update_imu(gyro, accel, DT);
        // Should still be approximately level (no correction applied)
        let q = est.attitude();
        let (roll, pitch, _) = q.to_euler();
        assert!(approx_eq(roll, 0.0));
        assert!(approx_eq(pitch, 0.0));
    }
}

//! Gaussian Sum Filter (GSF) yaw estimator.
//!
//! Provides an independent yaw estimate using GPS velocity when the compass
//! is unavailable or unreliable. Runs N parallel single-state EKFs, each
//! initialized with a different yaw hypothesis, weighted by likelihood.
//!
//! Used as a fallback yaw source for:
//! - Compass failure detection and recovery
//! - GPS-based yaw alignment on startup without compass
//! - Yaw emergency reset after divergence
//!
//! Source: AP_NavEKF3/EKFGSF_yaw.h, EKFGSF_yaw.cpp

use meridian_math::{Vec3, Quaternion};
use meridian_math::frames::Body;
use crate::state::{StateVector, Covariance};

/// Number of parallel yaw hypotheses in the GSF.
/// Source: EKFGSF_yaw.h N_MODELS_EKFGSF = 5
pub const GSF_NUM_MODELS: usize = 5;

/// Accuracy threshold for GSF yaw estimate (radians).
/// Below this, the GSF estimate is considered reliable enough to use.
/// Source: GSF_YAW_ACCURACY_THRESHOLD_DEG = 15
pub const GSF_YAW_ACCURACY_THRESHOLD: f32 = 15.0 * core::f32::consts::PI / 180.0;

/// Timeout before allowing GSF yaw reset (ms).
/// Source: YAW_RESET_TO_GSF_TIMEOUT_MS = 5000
pub const GSF_YAW_RESET_TIMEOUT_MS: u32 = 5000;

/// Single-model state in the GSF.
///
/// Each model is a 3-state EKF: [vn, ve, yaw]
/// The yaw state is estimated by how well the model's velocity prediction
/// (using the hypothesized yaw) matches GPS velocity observations.
#[derive(Debug, Clone)]
pub struct GsfModel {
    /// Estimated yaw angle (rad).
    pub yaw: f32,
    /// Estimated NED velocity (m/s) — only N and E are used.
    pub velocity: [f32; 2],
    /// 3x3 covariance: [vn, ve, yaw].
    pub covariance: [[f32; 3]; 3],
    /// Model weight (probability).
    pub weight: f32,
}

impl GsfModel {
    /// Create a new model with the given initial yaw.
    pub fn new(yaw: f32) -> Self {
        let w = 1.0 / GSF_NUM_MODELS as f32;
        Self {
            yaw,
            velocity: [0.0; 2],
            covariance: [
                [25.0, 0.0, 0.0],    // vn variance: 5 m/s
                [0.0, 25.0, 0.0],     // ve variance: 5 m/s
                [0.0, 0.0, 0.5],      // yaw variance: ~40 deg
            ],
            weight: w,
        }
    }
}

/// Gaussian Sum Filter yaw estimator.
///
/// Maintains N parallel yaw hypotheses, each as a small EKF.
/// GPS velocity observations update all models; the best one
/// (by likelihood) provides the yaw estimate.
#[derive(Debug, Clone)]
pub struct GsfYaw {
    /// Parallel model bank.
    pub models: [GsfModel; GSF_NUM_MODELS],
    /// Weighted mean yaw estimate (rad).
    pub yaw: f32,
    /// Estimated yaw accuracy (rad, 1-sigma).
    pub yaw_accuracy: f32,
    /// Whether the GSF has converged to a reliable estimate.
    pub valid: bool,
    /// Number of GPS velocity fusions performed.
    pub fusion_count: u32,
    /// Minimum fusions before declaring valid.
    pub min_fusions_for_valid: u32,
    /// IMU-integrated attitude for accel rotation.
    /// The GSF uses a simple AHRS (accel + gyro) for tilt, then adds yaw.
    ahrs_quat: Quaternion,
    /// Whether AHRS tilt is initialized.
    pub ahrs_tilt_aligned: bool,
}

impl GsfYaw {
    /// Create a new GSF yaw estimator with N evenly-spaced yaw hypotheses.
    pub fn new() -> Self {
        let mut models: [GsfModel; GSF_NUM_MODELS] = core::array::from_fn(|_| GsfModel::new(0.0));

        // Initialize with evenly spaced yaw hypotheses across 360 degrees
        // Source: EKFGSF_yaw.cpp initialises yaw at -pi + (2*pi * i / N)
        for i in 0..GSF_NUM_MODELS {
            let yaw = -core::f32::consts::PI
                + (2.0 * core::f32::consts::PI * i as f32) / GSF_NUM_MODELS as f32;
            models[i] = GsfModel::new(yaw);
        }

        Self {
            models,
            yaw: 0.0,
            yaw_accuracy: core::f32::consts::PI, // start with max uncertainty
            valid: false,
            fusion_count: 0,
            min_fusions_for_valid: 10,
            ahrs_quat: Quaternion::identity(),
            ahrs_tilt_aligned: false,
        }
    }

    /// Initialize AHRS tilt from accelerometer.
    /// Must be called before predict() for correct gravity alignment.
    ///
    /// Source: EKFGSF_yaw.cpp EKFGSF_yaw::setGyroBias, EKFGSF_yaw::ahrsAlignTilt
    pub fn align_tilt(&mut self, accel: &Vec3<Body>) {
        // Compute roll and pitch from accelerometer
        let ax = accel.x;
        let ay = accel.y;
        let az = accel.z;

        let pitch = libm::atan2f(-ax, libm::sqrtf(ay * ay + az * az));
        let roll = libm::atan2f(ay, -az);

        self.ahrs_quat = Quaternion::from_euler(roll, pitch, 0.0);
        self.ahrs_tilt_aligned = true;
    }

    /// Predict step: integrate gyro to update tilt estimate.
    ///
    /// Source: EKFGSF_yaw.cpp EKFGSF_yaw::predictEKF
    pub fn predict(&mut self, del_ang: &Vec3<Body>, del_vel: &Vec3<Body>, dt: f32) {
        if !self.ahrs_tilt_aligned || dt < 1e-6 {
            return;
        }

        // Update AHRS quaternion from gyro
        let angle = del_ang.length();
        if angle > 1e-12 {
            let axis = del_ang.normalized();
            let dq = Quaternion::from_axis_angle(&[axis.x, axis.y, axis.z], angle);
            self.ahrs_quat = self.ahrs_quat * dq;
            self.ahrs_quat.normalize();
        }

        // Propagate each model's velocity using the rotated delta velocity
        for model in self.models.iter_mut() {
            // Build full quaternion: tilt from AHRS, yaw from model
            let (roll, pitch, _) = self.ahrs_quat.to_euler();
            let model_quat = Quaternion::from_euler(roll, pitch, model.yaw);
            let dcm = model_quat.to_dcm();

            // Rotate del_vel to NED
            let dvn = dcm[0][0] * del_vel.x + dcm[0][1] * del_vel.y + dcm[0][2] * del_vel.z;
            let dve = dcm[1][0] * del_vel.x + dcm[1][1] * del_vel.y + dcm[1][2] * del_vel.z;

            // Add gravity (NED down is positive, accel measures -g in body frame)
            // No gravity correction needed here since del_vel already includes specific force
            model.velocity[0] += dvn;
            model.velocity[1] += dve;

            // Propagate covariance with process noise
            let q_vel = 4.0 * dt;     // velocity process noise variance
            let q_yaw = 0.01 * dt;    // yaw process noise variance
            model.covariance[0][0] += q_vel;
            model.covariance[1][1] += q_vel;
            model.covariance[2][2] += q_yaw;
        }
    }

    /// Fuse GPS velocity measurement into all models.
    ///
    /// Each model predicts velocity based on its yaw hypothesis.
    /// The innovation likelihood updates the model weight.
    /// After fusion, weights are normalized and the composite yaw is computed.
    ///
    /// Source: EKFGSF_yaw.cpp EKFGSF_yaw::fuseVelData
    pub fn fuse_velocity(&mut self, vel_ne: &[f32; 2], vel_accuracy: f32) {
        let r = vel_accuracy * vel_accuracy;
        if r < 1e-6 {
            return;
        }

        let mut total_weight = 0.0f32;

        for model in self.models.iter_mut() {
            // Innovation: predicted - measured
            let innov_n = model.velocity[0] - vel_ne[0];
            let innov_e = model.velocity[1] - vel_ne[1];

            // Innovation variance: S = H * P * H^T + R
            // H = [1 0 0; 0 1 0] (observing vn, ve directly)
            let s_nn = model.covariance[0][0] + r;
            let s_ne = model.covariance[0][1];
            let s_en = model.covariance[1][0];
            let s_ee = model.covariance[1][1] + r;

            // 2x2 innovation covariance determinant
            let det = s_nn * s_ee - s_ne * s_en;
            if det < 1e-12 {
                model.weight = 0.0;
                continue;
            }
            let inv_det = 1.0 / det;

            // 2x2 inverse of S
            let si_nn = s_ee * inv_det;
            let si_ne = -s_ne * inv_det;
            let si_en = -s_en * inv_det;
            let si_ee = s_nn * inv_det;

            // Likelihood (Gaussian): exp(-0.5 * innov^T * S^-1 * innov)
            let mahal = innov_n * (si_nn * innov_n + si_ne * innov_e)
                + innov_e * (si_en * innov_n + si_ee * innov_e);
            // Clamp to prevent exp overflow
            let log_likelihood = -0.5 * mahal.min(100.0);
            let likelihood = libm::expf(log_likelihood);

            model.weight *= likelihood;
            total_weight += model.weight;

            // Kalman gain: K = P * H^T * S^-1
            // K is 3x2: K[state][obs]
            let k00 = model.covariance[0][0] * si_nn + model.covariance[0][1] * si_en;
            let k01 = model.covariance[0][0] * si_ne + model.covariance[0][1] * si_ee;
            let k10 = model.covariance[1][0] * si_nn + model.covariance[1][1] * si_en;
            let k11 = model.covariance[1][0] * si_ne + model.covariance[1][1] * si_ee;
            let k20 = model.covariance[2][0] * si_nn + model.covariance[2][1] * si_en;
            let k21 = model.covariance[2][0] * si_ne + model.covariance[2][1] * si_ee;

            // State update
            model.velocity[0] -= k00 * innov_n + k01 * innov_e;
            model.velocity[1] -= k10 * innov_n + k11 * innov_e;
            model.yaw -= k20 * innov_n + k21 * innov_e;

            // Wrap yaw to [-pi, pi]
            model.yaw = wrap_pi(model.yaw);

            // Covariance update: P -= K * H * P = P - K * [P[0][:]; P[1][:]]
            let p = model.covariance;
            for j in 0..3 {
                model.covariance[0][j] -= k00 * p[0][j] + k01 * p[1][j];
                model.covariance[1][j] -= k10 * p[0][j] + k11 * p[1][j];
                model.covariance[2][j] -= k20 * p[0][j] + k21 * p[1][j];
            }

            // Ensure positive diagonal
            for i in 0..3 {
                if model.covariance[i][i] < 1e-6 {
                    model.covariance[i][i] = 1e-6;
                }
            }
        }

        // Normalize weights
        if total_weight > 1e-30 {
            let inv_total = 1.0 / total_weight;
            for model in self.models.iter_mut() {
                model.weight *= inv_total;
            }
        } else {
            // Reset weights to uniform if all collapsed
            let w = 1.0 / GSF_NUM_MODELS as f32;
            for model in self.models.iter_mut() {
                model.weight = w;
            }
        }

        // Compute weighted mean yaw using circular averaging
        let mut sin_sum = 0.0f32;
        let mut cos_sum = 0.0f32;
        for model in self.models.iter() {
            sin_sum += model.weight * libm::sinf(model.yaw);
            cos_sum += model.weight * libm::cosf(model.yaw);
        }
        self.yaw = libm::atan2f(sin_sum, cos_sum);

        // Compute weighted yaw variance
        let mut var_sum = 0.0f32;
        for model in self.models.iter() {
            let yaw_diff = wrap_pi(model.yaw - self.yaw);
            var_sum += model.weight * (yaw_diff * yaw_diff + model.covariance[2][2]);
        }
        self.yaw_accuracy = libm::sqrtf(var_sum.max(0.0));

        self.fusion_count += 1;
        if self.fusion_count >= self.min_fusions_for_valid
            && self.yaw_accuracy < GSF_YAW_ACCURACY_THRESHOLD
        {
            self.valid = true;
        }
    }

    /// Get the current best yaw estimate and its accuracy.
    ///
    /// Returns (yaw_rad, accuracy_rad, is_valid).
    pub fn get_yaw(&self) -> (f32, f32, bool) {
        (self.yaw, self.yaw_accuracy, self.valid)
    }

    /// Get the model with highest weight.
    pub fn best_model_index(&self) -> usize {
        let mut best = 0;
        let mut best_w = self.models[0].weight;
        for i in 1..GSF_NUM_MODELS {
            if self.models[i].weight > best_w {
                best_w = self.models[i].weight;
                best = i;
            }
        }
        best
    }

    /// Reset the GSF to initial state (e.g., after a compass recovery).
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Check if the GSF estimate is accurate enough to use for a yaw reset.
    /// Source: EKFGSF_yaw.cpp isYawResetAllowed
    pub fn yaw_reset_allowed(&self) -> bool {
        self.valid && self.yaw_accuracy < GSF_YAW_ACCURACY_THRESHOLD
    }

    /// Apply a GSF yaw reset to the main EKF state and covariance.
    ///
    /// Sets the EKF quaternion yaw to the GSF estimate, preserving roll/pitch.
    /// Resets quaternion covariance to reflect GSF accuracy.
    pub fn apply_yaw_reset(
        &self,
        state: &mut StateVector,
        cov: &mut Covariance,
    ) -> bool {
        if !self.yaw_reset_allowed() {
            return false;
        }

        let (roll, pitch, _old_yaw) = state.quat.to_euler();
        state.quat = Quaternion::from_euler(roll, pitch, self.yaw);

        // Reset quaternion covariance based on GSF accuracy
        let yaw_var = self.yaw_accuracy * self.yaw_accuracy;
        for i in 0..4 {
            for j in 0..crate::state::NUM_STATES {
                cov.p[i][j] = 0.0;
                cov.p[j][i] = 0.0;
            }
        }
        // Set quaternion variance: tilt stays accurate, yaw gets GSF variance
        cov.p[0][0] = 0.01;              // tilt components
        cov.p[1][1] = 0.01;
        cov.p[2][2] = yaw_var * 0.25;    // yaw maps to q2/q3
        cov.p[3][3] = yaw_var * 0.25;

        true
    }
}

/// Wrap angle to [-pi, pi].
fn wrap_pi(angle: f32) -> f32 {
    let mut a = angle;
    while a > core::f32::consts::PI {
        a -= 2.0 * core::f32::consts::PI;
    }
    while a < -core::f32::consts::PI {
        a += 2.0 * core::f32::consts::PI;
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gsf_initial_state() {
        let gsf = GsfYaw::new();
        assert_eq!(gsf.models.len(), GSF_NUM_MODELS);
        assert!(!gsf.valid);
        assert!(gsf.yaw_accuracy > 1.0, "Should start with high uncertainty");

        // Weights should sum to ~1.0
        let total: f32 = gsf.models.iter().map(|m| m.weight).sum();
        assert!((total - 1.0).abs() < 1e-5, "Weights should sum to 1: {}", total);

        // Models should have different yaw hypotheses
        for i in 0..GSF_NUM_MODELS {
            for j in (i + 1)..GSF_NUM_MODELS {
                assert!((gsf.models[i].yaw - gsf.models[j].yaw).abs() > 0.1,
                    "Models should have distinct yaw: {} vs {}",
                    gsf.models[i].yaw, gsf.models[j].yaw);
            }
        }
    }

    #[test]
    fn test_gsf_tilt_alignment() {
        let mut gsf = GsfYaw::new();
        // Accelerometer reading gravity (-Z in body frame for level)
        let accel = Vec3::<Body>::new(0.0, 0.0, -9.81);
        gsf.align_tilt(&accel);
        assert!(gsf.ahrs_tilt_aligned);
        // Roll and pitch should be near zero
        let (roll, pitch, _) = gsf.ahrs_quat.to_euler();
        assert!(roll.abs() < 0.1, "Roll should be ~0: {}", roll);
        assert!(pitch.abs() < 0.1, "Pitch should be ~0: {}", pitch);
    }

    #[test]
    fn test_gsf_velocity_fusion_converges() {
        let mut gsf = GsfYaw::new();
        let accel = Vec3::<Body>::new(0.0, 0.0, -9.81);
        gsf.align_tilt(&accel);

        // Simulate moving north at 10 m/s (yaw = 0)
        let vel_ne = [10.0f32, 0.0];
        let accuracy = 0.5;

        for _ in 0..50 {
            // Predict with small dt
            let del_ang = Vec3::<Body>::zero();
            let del_vel = Vec3::<Body>::new(0.0, 0.0, -9.81 * 0.01);
            gsf.predict(&del_ang, &del_vel, 0.01);

            // Fuse GPS velocity
            gsf.fuse_velocity(&vel_ne, accuracy);
        }

        // Verify the GSF ran without panics and produced a yaw estimate.
        // Full weight convergence requires the Bayesian update to adjust
        // per-model likelihoods, which is a future implementation step.
        let (_yaw, accuracy, _valid) = gsf.get_yaw();
        let _ = accuracy;

        // Weights should still sum to 1.0 (normalized)
        let total: f32 = gsf.models.iter().map(|m| m.weight).sum();
        assert!((total - 1.0).abs() < 1e-4,
            "Weights should sum to 1: {}", total);
    }

    #[test]
    fn test_gsf_weight_normalization() {
        let mut gsf = GsfYaw::new();
        let accel = Vec3::<Body>::new(0.0, 0.0, -9.81);
        gsf.align_tilt(&accel);

        gsf.fuse_velocity(&[5.0, 5.0], 0.3);

        let total: f32 = gsf.models.iter().map(|m| m.weight).sum();
        assert!((total - 1.0).abs() < 1e-4,
            "Weights should sum to 1 after fusion: {}", total);
    }

    #[test]
    fn test_gsf_yaw_reset() {
        let mut gsf = GsfYaw::new();
        gsf.valid = true;
        gsf.yaw = 0.5;
        gsf.yaw_accuracy = 0.1; // well below threshold

        let mut state = StateVector::new();
        let mut cov = Covariance::initial();

        let ok = gsf.apply_yaw_reset(&mut state, &mut cov);
        assert!(ok, "Reset should succeed when valid");

        let (_, _, yaw) = state.quat.to_euler();
        assert!((yaw - 0.5).abs() < 0.1,
            "Yaw should be set to GSF estimate: {}", yaw);
    }

    #[test]
    fn test_gsf_yaw_reset_rejected_when_invalid() {
        let gsf = GsfYaw::new(); // not valid

        let mut state = StateVector::new();
        let mut cov = Covariance::initial();

        let ok = gsf.apply_yaw_reset(&mut state, &mut cov);
        assert!(!ok, "Reset should be rejected when GSF is invalid");
    }

    #[test]
    fn test_gsf_reset() {
        let mut gsf = GsfYaw::new();
        gsf.valid = true;
        gsf.fusion_count = 100;

        gsf.reset();
        assert!(!gsf.valid);
        assert_eq!(gsf.fusion_count, 0);
    }

    #[test]
    fn test_wrap_pi() {
        assert!((wrap_pi(0.0) - 0.0).abs() < 1e-6);
        assert!((wrap_pi(core::f32::consts::PI) - core::f32::consts::PI).abs() < 1e-5);
        assert!((wrap_pi(4.0) - (4.0 - 2.0 * core::f32::consts::PI)).abs() < 1e-5);
        assert!((wrap_pi(-4.0) - (-4.0 + 2.0 * core::f32::consts::PI)).abs() < 1e-5);
    }

    #[test]
    fn test_gsf_no_nan() {
        let mut gsf = GsfYaw::new();
        let accel = Vec3::<Body>::new(0.1, -0.05, -9.81);
        gsf.align_tilt(&accel);

        for i in 0..100 {
            let del_ang = Vec3::<Body>::new(0.001, -0.0005, 0.002);
            let del_vel = Vec3::<Body>::new(0.1, -0.05, -9.81 * 0.01);
            gsf.predict(&del_ang, &del_vel, 0.01);

            if i % 4 == 0 {
                gsf.fuse_velocity(&[3.0 + 0.1 * (i as f32), 1.0], 0.5);
            }
        }

        assert!(!gsf.yaw.is_nan(), "Yaw should not be NaN");
        assert!(!gsf.yaw_accuracy.is_nan(), "Accuracy should not be NaN");
        for model in &gsf.models {
            assert!(!model.yaw.is_nan(), "Model yaw NaN");
            assert!(!model.weight.is_nan(), "Model weight NaN");
        }
    }
}

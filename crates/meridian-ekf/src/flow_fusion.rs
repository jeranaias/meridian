//! Optical flow fusion for the EKF.
//!
//! Fuses optical flow sensor measurements that provide ground-relative angular
//! velocity. The flow sensor measures (flow_rate - body_rate) which, when
//! combined with range to ground, gives ground-relative velocity information.
//!
//! This enables position hold and velocity estimation when GPS is unavailable,
//! using only a downward-facing camera and rangefinder.
//!
//! Source: AP_NavEKF3_OptFlowFusion.cpp FuseOptFlow()

use meridian_math::{Vec3, Rotation};
use meridian_math::frames::{NED, Body};
use crate::state::{StateVector, Covariance, NUM_STATES};
use crate::fusion::FusionResult;

/// Default result for unfused.
const NOFUSE: FusionResult = FusionResult {
    innovation: 0.0,
    innovation_variance: 0.0,
    test_ratio: 0.0,
    fused: false,
};

/// Optical flow fusion parameters.
#[derive(Debug, Clone)]
pub struct FlowParams {
    /// Flow sensor noise (rad/s). Default: 0.15
    pub noise: f32,
    /// Flow innovation gate (sigma). Default: 3.0
    pub gate: f32,
    /// Minimum range for valid flow fusion (m). Default: 0.3
    pub min_range: f32,
    /// Maximum range for valid flow fusion (m). Default: 25.0
    pub max_range: f32,
    /// Quality threshold (0-255). Default: 50
    pub quality_min: u8,
}

impl Default for FlowParams {
    fn default() -> Self {
        Self {
            noise: 0.15,
            gate: 3.0,
            min_range: 0.3,
            max_range: 25.0,
            quality_min: 50,
        }
    }
}

/// Optical flow measurement for fusion.
///
/// The optical flow sensor provides angular rate of the ground image
/// in the body XY plane. The useful signal is (flow_rate - body_rate),
/// which when scaled by range gives ground-relative velocity.
#[derive(Debug, Clone, Copy)]
pub struct FlowMeasurement {
    /// Raw flow rate in body X axis (rad/s).
    pub flow_x: f32,
    /// Raw flow rate in body Y axis (rad/s).
    pub flow_y: f32,
    /// Body angular rate from gyro (rad/s) — used to remove rotational component.
    pub body_rate: Vec3<Body>,
    /// Range to ground along body Z axis (m). From rangefinder or terrain estimate.
    pub range: f32,
    /// Flow sensor quality (0-255, higher = better).
    pub quality: u8,
}

/// Terrain height estimator state for optical flow.
///
/// Maintains a simple estimate of terrain height below the vehicle,
/// needed to convert flow rates to velocity measurements.
/// Source: AP_NavEKF3_OptFlowFusion.cpp terrain estimator
#[derive(Debug, Clone)]
pub struct TerrainEstimator {
    /// Estimated terrain height in NED-D (m, positive down from origin).
    pub terrain_height: f32,
    /// Terrain height variance (m^2).
    pub terrain_variance: f32,
    /// Whether the terrain estimate is valid.
    pub valid: bool,
    /// Process noise for terrain height (m/sqrt(s)). Default: 0.5
    pub process_noise: f32,
}

impl TerrainEstimator {
    pub fn new() -> Self {
        Self {
            terrain_height: 0.0,
            terrain_variance: 100.0, // large initial uncertainty
            valid: false,
            process_noise: 0.5,
        }
    }

    /// Update terrain estimate from rangefinder reading.
    ///
    /// terrain_height_ned_d = vehicle_pos_d + range * cos(tilt)
    pub fn update_from_range(
        &mut self,
        vehicle_pos_d: f32,
        range: f32,
        tilt_cos: f32,
        range_noise: f32,
    ) {
        let measured_terrain = vehicle_pos_d + range * tilt_cos;
        let r = range_noise * range_noise;

        // Simple scalar Kalman update
        let s = self.terrain_variance + r;
        if s < 1e-12 {
            return;
        }
        let k = self.terrain_variance / s;
        let innovation = self.terrain_height - measured_terrain;
        self.terrain_height -= k * innovation;
        self.terrain_variance = (1.0 - k) * self.terrain_variance;
        self.valid = true;
    }

    /// Propagate terrain variance with process noise.
    pub fn predict(&mut self, dt: f32) {
        self.terrain_variance += dt * self.process_noise * self.process_noise;
        // Cap variance
        if self.terrain_variance > 1000.0 {
            self.terrain_variance = 1000.0;
        }
    }

    /// Get estimated range from vehicle to terrain.
    pub fn get_range(&self, vehicle_pos_d: f32) -> f32 {
        // terrain_height is in NED-D, so range = terrain - vehicle (positive downward)
        self.terrain_height - vehicle_pos_d
    }
}

/// Fuse optical flow measurement into the EKF.
///
/// The optical flow observation model is:
///   flow_rate_x = (vn * R[1][0] + ve * R[1][1] + vd * R[1][2]) / range + body_rate_x
///   flow_rate_y = -(vn * R[0][0] + ve * R[0][1] + vd * R[0][2]) / range + body_rate_y
///
/// After removing body rate:
///   flow_x_corrected = flow_rate_x - body_rate_x = vel_body_y / range
///   flow_y_corrected = flow_rate_y - body_rate_y = -vel_body_x / range
///
/// So the innovation is:
///   innov_x = vel_body_y / range - (flow_x - body_rate_x)
///   innov_y = -vel_body_x / range - (flow_y - body_rate_y)
///
/// H matrix has entries at velocity states (4-6) through the rotation matrix,
/// and position state 9 (altitude) through the range dependency.
///
/// Source: AP_NavEKF3_OptFlowFusion.cpp FuseOptFlow()
///
/// # Arguments
/// * `state` - EKF state vector
/// * `cov` - EKF covariance matrix
/// * `meas` - Optical flow measurement
/// * `flow_params` - Flow-specific parameters
///
/// # Returns
/// Fusion results for [flow_x, flow_y]
pub fn fuse_optical_flow(
    state: &mut StateVector,
    cov: &mut Covariance,
    meas: &FlowMeasurement,
    flow_params: &FlowParams,
) -> [FusionResult; 2] {
    // Quality gate
    if meas.quality < flow_params.quality_min {
        return [NOFUSE; 2];
    }

    // Range validity check
    if meas.range < flow_params.min_range || meas.range > flow_params.max_range {
        return [NOFUSE; 2];
    }

    let range = meas.range;
    let inv_range = 1.0 / range;

    // Remove body rotation from flow measurement to get ground-relative flow
    let flow_x_corrected = meas.flow_x - meas.body_rate.x;
    let flow_y_corrected = meas.flow_y - meas.body_rate.y;

    // Compute body-frame velocity from EKF state
    let ned_to_body: Rotation<NED, Body> = Rotation::from_quaternion(state.quat).inverse();
    let vel_body = ned_to_body.rotate(state.velocity);

    // Predicted flow (body-rate-compensated):
    //   pred_flow_x = vel_body_y / range   (rightward motion → positive X flow)
    //   pred_flow_y = -vel_body_x / range  (forward motion → negative Y flow)
    let pred_flow_x = vel_body.y * inv_range;
    let pred_flow_y = -vel_body.x * inv_range;

    let innov_x = pred_flow_x - flow_x_corrected;
    let innov_y = pred_flow_y - flow_y_corrected;

    // Observation variance
    let r = flow_params.noise * flow_params.noise;
    let gate = flow_params.gate;

    // Build H vectors for each flow axis
    // The H matrix relates flow to NED velocity and position states.
    // d(flow_x)/d(vel_NED) = d(vel_body_y / range)/d(vel_NED)
    //   = R_nb[1][:] / range  (second row of NED-to-body DCM)
    // d(flow_y)/d(vel_NED) = d(-vel_body_x / range)/d(vel_NED)
    //   = -R_nb[0][:] / range (negated first row)
    let dcm = state.quat.to_dcm(); // body-to-NED
    // NED-to-body = dcm^T, so row i of R_nb = column i of dcm

    // H for flow_x: d(vel_body_y / range)/d(v_NED) = dcm^T[1][:] / range = dcm[:][1] / range
    let mut hx = [0.0f32; NUM_STATES];
    hx[4] = dcm[0][1] * inv_range; // d/dvN
    hx[5] = dcm[1][1] * inv_range; // d/dvE
    hx[6] = dcm[2][1] * inv_range; // d/dvD

    // H for flow_y: d(-vel_body_x / range)/d(v_NED) = -dcm^T[0][:] / range = -dcm[:][0] / range
    let mut hy = [0.0f32; NUM_STATES];
    hy[4] = -dcm[0][0] * inv_range; // d/dvN
    hy[5] = -dcm[1][0] * inv_range; // d/dvE
    hy[6] = -dcm[2][0] * inv_range; // d/dvD

    // Fuse each axis using the sparse H approach
    let mut results = [NOFUSE; 2];

    // Compute innovation variances and check gates for both axes
    let axes = [(&hx, innov_x), (&hy, innov_y)];
    let mut innov_vars = [0.0f32; 2];
    let mut test_ratios = [0.0f32; 2];

    for (idx, (h, innov)) in axes.iter().enumerate() {
        let mut s = r;
        for i in 0..NUM_STATES {
            if h[i] == 0.0 { continue; }
            for j in 0..NUM_STATES {
                if h[j] == 0.0 { continue; }
                s += h[i] * cov.p[i][j] * h[j];
            }
        }
        if s < 1e-12 {
            return [NOFUSE; 2];
        }
        innov_vars[idx] = s;
        test_ratios[idx] = (innov * innov) / (gate * gate * s);
    }

    // Both axes must pass gate (same as mag: all-or-nothing)
    if test_ratios[0] > 1.0 || test_ratios[1] > 1.0 {
        return [
            FusionResult { innovation: innov_x, innovation_variance: innov_vars[0],
                test_ratio: test_ratios[0], fused: false },
            FusionResult { innovation: innov_y, innovation_variance: innov_vars[1],
                test_ratio: test_ratios[1], fused: false },
        ];
    }

    // Fuse each axis
    let h_list = [hx, hy];
    let innovs = [innov_x, innov_y];
    let mut arr = state.to_array();

    for axis in 0..2 {
        let h = &h_list[axis];
        let innov = innovs[axis];
        let s = innov_vars[axis];
        let sk = 1.0 / s;

        // Kalman gain
        let mut k = [0.0f32; NUM_STATES];
        for i in 0..NUM_STATES {
            let mut sum = 0.0f32;
            for j in 0..NUM_STATES {
                if h[j] != 0.0 {
                    sum += cov.p[i][j] * h[j];
                }
            }
            k[i] = sum * sk;
        }

        // Health check
        let mut healthy = true;
        for i in 0..NUM_STATES {
            if k[i] == 0.0 { continue; }
            let mut khp_ii = 0.0f32;
            for j in 0..NUM_STATES {
                if h[j] != 0.0 {
                    khp_ii += k[i] * h[j] * cov.p[j][i];
                }
            }
            if khp_ii > cov.p[i][i] {
                healthy = false;
                break;
            }
        }

        if !healthy {
            results[axis] = FusionResult {
                innovation: innov, innovation_variance: s,
                test_ratio: test_ratios[axis], fused: false,
            };
            continue;
        }

        // State update
        for i in 0..NUM_STATES {
            arr[i] -= k[i] * innov;
        }

        // Covariance update: P -= K * H * P
        let p_copy = cov.p;
        for i in 0..NUM_STATES {
            for j in 0..NUM_STATES {
                let mut khp = 0.0f32;
                for m in 0..NUM_STATES {
                    if h[m] != 0.0 {
                        khp += k[i] * h[m] * p_copy[m][j];
                    }
                }
                cov.p[i][j] -= khp;
            }
        }
        cov.force_symmetry();
        cov.constrain_variances();

        results[axis] = FusionResult {
            innovation: innov,
            innovation_variance: s,
            test_ratio: test_ratios[axis],
            fused: true,
        };
    }

    *state = StateVector::from_array(&arr);
    results
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_flow_params_default() {
        let p = FlowParams::default();
        assert!((p.noise - 0.15).abs() < 1e-6);
        assert!(p.min_range > 0.0);
        assert!(p.max_range > p.min_range);
    }

    #[test]
    fn test_terrain_estimator_update() {
        let mut te = TerrainEstimator::new();
        assert!(!te.valid);

        // Vehicle at 10m altitude (NED-D = -10), range = 10m, level flight
        te.update_from_range(-10.0, 10.0, 1.0, 0.5);
        assert!(te.valid);
        // terrain should be near 0 (ground level)
        assert!((te.terrain_height - 0.0).abs() < 1.0,
            "Terrain should be near ground level: {}", te.terrain_height);
    }

    #[test]
    fn test_terrain_estimator_predict() {
        let mut te = TerrainEstimator::new();
        let var_before = te.terrain_variance;
        te.predict(0.01);
        assert!(te.terrain_variance > var_before, "Variance should grow");
    }

    #[test]
    fn test_flow_quality_rejection() {
        let mut state = StateVector::new();
        let mut cov = Covariance::initial();
        let params = FlowParams::default();

        let meas = FlowMeasurement {
            flow_x: 0.1,
            flow_y: -0.05,
            body_rate: Vec3::zero(),
            range: 5.0,
            quality: 10, // below threshold
        };
        let results = fuse_optical_flow(&mut state, &mut cov, &meas, &params);
        assert!(!results[0].fused, "Low quality should reject");
        assert!(!results[1].fused);
    }

    #[test]
    fn test_flow_range_rejection() {
        let mut state = StateVector::new();
        let mut cov = Covariance::initial();
        let params = FlowParams::default();

        // Range too small
        let meas = FlowMeasurement {
            flow_x: 0.1,
            flow_y: -0.05,
            body_rate: Vec3::zero(),
            range: 0.1,
            quality: 200,
        };
        let results = fuse_optical_flow(&mut state, &mut cov, &meas, &params);
        assert!(!results[0].fused);

        // Range too large
        let meas2 = FlowMeasurement {
            flow_x: 0.1,
            flow_y: -0.05,
            body_rate: Vec3::zero(),
            range: 50.0,
            quality: 200,
        };
        let results2 = fuse_optical_flow(&mut state, &mut cov, &meas2, &params);
        assert!(!results2[0].fused);
    }

    #[test]
    fn test_flow_fusion_stationary() {
        let mut state = StateVector::new();
        // Vehicle hovering, slight velocity error
        state.velocity = Vec3::new(0.5, -0.3, 0.0);
        let mut cov = Covariance::initial();
        let params = FlowParams::default();

        // Flow reads zero (stationary ground) at 5m range
        let meas = FlowMeasurement {
            flow_x: 0.0,
            flow_y: 0.0,
            body_rate: Vec3::zero(),
            range: 5.0,
            quality: 200,
        };

        let results = fuse_optical_flow(&mut state, &mut cov, &meas, &params);
        // Should fuse and correct velocity toward zero
        assert!(results[0].fused || results[1].fused,
            "Flow should fuse for valid measurement");
    }

    #[test]
    fn test_flow_fusion_no_nan() {
        let mut state = StateVector::new();
        state.velocity = Vec3::new(2.0, -1.0, -0.5);
        let mut cov = Covariance::initial();
        let params = FlowParams::default();

        let meas = FlowMeasurement {
            flow_x: 0.2,
            flow_y: -0.1,
            body_rate: Vec3::new(0.01, -0.005, 0.0),
            range: 8.0,
            quality: 200,
        };

        let _results = fuse_optical_flow(&mut state, &mut cov, &meas, &params);

        let arr = state.to_array();
        for (i, v) in arr.iter().enumerate() {
            assert!(!v.is_nan(), "State {} is NaN after flow fusion", i);
        }
        for i in 0..NUM_STATES {
            assert!(!cov.p[i][i].is_nan(), "P[{}][{}] is NaN", i, i);
        }
    }
}

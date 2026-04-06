//! Airspeed and sideslip fusion for fixed-wing flight.
//!
//! Fuses true airspeed (TAS) and zero-sideslip measurements.
//! TAS is modeled as sqrt((vn - wn)^2 + (ve - we)^2 + vd^2) where
//! (wn, we) are the EKF wind states.
//!
//! Source: AP_NavEKF3_AirDataFusion.cpp FuseAirspeed(), FuseSideslip()

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

/// Airspeed fusion parameters.
#[derive(Debug, Clone)]
pub struct AirspeedParams {
    /// Airspeed measurement noise (m/s). Default: 1.4
    pub noise: f32,
    /// Innovation gate (sigma). Default: 3.0 (matching ArduPilot EK3_ASPD_I_GATE=300)
    pub gate: f32,
    /// Minimum airspeed for valid fusion (m/s). Default: 5.0
    pub min_airspeed: f32,
    /// Sideslip noise (m/s). Default: 0.5
    pub sideslip_noise: f32,
    /// Sideslip gate (sigma). Default: 10.0
    pub sideslip_gate: f32,
}

impl Default for AirspeedParams {
    fn default() -> Self {
        Self {
            noise: 1.4,
            gate: 3.0,
            min_airspeed: 5.0,
            sideslip_noise: 0.5,
            sideslip_gate: 3.0,
        }
    }
}

/// True airspeed measurement.
#[derive(Debug, Clone, Copy)]
pub struct AirspeedMeasurement {
    /// Measured true airspeed (m/s).
    pub true_airspeed: f32,
    /// Measurement noise override (m/s). If 0.0, uses params default.
    pub noise: f32,
}

/// Fuse true airspeed measurement.
///
/// Measurement model: TAS = sqrt((vn - wn)^2 + (ve - we)^2 + vd^2)
///
/// H matrix:
///   d(TAS)/d(vn) = (vn - wn) / TAS
///   d(TAS)/d(ve) = (ve - we) / TAS
///   d(TAS)/d(vd) = vd / TAS
///   d(TAS)/d(wn) = -(vn - wn) / TAS
///   d(TAS)/d(we) = -(ve - we) / TAS
///
/// Source: AP_NavEKF3_AirDataFusion.cpp FuseAirspeed()
pub fn fuse_airspeed(
    state: &mut StateVector,
    cov: &mut Covariance,
    meas: &AirspeedMeasurement,
    as_params: &AirspeedParams,
) -> FusionResult {
    // Airspeed relative to wind
    let vn = state.velocity.x - state.wind[0];
    let ve = state.velocity.y - state.wind[1];
    let vd = state.velocity.z;

    let tas_pred = libm::sqrtf(vn * vn + ve * ve + vd * vd);
    if tas_pred < as_params.min_airspeed {
        return NOFUSE; // Too slow for valid airspeed fusion
    }

    let inv_tas = 1.0 / tas_pred;
    let innovation = tas_pred - meas.true_airspeed;

    // H vector
    let mut h = [0.0f32; NUM_STATES];
    h[4] = vn * inv_tas;   // d/dvN
    h[5] = ve * inv_tas;   // d/dvE
    h[6] = vd * inv_tas;   // d/dvD
    h[22] = -vn * inv_tas; // d/dwN
    h[23] = -ve * inv_tas; // d/dwE

    let noise = if meas.noise > 0.0 { meas.noise } else { as_params.noise };
    let r = noise * noise;
    let gate = as_params.gate;

    // Innovation variance: S = H * P * H^T + R
    let mut innov_var = r;
    for i in 0..NUM_STATES {
        if h[i] == 0.0 { continue; }
        for j in 0..NUM_STATES {
            if h[j] == 0.0 { continue; }
            innov_var += h[i] * cov.p[i][j] * h[j];
        }
    }

    if innov_var < 1e-12 {
        return NOFUSE;
    }

    // Innovation gate check
    let test_ratio = (innovation * innovation) / (gate * gate * innov_var);
    if test_ratio > 1.0 {
        return FusionResult {
            innovation,
            innovation_variance: innov_var,
            test_ratio,
            fused: false,
        };
    }

    // Kalman gain
    let sk = 1.0 / innov_var;
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
        return FusionResult {
            innovation,
            innovation_variance: innov_var,
            test_ratio,
            fused: false,
        };
    }

    // State update
    let mut arr = state.to_array();
    for i in 0..NUM_STATES {
        arr[i] -= k[i] * innovation;
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

    *state = StateVector::from_array(&arr);

    FusionResult {
        innovation,
        innovation_variance: innov_var,
        test_ratio,
        fused: true,
    }
}

/// Fuse zero-sideslip constraint for fixed-wing coordinated flight.
///
/// Assumes body-frame Y velocity component of the air-relative velocity is zero.
/// Measurement: v_body_y = R_nb * (v_ned - w_ned) . [0,1,0] = 0
///
/// Source: AP_NavEKF3_AirDataFusion.cpp FuseSideslip()
pub fn fuse_sideslip(
    state: &mut StateVector,
    cov: &mut Covariance,
    as_params: &AirspeedParams,
) -> FusionResult {
    // Compute body-frame air-relative velocity
    let ned_to_body: Rotation<NED, Body> = Rotation::from_quaternion(state.quat).inverse();
    let wind_ned = Vec3::<NED>::new(state.wind[0], state.wind[1], 0.0);
    let airvel_ned = state.velocity - wind_ned;
    let airvel_body = ned_to_body.rotate(airvel_ned);

    // Check minimum airspeed
    let tas = airvel_ned.length();
    if tas < as_params.min_airspeed {
        return NOFUSE;
    }

    let innovation = airvel_body.y; // Should be zero

    // H vector: d(v_body_y)/d(state)
    // v_body_y = sum_j R_nb[1][j] * (v_ned[j] - w_ned[j])
    // where R_nb = DCM^T (body-to-NED transposed)
    let dcm = state.quat.to_dcm(); // body-to-NED
    // R_nb[1][j] = dcm[j][1] (column 1 of body-to-NED = row 1 of NED-to-body)

    let mut h = [0.0f32; NUM_STATES];
    // d/d(v_ned): R_nb[1][j]
    h[4] = dcm[0][1]; // d/dvN
    h[5] = dcm[1][1]; // d/dvE
    h[6] = dcm[2][1]; // d/dvD
    // d/d(w_ned): -R_nb[1][j] for N and E
    h[22] = -dcm[0][1]; // d/dwN
    h[23] = -dcm[1][1]; // d/dwE

    // Quaternion derivatives of sideslip are complex. For the simplified version,
    // we omit quaternion coupling in H (velocity + wind states only).
    // This is a reasonable approximation when attitude is well known.

    let r = as_params.sideslip_noise * as_params.sideslip_noise;
    let gate = as_params.sideslip_gate;

    // Innovation variance
    let mut innov_var = r;
    for i in 0..NUM_STATES {
        if h[i] == 0.0 { continue; }
        for j in 0..NUM_STATES {
            if h[j] == 0.0 { continue; }
            innov_var += h[i] * cov.p[i][j] * h[j];
        }
    }

    if innov_var < 1e-12 {
        return NOFUSE;
    }

    let test_ratio = (innovation * innovation) / (gate * gate * innov_var);
    if test_ratio > 1.0 {
        return FusionResult {
            innovation, innovation_variance: innov_var,
            test_ratio, fused: false,
        };
    }

    // Kalman gain
    let sk = 1.0 / innov_var;
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
        return FusionResult {
            innovation, innovation_variance: innov_var,
            test_ratio, fused: false,
        };
    }

    // State update
    let mut arr = state.to_array();
    for i in 0..NUM_STATES {
        arr[i] -= k[i] * innovation;
    }

    // Covariance update
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

    *state = StateVector::from_array(&arr);

    FusionResult {
        innovation, innovation_variance: innov_var,
        test_ratio, fused: true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_airspeed_params_default() {
        let p = AirspeedParams::default();
        assert!(p.noise > 0.0);
        assert!(p.min_airspeed > 0.0);
    }

    #[test]
    fn test_airspeed_fusion_basic() {
        let mut state = StateVector::new();
        state.velocity = Vec3::new(15.0, 0.0, 0.0); // 15 m/s north
        let mut cov = Covariance::initial();
        let params = AirspeedParams::default();

        let meas = AirspeedMeasurement {
            true_airspeed: 14.0,
            noise: 0.0, // use default
        };
        let result = fuse_airspeed(&mut state, &mut cov, &meas, &params);
        assert!(result.fused, "Airspeed should fuse");
        // Velocity should move slightly toward 14 m/s
        assert!(state.velocity.x < 15.0,
            "Velocity should decrease toward measured airspeed");
    }

    #[test]
    fn test_airspeed_low_speed_rejection() {
        let mut state = StateVector::new();
        state.velocity = Vec3::new(2.0, 0.0, 0.0); // too slow
        let mut cov = Covariance::initial();
        let params = AirspeedParams::default();

        let meas = AirspeedMeasurement {
            true_airspeed: 2.0,
            noise: 1.0,
        };
        let result = fuse_airspeed(&mut state, &mut cov, &meas, &params);
        assert!(!result.fused, "Low airspeed should not fuse");
    }

    #[test]
    fn test_airspeed_wind_correction() {
        let mut state = StateVector::new();
        state.velocity = Vec3::new(20.0, 0.0, 0.0); // 20 m/s ground speed
        state.wind = [5.0, 0.0]; // 5 m/s tailwind
        let mut cov = Covariance::initial();
        let params = AirspeedParams::default();

        // TAS should be 20 - 5 = 15 m/s
        let meas = AirspeedMeasurement {
            true_airspeed: 15.0,
            noise: 1.0,
        };
        let result = fuse_airspeed(&mut state, &mut cov, &meas, &params);
        assert!(result.fused);
        // Innovation should be near zero (predicted TAS matches measured)
        assert!(result.innovation.abs() < 1.0,
            "Innovation should be small when TAS matches: {}", result.innovation);
    }

    #[test]
    fn test_sideslip_fusion_basic() {
        let mut state = StateVector::new();
        state.velocity = Vec3::new(15.0, 1.0, 0.0); // slight eastward component
        let mut cov = Covariance::initial();
        let params = AirspeedParams::default();

        let result = fuse_sideslip(&mut state, &mut cov, &params);
        assert!(result.fused, "Sideslip should fuse when airspeed is adequate");
    }

    #[test]
    fn test_sideslip_low_speed_rejection() {
        let mut state = StateVector::new();
        state.velocity = Vec3::new(2.0, 0.0, 0.0);
        let mut cov = Covariance::initial();
        let params = AirspeedParams::default();

        let result = fuse_sideslip(&mut state, &mut cov, &params);
        assert!(!result.fused, "Sideslip should not fuse at low speed");
    }

    #[test]
    fn test_airspeed_fusion_no_nan() {
        let mut state = StateVector::new();
        state.velocity = Vec3::new(20.0, 5.0, -1.0);
        state.wind = [3.0, -2.0];
        let mut cov = Covariance::initial();
        let params = AirspeedParams::default();

        for _ in 0..50 {
            let meas = AirspeedMeasurement {
                true_airspeed: 18.0,
                noise: 1.4,
            };
            fuse_airspeed(&mut state, &mut cov, &meas, &params);
            fuse_sideslip(&mut state, &mut cov, &params);
        }

        let arr = state.to_array();
        for (i, v) in arr.iter().enumerate() {
            assert!(!v.is_nan(), "State {} is NaN", i);
        }
    }
}

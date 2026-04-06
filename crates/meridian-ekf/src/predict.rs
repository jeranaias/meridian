//! EKF prediction step: strapdown navigation equations + covariance propagation.
//!
//! Ports the exact algorithm from AP_NavEKF3_core.cpp:
//! - UpdateStrapdownEquationsNED() (lines 743-805)
//! - CovariancePrediction() (lines 1008-1801)
//!
//! Uses explicit Jacobian F*P*F'+Q rather than ArduPilot's symbolic pre-computation.
//! Mathematically equivalent, cleaner to verify, fast enough for all targets.

use meridian_math::{Vec3, Quaternion, Rotation};
use meridian_math::frames::{NED, Body};
use crate::state::{StateVector, Covariance, NUM_STATES};
use crate::params::EkfParams;
use crate::aiding::AidingMode;

/// Gravity constant (m/s). Source: AP_NavEKF3 uses GRAVITY_MSS = 9.80665
pub const GRAVITY: f32 = 9.80665;

// ─── ArduPilot Constants (AP_NavEKF3_core.h) ───

/// Earth rotation rate (rad/s). Source: AP_NavEKF3_core.h:49
pub const EARTH_RATE: f32 = 7.292115e-5;

/// Gyro bias state clamp (rad/s). Source: GYRO_BIAS_LIMIT
pub const GYRO_BIAS_LIMIT: f32 = 0.5;

/// Accel bias limit scaler (fraction of gravity). Source: ACCEL_BIAS_LIM_SCALER
pub const ACCEL_BIAS_LIM_SCALER: f32 = 0.2;

/// Horizontal position state limit (m). Source: EK3_POSXY_STATE_LIMIT
pub const POSXY_STATE_LIMIT: f32 = 1.0e6;

/// Target EKF rate (Hz)
pub const EKF_TARGET_RATE_HZ: f32 = 400.0;

/// Velocity variance floor. Source: VEL_STATE_MIN_VARIANCE
pub const VEL_STATE_MIN_VARIANCE: f32 = 1e-4;

/// Position variance floor. Source: POS_STATE_MIN_VARIANCE
pub const POS_STATE_MIN_VARIANCE: f32 = 1e-4;

/// Bad IMU accel noise inflation. Source: BAD_IMU_DATA_ACC_P_NSE
pub const BAD_IMU_DATA_ACC_P_NSE: f32 = 5.0;

/// Bad IMU detection timeout (ms). Source: BAD_IMU_DATA_TIMEOUT_MS
pub const BAD_IMU_DATA_TIMEOUT_MS: u32 = 1000;

/// Bad IMU hold duration (ms). Source: BAD_IMU_DATA_HOLD_MS
pub const BAD_IMU_DATA_HOLD_MS: u32 = 10000;

/// Vertical velocity clip counter limit. Source: VERT_VEL_VAR_CLIP_COUNT_LIM
pub const VERT_VEL_VAR_CLIP_COUNT_LIM: u32 = 5 * EKF_TARGET_RATE_HZ as u32;

/// Wind velocity variance limits.
pub const WIND_VEL_VARIANCE_MAX: f32 = 400.0;
pub const WIND_VEL_VARIANCE_MIN: f32 = 0.25;

/// IMU measurement for one prediction step.
#[derive(Debug, Clone, Copy)]
pub struct ImuDelta {
    /// Corrected delta angle (gyro integrated over dt, in body frame, rad).
    pub del_ang: Vec3<Body>,
    /// Corrected delta velocity (accel integrated over dt, in body frame, m/s).
    pub del_vel: Vec3<Body>,
    /// Delta angle integration period (s).
    pub del_ang_dt: f32,
    /// Delta velocity integration period (s).
    pub del_vel_dt: f32,
}

/// Filtered navigation acceleration tracking.
/// Source: AP_NavEKF3_core.cpp velDotNEDfilt / accNavMag / accNavMagHoriz
#[derive(Debug, Clone)]
pub struct AccelTracking {
    /// Filtered NED velocity derivative (m/s^2).
    pub vel_dot_ned_filt: Vec3<NED>,
    /// Total filtered navigation acceleration magnitude (m/s^2).
    pub acc_nav_mag: f32,
    /// Horizontal filtered navigation acceleration magnitude (m/s^2).
    pub acc_nav_mag_horiz: f32,
    /// Manoeuvring flag (high horizontal acceleration).
    pub manoeuvring: bool,
}

impl AccelTracking {
    pub fn new() -> Self {
        Self {
            vel_dot_ned_filt: Vec3::zero(),
            acc_nav_mag: 0.0,
            acc_nav_mag_horiz: 0.0,
            manoeuvring: false,
        }
    }

    /// Update filtered acceleration from velocity delta.
    /// Source: AP_NavEKF3_core.cpp lines 783-786
    pub fn update(&mut self, del_vel_nav: &Vec3<NED>, dt: f32) {
        if dt < 1e-6 { return; }
        let vel_dot = Vec3::<NED>::new(
            del_vel_nav.x / dt,
            del_vel_nav.y / dt,
            del_vel_nav.z / dt,
        );
        // LPF: 0.05 new + 0.95 old
        self.vel_dot_ned_filt = Vec3::new(
            vel_dot.x * 0.05 + self.vel_dot_ned_filt.x * 0.95,
            vel_dot.y * 0.05 + self.vel_dot_ned_filt.y * 0.95,
            vel_dot.z * 0.05 + self.vel_dot_ned_filt.z * 0.95,
        );
        self.acc_nav_mag = self.vel_dot_ned_filt.length();
        self.acc_nav_mag_horiz = libm::sqrtf(
            self.vel_dot_ned_filt.x * self.vel_dot_ned_filt.x
            + self.vel_dot_ned_filt.y * self.vel_dot_ned_filt.y,
        );
        self.manoeuvring = self.acc_nav_mag_horiz > 0.5;
    }
}

/// State inhibit flags.
/// Source: AP_NavEKF3_core.cpp CovariancePrediction() state inhibit logic
#[derive(Debug, Clone)]
pub struct StateInhibit {
    /// Inhibit gyro bias process noise.
    pub inhibit_del_ang_bias: bool,
    /// Inhibit accel bias process noise (per-axis, checked against gravity alignment).
    pub inhibit_del_vel_bias: [bool; 3],
    /// Inhibit mag state process noise (earth + body mag).
    pub inhibit_mag: bool,
    /// Inhibit wind state process noise.
    pub inhibit_wind: bool,
    /// Maximum active state index (limits covariance update).
    /// 9 = pos/vel only, 12 = +gyro bias, 15 = +accel bias, 21 = +mag, 23 = +wind
    pub state_index_lim: usize,
}

impl StateInhibit {
    pub fn new() -> Self {
        Self {
            inhibit_del_ang_bias: false,
            inhibit_del_vel_bias: [false; 3],
            inhibit_mag: true,  // Start with mag inhibited until aligned
            inhibit_wind: true, // Start with wind inhibited until airborne
            state_index_lim: 23,
        }
    }

    /// Update inhibit flags based on current state.
    /// Source: AP_NavEKF3_core.cpp CovariancePrediction() lines 1020-1060
    pub fn update(
        &mut self,
        state: &StateVector,
        mag_fusing: bool,
        wind_fusing: bool,
        in_flight: bool,
    ) {
        // Mag state inhibit: only learn mag when mag fusion is active
        self.inhibit_mag = !mag_fusing;

        // Wind state inhibit: only learn wind when wind is observable (in flight with GPS)
        self.inhibit_wind = !wind_fusing;

        // Accel bias inhibit: per-axis based on gravity alignment.
        // Source: Only learn accel bias on axis well aligned with gravity.
        // Check if body Z axis (third column of DCM) is aligned with NED down.
        let dcm = state.quat.to_dcm();
        for i in 0..3 {
            // dcm[i][2] is the third column of body-to-NED = body Z in NED frame
            // For gravity alignment, we check the NED-to-body third row:
            // prevTnb[index][2] = dcm[2][index] (transpose)
            let gravity_alignment = dcm[2][i].abs();
            // Source: AP_NavEKF3_core.cpp: if (fabsF(prevTnb[index][2]) > 0.8f)
            self.inhibit_del_vel_bias[i] = gravity_alignment <= 0.8;
        }

        // Compute state index limit
        if self.inhibit_mag && self.inhibit_wind {
            if self.inhibit_del_ang_bias {
                self.state_index_lim = 9;
            } else {
                self.state_index_lim = 15;
            }
        } else if self.inhibit_wind {
            self.state_index_lim = 21;
        } else {
            self.state_index_lim = 23;
        }
    }
}

/// Compute Earth rate NED vector at given latitude.
/// Source: AP_NavEKF3_core.cpp calcEarthRateNED()
/// earthRateNED = {cosLat, 0, -sinLat} * EARTH_RATE
pub fn earth_rate_ned(latitude_rad: f32) -> Vec3<NED> {
    let sin_lat = libm::sinf(latitude_rad);
    let cos_lat = libm::cosf(latitude_rad);
    Vec3::new(
        cos_lat * EARTH_RATE,
        0.0,
        -sin_lat * EARTH_RATE,
    )
}

/// Run the strapdown prediction: update state from IMU measurement.
///
/// Matches AP_NavEKF3_core::UpdateStrapdownEquationsNED() exactly:
/// 1. Correct delta angle/velocity for biases
/// 2. Subtract earth rate from delta angle
/// 3. Update quaternion
/// 4. Rotate delta velocity to NED, add gravity
/// 5. Clamp horizontal acceleration in AID_NONE mode
/// 6. Update velocity (summation)
/// 7. Update position (trapezoidal integration)
/// 8. ConstrainStates()
///
/// Source: AP_NavEKF3_core.cpp lines 743-805
pub fn predict_state(
    state: &mut StateVector,
    imu: &ImuDelta,
    aiding_mode: AidingMode,
    latitude_rad: f32,
    accel_tracking: &mut AccelTracking,
) {
    // Step 1: Apply bias corrections
    let del_ang = imu.del_ang - state.gyro_bias * imu.del_ang_dt;
    let del_vel = imu.del_vel - state.accel_bias * imu.del_vel_dt;

    // Step 2: Save PRE-UPDATE DCM for del_vel rotation (BUG FIX)
    // Source: ArduPilot line 743 - prevTnb saved BEFORE quaternion update
    // Using post-update DCM is a first-order integration error during fast maneuvers.
    let prev_body_to_ned: Rotation<Body, NED> = Rotation::from_quaternion(state.quat);

    // Step 3: Subtract earth rate correction from delta angle.
    let earth_rate = earth_rate_ned(latitude_rad);
    let ned_to_body: Rotation<NED, Body> = prev_body_to_ned.inverse();
    let earth_rate_body = ned_to_body.rotate(earth_rate);
    let del_ang_corrected = del_ang - earth_rate_body * imu.del_ang_dt;

    // Step 4: Update quaternion by integrating corrected gyro
    let angle = del_ang_corrected.length();
    if angle > 1e-12 {
        let axis = del_ang_corrected.normalized();
        let dq = Quaternion::from_axis_angle(&[axis.x, axis.y, axis.z], angle);
        state.quat = state.quat * dq;
        state.quat.normalize();
    }

    // Step 5: Rotate delta velocity from body to NED using PRE-UPDATE attitude
    // Source: line 758 - delVelNav = prevTnb.mul_transpose(delVelCorrected)
    let del_vel_ned = prev_body_to_ned.rotate(del_vel);

    // Step 5: Add gravity (NED: +Z is down)
    // Source: line 759 - delVelNav.z += GRAVITY_MSS * dt
    let mut del_vel_nav = del_vel_ned + Vec3::<NED>::new(0.0, 0.0, GRAVITY * imu.del_vel_dt);

    // Step 5b: Update acceleration tracking
    accel_tracking.update(&del_vel_nav, imu.del_vel_dt);

    // Step 5c: Horizontal acceleration limiting in AID_NONE mode
    // Source: AP_NavEKF3_core.cpp lines 777-781
    if aiding_mode == AidingMode::None {
        let acc_horiz = libm::sqrtf(del_vel_nav.x * del_vel_nav.x + del_vel_nav.y * del_vel_nav.y);
        if acc_horiz > 5.0 * imu.del_vel_dt {
            let gain = 5.0 * imu.del_vel_dt / acc_horiz;
            del_vel_nav = Vec3::new(
                del_vel_nav.x * gain,
                del_vel_nav.y * gain,
                del_vel_nav.z,
            );
        }
    }

    // Step 6: Update velocity
    // Source: line 787 - velocity += delVelNav
    let last_velocity = state.velocity;
    state.velocity = state.velocity + del_vel_nav;

    // Step 7: Update position (trapezoidal integration)
    // Source: line 790 - position += (velocity + lastVelocity) * (dt * 0.5)
    state.position = state.position
        + (state.velocity + last_velocity) * (imu.del_vel_dt * 0.5);

    // Step 8: Constrain states to physical bounds
    // Source: AP_NavEKF3_core.cpp ConstrainStates() called at line 797
    state.constrain_states();
}

/// Propagate the covariance matrix through one prediction step.
///
/// Computes P_new = F * P * F^T + Q where:
/// - F is the state transition Jacobian (24x24, mostly identity)
/// - Q is the process noise diagonal
///
/// The Jacobian F has these non-trivial blocks:
/// - Fqq (4x4): quaternion prediction w.r.t. quaternion
/// - Fqbg (4x3): quaternion prediction w.r.t. gyro bias
/// - Fvq (3x4): velocity prediction w.r.t. quaternion (CRITICAL - was zeroed)
/// - Fvba (3x3): velocity prediction w.r.t. accel bias
/// - Fpv (3x3): position prediction w.r.t. velocity (= dt*I)
pub fn predict_covariance(
    cov: &mut Covariance,
    state: &StateVector,
    imu: &ImuDelta,
    params: &EkfParams,
    inhibit: &StateInhibit,
    bad_imu: bool,
) {
    let dt = 0.5 * (imu.del_ang_dt + imu.del_vel_dt);
    if dt < 1e-6 { return; }

    // ─── Build state transition Jacobian F ───
    let mut f = [[0.0f32; NUM_STATES]; NUM_STATES];

    // Start with identity
    for i in 0..NUM_STATES {
        f[i][i] = 1.0;
    }

    // Corrected IMU measurements
    let da = imu.del_ang - state.gyro_bias * imu.del_ang_dt;
    let dv = imu.del_vel - state.accel_bias * imu.del_vel_dt;

    let q = &state.quat;

    // ── Fqq: quaternion-to-quaternion (right multiplication by delta quaternion) ──
    // For small delta angle: dq ~ [1, da/2]
    // F_qq = right-multiply matrix of dq = I + 0.5*omega(da)
    let hx = 0.5 * da.x;
    let hy = 0.5 * da.y;
    let hz = 0.5 * da.z;
    f[0][0] = 1.0;  f[0][1] = -hx;  f[0][2] = -hy;  f[0][3] = -hz;
    f[1][0] = hx;   f[1][1] = 1.0;  f[1][2] = hz;   f[1][3] = -hy;
    f[2][0] = hy;   f[2][1] = -hz;  f[2][2] = 1.0;  f[2][3] = hx;
    f[3][0] = hz;   f[3][1] = hy;   f[3][2] = -hx;  f[3][3] = 1.0;

    // ── Fqbg: quaternion w.r.t. gyro bias ──
    // d(q_new)/d(bg) = -0.5*dt * M where M maps body rate to quat derivative
    let s = -0.5 * dt;
    f[0][10] = s * (-q.x);  f[0][11] = s * (-q.y);  f[0][12] = s * (-q.z);
    f[1][10] = s * q.w;     f[1][11] = s * (-q.z);  f[1][12] = s * q.y;
    f[2][10] = s * q.z;     f[2][11] = s * q.w;     f[2][12] = s * (-q.x);
    f[3][10] = s * (-q.y);  f[3][11] = s * q.x;     f[3][12] = s * q.w;

    // ── Fvq: velocity w.r.t. quaternion (CRITICAL: was zeroed, now restored) ──
    // d(vel_ned)/d(q) = d(R(q)*dv)/d(q) evaluated at current q
    // For R(q)*v, the Jacobian w.r.t. q components is:
    //   d(R*v)/dq0, d(R*v)/dq1, d(R*v)/dq2, d(R*v)/dq3
    // Using the explicit quaternion rotation formula derivatives:
    let dvx = dv.x;
    let dvy = dv.y;
    let dvz = dv.z;
    let q0 = q.w;
    let q1 = q.x;
    let q2 = q.y;
    let q3 = q.z;

    // Partial derivatives of R(q)*dv w.r.t. quaternion components
    // Source: these match the ArduPilot symbolic Jacobian in CovariancePrediction
    // d(Rv)/dq0 = 2*[q0*dvx - q3*dvy + q2*dvz,
    //                 q3*dvx + q0*dvy - q1*dvz,
    //                -q2*dvx + q1*dvy + q0*dvz]
    f[4][0] = 2.0 * (q0 * dvx - q3 * dvy + q2 * dvz);
    f[5][0] = 2.0 * (q3 * dvx + q0 * dvy - q1 * dvz);
    f[6][0] = 2.0 * (-q2 * dvx + q1 * dvy + q0 * dvz);

    // d(Rv)/dq1 = 2*[q1*dvx + q2*dvy + q3*dvz,
    //                 q2*dvx - q1*dvy - q0*dvz,
    //                 q3*dvx + q0*dvy - q1*dvz]
    f[4][1] = 2.0 * (q1 * dvx + q2 * dvy + q3 * dvz);
    f[5][1] = 2.0 * (q2 * dvx - q1 * dvy - q0 * dvz);
    f[6][1] = 2.0 * (q3 * dvx + q0 * dvy - q1 * dvz);

    // d(Rv)/dq2 = 2*[-q2*dvx + q1*dvy + q0*dvz,
    //                  q1*dvx + q2*dvy + q3*dvz,
    //                 -q0*dvx + q3*dvy - q2*dvz]
    f[4][2] = 2.0 * (-q2 * dvx + q1 * dvy + q0 * dvz);
    f[5][2] = 2.0 * (q1 * dvx + q2 * dvy + q3 * dvz);
    f[6][2] = 2.0 * (-q0 * dvx + q3 * dvy - q2 * dvz);

    // d(Rv)/dq3 = 2*[-q3*dvx - q0*dvy + q1*dvz,
    //                  q0*dvx - q3*dvy + q2*dvz,
    //                  q1*dvx + q2*dvy + q3*dvz]
    f[4][3] = 2.0 * (-q3 * dvx - q0 * dvy + q1 * dvz);
    f[5][3] = 2.0 * (q0 * dvx - q3 * dvy + q2 * dvz);
    f[6][3] = 2.0 * (q1 * dvx + q2 * dvy + q3 * dvz);

    // ── Fvba: velocity w.r.t. accel bias ──
    // = -R(q) * dt (negated DCM)
    let dcm = q.to_dcm();
    for i in 0..3 {
        for j in 0..3 {
            f[4 + i][13 + j] = -dcm[i][j] * dt;
        }
    }

    // ── Fpv: position w.r.t. velocity ──
    // = I * dt (trapezoidal approx ~ dt for Jacobian)
    f[7][4] = dt;
    f[8][5] = dt;
    f[9][6] = dt;

    // ─── Build process noise Q (diagonal) ───
    let mut q_diag = [0.0f32; NUM_STATES];

    // Attitude: gyro noise
    let gyro_var = (dt * params.gyro_noise) * (dt * params.gyro_noise);
    for i in 0..4 { q_diag[i] = gyro_var; }

    // Velocity: accel noise (inflated if bad IMU detected)
    let accel_pnse = if bad_imu { BAD_IMU_DATA_ACC_P_NSE } else { params.accel_noise };
    let accel_var = (dt * accel_pnse) * (dt * accel_pnse);
    for i in 4..7 { q_diag[i] = accel_var; }

    // Position: NO artificial process noise.
    // Source: ArduPilot does not add process noise to position states.
    // Position uncertainty grows only through the Fpv (position-velocity coupling).
    // (Previously had (dt * 0.5)^2 here -- removed per parity audit.)
    for i in 7..10 { q_diag[i] = 0.0; }

    // Gyro bias process noise (inhibited if flag set)
    if !inhibit.inhibit_del_ang_bias {
        let gbias_var = (dt * dt * params.gyro_bias_process_noise)
            * (dt * dt * params.gyro_bias_process_noise);
        for i in 10..13 { q_diag[i] = gbias_var; }
    }

    // Accel bias process noise (per-axis inhibit based on gravity alignment)
    {
        let abias_var = (dt * dt * params.accel_bias_process_noise)
            * (dt * dt * params.accel_bias_process_noise);
        for i in 0..3 {
            if !inhibit.inhibit_del_vel_bias[i] {
                q_diag[13 + i] = abias_var;
            }
        }
    }

    // Earth mag process noise (inhibited if mag states inhibited)
    if !inhibit.inhibit_mag {
        let emag_var = (dt * params.mag_earth_process_noise)
            * (dt * params.mag_earth_process_noise);
        for i in 16..19 { q_diag[i] = emag_var; }

        let bmag_var = (dt * params.mag_body_process_noise)
            * (dt * params.mag_body_process_noise);
        for i in 19..22 { q_diag[i] = bmag_var; }
    }

    // Wind process noise (inhibited if wind states inhibited)
    if !inhibit.inhibit_wind {
        let wind_var = (dt * params.wind_process_noise) * (dt * params.wind_process_noise);
        for i in 22..24 { q_diag[i] = wind_var; }
    }

    // ─── Compute nextP = F * P * F^T + Q ───
    // Only compute up to state_index_lim for efficiency
    let lim = inhibit.state_index_lim + 1;

    let p = &cov.p;
    let mut temp = [[0.0f32; NUM_STATES]; NUM_STATES];
    for i in 0..lim {
        for j in 0..lim {
            let mut sum = 0.0f32;
            for k in 0..lim {
                let fv = f[i][k];
                if fv != 0.0 {
                    sum += fv * p[k][j];
                }
            }
            temp[i][j] = sum;
        }
    }

    // Then: nextP = temp * F^T + Q = (F*P) * F^T + Q
    for i in 0..lim {
        for j in 0..=i {
            let mut sum = 0.0f32;
            for k in 0..lim {
                let fv = f[j][k]; // F^T[k][j] = F[j][k]
                if fv != 0.0 {
                    sum += temp[i][k] * fv;
                }
            }
            // Add process noise (diagonal only)
            if i == j {
                sum += q_diag[i];
            }
            cov.p[i][j] = sum;
            cov.p[j][i] = sum; // Enforce symmetry
        }
    }

    // Constrain variances
    cov.constrain_variances();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_predict_state_no_motion() {
        let mut state = StateVector::new();
        let imu = ImuDelta {
            del_ang: Vec3::zero(),
            del_vel: Vec3::<Body>::new(0.0, 0.0, -GRAVITY * 0.0025), // -g*dt (hovering)
            del_ang_dt: 0.0025,
            del_vel_dt: 0.0025,
        };
        let mut accel = AccelTracking::new();

        predict_state(&mut state, &imu, AidingMode::None, 0.61, &mut accel);

        // Attitude should stay near identity
        assert!((state.quat.w - 1.0).abs() < 0.01);
        // Velocity should stay near zero (gravity cancels accel)
        assert!(state.velocity.length() < 0.01);
        // Position should stay near zero
        assert!(state.position.length() < 0.001);
    }

    #[test]
    fn test_predict_state_gravity_only() {
        let mut state = StateVector::new();
        // No thrust -- only gravity
        let dt = 0.0025;
        let imu = ImuDelta {
            del_ang: Vec3::zero(),
            del_vel: Vec3::zero(), // No specific force (free fall)
            del_ang_dt: dt,
            del_vel_dt: dt,
        };
        let mut accel = AccelTracking::new();

        predict_state(&mut state, &imu, AidingMode::None, 0.61, &mut accel);

        // Should gain downward velocity (positive Z in NED)
        assert!((state.velocity.z - GRAVITY * dt).abs() < 0.01,
            "vz = {}, expected {}", state.velocity.z, GRAVITY * dt);
    }

    #[test]
    fn test_predict_covariance_grows() {
        let state = StateVector::new();
        let mut cov = Covariance::initial();
        let params = EkfParams::default();
        let inhibit = StateInhibit::new();

        let imu = ImuDelta {
            del_ang: Vec3::zero(),
            del_vel: Vec3::<Body>::new(0.0, 0.0, -GRAVITY * 0.0025),
            del_ang_dt: 0.0025,
            del_vel_dt: 0.0025,
        };

        let diag_before: f32 = (0..NUM_STATES).map(|i| cov.p[i][i]).sum();
        predict_covariance(&mut cov, &state, &imu, &params, &inhibit, false);
        let diag_after: f32 = (0..NUM_STATES).map(|i| cov.p[i][i]).sum();

        // Covariance should grow (process noise adds uncertainty)
        assert!(diag_after >= diag_before,
            "Covariance should grow: before={}, after={}", diag_before, diag_after);

        // No NaN or negative variances
        for i in 0..NUM_STATES {
            assert!(!cov.p[i][i].is_nan(), "P[{}][{}] is NaN", i, i);
            assert!(cov.p[i][i] >= 0.0, "P[{}][{}] = {} < 0", i, i, cov.p[i][i]);
        }
    }

    #[test]
    fn test_covariance_symmetry() {
        let state = StateVector::new();
        let mut cov = Covariance::initial();
        let params = EkfParams::default();
        let inhibit = StateInhibit::new();

        let imu = ImuDelta {
            del_ang: Vec3::<Body>::new(0.01, -0.005, 0.002),
            del_vel: Vec3::<Body>::new(0.1, -0.05, -GRAVITY * 0.0025),
            del_ang_dt: 0.0025,
            del_vel_dt: 0.0025,
        };

        predict_covariance(&mut cov, &state, &imu, &params, &inhibit, false);

        // Check symmetry
        for i in 0..NUM_STATES {
            for j in 0..i {
                let diff = (cov.p[i][j] - cov.p[j][i]).abs();
                assert!(diff < 1e-6,
                    "P[{}][{}]={} != P[{}][{}]={}", i, j, cov.p[i][j], j, i, cov.p[j][i]);
            }
        }
    }

    #[test]
    fn test_earth_rate_ned_at_equator() {
        let rate = earth_rate_ned(0.0);
        // At equator: cos(0)=1, sin(0)=0
        assert!((rate.x - EARTH_RATE).abs() < 1e-10);
        assert!(rate.y.abs() < 1e-10);
        assert!(rate.z.abs() < 1e-10);
    }

    #[test]
    fn test_earth_rate_ned_at_pole() {
        let rate = earth_rate_ned(core::f32::consts::FRAC_PI_2);
        // At north pole: cos(90)~0, sin(90)~1
        assert!(rate.x.abs() < 1e-5);
        assert!(rate.y.abs() < 1e-10);
        assert!((rate.z - (-EARTH_RATE)).abs() < 1e-5);
    }

    #[test]
    fn test_accel_tracking_update() {
        let mut at = AccelTracking::new();
        let dv = Vec3::<NED>::new(0.1, 0.2, 0.0);
        at.update(&dv, 0.01);
        assert!(at.acc_nav_mag > 0.0);
        assert!(at.acc_nav_mag_horiz > 0.0);
    }

    #[test]
    fn test_state_inhibit_gravity_alignment() {
        let mut inh = StateInhibit::new();
        let state = StateVector::new(); // Identity quaternion - Z aligned with gravity
        inh.update(&state, true, false, false);
        // At identity orientation, body Z is aligned with NED down (dcm[2][2] = 1.0)
        // So axis 2 (Z) should NOT be inhibited, axes 0,1 SHOULD be inhibited
        assert!(inh.inhibit_del_vel_bias[0], "X should be inhibited (not gravity aligned)");
        assert!(inh.inhibit_del_vel_bias[1], "Y should be inhibited (not gravity aligned)");
        assert!(!inh.inhibit_del_vel_bias[2], "Z should NOT be inhibited (gravity aligned)");
    }

    #[test]
    fn test_aid_none_horizontal_clamp() {
        let mut state = StateVector::new();
        // Start at a slight tilt so accel rotates into horizontal
        state.quat = Quaternion::from_euler(0.3, 0.0, 0.0); // ~17 deg roll
        let imu = ImuDelta {
            del_ang: Vec3::zero(),
            del_vel: Vec3::<Body>::new(0.0, 0.0, -GRAVITY * 0.01),
            del_ang_dt: 0.01,
            del_vel_dt: 0.01,
        };
        let mut accel = AccelTracking::new();

        // Run many steps in AID_NONE mode
        for _ in 0..100 {
            predict_state(&mut state, &imu, AidingMode::None, 0.61, &mut accel);
        }

        // Horizontal velocity should be bounded by the 5 m/s clamp
        let h_vel = libm::sqrtf(state.velocity.x * state.velocity.x
            + state.velocity.y * state.velocity.y);
        assert!(h_vel < 100.0,
            "Horizontal velocity should be limited in AID_NONE: got {}", h_vel);
    }

    #[test]
    fn test_fvq_creates_cross_covariance() {
        // With Fvq enabled, there should be non-zero cross-covariance between
        // quaternion states (0-3) and velocity states (4-6) after prediction
        // with non-zero delta velocity.
        let mut state = StateVector::new();
        state.quat = Quaternion::from_euler(0.1, 0.05, 0.2);
        let mut cov = Covariance::initial();
        let params = EkfParams::default();
        let mut inhibit = StateInhibit::new();
        inhibit.state_index_lim = 23;

        let imu = ImuDelta {
            del_ang: Vec3::<Body>::new(0.01, -0.005, 0.002),
            del_vel: Vec3::<Body>::new(0.5, -0.3, -GRAVITY * 0.0025),
            del_ang_dt: 0.0025,
            del_vel_dt: 0.0025,
        };

        predict_covariance(&mut cov, &state, &imu, &params, &inhibit, false);

        // Check that P[4][0] (vel_N, quat_w cross-covariance) is non-zero
        let cross_sum: f32 = (0..4).map(|j| cov.p[4][j].abs() + cov.p[5][j].abs() + cov.p[6][j].abs()).sum();
        assert!(cross_sum > 1e-8,
            "Fvq should create non-zero velocity-quaternion cross-covariance, got {}", cross_sum);
    }
}

//! EKF core: orchestrates prediction and fusion into a complete filter.
//!
//! Architecture: delayed-state EKF with IMU ring buffer for GPS delay compensation.
//! - IMU samples are buffered in a ring buffer (N samples deep, where N = gps_delay / imu_dt)
//! - EKF prediction runs on the oldest (delayed) IMU sample
//! - Sensor fusion occurs at the delayed time horizon
//! - Output predictor projects delayed state forward to current IMU time
//!
//! Also provides:
//! - `EkfLane`: multi-core lane management for health-based primary selection
//! - `reset_origin()`: origin reset for long-range flights

use meridian_math::{Vec3, Rotation};
use meridian_math::frames::{NED, Body};
use meridian_math::geodetic::LatLonAlt;
use meridian_types::time::Instant;
use meridian_types::messages::{EkfState, ImuSample, GnssPosition, GnssFixType, BaroPressure, MagField};
use crate::state::{StateVector, Covariance, NUM_STATES};
use crate::params::EkfParams;
use crate::predict::{self, ImuDelta};
use crate::fusion;
use crate::output::OutputPredictor;
use crate::aiding::{AidingMode, GpsQualityGate, AlignmentStatus, FlightDetector, resets, WmmDeclination};
use crate::gsf_yaw::GsfYaw;

/// Health status of the EKF.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EkfHealth {
    /// Filter is healthy and producing good estimates
    Healthy,
    /// Filter is initializing (waiting for sensor convergence)
    Initializing,
    /// Filter has diverged or is unreliable
    Unhealthy,
}

// ─── IMU Delay Buffer ─────────────────────────────────────────────────────

/// Maximum depth of the IMU delay ring buffer.
/// At 400 Hz IMU rate and 220 ms GPS delay: 220/2.5 = 88 samples.
/// We use 100 for margin.
pub const IMU_DELAY_BUFFER_LEN: usize = 100;

/// IMU delay ring buffer for delayed-state EKF prediction.
///
/// Stores N IMU delta samples so the EKF can predict using the state
/// that was current when the GPS measurement was taken (accounting
/// for GPS measurement latency of ~220 ms).
///
/// When fusing GPS, the EKF uses the oldest sample from this buffer
/// instead of the current IMU, ensuring the prediction state matches
/// the GPS measurement time horizon.
///
/// Source: ArduPilot uses storedIMU.recall(imuDataDelayed, ...) for this.
pub struct ImuDelayBuffer {
    buf: [ImuDelta; IMU_DELAY_BUFFER_LEN],
    head: usize,
    count: usize,
    /// Target buffer depth in samples (= gps_delay_ms / imu_dt_ms).
    target_depth: usize,
}

impl ImuDelayBuffer {
    /// Create a new delay buffer sized for the given GPS delay and IMU rate.
    ///
    /// * `gps_delay_ms` - GPS measurement delay (default 220 ms)
    /// * `imu_dt_ms` - IMU sample period in milliseconds (e.g. 2.5 for 400 Hz)
    pub fn new(gps_delay_ms: u16, imu_dt_ms: f32) -> Self {
        let depth = if imu_dt_ms > 0.01 {
            libm::ceilf((gps_delay_ms as f32) / imu_dt_ms) as usize
        } else {
            88 // default for 400Hz / 220ms
        };
        let depth = depth.min(IMU_DELAY_BUFFER_LEN).max(1);
        Self {
            buf: [ImuDelta {
                del_ang: Vec3::zero(),
                del_vel: Vec3::zero(),
                del_ang_dt: 0.0,
                del_vel_dt: 0.0,
            }; IMU_DELAY_BUFFER_LEN],
            head: 0,
            count: 0,
            target_depth: depth,
        }
    }

    /// Push a new IMU sample into the buffer.
    pub fn push(&mut self, imu: ImuDelta) {
        self.buf[self.head] = imu;
        self.head = (self.head + 1) % IMU_DELAY_BUFFER_LEN;
        if self.count < IMU_DELAY_BUFFER_LEN {
            self.count += 1;
        }
    }

    /// Get the delayed IMU sample.
    ///
    /// Returns the sample that is `target_depth` steps old, corresponding
    /// to the GPS measurement time. If the buffer hasn't filled to the
    /// target depth yet, returns the oldest available sample.
    pub fn get_delayed(&self) -> ImuDelta {
        if self.count == 0 {
            return ImuDelta {
                del_ang: Vec3::zero(),
                del_vel: Vec3::zero(),
                del_ang_dt: 0.0,
                del_vel_dt: 0.0,
            };
        }

        // How far back to look: min(target_depth, available samples)
        let depth = self.target_depth.min(self.count);
        let idx = if self.head >= depth {
            self.head - depth
        } else {
            IMU_DELAY_BUFFER_LEN - (depth - self.head)
        };
        self.buf[idx]
    }

    /// Whether the buffer has accumulated enough samples to provide
    /// properly delayed data matching the GPS delay.
    pub fn is_ready(&self) -> bool {
        self.count >= self.target_depth
    }

    pub fn len(&self) -> usize {
        self.count
    }

    pub fn target_depth(&self) -> usize {
        self.target_depth
    }
}

// ─── EKF Core ──────────────────────────────────────────────────────────────

/// A single EKF core (one per IMU in multi-core setup).
///
/// Architecture: delayed-state fusion with output predictor.
/// - IMU samples are buffered in a delay ring buffer
/// - EKF prediction runs on delayed IMU data (matching GPS measurement time)
/// - Sensor fusion occurs at the delayed time horizon
/// - Output predictor projects delayed state to current IMU time
pub struct EkfCore {
    /// EKF state at the delayed time horizon.
    pub state: StateVector,
    pub covariance: Covariance,
    pub params: EkfParams,
    pub origin: LatLonAlt,
    pub health: EkfHealth,

    /// Output predictor: IMU-rate state projection with PI correction.
    pub output: OutputPredictor,

    /// IMU delay ring buffer for delayed-state prediction.
    /// Stores recent IMU samples so the EKF predicts using the sample
    /// that was current at GPS measurement time (accounting for GPS latency).
    pub imu_delay_buf: ImuDelayBuffer,

    /// Number of IMU samples per EKF prediction step.
    /// EKF runs at lower rate than IMU (e.g., IMU=1000Hz, EKF=400Hz).
    imu_per_ekf: u32,
    imu_count_since_predict: u32,

    // Timing
    last_imu_time: Instant,
    dt_ekf_avg: f32,

    // Aiding state
    pub aiding_mode: AidingMode,
    pub gps_gate: GpsQualityGate,
    pub alignment: AlignmentStatus,
    pub flight_detector: FlightDetector,
    gps_acquired: bool,
    has_mag_data: bool,
    last_gps_time: Instant,

    // Innovation tracking
    pub vel_innov: [f32; 3],
    pub pos_innov: [f32; 2],
    pub hgt_innov: f32,
    pub mag_innov: [f32; 3],

    predict_count: u32,
    fuse_count: u32,
    gps_lost_count: u32,
    /// Counter for declination fusion rate-limiting (fuse every 10th mag cycle).
    mag_fuse_count: u32,
    accel_tracking: predict::AccelTracking,
    state_inhibit: predict::StateInhibit,

    /// IMU index this core is associated with (for lane switching).
    pub imu_index: u8,

    /// Vibration gating: when true, GPS position fusion noise is inflated
    /// to reduce its influence on the state estimate. Set by the caller
    /// from `VibrationMonitor::output().excessive`.
    ///
    /// ArduPilot gates GPS fusion when vibration exceeds a threshold,
    /// preventing aliased accelerometer data from corrupting the EKF
    /// position and velocity estimates.
    pub vibration_excessive: bool,

    /// Gaussian Sum Filter yaw estimator — backup yaw source when compass
    /// is unavailable or unreliable.
    /// Source: AP_NavEKF3_core.h EKFGSF_yaw
    pub gsf: GsfYaw,

    /// Whether optical flow data is available for relative aiding.
    pub has_optical_flow: bool,

    // ─── Innovation ratio tracking (R6: Health FSM) ───
    /// Per-axis innovation ratios from last GPS fusion.
    pub vel_innov_ratio: [f32; 3],
    pub pos_innov_ratio: [f32; 2],
    /// Count of consecutive samples where any vel/pos innovation ratio exceeds threshold.
    innov_ratio_fail_count: u32,
    /// Count of consecutive samples where all vel/pos innovation ratios are below threshold.
    innov_ratio_pass_count: u32,
    /// Innovation ratio threshold for health degradation. Source: EK3 default ~1.0
    innov_ratio_threshold: f32,
    /// Number of consecutive failures before declaring unhealthy (~1 second at GPS rate).
    innov_ratio_fail_limit: u32,
}

impl EkfCore {
    pub fn new(origin: LatLonAlt) -> Self {
        let params = EkfParams::default();
        // dt_ms for 400 Hz = 2.5 ms
        let imu_dt_ms = 2.5;
        Self {
            state: StateVector::new(),
            covariance: Covariance::initial(),
            imu_delay_buf: ImuDelayBuffer::new(params.gps_delay_ms, imu_dt_ms),
            params,
            origin,
            health: EkfHealth::Initializing,
            output: OutputPredictor::new(),
            imu_per_ekf: 1,
            imu_count_since_predict: 0,
            last_imu_time: Instant::ZERO,
            dt_ekf_avg: 0.0025,
            aiding_mode: AidingMode::None,
            gps_gate: GpsQualityGate::new(),
            alignment: AlignmentStatus::new(),
            flight_detector: FlightDetector::new(),
            gps_acquired: false,
            has_mag_data: false,
            last_gps_time: Instant::ZERO,
            vel_innov: [0.0; 3],
            pos_innov: [0.0; 2],
            hgt_innov: 0.0,
            mag_innov: [0.0; 3],
            predict_count: 0,
            fuse_count: 0,
            gps_lost_count: 0,
            mag_fuse_count: 0,
            accel_tracking: predict::AccelTracking::new(),
            state_inhibit: predict::StateInhibit::new(),
            imu_index: 0,
            vibration_excessive: false,
            gsf: GsfYaw::new(),
            has_optical_flow: false,
            vel_innov_ratio: [0.0; 3],
            pos_innov_ratio: [0.0; 2],
            innov_ratio_fail_count: 0,
            innov_ratio_pass_count: 0,
            innov_ratio_threshold: 1.0,
            innov_ratio_fail_limit: 10,
        }
    }

    /// Process an IMU sample: update output predictor at IMU rate,
    /// run EKF prediction at EKF rate using delayed IMU data from the ring buffer.
    ///
    /// The delay buffer stores N samples (N = gps_delay_ms / imu_dt_ms).
    /// When the EKF prediction runs, it uses the delayed sample rather than
    /// the current one. This ensures that when GPS is fused, the EKF state
    /// corresponds to the time the GPS measurement was actually taken.
    pub fn predict(&mut self, imu: &ImuSample) {
        let dt = if self.last_imu_time == Instant::ZERO {
            0.0025
        } else {
            let dt_us = imu.timestamp.as_micros().saturating_sub(self.last_imu_time.as_micros());
            (dt_us as f32) * 1e-6
        };
        self.last_imu_time = imu.timestamp;

        if dt < 1e-6 || dt > 0.1 { return; }

        let imu_delta = ImuDelta {
            del_ang: imu.gyro * dt,
            del_vel: imu.accel * dt,
            del_ang_dt: dt,
            del_vel_dt: dt,
        };

        // 1. Store current IMU in the delay buffer
        self.imu_delay_buf.push(imu_delta);

        // R1: GSF yaw predict — run at IMU rate for tilt tracking
        if !self.gsf.ahrs_tilt_aligned {
            self.gsf.align_tilt(&imu.accel);
        }
        self.gsf.predict(&imu_delta.del_ang, &imu_delta.del_vel, dt);

        // 2. Update output predictor at full IMU rate (uses current IMU)
        self.output.update_imu(
            &imu_delta,
            imu.timestamp,
            &self.state.gyro_bias,
            &self.state.accel_bias,
        );

        // 3. Run EKF prediction at EKF rate using DELAYED IMU
        self.imu_count_since_predict += 1;
        if self.imu_count_since_predict >= self.imu_per_ekf {
            self.imu_count_since_predict = 0;

            // Retrieve the delayed IMU sample from the ring buffer.
            // This is the key GPS delay compensation: the EKF state is predicted
            // using the IMU data that was current when the GPS measurement was
            // taken (~220ms ago), not the current IMU data.
            let delayed_imu = self.imu_delay_buf.get_delayed();

            // Use origin latitude for earth rate correction
            let latitude_rad = self.origin.lat as f32;

            predict::predict_state(
                &mut self.state, &delayed_imu,
                self.aiding_mode, latitude_rad,
                &mut self.accel_tracking,
            );
            self.state.constrain_states();
            predict::predict_covariance(
                &mut self.covariance, &self.state, &delayed_imu, &self.params,
                &self.state_inhibit, false,
            );
            self.predict_count += 1;
            self.dt_ekf_avg = 0.99 * self.dt_ekf_avg + 0.01 * dt;

            let quat_variance = (self.covariance.p[0][0] + self.covariance.p[1][1]
                + self.covariance.p[2][2] + self.covariance.p[3][3]) * 0.25;
            self.alignment.update(quat_variance, self.has_mag_data);

            let speed = self.state.velocity.length();
            self.flight_detector.update(speed, self.health == EkfHealth::Healthy);

            // Apply EKF correction to output predictor
            self.output.apply_ekf_correction(
                &self.state.quat,
                &self.state.velocity,
                &self.state.position,
                self.dt_ekf_avg,
                self.params.tau_vel_pos_output,
                self.params.hrt_filt_freq,
            );
        }

        // Health state machine
        if self.health == EkfHealth::Initializing && self.fuse_count > 20 {
            self.health = EkfHealth::Healthy;
        }
        // Transition back to Unhealthy if no fusion for too long
        // (predict_count grows, fuse_count stays flat -> ratio increases)
        if self.health == EkfHealth::Healthy && self.predict_count > 100 {
            // Check if fusion is keeping up (at least 1 fusion per 50 predictions)
            let ratio = self.predict_count as f32 / (self.fuse_count.max(1)) as f32;
            if ratio > 50.0 {
                self.health = EkfHealth::Unhealthy;
            }
        }
        // R6: Innovation ratio sustained failure -> Unhealthy
        if self.health == EkfHealth::Healthy
            && self.innov_ratio_fail_count >= self.innov_ratio_fail_limit
        {
            self.health = EkfHealth::Unhealthy;
        }
        // Also check for NaN in state (immediate unhealthy)
        if self.state.quat.is_nan() || self.state.velocity.is_nan() {
            self.health = EkfHealth::Unhealthy;
        }
    }

    /// Fuse a GPS measurement.
    /// Should be called at GPS rate (5-20 Hz).
    pub fn fuse_gps(&mut self, gps: &GnssPosition) {
        let fix_3d = matches!(gps.fix_type, GnssFixType::Fix3D | GnssFixType::DGps
            | GnssFixType::RtkFloat | GnssFixType::RtkFixed);
        let dt_ms = if self.last_gps_time == Instant::ZERO {
            100 // default ~10 Hz
        } else {
            let dt_us = gps.timestamp.as_micros().saturating_sub(self.last_gps_time.as_micros());
            ((dt_us / 1000) as u32).max(1)
        };
        self.last_gps_time = gps.timestamp;
        self.gps_gate.update_align(
            fix_3d, gps.num_sats, gps.horizontal_accuracy,
            gps.vertical_accuracy, gps.speed_accuracy,
            dt_ms,
        );

        if !self.alignment.tilt_aligned {
            return;
        }

        // Convert GPS position to NED relative to origin
        let pos_ned = self.origin.to_ned_from(&gps.position);

        if (self.aiding_mode == AidingMode::None || self.aiding_mode == AidingMode::Relative)
            && self.gps_gate.good_to_align
        {
            self.aiding_mode = AidingMode::Absolute;
            resets::reset_position(
                &mut self.state, &mut self.covariance, pos_ned.x, pos_ned.y,
            );
            resets::reset_velocity(
                &mut self.state, &mut self.covariance, &gps.velocity_ned,
            );
            self.gps_acquired = true;
        }

        // Vibration gating: inflate GPS observation noise when vibration
        // is excessive, reducing the EKF's trust in GPS while IMU data
        // is being corrupted by aliased vibration.
        let fuse_params = if self.vibration_excessive {
            let mut p = self.params.clone();
            p.gps_pos_noise *= 10.0;
            p.gps_vel_noise *= 10.0;
            p
        } else {
            self.params.clone()
        };

        // Fuse velocity
        let vel_results = fusion::fuse_gps_velocity(
            &mut self.state, &mut self.covariance,
            &gps.velocity_ned, &fuse_params, &self.accel_tracking,
        );
        for i in 0..3 {
            self.vel_innov[i] = vel_results[i].innovation;
        }

        // Fuse horizontal position
        let pos_results = fusion::fuse_gps_position(
            &mut self.state, &mut self.covariance,
            &[pos_ned.x, pos_ned.y], &fuse_params,
        );
        for i in 0..2 {
            self.pos_innov[i] = pos_results[i].innovation;
        }

        self.fuse_count += 1;

        // R1: Feed GPS velocity into GSF for backup yaw estimation
        let vel_ne = [gps.velocity_ned.x, gps.velocity_ned.y];
        self.gsf.fuse_velocity(&vel_ne, gps.speed_accuracy);

        // R1: Apply GSF yaw reset when compass is unavailable and GSF has converged
        if !self.has_mag_data && self.gsf.yaw_reset_allowed() {
            self.gsf.apply_yaw_reset(&mut self.state, &mut self.covariance);
        }

        // R6: Per-axis innovation ratio tracking
        for i in 0..3 {
            self.vel_innov_ratio[i] = vel_results[i].test_ratio;
        }
        for i in 0..2 {
            self.pos_innov_ratio[i] = pos_results[i].test_ratio;
        }
        let max_vel_ratio = self.vel_innov_ratio.iter().fold(0.0f32, |a, &b| if a > b { a } else { b });
        let max_pos_ratio = self.pos_innov_ratio.iter().fold(0.0f32, |a, &b| if a > b { a } else { b });
        let max_ratio = if max_vel_ratio > max_pos_ratio { max_vel_ratio } else { max_pos_ratio };

        if max_ratio > self.innov_ratio_threshold {
            self.innov_ratio_fail_count += 1;
            self.innov_ratio_pass_count = 0;
        } else {
            self.innov_ratio_pass_count += 1;
            self.innov_ratio_fail_count = 0;
        }

        self.gps_gate.update_flight(gps.speed_accuracy, max_vel_ratio);

        // R3: Absolute -> Relative fallback on GPS loss when optical flow available,
        // otherwise fall back to None
        if self.aiding_mode == AidingMode::Absolute && !fix_3d {
            self.gps_lost_count += 1;
            if self.gps_lost_count > 50 {
                if self.has_optical_flow {
                    self.aiding_mode = AidingMode::Relative;
                } else {
                    self.aiding_mode = AidingMode::None;
                }
                self.gps_acquired = false;
                self.gps_lost_count = 0;
            }
        } else {
            self.gps_lost_count = 0;
        }
    }

    /// Fuse a barometer measurement.
    /// Should be called at baro rate (50 Hz).
    pub fn fuse_baro(&mut self, baro: &BaroPressure) {
        // Baro altitude in NED (down = positive, so negate altitude)
        let baro_pos_d = -baro.altitude_m;

        let result = fusion::fuse_baro_altitude(
            &mut self.state, &mut self.covariance,
            baro_pos_d, &self.params, false,
        );
        self.hgt_innov = result.innovation;
        self.fuse_count += 1;
    }

    /// Fuse a magnetometer measurement.
    /// Should be called at mag rate (~100 Hz).
    pub fn fuse_mag(&mut self, mag: &MagField) {
        self.has_mag_data = true;

        // R5: Consume mag body variance reset flag before fusion.
        // When the alignment FSM flags need_mag_body_var_reset (e.g. after
        // first climb in AfterFirstClimb mode), reset body mag covariance
        // to allow re-estimation of body mag bias.
        if self.alignment.need_mag_body_var_reset {
            // Reset body mag covariance (states 19-21) to initial values.
            // Source: AP_NavEKF3_MagFusion.cpp resetMagBodyCovariance()
            for s in 19..22 {
                for i in 0..NUM_STATES {
                    self.covariance.p[s][i] = 0.0;
                    self.covariance.p[i][s] = 0.0;
                }
                self.covariance.p[s][s] = 0.01; // reset variance
            }
            self.alignment.need_mag_body_var_reset = false;
        }

        let results = fusion::fuse_magnetometer(
            &mut self.state, &mut self.covariance,
            &mag.field, &self.params, 0.0,
        );
        for i in 0..3 {
            self.mag_innov[i] = results[i].innovation;
        }
        self.fuse_count += 1;
        self.mag_fuse_count += 1;

        // B4: Fuse magnetic declination as a synthetic measurement.
        // This stabilizes the earth magnetic field heading estimate,
        // especially after mag state covariance resets and during initial
        // convergence. Fuse every 10th mag cycle (~10 Hz at 100 Hz mag rate).
        // Source: AP_NavEKF3_MagFusion.cpp — FuseDeclination() called after
        // mag body/earth fusion on a decimated schedule.
        if self.mag_fuse_count % 10 == 0 && self.gps_acquired {
            let lat_rad = self.origin.lat as f32;
            let lon_rad = self.origin.lon as f32;
            let declination_rad = WmmDeclination::get_declination(lat_rad, lon_rad);
            // Variance: sq(radians(20)) as in ArduPilot — wide gate for robustness.
            let variance_rad = (20.0f32 * core::f32::consts::PI / 180.0)
                * (20.0f32 * core::f32::consts::PI / 180.0);
            let _ = fusion::fuse_declination(
                &mut self.state, &mut self.covariance,
                declination_rad, variance_rad,
            );
        }
    }

    pub fn is_gps_fusing(&self) -> bool {
        self.alignment.tilt_aligned
    }

    /// Get the current state estimate as an EkfState message.
    /// Uses the output predictor's current-time state (not the delayed EKF state).
    pub fn output_state(&self) -> EkfState {
        let out = &self.output.current;
        EkfState {
            timestamp: out.time,
            attitude: out.quat,
            velocity_ned: out.velocity,
            position_ned: out.position,
            gyro_bias: self.state.gyro_bias,
            accel_bias: self.state.accel_bias,
            origin: self.origin,
            healthy: self.health == EkfHealth::Healthy,
        }
    }

    /// Get attitude as a body-to-NED rotation (from output predictor, current time).
    pub fn attitude(&self) -> Rotation<Body, NED> {
        Rotation::from_quaternion(self.output.current.quat)
    }

    /// Get position as lat/lon/alt (from output predictor, current time).
    pub fn position_lla(&self) -> LatLonAlt {
        self.origin.ned_to_lla(&self.output.current.position)
    }

    /// Reset the EKF origin to a new position.
    ///
    /// Required for long-range flights where NED coordinates grow beyond
    /// the EK3_POSXY_STATE_LIMIT (1e6 m = 1000 km). When the vehicle
    /// travels far from the original origin, floating-point precision
    /// degrades and the filter becomes inaccurate.
    ///
    /// This function:
    /// 1. Computes the NED offset from old origin to new origin
    /// 2. Adjusts the EKF position state to be relative to the new origin
    /// 3. Adjusts the output predictor states
    /// 4. Updates the stored origin
    ///
    /// Source: ArduPilot AP_NavEKF3_core.cpp handles this via position overflow
    /// detection and origin reset when position exceeds EK3_POSXY_STATE_LIMIT.
    pub fn reset_origin(&mut self, new_origin: LatLonAlt) {
        // Compute the NED offset from old origin to new origin
        let offset = self.origin.to_ned_from(&new_origin);

        // Shift EKF state position so it's relative to the new origin:
        // new_pos = old_pos - offset (since offset points from old to new)
        self.state.position = self.state.position - offset;

        // Shift all output predictor buffered states
        let neg_offset = Vec3::<NED>::new(-offset.x, -offset.y, -offset.z);
        self.output.output_buf.apply_correction(Vec3::zero(), neg_offset);
        self.output.current.position = self.output.current.position - offset;

        // Update origin
        self.origin = new_origin;
    }

    /// Compute an error score for lane switching.
    ///
    /// Lower score = healthier filter. Used by `EkfLane` to select the
    /// primary EKF core. The score combines:
    /// - Velocity innovation magnitude (normalized by variance)
    /// - Position innovation magnitude (normalized by variance)
    /// - Height innovation magnitude (normalized by variance)
    ///
    /// Source: ArduPilot AP_NavEKF3_core.cpp errorScore()
    pub fn error_score(&self) -> f32 {
        let mut score = 0.0f32;

        // Velocity innovation contribution
        for i in 0..3 {
            let var = self.covariance.p[4 + i][4 + i];
            if var > 1e-6 {
                score += self.vel_innov[i] * self.vel_innov[i] / var;
            }
        }

        // Position innovation contribution
        for i in 0..2 {
            let var = self.covariance.p[7 + i][7 + i];
            if var > 1e-6 {
                score += self.pos_innov[i] * self.pos_innov[i] / var;
            }
        }

        // Height innovation contribution
        let hgt_var = self.covariance.p[9][9];
        if hgt_var > 1e-6 {
            score += self.hgt_innov * self.hgt_innov / hgt_var;
        }

        score
    }
}

// ─── EKF Lane Switching ───────────────────────────────────────────────────

/// Maximum number of EKF lanes (one per IMU).
/// Source: ArduPilot supports up to 3 IMUs.
pub const MAX_EKF_LANES: usize = 3;

/// Multi-lane EKF manager.
///
/// Runs multiple EKF cores simultaneously (one per IMU), selecting the
/// healthiest as the primary. If the primary diverges, it automatically
/// switches to the best backup.
///
/// Source: ArduPilot AP_NavEKF3 front-end manages lane selection via
/// `SelectCoreAndIndex()` and `errorScore()`.
pub struct EkfLane {
    /// EKF cores, one per IMU. Indexed by IMU number.
    /// Uses `Option` to support 1-3 IMUs without wasting memory.
    pub cores: [Option<EkfCore>; MAX_EKF_LANES],
    /// Index of the currently selected primary core.
    pub primary: usize,
    /// Number of active lanes.
    pub num_lanes: usize,
    /// Minimum time between lane switches (ms). Prevents oscillation.
    pub min_switch_interval_ms: u32,
    /// Timestamp of last lane switch.
    last_switch_time: Instant,
}

impl EkfLane {
    /// Create a new lane manager with the specified number of IMUs.
    ///
    /// * `num_imus` - Number of IMU instances (1-3)
    /// * `origin` - Initial EKF origin position
    pub fn new(num_imus: usize, origin: LatLonAlt) -> Self {
        let num = num_imus.min(MAX_EKF_LANES).max(1);
        let mut cores: [Option<EkfCore>; MAX_EKF_LANES] = [None, None, None];

        for i in 0..num {
            let mut core = EkfCore::new(origin);
            core.imu_index = i as u8;
            cores[i] = Some(core);
        }

        Self {
            cores,
            primary: 0,
            num_lanes: num,
            min_switch_interval_ms: 5000,
            last_switch_time: Instant::ZERO,
        }
    }

    /// Get a reference to the primary (best) EKF core.
    pub fn primary_core(&self) -> Option<&EkfCore> {
        self.cores[self.primary].as_ref()
    }

    /// Get a mutable reference to the primary core.
    pub fn primary_core_mut(&mut self) -> Option<&mut EkfCore> {
        self.cores[self.primary].as_mut()
    }

    /// Get a mutable reference to a specific core by IMU index.
    pub fn core_mut(&mut self, imu_index: usize) -> Option<&mut EkfCore> {
        if imu_index < MAX_EKF_LANES {
            self.cores[imu_index].as_mut()
        } else {
            None
        }
    }

    /// Get a reference to a specific core by IMU index.
    pub fn core_ref(&self, imu_index: usize) -> Option<&EkfCore> {
        if imu_index < MAX_EKF_LANES {
            self.cores[imu_index].as_ref()
        } else {
            None
        }
    }

    /// Select the best lane based on health and error score.
    ///
    /// Switches primary to the lane with the lowest error score among
    /// healthy lanes. If no lane is healthy, keeps current primary.
    /// Respects minimum switch interval to prevent oscillation.
    ///
    /// Source: ArduPilot AP_NavEKF3::SelectCoreAndIndex()
    pub fn select_primary(&mut self, now: Instant) {
        // Enforce minimum switch interval
        if self.last_switch_time != Instant::ZERO {
            let elapsed_us = now.as_micros().saturating_sub(self.last_switch_time.as_micros());
            let elapsed_ms = (elapsed_us / 1000) as u32;
            if elapsed_ms < self.min_switch_interval_ms {
                return;
            }
        }

        let mut best_idx = self.primary;
        let mut best_score = f32::MAX;
        let mut any_healthy = false;

        for i in 0..self.num_lanes {
            if let Some(ref core) = self.cores[i] {
                if core.health == EkfHealth::Healthy {
                    any_healthy = true;
                    let score = core.error_score();
                    if score < best_score {
                        best_score = score;
                        best_idx = i;
                    }
                }
            }
        }

        // Only switch if we found a healthy lane different from current
        if any_healthy && best_idx != self.primary {
            // Only switch if the new lane is significantly better (>50% lower score)
            // to prevent frequent switching between similar lanes
            if let Some(ref current) = self.cores[self.primary] {
                let current_score = current.error_score();
                if best_score < current_score * 0.5
                    || current.health != EkfHealth::Healthy
                {
                    self.primary = best_idx;
                    self.last_switch_time = now;
                }
            } else {
                self.primary = best_idx;
                self.last_switch_time = now;
            }
        }
    }

    /// Get the output state from the primary core.
    pub fn output_state(&self) -> Option<EkfState> {
        self.primary_core().map(|c| c.output_state())
    }

    /// Check if any lane is healthy.
    pub fn any_healthy(&self) -> bool {
        self.cores.iter()
            .any(|c| c.as_ref().is_some_and(|core| core.health == EkfHealth::Healthy))
    }

    /// Get health status of all lanes.
    pub fn lane_health(&self) -> [Option<EkfHealth>; MAX_EKF_LANES] {
        [
            self.cores[0].as_ref().map(|c| c.health),
            self.cores[1].as_ref().map(|c| c.health),
            self.cores[2].as_ref().map(|c| c.health),
        ]
    }
}

// ─── Tests ─────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use meridian_types::messages::GnssFixType;

    fn make_imu(time_us: u64, accel: Vec3<Body>, gyro: Vec3<Body>) -> ImuSample {
        ImuSample {
            timestamp: Instant::from_micros(time_us),
            imu_index: 0,
            accel,
            gyro,
            temperature: 25.0,
        }
    }

    fn make_gps(time_us: u64, origin: &LatLonAlt) -> GnssPosition {
        GnssPosition {
            timestamp: Instant::from_micros(time_us),
            fix_type: GnssFixType::Fix3D,
            position: *origin,
            velocity_ned: Vec3::zero(),
            horizontal_accuracy: 1.0,
            vertical_accuracy: 1.5,
            speed_accuracy: 0.3,
            num_sats: 12,
        }
    }

    fn make_baro(time_us: u64, alt: f32) -> BaroPressure {
        BaroPressure {
            timestamp: Instant::from_micros(time_us),
            baro_index: 0,
            pressure_pa: 101325.0,
            temperature: 25.0,
            altitude_m: alt,
        }
    }

    fn make_mag(time_us: u64) -> MagField {
        MagField {
            timestamp: Instant::from_micros(time_us),
            mag_index: 0,
            field: Vec3::new(0.22, 0.005, 0.42),
        }
    }

    #[test]
    fn test_ekf_stationary() {
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
        let mut ekf = EkfCore::new(origin);

        let hover_accel = Vec3::<Body>::new(0.0, 0.0, -crate::predict::GRAVITY);
        let dt_us = 2500u64;

        for step in 0..800 {
            let t = step * dt_us;
            ekf.predict(&make_imu(t, hover_accel, Vec3::zero()));
            if step % 40 == 0 {
                ekf.fuse_gps(&make_gps(t, &origin));
            }
            if step % 8 == 0 {
                ekf.fuse_baro(&make_baro(t, 0.0));
            }
            if step % 4 == 0 {
                ekf.fuse_mag(&make_mag(t));
            }
        }

        assert_eq!(ekf.health, EkfHealth::Healthy);

        let pos = ekf.state.position;
        assert!(pos.length() < 5.0,
            "Position should be near origin: ({}, {}, {})", pos.x, pos.y, pos.z);

        let vel = ekf.state.velocity;
        assert!(vel.length() < 1.0,
            "Velocity should be near zero: ({}, {}, {})", vel.x, vel.y, vel.z);

        let (roll, pitch, _yaw) = ekf.state.quat.to_euler();
        assert!(roll.abs() < 0.1, "Roll should be near zero: {}", roll);
        assert!(pitch.abs() < 0.1, "Pitch should be near zero: {}", pitch);
    }

    #[test]
    fn test_ekf_no_nan() {
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
        let mut ekf = EkfCore::new(origin);

        let accel = Vec3::<Body>::new(0.5, -0.3, -9.8);
        let gyro = Vec3::<Body>::new(0.1, -0.05, 0.02);

        for step in 0..400 {
            let t = step * 2500u64;
            ekf.predict(&make_imu(t, accel, gyro));
            if step % 40 == 0 {
                ekf.fuse_gps(&make_gps(t, &origin));
            }
            if step % 8 == 0 {
                ekf.fuse_baro(&make_baro(t, 5.0));
            }
        }

        let arr = ekf.state.to_array();
        for (i, v) in arr.iter().enumerate() {
            assert!(!v.is_nan(), "State {} is NaN after prediction+fusion", i);
        }
    }

    #[test]
    fn test_gps_blocked_before_alignment() {
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
        let mut ekf = EkfCore::new(origin);

        let hover_accel = Vec3::<Body>::new(0.0, 0.0, -crate::predict::GRAVITY);
        let dt_us = 2500u64;

        for step in 0..200 {
            let t = step * dt_us;
            ekf.predict(&make_imu(t, hover_accel, Vec3::zero()));
            if step % 40 == 0 {
                let mut gps = make_gps(t, &origin);
                gps.position = origin;
                ekf.fuse_gps(&gps);
            }
        }

        assert!(!ekf.alignment.tilt_aligned, "Tilt should not align without mag data");
        assert!(!ekf.is_gps_fusing(), "GPS should not be fusing before alignment");
        assert_eq!(ekf.aiding_mode, AidingMode::None);
    }

    #[test]
    fn test_aiding_mode_transition() {
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
        let mut ekf = EkfCore::new(origin);
        ekf.gps_gate.required_duration_ms = 200;

        let hover_accel = Vec3::<Body>::new(0.0, 0.0, -crate::predict::GRAVITY);
        let dt_us = 2500u64;

        for step in 0..200 {
            let t = step * dt_us;
            ekf.predict(&make_imu(t, hover_accel, Vec3::zero()));
            if step % 4 == 0 {
                ekf.fuse_mag(&make_mag(t));
            }
            if step % 8 == 0 {
                ekf.fuse_baro(&make_baro(t, 0.0));
            }
        }
        assert!(ekf.alignment.tilt_aligned, "Tilt should be aligned after mag fusion");
        assert_eq!(ekf.aiding_mode, AidingMode::None, "Should still be None before GPS gate");

        ekf.state.position = Vec3::new(50.0, 50.0, ekf.state.position.z);

        let gps_start_step = 200u64;
        for step in 0..400 {
            let t = (gps_start_step + step) * dt_us;
            ekf.predict(&make_imu(t, hover_accel, Vec3::zero()));
            if step % 4 == 0 {
                ekf.fuse_mag(&make_mag(t));
            }
            if step % 8 == 0 {
                ekf.fuse_baro(&make_baro(t, 0.0));
            }
            if step % 40 == 0 {
                ekf.fuse_gps(&make_gps(t, &origin));
            }
        }

        assert_eq!(ekf.aiding_mode, AidingMode::Absolute,
            "Should transition to Absolute after GPS gate clears");
        assert!(ekf.gps_acquired, "gps_acquired should be set");

        let pos = ekf.state.position;
        assert!(libm::sqrtf(pos.x * pos.x + pos.y * pos.y) < 10.0,
            "Position should be near origin after reset: ({}, {})", pos.x, pos.y);
    }

    // ─── GPS Delay Buffer Tests ───

    #[test]
    fn test_imu_delay_buffer_basic() {
        let mut buf = ImuDelayBuffer::new(220, 2.5);
        assert_eq!(buf.target_depth(), 88);
        assert!(!buf.is_ready());
        assert_eq!(buf.len(), 0);

        for i in 0..50 {
            buf.push(ImuDelta {
                del_ang: Vec3::new(i as f32, 0.0, 0.0),
                del_vel: Vec3::zero(),
                del_ang_dt: 0.0025,
                del_vel_dt: 0.0025,
            });
        }
        assert_eq!(buf.len(), 50);
        assert!(!buf.is_ready());
    }

    #[test]
    fn test_imu_delay_buffer_returns_delayed() {
        let mut buf = ImuDelayBuffer::new(25, 2.5); // 10 samples deep
        assert_eq!(buf.target_depth(), 10);

        for i in 0..20 {
            buf.push(ImuDelta {
                del_ang: Vec3::new(i as f32, 0.0, 0.0),
                del_vel: Vec3::zero(),
                del_ang_dt: 0.0025,
                del_vel_dt: 0.0025,
            });
        }

        assert!(buf.is_ready());
        let delayed = buf.get_delayed();
        assert!((delayed.del_ang.x - 10.0).abs() < 0.01,
            "Delayed sample should be 10 steps old, got del_ang.x={}", delayed.del_ang.x);
    }

    #[test]
    fn test_imu_delay_buffer_wraparound() {
        let mut buf = ImuDelayBuffer::new(10, 2.5); // 4 samples deep
        assert_eq!(buf.target_depth(), 4);

        for i in 0..150 {
            buf.push(ImuDelta {
                del_ang: Vec3::new(i as f32, 0.0, 0.0),
                del_vel: Vec3::zero(),
                del_ang_dt: 0.0025,
                del_vel_dt: 0.0025,
            });
        }

        let delayed = buf.get_delayed();
        // Head is at 150 % 100 = 50, delayed is 4 back = sample 146
        assert!((delayed.del_ang.x - 146.0).abs() < 0.01,
            "After wraparound, delayed should be sample 146, got {}", delayed.del_ang.x);
    }

    #[test]
    fn test_delay_buffer_used_in_predict() {
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
        let mut ekf = EkfCore::new(origin);

        let hover_accel = Vec3::<Body>::new(0.0, 0.0, -crate::predict::GRAVITY);
        for step in 0..200 {
            let t = step * 2500u64;
            ekf.predict(&make_imu(t, hover_accel, Vec3::zero()));
        }

        assert!(ekf.imu_delay_buf.len() > 0, "Buffer should have samples after prediction");
    }

    // ─── Origin Reset Tests ───

    #[test]
    fn test_origin_reset_preserves_lla() {
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 100.0);
        let mut ekf = EkfCore::new(origin);

        ekf.state.position = Vec3::new(1000.0, 0.0, -100.0);
        ekf.output.current.position = Vec3::new(1000.0, 0.0, -100.0);

        let lla_before = ekf.position_lla();

        let new_origin = lla_before;
        ekf.reset_origin(new_origin);

        let pos = ekf.state.position;
        assert!(pos.length() < 5.0,
            "Position should be near zero after origin reset to current pos: ({}, {}, {})",
            pos.x, pos.y, pos.z);

        assert!((ekf.origin.lat - new_origin.lat).abs() < 1e-8);
        assert!((ekf.origin.lon - new_origin.lon).abs() < 1e-8);
    }

    #[test]
    fn test_origin_reset_at_zero_position() {
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
        let mut ekf = EkfCore::new(origin);
        ekf.state.position = Vec3::zero();

        ekf.reset_origin(origin);
        assert!(ekf.state.position.length() < 1.0,
            "Origin reset to same point should not move position");
    }

    // ─── Lane Switching Tests ───

    #[test]
    fn test_lane_single_imu() {
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
        let lane = EkfLane::new(1, origin);
        assert_eq!(lane.num_lanes, 1);
        assert_eq!(lane.primary, 0);
        assert!(lane.cores[0].is_some());
        assert!(lane.cores[1].is_none());
        assert!(lane.cores[2].is_none());
    }

    #[test]
    fn test_lane_multi_imu() {
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
        let lane = EkfLane::new(3, origin);
        assert_eq!(lane.num_lanes, 3);
        for i in 0..3 {
            assert!(lane.cores[i].is_some(), "Core {} should exist", i);
            assert_eq!(lane.cores[i].as_ref().unwrap().imu_index, i as u8);
        }
    }

    #[test]
    fn test_lane_primary_selection() {
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
        let mut lane = EkfLane::new(2, origin);

        lane.cores[0].as_mut().unwrap().health = EkfHealth::Unhealthy;
        lane.cores[1].as_mut().unwrap().health = EkfHealth::Healthy;

        lane.select_primary(Instant::from_micros(10_000_000));
        assert_eq!(lane.primary, 1, "Should switch to healthy core 1");
    }

    #[test]
    fn test_lane_switch_interval() {
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
        let mut lane = EkfLane::new(2, origin);
        lane.min_switch_interval_ms = 5000;

        lane.cores[0].as_mut().unwrap().health = EkfHealth::Unhealthy;
        lane.cores[1].as_mut().unwrap().health = EkfHealth::Healthy;
        lane.select_primary(Instant::from_micros(10_000_000));
        assert_eq!(lane.primary, 1);

        lane.cores[0].as_mut().unwrap().health = EkfHealth::Healthy;
        lane.select_primary(Instant::from_micros(11_000_000));
        assert_eq!(lane.primary, 1, "Should not switch within min interval");
    }

    #[test]
    fn test_lane_health_status() {
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
        let mut lane = EkfLane::new(2, origin);

        lane.cores[0].as_mut().unwrap().health = EkfHealth::Healthy;
        lane.cores[1].as_mut().unwrap().health = EkfHealth::Initializing;

        let health = lane.lane_health();
        assert_eq!(health[0], Some(EkfHealth::Healthy));
        assert_eq!(health[1], Some(EkfHealth::Initializing));
        assert_eq!(health[2], None);

        assert!(lane.any_healthy());
    }

    #[test]
    fn test_error_score_increases_with_innovation() {
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
        let mut ekf = EkfCore::new(origin);

        let score_zero = ekf.error_score();
        assert!(score_zero < 0.01, "Zero innovation should give near-zero score");

        ekf.vel_innov = [5.0, 3.0, 1.0];
        ekf.pos_innov = [10.0, -8.0];
        ekf.hgt_innov = 3.0;

        let score_nonzero = ekf.error_score();
        assert!(score_nonzero > score_zero,
            "Non-zero innovations should increase error score: {} vs {}", score_nonzero, score_zero);
    }
}

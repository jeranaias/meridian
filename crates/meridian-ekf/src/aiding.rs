//! EKF aiding mode management, GPS quality gates, alignment checks,
//! mag calibration state machine, height source switching, WMM declination.
//!
//! Ports: AP_NavEKF3_Control.cpp (setAidingMode, checkAttitudeAlignmentStatus)
//!        AP_NavEKF3_VehicleStatus.cpp (calcGpsGoodToAlign, calcGpsGoodForFlight, detectFlight)
//!        AP_NavEKF3_MagFusion.cpp (mag calibration FSM, MagTableConstrain)

use meridian_math::Vec3;
use meridian_math::frames::NED;

/// Aiding mode -- what sensors are actively correcting the filter.
/// Source: AP_NavEKF3_Control.cpp setAidingMode()
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AidingMode {
    /// No external aiding -- IMU dead reckoning only.
    None,
    /// Relative aiding (optical flow, etc.) -- no absolute position.
    Relative,
    /// Absolute aiding (GPS) -- full position/velocity corrections.
    Absolute,
}

/// Flight state detection.
/// Source: AP_NavEKF3_VehicleStatus.cpp detectFlight()
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightState {
    OnGround,
    InFlight,
}

/// Mag calibration mode.
/// Source: AP_NavEKF3_MagFusion.cpp MagCal enum
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MagCalMode {
    /// Always estimate mag states.
    Always,
    /// Only estimate mag states when flying.
    WhenFlying,
    /// Only estimate mag states when manoeuvring (high horizontal acceleration).
    WhenManoeuvring,
    /// Estimate once after first climb above threshold altitude, then hold.
    AfterFirstClimb,
    /// Estimate on ground and in flight.
    GroundAndInflight,
    /// Never estimate mag states -- use initial values.
    Never,
}

/// Mag calibration state machine.
/// Source: AP_NavEKF3_MagFusion.cpp mag calibration logic
#[derive(Debug, Clone)]
pub struct MagCalibration {
    /// Current calibration mode.
    pub mode: MagCalMode,
    /// Whether mag states are currently being estimated.
    pub mag_states_active: bool,
    /// Final in-flight mag init has been performed.
    pub final_inflight_mag_init: bool,
    /// Final in-flight yaw init has been performed.
    pub final_inflight_yaw_init: bool,
    /// Whether body mag variance needs reset.
    pub need_mag_body_var_reset: bool,
    /// Whether earth mag variance needs reset.
    pub need_earth_body_var_reset: bool,
    /// Number of mag anomaly resets performed. Max: MAG_ANOMALY_RESET_MAX = 2
    pub mag_anomaly_reset_count: u8,
    /// Altitude for final in-flight mag reset (m). Source: EK3_MAG_FINAL_RESET_ALT = 2.5
    pub final_reset_alt: f32,
}

impl MagCalibration {
    pub fn new() -> Self {
        Self {
            mode: MagCalMode::WhenFlying,
            mag_states_active: false,
            final_inflight_mag_init: false,
            final_inflight_yaw_init: false,
            need_mag_body_var_reset: false,
            need_earth_body_var_reset: false,
            mag_anomaly_reset_count: 0,
            final_reset_alt: 2.5,
        }
    }

    /// Update mag calibration state based on flight phase and mode.
    /// Source: AP_NavEKF3_MagFusion.cpp controlMagYawReset() + setWindMagStateLearningMode()
    pub fn update(
        &mut self,
        in_flight: bool,
        manoeuvring: bool,
        altitude_agl: f32,
        tilt_aligned: bool,
    ) {
        self.mag_states_active = match self.mode {
            MagCalMode::Always => tilt_aligned,
            MagCalMode::WhenFlying => in_flight && tilt_aligned,
            MagCalMode::WhenManoeuvring => in_flight && manoeuvring && tilt_aligned,
            MagCalMode::AfterFirstClimb => {
                if !self.final_inflight_mag_init && altitude_agl > self.final_reset_alt {
                    self.final_inflight_mag_init = true;
                    self.need_mag_body_var_reset = true;
                    self.need_earth_body_var_reset = true;
                }
                self.final_inflight_mag_init && tilt_aligned
            }
            MagCalMode::GroundAndInflight => tilt_aligned,
            MagCalMode::Never => false,
        };
    }

    /// Check if a mag anomaly reset should be performed.
    /// Source: MAG_ANOMALY_RESET_MAX = 2
    pub fn should_reset_anomaly(&self) -> bool {
        self.mag_anomaly_reset_count < 2
    }

    /// Record that an anomaly reset was performed.
    pub fn record_anomaly_reset(&mut self) {
        self.mag_anomaly_reset_count += 1;
    }
}

/// Height source selector.
/// Source: AP_NavEKF3_PosVelFusion.cpp activeHgtSource
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HeightSource {
    Baro,
    Gps,
    Rangefinder,
    Beacon,
    ExternalNav,
}

/// Height source state machine.
#[derive(Debug, Clone)]
pub struct HeightSourceSelector {
    /// Primary configured source.
    pub primary: HeightSource,
    /// Currently active source.
    pub active: HeightSource,
    /// Whether rangefinder is in range and valid.
    pub rng_valid: bool,
    /// Altitude AGL for rangefinder switching (m).
    pub rng_switch_alt: f32,
}

impl HeightSourceSelector {
    pub fn new(primary: HeightSource) -> Self {
        Self {
            primary,
            active: primary,
            rng_valid: false,
            rng_switch_alt: 10.0, // switch to rangefinder below 10m AGL
        }
    }

    /// Update height source based on available sensors and altitude.
    /// Source: AP_NavEKF3_PosVelFusion.cpp selectHeightForFusion()
    pub fn update(&mut self, altitude_agl: f32, rng_healthy: bool, rng_in_range: bool) {
        self.rng_valid = rng_healthy && rng_in_range;

        match self.primary {
            HeightSource::Rangefinder => {
                self.active = if self.rng_valid {
                    HeightSource::Rangefinder
                } else {
                    HeightSource::Baro // fallback
                };
            }
            _ => {
                // If rangefinder is configured as primary=Baro but we're low,
                // optionally switch to rangefinder for precision landing
                if self.rng_valid && altitude_agl < self.rng_switch_alt {
                    self.active = HeightSource::Rangefinder;
                } else {
                    self.active = self.primary;
                }
            }
        }
    }
}

/// GPS quality assessment for pre-arm and in-flight.
/// Source: AP_NavEKF3_VehicleStatus.cpp calcGpsGoodToAlign()
#[derive(Debug, Clone)]
pub struct GpsQualityGate {
    /// Has the gate been passing continuously for the required duration?
    pub good_to_align: bool,
    /// Time GPS quality has been continuously good (ms).
    good_duration_ms: u32,
    /// Required continuous good duration (ms). Default: 10000 (10s).
    pub required_duration_ms: u32,
    /// In-flight GPS health (peak-hold accuracy check).
    pub good_for_flight: bool,
    /// Last GPS horizontal accuracy seen.
    last_hacc: f32,
    /// Speed accuracy filter (peak-hold with decay).
    speed_acc_filt: f32,
    /// Peak speed accuracy (decays slowly).
    speed_acc_peak: f32,
    /// EKF innovation fail counter (for in-flight health).
    innov_fail_count: u32,
    /// EKF innovation pass counter (for in-flight health).
    innov_pass_count: u32,
    /// GPS position drift accumulator (for stationary check).
    pos_drift_accum: f32,
    /// Last GPS position for drift check.
    last_pos: [f32; 2],
    /// Whether we have a previous position for drift check.
    has_last_pos: bool,
}

impl GpsQualityGate {
    pub fn new() -> Self {
        Self {
            good_to_align: false,
            good_duration_ms: 0,
            required_duration_ms: 10000,
            good_for_flight: true,
            last_hacc: 99.0,
            speed_acc_filt: 0.0,
            speed_acc_peak: 0.0,
            innov_fail_count: 0,
            innov_pass_count: 0,
            pos_drift_accum: 0.0,
            last_pos: [0.0; 2],
            has_last_pos: false,
        }
    }

    /// Update pre-arm GPS quality gate. Call at GPS rate.
    /// Source: calcGpsGoodToAlign() checks: fix type, sats, hAcc, vAcc, sAcc, drift, HDOP
    pub fn update_align(
        &mut self,
        fix_3d: bool,
        num_sats: u8,
        hacc: f32,
        vacc: f32,
        sacc: f32,
        dt_ms: u32,
    ) {
        // Core quality checks (already present)
        let mut good = fix_3d
            && num_sats >= 6
            && hacc < 5.0       // horizontal accuracy < 5m
            && vacc < 8.0       // vertical accuracy < 8m
            && sacc < 1.0;      // speed accuracy < 1 m/s

        // Hysteresis: if already aligned, use 1.3x thresholds
        // Source: AP_NavEKF3_VehicleStatus.cpp checkScaler usage
        if self.good_to_align {
            good = fix_3d
                && num_sats >= 6
                && hacc < 6.5    // 5.0 * 1.3
                && vacc < 10.4   // 8.0 * 1.3
                && sacc < 1.3;   // 1.0 * 1.3
        }

        if good {
            self.good_duration_ms += dt_ms;
            if self.good_duration_ms >= self.required_duration_ms {
                self.good_to_align = true;
            }
        } else {
            self.good_duration_ms = 0;
            self.good_to_align = false;
        }

        self.last_hacc = hacc;
    }

    /// Update in-flight GPS health. Call at GPS rate.
    /// Source: calcGpsGoodForFlight()
    /// Uses two-state filter: LPF + peak-hold with 1.5 m/s fail / 1.0 m/s pass hysteresis,
    /// plus EKF innovation pass/fail hysteresis.
    pub fn update_flight(
        &mut self,
        sacc: f32,
        vel_innov_ratio: f32,
    ) {
        // LPF on speed accuracy
        self.speed_acc_filt = self.speed_acc_filt * 0.95 + sacc * 0.05;

        // Peak-hold with slow decay
        if sacc > self.speed_acc_peak {
            self.speed_acc_peak = sacc;
        } else {
            self.speed_acc_peak *= 0.995;
        }

        // Speed accuracy check with hysteresis
        let speed_ok = if self.good_for_flight {
            self.speed_acc_filt < 1.5 && self.speed_acc_peak < 2.0
        } else {
            self.speed_acc_filt < 1.0 && self.speed_acc_peak < 1.5
        };

        // Innovation check with hysteresis
        if vel_innov_ratio > 1.0 {
            self.innov_fail_count += 1;
            self.innov_pass_count = 0;
        } else {
            self.innov_pass_count += 1;
            self.innov_fail_count = 0;
        }

        // Innovation health: fail after 1s of bad (10 samples at 10Hz), pass after 10s (100 samples)
        let innov_ok = if self.good_for_flight {
            self.innov_fail_count < 10
        } else {
            self.innov_pass_count > 100
        };

        self.good_for_flight = speed_ok && innov_ok;
    }

    pub fn reset(&mut self) {
        self.good_to_align = false;
        self.good_duration_ms = 0;
        self.good_for_flight = true;
        self.speed_acc_filt = 0.0;
        self.speed_acc_peak = 0.0;
        self.innov_fail_count = 0;
        self.innov_pass_count = 0;
    }
}

/// Attitude alignment status.
/// Source: AP_NavEKF3_Control.cpp checkAttitudeAlignmentStatus()
#[derive(Debug, Clone)]
pub struct AlignmentStatus {
    /// Tilt (roll/pitch) alignment complete -- attitude is trustworthy.
    pub tilt_aligned: bool,
    /// Yaw alignment complete -- heading is trustworthy.
    pub yaw_aligned: bool,
    /// Variance threshold for tilt alignment (rad^2). Default: (5 deg)^2 = 0.0076
    pub tilt_variance_threshold: f32,
    /// R5: Whether body mag variance needs reset (consumed by fuse_mag).
    /// Set by MagCalibration FSM, consumed by EkfCore::fuse_mag().
    pub need_mag_body_var_reset: bool,
}

impl AlignmentStatus {
    pub fn new() -> Self {
        Self {
            tilt_aligned: false,
            yaw_aligned: false,
            tilt_variance_threshold: 0.025, // ~9 deg -- converges quickly with mag from cold start
            need_mag_body_var_reset: false,
        }
    }

    /// Update alignment status from covariance.
    /// Tilt is aligned when quaternion variance is below threshold.
    pub fn update(&mut self, quat_variance: f32, has_mag_data: bool) {
        if quat_variance < self.tilt_variance_threshold {
            self.tilt_aligned = true;
        }
        // Yaw aligns after mag fusion has run (or GPS heading)
        if self.tilt_aligned && has_mag_data {
            self.yaw_aligned = true;
        }
    }

    pub fn fully_aligned(&self) -> bool {
        self.tilt_aligned && self.yaw_aligned
    }
}

/// Flight state detector with hysteresis.
/// Source: AP_NavEKF3_VehicleStatus.cpp detectFlight()
#[derive(Debug, Clone)]
pub struct FlightDetector {
    pub state: FlightState,
    /// Speed threshold for in-flight detection (m/s).
    pub speed_threshold: f32,
    /// Speed threshold for on-ground detection (m/s) -- lower for hysteresis.
    pub ground_threshold: f32,
}

impl FlightDetector {
    pub fn new() -> Self {
        Self {
            state: FlightState::OnGround,
            speed_threshold: 3.0,   // declare in-flight above 3 m/s
            ground_threshold: 1.5,  // declare on-ground below 1.5 m/s
        }
    }

    /// Update flight state from velocity and arm status.
    pub fn update(&mut self, speed: f32, armed: bool) {
        if !armed {
            self.state = FlightState::OnGround;
            return;
        }
        match self.state {
            FlightState::OnGround => {
                if speed > self.speed_threshold {
                    self.state = FlightState::InFlight;
                }
            }
            FlightState::InFlight => {
                if speed < self.ground_threshold {
                    self.state = FlightState::OnGround;
                }
            }
        }
    }

    pub fn in_flight(&self) -> bool {
        self.state == FlightState::InFlight
    }
}

/// WMM (World Magnetic Model) magnetic declination table.
/// Source: AP_NavEKF3_MagFusion.cpp MagTableConstrain()
///
/// This is a simplified lookup; full WMM uses spherical harmonic coefficients.
/// Returns declination in radians given latitude and longitude in radians.
///
/// For a complete implementation, the full WMM coefficient table (90+ entries)
/// should be embedded. This provides a reasonable approximation using a
/// grid-interpolated table of declination at 10-degree intervals.
pub struct WmmDeclination;

impl WmmDeclination {
    /// Get approximate magnetic declination (radians).
    /// Source: Simplified from AP_Declination tables.
    ///
    /// This is a coarse approximation. For flight-grade use, replace with
    /// the full WMM2020/2025 spherical harmonic model.
    pub fn get_declination(lat_rad: f32, lon_rad: f32) -> f32 {
        let lat_deg = lat_rad * (180.0 / core::f32::consts::PI);
        let lon_deg = lon_rad * (180.0 / core::f32::consts::PI);

        // Coarse 4-region approximation (sufficient for test/development)
        // Production should use full WMM table
        let decl_deg = if lat_deg > 60.0 {
            // High northern latitudes: large westerly declination near North America
            if lon_deg > -130.0 && lon_deg < -60.0 { -15.0 } else { 5.0 }
        } else if lat_deg > 20.0 {
            // Mid northern latitudes
            if lon_deg > -130.0 && lon_deg < -60.0 { -10.0 }
            else if lon_deg > -60.0 && lon_deg < 30.0 { -5.0 }
            else { 2.0 }
        } else if lat_deg > -20.0 {
            // Equatorial
            if lon_deg > -80.0 && lon_deg < -30.0 { -20.0 }
            else { 0.0 }
        } else {
            // Southern hemisphere
            if lon_deg > 100.0 && lon_deg < 180.0 { 10.0 }
            else { -5.0 }
        };

        decl_deg * (core::f32::consts::PI / 180.0)
    }

    /// Constrain earth magnetic field magnitude to WMM expected range.
    /// Source: AP_NavEKF3_MagFusion.cpp MagTableConstrain()
    ///
    /// Prevents the earth mag field states from drifting to physically
    /// impossible values. Expected total field: 0.2 to 0.7 gauss.
    pub fn constrain_earth_field(earth_mag: &mut Vec3<NED>) {
        let total = earth_mag.length();
        if total > 0.7 {
            let scale = 0.7 / total;
            earth_mag.x *= scale;
            earth_mag.y *= scale;
            earth_mag.z *= scale;
        } else if total < 0.2 && total > 1e-6 {
            let scale = 0.2 / total;
            earth_mag.x *= scale;
            earth_mag.y *= scale;
            earth_mag.z *= scale;
        }
    }
}

/// State reset functions for aiding mode transitions.
/// Source: AP_NavEKF3_Control.cpp ResetPosition/ResetVelocity/ResetHeight
pub mod resets {
    use crate::state::{StateVector, Covariance, NUM_STATES};
    use meridian_math::Vec3;
    use meridian_math::frames::{NED, Body};

    /// Reset horizontal position states to GPS measurement.
    pub fn reset_position(state: &mut StateVector, cov: &mut Covariance, gps_n: f32, gps_e: f32) {
        state.position = Vec3::new(gps_n, gps_e, state.position.z);
        // Zero position cross-covariances
        for i in 0..NUM_STATES {
            cov.p[7][i] = 0.0; cov.p[i][7] = 0.0;
            cov.p[8][i] = 0.0; cov.p[i][8] = 0.0;
        }
        // Set position variance to GPS accuracy
        cov.p[7][7] = 4.0; // ~2m initial uncertainty
        cov.p[8][8] = 4.0;
    }

    /// Reset velocity states to GPS measurement.
    pub fn reset_velocity(state: &mut StateVector, cov: &mut Covariance, gps_vel: &Vec3<NED>) {
        state.velocity = *gps_vel;
        for i in 0..NUM_STATES {
            cov.p[4][i] = 0.0; cov.p[i][4] = 0.0;
            cov.p[5][i] = 0.0; cov.p[i][5] = 0.0;
            cov.p[6][i] = 0.0; cov.p[i][6] = 0.0;
        }
        cov.p[4][4] = 0.09; // sq(0.3) = VELNE_M_NSE default
        cov.p[5][5] = 0.09;
        cov.p[6][6] = 0.25; // sq(0.5) = VELD_M_NSE default
    }

    /// Reset height state to baro/GPS measurement.
    pub fn reset_height(state: &mut StateVector, cov: &mut Covariance, height_d: f32) {
        state.position = Vec3::new(state.position.x, state.position.y, height_d);
        for i in 0..NUM_STATES {
            cov.p[9][i] = 0.0; cov.p[i][9] = 0.0;
        }
        cov.p[9][9] = 4.0;
    }

    /// Reset gyro bias states.
    /// Source: AP_NavEKF3_GyroBias.cpp resetGyroBias()
    pub fn reset_gyro_bias(state: &mut StateVector, cov: &mut Covariance, dt_avg: f32) {
        state.gyro_bias = Vec3::<Body>::zero();
        for i in 0..NUM_STATES {
            cov.p[10][i] = 0.0; cov.p[i][10] = 0.0;
            cov.p[11][i] = 0.0; cov.p[i][11] = 0.0;
            cov.p[12][i] = 0.0; cov.p[i][12] = 0.0;
        }
        let bias_var = (0.5 * core::f32::consts::PI / 180.0 * dt_avg)
            * (0.5 * core::f32::consts::PI / 180.0 * dt_avg);
        cov.p[10][10] = bias_var;
        cov.p[11][11] = bias_var;
        cov.p[12][12] = bias_var;
    }

    /// Reset mag field states.
    /// Source: AP_NavEKF3_MagFusion.cpp resetMagFieldStates()
    pub fn reset_mag_field(
        state: &mut StateVector,
        cov: &mut Covariance,
        measured_mag: &Vec3<meridian_math::frames::Body>,
    ) {
        // Set body mag bias to zero, earth field to measured field rotated to NED
        state.body_mag = Vec3::zero();
        let body_to_ned: meridian_math::Rotation<meridian_math::frames::Body, meridian_math::frames::NED>
            = meridian_math::Rotation::from_quaternion(state.quat);
        let earth_mag = body_to_ned.rotate(*measured_mag);
        state.earth_mag = earth_mag;

        // Reset covariance for mag states
        for s in 16..22 {
            for i in 0..NUM_STATES {
                cov.p[s][i] = 0.0;
                cov.p[i][s] = 0.0;
            }
        }
        // Earth mag variance
        cov.p[16][16] = 0.01;
        cov.p[17][17] = 0.01;
        cov.p[18][18] = 0.01;
        // Body mag variance
        cov.p[19][19] = 0.01;
        cov.p[20][20] = 0.01;
        cov.p[21][21] = 0.01;
    }

    /// Realign yaw from GPS velocity.
    /// Source: AP_NavEKF3_Control.cpp realignYawGPS()
    ///
    /// Emergency yaw recovery when yaw has diverged but GPS velocity is available.
    /// Sets yaw to atan2(Ve, Vn) if speed > 5 m/s.
    pub fn realign_yaw_gps(
        state: &mut StateVector,
        cov: &mut Covariance,
        gps_vel: &Vec3<NED>,
    ) -> bool {
        let speed = libm::sqrtf(gps_vel.x * gps_vel.x + gps_vel.y * gps_vel.y);
        if speed < 5.0 {
            return false;
        }

        let new_yaw = libm::atan2f(gps_vel.y, gps_vel.x);
        let (roll, pitch, _old_yaw) = state.quat.to_euler();
        state.quat = meridian_math::Quaternion::from_euler(roll, pitch, new_yaw);

        // Reset quaternion covariance
        for i in 0..4 {
            for j in 0..NUM_STATES {
                cov.p[i][j] = 0.0;
                cov.p[j][i] = 0.0;
            }
        }
        cov.p[0][0] = 0.01;
        cov.p[1][1] = 0.01;
        cov.p[2][2] = 0.01;
        cov.p[3][3] = 0.01;

        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gps_quality_gate_10s() {
        let mut gate = GpsQualityGate::new();
        // Feed good GPS for 9s -- should NOT pass
        for _ in 0..90 {
            gate.update_align(true, 12, 1.0, 1.5, 0.3, 100);
        }
        assert!(!gate.good_to_align, "Should not pass before 10s");

        // Feed 1 more second -- should pass
        for _ in 0..10 {
            gate.update_align(true, 12, 1.0, 1.5, 0.3, 100);
        }
        assert!(gate.good_to_align, "Should pass after 10s");
    }

    #[test]
    fn test_gps_quality_resets_on_bad() {
        let mut gate = GpsQualityGate::new();
        // 8s of good
        for _ in 0..80 { gate.update_align(true, 12, 1.0, 1.5, 0.3, 100); }
        // 1 bad sample
        gate.update_align(true, 3, 10.0, 10.0, 5.0, 100);
        assert_eq!(gate.good_duration_ms, 0, "Should reset on bad sample");
        // Need full 10s again
        for _ in 0..99 { gate.update_align(true, 12, 1.0, 1.5, 0.3, 100); }
        assert!(!gate.good_to_align);
        gate.update_align(true, 12, 1.0, 1.5, 0.3, 100);
        assert!(gate.good_to_align);
    }

    #[test]
    fn test_gps_quality_hysteresis() {
        let mut gate = GpsQualityGate::new();
        // Get aligned first
        for _ in 0..100 { gate.update_align(true, 12, 1.0, 1.5, 0.3, 100); }
        assert!(gate.good_to_align);

        // Now hacc = 5.5 (above 5.0 but below 6.5 hysteresis)
        gate.update_align(true, 12, 5.5, 1.5, 0.3, 100);
        // Should still be good due to hysteresis
        assert!(gate.good_to_align, "Hysteresis should keep aligned at hacc=5.5");
    }

    #[test]
    fn test_alignment_status() {
        let mut align = AlignmentStatus::new();
        assert!(!align.fully_aligned());

        // Low variance + mag data -> aligned
        align.update(0.001, true);
        assert!(align.fully_aligned());
    }

    #[test]
    fn test_flight_detector_hysteresis() {
        let mut det = FlightDetector::new();
        assert_eq!(det.state, FlightState::OnGround);

        // Speed above threshold -> in flight
        det.update(5.0, true);
        assert!(det.in_flight());

        // Speed between thresholds -> stays in flight (hysteresis)
        det.update(2.0, true);
        assert!(det.in_flight());

        // Speed below ground threshold -> on ground
        det.update(1.0, true);
        assert!(!det.in_flight());
    }

    #[test]
    fn test_flight_detector_disarm() {
        let mut det = FlightDetector::new();
        det.update(10.0, true);
        assert!(det.in_flight());
        det.update(10.0, false); // disarmed
        assert!(!det.in_flight());
    }

    #[test]
    fn test_position_reset() {
        use crate::state::{StateVector, Covariance};
        let mut state = StateVector::new();
        let mut cov = Covariance::initial();
        state.position = meridian_math::Vec3::new(100.0, 200.0, -50.0);

        resets::reset_position(&mut state, &mut cov, 5.0, 3.0);
        assert!((state.position.x - 5.0).abs() < 0.01);
        assert!((state.position.y - 3.0).abs() < 0.01);
        assert!((state.position.z - (-50.0)).abs() < 0.01); // Z unchanged
    }

    #[test]
    fn test_gyro_bias_reset() {
        use crate::state::{StateVector, Covariance};
        let mut state = StateVector::new();
        let mut cov = Covariance::initial();
        state.gyro_bias = meridian_math::Vec3::new(0.1, -0.05, 0.02);

        resets::reset_gyro_bias(&mut state, &mut cov, 0.0025);
        assert!(state.gyro_bias.length() < 1e-6);
        assert!(cov.p[10][10] > 0.0); // variance reset, not zero
    }

    #[test]
    fn test_mag_calibration_when_flying() {
        let mut cal = MagCalibration::new();
        cal.mode = MagCalMode::WhenFlying;

        cal.update(false, false, 0.0, true);
        assert!(!cal.mag_states_active, "Should not be active on ground");

        cal.update(true, false, 5.0, true);
        assert!(cal.mag_states_active, "Should be active in flight");
    }

    #[test]
    fn test_mag_calibration_after_first_climb() {
        let mut cal = MagCalibration::new();
        cal.mode = MagCalMode::AfterFirstClimb;

        cal.update(true, false, 1.0, true);
        assert!(!cal.mag_states_active, "Should not be active below threshold");

        cal.update(true, false, 3.0, true);
        assert!(cal.mag_states_active, "Should be active after climbing above threshold");
        assert!(cal.final_inflight_mag_init);
    }

    #[test]
    fn test_height_source_switching() {
        let mut sel = HeightSourceSelector::new(HeightSource::Baro);
        assert_eq!(sel.active, HeightSource::Baro);

        // Below 10m with valid rangefinder -> switch
        sel.update(5.0, true, true);
        assert_eq!(sel.active, HeightSource::Rangefinder);

        // Above 10m -> back to baro
        sel.update(15.0, true, true);
        assert_eq!(sel.active, HeightSource::Baro);

        // Invalid rangefinder -> stay on baro
        sel.update(5.0, false, true);
        assert_eq!(sel.active, HeightSource::Baro);
    }

    #[test]
    fn test_wmm_declination_range() {
        // Declination should be in a reasonable range everywhere
        for lat in (-8..=8).map(|x| (x as f32) * 10.0) {
            for lon in (-18..=18).map(|x| (x as f32) * 10.0) {
                let decl = WmmDeclination::get_declination(
                    lat.to_radians(), lon.to_radians());
                assert!(decl.abs() < 0.5, // < ~30 degrees
                    "Declination at ({}, {}) = {} rad seems too large",
                    lat, lon, decl);
            }
        }
    }

    #[test]
    fn test_wmm_constrain_earth_field() {
        let mut mag = Vec3::<NED>::new(1.0, 0.0, 0.0); // 1.0 gauss = too high
        WmmDeclination::constrain_earth_field(&mut mag);
        assert!(mag.length() <= 0.71, "Field should be constrained to 0.7: {}", mag.length());

        let mut mag_low = Vec3::<NED>::new(0.05, 0.05, 0.05);
        WmmDeclination::constrain_earth_field(&mut mag_low);
        assert!(mag_low.length() >= 0.19, "Field should be raised to 0.2: {}", mag_low.length());
    }

    #[test]
    fn test_realign_yaw_gps() {
        use crate::state::{StateVector, Covariance};
        let mut state = StateVector::new();
        let mut cov = Covariance::initial();

        // Moving east at 10 m/s
        let gps_vel = Vec3::<NED>::new(0.0, 10.0, 0.0);
        let ok = resets::realign_yaw_gps(&mut state, &mut cov, &gps_vel);
        assert!(ok, "Should realign with speed > 5 m/s");

        let (_roll, _pitch, yaw) = state.quat.to_euler();
        // Moving east: yaw should be ~pi/2
        assert!((yaw - core::f32::consts::FRAC_PI_2).abs() < 0.1,
            "Yaw should be ~90 deg for eastward motion: {}", yaw);
    }

    #[test]
    fn test_realign_yaw_gps_too_slow() {
        use crate::state::{StateVector, Covariance};
        let mut state = StateVector::new();
        let mut cov = Covariance::initial();

        let gps_vel = Vec3::<NED>::new(1.0, 0.0, 0.0); // too slow
        let ok = resets::realign_yaw_gps(&mut state, &mut cov, &gps_vel);
        assert!(!ok, "Should not realign with speed < 5 m/s");
    }

    #[test]
    fn test_in_flight_gps_health_hysteresis() {
        let mut gate = GpsQualityGate::new();

        // Start healthy
        for _ in 0..200 {
            gate.update_flight(0.3, 0.1);
        }
        assert!(gate.good_for_flight, "Should be healthy with good data");

        // Brief bad innovation
        for _ in 0..5 {
            gate.update_flight(0.3, 2.0); // bad ratio
        }
        // Should still be healthy (need >10 consecutive failures)
        assert!(gate.good_for_flight, "Brief bad innovation should not fail");

        // Sustained bad innovation
        for _ in 0..15 {
            gate.update_flight(0.3, 2.0);
        }
        assert!(!gate.good_for_flight, "Sustained bad innovation should fail");
    }
}

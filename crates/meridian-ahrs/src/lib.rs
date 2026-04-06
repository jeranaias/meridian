#![no_std]

//! AHRS manager: wraps EKF core(s) with multi-IMU support,
//! lane switching, and DCM fallback.
//!
//! For Phase 3: single-core implementation. Multi-core and DCM
//! fallback will be added when hardware targets are implemented.

use meridian_math::{Vec3, Rotation};
use meridian_math::frames::{NED, Body};
use meridian_math::geodetic::LatLonAlt;
use meridian_types::time::Instant;
use meridian_types::messages::{EkfState, ImuSample, GnssPosition, BaroPressure, MagField};
use meridian_ekf::{EkfCore, EkfHealth, DcmEstimator, VerticalCF};

/// AHRS source selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AhrsSource {
    /// Using EKF (primary)
    Ekf,
    /// Using DCM fallback (degraded)
    Dcm,
}

/// The AHRS manager.
pub struct Ahrs {
    /// Primary EKF core
    ekf: EkfCore,
    /// DCM fallback estimator — used when EKF is unhealthy.
    dcm: DcmEstimator,
    /// Vertical complementary filter — lightweight alt/climb fallback.
    vert_cf: VerticalCF,
    /// Which source is active
    source: AhrsSource,
    /// Last raw IMU gyro (for gyro_corrected subtraction).
    last_raw_gyro: Vec3<Body>,
}

impl Ahrs {
    pub fn new(origin: LatLonAlt) -> Self {
        Self {
            ekf: EkfCore::new(origin),
            dcm: DcmEstimator::new(),
            vert_cf: VerticalCF::new(),
            source: AhrsSource::Ekf,
            last_raw_gyro: Vec3::zero(),
        }
    }

    /// Get a mutable reference to the EKF core (for configuration).
    pub fn ekf_mut(&mut self) -> &mut EkfCore {
        &mut self.ekf
    }

    // ─── IMU rate calls ───

    /// Process IMU sample (call at IMU rate, 400+ Hz).
    pub fn predict(&mut self, imu: &ImuSample) {
        // Store raw gyro for gyro_corrected()
        self.last_raw_gyro = imu.gyro;

        // Always run EKF predict
        self.ekf.predict(imu);

        // Always run DCM fallback so it stays warm
        self.dcm.update_imu(imu.gyro, imu.accel, 0.0025);

        // Switch source based on EKF health
        match self.source {
            AhrsSource::Ekf => {
                if self.ekf.health != EkfHealth::Healthy {
                    self.source = AhrsSource::Dcm;
                }
            }
            AhrsSource::Dcm => {
                if self.ekf.health == EkfHealth::Healthy {
                    self.source = AhrsSource::Ekf;
                }
            }
        }
    }

    // ─── Sensor fusion calls ───

    pub fn fuse_gps(&mut self, gps: &GnssPosition) {
        self.ekf.fuse_gps(gps);
    }

    pub fn fuse_baro(&mut self, baro: &BaroPressure) {
        self.ekf.fuse_baro(baro);
        // Feed vertical CF fallback with baro altitude
        self.vert_cf.update_baro(baro.altitude_m, 0.02); // ~50 Hz
    }

    pub fn fuse_mag(&mut self, mag: &MagField) {
        self.ekf.fuse_mag(mag);
    }

    // ─── Output accessors ───

    /// Get the best current state estimate.
    pub fn state(&self) -> EkfState {
        self.ekf.output_state()
    }

    /// Get attitude as body-to-NED rotation.
    /// Falls back to DCM when EKF is unhealthy.
    pub fn attitude(&self) -> Rotation<Body, NED> {
        match self.source {
            AhrsSource::Ekf => self.ekf.attitude(),
            AhrsSource::Dcm => Rotation::from_quaternion(self.dcm.attitude()),
        }
    }

    /// Get NED position relative to origin.
    pub fn position_ned(&self) -> Vec3<NED> {
        self.ekf.state.position
    }

    /// Get NED velocity.
    pub fn velocity_ned(&self) -> Vec3<NED> {
        self.ekf.state.velocity
    }

    /// Get position as lat/lon/alt.
    pub fn position_lla(&self) -> LatLonAlt {
        self.ekf.position_lla()
    }

    /// Get angular velocity (gyro minus estimated bias).
    pub fn gyro_corrected(&self) -> Vec3<Body> {
        let bias = self.ekf.state.gyro_bias;
        Vec3::new(
            self.last_raw_gyro.x - bias.x,
            self.last_raw_gyro.y - bias.y,
            self.last_raw_gyro.z - bias.z,
        )
    }

    /// Is the AHRS producing reliable estimates?
    pub fn healthy(&self) -> bool {
        self.ekf.health == EkfHealth::Healthy
    }

    /// Which source is active?
    pub fn source(&self) -> AhrsSource {
        self.source
    }

    /// Access the DCM fallback estimator.
    pub fn dcm(&self) -> &DcmEstimator {
        &self.dcm
    }

    /// Access the vertical complementary filter.
    pub fn vert_cf(&self) -> &VerticalCF {
        &self.vert_cf
    }
}

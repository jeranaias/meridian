//! External AHRS IMU passthrough driver.
//!
//! Accepts pre-fused IMU data from external AHRS systems like VectorNav,
//! Lord MicroStrain, or SBG Systems. The external AHRS handles its own
//! internal filtering and calibration; we pass through the accel/gyro
//! samples as-is.
//!
//! ArduPilot reference: `AP_InertialSensor_ExternalAHRS.cpp`

use meridian_math::{frames::Body, Vec3};

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// IMU sample from an external AHRS device.
#[derive(Debug, Clone, Copy)]
pub struct ExternalAhrsImuSample {
    /// Acceleration in body frame [m/s^2].
    pub accel: Vec3<Body>,
    /// Angular rate in body frame [rad/s].
    pub gyro: Vec3<Body>,
    /// Die temperature [deg C], if available (-1 = unavailable).
    pub temperature: f32,
    /// Timestamp from the external device [microseconds].
    pub device_timestamp_us: u64,
}

/// External AHRS orientation/attitude output.
#[derive(Debug, Clone, Copy)]
pub struct ExternalAhrsAttitude {
    /// Quaternion [w, x, y, z].
    pub quaternion: [f32; 4],
    /// Angular rates [rad/s] in body frame.
    pub angular_rate: Vec3<Body>,
    /// Timestamp from the external device [microseconds].
    pub device_timestamp_us: u64,
}

/// Configuration for the external AHRS passthrough.
#[derive(Debug, Clone, Copy)]
pub struct ExternalAhrsConfig {
    /// Expected sample rate from the external device (Hz).
    pub expected_rate_hz: u16,
    /// Maximum allowed age of a sample before it is considered stale (ms).
    pub max_age_ms: u32,
}

impl Default for ExternalAhrsConfig {
    fn default() -> Self {
        Self {
            expected_rate_hz: 200,
            max_age_ms: 50,
        }
    }
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

/// External AHRS IMU passthrough.
///
/// This is a soft driver: no hardware init. It receives data pushed from
/// a serial protocol handler (VectorNav binary, SBG, etc.) and stores the
/// latest samples for the IMU frontend to consume.
pub struct ExternalAhrsImu {
    config: ExternalAhrsConfig,
    last_imu: Option<ExternalAhrsImuSample>,
    last_attitude: Option<ExternalAhrsAttitude>,
    sample_count: u32,
    last_update_us: u64,
}

impl ExternalAhrsImu {
    pub fn new(config: ExternalAhrsConfig) -> Self {
        Self {
            config,
            last_imu: None,
            last_attitude: None,
            sample_count: 0,
            last_update_us: 0,
        }
    }

    /// Push an IMU sample received from the external AHRS.
    pub fn push_imu_sample(&mut self, sample: ExternalAhrsImuSample) {
        self.last_imu = Some(sample);
        self.sample_count += 1;
        self.last_update_us = sample.device_timestamp_us;
    }

    /// Push an attitude (quaternion) sample from the external AHRS.
    pub fn push_attitude(&mut self, att: ExternalAhrsAttitude) {
        self.last_attitude = Some(att);
    }

    /// Get the latest IMU sample, if fresh enough.
    pub fn get_imu_sample(&self, now_us: u64) -> Option<&ExternalAhrsImuSample> {
        if let Some(ref sample) = self.last_imu {
            let age_ms = now_us.wrapping_sub(self.last_update_us) / 1000;
            if age_ms <= self.config.max_age_ms as u64 {
                return Some(sample);
            }
        }
        None
    }

    /// Get the latest attitude, if available.
    pub fn get_attitude(&self) -> Option<&ExternalAhrsAttitude> {
        self.last_attitude.as_ref()
    }

    /// Whether the external AHRS is providing data.
    pub fn is_healthy(&self, now_us: u64) -> bool {
        self.get_imu_sample(now_us).is_some()
    }

    /// Total number of samples received since init.
    pub fn sample_count(&self) -> u32 {
        self.sample_count
    }

    /// Expected sample rate.
    pub fn expected_rate_hz(&self) -> u16 {
        self.config.expected_rate_hz
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_push_and_get() {
        let mut drv = ExternalAhrsImu::new(ExternalAhrsConfig::default());
        assert!(drv.get_imu_sample(0).is_none());

        let sample = ExternalAhrsImuSample {
            accel: Vec3::<Body>::new(0.0, 0.0, -9.81),
            gyro: Vec3::<Body>::zero(),
            temperature: 25.0,
            device_timestamp_us: 1_000_000,
        };
        drv.push_imu_sample(sample);
        assert!(drv.get_imu_sample(1_001_000).is_some());
        assert_eq!(drv.sample_count(), 1);
    }

    #[test]
    fn test_stale_sample_rejected() {
        let mut drv = ExternalAhrsImu::new(ExternalAhrsConfig { max_age_ms: 50, ..Default::default() });
        let sample = ExternalAhrsImuSample {
            accel: Vec3::<Body>::zero(),
            gyro: Vec3::<Body>::zero(),
            temperature: 25.0,
            device_timestamp_us: 1_000_000,
        };
        drv.push_imu_sample(sample);
        // 100ms later — stale
        assert!(drv.get_imu_sample(1_100_000).is_none());
    }
}

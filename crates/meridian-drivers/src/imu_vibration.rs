//! IMU vibration and clipping detection — per-sample analysis.
//!
//! ArduPilot reference: `AP_InertialSensor::calc_vibration_and_clipping()`
//!
//! Called per-IMU-sample in the driver layer (before data reaches the EKF).
//! Maintains a two-stage low-pass filter to produce vibration floor and
//! vibration level, plus per-axis clip counting.

use meridian_math::{frames::Body, Vec3};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Clipping threshold: +-3G in m/s^2 (matching ArduPilot).
const CLIP_THRESHOLD_MS2: f32 = 29.4;

/// LP filter coefficient for vibration floor (slow).
const VIBE_FLOOR_ALPHA: f32 = 0.001;

/// LP filter coefficient for vibration level (faster).
const VIBE_LEVEL_ALPHA: f32 = 0.01;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Per-axis clip counters.
#[derive(Debug, Clone, Copy, Default)]
pub struct ClipCounts {
    pub x: u32,
    pub y: u32,
    pub z: u32,
}

/// Vibration monitor output — suitable for MAVLink VIBRATION message and EKF gating.
#[derive(Debug, Clone, Copy)]
pub struct VibrationOutput {
    /// LP-filtered vibration floor (baseline accel magnitude).
    pub vibe_floor: f32,
    /// LP-filtered vibration level (deviation from floor).
    pub vibe_level: f32,
    /// Per-axis clip counts since boot.
    pub clip_counts: ClipCounts,
    /// Total sample count processed.
    pub sample_count: u32,
    /// Whether vibration is excessive (for EKF gating).
    pub excessive: bool,
}

// ---------------------------------------------------------------------------
// Vibration monitor
// ---------------------------------------------------------------------------

/// Per-IMU-instance vibration and clipping monitor.
///
/// Call `process_sample()` for every IMU sample, before the data reaches
/// the EKF. The output can be queried at any time for GCS reporting.
pub struct VibrationMonitor {
    vibe_floor_filter: f32,
    vibe_level_filter: f32,
    clip_counts: ClipCounts,
    sample_count: u32,
    /// Threshold above which vibration is considered excessive (m/s^2).
    excessive_threshold: f32,
}

impl VibrationMonitor {
    /// Create a new vibration monitor.
    ///
    /// `excessive_threshold`: vibration level above which EKF gating should
    /// reduce GPS fusion weight (default: 0.5 m/s^2 matches ArduPilot).
    pub fn new(excessive_threshold: f32) -> Self {
        Self {
            vibe_floor_filter: 0.0,
            vibe_level_filter: 0.0,
            clip_counts: ClipCounts::default(),
            sample_count: 0,
            excessive_threshold,
        }
    }

    /// Create with default excessive threshold (0.5 m/s^2).
    pub fn default_threshold() -> Self {
        Self::new(0.5)
    }

    /// Process a single IMU sample. Call this per-sample in the driver.
    pub fn process_sample(&mut self, accel: &Vec3<Body>) {
        self.sample_count += 1;

        // Per-axis clipping detection (ArduPilot checks each axis independently)
        if libm::fabsf(accel.x) >= CLIP_THRESHOLD_MS2 {
            self.clip_counts.x += 1;
        }
        if libm::fabsf(accel.y) >= CLIP_THRESHOLD_MS2 {
            self.clip_counts.y += 1;
        }
        if libm::fabsf(accel.z) >= CLIP_THRESHOLD_MS2 {
            self.clip_counts.z += 1;
        }

        // Accel magnitude
        let mag = libm::sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);

        // Two-stage LP filter:
        // 1) vibe_floor: slowly tracks the baseline accel magnitude (~1G)
        // 2) vibe_level: faster-tracking deviation from floor
        if self.sample_count == 1 {
            self.vibe_floor_filter = mag;
            self.vibe_level_filter = 0.0;
        } else {
            self.vibe_floor_filter += VIBE_FLOOR_ALPHA * (mag - self.vibe_floor_filter);
            let deviation = libm::fabsf(mag - self.vibe_floor_filter);
            self.vibe_level_filter += VIBE_LEVEL_ALPHA * (deviation - self.vibe_level_filter);
        }
    }

    /// Get the current vibration output.
    pub fn output(&self) -> VibrationOutput {
        VibrationOutput {
            vibe_floor: self.vibe_floor_filter,
            vibe_level: self.vibe_level_filter,
            clip_counts: self.clip_counts,
            sample_count: self.sample_count,
            excessive: self.vibe_level_filter > self.excessive_threshold,
        }
    }

    /// Total clip count across all axes.
    pub fn total_clip_count(&self) -> u32 {
        self.clip_counts.x + self.clip_counts.y + self.clip_counts.z
    }

    /// Reset all counters.
    pub fn reset(&mut self) {
        self.vibe_floor_filter = 0.0;
        self.vibe_level_filter = 0.0;
        self.clip_counts = ClipCounts::default();
        self.sample_count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_no_clipping_at_1g() {
        let mut mon = VibrationMonitor::default_threshold();
        let accel = Vec3::<Body>::new(0.0, 0.0, -9.81);
        for _ in 0..100 {
            mon.process_sample(&accel);
        }
        assert_eq!(mon.total_clip_count(), 0);
        assert!(!mon.output().excessive);
    }

    #[test]
    fn test_clipping_at_high_accel() {
        let mut mon = VibrationMonitor::default_threshold();
        let accel = Vec3::<Body>::new(30.0, 0.0, -9.81);
        mon.process_sample(&accel);
        assert_eq!(mon.clip_counts.x, 1);
        assert_eq!(mon.clip_counts.y, 0);
    }

    #[test]
    fn test_vibration_floor_converges() {
        let mut mon = VibrationMonitor::default_threshold();
        let accel = Vec3::<Body>::new(0.0, 0.0, -9.81);
        for _ in 0..10000 {
            mon.process_sample(&accel);
        }
        // Floor should converge near 9.81
        let out = mon.output();
        assert!((out.vibe_floor - 9.81).abs() < 0.1, "floor = {}", out.vibe_floor);
        // Level should be near zero for constant input
        assert!(out.vibe_level < 0.01, "level = {}", out.vibe_level);
    }
}

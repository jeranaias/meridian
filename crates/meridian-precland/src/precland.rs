//! Precision landing controller with state machine, retry logic, and XY descent gate.
//!
//! Source: ArduPilot `AC_PrecLand/AC_PrecLand.cpp`

use crate::kalman::PosVelKF;
use crate::inertial_history::{InertialHistory, InertialSample};
use crate::backend::PrecLandTarget;

/// Default measurement noise variance (m^2).
const DEFAULT_MEAS_NOISE: f32 = 0.01;

/// Target tracking state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetState {
    /// Target has never been seen.
    NeverFound,
    /// Target was found but is currently out of range.
    Searching,
    /// Target was recently acquired, building confidence.
    Acquiring,
    /// Target is being actively tracked with good confidence.
    Tracking,
    /// Target was recently lost (may recover).
    RecentlyLost,
}

/// Precision landing controller.
pub struct PrecisionLand {
    /// 2-state Kalman filter for X axis.
    pub kf_x: PosVelKF,
    /// 2-state Kalman filter for Y axis.
    pub kf_y: PosVelKF,
    /// Inertial history for lag compensation (64 samples ~ 640ms at 100Hz).
    pub inertial_history: InertialHistory<64>,
    /// Current target tracking state.
    pub target_state: TargetState,
    /// Timestamp of the last valid target measurement (ms).
    pub last_target_ms: u32,
    /// Measurement noise variance.
    pub meas_noise: f32,
    /// Sensor lag parameter (seconds). Used to look back in inertial history.
    pub lag_s: f32,
    /// Maximum XY distance to allow descent (meters). 0 = disabled.
    pub xy_dist_max_m: f32,
    /// Whether strict mode is enabled (abort landing if target lost).
    pub strict_mode: bool,
    /// Maximum retry attempts.
    pub retry_max: u8,
    /// Retry timeout (ms).
    pub retry_timeout_ms: u32,
    /// Timeout before declaring target lost (ms).
    pub target_timeout_ms: u32,

    // Internal state
    est_pos_x: f32,
    est_pos_y: f32,
    acquire_count: u8,
    retry_count: u8,
    lost_since_ms: u32,
}

impl PrecisionLand {
    /// Create a new precision landing controller.
    pub fn new() -> Self {
        Self {
            kf_x: PosVelKF::new(0.1, 0.5),
            kf_y: PosVelKF::new(0.1, 0.5),
            inertial_history: InertialHistory::new(),
            target_state: TargetState::NeverFound,
            last_target_ms: 0,
            meas_noise: DEFAULT_MEAS_NOISE,
            lag_s: 0.02,  // 20ms default
            xy_dist_max_m: 0.0,
            strict_mode: false,
            retry_max: 3,
            retry_timeout_ms: 5000,
            target_timeout_ms: 2000,
            est_pos_x: 0.0,
            est_pos_y: 0.0,
            acquire_count: 0,
            retry_count: 0,
            lost_since_ms: 0,
        }
    }

    /// Reset the controller state.
    pub fn reset(&mut self) {
        self.kf_x.reset();
        self.kf_y.reset();
        self.inertial_history.clear();
        self.target_state = TargetState::NeverFound;
        self.last_target_ms = 0;
        self.est_pos_x = 0.0;
        self.est_pos_y = 0.0;
        self.acquire_count = 0;
        self.retry_count = 0;
        self.lost_since_ms = 0;
    }

    /// Feed vehicle velocity for inertial history (call at IMU rate).
    pub fn feed_velocity(&mut self, vel_x: f32, vel_y: f32, now_ms: u32) {
        self.inertial_history.push(InertialSample {
            vel_x,
            vel_y,
            timestamp_ms: now_ms,
        });
    }

    /// Process a new landing target detection.
    pub fn handle_target(&mut self, target: &PrecLandTarget, altitude_m: f32, now_ms: u32) {
        // Convert angle measurement to position offset using altitude
        let meas_x = libm::tanf(target.angle_x) * altitude_m;
        let meas_y = libm::tanf(target.angle_y) * altitude_m;

        // Compensate for sensor lag using inertial history
        let lag_ms = (self.lag_s * 1000.0) as u32;
        let sensor_time = if target.timestamp_ms > 0 {
            target.timestamp_ms
        } else if now_ms > lag_ms {
            now_ms - lag_ms
        } else {
            0
        };

        let (dx, dy) = if sensor_time < now_ms {
            self.inertial_history.compute_displacement(sensor_time, now_ms)
        } else {
            (0.0, 0.0)
        };

        let adjusted_x = meas_x + dx;
        let adjusted_y = meas_y + dy;

        // Predict KFs forward
        let dt = if self.last_target_ms > 0 && now_ms > self.last_target_ms {
            (now_ms - self.last_target_ms) as f32 / 1000.0
        } else {
            0.01
        };

        self.kf_x.predict(dt);
        self.kf_y.predict(dt);

        // Update KFs with measurement (NIS rejection active)
        self.kf_x.update(adjusted_x, self.meas_noise);
        self.kf_y.update(adjusted_y, self.meas_noise);

        self.est_pos_x = self.kf_x.pos;
        self.est_pos_y = self.kf_y.pos;
        self.last_target_ms = now_ms;

        // State machine transition
        match self.target_state {
            TargetState::NeverFound | TargetState::Searching => {
                self.target_state = TargetState::Acquiring;
                self.acquire_count = 1;
            }
            TargetState::Acquiring => {
                self.acquire_count += 1;
                if self.acquire_count >= 5 {
                    self.target_state = TargetState::Tracking;
                }
            }
            TargetState::Tracking => {
                // Stay tracking
            }
            TargetState::RecentlyLost => {
                // Re-acquired
                self.target_state = TargetState::Tracking;
            }
        }
    }

    /// Run the prediction step (call when no measurement available).
    pub fn predict(&mut self, dt: f32, now_ms: u32) {
        if self.target_state == TargetState::NeverFound {
            return;
        }

        let elapsed = now_ms.wrapping_sub(self.last_target_ms);

        if elapsed > self.target_timeout_ms {
            match self.target_state {
                TargetState::Tracking | TargetState::Acquiring => {
                    self.target_state = TargetState::RecentlyLost;
                    self.lost_since_ms = now_ms;
                }
                TargetState::RecentlyLost => {
                    let lost_elapsed = now_ms.wrapping_sub(self.lost_since_ms);
                    if lost_elapsed > self.retry_timeout_ms {
                        if self.retry_count < self.retry_max {
                            self.retry_count += 1;
                            self.target_state = TargetState::Searching;
                        } else {
                            self.target_state = TargetState::Searching;
                        }
                    }
                }
                _ => {}
            }
        }

        if self.target_state == TargetState::Tracking {
            self.kf_x.predict(dt);
            self.kf_y.predict(dt);
            self.est_pos_x = self.kf_x.pos;
            self.est_pos_y = self.kf_y.pos;
        }
    }

    /// Check if descent is allowed (XY distance gate).
    pub fn descent_allowed(&self) -> bool {
        if self.target_state != TargetState::Tracking {
            return false;
        }
        if self.xy_dist_max_m <= 0.0 {
            return true; // gate disabled
        }
        let dist = libm::sqrtf(self.est_pos_x * self.est_pos_x + self.est_pos_y * self.est_pos_y);
        dist <= self.xy_dist_max_m
    }

    /// Whether retry attempts are exhausted.
    pub fn retries_exhausted(&self) -> bool {
        self.retry_count >= self.retry_max && self.target_state == TargetState::Searching
    }

    /// Get the estimated landing target position (x, y) in meters.
    pub fn get_target_position(&self) -> Option<(f32, f32)> {
        if self.target_state == TargetState::Tracking {
            Some((self.est_pos_x, self.est_pos_y))
        } else {
            None
        }
    }

    /// Get the estimated target velocity (vx, vy) in m/s.
    pub fn get_target_velocity(&self) -> Option<(f32, f32)> {
        if self.target_state == TargetState::Tracking {
            Some((self.kf_x.vel, self.kf_y.vel))
        } else {
            None
        }
    }

    /// Whether a target is currently being tracked.
    pub fn target_acquired(&self) -> bool {
        self.target_state == TargetState::Tracking
    }
}

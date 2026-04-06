//! Output predictor: bridges delayed EKF state to IMU-rate output.
//!
//! The EKF runs on delayed sensor data (~220ms for GPS). The output predictor:
//! 1. Integrates IMU at full rate to maintain a current-time state estimate
//! 2. When the EKF fuses sensors, computes the correction between EKF and delayed output
//! 3. Applies corrections to all buffered outputs via PI feedback
//!
//! Source: AP_NavEKF3_core.cpp calcOutputStates() lines 819-928

use meridian_math::{Vec3, Quaternion, Rotation};
use meridian_math::frames::{NED, Body};
use meridian_types::time::Instant;
use crate::predict::{ImuDelta, GRAVITY};

/// Maximum IMU buffer length (~250ms at 400Hz).
pub const IMU_BUFFER_LEN: usize = 100;

/// Output state at one time step.
#[derive(Debug, Clone, Copy)]
pub struct OutputState {
    pub quat: Quaternion,
    pub velocity: Vec3<NED>,
    pub position: Vec3<NED>,
    pub time: Instant,
}

impl Default for OutputState {
    fn default() -> Self {
        Self {
            quat: Quaternion::identity(),
            velocity: Vec3::zero(),
            position: Vec3::zero(),
            time: Instant::ZERO,
        }
    }
}

/// Ring buffer for IMU deltas (used for delayed-state prediction).
pub struct ImuBuffer {
    buf: [ImuDelta; IMU_BUFFER_LEN],
    head: usize, // next write position
    count: usize,
}

impl ImuBuffer {
    pub fn new() -> Self {
        Self {
            buf: [ImuDelta {
                del_ang: Vec3::zero(),
                del_vel: Vec3::zero(),
                del_ang_dt: 0.0,
                del_vel_dt: 0.0,
            }; IMU_BUFFER_LEN],
            head: 0,
            count: 0,
        }
    }

    pub fn push(&mut self, imu: ImuDelta) {
        self.buf[self.head] = imu;
        self.head = (self.head + 1) % IMU_BUFFER_LEN;
        if self.count < IMU_BUFFER_LEN {
            self.count += 1;
        }
    }

    /// Get the oldest element (delayed by buffer length).
    pub fn oldest(&self) -> Option<&ImuDelta> {
        if self.count == 0 { return None; }
        let idx = if self.count < IMU_BUFFER_LEN {
            0
        } else {
            self.head // head points to oldest when full
        };
        Some(&self.buf[idx])
    }

    /// Get the oldest element index.
    pub fn oldest_index(&self) -> usize {
        if self.count < IMU_BUFFER_LEN { 0 } else { self.head }
    }

    pub fn len(&self) -> usize {
        self.count
    }

    pub fn is_full(&self) -> bool {
        self.count >= IMU_BUFFER_LEN
    }
}

/// Ring buffer for output states (parallel to IMU buffer).
pub struct OutputBuffer {
    buf: [OutputState; IMU_BUFFER_LEN],
    head: usize,
    count: usize,
}

impl OutputBuffer {
    pub fn new() -> Self {
        Self {
            buf: [OutputState::default(); IMU_BUFFER_LEN],
            head: 0,
            count: 0,
        }
    }

    pub fn push(&mut self, state: OutputState) {
        self.buf[self.head] = state;
        self.head = (self.head + 1) % IMU_BUFFER_LEN;
        if self.count < IMU_BUFFER_LEN {
            self.count += 1;
        }
    }

    /// Get the oldest output state (at the delayed time horizon).
    pub fn oldest(&self) -> &OutputState {
        let idx = if self.count < IMU_BUFFER_LEN { 0 } else { self.head };
        &self.buf[idx]
    }

    /// Get the newest output state (current time).
    pub fn newest(&self) -> &OutputState {
        let idx = if self.head == 0 { IMU_BUFFER_LEN - 1 } else { self.head - 1 };
        &self.buf[idx]
    }

    /// Apply a velocity and position correction to ALL stored states.
    /// Source: AP_NavEKF3_core.cpp lines 917-926
    pub fn apply_correction(&mut self, vel_corr: Vec3<NED>, pos_corr: Vec3<NED>) {
        for i in 0..self.count {
            let idx = if self.count < IMU_BUFFER_LEN {
                i
            } else {
                (self.head + i) % IMU_BUFFER_LEN
            };
            self.buf[idx].velocity = self.buf[idx].velocity + vel_corr;
            self.buf[idx].position = self.buf[idx].position + pos_corr;
        }
    }

    /// Apply a quaternion correction to ALL stored states.
    pub fn apply_quat_correction(&mut self, correction: Quaternion) {
        for i in 0..self.count {
            let idx = if self.count < IMU_BUFFER_LEN {
                i
            } else {
                (self.head + i) % IMU_BUFFER_LEN
            };
            self.buf[idx].quat = correction * self.buf[idx].quat;
            self.buf[idx].quat.normalize();
        }
    }
}

/// The output predictor.
pub struct OutputPredictor {
    pub imu_buf: ImuBuffer,
    pub output_buf: OutputBuffer,

    /// Current (newest) output state — updated at IMU rate.
    pub current: OutputState,

    /// Attitude correction (small angle, applied each IMU step).
    /// Source: AP_NavEKF3_core.cpp line 898
    del_ang_correction: Vec3<Body>,

    /// Vertical complementary filter state.
    /// Source: AP_NavEKF3_core.cpp lines 844-851
    vert_comp_filt_pos: f32,
    vert_comp_filt_vel: f32,
    vert_comp_filt_acc: f32,

    /// Velocity error integrator (for PI feedback).
    vel_err_integral: Vec3<NED>,
    /// Position error integrator.
    pos_err_integral: Vec3<NED>,
}

impl OutputPredictor {
    pub fn new() -> Self {
        Self {
            imu_buf: ImuBuffer::new(),
            output_buf: OutputBuffer::new(),
            current: OutputState::default(),
            del_ang_correction: Vec3::zero(),
            vert_comp_filt_pos: 0.0,
            vert_comp_filt_vel: 0.0,
            vert_comp_filt_acc: 0.0,
            vel_err_integral: Vec3::zero(),
            pos_err_integral: Vec3::zero(),
        }
    }

    /// Process a new IMU sample at full IMU rate.
    /// Integrates the output state forward and stores in buffer.
    ///
    /// `gyro_bias` and `accel_bias`: current EKF bias estimates.
    pub fn update_imu(
        &mut self,
        imu: &ImuDelta,
        time: Instant,
        gyro_bias: &Vec3<Body>,
        accel_bias: &Vec3<Body>,
    ) {
        // Store IMU in ring buffer (for delayed EKF prediction)
        self.imu_buf.push(*imu);

        // Bias-corrected IMU
        let del_ang = imu.del_ang - *gyro_bias * imu.del_ang_dt + self.del_ang_correction;
        let del_vel = imu.del_vel - *accel_bias * imu.del_vel_dt;

        // Integrate attitude
        let angle = del_ang.length();
        if angle > 1e-12 {
            let axis = del_ang.normalized();
            let dq = Quaternion::from_axis_angle(&[axis.x, axis.y, axis.z], angle);
            self.current.quat = self.current.quat * dq;
            self.current.quat.normalize();
        }

        // Rotate delta velocity to NED
        let body_to_ned: Rotation<Body, NED> = Rotation::from_quaternion(self.current.quat);
        let del_vel_ned = body_to_ned.rotate(del_vel);
        let del_vel_nav = del_vel_ned + Vec3::<NED>::new(0.0, 0.0, GRAVITY * imu.del_vel_dt);

        // Integrate velocity
        let last_vel = self.current.velocity;
        self.current.velocity = self.current.velocity + del_vel_nav;

        // Integrate position (trapezoidal)
        self.current.position = self.current.position
            + (self.current.velocity + last_vel) * (imu.del_vel_dt * 0.5);

        self.current.time = time;

        // Store in output buffer
        self.output_buf.push(self.current);
    }

    /// Apply EKF correction when sensor fusion occurs.
    /// Computes error between EKF state and delayed output, distributes correction.
    ///
    /// Source: AP_NavEKF3_core.cpp calcOutputStates() lines 884-926
    pub fn apply_ekf_correction(
        &mut self,
        ekf_quat: &Quaternion,
        ekf_velocity: &Vec3<NED>,
        ekf_position: &Vec3<NED>,
        dt_ekf: f32,
        tau_vel_pos: f32,
        hrt_filt_freq: f32,
    ) {
        if self.output_buf.count == 0 { return; }

        let delayed = *self.output_buf.oldest();

        // ── Attitude correction ──
        // Source: lines 884-898
        // q_err = q_ekf / q_delayed_output = q_ekf * q_delayed^-1
        let q_err = *ekf_quat * delayed.quat.inverse();
        let q_err = q_err.normalized();

        // Convert to small-angle error vector
        let scaler = if q_err.w >= 0.0 { 2.0 } else { -2.0 };
        let delta_ang_err = Vec3::<Body>::new(
            scaler * q_err.x,
            scaler * q_err.y,
            scaler * q_err.z,
        );

        // Time delay from oldest to newest
        let time_delay = if self.imu_buf.is_full() {
            (self.current.time.as_micros().saturating_sub(delayed.time.as_micros())) as f32 * 1e-6
        } else {
            dt_ekf.max(0.01)
        };
        // R2: Correct attitude correction gain.
        // ArduPilot formula: 2*PI * hrt_filt_freq * dt / time_delay
        // Source: AP_NavEKF3_core.cpp line 898
        // Previous formula was 0.5/time_delay * dt, which is ~25x too small
        // for the default hrt_filt_freq=2.0 Hz.
        let error_gain = 2.0 * core::f32::consts::PI * hrt_filt_freq * dt_ekf / time_delay.max(0.001);

        self.del_ang_correction = delta_ang_err * error_gain;

        // ── Velocity and position correction ──
        // Source: lines 901-912
        let vel_err = *ekf_velocity - delayed.velocity;
        let pos_err = *ekf_position - delayed.position;

        let vel_pos_gain = dt_ekf / tau_vel_pos.max(dt_ekf);
        let vel_pos_gain_sq = vel_pos_gain * vel_pos_gain;

        self.vel_err_integral = self.vel_err_integral + vel_err;
        self.pos_err_integral = self.pos_err_integral + pos_err;

        // PI correction
        let vel_correction = vel_err * vel_pos_gain + self.vel_err_integral * (vel_pos_gain_sq * 0.1);
        let pos_correction = pos_err * vel_pos_gain + self.pos_err_integral * (vel_pos_gain_sq * 0.1);

        // Apply correction to ALL buffered states
        // Source: lines 917-926
        self.output_buf.apply_correction(vel_correction, pos_correction);

        // Update current from newest buffered (after correction)
        self.current = *self.output_buf.newest();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_imu_buffer_fifo() {
        let mut buf = ImuBuffer::new();
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
        assert!(!buf.is_full());

        // Oldest should be the first pushed
        let oldest = buf.oldest().unwrap();
        assert!((oldest.del_ang.x - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_imu_buffer_wraparound() {
        let mut buf = ImuBuffer::new();
        for i in 0..150 {
            buf.push(ImuDelta {
                del_ang: Vec3::new(i as f32, 0.0, 0.0),
                del_vel: Vec3::zero(),
                del_ang_dt: 0.0025,
                del_vel_dt: 0.0025,
            });
        }
        assert!(buf.is_full());
        assert_eq!(buf.len(), IMU_BUFFER_LEN);

        // Oldest should be from push #50 (first 100 were overwritten, then #50-149 remain)
        let oldest = buf.oldest().unwrap();
        assert!((oldest.del_ang.x - 50.0).abs() < 1e-6,
            "Oldest should be 50, got {}", oldest.del_ang.x);
    }

    #[test]
    fn test_output_predictor_stationary() {
        let mut pred = OutputPredictor::new();
        let bias_g = Vec3::<Body>::zero();
        let bias_a = Vec3::<Body>::zero();

        // Feed hover IMU for 0.5s at 400Hz
        for i in 0..200 {
            let imu = ImuDelta {
                del_ang: Vec3::zero(),
                del_vel: Vec3::<Body>::new(0.0, 0.0, -GRAVITY * 0.0025),
                del_ang_dt: 0.0025,
                del_vel_dt: 0.0025,
            };
            let t = Instant::from_micros(i * 2500);
            pred.update_imu(&imu, t, &bias_g, &bias_a);
        }

        // Position should be near zero (hovering)
        assert!(pred.current.position.length() < 1.0,
            "Position should be near zero: {:?}", pred.current.position);
        assert!(pred.current.velocity.length() < 0.5,
            "Velocity should be near zero: {:?}", pred.current.velocity);
    }

    #[test]
    fn test_correction_applied() {
        let mut pred = OutputPredictor::new();
        let bias_g = Vec3::<Body>::zero();
        let bias_a = Vec3::<Body>::zero();

        // Fill buffer
        for i in 0..100 {
            let imu = ImuDelta {
                del_ang: Vec3::zero(),
                del_vel: Vec3::<Body>::new(0.0, 0.0, -GRAVITY * 0.0025),
                del_ang_dt: 0.0025,
                del_vel_dt: 0.0025,
            };
            pred.update_imu(&imu, Instant::from_micros(i * 2500), &bias_g, &bias_a);
        }

        let pos_before = pred.current.position;

        // Apply a correction (EKF says we should be at position [1, 0, 0])
        pred.apply_ekf_correction(
            &Quaternion::identity(),
            &Vec3::zero(),
            &Vec3::<NED>::new(1.0, 0.0, 0.0),
            0.0025,
            0.25,
            2.0, // hrt_filt_freq
        );

        // Position should have shifted toward [1, 0, 0]
        assert!(pred.current.position.x > pos_before.x,
            "Position X should increase after correction");
    }
}

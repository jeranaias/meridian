//! 2-state (position + velocity) Kalman filter for precision landing.
//!
//! One instance per axis (X and Y). Completely separate from the main EKF.
//! Source: ArduPilot `AC_PrecLand/AC_PrecLand.cpp` — inline KF logic.

/// 2-state Kalman filter: position and velocity.
/// Includes NIS (Normalized Innovation Squared) outlier rejection.
#[derive(Debug, Clone)]
pub struct PosVelKF {
    /// Estimated position (meters, NED body-relative).
    pub pos: f32,
    /// Estimated velocity (m/s).
    pub vel: f32,
    /// 2x2 covariance matrix [[P00, P01], [P10, P11]].
    pub p: [[f32; 2]; 2],
    /// Process noise for position (m^2/s).
    pub q_pos: f32,
    /// Process noise for velocity (m^2/s^3).
    pub q_vel: f32,
    /// NIS threshold for outlier rejection (chi-squared, 1 DoF, 99.7% = 9.0).
    pub nis_threshold: f32,
    /// Consecutive outlier count for forced accept.
    nis_outlier_count: u8,
    /// Max consecutive outliers before forced accept.
    pub nis_max_outliers: u8,
}

impl PosVelKF {
    /// Create a new filter with initial uncertainties.
    pub fn new(q_pos: f32, q_vel: f32) -> Self {
        Self {
            pos: 0.0,
            vel: 0.0,
            p: [
                [1.0, 0.0],
                [0.0, 1.0],
            ],
            q_pos,
            q_vel,
            nis_threshold: 9.0,
            nis_outlier_count: 0,
            nis_max_outliers: 3,
        }
    }

    /// Reset the filter state.
    pub fn reset(&mut self) {
        self.pos = 0.0;
        self.vel = 0.0;
        self.p = [
            [1.0, 0.0],
            [0.0, 1.0],
        ];
        self.nis_outlier_count = 0;
    }

    /// Predict step: propagate state forward by `dt` seconds.
    pub fn predict(&mut self, dt: f32) {
        if dt <= 0.0 {
            return;
        }

        self.pos += self.vel * dt;

        let p00 = self.p[0][0] + dt * self.p[1][0];
        let p01 = self.p[0][1] + dt * self.p[1][1];
        let p10 = self.p[1][0];
        let p11 = self.p[1][1];

        self.p[0][0] = p00 + dt * p01 + self.q_pos * dt;
        self.p[0][1] = p01;
        self.p[1][0] = p10 + dt * p11;
        self.p[1][1] = p11 + self.q_vel * dt;
    }

    /// Update step with NIS-based outlier rejection.
    ///
    /// Returns true if the measurement was accepted, false if rejected as outlier.
    pub fn update(&mut self, measurement: f32, r: f32) -> bool {
        let y = measurement - self.pos;

        let s = self.p[0][0] + r;
        if s.abs() < 1e-12 {
            return false;
        }
        let s_inv = 1.0 / s;

        // Normalized Innovation Squared
        let nis = y * y * s_inv;

        // Outlier rejection
        if nis > self.nis_threshold {
            self.nis_outlier_count += 1;
            if self.nis_outlier_count < self.nis_max_outliers {
                return false; // reject outlier
            }
            // Forced accept after max consecutive outliers
        }

        // Accept measurement
        self.nis_outlier_count = 0;

        let k0 = self.p[0][0] * s_inv;
        let k1 = self.p[1][0] * s_inv;

        self.pos += k0 * y;
        self.vel += k1 * y;

        let new_p00 = self.p[0][0] - k0 * self.p[0][0];
        let new_p01 = self.p[0][1] - k0 * self.p[0][1];
        let new_p10 = self.p[1][0] - k1 * self.p[0][0];
        let new_p11 = self.p[1][1] - k1 * self.p[0][1];

        self.p[0][0] = new_p00;
        self.p[0][1] = new_p01;
        self.p[1][0] = new_p10;
        self.p[1][1] = new_p11;

        true
    }
}

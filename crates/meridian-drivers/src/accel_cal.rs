//! Minimal 6-point accelerometer calibration (AP_AccelCal equivalent).
//!
//! Source: ArduPilot AP_AccelCal/AP_AccelCal.cpp
//!
//! The vehicle is placed in 6 orientations (level, nose-up, nose-down,
//! left-side, right-side, inverted) and the accelerometer reads gravity
//! in each position. From 6 known gravity vectors we compute 3-axis
//! offset and scale corrections.
//!
//! This stub implements the offset-only model (no scale). A full
//! implementation would add 3-axis scale factors via least-squares.

/// The 6 calibration orientations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalOrientation {
    Level,        // Z down  → accel ≈ (0, 0, -g)
    NoseDown,     // X down  → accel ≈ (-g, 0, 0)
    NoseUp,       // X up    → accel ≈ (+g, 0, 0)
    LeftSide,     // Y down  → accel ≈ (0, -g, 0)
    RightSide,    // Y up    → accel ≈ (0, +g, 0)
    Inverted,     // Z up    → accel ≈ (0, 0, +g)
}

impl CalOrientation {
    /// Iterator-like: returns the next orientation in the sequence.
    pub fn next(self) -> Option<Self> {
        match self {
            Self::Level => Some(Self::NoseDown),
            Self::NoseDown => Some(Self::NoseUp),
            Self::NoseUp => Some(Self::LeftSide),
            Self::LeftSide => Some(Self::RightSide),
            Self::RightSide => Some(Self::Inverted),
            Self::Inverted => None,
        }
    }

    /// Human-readable name for GCS prompt.
    pub fn name(self) -> &'static str {
        match self {
            Self::Level => "LEVEL",
            Self::NoseDown => "NOSE DOWN",
            Self::NoseUp => "NOSE UP",
            Self::LeftSide => "LEFT SIDE",
            Self::RightSide => "RIGHT SIDE",
            Self::Inverted => "INVERTED",
        }
    }
}

const GRAVITY: f32 = 9.80665;

/// Accelerometer calibration state machine.
#[derive(Debug, Clone)]
pub struct AccelCal {
    /// Current orientation being sampled.
    pub current_step: Option<CalOrientation>,
    /// Accumulated samples per orientation: [sum_x, sum_y, sum_z, count].
    samples: [[f64; 4]; 6],
    /// Computed offsets [x, y, z] in m/s^2. Subtract from raw reading.
    pub offsets: [f32; 3],
    /// Whether calibration is complete and offsets are valid.
    pub calibrated: bool,
}

impl AccelCal {
    pub fn new() -> Self {
        Self {
            current_step: None,
            samples: [[0.0; 4]; 6],
            offsets: [0.0; 3],
            calibrated: false,
        }
    }

    /// Start a new calibration sequence.
    pub fn start(&mut self) {
        self.current_step = Some(CalOrientation::Level);
        self.samples = [[0.0; 4]; 6];
        self.offsets = [0.0; 3];
        self.calibrated = false;
    }

    /// Feed one accelerometer sample for the current orientation.
    /// Call repeatedly while the vehicle is held steady in the prompted position.
    pub fn feed_sample(&mut self, accel_x: f32, accel_y: f32, accel_z: f32) {
        if let Some(step) = self.current_step {
            let idx = step as usize;
            self.samples[idx][0] += accel_x as f64;
            self.samples[idx][1] += accel_y as f64;
            self.samples[idx][2] += accel_z as f64;
            self.samples[idx][3] += 1.0;
        }
    }

    /// How many samples have been collected for the current step.
    pub fn current_count(&self) -> u32 {
        if let Some(step) = self.current_step {
            self.samples[step as usize][3] as u32
        } else {
            0
        }
    }

    /// Accept the samples for the current orientation and advance to the next.
    /// Returns `true` if more orientations remain, `false` if calibration
    /// is complete (offsets are now valid).
    pub fn accept_step(&mut self) -> bool {
        if let Some(step) = self.current_step {
            // Must have at least 10 samples for a reasonable average
            if self.samples[step as usize][3] < 10.0 {
                return true; // not enough samples, stay on this step
            }
            match step.next() {
                Some(next) => {
                    self.current_step = Some(next);
                    true
                }
                None => {
                    // All 6 orientations captured — compute offsets
                    self.compute_offsets();
                    self.current_step = None;
                    self.calibrated = true;
                    false
                }
            }
        } else {
            false
        }
    }

    /// Compute 3-axis offsets from the 6 orientation averages.
    ///
    /// For each axis, we have two readings: one with gravity along +axis
    /// and one with gravity along -axis. The offset is the average of the
    /// two (which should cancel gravity, leaving only the bias).
    ///
    /// offset_x = (avg_nose_up_x + avg_nose_down_x) / 2
    /// offset_y = (avg_right_x + avg_left_x) / 2
    /// offset_z = (avg_level_z + avg_inverted_z) / 2
    fn compute_offsets(&mut self) {
        let avg = |idx: usize, axis: usize| -> f32 {
            let s = &self.samples[idx];
            if s[3] > 0.0 { (s[axis] / s[3]) as f32 } else { 0.0 }
        };

        // X axis: NoseUp sees +g on X, NoseDown sees -g on X
        let x_plus = avg(CalOrientation::NoseUp as usize, 0);    // ≈ +g + offset
        let x_minus = avg(CalOrientation::NoseDown as usize, 0);  // ≈ -g + offset
        self.offsets[0] = (x_plus + x_minus) / 2.0;

        // Y axis: RightSide sees +g on Y, LeftSide sees -g on Y
        let y_plus = avg(CalOrientation::RightSide as usize, 1);
        let y_minus = avg(CalOrientation::LeftSide as usize, 1);
        self.offsets[1] = (y_plus + y_minus) / 2.0;

        // Z axis: Inverted sees +g on Z, Level sees -g on Z
        let z_plus = avg(CalOrientation::Inverted as usize, 2);
        let z_minus = avg(CalOrientation::Level as usize, 2);
        self.offsets[2] = (z_plus + z_minus) / 2.0;
    }

    /// Apply calibration offsets to a raw accelerometer reading.
    pub fn correct(&self, raw_x: f32, raw_y: f32, raw_z: f32) -> (f32, f32, f32) {
        if !self.calibrated {
            return (raw_x, raw_y, raw_z);
        }
        (
            raw_x - self.offsets[0],
            raw_y - self.offsets[1],
            raw_z - self.offsets[2],
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_6_point_calibration() {
        let mut cal = AccelCal::new();
        cal.start();
        assert_eq!(cal.current_step, Some(CalOrientation::Level));

        // Simulate with a known bias of (0.1, -0.2, 0.3) m/s^2
        let bias = [0.1f32, -0.2, 0.3];
        let g = GRAVITY;

        let orientations = [
            (CalOrientation::Level, [0.0 + bias[0], 0.0 + bias[1], -g + bias[2]]),
            (CalOrientation::NoseDown, [-g + bias[0], 0.0 + bias[1], 0.0 + bias[2]]),
            (CalOrientation::NoseUp, [g + bias[0], 0.0 + bias[1], 0.0 + bias[2]]),
            (CalOrientation::LeftSide, [0.0 + bias[0], -g + bias[1], 0.0 + bias[2]]),
            (CalOrientation::RightSide, [0.0 + bias[0], g + bias[1], 0.0 + bias[2]]),
            (CalOrientation::Inverted, [0.0 + bias[0], 0.0 + bias[1], g + bias[2]]),
        ];

        for (orient, accel) in &orientations {
            assert_eq!(cal.current_step, Some(*orient));
            for _ in 0..20 {
                cal.feed_sample(accel[0], accel[1], accel[2]);
            }
            cal.accept_step();
        }

        assert!(cal.calibrated);
        assert!((cal.offsets[0] - 0.1).abs() < 0.01, "X offset: {}", cal.offsets[0]);
        assert!((cal.offsets[1] - (-0.2)).abs() < 0.01, "Y offset: {}", cal.offsets[1]);
        assert!((cal.offsets[2] - 0.3).abs() < 0.01, "Z offset: {}", cal.offsets[2]);

        // Verify correction
        let (cx, cy, cz) = cal.correct(0.1, -0.2, -9.80665 + 0.3);
        assert!((cx).abs() < 0.01);
        assert!((cy).abs() < 0.01);
        assert!((cz + g).abs() < 0.01);
    }

    #[test]
    fn test_uncalibrated_passthrough() {
        let cal = AccelCal::new();
        let (x, y, z) = cal.correct(1.0, 2.0, 3.0);
        assert_eq!(x, 1.0);
        assert_eq!(y, 2.0);
        assert_eq!(z, 3.0);
    }
}

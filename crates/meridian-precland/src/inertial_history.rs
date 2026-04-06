//! Inertial history ring buffer for sensor lag compensation.
//!
//! Stores recent vehicle velocity samples so that when a delayed sensor
//! measurement arrives, we can compensate for the vehicle motion that
//! occurred between the sensor timestamp and now.
//!
//! Source: ArduPilot `AC_PrecLand/AC_PrecLand.cpp` — inertial history logic.

/// A single inertial sample.
#[derive(Debug, Clone, Copy, Default)]
pub struct InertialSample {
    /// Vehicle velocity in NED X (m/s).
    pub vel_x: f32,
    /// Vehicle velocity in NED Y (m/s).
    pub vel_y: f32,
    /// Timestamp (ms).
    pub timestamp_ms: u32,
}

/// Ring buffer of inertial samples for lag compensation.
pub struct InertialHistory<const N: usize> {
    samples: [InertialSample; N],
    head: usize,
    count: usize,
}

impl<const N: usize> InertialHistory<N> {
    /// Create a new empty history buffer.
    pub fn new() -> Self {
        Self {
            samples: [InertialSample::default(); N],
            head: 0,
            count: 0,
        }
    }

    /// Push a new inertial sample.
    pub fn push(&mut self, sample: InertialSample) {
        self.samples[self.head] = sample;
        self.head = (self.head + 1) % N;
        if self.count < N {
            self.count += 1;
        }
    }

    /// Number of samples stored.
    pub fn len(&self) -> usize {
        self.count
    }

    /// Whether the buffer is empty.
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Clear all samples.
    pub fn clear(&mut self) {
        self.head = 0;
        self.count = 0;
    }

    /// Compute the total displacement (dx, dy) in meters from `from_ms` to `to_ms`
    /// by integrating stored velocity samples over that time range.
    ///
    /// This compensates for vehicle motion during sensor lag.
    pub fn compute_displacement(&self, from_ms: u32, to_ms: u32) -> (f32, f32) {
        if self.count < 2 || to_ms <= from_ms {
            return (0.0, 0.0);
        }

        let mut dx: f32 = 0.0;
        let mut dy: f32 = 0.0;

        // Iterate through samples in chronological order
        let start = if self.count < N { 0 } else { self.head };

        for i in 0..self.count.saturating_sub(1) {
            let idx_curr = (start + i) % N;
            let idx_next = (start + i + 1) % N;

            let curr = &self.samples[idx_curr];
            let next = &self.samples[idx_next];

            // Skip samples outside our time range
            if next.timestamp_ms <= from_ms || curr.timestamp_ms >= to_ms {
                continue;
            }

            // Clamp to the requested time window
            let t0 = if curr.timestamp_ms > from_ms {
                curr.timestamp_ms
            } else {
                from_ms
            };
            let t1 = if next.timestamp_ms < to_ms {
                next.timestamp_ms
            } else {
                to_ms
            };

            let dt_s = (t1 - t0) as f32 / 1000.0;

            // Use average velocity of the two samples
            let avg_vx = (curr.vel_x + next.vel_x) * 0.5;
            let avg_vy = (curr.vel_y + next.vel_y) * 0.5;

            dx += avg_vx * dt_s;
            dy += avg_vy * dt_s;
        }

        (dx, dy)
    }
}

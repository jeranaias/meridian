/// Monotonic timestamp in microseconds since boot.
/// No system clocks in the control path.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Instant(u64);

/// Duration in microseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Duration(u64);

impl Instant {
    pub const ZERO: Instant = Instant(0);

    #[inline]
    pub const fn from_micros(us: u64) -> Self {
        Self(us)
    }

    #[inline]
    pub const fn as_micros(&self) -> u64 {
        self.0
    }

    #[inline]
    pub fn elapsed_since(&self, earlier: Instant) -> Duration {
        Duration(self.0.saturating_sub(earlier.0))
    }

    #[inline]
    pub fn checked_add(&self, d: Duration) -> Option<Instant> {
        self.0.checked_add(d.0).map(Instant)
    }
}

impl Duration {
    pub const ZERO: Duration = Duration(0);

    #[inline]
    pub const fn from_micros(us: u64) -> Self {
        Self(us)
    }

    #[inline]
    pub const fn from_millis(ms: u64) -> Self {
        Self(ms * 1000)
    }

    #[inline]
    pub const fn from_secs(s: u64) -> Self {
        Self(s * 1_000_000)
    }

    #[inline]
    pub const fn from_hz(hz: u32) -> Self {
        Self(1_000_000 / hz as u64)
    }

    #[inline]
    pub const fn as_micros(&self) -> u64 {
        self.0
    }

    #[inline]
    pub const fn as_millis(&self) -> u64 {
        self.0 / 1000
    }

    #[inline]
    pub fn as_secs_f32(&self) -> f32 {
        self.0 as f32 * 1e-6
    }

    #[inline]
    pub fn as_secs_f64(&self) -> f64 {
        self.0 as f64 * 1e-6
    }
}

impl core::ops::Add<Duration> for Instant {
    type Output = Instant;
    fn add(self, rhs: Duration) -> Instant {
        Instant(self.0 + rhs.0)
    }
}

impl core::ops::Sub for Instant {
    type Output = Duration;
    fn sub(self, rhs: Instant) -> Duration {
        Duration(self.0.saturating_sub(rhs.0))
    }
}

impl core::ops::Add for Duration {
    type Output = Duration;
    fn add(self, rhs: Duration) -> Duration {
        Duration(self.0 + rhs.0)
    }
}

impl Default for Instant {
    fn default() -> Self {
        Self::ZERO
    }
}

impl Default for Duration {
    fn default() -> Self {
        Self::ZERO
    }
}

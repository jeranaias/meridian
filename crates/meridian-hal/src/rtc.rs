//! AP_RTC equivalent: Real-time clock from GPS time.
//!
//! Source: ArduPilot AP_RTC/AP_RTC.cpp
//!
//! Provides Unix timestamp by combining GPS time-of-week with a
//! system boot reference. Without GPS, timestamps are boot-relative.
//! Once GPS time is available, all timestamps become absolute UTC.

/// RTC source priority (lower = higher priority).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RtcSource {
    /// No time source available. Timestamps are boot-relative.
    None,
    /// Time from GPS receiver (most common source).
    Gps,
    /// Time from MAVLink SYSTEM_TIME message.
    Mavlink,
    /// Time from on-board hardware RTC (STM32 RTC peripheral).
    HardwareRtc,
}

/// Real-time clock manager.
///
/// Maintains a Unix timestamp by combining a high-resolution boot
/// timer with an absolute reference from GPS or MAVLink.
#[derive(Debug, Clone)]
pub struct Rtc {
    /// Unix epoch microseconds at the reference point.
    /// 0 = no valid time source yet.
    utc_reference_us: u64,
    /// System uptime microseconds when the reference was captured.
    boot_reference_us: u64,
    /// Whether we have a valid UTC reference.
    has_utc: bool,
    /// Source of the current reference.
    pub source: RtcSource,
}

impl Rtc {
    pub const fn new() -> Self {
        Self {
            utc_reference_us: 0,
            boot_reference_us: 0,
            has_utc: false,
            source: RtcSource::None,
        }
    }

    /// Set the UTC reference from GPS time.
    ///
    /// `gps_week`: GPS week number (weeks since 1980-01-06).
    /// `gps_tow_ms`: GPS time-of-week in milliseconds.
    /// `system_us`: current system uptime in microseconds.
    ///
    /// GPS epoch is 1980-01-06 00:00:00 UTC.
    /// Unix epoch is 1970-01-01 00:00:00 UTC.
    /// Offset: 315964800 seconds (10 years + leap days).
    /// GPS leap seconds as of 2024: 18 seconds ahead of UTC.
    pub fn set_from_gps(&mut self, gps_week: u16, gps_tow_ms: u32, system_us: u64) {
        const GPS_UNIX_OFFSET: u64 = 315_964_800; // seconds from Unix epoch to GPS epoch
        const GPS_LEAP_SECONDS: u64 = 18;          // GPS time is ahead of UTC by this much

        let gps_seconds = (gps_week as u64) * 604_800 + (gps_tow_ms as u64) / 1000;
        let utc_seconds = gps_seconds + GPS_UNIX_OFFSET - GPS_LEAP_SECONDS;
        let utc_us = utc_seconds * 1_000_000 + ((gps_tow_ms as u64 % 1000) * 1000);

        self.utc_reference_us = utc_us;
        self.boot_reference_us = system_us;
        self.has_utc = true;
        self.source = RtcSource::Gps;
    }

    /// Set the UTC reference from a MAVLink SYSTEM_TIME message.
    ///
    /// `unix_us`: Unix timestamp in microseconds from the GCS.
    /// `system_us`: current system uptime in microseconds.
    pub fn set_from_mavlink(&mut self, unix_us: u64, system_us: u64) {
        // Only accept MAVLink time if we don't already have GPS time
        if self.source == RtcSource::Gps {
            return;
        }
        self.utc_reference_us = unix_us;
        self.boot_reference_us = system_us;
        self.has_utc = true;
        self.source = RtcSource::Mavlink;
    }

    /// Get the current Unix timestamp in microseconds.
    ///
    /// `system_us`: current system uptime in microseconds.
    ///
    /// Returns `None` if no valid time source is available.
    /// Returns `Some(unix_us)` with the best available UTC time.
    pub fn unix_us(&self, system_us: u64) -> Option<u64> {
        if !self.has_utc {
            return None;
        }
        let elapsed = system_us.saturating_sub(self.boot_reference_us);
        Some(self.utc_reference_us + elapsed)
    }

    /// Get the current Unix timestamp in seconds.
    pub fn unix_secs(&self, system_us: u64) -> Option<u64> {
        self.unix_us(system_us).map(|us| us / 1_000_000)
    }

    /// Whether a valid UTC reference is available.
    pub fn has_utc(&self) -> bool {
        self.has_utc
    }

    /// Get system uptime in seconds from microsecond uptime counter.
    pub fn uptime_secs(system_us: u64) -> u32 {
        (system_us / 1_000_000) as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_no_source_returns_none() {
        let rtc = Rtc::new();
        assert!(rtc.unix_us(1_000_000).is_none());
        assert!(!rtc.has_utc());
    }

    #[test]
    fn test_gps_time_conversion() {
        let mut rtc = Rtc::new();
        // GPS week 2300, TOW = 0ms = start of that week
        // GPS week 2300 = 2300 * 604800 = 1,391,040,000 seconds since GPS epoch
        // + 315,964,800 (Unix offset) - 18 (leap seconds)
        // = 1,707,004,782 seconds Unix
        rtc.set_from_gps(2300, 0, 5_000_000); // 5s after boot

        assert!(rtc.has_utc());
        assert_eq!(rtc.source, RtcSource::Gps);

        // At system_us = 6_000_000 (1 second later), Unix time should be +1s
        let t1 = rtc.unix_secs(5_000_000).unwrap();
        let t2 = rtc.unix_secs(6_000_000).unwrap();
        assert_eq!(t2 - t1, 1);
    }

    #[test]
    fn test_mavlink_time() {
        let mut rtc = Rtc::new();
        let unix_ref = 1_700_000_000_000_000u64; // some Unix time in us
        rtc.set_from_mavlink(unix_ref, 1_000_000);

        assert!(rtc.has_utc());
        assert_eq!(rtc.source, RtcSource::Mavlink);

        let t = rtc.unix_us(2_000_000).unwrap();
        assert_eq!(t, unix_ref + 1_000_000); // 1 second elapsed
    }

    #[test]
    fn test_gps_takes_priority_over_mavlink() {
        let mut rtc = Rtc::new();
        rtc.set_from_mavlink(1_000_000_000_000_000, 0);
        assert_eq!(rtc.source, RtcSource::Mavlink);

        rtc.set_from_gps(2300, 0, 0);
        assert_eq!(rtc.source, RtcSource::Gps);

        // MAVLink should not override GPS
        rtc.set_from_mavlink(2_000_000_000_000_000, 100_000);
        assert_eq!(rtc.source, RtcSource::Gps);
    }

    #[test]
    fn test_uptime() {
        assert_eq!(Rtc::uptime_secs(5_500_000), 5);
        assert_eq!(Rtc::uptime_secs(0), 0);
    }
}

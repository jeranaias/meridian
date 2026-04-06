//! Geotagging — capture GPS + attitude at shutter trigger time.
//!
//! Source: ArduPilot `AP_Camera_Backend.cpp` geotag functionality.

/// A single geotag entry captured at the moment of shutter trigger.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeotagEntry {
    /// Latitude in degrees.
    pub lat: f64,
    /// Longitude in degrees.
    pub lon: f64,
    /// Altitude above home in meters (MSL).
    pub alt: f32,
    /// Altitude above ground level in meters (AGL).
    pub alt_agl: f32,
    /// Vehicle roll at trigger time (radians).
    pub roll: f32,
    /// Vehicle pitch at trigger time (radians).
    pub pitch: f32,
    /// Vehicle yaw at trigger time (radians).
    pub yaw: f32,
    /// Timestamp in milliseconds since boot.
    pub timestamp_ms: u32,
    /// Image index.
    pub image_index: u32,
}

impl Default for GeotagEntry {
    fn default() -> Self {
        Self {
            lat: 0.0,
            lon: 0.0,
            alt: 0.0,
            alt_agl: 0.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            timestamp_ms: 0,
            image_index: 0,
        }
    }
}

/// Ring buffer for storing geotag entries.
/// Fixed capacity to avoid heap allocation.
pub struct GeotagLog<const N: usize> {
    entries: [GeotagEntry; N],
    head: usize,
    count: usize,
}

impl<const N: usize> GeotagLog<N> {
    /// Create a new empty geotag log.
    pub fn new() -> Self {
        Self {
            entries: [GeotagEntry::default(); N],
            head: 0,
            count: 0,
        }
    }

    /// Record a geotag entry. Overwrites the oldest entry if full.
    pub fn push(&mut self, entry: GeotagEntry) {
        self.entries[self.head] = entry;
        self.head = (self.head + 1) % N;
        if self.count < N {
            self.count += 1;
        }
    }

    /// Number of entries stored.
    pub fn len(&self) -> usize {
        self.count
    }

    /// Whether the log is empty.
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// Get the most recently stored entry.
    pub fn last(&self) -> Option<&GeotagEntry> {
        if self.count == 0 {
            None
        } else {
            let idx = if self.head == 0 { N - 1 } else { self.head - 1 };
            Some(&self.entries[idx])
        }
    }

    /// Get entry by index (0 = oldest still in buffer).
    pub fn get(&self, index: usize) -> Option<&GeotagEntry> {
        if index >= self.count {
            return None;
        }
        let start = if self.count < N {
            0
        } else {
            self.head
        };
        let actual = (start + index) % N;
        Some(&self.entries[actual])
    }
}

#![no_std]

//! 3D proximity boundary for obstacle avoidance.
//!
//! Source: ArduPilot AP_Proximity + AC_Avoidance
//! 8 sectors × 5 layers = 40 faces in 3D boundary.
//! 3-sample median + IIR filter per face. 1s expiry.

use meridian_math::Vec3;
use meridian_math::frames::NED;

/// Number of horizontal sectors (45° each).
pub const NUM_SECTORS: usize = 8;
/// Number of vertical layers.
pub const NUM_LAYERS: usize = 5;
/// Sector angular width (radians).
pub const SECTOR_WIDTH: f32 = core::f32::consts::PI / 4.0; // 45°

/// Layer elevation ranges (degrees from horizontal).
const LAYER_ELEVATION: [(f32, f32); NUM_LAYERS] = [
    (-90.0, -45.0),  // below
    (-45.0, -10.0),  // low
    (-10.0, 10.0),   // level
    (10.0, 45.0),    // high
    (45.0, 90.0),    // above
];

/// A single boundary face — distance to nearest obstacle.
#[derive(Debug, Clone, Copy)]
struct BoundaryFace {
    /// 3-sample history for median filtering.
    history: [f32; 3],
    history_idx: u8,
    /// Filtered distance (meters).
    filtered_distance: f32,
    /// IIR filter coefficient.
    filter_alpha: f32,
    /// Timestamp of last update (ms).
    last_update_ms: u32,
    /// Whether this face has valid data.
    valid: bool,
}

impl BoundaryFace {
    const fn new() -> Self {
        Self {
            history: [0.0; 3],
            history_idx: 0,
            filtered_distance: f32::MAX,
            filter_alpha: 0.5,
            last_update_ms: 0,
            valid: false,
        }
    }

    fn update(&mut self, distance: f32, now_ms: u32) {
        if !self.valid {
            // First sample: fill all history slots
            self.history = [distance; 3];
            self.filtered_distance = distance;
            self.history_idx = 1;
            self.valid = true;
            self.last_update_ms = now_ms;
            return;
        }

        // Store in history
        self.history[self.history_idx as usize] = distance;
        self.history_idx = (self.history_idx + 1) % 3;

        // Median of 3 samples
        let mut sorted = self.history;
        if sorted[0] > sorted[1] { sorted.swap(0, 1); }
        if sorted[1] > sorted[2] { sorted.swap(1, 2); }
        if sorted[0] > sorted[1] { sorted.swap(0, 1); }
        let median = sorted[1];

        // IIR filter
        self.filtered_distance = self.filtered_distance * (1.0 - self.filter_alpha)
            + median * self.filter_alpha;

        self.last_update_ms = now_ms;
    }

    fn is_expired(&self, now_ms: u32) -> bool {
        now_ms.wrapping_sub(self.last_update_ms) > 1000 // 1s expiry
    }

    fn distance(&self, now_ms: u32) -> Option<f32> {
        if self.valid && !self.is_expired(now_ms) {
            Some(self.filtered_distance)
        } else {
            None
        }
    }
}

/// 3D proximity boundary.
pub struct ProximityBoundary {
    faces: [[BoundaryFace; NUM_LAYERS]; NUM_SECTORS],
}

impl ProximityBoundary {
    pub fn new() -> Self {
        Self {
            faces: [[BoundaryFace::new(); NUM_LAYERS]; NUM_SECTORS],
        }
    }

    /// Update a face with a new distance reading.
    /// `bearing_deg`: horizontal bearing (0=North, 90=East, etc.)
    /// `elevation_deg`: vertical angle (-90=down, 0=level, +90=up)
    /// `distance`: meters to obstacle
    pub fn update(&mut self, bearing_deg: f32, elevation_deg: f32, distance: f32, now_ms: u32) {
        let sector = bearing_to_sector(bearing_deg);
        let layer = elevation_to_layer(elevation_deg);
        if sector < NUM_SECTORS && layer < NUM_LAYERS {
            self.faces[sector][layer].update(distance, now_ms);
        }
    }

    /// Get the closest obstacle distance in a given direction.
    pub fn get_distance(&self, bearing_deg: f32, elevation_deg: f32, now_ms: u32) -> Option<f32> {
        let sector = bearing_to_sector(bearing_deg);
        let layer = elevation_to_layer(elevation_deg);
        if sector < NUM_SECTORS && layer < NUM_LAYERS {
            self.faces[sector][layer].distance(now_ms)
        } else {
            None
        }
    }

    /// Get the closest obstacle in any horizontal direction at the level layer.
    pub fn closest_horizontal(&self, now_ms: u32) -> Option<(f32, f32)> {
        let layer = 2; // level layer
        let mut closest: Option<(f32, f32)> = None;
        for sector in 0..NUM_SECTORS {
            if let Some(dist) = self.faces[sector][layer].distance(now_ms) {
                if closest.is_none() || dist < closest.unwrap().0 {
                    let bearing = sector as f32 * 45.0 + 22.5; // center of sector
                    closest = Some((dist, bearing));
                }
            }
        }
        closest
    }

    /// Get all valid distances as a flat array for avoidance.
    pub fn get_all_distances(&self, now_ms: u32) -> [[Option<f32>; NUM_LAYERS]; NUM_SECTORS] {
        let mut result = [[None; NUM_LAYERS]; NUM_SECTORS];
        for s in 0..NUM_SECTORS {
            for l in 0..NUM_LAYERS {
                result[s][l] = self.faces[s][l].distance(now_ms);
            }
        }
        result
    }

    /// Reset all faces.
    pub fn clear(&mut self) {
        self.faces = [[BoundaryFace::new(); NUM_LAYERS]; NUM_SECTORS];
    }
}

fn bearing_to_sector(bearing_deg: f32) -> usize {
    let mut b = bearing_deg % 360.0;
    if b < 0.0 { b += 360.0; }
    ((b / 45.0) as usize).min(NUM_SECTORS - 1)
}

fn elevation_to_layer(elevation_deg: f32) -> usize {
    for (i, (low, high)) in LAYER_ELEVATION.iter().enumerate() {
        if elevation_deg >= *low && elevation_deg < *high {
            return i;
        }
    }
    if elevation_deg >= 45.0 { return 4; }
    0 // default to below
}

// ─── Velocity Avoidance ───

/// Avoidance configuration.
pub struct AvoidConfig {
    /// Horizontal obstacle margin (meters).
    pub margin_m: f32,
    /// Hard-stop minimum distance (meters). Velocity zeroed if closer.
    pub hard_stop_m: f32,
    /// Fence boundary distances [N, NE, E, SE, S, SW, W, NW] in meters. 0 = no fence.
    pub fence_distances: [f32; NUM_SECTORS],
    /// Whether vertical avoidance is enabled (uses above/below layers).
    pub vertical_avoidance: bool,
}

impl Default for AvoidConfig {
    fn default() -> Self {
        Self {
            margin_m: 5.0,
            hard_stop_m: 1.0,
            fence_distances: [0.0; NUM_SECTORS],
            vertical_avoidance: false,
        }
    }
}

/// AC_Avoid-style velocity bending near obstacles.
/// Includes horizontal + vertical avoidance, fence velocity bending, and hard stop.
pub fn avoid_velocity(
    desired_vel: &Vec3<NED>,
    boundary: &ProximityBoundary,
    margin_m: f32,
    now_ms: u32,
) -> Vec3<NED> {
    let config = AvoidConfig { margin_m, ..AvoidConfig::default() };
    avoid_velocity_full(desired_vel, boundary, &config, now_ms)
}

/// Full avoidance with all features.
pub fn avoid_velocity_full(
    desired_vel: &Vec3<NED>,
    boundary: &ProximityBoundary,
    config: &AvoidConfig,
    now_ms: u32,
) -> Vec3<NED> {
    let mut result = *desired_vel;

    // ── Horizontal avoidance (level layer) ──
    for sector in 0..NUM_SECTORS {
        if let Some(dist) = boundary.faces[sector][2].distance(now_ms) {
            // Hard stop
            if dist <= config.hard_stop_m {
                let bearing_rad = ((sector as f32) * 45.0 + 22.5) * core::f32::consts::PI / 180.0;
                let obstacle_dir_n = libm::cosf(bearing_rad);
                let obstacle_dir_e = libm::sinf(bearing_rad);
                let vel_toward = result.x * obstacle_dir_n + result.y * obstacle_dir_e;
                if vel_toward > 0.0 {
                    result.x -= obstacle_dir_n * vel_toward;
                    result.y -= obstacle_dir_e * vel_toward;
                }
                continue;
            }

            if dist < config.margin_m {
                let scale = (dist / config.margin_m).clamp(0.0, 1.0);
                let bearing_rad = ((sector as f32) * 45.0 + 22.5) * core::f32::consts::PI / 180.0;
                let obstacle_dir_n = libm::cosf(bearing_rad);
                let obstacle_dir_e = libm::sinf(bearing_rad);
                let vel_toward = result.x * obstacle_dir_n + result.y * obstacle_dir_e;
                if vel_toward > 0.0 {
                    result.x -= obstacle_dir_n * vel_toward * (1.0 - scale);
                    result.y -= obstacle_dir_e * vel_toward * (1.0 - scale);
                }
            }
        }
    }

    // ── Vertical avoidance (above/below layers) ──
    if config.vertical_avoidance {
        // Check below layer (layer 0: -90 to -45 deg)
        for sector in 0..NUM_SECTORS {
            if let Some(dist) = boundary.faces[sector][0].distance(now_ms) {
                if dist < config.margin_m && result.z > 0.0 {
                    // Moving down toward obstacle below
                    let scale = (dist / config.margin_m).clamp(0.0, 1.0);
                    result.z *= scale;
                }
            }
        }

        // Check above layer (layer 4: 45 to 90 deg)
        for sector in 0..NUM_SECTORS {
            if let Some(dist) = boundary.faces[sector][4].distance(now_ms) {
                if dist < config.margin_m && result.z < 0.0 {
                    // Moving up toward obstacle above (NED: z negative = up)
                    let scale = (dist / config.margin_m).clamp(0.0, 1.0);
                    result.z *= scale;
                }
            }
        }
    }

    // ── Fence velocity bending ──
    for sector in 0..NUM_SECTORS {
        let fence_dist = config.fence_distances[sector];
        if fence_dist > 0.0 && fence_dist < config.margin_m {
            let scale = (fence_dist / config.margin_m).clamp(0.0, 1.0);
            let bearing_rad = ((sector as f32) * 45.0 + 22.5) * core::f32::consts::PI / 180.0;
            let dir_n = libm::cosf(bearing_rad);
            let dir_e = libm::sinf(bearing_rad);
            let vel_toward = result.x * dir_n + result.y * dir_e;
            if vel_toward > 0.0 {
                result.x -= dir_n * vel_toward * (1.0 - scale);
                result.y -= dir_e * vel_toward * (1.0 - scale);
            }
        }
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bearing_to_sector() {
        assert_eq!(bearing_to_sector(0.0), 0);   // North
        assert_eq!(bearing_to_sector(44.0), 0);
        assert_eq!(bearing_to_sector(45.0), 1);   // NE
        assert_eq!(bearing_to_sector(90.0), 2);   // East
        assert_eq!(bearing_to_sector(180.0), 4);  // South
        assert_eq!(bearing_to_sector(270.0), 6);  // West
        assert_eq!(bearing_to_sector(359.0), 7);
    }

    #[test]
    fn test_elevation_to_layer() {
        assert_eq!(elevation_to_layer(-60.0), 0); // below
        assert_eq!(elevation_to_layer(-20.0), 1); // low
        assert_eq!(elevation_to_layer(0.0), 2);   // level
        assert_eq!(elevation_to_layer(30.0), 3);  // high
        assert_eq!(elevation_to_layer(60.0), 4);  // above
    }

    #[test]
    fn test_boundary_update_and_read() {
        let mut boundary = ProximityBoundary::new();
        boundary.update(90.0, 0.0, 5.0, 1000); // East, level, 5m
        let dist = boundary.get_distance(90.0, 0.0, 1000);
        assert!(dist.is_some());
        assert!((dist.unwrap() - 5.0).abs() < 0.1);
    }

    #[test]
    fn test_boundary_expiry() {
        let mut boundary = ProximityBoundary::new();
        boundary.update(0.0, 0.0, 3.0, 1000);
        // At 1500ms — still valid (< 1s)
        assert!(boundary.get_distance(0.0, 0.0, 1500).is_some());
        // At 2500ms — expired (> 1s)
        assert!(boundary.get_distance(0.0, 0.0, 2500).is_none());
    }

    #[test]
    fn test_median_filter() {
        let mut boundary = ProximityBoundary::new();
        // Feed 3 samples with an outlier
        boundary.update(0.0, 0.0, 5.0, 100);
        boundary.update(0.0, 0.0, 100.0, 200); // outlier
        boundary.update(0.0, 0.0, 5.5, 300);
        let dist = boundary.get_distance(0.0, 0.0, 300).unwrap();
        // Median of [5.0, 100.0, 5.5] = 5.5
        assert!((dist - 5.5).abs() < 1.0, "Median should reject outlier: {}", dist);
    }

    #[test]
    fn test_closest_horizontal() {
        let mut boundary = ProximityBoundary::new();
        boundary.update(0.0, 0.0, 10.0, 1000);   // North 10m
        boundary.update(90.0, 0.0, 3.0, 1000);    // East 3m (closest)
        boundary.update(180.0, 0.0, 15.0, 1000);  // South 15m

        let (dist, bearing) = boundary.closest_horizontal(1000).unwrap();
        assert!((dist - 3.0).abs() < 1.0, "Closest should be 3m: {}", dist);
        assert!((bearing - 112.5).abs() < 1.0, "Bearing should be ~112.5: {}", bearing);
    }

    #[test]
    fn test_velocity_avoidance() {
        let mut boundary = ProximityBoundary::new();
        // Obstacle 2m to the east
        boundary.update(90.0, 0.0, 2.0, 1000);
        boundary.update(90.0, 0.0, 2.0, 1001);
        boundary.update(90.0, 0.0, 2.0, 1002);

        // Desired velocity: 5 m/s east
        let desired = Vec3::<NED>::new(0.0, 5.0, 0.0);
        let avoided = avoid_velocity(&desired, &boundary, 5.0, 1002);

        // Eastward component should be reduced
        assert!(avoided.y < desired.y, "Should reduce eastward velocity: {} vs {}", avoided.y, desired.y);
        // Northward component should be mostly unchanged (slight cross-coupling from sector center offset)
        assert!((avoided.x).abs() < 2.0, "North velocity should be small: {}", avoided.x);
    }

    #[test]
    fn test_no_avoidance_when_clear() {
        let boundary = ProximityBoundary::new(); // empty
        let desired = Vec3::<NED>::new(5.0, 3.0, 0.0);
        let avoided = avoid_velocity(&desired, &boundary, 5.0, 1000);
        assert!((avoided.x - 5.0).abs() < 0.01);
        assert!((avoided.y - 3.0).abs() < 0.01);
    }
}

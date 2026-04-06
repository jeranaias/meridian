//! Rally point management for alternate landing sites.
//!
//! Rally points are pre-configured safe locations where the vehicle can
//! divert to instead of returning all the way home. Used by RTL/SmartRTL
//! to find the closest safe haven.

use meridian_math::Vec3;
use meridian_math::frames::NED;

/// Maximum number of rally points.
const MAX_RALLY_POINTS: usize = 10;

/// A single rally point with position and altitude.
#[derive(Debug, Clone, Copy)]
pub struct RallyPoint {
    /// Position in NED frame (meters, relative to home).
    pub position: Vec3<NED>,
    /// Altitude above home (meters, positive up — stored as negative-D convention).
    pub alt: f32,
}

impl RallyPoint {
    pub fn new(position: Vec3<NED>, alt: f32) -> Self {
        Self { position, alt }
    }
}

/// Manages a set of rally points and finds the closest valid one.
pub struct RallyManager {
    points: [Option<RallyPoint>; MAX_RALLY_POINTS],
    count: usize,
}

impl RallyManager {
    pub fn new() -> Self {
        Self {
            points: [None; MAX_RALLY_POINTS],
            count: 0,
        }
    }

    /// Add a rally point. Returns false if the array is full.
    pub fn add(&mut self, point: RallyPoint) -> bool {
        if self.count >= MAX_RALLY_POINTS {
            return false;
        }
        self.points[self.count] = Some(point);
        self.count += 1;
        true
    }

    /// Remove the rally point at the given index.
    pub fn remove(&mut self, index: usize) -> bool {
        if index >= self.count {
            return false;
        }
        // Shift remaining points down
        for i in index..(self.count - 1) {
            self.points[i] = self.points[i + 1];
        }
        self.points[self.count - 1] = None;
        self.count -= 1;
        true
    }

    /// Clear all rally points.
    pub fn clear(&mut self) {
        for i in 0..self.count {
            self.points[i] = None;
        }
        self.count = 0;
    }

    /// Number of stored rally points.
    pub fn count(&self) -> usize {
        self.count
    }

    /// Calculate the closest valid rally point to the current position.
    /// Returns `None` if there are no rally points (caller should use home).
    pub fn calc_best_rally(&self, current_pos: &Vec3<NED>) -> Option<RallyPoint> {
        if self.count == 0 {
            return None;
        }

        let mut best: Option<(RallyPoint, f32)> = None;

        for i in 0..self.count {
            if let Some(rp) = &self.points[i] {
                let diff = rp.position - *current_pos;
                let dist_sq = diff.x * diff.x + diff.y * diff.y;
                match best {
                    None => best = Some((*rp, dist_sq)),
                    Some((_, best_dist_sq)) => {
                        if dist_sq < best_dist_sq {
                            best = Some((*rp, dist_sq));
                        }
                    }
                }
            }
        }

        best.map(|(rp, _)| rp)
    }

    /// Get a rally point by index.
    pub fn get(&self, index: usize) -> Option<&RallyPoint> {
        if index < self.count {
            self.points[index].as_ref()
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_add_rally_point() {
        let mut mgr = RallyManager::new();
        let rp = RallyPoint::new(Vec3::new(100.0, 200.0, -50.0), 50.0);
        assert!(mgr.add(rp));
        assert_eq!(mgr.count(), 1);
    }

    #[test]
    fn test_add_max_rally_points() {
        let mut mgr = RallyManager::new();
        for i in 0..MAX_RALLY_POINTS {
            let rp = RallyPoint::new(Vec3::new(i as f32 * 100.0, 0.0, -30.0), 30.0);
            assert!(mgr.add(rp), "should add rally point {}", i);
        }
        assert_eq!(mgr.count(), MAX_RALLY_POINTS);
        // One more should fail
        let overflow = RallyPoint::new(Vec3::new(9999.0, 0.0, 0.0), 0.0);
        assert!(!mgr.add(overflow));
    }

    #[test]
    fn test_remove_rally_point() {
        let mut mgr = RallyManager::new();
        mgr.add(RallyPoint::new(Vec3::new(100.0, 0.0, 0.0), 10.0));
        mgr.add(RallyPoint::new(Vec3::new(200.0, 0.0, 0.0), 20.0));
        mgr.add(RallyPoint::new(Vec3::new(300.0, 0.0, 0.0), 30.0));
        assert_eq!(mgr.count(), 3);

        assert!(mgr.remove(1)); // remove middle
        assert_eq!(mgr.count(), 2);
        // The third point should now be at index 1
        let rp = mgr.get(1).unwrap();
        assert!((rp.position.x - 300.0).abs() < 0.1);
    }

    #[test]
    fn test_remove_invalid_index() {
        let mut mgr = RallyManager::new();
        assert!(!mgr.remove(0));
        mgr.add(RallyPoint::new(Vec3::new(100.0, 0.0, 0.0), 10.0));
        assert!(!mgr.remove(5));
    }

    #[test]
    fn test_calc_best_rally_none() {
        let mgr = RallyManager::new();
        let pos = Vec3::<NED>::new(50.0, 50.0, -30.0);
        assert!(mgr.calc_best_rally(&pos).is_none());
    }

    #[test]
    fn test_calc_best_rally_closest() {
        let mut mgr = RallyManager::new();
        mgr.add(RallyPoint::new(Vec3::new(1000.0, 0.0, -50.0), 50.0)); // far north
        mgr.add(RallyPoint::new(Vec3::new(100.0, 100.0, -50.0), 50.0)); // nearby
        mgr.add(RallyPoint::new(Vec3::new(0.0, 2000.0, -50.0), 50.0)); // far east

        let pos = Vec3::<NED>::new(80.0, 80.0, -30.0);
        let best = mgr.calc_best_rally(&pos);
        assert!(best.is_some());
        let best = best.unwrap();
        // Should be the one at (100, 100)
        assert!((best.position.x - 100.0).abs() < 0.1);
        assert!((best.position.y - 100.0).abs() < 0.1);
    }

    #[test]
    fn test_clear() {
        let mut mgr = RallyManager::new();
        mgr.add(RallyPoint::new(Vec3::new(100.0, 0.0, 0.0), 10.0));
        mgr.add(RallyPoint::new(Vec3::new(200.0, 0.0, 0.0), 20.0));
        mgr.clear();
        assert_eq!(mgr.count(), 0);
        assert!(mgr.calc_best_rally(&Vec3::new(0.0, 0.0, 0.0)).is_none());
    }
}

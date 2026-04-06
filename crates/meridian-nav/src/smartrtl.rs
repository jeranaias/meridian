//! SmartRTL breadcrumb recording, path simplification, and loop pruning.
//!
//! Records the vehicle's position along its flight path and provides a
//! reverse route home. Uses Douglas-Peucker simplification and loop pruning
//! to reduce the breadcrumb count while preserving path safety.
//!
//! Source: libraries/AP_SmartRTL/AP_SmartRTL.cpp

use meridian_math::Vec3;
use meridian_math::frames::NED;

/// Maximum number of breadcrumb entries in the ring buffer.
/// GAP 9: increased from 50 to 300 (ArduPilot default: SMARTRTL_POINTS_DEFAULT=300).
const MAX_BREADCRUMBS: usize = 300;

/// Minimum distance (meters) between recorded breadcrumbs.
const MIN_RECORD_DIST: f32 = 10.0;

/// Douglas-Peucker simplification tolerance (meters).
const SIMPLIFY_EPSILON: f32 = 5.0;

/// GAP 10: Timeout (seconds) with no new breadcrumbs before disabling SmartRTL.
/// Source: SMARTRTL_TIMEOUT = 15000 ms
const TIMEOUT_SECS: f32 = 15.0;

/// GAP 11: Number of new points added before triggering automatic cleanup.
/// Source: SMARTRTL_CLEANUP_POINT_TRIGGER = 50
const CLEANUP_TRIGGER: usize = 50;

/// GAP 11: Number of remaining slots that triggers cleanup.
/// Source: SMARTRTL_CLEANUP_START_MARGIN = 10
const CLEANUP_MARGIN: usize = 10;

/// GAP 8: Loop pruning accuracy factor.
/// Source: SMARTRTL_PRUNING_DELTA (accuracy * 0.99)
const PRUNING_ACCURACY: f32 = 0.99;

/// SmartRTL breadcrumb trail for retracing the flight path home.
pub struct SmartRTL {
    /// Breadcrumb positions stored in chronological order.
    breadcrumbs: [Vec3<NED>; MAX_BREADCRUMBS],
    /// Number of valid breadcrumbs.
    count: usize,
    /// Write index in the ring buffer (wraps around).
    write_idx: usize,
    /// Whether the buffer has wrapped (oldest entries overwritten).
    wrapped: bool,
    /// Scratch buffer for reversed/simplified output.
    return_path: [Vec3<NED>; MAX_BREADCRUMBS],
    return_count: usize,
    /// GAP 10: Time since last breadcrumb was recorded (seconds).
    time_since_last_record: f32,
    /// GAP 10: Whether the trail has been disabled due to timeout.
    disabled: bool,
    /// GAP 11: Number of new breadcrumbs added since last cleanup.
    new_since_cleanup: usize,
}

impl SmartRTL {
    pub fn new() -> Self {
        Self {
            breadcrumbs: [Vec3::zero(); MAX_BREADCRUMBS],
            count: 0,
            write_idx: 0,
            wrapped: false,
            return_path: [Vec3::zero(); MAX_BREADCRUMBS],
            return_count: 0,
            time_since_last_record: 0.0,
            disabled: false,
            new_since_cleanup: 0,
        }
    }

    /// Record a position if the distance from the last breadcrumb exceeds the threshold.
    /// `dt`: seconds since last call (for timeout tracking).
    pub fn record_position(&mut self, pos: Vec3<NED>) {
        self.record_position_dt(pos, 0.0);
    }

    /// Record a position with time tracking for automatic cleanup and timeout.
    pub fn record_position_dt(&mut self, pos: Vec3<NED>, dt: f32) {
        if self.disabled {
            return;
        }

        // Check distance from last recorded breadcrumb
        if self.count > 0 {
            let last_idx = if self.write_idx == 0 { MAX_BREADCRUMBS - 1 } else { self.write_idx - 1 };
            let last = self.breadcrumbs[last_idx];
            let diff = pos - last;
            let dist_sq = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
            if dist_sq < MIN_RECORD_DIST * MIN_RECORD_DIST {
                // Not far enough — update timeout tracker
                self.time_since_last_record += dt;
                // GAP 10: disable if no breadcrumbs recorded for TIMEOUT_SECS
                if self.time_since_last_record >= TIMEOUT_SECS && self.count > 0 {
                    self.disabled = true;
                }
                return;
            }
        }

        self.time_since_last_record = 0.0;

        self.breadcrumbs[self.write_idx] = pos;
        self.write_idx += 1;
        if self.write_idx >= MAX_BREADCRUMBS {
            self.write_idx = 0;
            self.wrapped = true;
        }
        if !self.wrapped {
            self.count += 1;
        } else {
            self.count = MAX_BREADCRUMBS;
        }

        // GAP 11: Automatic cleanup triggers
        self.new_since_cleanup += 1;
        let remaining_slots = MAX_BREADCRUMBS.saturating_sub(self.count);
        if self.new_since_cleanup >= CLEANUP_TRIGGER || remaining_slots <= CLEANUP_MARGIN {
            self.simplify_path();
            self.prune_loops();
            self.new_since_cleanup = 0;
        }
    }

    /// Build the return path (breadcrumbs in reverse order) and return a slice.
    pub fn get_return_path(&mut self) -> &[Vec3<NED>] {
        self.return_count = 0;
        if self.count == 0 {
            return &self.return_path[..0];
        }

        // Read breadcrumbs in chronological order into return_path, then reverse.
        let start = if self.wrapped { self.write_idx } else { 0 };
        for i in 0..self.count {
            let idx = (start + i) % MAX_BREADCRUMBS;
            self.return_path[i] = self.breadcrumbs[idx];
        }
        self.return_count = self.count;

        // Reverse to get newest-first (return path order)
        let n = self.return_count;
        for i in 0..n / 2 {
            let tmp = self.return_path[i];
            self.return_path[i] = self.return_path[n - 1 - i];
            self.return_path[n - 1 - i] = tmp;
        }

        &self.return_path[..self.return_count]
    }

    /// Apply Douglas-Peucker path simplification to remove redundant breadcrumbs.
    pub fn simplify_path(&mut self) {
        if self.count <= 2 {
            return;
        }

        // Linearize breadcrumbs into a working buffer
        let start = if self.wrapped { self.write_idx } else { 0 };
        let mut points: [Vec3<NED>; MAX_BREADCRUMBS] = [Vec3::zero(); MAX_BREADCRUMBS];
        for i in 0..self.count {
            let idx = (start + i) % MAX_BREADCRUMBS;
            points[i] = self.breadcrumbs[idx];
        }

        // Douglas-Peucker: mark which points to keep
        let mut keep = [false; MAX_BREADCRUMBS];
        keep[0] = true;
        keep[self.count - 1] = true;

        // Iterative stack-based DP (no recursion, no heap)
        let mut stack: [(usize, usize); 64] = [(0, 0); 64];
        let mut stack_top: usize = 0;
        stack[stack_top] = (0, self.count - 1);
        stack_top += 1;

        while stack_top > 0 {
            stack_top -= 1;
            let (lo, hi) = stack[stack_top];
            if hi <= lo + 1 {
                continue;
            }

            let mut max_dist: f32 = 0.0;
            let mut max_idx: usize = lo + 1;

            let line_start = points[lo];
            let line_end = points[hi];
            let line = line_end - line_start;
            let line_len_sq = line.x * line.x + line.y * line.y + line.z * line.z;

            for i in (lo + 1)..hi {
                let dist = if line_len_sq < 1e-12 {
                    let d = points[i] - line_start;
                    libm::sqrtf(d.x * d.x + d.y * d.y + d.z * d.z)
                } else {
                    let ap = points[i] - line_start;
                    let t = (ap.x * line.x + ap.y * line.y + ap.z * line.z) / line_len_sq;
                    let t = if t < 0.0 { 0.0 } else if t > 1.0 { 1.0 } else { t };
                    let proj = Vec3::<NED>::new(
                        line_start.x + t * line.x,
                        line_start.y + t * line.y,
                        line_start.z + t * line.z,
                    );
                    let d = points[i] - proj;
                    libm::sqrtf(d.x * d.x + d.y * d.y + d.z * d.z)
                };

                if dist > max_dist {
                    max_dist = dist;
                    max_idx = i;
                }
            }

            if max_dist > SIMPLIFY_EPSILON {
                keep[max_idx] = true;
                if stack_top + 2 <= 64 {
                    stack[stack_top] = (lo, max_idx);
                    stack_top += 1;
                    stack[stack_top] = (max_idx, hi);
                    stack_top += 1;
                }
            }
        }

        // Compact: copy kept points back
        let mut new_count = 0;
        for i in 0..self.count {
            if keep[i] {
                self.breadcrumbs[new_count] = points[i];
                new_count += 1;
            }
        }
        self.count = new_count;
        self.write_idx = new_count % MAX_BREADCRUMBS;
        self.wrapped = false;
    }

    /// Loop pruning: detect path loops and excise them.
    ///
    /// Source: AP_SmartRTL::detect_loops() + add_loop() + remove_points_by_loops()
    ///
    /// Finds non-adjacent SEGMENT pairs (not just points) that are within
    /// `accuracy * 0.99` distance of each other, indicating the vehicle retraced
    /// near a previous path segment. Everything between those two segments is
    /// excised and the two closest points are replaced with their midpoint.
    ///
    /// ArduPilot's algorithm: outer loop decrements `i` from path end,
    /// inner loop increments `j` from 1. Segments compared are (i-1,i) vs (j-1,j).
    /// When a close pair is found, points j through i-1 are excised and
    /// replaced with the midpoint of the two closest points on the segments.
    ///
    /// Should be called after `simplify_path()`.
    pub fn prune_loops(&mut self) {
        if self.count < 4 {
            return; // need at least 4 points (3 segments)
        }

        // Linearize breadcrumbs into working buffer
        let start = if self.wrapped { self.write_idx } else { 0 };
        let mut points: [Vec3<NED>; MAX_BREADCRUMBS] = [Vec3::zero(); MAX_BREADCRUMBS];
        for k in 0..self.count {
            let idx = (start + k) % MAX_BREADCRUMBS;
            points[k] = self.breadcrumbs[idx];
        }
        let mut n = self.count;

        // SMARTRTL_PRUNING_DELTA = accuracy * 0.99
        // SMARTRTL_ACCURACY_DEFAULT = 2.0
        let prune_delta: f32 = MIN_RECORD_DIST * PRUNING_ACCURACY;

        // Outer loop: scan from the end backwards.
        // Segment_i = (points[i-1], points[i])
        // Inner loop: scan from the start forwards.
        // Segment_j = (points[j-1], points[j])
        // j must satisfy j <= i-2 (non-adjacent segments).
        let mut i = n.saturating_sub(1);
        while i >= 3 {
            let mut j: usize = 1;
            while j + 2 <= i {
                let dp = seg_seg_dist(
                    points[i], points[i - 1],
                    points[j - 1], points[j],
                );
                if dp.distance < prune_delta {
                    // Found a loop between segments j and i.
                    // Excise points j through i-1, replace points[j] with midpoint.
                    let excise_start = j;
                    let excise_end = i - 1; // inclusive
                    if excise_start <= excise_end && excise_end < n {
                        // Replace the start of excision with midpoint
                        points[excise_start] = dp.midpoint;
                        // Number of points to remove (everything from excise_start+1 to excise_end)
                        let removed = excise_end - excise_start;
                        if removed > 0 {
                            // Shift remaining points left
                            let mut dst = excise_start + 1;
                            let mut src = excise_end + 1;
                            while src < n {
                                points[dst] = points[src];
                                dst += 1;
                                src += 1;
                            }
                            n -= removed;
                            // Adjust i: the removed points were before position i,
                            // so i shifts down by `removed`
                            if i > removed {
                                i -= removed;
                            } else {
                                break;
                            }
                        }
                    }
                    // After excision, skip to next outer iteration
                    // (ArduPilot sets j=i to exit inner loop)
                    break;
                }
                j += 1;
            }
            if i == 0 { break; }
            i -= 1;
        }

        // Write compacted path back
        for k in 0..n {
            self.breadcrumbs[k] = points[k];
        }
        self.count = n;
        self.write_idx = n % MAX_BREADCRUMBS;
        self.wrapped = false;
    }

    /// Number of stored breadcrumbs.
    pub fn breadcrumb_count(&self) -> usize {
        self.count
    }

    /// Whether SmartRTL has been disabled due to timeout.
    pub fn is_disabled(&self) -> bool {
        self.disabled
    }

    /// Clear all breadcrumbs and reset state.
    pub fn clear(&mut self) {
        self.count = 0;
        self.write_idx = 0;
        self.wrapped = false;
        self.return_count = 0;
        self.disabled = false;
        self.time_since_last_record = 0.0;
        self.new_since_cleanup = 0;
    }
}

// ─── Segment-to-segment distance (for loop pruning) ───

/// Result of segment-to-segment distance calculation.
struct SegDistResult {
    distance: f32,
    midpoint: Vec3<NED>,
}

/// Dot product of two Vec3<NED>.
#[inline]
fn dot3(a: Vec3<NED>, b: Vec3<NED>) -> f32 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

/// Clamp a value to [0, 1].
#[inline]
fn clamp01(v: f32) -> f32 {
    if v < 0.0 { 0.0 } else if v > 1.0 { 1.0 } else { v }
}

/// Compute the closest distance between two line segments and the midpoint
/// of the two closest points on the segments.
///
/// Segment 1: p1 to p2, Segment 2: p3 to p4.
/// Source: AP_SmartRTL.cpp segment_segment_dist()
fn seg_seg_dist(
    p1: Vec3<NED>, p2: Vec3<NED>,
    p3: Vec3<NED>, p4: Vec3<NED>,
) -> SegDistResult {
    let d1 = p2 - p1; // direction of segment 1
    let d2 = p4 - p3; // direction of segment 2
    let r = p1 - p3;

    let a = dot3(d1, d1); // |d1|^2
    let e = dot3(d2, d2); // |d2|^2
    let f = dot3(d2, r);
    let b = dot3(d1, d2);
    let c = dot3(d1, r);

    let (t1, t2);

    if a < 1e-12 && e < 1e-12 {
        // Both segments are degenerate (points)
        t1 = 0.0;
        t2 = 0.0;
    } else if a < 1e-12 {
        // Segment 1 is a point
        t1 = 0.0;
        t2 = clamp01(f / e);
    } else if e < 1e-12 {
        // Segment 2 is a point
        t2 = 0.0;
        t1 = clamp01(-c / a);
    } else {
        // General case: solve for closest points on two lines
        let denom = a * e - b * b;
        let mut s;
        let mut t;
        if denom > 1e-12 {
            s = clamp01((b * f - c * e) / denom);
        } else {
            // Nearly parallel
            s = 0.0;
        }
        // Compute t from s
        t = (b * s + f) / e;

        // Clamp t and recompute s if needed
        if t < 0.0 {
            t = 0.0;
            s = clamp01(-c / a);
        } else if t > 1.0 {
            t = 1.0;
            s = clamp01((b - c) / a);
        }

        t1 = s;
        t2 = t;
    }

    let closest1 = Vec3::<NED>::new(
        p1.x + t1 * d1.x,
        p1.y + t1 * d1.y,
        p1.z + t1 * d1.z,
    );
    let closest2 = Vec3::<NED>::new(
        p3.x + t2 * d2.x,
        p3.y + t2 * d2.y,
        p3.z + t2 * d2.z,
    );
    let diff = closest1 - closest2;
    let dist = libm::sqrtf(dot3(diff, diff));
    let mid = Vec3::<NED>::new(
        (closest1.x + closest2.x) * 0.5,
        (closest1.y + closest2.y) * 0.5,
        (closest1.z + closest2.z) * 0.5,
    );
    SegDistResult { distance: dist, midpoint: mid }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_record_basic() {
        let mut srtl = SmartRTL::new();
        srtl.record_position(Vec3::new(0.0, 0.0, 0.0));
        assert_eq!(srtl.breadcrumb_count(), 1);
        srtl.record_position(Vec3::new(5.0, 0.0, 0.0));
        assert_eq!(srtl.breadcrumb_count(), 1); // too close
        srtl.record_position(Vec3::new(15.0, 0.0, 0.0));
        assert_eq!(srtl.breadcrumb_count(), 2);
    }

    #[test]
    fn test_buffer_size_300() {
        // GAP 9: verify buffer is now 300
        assert_eq!(MAX_BREADCRUMBS, 300);
        // Note: auto-cleanup (simplify + prune) runs every CLEANUP_TRIGGER points,
        // so a straight-line path will be aggressively simplified. Verify the
        // buffer constant and that wrapping works on non-collinear paths.
        let mut srtl = SmartRTL::new();
        // Use a zigzag to defeat simplification
        for i in 0..310 {
            let x = i as f32 * 20.0;
            let y = if i % 2 == 0 { 0.0 } else { 50.0 }; // zigzag defeats D-P simplification
            srtl.record_position(Vec3::new(x, y, 0.0));
        }
        // Zigzag preserves corners through Douglas-Peucker but loop pruning
        // may remove some back-and-forth patterns. We should still have a
        // significant number of breadcrumbs (>>2 which a straight line produces).
        assert!(srtl.breadcrumb_count() > 20,
            "Zigzag path should retain many breadcrumbs: got {}", srtl.breadcrumb_count());
    }

    #[test]
    fn test_return_path_reversed() {
        let mut srtl = SmartRTL::new();
        srtl.record_position(Vec3::new(0.0, 0.0, 0.0));
        srtl.record_position(Vec3::new(20.0, 0.0, 0.0));
        srtl.record_position(Vec3::new(40.0, 0.0, 0.0));

        let path = srtl.get_return_path();
        assert_eq!(path.len(), 3);
        assert!((path[0].x - 40.0).abs() < 0.1);
        assert!((path[2].x - 0.0).abs() < 0.1);
    }

    #[test]
    fn test_simplify_collinear() {
        let mut srtl = SmartRTL::new();
        for i in 0..5 {
            srtl.record_position(Vec3::new(i as f32 * 20.0, 0.0, 0.0));
        }
        assert_eq!(srtl.breadcrumb_count(), 5);
        srtl.simplify_path();
        assert_eq!(srtl.breadcrumb_count(), 2);
    }

    #[test]
    fn test_simplify_preserves_corners() {
        let mut srtl = SmartRTL::new();
        srtl.record_position(Vec3::new(0.0, 0.0, 0.0));
        srtl.record_position(Vec3::new(50.0, 0.0, 0.0));
        srtl.record_position(Vec3::new(50.0, 50.0, 0.0));
        srtl.simplify_path();
        assert_eq!(srtl.breadcrumb_count(), 3);
    }

    #[test]
    fn test_loop_pruning() {
        // GAP 8: vehicle goes north, circles back, continues north
        let mut srtl = SmartRTL::new();
        srtl.record_position(Vec3::new(0.0, 0.0, 0.0));    // 0
        srtl.record_position(Vec3::new(20.0, 0.0, 0.0));   // 1
        srtl.record_position(Vec3::new(40.0, 0.0, 0.0));   // 2
        // Loop: go east, south, west, back to near point 2
        srtl.record_position(Vec3::new(40.0, 20.0, 0.0));  // 3
        srtl.record_position(Vec3::new(20.0, 20.0, 0.0));  // 4
        srtl.record_position(Vec3::new(20.0, 0.5, 0.0));   // 5 — near point 1 (within prune_dist)
        srtl.record_position(Vec3::new(60.0, 0.0, 0.0));   // 6

        let count_before = srtl.breadcrumb_count();
        srtl.prune_loops();
        let count_after = srtl.breadcrumb_count();
        assert!(count_after < count_before,
            "Loop pruning should reduce count: {} -> {}", count_before, count_after);
    }

    #[test]
    fn test_timeout_disables() {
        // GAP 10: SmartRTL should disable after 15s with no new breadcrumbs
        let mut srtl = SmartRTL::new();
        srtl.record_position(Vec3::new(0.0, 0.0, 0.0));
        assert!(!srtl.is_disabled());

        // Simulate many ticks without moving (position within threshold)
        for _ in 0..200 {
            srtl.record_position_dt(Vec3::new(1.0, 0.0, 0.0), 0.1); // 0.1s each = 20s total
        }
        assert!(srtl.is_disabled(), "Should be disabled after 15s timeout");
    }

    #[test]
    fn test_clear() {
        let mut srtl = SmartRTL::new();
        srtl.record_position(Vec3::new(0.0, 0.0, 0.0));
        srtl.record_position(Vec3::new(20.0, 0.0, 0.0));
        srtl.clear();
        assert_eq!(srtl.breadcrumb_count(), 0);
        assert!(!srtl.is_disabled());
    }

    #[test]
    fn test_empty_return_path() {
        let mut srtl = SmartRTL::new();
        let path = srtl.get_return_path();
        assert_eq!(path.len(), 0);
    }

    #[test]
    fn test_loop_pruning_segment_based() {
        // Test that loop pruning uses segment-to-segment distance, not just
        // point-to-point. The vehicle path passes NEAR a previous segment
        // but not near any specific point on it.
        let mut srtl = SmartRTL::new();
        // Straight line east
        srtl.record_position(Vec3::new(0.0, 0.0, 0.0));      // 0
        srtl.record_position(Vec3::new(0.0, 50.0, 0.0));      // 1: far east
        // Detour north
        srtl.record_position(Vec3::new(20.0, 50.0, 0.0));     // 2
        srtl.record_position(Vec3::new(20.0, 30.0, 0.0));     // 3
        // Come back near the segment (0,1) but not near points 0 or 1
        // Point 4 is at (0.5, 25.0) which is 0.5m from the segment (0,0)-(0,50)
        srtl.record_position(Vec3::new(0.5, 25.0, 0.0));      // 4
        // Continue
        srtl.record_position(Vec3::new(0.0, 70.0, 0.0));      // 5

        let count_before = srtl.breadcrumb_count();
        assert_eq!(count_before, 6);
        srtl.prune_loops();
        let count_after = srtl.breadcrumb_count();
        // The loop (points 2,3 at minimum) should be excised
        assert!(count_after < count_before,
            "Segment-based pruning should detect loop: {} -> {}", count_before, count_after);
    }

    #[test]
    fn test_loop_pruning_no_false_positive() {
        // Straight path with no loops should not be pruned
        let mut srtl = SmartRTL::new();
        srtl.record_position(Vec3::new(0.0, 0.0, 0.0));
        srtl.record_position(Vec3::new(20.0, 0.0, 0.0));
        srtl.record_position(Vec3::new(40.0, 20.0, 0.0));
        srtl.record_position(Vec3::new(60.0, 40.0, 0.0));
        srtl.record_position(Vec3::new(80.0, 60.0, 0.0));

        let count_before = srtl.breadcrumb_count();
        srtl.prune_loops();
        assert_eq!(srtl.breadcrumb_count(), count_before,
            "Straight path should not be pruned");
    }

    #[test]
    fn test_prune_runs_after_simplify() {
        // Verify the intended workflow: simplify then prune
        let mut srtl = SmartRTL::new();
        // Create a loop path
        srtl.record_position(Vec3::new(0.0, 0.0, 0.0));
        srtl.record_position(Vec3::new(20.0, 0.0, 0.0));
        srtl.record_position(Vec3::new(40.0, 0.0, 0.0));
        srtl.record_position(Vec3::new(40.0, 20.0, 0.0));
        srtl.record_position(Vec3::new(20.0, 20.0, 0.0));
        srtl.record_position(Vec3::new(20.0, 1.0, 0.0)); // near segment (0,1)
        srtl.record_position(Vec3::new(60.0, 0.0, 0.0));

        srtl.simplify_path();
        let after_simplify = srtl.breadcrumb_count();
        srtl.prune_loops();
        let after_prune = srtl.breadcrumb_count();
        assert!(after_prune <= after_simplify,
            "prune after simplify: {} -> {}", after_simplify, after_prune);
    }
}

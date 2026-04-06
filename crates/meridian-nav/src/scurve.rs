//! 7-phase jerk-limited S-curve trajectory planner.
//!
//! Replaces simple linear interpolation for straight waypoint segments.
//! ArduPilot equivalent: `AP_Math/SCurve.h` / `AC_WPNav`.
//!
//! The 7 phases are:
//! 1. Jerk+ — acceleration increasing (jerk = +J)
//! 2. Constant accel — at max acceleration (jerk = 0)
//! 3. Jerk- — acceleration decreasing to zero (jerk = -J), reaching max velocity
//! 4. Cruise — at max velocity (accel = 0, jerk = 0)
//! 5. Jerk- — deceleration beginning (jerk = -J)
//! 6. Constant decel — at max deceleration (jerk = 0)
//! 7. Jerk+ — deceleration ending (jerk = +J), arriving at zero velocity

use meridian_math::Vec3;
use meridian_math::frames::NED;

/// A single straight-line segment with a 7-phase jerk-limited velocity profile.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct SCurveSegment {
    // Profile parameters
    jerk_max: f32,
    accel_max: f32,
    vel_max: f32,

    // Segment geometry
    start: Vec3<NED>,
    end: Vec3<NED>,
    length: f32,
    direction: Vec3<NED>, // unit vector start -> end

    // Phase durations (computed from constraints)
    t: [f32; 7],

    // Total time for the profile
    total_time: f32,

    // Current state
    time_elapsed: f32,
    position_along: f32, // distance along segment
    velocity: f32,
    acceleration: f32,
}

impl SCurveSegment {
    /// Create a new S-curve segment between two points.
    ///
    /// Computes the 7-phase duration profile from the kinematic constraints.
    /// If the segment is too short to reach max velocity or max acceleration,
    /// the profile is scaled down proportionally.
    pub fn new(
        start: Vec3<NED>,
        end: Vec3<NED>,
        jerk_max: f32,
        accel_max: f32,
        vel_max: f32,
    ) -> Self {
        let diff = end - start;
        let length = diff.length();
        let direction = if length > 1e-6 {
            diff.normalized()
        } else {
            Vec3::new(1.0, 0.0, 0.0)
        };

        let t = if length < 1e-6 {
            [0.0; 7]
        } else {
            compute_phase_durations(length, jerk_max, accel_max, vel_max)
        };

        let total_time = t[0] + t[1] + t[2] + t[3] + t[4] + t[5] + t[6];

        Self {
            jerk_max,
            accel_max,
            vel_max,
            start,
            end,
            length,
            direction,
            t,
            total_time,
            time_elapsed: 0.0,
            position_along: 0.0,
            velocity: 0.0,
            acceleration: 0.0,
        }
    }

    /// Advance the profile by `dt` seconds.
    ///
    /// Returns `(position, velocity_vector, progress_fraction)`.
    pub fn advance(&mut self, dt: f32) -> (Vec3<NED>, Vec3<NED>, f32) {
        if self.length < 1e-6 || self.total_time < 1e-6 {
            return (self.end, Vec3::zero(), 1.0);
        }

        self.time_elapsed += dt;
        if self.time_elapsed > self.total_time {
            self.time_elapsed = self.total_time;
        }

        // Evaluate the profile at current time_elapsed
        let (pos, vel, acc) = self.evaluate_at(self.time_elapsed);
        self.position_along = pos;
        self.velocity = vel;
        self.acceleration = acc;

        // Clamp position to segment length
        let clamped_pos = if self.position_along > self.length {
            self.length
        } else {
            self.position_along
        };

        let world_pos = self.start + self.direction * clamped_pos;
        let vel_vec = self.direction * self.velocity;
        let progress = if self.length > 0.0 {
            clamped_pos / self.length
        } else {
            1.0
        };

        (world_pos, vel_vec, progress)
    }

    /// Whether the segment is complete.
    pub fn is_complete(&self) -> bool {
        self.time_elapsed >= self.total_time - 1e-6
    }

    /// Remaining distance along the segment.
    pub fn remaining_distance(&self) -> f32 {
        (self.length - self.position_along).max(0.0)
    }

    /// Total duration of the profile in seconds.
    pub fn total_time(&self) -> f32 {
        self.total_time
    }

    /// Current velocity magnitude.
    pub fn current_velocity(&self) -> f32 {
        self.velocity
    }

    /// Current acceleration magnitude.
    pub fn current_acceleration(&self) -> f32 {
        self.acceleration
    }

    /// Phase durations [jerk+, const_accel, jerk-, cruise, jerk-, const_decel, jerk+].
    pub fn phase_durations(&self) -> &[f32; 7] {
        &self.t
    }

    /// Evaluate the 1-D profile at a given time, returning (position, velocity, acceleration).
    fn evaluate_at(&self, time: f32) -> (f32, f32, f32) {
        let t = &self.t;
        let j = self.jerk_max;

        let mut remaining = time.max(0.0);
        let mut pos: f32 = 0.0;
        let mut vel: f32 = 0.0;
        let mut acc: f32 = 0.0;

        // Phase 1: Jerk+ (jerk = +J, acc ramps from 0 to accel_max)
        let dt1 = remaining.min(t[0]);
        if dt1 > 0.0 {
            // a(t) = J*t, v(t) = J*t^2/2, s(t) = J*t^3/6
            acc = j * dt1;
            vel = j * dt1 * dt1 / 2.0;
            pos = j * dt1 * dt1 * dt1 / 6.0;
        }
        remaining -= t[0];
        if remaining <= 0.0 {
            return (pos, vel, acc);
        }

        // Phase 2: Constant accel (jerk = 0, acc = accel_max)
        let a1 = acc; // acceleration at end of phase 1
        let v1 = vel; // velocity at end of phase 1
        let s1 = pos; // position at end of phase 1
        let dt2 = remaining.min(t[1]);
        if dt2 > 0.0 {
            acc = a1;
            vel = v1 + a1 * dt2;
            pos = s1 + v1 * dt2 + a1 * dt2 * dt2 / 2.0;
        }
        remaining -= t[1];
        if remaining <= 0.0 {
            return (pos, vel, acc);
        }

        // Phase 3: Jerk- (jerk = -J, acc ramps from accel_max to 0)
        let a2 = acc;
        let v2 = vel;
        let s2 = pos;
        let dt3 = remaining.min(t[2]);
        if dt3 > 0.0 {
            acc = a2 - j * dt3;
            vel = v2 + a2 * dt3 - j * dt3 * dt3 / 2.0;
            pos = s2 + v2 * dt3 + a2 * dt3 * dt3 / 2.0 - j * dt3 * dt3 * dt3 / 6.0;
        }
        remaining -= t[2];
        if remaining <= 0.0 {
            return (pos, vel, acc);
        }

        // Phase 4: Cruise (jerk = 0, acc = 0, vel = vel_max)
        let v3 = vel;
        let s3 = pos;
        let dt4 = remaining.min(t[3]);
        if dt4 > 0.0 {
            acc = 0.0;
            vel = v3;
            pos = s3 + v3 * dt4;
        }
        remaining -= t[3];
        if remaining <= 0.0 {
            return (pos, vel, acc);
        }

        // Phase 5: Jerk- (jerk = -J, acc ramps from 0 to -accel_max)
        let v4 = vel;
        let s4 = pos;
        let dt5 = remaining.min(t[4]);
        if dt5 > 0.0 {
            acc = -j * dt5;
            vel = v4 - j * dt5 * dt5 / 2.0;
            pos = s4 + v4 * dt5 - j * dt5 * dt5 * dt5 / 6.0;
        }
        remaining -= t[4];
        if remaining <= 0.0 {
            return (pos, vel, acc);
        }

        // Phase 6: Constant decel (jerk = 0, acc = -accel_max)
        let a5 = acc;
        let v5 = vel;
        let s5 = pos;
        let dt6 = remaining.min(t[5]);
        if dt6 > 0.0 {
            acc = a5;
            vel = v5 + a5 * dt6;
            pos = s5 + v5 * dt6 + a5 * dt6 * dt6 / 2.0;
        }
        remaining -= t[5];
        if remaining <= 0.0 {
            return (pos, vel, acc);
        }

        // Phase 7: Jerk+ (jerk = +J, acc ramps from -accel_max to 0)
        let a6 = acc;
        let v6 = vel;
        let s6 = pos;
        let dt7 = remaining.min(t[6]);
        if dt7 > 0.0 {
            acc = a6 + j * dt7;
            vel = v6 + a6 * dt7 + j * dt7 * dt7 / 2.0;
            pos = s6 + v6 * dt7 + a6 * dt7 * dt7 / 2.0 + j * dt7 * dt7 * dt7 / 6.0;
        }

        (pos, vel.max(0.0), acc)
    }
}

/// Compute the 7 phase durations for a given segment length and kinematic constraints.
///
/// Returns `[t1, t2, t3, t4, t5, t6, t7]` where the profile is symmetric
/// (t1=t3=t5=t7 for jerk phases, t2=t6 for constant accel phases).
fn compute_phase_durations(
    distance: f32,
    jerk_max: f32,
    accel_max: f32,
    vel_max: f32,
) -> [f32; 7] {
    // Time for jerk phase to reach max acceleration
    let t_j = accel_max / jerk_max;

    // Velocity gained during one jerk phase (accel ramp from 0 to accel_max)
    // v_j = J * t_j^2 / 2
    let v_j = jerk_max * t_j * t_j / 2.0;

    // Check if we can reach vel_max with the given accel_max
    // Velocity gained during accel ramp-up (phases 1+2+3):
    //   During phase 1 (jerk+): v_j
    //   During phase 2 (const accel): accel_max * t_a
    //   During phase 3 (jerk-): v_j (symmetric to phase 1 in velocity gain)
    // Total = 2*v_j + accel_max * t_a = vel_max
    // So t_a = (vel_max - 2*v_j) / accel_max

    let actual_vel_max = vel_max;
    let mut actual_accel_max = accel_max;
    let mut t_jerk = t_j;

    // Check if max velocity can be reached
    let vel_from_jerk_only = 2.0 * v_j; // velocity from just the two jerk phases (no const accel)

    let t_const_accel = if vel_from_jerk_only >= vel_max {
        // Can reach vel_max with jerk phases alone (no constant accel phase needed)
        // Need to scale down: find t_j such that J * t_j^2 = vel_max
        t_jerk = libm::sqrtf(vel_max / jerk_max);
        actual_accel_max = jerk_max * t_jerk;
        0.0
    } else {
        (vel_max - vel_from_jerk_only) / accel_max
    };

    // Distance for the acceleration half (phases 1+2+3)
    let d_accel = compute_accel_distance(t_jerk, t_const_accel, jerk_max, actual_accel_max);
    // Deceleration half is symmetric
    let d_decel = d_accel;
    let d_ramp = d_accel + d_decel;

    if d_ramp > distance {
        // Segment too short for full profile — need to reduce vel_max
        // Binary search for the achievable velocity
        return compute_reduced_profile(distance, jerk_max, accel_max);
    }

    // Cruise phase distance
    let d_cruise = distance - d_ramp;
    let t_cruise = if actual_vel_max > 1e-6 {
        d_cruise / actual_vel_max
    } else {
        0.0
    };

    // Symmetric profile: phases mirror around cruise
    [t_jerk, t_const_accel, t_jerk, t_cruise, t_jerk, t_const_accel, t_jerk]
}

/// Compute distance covered during the acceleration portion (phases 1+2+3).
fn compute_accel_distance(t_j: f32, t_a: f32, jerk: f32, accel: f32) -> f32 {
    // Phase 1: s = J * t_j^3 / 6
    let s1 = jerk * t_j * t_j * t_j / 6.0;
    let v1 = jerk * t_j * t_j / 2.0; // velocity at end of phase 1

    // Phase 2: s = v1 * t_a + accel * t_a^2 / 2
    let s2 = v1 * t_a + accel * t_a * t_a / 2.0;
    let v2 = v1 + accel * t_a; // velocity at end of phase 2

    // Phase 3: s = v2 * t_j + accel * t_j^2 / 2 - jerk * t_j^3 / 6
    let s3 = v2 * t_j + accel * t_j * t_j / 2.0 - jerk * t_j * t_j * t_j / 6.0;

    s1 + s2 + s3
}

/// Compute a reduced profile when the segment is too short for full speed.
///
/// Iteratively reduces the target velocity until the profile fits.
fn compute_reduced_profile(distance: f32, jerk_max: f32, accel_max: f32) -> [f32; 7] {
    // Binary search for the achievable peak velocity
    let mut v_lo: f32 = 0.0;
    // Upper bound: the velocity where jerk-only distance = total distance
    let mut v_hi: f32 = libm::sqrtf(distance * jerk_max).min(accel_max * accel_max / jerk_max);
    // Ensure v_hi gives an achievable distance
    v_hi = v_hi.max(0.1);

    for _ in 0..30 {
        let v_mid = (v_lo + v_hi) / 2.0;
        let d = distance_for_velocity(v_mid, jerk_max, accel_max);
        if d < distance {
            v_lo = v_mid;
        } else {
            v_hi = v_mid;
        }
    }

    let v_target = (v_lo + v_hi) / 2.0;
    durations_for_velocity(v_target, jerk_max, accel_max, distance)
}

/// Compute the minimum distance needed to accelerate to `vel` and decelerate back to 0.
fn distance_for_velocity(vel: f32, jerk_max: f32, accel_max: f32) -> f32 {
    if vel <= 0.0 {
        return 0.0;
    }

    let t_j = accel_max / jerk_max;
    let v_j = jerk_max * t_j * t_j / 2.0;

    if vel <= 2.0 * v_j {
        // Triangular accel profile (no constant accel phase)
        let t_jerk = libm::sqrtf(vel / jerk_max);
        2.0 * compute_accel_distance(t_jerk, 0.0, jerk_max, jerk_max * t_jerk)
    } else {
        // Trapezoidal accel profile
        let t_a = (vel - 2.0 * v_j) / accel_max;
        2.0 * compute_accel_distance(t_j, t_a, jerk_max, accel_max)
    }
}

/// Compute phase durations for a given target velocity (no cruise phase).
fn durations_for_velocity(
    vel: f32,
    jerk_max: f32,
    accel_max: f32,
    distance: f32,
) -> [f32; 7] {
    if vel < 1e-6 {
        return [0.0; 7];
    }

    let t_j_full = accel_max / jerk_max;
    let v_j = jerk_max * t_j_full * t_j_full / 2.0;

    let (t_jerk, t_const) = if vel <= 2.0 * v_j {
        // Triangular: no constant accel phase
        let tj = libm::sqrtf(vel / jerk_max);
        (tj, 0.0f32)
    } else {
        // Trapezoidal
        let ta = (vel - 2.0 * v_j) / accel_max;
        (t_j_full, ta)
    };

    // Compute actual accel for this jerk time
    let actual_accel = jerk_max * t_jerk;
    let d_ramp = 2.0 * compute_accel_distance(t_jerk, t_const, jerk_max, actual_accel);
    let d_cruise = (distance - d_ramp).max(0.0);
    let t_cruise = if vel > 1e-6 { d_cruise / vel } else { 0.0 };

    [t_jerk, t_const, t_jerk, t_cruise, t_jerk, t_const, t_jerk]
}

// ---------------------------------------------------------------------------
// Catmull-Rom Spline Segment
// ---------------------------------------------------------------------------

/// A Catmull-Rom spline segment for smooth corner transitions between waypoints.
///
/// Given 4 control points P0, P1, P2, P3, the spline interpolates between P1 and P2
/// with C1 continuity. Tangents at P1 and P2 are derived from the surrounding points.
#[derive(Debug, Clone)]
pub struct SplineSegment {
    /// The 4 control points (spline passes through p1 -> p2).
    p0: Vec3<NED>,
    p1: Vec3<NED>,
    p2: Vec3<NED>,
    p3: Vec3<NED>,
}

impl SplineSegment {
    /// Create a new Catmull-Rom spline segment from 4 control points.
    ///
    /// The spline interpolates between `p1` and `p2`. `p0` and `p3` influence
    /// the tangent directions at the endpoints.
    pub fn new(p0: Vec3<NED>, p1: Vec3<NED>, p2: Vec3<NED>, p3: Vec3<NED>) -> Self {
        Self { p0, p1, p2, p3 }
    }

    /// Evaluate the spline at parameter `t` in [0, 1].
    ///
    /// Returns `(position, tangent)` where tangent is the derivative dp/dt.
    pub fn evaluate(&self, t: f32) -> (Vec3<NED>, Vec3<NED>) {
        let t = t.clamp(0.0, 1.0);
        let t2 = t * t;
        let t3 = t2 * t;

        // Catmull-Rom basis matrix (tau = 0.5):
        // P(t) = 0.5 * [ (-t^3 + 2t^2 - t) * P0
        //              + (3t^3 - 5t^2 + 2) * P1
        //              + (-3t^3 + 4t^2 + t) * P2
        //              + (t^3 - t^2)        * P3 ]

        let c0 = -t3 + 2.0 * t2 - t;
        let c1 = 3.0 * t3 - 5.0 * t2 + 2.0;
        let c2 = -3.0 * t3 + 4.0 * t2 + t;
        let c3 = t3 - t2;

        let pos = (self.p0 * c0 + self.p1 * c1 + self.p2 * c2 + self.p3 * c3) * 0.5;

        // Derivative: dp/dt
        let dc0 = -3.0 * t2 + 4.0 * t - 1.0;
        let dc1 = 9.0 * t2 - 10.0 * t;
        let dc2 = -9.0 * t2 + 8.0 * t + 1.0;
        let dc3 = 3.0 * t2 - 2.0 * t;

        let tangent = (self.p0 * dc0 + self.p1 * dc1 + self.p2 * dc2 + self.p3 * dc3) * 0.5;

        (pos, tangent)
    }

    /// Approximate arc length of the spline using numerical integration.
    pub fn arc_length(&self, steps: u32) -> f32 {
        let n = steps.max(4);
        let dt = 1.0 / n as f32;
        let mut length: f32 = 0.0;
        let mut prev = self.p1;
        for i in 1..=n {
            let t = i as f32 * dt;
            let (p, _) = self.evaluate(t);
            length += (p - prev).length();
            prev = p;
        }
        length
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f32, b: f32, tol: f32) -> bool {
        libm::fabsf(a - b) < tol
    }

    // -----------------------------------------------------------------------
    // S-Curve tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_long_segment_full_profile() {
        // 100m segment — plenty of room for full 7-phase profile
        let start = Vec3::<NED>::new(0.0, 0.0, 0.0);
        let end = Vec3::<NED>::new(100.0, 0.0, 0.0);
        let mut seg = SCurveSegment::new(start, end, 1.0, 2.5, 5.0);

        // All 7 phases should have nonzero time (or at least accel+cruise+decel)
        let t = seg.phase_durations();
        let total: f32 = t.iter().sum();
        assert!(total > 0.0, "Total time should be positive");

        // Jerk phases should be equal
        assert!(approx_eq(t[0], t[2], 1e-4), "t1 != t3: {} vs {}", t[0], t[2]);
        assert!(approx_eq(t[0], t[4], 1e-4), "t1 != t5: {} vs {}", t[0], t[4]);
        assert!(approx_eq(t[0], t[6], 1e-4), "t1 != t7: {} vs {}", t[0], t[6]);
        // Constant accel phases should be equal
        assert!(approx_eq(t[1], t[5], 1e-4), "t2 != t6: {} vs {}", t[1], t[5]);
        // Cruise phase should exist for 100m segment
        assert!(t[3] > 0.0, "Cruise phase should be > 0 for 100m segment");

        // Simulate through the profile
        let dt = 0.01;
        let mut max_vel: f32 = 0.0;
        let mut final_pos = Vec3::<NED>::zero();
        let mut steps = 0;

        while !seg.is_complete() && steps < 100_000 {
            let (pos, vel, _progress) = seg.advance(dt);
            let v = vel.length();
            if v > max_vel {
                max_vel = v;
            }
            final_pos = pos;
            steps += 1;
        }

        // Should reach near max velocity
        assert!(
            approx_eq(max_vel, 5.0, 0.15),
            "Peak velocity should be ~5.0 m/s, got {}",
            max_vel
        );

        // Should end near the target
        let end_error = (final_pos - end).length();
        assert!(
            end_error < 0.5,
            "Final position error too large: {} m",
            end_error
        );

        // Should end with near-zero velocity
        assert!(
            seg.current_velocity() < 0.2,
            "Final velocity should be ~0, got {}",
            seg.current_velocity()
        );
    }

    #[test]
    fn test_short_segment_reduced_profile() {
        // 2m segment — can NOT reach 5 m/s
        let start = Vec3::<NED>::new(0.0, 0.0, 0.0);
        let end = Vec3::<NED>::new(0.0, 2.0, 0.0);
        let mut seg = SCurveSegment::new(start, end, 1.0, 2.5, 5.0);

        let t = seg.phase_durations();
        let total: f32 = t.iter().sum();
        assert!(total > 0.0, "Short segment should still have nonzero time");

        // Simulate
        let dt = 0.01;
        let mut max_vel: f32 = 0.0;
        let mut steps = 0;

        while !seg.is_complete() && steps < 100_000 {
            let (_pos, vel, _progress) = seg.advance(dt);
            let v = vel.length();
            if v > max_vel {
                max_vel = v;
            }
            steps += 1;
        }

        // Peak velocity should be much less than 5.0
        assert!(
            max_vel < 3.0,
            "Peak velocity on 2m segment should be < 3.0, got {}",
            max_vel
        );
        assert!(max_vel > 0.1, "Should reach some velocity, got {}", max_vel);

        // Remaining distance should be ~0
        assert!(
            seg.remaining_distance() < 0.5,
            "Should have covered most of the distance, remaining: {}",
            seg.remaining_distance()
        );
    }

    #[test]
    fn test_zero_length_segment() {
        let p = Vec3::<NED>::new(10.0, 20.0, -5.0);
        let mut seg = SCurveSegment::new(p, p, 1.0, 2.5, 5.0);

        // Should be immediately complete
        assert!(seg.is_complete(), "Zero-length segment should be complete");
        assert!(
            approx_eq(seg.remaining_distance(), 0.0, 1e-6),
            "Remaining distance should be 0"
        );

        // Advance should return the point itself
        let (pos, vel, progress) = seg.advance(0.1);
        assert!(
            (pos - p).length() < 1e-4,
            "Position should be at the point"
        );
        assert!(vel.length() < 1e-4, "Velocity should be zero");
        assert!(
            approx_eq(progress, 1.0, 1e-4),
            "Progress should be 1.0"
        );
    }

    #[test]
    fn test_velocity_profile_smoothness() {
        // Verify velocity is smooth (no sudden jumps) by checking that
        // the velocity change between timesteps is bounded by accel_max * dt + margin
        let start = Vec3::<NED>::new(0.0, 0.0, 0.0);
        let end = Vec3::<NED>::new(50.0, 0.0, 0.0);
        let accel_max = 2.5;
        let mut seg = SCurveSegment::new(start, end, 1.0, accel_max, 5.0);

        let dt = 0.01;
        let mut prev_vel: f32 = 0.0;
        let mut steps = 0;
        let max_delta_v = accel_max * dt + 0.1; // margin for discrete integration

        while !seg.is_complete() && steps < 100_000 {
            let (_pos, vel_vec, _progress) = seg.advance(dt);
            let v = vel_vec.length();
            let delta_v = libm::fabsf(v - prev_vel);
            assert!(
                delta_v < max_delta_v,
                "Velocity jump too large at step {}: delta_v={}, limit={}",
                steps,
                delta_v,
                max_delta_v,
            );
            prev_vel = v;
            steps += 1;
        }
    }

    #[test]
    fn test_3d_diagonal_segment() {
        // Diagonal segment in 3D
        let start = Vec3::<NED>::new(0.0, 0.0, 0.0);
        let end = Vec3::<NED>::new(30.0, 40.0, -10.0);
        let length = (end - start).length(); // ~51.0m
        let mut seg = SCurveSegment::new(start, end, 1.0, 2.5, 5.0);

        // Direction should be unit vector
        let dir = (end - start).normalized();

        let dt = 0.01;
        let mut final_pos = start;
        let mut steps = 0;

        while !seg.is_complete() && steps < 100_000 {
            let (pos, vel, _) = seg.advance(dt);
            // Velocity should be parallel to direction
            if vel.length() > 0.1 {
                let vel_dir = vel.normalized();
                let dot = vel_dir.dot(&dir);
                assert!(
                    dot > 0.99,
                    "Velocity not parallel to direction at step {}: dot={}",
                    steps,
                    dot
                );
            }
            final_pos = pos;
            steps += 1;
        }

        let end_error = (final_pos - end).length();
        assert!(
            end_error < 0.5,
            "Final position error: {} m (length was {})",
            end_error,
            length
        );
    }

    // -----------------------------------------------------------------------
    // Spline tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_spline_endpoints() {
        // Catmull-Rom: at t=0 should be at p1, at t=1 should be at p2
        let p0 = Vec3::<NED>::new(-10.0, 0.0, 0.0);
        let p1 = Vec3::<NED>::new(0.0, 0.0, 0.0);
        let p2 = Vec3::<NED>::new(10.0, 5.0, 0.0);
        let p3 = Vec3::<NED>::new(20.0, 5.0, 0.0);

        let spline = SplineSegment::new(p0, p1, p2, p3);

        let (pos0, _) = spline.evaluate(0.0);
        let (pos1, _) = spline.evaluate(1.0);

        assert!(
            (pos0 - p1).length() < 1e-4,
            "Spline at t=0 should be at p1, got {:?}",
            pos0
        );
        assert!(
            (pos1 - p2).length() < 1e-4,
            "Spline at t=1 should be at p2, got {:?}",
            pos1
        );
    }

    #[test]
    fn test_spline_smooth_curve() {
        // Spline through 4 points forming an S-shape
        let p0 = Vec3::<NED>::new(0.0, 0.0, 0.0);
        let p1 = Vec3::<NED>::new(10.0, 0.0, 0.0);
        let p2 = Vec3::<NED>::new(20.0, 10.0, 0.0);
        let p3 = Vec3::<NED>::new(30.0, 10.0, 0.0);

        let spline = SplineSegment::new(p0, p1, p2, p3);

        // Sample along the spline and verify smoothness
        let steps = 100;
        let mut prev_pos = p1;
        let mut total_length: f32 = 0.0;

        for i in 1..=steps {
            let t = i as f32 / steps as f32;
            let (pos, tangent) = spline.evaluate(t);

            // Position should change smoothly
            let delta = (pos - prev_pos).length();
            total_length += delta;

            // Tangent should be nonzero (curve is always moving)
            assert!(
                tangent.length() > 1e-4,
                "Tangent too small at t={}: {:?}",
                t,
                tangent
            );

            prev_pos = pos;
        }

        // Arc length should be in a reasonable range (straight line ~14.1m, curve longer)
        let straight_dist = (p2 - p1).length();
        assert!(
            total_length > straight_dist * 0.9,
            "Arc length {} too short (straight = {})",
            total_length,
            straight_dist
        );
        assert!(
            total_length < straight_dist * 2.0,
            "Arc length {} too long (straight = {})",
            total_length,
            straight_dist
        );
    }

    #[test]
    fn test_spline_arc_length() {
        // Straight-line case: p0-p1-p2-p3 all collinear
        let p0 = Vec3::<NED>::new(0.0, 0.0, 0.0);
        let p1 = Vec3::<NED>::new(10.0, 0.0, 0.0);
        let p2 = Vec3::<NED>::new(20.0, 0.0, 0.0);
        let p3 = Vec3::<NED>::new(30.0, 0.0, 0.0);

        let spline = SplineSegment::new(p0, p1, p2, p3);
        let arc = spline.arc_length(100);
        let straight = (p2 - p1).length(); // 10.0

        assert!(
            approx_eq(arc, straight, 0.1),
            "Collinear arc length should equal straight distance: {} vs {}",
            arc,
            straight
        );
    }

    #[test]
    fn test_spline_tangent_direction() {
        // At t=0, tangent should be in the direction from p0 toward p2 (Catmull-Rom property)
        let p0 = Vec3::<NED>::new(0.0, 0.0, 0.0);
        let p1 = Vec3::<NED>::new(10.0, 0.0, 0.0);
        let p2 = Vec3::<NED>::new(20.0, 10.0, 0.0);
        let p3 = Vec3::<NED>::new(30.0, 10.0, 0.0);

        let spline = SplineSegment::new(p0, p1, p2, p3);
        let (_, tangent_0) = spline.evaluate(0.0);

        // Catmull-Rom tangent at p1 = 0.5 * (p2 - p0)
        let expected_tangent = (p2 - p0) * 0.5;
        let error = (tangent_0 - expected_tangent).length();
        assert!(
            error < 1e-4,
            "Tangent at t=0 should be 0.5*(p2-p0), error={}",
            error
        );
    }
}

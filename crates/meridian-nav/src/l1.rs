//! L1 lateral guidance controller for fixed-wing and rover path following.
//!
//! Computes the lateral acceleration needed to track a path between two waypoints.
//! Source: libraries/AP_L1_Control/AP_L1_Control.cpp
//!
//! The L1 controller tracks a reference point on the desired path that is L1 distance
//! ahead of the vehicle. The lateral acceleration steers toward this point.

use meridian_math::Vec3;
use meridian_math::frames::NED;

/// L1 controller configuration.
#[derive(Debug, Clone, Copy)]
pub struct L1Params {
    /// L1 period (seconds). Source: NAVL1_PERIOD, default 17.0
    pub period: f32,
    /// L1 damping ratio. Source: NAVL1_DAMPING, default 0.75
    pub damping: f32,
    /// Whether reverse mode is active (for reverse taxiing). Source: NAV_REVERSE
    pub reverse: bool,
}

impl Default for L1Params {
    fn default() -> Self {
        Self { period: 17.0, damping: 0.75, reverse: false }
    }
}

/// Crosstrack integral gain for persistent error compensation.
const XTRACK_I: f32 = 0.02;
/// Maximum crosstrack integral value (radians, matching ArduPilot).
/// ArduPilot integrates Nu1 (angle in radians) capped to +/-0.1 rad.
const XTRACK_I_MAX: f32 = 0.1;

/// L1 lateral guidance controller.
pub struct L1Controller {
    pub params: L1Params,
    /// Last computed lateral acceleration (m/s²).
    pub lateral_accel: f32,
    /// Last computed desired bearing (radians from north).
    pub nav_bearing: f32,
    /// Accumulated crosstrack integral (radians) for trim/wind compensation.
    /// ArduPilot integrates in radians (Nu1) capped to +/-0.1 rad.
    pub xtrack_integral: f32,
    /// Whether the last update triggered a waypoint advance due to overshoot.
    pub wp_overshoot: bool,
    /// Last computed Nu angle (radians) for prevent_indecision logic.
    last_nu: f32,
}

impl L1Controller {
    pub fn new() -> Self {
        Self {
            params: L1Params::default(),
            lateral_accel: 0.0,
            nav_bearing: 0.0,
            xtrack_integral: 0.0,
            wp_overshoot: false,
            last_nu: 0.0,
        }
    }

    /// Reset the crosstrack integral (call on waypoint change or mode switch).
    pub fn reset_integral(&mut self) {
        self.xtrack_integral = 0.0;
    }

    /// Update for waypoint tracking: compute lateral acceleration to follow
    /// the path from prev_wp to next_wp.
    ///
    /// Source: AP_L1_Control::update_waypoint()
    pub fn update_waypoint(
        &mut self,
        position: &Vec3<NED>,
        velocity: &Vec3<NED>,
        prev_wp: &Vec3<NED>,
        next_wp: &Vec3<NED>,
    ) -> f32 {
        self.update_waypoint_ext(position, velocity, prev_wp, next_wp, 0.0, 0.0)
    }

    /// Update for waypoint tracking with a time step for integral accumulation
    /// and an optional dist_min floor.
    ///
    /// `dt`: time step in seconds. Pass 0.0 to disable integral accumulation.
    /// `dist_min`: minimum L1 distance floor (meters). 0.0 = use default 0.1m floor.
    ///             Source: AP_L1_Control::update_waypoint(dist_min).
    pub fn update_waypoint_ext(
        &mut self,
        position: &Vec3<NED>,
        velocity: &Vec3<NED>,
        prev_wp: &Vec3<NED>,
        next_wp: &Vec3<NED>,
        dt: f32,
        dist_min: f32,
    ) -> f32 {
        self.wp_overshoot = false;

        let groundspeed = libm::sqrtf(velocity.x * velocity.x + velocity.y * velocity.y);
        if groundspeed < 0.1 {
            self.lateral_accel = 0.0;
            return 0.0;
        }

        // L1 distance: proportional to groundspeed and period
        // Source: L1 = (1/pi) * damping * period * groundspeed
        let l1_dist_raw = (1.0 / core::f32::consts::PI)
            * self.params.damping * self.params.period * groundspeed;
        // GAP 4: Apply configurable dist_min floor
        let floor = if dist_min > 0.0 { dist_min } else { 0.1 };
        let l1_dist = l1_dist_raw.max(floor);

        // Path direction (unit vector from prev to next)
        let ab = *next_wp - *prev_wp;
        let ab_len = libm::sqrtf(ab.x * ab.x + ab.y * ab.y);
        if ab_len < 0.1 {
            return self.update_loiter(position, velocity, next_wp, 20.0);
        }
        let ab_unit = Vec3::<NED>::new(ab.x / ab_len, ab.y / ab_len, 0.0);

        // Vector from prev_wp to vehicle
        let a_to_veh = *position - *prev_wp;

        // Along-track distance
        let along = a_to_veh.x * ab_unit.x + a_to_veh.y * ab_unit.y;

        // Cross-track error (positive = right of path)
        let cross = a_to_veh.x * ab_unit.y - a_to_veh.y * ab_unit.x;

        // --- Prevent indecision: detect overshoot ---
        if along > ab_len {
            let vel_along = velocity.x * ab_unit.x + velocity.y * ab_unit.y;
            if vel_along < 0.0 {
                self.wp_overshoot = true;
            }
        }

        // Bearing to next waypoint
        let target_x = next_wp.x - position.x;
        let target_y = next_wp.y - position.y;
        let target_bearing = libm::atan2f(target_y, target_x);

        // Vehicle bearing (account for reverse mode, GAP 5)
        let mut veh_bearing = libm::atan2f(velocity.y, velocity.x);
        if self.params.reverse {
            veh_bearing = wrap_pi(veh_bearing + core::f32::consts::PI);
        }

        // Bearing error (Nu angle)
        let mut nu = target_bearing - veh_bearing;
        nu = wrap_pi(nu);

        // --- GAP 2: prevent_indecision — ArduPilot's anti-oscillation Nu latch ---
        let pi_09 = 0.9 * core::f32::consts::PI;
        if libm::fabsf(nu) > pi_09
            && libm::fabsf(self.last_nu) > pi_09
            && nu * self.last_nu < 0.0
        {
            nu = self.last_nu;
        }
        self.last_nu = nu;

        // K_L1 coefficient: ArduPilot uses K_L1 = 4.0 * damping^2
        // With default damping=0.75: K_L1 = 4 * 0.5625 = 2.25
        let k_l1 = 4.0 * self.params.damping * self.params.damping;
        // Clamp bearing error to ±π/2 before computing sin, matching ArduPilot's
        // constrain_float(Nu, -1.5708, +1.5708). Prevents divergence at extreme angles.
        let clamped_nu = nu.clamp(-core::f32::consts::FRAC_PI_2, core::f32::consts::FRAC_PI_2);
        let sin_nu = libm::sinf(clamped_nu);
        self.lateral_accel = k_l1 * groundspeed * groundspeed * sin_nu / l1_dist;
        self.nav_bearing = target_bearing;

        // --- GAP 3 & 6: Crosstrack integral in radians, conditional integration ---
        // ArduPilot only integrates when |Nu1| < radians(5).
        let nu1 = libm::atan2f(cross, l1_dist);
        if dt > 0.0 && libm::fabsf(nu1) < 5.0 * core::f32::consts::PI / 180.0 {
            self.xtrack_integral += nu1 * dt;
            if self.xtrack_integral > XTRACK_I_MAX {
                self.xtrack_integral = XTRACK_I_MAX;
            } else if self.xtrack_integral < -XTRACK_I_MAX {
                self.xtrack_integral = -XTRACK_I_MAX;
            }
            self.lateral_accel += XTRACK_I * self.xtrack_integral
                * groundspeed * groundspeed / l1_dist;
        }

        self.lateral_accel
    }

    /// Backwards-compatible wrapper: update_waypoint_dt with dt but no dist_min.
    pub fn update_waypoint_dt(
        &mut self,
        position: &Vec3<NED>,
        velocity: &Vec3<NED>,
        prev_wp: &Vec3<NED>,
        next_wp: &Vec3<NED>,
        dt: f32,
    ) -> f32 {
        self.update_waypoint_ext(position, velocity, prev_wp, next_wp, dt, 0.0)
    }

    /// Update for loiter (orbit) around a point.
    pub fn update_loiter(
        &mut self,
        position: &Vec3<NED>,
        velocity: &Vec3<NED>,
        center: &Vec3<NED>,
        radius: f32,
    ) -> f32 {
        let groundspeed = libm::sqrtf(velocity.x * velocity.x + velocity.y * velocity.y);
        if groundspeed < 0.1 {
            self.lateral_accel = 0.0;
            return 0.0;
        }

        let to_veh = *position - *center;
        let dist = libm::sqrtf(to_veh.x * to_veh.x + to_veh.y * to_veh.y);

        if dist < 0.1 {
            self.lateral_accel = 0.0;
            return 0.0;
        }

        let centripetal = groundspeed * groundspeed / radius.abs();
        let radial_err = dist - radius.abs();
        let sign = if radius > 0.0 { 1.0 } else { -1.0 };
        self.lateral_accel = sign * centripetal + radial_err * 0.5;

        self.lateral_accel
    }

    /// Get the desired bank angle for a fixed-wing aircraft.
    pub fn bank_angle(&self, gravity: f32) -> f32 {
        libm::atan2f(self.lateral_accel, gravity)
    }
}

/// Wrap angle to [-PI, PI] range.
fn wrap_pi(mut angle: f32) -> f32 {
    use core::f32::consts::PI;
    const TWO_PI: f32 = 2.0 * PI;
    angle = libm::fmodf(angle + PI, TWO_PI);
    if angle < 0.0 { angle += TWO_PI; }
    angle - PI
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_l1_basic_tracking() {
        let mut l1 = L1Controller::new();
        let pos = Vec3::<NED>::new(0.0, 5.0, 0.0);
        let vel = Vec3::<NED>::new(15.0, 0.0, 0.0);
        let prev = Vec3::<NED>::new(0.0, 0.0, 0.0);
        let next = Vec3::<NED>::new(100.0, 0.0, 0.0);

        let accel = l1.update_waypoint(&pos, &vel, &prev, &next);
        assert!(accel < 0.0, "expected negative accel, got {}", accel);
    }

    #[test]
    fn test_l1_k_l1_coefficient() {
        let l1 = L1Controller::new();
        // K_L1 = 4 * damping^2
        // With default damping=0.75: 4 * 0.5625 = 2.25
        let k_l1 = 4.0 * l1.params.damping * l1.params.damping;
        assert!((k_l1 - 2.25).abs() < 0.001);
    }

    #[test]
    fn test_l1_integral_only_when_aligned() {
        let mut l1 = L1Controller::new();
        let pos = Vec3::<NED>::new(50.0, 50.0, 0.0);
        let vel = Vec3::<NED>::new(15.0, 0.0, 0.0);
        let prev = Vec3::<NED>::new(0.0, 0.0, 0.0);
        let next = Vec3::<NED>::new(200.0, 0.0, 0.0);

        for _ in 0..10 {
            l1.update_waypoint_dt(&pos, &vel, &prev, &next, 0.1);
        }
        assert!(l1.xtrack_integral.abs() < 0.01,
            "integral should not accumulate when far off track, got {}", l1.xtrack_integral);
    }

    #[test]
    fn test_l1_integral_accumulates_when_aligned() {
        let mut l1 = L1Controller::new();
        let pos = Vec3::<NED>::new(50.0, 0.5, 0.0);
        let vel = Vec3::<NED>::new(15.0, 0.0, 0.0);
        let prev = Vec3::<NED>::new(0.0, 0.0, 0.0);
        let next = Vec3::<NED>::new(200.0, 0.0, 0.0);

        for _ in 0..20 {
            l1.update_waypoint_dt(&pos, &vel, &prev, &next, 0.1);
        }
        assert!(l1.xtrack_integral.abs() > 0.0001,
            "integral should accumulate when aligned, got {}", l1.xtrack_integral);
    }

    #[test]
    fn test_l1_integral_windup_cap() {
        let mut l1 = L1Controller::new();
        let pos = Vec3::<NED>::new(50.0, 0.3, 0.0);
        let vel = Vec3::<NED>::new(15.0, 0.0, 0.0);
        let prev = Vec3::<NED>::new(0.0, 0.0, 0.0);
        let next = Vec3::<NED>::new(200.0, 0.0, 0.0);

        for _ in 0..2000 {
            l1.update_waypoint_dt(&pos, &vel, &prev, &next, 0.1);
        }
        assert!(libm::fabsf(l1.xtrack_integral) <= XTRACK_I_MAX + 0.001);
    }

    #[test]
    fn test_l1_integral_resets() {
        let mut l1 = L1Controller::new();
        l1.xtrack_integral = 0.05;
        l1.reset_integral();
        assert_eq!(l1.xtrack_integral, 0.0);
    }

    #[test]
    fn test_l1_prevent_indecision_overshoot() {
        let mut l1 = L1Controller::new();
        let prev = Vec3::<NED>::new(0.0, 0.0, 0.0);
        let next = Vec3::<NED>::new(100.0, 0.0, 0.0);
        let pos = Vec3::<NED>::new(110.0, 2.0, 0.0);
        let vel = Vec3::<NED>::new(-10.0, 0.0, 0.0);

        l1.update_waypoint(&pos, &vel, &prev, &next);
        assert!(l1.wp_overshoot);
    }

    #[test]
    fn test_l1_zero_speed() {
        let mut l1 = L1Controller::new();
        let pos = Vec3::<NED>::new(0.0, 0.0, 0.0);
        let vel = Vec3::<NED>::new(0.0, 0.0, 0.0);
        let prev = Vec3::<NED>::new(0.0, 0.0, 0.0);
        let next = Vec3::<NED>::new(100.0, 0.0, 0.0);

        let accel = l1.update_waypoint(&pos, &vel, &prev, &next);
        assert_eq!(accel, 0.0);
    }
}

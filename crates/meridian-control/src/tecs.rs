//! Total Energy Control System (TECS) for fixed-wing aircraft.
//!
//! Coordinates throttle and pitch angle to simultaneously control airspeed and altitude.
//! Throttle controls total energy (kinetic + potential).
//! Pitch angle controls energy distribution between KE and PE.
//!
//! Includes:
//! - Underspeed protection (safety-critical)
//! - Flare mode for landing
//! - Bad descent detection
//! - Speed/height weighting (ste_w applied correctly)
//!
//! Source: ArduPilot libraries/AP_TECS/AP_TECS.cpp (Paul Riseborough, 2013)

/// TECS configuration parameters.
#[derive(Debug, Clone, Copy)]
pub struct TecsParams {
    /// Climb rate time constant (s). Source: TECS_CLMB_MAX, default 5.0 m/s
    pub max_climb_rate: f32,
    /// Sink rate limit (m/s, positive). Source: TECS_SINK_MIN, default 2.0
    pub min_sink_rate: f32,
    /// Maximum sink rate (m/s). Source: TECS_SINK_MAX, default 5.0
    pub max_sink_rate: f32,
    /// Speed/height weight. 0=height priority, 2=speed priority, 1=equal.
    /// Source: TECS_SPDWEIGHT, default 1.0
    pub spdweight: f32,
    /// Time constant for height filter (s). Source: TECS_PTCH_DAMP, default 0.0
    pub pitch_damping: f32,
    /// Throttle damping. Source: TECS_THR_DAMP, default 0.5
    pub throttle_damping: f32,
    /// Height-to-speed ratio for underspeed protection.
    /// Source: TECS_VERT_ACC, default 7.0 m/s^2
    pub vert_accel_limit: f32,
    /// Time constant for integrators (s). Source: TECS_INTEG_GAIN, default 0.1
    pub integrator_gain: f32,
    /// Throttle limits
    pub throttle_max: f32, // 0..1, default 0.75
    pub throttle_min: f32, // 0..1, default 0.0 (or negative for gliders)
    pub throttle_cruise: f32, // 0..1, default 0.5
    /// Pitch limits (radians)
    pub pitch_max: f32,    // default 0.35 (20 deg)
    pub pitch_min: f32,    // default -0.35 (-20 deg)
    /// Minimum airspeed (m/s). Source: ARSPD_FBW_MIN, default 9.0
    pub airspeed_min: f32,
    /// Maximum airspeed (m/s). Source: ARSPD_FBW_MAX, default 22.0
    pub airspeed_max: f32,
    /// Time constant for the controller. Source: TECS_TIME_CONST, default 5.0
    pub time_const: f32,
    /// Flare mode parameters
    pub land_time_const: f32,   // TECS_LAND_TCONST, default 2.0
    pub land_sink_rate: f32,    // TECS_LAND_SINK, default 0.25 m/s
    pub flare_height: f32,      // TECS_FLARE_HGT, default 1.0 m
    pub land_approach_speed: f32, // TECS_LAND_ARSPD, 0 = use normal min
}

impl Default for TecsParams {
    fn default() -> Self {
        Self {
            max_climb_rate: 5.0,
            min_sink_rate: 2.0,
            max_sink_rate: 5.0,
            spdweight: 1.0,
            pitch_damping: 0.0,
            throttle_damping: 0.5,
            vert_accel_limit: 7.0,
            integrator_gain: 0.1,
            throttle_max: 0.75,
            throttle_min: 0.0,
            throttle_cruise: 0.5,
            pitch_max: 0.35,
            pitch_min: -0.35,
            airspeed_min: 9.0,
            airspeed_max: 22.0,
            time_const: 5.0,
            land_time_const: 2.0,
            land_sink_rate: 0.25,
            flare_height: 1.0,
            land_approach_speed: 0.0,
        }
    }
}

/// TECS output.
#[derive(Debug, Clone, Copy)]
pub struct TecsOutput {
    /// Throttle demand (0..1 or -1..1 with reverse thrust).
    pub throttle: f32,
    /// Pitch demand (radians, positive = nose up).
    pub pitch: f32,
}

/// TECS flags for safety states.
#[derive(Debug, Clone, Copy, Default)]
pub struct TecsFlags {
    /// True when airspeed is below minimum safe speed.
    pub underspeed: bool,
    /// True when sinking too fast (bad descent prevention).
    pub bad_descent: bool,
    /// True when in landing flare.
    pub in_flare: bool,
}

/// Total Energy Control System.
pub struct Tecs {
    pub params: TecsParams,

    // Internal state
    height_estimate: f32,
    height_rate: f32,
    speed_estimate: f32,
    speed_rate: f32,

    // Demands
    height_demand: f32,
    speed_demand: f32,

    // Energy errors
    ste_error: f32,      // total energy error
    seb_error: f32,      // energy balance error
    ste_rate_error: f32,
    seb_rate_error: f32,

    // Integrators
    throttle_integrator: f32,
    pitch_integrator: f32,

    // Filters
    height_filter: f32,
    speed_filter: f32,

    // Underspeed / bad descent
    pub flags: TecsFlags,
    /// Unconstrained pitch demand (for logging). Source: _pitch_dem_unc
    pitch_dem_unc: f32,
    /// Flight path angle estimate from baro+accel fusion (rad).
    dthetadt: f32,

    // Flare state
    flare_initialized: bool,
    flare_height_start: f32,
    flare_sink_rate: f32,

    initialized: bool,
}

const GRAVITY: f32 = 9.80665;

impl Tecs {
    pub fn new() -> Self {
        Self {
            params: TecsParams::default(),
            height_estimate: 0.0,
            height_rate: 0.0,
            speed_estimate: 0.0,
            speed_rate: 0.0,
            height_demand: 0.0,
            speed_demand: 0.0,
            ste_error: 0.0,
            seb_error: 0.0,
            ste_rate_error: 0.0,
            seb_rate_error: 0.0,
            throttle_integrator: 0.0,
            pitch_integrator: 0.0,
            height_filter: 0.0,
            speed_filter: 0.0,
            flags: TecsFlags::default(),
            pitch_dem_unc: 0.0,
            dthetadt: 0.0,
            flare_initialized: false,
            flare_height_start: 0.0,
            flare_sink_rate: 0.0,
            initialized: false,
        }
    }

    /// Update height and speed estimates from AHRS.
    /// Call at 50Hz.
    pub fn update_50hz(&mut self, altitude: f32, climb_rate: f32, airspeed: f32, accel_x: f32) {
        if !self.initialized {
            // Seed filters on first call
            self.height_filter = altitude;
            self.height_estimate = altitude;
            self.height_rate = climb_rate;
            self.speed_filter = airspeed;
            self.speed_estimate = airspeed;
            self.speed_rate = accel_x;
            self.initialized = true;
            return;
        }

        // 3rd-order complementary filter for height
        // Source: AP_TECS uses _hgtCompFiltOmega, configurable. Default 3.0 rad/s.
        let tau = self.params.time_const.max(1.0);
        let k1 = 3.0 / tau;
        let k2 = 3.0 / (tau * tau);
        let height_error = altitude - self.height_filter;
        self.height_rate = climb_rate; // use direct measurement + correction
        self.height_rate += height_error * k2 * 0.02;
        self.height_filter += (self.height_rate + height_error * k1) * 0.02;
        self.height_estimate = self.height_filter;

        // 2nd-order filter for speed (omega = 2.0 rad/s default)
        let speed_error = airspeed - self.speed_filter;
        self.speed_rate = accel_x;
        self.speed_filter += (self.speed_rate + speed_error * 2.0) * 0.02;
        self.speed_estimate = self.speed_filter;

        // Flight path angle estimate (for logging / bad descent detection)
        if self.speed_estimate > 1.0 {
            self.dthetadt = libm::atan2f(self.height_rate, self.speed_estimate);
        }
    }

    /// Check and set underspeed and bad descent flags.
    /// Source: AP_TECS::_detect_underspeed() and AP_TECS::_detect_bad_descent()
    fn detect_underspeed(&mut self) {
        // Underspeed: airspeed below minimum
        self.flags.underspeed = self.speed_estimate < self.params.airspeed_min * 0.9;

        // Bad descent: sinking faster than max_sink_rate and below demanded altitude
        self.flags.bad_descent = self.height_rate < -self.params.max_sink_rate
            && self.height_estimate < self.height_demand;
    }

    /// Update throttle and pitch demands.
    /// Call at 10-50Hz after update_50hz.
    ///
    /// `height_demand`: target altitude (m)
    /// `speed_demand`: target airspeed (m/s, true airspeed)
    /// `dt`: timestep
    pub fn update(&mut self, height_demand: f32, speed_demand: f32, dt: f32) -> TecsOutput {
        if dt < 0.001 || !self.initialized {
            return TecsOutput { throttle: self.params.throttle_cruise, pitch: 0.0 };
        }

        self.height_demand = height_demand;
        self.speed_demand = speed_demand;

        // Detect safety conditions
        self.detect_underspeed();

        // Effective demands -- modified by underspeed protection
        let (eff_height_demand, eff_speed_demand) = if self.flags.underspeed {
            // UNDERSPEED PROTECTION: override height demand to force descent for speed recovery.
            // Source: AP_TECS underspeed logic -- prioritize regaining airspeed.
            // Reduce height demand to trade PE for KE.
            let speed_deficit = self.params.airspeed_min - self.speed_estimate;
            let height_reduction = speed_deficit * speed_deficit / (2.0 * GRAVITY);
            let reduced_height = height_demand - height_reduction.max(0.0);
            (reduced_height, self.params.airspeed_min)
        } else if self.flags.bad_descent {
            // Bad descent: increase speed demand to help pull out of steep descent
            let boosted_speed = speed_demand.max(self.params.airspeed_min * 1.1);
            (height_demand, boosted_speed)
        } else {
            (height_demand, speed_demand)
        };

        // ---- Energy errors ----
        // Specific total energy = g*h + 0.5*v^2
        let ste_demand = GRAVITY * eff_height_demand + 0.5 * eff_speed_demand * eff_speed_demand;
        let ste_actual = GRAVITY * self.height_estimate + 0.5 * self.speed_estimate * self.speed_estimate;
        self.ste_error = ste_demand - ste_actual;

        // Specific energy balance error
        let seb_demand = GRAVITY * eff_height_demand - 0.5 * eff_speed_demand * eff_speed_demand;
        let seb_actual = GRAVITY * self.height_estimate - 0.5 * self.speed_estimate * self.speed_estimate;
        self.seb_error = seb_demand - seb_actual;

        // Energy rate errors
        let climb_demand = (eff_height_demand - self.height_estimate).clamp(
            -self.params.max_sink_rate, self.params.max_climb_rate);
        let speed_change_demand = (eff_speed_demand - self.speed_estimate).clamp(-2.0, 2.0);

        let ste_rate_demand = GRAVITY * climb_demand + eff_speed_demand * speed_change_demand;
        let ste_rate_actual = GRAVITY * self.height_rate + self.speed_estimate * self.speed_rate;
        self.ste_rate_error = ste_rate_demand - ste_rate_actual;

        let seb_rate_demand = GRAVITY * climb_demand - eff_speed_demand * speed_change_demand;
        let seb_rate_actual = GRAVITY * self.height_rate - self.speed_estimate * self.speed_rate;
        self.seb_rate_error = seb_rate_demand - seb_rate_actual;

        // ---- Throttle (controls total energy) ----
        let max_climb = self.params.max_climb_rate.max(0.1);
        let thr_p = self.ste_error * 0.5 / (GRAVITY * max_climb);
        let thr_d = self.ste_rate_error * self.params.throttle_damping / (GRAVITY * max_climb);

        self.throttle_integrator += self.ste_error * self.params.integrator_gain * dt;
        self.throttle_integrator = self.throttle_integrator.clamp(-0.3, 0.3);

        let mut throttle = self.params.throttle_cruise + thr_p + self.throttle_integrator + thr_d;

        // Underspeed: force max throttle to recover
        if self.flags.underspeed {
            throttle = self.params.throttle_max;
        }

        let throttle = throttle.clamp(self.params.throttle_min, self.params.throttle_max);

        // ---- Pitch (controls energy balance) ----
        // Speed/height weighting -- FIX: apply BOTH ste_w and seb_w correctly
        let w = self.params.spdweight.clamp(0.0, 2.0);
        let seb_w = if w > 1.0 { 2.0 - w } else { 1.0 };
        let ste_w = if w < 1.0 { w } else { 1.0 };

        // Weighted pitch demand using both SEB and STE contributions
        let pitch_seb = self.seb_error * seb_w * 0.5 / (GRAVITY * max_climb);
        let pitch_ste = self.ste_error * ste_w * 0.5 / (GRAVITY * max_climb);
        let pitch_p = pitch_seb + pitch_ste;
        let pitch_d = self.seb_rate_error * self.params.pitch_damping / (GRAVITY * max_climb);

        self.pitch_integrator += self.seb_error * self.params.integrator_gain * dt * 0.5;
        self.pitch_integrator = self.pitch_integrator.clamp(-0.2, 0.2);

        // Unconstrained pitch demand (for logging)
        self.pitch_dem_unc = pitch_p + self.pitch_integrator + pitch_d;

        // Underspeed: force pitch down to trade PE for KE
        let pitch = if self.flags.underspeed {
            self.pitch_dem_unc.min(0.0).clamp(self.params.pitch_min, self.params.pitch_max)
        } else {
            self.pitch_dem_unc.clamp(self.params.pitch_min, self.params.pitch_max)
        };

        TecsOutput { throttle, pitch }
    }

    /// Update TECS in flare mode for landing.
    /// Source: AP_TECS flare logic
    ///
    /// `height_above_ground`: height AGL (m)
    /// `speed_demand`: target approach speed (m/s)
    /// `distance_past_land`: horizontal distance past the land waypoint (m)
    /// `dt`: timestep
    pub fn update_flare(
        &mut self,
        height_above_ground: f32,
        speed_demand: f32,
        distance_past_land: f32,
        dt: f32,
    ) -> TecsOutput {
        if dt < 0.001 || !self.initialized {
            return TecsOutput { throttle: self.params.throttle_cruise, pitch: 0.0 };
        }

        self.flags.in_flare = true;

        // Initialize flare state on first call
        if !self.flare_initialized {
            self.flare_height_start = height_above_ground;
            self.flare_sink_rate = self.params.land_sink_rate;
            self.flare_initialized = true;
        }

        // Progressive sink rate ramp based on distance past land waypoint
        // Source: AP_TECS::_update_pitch() flare logic
        let sink_rate_target = if distance_past_land > 0.0 {
            // Increase sink rate as we pass the landing point
            self.params.land_sink_rate + distance_past_land * 0.02
        } else {
            self.params.land_sink_rate
        };
        self.flare_sink_rate = sink_rate_target.min(self.params.max_sink_rate);

        // Use reduced time constant for flare
        let tau = self.params.land_time_const.max(0.5);

        // Target height: ramp down from flare start to 0
        let height_target = if self.flare_height_start > 0.0 {
            (height_above_ground - self.flare_sink_rate * tau).max(0.0)
        } else {
            0.0
        };

        // Simple proportional pitch control to achieve desired sink rate
        let height_error = height_target - height_above_ground;
        let rate_error = -self.flare_sink_rate - self.height_rate; // desired vs actual climb rate

        let pitch = (height_error * 0.5 / tau + rate_error * self.params.pitch_damping)
            .clamp(self.params.pitch_min, 0.0); // Pitch should be zero or negative in flare

        // Reduce throttle to idle in flare (or approach speed reduced throttle)
        let throttle = if self.params.land_approach_speed > 0.0 {
            // Maintain some throttle for approach speed
            let speed_err = speed_demand.min(self.params.land_approach_speed) - self.speed_estimate;
            (self.params.throttle_cruise + speed_err * 0.1)
                .clamp(self.params.throttle_min, self.params.throttle_cruise)
        } else {
            self.params.throttle_min
        };

        TecsOutput { throttle, pitch }
    }

    /// Reset TECS state (on mode change).
    pub fn reset(&mut self) {
        self.throttle_integrator = 0.0;
        self.pitch_integrator = 0.0;
        self.ste_error = 0.0;
        self.seb_error = 0.0;
        self.flags = TecsFlags::default();
        self.flare_initialized = false;
        self.pitch_dem_unc = 0.0;
    }

    pub fn get_height_rate(&self) -> f32 { self.height_rate }
    pub fn get_speed_rate(&self) -> f32 { self.speed_rate }
    pub fn get_pitch_dem_unc(&self) -> f32 { self.pitch_dem_unc }
    pub fn get_dthetadt(&self) -> f32 { self.dthetadt }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tecs_level_flight() {
        let mut tecs = Tecs::new();
        // Simulate level flight at 20 m/s, 100m altitude
        for _ in 0..250 { // 5 seconds at 50Hz
            tecs.update_50hz(100.0, 0.0, 20.0, 0.0);
        }
        // Demand same altitude and speed -> throttle near cruise, pitch near 0
        let out = tecs.update(100.0, 20.0, 0.02);
        assert!((out.throttle - 0.5).abs() < 0.15,
            "Level flight throttle should be near cruise: {}", out.throttle);
        assert!(out.pitch.abs() < 0.1,
            "Level flight pitch should be near zero: {}", out.pitch);
    }

    #[test]
    fn test_tecs_climb() {
        let mut tecs = Tecs::new();
        for _ in 0..250 {
            tecs.update_50hz(100.0, 0.0, 20.0, 0.0);
        }
        // Demand higher altitude -> more throttle, positive pitch
        let out = tecs.update(120.0, 20.0, 0.02);
        assert!(out.throttle > 0.5, "Climbing should increase throttle: {}", out.throttle);
        assert!(out.pitch > 0.0, "Climbing should pitch up: {}", out.pitch);
    }

    #[test]
    fn test_tecs_speed_up() {
        let mut tecs = Tecs::new();
        for _ in 0..250 {
            tecs.update_50hz(100.0, 0.0, 20.0, 0.0);
        }
        // Demand more speed at same altitude -> more throttle, nose down (trade PE for KE)
        let out = tecs.update(100.0, 30.0, 0.02);
        assert!(out.throttle > 0.5, "Speeding up should increase throttle: {}", out.throttle);
    }

    #[test]
    fn test_tecs_throttle_limits() {
        let mut tecs = Tecs::new();
        for _ in 0..250 {
            tecs.update_50hz(100.0, 0.0, 20.0, 0.0);
        }
        // Extreme demand -> throttle should be clamped
        let out = tecs.update(500.0, 50.0, 0.02);
        assert!(out.throttle <= tecs.params.throttle_max);
        assert!(out.throttle >= tecs.params.throttle_min);
        assert!(out.pitch <= tecs.params.pitch_max);
        assert!(out.pitch >= tecs.params.pitch_min);
    }

    #[test]
    fn test_tecs_reset() {
        let mut tecs = Tecs::new();
        for _ in 0..250 {
            tecs.update_50hz(100.0, 0.0, 20.0, 0.0);
        }
        tecs.update(200.0, 30.0, 0.02); // build up integrators
        tecs.reset();
        assert_eq!(tecs.throttle_integrator, 0.0);
        assert_eq!(tecs.pitch_integrator, 0.0);
    }

    #[test]
    fn test_tecs_underspeed_protection() {
        let mut tecs = Tecs::new();
        tecs.params.airspeed_min = 10.0;
        // Simulate very slow speed -- below minimum
        for _ in 0..250 {
            tecs.update_50hz(100.0, 0.0, 5.0, 0.0); // 5 m/s, well below 10 m/s min
        }
        let out = tecs.update(110.0, 10.0, 0.02);

        // Should detect underspeed
        assert!(tecs.flags.underspeed, "Should detect underspeed at 5 m/s (min=10)");
        // Underspeed: max throttle, pitch <= 0 (nose down to recover speed)
        assert_eq!(out.throttle, tecs.params.throttle_max,
            "Underspeed should force max throttle");
        assert!(out.pitch <= 0.0,
            "Underspeed should pitch down or level: {}", out.pitch);
    }

    #[test]
    fn test_tecs_flare() {
        let mut tecs = Tecs::new();
        for _ in 0..250 {
            tecs.update_50hz(5.0, -1.0, 15.0, 0.0);
        }
        // Enter flare mode at 5m AGL
        let out = tecs.update_flare(5.0, 12.0, 0.0, 0.02);
        assert!(tecs.flags.in_flare, "Should be in flare mode");
        assert!(out.pitch <= 0.01, "Flare pitch should be near zero or negative: {}", out.pitch);
        assert!(out.throttle <= tecs.params.throttle_cruise,
            "Flare throttle should be reduced: {}", out.throttle);
    }
}

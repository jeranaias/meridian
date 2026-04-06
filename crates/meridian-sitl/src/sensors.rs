//! Simulated sensor outputs with configurable noise.
//!
//! Generates IMU, GPS, barometer, and magnetometer readings from physics state.
//! Noise models match ArduPilot SITL. Source: SIM_Aircraft.h:281-282

use meridian_math::{Vec3, Quaternion, Rotation};
use meridian_math::frames::{NED, Body};
use meridian_math::geodetic::LatLonAlt;
use meridian_types::time::Instant;
use meridian_types::messages::*;
use crate::physics::{PhysicsState, VehicleParams, GRAVITY};

/// Sensor noise parameters. Source: SIM_Aircraft.h
#[derive(Debug, Clone)]
pub struct NoiseParams {
    /// Gyro noise std dev (rad/s) at full throttle. Default: radians(0.1)
    pub gyro_noise: f32,
    /// Accel noise std dev (m/s²) at full throttle. Default: 0.3
    pub accel_noise: f32,
    /// Baro altitude noise std dev (m). Default: 0.3
    pub baro_noise: f32,
    /// GPS position noise std dev (m). Default: 0.5
    pub gps_pos_noise: f32,
    /// GPS velocity noise std dev (m/s). Default: 0.2
    pub gps_vel_noise: f32,
    /// Magnetometer noise std dev (gauss). Default: 0.01
    pub mag_noise: f32,
}

impl Default for NoiseParams {
    fn default() -> Self {
        Self {
            gyro_noise: 0.1 * core::f32::consts::PI / 180.0, // 0.1 deg/s
            accel_noise: 0.3,
            baro_noise: 0.3,
            gps_pos_noise: 0.5,
            gps_vel_noise: 0.2,
            mag_noise: 0.01,
        }
    }
}

/// Simple pseudo-random number generator (xorshift32).
/// Deterministic for reproducible SITL runs.
pub struct Rng {
    state: u32,
}

impl Rng {
    pub fn new(seed: u32) -> Self {
        Self { state: if seed == 0 { 1 } else { seed } }
    }

    /// Generate a uniform random f32 in [0, 1).
    pub fn uniform(&mut self) -> f32 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 17;
        self.state ^= self.state << 5;
        (self.state as f32) / (u32::MAX as f32)
    }

    /// Generate a standard normal (Gaussian) random f32 using Box-Muller.
    pub fn normal(&mut self) -> f32 {
        let u1 = self.uniform().max(1e-10); // avoid log(0)
        let u2 = self.uniform();
        libm::sqrtf(-2.0 * libm::logf(u1)) * libm::cosf(2.0 * core::f32::consts::PI * u2)
    }
}

/// Barometer ground calibration.
/// Averages the first N pressure samples to establish a ground reference.
/// All subsequent altitude readings are relative to this ground pressure.
#[derive(Debug, Clone)]
pub struct BaroGroundCal {
    /// Number of samples to average for calibration.
    pub cal_samples: u8,
    /// Accumulated pressure sum during calibration.
    pressure_sum: f32,
    /// Number of samples collected so far.
    sample_count: u8,
    /// Ground pressure after calibration (Pa).
    ground_pressure: f32,
    /// Whether calibration is complete.
    calibrated: bool,
}

impl BaroGroundCal {
    /// Create a new calibrator that averages `cal_samples` samples (default 5).
    pub fn new() -> Self {
        Self {
            cal_samples: 5,
            pressure_sum: 0.0,
            sample_count: 0,
            ground_pressure: 101325.0, // ISA sea level default
            calibrated: false,
        }
    }

    /// Feed a pressure sample. Returns true once calibration is complete.
    pub fn calibrate(&mut self, pressure_pa: f32) -> bool {
        if self.calibrated {
            return true;
        }
        self.pressure_sum += pressure_pa;
        self.sample_count += 1;
        if self.sample_count >= self.cal_samples {
            self.ground_pressure = self.pressure_sum / self.sample_count as f32;
            self.calibrated = true;
        }
        self.calibrated
    }

    /// Get the calibrated ground pressure (Pa).
    pub fn ground_pressure(&self) -> f32 {
        self.ground_pressure
    }

    /// Is calibration complete?
    pub fn is_calibrated(&self) -> bool {
        self.calibrated
    }

    /// Compute altitude relative to calibrated ground (m, positive up).
    /// Uses the barometric formula: alt = 44330 * (1 - (P/P0)^(1/5.255))
    pub fn altitude_relative(&self, pressure_pa: f32) -> f32 {
        if !self.calibrated {
            return 0.0;
        }
        44330.0 * (1.0 - libm::powf(pressure_pa / self.ground_pressure, 1.0 / 5.255))
    }

    /// Reset calibration state.
    pub fn reset(&mut self) {
        self.pressure_sum = 0.0;
        self.sample_count = 0;
        self.calibrated = false;
        self.ground_pressure = 101325.0;
    }
}

/// Sensor simulator that generates realistic sensor data from physics state.
pub struct SensorSim {
    pub noise: NoiseParams,
    pub rng: Rng,
    /// EKF origin (home position).
    pub origin: LatLonAlt,
    /// Earth magnetic field in NED (gauss). Typical mid-latitude.
    pub mag_field_ned: Vec3<NED>,
    /// Barometer ground calibration.
    pub baro_cal: BaroGroundCal,
}

impl SensorSim {
    pub fn new(origin: LatLonAlt) -> Self {
        Self {
            noise: NoiseParams::default(),
            rng: Rng::new(42),
            origin,
            // Typical mid-latitude magnetic field (~0.5 gauss total)
            // Source: WMM2020 at ~35°N
            mag_field_ned: Vec3::new(0.22, 0.005, 0.42),
            baro_cal: BaroGroundCal::new(),
        }
    }

    /// Generate an IMU sample from physics state.
    /// Accel measures specific force (non-gravitational) in body frame.
    /// Source: SIM_Aircraft.cpp add_noise()
    pub fn sample_imu(
        &mut self,
        state: &PhysicsState,
        params: &VehicleParams,
        motor_commands: &[f32],
        dt: f32,
    ) -> ImuSample {
        let body_to_ned: Rotation<Body, NED> = Rotation::from_quaternion(state.attitude);
        let ned_to_body: Rotation<NED, Body> = body_to_ned.inverse();

        // Compute total thrust in body frame
        let mut thrust_body = Vec3::<Body>::zero();
        for i in 0..params.motors.len() {
            let motor = &params.motors[i];
            let cmd = motor_commands.get(i).copied().unwrap_or(0.0).clamp(0.0, 1.0);
            let linearized = (1.0 - motor.expo) * cmd + motor.expo * cmd * cmd;
            let thrust = motor.max_thrust * linearized;
            thrust_body = thrust_body + Vec3::<Body>::new(0.0, 0.0, -thrust);
        }

        // Specific force: what the accelerometer actually measures.
        //
        // In flight: accel = (thrust/mass) - gravity_in_body
        // On ground: the ground reaction force cancels any downward thrust component,
        // so the accelerometer only reads -gravity_in_body (= [0,0,-g] when level).
        //
        // Transition: if thrust_z exceeds weight, ground reaction → 0, vehicle lifts off.
        let gravity_ned = Vec3::<NED>::new(0.0, 0.0, GRAVITY);
        let gravity_body = ned_to_body.rotate(gravity_ned);

        // Accelerometer measures specific force = non_gravitational_force / mass.
        // On ground: specific force = ground_reaction/mass = -gravity (in body frame)
        // In flight: specific force = thrust/mass (gravity NOT subtracted — EKF adds it)
        //
        // The EKF prediction adds gravity: v += R*accel*dt + [0,0,g]*dt
        // So the IMU should report just thrust/mass, not thrust/mass - gravity.
        let accel_body = if state.on_ground {
            // Ground reaction = mg upward → accel = g upward = -gravity in body frame
            Vec3::<Body>::new(0.0, 0.0, 0.0) - gravity_body
        } else {
            // In flight: just thrust + drag, no gravity
            let thrust_accel_body = thrust_body * (1.0 / params.mass);
            thrust_accel_body
        };

        // Noise proportional to average throttle
        let avg_throttle = motor_commands.iter()
            .take(params.motors.len())
            .sum::<f32>() / params.motors.len().max(1) as f32;
        let throttle_scale = avg_throttle.abs().max(0.1);

        let accel_noise = Vec3::<Body>::new(
            self.rng.normal() * self.noise.accel_noise * throttle_scale,
            self.rng.normal() * self.noise.accel_noise * throttle_scale,
            self.rng.normal() * self.noise.accel_noise * throttle_scale,
        );
        let gyro_noise = Vec3::<Body>::new(
            self.rng.normal() * self.noise.gyro_noise * throttle_scale,
            self.rng.normal() * self.noise.gyro_noise * throttle_scale,
            self.rng.normal() * self.noise.gyro_noise * throttle_scale,
        );

        ImuSample {
            timestamp: Instant::from_micros(state.time_us),
            imu_index: 0,
            accel: accel_body + accel_noise,
            gyro: state.gyro + gyro_noise,
            temperature: 25.0,
        }
    }

    /// Generate a GPS position fix.
    pub fn sample_gps(&mut self, state: &PhysicsState) -> GnssPosition {
        let ned = Vec3::<NED>::new(
            state.position[0] as f32,
            state.position[1] as f32,
            state.position[2] as f32,
        );
        let lla = self.origin.ned_to_lla(&ned);

        // Convert position noise (meters) to radians: divide by Earth radius
        // NOT 111320 (meters per degree) — LatLonAlt uses RADIANS
        let earth_radius = 6378137.0f32;
        let cos_lat = libm::cosf(self.origin.lat as f32);
        let pos_noise = LatLonAlt::new(
            (self.rng.normal() * self.noise.gps_pos_noise / earth_radius) as f64,
            (self.rng.normal() * self.noise.gps_pos_noise / (earth_radius * cos_lat)) as f64,
            (self.rng.normal() * self.noise.gps_pos_noise) as f64,
        );

        let vel_noise = Vec3::<NED>::new(
            self.rng.normal() * self.noise.gps_vel_noise,
            self.rng.normal() * self.noise.gps_vel_noise,
            self.rng.normal() * self.noise.gps_vel_noise,
        );

        GnssPosition {
            timestamp: Instant::from_micros(state.time_us),
            fix_type: GnssFixType::Fix3D,
            position: LatLonAlt::new(
                lla.lat + pos_noise.lat,
                lla.lon + pos_noise.lon,
                lla.alt + pos_noise.alt,
            ),
            velocity_ned: state.velocity + vel_noise,
            horizontal_accuracy: self.noise.gps_pos_noise,
            vertical_accuracy: self.noise.gps_pos_noise * 1.5,
            speed_accuracy: self.noise.gps_vel_noise,
            num_sats: 12,
        }
    }

    /// Generate a barometer reading.
    /// Feeds the ground calibrator on each sample. Once calibrated, altitude_m
    /// is relative to the ground reference pressure.
    pub fn sample_baro(&mut self, state: &PhysicsState) -> BaroPressure {
        let alt = -state.position[2] as f32;
        let noise = self.rng.normal() * self.noise.baro_noise;
        // Standard atmosphere: P = P0 * (1 - alt/44330)^5.255
        let pressure = 101325.0 * libm::powf(1.0 - (alt + noise) / 44330.0, 5.255);

        // Feed ground calibration
        self.baro_cal.calibrate(pressure);

        // Use calibrated relative altitude if available, else raw
        let altitude = if self.baro_cal.is_calibrated() {
            self.baro_cal.altitude_relative(pressure)
        } else {
            alt + noise
        };

        BaroPressure {
            timestamp: Instant::from_micros(state.time_us),
            baro_index: 0,
            pressure_pa: pressure,
            temperature: 25.0,
            altitude_m: altitude,
        }
    }

    /// Generate a magnetometer reading.
    pub fn sample_mag(&mut self, state: &PhysicsState) -> MagField {
        let body_to_ned: Rotation<Body, NED> = Rotation::from_quaternion(state.attitude);
        let ned_to_body: Rotation<NED, Body> = body_to_ned.inverse();
        let field_body = ned_to_body.rotate(self.mag_field_ned);

        let noise = Vec3::<Body>::new(
            self.rng.normal() * self.noise.mag_noise,
            self.rng.normal() * self.noise.mag_noise,
            self.rng.normal() * self.noise.mag_noise,
        );

        MagField {
            timestamp: Instant::from_micros(state.time_us),
            mag_index: 0,
            field: field_body + noise,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::PhysicsState;
    use meridian_vehicle::VehiclePhysics;

    #[test]
    fn test_imu_at_rest() {
        let vp = VehiclePhysics::default_quad();
        let params = VehicleParams::from_vehicle(&vp);
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
        let mut sensors = SensorSim::new(origin);
        sensors.noise = NoiseParams {
            gyro_noise: 0.0,
            accel_noise: 0.0,
            ..Default::default()
        };

        let state = PhysicsState::new();
        let cmds = [0.0f32; 12];
        let imu = sensors.sample_imu(&state, &params, &cmds, 1.0 / 1200.0);

        // At rest on ground with no thrust: accel should read [0, 0, -g]
        // (accelerometer measures specific force, on ground = -gravity)
        assert!((imu.accel.x).abs() < 0.1);
        assert!((imu.accel.y).abs() < 0.1);
        assert!((imu.accel.z - (-GRAVITY)).abs() < 0.1,
            "Accel Z at rest: {} (expected {})", imu.accel.z, -GRAVITY);

        // Gyro at rest should be zero
        assert!((imu.gyro.x).abs() < 0.01);
        assert!((imu.gyro.y).abs() < 0.01);
        assert!((imu.gyro.z).abs() < 0.01);
    }

    #[test]
    fn test_mag_at_level() {
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
        let mut sensors = SensorSim::new(origin);
        sensors.noise.mag_noise = 0.0;

        let state = PhysicsState::new(); // level
        let mag = sensors.sample_mag(&state);

        // At level attitude, body mag should equal NED mag
        assert!((mag.field.x - sensors.mag_field_ned.x).abs() < 0.01);
        assert!((mag.field.y - sensors.mag_field_ned.y).abs() < 0.01);
        assert!((mag.field.z - sensors.mag_field_ned.z).abs() < 0.01);
    }

    #[test]
    fn test_rng_deterministic() {
        let mut rng1 = Rng::new(42);
        let mut rng2 = Rng::new(42);
        for _ in 0..100 {
            assert_eq!(rng1.uniform().to_bits(), rng2.uniform().to_bits());
        }
    }

    // ── BaroGroundCal tests ──

    #[test]
    fn test_baro_cal_not_calibrated_initially() {
        let cal = BaroGroundCal::new();
        assert!(!cal.is_calibrated());
    }

    #[test]
    fn test_baro_cal_calibrates_after_5_samples() {
        let mut cal = BaroGroundCal::new();
        let ground_pressure = 101325.0;

        // Feed 4 samples — not yet calibrated
        for _ in 0..4 {
            assert!(!cal.calibrate(ground_pressure));
        }
        assert!(!cal.is_calibrated());

        // 5th sample — calibration complete
        assert!(cal.calibrate(ground_pressure));
        assert!(cal.is_calibrated());
        assert!((cal.ground_pressure() - ground_pressure).abs() < 0.1);
    }

    #[test]
    fn test_baro_cal_averages_pressure() {
        let mut cal = BaroGroundCal::new();
        // Feed 5 slightly different pressures
        let pressures = [101300.0, 101310.0, 101320.0, 101330.0, 101340.0];
        for &p in &pressures {
            cal.calibrate(p);
        }
        let expected = pressures.iter().sum::<f32>() / 5.0;
        assert!((cal.ground_pressure() - expected).abs() < 0.1);
    }

    #[test]
    fn test_baro_cal_relative_altitude_at_ground() {
        let mut cal = BaroGroundCal::new();
        let ground_pressure = 101325.0;
        for _ in 0..5 {
            cal.calibrate(ground_pressure);
        }
        // At ground pressure, relative altitude should be ~0
        let alt = cal.altitude_relative(ground_pressure);
        assert!(alt.abs() < 0.01, "Expected ~0 altitude at ground, got {}", alt);
    }

    #[test]
    fn test_baro_cal_relative_altitude_above_ground() {
        let mut cal = BaroGroundCal::new();
        let ground_pressure = 101325.0;
        for _ in 0..5 {
            cal.calibrate(ground_pressure);
        }
        // Pressure at 100m above: P = P0 * (1 - 100/44330)^5.255
        let p_100m = ground_pressure * libm::powf(1.0 - 100.0 / 44330.0, 5.255);
        let alt = cal.altitude_relative(p_100m);
        assert!((alt - 100.0).abs() < 1.0, "Expected ~100m altitude, got {}", alt);
    }

    #[test]
    fn test_baro_cal_altitude_zero_before_calibrated() {
        let cal = BaroGroundCal::new();
        let alt = cal.altitude_relative(101325.0);
        assert!(alt.abs() < 0.01, "Should return 0 before calibration");
    }

    #[test]
    fn test_baro_cal_reset() {
        let mut cal = BaroGroundCal::new();
        for _ in 0..5 {
            cal.calibrate(101325.0);
        }
        assert!(cal.is_calibrated());
        cal.reset();
        assert!(!cal.is_calibrated());
    }

    #[test]
    fn test_sensor_sim_baro_calibrates_after_samples() {
        let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
        let mut sensors = SensorSim::new(origin);
        sensors.noise.baro_noise = 0.0; // no noise for determinism
        let state = PhysicsState::new(); // on ground

        // First 5 samples calibrate
        for _ in 0..5 {
            sensors.sample_baro(&state);
        }
        assert!(sensors.baro_cal.is_calibrated());

        // Altitude at ground should be ~0 after calibration
        let baro = sensors.sample_baro(&state);
        assert!(baro.altitude_m.abs() < 0.5, "Ground alt should be ~0 after cal, got {}", baro.altitude_m);
    }
}

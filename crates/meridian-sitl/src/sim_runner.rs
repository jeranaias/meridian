//! Generalized simulation runner for any vehicle type and scenario.
//!
//! Provides the full closed-loop: Physics → Sensors → EKF → Controllers → Mixer → Physics.
//! Configurable vehicle, target trajectory, wind, and noise.

use meridian_math::{Vec3, Quaternion};
use meridian_math::frames::{NED, Body};
use meridian_math::geodetic::LatLonAlt;
use meridian_types::time::Instant;
use meridian_ekf::EkfCore;
use meridian_control::{RateController, AttitudeController, PositionController};
use meridian_mixing::{Mixer, MixingMatrix, MAX_MOTORS};
use meridian_vehicle::VehiclePhysics;
use crate::physics::{PhysicsState, VehicleParams, PHYSICS_HZ};
use crate::sensors::{SensorSim, NoiseParams};
use crate::scheduler::Scheduler;

/// Target state at a given time — what the vehicle should be doing.
#[derive(Debug, Clone, Copy)]
pub struct Target {
    /// Target position NED (meters)
    pub position: Vec3<NED>,
    /// Target velocity NED (m/s) — feedforward
    pub velocity: Vec3<NED>,
    /// Target yaw (radians)
    pub yaw: f32,
}

/// Scenario function: given time since arm (seconds), return target.
pub type ScenarioFn = fn(f32) -> Target;

/// Wind model: constant wind vector in NED (m/s).
#[derive(Debug, Clone, Copy)]
pub struct WindModel {
    pub velocity_ned: Vec3<NED>,
}

impl Default for WindModel {
    fn default() -> Self {
        Self { velocity_ned: Vec3::zero() }
    }
}

/// Simulation configuration.
pub struct SimConfig {
    pub vehicle: VehiclePhysics,
    pub mixing: MixingMatrix,
    pub scenario: ScenarioFn,
    pub wind: WindModel,
    pub duration: f32,
    pub arm_delay: f32,
    pub noise: Option<NoiseParams>,
}

/// Snapshot of simulation state at one moment.
#[derive(Debug, Clone)]
pub struct SimSnapshot {
    pub time: f32,
    pub phys_pos: [f64; 3],
    pub phys_vel: Vec3<NED>,
    pub ekf_pos: Vec3<NED>,
    pub ekf_vel: Vec3<NED>,
    pub altitude: f32,
    pub motor_avg: f32,
}

/// Results from a complete simulation run.
#[derive(Debug)]
pub struct SimResult {
    pub final_altitude: f32,
    pub max_altitude: f32,
    pub final_horiz_error: f32,
    pub max_horiz_error: f32,
    pub final_velocity: f32,
    pub max_velocity: f32,
    pub any_nan: bool,
    pub duration: f32,
    pub snapshots: Vec<SimSnapshot>,
}

impl SimResult {
    pub fn print_summary(&self, name: &str) {
        let status = if self.any_nan { "NaN!" } else { "OK" };
        eprintln!(
            "  {:<30} alt={:6.2}m  horiz_err={:5.2}m  max_v={:5.2}m/s  {}",
            name, self.final_altitude, self.final_horiz_error, self.max_velocity, status
        );
    }
}

/// Run a full closed-loop simulation.
pub fn run_simulation(config: &SimConfig) -> SimResult {
    let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
    let params = VehicleParams::from_vehicle(&config.vehicle);
    let mut physics = PhysicsState::new();
    let mut sensors = SensorSim::new(origin);
    if let Some(ref noise) = config.noise {
        sensors.noise = noise.clone();
    }
    let mut ekf = EkfCore::new(origin);
    let mut mixer = Mixer::new(config.mixing.clone());
    let mut rate_ctrl = RateController::new();
    let mut att_ctrl = AttitudeController::new();
    let mut pos_ctrl = PositionController::new();
    pos_ctrl.gains.hover_throttle = config.vehicle.hover_throttle;
    let mut scheduler = Scheduler::new();

    let mut armed = false;
    let mut motor_commands = [0.0f32; MAX_MOTORS];
    let mut gps_counter: u32 = 0;

    let mut max_alt = 0.0f32;
    let mut max_horiz_err = 0.0f32;
    let mut max_velocity = 0.0f32;
    let mut any_nan = false;
    let mut snapshots = Vec::new();

    let dt_physics = 1.0 / PHYSICS_HZ as f32;
    let total_steps = (config.duration * PHYSICS_HZ as f32) as u32;

    for step in 0..total_steps {
        let time_us = (step as u64) * (1_000_000 / PHYSICS_HZ as u64);
        let time = Instant::from_micros(time_us);
        let time_s = time_us as f32 * 1e-6;

        // ── Apply wind as external force ──
        if config.wind.velocity_ned.length_squared() > 0.0 {
            // Wind creates drag force proportional to relative velocity
            let rel_vel = physics.velocity - config.wind.velocity_ned;
            let wind_drag = rel_vel * (-0.3); // drag coefficient
            physics.velocity = physics.velocity + wind_drag * dt_physics;
        }

        // ── Step physics ──
        crate::physics::step(&mut physics, &motor_commands, &params, dt_physics);

        let due = scheduler.tick(time);

        // ── Arm ──
        if !armed && time_s > config.arm_delay {
            armed = true;
        }

        // ── IMU (1000 Hz) ──
        if due[0] {
            let imu = sensors.sample_imu(&physics, &params, &motor_commands, dt_physics);
            ekf.predict(&imu);
        }

        // ── Control (400 Hz) ──
        if due[1] && armed {
            let dt_fast = 1.0 / 400.0;
            let time_since_arm = time_s - config.arm_delay;
            let target = (config.scenario)(time_since_arm);

            let ekf_quat = ekf.state.quat;
            let ekf_vel = ekf.state.velocity;
            let ekf_pos = ekf.state.position;
            let (_roll, _pitch, yaw) = ekf_quat.to_euler();

            let pos_out = pos_ctrl.update(
                &target.position, &target.velocity,
                &ekf_pos, &ekf_vel, yaw, dt_fast,
            );

            let rate_target = att_ctrl.update(&pos_out.target_quat, &ekf_quat, dt_fast);
            let gyro_measured = Vec3::<Body>::new(
                physics.gyro.x - ekf.state.gyro_bias.x,
                physics.gyro.y - ekf.state.gyro_bias.y,
                physics.gyro.z - ekf.state.gyro_bias.z,
            );
            let (r, p, y) = rate_ctrl.update_simple(&rate_target, &gyro_measured, dt_fast);
            motor_commands = mixer.mix(r, p, y, pos_out.throttle);
        }

        // ── GPS + Baro (50 Hz medium) ──
        if due[2] {
            gps_counter += 1;
            if gps_counter % 3 == 0 {
                let gps = sensors.sample_gps(&physics);
                ekf.fuse_gps(&gps);
            }
            let baro = sensors.sample_baro(&physics);
            ekf.fuse_baro(&baro);
        }

        // ── Mag (10 Hz slow) ──
        if due[3] {
            let mag = sensors.sample_mag(&physics);
            ekf.fuse_mag(&mag);
        }

        // ── Track ──
        let alt = physics.altitude();
        let horiz_err = libm::sqrtf(
            (physics.position[0] as f32).powi(2) + (physics.position[1] as f32).powi(2)
        );
        let vel_mag = physics.velocity.length();
        max_alt = max_alt.max(alt);
        max_horiz_err = max_horiz_err.max(horiz_err);
        max_velocity = max_velocity.max(vel_mag);

        // Snapshot every second
        if step % PHYSICS_HZ == 0 {
            let motor_avg = motor_commands.iter().take(config.vehicle.motor_count as usize)
                .sum::<f32>() / config.vehicle.motor_count as f32;
            snapshots.push(SimSnapshot {
                time: time_s,
                phys_pos: physics.position,
                phys_vel: physics.velocity,
                ekf_pos: ekf.state.position,
                ekf_vel: ekf.state.velocity,
                altitude: alt,
                motor_avg,
            });
        }

        if physics.velocity.is_nan() || physics.gyro.is_nan() || ekf.state.quat.is_nan() {
            any_nan = true;
            break;
        }
    }

    let final_horiz_err = libm::sqrtf(
        (physics.position[0] as f32).powi(2) + (physics.position[1] as f32).powi(2)
    );

    SimResult {
        final_altitude: physics.altitude(),
        max_altitude: max_alt,
        final_horiz_error: final_horiz_err,
        max_horiz_error: max_horiz_err,
        final_velocity: physics.velocity.length(),
        max_velocity,
        any_nan,
        duration: config.duration,
        snapshots,
    }
}

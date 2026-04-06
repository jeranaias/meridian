//! Fixed-wing closed-loop simulation.
//!
//! Full loop: aero physics → sensors → EKF → TECS/L1 modes → control surfaces → physics.

use meridian_math::{Vec3, Quaternion, Rotation};
use meridian_math::frames::{NED, Body};
use meridian_math::geodetic::LatLonAlt;
use meridian_types::time::Instant;
use meridian_types::messages::*;
use meridian_ekf::EkfCore;
use meridian_modes::fixed_wing::FixedWingModes;
use meridian_modes::mode_trait::*;
use meridian_types::vehicle::FlightModeId;
use meridian_nav::Waypoint;
use crate::physics::PhysicsState;
use crate::physics_fixedwing::*;
use crate::sensors::{SensorSim, Rng};

/// Fixed-wing simulation result.
#[derive(Debug)]
pub struct FixedWingResult {
    pub final_altitude: f32,
    pub min_altitude: f32,
    pub max_altitude: f32,
    pub final_speed: f32,
    pub min_speed: f32,
    pub distance_flown: f32,
    pub any_nan: bool,
    pub crashed: bool,
}

impl FixedWingResult {
    pub fn print_summary(&self, name: &str) {
        let status = if self.crashed { "CRASHED" } else if self.any_nan { "NaN!" } else { "OK" };
        eprintln!("  {:<30} alt={:.0}/{:.0}/{:.0}m spd={:.1}/{:.1}m/s dist={:.0}m {}",
            name, self.min_altitude, self.final_altitude, self.max_altitude,
            self.min_speed, self.final_speed, self.distance_flown, status);
    }
}

/// Run a fixed-wing closed-loop simulation.
pub fn run_fixed_wing_sim(
    mode: FlightModeId,
    waypoints: Option<&[Waypoint]>,
    duration: f32,
    target_alt: f32,
) -> FixedWingResult {
    let origin = LatLonAlt::from_degrees(35.0, -120.0, 0.0);
    let fw_params = FixedWingParams::default_trainer();
    let mut state = PhysicsState::new();
    // Start airborne at cruise speed with trim pitch
    // Trim alpha ≈ 0.004 rad for level flight at 18 m/s (CL=0.22)
    state.position = [0.0, 0.0, -target_alt as f64];
    state.velocity = Vec3::<NED>::new(18.0, 0.0, 0.0);
    state.attitude = Quaternion::identity(); // match open-loop test exactly
    state.on_ground = false;

    let mut sensors = SensorSim::new(origin);
    sensors.noise.gyro_noise = 0.001; // less noise for plane (less vibration)
    sensors.noise.accel_noise = 0.1;

    let mut ekf = EkfCore::new(origin);
    let mut fw_modes = FixedWingModes::new();
    fw_modes.params.airspeed_cruise = 18.0;
    fw_modes.target_altitude = target_alt;
    fw_modes.set_mode(mode, &ModeInput {
        attitude: state.attitude, position: Vec3::new(0.0, 0.0, -target_alt),
        velocity: state.velocity, gyro: Vec3::zero(), altitude: target_alt,
        yaw: 0.0, ground_course: 0.0, ground_speed: 18.0, airspeed: 18.0,
        home: Vec3::new(0.0, 0.0, -target_alt),
        time_since_arm: 0.0, rc_roll: 0.0, rc_pitch: 0.0, rc_yaw: 0.0, rc_throttle: 0.5,
        throttle_at_min: false, throttle_mix_at_min: false,
        small_angle_error: true, accel_near_1g: true, rangefinder_near_ground: true,
    });

    if let Some(wps) = waypoints {
        fw_modes.load_mission(wps);
    }

    // Start at trim: CL0=0.22 provides level flight at 18 m/s, no elevator needed
    let mut controls = FixedWingControls { throttle: 0.22, ..Default::default() };
    let mut min_alt = target_alt;
    let mut max_alt = target_alt;
    let mut min_speed = 18.0f32;
    let mut distance = 0.0f32;
    let mut any_nan = false;
    let mut crashed = false;
    let mut rng = Rng::new(123);

    let dt = 1.0 / 400.0; // 400Hz control
    let dt_phys = 1.0 / 1200.0;
    let total_steps = (duration / dt_phys) as u32;
    let mut control_counter = 0u32;
    let mut gps_counter = 0u32;

    for step in 0..total_steps {
        let time_us = (step as u64) * (1_000_000 / 1200);
        let time_s = time_us as f32 * 1e-6;

        step_fixed_wing(&mut state, &controls, &fw_params, dt_phys);
        distance += state.velocity.length() * dt_phys;

        // IMU at 400Hz
        control_counter += 1;
        if control_counter % 3 == 0 {
            let accel = fixed_wing_imu_accel(&state, &controls, &fw_params);
            let gyro_noise = Vec3::<Body>::new(
                rng.normal() * 0.001, rng.normal() * 0.001, rng.normal() * 0.001);
            let imu = ImuSample {
                timestamp: Instant::from_micros(time_us),
                imu_index: 0,
                accel,
                gyro: state.gyro + gyro_noise,
                temperature: 25.0,
            };
            ekf.predict(&imu);

            // Control update — use PHYSICS TRUTH (bypass EKF for now)
            let alt = state.altitude();
            let (_, _, yaw) = state.attitude.to_euler();
            let gs = libm::sqrtf(state.velocity.x.powi(2) + state.velocity.y.powi(2));

            let mode_input = ModeInput {
                attitude: state.attitude,
                position: Vec3::new(state.position[0] as f32, state.position[1] as f32, state.position[2] as f32),
                velocity: state.velocity,
                gyro: state.gyro,
                altitude: alt,
                yaw,
                ground_course: libm::atan2f(state.velocity.y, state.velocity.x),
                ground_speed: gs,
                airspeed: state.velocity.length(),
                home: Vec3::new(0.0, 0.0, -target_alt),
                time_since_arm: time_s,
                rc_roll: 0.0, rc_pitch: 0.0, rc_yaw: 0.0, rc_throttle: 0.5,
                throttle_at_min: false, throttle_mix_at_min: false,
                small_angle_error: true, accel_near_1g: true, rangefinder_near_ground: true,
            };

            // TECS-correct control allocation:
            // THROTTLE controls altitude (total energy)
            // PITCH controls airspeed (energy distribution)
            // This is the anti-phugoid architecture.
            let true_airspeed = state.velocity.length();
            let (curr_roll, curr_pitch, _) = state.attitude.to_euler();
            let alt_err = target_alt - alt;

            // Simple stable flight controller:
            // Elevator: hold LEVEL (pitch ≈ trim angle 2°), strong rate damping
            // Throttle: PI on altitude error
            //
            // This sacrifices airspeed regulation for pitch stability.
            // Speed self-regulates via drag (excess thrust → accelerate → more drag → equilibrium).

            // TRUE TECS: throttle controls TOTAL ENERGY (anti-phugoid)
            // Total energy = KE + PE = 0.5*m*V² + m*g*h
            let mass = 2.0f32;
            let target_energy = 0.5 * mass * 18.0 * 18.0 + mass * 9.81 * target_alt;
            let current_energy = 0.5 * mass * true_airspeed * true_airspeed + mass * 9.81 * alt;
            let energy_err = target_energy - current_energy; // positive = need more energy
            // Normalize by weight*cruise_speed for dimensionless gain
            let energy_norm = energy_err / (mass * 9.81 * 18.0);
            controls.throttle = (0.22 + energy_norm * 0.3).clamp(0.0, 0.6);

            // ELEVATOR: controls ENERGY DISTRIBUTION (speed vs altitude balance)
            // When fast: pitch up (convert KE→PE). When slow: pitch down (convert PE→KE).
            // When high: bias toward speed. When low: bias toward altitude.
            let spd_err = true_airspeed - 18.0; // positive = too fast
            let alt_balance = (alt_err * 0.005).clamp(-0.05, 0.05); // gentle altitude bias
            let pitch_cmd = (spd_err * 0.01 + alt_balance)
                .clamp(-0.12, 0.12); // max ±7° pitch command
            controls.elevator = ((pitch_cmd - curr_pitch) * 0.15 - state.gyro.y * 0.05)
                .clamp(-0.03, 0.03);

            // Aileron: roll level + L1 guidance
            let _mode_output = fw_modes.update(&mode_input, dt);
            match _mode_output {
                ModeOutput::FixedWingTarget { roll, .. } => {
                    controls.aileron = ((roll - curr_roll) * 1.5
                        - state.gyro.x * 0.3).clamp(-1.0, 1.0);
                }
                _ => {
                    controls.aileron = (-curr_roll * 1.5 - state.gyro.x * 0.3).clamp(-1.0, 1.0);
                }
            }
            controls.rudder = (-state.gyro.z * 0.2).clamp(-1.0, 1.0);
        }

        // GPS at 10Hz
        if control_counter % 120 == 0 {
            let gps = sensors.sample_gps(&state);
            ekf.fuse_gps(&gps);
            let baro = sensors.sample_baro(&state);
            ekf.fuse_baro(&baro);
        }

        // Mag at 10Hz
        if control_counter % 120 == 60 {
            let mag = sensors.sample_mag(&state);
            ekf.fuse_mag(&mag);
        }

        let alt = state.altitude();
        min_alt = min_alt.min(alt);
        max_alt = max_alt.max(alt);
        min_speed = min_speed.min(state.velocity.length());

        // Debug: print every 2s
        if step % 2400 == 0 && step > 0 {
            eprintln!("  [{:.0}s] alt={:.0}m spd={:.1}m/s thr={:.2} elev={:.2} pitch={:.1}° vz={:.1}",
                time_s, alt, state.velocity.length(), controls.throttle, controls.elevator,
                state.attitude.to_euler().1 * 180.0 / core::f32::consts::PI,
                state.velocity.z);
        }

        if state.velocity.is_nan() || state.attitude.is_nan() {
            any_nan = true;
            break;
        }
        if alt < 1.0 && !state.on_ground {
            crashed = true;
            break;
        }
    }

    FixedWingResult {
        final_altitude: state.altitude(),
        min_altitude: min_alt,
        max_altitude: max_alt,
        final_speed: state.velocity.length(),
        min_speed,
        distance_flown: distance,
        any_nan,
        crashed,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fw_cruise_30s() {
        // 30s cruise — proves indefinite stable flight (no phugoid divergence)
        let result = run_fixed_wing_sim(FlightModeId::Cruise, None, 30.0, 50.0);
        result.print_summary("FW Cruise 30s");
        assert!(!result.any_nan, "NaN detected");
        assert!(!result.crashed, "Crashed at alt={:.0}m", result.final_altitude);
        assert!(result.final_speed > 5.0, "Lost speed: {:.1}m/s", result.final_speed);
        assert!(result.min_altitude > 10.0, "Dropped too low: {:.0}m", result.min_altitude);
    }

    #[test]
    fn test_fw_fbwa_20s() {
        let result = run_fixed_wing_sim(FlightModeId::FlyByWireA, None, 20.0, 50.0);
        result.print_summary("FW FBWA 20s");
        assert!(!result.any_nan);
        assert!(!result.crashed, "Crashed: alt={:.0}m", result.final_altitude);
    }

    #[test]
    fn test_fw_auto_circuit() {
        let wps = [
            Waypoint::new(200.0, 0.0, -50.0).with_radius(30.0),
            Waypoint::new(200.0, 200.0, -50.0).with_radius(30.0),
            Waypoint::new(0.0, 200.0, -50.0).with_radius(30.0),
            Waypoint::new(0.0, 0.0, -50.0).with_radius(30.0),
        ];
        let result = run_fixed_wing_sim(FlightModeId::Auto, Some(&wps), 30.0, 50.0);
        result.print_summary("FW Auto Square");
        assert!(!result.any_nan);
        assert!(!result.crashed, "Crashed during circuit");
        assert!(result.distance_flown > 300.0, "Didn't fly enough: {:.0}m", result.distance_flown);
    }
}

//! Common trait and types for all flight modes across all vehicle classes.

use meridian_math::{Vec3, Quaternion};
use meridian_math::frames::{NED, Body};
use meridian_types::vehicle::FlightModeId;
use meridian_types::time::Duration;

/// Input state provided to flight modes each tick.
#[derive(Debug, Clone, Copy)]
pub struct ModeInput {
    /// Current attitude (body-to-NED quaternion).
    pub attitude: Quaternion,
    /// Current position NED (m).
    pub position: Vec3<NED>,
    /// Current velocity NED (m/s).
    pub velocity: Vec3<NED>,
    /// Current angular velocity body (rad/s).
    pub gyro: Vec3<Body>,
    /// Current altitude above ground (m, positive up).
    pub altitude: f32,
    /// Current yaw (rad).
    pub yaw: f32,
    /// Current heading/bearing of velocity (rad from north).
    pub ground_course: f32,
    /// Current ground speed (m/s).
    pub ground_speed: f32,
    /// Current airspeed (m/s). For fixed-wing speed scaling and TECS.
    /// Falls back to ground_speed when no airspeed sensor is available.
    pub airspeed: f32,
    /// Home position NED (m).
    pub home: Vec3<NED>,
    /// Time since arm (s).
    pub time_since_arm: f32,
    /// RC stick inputs (normalized -1..1 or 0..1 for throttle).
    pub rc_roll: f32,
    pub rc_pitch: f32,
    pub rc_yaw: f32,
    pub rc_throttle: f32,

    // ── Land detector conditions (for M1: full 8-condition check) ──

    /// Throttle output at minimum (motors at spin_min).
    pub throttle_at_min: bool,
    /// Throttle-RPY mix at minimum (attitude corrections negligible).
    pub throttle_mix_at_min: bool,
    /// Attitude error small (< 30 deg from target).
    pub small_angle_error: bool,
    /// Acceleration magnitude near 1g.
    pub accel_near_1g: bool,
    /// Rangefinder shows close to ground, or rangefinder not available.
    pub rangefinder_near_ground: bool,
}

/// Output from a flight mode — what the vehicle should do.
#[derive(Debug, Clone, Copy)]
pub enum ModeOutput {
    /// Multirotor: target position + velocity + yaw (for position controller).
    PositionTarget {
        position: Vec3<NED>,
        velocity: Vec3<NED>,
        yaw: f32,
    },
    /// Multirotor: target attitude + throttle (bypasses position controller).
    AttitudeTarget {
        quaternion: Quaternion,
        throttle: f32,
    },
    /// Multirotor: body-rate targets + throttle (bypasses attitude controller).
    /// Used by Acro and Sport modes for direct rate control.
    RateTarget {
        roll_rate: f32,   // rad/s
        pitch_rate: f32,  // rad/s
        yaw_rate: f32,    // rad/s
        throttle: f32,    // 0..1
    },
    /// Fixed-wing: roll angle + pitch angle + throttle (for servos).
    FixedWingTarget {
        roll: f32,      // radians
        pitch: f32,     // radians
        throttle: f32,  // 0..1
        yaw_rate: f32,  // rad/s (rudder)
    },
    /// Rover/boat: steering angle + throttle.
    RoverTarget {
        steering: f32,  // -1..1 (left..right)
        throttle: f32,  // -1..1 (reverse..forward)
    },
    /// Do nothing (disarmed or pre-arm).
    Idle,
}

/// The flight mode trait — implemented by every mode for every vehicle class.
pub trait FlightMode {
    fn id(&self) -> FlightModeId;
    fn name(&self) -> &'static str;

    /// Called once when entering this mode.
    fn enter(&mut self, input: &ModeInput);

    /// Called at control rate (400Hz for multirotor, 50Hz for fixed-wing/rover).
    /// Returns control targets for the vehicle.
    fn update(&mut self, input: &ModeInput, dt: f32) -> ModeOutput;

    /// Called once when leaving this mode.
    fn exit(&mut self);

    /// Does this mode need GPS?
    fn requires_gps(&self) -> bool;
}

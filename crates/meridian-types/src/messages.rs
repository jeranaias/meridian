use crate::time::Instant;
use crate::vehicle::{FlightModeId, FailsafeReason, FailsafeAction, VehicleLifecycle};
use meridian_math::{Vec3, Quaternion, Body, NED};
use meridian_math::geodetic::LatLonAlt;

/// Trait for all messages on the bus.
pub trait Message: Sized + Send + 'static {
    /// Unique topic identifier.
    const TOPIC: &'static str;
    /// When this message was produced.
    fn timestamp(&self) -> Instant;
}

// ─── Sensor Messages ───

/// Raw IMU sample at 1000 Hz from a single IMU.
#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    pub timestamp: Instant,
    pub imu_index: u8,
    pub accel: Vec3<Body>,    // m/s² in body frame
    pub gyro: Vec3<Body>,     // rad/s in body frame
    pub temperature: f32,     // °C
}

impl Message for ImuSample {
    const TOPIC: &'static str = "imu/raw";
    fn timestamp(&self) -> Instant { self.timestamp }
}

/// GNSS position fix.
#[derive(Debug, Clone, Copy)]
pub struct GnssPosition {
    pub timestamp: Instant,
    pub fix_type: GnssFixType,
    pub position: LatLonAlt,
    pub velocity_ned: Vec3<NED>, // m/s
    pub horizontal_accuracy: f32, // m (1-sigma)
    pub vertical_accuracy: f32,   // m (1-sigma)
    pub speed_accuracy: f32,      // m/s (1-sigma)
    pub num_sats: u8,
}

impl Message for GnssPosition {
    const TOPIC: &'static str = "gnss/position";
    fn timestamp(&self) -> Instant { self.timestamp }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GnssFixType {
    NoFix,
    Fix2D,
    Fix3D,
    DGps,
    RtkFloat,
    RtkFixed,
}

/// 3-axis magnetometer reading.
#[derive(Debug, Clone, Copy)]
pub struct MagField {
    pub timestamp: Instant,
    pub mag_index: u8,
    pub field: Vec3<Body>,  // gauss in body frame
}

impl Message for MagField {
    const TOPIC: &'static str = "mag/field";
    fn timestamp(&self) -> Instant { self.timestamp }
}

/// Barometric pressure reading.
#[derive(Debug, Clone, Copy)]
pub struct BaroPressure {
    pub timestamp: Instant,
    pub baro_index: u8,
    pub pressure_pa: f32,
    pub temperature: f32,
    pub altitude_m: f32,  // derived from pressure via standard atmosphere
}

impl Message for BaroPressure {
    const TOPIC: &'static str = "baro/pressure";
    fn timestamp(&self) -> Instant { self.timestamp }
}

/// Rangefinder (lidar/sonar) reading.
#[derive(Debug, Clone, Copy)]
pub struct RangefinderReading {
    pub timestamp: Instant,
    pub distance_m: f32,
    pub signal_quality: f32, // 0..1
    pub valid: bool,
}

impl Message for RangefinderReading {
    const TOPIC: &'static str = "rangefinder/distance";
    fn timestamp(&self) -> Instant { self.timestamp }
}

// ─── State Estimation Messages ───

/// EKF output state at 400 Hz.
#[derive(Debug, Clone, Copy)]
pub struct EkfState {
    pub timestamp: Instant,
    pub attitude: Quaternion,
    pub velocity_ned: Vec3<NED>,
    pub position_ned: Vec3<NED>,
    pub gyro_bias: Vec3<Body>,
    pub accel_bias: Vec3<Body>,
    pub origin: LatLonAlt,
    pub healthy: bool,
}

impl Message for EkfState {
    const TOPIC: &'static str = "ekf/state";
    fn timestamp(&self) -> Instant { self.timestamp }
}

// ─── Control Messages ───

/// RC input channels.
#[derive(Debug, Clone, Copy)]
pub struct RcChannels {
    pub timestamp: Instant,
    pub channels: [u16; 16], // PWM values (1000-2000) or 0 if unused
    pub channel_count: u8,
    pub rssi: u8,            // 0-255
    pub failsafe: bool,
}

impl Message for RcChannels {
    const TOPIC: &'static str = "rc/input";
    fn timestamp(&self) -> Instant { self.timestamp }
}

/// Attitude target from mode/nav to attitude controller.
#[derive(Debug, Clone, Copy)]
pub struct AttitudeTarget {
    pub timestamp: Instant,
    pub quaternion: Quaternion,
    pub thrust: f32,       // 0..1 total thrust command
    pub use_yaw_rate: bool,
    pub yaw_rate: f32,     // rad/s, if use_yaw_rate
}

impl Message for AttitudeTarget {
    const TOPIC: &'static str = "control/attitude_target";
    fn timestamp(&self) -> Instant { self.timestamp }
}

/// Rate target from attitude controller to rate controller.
#[derive(Debug, Clone, Copy)]
pub struct RateTarget {
    pub timestamp: Instant,
    pub roll_rate: f32,   // rad/s
    pub pitch_rate: f32,
    pub yaw_rate: f32,
}

impl Message for RateTarget {
    const TOPIC: &'static str = "control/rate_target";
    fn timestamp(&self) -> Instant { self.timestamp }
}

/// Axis commands from rate controller (before mixing).
#[derive(Debug, Clone, Copy)]
pub struct AxisCommands {
    pub roll: f32,    // -1..1
    pub pitch: f32,   // -1..1
    pub yaw: f32,     // -1..1
    pub throttle: f32, // 0..1
}

/// Motor output after mixing.
#[derive(Debug, Clone, Copy)]
pub struct MotorOutput {
    pub timestamp: Instant,
    pub motors: [f32; 12],  // 0..1 for each motor, up to 12
    pub motor_count: u8,
}

impl Message for MotorOutput {
    const TOPIC: &'static str = "control/motor_output";
    fn timestamp(&self) -> Instant { self.timestamp }
}

// ─── Vehicle State Messages ───

/// High-level vehicle state published at 50 Hz.
#[derive(Debug, Clone, Copy)]
pub struct VehicleState {
    pub timestamp: Instant,
    pub lifecycle: VehicleLifecycle,
    pub armed: bool,
    pub mode: FlightModeId,
    pub battery_voltage: f32,
    pub battery_remaining_pct: f32,
    pub gps_fix: GnssFixType,
    pub ekf_healthy: bool,
}

impl Message for VehicleState {
    const TOPIC: &'static str = "vehicle/state";
    fn timestamp(&self) -> Instant { self.timestamp }
}

/// Failsafe event.
#[derive(Debug, Clone, Copy)]
pub struct FailsafeEvent {
    pub timestamp: Instant,
    pub reason: FailsafeReason,
    pub action: FailsafeAction,
}

impl Message for FailsafeEvent {
    const TOPIC: &'static str = "failsafe/event";
    fn timestamp(&self) -> Instant { self.timestamp }
}

/// Mission engine status.
#[derive(Debug, Clone, Copy)]
pub struct MissionStatus {
    pub timestamp: Instant,
    pub active: bool,
    pub current_item: u16,
    pub total_items: u16,
}

impl Message for MissionStatus {
    const TOPIC: &'static str = "mission/status";
    fn timestamp(&self) -> Instant { self.timestamp }
}

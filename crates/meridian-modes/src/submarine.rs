//! Submarine (ArduSub) flight modes.
//!
//! Source: ArduSub/ + docs/audit_wave2_vehicles.md
//!
//! Submarines use 6DOF thruster mixing (forward, lateral, vertical, roll, pitch, yaw).
//! Depth control uses pressure sensor → depth hold PID.
//! Buoyancy management via throttle trim.

use meridian_math::Vec3;
use meridian_math::frames::{NED, Body};
use meridian_types::vehicle::FlightModeId;
use crate::mode_trait::*;

/// Submarine control output.
/// 6 axes: forward, lateral, throttle (vertical), roll, pitch, yaw.
/// Each in [-1, 1] range.
#[derive(Debug, Clone, Copy)]
pub struct SubOutput {
    pub forward: f32,
    pub lateral: f32,
    pub throttle: f32,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
}

/// Submarine parameters.
#[derive(Debug, Clone, Copy)]
pub struct SubParams {
    /// Depth hold P gain.
    pub depth_p: f32,
    /// Maximum depth (m, positive down).
    pub max_depth: f32,
    /// Buoyancy compensation (fraction of throttle needed to hover, -1..1).
    pub buoyancy_comp: f32,
}

impl Default for SubParams {
    fn default() -> Self {
        Self {
            depth_p: 0.5,
            max_depth: 100.0,
            buoyancy_comp: 0.0,
        }
    }
}

/// Submarine mode state.
pub struct SubModes {
    active: FlightModeId,
    pub params: SubParams,
    depth_target: f32,
}

impl SubModes {
    pub fn new() -> Self {
        Self {
            active: FlightModeId::Manual,
            params: SubParams::default(),
            depth_target: 0.0,
        }
    }

    pub fn set_mode(&mut self, mode: FlightModeId, input: &ModeInput) {
        self.active = mode;
        self.enter(input);
    }
}

impl FlightMode for SubModes {
    fn id(&self) -> FlightModeId { self.active }
    fn name(&self) -> &'static str { self.active.name() }

    fn enter(&mut self, input: &ModeInput) {
        match self.active {
            FlightModeId::AltHold => {
                // AltHold = depth hold for subs
                self.depth_target = input.altitude; // depth from pressure
            }
            _ => {}
        }
    }

    fn update(&mut self, input: &ModeInput, _dt: f32) -> ModeOutput {
        let sub_out = match self.active {
            FlightModeId::Manual => self.update_manual(input),
            FlightModeId::Stabilize => self.update_stabilize(input),
            FlightModeId::AltHold => self.update_depth_hold(input),
            // GAP 30-31: sub modes
            FlightModeId::SubAcro => self.update_sub_acro(input),
            FlightModeId::SubAuto => self.update_stabilize(input),    // stub: stabilize
            FlightModeId::SubGuided => self.update_stabilize(input),  // stub: stabilize
            FlightModeId::SubPosHold => self.update_depth_hold(input), // stub: depth hold
            FlightModeId::SurfTrak => self.update_depth_hold(input),   // stub: depth hold
            _ => self.update_manual(input),
        };

        // Convert SubOutput to a generic ModeOutput
        // For submarine, we use RateTarget to carry the 6DOF commands
        // (the sub mixer interprets these differently than a copter)
        ModeOutput::RateTarget {
            roll_rate: sub_out.roll,
            pitch_rate: sub_out.pitch,
            yaw_rate: sub_out.yaw,
            throttle: sub_out.throttle,
        }
    }

    fn exit(&mut self) {}

    fn requires_gps(&self) -> bool { false } // subs mostly operate without GPS
}

impl SubModes {
    /// Manual: all 6 axes from RC sticks.
    fn update_manual(&self, input: &ModeInput) -> SubOutput {
        SubOutput {
            forward: input.rc_pitch,     // pitch stick = forward/back
            lateral: input.rc_roll,      // roll stick = lateral
            throttle: input.rc_throttle * 2.0 - 1.0, // center = neutral buoyancy
            roll: 0.0,
            pitch: 0.0,
            yaw: input.rc_yaw,
        }
    }

    /// Stabilize: auto-level roll/pitch, manual yaw + throttle + translation.
    fn update_stabilize(&self, input: &ModeInput) -> SubOutput {
        let (roll, pitch, _) = input.attitude.to_euler();
        SubOutput {
            forward: input.rc_pitch,
            lateral: input.rc_roll,
            throttle: input.rc_throttle * 2.0 - 1.0 + self.params.buoyancy_comp,
            roll: -roll * 2.0,   // auto-level roll
            pitch: -pitch * 2.0, // auto-level pitch
            yaw: input.rc_yaw,
        }
    }

    /// Depth hold: stabilize + hold depth via pressure sensor.
    fn update_depth_hold(&mut self, input: &ModeInput) -> SubOutput {
        let (roll, pitch, _) = input.attitude.to_euler();

        // Throttle stick adjusts depth target
        let stick = input.rc_throttle * 2.0 - 1.0; // -1..1
        let deadband = 0.1;
        if libm::fabsf(stick) > deadband {
            // Move depth target at rate proportional to stick displacement
            self.depth_target += stick * 0.05; // 5 cm/tick at full stick
            self.depth_target = self.depth_target.clamp(0.0, self.params.max_depth);
        }

        // Depth PID
        let depth_error = self.depth_target - input.altitude;
        let throttle_cmd = depth_error * self.params.depth_p + self.params.buoyancy_comp;

        SubOutput {
            forward: input.rc_pitch,
            lateral: input.rc_roll,
            throttle: throttle_cmd.clamp(-1.0, 1.0),
            roll: -roll * 2.0,
            pitch: -pitch * 2.0,
            yaw: input.rc_yaw,
        }
    }

    pub fn depth_target(&self) -> f32 { self.depth_target }

    /// GAP 31: Sub ACRO mode — rate-based control on all 6 axes.
    fn update_sub_acro(&self, input: &ModeInput) -> SubOutput {
        SubOutput {
            forward: input.rc_pitch,
            lateral: input.rc_roll,
            throttle: input.rc_throttle * 2.0 - 1.0,
            roll: input.rc_roll * 2.0,     // direct rate command
            pitch: input.rc_pitch * 2.0,   // direct rate command
            yaw: input.rc_yaw,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use meridian_math::Quaternion;

    fn default_input() -> ModeInput {
        ModeInput {
            attitude: Quaternion::identity(),
            position: Vec3::zero(),
            velocity: Vec3::zero(),
            gyro: Vec3::<Body>::zero(),
            altitude: 5.0, // 5m depth
            yaw: 0.0,
            ground_course: 0.0,
            ground_speed: 0.0,
            airspeed: 0.0,
            home: Vec3::zero(),
            time_since_arm: 10.0,
            rc_roll: 0.0,
            rc_pitch: 0.0,
            rc_yaw: 0.0,
            rc_throttle: 0.5, // neutral
            throttle_at_min: false,
            throttle_mix_at_min: false,
            small_angle_error: true,
            accel_near_1g: true,
            rangefinder_near_ground: true,
        }
    }

    #[test]
    fn test_sub_manual_neutral() {
        let mut sub = SubModes::new();
        let input = default_input();
        sub.set_mode(FlightModeId::Manual, &input);
        let out = sub.update(&input, 0.02);
        match out {
            ModeOutput::RateTarget { throttle, .. } => {
                assert!((throttle - 0.0).abs() < 0.01, "Neutral stick = 0 throttle: {}", throttle);
            }
            _ => panic!("Expected RateTarget"),
        }
    }

    #[test]
    fn test_sub_depth_hold_captures_depth() {
        let mut sub = SubModes::new();
        let input = default_input();
        sub.set_mode(FlightModeId::AltHold, &input);
        assert!((sub.depth_target() - 5.0).abs() < 0.01, "Should capture current depth");
    }

    #[test]
    fn test_sub_depth_hold_stick_adjusts_target() {
        let mut sub = SubModes::new();
        let mut input = default_input();
        sub.set_mode(FlightModeId::AltHold, &input);

        // Push throttle up (> 0.5 → positive stick → increase depth)
        input.rc_throttle = 0.8;
        for _ in 0..10 {
            sub.update(&input, 0.02);
        }
        assert!(sub.depth_target() > 5.0, "Depth target should increase: {}", sub.depth_target());
    }

    #[test]
    fn test_sub_stabilize_levels() {
        let mut sub = SubModes::new();
        let mut input = default_input();
        // Tilted 10° roll
        input.attitude = Quaternion::from_euler(0.17, 0.0, 0.0);
        sub.set_mode(FlightModeId::Stabilize, &input);
        let out = sub.update(&input, 0.02);
        match out {
            ModeOutput::RateTarget { roll_rate, .. } => {
                assert!(roll_rate < 0.0, "Should correct positive roll with negative rate: {}", roll_rate);
            }
            _ => panic!("Expected RateTarget"),
        }
    }

    #[test]
    fn test_sub_no_gps_required() {
        let sub = SubModes::new();
        assert!(!sub.requires_gps());
    }
}

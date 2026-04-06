#![no_std]

//! Vehicle definitions — data-driven config for any vehicle type.

#[cfg(feature = "std")]
extern crate std;

use meridian_math::Vec3;
use meridian_math::frames::Body;

/// Vehicle class — determines which controller traits are used.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleClass {
    Multirotor,
    FixedWing,
    Rover,
    Boat,
    Sub,
}

/// Multirotor frame type — selects the mixing matrix preset.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameType {
    QuadX,
    QuadPlus,
    HexX,
    HexPlus,
    OctaX,
    Custom,
}

/// Complete vehicle physics configuration.
/// Sufficient to set up SITL physics and the control system.
#[derive(Debug, Clone)]
pub struct VehiclePhysics {
    pub class: VehicleClass,
    pub frame: FrameType,
    pub mass_kg: f32,
    pub inertia: [f32; 3],        // Ixx, Iyy, Izz (kg·m²)
    pub drag_coefficient: f32,     // body drag area*Cd (m²)
    pub max_thrust_n: f32,         // total max thrust (all motors)
    pub motor_count: u8,
    pub diagonal_m: f32,           // frame diagonal (meters)

    // Motor model
    pub motor_expo: f32,           // thrust curve expo (0.65 default)
    pub hover_throttle: f32,       // throttle at hover (0.39 default)
    pub spin_min: f32,             // MOT_SPIN_MIN (0.15)
    pub spin_max: f32,             // MOT_SPIN_MAX (0.95)

    // Rotational damping
    pub terminal_rotation_rate: f32, // deg/s (120 default)
}

impl VehiclePhysics {
    /// Default quad-X configuration matching ArduPilot SITL defaults.
    /// Source: libraries/SITL/SIM_Frame.h
    pub fn default_quad() -> Self {
        let mass = 3.0;
        let diagonal = 0.35;
        let half_diag = diagonal / 2.0;
        Self {
            class: VehicleClass::Multirotor,
            frame: FrameType::QuadX,
            mass_kg: mass,
            // Source: SIM_Frame.cpp:636-640
            inertia: [
                mass * 0.25 * half_diag * half_diag,  // Ixx
                mass * 0.25 * half_diag * half_diag,  // Iyy
                mass * 0.5 * half_diag * half_diag,   // Izz
            ],
            drag_coefficient: 0.1,
            max_thrust_n: mass * 9.81 * 4.25,  // ~4.25:1 thrust-to-weight
            motor_count: 4,
            diagonal_m: diagonal,
            motor_expo: 0.65,        // SIM_Frame.h:111
            hover_throttle: 0.39,    // SIM_Frame.h:108
            spin_min: 0.15,          // AP_MotorsMulticopter.h:14
            spin_max: 0.95,          // AP_MotorsMulticopter.h:15
            terminal_rotation_rate: 120.0, // SIM_Frame.h:114
        }
    }

    /// Compute motor positions in body frame for a given frame type and diagonal.
    /// Returns Vec of (position, yaw_factor) for each motor.
    pub fn motor_positions(&self) -> heapless::Vec<(Vec3<Body>, f32), 12> {
        let mut positions = heapless::Vec::new();
        let r = self.diagonal_m / 2.0;

        // Motor angles and yaw factors from ArduPilot AP_MotorsMatrix.cpp.
        // These MUST match the mixing matrix or torques won't balance.
        let angles_yaw: &[(f32, f32)] = match self.frame {
            FrameType::QuadX => &[
                // setup_quad_matrix MOTOR_FRAME_TYPE_X: lines 592-601
                (45.0, 1.0), (-135.0, 1.0), (-45.0, -1.0), (135.0, -1.0),
            ],
            FrameType::QuadPlus => &[
                // setup_quad_matrix MOTOR_FRAME_TYPE_PLUS: lines 581-590
                (90.0, 1.0), (-90.0, 1.0), (0.0, -1.0), (180.0, -1.0),
            ],
            FrameType::HexX => &[
                // setup_hexa_matrix MOTOR_FRAME_TYPE_X: lines 793-803
                (90.0, -1.0), (-90.0, 1.0), (-30.0, -1.0),
                (150.0, 1.0), (30.0, 1.0), (-150.0, -1.0),
            ],
            FrameType::HexPlus => &[
                // setup_hexa_matrix MOTOR_FRAME_TYPE_PLUS: lines 780-790
                (0.0, -1.0), (180.0, 1.0), (-120.0, -1.0),
                (60.0, 1.0), (-60.0, 1.0), (120.0, -1.0),
            ],
            FrameType::OctaX => &[
                // setup_octa_matrix MOTOR_FRAME_TYPE_X: lines 875-888
                (22.5, -1.0), (-157.5, -1.0), (67.5, 1.0), (157.5, 1.0),
                (-22.5, 1.0), (-112.5, 1.0), (-67.5, -1.0), (112.5, -1.0),
            ],
            FrameType::Custom => &[],
        };

        for &(angle_deg, yaw) in angles_yaw {
            let rad = angle_deg * core::f32::consts::PI / 180.0;
            let pos = Vec3::<Body>::new(
                r * libm::cosf(rad),
                r * libm::sinf(rad),
                0.0,
            );
            let _ = positions.push((pos, yaw));
        }

        positions
    }

    /// Compute max thrust per motor.
    pub fn max_thrust_per_motor(&self) -> f32 {
        self.max_thrust_n / self.motor_count as f32
    }

    /// Compute thrust from a command (0..1) using the expo curve.
    /// Source: SIM_Motor.cpp thrust model
    /// `thrust = max_thrust * ((1 - expo) * cmd + expo * cmd²)`
    #[inline]
    pub fn thrust_from_command(&self, cmd: f32, max_thrust: f32) -> f32 {
        let cmd = cmd.clamp(0.0, 1.0);
        max_thrust * ((1.0 - self.motor_expo) * cmd + self.motor_expo * cmd * cmd)
    }
}

#[cfg(feature = "std")]
pub mod loader {
    //! TOML vehicle config loading (std only).
    //! For no_std embedded, configs are compiled in via build.rs.

    // Will be implemented when we need to load configs from files.
    // For now, VehiclePhysics::default_quad() is used directly.
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_default_quad_inertia() {
        let v = VehiclePhysics::default_quad();
        // Ixx = 3.0 * 0.25 * 0.175² = 0.0230
        assert!((v.inertia[0] - 0.0230).abs() < 0.001);
        assert!((v.inertia[1] - v.inertia[0]).abs() < 1e-6); // Ixx == Iyy
        assert!(v.inertia[2] > v.inertia[0]); // Izz > Ixx
    }

    #[test]
    fn test_thrust_curve() {
        let v = VehiclePhysics::default_quad();
        let max_t = v.max_thrust_per_motor();

        // At command 0, thrust is 0
        assert!((v.thrust_from_command(0.0, max_t)).abs() < 1e-6);

        // At command 1, thrust is max
        assert!((v.thrust_from_command(1.0, max_t) - max_t).abs() < 1e-3);

        // At hover throttle, thrust should be mass*g/4
        let hover_thrust = v.thrust_from_command(v.hover_throttle, max_t);
        let expected = v.mass_kg * 9.81 / v.motor_count as f32;
        // Should be approximately equal (within 10% — exact depends on max_thrust calc)
        assert!((hover_thrust - expected).abs() / expected < 0.15,
            "Hover thrust {} vs expected {}", hover_thrust, expected);
    }

    #[test]
    fn test_motor_positions_quad_x() {
        let v = VehiclePhysics::default_quad();
        let positions = v.motor_positions();
        assert_eq!(positions.len(), 4);

        // All motors should be at the same distance from center
        let r = v.diagonal_m / 2.0;
        for (pos, _yaw) in positions.iter() {
            let dist = pos.length();
            assert!((dist - r).abs() < 0.001, "Motor distance {} != {}", dist, r);
        }

        // Motor 0 at 45° should have positive x and y
        assert!(positions[0].0.x > 0.0);
        assert!(positions[0].0.y > 0.0);
    }
}

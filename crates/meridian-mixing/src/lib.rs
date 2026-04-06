#![no_std]

//! Motor mixing matrices and desaturation.
//!
//! Every frame definition is traced to AP_MotorsMatrix.cpp with line numbers.
//! Factors are normalized to ±0.5 for RPY, matching ArduPilot's normalise_rpy_factors().

pub mod spool;
pub use spool::{SpoolManager, SpoolState, SpoolConfig};

pub mod motor_test;

use core::f32::consts::PI;

/// Maximum number of motors supported (matches ArduPilot).
pub const MAX_MOTORS: usize = 12;

/// DShot zero-throttle constant. Values 0-47 are command slots; 48 = zero throttle.
/// Source: AP_HAL/RCOutput.h line 276: `static constexpr uint8_t DSHOT_ZERO_THROTTLE = 48`
pub const DSHOT_ZERO_THROTTLE: u16 = 48;

/// Co-rotating top-layer scale factor for OctaQuad co-rotating modes.
/// Source: AP_MotorsMatrix.h AP_MOTORS_FRAME_OCTAQUAD_COROTATING_SCALE_FACTOR
pub const OCTAQUAD_COROTATING_SCALE: f32 = 0.7;

/// Per-motor mixing factors after normalization.
#[derive(Debug, Clone, Copy)]
pub struct MotorFactor {
    pub roll: f32,     // [-0.5, 0.5]
    pub pitch: f32,    // [-0.5, 0.5]
    pub yaw: f32,      // [-0.5, 0.5]
    pub throttle: f32, // [0.0, 1.0]
}

/// A mixing matrix: one MotorFactor per motor.
#[derive(Debug, Clone)]
pub struct MixingMatrix {
    pub factors: heapless::Vec<MotorFactor, MAX_MOTORS>,
}

/// Convert motor angle (degrees from front, CW positive) and yaw factor
/// to roll/pitch factors. Source: AP_MotorsMatrix.cpp:537-545
fn angle_to_raw_factors(angle_deg: f32, yaw_factor: f32) -> MotorFactor {
    let angle_rad = angle_deg * PI / 180.0;
    MotorFactor {
        roll: libm::cosf(angle_rad + PI / 2.0),
        pitch: libm::cosf(angle_rad),
        yaw: yaw_factor,
        throttle: 1.0,
    }
}

/// Yaw factor constants matching ArduPilot. Source: AP_MotorsMatrix.h:10-11
pub const YAW_CW: f32 = -1.0;
pub const YAW_CCW: f32 = 1.0;

impl MixingMatrix {
    /// Create from raw motor definitions (angle + yaw factor pairs).
    /// Factors are automatically normalized to +/-0.5 range.
    pub fn from_angles(motors: &[(f32, f32)]) -> Self {
        let mut factors = heapless::Vec::new();
        for &(angle, yaw) in motors {
            let _ = factors.push(angle_to_raw_factors(angle, yaw));
        }
        let mut matrix = Self { factors };
        matrix.normalize();
        matrix
    }

    /// Create from raw roll/pitch/yaw factors (for special frames like V-tail, Y4).
    /// Factors are automatically normalized.
    pub fn from_raw(motors: &[(f32, f32, f32)]) -> Self {
        let mut factors = heapless::Vec::new();
        for &(roll, pitch, yaw) in motors {
            let _ = factors.push(MotorFactor { roll, pitch, yaw, throttle: 1.0 });
        }
        let mut matrix = Self { factors };
        matrix.normalize();
        matrix
    }

    /// Create from raw roll/pitch/yaw/throttle factors (for coaxial frames with
    /// non-uniform throttle such as OctaQuad co-rotating).
    pub fn from_raw4(motors: &[(f32, f32, f32, f32)]) -> Self {
        let mut factors = heapless::Vec::new();
        for &(roll, pitch, yaw, throttle) in motors {
            let _ = factors.push(MotorFactor { roll, pitch, yaw, throttle });
        }
        let mut matrix = Self { factors };
        matrix.normalize();
        matrix
    }

    /// Normalize RPY factors so max magnitude = 0.5, throttle max = 1.0.
    /// Source: AP_MotorsMatrix.cpp normalise_rpy_factors()
    fn normalize(&mut self) {
        let mut roll_max = 0.0f32;
        let mut pitch_max = 0.0f32;
        let mut yaw_max = 0.0f32;
        let mut throttle_max = 0.0f32;

        for f in self.factors.iter() {
            roll_max = roll_max.max(libm::fabsf(f.roll));
            pitch_max = pitch_max.max(libm::fabsf(f.pitch));
            yaw_max = yaw_max.max(libm::fabsf(f.yaw));
            throttle_max = throttle_max.max(f.throttle.max(0.0));
        }

        for f in self.factors.iter_mut() {
            if roll_max > 0.0 { f.roll = 0.5 * f.roll / roll_max; }
            if pitch_max > 0.0 { f.pitch = 0.5 * f.pitch / pitch_max; }
            if yaw_max > 0.0 { f.yaw = 0.5 * f.yaw / yaw_max; }
            if throttle_max > 0.0 { f.throttle = (f.throttle / throttle_max).max(0.0); }
        }
    }

    pub fn motor_count(&self) -> usize {
        self.factors.len()
    }

    // =================================================================
    //  QUAD presets (14 variants)
    //  Source: AP_MotorsMatrix.cpp setup_quad_matrix()
    // =================================================================

    /// Quad-X: Source: AP_MotorsMatrix.cpp:592-601
    pub fn quad_x() -> Self {
        Self::from_angles(&[
            (45.0, YAW_CCW),   // Motor 0: front-right, CCW
            (-135.0, YAW_CCW), // Motor 1: back-left, CCW
            (-45.0, YAW_CW),   // Motor 2: front-left, CW
            (135.0, YAW_CW),   // Motor 3: back-right, CW
        ])
    }

    /// Quad-Plus: Source: AP_MotorsMatrix.cpp:581-590
    pub fn quad_plus() -> Self {
        Self::from_angles(&[
            (90.0, YAW_CCW),   // Motor 0: right, CCW
            (-90.0, YAW_CCW),  // Motor 1: left, CCW
            (0.0, YAW_CW),     // Motor 2: front, CW
            (180.0, YAW_CW),   // Motor 3: back, CW
        ])
    }

    /// Quad NYT-Plus (No-Yaw-Torque Plus). Source: AP_MotorsMatrix.cpp:604-613
    pub fn quad_nyt_plus() -> Self {
        Self::from_angles(&[
            (90.0,  0.0),
            (-90.0, 0.0),
            (0.0,   0.0),
            (180.0, 0.0),
        ])
    }

    /// Quad NYT-X (No-Yaw-Torque X). Source: AP_MotorsMatrix.cpp:615-624
    pub fn quad_nyt_x() -> Self {
        Self::from_angles(&[
            (45.0,   0.0),
            (-135.0, 0.0),
            (-45.0,  0.0),
            (135.0,  0.0),
        ])
    }

    /// Quad BetaFlight-X. Source: AP_MotorsMatrix.cpp:627-636
    pub fn quad_bf_x() -> Self {
        Self::from_angles(&[
            (135.0, YAW_CW),
            (45.0,  YAW_CCW),
            (-135.0, YAW_CCW),
            (-45.0, YAW_CW),
        ])
    }

    /// Quad BetaFlight-X Reversed. Source: AP_MotorsMatrix.cpp:640-649
    pub fn quad_bf_x_rev() -> Self {
        Self::from_angles(&[
            (135.0, YAW_CCW),
            (45.0,  YAW_CW),
            (-135.0, YAW_CW),
            (-45.0, YAW_CCW),
        ])
    }

    /// Quad DJI-X. Source: AP_MotorsMatrix.cpp:652-661
    pub fn quad_dji_x() -> Self {
        Self::from_angles(&[
            (45.0,  YAW_CCW),
            (-45.0, YAW_CW),
            (-135.0, YAW_CCW),
            (135.0, YAW_CW),
        ])
    }

    /// Quad CW-X (Clockwise ordered X). Source: AP_MotorsMatrix.cpp:665-674
    pub fn quad_cw_x() -> Self {
        Self::from_angles(&[
            (45.0,  YAW_CCW),
            (135.0, YAW_CW),
            (-135.0, YAW_CCW),
            (-45.0, YAW_CW),
        ])
    }

    /// Quad-V. Source: AP_MotorsMatrix.cpp:678-687
    /// Asymmetric yaw factors: 0.7981 front, 1.0 rear.
    pub fn quad_v() -> Self {
        Self::from_angles(&[
            (45.0,   0.7981),
            (-135.0, 1.0),
            (-45.0, -0.7981),
            (135.0, -1.0),
        ])
    }

    /// Quad-H. Source: AP_MotorsMatrix.cpp:689-698
    /// X geometry with reversed spin directions.
    pub fn quad_h() -> Self {
        Self::from_angles(&[
            (45.0,  YAW_CW),
            (-135.0, YAW_CW),
            (-45.0, YAW_CCW),
            (135.0, YAW_CCW),
        ])
    }

    /// Quad V-Tail. Source: AP_MotorsMatrix.cpp:701-722
    /// Uses roll/pitch angle overload: (roll_deg, pitch_deg) -> (cos(roll+90), cos(pitch))
    pub fn quad_vtail() -> Self {
        Self::from_raw(&[
            (-0.8660,  0.5,     0.0),   // Motor 1: roll=cos(150°), pitch=cos(60°), no yaw
            ( 0.0,    -0.9397, -1.0),   // Motor 2: roll=cos(90°),  pitch=cos(-160°), CW
            ( 0.8660,  0.5,     0.0),   // Motor 3: roll=cos(30°),  pitch=cos(-60°), no yaw
            ( 0.0,    -0.9397,  1.0),   // Motor 4: roll=cos(90°),  pitch=cos(160°), CCW
        ])
    }

    /// Quad A-Tail. Source: AP_MotorsMatrix.cpp:724-742
    /// Same geometry as V-Tail but with swapped yaw directions.
    pub fn quad_atail() -> Self {
        Self::from_raw(&[
            (-0.8660,  0.5,     0.0),   // Motor 1: roll=cos(150°), pitch=cos(60°), no yaw
            ( 0.0,    -0.9397,  1.0),   // Motor 2: roll=cos(90°),  pitch=cos(-160°), CCW
            ( 0.8660,  0.5,     0.0),   // Motor 3: roll=cos(30°),  pitch=cos(-60°), no yaw
            ( 0.0,    -0.9397, -1.0),   // Motor 4: roll=cos(90°),  pitch=cos(160°), CW
        ])
    }

    /// Quad Plus Reversed. Source: AP_MotorsMatrix.cpp:744-753
    pub fn quad_plus_rev() -> Self {
        Self::from_angles(&[
            (90.0,  YAW_CW),
            (-90.0, YAW_CW),
            (0.0,   YAW_CCW),
            (180.0, YAW_CCW),
        ])
    }

    /// Quad Y4 (coaxial rear pair). Source: AP_MotorsMatrix.cpp:756-770
    /// Two front motors with opposed yaw, two coaxial rear motors with opposed yaw.
    pub fn quad_y4() -> Self {
        Self::from_raw(&[
            (-1.0,  1.0,  1.0),  // Motor 0: left-front, CCW
            ( 0.0, -1.0, -1.0),  // Motor 1: rear-upper, CW
            ( 0.0, -1.0,  1.0),  // Motor 2: rear-lower, CCW
            ( 1.0,  1.0, -1.0),  // Motor 3: right-front, CW
        ])
    }

    // =================================================================
    //  HEX presets (5 variants)
    //  Source: AP_MotorsMatrix.cpp setup_hexa_matrix()
    // =================================================================

    /// Hex-X: Source: AP_MotorsMatrix.cpp:793-803
    pub fn hex_x() -> Self {
        Self::from_angles(&[
            (90.0, YAW_CW),
            (-90.0, YAW_CCW),
            (-30.0, YAW_CW),
            (150.0, YAW_CCW),
            (30.0, YAW_CCW),
            (-150.0, YAW_CW),
        ])
    }

    /// Hex-Plus: Source: AP_MotorsMatrix.cpp:780-790
    /// Ordered by motor index (MOT_1-6 -> slot 0-5).
    pub fn hex_plus() -> Self {
        // ArduPilot motor index order (NOT clockwise around frame).
        // Source: AP_MotorsMatrix.cpp:780-790 setup_hex_plus()
        // CRITICAL: Wiring must match this exact index order or motors get wrong commands.
        Self::from_angles(&[
            (0.0,    YAW_CW),    // Motor 1 (index 0): front
            (180.0,  YAW_CCW),   // Motor 2 (index 1): back
            (-120.0, YAW_CW),    // Motor 3 (index 2): back-left
            (60.0,   YAW_CCW),   // Motor 4 (index 3): front-right
            (-60.0,  YAW_CCW),   // Motor 5 (index 4): front-left
            (120.0,  YAW_CW),    // Motor 6 (index 5): back-right
        ])
    }

    /// Hex-H. Source: AP_MotorsMatrix.cpp:806-816
    /// Motors ordered by index (1-6 in ArduPilot, 0-5 here).
    pub fn hex_h() -> Self {
        Self::from_raw(&[
            (-1.0,  1.0,  1.0),  // Motor 1: CCW
            (-1.0,  0.0, -1.0),  // Motor 2: CW
            (-1.0, -1.0,  1.0),  // Motor 3: CCW
            ( 1.0, -1.0, -1.0),  // Motor 4: CW
            ( 1.0,  0.0,  1.0),  // Motor 5: CCW
            ( 1.0,  1.0, -1.0),  // Motor 6: CW
        ])
    }

    /// Hex DJI-X. Source: AP_MotorsMatrix.cpp:820-830
    pub fn hex_dji_x() -> Self {
        Self::from_angles(&[
            (30.0,  YAW_CCW),
            (-30.0, YAW_CW),
            (-90.0, YAW_CCW),
            (-150.0, YAW_CW),
            (150.0, YAW_CCW),
            (90.0,  YAW_CW),
        ])
    }

    /// Hex CW-X. Source: AP_MotorsMatrix.cpp:833-843
    pub fn hex_cw_x() -> Self {
        Self::from_angles(&[
            (30.0,  YAW_CCW),
            (90.0,  YAW_CW),
            (150.0, YAW_CCW),
            (-150.0, YAW_CW),
            (-90.0, YAW_CCW),
            (-30.0, YAW_CW),
        ])
    }

    // =================================================================
    //  OCTA presets (7 variants)
    //  Source: AP_MotorsMatrix.cpp setup_octa_matrix()
    // =================================================================

    /// Octa-Plus. Source: AP_MotorsMatrix.cpp:859-872
    pub fn octa_plus() -> Self {
        Self::from_angles(&[
            (0.0,   YAW_CW),
            (180.0, YAW_CW),
            (45.0,  YAW_CCW),
            (135.0, YAW_CCW),
            (-45.0, YAW_CCW),
            (-135.0, YAW_CCW),
            (-90.0, YAW_CW),
            (90.0,  YAW_CW),
        ])
    }

    /// Octa-X: Source: AP_MotorsMatrix.cpp:875-888
    pub fn octa_x() -> Self {
        Self::from_angles(&[
            (22.5, YAW_CW),
            (-157.5, YAW_CW),
            (67.5, YAW_CCW),
            (157.5, YAW_CCW),
            (-22.5, YAW_CCW),
            (-112.5, YAW_CCW),
            (-67.5, YAW_CW),
            (112.5, YAW_CW),
        ])
    }

    /// Octa-V. Source: AP_MotorsMatrix.cpp:890-903
    /// Motors ordered by index (1-8 in ArduPilot, 0-7 here).
    pub fn octa_v() -> Self {
        Self::from_raw(&[
            (-1.0,   1.0,  -1.0),  // Motor 1: CW
            (-0.83,  0.34,  1.0),  // Motor 2: CCW
            (-0.67, -0.32, -1.0),  // Motor 3: CW
            (-0.50, -1.0,   1.0),  // Motor 4: CCW
            ( 0.50, -1.0,  -1.0),  // Motor 5: CW
            ( 0.67, -0.32,  1.0),  // Motor 6: CCW
            ( 0.83,  0.34, -1.0),  // Motor 7: CW
            ( 1.0,   1.0,   1.0),  // Motor 8: CCW
        ])
    }

    /// Octa-H. Source: AP_MotorsMatrix.cpp:905-918
    /// Motors ordered by index (1-8 in ArduPilot, 0-7 here).
    pub fn octa_h() -> Self {
        Self::from_raw(&[
            (-1.0,  1.0,   -1.0),  // Motor 1: CW
            (-1.0,  0.333,  1.0),  // Motor 2: CCW
            (-1.0, -0.333, -1.0),  // Motor 3: CW
            (-1.0, -1.0,    1.0),  // Motor 4: CCW
            ( 1.0, -1.0,   -1.0),  // Motor 5: CW
            ( 1.0, -0.333,  1.0),  // Motor 6: CCW
            ( 1.0,  0.333, -1.0),  // Motor 7: CW
            ( 1.0,  1.0,    1.0),  // Motor 8: CCW
        ])
    }

    /// Octa-I (linear arrangement). Source: AP_MotorsMatrix.cpp:920-933
    /// Motors ordered by index (1-8 in ArduPilot, 0-7 here).
    pub fn octa_i() -> Self {
        Self::from_raw(&[
            (-0.333,  1.0, -1.0),  // Motor 1: CW
            (-1.0,    1.0,  1.0),  // Motor 2: CCW
            (-1.0,   -1.0, -1.0),  // Motor 3: CW
            (-0.333, -1.0,  1.0),  // Motor 4: CCW
            ( 0.333, -1.0, -1.0),  // Motor 5: CW
            ( 1.0,   -1.0,  1.0),  // Motor 6: CCW
            ( 1.0,    1.0, -1.0),  // Motor 7: CW
            ( 0.333,  1.0,  1.0),  // Motor 8: CCW
        ])
    }

    /// Octa DJI-X. Source: AP_MotorsMatrix.cpp:935-948
    /// DJI motor ordering: motors are added by index (1,8,7,6,5,4,3,2),
    /// so the resulting slot order differs from CW-X.
    /// Slot 0 = Motor 1 at 22.5, Slot 1 = Motor 8 at -22.5, etc.
    pub fn octa_dji_x() -> Self {
        Self::from_angles(&[
            (22.5,   YAW_CCW),   // Motor 1 (index 1 -> slot 0)
            (-22.5,  YAW_CW),    // Motor 8 (index 8 -> slot 1)
            (-67.5,  YAW_CCW),   // Motor 7 (index 7 -> slot 2)
            (-112.5, YAW_CW),    // Motor 6 (index 6 -> slot 3)
            (-157.5, YAW_CCW),   // Motor 5 (index 5 -> slot 4)
            (157.5,  YAW_CW),    // Motor 4 (index 4 -> slot 5)
            (112.5,  YAW_CCW),   // Motor 3 (index 3 -> slot 6)
            (67.5,   YAW_CW),    // Motor 2 (index 2 -> slot 7)
        ])
    }

    /// Octa CW-X (Clockwise ordered). Source: AP_MotorsMatrix.cpp:950-963
    /// Ordered by motor index (1-8 -> 0-7).
    pub fn octa_cw_x() -> Self {
        Self::from_angles(&[
            (22.5,   YAW_CCW),   // Motor 1
            (67.5,   YAW_CW),    // Motor 2
            (112.5,  YAW_CCW),   // Motor 3
            (157.5,  YAW_CW),    // Motor 4
            (-157.5, YAW_CCW),   // Motor 5
            (-112.5, YAW_CW),    // Motor 6
            (-67.5,  YAW_CCW),   // Motor 7
            (-22.5,  YAW_CW),    // Motor 8
        ])
    }

    // =================================================================
    //  OCTAQUAD presets (9 variants -- coaxial quads)
    //  Source: AP_MotorsMatrix.cpp setup_octaquad_matrix()
    // =================================================================

    /// OctaQuad-Plus. Source: AP_MotorsMatrix.cpp:978-991
    /// Coaxial quad in plus config. Ordered by motor index (1-8 -> 0-7).
    pub fn octaquad_plus() -> Self {
        Self::from_angles(&[
            (0.0,    YAW_CCW),   // Motor 1: front-upper, CCW
            (0.0,    YAW_CW),    // Motor 2: front-lower, CW
            (90.0,   YAW_CW),    // Motor 3: right-upper, CW
            (90.0,   YAW_CCW),   // Motor 4: right-lower, CCW
            (180.0,  YAW_CCW),   // Motor 5: back-upper, CCW
            (180.0,  YAW_CW),    // Motor 6: back-lower, CW
            (-90.0,  YAW_CW),    // Motor 7: left-upper, CW
            (-90.0,  YAW_CCW),   // Motor 8: left-lower, CCW
        ])
    }

    /// OctaQuad-X. Source: AP_MotorsMatrix.cpp:993-1006
    /// Coaxial quad in X config. Ordered by motor index (1-8 -> 0-7).
    pub fn octaquad_x() -> Self {
        Self::from_angles(&[
            (45.0,   YAW_CCW),   // Motor 1: front-right-upper, CCW
            (45.0,   YAW_CW),    // Motor 2: front-right-lower, CW
            (135.0,  YAW_CW),    // Motor 3: back-right-upper, CW
            (135.0,  YAW_CCW),   // Motor 4: back-right-lower, CCW
            (-135.0, YAW_CCW),   // Motor 5: back-left-upper, CCW
            (-135.0, YAW_CW),    // Motor 6: back-left-lower, CW
            (-45.0,  YAW_CW),    // Motor 7: front-left-upper, CW
            (-45.0,  YAW_CCW),   // Motor 8: front-left-lower, CCW
        ])
    }

    /// OctaQuad-V. Source: AP_MotorsMatrix.cpp:1008-1021
    /// Uses asymmetric yaw factors (0.7981 front, 1.0 rear).
    /// Ordered by motor index (1-8 -> 0-7).
    pub fn octaquad_v() -> Self {
        Self::from_angles(&[
            (45.0,    0.7981),   // Motor 1: front-right-upper
            (45.0,   -0.7981),   // Motor 2: front-right-lower
            (135.0,  -1.0),      // Motor 3: back-right-upper
            (135.0,   1.0),      // Motor 4: back-right-lower
            (-135.0,  1.0),      // Motor 5: back-left-upper
            (-135.0, -1.0),      // Motor 6: back-left-lower
            (-45.0,  -0.7981),   // Motor 7: front-left-upper
            (-45.0,   0.7981),   // Motor 8: front-left-lower
        ])
    }

    /// OctaQuad-H. Source: AP_MotorsMatrix.cpp:1023-1036
    /// X geometry with reversed spin directions. Ordered by motor index.
    pub fn octaquad_h() -> Self {
        Self::from_angles(&[
            (45.0,   YAW_CW),    // Motor 1
            (45.0,   YAW_CCW),   // Motor 2
            (135.0,  YAW_CCW),   // Motor 3
            (135.0,  YAW_CW),    // Motor 4
            (-135.0, YAW_CW),    // Motor 5
            (-135.0, YAW_CCW),   // Motor 6
            (-45.0,  YAW_CCW),   // Motor 7
            (-45.0,  YAW_CW),    // Motor 8
        ])
    }

    /// OctaQuad CW-X. Source: AP_MotorsMatrix.cpp:1039-1053
    /// Ordered by motor index.
    pub fn octaquad_cw_x() -> Self {
        Self::from_angles(&[
            (45.0,   YAW_CCW),   // Motor 1
            (45.0,   YAW_CW),    // Motor 2
            (135.0,  YAW_CW),    // Motor 3
            (135.0,  YAW_CCW),   // Motor 4
            (-135.0, YAW_CCW),   // Motor 5
            (-135.0, YAW_CW),    // Motor 6
            (-45.0,  YAW_CW),    // Motor 7
            (-45.0,  YAW_CCW),   // Motor 8
        ])
    }

    /// OctaQuad BF-X. Source: AP_MotorsMatrix.cpp:1056-1069
    /// Betaflight motor order for OctaQuad. Ordered by motor index.
    pub fn octaquad_bf_x() -> Self {
        Self::from_angles(&[
            (45.0,   YAW_CCW),   // Motor 1: from BF motor 2
            (45.0,   YAW_CW),    // Motor 2: from BF motor 6
            (135.0,  YAW_CW),    // Motor 3: from BF motor 1
            (135.0,  YAW_CCW),   // Motor 4: from BF motor 5
            (-135.0, YAW_CCW),   // Motor 5: from BF motor 3
            (-135.0, YAW_CW),    // Motor 6: from BF motor 7
            (-45.0,  YAW_CW),    // Motor 7: from BF motor 4
            (-45.0,  YAW_CCW),   // Motor 8: from BF motor 8
        ])
    }

    /// OctaQuad BF-X Reversed. Source: AP_MotorsMatrix.cpp:1071-1084
    /// Ordered by motor index.
    pub fn octaquad_bf_x_rev() -> Self {
        Self::from_angles(&[
            (45.0,   YAW_CW),    // Motor 1
            (45.0,   YAW_CCW),   // Motor 2
            (135.0,  YAW_CCW),   // Motor 3
            (135.0,  YAW_CW),    // Motor 4
            (-135.0, YAW_CW),    // Motor 5
            (-135.0, YAW_CCW),   // Motor 6
            (-45.0,  YAW_CCW),   // Motor 7
            (-45.0,  YAW_CW),    // Motor 8
        ])
    }

    /// OctaQuad X Co-Rotating. Source: AP_MotorsMatrix.cpp:1087-1108
    pub fn octaquad_x_cor() -> Self {
        let s = OCTAQUAD_COROTATING_SCALE;
        Self::from_raw4(&[
            (-0.3536*s,  0.3536*s,  0.5*s,  1.0),
            (-0.3536,    0.3536,   -0.5,    1.0),
            ( 0.3536*s, -0.3536*s,  0.5*s,  1.0),
            ( 0.3536,   -0.3536,   -0.5,    1.0),
            ( 0.3536*s,  0.3536*s, -0.5*s,  1.0),
            ( 0.3536,    0.3536,    0.5,    1.0),
            (-0.3536*s, -0.3536*s, -0.5*s,  1.0),
            (-0.3536,   -0.3536,    0.5,    1.0),
        ])
    }

    /// OctaQuad CW-X Co-Rotating. Source: AP_MotorsMatrix.cpp:1110-1131
    pub fn octaquad_cw_x_cor() -> Self {
        let s = OCTAQUAD_COROTATING_SCALE;
        Self::from_raw4(&[
            (-0.3536*s,  0.3536*s,  0.5*s,  1.0),
            (-0.3536,    0.3536,   -0.5,    1.0),
            (-0.3536*s, -0.3536*s, -0.5*s,  1.0),
            (-0.3536,   -0.3536,    0.5,    1.0),
            ( 0.3536*s, -0.3536*s,  0.5*s,  1.0),
            ( 0.3536,   -0.3536,   -0.5,    1.0),
            ( 0.3536*s,  0.3536*s, -0.5*s,  1.0),
            ( 0.3536,    0.3536,    0.5,    1.0),
        ])
    }

    // =================================================================
    //  Y6 presets (3 variants)
    //  Source: AP_MotorsMatrix.cpp setup_y6_matrix()
    // =================================================================

    /// Y6-A (default). Source: AP_MotorsMatrix.cpp:1224-1237
    /// Three Y-arms, each with a CW/CCW stacked pair.
    /// Motors ordered by index (1-6 in ArduPilot, 0-5 here).
    pub fn y6() -> Self {
        Self::from_raw(&[
            (-1.0,  0.666, YAW_CW),   // Motor 1: left-upper, CW
            (-1.0,  0.666, YAW_CCW),  // Motor 2: left-lower, CCW
            ( 0.0, -1.333, YAW_CCW),  // Motor 3: rear-upper, CCW
            ( 0.0, -1.333, YAW_CW),   // Motor 4: rear-lower, CW
            ( 1.0,  0.666, YAW_CW),   // Motor 5: right-upper, CW
            ( 1.0,  0.666, YAW_CCW),  // Motor 6: right-lower, CCW
        ])
    }

    /// Y6-B. Source: AP_MotorsMatrix.cpp:1196-1209
    /// Motors ordered by index (1-6 in ArduPilot, 0-5 here).
    pub fn y6b() -> Self {
        Self::from_raw(&[
            (-1.0,  0.5, YAW_CW),     // Motor 1: left-upper, CW
            (-1.0,  0.5, YAW_CCW),    // Motor 2: left-lower, CCW
            ( 0.0, -1.0, YAW_CW),     // Motor 3: rear-upper, CW
            ( 0.0, -1.0, YAW_CCW),    // Motor 4: rear-lower, CCW
            ( 1.0,  0.5, YAW_CW),     // Motor 5: right-upper, CW
            ( 1.0,  0.5, YAW_CCW),    // Motor 6: right-lower, CCW
        ])
    }

    /// Y6-F (FireFly Y6). Source: AP_MotorsMatrix.cpp:1210-1223
    /// Motors ordered by index (1-6 in ArduPilot, 0-5 here).
    pub fn y6f() -> Self {
        Self::from_raw(&[
            (-1.0,  0.5, YAW_CCW),    // Motor 1: left-upper, CCW
            (-1.0,  0.5, YAW_CW),     // Motor 2: left-lower, CW
            ( 0.0, -1.0, YAW_CCW),    // Motor 3: rear-upper, CCW
            ( 0.0, -1.0, YAW_CW),     // Motor 4: rear-lower, CW
            ( 1.0,  0.5, YAW_CCW),    // Motor 5: right-upper, CCW
            ( 1.0,  0.5, YAW_CW),     // Motor 6: right-lower, CW
        ])
    }

    // =================================================================
    //  DODECAHEXA presets (2 variants -- 12-motor)
    //  Source: AP_MotorsMatrix.cpp setup_dodecahexa_matrix()
    // =================================================================

    /// DodecaHexa Plus. Source: AP_MotorsMatrix.cpp:1145-1163
    /// 12 motors in 6 stacked pairs. Ordered by motor index (1-12 -> 0-11).
    pub fn dodecahexa_plus() -> Self {
        Self::from_angles(&[
            (0.0,    YAW_CCW),   // Motor 1:  front-upper, CCW
            (0.0,    YAW_CW),    // Motor 2:  front-lower, CW
            (60.0,   YAW_CW),    // Motor 3:  right-front-upper, CW
            (60.0,   YAW_CCW),   // Motor 4:  right-front-lower, CCW
            (120.0,  YAW_CCW),   // Motor 5:  right-back-upper, CCW
            (120.0,  YAW_CW),    // Motor 6:  right-back-lower, CW
            (180.0,  YAW_CW),    // Motor 7:  back-upper, CW
            (180.0,  YAW_CCW),   // Motor 8:  back-lower, CCW
            (-120.0, YAW_CCW),   // Motor 9:  left-back-upper, CCW
            (-120.0, YAW_CW),    // Motor 10: left-back-lower, CW
            (-60.0,  YAW_CW),    // Motor 11: left-front-upper, CW
            (-60.0,  YAW_CCW),   // Motor 12: left-front-lower, CCW
        ])
    }

    /// DodecaHexa X. Source: AP_MotorsMatrix.cpp:1164-1182
    /// 12 motors in 6 stacked pairs, rotated 30deg from Plus. Ordered by motor index.
    pub fn dodecahexa_x() -> Self {
        Self::from_angles(&[
            (30.0,   YAW_CCW),   // Motor 1
            (30.0,   YAW_CW),    // Motor 2
            (90.0,   YAW_CW),    // Motor 3
            (90.0,   YAW_CCW),   // Motor 4
            (150.0,  YAW_CCW),   // Motor 5
            (150.0,  YAW_CW),    // Motor 6
            (-150.0, YAW_CW),    // Motor 7
            (-150.0, YAW_CCW),   // Motor 8
            (-90.0,  YAW_CCW),   // Motor 9
            (-90.0,  YAW_CW),    // Motor 10
            (-30.0,  YAW_CW),    // Motor 11
            (-30.0,  YAW_CCW),   // Motor 12
        ])
    }

    // =================================================================
    //  DECA presets (2 variants -- 10-motor)
    //  Source: AP_MotorsMatrix.cpp setup_deca_matrix()
    // =================================================================

    /// Deca Plus. Source: AP_MotorsMatrix.cpp:1247-1261
    /// 10 motors alternating CW/CCW. Ordered by motor index.
    pub fn deca_plus() -> Self {
        Self::from_angles(&[
            (0.0,    YAW_CCW),   // Motor 1
            (36.0,   YAW_CW),    // Motor 2
            (72.0,   YAW_CCW),   // Motor 3
            (108.0,  YAW_CW),    // Motor 4
            (144.0,  YAW_CCW),   // Motor 5
            (180.0,  YAW_CW),    // Motor 6
            (-144.0, YAW_CCW),   // Motor 7
            (-108.0, YAW_CW),    // Motor 8
            (-72.0,  YAW_CCW),   // Motor 9
            (-36.0,  YAW_CW),    // Motor 10
        ])
    }

    /// Deca X. Source: AP_MotorsMatrix.cpp:1264-1278
    /// 10 motors alternating CW/CCW, rotated 18deg from Plus. Ordered by motor index.
    pub fn deca_x() -> Self {
        Self::from_angles(&[
            (18.0,   YAW_CCW),   // Motor 1
            (54.0,   YAW_CW),    // Motor 2
            (90.0,   YAW_CCW),   // Motor 3
            (126.0,  YAW_CW),    // Motor 4
            (162.0,  YAW_CCW),   // Motor 5
            (-162.0, YAW_CW),    // Motor 6
            (-126.0, YAW_CCW),   // Motor 7
            (-90.0,  YAW_CW),    // Motor 8
            (-54.0,  YAW_CCW),   // Motor 9
            (-18.0,  YAW_CW),    // Motor 10
        ])
    }
}

// =================================================================
//  Limit flags
//  Source: AP_MotorsMulticopter.h limit struct
// =================================================================

/// Motor limit flags, set by the mixer to communicate saturation to the flight controller.
#[derive(Debug, Clone, Copy, Default)]
pub struct MixerLimits {
    /// RPY commands were scaled down (any axis saturated).
    pub rpy: bool,
    /// Throttle hit upper bound.
    pub throttle_upper: bool,
    /// Throttle hit lower bound.
    pub throttle_lower: bool,
    /// Yaw was reduced to preserve RP authority.
    pub yaw_limited: bool,
}

// =================================================================
//  Thrust linearization -- corrected quadratic inversion
//  Source: AP_Motors_Thrust_Linearization.cpp
// =================================================================

/// Thrust curve linearization using ArduPilot's exact quadratic inversion.
/// Source: AP_Motors_Thrust_Linearization.cpp apply_thrust_curve_and_volt_scaling()
///
/// The thrust model is: `thrust = expo * pwm^2 + (1-expo) * pwm`
/// We invert this to find pwm given desired thrust:
///   `pwm = ((expo-1) + sqrt((1-expo)^2 + 4*expo*thrust)) / (2*expo)`
pub fn apply_thrust_curve(thrust_in: f32, expo: f32) -> f32 {
    apply_thrust_curve_with_comp(thrust_in, expo, 1.0)
}

/// Thrust curve with battery voltage compensation integrated.
pub fn apply_thrust_curve_with_comp(thrust_in: f32, expo: f32, lift_max: f32) -> f32 {
    let t = thrust_in.clamp(0.0, 1.0);
    if expo <= 0.0 || expo > 1.0 {
        if lift_max > 0.0 { return (t / lift_max).clamp(0.0, 1.0); }
        return t;
    }
    let thr_scaled = t.min(1.0);
    let a = expo;
    let b = 1.0 - expo;
    let discriminant = b * b + 4.0 * a * thr_scaled;
    let pwm = if discriminant >= 0.0 {
        (-b + libm::sqrtf(discriminant)) / (2.0 * a)
    } else {
        thr_scaled
    };
    let result = if lift_max > 0.0 { pwm / lift_max } else { pwm };
    result.clamp(0.0, 1.0)
}

/// Inverse thrust curve: given actuator (PWM) output, compute the thrust it produces.
/// Source: AP_Motors_Thrust_Linearization.cpp actuator_to_thrust()
pub fn actuator_to_thrust(actuator: f32, expo: f32) -> f32 {
    let a = actuator.clamp(0.0, 1.0);
    if expo <= 0.0 || expo > 1.0 { return a; }
    expo * a * a + (1.0 - expo) * a
}

// =================================================================
//  Voltage compensation with LP filter
//  Source: AP_Motors_Thrust_Linearization.cpp update_lift_max_from_batt_voltage()
// =================================================================

/// Battery voltage compensation state with 0.5 Hz LP filter.
pub struct VoltageCompensation {
    /// Nominal (fully charged) voltage.
    pub voltage_nom: f32,
    /// Filtered voltage ratio (V_actual / V_nominal).
    filtered_ratio: f32,
    /// Whether the filter has been seeded.
    initialized: bool,
    /// Thrust expo for lift_max computation.
    pub expo: f32,
    /// Filter cutoff frequency (Hz). Default 0.5 Hz per ArduPilot.
    pub filter_hz: f32,
}

impl VoltageCompensation {
    pub fn new(voltage_nom: f32, expo: f32) -> Self {
        Self {
            voltage_nom,
            filtered_ratio: 1.0,
            initialized: false,
            expo,
            filter_hz: 0.5,
        }
    }

    /// Update with a new voltage reading and return the `lift_max` factor.
    pub fn update(&mut self, voltage_actual: f32, dt: f32) -> f32 {
        if self.voltage_nom <= 0.0 || voltage_actual <= 0.0 {
            return 1.0;
        }
        let ratio = (voltage_actual / self.voltage_nom).clamp(0.5, 1.5);

        if !self.initialized {
            self.filtered_ratio = ratio;
            self.initialized = true;
        } else {
            let rc = 1.0 / (2.0 * PI * self.filter_hz);
            let alpha = dt / (rc + dt);
            self.filtered_ratio += alpha * (ratio - self.filtered_ratio);
        }

        let f = self.filtered_ratio;
        let lift_max = self.expo * f * f + (1.0 - self.expo) * f;
        lift_max.clamp(0.1, 1.5)
    }

    /// Get current lift_max without updating.
    pub fn lift_max(&self) -> f32 {
        let f = self.filtered_ratio;
        let lm = self.expo * f * f + (1.0 - self.expo) * f;
        lm.clamp(0.1, 1.5)
    }

    /// Reset filter state.
    pub fn reset(&mut self) {
        self.initialized = false;
        self.filtered_ratio = 1.0;
    }
}

/// Simple voltage compensation (legacy, no filter).
pub fn apply_voltage_comp(motor_out: f32, voltage_nom: f32, voltage_actual: f32) -> f32 {
    if voltage_actual <= 0.0 || voltage_nom <= 0.0 { return motor_out; }
    let ratio = voltage_nom / voltage_actual;
    let comp = (ratio * ratio).min(1.25);
    (motor_out * comp).clamp(0.0, 1.0)
}

// =================================================================
//  Mixer configuration
// =================================================================

/// Mixer configuration matching ArduPilot's AP_MotorsMulticopter parameters.
#[derive(Debug, Clone, Copy)]
pub struct MixerConfig {
    /// MOT_SPIN_MIN default 0.15
    pub spin_min: f32,
    /// MOT_SPIN_MAX default 0.95
    pub spin_max: f32,
    /// MOT_YAW_HEADROOM (thousandths). Default 200 = 0.200 minimum yaw authority.
    pub yaw_headroom: u16,
    /// Enable thrust_boost motor-loss compensation.
    pub thrust_boost: bool,
    /// Index of lost motor (-1 = none).
    pub motor_lost_index: i8,
}

impl Default for MixerConfig {
    fn default() -> Self {
        Self {
            spin_min: 0.15,
            spin_max: 0.95,
            yaw_headroom: 200,
            thrust_boost: false,
            motor_lost_index: -1,
        }
    }
}

/// The mixer: takes axis commands + throttle -> per-motor outputs.
/// Implements ArduPilot's desaturation algorithm.
/// Source: AP_MotorsMatrix.cpp output_armed_stabilizing() lines 213-404
pub struct Mixer {
    pub matrix: MixingMatrix,
    pub config: MixerConfig,
    /// Last computed limit flags.
    pub limits: MixerLimits,
}

impl Mixer {
    pub fn new(matrix: MixingMatrix) -> Self {
        Self {
            matrix,
            config: MixerConfig::default(),
            limits: MixerLimits::default(),
        }
    }

    pub fn with_config(matrix: MixingMatrix, config: MixerConfig) -> Self {
        Self {
            matrix,
            config,
            limits: MixerLimits::default(),
        }
    }

    /// Mix axis commands into per-motor outputs with desaturation.
    /// Simple interface without compensation. Use `mix_compensated()` for full AP parity.
    pub fn mix(&mut self, roll: f32, pitch: f32, yaw: f32, throttle: f32) -> [f32; MAX_MOTORS] {
        self.mix_compensated(roll, pitch, yaw, throttle, 1.0, 0.5)
    }

    /// Full ArduPilot-equivalent mix with battery/altitude compensation.
    ///
    /// - `compensation_gain`: from VoltageCompensation (1/lift_max). 1.0 = no compensation.
    /// - `throttle_avg_max`: dynamic throttle ceiling. Pass 0.5 as default.
    ///
    /// Source: AP_MotorsMatrix.cpp output_armed_stabilizing() lines 213-404
    pub fn mix_compensated(
        &mut self,
        roll: f32,
        pitch: f32,
        yaw: f32,
        throttle: f32,
        compensation_gain: f32,
        throttle_avg_max: f32,
    ) -> [f32; MAX_MOTORS] {
        let mut output = [0.0f32; MAX_MOTORS];
        let n = self.matrix.motor_count();
        if n == 0 { return output; }

        // Reset limits
        self.limits = MixerLimits::default();

        // Step 1: Apply compensation gain to all inputs (AP L216-231)
        let roll_thrust = roll * compensation_gain;
        let pitch_thrust = pitch * compensation_gain;
        let yaw_thrust = yaw * compensation_gain;
        let throttle_thrust = throttle.clamp(0.0, 1.0) * compensation_gain;

        // Step 2: throttle_thrust_best_rpy (AP L251)
        let throttle_thrust_best_rpy = throttle_avg_max.min(0.5);

        // Step 3: RP contribution per motor
        let mut rp = [0.0f32; MAX_MOTORS];
        for i in 0..n {
            let f = &self.matrix.factors[i];
            rp[i] = roll_thrust * f.roll + pitch_thrust * f.pitch;
        }

        // Step 4: Per-motor directional yaw headroom (AP L277-325)
        let mut yaw_allowed = f32::MAX;
        for i in 0..n {
            let yf = self.matrix.factors[i].yaw;
            if libm::fabsf(yf) < 1e-6 { continue; }

            let thrust_rp_best_throttle = throttle_thrust_best_rpy + rp[i];

            let motor_room = if (yaw_thrust * yf) > 0.0 {
                1.0 - thrust_rp_best_throttle
            } else {
                thrust_rp_best_throttle
            };

            let motor_yaw_allowed = motor_room.max(0.0) / libm::fabsf(yf);
            yaw_allowed = yaw_allowed.min(motor_yaw_allowed);
        }
        if yaw_allowed == f32::MAX { yaw_allowed = 0.0; }

        // Step 5: MOT_YAW_HEADROOM floor (AP L302-308)
        let yaw_allowed_min = self.config.yaw_headroom as f32 * 0.001;

        // Step 6: Thrust boost / motor-loss override (AP L304-325)
        if self.config.thrust_boost && self.config.motor_lost_index >= 0 {
            let boost_ratio = 0.5;
            yaw_allowed = yaw_allowed.max(yaw_allowed_min * boost_ratio);
        } else {
            yaw_allowed = yaw_allowed.max(yaw_allowed_min);
        }

        // Step 7: Clamp yaw
        let actual_yaw = yaw_thrust.clamp(-yaw_allowed, yaw_allowed);
        if libm::fabsf(actual_yaw) < libm::fabsf(yaw_thrust) {
            self.limits.yaw_limited = true;
        }

        // Step 8: Combine RP + yaw (AP L334-357)
        let mut rpy = [0.0f32; MAX_MOTORS];
        let mut rpy_low = 0.0f32;
        let mut rpy_high = 0.0f32;
        for i in 0..n {
            let f = &self.matrix.factors[i];
            rpy[i] = rp[i] + actual_yaw * f.yaw;

            if self.config.thrust_boost
                && self.config.motor_lost_index >= 0
                && i == self.config.motor_lost_index as usize
            {
                rpy[i] = 0.0;
            }

            if i == 0 || rpy[i] < rpy_low { rpy_low = rpy[i]; }
            if i == 0 || rpy[i] > rpy_high { rpy_high = rpy[i]; }
        }

        // Step 9: RPY proportional scale (AP L360-366)
        let rpy_range = rpy_high - rpy_low;
        let mut rpy_scale = if rpy_range > 1.0 {
            self.limits.rpy = true;
            1.0 / rpy_range
        } else {
            1.0
        };

        // Secondary floor (AP L364-366)
        if rpy_low < 0.0 {
            let scale_floor = -throttle_avg_max / rpy_low;
            if scale_floor < rpy_scale {
                rpy_scale = scale_floor;
                self.limits.rpy = true;
            }
        }

        for i in 0..n {
            rpy[i] *= rpy_scale;
        }
        rpy_low *= rpy_scale;
        rpy_high *= rpy_scale;

        // Step 10: Throttle adjustment (AP L368-395)
        let thr_lo = -rpy_low;
        let thr_hi = 1.0 - rpy_high;
        let throttle_out = if thr_lo <= thr_hi {
            let clamped = throttle_thrust.clamp(thr_lo, thr_hi);
            if throttle_thrust >= thr_hi {
                self.limits.throttle_upper = true;
            }
            if throttle_thrust <= thr_lo {
                self.limits.throttle_lower = true;
            }
            clamped
        } else {
            self.limits.throttle_upper = true;
            self.limits.throttle_lower = true;
            (thr_lo + thr_hi) * 0.5
        };

        // Step 11: Final output (AP L391-395)
        for i in 0..n {
            let f = &self.matrix.factors[i];
            output[i] = throttle_out * f.throttle + rpy[i];
            output[i] = output[i].clamp(0.0, 1.0);
        }

        output
    }

    /// Simple mix without desaturation (for testing/debugging).
    pub fn mix_simple(&self, roll: f32, pitch: f32, yaw: f32, throttle: f32) -> [f32; MAX_MOTORS] {
        let mut output = [0.0f32; MAX_MOTORS];
        let n = self.matrix.motor_count();
        for i in 0..n {
            let f = &self.matrix.factors[i];
            output[i] = throttle * f.throttle + roll * f.roll + pitch * f.pitch + yaw * f.yaw;
            output[i] = output[i].clamp(0.0, 1.0);
        }
        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f32 = 1e-3;

    fn approx(a: f32, b: f32) -> bool {
        (a - b).abs() < TOL
    }

    #[test]
    fn test_quad_x_factors() {
        let mx = MixingMatrix::quad_x();
        assert_eq!(mx.motor_count(), 4);
        let f = &mx.factors;
        assert!(approx(f[0].roll, -0.5));
        assert!(approx(f[0].pitch, 0.5));
        assert!(approx(f[0].yaw, 0.5));
        assert!(approx(f[1].roll, 0.5));
        assert!(approx(f[1].pitch, -0.5));
        assert!(approx(f[1].yaw, 0.5));
        assert!(approx(f[2].roll, 0.5));
        assert!(approx(f[2].pitch, 0.5));
        assert!(approx(f[2].yaw, -0.5));
        assert!(approx(f[3].roll, -0.5));
        assert!(approx(f[3].pitch, -0.5));
        assert!(approx(f[3].yaw, -0.5));
    }

    #[test]
    fn test_quad_x_symmetry() {
        let mx = MixingMatrix::quad_x();
        let f = &mx.factors;
        assert!(approx(f[0].roll, f[3].roll));
        assert!(approx(f[1].roll, f[2].roll));
        assert!(approx(f[0].roll, -f[1].roll));
        assert!(approx(f[0].pitch, -f[1].pitch));
        assert!(approx(f[2].pitch, -f[3].pitch));
    }

    #[test]
    fn test_quad_plus_factors() {
        let mx = MixingMatrix::quad_plus();
        let f = &mx.factors;
        assert!(approx(f[0].roll, -0.5));
        assert!(approx(f[0].pitch, 0.0));
        assert!(approx(f[2].roll, 0.0));
        assert!(approx(f[2].pitch, 0.5));
    }

    #[test]
    fn test_hover_output() {
        let mut mixer = Mixer::new(MixingMatrix::quad_x());
        let out = mixer.mix(0.0, 0.0, 0.0, 0.5);
        for i in 0..4 {
            assert!(approx(out[i], 0.5));
        }
    }

    #[test]
    fn test_roll_right() {
        let mut mixer = Mixer::new(MixingMatrix::quad_x());
        let out = mixer.mix(0.3, 0.0, 0.0, 0.5);
        let mx = &mixer.matrix;
        for i in 0..4 {
            if mx.factors[i].roll > 0.0 {
                assert!(out[i] > 0.5, "Motor {} should increase for roll right", i);
            } else if mx.factors[i].roll < 0.0 {
                assert!(out[i] < 0.5, "Motor {} should decrease for roll right", i);
            }
        }
    }

    #[test]
    fn test_yaw_right() {
        let mut mixer = Mixer::new(MixingMatrix::quad_x());
        let out = mixer.mix(0.0, 0.0, 0.3, 0.5);
        let mx = &mixer.matrix;
        for i in 0..4 {
            if mx.factors[i].yaw > 0.0 {
                assert!(out[i] > 0.5, "Motor {} (yaw>0) should increase for yaw right", i);
            } else if mx.factors[i].yaw < 0.0 {
                assert!(out[i] < 0.5, "Motor {} (yaw<0) should decrease for yaw right", i);
            }
        }
    }

    #[test]
    fn test_output_bounded() {
        let mut mixer = Mixer::new(MixingMatrix::quad_x());
        let out = mixer.mix(1.0, 1.0, 1.0, 1.0);
        for i in 0..4 {
            assert!(out[i] >= 0.0 && out[i] <= 1.0, "Motor {} out of range: {}", i, out[i]);
        }
        let out2 = mixer.mix(-1.0, -1.0, -1.0, 0.0);
        for i in 0..4 {
            assert!(out2[i] >= 0.0 && out2[i] <= 1.0, "Motor {} out of range: {}", i, out2[i]);
        }
    }

    #[test]
    fn test_hex_x_count() {
        let mx = MixingMatrix::hex_x();
        assert_eq!(mx.motor_count(), 6);
    }

    #[test]
    fn test_octa_x_count() {
        let mx = MixingMatrix::octa_x();
        assert_eq!(mx.motor_count(), 8);
    }

    #[test]
    fn test_yaw_headroom() {
        let mut mixer = Mixer::new(MixingMatrix::quad_x());
        let out_ry = mixer.mix(0.8, 0.0, 0.8, 0.5);
        let out_r = mixer.mix(0.8, 0.0, 0.0, 0.5);
        for i in 0..4 {
            assert!(out_ry[i] >= 0.0 && out_ry[i] <= 1.0);
        }

        let mx = &mixer.matrix;
        let roll_diff_ry = {
            let (mut ps, mut ns) = (0.0f32, 0.0f32);
            let (mut pc, mut nc) = (0, 0);
            for i in 0..4 {
                if mx.factors[i].roll > 0.0 { ps += out_ry[i]; pc += 1; }
                else { ns += out_ry[i]; nc += 1; }
            }
            (ps / pc as f32) - (ns / nc as f32)
        };
        let roll_diff_r = {
            let (mut ps, mut ns) = (0.0f32, 0.0f32);
            let (mut pc, mut nc) = (0, 0);
            for i in 0..4 {
                if mx.factors[i].roll > 0.0 { ps += out_r[i]; pc += 1; }
                else { ns += out_r[i]; nc += 1; }
            }
            (ps / pc as f32) - (ns / nc as f32)
        };

        assert!(roll_diff_ry > roll_diff_r * 0.5,
            "Roll authority lost: roll+yaw diff={}, pure roll diff={}",
            roll_diff_ry, roll_diff_r);
    }

    #[test]
    fn test_rpy_proportional_scaling() {
        let mut mixer = Mixer::new(MixingMatrix::quad_x());
        let roll_cmd = 0.6;
        let pitch_cmd = 0.4;
        let out = mixer.mix(roll_cmd, pitch_cmd, 0.0, 0.5);

        let mx = &mixer.matrix;
        let mut roll_pos_sum = 0.0f32;
        let mut roll_neg_sum = 0.0f32;
        let mut roll_pos_cnt = 0;
        let mut roll_neg_cnt = 0;
        let mut pitch_pos_sum = 0.0f32;
        let mut pitch_neg_sum = 0.0f32;
        let mut pitch_pos_cnt = 0;
        let mut pitch_neg_cnt = 0;

        for i in 0..4 {
            if mx.factors[i].roll > 0.0 {
                roll_pos_sum += out[i]; roll_pos_cnt += 1;
            } else if mx.factors[i].roll < 0.0 {
                roll_neg_sum += out[i]; roll_neg_cnt += 1;
            }
            if mx.factors[i].pitch > 0.0 {
                pitch_pos_sum += out[i]; pitch_pos_cnt += 1;
            } else if mx.factors[i].pitch < 0.0 {
                pitch_neg_sum += out[i]; pitch_neg_cnt += 1;
            }
        }

        let eff_roll = (roll_pos_sum / roll_pos_cnt as f32) - (roll_neg_sum / roll_neg_cnt as f32);
        let eff_pitch = (pitch_pos_sum / pitch_pos_cnt as f32) - (pitch_neg_sum / pitch_neg_cnt as f32);

        let cmd_ratio = roll_cmd / pitch_cmd;
        let eff_ratio = eff_roll / eff_pitch;
        assert!((eff_ratio - cmd_ratio).abs() < cmd_ratio * 0.2,
            "RPY ratio not preserved: commanded={}, effective={}", cmd_ratio, eff_ratio);

        // Need a fresh borrow since mix() requires &mut self
        let out_extreme = mixer.mix(1.0, 1.0, 0.0, 0.5);
        {
            let mxe = &mixer.matrix;
            let mut re_pos = 0.0f32;
            let mut re_neg = 0.0f32;
            let mut pe_pos = 0.0f32;
            let mut pe_neg = 0.0f32;
            let (mut rpc, mut rnc, mut ppc, mut pnc) = (0, 0, 0, 0);
            for i in 0..4 {
                if mxe.factors[i].roll > 0.0 { re_pos += out_extreme[i]; rpc += 1; }
                else if mxe.factors[i].roll < 0.0 { re_neg += out_extreme[i]; rnc += 1; }
                if mxe.factors[i].pitch > 0.0 { pe_pos += out_extreme[i]; ppc += 1; }
                else if mxe.factors[i].pitch < 0.0 { pe_neg += out_extreme[i]; pnc += 1; }
            }
            let ext_roll = (re_pos / rpc as f32) - (re_neg / rnc as f32);
            let ext_pitch = (pe_pos / ppc as f32) - (pe_neg / pnc as f32);
            assert!((ext_roll - ext_pitch).abs() < 0.1,
                "Equal commands should produce equal differentials: roll={}, pitch={}", ext_roll, ext_pitch);
        }

        for i in 0..4 {
            assert!(out_extreme[i] >= 0.0 && out_extreme[i] <= 1.0);
        }
    }

    // --- Frame count tests ---

    #[test]
    fn test_all_frame_counts() {
        // Quad variants (14)
        assert_eq!(MixingMatrix::quad_x().motor_count(), 4);
        assert_eq!(MixingMatrix::quad_plus().motor_count(), 4);
        assert_eq!(MixingMatrix::quad_nyt_plus().motor_count(), 4);
        assert_eq!(MixingMatrix::quad_nyt_x().motor_count(), 4);
        assert_eq!(MixingMatrix::quad_bf_x().motor_count(), 4);
        assert_eq!(MixingMatrix::quad_bf_x_rev().motor_count(), 4);
        assert_eq!(MixingMatrix::quad_dji_x().motor_count(), 4);
        assert_eq!(MixingMatrix::quad_cw_x().motor_count(), 4);
        assert_eq!(MixingMatrix::quad_v().motor_count(), 4);
        assert_eq!(MixingMatrix::quad_h().motor_count(), 4);
        assert_eq!(MixingMatrix::quad_vtail().motor_count(), 4);
        assert_eq!(MixingMatrix::quad_atail().motor_count(), 4);
        assert_eq!(MixingMatrix::quad_plus_rev().motor_count(), 4);
        assert_eq!(MixingMatrix::quad_y4().motor_count(), 4);
        // Hex variants (5)
        assert_eq!(MixingMatrix::hex_x().motor_count(), 6);
        assert_eq!(MixingMatrix::hex_plus().motor_count(), 6);
        assert_eq!(MixingMatrix::hex_h().motor_count(), 6);
        assert_eq!(MixingMatrix::hex_dji_x().motor_count(), 6);
        assert_eq!(MixingMatrix::hex_cw_x().motor_count(), 6);
        // Octa variants (7)
        assert_eq!(MixingMatrix::octa_plus().motor_count(), 8);
        assert_eq!(MixingMatrix::octa_x().motor_count(), 8);
        assert_eq!(MixingMatrix::octa_v().motor_count(), 8);
        assert_eq!(MixingMatrix::octa_h().motor_count(), 8);
        assert_eq!(MixingMatrix::octa_i().motor_count(), 8);
        assert_eq!(MixingMatrix::octa_dji_x().motor_count(), 8);
        assert_eq!(MixingMatrix::octa_cw_x().motor_count(), 8);
        // OctaQuad variants (9)
        assert_eq!(MixingMatrix::octaquad_plus().motor_count(), 8);
        assert_eq!(MixingMatrix::octaquad_x().motor_count(), 8);
        assert_eq!(MixingMatrix::octaquad_v().motor_count(), 8);
        assert_eq!(MixingMatrix::octaquad_h().motor_count(), 8);
        assert_eq!(MixingMatrix::octaquad_cw_x().motor_count(), 8);
        assert_eq!(MixingMatrix::octaquad_bf_x().motor_count(), 8);
        assert_eq!(MixingMatrix::octaquad_bf_x_rev().motor_count(), 8);
        assert_eq!(MixingMatrix::octaquad_x_cor().motor_count(), 8);
        assert_eq!(MixingMatrix::octaquad_cw_x_cor().motor_count(), 8);
        // Y6 variants (3)
        assert_eq!(MixingMatrix::y6().motor_count(), 6);
        assert_eq!(MixingMatrix::y6b().motor_count(), 6);
        assert_eq!(MixingMatrix::y6f().motor_count(), 6);
        // DodecaHexa variants (2)
        assert_eq!(MixingMatrix::dodecahexa_plus().motor_count(), 12);
        assert_eq!(MixingMatrix::dodecahexa_x().motor_count(), 12);
        // Deca variants (2)
        assert_eq!(MixingMatrix::deca_plus().motor_count(), 10);
        assert_eq!(MixingMatrix::deca_x().motor_count(), 10);
    }

    // --- Thrust linearization tests ---

    #[test]
    fn test_thrust_curve_quadratic_inversion() {
        let expo = 0.65;
        for i in 0..=10 {
            let thrust = i as f32 / 10.0;
            let pwm = apply_thrust_curve(thrust, expo);
            let recovered = actuator_to_thrust(pwm, expo);
            assert!((recovered - thrust).abs() < 0.02,
                "Round-trip failed: thrust={}, pwm={}, recovered={}", thrust, pwm, recovered);
        }
    }

    #[test]
    fn test_thrust_curve_linear_at_zero_expo() {
        for i in 0..=10 {
            let t = i as f32 / 10.0;
            let pwm = apply_thrust_curve(t, 0.0);
            assert!(approx(pwm, t), "Linear at expo=0: t={}, pwm={}", t, pwm);
        }
    }

    #[test]
    fn test_actuator_to_thrust_inverse() {
        let expo = 0.65;
        let pwm = 0.7;
        let thrust = actuator_to_thrust(pwm, expo);
        assert!((thrust - 0.5635).abs() < 0.01);
    }

    // --- Voltage compensation tests ---

    #[test]
    fn test_voltage_comp_filter() {
        let mut vc = VoltageCompensation::new(16.8, 0.65);
        let lm = vc.update(16.8, 0.01);
        assert!((lm - 1.0).abs() < 0.05, "Nominal voltage -> lift_max~1.0, got {}", lm);

        let lm1 = vc.update(14.0, 0.01);
        let _ = lm1;
        let lm2 = vc.update(14.0, 0.01);
        assert!(lm2 < lm, "lift_max should decrease with low voltage");
    }

    // --- Limit flags tests ---

    #[test]
    fn test_limit_flags_throttle_upper() {
        let mut mixer = Mixer::new(MixingMatrix::quad_x());
        mixer.mix(0.0, 0.0, 0.0, 1.0);
        assert!(mixer.limits.throttle_upper, "Should hit throttle upper at full throttle");
    }

    #[test]
    fn test_limit_flags_rpy() {
        let mut mixer = Mixer::new(MixingMatrix::quad_x());
        mixer.mix(1.0, 1.0, 1.0, 0.5);
        assert!(mixer.limits.rpy || mixer.limits.yaw_limited,
            "Should hit RPY or yaw limit with extreme commands");
    }

    // --- Yaw headroom configuration test ---

    #[test]
    fn test_yaw_headroom_configurable() {
        let config_no = MixerConfig { yaw_headroom: 0, ..MixerConfig::default() };
        let mut mixer_nh = Mixer::with_config(MixingMatrix::quad_x(), config_no);

        let config_high = MixerConfig { yaw_headroom: 500, ..MixerConfig::default() };
        let mut mixer_hh = Mixer::with_config(MixingMatrix::quad_x(), config_high);

        let out_nh = mixer_nh.mix(0.9, 0.0, 0.5, 0.5);
        let out_hh = mixer_hh.mix(0.9, 0.0, 0.5, 0.5);

        for i in 0..4 {
            assert!(out_nh[i] >= 0.0 && out_nh[i] <= 1.0);
            assert!(out_hh[i] >= 0.0 && out_hh[i] <= 1.0);
        }
    }

    #[test]
    fn test_dshot_zero_throttle_constant() {
        assert_eq!(DSHOT_ZERO_THROTTLE, 48);
    }

    #[test]
    fn test_compensation_gain_affects_output() {
        let mut mixer = Mixer::new(MixingMatrix::quad_x());
        let out_normal = mixer.mix_compensated(0.3, 0.0, 0.0, 0.5, 1.0, 0.5);
        let out_comp = mixer.mix_compensated(0.3, 0.0, 0.0, 0.5, 1.15, 0.5);
        let diff_normal = (out_normal[1] - out_normal[0]).abs();
        let diff_comp = (out_comp[1] - out_comp[0]).abs();
        assert!(diff_comp > diff_normal,
            "Compensation gain should increase differential: normal={}, comp={}", diff_normal, diff_comp);
    }

    #[test]
    fn test_vtail_asymmetric_yaw() {
        let mx = MixingMatrix::quad_vtail();
        let f = &mx.factors;
        assert!(libm::fabsf(f[0].yaw) < 0.01);
        assert!(libm::fabsf(f[2].yaw) < 0.01);
        assert!(libm::fabsf(f[1].yaw) > 0.1);
        assert!(libm::fabsf(f[3].yaw) > 0.1);
    }

    // --- Motor test module ---

    #[test]
    fn test_motor_test_basic() {
        let mut mt = motor_test::MotorTest::new();
        assert!(!mt.is_active());
        mt.start(0, motor_test::ThrottleType::Percent, 0.5, 2.0);
        assert!(mt.is_active());
        let out = mt.get_output(4);
        assert!(out[0] > 0.4);
        for i in 1..4 { assert!(out[i] < 0.01); }
        mt.update(2.5);
        assert!(!mt.is_active());
    }
}

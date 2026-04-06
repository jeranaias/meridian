//! EKF state vector and covariance matrix.
//!
//! 24-state vector matching ArduPilot's AP_NavEKF3.
//! Source: AP_NavEKF3_core.h lines 574-583

use meridian_math::{Vec3, Quaternion};
use meridian_math::frames::{NED, Body};
use meridian_math::geodetic::LatLonAlt;

/// Number of states in the EKF.
pub const NUM_STATES: usize = 24;

/// The EKF state vector.
#[derive(Debug, Clone)]
pub struct StateVector {
    /// Attitude quaternion (NED-to-body, i.e. nav-to-body). States 0-3.
    /// Convention matches ArduPilot: `stateStruct.quat` rotates from local NED
    /// to body frame.  `to_dcm()` returns the body-to-NED rotation matrix
    /// (transpose of the nav-to-body quaternion direction) because Meridian's
    /// `Rotation::from_quaternion` applies the inverse convention. All downstream
    /// math is consistent.
    pub quat: Quaternion,
    /// NED velocity (m/s). States 4-6.
    pub velocity: Vec3<NED>,
    /// NED position relative to origin (m). States 7-9.
    pub position: Vec3<NED>,
    /// Gyro bias (rad/s). States 10-12.
    pub gyro_bias: Vec3<Body>,
    /// Accelerometer bias (m/s²). States 13-15.
    pub accel_bias: Vec3<Body>,
    /// Earth magnetic field NED (gauss). States 16-18.
    pub earth_mag: Vec3<NED>,
    /// Body magnetic field (gauss). States 19-21.
    pub body_mag: Vec3<Body>,
    /// Wind velocity NE (m/s). States 22-23. (unused for multirotor hover)
    pub wind: [f32; 2],
}

impl StateVector {
    /// Create initial state: level, stationary, at origin.
    pub fn new() -> Self {
        Self {
            quat: Quaternion::identity(),
            velocity: Vec3::zero(),
            position: Vec3::zero(),
            gyro_bias: Vec3::zero(),
            accel_bias: Vec3::zero(),
            // Typical mid-latitude earth field (will be overridden by mag fusion)
            earth_mag: Vec3::new(0.22, 0.005, 0.42),
            body_mag: Vec3::zero(),
            wind: [0.0; 2],
        }
    }

    /// Write state to a flat array (for covariance operations).
    pub fn to_array(&self) -> [f32; NUM_STATES] {
        [
            self.quat.w, self.quat.x, self.quat.y, self.quat.z,
            self.velocity.x, self.velocity.y, self.velocity.z,
            self.position.x, self.position.y, self.position.z,
            self.gyro_bias.x, self.gyro_bias.y, self.gyro_bias.z,
            self.accel_bias.x, self.accel_bias.y, self.accel_bias.z,
            self.earth_mag.x, self.earth_mag.y, self.earth_mag.z,
            self.body_mag.x, self.body_mag.y, self.body_mag.z,
            self.wind[0], self.wind[1],
        ]
    }

    /// Constrain states to physical bounds.
    /// Source: ArduPilot AP_NavEKF3_core.cpp ConstrainStates()
    /// Prevents bias states from growing unbounded.
    pub fn constrain_states(&mut self) {
        // Quaternion: renormalize
        self.quat = self.quat.normalized();

        // Velocity: ±500 m/s (supersonic = something is very wrong)
        self.velocity.x = self.velocity.x.clamp(-500.0, 500.0);
        self.velocity.y = self.velocity.y.clamp(-500.0, 500.0);
        self.velocity.z = self.velocity.z.clamp(-500.0, 500.0);

        // Position: ±1e6 m (1000 km from origin)
        self.position.x = self.position.x.clamp(-1.0e6, 1.0e6);
        self.position.y = self.position.y.clamp(-1.0e6, 1.0e6);
        self.position.z = self.position.z.clamp(-1.0e5, 1.0e5); // ±100km altitude

        // Gyro bias: ±0.5 rad/s (~29 deg/s — beyond this the sensor is dead)
        self.gyro_bias.x = self.gyro_bias.x.clamp(-0.5, 0.5);
        self.gyro_bias.y = self.gyro_bias.y.clamp(-0.5, 0.5);
        self.gyro_bias.z = self.gyro_bias.z.clamp(-0.5, 0.5);

        // Accel bias: ±5.0 m/s² (half of gravity — sensor needs recal beyond this)
        self.accel_bias.x = self.accel_bias.x.clamp(-5.0, 5.0);
        self.accel_bias.y = self.accel_bias.y.clamp(-5.0, 5.0);
        self.accel_bias.z = self.accel_bias.z.clamp(-5.0, 5.0);

        // Earth mag: ±1.0 gauss
        self.earth_mag.x = self.earth_mag.x.clamp(-1.0, 1.0);
        self.earth_mag.y = self.earth_mag.y.clamp(-1.0, 1.0);
        self.earth_mag.z = self.earth_mag.z.clamp(-1.0, 1.0);

        // Body mag: ±0.5 gauss
        self.body_mag.x = self.body_mag.x.clamp(-0.5, 0.5);
        self.body_mag.y = self.body_mag.y.clamp(-0.5, 0.5);
        self.body_mag.z = self.body_mag.z.clamp(-0.5, 0.5);

        // Wind: ±100 m/s
        self.wind[0] = self.wind[0].clamp(-100.0, 100.0);
        self.wind[1] = self.wind[1].clamp(-100.0, 100.0);
    }

    /// Read state from a flat array.
    pub fn from_array(a: &[f32; NUM_STATES]) -> Self {
        Self {
            quat: Quaternion::new(a[0], a[1], a[2], a[3]).normalized(),
            velocity: Vec3::new(a[4], a[5], a[6]),
            position: Vec3::new(a[7], a[8], a[9]),
            gyro_bias: Vec3::new(a[10], a[11], a[12]),
            accel_bias: Vec3::new(a[13], a[14], a[15]),
            earth_mag: Vec3::new(a[16], a[17], a[18]),
            body_mag: Vec3::new(a[19], a[20], a[21]),
            wind: [a[22], a[23]],
        }
    }
}

/// 24x24 covariance matrix (symmetric, stored full for simplicity).
/// ~2.3KB — well within embedded SRAM budgets.
#[derive(Clone)]
pub struct Covariance {
    pub p: [[f32; NUM_STATES]; NUM_STATES],
}

impl Covariance {
    /// Zero covariance.
    pub fn zero() -> Self {
        Self { p: [[0.0; NUM_STATES]; NUM_STATES] }
    }

    /// Initialize with reasonable uncertainty for cold start.
    pub fn initial() -> Self {
        let mut c = Self::zero();
        // Attitude uncertainty: ~10° = 0.17 rad → variance 0.03
        for i in 0..4 { c.p[i][i] = 0.03; }
        // Velocity uncertainty: sq(VELNE_M_NSE_DEFAULT) = sq(0.3) = 0.09
        // Source: ArduPilot ResetVelocity uses sq(frontend->_gpsHorizVelNoise)
        for i in 4..7 { c.p[i][i] = 0.09; }
        // Position uncertainty: 3 m (tighter init — GPS available from start)
        for i in 7..10 { c.p[i][i] = 9.0; }
        // Gyro bias: 0.01 rad/s
        for i in 10..13 { c.p[i][i] = 1e-4; }
        // Accel bias: 0.1 m/s²
        for i in 13..16 { c.p[i][i] = 0.01; }
        // Earth mag: 0.1 gauss
        for i in 16..19 { c.p[i][i] = 0.01; }
        // Body mag: 0.1 gauss
        for i in 19..22 { c.p[i][i] = 0.01; }
        // Wind: 5 m/s
        for i in 22..24 { c.p[i][i] = 25.0; }
        c
    }

    /// Enforce symmetry: P[i][j] = P[j][i] = average.
    pub fn force_symmetry(&mut self) {
        for i in 0..NUM_STATES {
            for j in 0..i {
                let avg = 0.5 * (self.p[i][j] + self.p[j][i]);
                self.p[i][j] = avg;
                self.p[j][i] = avg;
            }
        }
    }

    /// Constrain diagonal (variance) elements to valid range.
    /// Source: ArduPilot constrains position variance to 1e4 (100m²).
    pub fn constrain_variances(&mut self) {
        // Maximum allowed variance per state group
        const MAX_QUAT_VAR: f32 = 1.0;
        const MAX_VEL_VAR: f32 = 1e4;    // (100 m/s)²
        const MAX_POS_VAR: f32 = 1e6;    // (1000 m)²
        const MAX_BIAS_VAR: f32 = 1.0;
        const MAX_MAG_VAR: f32 = 1.0;
        const MAX_WIND_VAR: f32 = 1e4;

        for i in 0..NUM_STATES {
            // Floor: non-negative
            if self.p[i][i] < 0.0 { self.p[i][i] = 0.0; }
            // Ceiling: prevent f32 overflow
            let max = match i {
                0..=3 => MAX_QUAT_VAR,
                4..=6 => MAX_VEL_VAR,
                7..=9 => MAX_POS_VAR,
                10..=15 => MAX_BIAS_VAR,
                16..=21 => MAX_MAG_VAR,
                _ => MAX_WIND_VAR,
            };
            if self.p[i][i] > max { self.p[i][i] = max; }
        }
    }
}

impl core::fmt::Debug for Covariance {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "Covariance {{ diag: [")?;
        for i in 0..NUM_STATES {
            if i > 0 { write!(f, ", ")?; }
            write!(f, "{:.4}", self.p[i][i])?;
        }
        write!(f, "] }}")
    }
}

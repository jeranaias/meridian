//! EKF parameters — every value traced to ArduPilot EK3_* defaults.
//! Source: libraries/AP_NavEKF3/AP_NavEKF3.cpp lines 23-130

/// All tunable EKF parameters.
#[derive(Debug, Clone)]
pub struct EkfParams {
    // ─── Sensor noise (observation variance) ───
    /// GPS horizontal position noise (m). EK3_POSNE_NOISE. Source: line 25
    pub gps_pos_noise: f32,
    /// GPS horizontal velocity noise (m/s). EK3_VELNE_NOISE. Source: line 23
    pub gps_vel_noise: f32,
    /// GPS vertical velocity noise (m/s). EK3_VELD_M_NSE. Source: line 31
    pub gps_vert_vel_noise: f32,
    /// Barometer altitude noise (m). EK3_ALT_M_NSE. Source: line 26
    pub baro_alt_noise: f32,
    /// Magnetometer noise (gauss). EK3_MAG_M_NSE. Source: line 27
    pub mag_noise: f32,

    // ─── IMU process noise ───
    /// Gyro noise (rad/s). EK3_GYRO_P_NSE. Source: line 28
    pub gyro_noise: f32,
    /// Accelerometer noise (m/s²). EK3_ACC_P_NSE. Source: line 29
    pub accel_noise: f32,

    // ─── Bias process noise ───
    /// Gyro bias process noise (rad/s²). EK3_GBIAS_P_NSE
    pub gyro_bias_process_noise: f32,
    /// Accel bias process noise (m/s³). EK3_ABIAS_P_NSE
    pub accel_bias_process_noise: f32,

    // ─── Magnetic field process noise ───
    /// Earth mag field process noise (gauss/s). EK3_MAGE_P_NSE
    pub mag_earth_process_noise: f32,
    /// Body mag field process noise (gauss/s). EK3_MAGB_P_NSE
    pub mag_body_process_noise: f32,

    // ─── Wind process noise ───
    /// Wind velocity process noise (m/s/sqrt(s)). EK3_WIND_P_NSE. Copter default 0.2
    pub wind_process_noise: f32,

    // ─── Sensor delays ───
    /// GPS measurement delay (ms). EK3_GPS_DELAY. Source: line 73
    pub gps_delay_ms: u16,

    // ─── Innovation gates (in hundredths of sigma) ───
    /// GPS velocity innovation gate. EK3_VEL_I_GATE. Default: 500 (=5 sigma)
    pub vel_innov_gate: u16,
    /// GPS position innovation gate. EK3_POS_I_GATE. Default: 500 (=5 sigma)
    pub pos_innov_gate: u16,
    /// Height innovation gate. EK3_HGT_I_GATE. Default: 500 (=5 sigma)
    pub hgt_innov_gate: u16,
    /// Magnetometer innovation gate. EK3_MAG_I_GATE. Default: 300 (=3 sigma)
    pub mag_innov_gate: u16,

    // ─── Output predictor ───
    /// Output predictor complementary filter frequency (Hz)
    pub hrt_filt_freq: f32,
    /// Output velocity/position tracking time constant (s)
    pub tau_vel_pos_output: f32,

    // ─── Mag variance rate scaling ───
    /// Magnetometer observation variance scaling with angular rate.
    /// Source: AP_NavEKF3_core.h magVarRateScale. Default: 0.05
    pub mag_var_rate_scale: f32,

    // ─── GPS velocity variance scaling with acceleration ───
    /// Scaling factor for GPS velocity noise inflation during high-g maneuvers.
    /// Source: AP_NavEKF3_PosVelFusion.cpp accel_scale. Default: 0.2
    pub gps_vel_accel_scale: f32,

    // ─── GPS antenna offset (body frame, meters) ───
    /// Lever arm from IMU to GPS antenna in body frame.
    /// Source: EK3_GPS_POS_X/Y/Z
    pub gps_antenna_offset: [f32; 3],

    // ─── Ground effect compensation ───
    /// Baro noise multiplier when in ground effect (altitude < gnd_effect_alt).
    pub gnd_effect_baro_scale: f32,
    /// Altitude below which ground effect compensation is active (m AGL).
    pub gnd_effect_alt: f32,

    // ─── Height source ───
    /// Primary height source: 0=Baro, 1=GPS, 2=Rangefinder, 3=Beacon, 4=ExtNav
    pub height_source: u8,
    /// Rangefinder maximum usable range (m).
    pub rng_max_range: f32,
    /// Rangefinder noise (m). Default: 0.5
    pub rng_noise: f32,
}

impl Default for EkfParams {
    fn default() -> Self {
        Self {
            gps_pos_noise: 0.5,
            gps_vel_noise: 0.3,
            gps_vert_vel_noise: 0.5,
            baro_alt_noise: 2.0,
            mag_noise: 0.05,
            gyro_noise: 0.015,
            accel_noise: 0.35,
            gyro_bias_process_noise: 1.0e-3,   // EK3_GBIAS_P_NSE copter default (NOT 7e-6, that's too small)
            accel_bias_process_noise: 2.0e-2,   // EK3_ABIAS_P_NSE copter default
            mag_earth_process_noise: 1.0e-3,    // EK3_MAGE_P_NSE copter default
            mag_body_process_noise: 1.0e-4,     // EK3_MAGB_P_NSE copter default
            wind_process_noise: 0.2,            // EK3_WIND_P_NSE copter default
            gps_delay_ms: 220,
            vel_innov_gate: 500,
            pos_innov_gate: 500,
            hgt_innov_gate: 500,
            mag_innov_gate: 300,                // EK3_MAG_I_GATE copter default 3 sigma
            hrt_filt_freq: 2.0,
            tau_vel_pos_output: 0.25,
            mag_var_rate_scale: 0.05,
            gps_vel_accel_scale: 0.2,
            gps_antenna_offset: [0.0; 3],
            gnd_effect_baro_scale: 4.0,
            gnd_effect_alt: 2.0,
            height_source: 0, // Baro
            rng_max_range: 30.0,
            rng_noise: 0.5,
        }
    }
}

impl EkfParams {
    /// Convert innovation gate from hundredths to sigma multiplier.
    #[inline]
    pub fn gate_sigma(&self, gate_hundredths: u16) -> f32 {
        (gate_hundredths as f32 * 0.01).max(1.0)
    }
}

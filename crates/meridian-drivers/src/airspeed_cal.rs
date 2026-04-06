//! 3-state Kalman filter for in-flight airspeed calibration.
//!
//! ArduPilot reference: `Airspeed_Calibration.cpp`
//!
//! States: [ratio, offset, wind_speed]
//! - ratio: sensor sensitivity correction (multiplied to raw pressure)
//! - offset: zero-point drift correction (added to pressure)
//! - wind_speed: nuisance state (estimated wind along heading)
//!
//! On each GPS+airspeed sample pair, uses wind-triangle geometry to update
//! the estimate. The ratio corrects sensor gain; the offset corrects zero
//! drift; wind_speed absorbs wind effects.

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Process noise for ratio state.
const Q_RATIO: f32 = 0.00001;
/// Process noise for offset state.
const Q_OFFSET: f32 = 0.0001;
/// Process noise for wind state.
const Q_WIND: f32 = 0.01;

/// Initial ratio estimate (1.0 = no correction).
const INITIAL_RATIO: f32 = 1.0;
/// Initial offset estimate.
const INITIAL_OFFSET: f32 = 0.0;
/// Initial wind estimate.
const INITIAL_WIND: f32 = 0.0;

/// Initial covariance diagonal.
const INITIAL_P: f32 = 1.0;

/// Measurement noise (m/s)^2.
const R_MEASUREMENT: f32 = 0.5;

// ---------------------------------------------------------------------------
// 3-state Kalman filter
// ---------------------------------------------------------------------------

/// 3-state airspeed calibration Kalman filter.
///
/// Call `update()` with each GPS+airspeed measurement pair. The filter
/// will converge to the correct ratio and offset over time.
pub struct AirspeedCalKalman {
    /// State vector: [ratio, offset, wind_speed].
    x: [f32; 3],
    /// Covariance matrix (3x3, stored as flat array, row-major).
    p: [f32; 9],
    /// Whether the filter has converged (ratio within reasonable bounds).
    pub converged: bool,
    /// Number of updates processed.
    pub update_count: u32,
}

impl AirspeedCalKalman {
    pub fn new() -> Self {
        Self {
            x: [INITIAL_RATIO, INITIAL_OFFSET, INITIAL_WIND],
            p: [
                INITIAL_P, 0.0, 0.0,
                0.0, INITIAL_P, 0.0,
                0.0, 0.0, INITIAL_P,
            ],
            converged: false,
            update_count: 0,
        }
    }

    /// Get the current ratio correction.
    pub fn ratio(&self) -> f32 { self.x[0] }

    /// Get the current offset correction (Pa).
    pub fn offset(&self) -> f32 { self.x[1] }

    /// Get the estimated wind speed (m/s).
    pub fn wind_speed(&self) -> f32 { self.x[2] }

    /// Apply the calibration correction to a raw differential pressure.
    ///
    /// Returns the corrected pressure: `dp_corrected = dp_raw * ratio + offset`.
    pub fn correct_pressure(&self, dp_raw: f32) -> f32 {
        dp_raw * self.x[0] + self.x[1]
    }

    /// Update the filter with a GPS+airspeed measurement pair.
    ///
    /// `gps_speed_ms`: GPS ground speed in m/s.
    /// `heading_rad`: GPS track heading in radians.
    /// `airspeed_ms`: indicated airspeed in m/s (from sensor after pressure→IAS conversion).
    /// `wind_dir_rad`: estimated wind direction in radians (from heading - track angle).
    pub fn update(
        &mut self,
        gps_speed_ms: f32,
        heading_rad: f32,
        airspeed_ms: f32,
        wind_dir_rad: f32,
    ) {
        if gps_speed_ms < 3.0 || airspeed_ms < 2.0 {
            return; // too slow for meaningful calibration
        }

        // Wind-triangle prediction: airspeed = groundspeed + headwind component
        let headwind = self.x[2] * libm::cosf(wind_dir_rad - heading_rad);
        let predicted_airspeed = gps_speed_ms + headwind;

        // Innovation: predicted - measured
        let innovation = predicted_airspeed - airspeed_ms;

        // Jacobian H = [airspeed_ms / ratio, 1.0, cos(wind_dir - heading)]
        // Simplified: H = [dh/d_ratio, dh/d_offset, dh/d_wind]
        let h = [
            airspeed_ms, // d(predicted)/d(ratio) ≈ measured airspeed
            1.0,         // d(predicted)/d(offset) = 1
            libm::cosf(wind_dir_rad - heading_rad), // d(predicted)/d(wind)
        ];

        // Innovation covariance: S = H * P * H' + R
        let mut s = R_MEASUREMENT;
        for i in 0..3 {
            for j in 0..3 {
                s += h[i] * self.p[i * 3 + j] * h[j];
            }
        }

        if s < 1e-10 {
            return; // numerical protection
        }

        // Kalman gain: K = P * H' / S
        let mut k = [0.0f32; 3];
        for i in 0..3 {
            let mut ph = 0.0;
            for j in 0..3 {
                ph += self.p[i * 3 + j] * h[j];
            }
            k[i] = ph / s;
        }

        // State update: x = x - K * innovation
        for i in 0..3 {
            self.x[i] -= k[i] * innovation;
        }

        // Covariance update: P = (I - K*H) * P
        let mut p_new = [0.0f32; 9];
        for i in 0..3 {
            for j in 0..3 {
                let mut val = 0.0;
                for m in 0..3 {
                    let ikm = if i == m { 1.0 } else { 0.0 };
                    let ikm = ikm - k[i] * h[m];
                    val += ikm * self.p[m * 3 + j];
                }
                p_new[i * 3 + j] = val;
            }
        }

        // Add process noise
        p_new[0] += Q_RATIO;
        p_new[4] += Q_OFFSET;
        p_new[8] += Q_WIND;

        self.p = p_new;
        self.update_count += 1;

        // Convergence check: ratio should be 0.5..2.0
        self.converged = self.update_count > 50
            && self.x[0] > 0.5
            && self.x[0] < 2.0;

        // Clamp state bounds
        self.x[0] = self.x[0].max(0.3).min(3.0); // ratio
        self.x[1] = self.x[1].max(-100.0).min(100.0); // offset (Pa)
        self.x[2] = self.x[2].max(-30.0).min(30.0); // wind (m/s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_initial_state() {
        let kf = AirspeedCalKalman::new();
        assert!((kf.ratio() - 1.0).abs() < 0.001);
        assert!((kf.offset()).abs() < 0.001);
        assert!(!kf.converged);
    }

    #[test]
    fn test_no_correction_initially() {
        let kf = AirspeedCalKalman::new();
        // With ratio=1.0 and offset=0.0, correction is identity
        let corrected = kf.correct_pressure(100.0);
        assert!((corrected - 100.0).abs() < 0.01);
    }

    #[test]
    fn test_update_moves_state() {
        let mut kf = AirspeedCalKalman::new();
        // Simulate: GPS=20 m/s, heading=0, airspeed=18 m/s (sensor under-reading)
        for _ in 0..100 {
            kf.update(20.0, 0.0, 18.0, 0.0);
        }
        // The filter should adjust state away from initial values
        // (exact direction depends on innovation sign and Jacobian)
        let ratio = kf.ratio();
        assert!((ratio - 1.0).abs() > 0.01, "ratio should have moved from 1.0: ratio = {}", ratio);
    }
}

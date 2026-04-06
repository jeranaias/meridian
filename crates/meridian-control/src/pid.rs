//! PID controller with D-term filtering and integrator anti-windup.
//!
//! Matches ArduPilot's AC_PID. Source: libraries/AC_PID/AC_PID.cpp

use crate::notch_filter::{NotchFilter, LowPassFilter2p};

/// PID controller configuration.
/// Source: AC_PID.h — three independent LP filters + slew limiter.
#[derive(Debug, Clone, Copy)]
pub struct PidGains {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub ff: f32,        // feedforward
    pub kd_ff: f32,     // derivative feedforward on target (D_FF / kdff)
    pub imax: f32,      // integrator limit (absolute value)
    pub filt_hz: f32,   // D-term filter frequency (Hz) — _filt_D_hz
    pub filt_t_hz: f32, // Target filter frequency (Hz) — _filt_T_hz
    pub filt_e_hz: f32, // Error filter frequency (Hz) — _filt_E_hz
    pub smax: f32,      // Slew rate limit (output units/s). 0 = disabled. Source: ATC_RAT_*_SMAX
    pub pd_max: f32,    // PD sum maximum (PDMX). 0 = disabled.
    pub sr_tau: f32,    // Slew rate time constant for gain relaxation (s). Default 1.0.
}

/// PID telemetry / logging struct matching ArduPilot's AP_PIDInfo.
/// Source: AC_PID.h get_pid_info()
#[derive(Debug, Clone, Copy, Default)]
pub struct PidInfo {
    pub target: f32,
    pub actual: f32,
    pub error: f32,
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub ff: f32,
    pub d_ff: f32,
    pub d_mod: f32,     // Slew rate modifier (0.1 .. 1.0)
    pub slew_rate: f32,
    pub limit: bool,
    pub pd_limit: bool,
    pub reset: bool,
    pub i_term_set: bool,
}

/// Slew limiter state — adaptive gain-reduction per ArduPilot SlewLimiter.
/// Source: libraries/AC_PID/AC_PID.cpp SlewLimiter
#[derive(Debug, Clone)]
struct SlewLimiterState {
    /// Low-pass filtered slew rate (output units/s), filtered at 25 Hz.
    slew_rate_filtered: f32,
    /// Peak positive slew rate over window.
    peak_pos: f32,
    /// Peak negative slew rate over window.
    peak_neg: f32,
    /// Time since last positive exceedance (s).
    decay_pos: f32,
    /// Time since last negative exceedance (s).
    decay_neg: f32,
    /// Last P+D sum for computing slew rate.
    last_pd: f32,
}

impl SlewLimiterState {
    fn new() -> Self {
        Self {
            slew_rate_filtered: 0.0,
            peak_pos: 0.0,
            peak_neg: 0.0,
            decay_pos: 0.0,
            decay_neg: 0.0,
            last_pd: 0.0,
        }
    }

    fn reset(&mut self) {
        *self = Self::new();
    }
}

/// PID controller state.
/// Source: AC_PID.cpp with three independent LP filters and adaptive slew limiter.
#[derive(Debug, Clone)]
pub struct PidController {
    pub gains: PidGains,
    integrator: f32,
    prev_error: f32,
    prev_derivative: f32,
    prev_target: f32,
    filtered_target: f32,
    filtered_error: f32,
    prev_output: f32,
    initialized: bool,
    /// Adaptive slew limiter state.
    slew: SlewLimiterState,
    /// Last computed PidInfo for logging.
    pid_info: PidInfo,
    /// Per-PID notch filter on target signal.
    notch_target: Option<NotchFilter>,
    /// Per-PID notch filter on error signal.
    notch_error: Option<NotchFilter>,
    /// Optional 2nd-order Butterworth D-term low-pass filter.
    /// When set, replaces the 1st-order IIR D-term filter for better attenuation.
    /// Source: ArduPilot LowPassFilter2p on D-term.
    d_lpf2p: Option<LowPassFilter2p>,
    /// Flag indicating integrator was explicitly set (for logging).
    i_term_set: bool,
    /// Flag indicating controller was just reset (for logging).
    was_reset: bool,
}

impl PidController {
    pub fn new(gains: PidGains) -> Self {
        Self {
            gains,
            integrator: 0.0,
            prev_error: 0.0,
            prev_derivative: 0.0,
            prev_target: 0.0,
            filtered_target: 0.0,
            filtered_error: 0.0,
            prev_output: 0.0,
            initialized: false,
            slew: SlewLimiterState::new(),
            pid_info: PidInfo::default(),
            notch_target: None,
            notch_error: None,
            d_lpf2p: None,
            i_term_set: false,
            was_reset: false,
        }
    }

    /// Low-pass filter helper: first-order IIR.
    fn lp_filter(prev: f32, input: f32, freq_hz: f32, dt: f32) -> f32 {
        if freq_hz <= 0.0 || dt <= 0.0 { return input; }
        let rc = 1.0 / (2.0 * core::f32::consts::PI * freq_hz);
        let alpha = dt / (dt + rc);
        prev + alpha * (input - prev)
    }

    /// Compute the adaptive SMAX modifier (gain reduction multiplier).
    /// Returns a value in [0.1, 1.0].
    /// Source: AC_PID::SlewLimiter
    fn compute_slew_modifier(&mut self, p_term: f32, d_term: f32, dt: f32) -> f32 {
        let smax = self.gains.smax;
        if smax <= 0.0 || !self.initialized {
            self.slew.last_pd = p_term + d_term;
            return 1.0;
        }

        let pd = p_term + d_term;
        // Compute raw slew rate
        let raw_slew = (pd - self.slew.last_pd) / dt;
        self.slew.last_pd = pd;

        // Low-pass filter slew rate at 25 Hz
        self.slew.slew_rate_filtered = Self::lp_filter(
            self.slew.slew_rate_filtered, raw_slew, 25.0, dt,
        );

        let slew_rate = self.slew.slew_rate_filtered;

        // Track peak positive slew rate with decay
        let tau = if self.gains.sr_tau > 0.0 { self.gains.sr_tau } else { 1.0 };
        let window = 0.3; // 300 ms window for peak detection

        if slew_rate > self.slew.peak_pos {
            self.slew.peak_pos = slew_rate;
            self.slew.decay_pos = 0.0;
        } else {
            self.slew.decay_pos += dt;
            if self.slew.decay_pos > window {
                // Decay peak toward zero with time constant tau
                let decay = (-dt / tau).min(0.0);
                self.slew.peak_pos *= libm::expf(decay);
            }
        }

        // Track peak negative slew rate with decay
        if slew_rate < self.slew.peak_neg {
            self.slew.peak_neg = slew_rate;
            self.slew.decay_neg = 0.0;
        } else {
            self.slew.decay_neg += dt;
            if self.slew.decay_neg > window {
                let decay = (-dt / tau).min(0.0);
                self.slew.peak_neg *= libm::expf(decay);
            }
        }

        // Use the larger absolute peak slew
        let peak_slew = libm::fabsf(self.slew.peak_pos).max(libm::fabsf(self.slew.peak_neg));

        // Compute modifier: mod = smax / (smax + 1.5 * (peak_slew - smax))
        if peak_slew <= smax {
            1.0
        } else {
            let modifier = smax / (smax + 1.5 * (peak_slew - smax));
            // Floor at 0.1 per ArduPilot
            modifier.max(0.1)
        }
    }

    /// Full PID update with three independent LP filters and adaptive slew rate limiter.
    /// Source: AC_PID::update_all()
    ///
    /// `target`: desired value
    /// `actual`: measured value
    /// `dt`: timestep (seconds)
    /// `limit`: true when motor mixer reports saturation (stops integrator growth)
    /// `i_scale`: integrator scale factor from outer loop (1.0 = normal)
    pub fn update_full(&mut self, target: f32, actual: f32, dt: f32, limit: bool, i_scale: f32) -> f32 {
        if dt < 1e-6 { return 0.0; }

        let was_reset = !self.initialized;

        // Apply notch filter on target before LP filter
        let notched_target = if let Some(ref mut nf) = self.notch_target {
            nf.apply(target)
        } else {
            target
        };

        // Filter target (filt_T_hz)
        self.filtered_target = if self.initialized {
            Self::lp_filter(self.filtered_target, notched_target, self.gains.filt_t_hz, dt)
        } else {
            notched_target
        };

        // D_FF: derivative feedforward on filtered target
        let d_ff_term = if self.gains.kd_ff > 0.0 && self.initialized {
            let target_deriv = (self.filtered_target - self.prev_target) / dt;
            self.gains.kd_ff * target_deriv
        } else {
            0.0
        };

        // Compute error from filtered target
        let raw_error = self.filtered_target - actual;

        // Apply notch filter on error before LP filter
        let notched_error = if let Some(ref mut nf) = self.notch_error {
            nf.apply(raw_error)
        } else {
            raw_error
        };

        // Filter error (filt_E_hz)
        self.filtered_error = if self.initialized {
            Self::lp_filter(self.filtered_error, notched_error, self.gains.filt_e_hz, dt)
        } else {
            notched_error
        };

        // P term on filtered error
        let p_term = self.gains.kp * self.filtered_error;

        // D term with LP filter (filt_D_hz) — on error derivative.
        // Uses 2nd-order Butterworth (LowPassFilter2p) if configured, else 1st-order IIR.
        let d_term = if self.gains.kd > 0.0 && self.initialized {
            let derivative = (self.filtered_error - self.prev_error) / dt;
            let filtered = if let Some(ref mut lpf) = self.d_lpf2p {
                lpf.apply(derivative)
            } else {
                Self::lp_filter(self.prev_derivative, derivative, self.gains.filt_hz, dt)
            };
            self.prev_derivative = filtered;
            self.gains.kd * filtered
        } else {
            self.prev_derivative = 0.0;
            0.0
        };

        // Adaptive SMAX slew limiter — compute gain modifier
        let d_mod = self.compute_slew_modifier(p_term, d_term, dt);
        let p_out = p_term * d_mod;
        let d_out = d_term * d_mod;

        // PDMX: PD sum maximum
        let (p_final, d_final, pd_limit) = if self.gains.pd_max > 0.0 {
            let pd_sum_abs = libm::fabsf(p_out + d_out);
            if pd_sum_abs > self.gains.pd_max {
                let pd_limit_scale = self.gains.pd_max / pd_sum_abs;
                (p_out * pd_limit_scale, d_out * pd_limit_scale, true)
            } else {
                (p_out, d_out, false)
            }
        } else {
            (p_out, d_out, false)
        };

        let pd_output = p_final + d_final;

        // I term with anti-windup and limit flag
        // Source: AC_PID::update_i()
        // When `limit` is true, only allow integrator to shrink (decay toward zero), not grow.
        if !limit || ((self.integrator > 0.0 && self.filtered_error < 0.0) ||
                      (self.integrator < 0.0 && self.filtered_error > 0.0)) {
            self.integrator += self.gains.ki * self.filtered_error * i_scale * dt;
            self.integrator = self.integrator.clamp(-self.gains.imax, self.gains.imax);
        }

        // Feedforward
        let ff_term = self.gains.ff * target;

        self.prev_error = self.filtered_error;
        self.prev_target = self.filtered_target;
        let output = pd_output + self.integrator + ff_term + d_ff_term;
        self.prev_output = output;
        self.initialized = true;

        // Update PidInfo for logging
        self.pid_info = PidInfo {
            target,
            actual,
            error: self.filtered_error,
            p: p_final,
            i: self.integrator,
            d: d_final,
            ff: ff_term,
            d_ff: d_ff_term,
            d_mod,
            slew_rate: self.slew.slew_rate_filtered,
            limit,
            pd_limit,
            reset: was_reset,
            i_term_set: self.i_term_set,
        };
        self.i_term_set = false;
        self.was_reset = false;

        output
    }

    /// Simplified update (backwards-compatible): no limit flag, i_scale=1.0.
    /// Source: AC_PID::update_all()
    ///
    /// `target`: desired value
    /// `actual`: measured value
    /// `dt`: timestep (seconds)
    pub fn update(&mut self, target: f32, actual: f32, dt: f32) -> f32 {
        self.update_full(target, actual, dt, false, 1.0)
    }

    /// Update with error directly (no target/actual split).
    pub fn update_error(&mut self, error: f32, dt: f32) -> f32 {
        if dt < 1e-6 { return 0.0; }

        let p_term = self.gains.kp * error;

        let d_term = if self.gains.kd > 0.0 && self.initialized {
            let derivative = (error - self.prev_error) / dt;
            let filtered = Self::lp_filter(self.prev_derivative, derivative, self.gains.filt_hz, dt);
            self.prev_derivative = filtered;
            self.gains.kd * filtered
        } else {
            self.prev_derivative = 0.0;
            0.0
        };

        self.integrator += self.gains.ki * error * dt;
        self.integrator = self.integrator.clamp(-self.gains.imax, self.gains.imax);

        self.prev_error = error;
        self.initialized = true;

        p_term + self.integrator + d_term
    }

    /// Reset the controller state. Call on mode change.
    /// Source: AC_PID::reset_filter()
    pub fn reset(&mut self) {
        self.integrator = 0.0;
        self.prev_error = 0.0;
        self.prev_derivative = 0.0;
        self.prev_target = 0.0;
        self.filtered_target = 0.0;
        self.filtered_error = 0.0;
        self.prev_output = 0.0;
        self.initialized = false;
        self.slew.reset();
        self.was_reset = true;
        if let Some(ref mut nf) = self.notch_target { nf.reset(); }
        if let Some(ref mut nf) = self.notch_error { nf.reset(); }
        if let Some(ref mut lpf) = self.d_lpf2p { lpf.reset(); }
    }

    /// Get the current integrator value.
    pub fn get_integrator(&self) -> f32 { self.integrator }

    /// Set the integrator (for smooth mode transitions).
    pub fn set_integrator(&mut self, val: f32) {
        self.integrator = val.clamp(-self.gains.imax, self.gains.imax);
        self.i_term_set = true;
    }

    /// Exponentially relax the integrator toward a target value.
    /// Source: AC_PID::relax_integrator()
    ///
    /// `target`: target integrator value
    /// `dt`: timestep
    /// `tc`: time constant (seconds) — smaller = faster relaxation
    pub fn relax_integrator(&mut self, target: f32, dt: f32, tc: f32) {
        if tc <= 0.0 {
            self.set_integrator(target);
            return;
        }
        let alpha = dt / (dt + tc);
        self.integrator += alpha * (target - self.integrator);
        self.integrator = self.integrator.clamp(-self.gains.imax, self.gains.imax);
    }

    /// Get the PID telemetry info struct for logging.
    /// Source: AC_PID::get_pid_info()
    pub fn get_pid_info(&self) -> &PidInfo { &self.pid_info }

    /// Set per-PID notch filter on target signal.
    pub fn set_notch_target(&mut self, notch: NotchFilter) {
        self.notch_target = Some(notch);
    }

    /// Set per-PID notch filter on error signal.
    pub fn set_notch_error(&mut self, notch: NotchFilter) {
        self.notch_error = Some(notch);
    }

    /// Set 2nd-order Butterworth D-term filter (LowPassFilter2p).
    /// When set, this replaces the default 1st-order IIR on the D-term derivative.
    /// Source: ArduPilot AC_PID uses LowPassFilter2p for filt_D_hz.
    pub fn set_d_lpf2p(&mut self, lpf: LowPassFilter2p) {
        self.d_lpf2p = Some(lpf);
    }
}

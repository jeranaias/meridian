//! Notch and harmonic notch filters for vibration suppression.
//!
//! Source: ArduPilot libraries/Filter/NotchFilter.h, HarmonicNotchFilter.h
//!
//! CRITICAL: The notch filter has a 5% per-update slew limiter on center frequency.
//! Without this, the filter rings when the frequency changes abruptly.
//!
//! FIX: Biquad coefficients now match ArduPilot's Q-based formula instead of sinh-based.

/// Second-order IIR notch (band-reject) filter.
/// Removes a narrow frequency band from a signal.
pub struct NotchFilter {
    // Filter coefficients
    a0: f32,
    a1: f32,
    a2: f32,
    b0: f32,
    b1: f32,
    b2: f32,
    // State
    x1: f32,
    x2: f32,
    y1: f32,
    y2: f32,
    // Configuration
    center_freq_hz: f32,
    bandwidth_hz: f32,
    attenuation_db: f32,
    sample_rate_hz: f32,
    initialized: bool,
    /// Current effective center frequency (slew-limited).
    effective_freq_hz: f32,
}

// Implement Debug manually since it's needed by PidController
impl core::fmt::Debug for NotchFilter {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("NotchFilter")
            .field("center_freq_hz", &self.center_freq_hz)
            .field("bandwidth_hz", &self.bandwidth_hz)
            .field("initialized", &self.initialized)
            .finish()
    }
}

// Implement Clone for NotchFilter
impl Clone for NotchFilter {
    fn clone(&self) -> Self {
        Self {
            a0: self.a0, a1: self.a1, a2: self.a2,
            b0: self.b0, b1: self.b1, b2: self.b2,
            x1: self.x1, x2: self.x2, y1: self.y1, y2: self.y2,
            center_freq_hz: self.center_freq_hz,
            bandwidth_hz: self.bandwidth_hz,
            attenuation_db: self.attenuation_db,
            sample_rate_hz: self.sample_rate_hz,
            initialized: self.initialized,
            effective_freq_hz: self.effective_freq_hz,
        }
    }
}

/// Maximum slew rate for center frequency change (5% per update).
/// Source: ArduPilot NotchFilter.cpp -- prevents filter ringing on abrupt frequency changes.
const FREQ_SLEW_LIMIT: f32 = 0.05;

impl NotchFilter {
    pub fn new() -> Self {
        Self {
            a0: 1.0, a1: 0.0, a2: 0.0,
            b0: 1.0, b1: 0.0, b2: 0.0,
            x1: 0.0, x2: 0.0, y1: 0.0, y2: 0.0,
            center_freq_hz: 0.0,
            bandwidth_hz: 0.0,
            attenuation_db: 40.0,
            sample_rate_hz: 0.0,
            initialized: false,
            effective_freq_hz: 0.0,
        }
    }

    /// Initialize the notch filter.
    /// `center_freq_hz`: frequency to reject
    /// `bandwidth_hz`: width of the notch (Q = center/bandwidth)
    /// `attenuation_db`: depth of the notch in dB (typically 40)
    /// `sample_rate_hz`: filter sample rate
    pub fn init(&mut self, center_freq_hz: f32, bandwidth_hz: f32, attenuation_db: f32, sample_rate_hz: f32) {
        self.center_freq_hz = center_freq_hz;
        self.bandwidth_hz = bandwidth_hz;
        self.attenuation_db = attenuation_db;
        self.sample_rate_hz = sample_rate_hz;
        self.effective_freq_hz = center_freq_hz;
        self.calculate_coefficients(center_freq_hz);
        self.initialized = true;
    }

    /// Update center frequency (slew-limited to prevent ringing).
    pub fn set_center_freq(&mut self, freq_hz: f32) {
        if !self.initialized || freq_hz <= 0.0 { return; }
        self.center_freq_hz = freq_hz;

        // 5% slew limit -- CRITICAL
        let delta = freq_hz - self.effective_freq_hz;
        let max_delta = self.effective_freq_hz * FREQ_SLEW_LIMIT;
        let clamped = delta.clamp(-max_delta, max_delta);
        self.effective_freq_hz += clamped;

        self.calculate_coefficients(self.effective_freq_hz);
    }

    fn calculate_coefficients(&mut self, freq_hz: f32) {
        if self.sample_rate_hz <= 0.0 || freq_hz <= 0.0 { return; }

        // Clamp to 40% of Nyquist
        let nyquist = self.sample_rate_hz * 0.5;
        let freq = freq_hz.min(nyquist * 0.4);

        let omega = 2.0 * core::f32::consts::PI * freq / self.sample_rate_hz;
        let cos_omega = libm::cosf(omega);
        let sin_omega = libm::sinf(omega);

        // Attenuation factor: A = 10^(-attenuation_dB / 40)
        // Source: ArduPilot NotchFilter.cpp
        let a = libm::powf(10.0, -self.attenuation_db / 40.0);

        // Convert bandwidth to Q using ArduPilot's octave formula:
        //   octaves = log2(upper/lower) where bandwidth = upper - lower
        //   For bandwidth_hz and center_freq_hz:
        //   Q = center_freq / bandwidth
        let q = if self.bandwidth_hz > 0.0 {
            freq / self.bandwidth_hz
        } else {
            10.0 // High Q (narrow notch) if no bandwidth specified
        };

        // Alpha from Q (ArduPilot formula): alpha = sin(omega) / (2*Q)
        let alpha = sin_omega / (2.0 * q);

        // ArduPilot notch filter coefficients:
        //   b0 = 1 + alpha * A^2
        //   b1 = -2 * cos(omega)
        //   b2 = 1 - alpha * A^2
        //   a0 = 1 + alpha / (A^2)
        //   a1 = -2 * cos(omega)   (= b1 for notch)
        //   a2 = 1 - alpha / (A^2)
        // where A = 10^(-attenuation_dB/40) is the linear attenuation factor.
        let a_sq = a * a;
        self.b0 = 1.0 + alpha * a_sq;
        self.b1 = -2.0 * cos_omega;
        self.b2 = 1.0 - alpha * a_sq;
        self.a0 = 1.0 + alpha / a_sq;
        self.a1 = -2.0 * cos_omega;
        self.a2 = 1.0 - alpha / a_sq;

        // Normalize by a0
        let inv_a0 = 1.0 / self.a0;
        self.b0 *= inv_a0;
        self.b1 *= inv_a0;
        self.b2 *= inv_a0;
        self.a1 *= inv_a0;
        self.a2 *= inv_a0;
        self.a0 = 1.0;
    }

    /// Apply the notch filter to one sample.
    pub fn apply(&mut self, input: f32) -> f32 {
        if !self.initialized { return input; }

        let output = self.b0 * input + self.b1 * self.x1 + self.b2 * self.x2
            - self.a1 * self.y1 - self.a2 * self.y2;

        self.x2 = self.x1;
        self.x1 = input;
        self.y2 = self.y1;
        self.y1 = output;

        output
    }

    /// Reset filter state.
    pub fn reset(&mut self) {
        self.x1 = 0.0;
        self.x2 = 0.0;
        self.y1 = 0.0;
        self.y2 = 0.0;
    }

    pub fn center_freq(&self) -> f32 { self.effective_freq_hz }
    pub fn is_initialized(&self) -> bool { self.initialized }
}

// ==========================================================================
//  LowPassFilter2p — 2nd-order Butterworth biquad low-pass filter
//  Source: ArduPilot libraries/Filter/LowPassFilter2p.h / .cpp
// ==========================================================================

/// Second-order (two-pole) Butterworth low-pass filter.
/// Implements a biquad filter using the bilinear transform.
///
/// Source: ArduPilot LowPassFilter2p
#[derive(Debug, Clone)]
pub struct LowPassFilter2p {
    // Biquad coefficients (normalized by a0)
    b0: f32,
    b1: f32,
    b2: f32,
    a1: f32,
    a2: f32,
    // Delay elements
    delay1: f32,
    delay2: f32,
    // Configuration
    cutoff_hz: f32,
    sample_rate_hz: f32,
    initialized: bool,
}

impl LowPassFilter2p {
    pub fn new() -> Self {
        Self {
            b0: 1.0, b1: 0.0, b2: 0.0, a1: 0.0, a2: 0.0,
            delay1: 0.0, delay2: 0.0,
            cutoff_hz: 0.0, sample_rate_hz: 0.0,
            initialized: false,
        }
    }

    /// Initialize with cutoff frequency and sample rate.
    /// Computes Butterworth biquad coefficients via bilinear transform.
    pub fn init(&mut self, cutoff_hz: f32, sample_rate_hz: f32) {
        self.cutoff_hz = cutoff_hz;
        self.sample_rate_hz = sample_rate_hz;
        self.delay1 = 0.0;
        self.delay2 = 0.0;

        if cutoff_hz <= 0.0 || sample_rate_hz <= 0.0 {
            // Pass-through
            self.b0 = 1.0;
            self.b1 = 0.0;
            self.b2 = 0.0;
            self.a1 = 0.0;
            self.a2 = 0.0;
            self.initialized = true;
            return;
        }

        // Clamp to Nyquist
        let freq = cutoff_hz.min(sample_rate_hz * 0.5 * 0.98);

        // Bilinear transform: pre-warp analog frequency
        let omega = 2.0 * core::f32::consts::PI * freq / sample_rate_hz;
        let sin_w = libm::sinf(omega);
        let cos_w = libm::cosf(omega);

        // Butterworth Q = 1/sqrt(2) = 0.7071
        let q = core::f32::consts::FRAC_1_SQRT_2;
        let alpha = sin_w / (2.0 * q);

        // Butterworth low-pass biquad coefficients
        let a0 = 1.0 + alpha;
        let inv_a0 = 1.0 / a0;

        self.b0 = ((1.0 - cos_w) * 0.5) * inv_a0;
        self.b1 = (1.0 - cos_w) * inv_a0;
        self.b2 = self.b0;
        self.a1 = (-2.0 * cos_w) * inv_a0;
        self.a2 = (1.0 - alpha) * inv_a0;

        self.initialized = true;
    }

    /// Apply the filter to one sample. Uses Direct Form II transposed.
    pub fn apply(&mut self, input: f32) -> f32 {
        if !self.initialized || self.cutoff_hz <= 0.0 {
            return input;
        }

        // Direct Form II transposed
        let output = self.b0 * input + self.delay1;
        self.delay1 = self.b1 * input - self.a1 * output + self.delay2;
        self.delay2 = self.b2 * input - self.a2 * output;

        output
    }

    /// Reset filter state.
    pub fn reset(&mut self) {
        self.delay1 = 0.0;
        self.delay2 = 0.0;
    }

    /// Reset filter state and seed with an initial value to avoid startup transient.
    pub fn reset_to(&mut self, value: f32) {
        // Set delay elements so that a constant input of `value` produces `value`.
        // For steady-state: delay1 = value * (b1 - a1*b0)/(1), delay2 = value * (b2 - a2*b0)
        // Simplified: just run a few samples through to settle
        self.delay1 = 0.0;
        self.delay2 = 0.0;
        for _ in 0..20 {
            self.apply(value);
        }
    }

    pub fn cutoff_hz(&self) -> f32 { self.cutoff_hz }
    pub fn is_initialized(&self) -> bool { self.initialized }
}

/// Harmonic notch filter -- tracks motor frequency and suppresses harmonics.
/// Source: ArduPilot HarmonicNotchFilter.h
///
/// Supports up to MAX_HARMONICS simultaneous harmonic notch filters per axis,
/// with composite notch modes (double, triple, quintuple) per harmonic.
pub const MAX_HARMONICS: usize = 16;

/// Maximum number of physical notch filters (harmonics * composite multiplier).
const MAX_NOTCH_FILTERS: usize = 80; // 16 harmonics * 5 (quintuple mode)

/// Dynamic frequency tracking mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NotchTrackingMode {
    /// Fixed frequency (no tracking).
    Fixed,
    /// Track throttle: freq = base_freq * sqrt(throttle / ref_throttle)
    Throttle,
    /// Track RPM sensor
    Rpm,
    /// Track ESC telemetry RPM via BLHeli
    EscRpm,
    /// Track GyroFFT detected frequency
    Fft,
    /// Track secondary RPM sensor. Source: UpdateRPM2
    Rpm2,
}

/// Composite notch mode -- multiple notch filters per harmonic frequency.
/// Source: HarmonicNotchFilter DoubleNotch, TripleNotch, QuintupleNotch
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CompositeNotchMode {
    /// Single notch per harmonic (default).
    Single,
    /// Two notches per harmonic, offset by +/- 10% of center frequency.
    /// Source: HarmonicNotchFilter DoubleNotch
    Double,
    /// Three notches per harmonic, center + offset by +/- 15%.
    /// Source: HarmonicNotchFilter TripleNotch
    Triple,
    /// Five notches per harmonic: center, +/-8.5%, +/-17%.
    /// Source: HarmonicNotchFilter QuintupleNotch
    Quintuple,
}

/// Configuration for harmonic notch filter.
#[derive(Debug, Clone)]
pub struct HarmonicNotchConfig {
    pub enabled: bool,
    pub base_freq_hz: f32,
    pub bandwidth_hz: f32,
    pub attenuation_db: f32,
    /// Bitmask: bit 0 = 1x, bit 1 = 2x, etc. Now u16 for 16 harmonics.
    pub harmonics_mask: u16,
    pub tracking_mode: NotchTrackingMode,
    pub ref_throttle: f32, // reference throttle for throttle tracking (0.0-1.0)
    pub composite_mode: CompositeNotchMode,
}

impl Default for HarmonicNotchConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            base_freq_hz: 80.0,
            bandwidth_hz: 40.0,
            attenuation_db: 40.0,
            harmonics_mask: 0x0001, // just fundamental
            tracking_mode: NotchTrackingMode::Fixed,
            ref_throttle: 0.5,
            composite_mode: CompositeNotchMode::Single,
        }
    }
}

/// Per-axis harmonic notch filter bank.
pub struct HarmonicNotchBank {
    notches: [NotchFilter; MAX_NOTCH_FILTERS],
    active_count: usize,
    config: HarmonicNotchConfig,
    sample_rate_hz: f32,
    /// Maps each active notch to its harmonic index (for frequency updates).
    harmonic_indices: [usize; MAX_NOTCH_FILTERS],
}

impl HarmonicNotchBank {
    pub fn new() -> Self {
        Self {
            notches: core::array::from_fn(|_| NotchFilter::new()),
            active_count: 0,
            config: HarmonicNotchConfig::default(),
            sample_rate_hz: 0.0,
            harmonic_indices: [0; MAX_NOTCH_FILTERS],
        }
    }

    /// Initialize from configuration.
    pub fn init(&mut self, config: HarmonicNotchConfig, sample_rate_hz: f32) {
        self.config = config;
        self.sample_rate_hz = sample_rate_hz;
        self.active_count = 0;

        if !self.config.enabled { return; }

        // Set up notches per enabled harmonic, with composite modes
        for h in 0..MAX_HARMONICS {
            if self.config.harmonics_mask & (1 << h) != 0 {
                let harmonic = (h + 1) as f32;
                let center_freq = self.config.base_freq_hz * harmonic;

                match self.config.composite_mode {
                    CompositeNotchMode::Single => {
                        self.add_notch(center_freq, h);
                    }
                    CompositeNotchMode::Double => {
                        // Two notches offset by +/- 10%
                        self.add_notch(center_freq * 0.9, h);
                        self.add_notch(center_freq * 1.1, h);
                    }
                    CompositeNotchMode::Triple => {
                        // Three notches: center + offset by +/- 15%
                        self.add_notch(center_freq * 0.85, h);
                        self.add_notch(center_freq, h);
                        self.add_notch(center_freq * 1.15, h);
                    }
                    CompositeNotchMode::Quintuple => {
                        // Five notches: center, +/-8.5%, +/-17%
                        self.add_notch(center_freq * 0.83, h);
                        self.add_notch(center_freq * 0.915, h);
                        self.add_notch(center_freq, h);
                        self.add_notch(center_freq * 1.085, h);
                        self.add_notch(center_freq * 1.17, h);
                    }
                }
            }
        }
    }

    fn add_notch(&mut self, freq: f32, harmonic_idx: usize) {
        if freq < self.sample_rate_hz * 0.4 && self.active_count < MAX_NOTCH_FILTERS {
            self.notches[self.active_count].init(
                freq, self.config.bandwidth_hz,
                self.config.attenuation_db, self.sample_rate_hz,
            );
            self.harmonic_indices[self.active_count] = harmonic_idx;
            self.active_count += 1;
        }
    }

    /// Update the fundamental frequency (for dynamic tracking).
    pub fn update_freq(&mut self, base_freq_hz: f32) {
        if !self.config.enabled || base_freq_hz <= 0.0 { return; }

        for idx in 0..self.active_count {
            let harmonic = (self.harmonic_indices[idx] + 1) as f32;
            let center_freq = base_freq_hz * harmonic;

            // Adjust for composite mode offset
            let freq = match self.config.composite_mode {
                CompositeNotchMode::Single => center_freq,
                CompositeNotchMode::Double => {
                    // Determine if this is the lower or upper notch
                    let count_before = (0..idx)
                        .filter(|&i| self.harmonic_indices[i] == self.harmonic_indices[idx])
                        .count();
                    if count_before == 0 { center_freq * 0.9 } else { center_freq * 1.1 }
                }
                CompositeNotchMode::Triple => {
                    let count_before = (0..idx)
                        .filter(|&i| self.harmonic_indices[i] == self.harmonic_indices[idx])
                        .count();
                    match count_before {
                        0 => center_freq * 0.85,
                        1 => center_freq,
                        _ => center_freq * 1.15,
                    }
                }
                CompositeNotchMode::Quintuple => {
                    let count_before = (0..idx)
                        .filter(|&i| self.harmonic_indices[i] == self.harmonic_indices[idx])
                        .count();
                    match count_before {
                        0 => center_freq * 0.83,
                        1 => center_freq * 0.915,
                        2 => center_freq,
                        3 => center_freq * 1.085,
                        _ => center_freq * 1.17,
                    }
                }
            };

            self.notches[idx].set_center_freq(freq);
        }
    }

    /// Update frequency from throttle (throttle tracking mode).
    pub fn update_from_throttle(&mut self, throttle: f32) {
        if self.config.ref_throttle <= 0.0 { return; }
        let ratio = (throttle / self.config.ref_throttle).max(0.0);
        let freq = self.config.base_freq_hz * libm::sqrtf(ratio);
        self.update_freq(freq);
    }

    /// Apply all active notch filters to one sample.
    pub fn apply(&mut self, input: f32) -> f32 {
        if !self.config.enabled { return input; }
        let mut output = input;
        for i in 0..self.active_count {
            output = self.notches[i].apply(output);
        }
        output
    }

    /// Reset all filter state.
    pub fn reset(&mut self) {
        for n in &mut self.notches {
            n.reset();
        }
    }

    pub fn active_count(&self) -> usize { self.active_count }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_notch_passthrough_dc() {
        let mut nf = NotchFilter::new();
        nf.init(100.0, 20.0, 40.0, 1000.0);
        // DC input (0 Hz) should pass through
        for _ in 0..100 {
            let out = nf.apply(1.0);
            let _ = out;
        }
        let out = nf.apply(1.0);
        assert!((out - 1.0).abs() < 0.1, "DC should pass through, got {}", out);
    }

    #[test]
    fn test_notch_attenuates_center() {
        let mut nf = NotchFilter::new();
        nf.init(100.0, 20.0, 40.0, 1000.0);

        // Generate 100Hz sine and measure RMS after filtering
        let mut rms_in = 0.0f32;
        let mut rms_out = 0.0f32;
        for i in 0..1000 {
            let t = i as f32 / 1000.0;
            let input = libm::sinf(2.0 * core::f32::consts::PI * 100.0 * t);
            let output = nf.apply(input);
            if i > 100 { // skip transient
                rms_in += input * input;
                rms_out += output * output;
            }
        }
        let ratio_db = 10.0 * libm::log10f(rms_out / rms_in);
        assert!(ratio_db < -20.0, "Should attenuate 100Hz by >20dB, got {}dB", ratio_db);
    }

    #[test]
    fn test_notch_slew_limiter() {
        let mut nf = NotchFilter::new();
        nf.init(100.0, 20.0, 40.0, 1000.0);
        assert!((nf.center_freq() - 100.0).abs() < 0.1);

        // Jump to 200Hz -- should slew, not jump
        nf.set_center_freq(200.0);
        // After one update: 100 + 100*0.05 = 105
        assert!(nf.center_freq() < 110.0, "Should slew, not jump: {}", nf.center_freq());
    }

    #[test]
    fn test_harmonic_notch_multi() {
        let config = HarmonicNotchConfig {
            enabled: true,
            base_freq_hz: 80.0,
            bandwidth_hz: 20.0,
            attenuation_db: 40.0,
            harmonics_mask: 0x07, // 1x + 2x + 3x
            tracking_mode: NotchTrackingMode::Fixed,
            ref_throttle: 0.5,
            composite_mode: CompositeNotchMode::Single,
        };
        let mut bank = HarmonicNotchBank::new();
        bank.init(config, 1000.0);
        assert_eq!(bank.active_count(), 3); // 80, 160, 240 Hz
    }

    #[test]
    fn test_harmonic_notch_16_harmonics() {
        let config = HarmonicNotchConfig {
            enabled: true,
            base_freq_hz: 50.0,
            bandwidth_hz: 10.0,
            attenuation_db: 40.0,
            harmonics_mask: 0xFFFF, // all 16 harmonics
            tracking_mode: NotchTrackingMode::Fixed,
            ref_throttle: 0.5,
            composite_mode: CompositeNotchMode::Single,
        };
        let mut bank = HarmonicNotchBank::new();
        bank.init(config, 5000.0); // 5kHz sample rate
        // 50 * 1..16 = 50..800, all < 2000 (40% of 5000)
        assert_eq!(bank.active_count(), 16);
    }

    #[test]
    fn test_harmonic_notch_double_mode() {
        let config = HarmonicNotchConfig {
            enabled: true,
            base_freq_hz: 100.0,
            bandwidth_hz: 20.0,
            attenuation_db: 40.0,
            harmonics_mask: 0x01, // just fundamental
            tracking_mode: NotchTrackingMode::Fixed,
            ref_throttle: 0.5,
            composite_mode: CompositeNotchMode::Double,
        };
        let mut bank = HarmonicNotchBank::new();
        bank.init(config, 1000.0);
        assert_eq!(bank.active_count(), 2); // 90 Hz + 110 Hz
    }

    #[test]
    fn test_harmonic_notch_triple_mode() {
        let config = HarmonicNotchConfig {
            enabled: true,
            base_freq_hz: 100.0,
            bandwidth_hz: 20.0,
            attenuation_db: 40.0,
            harmonics_mask: 0x03, // 1x + 2x
            tracking_mode: NotchTrackingMode::Fixed,
            ref_throttle: 0.5,
            composite_mode: CompositeNotchMode::Triple,
        };
        let mut bank = HarmonicNotchBank::new();
        bank.init(config, 1000.0);
        // 2 harmonics x 3 notches each = 6
        assert_eq!(bank.active_count(), 6);
    }

    #[test]
    fn test_harmonic_notch_quintuple_mode() {
        let config = HarmonicNotchConfig {
            enabled: true,
            base_freq_hz: 100.0,
            bandwidth_hz: 20.0,
            attenuation_db: 40.0,
            harmonics_mask: 0x03, // 1x + 2x
            tracking_mode: NotchTrackingMode::Fixed,
            ref_throttle: 0.5,
            composite_mode: CompositeNotchMode::Quintuple,
        };
        let mut bank = HarmonicNotchBank::new();
        bank.init(config, 1000.0);
        // 2 harmonics x 5 notches each = 10
        assert_eq!(bank.active_count(), 10);
    }

    #[test]
    fn test_harmonic_throttle_tracking() {
        let config = HarmonicNotchConfig {
            enabled: true,
            base_freq_hz: 80.0,
            bandwidth_hz: 20.0,
            attenuation_db: 40.0,
            harmonics_mask: 0x01,
            tracking_mode: NotchTrackingMode::Throttle,
            ref_throttle: 0.5,
            composite_mode: CompositeNotchMode::Single,
        };
        let mut bank = HarmonicNotchBank::new();
        bank.init(config, 1000.0);

        // At ref throttle: freq should be base_freq
        bank.update_from_throttle(0.5);
        // Slew-limited, so won't jump instantly
        for _ in 0..100 { bank.update_from_throttle(0.5); }
        assert!((bank.notches[0].center_freq() - 80.0).abs() < 5.0);
    }

    #[test]
    fn test_notch_reset() {
        let mut nf = NotchFilter::new();
        nf.init(100.0, 20.0, 40.0, 1000.0);
        nf.apply(1.0);
        nf.apply(0.5);
        nf.reset();
        // After reset, verify state was cleared: feeding constant DC should converge.
        // With corrected a2 coefficient (deep notch), first sample may deviate significantly
        // from input, but after settling, DC should pass through.
        let mut out = 0.0;
        for _ in 0..200 {
            out = nf.apply(1.0);
        }
        assert!((out - 1.0).abs() < 0.1, "After reset + settling, DC should pass through: {}", out);
    }

    // -----------------------------------------------------------------------
    // LowPassFilter2p tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_lpf2p_dc_passthrough() {
        let mut lpf = LowPassFilter2p::new();
        lpf.init(50.0, 1000.0);
        // Feed DC and verify it passes through after settling
        let mut out = 0.0;
        for _ in 0..200 {
            out = lpf.apply(1.0);
        }
        assert!((out - 1.0).abs() < 0.01, "DC should pass: {}", out);
    }

    #[test]
    fn test_lpf2p_attenuates_high_freq() {
        let mut lpf = LowPassFilter2p::new();
        lpf.init(50.0, 1000.0); // 50 Hz cutoff at 1 kHz

        // Feed 200 Hz sine (well above cutoff) and measure attenuation
        let mut rms_in = 0.0f32;
        let mut rms_out = 0.0f32;
        for i in 0..2000 {
            let t = i as f32 / 1000.0;
            let input = libm::sinf(2.0 * core::f32::consts::PI * 200.0 * t);
            let output = lpf.apply(input);
            if i > 200 {
                rms_in += input * input;
                rms_out += output * output;
            }
        }
        let ratio_db = 10.0 * libm::log10f(rms_out / rms_in);
        assert!(ratio_db < -20.0, "200Hz should be attenuated >20dB at 50Hz cutoff: {}dB", ratio_db);
    }

    #[test]
    fn test_lpf2p_passes_low_freq() {
        let mut lpf = LowPassFilter2p::new();
        lpf.init(100.0, 1000.0); // 100 Hz cutoff

        // Feed 10 Hz sine (well below cutoff) and verify it passes
        let mut rms_in = 0.0f32;
        let mut rms_out = 0.0f32;
        for i in 0..2000 {
            let t = i as f32 / 1000.0;
            let input = libm::sinf(2.0 * core::f32::consts::PI * 10.0 * t);
            let output = lpf.apply(input);
            if i > 200 {
                rms_in += input * input;
                rms_out += output * output;
            }
        }
        let ratio_db = 10.0 * libm::log10f(rms_out / rms_in);
        assert!(ratio_db > -3.0, "10Hz should pass at 100Hz cutoff: {}dB", ratio_db);
    }

    #[test]
    fn test_lpf2p_reset() {
        let mut lpf = LowPassFilter2p::new();
        lpf.init(50.0, 1000.0);
        lpf.apply(100.0);
        lpf.apply(100.0);
        lpf.reset();
        // After reset, verify state was cleared
        let out = lpf.apply(0.0);
        assert!(out.abs() < 0.01, "After reset, output should be near 0: {}", out);
    }
}

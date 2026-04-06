//! Real-time gyro FFT with peak tracking and harmonic detection.

use heapless::Vec as HVec;
use libm::fabsf;
use crate::fft;

/// Maximum FFT window size supported.
const MAX_WINDOW: usize = 512;
/// Number of simultaneously tracked peaks.
const NUM_PEAKS: usize = 3;

/// A tracked frequency peak in the magnitude spectrum.
#[derive(Debug, Clone, Copy)]
pub struct TrackedPeak {
    pub frequency_hz: f32,
    pub magnitude: f32,
    pub is_valid: bool,
}

impl TrackedPeak {
    const fn invalid() -> Self {
        Self { frequency_hz: 0.0, magnitude: 0.0, is_valid: false }
    }
}

/// Real-time FFT processor for one gyro axis stream.
///
/// Feeds samples into a circular window buffer, runs radix-2 FFT when the
/// window is full, extracts magnitude spectrum, and tracks the top 3 peaks
/// with distance-matrix matching to prevent identity swaps between frames.
pub struct GyroFFT {
    /// FFT window size (power of 2, 32..=512).
    window_size: usize,
    /// Gyro sample rate (Hz).
    sample_rate_hz: f32,
    /// Sample accumulation buffer.
    window: HVec<f32, MAX_WINDOW>,
    /// Magnitude spectrum (window_size / 2 bins).
    fft_output: HVec<f32, 256>,
    /// Tracked peaks with frame-to-frame continuity.
    peaks: [TrackedPeak; NUM_PEAKS],
    /// Number of samples fed since last FFT.
    sample_count: usize,
}

impl GyroFFT {
    /// Create a new GyroFFT processor.
    ///
    /// `window_size`: must be a power of 2 in 32..=512.
    /// `sample_rate_hz`: gyro sample rate.
    pub fn new(window_size: usize, sample_rate_hz: f32) -> Self {
        debug_assert!(window_size.is_power_of_two());
        debug_assert!(window_size >= 32 && window_size <= MAX_WINDOW);

        Self {
            window_size,
            sample_rate_hz,
            window: HVec::new(),
            fft_output: HVec::new(),
            peaks: [TrackedPeak::invalid(); NUM_PEAKS],
            sample_count: 0,
        }
    }

    /// Feed one gyro sample. When the window is full, the FFT is computed
    /// and peaks are updated automatically.
    pub fn feed_sample(&mut self, value: f32) {
        if self.window.len() < self.window_size {
            let _ = self.window.push(value);
        }
        self.sample_count += 1;

        if self.window.len() == self.window_size {
            self.run_fft();
            self.window.clear();
            self.sample_count = 0;
        }
    }

    /// Get the current tracked peaks.
    pub fn get_peaks(&self) -> &[TrackedPeak; NUM_PEAKS] {
        &self.peaks
    }

    /// Get the primary (highest magnitude) peak frequency for notch filter use.
    pub fn get_center_frequency(&self) -> f32 {
        let mut best = &self.peaks[0];
        for p in &self.peaks {
            if p.is_valid && p.magnitude > best.magnitude {
                best = p;
            }
        }
        if best.is_valid { best.frequency_hz } else { 0.0 }
    }

    /// Get the noise floor (average magnitude excluding peaks).
    pub fn get_noise_floor(&self) -> f32 {
        if self.fft_output.is_empty() {
            return 0.0;
        }
        let peak_freqs: [f32; NUM_PEAKS] = [
            self.peaks[0].frequency_hz,
            self.peaks[1].frequency_hz,
            self.peaks[2].frequency_hz,
        ];
        let bin_hz = self.sample_rate_hz / (self.window_size as f32);
        let mut sum: f32 = 0.0;
        let mut count: u32 = 0;
        for (i, &mag) in self.fft_output.iter().enumerate() {
            let freq = (i as f32) * bin_hz;
            let is_peak = peak_freqs.iter().any(|&pf| fabsf(freq - pf) < bin_hz * 2.0);
            if !is_peak && i > 0 {
                sum += mag;
                count += 1;
            }
        }
        if count > 0 { sum / count as f32 } else { 0.0 }
    }

    /// Get signal-to-noise ratio for a peak (dB).
    pub fn get_snr_db(&self, peak_index: usize) -> f32 {
        if peak_index >= NUM_PEAKS || !self.peaks[peak_index].is_valid {
            return 0.0;
        }
        let noise_floor = self.get_noise_floor();
        if noise_floor <= 0.0 {
            return 0.0;
        }
        let signal = self.peaks[peak_index].magnitude;
        20.0 * libm::log10f(signal / noise_floor)
    }

    /// Check if a peak at `freq` is a harmonic of `fundamental`.
    /// Returns true if `|freq - N*fundamental| < fundamental * 0.1` for some N >= 2.
    pub fn is_harmonic(freq: f32, fundamental: f32) -> bool {
        if fundamental <= 0.0 {
            return false;
        }
        let ratio = freq / fundamental;
        let n = libm::roundf(ratio) as u32;
        if n < 2 {
            return false;
        }
        let expected = fundamental * (n as f32);
        fabsf(freq - expected) < fundamental * 0.1
    }

    // ── Internal ──────────────────────────────────────────────────────

    fn run_fft(&mut self) {
        let n = self.window_size;

        // Prepare real and imaginary buffers on the stack
        // (window_size <= 512, so this is fine for embedded)
        let mut re = [0.0f32; MAX_WINDOW];
        let mut im = [0.0f32; MAX_WINDOW];

        // Copy window data into real buffer
        for (i, &v) in self.window.iter().enumerate() {
            re[i] = v;
        }

        // Apply Hanning window
        fft::hanning_window(&mut re[..n], n);

        // Run FFT
        fft::fft(&mut re[..n], &mut im[..n], n);

        // Compute magnitude spectrum
        let half = n / 2;
        self.fft_output.clear();
        for i in 0..half {
            let mag = libm::sqrtf(re[i] * re[i] + im[i] * im[i]);
            let _ = self.fft_output.push(mag);
        }

        // Find top 3 peaks in magnitude spectrum
        let new_peaks = self.find_peaks();

        // Distance-matrix tracking: match new peaks to previous peaks
        self.match_peaks(new_peaks);
    }

    /// Find the top 3 peaks in the magnitude spectrum.
    /// A peak is a local maximum: `mag[i] > mag[i-1]` and `mag[i] > mag[i+1]`.
    fn find_peaks(&self) -> [TrackedPeak; NUM_PEAKS] {
        let mut candidates: [TrackedPeak; NUM_PEAKS] = [TrackedPeak::invalid(); NUM_PEAKS];
        let len = self.fft_output.len();
        if len < 3 {
            return candidates;
        }

        let bin_hz = self.sample_rate_hz / (self.window_size as f32);

        // Skip bin 0 (DC) and last bin
        for i in 1..(len - 1) {
            let mag = self.fft_output[i];
            if mag > self.fft_output[i - 1] && mag > self.fft_output[i + 1] {
                // Check if this peak is bigger than the smallest tracked
                let min_idx = Self::min_magnitude_index(&candidates);
                if mag > candidates[min_idx].magnitude {
                    candidates[min_idx] = TrackedPeak {
                        frequency_hz: (i as f32) * bin_hz,
                        magnitude: mag,
                        is_valid: true,
                    };
                }
            }
        }

        candidates
    }

    fn min_magnitude_index(peaks: &[TrackedPeak; NUM_PEAKS]) -> usize {
        let mut min_idx = 0;
        let mut min_mag = peaks[0].magnitude;
        for (i, p) in peaks.iter().enumerate().skip(1) {
            if p.magnitude < min_mag {
                min_mag = p.magnitude;
                min_idx = i;
            }
        }
        min_idx
    }

    /// Get the window size.
    pub fn window_size(&self) -> usize {
        self.window_size
    }

    /// Get the sample rate.
    pub fn sample_rate_hz(&self) -> f32 {
        self.sample_rate_hz
    }

    /// Distance-matrix peak matching: assign new peaks to the nearest
    /// previous peak by frequency to maintain identity across frames.
    fn match_peaks(&mut self, new_peaks: [TrackedPeak; NUM_PEAKS]) {
        // Count valid old and new peaks
        let old_valid: usize = self.peaks.iter().filter(|p| p.is_valid).count();
        let new_valid: usize = new_peaks.iter().filter(|p| p.is_valid).count();

        if old_valid == 0 || new_valid == 0 {
            // No previous peaks or no new peaks — just assign directly
            self.peaks = new_peaks;
            return;
        }

        // Build distance matrix: distance[old_i][new_j] = |freq_old - freq_new|
        let mut distance = [[f32::MAX; NUM_PEAKS]; NUM_PEAKS];
        for (i, old_p) in self.peaks.iter().enumerate() {
            if !old_p.is_valid { continue; }
            for (j, new_p) in new_peaks.iter().enumerate() {
                if !new_p.is_valid { continue; }
                distance[i][j] = fabsf(old_p.frequency_hz - new_p.frequency_hz);
            }
        }

        // Greedy assignment: pick the closest pair, assign, remove from candidates
        let mut used_old = [false; NUM_PEAKS];
        let mut used_new = [false; NUM_PEAKS];
        let mut result = [TrackedPeak::invalid(); NUM_PEAKS];

        for _ in 0..NUM_PEAKS {
            let mut best_dist = f32::MAX;
            let mut best_old = 0;
            let mut best_new = 0;

            for i in 0..NUM_PEAKS {
                if used_old[i] { continue; }
                for j in 0..NUM_PEAKS {
                    if used_new[j] { continue; }
                    if distance[i][j] < best_dist {
                        best_dist = distance[i][j];
                        best_old = i;
                        best_new = j;
                    }
                }
            }

            if best_dist == f32::MAX { break; }

            result[best_old] = new_peaks[best_new];
            used_old[best_old] = true;
            used_new[best_new] = true;
        }

        // Assign any unmatched new peaks to unused old slots
        for j in 0..NUM_PEAKS {
            if used_new[j] || !new_peaks[j].is_valid { continue; }
            for i in 0..NUM_PEAKS {
                if !used_old[i] {
                    result[i] = new_peaks[j];
                    used_old[i] = true;
                    used_new[j] = true;
                    break;
                }
            }
        }

        self.peaks = result;
    }
}

/// Multi-axis gyro FFT correlator.
/// Runs 3 independent GyroFFT instances (X, Y, Z) and correlates peaks.
pub struct TriAxisGyroFFT {
    pub axis_x: GyroFFT,
    pub axis_y: GyroFFT,
    pub axis_z: GyroFFT,
}

impl TriAxisGyroFFT {
    pub fn new(window_size: usize, sample_rate_hz: f32) -> Self {
        Self {
            axis_x: GyroFFT::new(window_size, sample_rate_hz),
            axis_y: GyroFFT::new(window_size, sample_rate_hz),
            axis_z: GyroFFT::new(window_size, sample_rate_hz),
        }
    }

    /// Feed one gyro sample for all 3 axes simultaneously.
    pub fn feed_sample(&mut self, x: f32, y: f32, z: f32) {
        self.axis_x.feed_sample(x);
        self.axis_y.feed_sample(y);
        self.axis_z.feed_sample(z);
    }

    /// Get the correlated center frequency — uses the axis with the strongest peak.
    pub fn get_center_frequency(&self) -> f32 {
        let fx = self.axis_x.get_center_frequency();
        let fy = self.axis_y.get_center_frequency();
        let fz = self.axis_z.get_center_frequency();

        let mx = self.axis_x.get_peaks().iter().filter(|p| p.is_valid).map(|p| p.magnitude).fold(0.0f32, f32::max);
        let my = self.axis_y.get_peaks().iter().filter(|p| p.is_valid).map(|p| p.magnitude).fold(0.0f32, f32::max);
        let mz = self.axis_z.get_peaks().iter().filter(|p| p.is_valid).map(|p| p.magnitude).fold(0.0f32, f32::max);

        if mx >= my && mx >= mz { fx }
        else if my >= mz { fy }
        else { fz }
    }

    /// Get per-axis center frequencies.
    pub fn get_per_axis_frequencies(&self) -> (f32, f32, f32) {
        (
            self.axis_x.get_center_frequency(),
            self.axis_y.get_center_frequency(),
            self.axis_z.get_center_frequency(),
        )
    }
}

/// Notch filter integration output.
/// The FFT drives the center frequency of a HarmonicNotchFilter.
pub struct NotchFilterConfig {
    /// Center frequency detected by FFT (Hz).
    pub center_freq_hz: f32,
    /// Bandwidth of the notch (Hz).
    pub bandwidth_hz: f32,
    /// Attenuation in dB.
    pub attenuation_db: f32,
    /// Harmonic multipliers to also notch.
    pub harmonics: u8,
    /// Whether the config is valid (FFT has detected a real peak).
    pub valid: bool,
}

impl NotchFilterConfig {
    /// Generate notch filter config from GyroFFT output.
    pub fn from_fft(fft: &GyroFFT, bandwidth_hz: f32, attenuation_db: f32, harmonics: u8) -> Self {
        let center = fft.get_center_frequency();
        Self {
            center_freq_hz: center,
            bandwidth_hz,
            attenuation_db,
            harmonics,
            valid: center > 0.0,
        }
    }

    /// Generate from TriAxisGyroFFT (uses correlated peak).
    pub fn from_tri_axis(fft: &TriAxisGyroFFT, bandwidth_hz: f32, attenuation_db: f32, harmonics: u8) -> Self {
        let center = fft.get_center_frequency();
        Self {
            center_freq_hz: center,
            bandwidth_hz,
            attenuation_db,
            harmonics,
            valid: center > 0.0,
        }
    }
}

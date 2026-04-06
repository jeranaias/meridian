//! Radix-2 DIT FFT (Cooley-Tukey) for power-of-2 sizes.
//!
//! Operates in-place on interleaved real/imaginary pairs.

use libm::{cosf, sinf, sqrtf};
use core::f32::consts::PI;

/// Bit-reversal permutation in-place.
/// `re` and `im` are parallel arrays of length `n` (must be power of 2).
fn bit_reverse(re: &mut [f32], im: &mut [f32], n: usize) {
    let mut j: usize = 0;
    for i in 0..n {
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
        let mut m = n >> 1;
        while m >= 1 && j >= m {
            j -= m;
            m >>= 1;
        }
        j += m;
    }
}

/// In-place radix-2 DIT FFT.
///
/// `re[0..n]` — real parts, `im[0..n]` — imaginary parts.
/// `n` must be a power of 2 and `<= 512`.
pub fn fft(re: &mut [f32], im: &mut [f32], n: usize) {
    debug_assert!(n.is_power_of_two());
    debug_assert!(re.len() >= n);
    debug_assert!(im.len() >= n);

    bit_reverse(re, im, n);

    let mut size: usize = 2;
    while size <= n {
        let half = size / 2;
        let angle_step = -2.0 * PI / (size as f32);

        let mut i = 0;
        while i < n {
            for k in 0..half {
                let angle = angle_step * (k as f32);
                let wr = cosf(angle);
                let wi = sinf(angle);

                let even = i + k;
                let odd = i + k + half;

                let tr = wr * re[odd] - wi * im[odd];
                let ti = wr * im[odd] + wi * re[odd];

                re[odd] = re[even] - tr;
                im[odd] = im[even] - ti;
                re[even] += tr;
                im[even] += ti;
            }
            i += size;
        }
        size <<= 1;
    }
}

/// Compute magnitude spectrum from FFT output.
/// Returns `n/2` magnitudes (DC to Nyquist) written into `mag[0..n/2]`.
pub fn magnitude_spectrum(re: &[f32], im: &[f32], mag: &mut [f32], n: usize) {
    let half = n / 2;
    for i in 0..half {
        mag[i] = sqrtf(re[i] * re[i] + im[i] * im[i]);
    }
}

/// Apply a Hanning window to `data[0..n]`.
pub fn hanning_window(data: &mut [f32], n: usize) {
    for i in 0..n {
        let w = 0.5 * (1.0 - cosf(2.0 * PI * (i as f32) / (n as f32)));
        data[i] *= w;
    }
}

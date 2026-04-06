//! Unit tests for the FFT and GyroFFT modules.

#[cfg(test)]
mod fft_tests {
    use crate::fft::{fft, magnitude_spectrum, hanning_window};
    use libm::sinf;
    use core::f32::consts::PI;

    #[test]
    fn test_dc_signal() {
        // Constant signal should produce energy only in bin 0
        let n = 64;
        let mut re = [1.0f32; 512];
        let mut im = [0.0f32; 512];
        fft(&mut re[..n], &mut im[..n], n);

        let mut mag = [0.0f32; 256];
        magnitude_spectrum(&re[..n], &im[..n], &mut mag, n);

        // Bin 0 (DC) should have all the energy
        assert!(mag[0] > 60.0, "DC bin should be large: {}", mag[0]);
        // Other bins should be near zero
        for i in 1..(n / 2) {
            assert!(mag[i] < 0.01, "Bin {} should be ~0 for DC signal: {}", i, mag[i]);
        }
    }

    #[test]
    fn test_single_tone() {
        // Generate a pure sine at bin 8 of a 64-point FFT
        let n = 64;
        let target_bin = 8;
        let mut re = [0.0f32; 512];
        let mut im = [0.0f32; 512];

        for i in 0..n {
            re[i] = sinf(2.0 * PI * (target_bin as f32) * (i as f32) / (n as f32));
        }

        fft(&mut re[..n], &mut im[..n], n);

        let mut mag = [0.0f32; 256];
        magnitude_spectrum(&re[..n], &im[..n], &mut mag, n);

        // The target bin should have the highest magnitude
        let mut max_bin = 0;
        let mut max_mag = 0.0;
        for i in 0..(n / 2) {
            if mag[i] > max_mag {
                max_mag = mag[i];
                max_bin = i;
            }
        }
        assert_eq!(max_bin, target_bin, "Peak should be at bin {}, got {}", target_bin, max_bin);
    }

    #[test]
    fn test_two_tones() {
        let n = 128;
        let bin_a = 5;
        let bin_b = 20;
        let mut re = [0.0f32; 512];
        let mut im = [0.0f32; 512];

        for i in 0..n {
            re[i] = sinf(2.0 * PI * (bin_a as f32) * (i as f32) / (n as f32))
                  + sinf(2.0 * PI * (bin_b as f32) * (i as f32) / (n as f32));
        }

        fft(&mut re[..n], &mut im[..n], n);

        let mut mag = [0.0f32; 256];
        magnitude_spectrum(&re[..n], &im[..n], &mut mag, n);

        // Both bins should be prominent peaks
        // Find top 2 bins
        let mut top = [(0usize, 0.0f32); 2];
        for i in 1..(n / 2) {
            if mag[i] > top[0].1 {
                top[1] = top[0];
                top[0] = (i, mag[i]);
            } else if mag[i] > top[1].1 {
                top[1] = (i, mag[i]);
            }
        }
        let mut bins = [top[0].0, top[1].0];
        bins.sort();
        assert_eq!(bins, [bin_a, bin_b],
            "Expected bins [{}, {}], got [{}, {}]", bin_a, bin_b, bins[0], bins[1]);
    }

    #[test]
    fn test_hanning_window_zeros_endpoints() {
        let n = 64;
        let mut data = [1.0f32; 512];
        hanning_window(&mut data[..n], n);
        assert!(data[0].abs() < 1e-6, "Hanning should zero the first sample: {}", data[0]);
        // Middle should be near 1.0
        assert!((data[n / 2] - 1.0).abs() < 0.01, "Hanning mid should be ~1.0: {}", data[n / 2]);
    }

    #[test]
    fn test_power_of_two_sizes() {
        // Test that FFT works for all supported sizes
        for &n in &[32, 64, 128, 256, 512] {
            let mut re = [0.0f32; 512];
            let mut im = [0.0f32; 512];
            re[0] = 1.0; // impulse
            fft(&mut re[..n], &mut im[..n], n);
            // All bins should have magnitude 1 for a unit impulse
            for i in 0..n {
                let mag = libm::sqrtf(re[i] * re[i] + im[i] * im[i]);
                assert!((mag - 1.0).abs() < 0.01,
                    "n={} bin={}: impulse FFT magnitude should be 1.0, got {}", n, i, mag);
            }
        }
    }
}

#[cfg(test)]
mod gyro_fft_tests {
    use crate::gyro_fft::GyroFFT;
    use libm::sinf;
    use core::f32::consts::PI;

    #[test]
    fn test_new_peaks_invalid() {
        let fft = GyroFFT::new(64, 1000.0);
        for p in fft.get_peaks() {
            assert!(!p.is_valid, "Peaks should start invalid");
        }
    }

    #[test]
    fn test_center_frequency_zero_when_no_data() {
        let fft = GyroFFT::new(64, 1000.0);
        assert_eq!(fft.get_center_frequency(), 0.0);
    }

    #[test]
    fn test_detects_single_tone() {
        let window_size = 128;
        let sample_rate = 1000.0;
        let target_freq = 100.0; // Hz
        let mut fft = GyroFFT::new(window_size, sample_rate);

        // Feed a pure sine at target_freq
        for i in 0..window_size {
            let t = (i as f32) / sample_rate;
            let sample = sinf(2.0 * PI * target_freq * t);
            fft.feed_sample(sample);
        }

        let center = fft.get_center_frequency();
        let bin_resolution = sample_rate / (window_size as f32); // 7.8125 Hz
        assert!((center - target_freq).abs() < bin_resolution * 1.5,
            "Center frequency should be near {} Hz, got {} Hz", target_freq, center);
    }

    #[test]
    fn test_detects_multiple_tones() {
        let window_size = 256;
        let sample_rate = 1000.0;
        let freq_a = 80.0;
        let freq_b = 200.0;
        let mut fft = GyroFFT::new(window_size, sample_rate);

        for i in 0..window_size {
            let t = (i as f32) / sample_rate;
            let sample = sinf(2.0 * PI * freq_a * t) + 0.8 * sinf(2.0 * PI * freq_b * t);
            fft.feed_sample(sample);
        }

        let peaks = fft.get_peaks();
        let valid: heapless::Vec<f32, 3> = peaks.iter()
            .filter(|p| p.is_valid)
            .map(|p| p.frequency_hz)
            .collect();

        assert!(valid.len() >= 2, "Should detect at least 2 peaks, got {}", valid.len());

        let bin_res = sample_rate / (window_size as f32);
        let has_a = valid.iter().any(|&f| (f - freq_a).abs() < bin_res * 2.0);
        let has_b = valid.iter().any(|&f| (f - freq_b).abs() < bin_res * 2.0);
        assert!(has_a, "Should detect peak near {} Hz. Got: {:?}", freq_a, valid.as_slice());
        assert!(has_b, "Should detect peak near {} Hz. Got: {:?}", freq_b, valid.as_slice());
    }

    #[test]
    fn test_peak_tracking_identity() {
        // Feed two windows with slightly shifted frequency to verify
        // peak identity is maintained (not swapped)
        let window_size = 128;
        let sample_rate = 1000.0;
        let mut fft = GyroFFT::new(window_size, sample_rate);

        // Window 1: tone at 100 Hz
        for i in 0..window_size {
            let t = (i as f32) / sample_rate;
            fft.feed_sample(sinf(2.0 * PI * 100.0 * t));
        }

        let peaks1 = *fft.get_peaks();

        // Window 2: tone at 105 Hz (slightly shifted)
        for i in 0..window_size {
            let t = (i as f32) / sample_rate;
            fft.feed_sample(sinf(2.0 * PI * 105.0 * t));
        }

        let peaks2 = *fft.get_peaks();

        // The peak that was tracking ~100 Hz should now be near ~105 Hz in the same slot
        // (not jumped to a different slot)
        let valid1: heapless::Vec<usize, 3> = peaks1.iter().enumerate()
            .filter(|(_, p)| p.is_valid)
            .map(|(i, _)| i)
            .collect();

        if let Some(&slot) = valid1.first() {
            assert!(peaks2[slot].is_valid, "Same slot should still be valid");
            let bin_res = sample_rate / (window_size as f32);
            assert!((peaks2[slot].frequency_hz - 105.0).abs() < bin_res * 2.0,
                "Tracked peak should follow to ~105 Hz, got {}", peaks2[slot].frequency_hz);
        }
    }

    #[test]
    fn test_harmonic_detection() {
        // 100 Hz fundamental
        assert!(GyroFFT::is_harmonic(200.0, 100.0), "200 is 2nd harmonic of 100");
        assert!(GyroFFT::is_harmonic(300.0, 100.0), "300 is 3rd harmonic of 100");
        assert!(GyroFFT::is_harmonic(198.0, 100.0), "198 is within 10% of 2*100");
        assert!(!GyroFFT::is_harmonic(150.0, 100.0), "150 is not a harmonic of 100");
        assert!(!GyroFFT::is_harmonic(100.0, 100.0), "Fundamental itself is not a harmonic (N>=2)");
        assert!(!GyroFFT::is_harmonic(50.0, 100.0), "Sub-harmonic should not match");
    }

    #[test]
    fn test_harmonic_edge_cases() {
        assert!(!GyroFFT::is_harmonic(100.0, 0.0), "Zero fundamental");
        assert!(!GyroFFT::is_harmonic(100.0, -50.0), "Negative fundamental");
    }

    #[test]
    fn test_no_crash_on_silence() {
        let mut fft = GyroFFT::new(64, 1000.0);
        for _ in 0..64 {
            fft.feed_sample(0.0);
        }
        // Should not crash, peaks should have zero magnitude
        let center = fft.get_center_frequency();
        assert!(center >= 0.0, "Center freq should be non-negative on silence");
    }
}

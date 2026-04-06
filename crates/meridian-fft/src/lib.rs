#![no_std]

//! Real-time FFT on gyro data for vibration analysis and notch filter tuning.
//!
//! Implements a radix-2 Cooley-Tukey DIT FFT with Hanning window,
//! peak detection with distance-matrix tracking, harmonic detection,
//! 3-axis correlation, SNR computation, and notch filter integration.

pub mod fft;
pub mod gyro_fft;

pub use gyro_fft::{GyroFFT, TrackedPeak, TriAxisGyroFFT, NotchFilterConfig};

#[cfg(test)]
mod tests;

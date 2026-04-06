#![no_std]

//! PID auto-tuning via multirotor twitch method and helicopter frequency sweep.
//!
//! Matches ArduPilot's AC_AutoTune_Multi and AC_AutoTune_Heli.

pub mod tuner;

pub use tuner::{AutoTuner, TuneAxis, TuneStep, TuneState, TuneResults, MultiAxisAutoTuner, HeliAutoTuner};

#[cfg(test)]
mod tests;

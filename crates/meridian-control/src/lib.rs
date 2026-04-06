#![no_std]

pub mod pid;
pub mod rate_controller;
pub mod attitude_controller;
pub mod position_controller;
pub mod air_density;
pub mod sqrt_controller;
pub mod notch_filter;
pub mod tecs;

pub use pid::{PidController, PidGains, PidInfo};
pub use rate_controller::RateController;
pub use attitude_controller::AttitudeController;
pub use position_controller::PositionController;
pub use air_density::{air_density_ratio, throttle_altitude_compensation};
pub use notch_filter::LowPassFilter2p;

#[cfg(test)]
mod tests;

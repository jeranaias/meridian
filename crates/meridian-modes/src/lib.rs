#![no_std]

//! Flight mode state machines for all vehicle classes.
//!
//! Each mode implements the `FlightMode` trait, producing control targets
//! (attitude + throttle for multirotors, roll/pitch/throttle for fixed-wing,
//! steering + throttle for rovers).

pub mod mode_trait;
pub mod multirotor;
pub mod fixed_wing;
pub mod rover;
pub mod submarine;

pub use mode_trait::{FlightMode, ModeInput, ModeOutput};
pub use multirotor::MultirotorModes;
pub use fixed_wing::FixedWingModes;
pub use rover::RoverModes;

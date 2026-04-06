#![no_std]

//! Precision landing with per-axis Kalman filters, NIS outlier rejection,
//! inertial lag compensation, XY descent gate, state machine, and retry logic.
//!
//! Source: ArduPilot `libraries/AC_PrecLand/`

pub mod kalman;
pub mod backend;
pub mod inertial_history;
pub mod precland;

#[cfg(test)]
mod tests;

pub use kalman::PosVelKF;
pub use backend::{PrecLandBackend, PrecLandTarget, IRLockBackend, MavlinkLandingBackend, SitlLandingBackend};
pub use inertial_history::InertialHistory;
pub use precland::{PrecisionLand, TargetState};

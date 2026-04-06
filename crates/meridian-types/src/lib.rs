#![no_std]

pub mod time;
pub mod vehicle;
pub mod messages;

pub use time::{Instant, Duration};
pub use vehicle::{VehicleLifecycle, FlightModeId, FailsafeReason, FailsafeAction};
pub use messages::Message;

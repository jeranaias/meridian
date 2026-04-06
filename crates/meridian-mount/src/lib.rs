#![no_std]

//! Gimbal/mount control subsystem.
//!
//! Supports servo PWM, MAVLink DO_MOUNT_CONTROL, Siyi binary protocol,
//! STorM32, Alexmos, Gremsy, Viewpro, Topotek, CADDX, and Xacti backends.
//! Source: ArduPilot `libraries/AP_Mount/`

pub mod backend;
pub mod servo;
pub mod mavlink;
pub mod siyi;
pub mod manager;

#[cfg(test)]
mod tests;

pub use backend::{MountBackend, MountTarget};
pub use servo::ServoMount;
pub use mavlink::MavlinkMount;
pub use siyi::SiyiMount;
pub use manager::{MountManager, MountMode, MountBackendType};

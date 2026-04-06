#![no_std]

//! Camera trigger, video control, distance/time auto-triggering, and geotagging.
//!
//! Source: ArduPilot `libraries/AP_Camera/`

pub mod backend;
pub mod geotag;
pub mod trigger;

#[cfg(test)]
mod tests;

pub use backend::{CameraBackend, ServoCameraBackend, RelayCameraBackend, MountCameraBackend, RunCamBackend, MavlinkCamV2Backend};
pub use geotag::GeotagEntry;
pub use trigger::{CameraManager, TriggerMode};

#![no_std]

pub mod frames;
pub mod vec3;
pub mod quaternion;
pub mod rotation;
pub mod geodetic;

// Re-exports for convenience
pub use frames::*;
pub use vec3::Vec3;
pub use quaternion::Quaternion;
pub use rotation::Rotation;
pub use geodetic::LatLonAlt;

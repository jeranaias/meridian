/// Coordinate frame marker types. Zero-sized — no runtime cost.
/// Used as type parameters on Vec3 and Rotation to prevent frame mismatches at compile time.

/// North-East-Down navigation frame (tangent to Earth surface)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NED;

/// Body frame (forward-right-down, attached to vehicle)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Body;

/// Earth-Centered Earth-Fixed frame
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ECEF;

/// Latitude-Longitude-Altitude (geodetic)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LLA;

/// Earth-North-Up (used by some sensors, converted early)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ENU;

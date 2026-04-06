use core::marker::PhantomData;
use core::ops::{Add, Sub, Neg, Mul};

/// A 3D vector tagged with its coordinate frame.
/// Frame safety: Vec3<Body> + Vec3<NED> is a compile error.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3<F> {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    _frame: PhantomData<F>,
}

impl<F> Vec3<F> {
    #[inline]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z, _frame: PhantomData }
    }

    #[inline]
    pub const fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    #[inline]
    pub fn length_squared(&self) -> f32 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    #[inline]
    pub fn length(&self) -> f32 {
        libm::sqrtf(self.length_squared())
    }

    #[inline]
    pub fn normalized(&self) -> Self {
        let len = self.length();
        if len > 1e-12 {
            let inv = 1.0 / len;
            Self::new(self.x * inv, self.y * inv, self.z * inv)
        } else {
            Self::zero()
        }
    }

    #[inline]
    pub fn dot(&self, other: &Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    #[inline]
    pub fn cross(&self, other: &Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    #[inline]
    pub fn scale(&self, s: f32) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }

    /// Element-wise multiply
    #[inline]
    pub fn element_mul(&self, other: &Self) -> Self {
        Self::new(self.x * other.x, self.y * other.y, self.z * other.z)
    }

    /// Constrain each component to [-limit, limit]
    #[inline]
    pub fn constrain(&self, limit: f32) -> Self {
        Self::new(
            self.x.clamp(-limit, limit),
            self.y.clamp(-limit, limit),
            self.z.clamp(-limit, limit),
        )
    }

    /// Returns true if any component is NaN
    #[inline]
    pub fn is_nan(&self) -> bool {
        self.x.is_nan() || self.y.is_nan() || self.z.is_nan()
    }

    /// Returns true if any component is infinite
    #[inline]
    pub fn is_infinite(&self) -> bool {
        self.x.is_infinite() || self.y.is_infinite() || self.z.is_infinite()
    }

    /// Cast to a different frame (unsafe — only use when you KNOW the frame is correct)
    #[inline]
    pub fn cast_frame<G>(self) -> Vec3<G> {
        Vec3::new(self.x, self.y, self.z)
    }
}

// Same-frame addition (Vec3<NED> + Vec3<NED> = Vec3<NED>)
impl<F> Add for Vec3<F> {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

// Same-frame subtraction
impl<F> Sub for Vec3<F> {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

// Negation
impl<F> Neg for Vec3<F> {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

// Scalar multiplication (Vec3 * f32)
impl<F> Mul<f32> for Vec3<F> {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f32) -> Self {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

// Scalar multiplication (f32 * Vec3)
impl<F> Mul<Vec3<F>> for f32 {
    type Output = Vec3<F>;
    #[inline]
    fn mul(self, rhs: Vec3<F>) -> Vec3<F> {
        Vec3::new(self * rhs.x, self * rhs.y, self * rhs.z)
    }
}

impl<F> Default for Vec3<F> {
    fn default() -> Self {
        Self::zero()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::{NED, Body};

    #[test]
    fn test_basic_ops() {
        let a: Vec3<NED> = Vec3::new(1.0, 2.0, 3.0);
        let b: Vec3<NED> = Vec3::new(4.0, 5.0, 6.0);
        let c = a + b;
        assert_eq!(c.x, 5.0);
        assert_eq!(c.y, 7.0);
        assert_eq!(c.z, 9.0);
    }

    #[test]
    fn test_dot_product() {
        let a: Vec3<NED> = Vec3::new(1.0, 0.0, 0.0);
        let b: Vec3<NED> = Vec3::new(0.0, 1.0, 0.0);
        assert_eq!(a.dot(&b), 0.0);
        assert_eq!(a.dot(&a), 1.0);
    }

    #[test]
    fn test_cross_product() {
        let x: Vec3<NED> = Vec3::new(1.0, 0.0, 0.0);
        let y: Vec3<NED> = Vec3::new(0.0, 1.0, 0.0);
        let z = x.cross(&y);
        assert_eq!(z.x, 0.0);
        assert_eq!(z.y, 0.0);
        assert_eq!(z.z, 1.0);
    }

    #[test]
    fn test_length_and_normalize() {
        let v: Vec3<Body> = Vec3::new(3.0, 4.0, 0.0);
        assert!((v.length() - 5.0).abs() < 1e-6);
        let n = v.normalized();
        assert!((n.length() - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_frame_safety_same_frame() {
        // This compiles: same frame addition
        let a: Vec3<NED> = Vec3::new(1.0, 0.0, 0.0);
        let b: Vec3<NED> = Vec3::new(0.0, 1.0, 0.0);
        let _c = a + b;
    }

    // This would NOT compile (frame mismatch):
    // let a: Vec3<NED> = Vec3::new(1.0, 0.0, 0.0);
    // let b: Vec3<Body> = Vec3::new(0.0, 1.0, 0.0);
    // let _c = a + b;  // COMPILE ERROR: mismatched types
}

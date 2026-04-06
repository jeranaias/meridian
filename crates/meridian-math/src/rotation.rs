use core::marker::PhantomData;
use crate::quaternion::Quaternion;
use crate::vec3::Vec3;

/// A rotation that converts vectors from frame `From` to frame `To`.
/// Wraps a unit quaternion with compile-time frame tracking.
///
/// This is a COMPILE ERROR if the frames don't match:
///   let body_to_ned: Rotation<Body, NED> = ...;
///   let v_body: Vec3<Body> = ...;
///   let v_ned: Vec3<NED> = body_to_ned.rotate(v_body); // OK
///   let v_ecef: Vec3<ECEF> = body_to_ned.rotate(v_body); // COMPILE ERROR
#[derive(Debug, Clone, Copy)]
pub struct Rotation<From, To> {
    q: Quaternion,
    _from: PhantomData<From>,
    _to: PhantomData<To>,
}

impl<From, To> Rotation<From, To> {
    /// Create a rotation from a unit quaternion.
    /// The caller is responsible for ensuring the quaternion represents
    /// the correct transformation from `From` to `To`.
    #[inline]
    pub fn from_quaternion(q: Quaternion) -> Self {
        Self {
            q,
            _from: PhantomData,
            _to: PhantomData,
        }
    }

    /// Identity rotation (no change, same frame interpretation)
    #[inline]
    pub fn identity() -> Self {
        Self::from_quaternion(Quaternion::identity())
    }

    /// Rotate a vector from frame `From` to frame `To`.
    #[inline]
    pub fn rotate(&self, v: Vec3<From>) -> Vec3<To> {
        let rotated = self.q.rotate_vector(v);
        rotated.cast_frame()
    }

    /// Inverse rotation: To → From
    #[inline]
    pub fn inverse(self) -> Rotation<To, From> {
        Rotation::from_quaternion(self.q.conjugate())
    }

    /// Chain rotations: From→To then To→Next gives From→Next
    #[inline]
    pub fn then<Next>(self, next: Rotation<To, Next>) -> Rotation<From, Next> {
        Rotation::from_quaternion(next.q * self.q)
    }

    /// Get the underlying quaternion
    #[inline]
    pub fn quaternion(&self) -> &Quaternion {
        &self.q
    }

    /// Create from Euler angles (roll, pitch, yaw in radians)
    pub fn from_euler(roll: f32, pitch: f32, yaw: f32) -> Self {
        Self::from_quaternion(Quaternion::from_euler(roll, pitch, yaw))
    }

    /// Convert to Euler angles (roll, pitch, yaw in radians)
    pub fn to_euler(&self) -> (f32, f32, f32) {
        self.q.to_euler()
    }
}

impl<F> Rotation<F, F> {
    /// An identity rotation within the same frame
    pub fn same_frame_identity() -> Self {
        Self::from_quaternion(Quaternion::identity())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frames::{Body, NED, ECEF};
    const TOL: f32 = 1e-5;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < TOL
    }

    #[test]
    fn test_rotate_identity() {
        let r: Rotation<Body, NED> = Rotation::identity();
        let v = Vec3::<Body>::new(1.0, 2.0, 3.0);
        let result = r.rotate(v);
        assert!(approx_eq(result.x, 1.0));
        assert!(approx_eq(result.y, 2.0));
        assert!(approx_eq(result.z, 3.0));
    }

    #[test]
    fn test_inverse_roundtrip() {
        let body_to_ned: Rotation<Body, NED> = Rotation::from_euler(0.3, 0.2, 1.0);
        let ned_to_body: Rotation<NED, Body> = body_to_ned.inverse();
        let v = Vec3::<Body>::new(1.0, 2.0, 3.0);
        let v_ned = body_to_ned.rotate(v);
        let v_back = ned_to_body.rotate(v_ned);
        assert!(approx_eq(v_back.x, v.x));
        assert!(approx_eq(v_back.y, v.y));
        assert!(approx_eq(v_back.z, v.z));
    }

    #[test]
    fn test_chain_rotations() {
        let body_to_ned: Rotation<Body, NED> = Rotation::from_euler(0.1, 0.2, 0.3);
        let ned_to_ecef: Rotation<NED, ECEF> = Rotation::from_euler(0.4, 0.5, 0.6);
        let body_to_ecef: Rotation<Body, ECEF> = body_to_ned.then(ned_to_ecef);

        let v = Vec3::<Body>::new(1.0, 0.0, 0.0);
        let direct = body_to_ecef.rotate(v);
        let chained = ned_to_ecef.rotate(body_to_ned.rotate(v));
        assert!(approx_eq(direct.x, chained.x));
        assert!(approx_eq(direct.y, chained.y));
        assert!(approx_eq(direct.z, chained.z));
    }
}

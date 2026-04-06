#![no_std]

//! Geofence engine: polygon and cylinder zones with inclusion/exclusion logic.

use meridian_math::Vec3;
use meridian_math::frames::NED;
use meridian_types::vehicle::FailsafeAction;

pub const MAX_POLYGON_VERTICES: usize = 16;
pub const MAX_ZONES: usize = 8;

#[derive(Debug, Clone, Copy)]
pub struct Point2D { pub n: f32, pub e: f32 }

#[derive(Debug, Clone)]
pub enum FenceZone {
    Cylinder { center: Point2D, radius: f32, alt_min: f32, alt_max: f32, inclusion: bool },
    Polygon { vertices: heapless::Vec<Point2D, MAX_POLYGON_VERTICES>, alt_min: f32, alt_max: f32, inclusion: bool },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BreachType { None, Altitude, Boundary, Both }

pub struct Geofence {
    zones: heapless::Vec<FenceZone, MAX_ZONES>,
    pub action: FailsafeAction,
    pub enabled: bool,
}

impl Geofence {
    pub fn new(action: FailsafeAction) -> Self {
        Self { zones: heapless::Vec::new(), action, enabled: true }
    }

    pub fn add_cylinder(&mut self, cn: f32, ce: f32, r: f32, alt_max: f32, inclusion: bool) {
        let _ = self.zones.push(FenceZone::Cylinder {
            center: Point2D { n: cn, e: ce }, radius: r, alt_min: 0.0, alt_max, inclusion,
        });
    }

    pub fn add_polygon(&mut self, verts: &[Point2D], alt_max: f32, inclusion: bool) {
        let mut v = heapless::Vec::new();
        for p in verts { let _ = v.push(*p); }
        let _ = self.zones.push(FenceZone::Polygon { vertices: v, alt_min: 0.0, alt_max, inclusion });
    }

    pub fn check(&self, pos: &Vec3<NED>, altitude: f32) -> BreachType {
        if !self.enabled { return BreachType::None; }
        let (n, e) = (pos.x, pos.y);
        let (mut bnd, mut alt_b) = (false, false);
        for zone in &self.zones {
            match zone {
                FenceZone::Cylinder { center, radius, alt_min, alt_max, inclusion } => {
                    let dn = n - center.n;
                    let de = e - center.e;
                    let d = libm::sqrtf(dn*dn + de*de);
                    let in_h = d <= *radius;
                    let in_a = altitude >= *alt_min && altitude <= *alt_max;
                    if *inclusion { if !in_h { bnd = true; } if !in_a { alt_b = true; } }
                    else { if in_h && in_a { bnd = true; } }
                }
                FenceZone::Polygon { vertices, alt_min, alt_max, inclusion } => {
                    let inside = point_in_polygon(n, e, vertices);
                    let in_a = altitude >= *alt_min && altitude <= *alt_max;
                    if *inclusion { if !inside { bnd = true; } if !in_a { alt_b = true; } }
                    else { if inside && in_a { bnd = true; } }
                }
            }
        }
        match (bnd, alt_b) {
            (true, true) => BreachType::Both,
            (true, false) => BreachType::Boundary,
            (false, true) => BreachType::Altitude,
            _ => BreachType::None,
        }
    }
}

fn point_in_polygon(n: f32, e: f32, verts: &[Point2D]) -> bool {
    let c = verts.len();
    if c < 3 { return false; }
    let mut inside = false;
    let mut j = c - 1;
    for i in 0..c {
        let (vi, vj) = (&verts[i], &verts[j]);
        if ((vi.e > e) != (vj.e > e)) && (n < (vj.n-vi.n)*(e-vi.e)/(vj.e-vi.e) + vi.n) {
            inside = !inside;
        }
        j = i;
    }
    inside
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cylinder_inclusion() {
        let mut f = Geofence::new(FailsafeAction::ReturnToLaunch);
        f.add_cylinder(0.0, 0.0, 100.0, 50.0, true);
        assert_eq!(f.check(&Vec3::new(50.0, 50.0, -30.0), 30.0), BreachType::None);
        assert_eq!(f.check(&Vec3::new(150.0, 0.0, -30.0), 30.0), BreachType::Boundary);
        assert_eq!(f.check(&Vec3::new(0.0, 0.0, -60.0), 60.0), BreachType::Altitude);
    }

    #[test]
    fn test_cylinder_exclusion() {
        let mut f = Geofence::new(FailsafeAction::ReturnToLaunch);
        f.add_cylinder(100.0, 100.0, 20.0, 200.0, false);
        assert_eq!(f.check(&Vec3::new(0.0, 0.0, -10.0), 10.0), BreachType::None);
        assert_eq!(f.check(&Vec3::new(105.0, 105.0, -10.0), 10.0), BreachType::Boundary);
    }

    #[test]
    fn test_polygon_inclusion() {
        let mut f = Geofence::new(FailsafeAction::Land);
        let poly = [
            Point2D{n:-100.0,e:-100.0}, Point2D{n:100.0,e:-100.0},
            Point2D{n:100.0,e:100.0}, Point2D{n:-100.0,e:100.0},
        ];
        f.add_polygon(&poly, 120.0, true);
        assert_eq!(f.check(&Vec3::new(0.0, 0.0, -50.0), 50.0), BreachType::None);
        assert_eq!(f.check(&Vec3::new(150.0, 0.0, -50.0), 50.0), BreachType::Boundary);
    }

    #[test]
    fn test_point_in_polygon() {
        let tri = [Point2D{n:0.0,e:0.0}, Point2D{n:10.0,e:0.0}, Point2D{n:5.0,e:10.0}];
        assert!(point_in_polygon(5.0, 3.0, &tri));
        assert!(!point_in_polygon(-1.0, -1.0, &tri));
    }

    #[test]
    fn test_disabled() {
        let mut f = Geofence::new(FailsafeAction::Land);
        f.add_cylinder(0.0, 0.0, 10.0, 10.0, true);
        f.enabled = false;
        assert_eq!(f.check(&Vec3::new(999.0, 999.0, 0.0), 999.0), BreachType::None);
    }
}

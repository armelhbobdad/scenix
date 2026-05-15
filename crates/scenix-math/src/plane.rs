use crate::{EPSILON, Ray3, Vec3};

/// A plane defined by a unit normal and signed distance from the origin.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Plane {
    /// Unit normal.
    pub normal: Vec3,
    /// Signed distance term in `normal.dot(point) + distance = 0`.
    pub distance: f32,
}

impl Plane {
    /// Creates a plane from a normal and signed distance.
    #[inline]
    pub fn new(normal: Vec3, distance: f32) -> Self {
        let normal = normal.normalize();
        Self { normal, distance }
    }

    /// Creates a plane through a point with the given normal.
    #[inline]
    pub fn from_normal_and_point(normal: Vec3, point: Vec3) -> Self {
        let normal = normal.normalize();
        Self {
            normal,
            distance: -normal.dot(point),
        }
    }

    /// Creates a plane from three points.
    #[inline]
    pub fn from_three_points(a: Vec3, b: Vec3, c: Vec3) -> Self {
        let normal = (b - a).cross(c - a).normalize();
        Self::from_normal_and_point(normal, a)
    }

    /// Returns the signed distance from a point to the plane.
    #[inline]
    pub fn signed_distance(self, point: Vec3) -> f32 {
        self.normal.dot(point) + self.distance
    }

    /// Projects a point onto the plane.
    #[inline]
    pub fn project_point(self, point: Vec3) -> Vec3 {
        point - self.normal * self.signed_distance(point)
    }

    /// Intersects a ray with the plane and returns non-negative `t`.
    pub fn intersect_ray(self, ray: Ray3) -> Option<f32> {
        let denom = self.normal.dot(ray.direction);
        if denom.abs() <= EPSILON {
            return None;
        }
        let t = -self.signed_distance(ray.origin) / denom;
        if t >= 0.0 { Some(t) } else { None }
    }

    /// Intersects a finite line segment with the plane.
    pub fn intersect_line(self, a: Vec3, b: Vec3) -> Option<Vec3> {
        let ab = b - a;
        let denom = self.normal.dot(ab);
        if denom.abs() <= EPSILON {
            return None;
        }
        let t = -self.signed_distance(a) / denom;
        if (0.0..=1.0).contains(&t) {
            Some(a + ab * t)
        } else {
            None
        }
    }
}

impl Default for Plane {
    #[inline]
    fn default() -> Self {
        Self::from_normal_and_point(Vec3::Y, Vec3::ZERO)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::assert_close;

    #[test]
    fn plane_distance_projection_and_ray_intersection_work() {
        let plane = Plane::from_normal_and_point(Vec3::Y, Vec3::ZERO);
        assert_close(plane.signed_distance(Vec3::new(0.0, 2.0, 0.0)), 2.0);
        assert_eq!(
            plane.project_point(Vec3::new(1.0, 2.0, 3.0)),
            Vec3::new(1.0, 0.0, 3.0)
        );
        let ray = Ray3::new(Vec3::new(0.0, 2.0, 0.0), Vec3::new(0.0, -1.0, 0.0));
        assert_close(plane.intersect_ray(ray).unwrap(), 2.0);
    }

    #[test]
    fn plane_from_three_points_has_expected_normal() {
        let plane = Plane::from_three_points(Vec3::ZERO, Vec3::X, Vec3::Z);
        assert_close(plane.normal.y, -1.0);
    }
}

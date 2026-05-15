use crate::{Mat4, Vec3};

/// An axis-aligned bounding box.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Aabb {
    /// Minimum corner.
    pub min: Vec3,
    /// Maximum corner.
    pub max: Vec3,
}

/// A bounding sphere.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Sphere {
    /// Sphere center.
    pub center: Vec3,
    /// Sphere radius.
    pub radius: f32,
}

impl Aabb {
    /// An empty zero-sized box at the origin.
    pub const ZERO: Self = Self::new(Vec3::ZERO, Vec3::ZERO);

    /// Creates an AABB from minimum and maximum corners.
    #[inline]
    pub const fn new(min: Vec3, max: Vec3) -> Self {
        Self { min, max }
    }

    /// Creates the smallest AABB containing all points.
    pub fn from_points(points: &[Vec3]) -> Self {
        let Some((first, rest)) = points.split_first() else {
            return Self::ZERO;
        };
        let mut min = *first;
        let mut max = *first;
        for point in rest {
            min.x = min.x.min(point.x);
            min.y = min.y.min(point.y);
            min.z = min.z.min(point.z);
            max.x = max.x.max(point.x);
            max.y = max.y.max(point.y);
            max.z = max.z.max(point.z);
        }
        Self::new(min, max)
    }

    /// Returns the center point.
    #[inline]
    pub fn center(self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    /// Returns half the box extents.
    #[inline]
    pub fn half_extents(self) -> Vec3 {
        (self.max - self.min) * 0.5
    }

    /// Returns whether a point is inside or on the boundary.
    #[inline]
    pub fn contains_point(self, point: Vec3) -> bool {
        point.x >= self.min.x
            && point.y >= self.min.y
            && point.z >= self.min.z
            && point.x <= self.max.x
            && point.y <= self.max.y
            && point.z <= self.max.z
    }

    /// Returns whether this box intersects another box.
    #[inline]
    pub fn intersects_aabb(self, other: Self) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }

    /// Returns the merged AABB containing both inputs.
    #[inline]
    pub fn merge(self, other: Self) -> Self {
        Self::new(
            Vec3::new(
                self.min.x.min(other.min.x),
                self.min.y.min(other.min.y),
                self.min.z.min(other.min.z),
            ),
            Vec3::new(
                self.max.x.max(other.max.x),
                self.max.y.max(other.max.y),
                self.max.z.max(other.max.z),
            ),
        )
    }

    /// Conservatively transforms the AABB by transforming all eight corners.
    pub fn transform(self, matrix: Mat4) -> Self {
        let min = self.min;
        let max = self.max;
        let corners = [
            Vec3::new(min.x, min.y, min.z),
            Vec3::new(max.x, min.y, min.z),
            Vec3::new(min.x, max.y, min.z),
            Vec3::new(max.x, max.y, min.z),
            Vec3::new(min.x, min.y, max.z),
            Vec3::new(max.x, min.y, max.z),
            Vec3::new(min.x, max.y, max.z),
            Vec3::new(max.x, max.y, max.z),
        ];
        let mut transformed = [Vec3::ZERO; 8];
        for (out, corner) in transformed.iter_mut().zip(corners) {
            *out = matrix.mul_vec3(corner);
        }
        Self::from_points(&transformed)
    }

    /// Returns the surface area.
    #[inline]
    pub fn surface_area(self) -> f32 {
        let extents = self.max - self.min;
        let x = extents.x.max(0.0);
        let y = extents.y.max(0.0);
        let z = extents.z.max(0.0);
        2.0 * (x * y + y * z + z * x)
    }

    /// Returns a conservative bounding sphere.
    #[inline]
    pub fn bounding_sphere(self) -> Sphere {
        let center = self.center();
        Sphere::new(center, self.max.distance(center))
    }
}

impl Default for Aabb {
    #[inline]
    fn default() -> Self {
        Self::ZERO
    }
}

impl Sphere {
    /// Creates a sphere from a center and radius.
    #[inline]
    pub const fn new(center: Vec3, radius: f32) -> Self {
        Self { center, radius }
    }

    /// Returns whether the point is inside or on the sphere.
    #[inline]
    pub fn contains_point(self, point: Vec3) -> bool {
        point.distance(self.center) <= self.radius
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn aabb_from_points_handles_empty_and_extents() {
        assert_eq!(Aabb::from_points(&[]), Aabb::ZERO);
        let aabb = Aabb::from_points(&[Vec3::new(-1.0, 2.0, 0.0), Vec3::new(3.0, -2.0, 1.0)]);
        assert_eq!(aabb.min, Vec3::new(-1.0, -2.0, 0.0));
        assert_eq!(aabb.max, Vec3::new(3.0, 2.0, 1.0));
        assert_eq!(aabb.center(), Vec3::new(1.0, 0.0, 0.5));
        assert_eq!(aabb.half_extents(), Vec3::new(2.0, 2.0, 0.5));
    }

    #[test]
    fn aabb_contains_intersects_merges_and_measures_area() {
        let a = Aabb::new(Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0));
        let b = Aabb::new(Vec3::new(0.5, 0.5, 0.5), Vec3::new(2.0, 2.0, 2.0));
        assert!(a.contains_point(Vec3::new(0.5, 0.5, 0.5)));
        assert!(a.intersects_aabb(b));
        assert_eq!(a.merge(b).max, Vec3::new(2.0, 2.0, 2.0));
        assert_eq!(a.surface_area(), 6.0);
    }
}

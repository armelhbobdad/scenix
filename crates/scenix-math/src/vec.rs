use core::ops::{
    Add, AddAssign, Div, DivAssign, Index, IndexMut, Mul, MulAssign, Neg, Sub, SubAssign,
};

use crate::{EPSILON, acos, clamp, sqrt};

/// A two-dimensional vector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Vec2 {
    /// X component.
    pub x: f32,
    /// Y component.
    pub y: f32,
}

/// A three-dimensional vector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Vec3 {
    /// X component.
    pub x: f32,
    /// Y component.
    pub y: f32,
    /// Z component.
    pub z: f32,
}

/// A four-dimensional vector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Vec4 {
    /// X component.
    pub x: f32,
    /// Y component.
    pub y: f32,
    /// Z component.
    pub z: f32,
    /// W component.
    pub w: f32,
}

impl Vec2 {
    /// The zero vector.
    pub const ZERO: Self = Self::new(0.0, 0.0);
    /// A vector with all components set to one.
    pub const ONE: Self = Self::new(1.0, 1.0);
    /// Unit X.
    pub const X: Self = Self::new(1.0, 0.0);
    /// Unit Y.
    pub const Y: Self = Self::new(0.0, 1.0);

    /// Creates a vector from components.
    #[inline]
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    /// Returns the dot product.
    #[inline]
    pub fn dot(self, rhs: Self) -> f32 {
        self.x * rhs.x + self.y * rhs.y
    }

    /// Returns the squared length.
    #[inline]
    pub fn length_squared(self) -> f32 {
        self.dot(self)
    }

    /// Returns the length.
    #[inline]
    pub fn length(self) -> f32 {
        sqrt(self.length_squared())
    }

    /// Returns a normalized vector, or zero for a near-zero input.
    #[inline]
    pub fn normalize(self) -> Self {
        let length = self.length();
        if length <= EPSILON {
            Self::ZERO
        } else {
            self / length
        }
    }

    /// Linearly interpolates toward `rhs`.
    #[inline]
    pub fn lerp(self, rhs: Self, t: f32) -> Self {
        self + (rhs - self) * t
    }

    /// Returns the distance to `rhs`.
    #[inline]
    pub fn distance(self, rhs: Self) -> f32 {
        (rhs - self).length()
    }

    /// Returns the angle between vectors in radians.
    #[inline]
    pub fn angle_between(self, rhs: Self) -> f32 {
        let denom = self.length() * rhs.length();
        if denom <= EPSILON {
            0.0
        } else {
            acos(clamp(self.dot(rhs) / denom, -1.0, 1.0))
        }
    }

    /// Returns the vector as an array.
    #[inline]
    pub const fn to_array(self) -> [f32; 2] {
        [self.x, self.y]
    }
}

impl Vec3 {
    /// The zero vector.
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);
    /// A vector with all components set to one.
    pub const ONE: Self = Self::new(1.0, 1.0, 1.0);
    /// Unit X.
    pub const X: Self = Self::new(1.0, 0.0, 0.0);
    /// Unit Y.
    pub const Y: Self = Self::new(0.0, 1.0, 0.0);
    /// Unit Z.
    pub const Z: Self = Self::new(0.0, 0.0, 1.0);
    /// Negative unit Z.
    pub const NEG_Z: Self = Self::new(0.0, 0.0, -1.0);
    /// World-space up.
    pub const UP: Self = Self::Y;

    /// Creates a vector from components.
    #[inline]
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Returns the dot product.
    #[inline]
    pub fn dot(self, rhs: Self) -> f32 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    /// Returns the cross product.
    #[inline]
    pub fn cross(self, rhs: Self) -> Self {
        Self::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        )
    }

    /// Returns the squared length.
    #[inline]
    pub fn length_squared(self) -> f32 {
        self.dot(self)
    }

    /// Returns the length.
    #[inline]
    pub fn length(self) -> f32 {
        sqrt(self.length_squared())
    }

    /// Returns a normalized vector, or zero for a near-zero input.
    #[inline]
    pub fn normalize(self) -> Self {
        let length = self.length();
        if length <= EPSILON {
            Self::ZERO
        } else {
            self / length
        }
    }

    /// Linearly interpolates toward `rhs`.
    #[inline]
    pub fn lerp(self, rhs: Self, t: f32) -> Self {
        self + (rhs - self) * t
    }

    /// Returns the distance to `rhs`.
    #[inline]
    pub fn distance(self, rhs: Self) -> f32 {
        (rhs - self).length()
    }

    /// Reflects this vector around a normal.
    #[inline]
    pub fn reflect(self, normal: Self) -> Self {
        self - normal * (2.0 * self.dot(normal))
    }

    /// Returns the angle between vectors in radians.
    #[inline]
    pub fn angle_between(self, rhs: Self) -> f32 {
        let denom = self.length() * rhs.length();
        if denom <= EPSILON {
            0.0
        } else {
            acos(clamp(self.dot(rhs) / denom, -1.0, 1.0))
        }
    }

    /// Multiplies components pairwise.
    #[inline]
    pub fn mul_elements(self, rhs: Self) -> Self {
        Self::new(self.x * rhs.x, self.y * rhs.y, self.z * rhs.z)
    }

    /// Divides components pairwise. Zero divisors produce zero in that lane.
    #[inline]
    pub fn div_elements(self, rhs: Self) -> Self {
        Self::new(
            if rhs.x.abs() <= EPSILON {
                0.0
            } else {
                self.x / rhs.x
            },
            if rhs.y.abs() <= EPSILON {
                0.0
            } else {
                self.y / rhs.y
            },
            if rhs.z.abs() <= EPSILON {
                0.0
            } else {
                self.z / rhs.z
            },
        )
    }

    /// Returns the vector as an array.
    #[inline]
    pub const fn to_array(self) -> [f32; 3] {
        [self.x, self.y, self.z]
    }
}

impl Vec4 {
    /// The zero vector.
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0, 0.0);
    /// A vector with all components set to one.
    pub const ONE: Self = Self::new(1.0, 1.0, 1.0, 1.0);
    /// Unit X.
    pub const X: Self = Self::new(1.0, 0.0, 0.0, 0.0);
    /// Unit Y.
    pub const Y: Self = Self::new(0.0, 1.0, 0.0, 0.0);
    /// Unit Z.
    pub const Z: Self = Self::new(0.0, 0.0, 1.0, 0.0);
    /// Unit W.
    pub const W: Self = Self::new(0.0, 0.0, 0.0, 1.0);

    /// Creates a vector from components.
    #[inline]
    pub const fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { x, y, z, w }
    }

    /// Returns the dot product.
    #[inline]
    pub fn dot(self, rhs: Self) -> f32 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z + self.w * rhs.w
    }

    /// Returns the squared length.
    #[inline]
    pub fn length_squared(self) -> f32 {
        self.dot(self)
    }

    /// Returns the length.
    #[inline]
    pub fn length(self) -> f32 {
        sqrt(self.length_squared())
    }

    /// Returns a normalized vector, or zero for a near-zero input.
    #[inline]
    pub fn normalize(self) -> Self {
        let length = self.length();
        if length <= EPSILON {
            Self::ZERO
        } else {
            self / length
        }
    }

    /// Linearly interpolates toward `rhs`.
    #[inline]
    pub fn lerp(self, rhs: Self, t: f32) -> Self {
        self + (rhs - self) * t
    }

    /// Truncates to a `Vec3`.
    #[inline]
    pub const fn truncate(self) -> Vec3 {
        Vec3::new(self.x, self.y, self.z)
    }

    /// Returns the vector as an array.
    #[inline]
    pub const fn to_array(self) -> [f32; 4] {
        [self.x, self.y, self.z, self.w]
    }
}

macro_rules! impl_vec_ops {
    ($type:ident { $($field:ident),+ }) => {
        impl Add for $type {
            type Output = Self;
            #[inline]
            fn add(self, rhs: Self) -> Self::Output {
                Self::new($(self.$field + rhs.$field),+)
            }
        }

        impl AddAssign for $type {
            #[inline]
            fn add_assign(&mut self, rhs: Self) {
                $(self.$field += rhs.$field;)+
            }
        }

        impl Sub for $type {
            type Output = Self;
            #[inline]
            fn sub(self, rhs: Self) -> Self::Output {
                Self::new($(self.$field - rhs.$field),+)
            }
        }

        impl SubAssign for $type {
            #[inline]
            fn sub_assign(&mut self, rhs: Self) {
                $(self.$field -= rhs.$field;)+
            }
        }

        impl Mul<f32> for $type {
            type Output = Self;
            #[inline]
            fn mul(self, rhs: f32) -> Self::Output {
                Self::new($(self.$field * rhs),+)
            }
        }

        impl Mul<$type> for f32 {
            type Output = $type;
            #[inline]
            fn mul(self, rhs: $type) -> Self::Output {
                rhs * self
            }
        }

        impl MulAssign<f32> for $type {
            #[inline]
            fn mul_assign(&mut self, rhs: f32) {
                $(self.$field *= rhs;)+
            }
        }

        impl Div<f32> for $type {
            type Output = Self;
            #[inline]
            fn div(self, rhs: f32) -> Self::Output {
                if rhs.abs() <= EPSILON {
                    Self::ZERO
                } else {
                    Self::new($(self.$field / rhs),+)
                }
            }
        }

        impl DivAssign<f32> for $type {
            #[inline]
            fn div_assign(&mut self, rhs: f32) {
                *self = *self / rhs;
            }
        }

        impl Neg for $type {
            type Output = Self;
            #[inline]
            fn neg(self) -> Self::Output {
                Self::new($(-self.$field),+)
            }
        }

    };
}

impl_vec_ops!(Vec2 { x, y });
impl_vec_ops!(Vec3 { x, y, z });
impl_vec_ops!(Vec4 { x, y, z, w });

impl Index<usize> for Vec2 {
    type Output = f32;

    #[inline]
    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self.x,
            1 => &self.y,
            _ => panic!("vector index out of bounds: {index} >= 2"),
        }
    }
}

impl IndexMut<usize> for Vec2 {
    #[inline]
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            _ => panic!("vector index out of bounds: {index} >= 2"),
        }
    }
}

impl Index<usize> for Vec3 {
    type Output = f32;

    #[inline]
    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("vector index out of bounds: {index} >= 3"),
        }
    }
}

impl IndexMut<usize> for Vec3 {
    #[inline]
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("vector index out of bounds: {index} >= 3"),
        }
    }
}

impl Index<usize> for Vec4 {
    type Output = f32;

    #[inline]
    fn index(&self, index: usize) -> &Self::Output {
        match index {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            3 => &self.w,
            _ => panic!("vector index out of bounds: {index} >= 4"),
        }
    }
}

impl IndexMut<usize> for Vec4 {
    #[inline]
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            3 => &mut self.w,
            _ => panic!("vector index out of bounds: {index} >= 4"),
        }
    }
}

#[cfg(feature = "approx")]
macro_rules! impl_approx {
    ($type:ident { $($field:ident),+ }) => {
        impl approx::AbsDiffEq for $type {
            type Epsilon = f32;

            #[inline]
            fn default_epsilon() -> Self::Epsilon {
                f32::default_epsilon()
            }

            #[inline]
            fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
                $(f32::abs_diff_eq(&self.$field, &other.$field, epsilon))&&+
            }
        }

        impl approx::RelativeEq for $type {
            #[inline]
            fn default_max_relative() -> Self::Epsilon {
                f32::default_max_relative()
            }

            #[inline]
            fn relative_eq(
                &self,
                other: &Self,
                epsilon: Self::Epsilon,
                max_relative: Self::Epsilon,
            ) -> bool {
                $(f32::relative_eq(&self.$field, &other.$field, epsilon, max_relative))&&+
            }
        }

        impl approx::UlpsEq for $type {
            #[inline]
            fn default_max_ulps() -> u32 {
                f32::default_max_ulps()
            }

            #[inline]
            fn ulps_eq(&self, other: &Self, epsilon: Self::Epsilon, max_ulps: u32) -> bool {
                $(f32::ulps_eq(&self.$field, &other.$field, epsilon, max_ulps))&&+
            }
        }
    };
}

#[cfg(feature = "approx")]
impl_approx!(Vec2 { x, y });
#[cfg(feature = "approx")]
impl_approx!(Vec3 { x, y, z });
#[cfg(feature = "approx")]
impl_approx!(Vec4 { x, y, z, w });

#[cfg(test)]
mod tests {
    use super::*;
    use crate::assert_close;

    #[test]
    fn vec3_cross_dot_normalize_and_lerp_work() {
        let x = Vec3::X;
        let y = Vec3::Y;
        assert_eq!(x.cross(y), Vec3::Z);
        assert_close(x.dot(y), 0.0);
        assert_close(Vec3::new(3.0, 4.0, 0.0).normalize().length(), 1.0);
        assert_eq!(Vec3::ZERO.normalize(), Vec3::ZERO);
        assert_eq!(x.lerp(y, 0.5), Vec3::new(0.5, 0.5, 0.0));
    }

    #[test]
    fn angle_between_handles_normal_and_zero_vectors() {
        assert_close(Vec3::X.angle_between(Vec3::Y), core::f32::consts::FRAC_PI_2);
        assert_close(Vec3::ZERO.angle_between(Vec3::Y), 0.0);
    }

    #[test]
    fn reflect_mirrors_about_normal() {
        let reflected = Vec3::new(1.0, -1.0, 0.0).reflect(Vec3::Y);
        assert_eq!(reflected, Vec3::new(1.0, 1.0, 0.0));
    }
}

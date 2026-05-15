use crate::{Mat4, Quat, asin, atan2, clamp, cos, sin};

/// Rotation order for Euler angles.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum RotationOrder {
    /// X, then Y, then Z.
    #[default]
    XYZ,
    /// Y, then X, then Z.
    YXZ,
    /// Z, then X, then Y.
    ZXY,
    /// Z, then Y, then X.
    ZYX,
    /// Y, then Z, then X.
    YZX,
    /// X, then Z, then Y.
    XZY,
}

/// Euler angles in radians plus a rotation order.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Euler {
    /// Rotation around X in radians.
    pub x: f32,
    /// Rotation around Y in radians.
    pub y: f32,
    /// Rotation around Z in radians.
    pub z: f32,
    /// Rotation order.
    pub order: RotationOrder,
}

impl Euler {
    /// Creates Euler angles from components.
    #[inline]
    pub const fn new(x: f32, y: f32, z: f32, order: RotationOrder) -> Self {
        Self { x, y, z, order }
    }

    /// Extracts Euler angles from a quaternion.
    #[inline]
    pub fn from_quat(q: Quat, order: RotationOrder) -> Self {
        Self::from_mat4(q.to_mat4(), order)
    }

    /// Extracts Euler angles from a rotation matrix.
    pub fn from_mat4(matrix: Mat4, order: RotationOrder) -> Self {
        let m11 = matrix.get(0, 0);
        let m12 = matrix.get(0, 1);
        let m13 = matrix.get(0, 2);
        let m21 = matrix.get(1, 0);
        let m22 = matrix.get(1, 1);
        let m23 = matrix.get(1, 2);
        let m31 = matrix.get(2, 0);
        let m32 = matrix.get(2, 1);
        let m33 = matrix.get(2, 2);

        let (x, y, z) = match order {
            RotationOrder::XYZ => {
                let y = asin(clamp(m13, -1.0, 1.0));
                if m13.abs() < 0.999_999_9 {
                    (atan2(-m23, m33), y, atan2(-m12, m11))
                } else {
                    (atan2(m32, m22), y, 0.0)
                }
            }
            RotationOrder::YXZ => {
                let x = asin(-clamp(m23, -1.0, 1.0));
                if m23.abs() < 0.999_999_9 {
                    (x, atan2(m13, m33), atan2(m21, m22))
                } else {
                    (x, atan2(-m31, m11), 0.0)
                }
            }
            RotationOrder::ZXY => {
                let x = asin(clamp(m32, -1.0, 1.0));
                if m32.abs() < 0.999_999_9 {
                    (x, atan2(-m31, m33), atan2(-m12, m22))
                } else {
                    (x, 0.0, atan2(m21, m11))
                }
            }
            RotationOrder::ZYX => {
                let y = asin(-clamp(m31, -1.0, 1.0));
                if m31.abs() < 0.999_999_9 {
                    (atan2(m32, m33), y, atan2(m21, m11))
                } else {
                    (0.0, y, atan2(-m12, m22))
                }
            }
            RotationOrder::YZX => {
                let z = asin(clamp(m21, -1.0, 1.0));
                if m21.abs() < 0.999_999_9 {
                    (atan2(-m23, m22), atan2(-m31, m11), z)
                } else {
                    (0.0, atan2(m13, m33), z)
                }
            }
            RotationOrder::XZY => {
                let z = asin(-clamp(m12, -1.0, 1.0));
                if m12.abs() < 0.999_999_9 {
                    (atan2(m32, m22), atan2(m13, m11), z)
                } else {
                    (atan2(-m23, m33), 0.0, z)
                }
            }
        };

        Self::new(x, y, z, order)
    }

    /// Converts these Euler angles to a quaternion.
    pub fn to_quat(self) -> Quat {
        let c1 = cos(self.x * 0.5);
        let c2 = cos(self.y * 0.5);
        let c3 = cos(self.z * 0.5);
        let s1 = sin(self.x * 0.5);
        let s2 = sin(self.y * 0.5);
        let s3 = sin(self.z * 0.5);

        match self.order {
            RotationOrder::XYZ => Quat::new(
                s1 * c2 * c3 + c1 * s2 * s3,
                c1 * s2 * c3 - s1 * c2 * s3,
                c1 * c2 * s3 + s1 * s2 * c3,
                c1 * c2 * c3 - s1 * s2 * s3,
            ),
            RotationOrder::YXZ => Quat::new(
                s1 * c2 * c3 + c1 * s2 * s3,
                c1 * s2 * c3 - s1 * c2 * s3,
                c1 * c2 * s3 - s1 * s2 * c3,
                c1 * c2 * c3 + s1 * s2 * s3,
            ),
            RotationOrder::ZXY => Quat::new(
                s1 * c2 * c3 - c1 * s2 * s3,
                c1 * s2 * c3 + s1 * c2 * s3,
                c1 * c2 * s3 + s1 * s2 * c3,
                c1 * c2 * c3 - s1 * s2 * s3,
            ),
            RotationOrder::ZYX => Quat::new(
                s1 * c2 * c3 - c1 * s2 * s3,
                c1 * s2 * c3 + s1 * c2 * s3,
                c1 * c2 * s3 - s1 * s2 * c3,
                c1 * c2 * c3 + s1 * s2 * s3,
            ),
            RotationOrder::YZX => Quat::new(
                s1 * c2 * c3 + c1 * s2 * s3,
                c1 * s2 * c3 + s1 * c2 * s3,
                c1 * c2 * s3 - s1 * s2 * c3,
                c1 * c2 * c3 - s1 * s2 * s3,
            ),
            RotationOrder::XZY => Quat::new(
                s1 * c2 * c3 - c1 * s2 * s3,
                c1 * s2 * c3 - s1 * c2 * s3,
                c1 * c2 * s3 + s1 * s2 * c3,
                c1 * c2 * c3 + s1 * s2 * s3,
            ),
        }
        .normalize()
    }
}

impl Default for Euler {
    #[inline]
    fn default() -> Self {
        Self::new(0.0, 0.0, 0.0, RotationOrder::XYZ)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Vec3, assert_close};

    #[test]
    fn xyz_round_trips_through_quat() {
        let euler = Euler::new(0.3, 0.4, 0.5, RotationOrder::XYZ);
        let out = Euler::from_quat(euler.to_quat(), RotationOrder::XYZ);
        assert_close(out.x, euler.x);
        assert_close(out.y, euler.y);
        assert_close(out.z, euler.z);
    }

    #[test]
    fn euler_rotates_vector() {
        let q = Euler::new(0.0, core::f32::consts::FRAC_PI_2, 0.0, RotationOrder::XYZ).to_quat();
        let out = q.mul_vec3(Vec3::X);
        assert_close(out.z, -1.0);
    }
}

use crate::{Mat4, Quat, Vec3};

/// A translation, rotation, and scale transform.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Transform {
    /// Local translation.
    pub translation: Vec3,
    /// Local rotation.
    pub rotation: Quat,
    /// Local scale.
    pub scale: Vec3,
}

impl Transform {
    /// Identity transform.
    pub const IDENTITY: Self = Self::new(Vec3::ZERO, Quat::IDENTITY, Vec3::ONE);

    /// Creates a transform from translation, rotation, and scale.
    #[inline]
    pub const fn new(translation: Vec3, rotation: Quat, scale: Vec3) -> Self {
        Self {
            translation,
            rotation,
            scale,
        }
    }

    /// Creates a translation transform.
    #[inline]
    pub const fn from_translation(value: Vec3) -> Self {
        Self::new(value, Quat::IDENTITY, Vec3::ONE)
    }

    /// Creates a rotation transform.
    #[inline]
    pub const fn from_rotation(value: Quat) -> Self {
        Self::new(Vec3::ZERO, value, Vec3::ONE)
    }

    /// Creates a scale transform.
    #[inline]
    pub const fn from_scale(value: Vec3) -> Self {
        Self::new(Vec3::ZERO, Quat::IDENTITY, value)
    }

    /// Creates a transform that looks from `eye` toward `target`.
    pub fn looking_at(eye: Vec3, target: Vec3, up: Vec3) -> Self {
        let view = Mat4::look_at(eye, target, up);
        let world = view.inverse().unwrap_or(Mat4::from_translation(eye));
        world.decompose().unwrap_or(Self::from_translation(eye))
    }

    /// Converts the transform to a column-major matrix.
    #[inline]
    pub fn to_mat4(self) -> Mat4 {
        Mat4::from_trs(self.translation, self.rotation, self.scale)
    }

    /// Creates a transform from a TRS matrix.
    #[inline]
    pub fn from_mat4(matrix: Mat4) -> Option<Self> {
        matrix.decompose()
    }

    /// Composes two transforms.
    #[inline]
    pub fn mul_transform(self, rhs: Self) -> Self {
        Self::from_mat4(self.to_mat4() * rhs.to_mat4()).unwrap_or(Self::IDENTITY)
    }

    /// Returns the inverse transform.
    #[inline]
    pub fn inverse(self) -> Self {
        self.to_mat4()
            .inverse()
            .and_then(Self::from_mat4)
            .unwrap_or(Self::IDENTITY)
    }

    /// Returns the transformed forward direction (`-Z`).
    #[inline]
    pub fn forward(self) -> Vec3 {
        self.rotation.mul_vec3(Vec3::NEG_Z).normalize()
    }

    /// Returns the transformed right direction (`+X`).
    #[inline]
    pub fn right(self) -> Vec3 {
        self.rotation.mul_vec3(Vec3::X).normalize()
    }

    /// Returns the transformed up direction (`+Y`).
    #[inline]
    pub fn up(self) -> Vec3 {
        self.rotation.mul_vec3(Vec3::Y).normalize()
    }

    /// Returns this transform translated by `delta`.
    #[inline]
    pub fn translate_by(mut self, delta: Vec3) -> Self {
        self.translation += delta;
        self
    }

    /// Returns this transform rotated by `rotation`.
    #[inline]
    pub fn rotate_by(mut self, rotation: Quat) -> Self {
        self.rotation = (self.rotation * rotation).normalize();
        self
    }

    /// Returns this transform scaled component-wise by `scale`.
    #[inline]
    pub fn scale_by(mut self, scale: Vec3) -> Self {
        self.scale = self.scale.mul_elements(scale);
        self
    }
}

impl Default for Transform {
    #[inline]
    fn default() -> Self {
        Self::IDENTITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::assert_close;

    #[test]
    fn transform_matrix_decomposes_back_to_trs() {
        let transform = Transform::new(
            Vec3::new(1.0, 2.0, 3.0),
            Quat::from_axis_angle(Vec3::Y, 0.8),
            Vec3::new(2.0, 2.0, 2.0),
        );
        let out = Transform::from_mat4(transform.to_mat4()).unwrap();
        assert_close(out.translation.x, transform.translation.x);
        assert_close(out.translation.y, transform.translation.y);
        assert_close(out.translation.z, transform.translation.z);
        assert_close(out.scale.x, transform.scale.x);
        assert_close(out.rotation.angle_between(transform.rotation), 0.0);
    }

    #[test]
    fn inverse_undoes_transform() {
        let transform = Transform::new(
            Vec3::new(1.0, 2.0, 3.0),
            Quat::from_axis_angle(Vec3::Y, 0.6),
            Vec3::new(2.0, 2.0, 2.0),
        );
        let point = Vec3::new(4.0, 5.0, 6.0);
        let moved = transform.to_mat4().mul_vec3(point);
        let restored = transform.inverse().to_mat4().mul_vec3(moved);
        assert_close(restored.x, point.x);
        assert_close(restored.y, point.y);
        assert_close(restored.z, point.z);
    }

    #[test]
    fn direction_vectors_follow_rotation() {
        let transform =
            Transform::from_rotation(Quat::from_axis_angle(Vec3::Y, core::f32::consts::FRAC_PI_2));
        assert_close(transform.forward().x, -1.0);
        assert_close(transform.right().z, -1.0);
        assert_close(transform.up().y, 1.0);
    }
}

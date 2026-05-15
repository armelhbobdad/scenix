use crate::{EPSILON, Vec3, acos, atan2, clamp, cos, sin};

/// Spherical coordinates using Y as the polar axis.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Spherical {
    /// Radius from origin.
    pub radius: f32,
    /// Polar angle from positive Y.
    pub phi: f32,
    /// Azimuth angle in the XZ plane.
    pub theta: f32,
}

impl Spherical {
    /// Creates spherical coordinates.
    #[inline]
    pub const fn new(radius: f32, phi: f32, theta: f32) -> Self {
        Self { radius, phi, theta }
    }

    /// Converts from a vector.
    pub fn from_vec3(value: Vec3) -> Self {
        let radius = value.length();
        if radius <= EPSILON {
            return Self::default();
        }
        Self::new(
            radius,
            acos(clamp(value.y / radius, -1.0, 1.0)),
            atan2(value.x, value.z),
        )
    }

    /// Converts to a vector.
    pub fn to_vec3(self) -> Vec3 {
        let sin_phi = sin(self.phi);
        Vec3::new(
            self.radius * sin_phi * sin(self.theta),
            self.radius * cos(self.phi),
            self.radius * sin_phi * cos(self.theta),
        )
    }

    /// Clamps the polar angle.
    #[inline]
    pub fn clamp_phi(mut self, min: f32, max: f32) -> Self {
        self.phi = clamp(self.phi, min, max);
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::assert_close;

    #[test]
    fn spherical_round_trips_vec3() {
        let value = Vec3::new(2.0, 3.0, 4.0);
        let out = Spherical::from_vec3(value).to_vec3();
        assert_close(out.x, value.x);
        assert_close(out.y, value.y);
        assert_close(out.z, value.z);
    }
}

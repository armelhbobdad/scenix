/// The transfer function used by color values.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ColorSpace {
    /// Linear light values.
    #[default]
    Linear,
    /// sRGB encoded values.
    Srgb,
}

/// RGBA color with `f32` channels.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Color {
    /// Red channel.
    pub r: f32,
    /// Green channel.
    pub g: f32,
    /// Blue channel.
    pub b: f32,
    /// Alpha channel.
    pub a: f32,
}

impl Color {
    /// Opaque white.
    pub const WHITE: Self = Self::rgba(1.0, 1.0, 1.0, 1.0);
    /// Opaque black.
    pub const BLACK: Self = Self::rgba(0.0, 0.0, 0.0, 1.0);
    /// Transparent black.
    pub const TRANSPARENT: Self = Self::rgba(0.0, 0.0, 0.0, 0.0);
    /// Opaque red.
    pub const RED: Self = Self::rgba(1.0, 0.0, 0.0, 1.0);
    /// Opaque green.
    pub const GREEN: Self = Self::rgba(0.0, 1.0, 0.0, 1.0);
    /// Opaque blue.
    pub const BLUE: Self = Self::rgba(0.0, 0.0, 1.0, 1.0);

    /// Creates an opaque RGB color.
    #[inline]
    pub const fn rgb(r: f32, g: f32, b: f32) -> Self {
        Self::rgba(r, g, b, 1.0)
    }

    /// Creates an opaque RGB color.
    #[inline]
    pub const fn from_rgb(r: f32, g: f32, b: f32) -> Self {
        Self::rgb(r, g, b)
    }

    /// Creates an RGBA color.
    #[inline]
    pub const fn rgba(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }

    /// Creates an RGBA color.
    #[inline]
    pub const fn from_rgba(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self::rgba(r, g, b, a)
    }

    /// Parses `0xRRGGBB` or `0xRRGGBBAA`.
    pub fn hex(hex: u32) -> Self {
        Self::from_hex(hex)
    }

    /// Parses `0xRRGGBB` or `0xRRGGBBAA`.
    pub fn from_hex(hex: u32) -> Self {
        if hex <= 0x00FF_FFFF {
            Self::from_srgb_u8(
                ((hex >> 16) & 0xFF) as u8,
                ((hex >> 8) & 0xFF) as u8,
                (hex & 0xFF) as u8,
            )
        } else {
            Self::rgba(
                ((hex >> 24) & 0xFF) as f32 / 255.0,
                ((hex >> 16) & 0xFF) as f32 / 255.0,
                ((hex >> 8) & 0xFF) as f32 / 255.0,
                (hex & 0xFF) as f32 / 255.0,
            )
        }
    }

    /// Creates a color from sRGB bytes without linearizing them.
    #[inline]
    pub fn from_srgb_u8(r: u8, g: u8, b: u8) -> Self {
        Self::rgb(r as f32 / 255.0, g as f32 / 255.0, b as f32 / 255.0)
    }

    /// Converts sRGB encoded RGB channels to linear light.
    pub fn to_linear(self) -> Self {
        Self::rgba(
            srgb_to_linear(self.r),
            srgb_to_linear(self.g),
            srgb_to_linear(self.b),
            self.a,
        )
    }

    /// Converts linear light RGB channels to sRGB.
    pub fn to_srgb(self) -> Self {
        Self::rgba(
            linear_to_srgb(self.r),
            linear_to_srgb(self.g),
            linear_to_srgb(self.b),
            self.a,
        )
    }

    /// Linearly interpolates toward `rhs`.
    #[inline]
    pub fn lerp(self, rhs: Self, t: f32) -> Self {
        Self::rgba(
            self.r + (rhs.r - self.r) * t,
            self.g + (rhs.g - self.g) * t,
            self.b + (rhs.b - self.b) * t,
            self.a + (rhs.a - self.a) * t,
        )
    }

    /// Returns `[r, g, b, a]`.
    #[inline]
    pub const fn to_array(self) -> [f32; 4] {
        [self.r, self.g, self.b, self.a]
    }

    /// Encodes the color as `0xRRGGBBAA`.
    pub fn to_hex_rgba(self) -> u32 {
        let r = channel_to_u8(self.r) as u32;
        let g = channel_to_u8(self.g) as u32;
        let b = channel_to_u8(self.b) as u32;
        let a = channel_to_u8(self.a) as u32;
        (r << 24) | (g << 16) | (b << 8) | a
    }
}

impl Default for Color {
    #[inline]
    fn default() -> Self {
        Self::WHITE
    }
}

#[inline]
fn channel_to_u8(value: f32) -> u8 {
    (clamp01(value) * 255.0 + 0.5) as u8
}

#[inline]
fn srgb_to_linear(value: f32) -> f32 {
    let value = clamp01(value);
    if value <= 0.04045 {
        value / 12.92
    } else {
        #[cfg(feature = "std")]
        {
            ((value + 0.055) / 1.055).powf(2.4)
        }
        #[cfg(not(feature = "std"))]
        {
            let value = (value + 0.055) / 1.055;
            value * value
        }
    }
}

#[inline]
fn linear_to_srgb(value: f32) -> f32 {
    let value = clamp01(value);
    if value <= 0.003_130_8 {
        value * 12.92
    } else {
        #[cfg(feature = "std")]
        {
            1.055 * value.powf(1.0 / 2.4) - 0.055
        }
        #[cfg(not(feature = "std"))]
        {
            1.055 * sqrt(value) - 0.055
        }
    }
}

#[inline]
fn clamp01(value: f32) -> f32 {
    value.clamp(0.0, 1.0)
}

#[cfg(not(feature = "std"))]
#[inline]
fn sqrt(value: f32) -> f32 {
    if value <= 0.0 {
        return 0.0;
    }
    let mut x = if value >= 1.0 { value } else { 1.0 };
    let mut i = 0;
    while i < 8 {
        x = 0.5 * (x + value / x);
        i += 1;
    }
    x
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f32, b: f32) {
        assert!((a - b).abs() <= 1.0e-4, "{a} != {b}");
    }

    #[test]
    fn color_hex_parses_rgb_and_rgba() {
        assert_eq!(Color::from_hex(0xFF_80_00).to_hex_rgba(), 0xFF_80_00_FF);
        assert_eq!(Color::from_hex(0xFF_80_00_7F).to_hex_rgba(), 0xFF_80_00_7F);
    }

    #[test]
    fn srgb_linear_conversion_round_trips() {
        let color = Color::rgb(0.2, 0.5, 0.8);
        let out = color.to_linear().to_srgb();
        close(out.r, color.r);
        close(out.g, color.g);
        close(out.b, color.b);
    }

    #[test]
    fn constants_and_lerp_work() {
        assert_eq!(
            Color::RED.lerp(Color::BLUE, 0.5),
            Color::rgba(0.5, 0.0, 0.5, 1.0)
        );
        assert_eq!(Color::TRANSPARENT.a, 0.0);
    }
}

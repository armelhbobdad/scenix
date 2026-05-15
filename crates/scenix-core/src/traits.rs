use scenix_math::{Aabb, Vec3};

/// A value that can provide a stable render order.
pub trait Renderable {
    /// Lower values render earlier.
    fn render_order(&self) -> u32;
}

/// A value that can report conservative bounds.
pub trait Bounded {
    /// Returns an axis-aligned bounding box.
    fn aabb(&self) -> Aabb;

    /// Returns `(center, radius)` for a conservative bounding sphere.
    fn bounding_sphere(&self) -> (Vec3, f32);
}

/// Converts CPU-side data into a plain GPU upload representation.
#[cfg(feature = "gpu")]
pub trait GpuUpload {
    /// Plain-old-data representation suitable for GPU buffers.
    type GpuData: bytemuck::Pod;

    /// Converts the value into GPU data.
    fn to_gpu(&self) -> Self::GpuData;
}

/// A named object.
#[cfg(feature = "std")]
pub trait Named {
    /// Returns the current name.
    fn name(&self) -> &str;

    /// Sets the current name.
    fn set_name(&mut self, name: impl Into<String>);
}

#![cfg_attr(not(feature = "std"), no_std)]

//! Shared Foundation types for scenix.

pub mod color;
pub mod error;
pub mod ids;
pub mod traits;

pub use color::{Color, ColorSpace};
pub use error::{GpuError, LoadError, ScenixError, ValidationError};
pub use ids::{CameraId, LightId, MaterialId, MeshId, NodeId, TextureId};
pub use traits::{Bounded, Renderable};

#[cfg(feature = "gpu")]
pub use traits::GpuUpload;
#[cfg(feature = "std")]
pub use traits::Named;

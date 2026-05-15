#![cfg_attr(not(feature = "std"), no_std)]

//! Facade crate for scenix v0.1.0 Foundation APIs.
//!
//! This release re-exports `scenix-math`, `scenix-core`, and `scenix-input`.
//! Rendering, scene graph, mesh, material, loader, and WASM crates are planned
//! for later roadmap milestones.

pub use scenix_core::*;
pub use scenix_input::*;
pub use scenix_math::*;

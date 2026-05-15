#![cfg_attr(not(feature = "std"), no_std)]

//! Platform-agnostic input state for scenix.

pub mod keyboard;
pub mod pointer;

pub use keyboard::{KeyCode, KeyboardState, Modifiers};
pub use pointer::{PointerButton, PointerState};

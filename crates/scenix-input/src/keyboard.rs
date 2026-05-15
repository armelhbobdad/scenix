/// Keyboard modifier state.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Modifiers {
    /// Shift is active.
    pub shift: bool,
    /// Control is active.
    pub ctrl: bool,
    /// Alt is active.
    pub alt: bool,
    /// Meta, command, or Windows key is active.
    pub meta: bool,
}

/// Portable key codes used by scenix controllers.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(u8)]
pub enum KeyCode {
    /// W key.
    KeyW = 0,
    /// A key.
    KeyA = 1,
    /// S key.
    KeyS = 2,
    /// D key.
    KeyD = 3,
    /// Q key.
    KeyQ = 4,
    /// E key.
    KeyE = 5,
    /// Space key.
    Space = 6,
    /// Left shift key.
    ShiftLeft = 7,
    /// Right shift key.
    ShiftRight = 8,
    /// Left control key.
    ControlLeft = 9,
    /// Right control key.
    ControlRight = 10,
    /// Left alt key.
    AltLeft = 11,
    /// Right alt key.
    AltRight = 12,
    /// Left meta key.
    MetaLeft = 13,
    /// Right meta key.
    MetaRight = 14,
    /// Up arrow.
    ArrowUp = 15,
    /// Down arrow.
    ArrowDown = 16,
    /// Left arrow.
    ArrowLeft = 17,
    /// Right arrow.
    ArrowRight = 18,
    /// Escape key.
    Escape = 19,
    /// Enter key.
    Enter = 20,
    /// Tab key.
    Tab = 21,
}

impl KeyCode {
    #[inline]
    const fn bit(self) -> u128 {
        1_u128 << (self as u8)
    }
}

/// Fixed-size keyboard state suitable for `no_std`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct KeyboardState {
    pressed: u128,
    modifiers: Modifiers,
}

impl KeyboardState {
    /// Creates an empty keyboard state.
    #[inline]
    pub const fn new() -> Self {
        Self {
            pressed: 0,
            modifiers: Modifiers {
                shift: false,
                ctrl: false,
                alt: false,
                meta: false,
            },
        }
    }

    /// Returns whether a key is currently pressed.
    #[inline]
    pub const fn is_pressed(self, key: KeyCode) -> bool {
        (self.pressed & key.bit()) != 0
    }

    /// Returns the current modifier state.
    #[inline]
    pub const fn modifiers(self) -> Modifiers {
        self.modifiers
    }

    /// Marks a key as pressed.
    #[inline]
    pub fn on_key_down(&mut self, key: KeyCode) {
        self.pressed |= key.bit();
        self.sync_modifier(key, true);
    }

    /// Marks a key as released.
    #[inline]
    pub fn on_key_up(&mut self, key: KeyCode) {
        self.pressed &= !key.bit();
        self.sync_modifier(key, false);
    }

    /// Clears all pressed keys and modifiers.
    #[inline]
    pub fn clear(&mut self) {
        self.pressed = 0;
        self.modifiers = Modifiers::default();
    }

    #[inline]
    fn sync_modifier(&mut self, key: KeyCode, pressed: bool) {
        match key {
            KeyCode::ShiftLeft | KeyCode::ShiftRight => {
                self.modifiers.shift = pressed
                    || self.is_pressed(KeyCode::ShiftLeft)
                    || self.is_pressed(KeyCode::ShiftRight);
            }
            KeyCode::ControlLeft | KeyCode::ControlRight => {
                self.modifiers.ctrl = pressed
                    || self.is_pressed(KeyCode::ControlLeft)
                    || self.is_pressed(KeyCode::ControlRight);
            }
            KeyCode::AltLeft | KeyCode::AltRight => {
                self.modifiers.alt = pressed
                    || self.is_pressed(KeyCode::AltLeft)
                    || self.is_pressed(KeyCode::AltRight);
            }
            KeyCode::MetaLeft | KeyCode::MetaRight => {
                self.modifiers.meta = pressed
                    || self.is_pressed(KeyCode::MetaLeft)
                    || self.is_pressed(KeyCode::MetaRight);
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn key_press_release_state_tracking_works() {
        let mut keyboard = KeyboardState::new();
        keyboard.on_key_down(KeyCode::KeyW);
        assert!(keyboard.is_pressed(KeyCode::KeyW));
        keyboard.on_key_up(KeyCode::KeyW);
        assert!(!keyboard.is_pressed(KeyCode::KeyW));
    }

    #[test]
    fn modifiers_track_both_sides() {
        let mut keyboard = KeyboardState::new();
        keyboard.on_key_down(KeyCode::ShiftLeft);
        keyboard.on_key_down(KeyCode::ShiftRight);
        keyboard.on_key_up(KeyCode::ShiftLeft);
        assert!(keyboard.modifiers().shift);
        keyboard.on_key_up(KeyCode::ShiftRight);
        assert!(!keyboard.modifiers().shift);
    }
}

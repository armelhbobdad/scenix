use scenix_math::Vec2;

/// Pointer button identifiers.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[repr(u8)]
pub enum PointerButton {
    /// Primary button.
    Left = 0,
    /// Secondary button.
    Right = 1,
    /// Middle button.
    Middle = 2,
    /// Additional back button.
    Back = 3,
    /// Additional forward button.
    Forward = 4,
}

impl PointerButton {
    #[inline]
    const fn mask(self) -> u8 {
        1 << (self as u8)
    }
}

/// Current pointer position, movement, and button state.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PointerState {
    /// Current pointer position in pixels.
    pub position: Vec2,
    /// Movement since the previous update.
    pub delta: Vec2,
    /// Pressed button bitmask.
    pub buttons: u8,
}

impl PointerState {
    /// Creates an empty pointer state.
    #[inline]
    pub const fn new() -> Self {
        Self {
            position: Vec2::ZERO,
            delta: Vec2::ZERO,
            buttons: 0,
        }
    }

    /// Updates the current position and computes delta.
    #[inline]
    pub fn set_position(&mut self, position: Vec2) {
        self.delta = position - self.position;
        self.position = position;
    }

    /// Clears the accumulated delta.
    #[inline]
    pub fn clear_delta(&mut self) {
        self.delta = Vec2::ZERO;
    }

    /// Marks a button as pressed.
    #[inline]
    pub fn on_button_down(&mut self, button: PointerButton) {
        self.buttons |= button.mask();
    }

    /// Marks a button as released.
    #[inline]
    pub fn on_button_up(&mut self, button: PointerButton) {
        self.buttons &= !button.mask();
    }

    /// Returns whether a button is currently pressed.
    #[inline]
    pub const fn is_pressed(self, button: PointerButton) -> bool {
        (self.buttons & button.mask()) != 0
    }

    /// Returns whether any button is pressed.
    #[inline]
    pub const fn any_pressed(self) -> bool {
        self.buttons != 0
    }
}

impl Default for PointerState {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

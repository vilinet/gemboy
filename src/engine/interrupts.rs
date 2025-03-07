use crate::engine::bit_utils::{bit_clear, bit_set, bit_test};

pub struct InterruptsState {
    state: u8,
}

#[derive(Copy, Clone)]
pub enum InterruptType {
    VBLANK = 0,
    LCD_STAT = 1,
    TIMER = 2,
    SERIAL = 3,
    JOYPAD = 4,
}

impl InterruptsState {
    pub fn new() -> Self {
        InterruptsState { state: 0 }
    }

    #[inline]
    pub fn state(&self) -> u8 {
        self.state
    }

    #[inline]
    pub fn load(&mut self, state: u8) {
        self.state = state;
    }

    #[inline]
    pub fn set(&mut self, int_type: InterruptType) {
        self.state = bit_set(self.state, int_type as u8);
    }

    #[inline]
    pub fn is_set(&self, int_type: InterruptType) -> bool {
        bit_test(self.state, int_type as u8)
    }

    #[inline]
    pub fn clear(&mut self, int_type: InterruptType) {
        self.state = bit_clear(self.state, int_type as u8);
    }
}
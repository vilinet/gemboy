pub struct Timer {
    enabled: bool,
    clock_mode: u8,
}

impl Timer {
    pub fn new() -> Self {
        return Timer {
            enabled: false,
            clock_mode: 0,
        };
    }

    pub fn set(&mut self, v: u8) {
        self.enabled = (v & 0b100) == 0;
        self.clock_mode = v & 0b011;
    }
}

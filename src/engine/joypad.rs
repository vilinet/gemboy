pub struct JoyPad {
    keys: u8,
    keys_state: u8,
}

impl JoyPad {
    pub fn new() -> Self {
        return JoyPad {
            keys: 0,
            keys_state: 0,
        };
    }

    pub fn read(&mut self) -> u8 {
        return self.keys;
    }

    pub fn write(&mut self, v: u8) {
        self.keys_state = v;
    }
}
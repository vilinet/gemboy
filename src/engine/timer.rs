pub struct Timer {
    divider: u16,
    tima: u8,
    tma: u8,
    tac: u8,
    /// [tima] will need to be incremented, when this bit in the [divider] changes. This bit depends on the current clock_mode.
    divider_value_to_check: u16,
}

#[derive(Copy, Clone, PartialEq)]
pub enum TimerInterruptRaised {
    No,
    Yes,
}

impl Timer {
    pub fn new() -> Self {
        let timer =  Timer {
            divider: 0xAC00,
            tac: 0,
            tima: 0,
            tma: 0,
            divider_value_to_check: 0,
        };
        timer
    }

    fn set_divider_bit_to_check(&mut self) {
        let bit = match (self.tac & 0b11) {
            0 => 9, //  1024 -> 4096 Hz
            1 => 3, // 16 -> 262144 Hz
            2 => 5, // 64 -> 65536 Hz
            3 => 7, // 256 -> 16384 Hz
            _ => panic!("Invalid clock mode")
        };

         self.divider_value_to_check = 1 << bit;
    }

    pub fn write(&mut self, addr: u16, v: u8) {
        match addr {
            0xFF04 => self.divider = 0,
            0xFF05 => self.tima = v,
            0xFF06 => self.tma = v,
            0xFF07 => {
                self.tac = v;
                self.set_divider_bit_to_check();
            },
            _ => panic!("Invalid timer register")
        }
    }

    pub fn read(&self, addr: u16) -> u8 {
        match addr {
            0xFF04 => (self.divider >> 8) as u8,
            0xFF05 => self.tima,
            0xFF06 => self.tma,
            0xFF07 => self.tac & 0b111,
            _ => panic!("Invalid timer register")
        }
    }

    pub fn tick(&mut self) ->TimerInterruptRaised {
        let old_divider = self.divider;
        self.divider = self.divider.wrapping_add(1);

        if (self.tac & 4) == 0 {
            return TimerInterruptRaised::No;
        }

        // check if the divider bit has changed, then we increment the [tima]
        if (old_divider & self.divider_value_to_check) != 0 && (self.divider & self.divider_value_to_check) == 0
        {
            self.tima = self.tima.wrapping_add(1);

            if self.tima == 0x0 {
                self.tima = self.tma;
                return TimerInterruptRaised::Yes;
            }
        }

        TimerInterruptRaised::No
    }
}
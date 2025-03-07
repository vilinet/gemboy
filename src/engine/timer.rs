pub struct Timer {
    // divider gets increase in every m-cycle
    divider: u16,
    tima: u8,
    tma: u8,
    tac: u8,
    /// AND this value with [divider] to see whether [tima] needs to be increased
    divider_value_to_check: u16,
}

#[derive(Copy, Clone, PartialEq)]
pub enum TimerInterruptRaised {
    No,
    Yes,
}

impl Timer {
    pub fn new() -> Self {
        let mut timer =  Timer {
            divider: 0,
            tac: 0,
            tima: 0,
            tma: 0,
            divider_value_to_check: 0,
        };

        timer.set_time_to_increase();

        timer
    }

    fn set_time_to_increase(&mut self) {
        self.divider_value_to_check = match self.tac & 0b11 {
            0 => 256,
            1 => 4,
            2 => 16,
            3 => 64,
            _ => panic!("Invalid clock mode")
        };
    }

    pub fn write(&mut self, addr: u16, v: u8) {
        match addr {
            0xFF04 => {
                self.divider = 0;
            },
            0xFF05 => self.tima = v,
            0xFF06 => self.tma = v,
            0xFF07 => {
                self.tac = v;
                self.set_time_to_increase();
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

        let old = self.divider;
        self.divider = self.divider.wrapping_add(1);

        if (self.tac & 4) == 0 {
            return TimerInterruptRaised::No;
        }

        // check if the divider bit has changed, then we increment the [tima]
        if (old & self.divider_value_to_check) != (self.divider & self.divider_value_to_check)
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
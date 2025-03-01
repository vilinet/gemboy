use std::io::Write;

pub struct Serial {
    control: u8,
    data: u8,
}

impl Serial {
    pub fn new() -> Self {
        Serial {
            control: 0,
            data: 0,
        }
    }

    pub fn tick(&mut self) {
        // what should i do?
    }

    pub fn write(&mut self, v: u8) {
        self.data = v;
        print!("{}", v as char);
        std::io::stdout().flush().unwrap();
    }

    pub fn read(&self) -> u8 {
        return 0;
    }

    pub fn set_control(&mut self, v: u8) {
        self.control = v;
    }
    pub fn control(&self) -> u8 {
        self.control
    }
}

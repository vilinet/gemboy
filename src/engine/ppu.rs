

pub struct PPU {
    pub oam: [u8; 0xA0],
    vblank: bool,
    cycles: u32,
    lcd_enabled: bool,
    ppu_enabled: bool,
    status: u8,
}

impl PPU {
    pub fn new() -> Self {
        PPU { oam: [0; 0xA0], vblank: false, cycles: 0, lcd_enabled: false, ppu_enabled: false, status: 0 }
    }

    pub fn is_vblanking(&self) -> bool {
        self.vblank
    }

    pub fn set_status(&mut self, v: u8) {
        self.lcd_enabled = (v & 0x80) != 0;
        self.ppu_enabled = (v & 0x40) != 0;
    }

    pub fn status(&self) -> u8 {
        self.status
    }

    pub fn tick(&mut self) {
        if(!self.lcd_enabled && !self.ppu_enabled) {
            return;
        }

        if self.lcd_enabled {
            self.vblank = false;
            if (self.cycles == 456) {
                self.cycles = 0;
                self.vblank = true;
            }
        }

        self.cycles += 1;
    }
}

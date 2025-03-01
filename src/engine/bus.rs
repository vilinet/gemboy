use std::io::Write;
use log::error;
use log::trace;

use crate::engine::audio::*;
use crate::engine::timer::*;

struct JoyPad {
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

struct Serial {
    control: u8,
    data: u8,
}

impl Serial {
    pub fn new() -> Self {
        return Serial {
            control: 0,
            data: 0,
        };
    }

    pub fn tick(&mut self) {
        // what should i do?
    }

    pub fn write(&mut self, v: u8) {
        self.data = v;
        print!("{}", v as char);
        std::io::stdout().flush().unwrap();
    }

    pub fn control(&mut self, v: u8) {
        self.control = v;
    }
}

pub struct Bus {
    cart: Vec<u8>,
    ram0: [u8; 0x2000],
    ram1: [u8; 0x2000],
    vram: [u8; 0x2000],
    hram: [u8; 0x80],
    io_mock: [u8; 0x80],
    timer: Timer,
    audio: Audio,
    serial: Serial,
    joypad: JoyPad,
    int_flag: u8,
    int_enable: u8,
    viewport_pos_x: u8,
    viewport_pos_y: u8,
    bg_palette: u8,
}

impl Bus {
    pub fn new() -> Self {
        return Bus {
            audio: Audio::new(),
            timer: Timer::new(),
            serial: Serial::new(),
            joypad: JoyPad::new(),
            int_flag: 0,
            int_enable: 0,
            cart: Vec::new(),
            ram0: [0; 0x2000],
            ram1: [0; 0x2000],
            hram: [0; 0x80],
            vram: [0; 0x2000],
            io_mock: [0; 0x80],
            viewport_pos_x: 0,
            viewport_pos_y: 0,
            bg_palette: 0,
        };
    }

    pub fn load_rom(&mut self, rom: Vec<u8>) {
        self.cart = rom;
    }

    pub fn tick(&mut self) {
        // what should i do?
    }

    pub fn grab(&self, addr: u16) -> u8 {
        return self.cart[addr as usize];
    }
    pub fn read(&mut self, addr: u16) -> u8 {
    self.grab(0);
        // trace!("read: {:#06x}", addr);
        let v = match addr {
            0x0000..=0x7FFF => self.cart[addr as usize],
            0x8000..=0x9FFF => self.vram[(addr - 0x8000) as usize],
            0xC000..=0xCFFF => self.ram0[(addr - 0xC000) as usize],
            0xD000..=0xDFFF => self.ram1[(addr - 0xD000) as usize],
            0xFF44 => 0x90, // LY, 0x90 for
            // the gameboy doctor
            0xFF00 => self.joypad.read(),
            0xFF01 => self.serial.data,
            0xFF02 => self.serial.control,
            0xFF0F => self.int_flag | 0b11100000,
            // https://gbdev.io/pandocs/STAT.html#ff41--stat-lcd-status
            0xFF00..=0xFF7F => self.io_mock[(addr - 0xFF00) as usize],
            // https://gbdev.io/pandocs/Palettes.html#lcd-color-palettes-cgb-only
            0xFF80..=0xFFFE => self.hram[(addr - 0xFF80) as usize],
            0xFFFF => self.int_enable,
            _ => unimplemented!()
        };

        return v;
    }

    pub fn write(&mut self, addr: u16, v: u8) -> bool {

        match addr {
            //0x0000..=0x7FFF => unimplemented!(),
            //0x8000..=0xBFFF => unimplemented!(),
            0xC000..=0xCFFF => self.ram0[(addr - 0xC000) as usize] = v,
            0xD000..=0xDFFF => self.ram1[(addr - 0xD000) as usize] = v,
            0x8000..=0x9FFF => self.vram[(addr - 0x8000) as usize] = v,
            0xFF01 => self.serial.write(v),
            0xFF02 => self.serial.control(v),
            0xFF07 => self.timer.set(v),
            0xFF24 => self.audio.set_master_volume_and_vin(v),
            0xFF25 => self.audio.set_panning(v),
            0xFF26 => self.audio.set_master_control(v),
            0xFF0F => self.int_flag = v,
            0xFF00..=0xFF7F => self.io_mock[(addr - 0xFF00) as usize] = v,
            0xFF80..=0xFFFE => self.hram[(addr - 0xFF80) as usize] = v,
            0xFFFF => self.int_enable = v,
            _ => {
                error!("Writing into {:#06x}", addr);
                unimplemented!()
            } 
        }

        return true
    }
}
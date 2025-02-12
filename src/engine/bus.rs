use crate::engine::audio::*;
use crate::engine::timer::*;

pub struct Bus {
    rom: Vec<u8>,
    ram0: [u8; 0x2000],
    ram1: [u8; 0x2000],
    timer: Timer,
    audio: Audio,
    int_flag: u8,
    int_enable: u8,
}

impl Bus {
    pub fn new() -> Self {
        return Bus {
            audio: Audio::new(),
            timer: Timer::new(),
            int_flag: 0,
            int_enable: 0,
            rom: Vec::new(),
            ram0: [0; 0x2000],
            ram1: [0; 0x2000],
        };
    }

    pub fn load_rom(&mut self, rom: Vec<u8>) {
        self.rom = rom;
    }

    pub fn tick4(&mut self) {}

    pub fn read(&mut self, addr: u16) -> u8 {
        let v = match addr {
            0x0000..=0x7FFF => self.rom[addr as usize],
            0x8000..=0xBFFF => unimplemented!(),
            0xC000..=0xCFFF => self.ram0[(addr - 0xC000) as usize],
            0xD000..=0xDFFF => self.ram1[(addr - 0xD000) as usize],
            0xFEA0..=0xFEFF => 0, // restricted
            0xFF07 => unimplemented!(),
            0xFF0F => self.int_flag | 0b11100000,
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
            0xFEA0..=0xFEFF => (), // restricted
            0xFF07 => self.timer.set(v),
            0xFF24 => self.audio.set_master_volume_and_vin(v),
            0xFF25 => self.audio.set_panning(v),
            0xFF26 => self.audio.set_master_control(v),
            0xFF0F => self.int_flag = v,
            0xFFFF => self.int_enable = v,

            _ => {
                println!("Writing into {:#06x}", addr);
                unimplemented!()
            } 
        }

        return true
    }
}
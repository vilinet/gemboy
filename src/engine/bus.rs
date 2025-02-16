use log::error;
use log::trace;

use crate::engine::audio::*;
use crate::engine::timer::*;

pub struct Bus {
    rom: Vec<u8>,
    ram0: [u8; 0x2000],
    ram1: [u8; 0x2000],
    hram: [u8; 0x80],
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
            hram: [0; 0x80]
        };
    }

    pub fn load_rom(&mut self, rom: Vec<u8>) {
        self.rom = rom;
    }

    pub fn tick(&mut self) {
        // what should i do?
    }

    pub fn read(&mut self, addr: u16) -> u8 {
        // trace!("read: {:#06x}", addr);
        let v = match addr {
            0x0000..=0x7FFF => self.rom[addr as usize],
            0x8000..=0xBFFF => unimplemented!(),
            0xC000..=0xCFFF => self.ram0[(addr - 0xC000) as usize],
            0xD000..=0xDFFF => self.ram1[(addr - 0xD000) as usize],
            0xFEA0..=0xFEFF => 0, // restricted
            0xFF07 => unimplemented!(),
            0xFF0F => self.int_flag | 0b11100000,

            // https://gbdev.io/pandocs/STAT.html#ff41--stat-lcd-status
            0xFF44 => 0x90, // LY, 0x90 for the gameboy doctor
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
            0xFEA0..=0xFEFF => (), // restricted
            0xFF07 => self.timer.set(v),
            0xFF24 => self.audio.set_master_volume_and_vin(v),
            0xFF25 => self.audio.set_panning(v),
            0xFF26 => self.audio.set_master_control(v),
            0xFF0F => self.int_flag = v,
            // https://gbdev.io/pandocs/LCDC.html#ff40--lcdc-lcd-control
            0xFF40 => trace!("0xff40 lcd control: {:#2x}", v),
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
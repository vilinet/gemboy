use std::fs::File;
use std::io::Read;
use crate::engine::cartridge::Cartridge;
use crate::engine::cpu::Cpu;

pub struct Gemboy {
    pub cpu: Cpu,
}

impl Gemboy {
    pub fn new() -> Self {
        Gemboy {
            cpu: Cpu::new()
        }
    }

    pub fn load_rom(&mut self, path: &str) {
        let mut rom = Vec::<u8>::new();
        File::open(path).unwrap().read_to_end(&mut rom).expect("failed to open file");
        self.load_rom_binary(rom)
    }

    pub fn load_rom_binary(&mut self, rom: Vec<u8>)
    {
        let cartridge = Cartridge::new(rom);
        self.cpu.bus.load_cartridge(cartridge);
        self.cpu.restart();
    }
}
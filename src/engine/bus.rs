use log::error;

use crate::engine::audio::Audio;
use crate::engine::cartridge::Cartridge;
use crate::engine::cpu::Cpu;
use crate::engine::interrupts::{InterruptType, InterruptsState};
use crate::engine::joypad::JoyPad;
use crate::engine::ppu::{PPUInterruptRaised, PPU};
use crate::engine::serial::Serial;
use crate::engine::timer::{Timer, TimerInterruptRaised};

pub struct Bus {
    pub int_flags: InterruptsState,
    int_enabled: InterruptsState,
    cartridge: Cartridge,
    ram0: [u8; 0x2000],
    ram1: [u8; 0x2000],
    vram: [u8; 0x2000],
    hram: [u8; 0x80],
    io_mock: [u8; 0x80],
    timer: Timer,
    audio: Audio,
    serial: Serial,
    joypad: JoyPad,
    ppu: PPU,
    viewport_pos_x: u8,
    viewport_pos_y: u8,
    bg_palette: u8,
}

impl Bus {
    pub fn new() -> Self {
        Bus {
            cartridge: Cartridge::default(),
            audio: Audio::new(),
            timer: Timer::new(),
            serial: Serial::new(),
            joypad: JoyPad::new(),
            ppu: PPU::new(),
            int_flags: InterruptsState::new(),
            int_enabled: InterruptsState::new(),
            ram0: [0; 0x2000],
            ram1: [0; 0x2000],
            hram: [0; 0x80],
            vram: [0; 0x2000],
            io_mock: [0; 0x80],
            viewport_pos_x: 0,
            viewport_pos_y: 0,
            bg_palette: 0,
        }
    }

    #[inline]
    pub fn should_run_interrupt(&self, int_type: InterruptType) -> bool {
        self.int_enabled.is_set(int_type) && self.int_flags.is_set(int_type)
    }

    pub fn load_cartridge(&mut self, cartridge: Cartridge) {
        self.cartridge = cartridge;
    }

    /// 1 tick is 4 T-Cycles
    pub fn tick(&mut self) {
        // Timer ticks on every M-Cycle( 4 T-Cycles)
        if self.timer.tick() == TimerInterruptRaised::Yes {
            self.int_flags.set(InterruptType::TIMER);
        }

        /*for _ in 0..4 {
            if self.ppu.tick() == PPUInterruptRaised::Yes && self.int_enabled.is_set(InterruptType::VBLANK) {
                self.int_flags.set(InterruptType::VBLANK);
            }
        }*/
    }

    pub fn read(&mut self, addr: u16) -> u8 {
        let v = match addr {
            0x0000..=0x7FFF => self.cartridge.read(addr),
            0x8000..=0x9FFF => self.vram[(addr - 0x8000) as usize],
            0xA000..=0xBFFF => self.cartridge.read(addr),
            0xC000..=0xCFFF => self.ram0[(addr - 0xC000) as usize],
            0xD000..=0xDFFF => self.ram1[(addr - 0xD000) as usize],
            0xE000..=0xFDFF => self.ram0[(addr - 0xE000) as usize], // echo ram
            0xFE00..=0xFE9F => self.ppu.oam[(addr - 0xFE00) as usize],
            0xFEA0..=0xFEFF => 0, // unused
            0xFF44 => 0x90,       // LY, 0x90 for
            // the gameboy doctor
            0xFF00 => self.joypad.read(),
            0xFF01 => self.serial.read(),
            0xFF02 => self.serial.control(),
            0xFF04..=0xFF07 => self.timer.read(addr),
            0xFF0F => self.int_flags.state(),
            0xFF40 => self.ppu.status(),
            0xFF4D => 0xFF, // GBC stuff, returns FF in classic GB
            // https://gbdev.io/pandocs/STAT.html#ff41--stat-lcd-status
            0xFF00..=0xFF7F => self.io_mock[(addr - 0xFF00) as usize],
            // https://gbdev.io/pandocs/Palettes.html#lcd-color-palettes-cgb-only
            0xFF80..=0xFFFE => self.hram[(addr - 0xFF80) as usize],
            0xFFFF => self.int_enabled.state() | 0b11100000,
            _ => {
                error!("Reading from {:#06x}", addr);
                todo!();
            }
        };

        v
    }

    pub fn write(&mut self, addr: u16, v: u8) -> bool {
        match addr {
            0x0000..=0x7FFF => self.cartridge.write(addr, v),
            0x8000..=0x9FFF => self.vram[(addr - 0x8000) as usize] = v,
            0xA000..=0xBFFF => self.cartridge.write(addr, v),
            0xC000..=0xCFFF => self.ram0[(addr - 0xC000) as usize] = v,
            0xD000..=0xDFFF => self.ram1[(addr - 0xD000) as usize] = v,
            0xE000..=0xFDFF => self.ram0[(addr - 0xE000) as usize] = v, // echo ram
            0xFE00..=0xFE9F => self.ppu.oam[(addr - 0xFE00) as usize] = v,
            0xFEA0..=0xFEFF => (), // unused
            0xFF01 => self.serial.write(v),
            0xFF02 => self.serial.set_control(v),
            0xFF04..=0xFF07 => self.timer.write(addr, v),
            0xFF24 => self.audio.set_master_volume_and_vin(v),
            0xFF25 => self.audio.set_panning(v),
            0xFF26 => self.audio.set_master_control(v),
            0xFF0F => self.int_flags.load(0xE0 | v),
            0xFF40 => self.ppu.set_status(v),
            0xFF00..=0xFF7F => self.io_mock[(addr - 0xFF00) as usize] = v,
            0xFF80..=0xFFFE => self.hram[(addr - 0xFF80) as usize] = v,
            0xFFFF => self.int_enabled.load(v),
            _ => {
                error!("Writing into {:#06x}", addr);
            }
        }

        return true;
    }
}

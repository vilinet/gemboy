use crate::engine::cartridge::Cartridge;

pub struct MapperNone {
}

impl MapperNone {
    #[inline]
    pub fn read(cart: &mut Cartridge, addr: u16) -> u8 {
        cart.rom[addr as usize]
    }

    #[inline]
    pub fn write(_: &mut Cartridge, _: u16, _: u8){

    }
}


pub struct MapperMBC1 {
}

impl MapperMBC1 {
    #[inline]
    pub fn read(cart: &mut Cartridge, addr: u16) -> u8 {
        match addr {
            0x0..=0x3FFF => cart.rom[addr as usize],
            0x4000..=0x7FFF => cart.rom[(cart.rom_bank * 0x4000 + (addr - 0x4000)) as usize],
            0xA000..=0xBFFF => cart.ram[(cart.ram_bank * 0x2000 + (addr - 0xA000)) as usize],
            _ =>  panic!("Should not get here, Bus!"),
        }
    }

    #[inline]
    pub fn write(cart: &mut Cartridge, addr: u16, v: u8) {
        match addr {
            0x0..=0x1FFF => {
                cart.ram_enabled = v == 0x0A;
            },
            0x2000..=0x3FFF => {
                cart.rom_bank = v as u16 & 0b11;
                if cart.rom_bank == 0 { cart.rom_bank = 1; }
            }
            0x4000..=0x5FFF => {
                cart.ram_bank = v as u16 & 0b11;
                cart.rom_bank |= ((v & 0b1100) >> 2) as u16;
            }
            0xA000..=0xBFFF =>{
                if cart.ram_enabled {
                    cart.ram[(cart.ram_bank * 0x2000 + (addr - 0xA000)) as usize] = v;
                }
            }
            _ => {
                log::error!("Write: {}", addr)
            }
        }
    }
}
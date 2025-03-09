use crate::engine::mappers::*;

type CartridgeRead = fn(&mut Cartridge, u16) -> u8;
type CartridgeWrite = fn(&mut Cartridge, u16, u8);

pub struct Cartridge {
    pub rom: Vec<u8>,
    pub ram: Vec<u8>,
    pub rom_bank: u16,
    pub ram_bank: u16,
    pub ram_enabled: bool,
    read_impl: CartridgeRead,
    write_impl: CartridgeWrite,
}

impl Default for Cartridge {
    fn default() -> Self {
        Cartridge{
            rom: vec![],
            ram: vec![],
            rom_bank: 1,
            ram_bank: 0,
            ram_enabled: false,
            read_impl: MapperNone::read,
            write_impl: MapperNone::write
        }
    }
}


impl Cartridge {
    pub fn new(rom: Vec<u8>) -> Self {
        let mut cart = Cartridge::default();

        cart.load_rom(rom);

        cart
    }

    #[inline]
    pub fn read(&mut self, addr: u16) -> u8 {
        (self.read_impl)(self, addr)
    }

    #[inline]
    pub fn write(&mut self, addr: u16, value: u8)
    {
        (self.write_impl)(self, addr, value)
    }

    fn load_rom(&mut self, rom: Vec<u8>)
    {
        self.rom_bank = 1;

        let cart_type = rom[0x147];
        self.ram.resize(0x2000 * 4, 0);
        match cart_type {
            0 => {},
            1..=0x03 => {
                self.read_impl = MapperMBC1::read;
                self.write_impl = MapperMBC1::write;
            },
            _ => todo!("implement other cart types")
        }

        self.rom = rom;
    }
}

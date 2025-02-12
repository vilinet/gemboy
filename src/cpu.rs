#[derive(Copy, Clone)]
struct AudioChannel {
    enabled: bool,
    panning_left: bool,
    panning_right: bool
}

impl AudioChannel {
    fn new() -> Self {
        return AudioChannel{enabled: false, panning_left: false, panning_right: false}; 
    }
}

struct Audio {
    enabled: bool,
    channels: [AudioChannel; 4],
    vin_left: bool,
    vin_right: bool,
    volume_left: u8,
    volume_right: u8
}

impl Audio {
    fn new() ->Self {
        return Audio{ vin_left:false, vin_right: false, volume_left: 0, volume_right: 0, enabled: false, channels: [AudioChannel::new(); 4]};
    }

    /// NR52
    fn set_master_control(&mut self, v: u8) {
        self.channels[0].enabled = (v & 0b0001) == 1;
        self.channels[1].enabled = (v & 0b0010) == 1;
        self.channels[2].enabled = (v & 0b0100) == 1;
        self.channels[3].enabled = (v & 0b1000) == 1;
        self.enabled = (v & 0x80) == 1;
    }

    /// NR51
    fn set_panning(&mut self, v: u8)
    {
        self.channels[0].panning_left = (v & 1) == 1;
        self.channels[0].panning_right = (v & 2) == 1;

        self.channels[1].panning_left = (v & 4) == 1;
        self.channels[1].panning_right = (v & 8) == 1;
        
        self.channels[2].panning_left = (v & 16) == 1;
        self.channels[2].panning_right = (v & 32) == 1;
    
        self.channels[3].panning_left = (v & 64) == 1;
        self.channels[3].panning_right = (v & 128) == 1;
    }

    /// NR50
    fn set_master_volume_and_vin(&mut self, v: u8)
    {
        self.volume_right = v & 0b00000111;
        self.vin_right = (v & 0b00001000) == 1;
        self.volume_left = v & 0b01110000;
        self.vin_left = (v & 0b00000000) == 1;
        
    }
}

pub struct Timer 
{
    enabled: bool,
    clock_mode: u8,
}

impl Timer {
    fn new() ->Self{
        return Timer{enabled: false, clock_mode: 0};
    }

    fn set(&mut self, v: u8) {
        self.enabled = (v & 0b100) == 0;
        self.clock_mode = v & 0b011;
    }
}

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

/// Gameboy CPU emulator. Tracks the T-Cycles
pub struct Cpu {
    a: u8,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    f: u8,
    h: u8,
    l: u8,
    ime: u8,
    sp: u16,
    pc: u16,
    pub bus: Bus,
    /* T Cycles */
    cycles: u16,
}

impl Cpu {
    pub fn new() -> Self {
        return Cpu {
            bus: Bus::new(),
            ime: 0,
            a: 1,
            b: 0,
            c: 0x13,
            d: 0,
            e: 0xd8,
            f: 0xb0,
            h: 1,
            l: 0x4d,
            sp: 0xFFFE,
            pc: 0x100,
            cycles: 0,
        };
    }

    pub fn reset(self: &mut Self) {
        self.ime = 0;
        self.a = 1;
        self.b = 0;
        self.c = 0x13;
        self.d = 0;
        self.e = 0xd8;
        self.f = 0xb0;
        self.h = 1;
        self.l = 0x4d;
        self.sp = 0xFFFE;
        self.pc = 0x100;
    }

    pub fn step(&mut self) {
        let ppc = self.pc;
        let cyc = self.cycles;
        let op = self.fetch();
        println!("{:#06x}: OP {:#04x}, Cycle: {}", ppc, op, cyc);
        self.execute(op);
    }

    fn get_hl(&self) -> u16 {
        return (self.h as u16) << 8 | self.l as u16;
    }

    fn push(&mut self, v: u8)
    {
        self.sp -= 1;
        self.bus.write(self.sp, v);
    }

    fn pop(&mut self) -> u8 {
        let v = self.bus.read(self.sp);
        self.sp += 1;
        return v;
    }

    fn pop_pc(&mut self) -> u16
    {
        let low = self.pop() as u16; 
        let high = self.pop() as u16;
        self.pc = high << 8 | low;
        return self.pc;
    }

    fn push_pc(&mut self){
        let high =  (self.pc >> 8) as u8;
        let low = (self.pc & 0xFF) as u8;
        self.push(high);
        self.push(low);
    }

    fn execute(&mut self, opcode: u8) {
        let s = self;
        match opcode {
        
            0 => println!("NOP"),
            0x21 => {
                s.l = s.fetch();
                s.h = s.fetch();
                println!("LD HL, u16> ${:#06x}", s.get_hl());
            }
            0x31 => {
                let v = s.fetch16();
                println!("LD SP, u16: ${:#06x}", v);
                s.sp = v;
            }
            0x3E => {
                let v = s.fetch();
                println!("LD A,u8: ${:#04x}", v);
                s.a = v;
            }
            0xC3 => {
                let v = s.fetch16();
                println!("JP u16: ${:#06x}", v);
                s.jmp(v);
            }
            0xCD => {
                let addr = s.fetch16();
                s.push_pc();
                s.pc = addr;
                println!("CALL u16: ${:#06x}", addr);
            }
            0xE0 => {
                let rel = s.fetch();
                let addr = 0xFF00u16 + rel as u16;
                println!("LD (FF00+u8),A: {:#06x} <- {:#04x}", addr, s.a);
                s.bus.write(addr, s.a);
            }
            0xEA => {
                let addr = s.fetch16();
                println!("LD (u16),A: {:#06x} <- {:#04x}", addr, s.a);
                s.bus.write(addr, s.a);
            }
            0xF3 => {
                println!("DI");
                s.ime = 0;
            }
            _ => println!("Unhandled opcode"),
        }
    }

    fn jmp(&mut self, par: u16) {
        self.tick4();
        self.pc = par;
    }

    fn tick4(&mut self) {
        self.cycles += 4;
        self.bus.tick4();

        // drive ppu  and others!
    }

    fn fetch(&mut self) -> u8 {
        self.tick4();
        let addr = self.pc;
        self.pc += 1;
        return self.bus.read(addr);
    }

    fn fetch16(&mut self) -> u16 {
        let lower = self.fetch() as u16;
        let upper = self.fetch() as u16;
        return upper << 8 | lower;
    }

    fn read(&mut self, addr: u16) -> u8 {
        self.tick4();
        return self.bus.read(addr);
    }

    fn peek(&mut self, addr: u16) -> u8 {
        return self.bus.read(addr);
    }

    fn peek_16(&mut self, addr: u16) -> u16 {
        let lower = self.bus.read(addr) as u16;
        let upper = self.bus.read(addr + 1) as u16;
        return upper << 8 | lower;
    }
}

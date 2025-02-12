use crate::engine::bus::*;

// Detailed T-cycle instruction table: https://izik1.github.io/gbops/
// An older table that is more helpful for the instruction's job: https://meganesu.github.io/generate-gb-opcodes/

/// Gameboy CPU emulator.
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
        self.tick();
        self.sp -= 1;
        self.bus.write(self.sp, v);
    }

    fn pop(&mut self) -> u8 {
        self.tick();
        let v = self.bus.read(self.sp);
        self.sp += 1;
        return v;
    }

    fn pop_pc(&mut self)
    {
        let low = self.pop() as u16; 
        let high = self.pop() as u16;
        self.pc = high << 8 | low;
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
                s.internal_decision();
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
        self.internal_decision();
        self.pc = par;
    }

    /// Increases T-Cycles by 4 and drives the "circuit"
    fn tick(&mut self) {
        self.cycles += 4;
        self.bus.tick();

        // drive ppu  and others!
    }

    /// Mimicks some internal cpu decision. Helps to keep the T-cycles in sync.
    fn internal_decision(&mut self)
    {
        self.tick();
    }


    fn fetch(&mut self) -> u8 {
        self.tick();
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
        self.tick();
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

#[cfg(test)]
mod tests {
    use std::vec;
    use super::*;

    const STACK_START: u16 = 0xDFFF;

    fn create_cpu(rom: Vec<u8>) -> Cpu
    {
        let mut cpu = Cpu::new();
        cpu.bus.load_rom(rom);
        cpu.pc = 0;
        cpu.sp = STACK_START;
        return cpu;
    }

    #[test]
    fn cpu_initial_state()
    {
        let mut cpu = Cpu::new();
        assert_eq!(0x100, cpu.pc);
        assert_eq!(0xFFFE, cpu.sp);

        cpu.pc = 0;
        cpu.sp = 0;

        cpu.reset();
        assert_eq!(0x100, cpu.pc);
        assert_eq!(0xFFFE, cpu.sp);
    }

    #[test]
    fn stack_u8() {
        let mut cpu = create_cpu(vec![]);

        cpu.push(1);
        assert_eq!(0xDFFE, cpu.sp);

        assert_eq!(1, cpu.pop());
        assert_eq!(STACK_START, cpu.sp);

        // TODO: address over(under)flows once it handles all regions
    }

    #[test]
    fn stack_pc()
    {
        let mut cpu = create_cpu(vec![]);
        cpu.pc = 0x1234;

        cpu.push_pc();
        assert_eq!(STACK_START - 2, cpu.sp);

        cpu.pop_pc();
        assert_eq!(0x1234, cpu.pc);
        assert_eq!(STACK_START, cpu.sp);
    }

    #[test]
    fn call_u16()
    {
        let mut cpu = create_cpu(vec![0xCD, 0x34, 0x12]);
        cpu.step();
        
        assert_eq!(24, cpu.cycles);

        // jumped to new pc
        assert_eq!(0x1234, cpu.pc);

        // returning pc should be on the stack, and it should be 3
        assert_eq!(STACK_START - 2, cpu.sp);

        cpu.pop_pc();

        assert_eq!(3, cpu.pc);
    }
}
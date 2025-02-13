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
            f: 0x80,
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
        self.f = 0x80;
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
        return self.get_reg_pair(self.h, self.l);
    }

    fn get_af(&self) -> u16 {
        return self.get_reg_pair(self.a, self.f);
    }

    fn get_bc(&self) -> u16 {
        return self.get_reg_pair(self.b, self.c);
    }

    fn get_de(&self) -> u16 {
        return self.get_reg_pair(self.d, self.e);
    }

    fn set_reg_pair(left: &mut u8, right: &mut u8, v: u16) {
        *left = (v >> 8) as u8;
        *right = (v & 0xFF) as u8;
    }

    fn get_reg_pair(&self, left: u8, right: u8) -> u16 {
        return (left as u16) << 8 | right as u16;
    }

    fn set_hl(&mut self, v: u16) {
        Cpu::set_reg_pair(&mut self.h, &mut self.l, v);
    }

    fn set_af(&mut self, v: u16) {
        Cpu::set_reg_pair(&mut self.a, &mut self.f, v);
    }

    fn set_bc(&mut self, v: u16) {
        Cpu::set_reg_pair(&mut self.b, &mut self.c, v);
    }

    fn set_de(&mut self, v: u16) {
        Cpu::set_reg_pair(&mut self.d, &mut self.e, v);
    }

    fn push(&mut self, v: u8) {
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

    fn pop_pc(&mut self) {
        self.pc = self.pop16();
    }

    fn push16(&mut self, v: u16) {
        let high = (v >> 8) as u8;
        let low = (v & 0xFF) as u8;
        self.push(high);
        self.push(low);
    }

    fn pop16(&mut self) -> u16 {
        let low = self.pop() as u16;
        let high = self.pop() as u16;
        return high << 8 | low;
    }

    fn push_pc(&mut self) {
        self.push16(self.pc);
    }

    fn rel_pc(&self, rel: i8) -> u16 {
        if rel < 0 {
            // TODO: (self.pc as i32 -  rel) as usize ?  or just self.pc + negate(rel) and +1
            unimplemented!();
        }
        return self.pc + rel as u16;
    }

    fn execute(&mut self, opcode: u8) {
        let s = self;
        match opcode {
            0x00 => println!("NOP"),
            0x01 => {
                let v = s.fetch16();
                s.set_bc(v);
                println!("LD BC,u16: {:#06x}", v);
            }
            0x03 => {
                let v = s.get_bc();
                s.internal_work();
                s.set_bc(v + 1);
                println!("INC BC: {:#06x} -> {:#06x}", v, s.get_bc());
            }
            0x18 => {
                let v = s.fetch() as i8;
                s.pc = s.rel_pc(v);
                println!("JR i8: rel: {:#04x} -> {:#06x}", v, s.pc);
            }
            0x21 => {
                let v = s.fetch16();
                s.set_hl(v);
                println!("LD HL, u16: {:#06x}", s.get_hl());
            }
            0x23 => {
                let v = s.get_hl();
                s.internal_work();
                s.set_hl(v + 1);
                println!("INC HL: {:#06x} -> {:#06x}", v, s.get_hl());
            }
            0x2A => {
                s.a = s.bus.read(s.get_hl());
                s.set_hl(s.get_hl() + 1);
                println!("LD A, (HL+): <- ({:#06x}):  {:#04x}", s.get_hl(), s.a);
            }
            0x31 => {
                let v = s.fetch16();
                s.sp = v;
                println!("LD SP, u16: {:#06x}", v);
            }
            0x3E => {
                let v = s.fetch();
                s.a = v;
                println!("LD A,u8: {:#04x}", v);
            }
            0x7C => {
                s.a = s.h;
                s.internal_work();
                println!("LD A, H:  {:#04x}", s.h);
            }
            0x7D => {
                s.a = s.l;
                s.internal_work();
                println!("LD A, L:  {:#04x}", s.l);
            }
            0xC3 => {
                let v = s.fetch16();
                s.jmp(v);
                println!("JP u16: {:#06x}", v);
            }
            0xC5 => {
                s.internal_work();
                s.push16(s.get_bc());
                println!("PUSH BC:  {:#06x}", s.get_bc());
            }
            0xC9 => {
                s.pop_pc();
                println!("RET: {:#06x}", s.pc);
            }
            0xCD => {
                let addr = s.fetch16();
                s.push_pc();
                s.internal_work();
                s.pc = addr;
                println!("CALL u16: {:#06x}", addr);
            }
            0xE0 => {
                let rel = s.fetch();
                let addr = 0xFF00u16 + rel as u16;
                s.bus.write(addr, s.a);
                println!("LD (FF00+u8),A: {:#06x} <- {:#04x}", addr, s.a);
            }
            0xE1 => {
                let v = s.pop16();
                s.set_hl(v);
                println!("POP HL: {:#06x}", s.get_hl());
            }
            0xE5 => {
                s.push16(s.get_hl());
                println!("PUSH HL: {:#06x}", s.get_hl());
            }
            0xEA => {
                let addr = s.fetch16();
                s.bus.write(addr, s.a);
                println!("LD (u16),A: {:#06x} <- {:#04x}", addr, s.a);
            }
            0xF1 => {
                let v = s.pop16();
                s.set_af(v);
                println!("POP AF: {:#06x}", s.get_af());
            }
            0xF3 => {
                s.ime = 0;
                println!("DI");
            }
            0xF5 => {
                s.push16(s.get_af());
                println!("PUSH AF: {:#06x}", s.get_af());
            }
            _ => unimplemented!("unhandled opcode"),
        }

        println!();
    }

    fn jmp(&mut self, par: u16) {
        self.internal_work();
        self.pc = par;
    }

    /// Increases T-Cycles by 4 and drives the "circuit"
    fn tick(&mut self) {
        self.cycles += 4;
        self.bus.tick();

        // drive ppu  and others!
    }

    /// Mimicks some internal cpu work. Helps to keep the T-cycles in sync.
    fn internal_work(&mut self) {
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec;

    const STACK_START: u16 = 0xDFFF;

    fn create_cpu(rom: Vec<u8>) -> Cpu {
        let mut cpu = Cpu::new();
        cpu.bus.load_rom(rom);
        cpu.pc = 0;
        cpu.sp = STACK_START;
        return cpu;
    }

    #[test]
    fn cpu_initial_state() {
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
    fn stack_pc() {
        let mut cpu = create_cpu(vec![]);
        cpu.pc = 0x1234;

        cpu.push_pc();
        assert_eq!(STACK_START - 2, cpu.sp);

        cpu.pop_pc();
        assert_eq!(0x1234, cpu.pc);
        assert_eq!(STACK_START, cpu.sp);
    }

    #[test]
    fn call_ret_u16() {
        // call 0x04
        // a byte gap we will return tro
        // ret
        let mut cpu = create_cpu(vec![0xCD, 0x04, 0x00, 0x01, 0xC9]);
        cpu.step();

        assert_eq!(24, cpu.cycles);

        // jumped to new pc
        assert_eq!(0x04, cpu.pc);

        // returning pc should be on the stack, and it should be 3
        assert_eq!(STACK_START - 2, cpu.sp);
        assert_eq!(4, cpu.pc);

        // execute RET that jumps to 0x01 => 3th byte
        cpu.step();

        assert_eq!(STACK_START, cpu.sp);
        assert_eq!(3, cpu.pc);
        assert_eq!(1, cpu.bus.read(cpu.pc))
    }
}

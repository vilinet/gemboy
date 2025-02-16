use crate::engine::bus::*;
use log::{info, warn, trace};

// Detailed T-cycle instruction table: https://izik1.github.io/gbops/
// An older table that is more helpful for the instruction's job: https://meganesu.github.io/generate-gb-opcodes/

fn bit_set(v: u8, bit: u8) -> u8 {
    return v | (1 << bit);
}

fn bit_clear(v: u8, bit: u8) -> u8 {
    return v & !(1 << bit);
}

fn bit_test(v: u8, bit: u8) -> bool {
    return (v & (1 << bit)) != 0;
}

/// Gameboy CPU emulator.
pub struct Cpu {
    pub a: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
    pub e: u8,
    pub f: u8,
    pub h: u8,
    pub l: u8,
    pub ime: u8,
    pub sp: u16,
    pub pc: u16,
    pub bus: Bus,
    /* T Cycles */
    cycles: u32
}

impl Cpu {
    const ZERO_BIT: u8 = 7;
    const SUBSTRACTION_BIT: u8 = 6;
    const HALF_CARRY_BIT: u8 = 5;
    const CARRY_BIT: u8 = 4;

    /// Creates CPU in a state after the official BOOT rom.
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
            pc: 0x0100,
            cycles: 0,
        };
    }

    /// Resets the CPU to the state after the official BOOT rom.
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
        self.pc = 0x0100;
    }

    pub fn step(&mut self) {
        let ppc = self.pc;
        let cyc = self.cycles;
        let op = self.fetch();
        warn!("{:#06x}: OP {:#04x}, Cycle: {}", ppc, op, cyc);
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

    fn zero_flag(&self) -> u8 {
        return bit_test(self.f, Self::ZERO_BIT) as u8;
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
        return (self.pc as i32 + rel as i32) as u16;
    }

    fn dump_registers(&self)
    {
        trace!("AF: {:#06x}, BC: {:#06x}, DE: {:#06x}, HL: {:#06x}, SP: {:#06x}\n", self.get_af(), self.get_bc(), self.get_de(), self.get_hl(), self.sp);
    }

    fn cmp(&mut self, v: u8) {
        let diff = (std::num::Wrapping(self.a) - std::num::Wrapping(v)).0;
        self.set_flag_zero(diff);
        self.set_flag_sub(true);
        self.set_flag_half_carry(self.a, v);
        self.set_flag_carry(self.a < v);
    }

    fn inc(&mut self, v: u8) -> u8 {
        let res = v.wrapping_add(1);
        self.set_flag_zero(res);
        self.set_flag_sub(false);
        self.set_flag_half_carry(v, res);
        return res;
    }

    fn set_flag_carry(&mut self, carry: bool) {
        if carry {
            self.f = bit_set(self.f, Self::CARRY_BIT)
        } else {
            self.f = bit_clear(self.f, Self::CARRY_BIT)
        }
    }

    fn set_flag_half_carry(&mut self, old: u8, new: u8) {
        let is_hc = (old & 0xF) > (new & 0xF);
        if is_hc {
            self.f = bit_set(self.f, Self::HALF_CARRY_BIT)
        } else {
            self.f = bit_clear(self.f, Self::HALF_CARRY_BIT)
        }
    }

    fn set_flag_sub(&mut self, sub: bool) {
        if sub {
            self.f = bit_set(self.f, Self::SUBSTRACTION_BIT)
        } else {
            self.f = bit_clear(self.f, Self::SUBSTRACTION_BIT)
        }
    }

    fn set_flag_zero(&mut self, v: u8) {
        if v == 0 {
            self.f = bit_set(self.f, Self::ZERO_BIT)
        } else {
            self.f = bit_clear(self.f, Self::ZERO_BIT)
        }
    }

    fn jr(&mut self, addr: u16, condition: u8) {
        if condition != 0 {
            self.jump(addr);
        }
    }

    fn jump(&mut self, addr: u16) {
        self.internal_work();
        self.pc = addr;
    }

    fn execute(&mut self, opcode: u8) {
        let s = self;
        match opcode {
            0x00 => trace!("NOP"),
            0x01 => {
                let v = s.fetch16();
                s.set_bc(v);
                trace!("LD BC,u16: {:#04x}", v);
            }
            0x03 => {
                let v = s.get_bc();
                s.internal_work();
                s.set_bc(v.wrapping_add(1));
                trace!("INC BC: {:#06x} -> {:#06x}", v, s.get_bc());
            }
            0x0E => {
                let v = s.fetch();
                s.c = v;
                trace!("LD C,u8: {:#04x}", v);
            }
            0x11 =>{
                let v = s.fetch16();
                s.set_de(v);
                trace!("LD DE,u16: {:#06x}",  v); 
            }
            0x12 => {
                s.bus.write(s.get_de(), s.a);
                trace!("LD (DE),A: {:#06x} <- {:#04x}", s.get_de(), s.a);
            }
            0x18 => {
                let v = s.fetch() as i8;
                s.jump(s.rel_pc(v));
                trace!("JR i8: rel: {:#04x} -> {:#06x}", v, s.pc);
            }
            0x1C => {
                s.e = s.inc(s.e);
                trace!("INC E: {:#04x}", s.e);
            }
            0x20 => {
                let rel = s.fetchi8();
                let condition: u8 = s.zero_flag();
                let addr = s.rel_pc(rel);
                s.jr(addr, condition);
               trace!("JR NZ,i8: jumps: {:#04x} -> {:#06x}", condition, addr);
            }
            0x21 => {
                let v = s.fetch16();
                s.set_hl(v);
                trace!("LD HL, u16: {:#06x}", s.get_hl());
            }
            0x23 => {
                let v = s.get_hl();
                s.internal_work();
                s.set_hl(v + 1);
                trace!("INC HL: {:#06x} -> {:#06x}", v, s.get_hl());
            }
            0x28 => {
                let rel = s.fetchi8();
                let condition: u8 = !s.zero_flag();
                let addr = s.rel_pc(rel);
                s.jr(addr, condition);
                trace!("JR Z,i8: Z: {:#04x} -> {:#06x}", condition, addr);
            }

            0x2A => {
                s.a = s.bus.read(s.get_hl());
                s.set_hl(s.get_hl() + 1);
                trace!("LD A, (HL+): <- ({:#06x}):  {:#04x}", s.get_hl(), s.a);
            }
            0x31 => {
                let v = s.fetch16();
                s.sp = v;
                trace!("LD SP, u16: {:#06x}", v);
            }
            0x3E => {
                let v = s.fetch();
                s.a = v;
                trace!("LD A,u8: {:#04x}", v);
            }
            0x47 => {
                s.b = s.a;
                trace!("LD B,A: {:#04x}", s.b);
            }
            0x7C => {
                s.a = s.h;
                s.internal_work();
                trace!("LD A, H:  {:#04x}", s.h);
            }
            0x78 => {
                s.a = s.b;
                trace!("LD A,B: {:#04x}", s.a);
            }
            0x7D => {
                s.a = s.l;
                s.internal_work();
                trace!("LD A, L:  {:#04x}", s.l);
            }
            0xB1 => {
                s.f = 0;
                let v = s.a | s.c;
                s.set_flag_zero(v);
                trace!("OR C: A:{:#04x} OR C:{:#04x} => {:#04x}, flags:  {:#04x}",s.a, s.c, v, s.f);
                s.a = v;
            }
            0xC1 => {
                let bc = s.pop16();
                s.set_bc(bc);
                trace!("POP BC: {:#06x}", bc);
            }
            0xC3 => {
                let v = s.fetch16();
                s.jump(v);
                trace!("JP u16: {:#06x}", v);
            }
            0xC5 => {
                s.internal_work();
                s.push16(s.get_bc());
                trace!("PUSH BC:  {:#06x}", s.get_bc());
            }
            0xC9 => {
                s.pop_pc();
                trace!("RET: {:#06x}", s.pc);
            }
            0xCD => {
                let addr = s.fetch16();
                s.push_pc();
                s.jump(addr);
                trace!("CALL u16: {:#06x}", addr);
            }
            0xE0 => {
                let rel = s.fetch();
                let addr = 0xFF00u16 + rel as u16;
                s.bus.write(addr, s.a);
                trace!("LD (FF00+u8),A: {:#06x} <- {:#04x}", addr, s.a);
            }
            0xE1 => {
                let v = s.pop16();
                s.set_hl(v);
                trace!("POP HL: {:#06x}", s.get_hl());
            }
            0xE5 => {
                s.push16(s.get_hl());
                trace!("PUSH HL: {:#06x}", s.get_hl());
            }
            0xEA => {
                let addr = s.fetch16();
                s.bus.write(addr, s.a);
                trace!("LD (u16),A: {:#06x} <- {:#04x}", addr, s.a);
            }
            0xF0 => {
                let rel = s.fetch();
                let addr = 0xFF00u16 + rel as u16;
                let v = s.bus.read(addr);
                s.a = v;
                trace!("LD A,(FF00+u8): {:#06x} <- {:#04x}", addr, v);
            }
            0xF1 => {
                let v = s.pop16();
                s.set_af(v);
                trace!("POP AF: {:#06x}", s.get_af());
            }
            0xF3 => {
                s.ime = 0;
                trace!("DI");
            }
            0xF5 => {
                s.push16(s.get_af());
                trace!("PUSH AF: {:#06x}", s.get_af());
            }
            0xFE => {
                let v = s.fetch();
                s.cmp(v); 
                trace!("CP A,u8: {:#04x}, Z: {}", v, s.zero_flag());
            }
            _ => unimplemented!("unhandled opcode"),
        }

        s.dump_registers();
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

    fn fetchi8(&mut self) -> i8 {
        return self.fetch() as i8;
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

use crate::engine::bus::*;
use log::{error, trace, warn};

// Detailed T-cycle instruction table: https://izik1.github.io/gbops/
// An older table that is more helpful for the instruction's job: https://meganesu.github.io/generate-gb-opcodes/

const fn bit_set(v: u8, bit: u8) -> u8 {
    return v | (1 << bit);
}

const fn bit_clear(v: u8, bit: u8) -> u8 {
    return v & !(1 << bit);
}

const fn bit_test(v: u8, bit: u8) -> bool {
    return (v & (1 << bit)) != 0;
}

/// Logical NOT operation on a u8 value, not like the ! operator.
const fn not(v: u8) -> u8 {
    return match v {
        0 => 1,
        _ => 0,
    };
}

#[derive(Debug, Copy, Clone)]
enum Reg {
    A,
    F,
    B,
    C,
    D,
    E,
    H,
    L,
    AF,
    BC,
    DE,
    HL,
    SP,
    PC,
}

#[derive(Debug, Copy, Clone)]
pub enum Op {
    Reg(Reg),
    Imm8,
    Imm16,
    IndirectAddress,
    AddressHL,
    AddressBC,
    AddressDE,
    /// LD (FF00+C),A
    ZeroPageRegister,
    /// LD (FF00+u8),A
    ZeroPageDirect,
}

#[derive(Debug, Copy, Clone)]
pub enum JumpCondition {
    Zero,
    NotZero,
    Carry,
    NotCarry,
}

type InstructionCall = fn(&mut Cpu);

#[derive(Debug, Copy, Clone)]
pub struct Instruction {
    name: &'static str,
    call: InstructionCall,
    dest: Option<Op>,
    src: Option<Op>,
    jump_condition: Option<JumpCondition>,
}

impl Instruction {
    pub fn new(
        name: &'static str,
        call: InstructionCall,
        dest: Option<Op>,
        src: Option<Op>,
    ) -> Self {
        return Instruction {
            name,
            call,
            dest,
            src,
            jump_condition: None,
        };
    }

    pub fn new_cond_jump(
        name: &'static str,
        call: InstructionCall,
        dest: Option<Op>,
        src: Option<Op>,
        cond: JumpCondition,
    ) -> Self {
        return Instruction {
            name,
            call,
            dest,
            src,
            jump_condition: Some(cond),
        };
    }
}

/// Gameboy CPU emulator.
pub struct Cpu {
    inst_counter: u32,
    pub opcode: u8,
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

    // the value cpu reads and writes out when needed
    value: u16,

    /* T Cycles */
    cycles: u32,
    instructions: [Instruction; 256],
}

impl Cpu {
    /// Z
    const ZERO_BIT: u8 = 7;

    /// N
    const SUBSTRACTION_BIT: u8 = 6;

    /// H
    const HALF_CARRY_BIT: u8 = 5;

    /// C
    const CARRY_BIT: u8 = 4;

    /// Creates CPU in a state after the official BOOT rom.
    #[rustfmt::skip]
    pub fn new() -> Self {
        let mut cpu = Cpu {
            value: 0,
            bus: Bus::new(),
            opcode: 0,
            ime: 0,
            a: 0,
            b: 0,
            c: 0,
            d: 0,
            e: 0,
            f: 0,
            h: 1,
            l: 0,
            sp: 0,
            pc: 0,
            inst_counter: 0,
            cycles: 0,
            instructions: [Instruction::new("UNSUPPORTED", Cpu::not_supported, None, None); 256]
        };
        
        let REG_A = Some(Op::Reg(Reg::A));
        let REG_B = Some(Op::Reg(Reg::B));
        let REG_C = Some(Op::Reg(Reg::C));
        let REG_D = Some(Op::Reg(Reg::D));
        let REG_E = Some(Op::Reg(Reg::E));
        let REG_H = Some(Op::Reg(Reg::H));
        let REG_L = Some(Op::Reg(Reg::L));
        let REG_AF = Some(Op::Reg(Reg::AF));
        let REG_BC = Some(Op::Reg(Reg::BC));
        let REG_DE = Some(Op::Reg(Reg::DE));
        let REG_HL = Some(Op::Reg(Reg::HL));
        let REG_SP = Some(Op::Reg(Reg::SP));
        let REG_PC = Some(Op::Reg(Reg::PC));
        let ADDR_HL = Some(Op::AddressHL);
        let ADDR_BC = Some(Op::AddressBC);
        let ADDR_DE = Some(Op::AddressDE);
        let ADDR_IND = Some(Op::IndirectAddress);
        let ADDR_ZERO_PAGE_DIRECT = Some(Op::ZeroPageDirect);
        let IMM8 = Some(Op::Imm8);
        let IMM16 = Some(Op::Imm16);
    
        cpu.instructions[0x00] = Instruction::new("NOP", Cpu::nop, None, None);
        cpu.instructions[0x01] = Instruction::new("LD BC,u16", Cpu::ld, REG_BC, IMM16);
        cpu.instructions[0x02] = Instruction::new("LD (BC),A", Cpu::ld, ADDR_BC, REG_A);
        cpu.instructions[0x03] = Instruction::new("INC BC", Cpu::inc_reg_16, REG_BC, REG_BC);
        cpu.instructions[0x04] = Instruction::new("INC B", Cpu::inc_reg, REG_B, REG_B);
        cpu.instructions[0x05] = Instruction::new("DEC B", Cpu::dec_reg, REG_B, REG_B);
        cpu.instructions[0x06] = Instruction::new("LD B,u8", Cpu::ld, REG_B, IMM8);
        cpu.instructions[0x07] = Instruction::new("RLCA", Cpu::rlca, None, None);
        cpu.instructions[0x0D] = Instruction::new("DEC C", Cpu::dec_reg, REG_C, REG_C);
        cpu.instructions[0x0E] = Instruction::new("LD C,u8", Cpu::ld, REG_C, IMM8);
        cpu.instructions[0x11] = Instruction::new("LD DE,u16", Cpu::ld, REG_DE, IMM16);
        cpu.instructions[0x12] = Instruction::new("LD (DE),A", Cpu::ld, ADDR_DE, REG_A);
        cpu.instructions[0x13] = Instruction::new("INC DE", Cpu::inc_reg_16, REG_DE, REG_DE);
        cpu.instructions[0x14] = Instruction::new("INC D", Cpu::inc_reg, REG_D, REG_D);
        cpu.instructions[0x15] = Instruction::new("DEC D", Cpu::dec_reg, REG_D, REG_D);
        cpu.instructions[0x16] = Instruction::new( "LD D,u8", Cpu::ld, REG_D, IMM8);
        cpu.instructions[0x17] = Instruction::new("RLA", Cpu::rla, None, None);
        cpu.instructions[0x18] = Instruction::new("JR i8", Cpu::jr, None, IMM8);
        cpu.instructions[0x19] = Instruction::new("ADD HL,DE", Cpu::add_reg_16, None, REG_DE); 
        cpu.instructions[0x1F] = Instruction::new("RRA", Cpu::rra, None, None);
        cpu.instructions[0x22] = Instruction::new("LD (HL+),A", Cpu::ld_hl_plus, ADDR_HL, REG_A);
        cpu.instructions[0x1A] = Instruction::new("LD A,(DE)", Cpu::ld, REG_A, ADDR_DE);
        cpu.instructions[0x1C] = Instruction::new("INC E", Cpu::inc_reg, REG_E, REG_E);
        cpu.instructions[0x20] = Instruction::new_cond_jump("JR NZ,i8", Cpu::jr, None, IMM8, JumpCondition::NotZero);
        cpu.instructions[0x21] = Instruction::new("LD HL,u16", Cpu::ld, REG_HL, IMM16);
        cpu.instructions[0x23] = Instruction::new("INC HL", Cpu::inc_reg_16, REG_HL, REG_HL);
        cpu.instructions[0x24] = Instruction::new("INC H", Cpu::inc_reg, REG_H, REG_H);
        cpu.instructions[0x25] = Instruction::new("DEC H", Cpu::dec_reg, REG_H, REG_H);
        cpu.instructions[0x26] = Instruction::new("LD H,u8", Cpu::ld, REG_H, IMM8);
        cpu.instructions[0x28] = Instruction::new_cond_jump("JR Z,i8", Cpu::jr, None, IMM8, JumpCondition::Zero);
        cpu.instructions[0x2A] = Instruction::new("LD A,(HL+)", Cpu::ld_hl_plus, REG_A, ADDR_HL);
        cpu.instructions[0x2C] = Instruction::new("INC L", Cpu::inc_reg, REG_L, REG_L);
        cpu.instructions[0x2D] = Instruction::new("DEC L", Cpu::dec_reg, REG_L, REG_L);
        cpu.instructions[0x30] = Instruction::new_cond_jump( "JR NC,i8", Cpu::jr, None, IMM8, JumpCondition::NotCarry);
        cpu.instructions[0x31] = Instruction::new("LD SP,u16", Cpu::ld, REG_SP, IMM16);
        cpu.instructions[0x32] = Instruction::new("LD (HL-),A", Cpu::ld_hl_minus_a, None, None);
        cpu.instructions[0x3A] = Instruction::new("LD A, (HL-)", Cpu::ld_a_hl_minus, None, None);
        cpu.instructions[0x46] = Instruction::new("LD B,(HL)", Cpu::ld, REG_B, ADDR_HL);
        cpu.instructions[0x3E] = Instruction::new("LD A,u8", Cpu::ld, REG_A, IMM8);
        cpu.instructions[0x47] = Instruction::new("LD B,A", Cpu::ld, REG_B, REG_A);
        cpu.instructions[0x4E] = Instruction::new("LD C,(HL)", Cpu::ld, REG_C, ADDR_HL);
        cpu.instructions[0x4F] = Instruction::new("LD C,A", Cpu::ld, REG_C, REG_A);
        cpu.instructions[0x50] = Instruction::new("LD D,B", Cpu::ld, REG_D, REG_B);
        cpu.instructions[0x51] = Instruction::new("LD D,C", Cpu::ld, REG_D, REG_C);
        cpu.instructions[0x52] = Instruction::new("LD D,D", Cpu::ld, REG_D, REG_D);
        cpu.instructions[0x53] = Instruction::new("LD D,E", Cpu::ld, REG_D, REG_E);
        cpu.instructions[0x54] = Instruction::new("LD D,H", Cpu::ld, REG_D, REG_H);
        cpu.instructions[0x55] = Instruction::new("LD D,L", Cpu::ld, REG_D, REG_L);
        cpu.instructions[0x56] = Instruction::new("LD D,(HL)", Cpu::ld, REG_D, ADDR_HL);
        cpu.instructions[0x57] = Instruction::new("LD D,A", Cpu::ld, REG_D, REG_A);
        cpu.instructions[0x58] = Instruction::new("LD E,B", Cpu::ld, REG_E, REG_B);
        cpu.instructions[0x59] = Instruction::new("LD E,C", Cpu::ld, REG_E, REG_C);
        cpu.instructions[0x5A] = Instruction::new("LD E,D", Cpu::ld, REG_E, REG_D);
        cpu.instructions[0x5B] = Instruction::new("LD E,E", Cpu::ld, REG_E, REG_E);
        cpu.instructions[0x5C] = Instruction::new("LD E,H", Cpu::ld, REG_E, REG_H);
        cpu.instructions[0x5D] = Instruction::new("LD E,L", Cpu::ld, REG_E, REG_L);
        cpu.instructions[0x5E] = Instruction::new("LD E,(HL)", Cpu::ld, REG_E, ADDR_HL);
        cpu.instructions[0x5F] = Instruction::new("LD E,A", Cpu::ld, REG_E, REG_A);
        cpu.instructions[0x60] = Instruction::new("LD H,B", Cpu::ld, REG_H, REG_B);
        cpu.instructions[0x61] = Instruction::new("LD H,C", Cpu::ld, REG_H, REG_C);
        cpu.instructions[0x62] = Instruction::new("LD H,D", Cpu::ld, REG_H, REG_D);
        cpu.instructions[0x63] = Instruction::new("LD H,E", Cpu::ld, REG_H, REG_E);
        cpu.instructions[0x64] = Instruction::new("LD H,H", Cpu::ld, REG_H, REG_H);
        cpu.instructions[0x65] = Instruction::new("LD H,L", Cpu::ld, REG_H, REG_L);
        cpu.instructions[0x66] = Instruction::new("LD H,(HL)", Cpu::ld, REG_H, ADDR_HL);
        cpu.instructions[0x67] = Instruction::new("LD H,A", Cpu::ld, REG_H, REG_A);
        cpu.instructions[0x68] = Instruction::new("LD L,B", Cpu::ld, REG_L, REG_B);
        cpu.instructions[0x69] = Instruction::new("ADD HL,HL", Cpu::add_reg_16, None, REG_HL);
        cpu.instructions[0x70] = Instruction::new("LD (HL),B", Cpu::ld, ADDR_HL, REG_B);
        cpu.instructions[0x71] = Instruction::new("LD (HL),C", Cpu::ld, ADDR_HL, REG_C);
        cpu.instructions[0x72] = Instruction::new("LD (HL),D", Cpu::ld, ADDR_HL, REG_D);
        cpu.instructions[0x73] = Instruction::new("LD (HL),E", Cpu::ld, ADDR_HL, REG_E);
        cpu.instructions[0x74] = Instruction::new("LD (HL),H", Cpu::ld, ADDR_HL, REG_H);
        cpu.instructions[0x75] = Instruction::new("LD (HL),L", Cpu::ld, ADDR_HL, REG_L);
        cpu.instructions[0x76] = Instruction::new("HALT", Cpu::halt, None, None);  
        cpu.instructions[0x77] = Instruction::new("LD (HL),A", Cpu::ld, ADDR_HL, REG_A);
        cpu.instructions[0x78] = Instruction::new("LD A,B", Cpu::ld, REG_A, REG_B);
        cpu.instructions[0x79] = Instruction::new("LD A,C", Cpu::ld, REG_A, REG_C);
        cpu.instructions[0x7A] = Instruction::new("LD A,D", Cpu::ld, REG_A, REG_D);
        cpu.instructions[0x7B] = Instruction::new("LD A,E", Cpu::ld, REG_A, REG_E);
        cpu.instructions[0x7C] = Instruction::new("LD A,H", Cpu::ld, REG_A, REG_H);
        cpu.instructions[0x7D] = Instruction::new("LD A,L", Cpu::ld, REG_A, REG_L);
        cpu.instructions[0xA9] = Instruction::new("XOR C", Cpu::xor, None, REG_C);
        cpu.instructions[0xAE] = Instruction::new("XOR (HL)", Cpu::xor, None, ADDR_HL);
        cpu.instructions[0xB1] = Instruction::new("OR C", Cpu::or, None, REG_C);
        cpu.instructions[0xB7] = Instruction::new("OR A", Cpu::or, None, REG_A);
        cpu.instructions[0xC1] = Instruction::new("POP BC", Cpu::pop_reg, REG_BC, None);
        cpu.instructions[0xC3] = Instruction::new("JP u16", Cpu::jp, None, IMM16);
        cpu.instructions[0xC4] = Instruction::new_cond_jump("CALL NZ,u16", Cpu::call, None, IMM16, JumpCondition::NotZero);
        cpu.instructions[0xC5] = Instruction::new("PUSH BC", Cpu::push_reg, None, REG_B);
        cpu.instructions[0xC6] = Instruction::new("ADD A,u8", Cpu::add_reg_8, None, IMM8);
        cpu.instructions[0xC9] = Instruction::new("RET", Cpu::pop_pc, None, None);
        cpu.instructions[0xCB] = Instruction::new("CB", Cpu::cb, None, None);
        cpu.instructions[0xCC] = Instruction::new_cond_jump("CALL Z,u16", Cpu::call, None, IMM16, JumpCondition::Zero);
        cpu.instructions[0xCD] = Instruction::new("CALL u16", Cpu::call, None, IMM16);
        cpu.instructions[0xCE] = Instruction::new("ADC A,u8", Cpu::adc, None, IMM8);
        cpu.instructions[0xCF] = Instruction::new("RST 08H", Cpu::reset, None, IMM8);
        cpu.instructions[0xD0] = Instruction::new_cond_jump("RET NC", Cpu::ret_cond, None, None, JumpCondition::NotCarry);
        cpu.instructions[0xD1] = Instruction::new("POP DE", Cpu::pop_reg, REG_DE, None);
        cpu.instructions[0xD2] = Instruction::new_cond_jump( "JP NC,u16", Cpu::jp, None, IMM16, JumpCondition::NotCarry);
        cpu.instructions[0xD3] = Instruction::new("UNSUPPORTED", Cpu::not_supported, None, None);
        cpu.instructions[0xD4] = Instruction::new_cond_jump( "CALL NC,u16", Cpu::call, None, IMM16, JumpCondition::NotCarry);
        cpu.instructions[0xD5] = Instruction::new( "PUSH DE", Cpu::push_reg, None, REG_DE);
        cpu.instructions[0xD6] = Instruction::new( "SUB A, u8", Cpu::sub_reg_8, None, IMM8);
        cpu.instructions[0xD7] = Instruction::new( "RST 10H", Cpu::reset, None, IMM8);
        cpu.instructions[0xD8] = Instruction::new_cond_jump( "RET C", Cpu::ret_cond, None, None, JumpCondition::Carry);
        cpu.instructions[0xD9] = Instruction::new("RETI", Cpu::reti, None, None);
        cpu.instructions[0xDA] = Instruction::new_cond_jump("JP C,u16", Cpu::jp, None, IMM16, JumpCondition::Carry);
        cpu.instructions[0xE0] = Instruction::new("LD (FF00+u8),A", Cpu::ld, ADDR_ZERO_PAGE_DIRECT, REG_A);
        cpu.instructions[0xE1] = Instruction::new("POP HL", Cpu::pop_reg, REG_HL, None);
        cpu.instructions[0xE5] = Instruction::new("PUSH HL", Cpu::push_reg, None, REG_HL);
        cpu.instructions[0xE6] = Instruction::new("AND A,u8", Cpu::and, None, IMM8);
        cpu.instructions[0xEA] = Instruction::new("LD (u16),A", Cpu::ld, ADDR_IND, REG_A);
        cpu.instructions[0xEE] = Instruction::new("XOR u8", Cpu::xor, None, IMM8);
        cpu.instructions[0xF0] = Instruction::new("LD A,(FF00+u8)", Cpu::ld, REG_A, ADDR_ZERO_PAGE_DIRECT);
        cpu.instructions[0xF1] = Instruction::new("POP AF", Cpu::pop_reg, REG_AF, None);
        cpu.instructions[0xF3] = Instruction::new("DI", Cpu::di, None, None);
        cpu.instructions[0xFA] = Instruction::new("LD A,(u16)", Cpu::ld, REG_A, ADDR_IND);
        cpu.instructions[0xF5] = Instruction::new("PUSH AF", Cpu::push_reg, None, REG_AF);
        cpu.instructions[0xFE] = Instruction::new("CP u8", Cpu::cp, None, IMM8);

        cpu.reset();
        return cpu;
    }

    fn add_reg_16(&mut self)
    {
        let left = self.get_hl();
        let v = left.wrapping_add(self.value);
        self.internal_work();

        self.set_flag_sub(false);
        self.update_flag_half_carry_16(left, self.value, false);
        self.set_flag_carry(v < left);

        self.value = v;
    }

    fn inc_reg(&mut self) {
        self.value = self.inc(self.value as u8) as u16;
    }

    fn inc_reg_16(&mut self) {
        self.value = self.value.wrapping_add(1);
    }

    fn dec_reg_16(&mut self) {
        self.value = self.value.wrapping_add(1);
    }

    fn ld_hl_plus(&mut self) {
        self.set_hl(self.get_hl() + 1);
    }

    fn ld_hl_minus_a(&mut self) {
        self.bus.write(self.get_hl(), self.a);
        self.set_hl(self.get_hl().wrapping_sub(1));
    }

    fn ld_a_hl_minus(&mut self) {
        let v = self.bus.read(self.get_hl());
        self.a = v;
        self.set_hl(self.get_hl().wrapping_sub(1));
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

    fn not_supported(&mut self) {
        unimplemented!("Not supported op: {:#02X}", self.opcode);
    }

    fn fetch_source(&mut self, src: Op) {
        let value: u16 = match src {
            Op::Imm8 => self.fetch() as u16,
            Op::Imm16 => self.fetch16(),
            Op::IndirectAddress => {
                let addr = self.fetch16();
                self.bus.read(addr) as u16
            }
            Op::AddressHL => self.bus.read(self.get_hl()) as u16,
            Op::AddressBC => self.bus.read(self.get_bc()) as u16,
            Op::AddressDE => self.bus.read(self.get_de()) as u16,
            Op::ZeroPageDirect => {
                let addr = 0xFF00 + self.fetch() as u16;
                self.bus.read(addr) as u16
            }
            Op::ZeroPageRegister => {
                let addr = 0xFF00 + self.c as u16;
                self.bus.read(addr) as u16
            }
            Op::Reg(r) => match r {
                Reg::A => self.a as u16,
                Reg::F => self.f as u16,
                Reg::B => self.b as u16,
                Reg::C => self.c as u16,
                Reg::D => self.d as u16,
                Reg::E => self.e as u16,
                Reg::H => self.h as u16,
                Reg::L => self.l as u16,
                Reg::AF => self.get_af(),
                Reg::BC => self.get_bc(),
                Reg::DE => self.get_de(),
                Reg::HL => self.get_hl(),
                Reg::SP => self.sp as u16,
                Reg::PC => self.pc as u16,
            },
        };

        self.value = value;
    }

    fn write_dest(&mut self, dst: Op) {
        match dst {
            Op::Reg(r) => match r {
                Reg::A => self.a = self.value as u8,
                Reg::F => self.f = self.value as u8,
                Reg::B => self.b = self.value as u8,
                Reg::C => self.c = self.value as u8,
                Reg::D => self.d = self.value as u8,
                Reg::E => self.e = self.value as u8,
                Reg::H => self.h = self.value as u8,
                Reg::L => self.l = self.value as u8,
                Reg::AF => self.set_af(self.value),
                Reg::BC => self.set_bc(self.value),
                Reg::DE => self.set_de(self.value),
                Reg::HL => self.set_hl(self.value),
                Reg::SP => self.sp = self.value,
                Reg::PC => self.pc = self.value,
            },
            Op::IndirectAddress => {
                assert!(self.value < 256);
                let addr = self.fetch16() as u16;
                self.bus.write(addr, self.value as u8);
            }
            Op::AddressHL => {
                self.bus.write(self.get_hl(), self.value as u8);
            }
            Op::AddressBC => {
                self.bus.write(self.get_bc(), self.value as u8);
            }
            Op::AddressDE => {
                self.bus.write(self.get_de(), self.value as u8);
            }
            Op::ZeroPageDirect => {
                let addr = 0xFF00 + self.fetch() as u16;
                self.bus.write(addr, self.value as u8);
            }
            Op::ZeroPageRegister => {
                let addr = 0xFF00 + self.c as u16;
                self.bus.write(addr, self.value as u8);
            }
            _ => unimplemented!(),
        }
    }

    pub fn step(&mut self) {
        let ppc = self.pc;
        let cyc = self.cycles;
        self.opcode = self.fetch();
        let &ins = &self.instructions[self.opcode as usize];
        warn!(
            "{:#06x}: OP {:#04x}, {} Cycle: {}",
            ppc, self.opcode, ins.name, cyc
        );

        // for debugging error line - 1
        if self.inst_counter == 16477 - 1 {
            trace!("aa");
        }
        if let Some(src) = ins.src {
            self.fetch_source(src);
            trace!("Value: {:#06x}", self.value);
        } else {
            self.value = 0x1234;
        }

        (ins.call)(self);

        if let Some(dst) = ins.dest {
            self.write_dest(dst);
        }

        self.dump_registers();

        self.inst_counter += 1;
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
    
    fn flag_carry(&self) -> u8 {
        return bit_test(self.f, Self::CARRY_BIT) as u8;
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

    fn push_reg(&mut self) {
        self.push16(self.value);
    }

    fn pop_reg(&mut self) {
        self.value = self.pop16();
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

    fn set_flag_carry(&mut self, carry: bool) {
        if carry {
            self.f = bit_set(self.f, Self::CARRY_BIT)
        } else {
            self.f = bit_clear(self.f, Self::CARRY_BIT)
        }
    }

    fn set_flag_half_carry(&mut self, set: bool) {
        if set {
            self.f = bit_set(self.f, Self::HALF_CARRY_BIT)
        } else {
            self.f = bit_clear(self.f, Self::HALF_CARRY_BIT)
        }
    }

    fn update_flag_half_carry(&mut self, a: u8, b: u8, sub: bool) {
        let is_hc = match sub {
            false => (((a & 0xF).wrapping_add(b & 0xF)) & 0x10) == 0x10,
            true => (((a & 0xF).wrapping_sub(b & 0xF)) & 0x10) == 0x10,
        };

        self.set_flag_half_carry(is_hc);
    }

    fn update_flag_half_carry_16(&mut self, a: u16, b: u16, sub: bool) {
        let is_hc = match sub {
            false => (((a & 0xFFF).wrapping_add(b & 0xFFF)) & 0x1000) == 0x1000,
            true => (((a & 0xFFF).wrapping_sub(b & 0xFFF)) & 0x1000) == 0x1000,
        };

        self.set_flag_half_carry(is_hc);
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

    fn rel_pc(&self, rel: i8) -> u16 {
        return (self.pc as i32 + rel as i32) as u16;
    }

    fn dump_registers(&self) {
        trace!(
            "PC: {:#06x}, AF: {:#06x}, BC: {:#06x}, DE: {:#06x}, HL: {:#06x}, SP: {:#06x}\n",
            self.pc,
            self.get_af(),
            self.get_bc(),
            self.get_de(),
            self.get_hl(),
            self.sp
        );
    }

    fn instr(&self) -> &Instruction {
        return &self.instructions[self.opcode as usize];
    }

    fn cb_bit(&mut self, bit: u8) {
        unimplemented!();
    }

    fn cb_res(&mut self, bit: u8) {
        unimplemented!();
    }

    fn cb_set(&mut self, bit: u8) {
        unimplemented!();
    }

    fn cb_rlc(&mut self) {
        unimplemented!();
    }

    fn cb_rrc(&mut self) {
        self.value = self.do_rrc(self.value as u8) as u16;
    }

    fn cb_rl(&mut self) {
        unimplemented!();
    }

    fn cb_rr(&mut self) {
        self.value = self.do_rr(self.value as u8) as u16;
    }

    fn cb_sla(&mut self) {
        unimplemented!();
    }
    fn cb_sra(&mut self) {
        unimplemented!();
    }
    fn cb_swap(&mut self) {
        unimplemented!();
    }

    fn di(&mut self) {
        self.ime = 0;
    }

    /// Shift the contents of register to the right
    fn cb_srl(&mut self) {
        let v = self.value as u8;

        /* Shift the contents of register to the right.
        That is, the contents of bit 7 are copied to bit 6,
        and the previous contents of bit 6 (before the copy operation) are copied to bit 5.
        The same operation is repeated in sequence for the rest of the register.
        The contents of bit 0 are copied to the CY flag, and bit 7 of register B is reset to 0. */

        let carry = v & 0x1;
        let res = v >> 1;
        self.set_flag_carry(carry == 1);
        self.set_flag_zero(res);
        self.set_flag_sub(false);
        self.set_flag_half_carry(false);

        self.value = res as u16;
    }

    fn cb(&mut self) {
        let cb_opcode = self.fetch();
        let x = (cb_opcode >> 6) & 0x3;
        let y = (cb_opcode >> 3) & 0x7;
        let z = cb_opcode & 0x7;

        let operand = match z {
            0 => Op::Reg(Reg::B),
            1 =>  Op::Reg(Reg::C),
            2 =>  Op::Reg(Reg::D),
            3 =>  Op::Reg(Reg::E),
            4 =>  Op::Reg(Reg::H),
            5 =>  Op::Reg(Reg::L),
            6 => Op::AddressHL,
            7 => Op::Reg(Reg::A),
            _ => unreachable!(),
        };

        error!(
            "CB instruction: {:#02X}, x: {}, y: {}, z: {}, operand: {:?}",
            cb_opcode, x, y, z, operand
        );

        self.fetch_source(operand);

        match x {
            0 => match y {
                0 => self.cb_rlc(),
                1 => self.cb_rrc(),
                2 => self.cb_rl(),
                3 => self.cb_rr(),
                4 => self.cb_sla(),
                5 => self.cb_sra(),
                6 => self.cb_swap(),
                7 => self.cb_srl(),
                _ => unreachable!(),
            },
            1 => self.cb_bit(y),
            2 => self.cb_res(y),
            3 => self.cb_set(y),
            _ => unreachable!(),
        }

        self.write_dest(operand);

        // todo!("CB instruction: {:#02X}", cb_opcode);
    }

    fn nop(&mut self) {}

    fn and(&mut self) {
        self.a &= self.value as u8;
        self.set_flag_zero(self.a);
        self.set_flag_sub(false);
        self.set_flag_half_carry(true);
        self.set_flag_carry(false);
    }

    fn or(&mut self) {
        self.a |= self.value as u8;
        self.set_flag_zero(self.a);
        self.set_flag_sub(false);
        self.set_flag_half_carry(false);
        self.set_flag_carry(false);
    }

    fn xor(&mut self) {
        self.a ^= self.value as u8;
        self.set_flag_zero(self.a);
        self.set_flag_sub(false);
        self.set_flag_half_carry(false);
        self.set_flag_carry(false);
    }

    fn cp(&mut self) {
        let v = self.value as u8;
        let diff = self.a.wrapping_sub(v);
        self.set_flag_zero(diff);
        self.set_flag_sub(true);
        self.update_flag_half_carry(self.a, v, true);
        self.set_flag_carry(self.a < v);
    }

    fn do_rrc(&mut self, v: u8) -> u8
    {
        // Rotate the contents of register A to the right.
        // That is, the contents of bit 7 are copied to bit 6, and the previous contents of bit 6 (before the copy) are copied to bit 5.
        // The same operation is repeated in sequence for the rest of the register. The contents of bit 0 are placed in both the CY flag and bit 7 of register A.
        let carry = v & 1;
        let result = (carry << 7) | (v >> 1);
        self.set_flag_carry(carry == 1);
        self.set_flag_zero(self.a);
        self.set_flag_sub(false);
        self.set_flag_half_carry(false);
        
        result
    }

    fn do_rr(&mut self, v: u8) -> u8
    {
        //Rotate the contents of register A to the right.
        //through the carry (CY) flag.
        //That is, the contents of bit 7 are copied to bit 6, and the previous contents of bit 6 (before the copy) are copied to bit 5.
        //The same operation is repeated in sequence for the rest of the register.
        // The previous contents of the carry flag are copied to bit 7.

        let had_carry = self.flag_carry();
        let will_have_carry= v & 1;
        let res= (had_carry << 7) | (v >> 1);

        self.set_flag_carry(will_have_carry == 1);
        self.set_flag_zero(res);
        self.set_flag_sub(false);
        self.set_flag_half_carry(false);

        return res;
    }

    fn rlca(&mut self)
    {
        todo!("RLCA");
    }

    fn rla(&mut self)
    {
        todo!("RLA");
    }

    fn rra(&mut self)
    {
        self.a = self.do_rr(self.a);
    }

    fn rrca(&mut self)
    {
        self.a = self.do_rrc(self.a);
    }

    fn inc(&mut self, v: u8) -> u8 {
        let res = v.wrapping_add(1);
        self.set_flag_zero(res);
        self.set_flag_sub(false);
        self.update_flag_half_carry(v, 1, false);
        return res;
    }

    fn dec_reg(&mut self) {
        self.value = self.dec(self.value as u8) as u16;
    }

    fn adc(&mut self) {
        let carry = self.flag_carry();
        let v = self.a.wrapping_add(self.value as u8).wrapping_add(carry);
        self.set_flag_zero(v);
        self.set_flag_sub(false);
        self.update_flag_half_carry(self.a as u8, self.value as u8, false);
        self.set_flag_carry(v < self.a);
        self.a = v;
    }

    fn add_reg_8(&mut self) {
        let v = self.a.wrapping_add(self.value as u8);
        self.set_flag_zero(v);
        self.set_flag_sub(false);
        self.update_flag_half_carry(self.a as u8, self.value as u8, false);
        self.set_flag_carry(v < self.a);
        self.a = v;
    }

    fn sub_reg_8(&mut self) {
        let v = self.a.wrapping_sub(self.value as u8);
        self.set_flag_zero(v);
        self.set_flag_sub(true);
        self.update_flag_half_carry(self.a as u8, self.value as u8, true);
        self.set_flag_carry(v > self.a);
        self.a = v;
    }

    fn dec(&mut self, v: u8) -> u8 {
        let res = v.wrapping_sub(1);
        self.set_flag_zero(res);
        self.set_flag_sub(true);
        self.update_flag_half_carry(v, 1, true);
        return res;
    }

    fn do_jump(&mut self, addr: u16) {
        self.internal_work();
        self.pc = addr;
    }

    fn get_jump_condition(&self, condition: JumpCondition) -> u8 {
        return match condition {
            JumpCondition::Zero => self.zero_flag(),
            JumpCondition::NotZero => not(self.zero_flag()),
            JumpCondition::Carry => self.f & 0x10,
            JumpCondition::NotCarry => not(self.f & 0x10),
        };
    }

    fn reti(&mut self) {
        todo!("RETI");
    }
    
    fn jp(&mut self) {
        if let Some(jmp_condition) = self.instr().jump_condition {
            if self.get_jump_condition(jmp_condition) == 0 {
                return;
            }
        }

        self.do_jump(self.value);
    }

    fn ret_cond(&mut self) {
        if let Some(jump_condition) = self.instr().jump_condition {
            if self.get_jump_condition(jump_condition) == 0 {
                return;
            }
        }

        self.pop_pc();
    }

    fn jr(&mut self) {
        self.value = self.rel_pc(self.value as i8);
        self.jp();
    }

    fn call(&mut self) {
        if let Some(jump_condition) = self.instr().jump_condition {
            if self.get_jump_condition(jump_condition) == 0 {
                return;
            }
        }

        self.push_pc();
        self.do_jump(self.value);
    }

    fn halt(&mut self)
    {
        error!("halting");
    }

    fn ld(&mut self) {
        // fetch_source and write_dest do the job
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

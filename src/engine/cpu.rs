use crate::engine::bus::*;
use log::{warn, trace};

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
const fn not(v: u8) -> u8
{
    return match v {
        0 => 1,
        _ => 0
        
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
    ZeroPageRegister ,
    /// LD (FF00+u8),A 
    ZeroPageDirect,
}

#[derive(Debug, Copy, Clone)]
pub enum OpType {
    Load,
    Add,
    Sub,
    And,
    Or,
    Xor,
    Inc,
    Dec,
    Jp,
    Jr,
    Call,
    Ret,
    Push,
    Pop,
    Cp,
    Nop,
    Di,
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
pub struct Instruction 
{
    op_type: OpType,
    mnemonic: &'static str,
    call: InstructionCall,
    dest: Option<Op>,
    src: Option<Op>,
    jump_condition: Option<JumpCondition>,
}

impl Instruction {
    pub fn new(op_type: OpType, mnemonic: &'static str, call: InstructionCall, dest: Option<Op>, src: Option<Op>) -> Self {
        return Instruction {
            op_type,
            mnemonic,
            call,
            dest,
            src,
            jump_condition: None,
        };
    }

    pub fn new_cond_jump(op_type: OpType, mnemonic: &'static str, call: InstructionCall, dest: Option<Op>, src: Option<Op>, cond: JumpCondition) -> Self {
        return Instruction {
            op_type,
            mnemonic,
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
    instructions: [Instruction;256]
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
    pub fn new() -> Self {
         let mut cpu =  Cpu {
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
            instructions: [Instruction::new(OpType::Nop, "UNSUPPORTED", Cpu::not_supported, None, None); 256]
        };

        cpu.instructions[0x00] = Instruction::new(OpType::Nop, "NOP", Cpu::nop, None, None);
        cpu.instructions[0x01] = Instruction::new(OpType::Load, "LD BC,u16", Cpu::ld, Some(Op::Reg(Reg::BC)), Some(Op::Imm16));
        cpu.instructions[0x02] = Instruction::new(OpType::Load, "LD (BC),A", Cpu::ld, Some(Op::AddressBC), Some(Op::Reg(Reg::A)));
        cpu.instructions[0x03] = Instruction::new(OpType::Inc, "INC BC", Cpu::inc_reg_16, Some(Op::Reg(Reg::BC)), Some(Op::Reg(Reg::BC)));
        cpu.instructions[0x04] = Instruction::new(OpType::Inc, "INC B", Cpu::inc_reg, Some(Op::Reg(Reg::B)), Some(Op::Reg(Reg::B)));
        cpu.instructions[0x05] = Instruction::new(OpType::Dec, "DEC B", Cpu::dec_reg, Some(Op::Reg(Reg::B)), Some(Op::Reg(Reg::B)));
        cpu.instructions[0x06] = Instruction::new(OpType::Load, "LD B,u8", Cpu::ld, Some(Op::Reg(Reg::B)), Some(Op::Imm8));
        cpu.instructions[0x0D] = Instruction::new(OpType::Dec, "DEC C", Cpu::dec_reg, Some(Op::Reg(Reg::C)), Some(Op::Reg(Reg::C)));
        cpu.instructions[0x0E] = Instruction::new(OpType::Load, "LD C,u8", Cpu::ld, Some(Op::Reg(Reg::C)), Some(Op::Imm8));
        cpu.instructions[0x11] = Instruction::new(OpType::Load, "LD DE,u16", Cpu::ld, Some(Op::Reg(Reg::DE)), Some(Op::Imm16));
        cpu.instructions[0x12] = Instruction::new(OpType::Load, "LD (DE),A", Cpu::ld, Some(Op::AddressDE), Some(Op::Reg(Reg::A)));
        cpu.instructions[0x13] = Instruction::new(OpType::Inc, "INC DE", Cpu::inc_reg_16, Some(Op::Reg(Reg::DE)), Some(Op::Reg(Reg::DE)));
        cpu.instructions[0x14] = Instruction::new(OpType::Inc, "INC D", Cpu::inc_reg, Some(Op::Reg(Reg::D)), Some(Op::Reg(Reg::D)));
        cpu.instructions[0x15] = Instruction::new(OpType::Dec, "DEC D", Cpu::dec_reg, Some(Op::Reg(Reg::D)), Some(Op::Reg(Reg::D)));
        cpu.instructions[0x16] = Instruction::new(OpType::Load, "LD D,u8", Cpu::ld, Some(Op::Reg(Reg::D)), Some(Op::Imm8));
        // cpu.instructions[0x17] = Instruction::new(OpType::Rla, "RLA", Cpu::rla, None, None);
        cpu.instructions[0x18] = Instruction::new(OpType::Jr, "JR i8", Cpu::jr, None, Some(Op::Imm8));
        cpu.instructions[0x22] = Instruction::new(OpType::Load, "LD (HL+),A", Cpu::ld_hl_plus, Some(Op::AddressHL), Some(Op::Reg(Reg::A)));
        cpu.instructions[0x1A] = Instruction::new(OpType::Load, "LD A,(DE)", Cpu::ld, Some(Op::Reg(Reg::A)), Some(Op::AddressDE));
        cpu.instructions[0x1C] = Instruction::new(OpType::Inc, "INC E", Cpu::inc_reg, Some(Op::Reg(Reg::E)), Some(Op::Reg(Reg::E)));
        cpu.instructions[0x20] = Instruction::new_cond_jump(OpType::Jr, "JR NZ,i8", Cpu::jr, None, Some(Op::Imm8), JumpCondition::NotZero);
        cpu.instructions[0x21] = Instruction::new(OpType::Load, "LD HL,u16", Cpu::ld, Some(Op::Reg(Reg::HL)), Some(Op::Imm16));
        cpu.instructions[0x23] = Instruction::new(OpType::Inc, "INC HL", Cpu::inc_reg_16, Some(Op::Reg(Reg::HL)), Some(Op::Reg(Reg::HL)));
        cpu.instructions[0x24] = Instruction::new(OpType::Inc, "INC H", Cpu::inc_reg, Some(Op::Reg(Reg::H)), Some(Op::Reg(Reg::H)));
        cpu.instructions[0x26] = Instruction::new(OpType::Load, "LD H,u8", Cpu::ld, Some(Op::Reg(Reg::H)), Some(Op::Imm8));
        cpu.instructions[0x28] = Instruction::new_cond_jump(OpType::Jr, "JR Z,i8", Cpu::jr, None, Some(Op::Imm8), JumpCondition::Zero);
        cpu.instructions[0x2A] = Instruction::new(OpType::Load, "LD A,(HL+)", Cpu::ld_hl_plus, Some(Op::Reg(Reg::A)), Some(Op::AddressHL));
        cpu.instructions[0x2C] = Instruction::new(OpType::Inc, "INC L", Cpu::inc_reg, Some(Op::Reg(Reg::L)), Some(Op::Reg(Reg::L)));
        cpu.instructions[0x2D] = Instruction::new(OpType::Dec, "DEC L", Cpu::dec_reg, Some(Op::Reg(Reg::L)), Some(Op::Reg(Reg::L)));
        cpu.instructions[0x31] = Instruction::new(OpType::Load, "LD SP,u16", Cpu::ld, Some(Op::Reg(Reg::SP)), Some(Op::Imm16));
        cpu.instructions[0x32] = Instruction::new(OpType::Load, "LD (HL-),A", Cpu::ld_hl_minus_a, None, None);
        cpu.instructions[0x3A] = Instruction::new(OpType::Load, "LD A, (HL-)", Cpu::ld_a_hl_minus, None, None);
        cpu.instructions[0x46] = Instruction::new(OpType::Load, "LD B,(HL)", Cpu::ld, Some(Op::Reg(Reg::B)), Some(Op::AddressHL));
        cpu.instructions[0x3E] = Instruction::new(OpType::Load, "LD A,u8", Cpu::ld, Some(Op::Reg(Reg::A)), Some(Op::Imm8));
        cpu.instructions[0x47] = Instruction::new(OpType::Load, "LD B,A", Cpu::ld, Some(Op::Reg(Reg::B)), Some(Op::Reg(Reg::A)));
        cpu.instructions[0x4E] = Instruction::new(OpType::Load, "LD C,(HL)", Cpu::ld, Some(Op::Reg(Reg::C)), Some(Op::AddressHL));
        cpu.instructions[0x56] = Instruction::new(OpType::Load, "LD D,(HL)", Cpu::ld, Some(Op::Reg(Reg::D)), Some(Op::AddressHL));
        cpu.instructions[0x77] = Instruction::new(OpType::Load, "LD (HL),A", Cpu::ld, Some(Op::AddressHL), Some(Op::Reg(Reg::A)));
        cpu.instructions[0x78] = Instruction::new(OpType::Load, "LD A,B", Cpu::ld, Some(Op::Reg(Reg::A)), Some(Op::Reg(Reg::B)));
        cpu.instructions[0x7C] = Instruction::new(OpType::Load, "LD A,H", Cpu::ld, Some(Op::Reg(Reg::A)), Some(Op::Reg(Reg::H))); 
        cpu.instructions[0x7D] = Instruction::new(OpType::Load, "LD A,L", Cpu::ld, Some(Op::Reg(Reg::A)), Some(Op::Reg(Reg::L)));
        cpu.instructions[0xA9] = Instruction::new(OpType::Xor, "XOR C", Cpu::xor, None, Some(Op::Reg(Reg::C)));
        cpu.instructions[0xAE] = Instruction::new(OpType::Xor, "XOR (HL)", Cpu::xor, None, Some(Op::AddressHL));
        cpu.instructions[0xB1] = Instruction::new(OpType::Or, "OR C", Cpu::or, None, Some(Op::Reg(Reg::C)));
        cpu.instructions[0xB7] = Instruction::new(OpType::Or, "OR A", Cpu::or, None, Some(Op::Reg(Reg::A)));
        cpu.instructions[0xC1] = Instruction::new(OpType::Pop, "POP BC", Cpu::pop_reg, Some(Op::Reg(Reg::BC)), None);
        cpu.instructions[0xC3] = Instruction::new(OpType::Jp, "JP u16", Cpu::jp, None, Some(Op::Imm16));
        cpu.instructions[0xC4] = Instruction::new_cond_jump(OpType::Call, "CALL NZ,u16", Cpu::call, None, Some(Op::Imm16), JumpCondition::NotZero);
        cpu.instructions[0xC5] = Instruction::new(OpType::Push, "PUSH BC", Cpu::push_reg, None, Some(Op::Reg(Reg::BC)));
        cpu.instructions[0xC6] = Instruction::new(OpType::Add, "ADD A,u8", Cpu::add_reg_8, None, Some(Op::Imm8));
        cpu.instructions[0xC9] = Instruction::new(OpType::Ret, "RET", Cpu::pop_pc, None, None);
        cpu.instructions[0xCB] = Instruction::new(OpType::Nop, "CB", Cpu::cb, None, None);
        cpu.instructions[0xCD] = Instruction::new(OpType::Call, "CALL u16", Cpu::call, None, Some(Op::Imm16));
        cpu.instructions[0xD5] = Instruction::new(OpType::Push, "PUSH DE", Cpu::push_reg, None, Some(Op::Reg(Reg::DE)));
        cpu.instructions[0xD6] = Instruction::new(OpType::Sub, "SUB A, u8", Cpu::sub_reg_8, None, Some(Op::Imm8));
        cpu.instructions[0xE0] = Instruction::new(OpType::Load, "LD (FF00+u8),A", Cpu::ld, Some(Op::ZeroPageDirect), Some(Op::Reg(Reg::A)));
        cpu.instructions[0xE1] = Instruction::new(OpType::Pop, "POP HL", Cpu::pop_reg, Some(Op::Reg(Reg::HL)), None);
        cpu.instructions[0xE5] = Instruction::new(OpType::Push, "PUSH HL", Cpu::push_reg, None, Some(Op::Reg(Reg::HL)));
        cpu.instructions[0xE6] = Instruction::new(OpType::And, "AND A,u8", Cpu::and, None, Some(Op::Imm8));
        cpu.instructions[0xEA] = Instruction::new(OpType::Load, "LD (u16),A", Cpu::ld, Some(Op::IndirectAddress), Some(Op::Reg(Reg::A)));
        cpu.instructions[0xF0] = Instruction::new(OpType::Load, "LD A,(FF00+u8)", Cpu::ld, Some(Op::Reg(Reg::A)), Some(Op::ZeroPageDirect));
        cpu.instructions[0xF1] = Instruction::new(OpType::Pop, "POP AF", Cpu::pop_reg, Some(Op::Reg(Reg::AF)), None);
        cpu.instructions[0xF3] = Instruction::new(OpType::Di, "DI", |s|{s.ime = 0;}, None, None);
        cpu.instructions[0xFA] =  Instruction::new(OpType::Load, "LD A,(u16)", Cpu::ld, Some(Op::Reg(Reg::A)), Some(Op::IndirectAddress));
        cpu.instructions[0xF5] = Instruction::new(OpType::Push, "PUSH AF", Cpu::push_reg, None, Some(Op::Reg(Reg::AF)));
        cpu.instructions[0xFE] = Instruction::new(OpType::Cp, "CP u8", Cpu::cp, None, Some(Op::Imm8));

        cpu.reset();
        return cpu;

    }

    fn inc_reg(&mut self)
    {
        self.value = self.inc(self.value as u8) as u16;
    }

    fn inc_reg_16(&mut self)
    {
        self.value = self.value.wrapping_add(1);
    }

    fn dec_reg_16(&mut self)
    {
        self.value = self.value.wrapping_add(1);
    }

    fn ld_hl_plus(&mut self) {
        self.set_hl(self.get_hl() + 1);
    }

   fn ld_hl_minus_a(&mut self)
   {
        self.bus.write(self.get_hl(), self.a);
        self.set_hl(self.get_hl().wrapping_sub(1));
   }

    fn ld_a_hl_minus(&mut self)
    {
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

    fn fetch_source(&mut self, src: Op)
    {
        let value :u16 = match src {
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
            Op::Reg(r) => 
                match r {
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
            }
        };

        self.value = value;
    }


    fn write_dest(&mut self, dst: Op)
    {
        match dst {
            Op::Reg(r) => {
                match r {
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
                }
            }
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
            _ => unimplemented!()
            
        }

    }

    pub fn step(&mut self) {
        let ppc = self.pc;
        let cyc = self.cycles;
        self.opcode = self.fetch();
        let &ins = &self.instructions[self.opcode as usize];
        warn!("{:#06x}: OP {:#04x}, {} Cycle: {}", ppc, self.opcode, ins.mnemonic,  cyc);
        
        // for debugging error line - 1
        if self.inst_counter == 16477 -1 
        {
            trace!("aa");
        }
        if let Some(src) = ins.src
        {
            self.fetch_source(src);
            trace!("Value: {:#06x}", self.value);
        }
        else {
            self.value = 0x1234;
        }

        (ins.call)(self);

        if let Some(dst) = ins.dest
        {
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

    fn set_af(&mut self, v: u16) {
        Cpu::set_reg_pair(&mut self.a, &mut self.f, v);
    }

    fn set_bc(&mut self, v: u16) {
        Cpu::set_reg_pair(&mut self.b, &mut self.c, v);
    }

    fn set_de(&mut self, v: u16) {
        Cpu::set_reg_pair(&mut self.d, &mut self.e, v);
    }

    fn push_reg(&mut self)
    {
        self.push16(self.value);
    }

    fn pop_reg(&mut self)
    {
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
            true => (((a & 0xF).wrapping_sub(b & 0xF)) & 0x10) == 0x10
        };
            
        self.set_flag_half_carry(is_hc);
    }

    fn update_flag_half_carry_16(&mut self, a: u16, b: u16, sub: bool) {

        let is_hc = match sub {
            false => (((a & 0xFFF).wrapping_add(b & 0xFFF)) & 0x1000) == 0x1000,
            true  => (((a & 0xFFF).wrapping_sub(b & 0xFFF)) & 0x1000) == 0x1000,
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

    fn dump_registers(&self)
    {
        trace!("PC: {:#06x}, AF: {:#06x}, BC: {:#06x}, DE: {:#06x}, HL: {:#06x}, SP: {:#06x}\n", self.pc, self.get_af(), self.get_bc(), self.get_de(), self.get_hl(), self.sp);
    }

    fn instr(&self) -> &Instruction {
        return &self.instructions[self.opcode as usize];
    }

    fn cb(&mut self) {
        todo!("CB instruction");
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

    fn inc(&mut self, v: u8) -> u8 {
        let res = v.wrapping_add(1);
        self.set_flag_zero(res);
        self.set_flag_sub(false);
        self.update_flag_half_carry(v, 1, false);
        return res;
    }
    
    fn dec_reg(&mut self)
    {
        self.value = self.dec(self.value as u8) as u16;
    }

    fn add_reg_8(&mut self)
    {
        let v  = self.a.wrapping_add(self.value as u8);
        self.set_flag_zero(v);
        self.set_flag_sub(false);
        self.update_flag_half_carry(self.a as u8, self.value as u8, false);
        self.set_flag_carry(v < self.a);
        self.a = v;
    }

    fn sub_reg_8(&mut self)
    {
        let v  = self.a.wrapping_sub(self.value as u8);
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

    fn get_jump_condition(&self, condition: JumpCondition) -> u8
    {
        return match condition {
            JumpCondition::Zero => self.zero_flag(),
            JumpCondition::NotZero => not(self.zero_flag()),
            JumpCondition::Carry => self.f & 0x10,
            JumpCondition::NotCarry => not(self.f & 0x10),
        };
    }

    fn jp(&mut self) {
        if let Some(jmp_condition) = self.instr().jump_condition {
            if self.get_jump_condition(jmp_condition) == 0 {
                return;
            }
        }

        self.do_jump(self.value);
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

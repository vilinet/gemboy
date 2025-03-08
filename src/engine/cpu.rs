use crate::engine::bit_utils::*;
use crate::engine::bus::*;
use crate::engine::interrupts::InterruptType;
use log::{error, trace, warn};
use crate::engine::interrupts::InterruptType::TIMER;
use crate::engine::timer::TimerInterruptRaised;

// Detailed T-cycle instruction table: https://izik1.github.io/gbops/
// An older table that is more helpful for the instruction's job: https://meganesu.github.io/generate-gb-opcodes/

#[derive(Debug, Copy, Clone)]
enum Reg {
    A,
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
}

#[derive(Debug, Copy, Clone)]
enum Op {
    Reg(Reg),
    Imm8,
    Imm16,
    IndirectAddress,
    AddressHL,
    AddressBC,
    AddressDE,
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
    pub name: &'static str,
    call: InstructionCall,
    dest: Option<Op>,
    src: Option<Op>,
    jump_condition: Option<JumpCondition>,
}

impl Instruction {
    fn new(name: &'static str, call: InstructionCall, dest: Option<Op>, src: Option<Op>) -> Self {
        Instruction {
            name,
            call,
            dest,
            src,
            jump_condition: None,
        }
    }

    fn new_cond_jump(
        name: &'static str,
        call: InstructionCall,
        dest: Option<Op>,
        src: Option<Op>,
        cond: JumpCondition,
    ) -> Self {
        Instruction {
            name,
            call,
            dest,
            src,
            jump_condition: Some(cond),
        }
    }
}

/// Gameboy CPU emulator.
pub struct Cpu {
    pub opcode: u8,
    pub cb_opcode: u8,
    pub a: u8,
    pub b: u8,
    pub c: u8,
    pub d: u8,
    pub e: u8,
    pub f: u8,
    pub h: u8,
    pub l: u8,
    pub sp: u16,
    pub pc: u16,
    pub bus: Bus,

    /// The value cpu reads and writes based on the [Op], instructions need to use this value and modify.
    value: u16,

    /* T Cycles */
    cycles: u32,
    instructions: [Instruction; 256],
    /// Whether we are fetching the next byte for a CB instruction.
    /// In this time slot need to delay interrupt to the next cycle to avoid losing state.
    fetching_cb: bool,
    ime: bool,
    halted: bool,
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
            fetching_cb: false,
            value: 0,
            bus: Bus::new(),
            opcode: 0,
            cb_opcode: 0,
            ime: false,
            a: 0,
            b: 0,
            c: 0,
            d: 0,
            e: 0,
            f: 0,
            h: 0,
            l: 0,
            sp: 0,
            pc: 0,
            cycles: 0,
            instructions: [Instruction::new("UNSUPPORTED", Cpu::not_supported, None, None); 256],
            halted: false,
        };

        let reg_a = Some(Op::Reg(Reg::A));
        let reg_b = Some(Op::Reg(Reg::B));
        let reg_c = Some(Op::Reg(Reg::C));
        let reg_d = Some(Op::Reg(Reg::D));
        let reg_e = Some(Op::Reg(Reg::E));
        let reg_h = Some(Op::Reg(Reg::H));
        let reg_l = Some(Op::Reg(Reg::L));
        let reg_af = Some(Op::Reg(Reg::AF));
        let reg_bc = Some(Op::Reg(Reg::BC));
        let reg_de = Some(Op::Reg(Reg::DE));
        let reg_hl = Some(Op::Reg(Reg::HL));
        let reg_sp = Some(Op::Reg(Reg::SP));
        let addr_hl = Some(Op::AddressHL);
        let addr_bc = Some(Op::AddressBC);
        let addr_de = Some(Op::AddressDE);
        let addr_ind = Some(Op::IndirectAddress);

        let imm8 = Some(Op::Imm8);
        let imm16 = Some(Op::Imm16);

        cpu.instructions[0x00] = Instruction::new("NOP", Cpu::nop, None, None);
        cpu.instructions[0x01] = Instruction::new("LD BC,u16", Cpu::ld, reg_bc, imm16);
        cpu.instructions[0x02] = Instruction::new("LD (BC),A", Cpu::ld, addr_bc, reg_a);
        cpu.instructions[0x03] = Instruction::new("INC BC", Cpu::inc_reg_16, reg_bc, reg_bc);
        cpu.instructions[0x04] = Instruction::new("INC B", Cpu::inc_reg, reg_b, reg_b);
        cpu.instructions[0x05] = Instruction::new("DEC B", Cpu::dec_reg, reg_b, reg_b);
        cpu.instructions[0x06] = Instruction::new("LD B,u8", Cpu::ld, reg_b, imm8);
        cpu.instructions[0x07] = Instruction::new("RLCA", Cpu::rlca, None, None);
        cpu.instructions[0x08] = Instruction::new("LD (u16),SP", Cpu::ld_sp, None, None);
        cpu.instructions[0x09] = Instruction::new("ADD HL,BC", Cpu::add_reg_16, None, reg_bc);
        cpu.instructions[0x0A] = Instruction::new("LD A,(BC)", Cpu::ld, reg_a, addr_bc);
        cpu.instructions[0x0B] = Instruction::new("DEC BC", Cpu::dec_reg_16, reg_bc, reg_bc);
        cpu.instructions[0x0C] = Instruction::new("INC C", Cpu::inc_reg, reg_c, reg_c);
        cpu.instructions[0x0D] = Instruction::new("DEC C", Cpu::dec_reg, reg_c, reg_c);
        cpu.instructions[0x0E] = Instruction::new("LD C,u8", Cpu::ld, reg_c, imm8);
        cpu.instructions[0x0F] = Instruction::new("RRCA", Cpu::rrca, None, None);

        cpu.instructions[0x10] = Instruction::new("STOP", Cpu::stop, None, None);
        cpu.instructions[0x11] = Instruction::new("LD DE,u16", Cpu::ld, reg_de, imm16);
        cpu.instructions[0x12] = Instruction::new("LD (DE),A", Cpu::ld, addr_de, reg_a);
        cpu.instructions[0x13] = Instruction::new("INC DE", Cpu::inc_reg_16, reg_de, reg_de);
        cpu.instructions[0x14] = Instruction::new("INC D", Cpu::inc_reg, reg_d, reg_d);
        cpu.instructions[0x15] = Instruction::new("DEC D", Cpu::dec_reg, reg_d, reg_d);
        cpu.instructions[0x16] = Instruction::new( "LD D,u8", Cpu::ld, reg_d, imm8);
        cpu.instructions[0x17] = Instruction::new("RLA", Cpu::rla, None, None);
        cpu.instructions[0x18] = Instruction::new("JR i8", Cpu::jr, None, imm8);
        cpu.instructions[0x19] = Instruction::new("ADD HL,DE", Cpu::add_reg_16, None, reg_de);
        cpu.instructions[0x1A] = Instruction::new("LD A,(DE)", Cpu::ld, reg_a, addr_de);
        cpu.instructions[0x1B] = Instruction::new("DEC DE", Cpu::dec_reg_16, reg_de, reg_de);
        cpu.instructions[0x1C] = Instruction::new("INC E", Cpu::inc_reg, reg_e, reg_e);
        cpu.instructions[0x1D] = Instruction::new("DEC E", Cpu::dec_reg, reg_e, reg_e);
        cpu.instructions[0x1E] = Instruction::new("LD E,u8", Cpu::ld, reg_e, imm8);
        cpu.instructions[0x1F] = Instruction::new("RRA", Cpu::rra, None, None);

        cpu.instructions[0x20] = Instruction::new_cond_jump("JR NZ,i8", Cpu::jr, None, imm8, JumpCondition::NotZero);
        cpu.instructions[0x21] = Instruction::new("LD HL,u16", Cpu::ld, reg_hl, imm16);
        cpu.instructions[0x22] = Instruction::new("LD (HL+),A", Cpu::ld_hl_plus_a, None, None);
        cpu.instructions[0x23] = Instruction::new("INC HL", Cpu::inc_reg_16, reg_hl, reg_hl);
        cpu.instructions[0x24] = Instruction::new("INC H", Cpu::inc_reg, reg_h, reg_h);
        cpu.instructions[0x25] = Instruction::new("DEC H", Cpu::dec_reg, reg_h, reg_h);
        cpu.instructions[0x26] = Instruction::new("LD H,u8", Cpu::ld, reg_h, imm8);
        cpu.instructions[0x27] = Instruction::new("DAA", Cpu::daa, None, None);
        cpu.instructions[0x28] = Instruction::new_cond_jump("JR Z,i8", Cpu::jr, None, imm8, JumpCondition::Zero);
        cpu.instructions[0x29] = Instruction::new("ADD HL,HL", Cpu::add_reg_16, None, reg_hl);
        cpu.instructions[0x2A] = Instruction::new("LD A,(HL+)", Cpu::ld_a_hl_plus, None, None);
        cpu.instructions[0x2B] = Instruction::new("DEC HL", Cpu::dec_reg_16, reg_hl, reg_hl);
        cpu.instructions[0x2C] = Instruction::new("INC L", Cpu::inc_reg, reg_l, reg_l);
        cpu.instructions[0x2D] = Instruction::new("DEC L", Cpu::dec_reg, reg_l, reg_l);
        cpu.instructions[0x2E] = Instruction::new("LD L,u8", Cpu::ld, reg_l, imm8);
        cpu.instructions[0x2F] = Instruction::new("CPL", Cpu::cpl, None, None);

        cpu.instructions[0x30] = Instruction::new_cond_jump( "JR NC,i8", Cpu::jr, None, imm8, JumpCondition::NotCarry);
        cpu.instructions[0x31] = Instruction::new("LD SP,u16", Cpu::ld, reg_sp, imm16);
        cpu.instructions[0x32] = Instruction::new("LD (HL-),A", Cpu::ld_hl_minus_a, None, None);
        cpu.instructions[0x33] = Instruction::new("INC SP", Cpu::inc_reg_16, reg_sp, reg_sp);
        cpu.instructions[0x34] = Instruction::new("INC (HL)", Cpu::inc_reg, addr_hl, addr_hl);
        cpu.instructions[0x35] = Instruction::new("DEC (HL)", Cpu::dec_reg, addr_hl, addr_hl);
        cpu.instructions[0x36] = Instruction::new("LD (HL),u8", Cpu::ld, addr_hl, imm8);
        cpu.instructions[0x37] = Instruction::new("SCF", Cpu::scf, None, None);
        cpu.instructions[0x38] = Instruction::new_cond_jump("JR C,i8", Cpu::jr, None, imm8, JumpCondition::Carry);
        cpu.instructions[0x39] = Instruction::new("ADD HL,SP", Cpu::add_reg_16, None, reg_sp);
        cpu.instructions[0x3A] = Instruction::new("LD A, (HL-)", Cpu::ld_a_hl_minus, None, None);
        cpu.instructions[0x3B] = Instruction::new("DEC SP", Cpu::dec_reg_16, reg_sp, reg_sp);
        cpu.instructions[0x3C] = Instruction::new("INC A", Cpu::inc_reg, reg_a, reg_a);
        cpu.instructions[0x3D] = Instruction::new("DEC A", Cpu::dec_reg, reg_a, reg_a);
        cpu.instructions[0x3E] = Instruction::new("LD A,u8", Cpu::ld, reg_a, imm8);
        cpu.instructions[0x3F] = Instruction::new("CCF", Cpu::ccf, None, None);

        cpu.instructions[0x40] = Instruction::new("LD B,B", Cpu::ld, reg_b, reg_b);
        cpu.instructions[0x41] = Instruction::new("LD B,C", Cpu::ld, reg_b, reg_c);
        cpu.instructions[0x42] = Instruction::new("LD B,D", Cpu::ld, reg_b, reg_d);
        cpu.instructions[0x43] = Instruction::new("LD B,E", Cpu::ld, reg_b, reg_e);
        cpu.instructions[0x44] = Instruction::new("LD B,H", Cpu::ld, reg_b, reg_h);
        cpu.instructions[0x45] = Instruction::new("LD B,L", Cpu::ld, reg_b, reg_l);
        cpu.instructions[0x46] = Instruction::new("LD B,(HL)", Cpu::ld, reg_b, addr_hl);
        cpu.instructions[0x47] = Instruction::new("LD B,A", Cpu::ld, reg_b, reg_a);
        cpu.instructions[0x48] = Instruction::new("LD C,B", Cpu::ld, reg_c, reg_b);
        cpu.instructions[0x49] = Instruction::new("LD C,C", Cpu::ld, reg_c, reg_c);
        cpu.instructions[0x4A] = Instruction::new("LD C,D", Cpu::ld, reg_c, reg_d);
        cpu.instructions[0x4B] = Instruction::new("LD C,E", Cpu::ld, reg_c, reg_e);
        cpu.instructions[0x4C] = Instruction::new("LD C,H", Cpu::ld, reg_c, reg_h);
        cpu.instructions[0x4D] = Instruction::new("LD C,L", Cpu::ld, reg_c, reg_l);
        cpu.instructions[0x4E] = Instruction::new("LD C,(HL)", Cpu::ld, reg_c, addr_hl);
        cpu.instructions[0x4F] = Instruction::new("LD C,A", Cpu::ld, reg_c, reg_a);

        cpu.instructions[0x50] = Instruction::new("LD D,B", Cpu::ld, reg_d, reg_b);
        cpu.instructions[0x51] = Instruction::new("LD D,C", Cpu::ld, reg_d, reg_c);
        cpu.instructions[0x52] = Instruction::new("LD D,D", Cpu::ld, reg_d, reg_d);
        cpu.instructions[0x53] = Instruction::new("LD D,E", Cpu::ld, reg_d, reg_e);
        cpu.instructions[0x54] = Instruction::new("LD D,H", Cpu::ld, reg_d, reg_h);
        cpu.instructions[0x55] = Instruction::new("LD D,L", Cpu::ld, reg_d, reg_l);
        cpu.instructions[0x56] = Instruction::new("LD D,(HL)", Cpu::ld, reg_d, addr_hl);
        cpu.instructions[0x57] = Instruction::new("LD D,A", Cpu::ld, reg_d, reg_a);
        cpu.instructions[0x58] = Instruction::new("LD E,B", Cpu::ld, reg_e, reg_b);
        cpu.instructions[0x59] = Instruction::new("LD E,C", Cpu::ld, reg_e, reg_c);
        cpu.instructions[0x5A] = Instruction::new("LD E,D", Cpu::ld, reg_e, reg_d);
        cpu.instructions[0x5B] = Instruction::new("LD E,E", Cpu::ld, reg_e, reg_e);
        cpu.instructions[0x5C] = Instruction::new("LD E,H", Cpu::ld, reg_e, reg_h);
        cpu.instructions[0x5D] = Instruction::new("LD E,L", Cpu::ld, reg_e, reg_l);
        cpu.instructions[0x5E] = Instruction::new("LD E,(HL)", Cpu::ld, reg_e, addr_hl);
        cpu.instructions[0x5F] = Instruction::new("LD E,A", Cpu::ld, reg_e, reg_a);

        cpu.instructions[0x60] = Instruction::new("LD H,B", Cpu::ld, reg_h, reg_b);
        cpu.instructions[0x61] = Instruction::new("LD H,C", Cpu::ld, reg_h, reg_c);
        cpu.instructions[0x62] = Instruction::new("LD H,D", Cpu::ld, reg_h, reg_d);
        cpu.instructions[0x63] = Instruction::new("LD H,E", Cpu::ld, reg_h, reg_e);
        cpu.instructions[0x64] = Instruction::new("LD H,H", Cpu::ld, reg_h, reg_h);
        cpu.instructions[0x65] = Instruction::new("LD H,L", Cpu::ld, reg_h, reg_l);
        cpu.instructions[0x66] = Instruction::new("LD H,(HL)", Cpu::ld, reg_h, addr_hl);
        cpu.instructions[0x67] = Instruction::new("LD H,A", Cpu::ld, reg_h, reg_a);
        cpu.instructions[0x68] = Instruction::new("LD L,B", Cpu::ld, reg_l, reg_b);
        cpu.instructions[0x69] = Instruction::new("LD L,C", Cpu::ld, reg_l, reg_c);
        cpu.instructions[0x6A] = Instruction::new("LD L,D", Cpu::ld, reg_l, reg_d);
        cpu.instructions[0x6B] = Instruction::new("LD L,E", Cpu::ld, reg_l, reg_e);
        cpu.instructions[0x6C] = Instruction::new("LD L,H", Cpu::ld, reg_l, reg_h);
        cpu.instructions[0x6D] = Instruction::new("LD L,L", Cpu::ld, reg_l, reg_l);
        cpu.instructions[0x6E] = Instruction::new("LD L,(HL)", Cpu::ld, reg_l, addr_hl);
        cpu.instructions[0x6F] = Instruction::new("LD L,A", Cpu::ld, reg_l, reg_a);

        cpu.instructions[0x70] = Instruction::new("LD (HL),B", Cpu::ld, addr_hl, reg_b);
        cpu.instructions[0x71] = Instruction::new("LD (HL),C", Cpu::ld, addr_hl, reg_c);
        cpu.instructions[0x72] = Instruction::new("LD (HL),D", Cpu::ld, addr_hl, reg_d);
        cpu.instructions[0x73] = Instruction::new("LD (HL),E", Cpu::ld, addr_hl, reg_e);
        cpu.instructions[0x74] = Instruction::new("LD (HL),H", Cpu::ld, addr_hl, reg_h);
        cpu.instructions[0x75] = Instruction::new("LD (HL),L", Cpu::ld, addr_hl, reg_l);
        cpu.instructions[0x76] = Instruction::new("HALT", Cpu::halt, None, None);
        cpu.instructions[0x77] = Instruction::new("LD (HL),A", Cpu::ld, addr_hl, reg_a);
        cpu.instructions[0x78] = Instruction::new("LD A,B", Cpu::ld, reg_a, reg_b);
        cpu.instructions[0x79] = Instruction::new("LD A,C", Cpu::ld, reg_a, reg_c);
        cpu.instructions[0x7A] = Instruction::new("LD A,D", Cpu::ld, reg_a, reg_d);
        cpu.instructions[0x7B] = Instruction::new("LD A,E", Cpu::ld, reg_a, reg_e);
        cpu.instructions[0x7C] = Instruction::new("LD A,H", Cpu::ld, reg_a, reg_h);
        cpu.instructions[0x7D] = Instruction::new("LD A,L", Cpu::ld, reg_a, reg_l);
        cpu.instructions[0x7E] = Instruction::new("LD A,(HL)", Cpu::ld, reg_a, addr_hl);
        cpu.instructions[0x7F] = Instruction::new("LD A,A", Cpu::ld, reg_a, reg_a);

        cpu.instructions[0x80] = Instruction::new("ADD A,B", Cpu::add_reg_8, None, reg_b);
        cpu.instructions[0x81] = Instruction::new("ADD A,C", Cpu::add_reg_8, None, reg_c);
        cpu.instructions[0x82] = Instruction::new("ADD A,D", Cpu::add_reg_8, None, reg_d);
        cpu.instructions[0x83] = Instruction::new("ADD A,E", Cpu::add_reg_8, None, reg_e);
        cpu.instructions[0x84] = Instruction::new("ADD A,H", Cpu::add_reg_8, None, reg_h);
        cpu.instructions[0x85] = Instruction::new("ADD A,L", Cpu::add_reg_8, None, reg_l);
        cpu.instructions[0x86] = Instruction::new("ADD A,(HL)", Cpu::add_reg_8, None, addr_hl);
        cpu.instructions[0x87] = Instruction::new("ADD A,A", Cpu::add_reg_8, None, reg_a);
        cpu.instructions[0x88] = Instruction::new("ADC A,B", Cpu::adc, None, reg_b);
        cpu.instructions[0x89] = Instruction::new("ADC A,C", Cpu::adc, None, reg_c);
        cpu.instructions[0x8A] = Instruction::new("ADC A,D", Cpu::adc, None, reg_d);
        cpu.instructions[0x8B] = Instruction::new("ADC A,E", Cpu::adc, None, reg_e);
        cpu.instructions[0x8C] = Instruction::new("ADC A,H", Cpu::adc, None, reg_h);
        cpu.instructions[0x8D] = Instruction::new("ADC A,L", Cpu::adc, None, reg_l);
        cpu.instructions[0x8E] = Instruction::new("ADC A,(HL)", Cpu::adc, None, addr_hl);
        cpu.instructions[0x8F] = Instruction::new("ADC A,A", Cpu::adc, None, reg_a);

        cpu.instructions[0x90] = Instruction::new("SUB B", Cpu::sub_reg_8, None, reg_b);
        cpu.instructions[0x91] = Instruction::new("SUB C", Cpu::sub_reg_8, None, reg_c);
        cpu.instructions[0x92] = Instruction::new("SUB D", Cpu::sub_reg_8, None, reg_d);
        cpu.instructions[0x93] = Instruction::new("SUB E", Cpu::sub_reg_8, None, reg_e);
        cpu.instructions[0x94] = Instruction::new("SUB H", Cpu::sub_reg_8, None, reg_h);
        cpu.instructions[0x95] = Instruction::new("SUB L", Cpu::sub_reg_8, None, reg_l);
        cpu.instructions[0x96] = Instruction::new("SUB (HL)", Cpu::sub_reg_8, None, addr_hl);
        cpu.instructions[0x97] = Instruction::new("SUB A", Cpu::sub_reg_8, None, reg_a);
        cpu.instructions[0x98] = Instruction::new("SBC A,B", Cpu::sbc, None, reg_b);
        cpu.instructions[0x99] = Instruction::new("SBC A,C", Cpu::sbc, None, reg_c);
        cpu.instructions[0x9A] = Instruction::new("SBC A,D", Cpu::sbc, None, reg_d);
        cpu.instructions[0x9B] = Instruction::new("SBC A,E", Cpu::sbc, None, reg_e);
        cpu.instructions[0x9C] = Instruction::new("SBC A,H", Cpu::sbc, None, reg_h);
        cpu.instructions[0x9D] = Instruction::new("SBC A,L", Cpu::sbc, None, reg_l);
        cpu.instructions[0x9E] = Instruction::new("SBC A,(HL)", Cpu::sbc, None, addr_hl);
        cpu.instructions[0x9F] = Instruction::new("SBC A,A", Cpu::sbc, None, reg_a);

        cpu.instructions[0xA0] = Instruction::new("AND B", Cpu::and, None, reg_b);
        cpu.instructions[0xA1] = Instruction::new("AND C", Cpu::and, None, reg_c);
        cpu.instructions[0xA2] = Instruction::new("AND D", Cpu::and, None, reg_d);
        cpu.instructions[0xA3] = Instruction::new("AND E", Cpu::and, None, reg_e);
        cpu.instructions[0xA4] = Instruction::new("AND H", Cpu::and, None, reg_h);
        cpu.instructions[0xA5] = Instruction::new("AND L", Cpu::and, None, reg_l);
        cpu.instructions[0xA6] = Instruction::new("AND (HL)", Cpu::and, None, addr_hl);
        cpu.instructions[0xA7] = Instruction::new("AND A", Cpu::and, None, reg_a);
        cpu.instructions[0xA8] = Instruction::new("XOR B", Cpu::xor, None, reg_b);
        cpu.instructions[0xA9] = Instruction::new("XOR C", Cpu::xor, None, reg_c);
        cpu.instructions[0xAA] = Instruction::new("XOR D", Cpu::xor, None, reg_d);
        cpu.instructions[0xAB] = Instruction::new("XOR E", Cpu::xor, None, reg_e);
        cpu.instructions[0xAC] = Instruction::new("XOR H", Cpu::xor, None, reg_h);
        cpu.instructions[0xAD] = Instruction::new("XOR L", Cpu::xor, None, reg_l);
        cpu.instructions[0xAE] = Instruction::new("XOR (HL)", Cpu::xor, None, addr_hl);
        cpu.instructions[0xAF] = Instruction::new("XOR A", Cpu::xor, None, reg_a);

        cpu.instructions[0xB0] = Instruction::new("OR B", Cpu::or, None, reg_b);
        cpu.instructions[0xB1] = Instruction::new("OR C", Cpu::or, None, reg_c);
        cpu.instructions[0xB2] = Instruction::new("OR D", Cpu::or, None, reg_d);
        cpu.instructions[0xB3] = Instruction::new("OR E", Cpu::or, None, reg_e);
        cpu.instructions[0xB4] = Instruction::new("OR H", Cpu::or, None, reg_h);
        cpu.instructions[0xB5] = Instruction::new("OR L", Cpu::or, None, reg_l);
        cpu.instructions[0xB6] = Instruction::new("OR (HL)", Cpu::or, None, addr_hl);
        cpu.instructions[0xB7] = Instruction::new("OR A", Cpu::or, None, reg_a);
        cpu.instructions[0xB8] = Instruction::new("CP B", Cpu::cp, None, reg_b);
        cpu.instructions[0xB9] = Instruction::new("CP C", Cpu::cp, None, reg_c);
        cpu.instructions[0xBA] = Instruction::new("CP D", Cpu::cp, None, reg_d);
        cpu.instructions[0xBB] = Instruction::new("CP E", Cpu::cp, None, reg_e);
        cpu.instructions[0xBC] = Instruction::new("CP H", Cpu::cp, None, reg_h);
        cpu.instructions[0xBD] = Instruction::new("CP L", Cpu::cp, None, reg_l);
        cpu.instructions[0xBE] = Instruction::new("CP (HL)", Cpu::cp, None, addr_hl);
        cpu.instructions[0xBF] = Instruction::new("CP A", Cpu::cp, None, reg_a);

        cpu.instructions[0xC0] = Instruction::new_cond_jump("RET NZ", Cpu::ret_cond, None, None, JumpCondition::NotZero);
        cpu.instructions[0xC1] = Instruction::new("POP BC", Cpu::pop_bc, None, None);
        cpu.instructions[0xC2] = Instruction::new_cond_jump("JP NZ,u16", Cpu::jp, None, imm16, JumpCondition::NotZero);
        cpu.instructions[0xC3] = Instruction::new("JP u16", Cpu::jp, None, imm16);
        cpu.instructions[0xC4] = Instruction::new_cond_jump("CALL NZ,u16", Cpu::call, None, imm16, JumpCondition::NotZero);
        cpu.instructions[0xC5] = Instruction::new("PUSH BC", Cpu::push_reg, None, reg_bc);
        cpu.instructions[0xC6] = Instruction::new("ADD A,u8", Cpu::add_reg_8, None, imm8);
        cpu.instructions[0xC7] = Instruction::new("RST 00H", |cpu| cpu.rst(0x00), None, None);
        cpu.instructions[0xC8] = Instruction::new_cond_jump("RET Z", Cpu::ret_cond, None, None, JumpCondition::Zero);
        cpu.instructions[0xC9] = Instruction::new("RET", Cpu::ret_cond, None, None);
        cpu.instructions[0xCA] = Instruction::new_cond_jump("JP Z,u16", Cpu::jp, None, imm16, JumpCondition::Zero);
        cpu.instructions[0xCB] = Instruction::new("CB", Cpu::cb, None, None);
        cpu.instructions[0xCC] = Instruction::new_cond_jump("CALL Z,u16", Cpu::call, None, imm16, JumpCondition::Zero);
        cpu.instructions[0xCD] = Instruction::new("CALL u16", Cpu::call, None, imm16);
        cpu.instructions[0xCE] = Instruction::new("ADC A,u8", Cpu::adc, None, imm8);
        cpu.instructions[0xCF] = Instruction::new("RST 08H", |cpu| cpu.rst(0x08), None, None);

        cpu.instructions[0xD0] = Instruction::new_cond_jump("RET NC", Cpu::ret_cond, None, None, JumpCondition::NotCarry);
        cpu.instructions[0xD1] = Instruction::new("POP DE", Cpu::pop_de, None, None);
        cpu.instructions[0xD2] = Instruction::new_cond_jump("JP NC,u16", Cpu::jp, None, imm16, JumpCondition::NotCarry);
        cpu.instructions[0xD3] = Instruction::new("UNSUPPORTED", Cpu::not_supported, None, None);
        cpu.instructions[0xD4] = Instruction::new_cond_jump( "CALL NC,u16", Cpu::call, None, imm16, JumpCondition::NotCarry);
        cpu.instructions[0xD5] = Instruction::new( "PUSH DE", Cpu::push_reg, None, reg_de);
        cpu.instructions[0xD6] = Instruction::new( "SUB A, u8", Cpu::sub_reg_8, None, imm8);
        cpu.instructions[0xD7] = Instruction::new( "RST 10H", |cpu| cpu.rst(0x10), None, None);
        cpu.instructions[0xD8] = Instruction::new_cond_jump( "RET C", Cpu::ret_cond, None, None, JumpCondition::Carry);
        cpu.instructions[0xD9] = Instruction::new("RETI", Cpu::reti, None, None);
        cpu.instructions[0xDA] = Instruction::new_cond_jump("JP C,u16", Cpu::jp, None, imm16, JumpCondition::Carry);
        cpu.instructions[0xDB] = Instruction::new("UNSUPPORTED", Cpu::not_supported, None, None);
        cpu.instructions[0xDC] = Instruction::new_cond_jump("CALL C,u16", Cpu::call, None, imm16, JumpCondition::Carry);
        cpu.instructions[0xDE] = Instruction::new("SBC A,u8", Cpu::sbc, None, imm8);
        cpu.instructions[0xDF] = Instruction::new("RST 18H", |cpu| cpu.rst(0x18), None, None);

        cpu.instructions[0xE0] = Instruction::new("LD (FF00+u8),A", Cpu::ld_ff00_u8_a, None, None);
        cpu.instructions[0xE1] = Instruction::new("POP HL", Cpu::pop_hl, None, None);
        cpu.instructions[0xE2] = Instruction::new("LD (FF00+C),A", Cpu::ld_ff00_c_a, None, None);
        cpu.instructions[0xE3] = Instruction::new("UNSUPPORTED", Cpu::not_supported, None, None);
        cpu.instructions[0xE4] = Instruction::new("UNSUPPORTED", Cpu::not_supported, None, None);
        cpu.instructions[0xE5] = Instruction::new("PUSH HL", Cpu::push_reg, None, reg_hl);
        cpu.instructions[0xE6] = Instruction::new("AND A,u8", Cpu::and_, None, imm8);
        cpu.instructions[0xE7] = Instruction::new("RST 20H", |cpu| cpu.rst(0x20), None, None);
        cpu.instructions[0xE8] = Instruction::new("ADD SP,i8", Cpu::add_to_sp_signed, None, None);
        cpu.instructions[0xE9] = Instruction::new("JP (HL)", Cpu::jp_uncond, None, reg_hl);
        cpu.instructions[0xEA] = Instruction::new("LD (u16),A", Cpu::ld, addr_ind, reg_a);
        cpu.instructions[0xEB] = Instruction::new("UNSUPPORTED", Cpu::not_supported, None, None);
        cpu.instructions[0xEC] = Instruction::new("UNSUPPORTED", Cpu::not_supported, None, None);
        cpu.instructions[0xED] = Instruction::new("UNSUPPORTED", Cpu::not_supported, None, None);
        cpu.instructions[0xEE] = Instruction::new("XOR u8", Cpu::xor, None, imm8);
        cpu.instructions[0xEF] = Instruction::new("RST 28H", |cpu| cpu.rst(0x28), None, None);

        cpu.instructions[0xF0] = Instruction::new("LD A,(FF00+u8)", Cpu::ld_a_ff00_u8, None, None);
        cpu.instructions[0xF1] = Instruction::new("POP AF", Cpu::pop_af, None, None);
        cpu.instructions[0xF2] = Instruction::new("LD A,(FF00+C)", Cpu::ld_a_ff00_c, None, None);
        cpu.instructions[0xF3] = Instruction::new("DI", Cpu::di, None, None);
        cpu.instructions[0xF4] = Instruction::new("UNSUPPORTED", Cpu::not_supported, None, None);
        cpu.instructions[0xF5] = Instruction::new("PUSH AF", Cpu::push_reg, None, reg_af);
        cpu.instructions[0xF6] = Instruction::new("OR u8", Cpu::or, None, imm8);
        cpu.instructions[0xF7] = Instruction::new("RST 30H", |cpu| cpu.rst(0x30), None, None);
        cpu.instructions[0xF8] = Instruction::new("LD HL,SP+i8", Cpu::ld_hl_sp_i8, None, None);
        cpu.instructions[0xF9] = Instruction::new("LD SP,HL", Cpu::ld_16, reg_sp, reg_hl);
        cpu.instructions[0xFA] = Instruction::new("LD A,(u16)", Cpu::ld, reg_a, addr_ind);
        cpu.instructions[0xFB] = Instruction::new("EI", Cpu::ei, None, None);
        cpu.instructions[0xFC] = Instruction::new("UNSUPPORTED", Cpu::not_supported, None, None);
        cpu.instructions[0xFD] = Instruction::new("UNSUPPORTED", Cpu::not_supported, None, None);
        cpu.instructions[0xFE] = Instruction::new("CP u8", Cpu::cp, None, imm8);
        cpu.instructions[0xFF] = Instruction::new("RST 38H", |cpu| cpu.rst(0x38), None, None);
        cpu.restart();

        cpu
    }

    fn ld_16(&mut self)
    {
        self.tick();
    }

    pub fn cycles(&self) -> u32 {
        self.cycles
    }

    fn calc_sp_i8(&mut self) -> u16 {
        let value: i8 = self.fetch() as i8;
        let res = ((self.sp as i32) + (value as i32)) as u16;

        self.set_flag_sub(false);
        self.set_flag_zero(false);

        self.set_flag_half_carry(((self.sp ^ (value as u16) ^ res) & 0x10) == 0x10);
        self.set_flag_carry(((self.sp ^ (value as u16) ^ res) & 0x100) == 0x100);

        res
    }

    fn ld_hl_sp_i8(&mut self) {
        let sp = self.calc_sp_i8();
        self.set_hl(sp);
        self.tick();
    }

    fn add_to_sp_signed(&mut self) {
        self.sp = self.calc_sp_i8();
        self.tick();
        self.tick();
    }

    fn add_reg_16(&mut self) {
        let left = self.get_hl();
        let v = left.wrapping_add(self.value);
        self.tick();

        self.set_flag_sub(false);
        self.update_flag_half_carry_16(left, self.value, false);
        self.set_flag_carry(v < left);

        self.set_hl(v);
    }

    fn inc_reg(&mut self) {
        self.value = self.inc(self.value as u8) as u16;
    }

    fn inc_reg_16(&mut self) {
        self.value = self.value.wrapping_add(1);
        self.tick();
    }

    fn dec_reg_16(&mut self) {
        self.value = self.value.wrapping_sub(1);
        self.tick();
    }

    fn ld_ff00_c_a(&mut self) {
        // LD (FF00+C),A", Cpu::ld, ld_ff00_c_a)
        let addr = 0xFF00u16 + self.c as u16;
        self.write(addr, self.a);
    }

    fn ld_a_ff00_u8(&mut self){
        let off = self.fetch() as u16;
        self.a = self.read(0xFF00 + off);
    }

    fn ld_a_ff00_c(&mut self) {
        let addr = 0xFF00u16 + self.c as u16;
        self.a = self.read(addr);
    }

    fn ld_hl_plus_a(&mut self) {
        self.write(self.get_hl(), self.a);
        self.set_hl(self.get_hl().wrapping_add(1));
    }

    fn ld_a_hl_plus(&mut self) {
        self.a = self.read(self.get_hl());
        self.set_hl(self.get_hl().wrapping_add(1));
    }

    fn ld_hl_minus_a(&mut self) {
        self.write(self.get_hl(), self.a);
        self.set_hl(self.get_hl().wrapping_sub(1));
    }

    fn ld_a_hl_minus(&mut self) {
        self.a = self.read(self.get_hl());
        self.set_hl(self.get_hl().wrapping_sub(1));
    }

    /// Reads value from the bus, increasing the cycle accordingly!
    fn read(&mut self, pos: u16) -> u8 {
        let v = self.bus.read(pos);
        self.tick();

        v
    }

    fn write(&mut self, addr: u16, v: u8) {
        self.bus.write(addr, v);
        self.tick();
    }

    fn write_16(&mut self, addr: u16, v: u16) {
        self.write(addr, (v & 0xFF) as u8);
        self.write(addr.wrapping_add(1), (v >> 8) as u8);
    }

    /// Restart the CPU to the state after the official BOOT rom.
    pub fn restart(self: &mut Self) {
        self.ime = true;
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

    fn rst(&mut self, offs: u8) {
        self.push_pc();
        self.pc = offs as u16;
        self.tick();
    }

    fn stop(&mut self) {
        todo!("STOP");

    }

    fn not_supported(&mut self) {
        unimplemented!("Not supported op: {:#02X}", self.opcode);
    }

    fn fetch_source(&mut self, src: Option<Op>) {
        if (src.is_none()) {
            return;
        }

        let value: u16 = match src.unwrap() {
            Op::Imm8 => self.fetch() as u16,
            Op::Imm16 => self.fetch16(),
            Op::IndirectAddress => {
                let addr = self.fetch16();
                self.read(addr) as u16
            }
            Op::AddressHL => self.read(self.get_hl()) as u16,
            Op::AddressBC => self.read(self.get_bc()) as u16,
            Op::AddressDE => self.read(self.get_de()) as u16,
            Op::Reg(r) => match r {
                Reg::A => self.a as u16,
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
                Reg::SP => self.sp,
            },
        };

        self.value = value;
    }

    fn write_dest(&mut self, dst: Option<Op>) {
        if dst.is_none() {
            return;
        }

        match dst.unwrap() {
            Op::Reg(r) => match r {
                Reg::A => self.a = self.value as u8,
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
            },
            Op::IndirectAddress => {
                assert!(self.value < 256);
                let addr = self.fetch16();
                self.write(addr, self.value as u8);
            }
            Op::AddressHL => {
                self.write(self.get_hl(), self.value as u8);
            }
            Op::AddressBC => {
                self.write(self.get_bc(), self.value as u8);
            }
            Op::AddressDE => {
                self.write(self.get_de(), self.value as u8);
            }
            _ => unimplemented!(),
        }
    }

    fn execute_interrupt(&mut self, addr: u16, interrupt_type: InterruptType) {
        self.tick();
        self.tick();

        self.push_pc();

        self.pc = addr;
        self.tick();

        self.ime = false;
        self.bus.int_flags.clear(interrupt_type);
    }

    fn handle_interrupts(&mut self) {
        if !self.ime || self.bus.int_flags.state() == 0 {
            return;
        }

        if self.bus.should_run_interrupt(InterruptType::VBLANK) {
            self.execute_interrupt(0x40, InterruptType::VBLANK);
        } else if self.bus.should_run_interrupt(InterruptType::LCD_STAT) {
            self.execute_interrupt(0x48, InterruptType::LCD_STAT);
        } else if self.bus.should_run_interrupt(InterruptType::TIMER) {
            self.execute_interrupt(0x50, InterruptType::TIMER);
        } else if self.bus.should_run_interrupt(InterruptType::SERIAL) {
            self.execute_interrupt(0x58, InterruptType::SERIAL);
        } else if self.bus.should_run_interrupt(InterruptType::JOYPAD) {
            self.execute_interrupt(0x60, InterruptType::JOYPAD);
        }
    }

    pub fn step(&mut self) {
        if !self.halted {
            self.opcode = self.fetch();
            let &ins = &self.instructions[self.opcode as usize];
            self.fetch_source(ins.src);
            (ins.call)(self);
            self.write_dest(ins.dest);
        }
        else {
            let old_int_flags_state = self.bus.int_flags.state();
            self.tick();
            self.halted = old_int_flags_state == self.bus.int_flags.state();
        }

        self.handle_interrupts();
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

    fn zero_flag(&self) -> u8 {
        return bit_test(self.f, Self::ZERO_BIT) as u8;
    }

    fn carry_flag(&self) -> u8 {
        return bit_test(self.f, Self::CARRY_BIT) as u8;
    }

    fn flag_carry(&self) -> bool {
        bit_test(self.f, Self::CARRY_BIT)
    }

    fn flag_half_carry(&self) -> bool {
        bit_test(self.f, Self::HALF_CARRY_BIT)
    }

    fn flag_sub(&self) -> bool {
        bit_test(self.f, Self::SUBSTRACTION_BIT)
    }

    fn set_af(&mut self, v: u16) {
        // clear the lower 4 bits of the F registers
        let v = v & 0xFFF0;
        Cpu::set_reg_pair(&mut self.a, &mut self.f, v);
    }

    fn set_bc(&mut self, v: u16) {
        Cpu::set_reg_pair(&mut self.b, &mut self.c, v);
    }

    fn set_hl(&mut self, v: u16) {
        Cpu::set_reg_pair(&mut self.h, &mut self.l, v);
    }

    fn set_de(&mut self, v: u16) {
        Cpu::set_reg_pair(&mut self.d, &mut self.e, v);
    }

    fn pop_af(&mut self) {
        let v = self.pop16();
        self.set_af(v);
    }

    fn pop_bc(&mut self) {
        let v = self.pop16();
        self.set_bc(v);
    }

    fn pop_de(&mut self) {
        let v = self.pop16();
        self.set_de(v);
    }

    fn pop_hl(&mut self) {
        let v = self.pop16();
        self.set_hl(v);
    }

    fn push_reg(&mut self) {
        self.push16(self.value);
        // extra tick for reg value transfer
        self.tick();
    }

    fn push(&mut self, v: u8) {
        self.sp = self.sp.wrapping_sub(1);
        self.write(self.sp, v);
    }

    fn pop(&mut self) -> u8 {
        let v = self.read(self.sp);
        self.sp = self.sp.wrapping_add(1);
        v
    }

    /// Increases T-Cycles by 4 and drives the "circuit"
    fn tick(&mut self) {
        self.bus.tick();
        self.cycles += self.cycles.wrapping_add(4);
    }

    fn fetch(&mut self) -> u8 {
        let addr = self.pc;
        self.pc = self.pc.wrapping_add(1);
        self.read(addr)
    }

    fn fetch16(&mut self) -> u16 {
        let lower = self.fetch() as u16;
        let upper = self.fetch() as u16;
        return upper << 8 | lower;
    }

    fn pop_pc(&mut self) {
        self.pc = self.pop16();
        // internal .pc set?
        self.tick();
    }

    fn push16(&mut self, v: u16) {
        self.push((v >> 8) as u8);
        self.push((v & 0xFF) as u8);
    }

    fn pop16(&mut self) -> u16 {
        let low = self.pop() as u16;
        let high = self.pop() as u16;
        high << 8 | low
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

    fn update_flag_zero(&mut self, v: u8) {
        self.set_flag_zero(v == 0);
    }

    fn set_flag_zero(&mut self, v: bool) {
    if v {
            self.f = bit_set(self.f, Self::ZERO_BIT)
        } else {
            self.f = bit_clear(self.f, Self::ZERO_BIT)
        }
    }

    fn rel_pc(&self, rel: i8) -> u16 {
        (self.pc as i32 + rel as i32) as u16
    }

    fn dump_registers(&self) {
        error!(
            "PC: {:#06x}, AF: {:#06x}, BC: {:#06x}, DE: {:#06x}, HL: {:#06x}, SP: {:#06x}\n",
            self.pc,
            self.get_af(),
            self.get_bc(),
            self.get_de(),
            self.get_hl(),
            self.sp
        );
    }

    pub fn instr(&self) -> &Instruction {
        &self.instructions[self.opcode as usize]
    }

    fn cb_bit(&mut self, bit: u8) {
        let v = self.value as u8;
        self.set_flag_zero(!bit_test(v, bit));
        self.set_flag_sub(false);
        self.set_flag_half_carry(true);
    }

    fn cb_res(&mut self, bit: u8) {
        self.value = bit_clear(self.value as u8, bit) as u16;
    }

    fn cb_set(&mut self, bit: u8) {
        self.value = bit_set(self.value as u8, bit) as u16;
    }

    fn cb_rlc(&mut self) {
        self.value = self.do_rlc(self.value as u8) as u16;
    }

    fn cb_rrc(&mut self) {
        self.value = self.do_rrc(self.value as u8) as u16;
    }

    fn cb_rl(&mut self) {
        self.value = self.do_rl(self.value as u8) as u16;
    }

    fn do_rl(&mut self, v: u8) -> u8 {
        // Rotate the contents of register A to the left.
        // That is, the contents of bit 7 are copied to bit 0, and the previous contents of bit 7 (before the copy) are copied to the CY flag.
        // The same operation is repeated in sequence for the rest of the register.
        // The contents of bit 0 are placed in both the CY flag and bit 7 of register A.

        let carry = bit_test(v, 7) as u8;

        let result = (v << 1) | self.flag_carry() as u8;
        self.set_flag_carry(carry != 0);
        self.update_flag_zero(result);
        self.set_flag_sub(false);
        self.set_flag_half_carry(false);

        result
    }

    fn cb_rr(&mut self) {
        self.value = self.do_rr(self.value as u8) as u16;
    }

    fn cb_sla(&mut self) {
        // Shift the contents of register A to the left.
        // That is, the contents of bit 7 are copied to the CY flag, and 0 is placed in bit 0.
        // The same operation is repeated in sequence for the rest of the register.

        let carry = (self.value as u8 >> 7) & 1;
        let result = (self.value as u8) << 1;
        self.set_flag_carry(carry == 1);
        self.update_flag_zero(result);
        self.set_flag_sub(false);
        self.set_flag_half_carry(false);

        self.value = result as u16;
    }

    fn cb_sra(&mut self) {
        // Shift the contents of register A to the right.
        // That is, the contents of bit 0 are copied to the CY flag, and the previous contents of bit 7 (before the copy) are copied to bit 7.
        // The same operation is repeated in sequence for the rest of the register.

        let carry = self.value & 1;
        let result = ((self.value as u8) >> 1) | ((self.value as u8) & 0x80);
        self.set_flag_carry(carry == 1);
        self.update_flag_zero(result);
        self.set_flag_sub(false);
        self.set_flag_half_carry(false);

        self.value = result as u16;
    }
    fn cb_swap(&mut self) {
        // Shift the contents of the lower-order four bits (0-3) of register B
        // to the higher-order four bits (4-7) of the register,
        // and shift the higher-order four bits to the lower-order four bits.
        let reg = self.value as u8;
        let res = ((reg & 0xF) << 4) | (reg >> 4);

        self.update_flag_zero(res);
        self.set_flag_sub(false);
        self.set_flag_half_carry(false);
        self.set_flag_carry(false);

        self.value = res as u16;
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
        self.update_flag_zero(res);
        self.set_flag_sub(false);
        self.set_flag_half_carry(false);

        self.value = res as u16;
    }

    fn cb(&mut self) {
        // must not raise an interrupt while fetching the next CB instruction,
        self.fetching_cb = true;
        self.cb_opcode = self.fetch();
        self.fetching_cb = false;

        let x = (self.cb_opcode >> 6) & 0x3;
        let y = (self.cb_opcode >> 3) & 0x7;
        let z = self.cb_opcode & 0x7;

        let operand = match z {
            0 => Op::Reg(Reg::B),
            1 => Op::Reg(Reg::C),
            2 => Op::Reg(Reg::D),
            3 => Op::Reg(Reg::E),
            4 => Op::Reg(Reg::H),
            5 => Op::Reg(Reg::L),
            6 => Op::AddressHL,
            7 => Op::Reg(Reg::A),
            _ => unreachable!(),
        };

        self.fetch_source(Some(operand));

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

        // cb_bit() does not modify anything
        if x != 1
        {
            self.write_dest(Some(operand));
        }
    }

    fn nop(&mut self) {}

    fn and_(&mut self){
        // in other emu:
        // 1381856
        // 1391528
        // distance: 9672 = 1391528 - 1381856
        self.and();
    }
    fn and(&mut self) {
        self.a &= self.value as u8;
        self.update_flag_zero(self.a);
        self.set_flag_sub(false);
        self.set_flag_half_carry(true);
        self.set_flag_carry(false);
    }

    fn or(&mut self) {
        self.a |= self.value as u8;
        self.update_flag_zero(self.a);
        self.set_flag_sub(false);
        self.set_flag_half_carry(false);
        self.set_flag_carry(false);
    }

    fn xor(&mut self) {
        self.a ^= self.value as u8;
        self.update_flag_zero(self.a);
        self.set_flag_sub(false);
        self.set_flag_half_carry(false);
        self.set_flag_carry(false);
    }

    fn cp(&mut self) {
        let v = self.value as u8;
        let diff = self.a.wrapping_sub(v);
        self.update_flag_zero(diff);
        self.set_flag_sub(true);
        self.update_flag_half_carry(self.a, v, true);
        self.set_flag_carry(self.a < v);
    }

    fn do_rlc(&mut self, v: u8) -> u8 {
        // Rotate the contents of register A to the left.
        // That is, the contents of bit 0 are copied to bit 7, and the previous contents of bit 7 (before the copy) are copied to bit 6.
        // The same operation is repeated in sequence for the rest of the register.
        // The contents of bit 7 are placed in both the CY flag and bit 0 of register A.

        let left_bit = bit_test(v, 7) as u8;
        let result = (v << 1) | left_bit;
        self.set_flag_carry(left_bit == 1);
        self.update_flag_zero(result);
        self.set_flag_sub(false);
        self.set_flag_half_carry(false);

        result
    }

    fn do_rrc(&mut self, v: u8) -> u8 {
        // Rotate the contents of register A to the right.
        // That is, the contents of bit 7 are copied to bit 6, and the previous contents of bit 6 (before the copy) are copied to bit 5.
        // The same operation is repeated in sequence for the rest of the register. The contents of bit 0 are placed in both the CY flag and bit 7 of register A.
        let carry = v & 1;
        let result = (carry << 7) | (v >> 1);
        self.set_flag_carry(carry == 1);
        self.update_flag_zero(result);
        self.set_flag_sub(false);
        self.set_flag_half_carry(false);

        result
    }

    fn do_rr(&mut self, v: u8) -> u8 {
        //Rotate the contents of register A to the right.
        //through the carry (CY) flag.
        //That is, the contents of bit 7 are copied to bit 6, and the previous contents of bit 6 (before the copy) are copied to bit 5.
        //The same operation is repeated in sequence for the rest of the register.
        // The previous contents of the carry flag are copied to bit 7.

        let left_over = self.flag_carry() as u8;
        let will_have_carry = v & 1;
        let res = (left_over << 7) | (v >> 1);

        self.set_flag_carry(will_have_carry == 1);
        self.update_flag_zero(res);
        self.set_flag_sub(false);
        self.set_flag_half_carry(false);

        return res;
    }

    fn rlca(&mut self) {
        self.a = self.do_rlc(self.a);
        self.set_flag_zero(false);
    }

    fn rla(&mut self) {
        self.a = self.do_rl(self.a);
        self.set_flag_zero(false);
    }

    fn rra(&mut self) {
        self.a = self.do_rr(self.a);
        self.set_flag_zero(false);
    }

    fn rrca(&mut self) {
        self.a = self.do_rrc(self.a);
        self.set_flag_zero(false);
    }

    fn inc(&mut self, v: u8) -> u8 {
        let res = v.wrapping_add(1);
        self.update_flag_zero(res);
        self.set_flag_sub(false);
        self.update_flag_half_carry(v, 1, false);
        return res;
    }

    fn dec_reg(&mut self) {
        self.value = self.dec(self.value as u8) as u16;
    }

    fn adc(&mut self) {
        let carry = self.flag_carry() as u8;
        let full = self.a as u32 + self.value as u32 + carry as u32;
        let v = full as u8;
        self.update_flag_zero(v);
        self.set_flag_sub(false);
        let cond = ((self.a & 0xf) + (self.value as u8 & 0xf) + carry) > 0xF;
        self.set_flag_half_carry(cond);
        self.set_flag_carry(full > 0xFF);
        self.a = v;
    }

    fn sbc(&mut self) {
        let carry = self.flag_carry() as u8;
        let full = self.a as i32 - self.value as i32 - carry as i32;
        let v = full as u8;

        self.update_flag_zero(v);
        self.set_flag_sub(true);
        self.set_flag_half_carry((self.a & 0xf) < (self.value as u8 & 0xf) + carry);
        self.set_flag_carry(full < 0);
        self.a = v;
    }

    fn add_reg_8(&mut self) {
        let v = self.a.wrapping_add(self.value as u8);
        self.update_flag_zero(v);
        self.set_flag_sub(false);
        self.update_flag_half_carry(self.a as u8, self.value as u8, false);
        self.set_flag_carry(v < self.a);
        self.a = v;
    }

    fn sub_reg_8(&mut self) {
        let v = self.a.wrapping_sub(self.value as u8);
        self.update_flag_zero(v);
        self.set_flag_sub(true);
        self.update_flag_half_carry(self.a as u8, self.value as u8, true);
        self.set_flag_carry(v > self.a);
        self.a = v;
    }

    fn dec(&mut self, v: u8) -> u8 {
        let res = v.wrapping_sub(1);
        self.update_flag_zero(res);
        self.set_flag_sub(true);
        self.update_flag_half_carry(v, 1, true);
        return res;
    }

    fn get_jump_condition(&self, condition: JumpCondition) -> u8 {
        return match condition {
            JumpCondition::Zero => self.zero_flag(),
            JumpCondition::NotZero => not(self.zero_flag()),
            JumpCondition::Carry => self.carry_flag(),
            JumpCondition::NotCarry => not(self.carry_flag()),
        };
    }

    fn reti(&mut self) {
        self.pop_pc();
        self.ime = true;
    }

    fn ei(&mut self) {
        self.ime = true;
    }

    fn di(&mut self) {
        self.ime = false;
    }

    fn jp(&mut self) {
        if let Some(jmp_condition) = self.instr().jump_condition {
            if self.get_jump_condition(jmp_condition) == 0 {
                return;
            }
        }

        self.tick();
        self.pc = self.value;
    }

    fn jp_uncond(&mut self)
    {
        self.pc = self.value;
    }

    fn ret_cond(&mut self) {
        if let Some(jump_condition) = self.instr().jump_condition {
            // internal branch decision
            self.tick();

            if self.get_jump_condition(jump_condition) == 0 {
                return;
            }
        }

        self.pop_pc();
    }

    fn daa(&mut self) {
        let mut v = self.a;

        let mut correction: u16 = if self.flag_carry() { 0x60 } else { 0x00 };

        if (self.flag_half_carry() || (!self.flag_sub() && ((v & 0x0F) > 9))) {
            correction |= 0x06;
        }

        if (self.flag_carry() || (!self.flag_sub() && (v > 0x99))) {
            correction |= 0x60;
        }

        if (self.flag_sub()) {
            v = (v as u16).wrapping_sub(correction) as u8;
        } else {
            v = (v as u16).wrapping_add(correction) as u8;
        }

        if (((correction << 2) & 0x100) != 0) {
            self.set_flag_carry(true);
        }

        self.set_flag_half_carry(false);
        self.update_flag_zero(v);

        self.a = v;
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
        self.pc = self.value;
        self.tick();
    }

    fn halt(&mut self) {
        self.halted = true;
    }

    fn ld(&mut self) {}

    fn ld_ff00_u8_a(&mut self){
        let off = self.fetch() as u16;
        self.write(0xFF00 + off, self.a);
    }

    fn ld_sp(&mut self) {
        let addr = self.fetch16();
        self.write_16(addr, self.sp);
    }

    fn cpl(&mut self) {
        self.a = !self.a;
        self.set_flag_sub(true);
        self.set_flag_half_carry(true);
    }

    fn scf(&mut self) {
        self.set_flag_sub(false);
        self.set_flag_half_carry(false);
        self.set_flag_carry(true);
    }

    fn ccf(&mut self) {
        self.set_flag_sub(false);
        self.set_flag_half_carry(false);
        self.set_flag_carry(!self.flag_carry());
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

        cpu.restart();
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

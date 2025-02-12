use std::{fs::File, io::Read};
mod cpu;
use cpu::*;

fn main() {
    let mut rom = Vec::<u8>::new();
    File::open("cpu_instrs.gb").unwrap().read_to_end(&mut rom).expect("failed to open file");

    let mut cpu = Cpu::new();
    cpu.bus.load_rom(rom);

    for _ in 0..25  {
        cpu.step();
    }
}

mod engine;

use std::{fs::File, io::Read};

use log::Log;

fn main() {
    
    colog::basic_builder().filter_level(log::LevelFilter::Trace).init();

    let mut rom = Vec::<u8>::new();
    File::open("cpu_instrs.gb").unwrap().read_to_end(&mut rom).expect("failed to open file");

    let mut cpu = engine::cpu::Cpu::new();
    cpu.bus.load_rom(rom);

    for _ in 0..10000  {
        cpu.step();
    }
}

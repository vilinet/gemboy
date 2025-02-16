mod engine;

use std::{fs::File, io::Read};

use log::Log;

fn main() {

    std::fs::remove_file("cpu.log").unwrap_or_default();
    colog::basic_builder().filter_level(log::LevelFilter::Trace).init();

    let mut rom = Vec::<u8>::new();
    File::open("01-special.gb").unwrap().read_to_end(&mut rom).expect("failed to open file");
    
    let mut cpu = engine::cpu::Cpu::new("cpu.log");
    
    cpu.bus.load_rom(rom);

    cpu.write_state();
    for _ in 0..1000  {
        cpu.step();
        cpu.write_state();
    }
}

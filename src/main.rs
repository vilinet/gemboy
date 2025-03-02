mod engine;

use std::{fs::File, io::Read, io::Write};
static mut LINE: i32 = 1;
pub fn write_state(file: &mut File, cpu: &mut engine::cpu::Cpu)
{
    // A:01 F:B0 B:00 C:13 D:00 E:D8 H:01 L:4D SP:FFFE PC:0100 PCMEM:00,C3,13,02
    let pc1 = cpu.bus.read(cpu.pc);
    let pc2 = cpu.bus.read(cpu.pc + 1);
    let pc3 = cpu.bus.read(cpu.pc + 2);
    let pc4 = cpu.bus.read(cpu.pc + 3);

    let line = format!("A:{:02X} F:{:02X} B:{:02X} C:{:02X} D:{:02X} E:{:02X} H:{:02X} L:{:02X} SP:{:04X} PC:{:04X} PCMEM:{:02X},{:02X},{:02X},{:02X}\n", cpu.a, cpu.f, cpu.b, cpu.c, cpu.d, cpu.e, cpu.h, cpu.l, cpu.sp, cpu.pc, pc1, pc2, pc3, pc4);
    file.write(line.as_bytes()).unwrap();
}

fn main() {

    colog::basic_builder().filter_level(log::LevelFilter::Error).init();

    let mut rom = Vec::<u8>::new();
    File::open("roms/02-interrupts.gb").unwrap().read_to_end(&mut rom).expect("failed to open file");
    
    let mut cpu = engine::cpu::Cpu::new();
    
    cpu.bus.load_rom(rom);

    std::fs::remove_file("cpu.log").unwrap_or_default();
    let mut log = File::create("cpu.log").unwrap();
    write_state(&mut log ,&mut cpu);

    for _ in 0..100000000  {
        cpu.step();
        write_state(&mut log, &mut cpu);
    }
}

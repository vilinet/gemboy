use std::collections::HashMap;
use std::fs::File;
use std::io::Read;
use serde_json::from_str;


#[derive(serde::Deserialize)]
struct Entry {
    cycles: Vec<u8>,
}

struct Result {
    opcode: String,
    cycles: u8,
}

impl Result{
    pub fn new(opcode: String, cycles: u8) ->Self{
        Result{ opcode, cycles }
    }
}

pub fn opcode_cycle_validator()
{
    let mut json_content: String = String::new();

    File::open("opcodes.json").unwrap().read_to_string(&mut json_content).expect("Failed to read file");
    let expected: HashMap<String, HashMap<String, Entry>> = serde_json::from_str(json_content.as_str()).expect("Failed to parse JSON");

    json_content = String::new();
    File::open("cpu.log").unwrap().read_to_string(&mut json_content).expect("Failed to read file");
    for line in json_content.lines() {
        let parts = line.split(";").collect::<Vec<&str>>();
        let actual_cycles = from_str::<u8>(&parts[2]).expect("Invalid u8 cycle value");
        let entry = if parts[0] == "0xcb" {
            expected.get("cbprefixed").expect("Not found opcode").get(parts[1])
        }
        else {
            expected.get("unprefixed").expect("Not found opcode").get(parts[0])
        }
        ;

        if entry.is_none(){
            println!("CB opcode not found: {}", parts[1]);
        }

        let entry = entry.unwrap();

        let okay = entry.cycles.iter().any(| &x|  return x == actual_cycles);
        if !okay{
            println!("INVALID: {} {}, {:?} <-> {}", parts[3], parts[1], entry.cycles, actual_cycles);
        }
    }
}
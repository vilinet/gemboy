pub const fn bit_set(v: u8, bit: u8) -> u8 {
    v | (1 << bit)
}

pub const fn bit_clear(v: u8, bit: u8) -> u8 {
    v & !(1 << bit)
}

pub const fn bit_test(v: u8, bit: u8) -> bool {
    (v & (1 << bit)) != 0
}

/// Logical NOT operation on a u8 value, not like the ! operator.
pub const fn not(v: u8) -> u8 {
    match v {
        0 => 1,
        _ => 0,
    }
}

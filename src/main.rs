use std;

enum _Registers {
    R0 = 0,
    R1,
    R2,
    R3,
    R4,
    R5,
    R6,
    R7,
    PC,     /* program counter */
    COND,
    COUNT,
}

enum _ConditionFlags {
    POS = 1 << 0,   /* P */
    ZRO = 1 << 1,   /* Z */
    NEG = 1 << 2,   /* N */
}

enum _Opcodes {
    BR = 0, /* branch */
    ADD,    /* add  */
    LD,     /* load */
    ST,     /* store */
    JSR,    /* jump register */
    AND,    /* bitwise and */
    LDR,    /* load register */
    STR,    /* store register */
    RTI,    /* unused */
    NOT,    /* bitwise not */
    LDI,    /* load indirect */
    STI,    /* store indirect */
    JMP,    /* jump */
    RES,    /* reserved (unused) */
    LEA,    /* load effective address */
    TRAP,    /* execute trap */
}

fn run() {
    // Define and initialize the memory
    let mut _memory: [u16; std::u16::MAX as usize]  = [0; std::u16::MAX as usize];
    // println!("{:?}", &_memory[..]);

    // Define and initialize registers
    let mut _regs: [u16; _Registers::COUNT as usize] = [0; _Registers::COUNT as usize];
    // println!("{:?}", &_regs[..]);
}

fn main() {
    run();
}
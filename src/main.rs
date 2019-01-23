use std;

enum Registers {
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

enum ConditionFlags {
    POS = 1 << 0,   /* P */
    ZRO = 1 << 1,   /* Z */
    NEG = 1 << 2,   /* N */
}

/* These variables are of type 'Opcodes', which
 * will require a cast every single goddamn time.
 * Like that isn't enough, match won't let me cast
 * them easily and I have to do it in an extra if
 * clause. *sigh*
 */
enum Opcodes {
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

fn sign_extend(x: u16, bit_count: u16) -> u16{
    if (x >> (bit_count - 1)) & 1 == 1 {
        return x | 0xffff << bit_count;
    }
    x
}

fn update_flags(regs: &mut [u16], r: usize) {
    regs[Registers::COND as usize] =  match regs[r] {
        0                   => ConditionFlags::ZRO as u16,
        r if r >> 15 == 1   => ConditionFlags::NEG as u16,
        _                   => ConditionFlags::POS as u16,
    };
}


fn mem_read(addr: u16) -> u16 {
    0xff00
}

fn run() {
    // Define and initialize the memory
    let mut memory: [u16; std::u16::MAX as usize]  = [0; std::u16::MAX as usize];
    // println!("{:?}", &memory[..]);

    // Define and initialize registers
    let mut regs: [u16; Registers::COUNT as usize] = [0; Registers::COUNT as usize];
    // println!("{:?}", &_regs[..]);

    // Set the PC
    let pc_start: u16 = 0x3000;
    regs[Registers::PC as usize] = pc_start;

    let mut running: bool = true;
    while running {
        let instruction: u16 = mem_read(regs[Registers::PC as usize]);
        regs[Registers::PC as usize] += 1;
        let op: u16 = instruction >> 12;

        match op {
            _ if op == Opcodes::BR  as u16  => println!("[*] BR"),
            _ if op == Opcodes::ADD as u16  => println!("[*] ADD"),
            _ if op == Opcodes::LD  as u16  => println!("[*] LD"),
            _ if op == Opcodes::ST  as u16  => println!("[*] ST"),
            _ if op == Opcodes::JSR as u16  => println!("[*] JSR"),
            _ if op == Opcodes::AND as u16  => println!("[*] AND"),
            _ if op == Opcodes::LDR as u16  => println!("[*] LDR"),
            _ if op == Opcodes::STR as u16  => println!("[*] STR"),
            _ if op == Opcodes::RTI as u16  => println!("[*] RTI"),
            _ if op == Opcodes::NOT as u16  => println!("[*] NOT"),
            _ if op == Opcodes::LDI as u16  => println!("[*] LDI"),
            _ if op == Opcodes::STI as u16  => println!("[*] STI"),
            _ if op == Opcodes::JMP as u16  => println!("[*] JMP"),
            _ if op == Opcodes::RES as u16  => println!("[*] RES"),
            _ if op == Opcodes::LEA as u16  => println!("[*] LEA"),
            _ if op == Opcodes::TRAP as u16 => println!("[*] TRAP"),
            _             => {
                println!("[!] Something else!");
                running = false;
            }
        }

        running = false;
    }
}

fn main() {
    run();
}
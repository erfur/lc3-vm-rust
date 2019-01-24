extern crate termios;

// use std;
use std::env;
use std::io::Read;
// use std::error::Error;
use std::fs::File;
use std::fs;
// use std::io::prelude::*;
use std::path::Path;
use std::io;
use std::os::unix::io::RawFd;
use termios::*;

enum Registers {
    R0 = 0,
    _R1,
    _R2,
    _R3,
    _R4,
    _R5,
    _R6,
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
 * them easily; I have to do it in an extra if
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
    TRAP,   /* execute trap */
}

enum TrapCodes {
    GETC = 0x20,  /* get character from keyboard */
    OUT = 0x21,   /* output a character */
    PUTS = 0x22,  /* output a word string */
    IN = 0x23,    /* input a string */
    PUTSP = 0x24, /* output a byte string */
    HALT = 0x25,  /* halt the program */
}

fn sign_extend(x: u16, bit_count: u16) -> u16{
    if (x >> (bit_count - 1)) & 1 != 0 {
        return x | (0xffff << bit_count);
    }
    x
}

fn update_flags(regs_ref: &mut [u16], r: usize) {
    regs_ref[Registers::COND as usize] =  match regs_ref[r] {
        0                   => ConditionFlags::ZRO as u16,
        r if r >> 15 == 1   => ConditionFlags::NEG as u16,
        _                   => ConditionFlags::POS as u16,
    };
}

enum MemoryMappedRegisters {
    KBSR = 0xfe00, /* keyboard status */
    KBDR = 0xfe02, /* keyboard data */
}

fn mem_read(mem: &mut [u16], addr: u16) -> u16 {
    if addr == MemoryMappedRegisters::KBSR as u16 {
        let mut buffer = [0; 1];
        std::io::stdin().read_exact(&mut buffer).unwrap();
        // println!("[*] MEM_READ");
        if buffer[0] != 0 {
            mem[MemoryMappedRegisters::KBSR as usize] = 1 << 15;
            mem[MemoryMappedRegisters::KBDR as usize] = buffer[0] as u16;
        } else {
            mem[MemoryMappedRegisters::KBSR as usize] = 0;
        }
    }
    mem[addr as usize]
}

fn mem_write(mem: &mut [u16], ind: u16, val: u16) {
    mem[ind as usize] = val;
}

fn abort() {
    std::process::exit(1);
}

// fn swap16(x: u16) -> u16 {
//     x << 8 | x >> 8
// }

fn get16(mem: &[u8], ind: usize) -> u16 {
    ((mem[ind] as u16) << 8) + mem[ind+1] as u16
}

// https://doc.rust-lang.org/rust-by-example/std_misc/file/open.html
fn read_image(mem: &mut [u16], image_path: &str) -> u32 {
    let path = Path::new(image_path);
     // println!("[*] Loading {}", path.to_str().unwrap());
    let mut file = File::open(&path).expect("Couldn't open file.");

    const SIZE: u32 = std::u16::MAX as u32 * 2 - 2;
    let mut mem_buffer: [u8; SIZE as usize] = [0; SIZE as usize];
    file.read(&mut mem_buffer).expect("Couldn't read file.");
    let length = file.metadata().unwrap().len();
     // println!("[*] File length {}", length);

    let base = get16(&mem_buffer, 0) as usize;
    for i in (2..length).step_by(2) {
        // println!("{}",i);
        mem[base+(i/2 - 1) as usize] = get16(&mem_buffer, i as usize);
    }
    // println!("{:?}", &mem[0x3000..0x4000]);
    length as u32
}

fn run(args: &Vec<String>) {
    // Define and initialize the memory
    let mut memory: [u16; std::u16::MAX as usize]  = [0; std::u16::MAX as usize];
    // println!("{:?}", &memory[..]);

    for image in args.iter().skip(1) {
        read_image(&mut memory, image);
    }

    // Define and initialize registers
    let mut regs: [u16; Registers::COUNT as usize] = [0; Registers::COUNT as usize];
    // println!("{:?}", &_regs[..]);

    // Set the PC
    let pc_start: u16 = 0x3000;
    regs[Registers::PC as usize] = pc_start;

    let mut running: bool = true;
    while running {
        let instruction: u16 = mem_read(&mut memory, regs[Registers::PC as usize]);
        regs[Registers::PC as usize] += 1;
         // println!("[*] PC@0x{:x}", regs[Registers::PC as usize]);
        let op: u16 = instruction >> 12;

        match op {
            // BRANCH
            _ if op == Opcodes::BR  as u16  => {
                let pc_offset: u16 = sign_extend(instruction & 0x1ff, 9);
                let cond_flag: u16 = instruction >> 9 & 0b111;
                 // println!("[*] BR {} {}", pc_offset, cond_flag);

                if cond_flag & regs[Registers::COND as usize] != 0 {
                    regs[Registers::PC as usize] += pc_offset;
                }

            },
            // ADD
            _ if op == Opcodes::ADD as u16  => {
                let r0: usize = (instruction >> 9 & 0b111) as usize; /* DST */
                let r1: usize = (instruction >> 6 & 0b111) as usize; /* SRC */
                let imm_flag: u16 = instruction >> 5 & 0b1;          /* Immediate mode flag */
                 // println!("[*] ADD {} {} {}", r0, r1, imm_flag);

                if imm_flag == 1 {
                    let imm5 = sign_extend(instruction & 0b11111, 5);
                    regs[r0] = regs[r1] + imm5;
                } else {
                    let r2: usize = (instruction & 0b111) as usize;
                    regs[r0] = regs[r1] + regs[r2];
                }

                update_flags(&mut regs, r0);

            },
            // LOAD
            _ if op == Opcodes::LD  as u16  => {
                let r0: usize = (instruction >> 9 & 0b111) as usize;
                let pc_offset: u16 = sign_extend(instruction & 0x1ff, 9);
                 // println!("[*] LOAD {} {}", r0, pc_offset);

                regs[r0] = mem_read(&mut memory, regs[Registers::PC as usize] + pc_offset);
                update_flags(&mut regs, r0);

            },
            // STORE
            _ if op == Opcodes::ST  as u16  => {
                let r0: usize = (instruction >> 9 & 0b111) as usize;
                let pc_offset: u16 = sign_extend(instruction & 0x1ff, 9);
                 // println!("[*] STORE {} {}", r0, pc_offset);

                mem_write(&mut memory, regs[Registers::PC as usize] + pc_offset, regs[r0]);

            },
            // JUMP REGISTER
            _ if op == Opcodes::JSR as u16  => {
                let r1: usize = (instruction >> 6 & 0b111) as usize;
                let long_pc_offset: u16 = sign_extend(instruction & 0x7ff, 11);
                let long_flag: u16 = instruction >> 11 & 1;
                 // println!("[*] JUMP {} {} {}", r1, long_pc_offset, long_flag);

                regs[Registers::R7 as usize] = regs[Registers::PC as usize];
                if long_flag != 0 {
                    regs[Registers::PC as usize] += long_pc_offset;
                } else {
                    regs[Registers::PC as usize] = regs[r1];
                }
                
            },
            // BITWISE AND
            _ if op == Opcodes::AND as u16  => {
                let r0: usize = (instruction >> 9 & 0b111) as usize;
                let r1: usize = (instruction >> 6 & 0b111) as usize;
                let imm_flag: u16 = instruction >> 5 & 0b1;
                 // println!("[*] AND {} {} {}", r0, r1, imm_flag);

                if imm_flag == 1 {
                    let imm5: u16 = sign_extend(instruction & 0x1f, 5);
                    regs[r0] = regs[r1] & imm5;
                } else {
                    let r2: usize = (instruction & 0b111) as usize;
                    regs[r0] = regs[r1] & regs[r2];
                }
                update_flags(&mut regs, r0);

            },
            // LOAD REGISTER
            _ if op == Opcodes::LDR as u16  => {
                let r0: usize = (instruction >> 9 & 0b111) as usize;
                let r1: usize = (instruction >> 6 & 0b111) as usize;
                let offset: u16 = sign_extend(instruction & 0x3f, 6);
                 // println!("[*] LDR {} {} {}", r0, r1, offset);

                regs[r0] = mem_read(&mut memory, regs[r1] + offset);
                update_flags(&mut regs, r0);

            },
            // STORE REGISTER
            _ if op == Opcodes::STR as u16  => {
                let r0: usize = (instruction >> 9 & 0b111) as usize;
                let r1: usize = (instruction >> 6 & 0b111) as usize;
                let offset: u16 = sign_extend(instruction & 0x3f, 6);
                 // println!("[*] STR {} {} {}", r0, r1 ,offset);

                mem_write(&mut memory, regs[r1] + offset, regs[r0]);

            },
            _ if op == Opcodes::RTI as u16  => {
                 // println!("[*] BAD OPCODE(RTI)");
                abort();
            },
            // BITWISE NOT
            _ if op == Opcodes::NOT as u16  => {
                let r0: usize = (instruction >> 9 & 0b111) as usize;
                let r1: usize = (instruction >> 6 & 0b111) as usize;
                 // println!("[*] NOT {} {}", r0, r1);

                regs[r0] = !regs[r1];
                update_flags(&mut regs, r0);
            },
            // LOAD INDIRECT
            _ if op == Opcodes::LDI as u16  => {
                let r0: usize = (instruction >> 9 & 0b111) as usize;
                let pc_offset: u16 = sign_extend(instruction & 0x1ff, 9);

                let addr: u16 = mem_read(&mut memory, regs[Registers::PC as usize] + pc_offset);
                 // println!("[*] LDI {} {} addr: {}", r0, pc_offset, addr);
                regs[r0] = mem_read(&mut memory, addr);
                update_flags(&mut regs, r0);
            },
            // STORE INDIRECT
            _ if op == Opcodes::STI as u16  => {
                 // println!("[*] BAD OPCODE(STI)");
                abort();
            },
            // JUMP
            _ if op == Opcodes::JMP as u16  => {
                let r1: usize = (instruction >> 6 & 0b111) as usize;
                 // println!("[*] JMP {}", r1);
                regs[Registers::PC as usize] = regs[r1];

            },
            // RESERVED(UNUSED)
            _ if op == Opcodes::RES as u16  => {
                 // println!("[*] RES");
            },
            // LOAD EFFECTIVE ADDRESS
            _ if op == Opcodes::LEA as u16  => {
                let r0: usize = (instruction >> 9 & 0b111) as usize;
                let pc_offset: u16 = sign_extend(instruction & 0x1ff, 9);
                 // println!("[*] LEA {} {}", r0, pc_offset);

                regs[r0] = regs[Registers::PC as usize] + pc_offset;
                update_flags(&mut regs, r0);

            },
            // TRAP
            _ if op == Opcodes::TRAP as u16 => {
                match instruction & 0xff {
                    // GET CHARACTER FROM KEYBOARD
                    _ if instruction & 0xff == TrapCodes::GETC as u16  => {
                        // https://stackoverflow.com/questions/30678953/how-to-read-a-single-character-from-input-as-u8
                        // regs[Registers::R0 as usize] = std::io::stdin()
                        //     .bytes()
                        //     .next()
                        //     .and_then(|result| result.ok())
                        //     .map(|byte| byte as u16)
                        //     .unwrap();
                        let mut buffer = [0; 1];
                        std::io::stdin().read_exact(&mut buffer).unwrap();
                        regs[Registers::R0 as usize] = buffer[0] as u16;
                        // println!("[*] GETC");
                    },
                    // OUTPUT A CHARACTER
                    _ if instruction & 0xff == TrapCodes::OUT  as u16  => {
                        let c = regs[Registers::R0 as usize] as u8;
                        print!("{}", c as char);
                         // println!("[*] OUT");
                    },
                    // OUTPUT A STRING
                    _ if instruction & 0xff == TrapCodes::PUTS as u16  => {
                         // println!("[*] PUTS");
                        for c in &memory[regs[Registers::R0 as usize] as usize..] {
                            let c8 = (c & 0xff) as u8;
                            if c8 != 0x00 {
                                print!("{}", c8 as char);
                            } else {
                                break;
                            }
                        }
                    },
                    // INPUT A STRING
                    _ if instruction & 0xff == TrapCodes::IN   as u16  => {
                        print!("Enter a character: ");
                        regs[Registers::R0 as usize] = std::io::stdin()
                            .bytes()
                            .next()
                            .and_then(|result| result.ok())
                            .map(|byte| byte as u16)
                            .unwrap();
                         // println!("[*] IN");
                    },
                    // OUTPUT A BYTE STRING
                    _ if instruction & 0xff == TrapCodes::PUTSP as u16 => {
                        for c in &memory[regs[Registers::R0 as usize] as usize..] {
                            let b1 = (*c >> 8) as u8;
                            let b2 = (*c & 0xff) as u8;
                            if b1!= 0 {
                                print!("{}", b1 as char);
                                if b2 != 0 {
                                    print!("{}", b2 as char);
                                }
                            }
                        }
                         // println!("[*] PUTSP");
                    },
                    // HALT THE PROGRAM
                    _ if instruction & 0xff == TrapCodes::HALT  as u16 => {
                        println!("[!] HALT");
                        running = false;
                    },
                    _ => {
                        println!("[!] UNKNOWN TRAPCODE");
                        abort();
                    },
                }
            },
            _ => {
                println!("[!] UNKNOWN OPCODE");
                abort();
            },
        }
    }
}

fn main() {
    // https://stackoverflow.com/questions/26321592/how-can-i-read-one-character-from-stdin-without-having-to-hit-enter
    let stdin = 0; // couldn't get std::os::unix::io::FromRawFd to work 
                   // on /dev/stdin or /dev/tty
    let termios = Termios::from_fd(stdin).unwrap();
    let mut new_termios = termios.clone();  // make a mutable copy of termios 
                                            // that we will modify
    new_termios.c_iflag &= IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON;
    new_termios.c_lflag &= !(ICANON | ECHO); // no echo and canonical mode
    tcsetattr(stdin, TCSANOW, &mut new_termios).unwrap();

    let args: Vec<String> = env::args().collect();
    run(&args);

    tcsetattr(stdin, TCSANOW, & termios).unwrap();  // reset the stdin to 
                                                // original termios data
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_read_file() {
        let mut mem: [u16; std::u16::MAX as usize] = [0; std::u16::MAX as usize];
        assert_eq!(read_image(&mut mem, "Cargo.lock"), 0);
    }

    #[test]
    fn test_get16() {
        let mem: [u8; 2] = [0xaa, 0xbb];
        assert_eq!(get16(&mem, 0), 0xaabb);
    }
}
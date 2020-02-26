extern crate termios;

use std::env;
use std::fs::File;
use std::io::Read;
use std::path::Path;
use termios::*;

mod instruction;
use instruction::Instruction;
use instruction::Opcode;

enum Registers {
    R0 = 0,
    _R1,
    _R2,
    _R3,
    _R4,
    _R5,
    _R6,
    R7,
    PC, /* program counter */
    COUNT,
}

pub struct Flags {
    n: bool,
    z: bool,
    p: bool,
}

impl Flags {
    pub fn update(&mut self, reg: u16) {
        match reg {
            0x0000 => {
                self.n = false;
                self.z = true;
                self.p = false;
            }
            0x0001..=0x7fff => {
                self.n = false;
                self.p = true;
                self.z = false;
            }
            0x8000..=0xffff => {
                self.n = true;
                self.p = false;
                self.z = false;
            }
        };
    }
}

impl From<u8> for Flags {
    fn from(flags: u8) -> Flags {
        Flags {
            n: ((flags & 0b100) >> 2) == 1,
            z: ((flags & 0b010) >> 1) == 1,
            p: (flags & 0b001) == 1,
        }
    }
}

enum TrapCode {
    Getc(),  /* get character from keyboard */
    Out(),   /* output a character */
    Puts(),  /* output a word string */
    In(),    /* input a string */
    Putsp(), /* output a byte string */
    Halt(),  /* halt the program */
    Unknown(),
}

impl From<u8> for TrapCode {
    fn from(vector: u8) -> TrapCode {
        match vector {
            0x20 => TrapCode::Getc(),
            0x21 => TrapCode::Out(),
            0x22 => TrapCode::Puts(),
            0x23 => TrapCode::In(),
            0x24 => TrapCode::Putsp(),
            0x25 => TrapCode::Halt(),
            _ => TrapCode::Unknown(),
        }
    }
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

fn get16(mem: &[u8], ind: usize) -> u16 {
    ((mem[ind] as u16) << 8) + mem[ind + 1] as u16
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
        mem[base + (i / 2 - 1) as usize] = get16(&mem_buffer, i as usize);
    }
    // println!("{:?}", &mem[0x3000..0x4000]);
    length as u32
}

fn run(args: &Vec<String>) {
    // Define and initialize the memory
    let mut memory: [u16; std::u16::MAX as usize] = [0; std::u16::MAX as usize];
    // println!("{:?}", &memory[..]);

    for image in args.iter().skip(1) {
        read_image(&mut memory, image);
    }

    // Define and initialize registers
    let mut regs: [u16; Registers::COUNT as usize] = [0; Registers::COUNT as usize];
    let mut flags = Flags::from(0);
    // println!("{:?}", &_regs[..]);

    // Set the PC
    let pc_start: u16 = 0x3000;
    regs[Registers::PC as usize] = pc_start;

    let mut running: bool = true;
    while running {
        let opcode = Opcode::from(mem_read(&mut memory, regs[Registers::PC as usize]));
        // println!("[*] {:0b}", opcode.raw_value());
        let instruction = Instruction::new(opcode).unwrap();
        regs[Registers::PC as usize] += 1;
        // println!("[*] PC@0x{:x}", regs[Registers::PC as usize]);
        // println!("[*] {:?}", instruction);

        match instruction {
            // BRANCH
            Instruction::Br(n, z, p, offset) => {
                if (n & flags.n) || (z & flags.z) || (p & flags.p) {
                    regs[Registers::PC as usize] += offset;
                }
            }
            // ADD
            Instruction::AddReg(dr, sr1, sr2) => {
                regs[dr] = regs[sr1] + regs[sr2];
                flags.update(regs[dr]);
            }
            Instruction::AddImm(dr, sr1, imm5) => {
                regs[dr] = regs[sr1] + (imm5 as u16);
                flags.update(regs[dr]);
            }
            // LOAD
            Instruction::Ld(dr, offset) => {
                regs[dr] = mem_read(&mut memory, regs[Registers::PC as usize] + offset);
                flags.update(regs[dr]);
            }
            // STORE
            Instruction::St(sr, offset) => {
                mem_write(&mut memory, regs[Registers::PC as usize] + offset, regs[sr]);
            }
            // JUMP REGISTER
            Instruction::Jsr(offset) => {
                regs[Registers::R7 as usize] = regs[Registers::PC as usize];
                regs[Registers::PC as usize] += offset;
            }
            Instruction::Jsrr(base_r) => {
                regs[Registers::R7 as usize] = regs[Registers::PC as usize];
                regs[Registers::PC as usize] = regs[base_r];
            }
            // BITWISE AND
            Instruction::AndReg(dr, sr1, sr2) => {
                regs[dr] = regs[sr1] & regs[sr2];
                flags.update(regs[dr]);
            }
            Instruction::AndImm(dr, sr1, imm) => {
                regs[dr] = regs[sr1] & (imm as u16);
                flags.update(regs[dr]);
            }
            // LOAD REGISTER
            Instruction::Ldr(dr, base_r, offset) => {
                regs[dr] = mem_read(&mut memory, regs[base_r] + offset);
                flags.update(regs[dr]);
            }
            // STORE REGISTER
            Instruction::Str(dr, base_r, offset) => {
                mem_write(&mut memory, regs[base_r] + offset, regs[dr]);
            }
            Instruction::Rti() => {
                // println!("[*] BAD OPCODE(RTI)");
                abort();
            }
            // BITWISE NOT
            Instruction::Not(dr, sr) => {
                regs[dr] = !regs[sr];
                flags.update(regs[dr]);
            }
            // LOAD INDIRECT
            Instruction::Ldi(dr, offset) => {
                let addr: u16 = mem_read(&mut memory, regs[Registers::PC as usize] + offset);
                regs[dr] = mem_read(&mut memory, addr);
                flags.update(regs[dr]);
            }
            // STORE INDIRECT
            Instruction::Sti(_sr, _offset) => {
                // println!("[*] BAD OPCODE(STI)");
                abort();
            }
            // JUMP
            Instruction::Jmp(base_r) => {
                regs[Registers::PC as usize] = regs[base_r];
            }
            // RESERVED(UNUSED)
            Instruction::Reserved() => {
                // println!("[*] RES");
            }
            // LOAD EFFECTIVE ADDRESS
            Instruction::Lea(dr, offset) => {
                regs[dr] = regs[Registers::PC as usize] + offset;
                flags.update(regs[dr]);
            }
            // TRAP
            Instruction::Trap(trap_vector) => {
                match TrapCode::from(trap_vector) {
                    // GET CHARACTER FROM KEYBOARD
                    TrapCode::Getc() => {
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
                    }
                    // OUTPUT A CHARACTER
                    TrapCode::Out() => {
                        let c = regs[Registers::R0 as usize] as u8;
                        print!("{}", c as char);
                        // println!("[*] OUT");
                    }
                    // OUTPUT A STRING
                    TrapCode::Puts() => {
                        // println!("[*] PUTS");
                        for c in &memory[regs[Registers::R0 as usize] as usize..] {
                            let c8 = (c & 0xff) as u8;
                            if c8 != 0x00 {
                                print!("{}", c8 as char);
                            } else {
                                break;
                            }
                        }
                    }
                    // INPUT A STRING
                    TrapCode::In() => {
                        print!("Enter a character: ");
                        regs[Registers::R0 as usize] = std::io::stdin()
                            .bytes()
                            .next()
                            .and_then(|result| result.ok())
                            .map(|byte| byte as u16)
                            .unwrap();
                        // println!("[*] IN");
                    }
                    // OUTPUT A BYTE STRING
                    TrapCode::Putsp() => {
                        for c in &memory[regs[Registers::R0 as usize] as usize..] {
                            let b1 = (*c >> 8) as u8;
                            let b2 = (*c & 0xff) as u8;
                            if b1 != 0 {
                                print!("{}", b1 as char);
                                if b2 != 0 {
                                    print!("{}", b2 as char);
                                }
                            }
                        }
                        // println!("[*] PUTSP");
                    }
                    // HALT THE PROGRAM
                    TrapCode::Halt() => {
                        println!("[!] HALT");
                        running = false;
                    }
                    TrapCode::Unknown() => {
                        println!("[!] UNKNOWN TRAPCODE");
                        abort();
                    }
                }
            }
        }
    }
}

fn main() {
    // https://stackoverflow.com/questions/26321592/how-can-i-read-one-character-from-stdin-without-having-to-hit-enter
    let stdin = 0; // couldn't get std::os::unix::io::FromRawFd to work
                   // on /dev/stdin or /dev/tty
    let termios = Termios::from_fd(stdin).unwrap();
    let mut new_termios = termios.clone(); // make a mutable copy of termios
                                           // that we will modify
    new_termios.c_iflag &= IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON;
    new_termios.c_lflag &= !(ICANON | ECHO); // no echo and canonical mode
    tcsetattr(stdin, TCSANOW, &mut new_termios).unwrap();

    let args: Vec<String> = env::args().collect();
    run(&args);

    tcsetattr(stdin, TCSANOW, &termios).unwrap(); // reset the stdin to
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

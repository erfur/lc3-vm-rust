pub type Offset = u16;
pub type Imm = u16;
pub type TrapVector = u8;
pub type Flag = bool;
pub type Register = usize;

pub struct Opcode(u16);

impl Opcode {
    pub fn raw_value(&self) -> u16 {
        self.0
    }

    pub fn op_type(&self) -> u8 {
        ((self.0 & 0xF000) >> 12) as u8
    }

    pub fn xooo(&self) -> usize {
        ((self.0 & 0b0000_1110_0000_0000) >> 9) as usize
    }

    pub fn oxoo(&self) -> usize {
        ((self.0 & 0b0000_0001_1100_0000) >> 6) as usize
    }

    pub fn _ooxo(&self) -> usize {
        ((self.0 & 0b0000_0000_0011_1000) >> 3) as usize
    }

    pub fn ooox(&self) -> usize {
        (self.0 & 0b0000_0000_0000_0111) as usize
    }

    pub fn offset6(&self) -> u16 {
        Opcode::sign_extend(self.0 & 0b0000_0000_0011_1111, 6)
    }

    pub fn offset9(&self) -> u16 {
        Opcode::sign_extend(self.0 & 0b0000_0001_1111_1111, 9)
    }

    pub fn offset11(&self) -> u16 {
        Opcode::sign_extend(self.0 & 0b0000_0111_1111_1111, 11)
    }

    pub fn imm5(&self) -> u16 {
        Opcode::sign_extend(self.0 & 0b0011_1111, 5)
    }

    pub fn bit5(&self) -> bool {
        ((self.0 & 0b0000_0000_0010_0000) >> 5) == 1
    }

    pub fn bit11(&self) -> bool {
        ((self.0 & 0b0000_1000_0000_0000) >> 5) == 1
    }

    pub fn n(&self) -> bool {
        ((self.0 & 0b1000_0000_0000) >> 11) == 1
    }

    pub fn z(&self) -> bool {
        ((self.0 & 0b0100_0000_0000) >> 10) == 1
    }

    pub fn p(&self) -> bool {
        ((self.0 & 0b0010_0000_0000) >> 9) == 1
    }

    pub fn trap_vector(&self) -> u8 {
        (self.0 & 0xff) as u8
    }

    #[inline]
    fn sign_extend(data: u16, size: u16) -> u16 {
        (((data << (16 - size)) as i16) >> (16 - size)) as u16
    }
}

impl From<u16> for Opcode {
    fn from(opcode: u16) -> Opcode {
        Opcode(opcode)
    }
}

#[derive(Debug)]
pub enum Instruction {
    AddReg(Register, Register, Register),
    AddImm(Register, Register, Imm),
    AndReg(Register, Register, Register),
    AndImm(Register, Register, Imm),
    Br(Flag, Flag, Flag, Offset),
    Jmp(Register),
    Jsr(Offset),
    Jsrr(Register),
    Ld(Register, Offset),
    Ldi(Register, Offset),
    Ldr(Register, Register, Offset),
    Lea(Register, Offset),
    Not(Register, Register),
    Rti(),
    St(Register, Offset),
    Sti(Register, Offset),
    Str(Register, Register, Offset),
    Trap(TrapVector),
    Reserved(),
}

impl Instruction {
    pub fn new<T: Into<Opcode>>(opcode: T) -> Option<Instruction> {
        let opcode = opcode.into();
        match opcode.op_type() {
            0b0001 => match opcode.bit5() {
                false => Some(Instruction::AddReg(
                    opcode.xooo(),
                    opcode.oxoo(),
                    opcode.ooox(),
                )),
                true => Some(Instruction::AddImm(
                    opcode.xooo(),
                    opcode.oxoo(),
                    opcode.imm5(),
                )),
            },
            0b0101 => match opcode.bit5() {
                false => Some(Instruction::AndReg(
                    opcode.xooo(),
                    opcode.oxoo(),
                    opcode.ooox(),
                )),
                true => Some(Instruction::AndImm(
                    opcode.xooo(),
                    opcode.oxoo(),
                    opcode.imm5(),
                )),
            },
            0b0000 => Some(Instruction::Br(
                opcode.n(),
                opcode.z(),
                opcode.p(),
                opcode.offset9(),
            )),
            0b1100 => Some(Instruction::Jmp(opcode.oxoo())),
            0b0100 => match opcode.bit11() {
                false => Some(Instruction::Jsr(opcode.offset11())),
                true => Some(Instruction::Jsrr(opcode.oxoo())),
            },
            0b0010 => Some(Instruction::Ld(opcode.xooo(), opcode.offset9())),
            0b1010 => Some(Instruction::Ldi(opcode.xooo(), opcode.offset9())),
            0b0110 => Some(Instruction::Ldr(
                opcode.xooo(),
                opcode.oxoo(),
                opcode.offset6(),
            )),
            0b1110 => Some(Instruction::Lea(opcode.xooo(), opcode.offset9())),
            0b1001 => Some(Instruction::Not(opcode.xooo(), opcode.oxoo())),
            0b1000 => Some(Instruction::Rti()),
            0b0011 => Some(Instruction::St(opcode.xooo(), opcode.offset9())),
            0b1011 => Some(Instruction::Sti(opcode.xooo(), opcode.offset9())),
            0b0111 => Some(Instruction::Str(
                opcode.xooo(),
                opcode.oxoo(),
                opcode.offset6(),
            )),
            0b1111 => Some(Instruction::Trap(opcode.trap_vector())),
            0b1101 => Some(Instruction::Reserved()),
            _ => None,
        }
    }
}

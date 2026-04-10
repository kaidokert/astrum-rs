use crate::register::RegisterBank;

const REG_SLOTS: usize = 18;

pub struct ArrayRegisterBank {
    regs: [u32; REG_SLOTS],
}

impl Default for ArrayRegisterBank {
    fn default() -> Self {
        Self {
            regs: [0; REG_SLOTS],
        }
    }
}

impl ArrayRegisterBank {
    pub fn new() -> Self {
        Self::default()
    }
}

impl RegisterBank for ArrayRegisterBank {
    fn read(&self, offset: usize) -> u32 {
        self.regs[offset / 4]
    }
    fn write(&mut self, offset: usize, value: u32) {
        self.regs[offset / 4] = value;
    }
}

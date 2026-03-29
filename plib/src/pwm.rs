//! PWM0 channel wrapper implementing [`embedded_hal::PwmPin`].
//!
//! Thin abstraction over raw PWM0 register pointers for the LM3S6965
//! (QEMU `lm3s6965evb`). Each channel corresponds to one PWM generator
//! (0–2) using output A with CMPA-based duty control.

use core::cell::RefCell;

use embedded_hal::PwmPin;
use rtos_traits::register::{MmioRegisterBank, RegisterBank};

// ── Register offsets (LM3S6965 PWM0) ────────────────────────────────
const ENABLE_OFF: usize = 0x008;
const GEN0_CTL_OFF: usize = 0x040;
const GEN_STRIDE: usize = 0x040;
const LOAD_REL: usize = 0x10; // relative to generator CTL
const CMPA_REL: usize = 0x18;
const GENA_REL: usize = 0x20; // PWMnGENA: output-A action control

// ── Bit-band constants (Cortex-M3 peripheral region) ────────────────
#[cfg(not(test))]
const PERIPH_BASE: usize = 0x4000_0000;
#[cfg(not(test))]
const PERIPH_BB_BASE: usize = 0x4200_0000;

/// Compute the bit-band alias address for a single bit in the peripheral
/// region. Writing 1/0 to the alias atomically sets/clears that bit
/// without a read-modify-write on the underlying register.
#[cfg(not(test))]
#[inline]
fn periph_bb_addr(reg_addr: usize, bit: u32) -> *mut u32 {
    let byte_off = reg_addr - PERIPH_BASE;
    (PERIPH_BB_BASE + byte_off * 32 + (bit as usize) * 4) as *mut u32
}

/// GENA value: drive high on LOAD, drive low on CMPA-down.
/// ActLoad (bits 3:2) = 0b11 (high), ActCmpAD (bits 7:6) = 0b10 (low).
const GENA_HIGH_ON_LOAD_LOW_ON_CMPA: u32 = (0b11 << 2) | (0b10 << 6);

/// Wrapper around a single PWM0 generator channel.
///
/// `Duty` type is `u16`, matching the 16-bit LOAD / CMP registers.
/// Duty semantics: `set_duty(0)` → 0 %, `set_duty(get_max_duty())` → 100 %.
pub struct Pwm0Channel<R: RegisterBank> {
    base_addr: usize,
    regs: RefCell<R>,
    channel: u8,
}

/// Production type alias using MMIO-backed registers.
pub type HwPwm0Channel = Pwm0Channel<MmioRegisterBank>;

// SAFETY: Each `Pwm0Channel` owns exclusive access to its generator's registers.
// The shared ENABLE register is modified via Cortex-M3 bit-band aliases, which
// provide atomic single-bit set/clear without read-modify-write races.
// The caller of `new` guarantees no aliasing of the same channel.
unsafe impl<R: RegisterBank> Send for Pwm0Channel<R> {}

/// Errors returned by [`Pwm0Channel::new`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pwm0Error {
    /// `base_addr` is not 4-byte aligned.
    UnalignedBase,
    /// `channel` is not in 0..=2.
    InvalidChannel,
}

impl<R: RegisterBank> Pwm0Channel<R> {
    /// Create a new PWM0 channel wrapper.
    ///
    /// # Errors
    ///
    /// Returns [`Pwm0Error::UnalignedBase`] if `base_addr` is not 4-byte aligned.
    /// Returns [`Pwm0Error::InvalidChannel`] if `channel >= 3`.
    pub fn with_backend(base_addr: usize, backend: R, channel: u8) -> Result<Self, Pwm0Error> {
        if base_addr & 0x3 != 0 {
            return Err(Pwm0Error::UnalignedBase);
        }
        if channel >= 3 {
            return Err(Pwm0Error::InvalidChannel);
        }
        Ok(Self {
            base_addr,
            regs: RefCell::new(backend),
            channel,
        })
    }

    /// Byte offset from base to this generator's CTL register.
    #[inline]
    fn gen_ctl_off(&self) -> usize {
        GEN0_CTL_OFF + (self.channel as usize) * GEN_STRIDE
    }

    #[inline]
    fn read_reg(&self, off: usize) -> u32 {
        self.regs.borrow().read(off)
    }

    #[inline]
    fn write_reg(&self, off: usize, val: u32) {
        self.regs.borrow_mut().write(off, val);
    }

    /// Bit mask for this channel's output-A in the ENABLE register.
    #[cfg(test)]
    #[inline]
    fn enable_mask(&self) -> u32 {
        1u32 << (2 * self.channel as u32)
    }

    /// Atomically set or clear this channel's ENABLE bit via bit-band alias.
    #[cfg(not(test))]
    #[inline]
    fn bb_write_enable(&self, val: u32) {
        let enable_addr = self.base_addr + ENABLE_OFF;
        let bit = 2 * self.channel as u32;
        let bb = periph_bb_addr(enable_addr, bit);
        // SAFETY: `self.base_addr` is in the peripheral region (guaranteed by
        // caller of `new`), so the bit-band alias address is valid.
        unsafe { core::ptr::write_volatile(bb, val) }
    }

    /// Test fallback: read-modify-write (no bit-band on host).
    #[cfg(test)]
    #[inline]
    fn bb_write_enable(&self, val: u32) {
        let mask = self.enable_mask();
        let cur = self.read_reg(ENABLE_OFF);
        if val != 0 {
            self.write_reg(ENABLE_OFF, cur | mask);
        } else {
            self.write_reg(ENABLE_OFF, cur & !mask);
        }
    }
}

impl HwPwm0Channel {
    /// Create a new PWM0 channel backed by MMIO registers.
    ///
    /// # Safety
    /// Same as [`Pwm0Channel::with_backend`], plus `base_addr` must be valid MMIO.
    pub unsafe fn new(base_addr: usize, channel: u8) -> Result<Self, Pwm0Error> {
        Self::with_backend(
            base_addr,
            MmioRegisterBank::new(base_addr as *mut u32),
            channel,
        )
    }
}

impl<R: RegisterBank> PwmPin for Pwm0Channel<R> {
    type Duty = u16;

    fn enable(&mut self) {
        let ctl_off = self.gen_ctl_off();
        // Configure output-A actions: high on LOAD, low on CMPA-down.
        self.write_reg(ctl_off + GENA_REL, GENA_HIGH_ON_LOAD_LOW_ON_CMPA);
        // Set generator CTL enable bit (bit 0).
        self.write_reg(ctl_off, self.read_reg(ctl_off) | 1);
        // Atomically set this channel's output-enable bit via bit-band alias.
        self.bb_write_enable(1);
    }

    fn disable(&mut self) {
        // Atomically clear this channel's output-enable bit via bit-band alias.
        self.bb_write_enable(0);
        // Clear generator CTL enable bit.
        let ctl_off = self.gen_ctl_off();
        self.write_reg(ctl_off, self.read_reg(ctl_off) & !1);
    }

    fn get_duty(&self) -> u16 {
        let load = self.read_reg(self.gen_ctl_off() + LOAD_REL) as u16;
        let cmpa = self.read_reg(self.gen_ctl_off() + CMPA_REL) as u16;
        load.saturating_sub(cmpa)
    }

    fn get_max_duty(&self) -> u16 {
        self.read_reg(self.gen_ctl_off() + LOAD_REL) as u16
    }

    fn set_duty(&mut self, duty: u16) {
        let load = self.read_reg(self.gen_ctl_off() + LOAD_REL) as u16;
        let cmpa = load.saturating_sub(duty);
        self.write_reg(self.gen_ctl_off() + CMPA_REL, cmpa as u32);
    }
}

#[cfg(any(test, feature = "mock-hal"))]
pub type MockPwm0Channel = Pwm0Channel<rtos_traits::register::MockRegisterBank>;
#[cfg(any(test, feature = "mock-hal"))]
impl MockPwm0Channel {
    pub fn new_mock(channel: u8) -> Result<Self, Pwm0Error> {
        Self::with_backend(
            0x4002_8000,
            rtos_traits::register::MockRegisterBank::new(),
            channel,
        )
    }
    pub fn set_reg(&self, offset: usize, value: u32) {
        self.regs.borrow_mut().set(offset, value);
    }
    pub fn assert_sequence(&self, expected: &[(usize, u32)]) {
        self.regs.borrow().assert_sequence(expected);
    }
    pub fn assert_wrote(&self, offset: usize, value: u32) {
        self.regs.borrow().assert_wrote(offset, value);
    }
    pub fn peek_reg(&self, offset: usize) -> u32 {
        self.regs.borrow().read(offset)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const GENA_VAL: u32 = GENA_HIGH_ON_LOAD_LOW_ON_CMPA;

    fn mock_ch(channel: u8) -> MockPwm0Channel {
        MockPwm0Channel::new_mock(channel).unwrap()
    }

    #[test]
    fn new_validation() {
        let ch = mock_ch(2);
        assert_eq!(ch.base_addr, 0x4002_8000);
        assert_eq!(ch.channel, 2);
        let r = MockPwm0Channel::with_backend(
            0x4002_8001,
            rtos_traits::register::MockRegisterBank::new(),
            0,
        );
        assert!(matches!(r, Err(Pwm0Error::UnalignedBase)));
        assert!(matches!(
            MockPwm0Channel::new_mock(3),
            Err(Pwm0Error::InvalidChannel)
        ));
    }

    #[test]
    fn enable_write_sequence() {
        let mut ch0 = mock_ch(0);
        ch0.enable();
        ch0.assert_sequence(&[(0x060, GENA_VAL), (0x040, 1), (ENABLE_OFF, 1)]);
        let mut ch1 = mock_ch(1);
        ch1.enable();
        ch1.assert_sequence(&[(0x0A0, GENA_VAL), (0x080, 1), (ENABLE_OFF, 0b100)]);
        let mut ch2 = mock_ch(0);
        ch2.set_reg(0x040, 0xF0);
        ch2.enable();
        ch2.assert_wrote(0x040, 0xF1);
    }

    #[test]
    fn set_duty_writes_cmpa_per_channel() {
        let mut ch0 = mock_ch(0);
        ch0.set_reg(0x050, 999);
        ch0.set_duty(500);
        ch0.assert_wrote(0x058, 499);
        assert_eq!(ch0.get_duty(), 500);
        let mut ch1 = mock_ch(1);
        ch1.set_reg(0x090, 2000);
        ch1.set_duty(800);
        ch1.assert_wrote(0x098, 1200);
        assert_eq!(ch1.get_duty(), 800);
        let mut ch2 = mock_ch(2);
        ch2.set_reg(0x0D0, 3000);
        ch2.set_duty(750);
        ch2.assert_wrote(0x0D8, 2250);
        assert_eq!(ch2.get_duty(), 750);
    }

    #[test]
    fn get_duty_and_boundary_cases() {
        let ch = mock_ch(0);
        ch.set_reg(0x050, 1000);
        ch.set_reg(0x058, 300);
        assert_eq!(ch.get_duty(), 700);
        assert_eq!(ch.get_max_duty(), 1000);
        let mut ch2 = mock_ch(0);
        ch2.set_reg(0x050, 999);
        ch2.set_duty(999);
        ch2.assert_wrote(0x058, 0);
        assert_eq!(ch2.get_duty(), 999);
        let ch1 = mock_ch(1);
        ch1.set_reg(0x090, 7999);
        assert_eq!(ch1.get_max_duty(), 7999);
    }

    #[test]
    fn channel_isolation_enable_bits() {
        let mut ch0 = mock_ch(0);
        let mut ch1 = mock_ch(1);
        ch0.enable();
        assert_eq!(ch0.peek_reg(ENABLE_OFF) & 0b100, 0);
        ch1.enable();
        assert_eq!(ch1.peek_reg(ENABLE_OFF) & 0b001, 0);
        let mut ch1b = mock_ch(1);
        ch1b.set_reg(ENABLE_OFF, 0b001);
        ch1b.enable();
        ch1b.assert_wrote(ENABLE_OFF, 0b101);
    }

    #[test]
    fn disable_clears_bits_and_preserves_others() {
        let mut ch = mock_ch(0);
        ch.set_reg(0x040, 0xFF);
        ch.set_reg(ENABLE_OFF, 0b101);
        ch.disable();
        ch.assert_wrote(ENABLE_OFF, 0b100);
        ch.assert_wrote(0x040, 0xFE);
    }
}

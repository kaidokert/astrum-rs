//! UART HAL register definitions for the LM3S6965 (Stellaris).
//!
//! Provides [`UartRegs`], a thin wrapper around a UART base address that
//! exposes raw volatile register accessors, plus a pure-function
//! baud-rate divisor calculation suitable for any clock frequency.

// ---------------------------------------------------------------------------
// Register offsets (LM3S6965 datasheet, Table 12-3)
// ---------------------------------------------------------------------------

/// Data Register offset.
pub const DR: u32 = 0x000;
/// Flag Register offset.
pub const FR: u32 = 0x018;
/// Integer Baud-Rate Divisor Register offset.
pub const IBRD: u32 = 0x024;
/// Fractional Baud-Rate Divisor Register offset.
pub const FBRD: u32 = 0x028;
/// Line Control Register offset.
pub const LCRH: u32 = 0x02C;
/// Control Register offset.
pub const CTL: u32 = 0x030;
/// Interrupt FIFO Level Select Register offset.
pub const IFLS: u32 = 0x034;
/// Interrupt Mask Register offset.
pub const IM: u32 = 0x038;
/// Raw Interrupt Status Register offset.
pub const RIS: u32 = 0x03C;
/// Interrupt Clear Register offset.
pub const ICR: u32 = 0x044;

// ---------------------------------------------------------------------------
// Flag Register (FR) bit masks
// ---------------------------------------------------------------------------

/// Transmit FIFO Full (bit 5).
pub const FR_TXFF: u32 = 1 << 5;
/// Receive FIFO Empty (bit 4).
pub const FR_RXFE: u32 = 1 << 4;
/// UART Busy (bit 3).
pub const FR_BUSY: u32 = 1 << 3;

// ---------------------------------------------------------------------------
// UartRegs — volatile register accessor
// ---------------------------------------------------------------------------

/// Thin wrapper around a UART peripheral base address.
///
/// All accesses go through raw volatile reads/writes so the compiler
/// cannot reorder or elide them.  The caller is responsible for
/// ensuring the base address is valid and the UART clock is enabled.
pub struct UartRegs {
    base: u32,
}

impl UartRegs {
    /// Create a new `UartRegs` handle for the given MMIO base address.
    pub const fn new(base: u32) -> Self {
        Self { base }
    }

    /// Return the base address.
    pub const fn base(&self) -> u32 {
        self.base
    }

    /// Read a 32-bit register at `offset` from the base address.
    ///
    /// # Safety
    ///
    /// `offset` must be a valid register offset for this UART block and
    /// the base address must point to a mapped UART peripheral.
    #[inline]
    pub unsafe fn read(&self, offset: u32) -> u32 {
        core::ptr::read_volatile((self.base + offset) as *const u32)
    }

    /// Write a 32-bit value to a register at `offset` from the base.
    ///
    /// # Safety
    ///
    /// `offset` must be a valid register offset for this UART block and
    /// the base address must point to a mapped UART peripheral.
    #[inline]
    pub unsafe fn write(&self, offset: u32, val: u32) {
        core::ptr::write_volatile((self.base + offset) as *mut u32, val)
    }
}

// ---------------------------------------------------------------------------
// Baud-rate divisor calculation
// ---------------------------------------------------------------------------

/// Compute the integer and fractional baud-rate divisor registers for
/// the given clock frequency and desired baud rate.
///
/// Returns `(ibrd, fbrd)` where `ibrd` is the 16-bit integer part and
/// `fbrd` is the 6-bit fractional part, ready to be written to the
/// IBRD and FBRD registers respectively.
///
/// The formula (from the LM3S6965 datasheet, section 12.4):
///
/// ```text
/// BRD    = clock_hz / (16 * baud)
/// IBRD   = integer(BRD)
/// FBRD   = integer(fraction(BRD) * 64 + 0.5)
/// ```
///
/// Returns `(0, 0)` if `baud` is zero.
pub fn compute_baud_divisors(clock_hz: u32, baud: u32) -> (u16, u8) {
    if baud == 0 {
        return (0, 0);
    }

    // Use 128× scaled arithmetic so we can round the 6-bit fractional part.
    // BRD × 128 = (clock_hz × 128) / (16 × baud) = (clock_hz × 8) / baud
    let brd_times_128 = (clock_hz as u64 * 8) / baud as u64;

    let mut ibrd = (brd_times_128 / 128) as u16;
    let frac_128 = (brd_times_128 % 128) as u32;
    // Round: integer(fraction(BRD) × 64 + 0.5) = div_ceil(frac_128, 2)
    let mut fbrd = frac_128.div_ceil(2) as u8;

    // If rounding causes fbrd to reach 64, carry into ibrd per datasheet.
    if fbrd >= 64 {
        fbrd = 0;
        ibrd = ibrd.wrapping_add(1);
    }

    (ibrd, fbrd)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -- Register offset constants ------------------------------------------

    #[test]
    fn register_offsets() {
        assert_eq!(DR, 0x000);
        assert_eq!(FR, 0x018);
        assert_eq!(IBRD, 0x024);
        assert_eq!(FBRD, 0x028);
        assert_eq!(LCRH, 0x02C);
        assert_eq!(CTL, 0x030);
        assert_eq!(IFLS, 0x034);
        assert_eq!(IM, 0x038);
        assert_eq!(RIS, 0x03C);
        assert_eq!(ICR, 0x044);
    }

    #[test]
    fn flag_register_bits() {
        assert_eq!(FR_TXFF, 0x20); // bit 5
        assert_eq!(FR_RXFE, 0x10); // bit 4
        assert_eq!(FR_BUSY, 0x08); // bit 3
                                   // Bits should not overlap
        assert_eq!(FR_TXFF & FR_RXFE, 0);
        assert_eq!(FR_TXFF & FR_BUSY, 0);
        assert_eq!(FR_RXFE & FR_BUSY, 0);
    }

    // -- UartRegs constructor -----------------------------------------------

    #[test]
    fn uart_regs_base_address() {
        let uart = UartRegs::new(0x4000_C000);
        assert_eq!(uart.base(), 0x4000_C000);
    }

    #[test]
    fn uart_regs_const_constructor() {
        // Verify the constructor is usable in const context.
        const UART: UartRegs = UartRegs::new(0x4000_D000);
        assert_eq!(UART.base(), 0x4000_D000);
    }

    // -- Baud-rate divisor calculation --------------------------------------

    /// 12 MHz / (16 × 115200) = 6.51041666…
    /// IBRD = 6, FBRD = int(0.51041666… × 64 + 0.5) = int(33.166…) = 33
    #[test]
    fn baud_115200_at_12mhz() {
        let (ibrd, fbrd) = compute_baud_divisors(12_000_000, 115_200);
        assert_eq!(ibrd, 6);
        assert_eq!(fbrd, 33);
    }

    /// 12 MHz / (16 × 9600) = 78.125
    /// IBRD = 78, FBRD = int(0.125 × 64 + 0.5) = int(8.5) = 8
    #[test]
    fn baud_9600_at_12mhz() {
        let (ibrd, fbrd) = compute_baud_divisors(12_000_000, 9600);
        assert_eq!(ibrd, 78);
        assert_eq!(fbrd, 8);
    }

    /// Edge case: baud = 0 should not panic.
    #[test]
    fn baud_zero_returns_zero() {
        let (ibrd, fbrd) = compute_baud_divisors(12_000_000, 0);
        assert_eq!(ibrd, 0);
        assert_eq!(fbrd, 0);
    }

    /// Edge case: very high baud rate relative to clock.
    /// BRD = 1000 / (16 × 1000) = 0.0625
    /// IBRD = 0, FBRD = int(0.0625 × 64 + 0.5) = int(4.5) = 4
    #[test]
    fn baud_equal_to_clock() {
        let (ibrd, fbrd) = compute_baud_divisors(1_000, 1_000);
        assert_eq!(ibrd, 0);
        assert_eq!(fbrd, 4);
    }

    /// 12 MHz / (16 × 38400) = 19.53125
    /// IBRD = 19, FBRD = int(0.53125 × 64 + 0.5) = int(34.5) = 34
    #[test]
    fn baud_38400_at_12mhz() {
        let (ibrd, fbrd) = compute_baud_divisors(12_000_000, 38_400);
        assert_eq!(ibrd, 19);
        assert_eq!(fbrd, 34);
    }

    /// Verify that baud = 1 doesn't overflow with a 12 MHz clock.
    /// The u16 truncation of ibrd is expected for extreme inputs.
    #[test]
    fn baud_one_no_panic() {
        let (_ibrd, fbrd) = compute_baud_divisors(12_000_000, 1);
        assert!(fbrd <= 63);
    }

    /// 120 MHz / (16 × 115200) = 65.10416…
    /// IBRD = 65, FBRD = int(0.10416… × 64 + 0.5) = int(7.166…) = 7
    #[test]
    fn baud_115200_at_120mhz() {
        let (ibrd, fbrd) = compute_baud_divisors(120_000_000, 115_200);
        assert_eq!(ibrd, 65);
        assert_eq!(fbrd, 7);
    }
}

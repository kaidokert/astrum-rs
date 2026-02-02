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

// LCRH bit masks
/// Word length 8 bits: WLEN = 0b11 (bits 6:5).
pub const LCRH_WLEN_8: u32 = 0b11 << 5;
/// Enable FIFOs (bit 4).
pub const LCRH_FEN: u32 = 1 << 4;
/// 8N1 with FIFOs enabled.
pub const LCRH_8N1_FIFO: u32 = LCRH_WLEN_8 | LCRH_FEN;

// CTL bit masks
/// UART Enable (bit 0).
pub const CTL_UARTEN: u32 = 1 << 0;
/// Transmit Enable (bit 8).
pub const CTL_TXE: u32 = 1 << 8;
/// Receive Enable (bit 9).
pub const CTL_RXE: u32 = 1 << 9;
/// UART enabled with TX and RX.
pub const CTL_ENABLE_TXRX: u32 = CTL_UARTEN | CTL_TXE | CTL_RXE;

// IM / ICR bit masks
/// Receive interrupt mask bit (bit 4).
pub const IM_RXIM: u32 = 1 << 4;
/// Receive timeout interrupt mask bit (bit 6).
pub const IM_RTIM: u32 = 1 << 6;
/// Receive interrupt clear bit (bit 4).
pub const ICR_RXIC: u32 = 1 << 4;
/// Receive timeout interrupt clear bit (bit 6).
pub const ICR_RTIC: u32 = 1 << 6;

// ---------------------------------------------------------------------------
// UartRegs — volatile register accessor
// ---------------------------------------------------------------------------

/// Number of 32-bit register slots in the mock backing store.
/// Covers offsets 0x000..=0x044 (i.e. ICR) → 18 words.
#[cfg(test)]
const MOCK_REG_COUNT: usize = 18;

/// Thin wrapper around a UART peripheral base address.
///
/// All accesses go through raw volatile reads/writes so the compiler
/// cannot reorder or elide them.  The caller is responsible for
/// ensuring the base address is valid and the UART clock is enabled.
///
/// In test builds the struct carries a `Cell`-based mock register file
/// so that higher-level methods can be unit-tested without hardware.
pub struct UartRegs {
    base: u32,
    #[cfg(test)]
    mock_regs: [core::cell::Cell<u32>; MOCK_REG_COUNT],
}

impl UartRegs {
    /// Create a new `UartRegs` handle for the given MMIO base address.
    #[cfg(not(test))]
    pub const fn new(base: u32) -> Self {
        Self { base }
    }

    /// Create a new `UartRegs` handle with a mock register file (test only).
    #[cfg(test)]
    pub fn new(base: u32) -> Self {
        #[allow(clippy::declare_interior_mutable_const)]
        const ZERO: core::cell::Cell<u32> = core::cell::Cell::new(0);
        Self {
            base,
            mock_regs: [ZERO; MOCK_REG_COUNT],
        }
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
    #[cfg(not(test))]
    pub unsafe fn read(&self, offset: u32) -> u32 {
        core::ptr::read_volatile((self.base + offset) as *const u32)
    }

    #[inline]
    #[cfg(test)]
    pub fn read(&self, offset: u32) -> u32 {
        self.mock_regs[(offset / 4) as usize].get()
    }

    /// Write a 32-bit value to a register at `offset` from the base.
    ///
    /// # Safety
    ///
    /// `offset` must be a valid register offset for this UART block and
    /// the base address must point to a mapped UART peripheral.
    #[inline]
    #[cfg(not(test))]
    pub unsafe fn write(&self, offset: u32, val: u32) {
        core::ptr::write_volatile((self.base + offset) as *mut u32, val)
    }

    #[inline]
    #[cfg(test)]
    pub fn write(&self, offset: u32, val: u32) {
        self.mock_regs[(offset / 4) as usize].set(val);
    }

    /// Safe wrapper around `read` for use by higher-level methods.
    ///
    /// In production this calls the unsafe volatile read (the caller of the
    /// higher-level method is responsible for guaranteeing the base address is
    /// valid).  In test builds it delegates to the safe mock implementation.
    #[inline]
    fn read_reg(&self, offset: u32) -> u32 {
        #[cfg(not(test))]
        // SAFETY: caller of the public method guarantees base points to a
        // valid UART peripheral.
        unsafe {
            self.read(offset)
        }
        #[cfg(test)]
        self.read(offset)
    }

    /// Safe wrapper around `write` for use by higher-level methods.
    #[inline]
    fn write_reg(&self, offset: u32, val: u32) {
        #[cfg(not(test))]
        // SAFETY: caller of the public method guarantees base points to a
        // valid UART peripheral.
        unsafe {
            self.write(offset, val)
        }
        #[cfg(test)]
        self.write(offset, val)
    }
}

/// Register values that `init()` would program. Exposed for unit testing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InitValues {
    pub ibrd: u16,
    pub fbrd: u8,
    pub lcrh: u32,
    pub ctl: u32,
}

impl UartRegs {
    /// Compute the register values that `init` would program (pure, no HW).
    pub fn compute_init_values(baud: u32, clock_hz: u32) -> InitValues {
        let (ibrd, fbrd) = compute_baud_divisors(clock_hz, baud);
        InitValues {
            ibrd,
            fbrd,
            lcrh: LCRH_8N1_FIFO,
            ctl: CTL_ENABLE_TXRX,
        }
    }

    /// Initialize the UART: program baud rate, set 8N1, enable UART.
    pub fn init(&self, baud: u32, clock_hz: u32) {
        let vals = Self::compute_init_values(baud, clock_hz);
        // Disable UART while configuring.
        self.write_reg(CTL, 0);
        self.write_reg(IBRD, vals.ibrd as u32);
        self.write_reg(FBRD, vals.fbrd as u32);
        self.write_reg(LCRH, vals.lcrh);
        self.write_reg(CTL, vals.ctl);
    }

    /// Returns `true` if the transmit FIFO is full.
    pub fn is_tx_full(&self) -> bool {
        let fr = self.read_reg(FR);
        fr & FR_TXFF != 0
    }

    /// Returns `true` if the receive FIFO is empty.
    pub fn is_rx_empty(&self) -> bool {
        let fr = self.read_reg(FR);
        fr & FR_RXFE != 0
    }

    /// Returns `true` if the UART is busy transmitting.
    pub fn is_busy(&self) -> bool {
        let fr = self.read_reg(FR);
        fr & FR_BUSY != 0
    }

    /// Write a byte to the UART data register (spins while TX FIFO full).
    pub fn write_byte(&self, byte: u8) {
        while self.is_tx_full() {}
        self.write_reg(DR, byte as u32);
    }

    /// Read a byte from DR, or `None` if RX FIFO is empty.
    pub fn read_byte(&self) -> Option<u8> {
        if self.is_rx_empty() {
            return None;
        }
        let val = self.read_reg(DR);
        Some(val as u8)
    }

    /// Interrupt mask value for RX + RX timeout interrupts.
    pub fn rx_interrupt_mask() -> u32 {
        IM_RXIM | IM_RTIM
    }

    /// Enable receive and receive-timeout interrupts.
    pub fn enable_rx_interrupt(&self) {
        let mask = Self::rx_interrupt_mask();
        let im = self.read_reg(IM);
        self.write_reg(IM, im | mask);
    }

    /// Disable receive and receive-timeout interrupts.
    pub fn disable_rx_interrupt(&self) {
        let mask = Self::rx_interrupt_mask();
        let im = self.read_reg(IM);
        self.write_reg(IM, im & !mask);
    }

    /// Clear pending receive and receive-timeout interrupts.
    pub fn clear_rx_interrupt(&self) {
        let clear_bits = ICR_RXIC | ICR_RTIC;
        // ICR is write-1-to-clear, so writing these bits clears them.
        self.write_reg(ICR, clear_bits);
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
    fn uart_regs_second_instance() {
        let uart = UartRegs::new(0x4000_D000);
        assert_eq!(uart.base(), 0x4000_D000);
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

    // -- init() value computation -------------------------------------------

    #[test]
    fn init_values_115200_at_12mhz() {
        let vals = UartRegs::compute_init_values(115_200, 12_000_000);
        assert_eq!(vals.ibrd, 6);
        assert_eq!(vals.fbrd, 33);
        assert_eq!(vals.lcrh, LCRH_8N1_FIFO);
        assert_eq!(vals.ctl, CTL_ENABLE_TXRX);
    }

    #[test]
    fn init_values_9600_at_12mhz() {
        let vals = UartRegs::compute_init_values(9600, 12_000_000);
        assert_eq!(vals.ibrd, 78);
        assert_eq!(vals.fbrd, 8);
    }

    #[test]
    fn init_lcrh_is_8n1_with_fifo() {
        // WLEN = 0b11 (bits 6:5) and FEN (bit 4).
        assert_eq!(LCRH_8N1_FIFO, (0b11 << 5) | (1 << 4));
        assert_eq!(LCRH_8N1_FIFO, 0x70);
    }

    #[test]
    fn init_ctl_has_uarten_txe_rxe() {
        assert_ne!(CTL_ENABLE_TXRX & CTL_UARTEN, 0);
        assert_ne!(CTL_ENABLE_TXRX & CTL_TXE, 0);
        assert_ne!(CTL_ENABLE_TXRX & CTL_RXE, 0);
        // Should be exactly those three bits.
        assert_eq!(CTL_ENABLE_TXRX, 0x301);
    }

    // -- Interrupt mask bit logic -------------------------------------------

    #[test]
    fn rx_interrupt_mask_has_rxim_and_rtim() {
        let mask = UartRegs::rx_interrupt_mask();
        assert_ne!(mask & IM_RXIM, 0, "RXIM bit must be set");
        assert_ne!(mask & IM_RTIM, 0, "RTIM bit must be set");
        assert_eq!(mask, IM_RXIM | IM_RTIM);
    }

    #[test]
    fn rx_interrupt_mask_bits_correct() {
        assert_eq!(IM_RXIM, 1 << 4); // bit 4
        assert_eq!(IM_RTIM, 1 << 6); // bit 6
    }

    #[test]
    fn icr_clear_bits_correct() {
        assert_eq!(ICR_RXIC, 1 << 4); // bit 4
        assert_eq!(ICR_RTIC, 1 << 6); // bit 6
                                      // Combined clear value.
        assert_eq!(ICR_RXIC | ICR_RTIC, 0x50);
    }

    // -- Mock-backed method tests ---------------------------------------------

    #[test]
    fn is_tx_full_when_txff_set() {
        let uart = UartRegs::new(0x4000_C000);
        // FR register defaults to 0 → TX not full.
        assert!(!uart.is_tx_full());
        // Set TXFF bit in the FR mock register.
        uart.write(FR, FR_TXFF);
        assert!(uart.is_tx_full());
    }

    #[test]
    fn is_rx_empty_when_rxfe_set() {
        let uart = UartRegs::new(0x4000_C000);
        // FR = 0 → RX not empty.
        assert!(!uart.is_rx_empty());
        // Set RXFE bit.
        uart.write(FR, FR_RXFE);
        assert!(uart.is_rx_empty());
    }

    #[test]
    fn is_busy_when_busy_set() {
        let uart = UartRegs::new(0x4000_C000);
        assert!(!uart.is_busy());
        uart.write(FR, FR_BUSY);
        assert!(uart.is_busy());
    }

    #[test]
    fn write_byte_stores_to_dr() {
        let uart = UartRegs::new(0x4000_C000);
        // FR = 0 → TX not full, so write_byte should not spin.
        uart.write_byte(0x42);
        assert_eq!(uart.read(DR), 0x42);
    }

    #[test]
    fn read_byte_returns_none_when_rx_empty() {
        let uart = UartRegs::new(0x4000_C000);
        // Set RXFE so the FIFO looks empty.
        uart.write(FR, FR_RXFE);
        assert_eq!(uart.read_byte(), None);
    }

    #[test]
    fn read_byte_returns_data_when_rx_has_data() {
        let uart = UartRegs::new(0x4000_C000);
        // FR = 0 → RXFE clear, so FIFO is non-empty.
        uart.write(DR, 0xAB);
        assert_eq!(uart.read_byte(), Some(0xAB));
    }

    #[test]
    fn init_programs_registers() {
        let uart = UartRegs::new(0x4000_C000);
        uart.init(115_200, 12_000_000);
        assert_eq!(uart.read(IBRD), 6);
        assert_eq!(uart.read(FBRD), 33);
        assert_eq!(uart.read(LCRH), LCRH_8N1_FIFO);
        assert_eq!(uart.read(CTL), CTL_ENABLE_TXRX);
    }

    #[test]
    fn enable_rx_interrupt_sets_bits() {
        let uart = UartRegs::new(0x4000_C000);
        // IM starts at 0.
        uart.enable_rx_interrupt();
        let im = uart.read(IM);
        assert_ne!(im & IM_RXIM, 0);
        assert_ne!(im & IM_RTIM, 0);
    }

    #[test]
    fn disable_rx_interrupt_clears_bits() {
        let uart = UartRegs::new(0x4000_C000);
        // Pre-set the interrupt bits.
        uart.write(IM, IM_RXIM | IM_RTIM);
        uart.disable_rx_interrupt();
        let im = uart.read(IM);
        assert_eq!(im & IM_RXIM, 0);
        assert_eq!(im & IM_RTIM, 0);
    }

    #[test]
    fn clear_rx_interrupt_writes_icr() {
        let uart = UartRegs::new(0x4000_C000);
        uart.clear_rx_interrupt();
        let icr = uart.read(ICR);
        assert_ne!(icr & ICR_RXIC, 0);
        assert_ne!(icr & ICR_RTIC, 0);
    }
}

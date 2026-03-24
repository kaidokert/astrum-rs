//! Fault register reading and fault detail capture for MemManage / BusFault / UsageFault.

// CFSR bit constants (ARMv7-M Architecture Reference Manual, B3.2.15)

// MMFSR — CFSR bits [7:0]
pub const CFSR_IACCVIOL: u32 = 1 << 0;
pub const CFSR_DACCVIOL: u32 = 1 << 1;
pub const CFSR_MUNSTKERR: u32 = 1 << 3;
pub const CFSR_MSTKERR: u32 = 1 << 4;
pub const CFSR_MLSPERR: u32 = 1 << 5;
pub const CFSR_MMARVALID: u32 = 1 << 7;

// BFSR — CFSR bits [15:8]
pub const CFSR_IBUSERR: u32 = 1 << 8;
pub const CFSR_PRECISERR: u32 = 1 << 9;
pub const CFSR_IMPRECISERR: u32 = 1 << 10;
pub const CFSR_UNSTKERR: u32 = 1 << 11;
pub const CFSR_STKERR: u32 = 1 << 12;
pub const CFSR_LSPERR: u32 = 1 << 13;
pub const CFSR_BFARVALID: u32 = 1 << 15;

// UFSR — CFSR bits [31:16]
pub const CFSR_UNDEFINSTR: u32 = 1 << 16;
pub const CFSR_INVSTATE: u32 = 1 << 17;
pub const CFSR_INVPC: u32 = 1 << 18;
pub const CFSR_NOCP: u32 = 1 << 19;
pub const CFSR_UNALIGNED: u32 = 1 << 24;
pub const CFSR_DIVBYZERO: u32 = 1 << 25;

// Sub-register masks
pub const CFSR_MMFSR_MASK: u32 = 0x0000_00FF;
pub const CFSR_BFSR_MASK: u32 = 0x0000_FF00;
pub const CFSR_UFSR_MASK: u32 = 0xFFFF_0000;

// EXC_RETURN bit 2: 1 = PSP, 0 = MSP.
const EXC_RETURN_SPSEL_BIT: u32 = 1 << 2;
// Stacked PC offset in hardware exception frame (r0..xpsr): PC at word 6 = 0x18.
const STACKED_PC_OFFSET: u32 = 0x18;

/// Snapshot of fault state captured from an exception handler.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaultDetails {
    pub partition_id: u8,
    pub cfsr: u32,
    pub mmfar: u32,
    pub faulting_pc: u32,
}

impl FaultDetails {
    pub const fn new(partition_id: u8, cfsr: u32, mmfar: u32, faulting_pc: u32) -> Self {
        Self {
            partition_id,
            cfsr,
            mmfar,
            faulting_pc,
        }
    }

    pub const fn is_daccviol(&self) -> bool {
        self.cfsr & CFSR_DACCVIOL != 0
    }
    pub const fn is_iaccviol(&self) -> bool {
        self.cfsr & CFSR_IACCVIOL != 0
    }
    pub const fn is_mmarvalid(&self) -> bool {
        self.cfsr & CFSR_MMARVALID != 0
    }
    pub const fn is_munstkerr(&self) -> bool {
        self.cfsr & CFSR_MUNSTKERR != 0
    }
    pub const fn is_mstkerr(&self) -> bool {
        self.cfsr & CFSR_MSTKERR != 0
    }
    pub const fn is_preciserr(&self) -> bool {
        self.cfsr & CFSR_PRECISERR != 0
    }
    pub const fn is_bfarvalid(&self) -> bool {
        self.cfsr & CFSR_BFARVALID != 0
    }
    pub const fn is_undefinstr(&self) -> bool {
        self.cfsr & CFSR_UNDEFINSTR != 0
    }
    pub const fn is_divbyzero(&self) -> bool {
        self.cfsr & CFSR_DIVBYZERO != 0
    }

    /// Extract the MMFSR byte (bits [7:0]).
    pub const fn mmfsr(&self) -> u8 {
        (self.cfsr & CFSR_MMFSR_MASK) as u8
    }
    /// Extract the BFSR byte (bits [15:8]).
    pub const fn bfsr(&self) -> u8 {
        ((self.cfsr & CFSR_BFSR_MASK) >> 8) as u8
    }
    /// Extract the UFSR halfword (bits [31:16]).
    pub const fn ufsr(&self) -> u16 {
        ((self.cfsr & CFSR_UFSR_MASK) >> 16) as u16
    }
}

/// Read the full 32-bit CFSR from the SCB.
///
/// # Safety
/// Must be called from privileged mode (e.g., an exception handler).
pub unsafe fn read_cfsr() -> u32 {
    // SAFETY: Caller guarantees privileged context.
    let scb = &*cortex_m::peripheral::SCB::PTR;
    scb.cfsr.read()
}

/// Read the MMFAR (MemManage Fault Address Register) from the SCB.
///
/// # Safety
/// Must be called from privileged mode (e.g., an exception handler).
pub unsafe fn read_mmfar() -> u32 {
    // SAFETY: Caller guarantees privileged context.
    let scb = &*cortex_m::peripheral::SCB::PTR;
    scb.mmfar.read()
}

/// Clear the MMFSR bits (lower byte of CFSR) by writing ones to clear.
///
/// # Safety
/// Must be called from privileged mode (e.g., an exception handler).
pub unsafe fn clear_mmfsr() {
    // SAFETY: Caller guarantees privileged context. CFSR is write-1-to-clear.
    let scb = &*cortex_m::peripheral::SCB::PTR;
    scb.cfsr.write(CFSR_MMFSR_MASK);
}

/// Whether an EXC_RETURN value indicates the thread was using the PSP.
pub const fn exc_return_uses_psp(exc_return: u32) -> bool {
    exc_return & EXC_RETURN_SPSEL_BIT != 0
}

/// Extract the faulting PC from the exception stack frame via PSP.
///
/// Returns `None` if EXC_RETURN indicates MSP (kernel fault).
///
/// # Safety
/// - Must be called from an exception handler in privileged mode.
/// - `psp` must point to a valid, aligned exception frame stacked by hardware.
pub unsafe fn faulting_pc_from_psp(exc_return: u32, psp: u32) -> Option<u32> {
    if !exc_return_uses_psp(exc_return) {
        return None;
    }
    // SAFETY: Caller guarantees PSP points to a valid exception frame.
    let pc_ptr = (psp + STACKED_PC_OFFSET) as *const u32;
    Some(core::ptr::read_volatile(pc_ptr))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fault_details_construction_and_traits() {
        let fd = FaultDetails::new(3, 0x0000_0082, 0x2000_1000, 0x0800_0ABC);
        assert_eq!(fd.partition_id, 3);
        assert_eq!(fd.cfsr, 0x0000_0082);
        assert_eq!(fd.mmfar, 0x2000_1000);
        assert_eq!(fd.faulting_pc, 0x0800_0ABC);
        let copy = fd;
        assert_eq!(fd, copy);
        assert_eq!(fd, fd.clone());
        assert!(format!("{:?}", fd).contains("FaultDetails"));
    }

    #[test]
    fn cfsr_bit_parsing_helpers() {
        let fd = FaultDetails::new(0, CFSR_DACCVIOL | CFSR_MMARVALID, 0x2000_F000, 0x100);
        assert!(fd.is_daccviol() && fd.is_mmarvalid());
        assert!(!fd.is_iaccviol() && !fd.is_munstkerr() && !fd.is_mstkerr());
        assert!(!fd.is_preciserr() && !fd.is_undefinstr() && !fd.is_divbyzero());

        let fd = FaultDetails::new(1, CFSR_IACCVIOL, 0, 0x200);
        assert!(fd.is_iaccviol() && !fd.is_daccviol() && !fd.is_mmarvalid());

        let fd = FaultDetails::new(0, CFSR_PRECISERR | CFSR_BFARVALID, 0x4000_0000, 0x300);
        assert!(fd.is_preciserr() && fd.is_bfarvalid() && !fd.is_daccviol());

        let fd = FaultDetails::new(2, CFSR_UNDEFINSTR | CFSR_DIVBYZERO, 0, 0x400);
        assert!(fd.is_undefinstr() && fd.is_divbyzero() && !fd.is_iaccviol());

        let fd = FaultDetails::new(0, CFSR_MSTKERR | CFSR_MUNSTKERR, 0, 0);
        assert!(fd.is_mstkerr() && fd.is_munstkerr() && !fd.is_daccviol());
    }

    #[test]
    fn subregister_extraction() {
        let fd = FaultDetails::new(0, CFSR_DACCVIOL | CFSR_MMARVALID | CFSR_PRECISERR, 0, 0);
        assert_eq!(fd.mmfsr(), 0x82);
        assert_eq!(fd.bfsr(), 0x02);
        assert_eq!(fd.ufsr(), 0x0000);

        let fd = FaultDetails::new(0, CFSR_IBUSERR | CFSR_STKERR, 0, 0);
        assert_eq!(fd.bfsr(), 0x11);
        assert_eq!(fd.mmfsr(), 0x00);

        let fd = FaultDetails::new(0, CFSR_INVSTATE | CFSR_UNALIGNED, 0, 0);
        assert_eq!(fd.ufsr(), 0x0102);

        let fd = FaultDetails::new(0, CFSR_IACCVIOL | CFSR_IMPRECISERR | CFSR_NOCP, 0, 0);
        assert_eq!(fd.mmfsr(), 0x01);
        assert_eq!(fd.bfsr(), 0x04);
        assert_eq!(fd.ufsr(), 0x0008);
    }

    #[test]
    fn cfsr_constants_and_masks() {
        assert_eq!(CFSR_IACCVIOL, 1 << 0);
        assert_eq!(CFSR_DACCVIOL, 1 << 1);
        assert_eq!(CFSR_MMARVALID, 1 << 7);
        assert_eq!(CFSR_IBUSERR, 1 << 8);
        assert_eq!(CFSR_BFARVALID, 1 << 15);
        assert_eq!(CFSR_UNDEFINSTR, 1 << 16);
        assert_eq!(CFSR_DIVBYZERO, 1 << 25);
        // Masks are non-overlapping and cover all 32 bits
        assert_eq!(
            CFSR_MMFSR_MASK | CFSR_BFSR_MASK | CFSR_UFSR_MASK,
            0xFFFF_FFFF
        );
        assert_eq!(CFSR_MMFSR_MASK & CFSR_BFSR_MASK, 0);
    }

    #[test]
    fn exc_return_psp_detection() {
        assert!(exc_return_uses_psp(0xFFFF_FFFD));
        assert!(exc_return_uses_psp(0xFFFF_FFED));
        assert!(!exc_return_uses_psp(0xFFFF_FFF9));
        assert!(!exc_return_uses_psp(0xFFFF_FFF1));
    }

    #[test]
    fn faulting_pc_from_exception_frame() {
        let frame: [u32; 8] = [0, 0, 0, 0, 0, 0, 0x0800_1234, 0x0100_0000];
        // SAFETY: valid local array, offset 6 is in bounds.
        let pc = unsafe { core::ptr::read_volatile(frame.as_ptr().add(6)) };
        assert_eq!(pc, 0x0800_1234);
    }

    #[test]
    fn faulting_pc_returns_none_for_msp() {
        // SAFETY: Never dereferences — returns None for MSP EXC_RETURN.
        let pc = unsafe { faulting_pc_from_psp(0xFFFF_FFF9, 0x2000_0000) };
        assert_eq!(pc, None);
    }

    #[cfg(target_pointer_width = "32")]
    #[test]
    fn faulting_pc_from_psp_on_target() {
        let frame: [u32; 8] = [0, 0, 0, 0, 0, 0, 0x0800_ABCD, 0x0100_0000];
        let psp = frame.as_ptr() as u32;
        // SAFETY: 32-bit target, pointer fits in u32; frame is valid.
        let pc = unsafe { faulting_pc_from_psp(0xFFFF_FFFD, psp) };
        assert_eq!(pc, Some(0x0800_ABCD));
    }
}

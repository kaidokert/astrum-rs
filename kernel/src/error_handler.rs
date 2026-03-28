//! Fault classification and error status types for post-fault diagnostics.

use crate::fault::{
    FaultDetails, CFSR_BFSR_MASK, CFSR_MMFSR_MASK, CFSR_MSTKERR, CFSR_MUNSTKERR, CFSR_UFSR_MASK,
};

/// Classification of the fault that caused a partition to be stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultKind {
    /// Memory-management fault (MPU violation, instruction access violation).
    MemManage,
    /// Bus fault (precise/imprecise data bus error).
    BusFault,
    /// Usage fault (undefined instruction, divide-by-zero, invalid state).
    UsageFault,
    /// Stack overflow detected via MemManage stacking error.
    StackOverflow,
    /// Software-level deadline miss (no hardware fault bits set).
    DeadlineMiss,
}

impl FaultKind {
    /// Convert to a raw `u32` discriminant for ABI packing.
    pub const fn as_u32(self) -> u32 {
        match self {
            Self::MemManage => 0,
            Self::BusFault => 1,
            Self::UsageFault => 2,
            Self::StackOverflow => 3,
            Self::DeadlineMiss => 4,
        }
    }

    /// Convert from a raw `u32` discriminant.
    pub const fn from_u32(v: u32) -> Option<Self> {
        match v {
            0 => Some(Self::MemManage),
            1 => Some(Self::BusFault),
            2 => Some(Self::UsageFault),
            3 => Some(Self::StackOverflow),
            4 => Some(Self::DeadlineMiss),
            _ => None,
        }
    }
}

/// Snapshot of a partition fault for post-mortem inspection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ErrorStatus {
    kind: FaultKind,
    failed_partition: u8,
    faulting_addr: u32,
    cfsr: u32,
    faulting_pc: u32,
}

impl ErrorStatus {
    /// Construct a new `ErrorStatus` with all fields specified.
    pub const fn new(
        kind: FaultKind,
        failed_partition: u8,
        faulting_addr: u32,
        cfsr: u32,
        faulting_pc: u32,
    ) -> Self {
        Self {
            kind,
            failed_partition,
            faulting_addr,
            cfsr,
            faulting_pc,
        }
    }

    pub const fn kind(&self) -> FaultKind {
        self.kind
    }

    pub const fn failed_partition(&self) -> u8 {
        self.failed_partition
    }

    pub const fn faulting_addr(&self) -> u32 {
        self.faulting_addr
    }

    pub const fn cfsr(&self) -> u32 {
        self.cfsr
    }

    pub const fn faulting_pc(&self) -> u32 {
        self.faulting_pc
    }
}

/// Determine [`FaultKind`] from raw CFSR bits.
///
/// Priority: StackOverflow (MSTKERR/MUNSTKERR) > MemManage > BusFault >
/// UsageFault > DeadlineMiss (no hardware bits).
const fn classify_cfsr(cfsr: u32) -> FaultKind {
    // Stack-overflow is a MemManage sub-case we promote to its own variant.
    if cfsr & (CFSR_MSTKERR | CFSR_MUNSTKERR) != 0 {
        return FaultKind::StackOverflow;
    }
    if cfsr & CFSR_MMFSR_MASK != 0 {
        return FaultKind::MemManage;
    }
    if cfsr & CFSR_BFSR_MASK != 0 {
        return FaultKind::BusFault;
    }
    if cfsr & CFSR_UFSR_MASK != 0 {
        return FaultKind::UsageFault;
    }
    FaultKind::DeadlineMiss
}

impl From<FaultDetails> for ErrorStatus {
    fn from(fd: FaultDetails) -> Self {
        Self {
            kind: classify_cfsr(fd.cfsr),
            failed_partition: fd.partition_id,
            faulting_addr: fd.mmfar,
            cfsr: fd.cfsr,
            faulting_pc: fd.faulting_pc,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fault::*;

    // ── FaultKind traits ───────────────────────────────────────────

    #[test]
    fn fault_kind_debug_clone_copy_eq() {
        let k = FaultKind::MemManage;
        let k2 = k; // Copy
        assert_eq!(k, k2);
        assert_eq!(k, k.clone());
        assert!(format!("{:?}", k).contains("MemManage"));
    }

    #[test]
    fn fault_kind_all_variants_distinct() {
        let variants = [
            FaultKind::MemManage,
            FaultKind::BusFault,
            FaultKind::UsageFault,
            FaultKind::StackOverflow,
            FaultKind::DeadlineMiss,
        ];
        for (i, a) in variants.iter().enumerate() {
            for (j, b) in variants.iter().enumerate() {
                assert_eq!(i == j, a == b);
            }
        }
    }

    // ── ErrorStatus construction & accessors ───────────────────────

    #[test]
    fn error_status_new_and_accessors() {
        let es = ErrorStatus::new(FaultKind::BusFault, 2, 0x4000_0000, 0x0200, 0x0800_1234);
        assert_eq!(es.kind(), FaultKind::BusFault);
        assert_eq!(es.failed_partition(), 2);
        assert_eq!(es.faulting_addr(), 0x4000_0000);
        assert_eq!(es.cfsr(), 0x0200);
        assert_eq!(es.faulting_pc(), 0x0800_1234);
    }

    #[test]
    fn error_status_traits() {
        let es = ErrorStatus::new(FaultKind::MemManage, 0, 0, 0, 0);
        let es2 = es; // Copy
        assert_eq!(es, es2);
        assert_eq!(es, es.clone());
        assert!(format!("{:?}", es).contains("ErrorStatus"));
    }

    #[test]
    fn error_status_const_new() {
        // Verify const-evaluable construction.
        const ES: ErrorStatus = ErrorStatus::new(FaultKind::DeadlineMiss, 1, 0, 0, 0x0800_0000);
        assert_eq!(ES.kind(), FaultKind::DeadlineMiss);
        assert_eq!(ES.failed_partition(), 1);
        assert_eq!(ES.faulting_pc(), 0x0800_0000);
    }

    // ── classify_cfsr ──────────────────────────────────────────────

    #[test]
    fn classify_stack_overflow_variants() {
        assert_eq!(classify_cfsr(CFSR_MSTKERR), FaultKind::StackOverflow);
        assert_eq!(classify_cfsr(CFSR_MUNSTKERR), FaultKind::StackOverflow);
        assert_eq!(
            classify_cfsr(CFSR_MSTKERR | CFSR_MUNSTKERR),
            FaultKind::StackOverflow
        );
        // StackOverflow takes priority over other MemManage bits.
        assert_eq!(
            classify_cfsr(CFSR_MSTKERR | CFSR_DACCVIOL),
            FaultKind::StackOverflow
        );
    }

    #[test]
    fn classify_memmanage_variants() {
        assert_eq!(classify_cfsr(CFSR_DACCVIOL), FaultKind::MemManage);
        assert_eq!(classify_cfsr(CFSR_IACCVIOL), FaultKind::MemManage);
        assert_eq!(
            classify_cfsr(CFSR_DACCVIOL | CFSR_MMARVALID),
            FaultKind::MemManage
        );
    }

    #[test]
    fn classify_busfault_variants() {
        assert_eq!(classify_cfsr(CFSR_PRECISERR), FaultKind::BusFault);
        assert_eq!(classify_cfsr(CFSR_IBUSERR), FaultKind::BusFault);
        assert_eq!(classify_cfsr(CFSR_IMPRECISERR), FaultKind::BusFault);
    }

    #[test]
    fn classify_usagefault_variants() {
        assert_eq!(classify_cfsr(CFSR_UNDEFINSTR), FaultKind::UsageFault);
        assert_eq!(classify_cfsr(CFSR_DIVBYZERO), FaultKind::UsageFault);
        assert_eq!(classify_cfsr(CFSR_INVSTATE), FaultKind::UsageFault);
    }

    #[test]
    fn classify_zero_cfsr_is_deadline_miss() {
        assert_eq!(classify_cfsr(0), FaultKind::DeadlineMiss);
    }

    // ── From<FaultDetails> ─────────────────────────────────────────

    #[test]
    fn from_fault_details_memmanage() {
        let fd = FaultDetails::new(3, CFSR_DACCVIOL | CFSR_MMARVALID, 0x2000_1000, 0x0800_0ABC);
        let es: ErrorStatus = fd.into();
        assert_eq!(es.kind(), FaultKind::MemManage);
        assert_eq!(es.failed_partition(), 3);
        assert_eq!(es.faulting_addr(), 0x2000_1000);
        assert_eq!(es.cfsr(), CFSR_DACCVIOL | CFSR_MMARVALID);
        assert_eq!(es.faulting_pc(), 0x0800_0ABC);
    }

    #[test]
    fn from_fault_details_busfault() {
        let fd = FaultDetails::new(1, CFSR_PRECISERR | CFSR_BFARVALID, 0x4000_0000, 0x0800_1000);
        let es = ErrorStatus::from(fd);
        assert_eq!(es.kind(), FaultKind::BusFault);
        assert_eq!(es.failed_partition(), 1);
        assert_eq!(es.faulting_addr(), 0x4000_0000);
    }

    #[test]
    fn from_fault_details_usagefault() {
        let fd = FaultDetails::new(2, CFSR_UNDEFINSTR, 0, 0x0800_2000);
        let es = ErrorStatus::from(fd);
        assert_eq!(es.kind(), FaultKind::UsageFault);
        assert_eq!(es.faulting_pc(), 0x0800_2000);
    }

    #[test]
    fn from_fault_details_stack_overflow() {
        let fd = FaultDetails::new(0, CFSR_MSTKERR, 0, 0x0800_3000);
        let es = ErrorStatus::from(fd);
        assert_eq!(es.kind(), FaultKind::StackOverflow);
        assert_eq!(es.failed_partition(), 0);
    }

    #[test]
    fn from_fault_details_deadline_miss() {
        let fd = FaultDetails::new(1, 0, 0, 0);
        let es = ErrorStatus::from(fd);
        assert_eq!(es.kind(), FaultKind::DeadlineMiss);
    }

    #[test]
    fn from_fault_details_preserves_all_fields() {
        let fd = FaultDetails::new(7, CFSR_IBUSERR, 0xDEAD_BEEF, 0xCAFE_BABE);
        let es = ErrorStatus::from(fd);
        assert_eq!(es.failed_partition(), 7);
        assert_eq!(es.faulting_addr(), 0xDEAD_BEEF);
        assert_eq!(es.cfsr(), CFSR_IBUSERR);
        assert_eq!(es.faulting_pc(), 0xCAFE_BABE);
    }
}

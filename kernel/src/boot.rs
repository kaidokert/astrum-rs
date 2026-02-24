//! Kernel boot initialization.

#[cfg(not(test))]
use crate::{
    blackboard::BlackboardPool,
    config::{CoreOps, KernelConfig, MsgOps, PortsOps, SyncOps},
    message::MessagePool,
    mutex::MutexPool,
    partition::PartitionTable,
    queuing::QueuingPortPool,
    sampling::SamplingPortPool,
    scheduler::ScheduleTable,
    semaphore::SemaphorePool,
};

/// Minimum MPU region size (32 bytes for ARMv7-M and ARMv8-M).
pub const MPU_MIN_REGION_SIZE: u32 = 32;

/// Boot initialization errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootError {
    /// Stack frame init failed.
    StackInitFailed { partition_index: usize },
    /// No partition ready at boot.
    NoReadyPartition,
    /// Stack buffer alignment error (MPU requires base aligned to size).
    StackAlignmentError {
        partition_index: usize,
        base: u32,
        size: u32,
    },
    /// Stack size is invalid for MPU (must be power-of-two >= 32 bytes).
    StackSizeError { partition_index: usize, size: u32 },
    /// Stack size arithmetic overflow.
    StackSizeOverflow { partition_index: usize },
    /// Kernel storage buffer misaligned (requires KERNEL_ALIGNMENT-byte alignment).
    StorageMisaligned {
        /// Actual address of the storage buffer.
        address: u32,
        /// Required alignment in bytes.
        required: u32,
    },
    /// Failed to update PCB stack region fields.
    StackRegionError { partition_index: usize },
}

impl core::fmt::Display for BootError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::StackInitFailed { partition_index } => {
                write!(f, "stack init failed: partition {partition_index}")
            }
            Self::NoReadyPartition => write!(f, "no partition ready at boot"),
            Self::StackAlignmentError {
                partition_index,
                base,
                size,
            } => {
                write!(
                    f,
                    "stack buffer misaligned for MPU: partition {partition_index}, base=0x{base:08x}, size={size}"
                )
            }
            Self::StackSizeError {
                partition_index,
                size,
            } => {
                write!(
                    f,
                    "invalid stack size for MPU: partition {partition_index}, size={size} (must be power-of-two >= 32)"
                )
            }
            Self::StackSizeOverflow { partition_index } => {
                write!(
                    f,
                    "stack size arithmetic overflow: partition {partition_index}"
                )
            }
            Self::StorageMisaligned { address, required } => {
                write!(
                    f,
                    "kernel storage misaligned: address=0x{address:08x}, required={required}-byte alignment"
                )
            }
            Self::StackRegionError { partition_index } => {
                write!(
                    f,
                    "failed to update PCB stack region: partition {partition_index}"
                )
            }
        }
    }
}

/// Uninhabited type for diverging functions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Never {}

/// Check if stack buffer size is valid for MPU.
///
/// For ARMv7-M MPU, region size must be a power of two and at least 32 bytes.
/// For ARMv8-M MPU, minimum region size is also 32 bytes.
///
/// Returns `Ok(())` if the size is valid, or `Err(BootError::StackSizeError)`.
#[inline]
pub fn check_stack_mpu_size(partition_index: usize, size: u32) -> Result<(), BootError> {
    // MPU requires size to be a power of two and at least MPU_MIN_REGION_SIZE.
    if size < MPU_MIN_REGION_SIZE || !size.is_power_of_two() {
        return Err(BootError::StackSizeError {
            partition_index,
            size,
        });
    }
    Ok(())
}

/// Check if stack buffer base address is properly aligned for MPU.
///
/// For ARMv7-M MPU, the region base address must be aligned to the region size.
/// For ARMv8-M MPU, minimum alignment is 32 bytes.
///
/// **Precondition**: `size` must be a valid MPU size (power-of-two >= 32).
/// Call `check_stack_mpu_size` first to validate size.
///
/// Returns `Ok(())` if the base address is properly aligned, or
/// `Err(BootError::StackAlignmentError)` with the partition index, base, and size.
#[inline]
pub fn check_stack_mpu_alignment(
    partition_index: usize,
    base: u32,
    size: u32,
) -> Result<(), BootError> {
    // SAFETY: This bitwise check (base & (size - 1) == 0) is only valid when size
    // is a power of two. Caller must validate size with check_stack_mpu_size first.
    // For v8-M, minimum alignment is 32 bytes; this check is conservative for v7-M
    // which requires alignment to the full region size.
    // TODO: v8-M only needs 32-byte minimum alignment; this check is conservative for v7-M.
    debug_assert!(
        size.is_power_of_two() && size >= MPU_MIN_REGION_SIZE,
        "check_stack_mpu_alignment requires valid MPU size"
    );
    if base & (size - 1) != 0 {
        return Err(BootError::StackAlignmentError {
            partition_index,
            base,
            size,
        });
    }
    Ok(())
}

/// Check if the kernel storage buffer is properly aligned.
///
/// Returns `Ok(())` if aligned to [`crate::state::KERNEL_ALIGNMENT`], or
/// `Err(BootError::StorageMisaligned)` with address and required alignment.
#[inline]
pub fn check_storage_alignment(address: u32, required: u32) -> Result<(), BootError> {
    if address & (required - 1) != 0 {
        return Err(BootError::StorageMisaligned { address, required });
    }
    Ok(())
}

/// AAPCS requires 8-byte stack pointer alignment at public interfaces.
pub const AAPCS_STACK_ALIGNMENT: u32 = 8;

/// Check if a stack pointer is 8-byte aligned per AAPCS.
#[inline]
pub fn is_stack_aapcs_aligned(sp: u32) -> bool {
    sp & (AAPCS_STACK_ALIGNMENT - 1) == 0
}

/// Initialize stacks, priorities, start schedule, enable SysTick, enter idle loop.
#[cfg(not(test))]
pub fn boot<C: KernelConfig>(
    partitions: &[(extern "C" fn() -> !, u32)],
    peripherals: &mut cortex_m::Peripherals,
) -> Result<Never, BootError>
where
    [(); C::N]:,
    [(); C::SCHED]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BP]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::BZ]:,
    #[cfg(feature = "dynamic-mpu")]
    [(); C::DR]:,
    C::Core:
        CoreOps<PartTable = PartitionTable<{ C::N }>, SchedTable = ScheduleTable<{ C::SCHED }>>,
    C::Sync: SyncOps<
        SemPool = SemaphorePool<{ C::S }, { C::SW }>,
        MutPool = MutexPool<{ C::MS }, { C::MW }>,
    >,
    C::Msg: MsgOps<
        MsgPool = MessagePool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
        QueuingPool = QueuingPortPool<{ C::QS }, { C::QD }, { C::QM }, { C::QW }>,
    >,
    C::Ports: PortsOps<
        SamplingPool = SamplingPortPool<{ C::SP }, { C::SM }>,
        BlackboardPool = BlackboardPool<{ C::BS }, { C::BM }, { C::BW }>,
    >,
{
    use core::ptr::addr_of;
    use cortex_m::peripheral::{scb::SystemHandler, syst::SystClkSource, SCB};

    #[cfg(klog_backend = "rtt")]
    rtt_target::rtt_init_print!();

    // Verify kernel storage alignment before proceeding.
    let storage_addr = addr_of!(crate::state::UNIFIED_KERNEL_STORAGE) as u32;
    check_storage_alignment(storage_addr, crate::state::KERNEL_ALIGNMENT as u32)?;

    crate::state::with_kernel_mut::<C, _, _>(|k| {
        for (i, &(ep, hint)) in partitions.iter().enumerate() {
            let stk = k
                .core_stack_mut(i)
                .ok_or(BootError::StackInitFailed { partition_index: i })?;
            let base = stk.as_ptr() as u32;
            // Use checked arithmetic for size calculation to avoid overflow.
            let size = stk
                .len()
                .checked_mul(4)
                .and_then(|s| u32::try_from(s).ok())
                .ok_or(BootError::StackSizeOverflow { partition_index: i })?;
            // Verify stack buffer is MPU-compatible before initializing.
            check_stack_mpu_size(i, size)?;
            check_stack_mpu_alignment(i, base, size)?;
            let ix = crate::context::init_stack_frame(stk, ep as *const () as u32, Some(hint))
                .ok_or(BootError::StackInitFailed { partition_index: i })?;
            // Use checked arithmetic for SP calculation.
            let sp = (ix as u32)
                .checked_mul(4)
                .and_then(|offset| base.checked_add(offset))
                .ok_or(BootError::StackSizeOverflow { partition_index: i })?;
            // Validate AAPCS 8-byte alignment; warn but do not fail boot.
            if !is_stack_aapcs_aligned(sp) {
                crate::klog!(
                    "WARN: partition {} stack pointer 0x{:08x} not 8-byte aligned (AAPCS)",
                    i,
                    sp
                );
            }
            k.set_sp(i, sp);
            // Sync PCB stack fields with actual relocated stack memory.
            if !k.fix_stack_region(i, base, size) {
                return Err(BootError::StackRegionError { partition_index: i });
            }
        }
        Ok::<(), BootError>(())
    })?;

    const { crate::config::assert_priority_order::<C>() }
    const { crate::config::assert_systick_reload::<C>() }
    // SAFETY: Called once before scheduler starts; single-core exclusive access to SCB.
    unsafe {
        peripherals
            .SCB
            .set_priority(SystemHandler::SVCall, C::SVCALL_PRIORITY);
        peripherals
            .SCB
            .set_priority(SystemHandler::PendSV, C::PENDSV_PRIORITY);
        peripherals
            .SCB
            .set_priority(SystemHandler::SysTick, C::SYSTICK_PRIORITY);
    }

    let first = crate::state::with_kernel_mut::<C, _, _>(|k| {
        k.start_schedule().inspect(|&pid| k.set_next_partition(pid))
    })
    .ok_or(BootError::NoReadyPartition)?;
    let _ = first;

    // Program MPU for the first scheduled partition before PendSV fires.
    {
        extern "Rust" {
            fn __boot_mpu_init(mpu: &cortex_m::peripheral::MPU);
        }
        // SAFETY: __boot_mpu_init is generated by define_unified_harness! and
        // called exactly once at boot, before SCB::set_pendsv() triggers the
        // first context switch. It programs static MPU regions R0-R3 for the
        // partition selected by start_schedule(). With dynamic-mpu, it also
        // programs dynamic slot 0 (R4).
        unsafe { __boot_mpu_init(&peripherals.MPU) };
    }

    // Select SysTick clock source based on KernelConfig::USE_PROCESSOR_CLOCK.
    // CLKSOURCE = 1 (Core) is the default; BSPs needing the external
    // reference clock set USE_PROCESSOR_CLOCK = false and adjust
    // CORE_CLOCK_HZ to match the external frequency.
    peripherals
        .SYST
        .set_clock_source(if C::USE_PROCESSOR_CLOCK {
            SystClkSource::Core
        } else {
            SystClkSource::External
        });
    peripherals.SYST.set_reload(C::SYSTICK_CYCLES - 1);
    peripherals.SYST.clear_current();
    peripherals.SYST.enable_counter();
    peripherals.SYST.enable_interrupt();
    SCB::set_pendsv();
    loop {
        cortex_m::asm::wfi();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    #[test]
    fn boot_error_construction_and_display() {
        let err = BootError::StackInitFailed { partition_index: 2 };
        assert_eq!(err, BootError::StackInitFailed { partition_index: 2 });
        assert_ne!(err, BootError::NoReadyPartition);
        assert_eq!(std::format!("{err}"), "stack init failed: partition 2");
        assert_eq!(
            std::format!("{}", BootError::NoReadyPartition),
            "no partition ready at boot"
        );
        let (e2, e3) = (err, err);
        assert_eq!(e2, e3);

        // Test StackSizeError display
        let err = BootError::StackSizeError {
            partition_index: 1,
            size: 16,
        };
        assert_eq!(
            std::format!("{err}"),
            "invalid stack size for MPU: partition 1, size=16 (must be power-of-two >= 32)"
        );

        // Test StackSizeOverflow display
        let err = BootError::StackSizeOverflow { partition_index: 3 };
        assert_eq!(
            std::format!("{err}"),
            "stack size arithmetic overflow: partition 3"
        );

        // Test StorageMisaligned display
        let err = BootError::StorageMisaligned {
            address: 0x2000_0200,
            required: 1024,
        };
        assert_eq!(
            std::format!("{err}"),
            "kernel storage misaligned: address=0x20000200, required=1024-byte alignment"
        );

        // Test StackRegionError display
        let err = BootError::StackRegionError { partition_index: 5 };
        assert_eq!(
            std::format!("{err}"),
            "failed to update PCB stack region: partition 5"
        );
    }

    #[test]
    fn check_stack_mpu_size_accepts_valid_sizes() {
        // Minimum size (32 bytes)
        assert!(check_stack_mpu_size(0, 32).is_ok());
        // Common stack sizes
        assert!(check_stack_mpu_size(0, 64).is_ok());
        assert!(check_stack_mpu_size(0, 128).is_ok());
        assert!(check_stack_mpu_size(0, 256).is_ok());
        assert!(check_stack_mpu_size(0, 512).is_ok());
        assert!(check_stack_mpu_size(0, 1024).is_ok());
        assert!(check_stack_mpu_size(0, 2048).is_ok());
        assert!(check_stack_mpu_size(0, 4096).is_ok());
    }

    #[test]
    fn check_stack_mpu_size_rejects_zero() {
        let result = check_stack_mpu_size(0, 0);
        assert_eq!(
            result,
            Err(BootError::StackSizeError {
                partition_index: 0,
                size: 0,
            })
        );
    }

    #[test]
    fn check_stack_mpu_size_rejects_too_small() {
        // Less than minimum (32 bytes)
        let result = check_stack_mpu_size(0, 16);
        assert_eq!(
            result,
            Err(BootError::StackSizeError {
                partition_index: 0,
                size: 16,
            })
        );
        let result = check_stack_mpu_size(1, 4);
        assert_eq!(
            result,
            Err(BootError::StackSizeError {
                partition_index: 1,
                size: 4,
            })
        );
    }

    #[test]
    fn check_stack_mpu_size_rejects_non_power_of_two() {
        // Non-power-of-two sizes
        let result = check_stack_mpu_size(0, 384);
        assert_eq!(
            result,
            Err(BootError::StackSizeError {
                partition_index: 0,
                size: 384,
            })
        );
        let result = check_stack_mpu_size(1, 100);
        assert_eq!(
            result,
            Err(BootError::StackSizeError {
                partition_index: 1,
                size: 100,
            })
        );
        let result = check_stack_mpu_size(2, 1000);
        assert_eq!(
            result,
            Err(BootError::StackSizeError {
                partition_index: 2,
                size: 1000,
            })
        );
    }

    #[test]
    fn check_stack_mpu_alignment_accepts_properly_aligned() {
        // Base aligned to size (power-of-2 alignment for MPU)
        // 256-byte region at 256-byte aligned address
        assert!(check_stack_mpu_alignment(0, 0x2000_0100, 256).is_ok());
        // 1024-byte region at 1024-byte aligned address
        assert!(check_stack_mpu_alignment(0, 0x2000_0400, 1024).is_ok());
        // 512-byte region at 512-byte aligned address
        assert!(check_stack_mpu_alignment(1, 0x2000_0200, 512).is_ok());
        // 32-byte region (minimum for v8-M) at 32-byte aligned address
        assert!(check_stack_mpu_alignment(0, 0x2000_0020, 32).is_ok());
    }

    #[test]
    fn check_stack_mpu_alignment_rejects_misaligned_base() {
        // 1024-byte region but base only 512-byte aligned
        let result = check_stack_mpu_alignment(0, 0x2000_0200, 1024);
        assert_eq!(
            result,
            Err(BootError::StackAlignmentError {
                partition_index: 0,
                base: 0x2000_0200,
                size: 1024,
            })
        );
        // 256-byte region but base only 128-byte aligned
        let result = check_stack_mpu_alignment(1, 0x2000_0080, 256);
        assert_eq!(
            result,
            Err(BootError::StackAlignmentError {
                partition_index: 1,
                base: 0x2000_0080,
                size: 256,
            })
        );
        // 512-byte region but base at odd offset
        let result = check_stack_mpu_alignment(2, 0x2000_0100, 512);
        assert_eq!(
            result,
            Err(BootError::StackAlignmentError {
                partition_index: 2,
                base: 0x2000_0100,
                size: 512,
            })
        );
    }

    #[test]
    fn check_storage_alignment_accepts_aligned_address() {
        // 1024-byte aligned addresses should pass
        assert!(check_storage_alignment(0x2000_0000, 1024).is_ok());
        assert!(check_storage_alignment(0x2000_0400, 1024).is_ok());
        assert!(check_storage_alignment(0x2000_0800, 1024).is_ok());
    }

    #[test]
    fn check_storage_alignment_rejects_misaligned_address() {
        // 512-byte aligned but 1024 required
        let result = check_storage_alignment(0x2000_0200, 1024);
        assert_eq!(
            result,
            Err(BootError::StorageMisaligned {
                address: 0x2000_0200,
                required: 1024,
            })
        );
        // 256-byte aligned but 1024 required
        let result = check_storage_alignment(0x2000_0100, 1024);
        assert_eq!(
            result,
            Err(BootError::StorageMisaligned {
                address: 0x2000_0100,
                required: 1024,
            })
        );
    }

    // TODO: boot() is #[cfg(not(test))] so we cannot directly test that
    // boot() returns Err(StorageMisaligned). This test covers the helper;
    // an integration test on a real target would be needed to verify boot().
    #[test]
    fn check_storage_alignment_returns_misaligned_for_kernel_alignment() {
        use crate::state::KERNEL_ALIGNMENT;
        let required = KERNEL_ALIGNMENT as u32;
        // Address offset by 1 byte from alignment boundary.
        let addr = required + 1;
        let result = check_storage_alignment(addr, required);
        assert_eq!(
            result,
            Err(BootError::StorageMisaligned {
                address: addr,
                required,
            })
        );
        // Address at half the alignment (e.g., 2048 for 4096 alignment).
        let half = required / 2;
        let result = check_storage_alignment(half, required);
        assert_eq!(
            result,
            Err(BootError::StorageMisaligned {
                address: half,
                required,
            })
        );
        // Properly aligned address succeeds.
        assert!(check_storage_alignment(required, required).is_ok());
        assert!(check_storage_alignment(required * 3, required).is_ok());
    }

    #[test]
    fn is_stack_aapcs_aligned_accepts_8byte_aligned() {
        // 8-byte aligned addresses
        assert!(is_stack_aapcs_aligned(0x0000_0000));
        assert!(is_stack_aapcs_aligned(0x0000_0008));
        assert!(is_stack_aapcs_aligned(0x0000_0010));
        assert!(is_stack_aapcs_aligned(0x2000_0000));
        assert!(is_stack_aapcs_aligned(0x2000_0008));
        assert!(is_stack_aapcs_aligned(0x2000_0100));
    }

    #[test]
    fn is_stack_aapcs_aligned_rejects_misaligned() {
        // Not 8-byte aligned (4-byte aligned only)
        assert!(!is_stack_aapcs_aligned(0x0000_0004));
        assert!(!is_stack_aapcs_aligned(0x0000_000C));
        assert!(!is_stack_aapcs_aligned(0x2000_0004));
        assert!(!is_stack_aapcs_aligned(0x2000_000C));
        // Odd addresses
        assert!(!is_stack_aapcs_aligned(0x0000_0001));
        assert!(!is_stack_aapcs_aligned(0x0000_0002));
    }
}

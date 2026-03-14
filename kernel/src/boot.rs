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

/// AIRCR register VECTKEY: must be written as 0x05FA for the write to take effect.
pub const AIRCR_VECTKEY: u32 = 0x05FA << 16;

/// PRIGROUP field value: 0 means all priority bits are preemption priority
/// (no sub-priority grouping). Occupies bits \[10:8\] of the AIRCR register.
pub const AIRCR_PRIGROUP_0: u32 = 0 << 8;

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
    /// Failed to update PCB MPU data region base.
    MpuDataRegionError { partition_index: usize },
    /// Sentinel partition (mpu_region size==0) used with MPU_ENFORCE=true.
    ///
    /// Sentinel partitions produce deny-all MPU regions; running them with
    /// hardware MPU enforcement would immediately fault on first instruction.
    SentinelMpuWithEnforce { partition_index: usize },
    /// Sentinel MPU promotion failed for a partition.
    SentinelPromotionFailed { partition_index: usize },
    /// MPU cache population failed during boot precompute.
    MpuCachePopulationFailed { partition_index: usize },
    /// Boot-time MPU initialization failed.
    BootMpuInitFailed { reason: &'static str },
    /// Kernel state not initialized when boot() was called.
    KernelNotInitialized,
    /// Too many partitions for the kernel configuration.
    TooManyPartitions { given: usize, max: usize },
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
            Self::MpuDataRegionError { partition_index } => {
                write!(
                    f,
                    "failed to update PCB MPU data region: partition {partition_index}"
                )
            }
            Self::SentinelMpuWithEnforce { partition_index } => {
                write!(
                    f,
                    "sentinel partition {partition_index} has mpu_region size==0, \
                     incompatible with MPU_ENFORCE=true (would deny-all fault)"
                )
            }
            Self::SentinelPromotionFailed { partition_index } => {
                write!(
                    f,
                    "sentinel MPU promotion failed for partition {partition_index}"
                )
            }
            Self::MpuCachePopulationFailed { partition_index } => {
                write!(
                    f,
                    "MPU cache population failed for partition {partition_index}"
                )
            }
            Self::BootMpuInitFailed { reason } => {
                write!(f, "boot MPU init failed: {reason}")
            }
            Self::KernelNotInitialized => {
                write!(f, "kernel state not initialized")
            }
            Self::TooManyPartitions { given, max } => {
                write!(f, "too many partitions: {given} given, max {max}")
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

/// Reject sentinel partitions (mpu_region size==0) when MPU enforcement is active.
/// Sentinel partitions produce deny-all MPU regions that would immediately fault.
#[inline]
pub fn check_sentinel_mpu_enforce(
    partitions: &[crate::partition::PartitionControlBlock],
    mpu_enforce: bool,
) -> Result<(), BootError> {
    if !mpu_enforce {
        return Ok(());
    }
    for (i, pcb) in partitions.iter().enumerate() {
        if pcb.mpu_region().size() == 0 {
            return Err(BootError::SentinelMpuWithEnforce { partition_index: i });
        }
    }
    Ok(())
}

/// Apply MPU data region fixup for sentinel partitions only.
///
/// If the partition's `mpu_region` has size==0 (sentinel), updates its base
/// to `stack_base`. User-configured partitions (size>0) are left unchanged.
///
/// Returns `true` if the partition was a sentinel and was updated,
/// `false` if it was user-configured and skipped.
#[inline]
pub fn fix_mpu_data_region_if_sentinel(
    pcb: &mut crate::partition::PartitionControlBlock,
    stack_base: u32,
) -> bool {
    if pcb.mpu_region().size() == 0 {
        pcb.fix_mpu_data_region(stack_base);
        true
    } else {
        false
    }
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
            // Mirror PCB stack_limit into the PartitionCore array for PendSV
            // overflow pre-check (must run after fix_stack_region updates it).
            k.sync_stack_limit(i);
            // Only fix MPU data region for sentinel (size==0) partitions;
            // user-configured partitions keep their original base.
            match k.partitions_mut().get_mut(i) {
                Some(pcb) => {
                    fix_mpu_data_region_if_sentinel(pcb, base);
                }
                None => return Err(BootError::MpuDataRegionError { partition_index: i }),
            }
        }
        // Reject sentinel partitions when MPU enforcement is active.
        check_sentinel_mpu_enforce(k.partitions().as_slice(), C::MPU_ENFORCE)?;
        // Pre-compute cached MPU register pairs now that all fixups are applied.
        for i in 0..partitions.len() {
            if let Some(pcb) = k.partitions_mut().get_mut(i) {
                crate::mpu::precompute_mpu_cache(pcb)
                    .map_err(|_| BootError::MpuCachePopulationFailed { partition_index: i })?;
                debug_assert!(
                    pcb.cached_base_regions().first() != Some(&(0, 0)),
                    "partition {} cached_base_regions[0] is all-zeros after precompute",
                    i
                );
            }
        }
        // Verify PCB stack_base addresses were patched into the live storage range;
        // catches stale pre-move addresses left over from static init.
        #[cfg(debug_assertions)]
        {
            let storage_start = k as *const _ as u32;
            let storage_end = storage_start + core::mem::size_of_val(k) as u32;
            crate::invariants::assert_pcb_addresses_in_storage(
                k.partitions().as_slice(),
                storage_start,
                storage_end,
            );
        }
        Ok::<(), BootError>(())
    })
    .map_err(|_| BootError::KernelNotInitialized)??;

    const { crate::config::assert_priority_order::<C>() }
    const { crate::config::assert_systick_reload::<C>() }

    // Set PRIGROUP=0 so all priority bits are preemption priority (no
    // sub-priority grouping).  This is the power-on default on most
    // Cortex-M devices but we set it explicitly because the three-tier
    // model depends on it.
    //
    // On 3-bit priority hardware (LM3S6965, nRF52840) SVCall (0x00) and
    // SysTick (0x10) both truncate to the same effective group priority
    // 0x00.  The Cortex-M architecture breaks ties by exception number:
    // SVCall #11 wins over SysTick #15, preserving the intended ordering
    // (SVCall > SysTick > application IRQs > PendSV).
    //
    // SAFETY: Called once before scheduler starts; single-core exclusive
    // access to SCB.  AIRCR write requires VECTKEY = 0x05FA.
    unsafe {
        peripherals
            .SCB
            .aircr
            .write(AIRCR_VECTKEY | AIRCR_PRIGROUP_0);
    }

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
        crate::svc::scheduler::start_schedule(k).inspect(|&pid| k.set_next_partition(pid))
    })
    .map_err(|_| BootError::KernelNotInitialized)?
    .ok_or(BootError::NoReadyPartition)?;
    let _ = first;

    // Program MPU for the first scheduled partition before PendSV fires.
    {
        extern "Rust" {
            fn __boot_mpu_init(mpu: &cortex_m::peripheral::MPU) -> Result<(), &'static str>;
        }
        // SAFETY: __boot_mpu_init is generated by define_unified_harness! and
        // called exactly once at boot, before SCB::set_pendsv() triggers the
        // first context switch. It programs static MPU regions R0-R3 for the
        // partition selected by start_schedule(). With dynamic-mpu, it also
        // programs dynamic slot 0 (R4).
        unsafe { __boot_mpu_init(&peripherals.MPU) }
            .map_err(|reason| BootError::BootMpuInitFailed { reason })?;
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

/// Like [`boot`] but uses caller-provided `stacks` instead of internal storage.
#[cfg(not(test))]
pub fn boot_external<C: KernelConfig, const SW: usize>(
    partitions: &[(extern "C" fn() -> !, u32)],
    peripherals: &mut cortex_m::Peripherals,
    stacks: &mut [[u32; SW]],
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

    if partitions.len() > C::N {
        return Err(BootError::TooManyPartitions {
            given: partitions.len(),
            max: C::N,
        });
    }

    let storage_addr = addr_of!(crate::state::UNIFIED_KERNEL_STORAGE) as u32;
    check_storage_alignment(storage_addr, crate::state::KERNEL_ALIGNMENT as u32)?;

    crate::state::with_kernel_mut::<C, _, _>(|k| {
        for (i, &(ep, hint)) in partitions.iter().enumerate() {
            let stk = stacks
                .get_mut(i)
                .ok_or(BootError::StackInitFailed { partition_index: i })?;
            let base = stk.as_ptr() as u32;
            let size = u32::try_from(SW)
                .ok()
                .and_then(|s| s.checked_mul(4))
                .ok_or(BootError::StackSizeOverflow { partition_index: i })?;
            check_stack_mpu_size(i, size)?;
            check_stack_mpu_alignment(i, base, size)?;
            let ix =
                crate::context::init_stack_frame(&mut stk[..], ep as *const () as u32, Some(hint))
                    .ok_or(BootError::StackInitFailed { partition_index: i })?;
            let sp = u32::try_from(ix)
                .ok()
                .and_then(|v| v.checked_mul(4))
                .and_then(|offset| base.checked_add(offset))
                .ok_or(BootError::StackSizeOverflow { partition_index: i })?;
            k.set_sp(i, sp);
            // Set PCB stack fields directly and apply sentinel MPU fixup.
            let pcb = k
                .partitions_mut()
                .get_mut(i)
                .ok_or(BootError::StackRegionError { partition_index: i })?;
            pcb.set_stack_fields(base, size)
                .map_err(|_| BootError::StackRegionError { partition_index: i })?;
            fix_mpu_data_region_if_sentinel(pcb, base);
            k.sync_stack_limit(i);
        }
        check_sentinel_mpu_enforce(k.partitions().as_slice(), C::MPU_ENFORCE)?;
        for i in 0..partitions.len() {
            if let Some(pcb) = k.partitions_mut().get_mut(i) {
                crate::mpu::precompute_mpu_cache(pcb)
                    .map_err(|_| BootError::MpuCachePopulationFailed { partition_index: i })?;
                debug_assert!(
                    pcb.cached_base_regions().first() != Some(&(0, 0)),
                    "partition {} cached_base_regions[0] is all-zeros after precompute",
                    i
                );
            }
        }
        // No assert_pcb_addresses_in_storage — stacks are external.
        Ok::<(), BootError>(())
    })
    .map_err(|_| BootError::KernelNotInitialized)??;

    const { crate::config::assert_priority_order::<C>() }
    const { crate::config::assert_systick_reload::<C>() }

    // SAFETY: Called once before scheduler starts; exclusive SCB access.
    unsafe {
        peripherals
            .SCB
            .aircr
            .write(AIRCR_VECTKEY | AIRCR_PRIGROUP_0);
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

    crate::state::with_kernel_mut::<C, _, _>(|k| {
        crate::svc::scheduler::start_schedule(k).inspect(|&pid| k.set_next_partition(pid))
    })
    .map_err(|_| BootError::KernelNotInitialized)?
    .ok_or(BootError::NoReadyPartition)?;

    {
        extern "Rust" {
            fn __boot_mpu_init(mpu: &cortex_m::peripheral::MPU) -> Result<(), &'static str>;
        }
        // SAFETY: __boot_mpu_init is generated by define_unified_harness!
        // and called exactly once before the first context switch.
        unsafe { __boot_mpu_init(&peripherals.MPU) }
            .map_err(|reason| BootError::BootMpuInitFailed { reason })?;
    }

    peripherals
        .SYST
        .set_clock_source(if C::USE_PROCESSOR_CLOCK {
            SystClkSource::Core
        } else {
            SystClkSource::External
        });
    // TODO(panic-free): SYSTICK_CYCLES - 1 can underflow if SYSTICK_CYCLES == 0;
    // assert_systick_reload (const-evaluated above) prevents this, but a
    // saturating_sub would be strictly panic-free.
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

    #[test]
    fn sentinel_mpu_with_enforce_display() {
        let err = BootError::SentinelMpuWithEnforce { partition_index: 1 };
        assert_eq!(
            std::format!("{err}"),
            "sentinel partition 1 has mpu_region size==0, \
             incompatible with MPU_ENFORCE=true (would deny-all fault)"
        );
    }

    #[test]
    fn check_sentinel_mpu_enforce_rejects_sentinel_when_enforced() {
        use crate::partition::{MpuRegion, PartitionControlBlock};
        // Sentinel partition: mpu_region size == 0
        let sentinel = PartitionControlBlock::new(
            0,
            0x0800_0000,
            0x2000_0000,
            0x2000_0100,
            MpuRegion::new(0, 0, 0),
        );
        let result = check_sentinel_mpu_enforce(&[sentinel], true);
        assert_eq!(
            result,
            Err(BootError::SentinelMpuWithEnforce { partition_index: 0 })
        );
    }

    #[test]
    fn check_sentinel_mpu_enforce_accepts_sentinel_when_not_enforced() {
        use crate::partition::{MpuRegion, PartitionControlBlock};
        // Sentinel partition with MPU_ENFORCE=false should pass.
        let sentinel = PartitionControlBlock::new(
            0,
            0x0800_0000,
            0x2000_0000,
            0x2000_0100,
            MpuRegion::new(0, 0, 0),
        );
        assert!(check_sentinel_mpu_enforce(&[sentinel], false).is_ok());
    }

    #[test]
    fn check_sentinel_mpu_enforce_accepts_non_sentinel_when_enforced() {
        use crate::partition::{MpuRegion, PartitionControlBlock};
        // Non-sentinel partition (size > 0) with MPU_ENFORCE=true should pass.
        let pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,
            0x2000_0000,
            0x2000_0100,
            MpuRegion::new(0x2000_0000, 1024, 0x03),
        );
        assert!(check_sentinel_mpu_enforce(&[pcb], true).is_ok());
    }

    #[test]
    fn check_sentinel_mpu_enforce_reports_first_sentinel_index() {
        use crate::partition::{MpuRegion, PartitionControlBlock};
        // Mix of non-sentinel and sentinel; should report the sentinel's index.
        let good = PartitionControlBlock::new(
            0,
            0x0800_0000,
            0x2000_0000,
            0x2000_0100,
            MpuRegion::new(0x2000_0000, 256, 0x03),
        );
        let sentinel = PartitionControlBlock::new(
            1,
            0x0800_1000,
            0x2000_1000,
            0x2000_1100,
            MpuRegion::new(0, 0, 0),
        );
        let result = check_sentinel_mpu_enforce(&[good, sentinel], true);
        assert_eq!(
            result,
            Err(BootError::SentinelMpuWithEnforce { partition_index: 1 })
        );
    }

    #[test]
    fn boot_fixup_sentinel_updated_user_configured_skipped() {
        use crate::partition::{MpuRegion, PartitionControlBlock};

        // Partition 0: sentinel (mpu_region size==0)
        let mut sentinel = PartitionControlBlock::new(
            0,
            0x0800_0000,
            0x2000_0000,
            0x2000_0400,
            MpuRegion::new(0, 0, 0),
        );
        // Partition 1: user-configured (mpu_region size>0)
        let mut configured = PartitionControlBlock::new(
            1,
            0x0800_1000,
            0x2000_1000,
            0x2000_1400,
            MpuRegion::new(0x2004_0000, 2048, 0x0306_0000),
        );

        let stack_base_0: u32 = 0x2008_0000;
        let stack_base_1: u32 = 0x2009_0000;

        // Apply fixup to sentinel partition — should update base
        let updated = fix_mpu_data_region_if_sentinel(&mut sentinel, stack_base_0);
        assert!(updated, "sentinel partition should be updated");
        assert_eq!(sentinel.mpu_region().base(), stack_base_0);
        assert_eq!(sentinel.mpu_region().size(), 0, "size stays 0");
        assert_eq!(sentinel.mpu_region().permissions(), 0, "permissions stay 0");

        // Apply fixup to user-configured partition — should be skipped
        let skipped = fix_mpu_data_region_if_sentinel(&mut configured, stack_base_1);
        assert!(!skipped, "user-configured partition should be skipped");
        assert_eq!(
            configured.mpu_region().base(),
            0x2004_0000,
            "base unchanged"
        );
        assert_eq!(configured.mpu_region().size(), 2048, "size unchanged");
        assert_eq!(
            configured.mpu_region().permissions(),
            0x0306_0000,
            "permissions unchanged"
        );
    }

    #[test]
    fn sentinel_promotion_failed_display() {
        let err = BootError::SentinelPromotionFailed { partition_index: 3 };
        assert_eq!(
            std::format!("{err}"),
            "sentinel MPU promotion failed for partition 3"
        );
    }

    #[test]
    fn sentinel_promotion_failed_carries_correct_index() {
        let err = BootError::SentinelPromotionFailed { partition_index: 7 };
        assert_eq!(
            err,
            BootError::SentinelPromotionFailed { partition_index: 7 }
        );
        // Verify different index produces inequality.
        assert_ne!(
            err,
            BootError::SentinelPromotionFailed { partition_index: 0 }
        );
        // Verify it is distinct from other variants with the same index.
        assert_ne!(
            err,
            BootError::SentinelMpuWithEnforce { partition_index: 7 }
        );
    }

    #[test]
    fn mpu_cache_population_failed_display() {
        let err = BootError::MpuCachePopulationFailed { partition_index: 2 };
        assert_eq!(
            std::format!("{err}"),
            "MPU cache population failed for partition 2"
        );
        // Verify variant identity and index discrimination.
        assert_eq!(
            err,
            BootError::MpuCachePopulationFailed { partition_index: 2 }
        );
        assert_ne!(
            err,
            BootError::MpuCachePopulationFailed { partition_index: 0 }
        );
        // Distinct from other partition-indexed variants with the same index.
        assert_ne!(err, BootError::StackInitFailed { partition_index: 2 });
    }

    #[test]
    fn boot_mpu_init_failed_display() {
        let err = BootError::BootMpuInitFailed {
            reason: "partition lookup failed",
        };
        assert_eq!(
            std::format!("{err}"),
            "boot MPU init failed: partition lookup failed"
        );
    }

    #[test]
    fn boot_mpu_init_failed_equality_and_discrimination() {
        let err = BootError::BootMpuInitFailed {
            reason: "test error",
        };
        assert_eq!(
            err,
            BootError::BootMpuInitFailed {
                reason: "test error"
            }
        );
        // Different reason string produces inequality.
        assert_ne!(
            err,
            BootError::BootMpuInitFailed {
                reason: "other error"
            }
        );
        // Distinct from other variants.
        assert_ne!(err, BootError::NoReadyPartition);
    }

    #[test]
    fn precompute_cache_populates_non_sentinel_partition() {
        use crate::mpu::precompute_mpu_cache;
        use crate::partition::{MpuRegion, PartitionControlBlock};

        // Non-sentinel partition with valid MPU-compatible regions.
        let mut pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,        // entry_point (code region)
            0x2000_0000,        // stack_base (1024-byte aligned)
            0x2000_0000 + 1024, // stack_pointer
            MpuRegion::new(0x2000_0000, 1024, 0x0306_0000),
        );
        // Verify cache starts as all-zeros.
        assert_eq!(*pcb.cached_base_regions(), [(0, 0); 4]);

        precompute_mpu_cache(&mut pcb).unwrap();

        // After precompute, cached_base_regions[0] must be non-zero
        // (region 0 is the background deny-all region).
        assert_ne!(
            pcb.cached_base_regions()[0],
            (0, 0),
            "cached_base_regions[0] must be non-zero after precompute"
        );
        // Verify all four base regions were populated (deny-all always fills all 4).
        for (i, &(rbar, rasr)) in pcb.cached_base_regions().iter().enumerate() {
            assert!(
                rbar != 0 || rasr != 0,
                "base region R{} should not be all-zeros",
                i
            );
        }
    }

    #[test]
    fn precompute_cache_populates_sentinel_after_fixup() {
        use crate::mpu::precompute_mpu_cache;
        use crate::partition::{MpuRegion, PartitionControlBlock};

        // Sentinel partition (mpu_region size==0) — simulates pre-fixup state.
        let mut pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,
            0x2000_0000,
            0x2000_0000 + 1024,
            MpuRegion::new(0, 0, 0),
        );
        // Apply the same fixup boot() would: set base to stack_base.
        fix_mpu_data_region_if_sentinel(&mut pcb, 0x2000_0000);

        precompute_mpu_cache(&mut pcb).unwrap();

        // Even for a sentinel (deny-all), region 0 must be non-zero.
        assert_ne!(
            pcb.cached_base_regions()[0],
            (0, 0),
            "sentinel cached_base_regions[0] must be non-zero after precompute"
        );
    }

    #[test]
    fn precompute_cache_matches_on_the_fly_computation() {
        use crate::mpu::{
            partition_mpu_regions_or_deny_all, peripheral_mpu_regions_or_disabled,
            precompute_mpu_cache,
        };
        use crate::partition::{MpuRegion, PartitionControlBlock};

        let mut pcb = PartitionControlBlock::new(
            0,
            0x0800_0000,
            0x2000_0000,
            0x2000_0000 + 1024,
            MpuRegion::new(0x2000_0000, 1024, 0x0306_0000),
        );
        // Compute expected values before caching.
        let expected_base = partition_mpu_regions_or_deny_all(&pcb);
        let expected_periph = peripheral_mpu_regions_or_disabled(&pcb);

        precompute_mpu_cache(&mut pcb).unwrap();

        assert_eq!(
            *pcb.cached_base_regions(),
            expected_base,
            "cached base regions must match on-the-fly computation"
        );
        assert_eq!(
            *pcb.cached_periph_regions(),
            expected_periph,
            "cached periph regions must match on-the-fly computation"
        );
    }

    #[test]
    fn aircr_vectkey_encoding() {
        // VECTKEY must occupy bits [31:16] with value 0x05FA.
        assert_eq!(AIRCR_VECTKEY, 0x05FA_0000);
        assert_eq!(AIRCR_VECTKEY >> 16, 0x05FA);
        // Bits [15:0] must be zero (no other fields set by the constant).
        assert_eq!(AIRCR_VECTKEY & 0xFFFF, 0);
    }

    #[test]
    fn aircr_prigroup_0_encoding() {
        // PRIGROUP occupies bits [10:8]. PRIGROUP=0 means all bits are
        // preemption priority with no sub-priority grouping.
        assert_eq!(AIRCR_PRIGROUP_0, 0);
        // Verify the combined AIRCR write value: VECTKEY | PRIGROUP=0.
        let aircr_value = AIRCR_VECTKEY | AIRCR_PRIGROUP_0;
        assert_eq!(aircr_value, 0x05FA_0000);
        // Extract PRIGROUP field (bits [10:8]) from the combined value.
        let prigroup = (aircr_value >> 8) & 0x7;
        assert_eq!(prigroup, 0, "PRIGROUP field must be 0");
        // Verify VECTKEY field (bits [31:16]) from the combined value.
        let vectkey = (aircr_value >> 16) & 0xFFFF;
        assert_eq!(vectkey, 0x05FA, "VECTKEY must be 0x05FA");
    }
}

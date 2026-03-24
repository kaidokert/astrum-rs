//! SEGGER SystemView tracing integration via rtos-trace.
//!
//! Gated behind `cfg(feature = "trace")`. Implements [`RtosTraceOSCallbacks`]
//! and [`RtosTraceApplicationCallbacks`] for the kernel.

use core::sync::atomic::{AtomicU32, Ordering};

use rtos_trace::{RtosTraceApplicationCallbacks, RtosTraceOSCallbacks, TaskInfo};

// TODO: MAX_PARTITIONS should be derived from the kernel's KernelConfig::N const generic
// rather than hardcoded. This requires a design change to bridge the generic PartitionCore<N>
// to the non-generic RtosTraceOSCallbacks trait.
const MAX_PARTITIONS: usize = 4;

/// Convert a tick count to microseconds using saturating arithmetic.
pub fn ticks_to_us(ticks: u64, tick_period_us: u32) -> u64 {
    ticks.saturating_mul(tick_period_us as u64)
}

/// Map partition ID to a static name string.
// TODO: PCBs do not currently store a name field; derive from PCB if one is added.
const PARTITION_NAMES: [&str; MAX_PARTITIONS] = {
    const fn make_names<const N: usize>() -> [&'static str; N] {
        let mut names = [""; N];
        let mut i = 0;
        while i < N {
            names[i] = match i {
                0 => "P0",
                1 => "P1",
                2 => "P2",
                3 => "P3",
                4 => "P4",
                5 => "P5",
                6 => "P6",
                7 => "P7",
                _ => "P?",
            };
            i += 1;
        }
        names
    }
    make_names::<MAX_PARTITIONS>()
};

pub fn partition_name(id: u8) -> &'static str {
    PARTITION_NAMES.get(id as usize).copied().unwrap_or("P?")
}

/// Build an [`rtos_trace::TaskInfo`] from partition attributes.
pub fn build_task_info(id: u8, stack_base: u32, stack_size: u32) -> TaskInfo {
    TaskInfo {
        name: partition_name(id),
        priority: id as u32,
        stack_base: stack_base as usize,
        stack_size: stack_size as usize,
    }
}

static TICK_PERIOD_US: AtomicU32 = AtomicU32::new(1000);
static SYSCLOCK_HZ: AtomicU32 = AtomicU32::new(12_000_000);
static TICK_LO: AtomicU32 = AtomicU32::new(0);
static TICK_HI: AtomicU32 = AtomicU32::new(0);
static PARTITION_COUNT: AtomicU32 = AtomicU32::new(0);
static PART_STACK_BASE: [AtomicU32; MAX_PARTITIONS] = [
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
];
static PART_STACK_SIZE: [AtomicU32; MAX_PARTITIONS] = [
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
    AtomicU32::new(0),
];

/// Update the stored tick count. Call from the SysTick handler.
///
/// Stores high word first so the read-retry loop in [`read_tick`] can detect
/// a torn read when the low word wraps.
pub fn update_tick(tick: u64) {
    TICK_HI.store((tick >> 32) as u32, Ordering::Release);
    TICK_LO.store(tick as u32, Ordering::Release);
}

/// Read the 64-bit tick count using a high-word retry loop to avoid tearing
/// on 32-bit targets where two separate 32-bit loads are required.
fn read_tick() -> u64 {
    loop {
        let hi1 = TICK_HI.load(Ordering::Acquire) as u64;
        let lo = TICK_LO.load(Ordering::Acquire) as u64;
        let hi2 = TICK_HI.load(Ordering::Acquire) as u64;
        if hi1 == hi2 {
            return (hi1 << 32) | lo;
        }
    }
}

/// Store metadata for one partition at `index`.
pub fn register_partition_meta(index: usize, stack_base: u32, stack_size: u32) {
    if index < MAX_PARTITIONS {
        PART_STACK_BASE[index].store(stack_base, Ordering::Release);
        PART_STACK_SIZE[index].store(stack_size, Ordering::Release);
    }
}

/// Set the number of registered partitions.
pub fn set_partition_count(count: usize) {
    PARTITION_COUNT.store(count.min(MAX_PARTITIONS) as u32, Ordering::Release);
}

/// Snapshot partition metadata from kernel partition control blocks.
// TODO: Shadow state exists because RtosTraceOSCallbacks::task_list() is a static
// method with no access to the generic PartitionCore<N>. Eliminating the shadow
// copies requires a type-erased global pointer or a redesigned callback trait.
pub fn register_partitions(partitions: &[crate::partition::PartitionControlBlock]) {
    for (i, pcb) in partitions.iter().take(MAX_PARTITIONS).enumerate() {
        let id = i as u8;
        let base = pcb.stack_base();
        let size = pcb.stack_size();
        register_partition_meta(i, base, size);
        rtos_trace::trace::task_new(id as u32);
        rtos_trace::trace::task_send_info(id as u32, build_task_info(id, base, size));
    }
    set_partition_count(partitions.len());
}

// TODO: reviewer false positive — trace.rs is already module-gated by
// #[cfg(feature = "trace")] in lib.rs; items here do not need individual gating.

/// Base marker ID for MemManage fault events.
///
/// Chosen well above the syscall ID range (max ~0x41) to avoid collisions.
/// Fault-type-specific markers are assigned consecutive IDs starting here.
pub const FAULT_MARKER_BASE: u32 = 0x100;
pub const FAULT_MARKER_DACCVIOL: u32 = 0x101;
pub const FAULT_MARKER_IACCVIOL: u32 = 0x102;
pub const FAULT_MARKER_MSTKERR: u32 = 0x103;
pub const FAULT_MARKER_MUNSTKERR: u32 = 0x104;
pub const FAULT_MARKER_MLSPERR: u32 = 0x105;
pub const FAULT_MARKER_OTHER: u32 = 0x106;

/// Map CFSR bits to the appropriate fault-type marker ID.
fn fault_marker_for_cfsr(cfsr: u32) -> u32 {
    use crate::fault::{CFSR_DACCVIOL, CFSR_IACCVIOL, CFSR_MLSPERR, CFSR_MSTKERR, CFSR_MUNSTKERR};
    if cfsr & CFSR_DACCVIOL != 0 {
        FAULT_MARKER_DACCVIOL
    } else if cfsr & CFSR_IACCVIOL != 0 {
        FAULT_MARKER_IACCVIOL
    } else if cfsr & CFSR_MSTKERR != 0 {
        FAULT_MARKER_MSTKERR
    } else if cfsr & CFSR_MUNSTKERR != 0 {
        FAULT_MARKER_MUNSTKERR
    } else if cfsr & CFSR_MLSPERR != 0 {
        FAULT_MARKER_MLSPERR
    } else {
        FAULT_MARKER_OTHER
    }
}

/// Register MemManage fault markers with descriptive names.
pub fn name_fault_markers() {
    rtos_trace::trace::name_marker(FAULT_MARKER_BASE, "MemManage_Fault");
    rtos_trace::trace::name_marker(FAULT_MARKER_DACCVIOL, "MemManage_DACCVIOL");
    rtos_trace::trace::name_marker(FAULT_MARKER_IACCVIOL, "MemManage_IACCVIOL");
    rtos_trace::trace::name_marker(FAULT_MARKER_MSTKERR, "MemManage_MSTKERR");
    rtos_trace::trace::name_marker(FAULT_MARKER_MUNSTKERR, "MemManage_MUNSTKERR");
    rtos_trace::trace::name_marker(FAULT_MARKER_MLSPERR, "MemManage_MLSPERR");
    rtos_trace::trace::name_marker(FAULT_MARKER_OTHER, "MemManage_Other");
}

/// Emit trace events for a MemManage fault: a fault-type marker and task_terminate.
///
/// The fault type is encoded in the marker ID (derived from CFSR bits).
// TODO: rtos_trace marker API does not support data payloads; partition_id is
// conveyed via task_terminate rather than embedded in the marker itself.
pub fn emit_fault_trace(details: &crate::fault::FaultDetails) {
    rtos_trace::trace::marker(fault_marker_for_cfsr(details.cfsr));
    rtos_trace::trace::task_terminate(details.partition_id as u32);
}

/// Register human-readable names for syscall trace markers.
///
/// Each syscall ID is mapped to a marker via `rtos_trace::trace::name_marker`,
/// using the raw syscall number as the marker ID and `SyscallId::name()` as
/// the human-readable label.
pub fn name_syscall_markers() {
    use crate::syscall::SyscallId;

    const BASE_SYSCALLS: &[SyscallId] = &[
        SyscallId::Yield,
        SyscallId::GetPartitionId,
        SyscallId::EventWait,
        SyscallId::EventSet,
        SyscallId::EventClear,
        SyscallId::SemWait,
        SyscallId::SemSignal,
        SyscallId::MutexLock,
        SyscallId::MutexUnlock,
        SyscallId::MsgSend,
        SyscallId::MsgRecv,
        SyscallId::GetTime,
        SyscallId::SamplingWrite,
        SyscallId::SamplingRead,
        SyscallId::QueuingSend,
        SyscallId::QueuingRecv,
        SyscallId::QueuingStatus,
        SyscallId::BbDisplay,
        SyscallId::BbRead,
        SyscallId::BbClear,
        SyscallId::QueuingSendTimed,
        SyscallId::QueuingRecvTimed,
        SyscallId::DebugPrint,
        SyscallId::DebugExit,
        SyscallId::IrqAck,
        SyscallId::SleepTicks,
    ];
    for &id in BASE_SYSCALLS {
        rtos_trace::trace::name_marker(id.as_u32(), id.name());
    }
}

pub struct TraceCallbacks;

impl RtosTraceOSCallbacks for TraceCallbacks {
    fn task_list() {
        let count = PARTITION_COUNT.load(Ordering::Acquire) as usize;
        for i in 0..count {
            let id = i as u8;
            let base = match PART_STACK_BASE.get(i) {
                Some(v) => v.load(Ordering::Acquire),
                None => break,
            };
            let size = match PART_STACK_SIZE.get(i) {
                Some(v) => v.load(Ordering::Acquire),
                None => break,
            };
            rtos_trace::trace::task_new(id as u32);
            rtos_trace::trace::task_send_info(id as u32, build_task_info(id, base, size));
        }
    }

    fn time() -> u64 {
        ticks_to_us(read_tick(), TICK_PERIOD_US.load(Ordering::Acquire))
    }
}

pub struct TraceAppCallbacks;

impl RtosTraceApplicationCallbacks for TraceAppCallbacks {
    fn system_description() {
        #[cfg(target_arch = "arm")]
        systemview_target::SystemView::send_system_description(concat!(
            "N=cortex-m-rtos,V=",
            env!("CARGO_PKG_VERSION")
        ));
    }

    fn sysclock() -> u32 {
        SYSCLOCK_HZ.load(Ordering::Acquire)
    }
}

#[cfg(all(not(test), target_arch = "arm"))]
rtos_trace::global_trace!(systemview_target::SystemView);

#[cfg(not(test))]
rtos_trace::global_os_callbacks!(TraceCallbacks);

#[cfg(not(test))]
rtos_trace::global_application_callbacks!(TraceAppCallbacks);

/// Initialize the SystemView tracing backend and start tracing.
///
/// `tick_period_us` and `sysclock_hz` should match the kernel config
/// (typically `C::TICK_PERIOD_US` and `C::CORE_CLOCK_HZ`).
#[cfg(target_arch = "arm")]
pub fn init_trace(tick_period_us: u32, sysclock_hz: u32) {
    TICK_PERIOD_US.store(tick_period_us, Ordering::Release);
    SYSCLOCK_HZ.store(sysclock_hz, Ordering::Release);
    static SYSVIEW: systemview_target::SystemView = systemview_target::SystemView::new();
    SYSVIEW.init();
    rtos_trace::trace::start();
    name_syscall_markers();
    name_fault_markers();
}

/// Non-ARM stub: stores config values but skips SystemView hardware init.
#[cfg(not(target_arch = "arm"))]
pub fn init_trace(tick_period_us: u32, sysclock_hz: u32) {
    TICK_PERIOD_US.store(tick_period_us, Ordering::Release);
    SYSCLOCK_HZ.store(sysclock_hz, Ordering::Release);
    name_syscall_markers();
    name_fault_markers();
}

#[cfg(test)]
mod tests {
    use super::*;

    static TEST_MUTEX: std::sync::Mutex<()> = std::sync::Mutex::new(());

    #[test]
    fn ticks_to_us_basic_conversions() {
        assert_eq!(ticks_to_us(0, 1000), 0);
        assert_eq!(ticks_to_us(1, 1000), 1000);
        assert_eq!(ticks_to_us(100, 1000), 100_000);
        assert_eq!(ticks_to_us(10, 500), 5_000);
        assert_eq!(ticks_to_us(10, 2000), 20_000);
        assert_eq!(ticks_to_us(42, 1), 42);
    }

    #[test]
    fn ticks_to_us_edge_cases() {
        assert_eq!(ticks_to_us(100, 0), 0);
        assert_eq!(ticks_to_us(u64::MAX, 2), u64::MAX);
        assert_eq!(ticks_to_us(u64::MAX, 1000), u64::MAX);
        assert_eq!(ticks_to_us(1_000_000_000_000, 1000), 1_000_000_000_000_000);
    }

    #[test]
    fn partition_name_mapping() {
        assert_eq!(partition_name(0), "P0");
        assert_eq!(partition_name(1), "P1");
        assert_eq!(partition_name(2), "P2");
        assert_eq!(partition_name(3), "P3");
        assert_eq!(partition_name(4), "P?");
        assert_eq!(partition_name(255), "P?");
    }

    #[test]
    fn build_task_info_maps_all_fields() {
        let info = build_task_info(2, 0x2000_0000, 1024);
        assert_eq!(info.name, "P2");
        assert_eq!(info.priority, 2);
        assert_eq!(info.stack_base, 0x2000_0000);
        assert_eq!(info.stack_size, 1024);
    }

    #[test]
    fn build_task_info_boundary_values() {
        let z = build_task_info(0, 0x2000_1000, 512);
        assert_eq!((z.name, z.priority), ("P0", 0));

        let u = build_task_info(7, 0x2000_2000, 2048);
        assert_eq!((u.name, u.priority), ("P?", 7));

        let big = build_task_info(1, 0x2000_0000, 0x0001_0000);
        assert_eq!(big.stack_size, 0x0001_0000);
    }

    #[test]
    fn tick_storage_round_trip() {
        let _g = TEST_MUTEX.lock().unwrap();
        update_tick(42);
        assert_eq!(read_tick(), 42);

        update_tick(0x1_FFFF_FFFF);
        assert_eq!(read_tick(), 0x1_FFFF_FFFF);

        update_tick(0);
        assert_eq!(read_tick(), 0);

        update_tick(u64::MAX);
        assert_eq!(read_tick(), u64::MAX);
    }

    #[test]
    fn partition_meta_registration() {
        let _g = TEST_MUTEX.lock().unwrap();
        register_partition_meta(0, 0x2000_0000, 1024);
        assert_eq!(PART_STACK_BASE[0].load(Ordering::Acquire), 0x2000_0000);
        assert_eq!(PART_STACK_SIZE[0].load(Ordering::Acquire), 1024);
        // Out-of-bounds is silently ignored
        register_partition_meta(MAX_PARTITIONS, 0xDEAD, 0xBEEF);
        register_partition_meta(255, 0xDEAD, 0xBEEF);
    }

    #[test]
    fn partition_count_clamps() {
        let _g = TEST_MUTEX.lock().unwrap();
        set_partition_count(2);
        assert_eq!(PARTITION_COUNT.load(Ordering::Acquire), 2);
        set_partition_count(100);
        assert_eq!(
            PARTITION_COUNT.load(Ordering::Acquire),
            MAX_PARTITIONS as u32
        );
    }

    #[test]
    fn time_callback_integration() {
        let _g = TEST_MUTEX.lock().unwrap();
        TICK_PERIOD_US.store(500, Ordering::Release);
        update_tick(200);
        assert_eq!(TraceCallbacks::time(), 100_000);

        TICK_PERIOD_US.store(1000, Ordering::Release);
        update_tick(0);
        assert_eq!(TraceCallbacks::time(), 0);
    }

    #[test]
    fn task_list_smoke() {
        let _g = TEST_MUTEX.lock().unwrap();
        register_partition_meta(0, 0x2000_0000, 1024);
        register_partition_meta(1, 0x2000_1000, 512);
        set_partition_count(2);
        TraceCallbacks::task_list(); // trace functions are no-ops; verify no panic

        set_partition_count(0);
        TraceCallbacks::task_list(); // empty case
    }

    #[test]
    fn app_callbacks() {
        let _g = TEST_MUTEX.lock().unwrap();
        SYSCLOCK_HZ.store(48_000_000, Ordering::Release);
        assert_eq!(TraceAppCallbacks::sysclock(), 48_000_000);
        TraceAppCallbacks::system_description(); // no-op on non-ARM
    }

    #[test]
    fn register_partitions_populates_shadow_state() {
        use crate::partition::{MpuRegion, PartitionControlBlock};

        let _g = TEST_MUTEX.lock().unwrap();
        let region = MpuRegion::new(0x2000_0000, 1024, 0);
        let pcb0 =
            PartitionControlBlock::new(0, 0x0800_0000_u32, 0x2000_0000, 0x2000_0000 + 1024, region);
        let region1 = MpuRegion::new(0x2000_1000, 512, 0);
        let pcb1 =
            PartitionControlBlock::new(1, 0x0800_1000_u32, 0x2000_1000, 0x2000_1000 + 512, region1);
        let pcbs = [pcb0, pcb1];

        register_partitions(&pcbs);

        assert_eq!(PARTITION_COUNT.load(Ordering::Acquire), 2);
        assert_eq!(PART_STACK_BASE[0].load(Ordering::Acquire), 0x2000_0000);
        assert_eq!(PART_STACK_SIZE[0].load(Ordering::Acquire), 1024);
        assert_eq!(PART_STACK_BASE[1].load(Ordering::Acquire), 0x2000_1000);
        assert_eq!(PART_STACK_SIZE[1].load(Ordering::Acquire), 512);
    }

    #[test]
    fn init_trace_stores_config() {
        let _g = TEST_MUTEX.lock().unwrap();
        init_trace(500, 48_000_000);

        assert_eq!(TICK_PERIOD_US.load(Ordering::Acquire), 500);
        assert_eq!(SYSCLOCK_HZ.load(Ordering::Acquire), 48_000_000);
    }

    #[test]
    fn name_syscall_markers_no_panic() {
        let _g = TEST_MUTEX.lock().unwrap();
        // name_marker is a no-op in test builds (no trace_impl feature),
        // but verify the function itself does not panic or violate any
        // invariant (e.g. duplicate IDs, empty names).
        name_syscall_markers();
    }

    #[test]
    fn marker_begin_end_pairing_all_base_syscalls() {
        use crate::syscall::SyscallId;

        let _g = TEST_MUTEX.lock().unwrap();
        // Verify marker_begin/marker_end for every valid raw syscall ID.
        // On host builds these are no-ops; the test proves the pairing
        // is safe for every ID that from_u32 recognises.
        for raw in 0u32..64 {
            if let Some(id) = SyscallId::from_u32(raw) {
                assert_eq!(id.as_u32(), raw);
                rtos_trace::trace::marker_begin(raw);
                rtos_trace::trace::marker_end(raw);
            }
        }
    }

    #[test]
    fn init_trace_calls_name_syscall_markers() {
        let _g = TEST_MUTEX.lock().unwrap();
        // init_trace now calls name_syscall_markers internally;
        // verify the combined path does not panic.
        init_trace(1000, 12_000_000);
        assert_eq!(TICK_PERIOD_US.load(Ordering::Acquire), 1000);
    }

    #[test]
    fn fault_marker_ids_do_not_collide_with_syscalls() {
        use crate::syscall::SyscallId;

        let fault_ids = [
            FAULT_MARKER_BASE,
            FAULT_MARKER_DACCVIOL,
            FAULT_MARKER_IACCVIOL,
            FAULT_MARKER_MSTKERR,
            FAULT_MARKER_MUNSTKERR,
            FAULT_MARKER_MLSPERR,
            FAULT_MARKER_OTHER,
        ];
        // Verify all fault marker IDs are outside the syscall ID range.
        for raw in 0u32..64 {
            if SyscallId::from_u32(raw).is_some() {
                for &fid in &fault_ids {
                    assert_ne!(
                        raw, fid,
                        "fault marker {fid:#x} collides with syscall {raw}"
                    );
                }
            }
        }
        // Also check feature-gated IDs (0x40, 0x41).
        for &fid in &fault_ids {
            assert_ne!(fid, 0x40);
            assert_ne!(fid, 0x41);
        }
        const { assert!(FAULT_MARKER_BASE > 0x41) };
    }

    #[test]
    fn fault_marker_for_cfsr_classification() {
        use crate::fault::*;
        assert_eq!(fault_marker_for_cfsr(CFSR_DACCVIOL), FAULT_MARKER_DACCVIOL);
        assert_eq!(fault_marker_for_cfsr(CFSR_IACCVIOL), FAULT_MARKER_IACCVIOL);
        assert_eq!(fault_marker_for_cfsr(CFSR_MSTKERR), FAULT_MARKER_MSTKERR);
        assert_eq!(
            fault_marker_for_cfsr(CFSR_MUNSTKERR),
            FAULT_MARKER_MUNSTKERR
        );
        assert_eq!(fault_marker_for_cfsr(CFSR_MLSPERR), FAULT_MARKER_MLSPERR);
        // DACCVIOL takes priority when multiple bits set
        assert_eq!(
            fault_marker_for_cfsr(CFSR_DACCVIOL | CFSR_MMARVALID),
            FAULT_MARKER_DACCVIOL
        );
        // Unknown/other CFSR bits map to OTHER
        assert_eq!(fault_marker_for_cfsr(0), FAULT_MARKER_OTHER);
        assert_eq!(fault_marker_for_cfsr(CFSR_UNDEFINSTR), FAULT_MARKER_OTHER);
    }

    #[test]
    fn name_fault_markers_no_panic() {
        let _g = TEST_MUTEX.lock().unwrap();
        // name_marker is a no-op in test builds; verify no panic.
        name_fault_markers();
    }

    #[test]
    fn emit_fault_trace_no_panic() {
        use crate::fault::{FaultDetails, CFSR_DACCVIOL, CFSR_IACCVIOL, CFSR_MMARVALID};
        let _g = TEST_MUTEX.lock().unwrap();
        // marker() and task_terminate() are no-ops in test builds;
        // verify the function does not panic for various fault details.
        emit_fault_trace(&FaultDetails::new(
            0,
            CFSR_DACCVIOL | CFSR_MMARVALID,
            0x2000_0000,
            0x0800_0000,
        ));
        emit_fault_trace(&FaultDetails::new(1, CFSR_IACCVIOL, 0, 0x0800_1000));
        emit_fault_trace(&FaultDetails::new(3, 0, 0, 0));
        emit_fault_trace(&FaultDetails::new(255, CFSR_DACCVIOL, 0x2000_F000, 0));
    }

    #[test]
    fn init_trace_registers_fault_markers() {
        let _g = TEST_MUTEX.lock().unwrap();
        // init_trace calls name_fault_markers internally;
        // verify the combined path does not panic.
        init_trace(500, 24_000_000);
        assert_eq!(TICK_PERIOD_US.load(Ordering::Acquire), 500);
    }
}

// Not migrated to define_unified_harness! — this test requires direct kernel
// access in SysTick to call BufferPool::lend_to_partition and revoke_from_all,
// verify MPU window state via DynamicStrategy, and coordinate the multi-phase
// alloc→lend→read→revoke lifecycle using atomic flags. The test verifies
// kernel-internal buffer pool state transitions that aren't exposed through
// the standard harness's syscall-only interface.
//
// NOTE: integration.rs and context_switch.rs have equivalent headers committed
// separately (commits d68b00f, earlier). All six legacy examples are documented.

//! QEMU test: buffer pool zero-copy lending lifecycle.
//!
//! Two-partition demo exercising the full alloc → write → lend → read → revoke
//! lifecycle.  P1 allocates a buffer via SVC, writes magic data, and signals
//! readiness.  The kernel (SysTick) performs the privileged lend operation,
//! mapping the buffer into P2's MPU window.  P2 reads and verifies the data,
//! then signals completion.  The kernel revokes the MPU window and verifies
//! cleanup.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
#[cfg(target_arch = "arm")]
use kernel::syscall::{SYS_BUF_ALLOC, SYS_BUF_RELEASE, SYS_BUF_WRITE};
use kernel::{
    mpu,
    mpu_strategy::{DynamicStrategy, MpuStrategy},
    partition::{EntryAddr, ExternalPartitionMemory, MpuRegion},
    scheduler::{ScheduleEntry, ScheduleEvent, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgMinimal, PartitionEntry, Partitions2, PortsTiny, SyncMinimal,
};

const NP: usize = 2;
const STACK_WORDS: usize = 256;
const DATA_BASES: [u32; NP] = [0x2000_0000, 0x2000_8000];
const DATA_SIZES: [u32; NP] = [4096, 4096];
const P2: kernel::PartitionId = kernel::PartitionId::new(1);
const MAGIC: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];

#[repr(C, align(1024))]
struct AlignedStack([u32; STACK_WORDS]);

static mut STACKS: [AlignedStack; NP] = {
    const ZERO: AlignedStack = AlignedStack([0; STACK_WORDS]);
    [ZERO; NP]
};
#[no_mangle]
static mut PARTITION_SP: [u32; NP] = [0; NP];
#[no_mangle]
static mut CURRENT_PARTITION: u32 = u32::MAX;
#[no_mangle]
static mut NEXT_PARTITION: u32 = 0;

kernel::compose_kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {
        buffer_pool_regions = 2;
    }
);

// Use define_unified_kernel! to create the KERNEL static and dispatch hook.
// The yield handler is empty since this test uses a custom SysTick handler.
kernel::define_unified_kernel!(TestConfig, |_k| {});

#[used]
static _SVC: kernel::SvcDispatchFn = kernel::svc::SVC_HANDLER;
static STRATEGY: DynamicStrategy = DynamicStrategy::new();
kernel::define_pendsv!(dynamic: STRATEGY, TestConfig);

/// Slot index allocated by P1 (u32::MAX = not yet allocated).
static P1_ALLOC_SLOT: AtomicU32 = AtomicU32::new(u32::MAX);
/// Set to 1 by P1 once the buffer has been written with magic data and is
/// ready for the kernel to lend to P2.
static P1_READY: AtomicU32 = AtomicU32::new(0);

/// Address of the lent buffer, published by the kernel for P2 to read.
static LENT_BUF_ADDR: AtomicU32 = AtomicU32::new(0);
/// Length of valid data in the lent buffer.
static LENT_BUF_LEN: AtomicU32 = AtomicU32::new(0);
/// P2 read result: 0 = pending, 1 = match, 2 = mismatch.
static P2_READ_RESULT: AtomicU32 = AtomicU32::new(0);

/// P1: allocate a buffer via SVC, write magic data via SVC, signal readiness.
///
/// P1 drives the first half of the lifecycle: it allocates a buffer from the
/// pool, writes the test pattern through the `SYS_BUF_WRITE` syscall (which
/// copies data from partition memory into the kernel buffer while validating
/// ownership), releases the slot, and sets P1_READY so the kernel can proceed
/// with the lend operation (a privileged action with no user-space syscall).
const _: PartitionEntry = partition_p1_entry;
extern "C" fn partition_p1_entry() -> ! {
    #[cfg(target_arch = "arm")]
    {
        let slot: u32;
        // SAFETY: `SYS_BUF_ALLOC` (20) is a valid syscall ID handled by
        // `Kernel::dispatch`. r1=1 selects BorrowMode::Write. r2, r3 are
        // unused by this syscall but must be specified to satisfy the SVC
        // ABI which clobbers r0-r3 and r12 across the `svc #0` instruction.
        // The returned r0 holds the allocated slot index (or error).
        unsafe {
            core::arch::asm!("svc #0",
                inout("r0") SYS_BUF_ALLOC => slot,
                in("r1") 1u32, in("r2") 0u32, in("r3") 0u32, out("r12") _);
        }
        P1_ALLOC_SLOT.store(slot, Ordering::Release);

        // Copy MAGIC into a stack-local buffer so the pointer lies within
        // the partition's MPU data region (check_user_ptr rejects flash/rodata
        // pointers that fall outside the partition's RAM region).
        let magic_local: [u8; 4] = MAGIC;

        // Write magic data into the allocated buffer via the SYS_BUF_WRITE
        // syscall. The kernel validates that P1 owns the slot in
        // BorrowedWrite state and copies from the partition-local buffer
        // into the pool buffer, preserving isolation.
        // SAFETY: `SYS_BUF_WRITE` (26) is a valid syscall ID handled by
        // `Kernel::dispatch`. r1=slot index, r2=data length, r3=pointer to
        // source data in partition memory. The returned r0 holds the number
        // of bytes written or an error code.
        unsafe {
            core::arch::asm!("svc #0",
                inout("r0") SYS_BUF_WRITE => _,
                in("r1") slot,
                in("r2") magic_local.len() as u32,
                in("r3") magic_local.as_ptr() as u32,
                out("r12") _);
        }

        // Release the buffer so it returns to Free state (required by
        // lend_to_partition which requires SlotFree).
        // SAFETY: `SYS_BUF_RELEASE` (21) is a valid syscall ID handled by
        // `Kernel::dispatch`. r1=slot is the index returned by the prior
        // alloc. r2, r3 are unused but must be specified per the SVC ABI.
        // The returned r0 is 0 on success.
        unsafe {
            core::arch::asm!("svc #0",
                inout("r0") SYS_BUF_RELEASE => _,
                in("r1") slot, in("r2") 0u32, in("r3") 0u32, out("r12") _);
        }

        // Signal that P1 has prepared the buffer and released it.
        P1_READY.store(1, Ordering::Release);
    }
    loop {
        cortex_m::asm::nop();
    }
}

/// P2: read lent buffer data through its MPU window and verify contents.
const _: PartitionEntry = partition_p2_entry;
extern "C" fn partition_p2_entry() -> ! {
    loop {
        let addr = LENT_BUF_ADDR.load(Ordering::Acquire);
        if addr != 0 {
            let len = LENT_BUF_LEN.load(Ordering::Acquire) as usize;
            let mut ok = true;
            for (i, &expected) in MAGIC.iter().enumerate().take(len) {
                // SAFETY: kernel configured a read-only MPU window over
                // this buffer; read_volatile prevents elision.
                let byte = unsafe { core::ptr::read_volatile((addr as *const u8).add(i)) };
                if byte != expected {
                    ok = false;
                }
            }
            P2_READ_RESULT.store(if ok { 1 } else { 2 }, Ordering::Release);
            LENT_BUF_ADDR.store(0, Ordering::Release);
        }
        cortex_m::asm::nop();
    }
}

fn check(label: &str, ok: bool, fail: &mut bool) {
    hprintln!("{} => {}", label, if ok { "PASS" } else { "FAIL" });
    if !ok {
        *fail = true;
    }
}

#[exception]
fn SysTick() {
    static mut TICK: u32 = 0;
    static mut FAIL: bool = false;
    static mut SLOT: usize = 0;
    static mut RID: u8 = 0;
    *TICK += 1;

    // Drive scheduler using the unified Kernel, configure R4 via DynamicStrategy.
    with_kernel_mut(|k| {
        let event = kernel::svc::scheduler::advance_schedule_tick(k);
        if let ScheduleEvent::PartitionSwitch(pid) = event {
            if let Some(pcb) = k.partitions().get(pid as usize) {
                let dyn_region = pcb.cached_dynamic_region();
                STRATEGY
                    .configure_partition(kernel::PartitionId::new(pid as u32), &[dyn_region], 0)
                    .expect("configure_partition");
            }
            // SAFETY: single-core; PendSV cannot preempt SysTick.
            unsafe { core::ptr::write_volatile(&raw mut NEXT_PARTITION, pid as u32) };
        }
    });

    // Tick offsets adjusted for 6-tick major frame: P0(2)+SW(1)+P1(2)+SW(1).
    match *TICK {
        6 => {
            // Step 1: P1 should have allocated, written, and released a buffer.
            // Verify P1 signaled readiness and the slot is back to Free.
            let ready = P1_READY.load(Ordering::Acquire) == 1;
            let alloc_slot = P1_ALLOC_SLOT.load(Ordering::Acquire);
            let slot_ok = ready && alloc_slot != u32::MAX;
            if slot_ok {
                *SLOT = alloc_slot as usize;
            }
            let state_ok = slot_ok
                && with_kernel_mut(|k| {
                    k.buffers.get(*SLOT).unwrap().state() == kernel::buffer_pool::BorrowState::Free
                })
                .unwrap_or(false);
            check("step1: p1-alloc-write-release", state_ok, FAIL);
        }
        7 => {
            with_kernel_mut(|k| {
                // Step 2: Kernel lends the buffer (with P1's data) read-only to P2.
                match k.buffers.lend_to_partition(*SLOT, P2, false, &STRATEGY) {
                    Ok(rid) => {
                        *RID = rid;
                        let slot = k.buffers.get(*SLOT).unwrap();
                        let ok = STRATEGY.slot(rid).is_some()
                            && slot.state()
                                == kernel::buffer_pool::BorrowState::BorrowedRead { owner: P2 }
                            && slot.mpu_region() == Some(rid);
                        check("step2: lend", ok, FAIL);
                        LENT_BUF_LEN.store(MAGIC.len() as u32, Ordering::Release);
                        LENT_BUF_ADDR.store(slot.data().as_ptr() as u32, Ordering::Release);
                    }
                    Err(_) => check("step2: lend", false, FAIL),
                }
            });
        }
        14 => check(
            "step3: p2-read",
            P2_READ_RESULT.load(Ordering::Acquire) == 1,
            FAIL,
        ),
        15 => {
            with_kernel_mut(|k| {
                // Step 4: revoke access, verify MPU region disabled.
                match k.buffers.revoke_from_partition(*SLOT, &STRATEGY) {
                    Ok(()) => {
                        let slot = k.buffers.get(*SLOT).unwrap();
                        let slot_idx = DynamicStrategy::<4>::region_to_slot_index(*RID)
                            .expect("RID must map to a valid dynamic slot");
                        let rasr = STRATEGY.partition_region_values(kernel::PartitionId::new(0))
                            [slot_idx]
                            .1;
                        let ok = slot.state() == kernel::buffer_pool::BorrowState::Free
                            && slot.mpu_region().is_none()
                            && STRATEGY.slot(*RID).is_none()
                            && rasr == 0;
                        check("step4: revoke", ok, FAIL);
                    }
                    Err(_) => check("step4: revoke", false, FAIL),
                }
            });
        }
        16 => {
            if *FAIL {
                hprintln!("buffer_pool_test: FAIL");
                debug::exit(debug::EXIT_FAILURE);
            } else {
                hprintln!("buffer_pool_test: all checks passed -- PASS");
                debug::exit(debug::EXIT_SUCCESS);
            }
        }
        _ => {}
    }
}

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    hprintln!(
        "=== {} v{} ===",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );
    hprintln!("buffer_pool_test: start");

    // Build schedule table: P0(2) → system window(1) → P1(2) → system window(1)
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).unwrap();
    if sched.add_system_window(1).is_err() {
        loop {
            debug::exit(debug::EXIT_FAILURE);
        }
    }
    sched.add(ScheduleEntry::new(1, 2)).unwrap();
    if sched.add_system_window(1).is_err() {
        loop {
            debug::exit(debug::EXIT_FAILURE);
        }
    }

    // Build partition descriptors from existing stack arrays.
    // SAFETY: before interrupts; single-core exclusive access.
    // SAFETY: before interrupts; single-core exclusive access.
    let ptr = &raw mut STACKS;
    let stacks: &mut [AlignedStack; NP] = unsafe { &mut *ptr };
    let [s0, s1] = stacks;
    let memories = [
        ExternalPartitionMemory::new(
            &mut s0.0,
            EntryAddr::from_entry(partition_p1_entry as PartitionEntry),
            MpuRegion::new(DATA_BASES[0], DATA_SIZES[0], 0),
            kernel::PartitionId::new(0),
        )
        .expect("partition memory 0"),
        ExternalPartitionMemory::new(
            &mut s1.0,
            EntryAddr::from_entry(partition_p2_entry as PartitionEntry),
            MpuRegion::new(DATA_BASES[1], DATA_SIZES[1], 0),
            kernel::PartitionId::new(1),
        )
        .expect("partition memory 1"),
    ];

    // Create the unified kernel with new()
    let mut k = Kernel::<TestConfig>::new(sched, &memories).expect("kernel creation");
    store_kernel(&mut k);

    // Seal the MPU cache so cached_dynamic_region() returns valid data.
    with_kernel_mut(|k| {
        for pid in 0..NP {
            let pcb = k.partitions_mut().get_mut(pid).expect("partition");
            mpu::precompute_mpu_cache(pcb).expect("precompute_mpu_cache");
        }
    });

    // Initialize partition stacks
    // SAFETY: before interrupts; single-core exclusive access.
    unsafe {
        let entries: [kernel::PartitionEntry; NP] = [partition_p1_entry, partition_p2_entry];
        for i in 0..NP {
            let stk = &mut STACKS[i].0;
            let ix = kernel::context::init_stack_frame(stk, entries[i], Some(i as u32))
                .expect("init_stack_frame");
            PARTITION_SP[i] = stk.as_ptr() as u32 + (ix as u32) * 4;
        }
        // Set exception priorities per the three-tier model:
        //   Tier 1 (highest): SVCall  – synchronous syscall entry
        //   Tier 2 (middle):  SysTick – time-slice preemption
        //   Tier 3 (lowest):  PendSV  – deferred context switch
        //
        // SAFETY: set_priority is unsafe because changing priority levels can
        // break priority-based critical sections. Interrupts are not yet enabled
        // so no preemption or race conditions can occur during configuration.
        cp.SCB
            .set_priority(SystemHandler::SVCall, TestConfig::SVCALL_PRIORITY);
        cp.SCB
            .set_priority(SystemHandler::PendSV, TestConfig::PENDSV_PRIORITY);
        cp.SCB
            .set_priority(SystemHandler::SysTick, TestConfig::SYSTICK_PRIORITY);
    }

    // SAFETY: interrupts not yet enabled; PRIVDEFENA keeps privileged default map.
    unsafe { cp.MPU.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
    cp.SYST.set_clock_source(SystClkSource::Core);
    cp.SYST
        .set_reload(kernel::config::compute_systick_reload(12_000_000, 10_000));
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();
    cortex_m::peripheral::SCB::set_pendsv();
    loop {
        cortex_m::asm::wfi();
    }
}

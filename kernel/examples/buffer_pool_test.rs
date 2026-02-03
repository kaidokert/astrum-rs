//! QEMU test: buffer pool zero-copy lending lifecycle.
//!
//! Two-partition demo exercising the full alloc → write → lend → read → revoke
//! lifecycle.  P1 allocates a buffer via SVC, writes magic data, and signals
//! readiness.  The kernel (SysTick) performs the privileged lend operation,
//! mapping the buffer into P2's MPU window.  P2 reads and verifies the data,
//! then signals completion.  The kernel revokes the MPU window and verifies
//! cleanup.
//!
//! Uses `on_systick` (not `on_systick_dynamic`) to avoid the R0 deny-all
//! that blocks handler access.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
#[cfg(target_arch = "arm")]
use kernel::syscall::{SYS_BUF_ALLOC, SYS_BUF_RELEASE, SYS_BUF_WRITE};
use kernel::{
    config::KernelConfig,
    kernel::KernelState,
    mpu,
    mpu_strategy::{DynamicStrategy, MpuStrategy},
    partition::{MpuRegion, PartitionConfig},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
};
use panic_semihosting as _;

const NP: usize = 2;
const STACK_WORDS: usize = 256;
const STACK_SIZE: u32 = (STACK_WORDS * 4) as u32;
const DATA_BASES: [u32; NP] = [0x2000_0000, 0x2000_8000];
const DATA_SIZES: [u32; NP] = [4096, 4096];
const P2: u8 = 1;
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

struct TestConfig;
impl KernelConfig for TestConfig {
    const N: usize = 2;
    const S: usize = 1;
    const SW: usize = 1;
    const MS: usize = 1;
    const MW: usize = 1;
    const QS: usize = 1;
    const QD: usize = 1;
    const QM: usize = 1;
    const QW: usize = 1;
    const SP: usize = 1;
    const SM: usize = 1;
    const BS: usize = 1;
    const BM: usize = 1;
    const BW: usize = 1;
    const BP: usize = 2;
    const BZ: usize = 32;
    const DR: usize = 4;
}

kernel::define_dispatch_hook!(TestConfig);
#[used]
static _SVC: unsafe extern "C" fn(&mut kernel::context::ExceptionFrame) = kernel::svc::SVC_HANDLER;
static KS: Mutex<RefCell<Option<KernelState<NP, 4>>>> = Mutex::new(RefCell::new(None));
static STRATEGY: DynamicStrategy = DynamicStrategy::new();
kernel::define_pendsv_dynamic!(STRATEGY);

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
        // the partition's MPU data region (validated_ptr rejects flash/rodata
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

    // Drive scheduler, configure R4 via DynamicStrategy (skip R0-R3).
    cortex_m::interrupt::free(|cs| {
        let mut ks = KS.borrow(cs).borrow_mut();
        let state = ks.as_mut().expect("KS");
        let event = kernel::tick::on_systick(state);
        if let kernel::scheduler::ScheduleEvent::PartitionSwitch(pid) = event {
            if let Some(pcb) = state.partitions().get(pid as usize) {
                if let Some(regions) = mpu::partition_dynamic_regions(pcb) {
                    let _ = STRATEGY.configure_partition(pid, &regions);
                }
            }
            if let Some(k) = KERN.borrow(cs).borrow_mut().as_mut() {
                k.current_partition = pid;
            }
            // SAFETY: single-core; PendSV cannot preempt SysTick.
            unsafe { core::ptr::write_volatile(&raw mut NEXT_PARTITION, pid as u32) };
        }
    });

    match *TICK {
        4 => {
            // Step 1: P1 should have allocated, written, and released a buffer.
            // Verify P1 signaled readiness and the slot is back to Free.
            let ready = P1_READY.load(Ordering::Acquire) == 1;
            let alloc_slot = P1_ALLOC_SLOT.load(Ordering::Acquire);
            let slot_ok = ready && alloc_slot != u32::MAX;
            if slot_ok {
                *SLOT = alloc_slot as usize;
            }
            let state_ok = slot_ok
                && cortex_m::interrupt::free(|cs| {
                    KERN.borrow(cs)
                        .borrow_mut()
                        .as_mut()
                        .map(|k| {
                            k.buffers.get(*SLOT).unwrap().state()
                                == kernel::buffer_pool::BorrowState::Free
                        })
                        .unwrap_or(false)
                });
            check("step1: p1-alloc-write-release", state_ok, FAIL);
        }
        5 => cortex_m::interrupt::free(|cs| {
            // Step 2: Kernel lends the buffer (with P1's data) read-only to P2.
            // lend_to_partition is a privileged kernel operation with no
            // corresponding user-space syscall.
            if let Some(k) = KERN.borrow(cs).borrow_mut().as_mut() {
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
            }
        }),
        10 => check(
            "step3: p2-read",
            P2_READ_RESULT.load(Ordering::Acquire) == 1,
            FAIL,
        ),
        11 => cortex_m::interrupt::free(|cs| {
            // Step 4: revoke access, verify MPU region disabled.
            if let Some(k) = KERN.borrow(cs).borrow_mut().as_mut() {
                match k.buffers.revoke_from_partition(*SLOT, &STRATEGY) {
                    Ok(()) => {
                        let slot = k.buffers.get(*SLOT).unwrap();
                        let slot_idx = DynamicStrategy::region_to_slot_index(*RID)
                            .expect("RID must map to a valid dynamic slot");
                        let rasr = STRATEGY.compute_region_values()[slot_idx].1;
                        let ok = slot.state() == kernel::buffer_pool::BorrowState::Free
                            && slot.mpu_region().is_none()
                            && STRATEGY.slot(*RID).is_none()
                            && rasr == 0;
                        check("step4: revoke", ok, FAIL);
                    }
                    Err(_) => check("step4: revoke", false, FAIL),
                }
            }
        }),
        12 => {
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
    hprintln!("buffer_pool_test: start");
    // SAFETY: before interrupts; single-core exclusive access.
    unsafe {
        let mut sched = ScheduleTable::<4>::new();
        sched.add(ScheduleEntry::new(0, 2)).unwrap();
        sched.add(ScheduleEntry::new(1, 2)).unwrap();
        sched.start();
        let entries = [
            partition_p1_entry as *const () as u32,
            partition_p2_entry as *const () as u32,
        ];
        let cfgs: [PartitionConfig; NP] = core::array::from_fn(|i| PartitionConfig {
            id: i as u8,
            entry_point: 0,
            stack_base: STACKS[i].0.as_ptr() as u32,
            stack_size: STACK_SIZE,
            mpu_region: MpuRegion::new(DATA_BASES[i], DATA_SIZES[i], 0),
        });
        // TODO: register devices in the registry at init time (backlog item 195).
        store_kernel(Kernel::<TestConfig>::new(
            kernel::virtual_device::DeviceRegistry::new(),
        ));
        // Register partitions in the Kernel struct so that
        // validate_user_ptr can verify SVC pointer arguments.
        // TODO: DRY – PCB construction from PartitionConfig is duplicated here
        // and in KernelState::new (kernel.rs). Add a PartitionTable::init_from_configs
        // helper to share the logic (also affects harness.rs and uart1_loopback.rs).
        cortex_m::interrupt::free(|cs| {
            if let Some(k) = KERN.borrow(cs).borrow_mut().as_mut() {
                for c in &cfgs {
                    let sp = c.stack_base.wrapping_add(c.stack_size);
                    let pcb = kernel::partition::PartitionControlBlock::new(
                        c.id,
                        c.entry_point,
                        c.stack_base,
                        sp,
                        c.mpu_region,
                    );
                    k.partitions
                        .add(pcb)
                        .expect("failed to add partition during test setup");
                }
            }
        });
        cortex_m::interrupt::free(|cs| {
            KS.borrow(cs).replace(Some(
                KernelState::new(sched, &cfgs).expect("invalid kernel config"),
            ));
        });
        for i in 0..NP {
            let stk = &mut STACKS[i].0;
            let ix = kernel::context::init_stack_frame(stk, entries[i], Some(i as u32))
                .expect("init_stack_frame");
            PARTITION_SP[i] = stk.as_ptr() as u32 + (ix as u32) * 4;
        }
        cp.SCB.set_priority(SystemHandler::SVCall, 0x00);
        cp.SCB.set_priority(SystemHandler::PendSV, 0xFF);
        cp.SCB.set_priority(SystemHandler::SysTick, 0xFE);
    }
    // SAFETY: interrupts not yet enabled; PRIVDEFENA keeps privileged default map.
    unsafe { cp.MPU.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
    cp.SYST.set_clock_source(SystClkSource::Core);
    cp.SYST.set_reload(120_000 - 1);
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();
    cortex_m::peripheral::SCB::set_pendsv();
    loop {
        cortex_m::asm::wfi();
    }
}

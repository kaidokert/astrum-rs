//! Dynamic MPU R4 reprogramming across partition context switches.
//!
//! Two partitions with different data regions. At boot the kernel's
//! `DynamicStrategy` is pre-loaded with each partition's (RBAR, RASR) pair
//! for MPU region 4.  On every context switch, PendSV programs R4–R7
//! from the strategy — just before restoring the incoming partition.
//!
//! The SysTick hook reads back MPU R4's RBAR register after each switch
//! and asserts the base address matches the active partition's data region.
//!
//! | Partition | Data region base | Size  |
//! |-----------|------------------|-------|
//! | P0        | 0x2000_0000      | 4 KiB |
//! | P1        | 0x2000_8000      | 4 KiB |
//!
//! # Success criteria
//!
//! Each partition must be independently verified at least 4 times with
//! correct MPU R4 readback.  Prints `02_mpu_dynamic: PASS` and exits
//! with semihosting success.  Any mismatch is a fatal assertion failure.

#![no_std]
#![no_main]
#![allow(incomplete_features, unexpected_cfgs)]
#![feature(generic_const_exprs)]

use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    mpu::{self, RBAR_ADDR_MASK},
    mpu_strategy::MpuStrategy,
    partition::{ExternalPartitionMemory, MpuRegion},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2, PortsTiny, SyncMinimal,
};
use porting_guide::klog;

const NP: usize = 2;
const DATA_BASES: [u32; NP] = [0x2000_0000, 0x2000_8000];
const DATA_SZ: u32 = 4096;
const TARGET_PER_PARTITION: u32 = 4;

kernel::kernel_config!(
    TestConfig < Partitions2,
    SyncMinimal,
    MsgMinimal,
    PortsTiny,
    DebugEnabled > {
        mpu_enforce = true;
    }
);

const _: PartitionEntry = partition_entry;
extern "C" fn partition_entry() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

/// Read (RBAR, RASR) for MPU region `n`.
///
/// # Safety
/// Caller must have exclusive MPU access (no higher-priority RNR writer).
unsafe fn read_mpu_region(mpu: &cortex_m::peripheral::MPU, n: u32) -> (u32, u32) {
    // SAFETY: Caller guarantees exclusive MPU access (no concurrent RNR writer
    // at equal or higher priority). RNR is saved/restored so the global side
    // effect (selecting a region via RNR) is transient and invisible to the caller.
    let saved_rnr = mpu.rnr.read();
    mpu.rnr.write(n);
    let result = (mpu.rbar.read(), mpu.rasr.read());
    mpu.rnr.write(saved_rnr);
    result
}

/// Compute an (RBAR, RASR) pair for a data region at `base` with `size` bytes.
fn data_region_pair(base: u32, size: u32) -> (u32, u32) {
    let rbar = mpu::build_rbar(base, 4).expect("aligned base");
    let sf = mpu::encode_size(size).expect("valid size");
    let rasr = mpu::build_rasr(sf, mpu::AP_FULL_ACCESS, true, (true, true, false));
    (rbar, rasr)
}

/// Build base regions (R0-R3) safe for handler-mode MPU enforcement.
fn handler_safe_base_regions(stack_base: u32, stack_size: u32) -> [(u32, u32); 4] {
    let flash_sf = mpu::encode_size(256 * 1024).expect("256K");
    let r0_rbar = mpu::build_rbar(0x0000_0000, 0).expect("flash base");
    let r0_rasr = mpu::build_rasr(flash_sf, mpu::AP_RO_RO, false, (false, true, false));
    let r1_rbar = mpu::build_rbar(0, 1).expect("r1");
    let stk_sf = mpu::encode_size(stack_size).expect("stack size");
    let r2_rbar = mpu::build_rbar(stack_base, 2).expect("stack base");
    let r2_rasr = mpu::build_rasr(stk_sf, mpu::AP_FULL_ACCESS, true, (true, true, false));
    let r3_rbar = mpu::build_rbar(0, 3).expect("r3");
    [
        (r0_rbar, r0_rasr),
        (r1_rbar, 0),
        (r2_rbar, r2_rasr),
        (r3_rbar, 0),
    ]
}

kernel::define_kernel!(no_boot, TestConfig, |tick, k| {
    use core::sync::atomic::{AtomicU32, Ordering};
    static VERIFIED: [AtomicU32; NP] = [AtomicU32::new(0), AtomicU32::new(0)];

    if tick > 2 {
        let pid = k.current_partition as usize;
        if pid >= NP {
            return;
        }
        let expected = match DATA_BASES.get(pid) {
            Some(&v) => v,
            None => return,
        };
        // SAFETY: SysTick > PendSV priority; steal() is the only way in handler mode.
        let p = unsafe { cortex_m::Peripherals::steal() };
        // SAFETY: No higher-priority handler writes RNR; NMI is unused.
        let (hw_rbar, _) = unsafe { read_mpu_region(&p.MPU, 4) };
        let got_base = hw_rbar & RBAR_ADDR_MASK;

        klog!(
            "verify: p{} R4={:#010x} exp={:#010x} {}",
            pid,
            got_base,
            expected,
            if got_base == expected { "OK" } else { "FAIL" }
        );

        if got_base != expected {
            klog!("02_mpu_dynamic: FAIL R4 base mismatch");
            kernel::kexit!(failure);
        }

        let counter = match VERIFIED.get(pid) {
            Some(v) => v,
            None => return,
        };
        counter.fetch_add(1, Ordering::Relaxed);

        let all_done = VERIFIED
            .iter()
            .all(|v| v.load(Ordering::Relaxed) >= TARGET_PER_PARTITION);
        if all_done {
            klog!(
                "02_mpu_dynamic: PASS (each partition verified {} times)",
                TARGET_PER_PARTITION
            );
            kernel::kexit!(success);
        }
    }

    if tick >= 100 {
        klog!("02_mpu_dynamic: FAIL timeout");
        kernel::kexit!(failure);
    }
});

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().expect("peripherals");
    klog!("02_mpu_dynamic: start");

    let entry_fns: [PartitionEntry; NP] = [partition_entry; NP];

    // Schedule: P0(2 ticks) → system(1) → P1(2 ticks) → system(1)
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched 0");
    sched.add_system_window(1).expect("syswin 0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched 1");
    sched.add_system_window(1).expect("syswin 1");

    // Build partition memories and create kernel.
    // NOTE: `partition_stacks!` returns `&'static mut` (backed by a module-level
    // static), so stacks outlive this block. `Kernel::new` copies config values
    // from `memories`; it does not retain the borrow.
    // TODO: replace raw-pointer `stacks_ptr.add(i)` with safe indexing once
    // the codebase-wide pattern (used in queuing_demo, qemu_pac_timer0_pwm, etc.)
    // is refactored to use split_at_mut or similar.
    let mut k = {
        let stacks = kernel::partition_stacks!(TestConfig, NP);
        let stacks_ptr = stacks.as_mut_ptr();
        let memories: [_; NP] = core::array::from_fn(|i| {
            // SAFETY: i < NP, stacks has NP elements, each index visited once.
            let stk = unsafe { &mut *stacks_ptr.add(i) };
            let spec = PartitionSpec::entry(entry_fns[i]).with_data_mpu(MpuRegion::new(
                DATA_BASES[i],
                DATA_SZ,
                0,
            ));
            ExternalPartitionMemory::from_spec(stk, &spec, kernel::PartitionId::new(i as u32))
                .expect("mem")
        });
        Kernel::<TestConfig>::new(sched, &memories).expect("kernel")
    };
    store_kernel(&mut k);

    // Step 1: Init stack frames for PendSV context switching.
    with_kernel_mut(|k| {
        for i in 0..NP {
            let pcb = k.partitions().get(i).expect("partition");
            let (entry, base, size) = (pcb.entry_point(), pcb.stack_base(), pcb.stack_size());
            // SAFETY: stack region is valid, aligned, exclusively owned pre-interrupt.
            let stack_slice =
                unsafe { core::slice::from_raw_parts_mut(base as *mut u32, (size / 4) as usize) };
            let ix = kernel::context::init_stack_frame(stack_slice, entry, Some(i as u32))
                .expect("init_stack_frame");
            k.set_sp(i, base + (ix as u32) * 4);
            k.sync_stack_limit(i);
        }
    });

    // Step 2: Seal MPU caches with handler-safe base regions.
    with_kernel_mut(|k| {
        for pid in 0..NP {
            let pcb = k.partitions_mut().get_mut(pid).expect("partition");
            let base = handler_safe_base_regions(pcb.stack_base(), pcb.stack_size());
            pcb.set_cached_base_regions(base).expect("set base");
            pcb.set_cached_periph_regions([(0, 0); 3])
                .expect("set periph");
            pcb.seal_cache();
        }
    });

    // Step 3: Configure HARNESS_STRATEGY with each partition's data region for R4.
    for (pid, &base) in DATA_BASES.iter().enumerate() {
        let dyn_region = data_region_pair(base, DATA_SZ);
        HARNESS_STRATEGY
            .configure_partition(kernel::PartitionId::new(pid as u32), &[dyn_region], 0)
            .expect("configure_partition");
    }

    // Step 4: Exception priorities and MemManage enable.
    // SAFETY: Interrupts not yet enabled; no preemption during priority config.
    unsafe {
        cp.SCB
            .set_priority(SystemHandler::SVCall, TestConfig::SVCALL_PRIORITY);
        cp.SCB
            .set_priority(SystemHandler::PendSV, TestConfig::PENDSV_PRIORITY);
        cp.SCB
            .set_priority(SystemHandler::SysTick, TestConfig::SYSTICK_PRIORITY);
    }
    cp.SCB
        .enable(cortex_m::peripheral::scb::Exception::MemoryManagement);

    // Step 5: Start schedule and select first partition.
    with_kernel_mut(|k| {
        let pid = kernel::svc::scheduler::start_schedule(k).expect("start_schedule");
        k.set_next_partition(pid);
    });

    // Step 6: Enable MPU with PRIVDEFENA.
    // SAFETY: Interrupts not yet enabled; PRIVDEFENA preserves privileged default map.
    unsafe { cp.MPU.ctrl.write(mpu::MPU_CTRL_ENABLE_PRIVDEFENA) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // Step 7: Start SysTick and trigger first PendSV.
    cp.SYST.set_clock_source(SystClkSource::Core);
    cp.SYST.set_reload(10_000 - 1);
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();
    cortex_m::peripheral::SCB::set_pendsv();

    loop {
        cortex_m::asm::wfi();
    }
}

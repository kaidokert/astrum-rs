//! Benchmark: context switch overhead with FPU save/restore.
//!
// TODO: benchmark only measures fpu-context=on (FPU-active vs integer-only partitions).
// To fully isolate software save/restore overhead vs hardware lazy-stacking overhead,
// compare results across feature-on and feature-off builds (requires a separate runner script).
//!
//! Two round-robin partitions: A touches FPU (triggers s16-s31 save),
//! B does integer-only work. Throughput ratio isolates FPU overhead.
//!
//! Run:
//!   cargo run -p kernel --target thumbv7em-none-eabihf \
//!     --features qemu,log-semihosting,fpu-context --example bench_context_switch
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

// Ensure this example is only compiled when fpu-context is active.
#[cfg(not(feature = "fpu-context"))]
compile_error!("bench_context_switch requires the `fpu-context` feature");

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    scheduler::ScheduleTable, DebugEnabled, MsgMinimal, PartitionEntry, PartitionSpec, Partitions2,
    PortsTiny, SyncMinimal,
};

kernel::kernel_config!(Cfg<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

#[inline(always)]
fn read_cyccnt() -> u32 {
    // SAFETY: DWT CYCCNT is a read-only memory-mapped register; we use the
    // cortex-m PAC pointer rather than a hardcoded address.
    unsafe { (*cortex_m::peripheral::DWT::PTR).cyccnt.read() }
}

static WORK_FPU: AtomicU32 = AtomicU32::new(0);
static WORK_NOFPU: AtomicU32 = AtomicU32::new(0);
static PREV_CYCCNT: AtomicU32 = AtomicU32::new(0);
static PREV_WORK_FPU: AtomicU32 = AtomicU32::new(0);
static PREV_WORK_NOFPU: AtomicU32 = AtomicU32::new(0);
static CYCLES_SUM: AtomicU32 = AtomicU32::new(0);
static WORK_FPU_SUM: AtomicU32 = AtomicU32::new(0);
static WORK_NOFPU_SUM: AtomicU32 = AtomicU32::new(0);
static HAS_BASELINE: AtomicU32 = AtomicU32::new(0);
static SAMPLES: AtomicU32 = AtomicU32::new(0);
static FPU_ERRORS: AtomicU32 = AtomicU32::new(0);

const REQUIRED_SAMPLES: u32 = 64;
const TIMEOUT_TICKS: u32 = 500;

const SENTINEL_A: u32 = 42.0_f32.to_bits();

kernel::define_kernel!(Cfg, |tick, _k| {
    let now = read_cyccnt();
    let work_fpu = WORK_FPU.load(Ordering::Relaxed);
    let work_nofpu = WORK_NOFPU.load(Ordering::Relaxed);

    let prev_cyc = PREV_CYCCNT.swap(now, Ordering::Relaxed);
    let prev_fpu = PREV_WORK_FPU.swap(work_fpu, Ordering::Relaxed);
    let prev_nofpu = PREV_WORK_NOFPU.swap(work_nofpu, Ordering::Relaxed);

    // Skip the first tick (no baseline yet).
    if HAS_BASELINE.load(Ordering::Relaxed) == 0 {
        HAS_BASELINE.store(1, Ordering::Relaxed);
    } else {
        let cycles = now.wrapping_sub(prev_cyc);
        let delta_fpu = work_fpu.wrapping_sub(prev_fpu);
        let delta_nofpu = work_nofpu.wrapping_sub(prev_nofpu);

        CYCLES_SUM.fetch_add(cycles, Ordering::Relaxed);
        WORK_FPU_SUM.fetch_add(delta_fpu, Ordering::Relaxed);
        WORK_NOFPU_SUM.fetch_add(delta_nofpu, Ordering::Relaxed);

        let n = SAMPLES.fetch_add(1, Ordering::Relaxed) + 1;
        if n >= REQUIRED_SAMPLES {
            let total_cycles = CYCLES_SUM.load(Ordering::Relaxed);
            let total_fpu = WORK_FPU_SUM.load(Ordering::Relaxed);
            let total_nofpu = WORK_NOFPU_SUM.load(Ordering::Relaxed);
            let fpu_errs = FPU_ERRORS.load(Ordering::Relaxed);

            let avg_cycles = total_cycles / n;

            let fpu_per_tick = total_fpu / n;
            let nofpu_per_tick = total_nofpu / n;
            // Ratio >1 means FPU partition is slower (isolates save/restore overhead).
            let ratio_x100 = (nofpu_per_tick * 100)
                .checked_div(fpu_per_tick)
                .unwrap_or(0);

            hprintln!(
                "bench_context_switch: {} samples, avg_cycles/tick={} \
                 fpu_work/tick={} nofpu_work/tick={} \
                 slowdown_ratio={}.{:02}x",
                n,
                avg_cycles,
                fpu_per_tick,
                nofpu_per_tick,
                ratio_x100 / 100,
                ratio_x100 % 100,
            );

            if fpu_errs > 0 {
                hprintln!(
                    "bench_context_switch: FAIL ({} FPU integrity errors)",
                    fpu_errs
                );
                debug::exit(debug::EXIT_FAILURE);
            }

            hprintln!("bench_context_switch: PASS");
            debug::exit(debug::EXIT_SUCCESS);
        }
    }

    if tick >= TIMEOUT_TICKS {
        let n = SAMPLES.load(Ordering::Relaxed);
        hprintln!("bench_context_switch: FAIL timeout ({} samples)", n);
        debug::exit(debug::EXIT_FAILURE);
    }
});

/// Write sentinel to s16 (callee-saved), yield, verify preservation.
#[inline(always)]
#[allow(asm_sub_register)]
fn touch_fpu_and_verify(sentinel_bits: u32, errors: &AtomicU32) {
    // SAFETY: inline asm writes/reads s16; PendSV saves/restores it.
    // s16 is declared as clobbered so the compiler does not assume it is preserved.
    unsafe {
        let readback: u32;
        core::arch::asm!(
            "vmov s16, {val}",
            val = in(reg) sentinel_bits,
            out("s16") _,
            options(nomem, nostack),
        );
        cortex_m::asm::nop();
        core::arch::asm!("vmov {out}, s16", out = out(reg) readback, options(nomem, nostack));
        if readback != sentinel_bits {
            errors.fetch_add(1, Ordering::Relaxed);
        }
    }
}

const _: PartitionEntry = partition_a;
extern "C" fn partition_a() -> ! {
    loop {
        touch_fpu_and_verify(SENTINEL_A, &FPU_ERRORS);
        WORK_FPU.fetch_add(1, Ordering::Relaxed);
    }
}

const _: PartitionEntry = partition_b;
extern "C" fn partition_b() -> ! {
    loop {
        cortex_m::asm::nop();
        WORK_NOFPU.fetch_add(1, Ordering::Relaxed);
    }
}

macro_rules! fail {
    ($($arg:tt)*) => {{
        hprintln!($($arg)*);
        debug::exit(debug::EXIT_FAILURE);
        loop { cortex_m::asm::nop(); }
    }};
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take()
        .unwrap_or_else(|| fail!("bench_context_switch: FAIL (Peripherals::take returned None)"));
    p.DCB.enable_trace();
    p.DWT.enable_cycle_counter();
    hprintln!("bench_context_switch: start (fpu-context enabled, DWT CYCCNT active)");

    let sched = ScheduleTable::<{ Cfg::SCHED }>::round_robin(2, 2)
        .unwrap_or_else(|_| fail!("bench_context_switch: FAIL (schedule table creation failed)"));
    let parts: [PartitionSpec; Cfg::N] = [
        PartitionSpec::new(partition_a as PartitionEntry, 0),
        PartitionSpec::new(partition_b as PartitionEntry, 0),
    ];
    init_kernel(sched, &parts)
        .unwrap_or_else(|_| fail!("bench_context_switch: FAIL (init_kernel failed)"));
    match boot(p) {
        Ok(n) => match n {},
        Err(_) => fail!("bench_context_switch: FAIL (boot failed)"),
    }
}

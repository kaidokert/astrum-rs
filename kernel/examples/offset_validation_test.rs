//! QEMU smoke test: validates that linker-exported PendSV offset constants
//! match `core::mem::offset_of!` values at runtime on the actual ARM target.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::{
    config::KernelConfig, pendsv::LITERAL_POOL_OFFSET_LIMIT, svc::Kernel, DebugEnabled, MsgMinimal,
    Partitions2, PortsTiny, SyncMinimal,
};

kernel::compose_kernel_config!(TestConfig<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

kernel::define_unified_harness!(TestConfig);

type K = Kernel<TestConfig>;
type C = <TestConfig as KernelConfig>::Core;

// TODO: The offset symbols used below (KERNEL_CURRENT_PARTITION_OFFSET, etc.)
// are `#[no_mangle] static usize` items emitted by `define_unified_harness!`.
// They cannot be redeclared via `extern "C"` without causing duplicate-symbol
// errors.  Ideally the macro would export them through an explicit public API
// rather than relying on implicit `#[no_mangle]` visibility.

/// Check one offset: return true if it matches, false otherwise.
fn check_offset(name: &str, linker_val: usize, expected: usize) -> bool {
    if linker_val == expected {
        hprintln!("  OK  {} = {}", name, linker_val);
        true
    } else {
        hprintln!(
            "  FAIL {}: linker={} expected={}",
            name,
            linker_val,
            expected
        );
        false
    }
}

/// Check one offset is below the literal-pool limit.
fn check_limit(name: &str, value: usize) -> bool {
    if value < LITERAL_POOL_OFFSET_LIMIT {
        true
    } else {
        hprintln!(
            "  FAIL {} = {} >= LIMIT {}",
            name,
            value,
            LITERAL_POOL_OFFSET_LIMIT
        );
        false
    }
}

#[entry]
fn main() -> ! {
    hprintln!("offset_validation_test: start");

    // Read the macro-emitted statics via read_volatile to ensure we get
    // the actual linker-resolved values (not const-folded duplicates).
    //
    // SAFETY: Each symbol is a `#[no_mangle] static usize` emitted by
    // `define_unified_harness!` and resolved by the linker to the
    // corresponding struct-field byte offset.  Reading them via
    // `read_volatile` is safe because they are valid, aligned, initialised
    // `usize` values that live for the entire program lifetime.
    let (kcp, kco, ktd, cnp, cps, csl) = unsafe {
        (
            core::ptr::read_volatile(&KERNEL_CURRENT_PARTITION_OFFSET),
            core::ptr::read_volatile(&KERNEL_CORE_OFFSET),
            core::ptr::read_volatile(&KERNEL_TICKS_DROPPED_OFFSET),
            core::ptr::read_volatile(&CORE_NEXT_PARTITION_OFFSET),
            core::ptr::read_volatile(&CORE_PARTITION_SP_OFFSET),
            core::ptr::read_volatile(&CORE_PARTITION_STACK_LIMIT_OFFSET),
        )
    };

    let mut pass = true;

    // 1. Verify all 6 offsets match offset_of!
    pass &= check_offset(
        "KERNEL_CURRENT_PARTITION_OFFSET",
        kcp,
        core::mem::offset_of!(K, current_partition),
    );
    pass &= check_offset("KERNEL_CORE_OFFSET", kco, core::mem::offset_of!(K, core));
    pass &= check_offset(
        "KERNEL_TICKS_DROPPED_OFFSET",
        ktd,
        core::mem::offset_of!(K, ticks_dropped),
    );
    pass &= check_offset(
        "CORE_NEXT_PARTITION_OFFSET",
        cnp,
        core::mem::offset_of!(C, next_partition),
    );
    pass &= check_offset(
        "CORE_PARTITION_SP_OFFSET",
        cps,
        core::mem::offset_of!(C, partition_sp),
    );
    pass &= check_offset(
        "CORE_PARTITION_STACK_LIMIT_OFFSET",
        csl,
        core::mem::offset_of!(C, partition_stack_limits),
    );

    // 2. Verify all offsets are below LITERAL_POOL_OFFSET_LIMIT
    pass &= check_limit("KERNEL_CURRENT_PARTITION_OFFSET", kcp);
    pass &= check_limit("KERNEL_TICKS_DROPPED_OFFSET", ktd);
    pass &= check_limit("KERNEL_CORE_OFFSET", kco);
    pass &= check_limit("KERNEL_CORE_OFFSET + CORE_NEXT_PARTITION_OFFSET", kco + cnp);
    pass &= check_limit("KERNEL_CORE_OFFSET + CORE_PARTITION_SP_OFFSET", kco + cps);
    pass &= check_limit(
        "KERNEL_CORE_OFFSET + CORE_PARTITION_STACK_LIMIT_OFFSET",
        kco + csl,
    );

    if pass {
        hprintln!("offset_validation_test: PASS");
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        hprintln!("offset_validation_test: FAIL");
        debug::exit(debug::EXIT_FAILURE);
    }
    loop {
        cortex_m::asm::nop();
    }
}

// Not migrated to define_unified_harness! — this test requires direct access
// to HwUartBackend in SysTick for software loopback transfer() calls and
// multi-message verification with custom message patterns. The test also
// exercises TX-full and RX-empty edge cases that require coordinated timing
// between SysTick bottom-half processing and partition syscalls, which the
// standard harness's fixed bottom-half sequence doesn't support.

//! UART1 loopback integration test with two partitions on QEMU.
//!
//! P1 (writer) sends multiple messages of varying lengths via HwUartBackend
//! (device_id=2, software loopback). P2 (reader) verifies each message.
//! Also tests read-when-empty (returns 0) and write-when-TX-full (partial
//! write count).

#![no_std]
#![no_main]
#![allow(incomplete_features, clippy::deref_addrof)]
#![feature(generic_const_exprs)]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    hw_uart::HwUartBackend,
    mpu_strategy::DynamicStrategy,
    partition::{MpuRegion, PartitionConfig},
    scheduler::{ScheduleEntry, ScheduleEvent, ScheduleTable},
    svc,
    svc::{Kernel, SvcError, YieldResult},
    syscall::{SYS_DEV_OPEN, SYS_DEV_READ, SYS_DEV_WRITE, SYS_YIELD},
    uart_hal::UartRegs,
    virtual_device::VirtualDevice,
    DebugEnabled, MsgMinimal, Partitions4, PortsTiny, SyncMinimal,
};
use panic_semihosting as _;

const NUM_PARTITIONS: usize = 2;
const STACK_WORDS: usize = DemoConfig::STACK_WORDS;
const STACK_BYTES: u32 = (STACK_WORDS * 4) as u32;
const HW_UART_DEV: u32 = 2;

const MSG_SHORT: &[u8] = b"Hi";
const MSG_MEDIUM: &[u8] = b"Hello from P1";
const MSG_LONG: &[u8] = b"ARINC-653 UART loopback test pattern 1234567890!";
const NUM_MESSAGES: usize = 3;
const MESSAGES: [&[u8]; NUM_MESSAGES] = [MSG_SHORT, MSG_MEDIUM, MSG_LONG];
/// TX ring buffer capacity in HwUartBackend (must match hw_uart.rs CAPACITY).
const TX_CAPACITY: usize = 64;

kernel::compose_kernel_config!(DemoConfig<Partitions4, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

#[repr(C, align(1024))]
struct AlignedStack([u32; STACK_WORDS]);

static mut STACKS: [AlignedStack; NUM_PARTITIONS] = {
    const ZERO: AlignedStack = AlignedStack([0; STACK_WORDS]);
    [ZERO; NUM_PARTITIONS]
};

#[no_mangle]
static mut PARTITION_SP: [u32; NUM_PARTITIONS] = [0; NUM_PARTITIONS];
#[no_mangle]
static mut CURRENT_PARTITION: u32 = u32::MAX;
#[no_mangle]
static mut NEXT_PARTITION: u32 = 0;

struct TestCounters {
    pass: u32,
    fail: u32,
}

static COUNTERS: Mutex<RefCell<TestCounters>> =
    Mutex::new(RefCell::new(TestCounters { pass: 0, fail: 0 }));

fn record(pass: bool, name: &str) {
    if pass {
        hprintln!("  [PASS] {}", name);
    } else {
        hprintln!("  [FAIL] {}", name);
    }
    cortex_m::interrupt::free(|cs| {
        let mut c = COUNTERS.borrow(cs).borrow_mut();
        if pass {
            c.pass += 1;
        } else {
            c.fail += 1;
        }
    });
}

static STRATEGY: DynamicStrategy = DynamicStrategy::new();

/// Run the bottom-half processing for virtual-UART, ISR-ring, and
/// HwUartBackend.  Software loopback is handled internally by
/// `HwUartBackend::drain_tx_to_hw` when loopback mode is enabled.
fn do_bottom_half(k: &mut Kernel<DemoConfig>, current_tick: u64) {
    kernel::tick::run_bottom_half(
        &mut k.uart_pair,
        &mut k.isr_ring,
        &mut k.buffers,
        &mut k.hw_uart,
        current_tick,
        &STRATEGY,
    );
}

// Use define_unified_kernel! with a custom yield handler that runs through
// system windows processing the bottom-half before continuing.
kernel::define_unified_kernel!(DemoConfig, |k| {
    // Yield and advance through system windows (run loopback).
    loop {
        let result = k.yield_current_slot();
        if let Some(pid) = result.partition_id() {
            // SAFETY: single-core Cortex-M — NEXT_PARTITION is only
            // read by PendSV (lower priority), which cannot preempt.
            unsafe { core::ptr::write_volatile(&raw mut NEXT_PARTITION, pid as u32) }
            break;
        }
        // System window or None — run software loopback and advance.
        do_bottom_half(k, k.tick().get());
    }
});

#[used]
static _SVC: unsafe extern "C" fn(&mut kernel::context::ExceptionFrame) = kernel::svc::SVC_HANDLER;

kernel::define_pendsv!();

#[exception]
fn SysTick() {
    with_kernel_mut(|k| {
        let event = k.advance_schedule_tick();
        let current_tick = k.tick().get();
        match event {
            ScheduleEvent::PartitionSwitch(pid) => {
                // SAFETY: single-core Cortex-M — NEXT_PARTITION is only read
                // by PendSV (lower priority), which cannot preempt SysTick.
                unsafe { core::ptr::write_volatile(&raw mut NEXT_PARTITION, pid as u32) }
                cortex_m::peripheral::SCB::set_pendsv();
            }
            ScheduleEvent::SystemWindow => {
                do_bottom_half(k, current_tick);
            }
            ScheduleEvent::Idle | ScheduleEvent::None => {}
        }
        k.expire_timed_waits::<{ <DemoConfig as kernel::config::KernelConfig>::N }>(current_tick);
    });
}

fn boot(partitions: &[(extern "C" fn() -> !, u32)], peripherals: &mut cortex_m::Peripherals) -> ! {
    use cortex_m::peripheral::scb::SystemHandler;
    use cortex_m::peripheral::syst::SystClkSource;
    use cortex_m::peripheral::SCB;
    // SAFETY: called once from main before the scheduler starts (interrupts
    // disabled). STACKS, PARTITION_SP are only written here and read later
    // by PendSV; no concurrent access is possible at this point.
    // Exception priorities are set via the valid SCB peripheral reference.
    unsafe {
        let stacks = &mut *(&raw mut STACKS);
        let partition_sp = &mut *(&raw mut PARTITION_SP);
        for (i, &(ep, hint)) in partitions.iter().enumerate() {
            let stk = &mut stacks[i].0;
            let ix = kernel::context::init_stack_frame(stk, ep as *const () as u32, Some(hint))
                .expect("init_stack_frame");
            partition_sp[i] = stk.as_ptr() as u32 + (ix as u32) * 4;
        }
        const { kernel::config::assert_priority_order::<DemoConfig>() }
        peripherals
            .SCB
            .set_priority(SystemHandler::SVCall, DemoConfig::SVCALL_PRIORITY);
        peripherals
            .SCB
            .set_priority(SystemHandler::PendSV, DemoConfig::PENDSV_PRIORITY);
        peripherals
            .SCB
            .set_priority(SystemHandler::SysTick, DemoConfig::SYSTICK_PRIORITY);
    }
    peripherals.SYST.set_clock_source(SystClkSource::Core);
    peripherals
        .SYST
        .set_reload(kernel::config::compute_systick_reload(12_000_000, 10_000));
    peripherals.SYST.clear_current();
    peripherals.SYST.enable_counter();
    peripherals.SYST.enable_interrupt();
    SCB::set_pendsv();
    loop {
        cortex_m::asm::wfi();
    }
}

/// Helper: write `len` bytes from `buf` to the device, return bytes written.
fn dev_write(buf: &[u8], len: usize) -> u32 {
    svc!(SYS_DEV_WRITE, HW_UART_DEV, len as u32, buf.as_ptr() as u32)
}

/// Helper: read from the device into `buf`, return bytes read (or error).
fn dev_read(buf: &mut [u8]) -> u32 {
    svc!(
        SYS_DEV_READ,
        HW_UART_DEV,
        buf.len() as u32,
        buf.as_mut_ptr() as u32
    )
}

extern "C" fn p1_main() -> ! {
    // --- Open the device ---
    let rc = svc!(SYS_DEV_OPEN, HW_UART_DEV, 0u32, 0u32);
    if SvcError::is_error(rc) {
        record(false, "P1: DEV_OPEN");
        finish();
    }

    // --- Test: read-when-empty (RX buffer should be empty at start) ---
    hprintln!("[P1] test: read-when-empty");
    let mut rx_buf = [0u8; 16];
    let n = dev_read(&mut rx_buf);
    if !SvcError::is_error(n) && n == 0 {
        record(true, "P1: read-when-empty returns 0");
    } else {
        record(false, "P1: read-when-empty (expected 0)");
    }

    // Yield without writing — lets P2 test read-when-empty on a clean RX.
    svc!(SYS_YIELD, 0u32, 0u32, 0u32);

    // --- Send each message, yielding after each to let the system window
    //     drain TX→RX via software loopback. ---
    let mut tx_buf = [0u8; 64];
    for (i, msg) in MESSAGES.iter().enumerate() {
        tx_buf[..msg.len()].copy_from_slice(msg);
        let written = dev_write(&tx_buf, msg.len());
        hprintln!("[P1] msg{}: wrote {}/{} bytes", i, written, msg.len());
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }

    // --- Test: write-when-TX-full ---
    // Fill the TX buffer to capacity with a known pattern.
    hprintln!("[P1] test: write-when-TX-full");
    let fill_data = [0xABu8; TX_CAPACITY];
    tx_buf = [0u8; 64];
    tx_buf[..TX_CAPACITY].copy_from_slice(&fill_data);
    let n_fill = svc!(
        SYS_DEV_WRITE,
        HW_UART_DEV,
        TX_CAPACITY as u32,
        tx_buf.as_ptr() as u32
    );
    // Now attempt to write one more byte — should return 0 (partial write).
    let mut overflow_buf = [0xFFu8; 1];
    let n_over = svc!(
        SYS_DEV_WRITE,
        HW_UART_DEV,
        1u32,
        overflow_buf.as_mut_ptr() as u32
    );
    if !SvcError::is_error(n_fill)
        && n_fill == TX_CAPACITY as u32
        && !SvcError::is_error(n_over)
        && n_over == 0
    {
        record(true, "P1: write-when-TX-full (partial write = 0)");
    } else {
        hprintln!(
            "[P1] TX-full: fill={}, overflow={} (expected {}, 0)",
            n_fill,
            n_over,
            TX_CAPACITY
        );
        record(false, "P1: write-when-TX-full");
    }

    // Yield to let the system window drain, then idle.
    loop {
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
}

extern "C" fn p2_main() -> ! {
    // --- Open the device ---
    let rc = svc!(SYS_DEV_OPEN, HW_UART_DEV, 0u32, 0u32);
    if SvcError::is_error(rc) {
        record(false, "P2: DEV_OPEN");
        finish();
    }

    // --- Test: read-when-empty (P1 hasn't written yet on first P2 slot) ---
    hprintln!("[P2] test: read-when-empty");
    let mut buf = [0u8; 64];
    let n = dev_read(&mut buf);
    if !SvcError::is_error(n) && n == 0 {
        record(true, "P2: read-when-empty returns 0");
    } else {
        record(false, "P2: read-when-empty (expected 0)");
    }

    // Yield so P1 gets its first timeslot to write message 0.
    svc!(SYS_YIELD, 0u32, 0u32, 0u32);

    // --- Verify each message ---
    for (i, expected) in MESSAGES.iter().enumerate() {
        // After P1 writes and the system window drains, data is in RX.
        // Read in a polling loop (may need a couple of yields if the
        // system window hasn't drained yet).
        // NOTE: The retry limit of 10 is arbitrary and not derived from a
        // system timing guarantee. It works in practice because the
        // schedule table gives P1 enough slots to write before P2 polls,
        // but a schedule change could break this assumption.
        let mut total = 0usize;
        buf = [0u8; 64];
        for _attempt in 0..10 {
            let n = dev_read(&mut buf[total..]);
            if SvcError::is_error(n) {
                break;
            }
            total += n as usize;
            if total >= expected.len() {
                break;
            }
            svc!(SYS_YIELD, 0u32, 0u32, 0u32);
        }
        let ok = total == expected.len() && buf[..total] == **expected;
        let label = match i {
            0 => "msg0: short (2B)",
            1 => "msg1: medium (13B)",
            _ => "msg2: long (48B)",
        };
        if ok {
            record(true, label);
        } else {
            hprintln!(
                "[P2] {}: got {} bytes, expected {}",
                label,
                total,
                expected.len()
            );
            record(false, label);
        }
        // Yield to let P1 send the next message.
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }

    // --- Drain the TX-full test data (64 bytes of 0xAB) ---
    // P1 filled TX with 0xAB bytes; after system window drain they're in RX.
    let mut drain_buf = [0u8; TX_CAPACITY];
    let mut drained = 0usize;
    // NOTE: Same arbitrary retry limit; see comment above.
    for _attempt in 0..10 {
        let n = dev_read(&mut drain_buf[drained..]);
        if SvcError::is_error(n) {
            break;
        }
        drained += n as usize;
        if drained >= TX_CAPACITY {
            break;
        }
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
    if drained == TX_CAPACITY && drain_buf.iter().all(|&b| b == 0xAB) {
        record(true, "P2: TX-full data received and verified");
    } else {
        hprintln!(
            "[P2] TX-full drain: got {} bytes (expected {})",
            drained,
            TX_CAPACITY
        );
        record(false, "P2: TX-full data verification");
    }

    finish();
}

fn finish() -> ! {
    let (pass, fail) = cortex_m::interrupt::free(|cs| {
        let c = COUNTERS.borrow(cs).borrow();
        (c.pass, c.fail)
    });
    hprintln!("--- Results: {} passed, {} failed ---", pass, fail);
    if fail == 0 {
        hprintln!("UART loopback test: PASS");
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        hprintln!("UART loopback test: FAIL");
        debug::exit(debug::EXIT_FAILURE);
    }
    loop {
        cortex_m::asm::wfi();
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();
    hprintln!("uart1_loopback: start");

    // Build schedule: P1(3) → system window(1) → P2(3) → system window(1)
    let mut sched = ScheduleTable::<{ DemoConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 3)).unwrap();
    if sched.add_system_window(1).is_err() {
        loop {
            debug::exit(debug::EXIT_FAILURE);
        }
    }
    sched.add(ScheduleEntry::new(1, 3)).unwrap();
    if sched.add_system_window(1).is_err() {
        loop {
            debug::exit(debug::EXIT_FAILURE);
        }
    }

    // Build partition configs
    // SAFETY: called once before the scheduler starts (interrupts disabled,
    // single-core). STACKS are only written here; no concurrent access.
    let cfgs: [PartitionConfig; NUM_PARTITIONS] = unsafe {
        let stacks_ref = &*(&raw const STACKS);
        core::array::from_fn(|i| {
            let base = stacks_ref[i].0.as_ptr() as u32;
            PartitionConfig {
                id: i as u8,
                entry_point: 0,
                stack_base: base,
                stack_size: STACK_BYTES,
                mpu_region: MpuRegion::new(base, STACK_BYTES, 0),
                peripheral_regions: heapless::Vec::new(),
            }
        })
    };

    // Create unified kernel with schedule and partitions
    let mut kern =
        Kernel::<DemoConfig>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new())
            .expect("kernel creation");

    // Set up HW UART backend with software loopback
    let regs = UartRegs::new(0x4000_D000);
    regs.init(115_200, 12_000_000);
    let mut hw_backend = HwUartBackend::new(HW_UART_DEV as u8, regs);
    hw_backend.set_loopback(true);
    kern.set_hw_uart(hw_backend);

    // Store kernel and register device backends
    store_kernel(kern);

    with_kernel_mut(|k| {
        // SAFETY: Kernel state is stored in a 'static global
        // (UNIFIED_KERNEL_STORAGE). The backends live inside the kernel
        // (uart_pair and hw_uart fields). with_kernel_mut runs inside
        // interrupt::free, guaranteeing exclusive access on single-core
        // Cortex-M. The 'static lifetime is valid because the storage
        // is 'static.
        unsafe {
            let a: &'static mut dyn VirtualDevice = &mut *(&mut k.uart_pair.a as *mut _);
            let b: &'static mut dyn VirtualDevice = &mut *(&mut k.uart_pair.b as *mut _);
            k.registry.add(a).expect("register UART-A");
            k.registry.add(b).expect("register UART-B");
            if let Some(hw) = k.hw_uart.as_mut() {
                let hw: &'static mut dyn VirtualDevice = &mut *(hw as *mut HwUartBackend as *mut _);
                k.registry.add(hw).expect("register HW UART");
            }
        }
    });

    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(p1_main, 0), (p2_main, 0)];
    #[allow(clippy::diverging_sub_expression, unreachable_code)]
    {
        let _result: Result<kernel::harness::Never, kernel::harness::BootError> =
            boot(&parts, &mut p);
        match _result {
            Ok(never) => match never {},
            Err(_e) => loop {
                cortex_m::asm::wfi();
            },
        }
    }
}

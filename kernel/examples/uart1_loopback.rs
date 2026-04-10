// Not migrated to define_kernel! — this test requires direct access
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
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    hw_uart::HwUartBackend,
    mpu_strategy::{DynamicStrategy, MpuStrategy},
    partition::{ExternalPartitionMemory, MpuRegion},
    scheduler::{ScheduleEntry, ScheduleEvent, ScheduleTable},
    svc::{Kernel, YieldResult},
    uart_hal::UartRegs,
    virtual_device::VirtualDevice,
    DebugEnabled, MsgMinimal, PartitionEntry, Partitions4, PortsTiny, SyncMinimal,
};

const NUM_PARTITIONS: usize = 2;
const STACK_WORDS: usize = 256;
const STACK_BYTES: u32 = (STACK_WORDS * 4) as u32;
const HW_UART_DEV: u32 = 2;

const MSG_SHORT: &[u8] = b"Hi";
const MSG_MEDIUM: &[u8] = b"Hello from P1";
const MSG_LONG: &[u8] = b"ARINC-653 UART loopback test pattern 1234567890!";
const NUM_MESSAGES: usize = 3;
const MESSAGES: [&[u8]; NUM_MESSAGES] = [MSG_SHORT, MSG_MEDIUM, MSG_LONG];
/// TX ring buffer capacity in HwUartBackend (must match hw_uart.rs CAPACITY).
const TX_CAPACITY: usize = 64;

kernel::kernel_config!(DemoConfig<Partitions4, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled>);

#[repr(C, align(1024))]
struct AlignedStack([u32; STACK_WORDS]);

static mut STACKS: [AlignedStack; NUM_PARTITIONS] = {
    const ZERO: AlignedStack = AlignedStack([0; STACK_WORDS]);
    [ZERO; NUM_PARTITIONS]
};

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
/// device registry.  Software loopback is handled internally by
/// `HwUartBackend::drain_tx_to_hw` when loopback mode is enabled.
fn do_bottom_half(k: &mut Kernel<'_, DemoConfig>, current_tick: u64) {
    kernel::tick::run_bottom_half(
        &mut k.uart_pair,
        &mut k.isr_ring,
        &mut k.buffers,
        &mut k.registry,
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
            k.set_next_partition(pid);
            break;
        }
        // System window or None — run software loopback and advance.
        do_bottom_half(k, k.tick().get());
    }
});

#[used]
static _SVC: kernel::SvcDispatchFn = kernel::svc::SVC_HANDLER;

kernel::define_pendsv!(dynamic: STRATEGY, DemoConfig);

#[exception]
fn SysTick() {
    with_kernel_mut(|k| {
        // TODO: svc_scheduler→svc::scheduler rename is an out-of-scope refactor;
        // should be split into a separate PR if not required for this migration.
        let event = kernel::svc::scheduler::advance_schedule_tick(k);
        let current_tick = k.tick().get();
        match event {
            ScheduleEvent::PartitionSwitch(pid) => {
                if let Some(pcb) = k.partitions().get(pid as usize) {
                    let dyn_region = pcb.cached_dynamic_region();
                    STRATEGY
                        .configure_partition(kernel::PartitionId::new(pid as u32), &[dyn_region], 0)
                        .expect("configure_partition");
                }
                k.set_next_partition(pid);
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

fn boot(
    mut peripherals: cortex_m::Peripherals,
) -> Result<kernel::harness::Never, kernel::harness::BootError> {
    use cortex_m::peripheral::scb::SystemHandler;
    use cortex_m::peripheral::syst::SystClkSource;
    use cortex_m::peripheral::SCB;

    // Initialize stack frames, seal MPU caches, and start the schedule.
    with_kernel_mut(|k| {
        let n = k.partitions().len();
        for i in 0..n {
            let pcb = k.partitions_mut().get_mut(i).expect("partition PCB");
            kernel::mpu::precompute_mpu_cache(pcb).expect("precompute_mpu_cache");

            let entry = pcb.entry_point();
            let base = pcb.stack_base();
            let size = pcb.stack_size();
            let word_count = (size / 4) as usize;

            // SAFETY: PCBs hold valid stack memory pointers set up from
            // ExternalPartitionMemory; called once before scheduler starts.
            let stack_slice =
                unsafe { core::slice::from_raw_parts_mut(base as *mut u32, word_count) };
            let ix = kernel::context::init_stack_frame(stack_slice, entry, Some(pcb.r0_hint()))
                .expect("init_stack_frame");
            let sp = base + (ix as u32) * 4;
            k.set_sp(i, sp);
        }

        // Start the schedule and select the first partition for PendSV.
        if let Some(pid) = kernel::svc::scheduler::start_schedule(k) {
            // Configure the dynamic MPU strategy for the first partition
            // before the initial PendSV fires.
            if let Some(pcb) = k.partitions().get(pid as usize) {
                let dyn_region = pcb.cached_dynamic_region();
                STRATEGY
                    .configure_partition(kernel::PartitionId::new(pid as u32), &[dyn_region], 0)
                    .expect("configure_partition");
            }
            k.set_next_partition(pid);
        }
    });

    // SAFETY: called once from main before the scheduler starts.
    unsafe {
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
    // SAFETY: Interrupts not yet enabled. PRIVDEFENA ensures privileged
    // code retains a default memory map.
    unsafe {
        peripherals
            .MPU
            .ctrl
            .write(kernel::mpu::MPU_CTRL_ENABLE_PRIVDEFENA)
    };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

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

const DEV: plib::DeviceId = plib::DeviceId::new(HW_UART_DEV as u8);

const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    // --- Open the device ---
    if plib::sys_dev_open(DEV).is_err() {
        record(false, "P1: DEV_OPEN");
        finish();
    }

    // --- Test: read-when-empty (RX buffer should be empty at start) ---
    hprintln!("[P1] test: read-when-empty");
    let mut rx_buf = [0u8; 16];
    match plib::sys_dev_read(DEV, &mut rx_buf) {
        Ok(0) => record(true, "P1: read-when-empty returns 0"),
        _ => record(false, "P1: read-when-empty (expected 0)"),
    }

    // Yield without writing — lets P2 test read-when-empty on a clean RX.
    let _ = plib::sys_yield();

    // --- Send each message, yielding after each to let the system window
    //     drain TX→RX via software loopback. ---
    let mut tx_buf = [0u8; 64];
    for (i, msg) in MESSAGES.iter().enumerate() {
        tx_buf[..msg.len()].copy_from_slice(msg);
        let written = plib::sys_dev_write(DEV, &tx_buf[..msg.len()]).unwrap_or(0);
        hprintln!("[P1] msg{}: wrote {}/{} bytes", i, written, msg.len());
        let _ = plib::sys_yield();
    }

    // --- Test: write-when-TX-full ---
    // Fill the TX buffer to capacity with a known pattern.
    hprintln!("[P1] test: write-when-TX-full");
    let fill_data = [0xABu8; TX_CAPACITY];
    tx_buf = [0u8; 64];
    tx_buf[..TX_CAPACITY].copy_from_slice(&fill_data);
    let n_fill = plib::sys_dev_write(DEV, &tx_buf[..TX_CAPACITY]);
    // Now attempt to write one more byte — should return 0 (partial write).
    let overflow_buf = [0xFFu8; 1];
    let n_over = plib::sys_dev_write(DEV, &overflow_buf);
    match (n_fill, n_over) {
        (Ok(fill), Ok(0)) if fill == TX_CAPACITY as u32 => {
            record(true, "P1: write-when-TX-full (partial write = 0)");
        }
        _ => {
            hprintln!(
                "[P1] TX-full: fill={:?}, overflow={:?} (expected {}, 0)",
                n_fill,
                n_over,
                TX_CAPACITY
            );
            record(false, "P1: write-when-TX-full");
        }
    }

    // Yield to let the system window drain, then idle.
    loop {
        let _ = plib::sys_yield();
    }
}

const _: PartitionEntry = p2_main;
extern "C" fn p2_main() -> ! {
    // --- Open the device ---
    if plib::sys_dev_open(DEV).is_err() {
        record(false, "P2: DEV_OPEN");
        finish();
    }

    // --- Test: read-when-empty (P1 hasn't written yet on first P2 slot) ---
    hprintln!("[P2] test: read-when-empty");
    let mut buf = [0u8; 64];
    match plib::sys_dev_read(DEV, &mut buf) {
        Ok(0) => record(true, "P2: read-when-empty returns 0"),
        _ => record(false, "P2: read-when-empty (expected 0)"),
    }

    // Yield so P1 gets its first timeslot to write message 0.
    let _ = plib::sys_yield();

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
            match plib::sys_dev_read(DEV, &mut buf[total..]) {
                Ok(n) => {
                    total += n as usize;
                    if total >= expected.len() {
                        break;
                    }
                }
                Err(_) => break,
            }
            let _ = plib::sys_yield();
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
        let _ = plib::sys_yield();
    }

    // --- Drain the TX-full test data (64 bytes of 0xAB) ---
    // P1 filled TX with 0xAB bytes; after system window drain they're in RX.
    let mut drain_buf = [0u8; TX_CAPACITY];
    let mut drained = 0usize;
    // NOTE: Same arbitrary retry limit; see comment above.
    for _attempt in 0..10 {
        match plib::sys_dev_read(DEV, &mut drain_buf[drained..]) {
            Ok(n) => {
                drained += n as usize;
                if drained >= TX_CAPACITY {
                    break;
                }
            }
            Err(_) => break,
        }
        let _ = plib::sys_yield();
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
    let p = cortex_m::Peripherals::take().unwrap();
    hprintln!(
        "=== {} v{} ===",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );
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

    // Build partition memories and create kernel
    let mut kern = {
        // SAFETY: called once before the scheduler starts (interrupts disabled,
        // single-core). STACKS are only written here; no concurrent access.
        let ptr = &raw mut STACKS;
        let stacks: &mut [AlignedStack; NUM_PARTITIONS] = unsafe { &mut *ptr };
        let [ref mut s0, ref mut s1] = *stacks;
        let base0 = s0.0.as_ptr() as u32;
        let base1 = s1.0.as_ptr() as u32;
        // Code region: LM3S6965 flash starts at 0x0000_0000, 256 KiB.
        let code_region = MpuRegion::new(0x0000_0000, 256 * 1024, 0);
        let memories = [
            ExternalPartitionMemory::new(
                &mut s0.0,
                p1_main as PartitionEntry,
                MpuRegion::new(base0, STACK_BYTES, 0),
                kernel::PartitionId::new(0),
            )
            .expect("ext mem")
            .with_code_mpu_region(code_region)
            .expect("code region"),
            ExternalPartitionMemory::new(
                &mut s1.0,
                p2_main as PartitionEntry,
                MpuRegion::new(base1, STACK_BYTES, 0),
                kernel::PartitionId::new(1),
            )
            .expect("ext mem")
            .with_code_mpu_region(code_region)
            .expect("code region"),
        ];
        Kernel::<DemoConfig>::new(sched, &memories).expect("kernel creation")
    };

    // Set up HW UART backend with software loopback.
    // SAFETY: 0x4000_D000 is the UART1 base address on LM3S6965; the
    // peripheral region is mapped and we have exclusive access before
    // interrupts are enabled.
    let regs = unsafe { UartRegs::from_base(0x4000_D000) };
    regs.init(115_200, 12_000_000);
    // SAFETY: HW_UART_BACKEND is written once here before interrupts are
    // enabled and the scheduler starts. All subsequent accesses go through
    // the device registry which borrows it as &'static mut.
    static mut HW_UART_BACKEND: Option<HwUartBackend> = None;
    let hw_ref: &'static mut HwUartBackend = unsafe {
        let ptr = &raw mut HW_UART_BACKEND;
        let mut backend = HwUartBackend::new(HW_UART_DEV as u8, regs);
        backend.set_loopback(true);
        (*ptr) = Some(backend);
        (*ptr).as_mut().unwrap()
    };

    // Store kernel and register device backends
    store_kernel(&mut kern);

    with_kernel_mut(|k| {
        // SAFETY: Kernel state lives on the caller's stack in a -> !
        // function, so it is effectively 'static. The backends live inside
        // the kernel (uart_pair) or a static (HW_UART_BACKEND).
        // with_kernel_mut runs inside interrupt::free, guaranteeing
        // exclusive access on single-core Cortex-M.
        unsafe {
            let a: &'static mut dyn VirtualDevice = &mut *(&mut k.uart_pair.a as *mut _);
            let b: &'static mut dyn VirtualDevice = &mut *(&mut k.uart_pair.b as *mut _);
            k.registry.add(a).expect("register UART-A");
            k.registry.add(b).expect("register UART-B");
            let hw: &'static mut dyn VirtualDevice = &mut *(hw_ref as *mut HwUartBackend as *mut _);
            k.registry.add(hw).expect("register HW UART");
        }
    });

    #[allow(clippy::diverging_sub_expression, unreachable_code)]
    {
        let _result: Result<kernel::harness::Never, kernel::harness::BootError> = boot(p);
        match _result {
            Ok(never) => match never {},
            Err(_e) => loop {
                cortex_m::asm::wfi();
            },
        }
    }
}

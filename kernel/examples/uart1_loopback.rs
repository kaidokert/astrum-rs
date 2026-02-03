//! UART1 loopback integration test with two partitions on QEMU.
//!
//! P1 (writer) sends multiple messages of varying lengths via HwUartBackend
//! (device_id=2, software loopback). P2 (reader) verifies each message.
//! Also tests read-when-empty (returns 0) and write-when-TX-full (partial
//! write count).

#![no_std]
#![no_main]
#![allow(incomplete_features, static_mut_refs)]
#![feature(generic_const_exprs)]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    config::KernelConfig,
    hw_uart::HwUartBackend,
    kernel::KernelState,
    partition::{MpuRegion, PartitionConfig, PartitionControlBlock},
    scheduler::{ScheduleEntry, ScheduleEvent, ScheduleTable},
    svc,
    svc::{Kernel, SvcError},
    syscall::{SYS_DEV_OPEN, SYS_DEV_READ, SYS_DEV_WRITE, SYS_YIELD},
    uart_hal::UartRegs,
    virtual_device::VirtualDevice,
};
use panic_semihosting as _;

const NUM_PARTITIONS: usize = 2;
const MAX_SCHEDULE_ENTRIES: usize = 8;
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

struct DemoConfig;
impl KernelConfig for DemoConfig {
    const N: usize = 4;
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
    const BP: usize = 1;
    const BZ: usize = 32;
    const DR: usize = 4;
}

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

/// Run the bottom-half processing for virtual-UART, ISR-ring, and
/// HwUartBackend.  Software loopback is handled internally by
/// `HwUartBackend::drain_tx_to_hw` when loopback mode is enabled.
fn do_bottom_half(k: &mut Kernel<DemoConfig>) {
    kernel::tick::run_bottom_half(
        &mut k.uart_pair,
        &mut k.isr_ring,
        &mut k.buffers,
        &mut k.hw_uart,
    );
}

kernel::define_dispatch_hook!(DemoConfig, |k| {
    // SAFETY: single-core Cortex-M — SVC (priority 0x00) has exclusive
    // access to KS; PendSV (lower priority) cannot preempt us.
    if let Some(ks) = unsafe { KS.as_mut() } {
        use kernel::kernel::YieldResult;
        // Yield the current slot and advance through any system windows,
        // running the software loopback for each one encountered.
        loop {
            let result = ks.yield_current_slot();
            if let Some(pid) = result.partition_id() {
                // SAFETY: single-core Cortex-M — NEXT_PARTITION is only
                // read by PendSV (lower priority), which cannot preempt.
                unsafe { core::ptr::write_volatile(&raw mut NEXT_PARTITION, pid as u32) }
                break;
            }
            // System window or None — run software loopback and advance.
            do_bottom_half(k);
        }
    }
});

static mut KS: Option<KernelState<{ <DemoConfig as KernelConfig>::N }, MAX_SCHEDULE_ENTRIES>> =
    None;

#[used]
static _SVC: unsafe extern "C" fn(&mut kernel::context::ExceptionFrame) = kernel::svc::SVC_HANDLER;

kernel::define_pendsv!();

#[exception]
fn SysTick() {
    // SAFETY: single-core Cortex-M — SysTick has exclusive access to KS
    // because higher-priority interrupts do not touch it, and PendSV
    // (lower priority) cannot preempt us.
    let event = unsafe { KS.as_mut() }.expect("KS").advance_schedule_tick();
    match event {
        ScheduleEvent::PartitionSwitch(pid) => {
            // SAFETY: single-core Cortex-M — NEXT_PARTITION is only read
            // by PendSV (lower priority), which cannot preempt SysTick.
            unsafe { core::ptr::write_volatile(&raw mut NEXT_PARTITION, pid as u32) }
            cortex_m::peripheral::SCB::set_pendsv();
        }
        ScheduleEvent::SystemWindow => {
            cortex_m::interrupt::free(|cs| {
                if let Some(k) = KERN.borrow(cs).borrow_mut().as_mut() {
                    do_bottom_half(k);
                }
            });
        }
        ScheduleEvent::None => {}
    }
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
        let stacks = &mut STACKS;
        let partition_sp = &mut PARTITION_SP;
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
    peripherals.SYST.set_reload(120_000 - 1);
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

    // SAFETY: called once before the scheduler starts (interrupts disabled,
    // single-core). STACKS, KS are only written here; no concurrent access.
    // UartRegs::new and init use a valid MMIO base for UART1 on LM3S6965.
    // HwUartBackend::new is safe given a valid UartRegs; set_loopback is a
    // plain field write with no unsafe invariants.
    unsafe {
        let regs = UartRegs::new(0x4000_D000);
        regs.init(115_200, 12_000_000);
        let mut hw_backend = HwUartBackend::new(HW_UART_DEV as u8, regs);
        hw_backend.set_loopback(true);

        let stacks_ref = &STACKS;
        let bases: [u32; NUM_PARTITIONS] =
            core::array::from_fn(|i| stacks_ref[i].0.as_ptr() as u32);

        // Populate the Kernel's partition table for pointer validation.
        let mut kern = Kernel::<DemoConfig>::new(kernel::virtual_device::DeviceRegistry::new());
        for (i, &base) in bases.iter().enumerate() {
            let region = MpuRegion::new(base, STACK_BYTES, 0);
            let pcb = PartitionControlBlock::new(i as u8, 0, base, base + STACK_BYTES, region);
            kern.partitions.add(pcb).ok();
        }
        kern.set_hw_uart(hw_backend);
        store_kernel(kern);

        // Register the uart_pair and hw_uart backends in the device
        // registry so that dev_dispatch routes SYS_DEV_* syscalls to them.
        cortex_m::interrupt::free(|cs| {
            if let Some(k) = KERN.borrow(cs).borrow_mut().as_mut() {
                // SAFETY: KERN is a static that is never dropped. The
                // backends live inside KERN (uart_pair and hw_uart
                // fields). Interrupts are disabled (interrupt::free),
                // guaranteeing exclusive access on single-core Cortex-M.
                // The 'static lifetime is valid because KERN is 'static.
                let a: &'static mut dyn VirtualDevice =
                    &mut *(&mut k.uart_pair.a as *mut _);
                let b: &'static mut dyn VirtualDevice =
                    &mut *(&mut k.uart_pair.b as *mut _);
                k.registry.add(a).expect("register UART-A");
                k.registry.add(b).expect("register UART-B");
                if let Some(hw) = k.hw_uart.as_mut() {
                    let hw: &'static mut dyn VirtualDevice =
                        &mut *(hw as *mut HwUartBackend as *mut _);
                    k.registry.add(hw).expect("register HW UART");
                }
            }
        });

        // Schedule: P1(3) → system window(1) → P2(3) → system window(1)
        let mut sched = ScheduleTable::<MAX_SCHEDULE_ENTRIES>::new();
        sched.add(ScheduleEntry::new(0, 3)).unwrap();
        sched.add_system_window(1).unwrap();
        sched.add(ScheduleEntry::new(1, 3)).unwrap();
        sched.add_system_window(1).unwrap();
        sched.start();

        let cfgs: [PartitionConfig; NUM_PARTITIONS] = core::array::from_fn(|i| PartitionConfig {
            id: i as u8,
            entry_point: 0,
            stack_base: bases[i],
            stack_size: STACK_BYTES,
            mpu_region: MpuRegion::new(bases[i], STACK_BYTES, 0),
        });
        KS = Some(KernelState::new(sched, &cfgs).expect("invalid kernel config"));
    }

    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(p1_main, 0), (p2_main, 0)];
    boot(&parts, &mut p)
}

//! UART1 TX/RX loopback demo with two partitions on QEMU.
//! P1 writes via HwUartBackend (device_id=2, base 0x4000_D000), the system
//! window drains TX and injects the data back into RX, then P2 reads and verifies.

#![no_std]
#![no_main]
#![allow(incomplete_features, static_mut_refs)]
#![feature(generic_const_exprs)]

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
};
use panic_semihosting as _;

const NUM_PARTITIONS: usize = 2;
const MAX_SCHEDULE_ENTRIES: usize = 8;
const STACK_WORDS: usize = 256;
const STACK_BYTES: u32 = (STACK_WORDS * 4) as u32;
const HW_UART_DEV: u32 = 2;
const MSG: &[u8] = b"Hello from P1";

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

/// Software loopback: drain TX ring into a local buffer and inject
/// that data back into RX, then run the normal bottom-half for
/// virtual-UART and ISR-ring processing.
fn do_loopback(k: &mut Kernel<DemoConfig>) {
    // Capture TX bytes before run_bottom_half drains them to hardware.
    let mut buf = [0u8; 64];
    let n = k.hw_uart.as_mut().map_or(0, |hw| hw.drain_tx(&mut buf));

    kernel::tick::run_bottom_half(
        &mut k.uart_pair,
        &mut k.isr_ring,
        &mut k.buffers,
        &mut k.hw_uart,
    );

    // Inject the captured TX data into RX (software loopback).
    if n > 0 {
        if let Some(hw) = k.hw_uart.as_mut() {
            hw.push_rx_from_isr(&buf[..n]);
        }
    }
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
            do_loopback(k);
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
                    do_loopback(k);
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

extern "C" fn p1_main() -> ! {
    hprintln!("[P1] opening hw_uart device {}", HW_UART_DEV);
    let rc = svc!(SYS_DEV_OPEN, HW_UART_DEV, 0u32, 0u32);
    assert_or_fail(!SvcError::is_error(rc), "P1: DEV_OPEN failed");

    let mut tx_buf = [0u8; 16]; // stack copy for validate_user_ptr
    tx_buf[..MSG.len()].copy_from_slice(MSG);
    hprintln!("[P1] writing \"Hello from P1\"");
    let rc = svc!(
        SYS_DEV_WRITE,
        HW_UART_DEV,
        MSG.len() as u32,
        tx_buf.as_ptr() as u32
    );
    assert_or_fail(rc == MSG.len() as u32, "P1: DEV_WRITE short");
    hprintln!("[P1] wrote {} bytes to UART1 TX", rc);
    loop {
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
}

extern "C" fn p2_main() -> ! {
    hprintln!("[P2] opening hw_uart device {}", HW_UART_DEV);
    let rc = svc!(SYS_DEV_OPEN, HW_UART_DEV, 0u32, 0u32);
    assert_or_fail(!SvcError::is_error(rc), "P2: DEV_OPEN failed");
    loop {
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
        let mut buf = [0u8; 32];
        let n = svc!(
            SYS_DEV_READ,
            HW_UART_DEV,
            buf.len() as u32,
            buf.as_mut_ptr() as u32
        );
        if SvcError::is_error(n) || n == 0 {
            continue;
        }
        hprintln!("[P2] read {} bytes from UART1 RX", n);
        if n as usize == MSG.len() && buf[..n as usize] == *MSG {
            hprintln!("[P2] content verified — data integrity OK");
            hprintln!("uart1_loopback: PASS");
            debug::exit(debug::EXIT_SUCCESS);
        } else {
            hprintln!(
                "[P2] MISMATCH: expected {:?}, got {:?}",
                MSG,
                &buf[..n as usize]
            );
            hprintln!("uart1_loopback: FAIL");
            debug::exit(debug::EXIT_FAILURE);
        }
    }
}

fn assert_or_fail(cond: bool, msg: &str) {
    if !cond {
        hprintln!("uart1_loopback: FAIL — {}", msg);
        debug::exit(debug::EXIT_FAILURE);
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();
    hprintln!("uart1_loopback: start");

    // SAFETY: called once before the scheduler starts (interrupts disabled,
    // single-core). STACKS, KS are only written here; no concurrent access.
    // UartRegs::new and init use a valid MMIO base for UART1 on LM3S6965.
    unsafe {
        let regs = UartRegs::new(0x4000_D000);
        regs.init(115_200, 12_000_000);
        let hw_backend = HwUartBackend::new(HW_UART_DEV as u8, regs);

        let stacks_ref = &STACKS;
        let bases: [u32; NUM_PARTITIONS] =
            core::array::from_fn(|i| stacks_ref[i].0.as_ptr() as u32);

        // Populate the Kernel's partition table for pointer validation.
        let mut kern = Kernel::<DemoConfig>::new();
        for (i, &base) in bases.iter().enumerate() {
            let region = MpuRegion::new(base, STACK_BYTES, 0);
            let pcb = PartitionControlBlock::new(i as u8, 0, base, base + STACK_BYTES, region);
            kern.partitions.add(pcb).ok();
        }
        kern.set_hw_uart(hw_backend);
        store_kernel(kern);

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
        KS = Some(KernelState::new(sched, &cfgs).unwrap());
    }

    let parts: [(extern "C" fn() -> !, u32); NUM_PARTITIONS] = [(p1_main, 0), (p2_main, 0)];
    boot(&parts, &mut p)
}

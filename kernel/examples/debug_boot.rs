//! Debug test to isolate unified harness boot issue.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception, ExceptionFrame};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    config::KernelConfig,
    context::init_stack_frame,
    msg_pools::MsgPools,
    partition::{MpuRegion, PartitionConfig},
    partition_core::PartitionCore,
    port_pools::PortPools,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    sync_pools::SyncPools,
};
use panic_semihosting as _;

const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = 256;

struct TestConfig;
impl KernelConfig for TestConfig {
    const N: usize = 2;
    const SCHED: usize = 4;
    const STACK_WORDS: usize = 256;
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
    #[cfg(feature = "dynamic-mpu")]
    const BP: usize = 1;
    #[cfg(feature = "dynamic-mpu")]
    const BZ: usize = 32;
    #[cfg(feature = "dynamic-mpu")]
    const DR: usize = 4;

    type Core = PartitionCore<{ Self::N }, { Self::SCHED }, { Self::STACK_WORDS }>;
    type Sync = SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
    type Msg = MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
    type Ports = PortPools<{ Self::SP }, { Self::SM }, { Self::BS }, { Self::BM }, { Self::BW }>;
}

// Manual statics (not using define_unified_harness!)
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

// Kernel storage
static KERNEL: cortex_m::interrupt::Mutex<core::cell::RefCell<Option<Kernel<TestConfig>>>> =
    cortex_m::interrupt::Mutex::new(core::cell::RefCell::new(None));

// Simple PendSV handler
kernel::define_pendsv!();

// Atomic flag to indicate partition ran
static PARTITION_RAN: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

extern "C" fn partition_main() -> ! {
    // Set flag instead of using semihosting (which requires privileged mode)
    PARTITION_RAN.store(true, core::sync::atomic::Ordering::Release);
    loop {
        cortex_m::asm::nop();
    }
}

#[exception]
fn SysTick() {
    static mut TICK: u32 = 0;
    // Handler-local static: cortex-m-rt guarantees exclusive access.
    *TICK += 1;
    let tick = *TICK;
    if tick == 1 {
        hprintln!("SysTick #1");
        if PARTITION_RAN.load(core::sync::atomic::Ordering::Acquire) {
            hprintln!("debug_boot: PASS - partition ran!");
            debug::exit(debug::EXIT_SUCCESS);
        } else {
            hprintln!("debug_boot: partition not yet run");
        }
    } else if tick == 2 {
        hprintln!("SysTick #2");
        if PARTITION_RAN.load(core::sync::atomic::Ordering::Acquire) {
            hprintln!("debug_boot: PASS - partition ran!");
            debug::exit(debug::EXIT_SUCCESS);
        } else {
            hprintln!("debug_boot: FAIL - partition never ran");
            debug::exit(debug::EXIT_FAILURE);
        }
    }
}

#[exception]
unsafe fn HardFault(ef: &ExceptionFrame) -> ! {
    hprintln!("HardFault!");
    hprintln!("  r0  = {:#010x}", ef.r0());
    hprintln!("  r1  = {:#010x}", ef.r1());
    hprintln!("  r2  = {:#010x}", ef.r2());
    hprintln!("  r3  = {:#010x}", ef.r3());
    hprintln!("  r12 = {:#010x}", ef.r12());
    hprintln!("  lr  = {:#010x}", ef.lr());
    hprintln!("  pc  = {:#010x}", ef.pc());
    hprintln!("  xpsr = {:#010x}", ef.xpsr());
    debug::exit(debug::EXIT_FAILURE);
    #[allow(clippy::empty_loop)]
    loop {}
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();
    hprintln!("debug_boot: start");

    // Build schedule
    let mut sched = ScheduleTable::<{ TestConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched");

    // Build partition configs
    let cfgs: [PartitionConfig; NUM_PARTITIONS] = unsafe {
        core::array::from_fn(|i| {
            let b = STACKS[i].0.as_ptr() as u32;
            PartitionConfig {
                id: i as u8,
                entry_point: 0,
                stack_base: b,
                stack_size: (STACK_WORDS * 4) as u32,
                mpu_region: MpuRegion::new(b, (STACK_WORDS * 4) as u32, 0),
                peripheral_regions: heapless::Vec::new(),
            }
        })
    };

    // Create kernel
    #[cfg(feature = "dynamic-mpu")]
    let mut k =
        Kernel::<TestConfig>::new(sched, &cfgs, kernel::virtual_device::DeviceRegistry::new())
            .expect("kernel");
    #[cfg(not(feature = "dynamic-mpu"))]
    let mut k = Kernel::<TestConfig>::new(sched, &cfgs).expect("kernel");
    hprintln!("debug_boot: kernel created");

    // Initialize stacks BEFORE storing kernel
    // SAFETY: single-core, interrupts disabled — exclusive access to statics.
    #[allow(clippy::deref_addrof)]
    unsafe {
        let stacks = &mut *(&raw mut STACKS);
        let partition_sp = &mut *(&raw mut PARTITION_SP);

        let ep = partition_main as *const () as u32;
        hprintln!("debug_boot: entry point = {:#010x}", ep);

        let stk = &mut stacks[0].0;
        let ix = init_stack_frame(stk, ep, Some(0)).expect("init_stack_frame");
        let sp = stk.as_ptr() as u32 + (ix as u32) * 4;
        partition_sp[0] = sp;

        hprintln!(
            "debug_boot: stack @ {:#010x}, SP = {:#010x}",
            stk.as_ptr() as u32,
            sp
        );

        // Debug: print the initialized stack frame
        hprintln!(
            "debug_boot: stack[{}..{}] (SavedContext r4-r11):",
            ix,
            ix + 8
        );
        for j in 0..8 {
            hprintln!("  stack[{}] = {:#010x}", ix + j, stk[ix + j]);
        }
        hprintln!(
            "debug_boot: stack[{}..{}] (ExceptionFrame):",
            ix + 8,
            ix + 16
        );
        for j in 8..16 {
            hprintln!("  stack[{}] = {:#010x}", ix + j, stk[ix + j]);
        }
    }

    // Start schedule and get first partition
    hprintln!("debug_boot: starting schedule");
    let first_pid = k.start_schedule();
    hprintln!("debug_boot: first_pid = {:?}", first_pid);

    // Store kernel
    cortex_m::interrupt::free(|cs| {
        *KERNEL.borrow(cs).borrow_mut() = Some(k);
    });
    hprintln!("debug_boot: kernel stored");

    // Set NEXT_PARTITION
    let pid = first_pid.expect("no partition") as u32;
    unsafe {
        core::ptr::write_volatile(&raw mut NEXT_PARTITION, pid);
    }
    hprintln!("debug_boot: NEXT_PARTITION = {}", pid);

    // Set exception priorities
    unsafe {
        p.SCB.set_priority(SystemHandler::SVCall, 0x00);
        p.SCB.set_priority(SystemHandler::PendSV, 0xFF);
        p.SCB.set_priority(SystemHandler::SysTick, 0xFE);
    }
    hprintln!("debug_boot: priorities set");

    // Configure SysTick
    p.SYST.set_clock_source(SystClkSource::Core);
    p.SYST.set_reload(120_000 - 1);
    p.SYST.clear_current();
    p.SYST.enable_counter();
    p.SYST.enable_interrupt();
    hprintln!("debug_boot: SysTick configured");

    // Trigger PendSV
    hprintln!("debug_boot: triggering PendSV");
    cortex_m::peripheral::SCB::set_pendsv();

    hprintln!("debug_boot: entering idle loop");
    loop {
        cortex_m::asm::wfi();
    }
}

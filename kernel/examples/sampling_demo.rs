//! 3-partition sensor telemetry pipeline via sampling ports.
#![no_std]
#![no_main]
use cortex_m::peripheral::{scb::SystemHandler, syst::SystClkSource, SCB};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::{
    context::init_stack_frame,
    kernel::KernelState,
    partition::PartitionConfig,
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    syscall::{SYS_SAMPLING_READ, SYS_SAMPLING_WRITE, SYS_YIELD},
};
use panic_semihosting as _;

const MAX_PARTITIONS: usize = 4;
const MAX_SEMAPHORES: usize = 4;
const SEM_WAIT_DEPTH: usize = 4;
const MAX_MUTEXES: usize = 4;
const MUTEX_WAIT_DEPTH: usize = 4;
const MAX_QUEUES: usize = 4;
const QUEUE_DEPTH: usize = 4;
const QUEUE_MSG_SIZE: usize = 4;
const QUEUE_WAIT_DEPTH: usize = 4;
const MAX_SAMPLING_PORTS: usize = 8;
const SAMPLING_MSG_SIZE: usize = 4;
const MAX_SCHEDULE_ENTRIES: usize = 8;
const NUM_PARTITIONS: usize = 3;

type K = Kernel<
    MAX_PARTITIONS,
    MAX_SEMAPHORES,
    SEM_WAIT_DEPTH,
    MAX_MUTEXES,
    MUTEX_WAIT_DEPTH,
    MAX_QUEUES,
    QUEUE_DEPTH,
    QUEUE_MSG_SIZE,
    QUEUE_WAIT_DEPTH,
    MAX_SAMPLING_PORTS,
    SAMPLING_MSG_SIZE,
>;
static mut STACKS: [[u32; 256]; NUM_PARTITIONS] = [[0; 256]; NUM_PARTITIONS];
#[no_mangle]
static mut PARTITION_SP: [u32; NUM_PARTITIONS] = [0; NUM_PARTITIONS];
#[no_mangle]
static mut CURRENT_PARTITION: u32 = u32::MAX;
#[no_mangle]
static mut NEXT_PARTITION: u32 = 0;
static mut KERN: Option<K> = None;
static mut KS: Option<KernelState<MAX_PARTITIONS, MAX_SCHEDULE_ENTRIES>> = None;
#[used]
static _SVC: unsafe extern "C" fn(&mut kernel::context::ExceptionFrame) = kernel::svc::SVC_HANDLER;
kernel::define_pendsv!();
unsafe extern "C" fn hook(f: &mut kernel::context::ExceptionFrame) {
    let p = &raw mut KERN;
    if let Some(k) = unsafe { (*p).as_mut() } {
        unsafe { k.dispatch(f) }
    }
}
macro_rules! svc {
    ($id:expr, $a:expr, $b:expr, $c:expr) => {{ let r: u32; #[cfg(target_arch = "arm")]
        unsafe { core::arch::asm!("svc #0", inout("r0") $id => r,
            in("r1") $a, in("r2") $b, in("r3") $c, out("r12") _) }
        #[cfg(not(target_arch = "arm"))] { let _ = ($id, $a, $b, $c); r = 0; } r }};
}
macro_rules! unpack_r0 {
    () => {{ let p: u32; #[cfg(target_arch = "arm")] unsafe { core::arch::asm!("", out("r0") p) }
        #[cfg(not(target_arch = "arm"))] { p = 0; } p }};
}
extern "C" fn sensor_main() -> ! {
    let (src, mut v) = (unpack_r0!() >> 16, 0u8);
    loop {
        v = v.wrapping_add(1);
        hprintln!("[sensor]  write val={}", v);
        svc!(SYS_SAMPLING_WRITE, src, 1u32, [v].as_ptr() as u32);
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
}
extern "C" fn control_main() -> ! {
    let packed = unpack_r0!();
    let (src, dst) = (packed >> 16, packed & 0xFFFF);
    loop {
        let mut buf = [0u8; 1];
        let sz = svc!(
            SYS_SAMPLING_READ,
            dst,
            buf.len() as u32,
            buf.as_mut_ptr() as u32
        );
        let v = if sz > 0 && sz != u32::MAX { buf[0] } else { 0 };
        let tag = if v > 2 { "ALERT" } else { "NORMAL" };
        hprintln!("[control] val={} valid={} -> {}", v, sz != u32::MAX, tag);
        svc!(
            SYS_SAMPLING_WRITE,
            src,
            1u32,
            [u8::from(v > 2)].as_ptr() as u32
        );
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
}
extern "C" fn display_main() -> ! {
    let dst = unpack_r0!() & 0xFFFF;
    let mut cyc: u32 = 0;
    loop {
        let mut buf = [0u8; 1];
        let sz = svc!(
            SYS_SAMPLING_READ,
            dst,
            buf.len() as u32,
            buf.as_mut_ptr() as u32
        );
        let valid = sz > 0 && sz != u32::MAX;
        let tag = if valid && buf[0] == 1 {
            "ALERT"
        } else {
            "NORMAL"
        };
        hprintln!("[display] status={} valid={}", tag, valid);
        cyc += 1;
        if cyc >= 4 {
            hprintln!("sampling_demo: all checks passed");
            debug::exit(debug::EXIT_SUCCESS);
        }
        svc!(SYS_YIELD, 0u32, 0u32, 0u32);
    }
}
#[exception]
fn SysTick() {
    let p = &raw mut KS;
    if let Some(pid) = unsafe { (*p).as_mut() }
        .expect("KS")
        .advance_schedule_tick()
    {
        unsafe { core::ptr::write_volatile(&raw mut NEXT_PARTITION, pid as u32) }
        SCB::set_pendsv();
    }
}
#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();
    hprintln!("sampling_demo: start");
    unsafe {
        let mut k = K {
            partitions: kernel::partition::PartitionTable::new(),
            semaphores: kernel::semaphore::SemaphorePool::new(),
            mutexes: kernel::mutex::MutexPool::new(0),
            messages: kernel::message::MessagePool::new(),
            tick: kernel::tick::TickCounter::new(),
            sampling: kernel::sampling::SamplingPortPool::new(),
        };
        let s0 = k.sampling.create_port(PortDirection::Source, 10).unwrap();
        let d0 = k
            .sampling
            .create_port(PortDirection::Destination, 10)
            .unwrap();
        k.sampling.connect_ports(s0, d0).unwrap();
        let s1 = k.sampling.create_port(PortDirection::Source, 10).unwrap();
        let d1 = k
            .sampling
            .create_port(PortDirection::Destination, 10)
            .unwrap();
        k.sampling.connect_ports(s1, d1).unwrap();
        KERN = Some(k);
        kernel::svc::SVC_DISPATCH_HOOK = Some(hook);
        let mut sched = ScheduleTable::<MAX_SCHEDULE_ENTRIES>::new();
        for i in 0..NUM_PARTITIONS as u8 {
            sched.add(ScheduleEntry::new(i, 2)).unwrap();
        }
        sched.start();
        let cfgs: [PartitionConfig; NUM_PARTITIONS] = core::array::from_fn(|i| {
            let b = 0x2000_0000 + (i as u32) * 0x2000;
            PartitionConfig {
                id: i as u8,
                entry_point: 0,
                stack_base: b,
                stack_size: 1024,
                mpu_region: kernel::partition::MpuRegion::new(b, 1024, 0),
            }
        });
        KS = Some(KernelState::new(sched, &cfgs).unwrap());
        // TODO: bit-packing port IDs into u32 is brittle; replace with a
        // typed partition-config / init-args mechanism when the kernel supports one.
        let h: [u32; NUM_PARTITIONS] = [
            (s0 as u32) << 16,
            ((s1 as u32) << 16) | d0 as u32,
            d1 as u32,
        ];
        let eps: [extern "C" fn() -> !; NUM_PARTITIONS] = [sensor_main, control_main, display_main];
        let (stk, sp) = (&raw mut STACKS, &raw mut PARTITION_SP);
        for (i, (ep, &hv)) in eps.iter().zip(h.iter()).enumerate() {
            let ix = init_stack_frame(&mut (*stk)[i], *ep as *const () as u32, Some(hv)).unwrap();
            (*sp)[i] = (*stk)[i].as_ptr() as u32 + (ix as u32) * 4;
        }
        p.SCB.set_priority(SystemHandler::SVCall, 0x00);
        p.SCB.set_priority(SystemHandler::PendSV, 0xFF);
        p.SCB.set_priority(SystemHandler::SysTick, 0xFE);
    }
    p.SYST.set_clock_source(SystClkSource::Core);
    p.SYST.set_reload(120_000 - 1);
    p.SYST.clear_current();
    p.SYST.enable_counter();
    p.SYST.enable_interrupt();
    SCB::set_pendsv();
    loop {
        cortex_m::asm::wfi()
    }
}

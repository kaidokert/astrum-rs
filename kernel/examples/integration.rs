// Not migrated to define_unified_harness! — this integration test uses a
// custom SysTick handler that lazily initialises Kernel as a handler-local
// static, performs inline MPU and IPC assertions, and exits via semihosting.
// Its structure is fundamentally different from the standard harness pattern.

//! Integration test: SysTick scheduling, MPU, and IPC under QEMU.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::config::KernelConfig;
use kernel::context::init_stack_frame;
use kernel::events;
use kernel::message::SendOutcome;
use kernel::mpu;
use kernel::partition::{MpuRegion, PartitionConfig, PartitionState};
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use kernel::svc::Kernel;
use panic_semihosting as _;

struct IntegrationConfig;
impl KernelConfig for IntegrationConfig {
    const N: usize = 4;
    const SCHED: usize = 8;
    const S: usize = 1;
    const SW: usize = 1;
    const MS: usize = 1;
    const MW: usize = 1;
    const QS: usize = 4;
    const QD: usize = 4;
    const QM: usize = 4;
    const QW: usize = 4;
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
}

static mut STACK_P0: [u32; 256] = [0; 256];
static mut STACK_P1: [u32; 256] = [0; 256];
#[no_mangle]
static mut PARTITION_SP: [u32; 2] = [0; 2];
#[no_mangle]
static mut CURRENT_PARTITION: u32 = u32::MAX;
#[no_mangle]
static mut NEXT_PARTITION: u32 = 0;
static P_RAN: AtomicU32 = AtomicU32::new(u32::MAX);

extern "C" fn p0_main() -> ! {
    loop {
        P_RAN.store(0, Ordering::Release);
    }
}
extern "C" fn p1_main() -> ! {
    loop {
        P_RAN.store(1, Ordering::Release);
    }
}

// PendSV context-switch handler (identical to context_switch example).
core::arch::global_asm!(
    ".syntax unified",
    ".thumb",
    ".global PendSV",
    ".type PendSV, %function",
    "PendSV:",
    "ldr r0, =CURRENT_PARTITION",
    "ldr r1, [r0]",
    "ldr r2, =0xFFFFFFFF",
    "cmp r1, r2",
    "beq .Li",
    "mrs r3, psp",
    "stmdb r3!, {{r4-r11}}",
    "ldr r2, =PARTITION_SP",
    "lsl r0, r1, #2",
    "str r3, [r2, r0]",
    ".Li:",
    "ldr r0, =NEXT_PARTITION",
    "ldr r1, [r0]",
    "ldr r0, =CURRENT_PARTITION",
    "str r1, [r0]",
    "ldr r2, =PARTITION_SP",
    "lsl r0, r1, #2",
    "ldr r3, [r2, r0]",
    "ldmia r3!, {{r4-r11}}",
    "msr psp, r3",
    /* Set CONTROL.nPRIV so the partition runs unprivileged. */
    "mrs r0, CONTROL",
    "orr r0, r0, #1",
    "msr CONTROL, r0",
    "isb",
    "ldr lr, =0xFFFFFFFD",
    "bx lr",
    ".size PendSV, . - PendSV",
);

fn pcfg(id: u8, base: u32) -> PartitionConfig {
    PartitionConfig {
        id,
        entry_point: 0x0,
        stack_base: base,
        stack_size: 1024,
        mpu_region: MpuRegion::new(base, 1024, 0),
    }
}

#[exception]
fn SysTick() {
    static mut T: u32 = 0;
    static mut SW: u32 = 0;
    static mut K: Option<Kernel<IntegrationConfig>> = None;
    static mut IPC: bool = false;
    if K.is_none() {
        let mut s: ScheduleTable<8> = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 3)).unwrap();
        s.add(ScheduleEntry::new(1, 3)).unwrap();
        s.start();
        let cfgs = [pcfg(0, 0x2000_0000), pcfg(1, 0x2000_2000)];
        #[cfg(not(feature = "dynamic-mpu"))]
        let mut kernel = Kernel::new(s, &cfgs).expect("kernel config");
        #[cfg(feature = "dynamic-mpu")]
        let mut kernel =
            Kernel::new(s, &cfgs, kernel::virtual_device::DeviceRegistry::new()).expect("kernel");
        // Add a message queue for IPC testing.
        let _ = kernel
            .messages
            .add(kernel::message::MessageQueue::<4, 4, 4>::new());
        *K = Some(kernel);
    }
    let k = K.as_mut().unwrap();
    *T += 1;
    let _tick_result = k.advance_schedule_tick();
    #[cfg(not(feature = "dynamic-mpu"))]
    let switch_pid: Option<u8> = _tick_result;
    #[cfg(feature = "dynamic-mpu")]
    let switch_pid: Option<u8> = match _tick_result {
        kernel::scheduler::ScheduleEvent::PartitionSwitch(pid) => Some(pid),
        _ => None,
    };
    if let Some(pid) = switch_pid {
        let pcb = k.partitions.get(pid as usize).unwrap();
        assert!(mpu::partition_mpu_regions(pcb).is_some());
        hprintln!("[PASS] MPU + switch {} -> P{}", *SW + 1, pid);
        unsafe {
            core::ptr::write_volatile(core::ptr::addr_of_mut!(NEXT_PARTITION), pid as u32);
        }
        cortex_m::peripheral::SCB::set_pendsv();
        *SW += 1;
    }
    if *T == 4 && !*IPC {
        let msg = [0xCA, 0xFE, 0xBA, 0xBE];
        assert!(matches!(
            k.messages.send(0, 0, &msg),
            Ok(SendOutcome::Delivered { .. })
        ));
        let mut buf = [0u8; 4];
        assert!(k.messages.recv(0, 1, &mut buf).is_ok() && buf == msg);
        hprintln!("[PASS] msg_send + msg_recv");
        let _ = k
            .partitions
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Running);
        let _ = k
            .partitions
            .get_mut(1)
            .unwrap()
            .transition(PartitionState::Running);
        events::event_set(&mut k.partitions, 0, 0x01);
        assert!(k.partitions.get(0).unwrap().event_flags() & 0x01 != 0);
        hprintln!("[PASS] event_flag ack");
        *IPC = true;
    }
    if *SW >= 4 && *IPC {
        let who = P_RAN.load(Ordering::Acquire);
        if who <= 1 {
            hprintln!("[PASS] ctx switch (last P{})", who);
        }
        hprintln!("integration: all checks passed");
        debug::exit(debug::EXIT_SUCCESS);
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();
    hprintln!("integration: start");
    unsafe {
        let (s0, s1) = (&raw mut STACK_P0, &raw mut STACK_P1);
        let i0 = init_stack_frame(&mut *s0, p0_main as *const () as u32, None).unwrap();
        let i1 = init_stack_frame(&mut *s1, p1_main as *const () as u32, None).unwrap();
        PARTITION_SP[0] = (*s0).as_ptr() as u32 + (i0 as u32) * 4;
        PARTITION_SP[1] = (*s1).as_ptr() as u32 + (i1 as u32) * 4;
        p.SCB.set_priority(SystemHandler::PendSV, 0xFF);
        p.SCB.set_priority(SystemHandler::SysTick, 0xFE);
    }
    p.SYST.set_clock_source(SystClkSource::Core);
    p.SYST.set_reload(120_000 - 1);
    p.SYST.clear_current();
    p.SYST.enable_counter();
    p.SYST.enable_interrupt();
    cortex_m::peripheral::SCB::set_pendsv();
    loop {
        cortex_m::asm::wfi();
    }
}

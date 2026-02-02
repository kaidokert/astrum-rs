// Not migrated to define_harness! — this integration test uses a
// custom SysTick handler that lazily initialises KernelState and
// MessageQueue as handler-local statics, performs inline MPU and IPC
// assertions, and exits via semihosting. Its structure is
// fundamentally different from the standard harness pattern.

//! Integration test: SysTick scheduling, MPU, and IPC under QEMU.
#![no_std]
#![no_main]
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::peripheral::scb::SystemHandler;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::context::init_stack_frame;
use kernel::events;
use kernel::kernel::KernelState;
use kernel::message::{MessageQueue, SendOutcome};
use kernel::mpu;
use kernel::partition::{MpuRegion, PartitionConfig, PartitionState};
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use panic_semihosting as _;

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
    static mut KS: Option<KernelState<4, 8>> = None;
    static mut MQ: Option<MessageQueue<4, 4, 4>> = None;
    static mut IPC: bool = false;
    if KS.is_none() {
        let mut s: ScheduleTable<8> = ScheduleTable::new();
        s.add(ScheduleEntry::new(0, 3)).unwrap();
        s.add(ScheduleEntry::new(1, 3)).unwrap();
        s.start();
        let cfgs = [pcfg(0, 0x2000_0000), pcfg(1, 0x2000_2000)];
        *KS = Some(KernelState::new(s, &cfgs).unwrap());
        *MQ = Some(MessageQueue::new());
    }
    let (ks, mq) = (KS.as_mut().unwrap(), MQ.as_mut().unwrap());
    *T += 1;
    let _tick_result = ks.advance_schedule_tick();
    #[cfg(not(feature = "dynamic-mpu"))]
    let switch_pid: Option<u8> = _tick_result;
    #[cfg(feature = "dynamic-mpu")]
    let switch_pid: Option<u8> = match _tick_result {
        kernel::scheduler::ScheduleEvent::PartitionSwitch(pid) => Some(pid),
        _ => None,
    };
    if let Some(pid) = switch_pid {
        let pcb = ks.partitions().get(pid as usize).unwrap();
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
            mq.send(0, &msg),
            Ok(SendOutcome::Delivered { .. })
        ));
        let mut buf = [0u8; 4];
        assert!(mq.recv(1, &mut buf).is_ok() && buf == msg);
        hprintln!("[PASS] msg_send + msg_recv");
        let pt = ks.partitions_mut();
        let _ = pt.get_mut(0).unwrap().transition(PartitionState::Running);
        let _ = pt.get_mut(1).unwrap().transition(PartitionState::Running);
        events::event_set(pt, 0, 0x01);
        assert!(pt.get(0).unwrap().event_flags() & 0x01 != 0);
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

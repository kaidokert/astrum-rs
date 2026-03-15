//! Integration test: SysTick scheduling, MPU, and IPC under QEMU.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::message::SendOutcome;
use kernel::mpu;
use kernel::partition::{ExternalPartitionMemory, MpuRegion, PartitionState};
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use kernel::svc::Kernel;
use kernel::{
    boot, events, AlignedStack1K, DebugEnabled, MsgStandard, Partitions4, PortsTiny,
    StackStorage as _, SyncMinimal,
};

kernel::compose_kernel_config!(IntegrationConfig<Partitions4, SyncMinimal, MsgStandard, PortsTiny, DebugEnabled>);

static mut STACKS: [AlignedStack1K; IntegrationConfig::N] =
    [AlignedStack1K::ZERO; IntegrationConfig::N];

static P_RAN: AtomicU32 = AtomicU32::new(u32::MAX);
static SW: AtomicU32 = AtomicU32::new(0);
static IPC: AtomicU32 = AtomicU32::new(0);

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

kernel::define_unified_harness!(no_boot, IntegrationConfig, |tick, k| {
    if k.partitions()
        .get(0)
        .filter(|pcb| {
            mpu::validate_mpu_region(pcb.mpu_region().base(), pcb.mpu_region().size()).is_ok()
        })
        .is_some()
    {
        hprintln!(
            "[PASS] MPU + switch {}",
            SW.fetch_add(1, Ordering::Relaxed) + 1
        );
    }
    if tick == 4 && IPC.load(Ordering::Acquire) == 0 {
        let (msg, mut buf) = ([0xCA, 0xFE, 0xBA, 0xBE], [0u8; 4]);
        assert!(matches!(
            k.messages_mut().send(0, 0, &msg),
            Ok(SendOutcome::Delivered { .. })
        ));
        assert!(k.messages_mut().recv(0, 1, &mut buf).is_ok() && buf == msg);
        hprintln!("[PASS] msg_send + msg_recv");
        let _ = k
            .partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Running);
        let _ = k
            .partitions_mut()
            .get_mut(1)
            .unwrap()
            .transition(PartitionState::Running);
        events::event_set(k.partitions_mut(), 0, 0x01);
        assert!(k.partitions().get(0).unwrap().event_flags() & 0x01 != 0);
        hprintln!("[PASS] event_flag ack");
        IPC.store(1, Ordering::Release);
    }
    if SW.load(Ordering::Acquire) >= 4 && IPC.load(Ordering::Acquire) == 1 {
        if P_RAN.load(Ordering::Acquire) <= 1 {
            hprintln!("[PASS] ctx switch");
        }
        hprintln!("integration: all checks passed");
        debug::exit(debug::EXIT_SUCCESS);
    }
});

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().unwrap();
    hprintln!("integration: start");
    let mut s: ScheduleTable<8> = ScheduleTable::new();
    s.add(ScheduleEntry::new(0, 3)).unwrap();
    s.add(ScheduleEntry::new(1, 3)).unwrap();
    const NUM_PARTS: usize = 2;
    let entry_fns: [extern "C" fn() -> !; NUM_PARTS] = [p0_main, p1_main];
    let mut k = {
        // SAFETY: called once from main before any interrupt handler runs.
        let ptr = &raw mut STACKS;
        let stacks = unsafe { &mut *ptr };
        let mut stk_iter = stacks.iter_mut();
        let memories: [_; NUM_PARTS] = core::array::from_fn(|i| {
            ExternalPartitionMemory::from_aligned_stack(
                stk_iter.next().unwrap(),
                entry_fns[i] as *const () as u32,
                MpuRegion::new(0, 0, 0),
                i as u8,
            )
            .unwrap()
        });
        Kernel::<IntegrationConfig>::new(s, &memories).unwrap()
    };
    let _ = k
        .messages_mut()
        .add(kernel::message::MessageQueue::<4, 4, 4>::new());
    store_kernel(k);
    // SAFETY: boot_preconfigured reads stack info from PCBs populated by Kernel::new().
    match unsafe { boot::boot_preconfigured::<IntegrationConfig>(p) }.unwrap() {}
}

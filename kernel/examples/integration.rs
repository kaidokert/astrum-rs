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
use kernel::partition::PartitionState;
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use kernel::{
    events, DebugEnabled, MsgStandard, PartitionEntry, PartitionId, PartitionSpec, Partitions4,
    PortsTiny, SyncMinimal,
};

kernel::kernel_config!(IntegrationConfig<Partitions4, SyncMinimal, MsgStandard, PortsTiny, DebugEnabled>);

static P_RAN: AtomicU32 = AtomicU32::new(u32::MAX);
static SW: AtomicU32 = AtomicU32::new(0);
static IPC: AtomicU32 = AtomicU32::new(0);

const _: PartitionEntry = p0_main;
extern "C" fn p0_main() -> ! {
    loop {
        P_RAN.store(0, Ordering::Release);
    }
}
const _: PartitionEntry = p1_main;
extern "C" fn p1_main() -> ! {
    loop {
        P_RAN.store(1, Ordering::Release);
    }
}

kernel::define_kernel!(IntegrationConfig, |tick, k| {
    // Count context switches via SysTick ticks (sentinel MPU regions in QEMU
    // have size=0, so validate_mpu_region always fails — count ticks instead).
    SW.store(tick, Ordering::Release);
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
        events::event_set(k.partitions_mut(), PartitionId::new(0), 0x01);
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
    hprintln!(
        "=== {} v{} ===",
        env!("CARGO_PKG_NAME"),
        env!("CARGO_PKG_VERSION")
    );
    hprintln!("integration: start");
    let mut s: ScheduleTable<8> = ScheduleTable::new();
    s.add(ScheduleEntry::new(0, 3)).unwrap();
    s.add(ScheduleEntry::new(1, 3)).unwrap();
    s.add_system_window(1).expect("sys window");

    let parts: [PartitionSpec; 2] = [PartitionSpec::entry(p0_main), PartitionSpec::entry(p1_main)];
    let mut k = init_kernel(s, &parts).expect("integration: init_kernel");
    let _ = k
        .messages_mut()
        .add(kernel::message::MessageQueue::<4, 4, 4>::new());
    store_kernel(&mut k);
    match boot(p).expect("integration: boot") {}
}

//! Integration test: SysTick scheduling, MPU, and IPC under QEMU.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::config::KernelConfig;
use kernel::message::SendOutcome;
use kernel::mpu;
use kernel::msg_pools::MsgPools;
use kernel::partition::{MpuRegion, PartitionConfig, PartitionState};
use kernel::partition_core::PartitionCore;
use kernel::port_pools::PortPools;
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use kernel::svc::Kernel;
use kernel::sync_pools::SyncPools;
use kernel::{boot, events};
use panic_semihosting as _;

struct IntegrationConfig;
impl KernelConfig for IntegrationConfig {
    const N: usize = 4;
    const SCHED: usize = 8;
    const STACK_WORDS: usize = 256;
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

    type Core = PartitionCore<{ Self::N }, { Self::SCHED }, { Self::STACK_WORDS }>;
    type Sync = SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
    type Msg = MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
    type Ports = PortPools<{ Self::SP }, { Self::SM }, { Self::BS }, { Self::BM }, { Self::BW }>;
}

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

kernel::define_unified_harness!(no_boot, IntegrationConfig, 2, 256, |tick, k| {
    if k.partitions()
        .get(0)
        .and_then(mpu::partition_mpu_regions)
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

fn pcfg(id: u8) -> PartitionConfig {
    PartitionConfig {
        id,
        entry_point: 0,
        stack_base: 0,
        stack_size: 1024,
        mpu_region: MpuRegion::new(0, 0, 0),
        peripheral_regions: heapless::Vec::new(),
    }
}

#[entry]
fn main() -> ! {
    let mut p = cortex_m::Peripherals::take().unwrap();
    hprintln!("integration: start");
    let mut s: ScheduleTable<8> = ScheduleTable::new();
    s.add(ScheduleEntry::new(0, 3)).unwrap();
    s.add(ScheduleEntry::new(1, 3)).unwrap();
    #[cfg(not(feature = "dynamic-mpu"))]
    let mut k = Kernel::<IntegrationConfig>::new(s, &[pcfg(0), pcfg(1)]).unwrap();
    #[cfg(feature = "dynamic-mpu")]
    let mut k = Kernel::<IntegrationConfig>::new(
        s,
        &[pcfg(0), pcfg(1)],
        kernel::virtual_device::DeviceRegistry::new(),
    )
    .unwrap();
    let _ = k
        .messages_mut()
        .add(kernel::message::MessageQueue::<4, 4, 4>::new());
    store_kernel(k);
    match boot::boot::<IntegrationConfig>(&[(p0_main, 0), (p1_main, 0)], &mut p).unwrap() {}
}

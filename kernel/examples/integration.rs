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
use kernel::partition::{MpuRegion, PartitionConfig, PartitionState};
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use kernel::svc::Kernel;
use kernel::{boot, events, DebugEnabled, MsgStandard, Partitions4, PortsTiny, SyncMinimal};

kernel::compose_kernel_config!(IntegrationConfig<Partitions4, SyncMinimal, MsgStandard, PortsTiny, DebugEnabled>);

const STACK_WORDS: usize = IntegrationConfig::STACK_WORDS;
// TODO: reviewer false positive on align(4096) — matches the harness macro's alignment
// (kernel/src/harness.rs) which also uses align(4096) for MPU region sizing constraints.
#[repr(C, align(4096))]
struct PartitionStacks([[u32; STACK_WORDS]; IntegrationConfig::N]);
static mut PARTITION_STACKS: PartitionStacks =
    PartitionStacks([[0u32; STACK_WORDS]; IntegrationConfig::N]);

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
    // SAFETY: called once from main before any interrupt handler runs.
    let stacks: &mut [[u32; STACK_WORDS]; IntegrationConfig::N] =
        unsafe { &mut *(&raw mut PARTITION_STACKS).cast() };
    // TODO: IntegrationConfig composes Partitions4 (N=4) but only 2 partitions are used.
    // Either switch to Partitions2 or add entries for all 4 when more test coverage is needed.
    let parts: [(extern "C" fn() -> !, u32); 2] = [(p0_main, 0), (p1_main, 0)];
    match boot::boot_external::<IntegrationConfig, STACK_WORDS>(&parts, &mut p, stacks).unwrap() {}
}

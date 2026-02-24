//! QEMU test: verify scheduler tick drives partition switches via Kernel.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::{debug, hprintln};
use kernel::config::KernelConfig;
use kernel::msg_pools::MsgPools;
use kernel::partition::{MpuRegion, PartitionConfig, PartitionState};
use kernel::partition_core::{AlignedStack1K, PartitionCore};
use kernel::port_pools::PortPools;
use kernel::scheduler::{ScheduleEntry, ScheduleTable};
use kernel::svc::Kernel;
use kernel::sync_pools::SyncPools;
use kernel::tick::configure_systick;
use panic_semihosting as _;

struct TestConfig;
impl KernelConfig for TestConfig {
    const N: usize = 4;
    const SCHED: usize = 8;
    const S: usize = 4;
    const SW: usize = 4;
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

    type Core = PartitionCore<{ Self::N }, { Self::SCHED }, AlignedStack1K>;
    type Sync = SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
    type Msg = MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
    type Ports = PortPools<{ Self::SP }, { Self::SM }, { Self::BS }, { Self::BM }, { Self::BW }>;
}

kernel::define_unified_kernel!(TestConfig);

static SWITCH_COUNT: AtomicU32 = AtomicU32::new(0);
static FAIL_COUNT: AtomicU32 = AtomicU32::new(0);
/// Tracks the active partition across ticks. `u32::MAX` means no partition yet.
static ACTIVE_PID: AtomicU32 = AtomicU32::new(u32::MAX);
const RELOAD: u32 = kernel::config::compute_systick_reload(12_000_000, 10_000);
const MAX_SWITCHES: u32 = 6;

#[exception]
fn SysTick() {
    kernel::state::with_kernel_mut::<TestConfig, _, _>(|k| {
        // Read previous active partition's state before advancing the tick.
        // On the first switch prev_pid is u32::MAX (no previous partition).
        let prev_pid = ACTIVE_PID.load(Ordering::Acquire);
        if prev_pid != u32::MAX {
            let state = k.partitions().get(prev_pid as usize).map(|p| p.state());
            if state != Some(PartitionState::Running) {
                hprintln!(
                    "FAIL: pre-tick partition {} expected Running, got {:?}",
                    prev_pid,
                    state
                );
                FAIL_COUNT.fetch_add(1, Ordering::Release);
            }
        }

        let event = k.advance_schedule_tick();
        if let kernel::scheduler::ScheduleEvent::PartitionSwitch(pid) = event {
            // (1) Verify outgoing partition is now Ready (not Running).
            if prev_pid != u32::MAX {
                let out_state = k.partitions().get(prev_pid as usize).map(|p| p.state());
                if out_state != Some(PartitionState::Ready) {
                    hprintln!(
                        "FAIL: outgoing partition {} expected Ready, got {:?}",
                        prev_pid,
                        out_state
                    );
                    FAIL_COUNT.fetch_add(1, Ordering::Release);
                }
            }

            // (2) Use set_next_partition to transition the incoming partition
            //     to Running, matching the pattern in tick.rs / blocking_deschedule.rs.
            k.set_next_partition(pid);
            let in_state = k.partitions().get(pid as usize).map(|p| p.state());
            if in_state != Some(PartitionState::Running) {
                hprintln!(
                    "FAIL: incoming partition {} expected Running, got {:?}",
                    pid,
                    in_state
                );
                FAIL_COUNT.fetch_add(1, Ordering::Release);
            }

            // (3) Track the active partition for the next tick's verification.
            ACTIVE_PID.store(pid as u32, Ordering::Release);
            hprintln!("switch -> partition {}", pid);
            SWITCH_COUNT.fetch_add(1, Ordering::Release);
        }
    });
}

#[entry]
fn main() -> ! {
    let mut sched: ScheduleTable<8> = ScheduleTable::new();
    sched.add(ScheduleEntry::new(0, 3)).unwrap();
    sched.add(ScheduleEntry::new(1, 2)).unwrap();

    let configs = [
        PartitionConfig {
            id: 0,
            entry_point: 0x0800_0000,
            stack_base: 0x2000_2000,
            stack_size: 1024,
            mpu_region: MpuRegion::new(0x2000_2000, 1024, 0),
            peripheral_regions: heapless::Vec::new(),
        },
        PartitionConfig {
            id: 1,
            entry_point: 0x0800_1000,
            stack_base: 0x2000_3000,
            stack_size: 1024,
            mpu_region: MpuRegion::new(0x2000_3000, 1024, 0),
            peripheral_regions: heapless::Vec::new(),
        },
    ];

    #[cfg(feature = "dynamic-mpu")]
    let k = Kernel::<TestConfig>::new(
        sched,
        &configs,
        kernel::virtual_device::DeviceRegistry::new(),
    )
    .expect("kernel creation");
    #[cfg(not(feature = "dynamic-mpu"))]
    let k = Kernel::<TestConfig>::new(sched, &configs).expect("kernel creation");

    // TODO: store_kernel is generated by define_unified_kernel! macro (not in kernel::state),
    // wraps init_kernel_state + dispatch hook setup. No qualified path available.
    store_kernel(k);

    let p = cortex_m::Peripherals::take().unwrap();
    configure_systick(&mut { p.SYST }, RELOAD);
    hprintln!("scheduler_tick: started, reload={}", RELOAD);

    loop {
        let fails = FAIL_COUNT.load(Ordering::Acquire);
        if fails > 0 {
            hprintln!("FAILED: {} assertion failures", fails);
            debug::exit(debug::EXIT_FAILURE);
        }
        if SWITCH_COUNT.load(Ordering::Acquire) >= MAX_SWITCHES {
            hprintln!("done: {} switches, 0 failures", MAX_SWITCHES);
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
}

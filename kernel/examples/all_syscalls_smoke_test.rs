//! Single-partition QEMU test exercising 13 non-blocking base syscalls for ABI regression.
#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
#[allow(unused_imports)]
use kernel::kpanic as _;
use kernel::message::MessageQueue;
use kernel::sampling::PortDirection;
use kernel::scheduler::ScheduleTable;
use kernel::semaphore::Semaphore;
use kernel::svc::Kernel;
use kernel::{DebugEnabled, MsgMinimal, Partitions1, PortsSmall, SyncMinimal};

kernel::compose_kernel_config!(Cfg<Partitions1, SyncMinimal, MsgMinimal, PortsSmall, DebugEnabled>);

const TIMEOUT: u32 = 50;
const ALL: u32 = (1 << 13) - 1;
static DONE: AtomicU32 = AtomicU32::new(0);
static FAIL: AtomicU32 = AtomicU32::new(0);

kernel::define_unified_harness!(Cfg, |tick, _k| {
    let (done, fail) = (DONE.load(Ordering::Acquire), FAIL.load(Ordering::Acquire));
    if fail != 0 {
        hprintln!("all_syscalls_smoke_test: FAIL f={:#x} d={:#x}", fail, done);
        kernel::kexit!(failure);
    }
    if done == ALL {
        hprintln!("all_syscalls_smoke_test: PASS (13 syscalls ok)");
        kernel::kexit!(success);
    }
    if tick >= TIMEOUT {
        hprintln!("all_syscalls_smoke_test: FAIL timeout d={:#x}", done);
        kernel::kexit!(failure);
    }
});

fn chk(bit: u32, ok: bool) {
    if ok {
        DONE.fetch_or(1 << bit, Ordering::Release);
    } else {
        FAIL.fetch_or(1 << bit, Ordering::Release);
    }
}

extern "C" fn partition_main() -> ! {
    // TODO: reviewer false positive – sys_yield() returns Result<u32, SvcError>, Ok(0) is correct.
    chk(0, plib::sys_yield() == Ok(0));
    chk(
        1,
        plib::sys_get_partition_id().is_ok_and(|id| id.as_raw() == 0),
    );
    chk(2, plib::sys_get_time().is_ok_and(|t| t > 0));
    chk(
        3,
        plib::sys_event_set(plib::PartitionId::new(0), plib::EventMask::new(0x01)).is_ok(),
    );
    // TODO: event verify is indirect (checks clear return value rather than
    // re-reading after clear). Sufficient for ABI smoke test; a dedicated
    // event test can assert post-clear state separately.
    chk(
        4,
        plib::sys_event_clear(plib::EventMask::new(0x01)).is_ok_and(|p| p.as_raw() & 0x01 != 0),
    );
    chk(5, plib::sys_sem_signal(plib::SemaphoreId::new(0)).is_ok());
    // Data buffers stack-local: pointer validation rejects rodata in flash.
    // Use 4-byte buffer to match PortsSmall::SAMPLING_MAX_MSG_SIZE.
    let sample = [42u8, 0, 0, 0];
    chk(
        6,
        plib::sys_sampling_write(plib::SamplingPortId::new(0), &sample).is_ok(),
    );
    let mut sbuf = [0u8; 4];
    let sr = plib::sys_sampling_read(plib::SamplingPortId::new(1), &mut sbuf);
    chk(7, sr.is_ok_and(|sz| sz >= 4 && sbuf[0] == 42));
    let bb_data = [0xDEu8, 0xAD];
    chk(
        8,
        plib::sys_bb_display(plib::BlackboardId::new(0), &bb_data).is_ok(),
    );
    let mut bb_buf = [0u8; 4];
    let br = plib::sys_bb_read(plib::BlackboardId::new(0), &mut bb_buf);
    chk(
        9,
        br.is_ok_and(|sz| sz >= 2 && bb_buf[0] == 0xDE && bb_buf[1] == 0xAD),
    );
    chk(10, plib::sys_bb_clear(plib::BlackboardId::new(0)).is_ok());
    chk(11, plib::sys_debug_print(b"syscall smoke ok").is_ok());
    // MsgMinimal: MAX_MSG_SIZE=1, so send exactly 1 byte to queue 0.
    let msg = [0xABu8];
    chk(
        12,
        plib::sys_msg_send(plib::PartitionId::new(0), &msg).is_ok(),
    );
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let p = cortex_m::Peripherals::take().expect("Peripherals::take");
    hprintln!("all_syscalls_smoke_test: start");
    let sched = ScheduleTable::<{ Cfg::SCHED }>::round_robin(1, 3).expect("round_robin");
    let mut k = Kernel::<Cfg>::create_sentinels(sched).expect("Kernel::create");
    k.semaphores_mut().add(Semaphore::new(0, 1)).expect("sem");
    let s = k
        .sampling_mut()
        .create_port(PortDirection::Source, 4)
        .expect("src");
    let d = k
        .sampling_mut()
        .create_port(PortDirection::Destination, 4)
        .expect("dst");
    k.sampling_mut().connect_ports(s, d).expect("connect");
    k.blackboards_mut().create().expect("bb");
    k.messages_mut().add(MessageQueue::new()).expect("mq");
    store_kernel(k);
    let parts: [(extern "C" fn() -> !, u32); Cfg::N] = [(partition_main, 0)];
    match boot(&parts, p).expect("boot") {}
}

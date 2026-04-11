//! Working Sampling Port Demo - Producer/Consumer with Latest-Value Semantics
//!
//! This demonstrates proper sampling port usage in a partitioned kernel:
//! - Sampling ports for inter-partition data transfer (latest-value-wins)
//! - NO shared memory between partitions
//! - NO global atomics accessible by partitions
//! - Telemetry via kernel tick handler (privileged context)
//!
//! Pattern:
//! - P0 (Producer): Writes incrementing counter (0-255) to sampling port
//! - P1 (Consumer): Reads from sampling port (may miss intermediate values)
//! - Kernel tick handler reports port status and statistics

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use cortex_m_rt::{entry, exception};
use kernel::{PartitionSpec, 
    sampling::PortDirection,
    scheduler::{ScheduleEntry, ScheduleTable},
    {Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled},
};
use core::sync::atomic::{AtomicU32, Ordering};
// use panic_rtt_target as _;  // Let kernel's panic-halt handle panics
use rtt_target::{rprintln, rtt_init_print};

const NUM_PARTITIONS: usize = 2;
const MSG_SIZE: usize = 1; // 1 byte messages

static WRITE_COUNT: AtomicU32 = AtomicU32::new(0);
static READ_COUNT: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(SamplingConfig<Partitions4, SyncMinimal, MsgSmall, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    SW = 2; MS = 2; MW = 2;
    SP = 2; SM = 16; BS = 2; BM = 4; BW = 2;
});

kernel::define_kernel!(SamplingConfig, |tick, k| {
    // Kernel tick handler - privileged context for telemetry
    if tick % 500 == 0 {
        let writes = WRITE_COUNT.load(Ordering::Acquire);
        let reads = READ_COUNT.load(Ordering::Acquire);
        rprintln!("[KERNEL] Tick {}: Sampling port status: WRITE={} READ={}", tick, writes, reads);

        // Port 0 (source)
        if let Some(src) = k.sampling().get(0) {
            rprintln!("  Port 0 (source): size={}, ts={}, connected={:?}",
                src.current_size(), src.timestamp(), src.connected_port());
            if src.current_size() > 0 {
                rprintln!("    Data: {:02x?}", &src.data()[..src.current_size().min(4)]);
            }
        } else {
            rprintln!("  Port 0: NOT FOUND!");
        }

        // Port 1 (destination)
        if let Some(dst) = k.sampling().get(1) {
            rprintln!("  Port 1 (dest): size={}, ts={}, connected={:?}",
                dst.current_size(), dst.timestamp(), dst.connected_port());
            if dst.current_size() > 0 {
                rprintln!("    Data: {:02x?}", &dst.data()[..dst.current_size().min(4)]);
            }
        } else {
            rprintln!("  Port 1: NOT FOUND!");
        }

        // Total port count
        rprintln!("  Total ports in pool: {}", k.sampling().len());

        if writes > 20 && reads > 10 {
            rprintln!("✓ SUCCESS: Sampling ports working! WRITE={} READ={}", writes, reads);
        }
    }
});

extern "C" fn producer_main_body(r0: u32) -> ! {
    let port_id = r0.into();
    let mut counter: u8 = 0;  // Partition-local (stack) - fine!

    loop {
        // Write incrementing counter to sampling port
        let buf = [counter];
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);

        if plib::sys_sampling_write(port_id, &buf).is_ok() {
            WRITE_COUNT.fetch_add(1, Ordering::Release);
        }

        // Increment counter (wraps 0-255)
        counter = counter.wrapping_add(1);

        // Small delay between writes
        for _ in 0..5000 {
            core::hint::spin_loop();
        }

        plib::sys_yield().ok();
    }
}

extern "C" fn consumer_main_body(r0: u32) -> ! {
    let port_id = r0.into();
    let mut last_value: u8 = 0;     // Partition-local (stack) - fine!
    let mut read_count: u32 = 0;    // Partition-local (stack) - fine!

    loop {
        // Read from sampling port
        let mut buf = [0u8; 1];
        if let Ok(sz) = plib::sys_sampling_read(port_id, &mut buf) {
            core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Acquire);
            if sz > 0 {
                last_value = buf[0];
                read_count = read_count.wrapping_add(1);
                READ_COUNT.store(read_count, Ordering::Release);
            }
        }

        // Small delay
        for _ in 0..3000 {
            core::hint::spin_loop();
        }

        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(producer_main => producer_main_body);
kernel::partition_trampoline!(consumer_main => consumer_main_body);

#[entry]
fn main() -> ! {
    rprintln!("\n=== Working Sampling Port Demo - Producer/Consumer ===");

    // Disable data cache and instruction cache (CCR register)
    unsafe {
        let ccr = core::ptr::read_volatile(0xE000ED14 as *const u32);
        // Disable D-cache (bit 16) and I-cache (bit 17)
        let ccr_no_cache = ccr & !(1 << 16) & !(1 << 17);
        core::ptr::write_volatile(0xE000ED14 as *mut u32, ccr_no_cache);
        rprintln!("[INIT] Caches disabled (CCR: 0x{:08x} -> 0x{:08x})", ccr, ccr_no_cache);
    }

    let mut p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ SamplingConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(1).expect("sys_window");

    // Port IDs are deterministic: src=0, dst=1
    let src_id: u32 = 0;
    let dst_id: u32 = 1;

    let parts: [PartitionSpec; NUM_PARTITIONS] = [
        PartitionSpec::new(producer_main as kernel::PartitionEntry, src_id),
        PartitionSpec::new(consumer_main as kernel::PartitionEntry, dst_id),
    ];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));
    rprintln!("[INIT] Kernel created");

    // Create sampling port pair: source (P0 writes) -> destination (P1 reads)
    kernel::state::with_kernel_mut::<SamplingConfig, _, _>(|k| {
        let s = k.sampling_mut()
            .create_port(PortDirection::Source, 100)
            .expect("source port");
        let d = k.sampling_mut()
            .create_port(PortDirection::Destination, 100)
            .expect("dest port");
        k.sampling_mut()
            .connect_ports(s, d)
            .expect("connect ports");
        Ok::<(), ()>(())
    }).expect("ipc setup");

    rprintln!("[INIT] Sampling ports created and linked: {} (source) -> {} (dest)", src_id, dst_id);
    rprintln!("[INIT] Message size: {} byte(s)", MSG_SIZE);

    rprintln!("[INIT] Booting...\n");
    match boot(p).expect("boot") {
    }
}

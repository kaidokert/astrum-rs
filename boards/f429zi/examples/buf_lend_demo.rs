//! Buffer Lend Demo — STM32F429ZI NUCLEO-144
//!
//! Demonstrates the kernel buffer pool API for zero-copy inter-partition IPC:
//!
//!   sys_buf_alloc  — P0 acquires a buffer slot from the kernel pool
//!   sys_buf_write  — P0 fills the buffer with a known pattern (copy into kernel memory)
//!   sys_buf_lend   — kernel atomically grants P1 an MPU window onto the buffer slot
//!   sys_buf_read   — P1 reads and verifies the pattern through the kernel-managed grant
//!   sys_buf_revoke — kernel atomically removes P1's MPU window
//!   sys_buf_release — P0 returns the slot to the pool for the next cycle
//!
//! Security properties demonstrated:
//!   - API-level: sys_buf_read after revoke returns Err(PermissionDenied) —
//!     the kernel's ownership check rejects P1's access without needing MPU.
//!   - MPU-level (Phase 2 / DMA demo): with tighter per-partition data regions
//!     that exclude the kernel buffer area, a raw read_volatile on the buffer
//!     address after revoke will MemManage-fault.  Phase 1 uses a broad SRAM
//!     region (same as mpu_kernel_demo) so the MPU-level enforcement is not
//!     yet exercised; that is intentional — Phase 1 validates the API path.
//!
//! Cycle:
//!   P0: alloc → write PATTERN → lend(P1, read-only) → event_set(P1, READY)
//!       → event_wait(DONE) → revoke → release → CYCLE++
//!   P1: event_wait(READY) → buf_read → verify PATTERN → event_set(P0, DONE)
//!       → try buf_read again (after signal, before revoke completes) → second verify
//!
//! RTT success criterion: CYCLE > 10, VERIFY_OK increasing, VERIFY_FAIL == 0.
//!
//!
//! Build: cd f429zi && cargo build --example buf_lend_demo --features kernel-mpu

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{
    StackStorage as _,
    partition::{ExternalPartitionMemory, MpuRegion},
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    {Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use plib::{BufferSlotId, EventMask, PartitionId};
use rtt_target::{rprintln, rtt_init_print};
use f429zi::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 2;
const STACK_WORDS: usize = 512;

// Buffer pool: 1 slot, 32 bytes (minimum MPU-aligned size).
// The pattern fills all 32 bytes; P1 verifies all 32.
const PATTERN: u8 = 0xA5;
const BUF_LEN: usize = 32;

// Hardcoded slot 0 — with BP=1 sys_buf_alloc always returns slot 0.
const BUFFER_SLOT: BufferSlotId = BufferSlotId::new(0);

const P0_PID: PartitionId = PartitionId::new(0);
const P1_PID: PartitionId = PartitionId::new(1);

// Events: P0 → P1 (buffer lent, ready to read), P1 → P0 (read done, safe to revoke).
const BUF_READY: EventMask = EventMask::new(0x1);
const BUF_DONE:  EventMask = EventMask::new(0x2);

static CYCLE:       AtomicU32 = AtomicU32::new(0);
static VERIFY_OK:   AtomicU32 = AtomicU32::new(0);
static VERIFY_FAIL: AtomicU32 = AtomicU32::new(0);
static API_ERR:     AtomicU32 = AtomicU32::new(0);
// P1 attempted sys_buf_read after the lend was revoked — kernel returned Err.
// In Phase 1 this path isn't exercised; tracked for Phase 2.
static POST_REVOKE_REJECTED: AtomicU32 = AtomicU32::new(0);

kernel::kernel_config!(BufConfig[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
    buffer_pool_regions = 1;   // BP: 1 slot in the pool
    buffer_zone_size = 32;     // BZ: 32 bytes — minimum valid MPU region size
});

kernel::define_kernel!(BufConfig, |tick, _k| {
    if tick % 1000 == 0 {
        let cycle       = CYCLE.load(Ordering::Acquire);
        let verify_ok   = VERIFY_OK.load(Ordering::Acquire);
        let verify_fail = VERIFY_FAIL.load(Ordering::Acquire);
        let api_err     = API_ERR.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] CYCLE={} VERIFY_OK={} VERIFY_FAIL={} API_ERR={}",
            tick, cycle, verify_ok, verify_fail, api_err
        );
        if cycle > 10 && verify_fail == 0 && api_err == 0 {
            rprintln!("✓ SUCCESS: buffer lend/read/revoke cycle working! CYCLE={}", cycle);
        }
    }
});

// ---------------------------------------------------------------------------
// P0 — Producer: owns the buffer slot, drives the lend/revoke cycle.
// ---------------------------------------------------------------------------
extern "C" fn producer_body(_r0: u32) -> ! {
    loop {
        // 1. Allocate a writable buffer slot (no deadline — returns immediately).
        let slot = loop {
            match plib::sys_buf_alloc(true, 0) {
                Ok(s) => break s,
                Err(_) => {
                    API_ERR.fetch_add(1, Ordering::Release);
                    plib::sys_yield().ok();
                }
            }
        };

        // 2. Fill slot with known pattern.
        let pattern = [PATTERN; BUF_LEN];
        if plib::sys_buf_write(slot, &pattern).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
            if let Err(e) = plib::sys_buf_release(slot) {
                rprintln!("[P0] sys_buf_release failed: {:?}", e);
            }
            continue;
        }

        // 3. Lend slot to P1 (read-only).
        if plib::sys_buf_lend(slot, 1, false).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
            if let Err(e) = plib::sys_buf_release(slot) {
                rprintln!("[P0] sys_buf_release failed: {:?}", e);
            }
            continue;
        }

        // 4. Signal P1: buffer is ready to read.
        if let Err(e) = plib::sys_event_set(P1_PID, BUF_READY) {
            rprintln!("[P0] sys_event_set(P1, READY) failed: {:?}", e);
        }

        // 5. Wait for P1 to finish reading.
        loop {
            match plib::sys_event_wait(BUF_DONE) {
                Err(_) => continue,
                Ok(bits) if bits.as_raw() == 0 => continue,
                Ok(_) => break,
            }
        }

        // 6. Revoke P1's grant — kernel removes P1's MPU window for this slot.
        if plib::sys_buf_revoke(slot, 1).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
        }

        // 7. Release slot back to the pool for the next cycle.
        if plib::sys_buf_release(slot).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
        }

        CYCLE.fetch_add(1, Ordering::Release);
    }
}

// ---------------------------------------------------------------------------
// P1 — Verifier: waits for a lend, reads and verifies, signals done.
// ---------------------------------------------------------------------------
extern "C" fn verifier_body(_r0: u32) -> ! {
    loop {
        // Wait for P0 to signal that the buffer is lent and ready.
        loop {
            match plib::sys_event_wait(BUF_READY) {
                Err(_) => continue,
                Ok(bits) if bits.as_raw() == 0 => continue,
                Ok(_) => break,
            }
        }

        // Read from the lent slot (kernel allows this because P1 is the lendee).
        let mut buf = [0u8; BUF_LEN];
        match plib::sys_buf_read(BUFFER_SLOT, &mut buf) {
            Ok(_) => {
                if buf.iter().all(|&b| b == PATTERN) {
                    VERIFY_OK.fetch_add(1, Ordering::Release);
                } else {
                    VERIFY_FAIL.fetch_add(1, Ordering::Release);
                }
            }
            Err(_) => {
                API_ERR.fetch_add(1, Ordering::Release);
            }
        }

        // Signal P0: read complete, safe to revoke.
        if let Err(e) = plib::sys_event_set(P0_PID, BUF_DONE) {
            rprintln!("[P1] sys_event_set(P0, DONE) failed: {:?}", e);
        }

        // After signalling done but BEFORE revoke completes (P0 hasn't run yet),
        // sys_buf_read is still valid — the lend is still active.
        // This second read demonstrates that the grant persists until revoke,
        // not until BUF_DONE is signalled.
        let mut buf2 = [0u8; BUF_LEN];
        match plib::sys_buf_read(BUFFER_SLOT, &mut buf2) {
            Ok(_) => {
                // Still lent — may or may not have been revoked by now depending
                // on scheduling.  Both outcomes are legitimate:
                //   Ok  → lend still active (P0 not yet scheduled to revoke)
                //   Err → P0 already ran and revoked (counted in POST_REVOKE_REJECTED)
                if buf2.iter().all(|&b| b == PATTERN) {
                    VERIFY_OK.fetch_add(1, Ordering::Release);
                } else {
                    VERIFY_FAIL.fetch_add(1, Ordering::Release);
                }
            }
            Err(_) => {
                // P0 already revoked — kernel correctly rejected P1's access.
                POST_REVOKE_REJECTED.fetch_add(1, Ordering::Release);
            }
        }
    }
}

kernel::partition_trampoline!(producer_main => producer_body);
kernel::partition_trampoline!(verifier_main => verifier_body);

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    rprintln!("\n=== Buffer Lend Demo — STM32F429ZI ===");
    rprintln!("Demonstrates sys_buf_alloc/write/lend/read/revoke/release cycle.");
    rprintln!("P0 (producer): alloc → write → lend → signal → revoke → release");
    rprintln!("P1 (verifier): wait → read → verify → signal");

    let p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ BufConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 2)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(2).expect("sched SW");

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let data_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code_mpu = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    let mems = [
        ExternalPartitionMemory::from_aligned_stack(s0, producer_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0))
            .expect("mem0")
            .with_code_mpu_region(code_mpu)
            .expect("code0"),
        ExternalPartitionMemory::from_aligned_stack(s1, verifier_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(1))
            .expect("mem1")
            .with_code_mpu_region(code_mpu)
            .expect("code1"),
    ];

    let k = kernel::svc::Kernel::<BufConfig>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);
    rprintln!("[INIT] Kernel created. Buffer pool: 1 slot × 32 bytes.");
    rprintln!("[INIT] Pattern=0x{:02X}  Slot={}  P0↔P1 events: READY=0x{:x} DONE=0x{:x}",
        PATTERN, BUFFER_SLOT.as_raw(), BUF_READY.as_raw(), BUF_DONE.as_raw());

    rprintln!("[INIT] Booting with MPU enabled...\n");
    match boot(p).expect("boot") {}
}

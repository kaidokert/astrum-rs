//! Buffer Pool + DMA M2M Demo — SAME51 Curiosity Nano
//!
//! Demonstrates DMA memory-to-memory transfer inside a kernel partition,
//! combined with the kernel buffer pool for inter-partition data passing.
//!
//! Architecture:
//!   P0 (DMA producer):
//!     1. Alloc buffer slot from kernel pool
//!     2. DMA M2M: copy 64-byte pattern from static src → static dst
//!     3. sys_buf_write: copy DMA result into kernel buffer slot
//!     4. sys_buf_lend(P1) + event_set → P1 reads and verifies
//!     5. Wait for P1 done → revoke → release → CYCLE++
//!
//!   P1 (verifier):
//!     event_wait → sys_buf_read → verify pattern → event_set(P0, DONE)
//!
//! Key property: DMA hardware moves data without CPU involvement.
//! The kernel buffer pool then passes it to P1 via zero-copy lend.
//!
//! Peripheral access: P0 needs MPU region for DMAC (0x4100_A000).
//! DMA descriptors live in SRAM (already granted).
//!
//! Build: cd same51_curiosity && cargo build --example buf_dma_demo --features kernel-mpu

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use atsamd_hal as hal;
use hal::dmac::{self, DmaController, PriorityLevel, Transfer, TriggerAction, TriggerSource};
use kernel::{
    StackStorage as _,
    partition::{ExternalPartitionMemory, MpuRegion},
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use plib::{BufferSlotId, EventMask, PartitionId};
use rtt_target::rprintln;
use same51_curiosity::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 2;
const BUF_LEN: usize = 32;
const DUMMY_IRQ: u8 = 0;
const IRQ_COUNT: usize = 136;

// DMAC peripheral base (ATSAME51 datasheet Table 10-1).
const DMAC_BASE: u32 = 0x4100_A000;
// DMAC register block + channel registers. MPU region must be power-of-2.
const DMAC_SIZE: u32 = 0x100;
// PORT base (GPIO registers) — used as second peripheral region
// (kernel requires exactly 2 peripheral-reserved slots).
const PORT_BASE: u32 = 0x4100_8000;
const PORT_SIZE: u32 = 0x100;

const P0_PID: PartitionId = PartitionId::new(0);
const P1_PID: PartitionId = PartitionId::new(1);
const BUF_READY: EventMask = EventMask::new(0x1);
const BUF_DONE: EventMask = EventMask::new(0x2);

// Compile-time pattern: 0x00..0x3F
const PATTERN: [u8; BUF_LEN] = {
    let mut a = [0u8; BUF_LEN];
    let mut i = 0usize;
    while i < BUF_LEN { a[i] = i as u8; i += 1; }
    a
};

static CYCLE: AtomicU32 = AtomicU32::new(0);
static VERIFY_OK: AtomicU32 = AtomicU32::new(0);
static VERIFY_FAIL: AtomicU32 = AtomicU32::new(0);
static DMA_ERR: AtomicU32 = AtomicU32::new(0);
static API_ERR: AtomicU32 = AtomicU32::new(0);

// DMA channel passed from main() to partition.
type DmaChan = dmac::Channel<dmac::Ch0, dmac::Ready>;
static mut DMA_CHAN: Option<DmaChan> = None;

kernel::kernel_config!(BufDmaConfig[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = same51_curiosity::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
    buffer_pool_regions = 1;
    buffer_zone_size = 32;
});

unsafe extern "C" fn dummy_irq() {}

kernel::bind_interrupts!(BufDmaConfig, IRQ_COUNT,
    DUMMY_IRQ => (0, 0x0000_0001, handler: dummy_irq),
);

kernel::define_kernel!(BufDmaConfig, |tick, _k| {
    if tick % 1000 == 0 {
        let cycle = CYCLE.load(Ordering::Acquire);
        let verify_ok = VERIFY_OK.load(Ordering::Acquire);
        let verify_fail = VERIFY_FAIL.load(Ordering::Acquire);
        let dma_err = DMA_ERR.load(Ordering::Acquire);
        let api_err = API_ERR.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] CYCLE={} VERIFY_OK={} VERIFY_FAIL={} DMA_ERR={} API_ERR={}",
            tick, cycle, verify_ok, verify_fail, dma_err, api_err
        );
        if cycle > 5 && verify_fail == 0 && dma_err == 0 && api_err == 0 && cycle > 5 {
            rprintln!("SUCCESS: SAME51 DMA+buffer pool working! CYCLE={}", cycle);
        }
    }
});

// Static DMA buffers — must be 'static for Transfer::new_from_arrays.
static mut DMA_SRC: [u8; BUF_LEN] = [0u8; BUF_LEN];
static mut DMA_DST: [u8; BUF_LEN] = [0u8; BUF_LEN];

// ---------------------------------------------------------------------------
// P0 — DMA producer
// ---------------------------------------------------------------------------
extern "C" fn producer_body(_r0: u32) -> ! {
    let chan = unsafe { (*addr_of_mut!(DMA_CHAN)).take().expect("dma chan") };
    let mut chan_slot: Option<DmaChan> = Some(chan);

    loop {
        // 1. Alloc writable buffer slot.
        let slot = loop {
            match plib::sys_buf_alloc(true, 0) {
                Ok(s) => break s,
                Err(_) => {
                    API_ERR.fetch_add(1, Ordering::Release);
                    plib::sys_yield().ok();
                }
            }
        };

        // 2. Prepare source pattern and clear destination.
        // SAFETY: P0 is the only partition accessing these statics.
        let (src, dst) = unsafe {
            let s = &mut *addr_of_mut!(DMA_SRC);
            let d = &mut *addr_of_mut!(DMA_DST);
            s.copy_from_slice(&PATTERN);
            d.fill(0xFF);
            // We need &'static mut references for Transfer::new_from_arrays.
            // SAFETY: These statics are only used by P0, and we wait() before
            // the next cycle, so no aliasing.
            (
                &mut *(s as *mut [u8; BUF_LEN]),
                &mut *(d as *mut [u8; BUF_LEN]),
            )
        };

        // 3. DMA M2M transfer: src → dst.
        let ch = chan_slot.take().expect("channel available");
        let xfer = Transfer::new_from_arrays(ch, src, dst, false)
            .begin(TriggerSource::Disable, TriggerAction::Block);
        let (returned_chan, _src_ret, dst_ret) = xfer.wait();
        chan_slot = Some(returned_chan);

        // 4. Verify DMA result locally before committing to buffer pool.
        let dma_ok = dst_ret.iter().enumerate().all(|(i, &b)| b == i as u8);
        if !dma_ok {
            DMA_ERR.fetch_add(1, Ordering::Release);
            if let Err(e) = plib::sys_buf_release(slot) {
                rprintln!("[P0] sys_buf_release failed: {:?}", e);
            }
            continue;
        }

        // 5. Write DMA result into kernel buffer slot.
        if plib::sys_buf_write(slot, dst_ret).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
            if let Err(e) = plib::sys_buf_release(slot) {
                rprintln!("[P0] sys_buf_release failed: {:?}", e);
            }
            continue;
        }

        // 6. Lend to P1 (read-only).
        if plib::sys_buf_lend(slot, 1, false).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
            if let Err(e) = plib::sys_buf_release(slot) {
                rprintln!("[P0] sys_buf_release failed: {:?}", e);
            }
            continue;
        }

        // 7. Signal P1.
        if let Err(e) = plib::sys_event_set(P1_PID, BUF_READY) {
            rprintln!("[P0] sys_event_set failed: {:?}", e);
        }

        // 8. Wait for P1 to finish reading.
        loop {
            match plib::sys_event_wait(BUF_DONE) {
                Err(_) => continue,
                Ok(bits) if bits.as_raw() == 0 => continue,
                Ok(_) => break,
            }
        }

        // 9. Revoke + release.
        if plib::sys_buf_revoke(slot, 1).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
        }
        if plib::sys_buf_release(slot).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
        }

        CYCLE.fetch_add(1, Ordering::Release);
    }
}

// ---------------------------------------------------------------------------
// P1 — Verifier
// ---------------------------------------------------------------------------
extern "C" fn verifier_body(_r0: u32) -> ! {
    let buffer_slot = BufferSlotId::new(0);
    loop {
        // Wait for P0 to signal buffer is ready.
        loop {
            match plib::sys_event_wait(BUF_READY) {
                Err(_) => continue,
                Ok(bits) if bits.as_raw() == 0 => continue,
                Ok(_) => break,
            }
        }

        // Read from kernel buffer and verify.
        let mut buf = [0u8; BUF_LEN];
        match plib::sys_buf_read(buffer_slot, &mut buf) {
            Ok(_) => {
                if buf.iter().enumerate().all(|(i, &b)| b == i as u8) {
                    VERIFY_OK.fetch_add(1, Ordering::Release);
                } else {
                    VERIFY_FAIL.fetch_add(1, Ordering::Release);
                }
            }
            Err(_) => {
                API_ERR.fetch_add(1, Ordering::Release);
            }
        }

        // Signal P0: done.
        if let Err(e) = plib::sys_event_set(P0_PID, BUF_DONE) {
            rprintln!("[P1] sys_event_set failed: {:?}", e);
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
    rprintln!("=== Buffer Pool + DMA M2M Demo — SAME51 Curiosity Nano ===");

    let mut pac_peripherals = hal::pac::Peripherals::take().unwrap();
    let p = cortex_m::Peripherals::take().unwrap();

    let _clocks = hal::clock::GenericClockController::with_internal_32kosc(
        pac_peripherals.gclk,
        &mut pac_peripherals.mclk,
        &mut pac_peripherals.osc32kctrl,
        &mut pac_peripherals.oscctrl,
        &mut pac_peripherals.nvmctrl,
    );

    // Init DMAC and hand channel 0 to the partition.
    let mut dmac = DmaController::init(pac_peripherals.dmac, &mut pac_peripherals.pm);
    let channels = dmac.split();
    let chan0 = channels.0.init(PriorityLevel::Lvl0);
    unsafe { *addr_of_mut!(DMA_CHAN) = Some(chan0); }
    rprintln!("[INIT] DMAC initialised, channel 0 ready.");

    // Kernel setup.
    let mut sched = ScheduleTable::<{ BufDmaConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(2).expect("sched SW");

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let data_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code_mpu = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    // P0: SRAM + flash + DMAC peripheral region.
    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, producer_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0)
        )
        .expect("mem0")
        .with_code_mpu_region(code_mpu)
        .expect("code0")
        .with_peripheral_regions(&[
            MpuRegion::new(DMAC_BASE, DMAC_SIZE, 0),
            MpuRegion::new(PORT_BASE, PORT_SIZE, 0),
        ])
        .expect("periph0");

    // P1: SRAM + flash only (no peripheral access needed).
    let mem1 = ExternalPartitionMemory::from_aligned_stack(
            s1, verifier_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(1)
        )
        .expect("mem1")
        .with_code_mpu_region(code_mpu)
        .expect("code1");

    let mems = [mem0, mem1];
    let mut k = Kernel::<BufDmaConfig>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);
    rprintln!("[INIT] Kernel created. Buffer pool: 1 slot x 64 bytes.");
    rprintln!("[INIT] P0: DMAC grant at 0x{:08X}/{}. Pattern: 0x00..0x{:02X}",
        DMAC_BASE, DMAC_SIZE, BUF_LEN as u8 - 1);

    rprintln!("[INIT] Booting with MPU enabled...\n");
    match boot(p).expect("boot") {}
}

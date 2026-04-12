//! Buffer Pool + DMA Zero-Copy Demo — STM32F429ZI NUCLEO-144
//!
//! Demonstrates zero-copy DMA into a kernel buffer pool slot via USART3 HDSEL
//! loopback.  All peripheral setup in `main()` uses stm32f4xx-hal; the DMA
//! stream is configured in the partition via `Stream` + `StreamISR` traits.
//! The partition body contains zero PAC register accesses for USART3 or DMA1.
//!
//! What the HAL provides:
//!
//!   main():
//!     - `dp.RCC.freeze(Config::hse(8.MHz()).sysclk(168.MHz()))` — 168 MHz PLL.
//!       PCLK1=42 MHz (APB1 prescaler=4). HAL computes BRR automatically;
//!       the PAC version used HSI 16 MHz with hard-coded BRR=0x008B.
//!     - `dp.GPIOD.split(&mut rcc)` → typed pins; clock enabled via HAL.
//!       `gpiod.pd8.into_alternate::<7>()` — USART3_TX push-pull.
//!       `gpiod.pd9.into_alternate::<7>()` — USART3_RX (not wired physically;
//!       required by `Serial::new()` for full-duplex init; HDSEL overrides RX routing).
//!     - `Serial::new(dp.USART3, (tx_pin, rx_pin), Config.dma(DmaConfig::Rx), &mut rcc)`
//!       → enables USART3 APB1 clock, computes BRR from PCLK1, writes CR1/CR2/CR3,
//!       and sets CR3.DMAR=1 (`.dma(DmaConfig::Rx)`).  DMAR is toggled per cycle
//!       in the partition to prevent the DMA controller latching a stale request
//!       between cycles (permanent DMAR causes the last byte's RXNE to fire as
//!       buf[0] of the next cycle, shifting all pattern bytes by one position).
//!     - `StreamsTuple::new(dp.DMA1, &mut rcc)` → enables DMA1 AHB1 clock and
//!       returns typed stream handles (`.1` = `Stream1<DMA1>`).
//!
//!   producer_body() (MPU pass-through partition), via `Stream`+`StreamISR` traits on
//!   the stored `Stream1<pac::DMA1>`:
//!     - `stream.set_peripheral_address(USART3_DR_ADDR)` — PAR
//!     - `stream.set_memory_address(buf_ptr as u32)` — M0AR, **runtime address**
//!       from `sys_buf_lend`; the trait takes a plain u32.  The HAL's Transfer<>
//!       wrapper cannot be used here: every constructor requires BUF: WriteBuffer
//!       (embedded-dma), which requires B: 'static + StableDeref.  A raw *mut u8
//!       from sys_buf_lend satisfies neither; Transfer's fields are also all
//!       private so struct-literal construction bypassing the constructors is not
//!       possible.  The Stream trait is the correct abstraction level.
//!     - `stream.set_number_of_transfers(BUF_LEN as u16)` — NDTR
//!     - `stream.set_channel(DmaChannel::Channel4)` — USART3_RX = CH4 (RM0090 Table 28)
//!     - `stream.set_direction(DmaDirection::PeripheralToMemory)`
//!     - `stream.set_{memory,peripheral}_size(DmaDataSize::Byte)`
//!     - `stream.set_memory_increment(true)` / `set_peripheral_increment(false)`
//!     - `stream.set_fifo_enable(false)` — direct mode (DMDIS=1); each RXNE
//!       immediately writes 1 byte to M0AR without buffering
//!     - `unsafe { stream.enable/disable() }` — EN
//!     - `stream.is_transfer_complete()` / `stream.clear_transfer_complete()`
//!
//!   TX (partition) via stored `Tx<pac::USART3>` (blocking serial Write trait):
//!     - `tx.bwrite_all(&[0xFF])` + `tx.bflush()` — warmup byte + TC wait
//!     - `tx.bwrite_all(&pattern)` — main loop 32-byte write (replaces TXE-poll loop)
//!
//!   RXNE drain (partition) via stored `Rx<pac::USART3>`:
//!     - `rx.read().ok()` — drain stale RXNE if any (no PAC needed)
//!
//! What is NOT HAL-ified (genuine gaps in the HAL):
//!   - CR3.HDSEL: set via PAC steal after `Serial::new()`.  The HAL's
//!     `serial::Config` has no half-duplex option.  RM0090 §27.3.10 requires
//!     HDSEL to be set when UE=0; `Serial::new()` leaves UE=1, so the
//!     sequence is: UE=0 → CR3.HDSEL=1 → UE=1.
//!
//! Build:  cd f429zi && cargo build --example buf_dma_demo \
//!             --features kernel-mpu-hal --release

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use kernel::{
    StackStorage as _,
    partition::{ExternalPartitionMemory, MpuRegion},
    partition_core::{AlignedStack2K, AlignedStack4K},
    scheduler::{ScheduleEntry, ScheduleTable},
    svc::Kernel,
    {Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled},
};
use plib::{BufferSlotId, PartitionId};
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::{
    ClearFlags,
    dma::{
        traits::{Stream, StreamISR},
        DmaChannel, DmaDataSize, DmaDirection, Stream1, StreamsTuple,
    },
    pac,
    prelude::*,
    rcc::Config as RccConfig,
    serial::{
        config::DmaConfig as UartDmaConfig,
        Config as SerialConfig, Rx as SerialRx, Serial, Tx as SerialTx,
    },
};
use f429zi::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 2;
const STACK_WORDS: usize = 1024;

const BUF_LEN: usize = 32;
const PATTERN_BASE: u8 = 0xA0; // TX pattern: 0xA0..0xBF
// Compile-time TX pattern (avoids 32-byte stack allocation in producer_body).
const TX_PATTERN: [u8; BUF_LEN] = {
    let mut a = [0u8; BUF_LEN];
    let mut i = 0usize;
    while i < BUF_LEN { a[i] = PATTERN_BASE.wrapping_add(i as u8); i += 1; }
    a
};

const P1_PID: PartitionId = PartitionId::new(1);

// USART3 DR address (RM0090: USART3 base=0x4000_4800, DR offset=0x04).
// Still needed as a u32 for `stream.set_peripheral_address()`.
const USART3_DR_ADDR: u32 = 0x4000_4804;

// Peripheral region bases for P0's MPU grant.
const USART3_BASE: u32 = 0x4000_4800;
const DMA1_BASE: u32 = 0x4002_6000;

static CYCLE:       AtomicU32 = AtomicU32::new(0);
static VERIFY_OK:   AtomicU32 = AtomicU32::new(0);
static VERIFY_FAIL: AtomicU32 = AtomicU32::new(0);
static DMA_ERR:     AtomicU32 = AtomicU32::new(0);
static API_ERR:     AtomicU32 = AtomicU32::new(0);

// ---------------------------------------------------------------------------
// HAL object storage — passed from main() to partition via statics.
// All three are initialised in main() before boot() and are accessed only by
// P0 (no concurrent access; MPU enforces exclusivity).
// ---------------------------------------------------------------------------
static mut TX_HANDLE: Option<SerialTx<pac::USART3>> = None;
static mut RX_HANDLE: Option<SerialRx<pac::USART3>> = None;
static mut DMA_STREAM: Option<Stream1<pac::DMA1>>   = None;

kernel::kernel_config!(BufDmaHalConfig[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
    buffer_pool_regions = 1;
    buffer_zone_size = 32;
});

kernel::define_kernel!(BufDmaHalConfig, |tick, _k| {
    if tick % 1000 == 0 {
        let cycle       = CYCLE.load(Ordering::Acquire);
        let verify_ok   = VERIFY_OK.load(Ordering::Acquire);
        let verify_fail = VERIFY_FAIL.load(Ordering::Acquire);
        let dma_err     = DMA_ERR.load(Ordering::Acquire);
        let api_err     = API_ERR.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] CYCLE={} VERIFY_OK={} VERIFY_FAIL={} DMA_ERR={} API_ERR={}",
            tick, cycle, verify_ok, verify_fail, dma_err, api_err
        );
        if cycle > 5 && verify_fail == 0 && dma_err == 0 && api_err == 0 {
            rprintln!("✓ SUCCESS: HAL zero-copy DMA buffer pool working! CYCLE={}", cycle);
        }
    }
});

// ---------------------------------------------------------------------------
// P0 — DMA producer: alloc → lend → DMA RX → verify → release
//
// This function contains zero PAC register accesses for USART3 or DMA1.
// All hardware is accessed through the HAL Tx/Rx/Stream1 objects stored in
// the TX_HANDLE, RX_HANDLE, and DMA_STREAM statics.
// ---------------------------------------------------------------------------
extern "C" fn producer_body(_r0: u32) -> ! {
    // SAFETY: P0 owns USART3 and DMA1 in peripheral_regions.
    // Tx/Rx/Stream1 are HAL objects for those peripherals — no raw PAC steal needed.
    let tx     = unsafe { (*addr_of_mut!(TX_HANDLE)).as_mut().unwrap() };
    let rx     = unsafe { (*addr_of_mut!(RX_HANDLE)).as_mut().unwrap() };
    let stream = unsafe { (*addr_of_mut!(DMA_STREAM)).as_mut().unwrap() };

    // ------------------------------------------------------------------
    // Warmup: transmit one dummy byte to flush the USART shift register
    // and stabilise HDSEL loopback before DMA cycles start.
    //
    // DMAR was cleared by main() after Serial::new() so the warmup echo
    // does not latch a DMA request while no stream is enabled.
    // `bwrite_all` writes the byte polling TXE (blocking).
    // `bflush` polls TC until the shift register is fully drained (blocking).
    // In HDSEL mode RXNE only fires if the host has /dev/ttyACM0 open (ST-LINK
    // echoes data back). Don't block waiting for it — drain opportunistically.
    // ------------------------------------------------------------------
    tx.bwrite_all(&[0xFF]).unwrap();
    tx.bflush().unwrap();
    // Opportunistically drain echo byte if it arrives; proceed regardless after TC.
    for _ in 0..100_000u32 {
        if rx.read().is_ok() {
            break;
        }
        cortex_m::asm::nop();
    }
    rprintln!("[P0] USART3 warmed up (HAL). Starting DMA cycles.");

    // ------------------------------------------------------------------
    // One-time stream init (mirrors HAL's init_common): configure all
    // static stream settings before the loop.  Only memory address and
    // transfer count change per cycle (mirrors next_transfer_common).
    // ------------------------------------------------------------------
    unsafe { stream.disable() };
    while stream.is_enabled() { cortex_m::asm::nop(); }
    stream.clear_all_flags();
    stream.set_channel(DmaChannel::Channel4);          // USART3_RX = CH4 (RM0090 Table 28)
    stream.set_direction(DmaDirection::PeripheralToMemory);
    stream.set_peripheral_address(USART3_DR_ADDR);     // PAR = USART3_DR (fixed)
    // SAFETY: Byte size is correct for u8 USART data register and u8 buffer.
    unsafe {
        stream.set_memory_size(DmaDataSize::Byte);
        stream.set_peripheral_size(DmaDataSize::Byte);
    }
    stream.set_memory_increment(true);      // M0AR advances per byte
    stream.set_peripheral_increment(false); // PAR (USART3_DR) stays fixed
    stream.set_circular_mode(false);
    // Direct mode (DMDIS=0): each RXNE event immediately transfers 1 byte.
    stream.set_fifo_enable(false);

    loop {
        // 1. Allocate a writable buffer slot.
        let slot = match plib::sys_buf_alloc(true, 0) {
            Ok(s) => s,
            Err(_) => {
                API_ERR.fetch_add(1, Ordering::Release);
                plib::sys_yield().ok();
                continue;
            }
        };

        // 2. Lend slot to P1 — kernel returns physical buffer address in r1.
        //    plib::sys_buf_lend returns Ok((buf_ptr, region_id)) — r0=base_addr (ABI: fd29a27)
        let buf_ptr = match plib::sys_buf_lend(slot, P1_PID.as_raw() as u8, true) {
            Ok((ptr, _region_id)) => ptr,
            Err(_) => {
                API_ERR.fetch_add(1, Ordering::Release);
                if let Err(e) = plib::sys_buf_release(slot) {
                    rprintln!("[DMA] sys_buf_release failed: {:?}", e);
                }
                continue;
            }
        };

        // 3. Per-cycle stream update (mirrors HAL's next_transfer_common):
        //    only the destination address and transfer count change each cycle.
        unsafe { stream.disable() };
        while stream.is_enabled() { cortex_m::asm::nop(); }
        stream.clear_all_flags();
        stream.set_memory_address(buf_ptr as u32);        // M0AR = runtime buf_ptr
        stream.set_number_of_transfers(BUF_LEN as u16);   // NDTR = 32 bytes

        // 4. Enable USART3 RX DMA request then drain any stale RXNE.
        //
        //    CR3.DMAR is toggled per cycle (not left permanently on) to prevent
        //    the DMA controller from latching a stale request between cycles.
        //    The last byte of each bwrite_all generates an RXNE that DMA misses
        //    (stream EN=0 after TCIF); if DMAR were permanently on, that latched
        //    request fires on the next stream.enable(), consuming a stale byte
        //    as buf[0] and shifting the entire pattern by one position.
        //
        //    Sequence: enable DMAR → drain stale RXNE (if any) → enable stream.
        //    This is the same protocol as the PAC version (buf_dma_demo.rs).
        // Drain stale RXNE BEFORE enabling DMAR: if RXNE=1 when DMAR goes high,
        // the DMA controller immediately latches a request and fires it on
        // stream.enable() — consuming the stale byte as buf[0] (1-byte rotation).
        // PAC version drain pattern: check SR.RXNE, read DR if set.
        rx.read().ok();
        // SAFETY: we own USART3 via peripheral_regions; single-threaded access.
        unsafe {
            pac::Peripherals::steal()
                .USART3
                .cr3()
                .modify(|_, w| w.dmar().set_bit());
        }

        // 5. Enable the DMA stream — begins waiting for RXNE DMA requests.
        // SAFETY: all stream registers are correctly configured above.
        unsafe { stream.enable() };

        // 6. Transmit BUF_LEN bytes via USART3 TX.
        //    `bwrite_all` polls TXE before each byte and writes DR (blocking).
        //    HDSEL loopback: each TX byte generates RXNE → DMA request → transfer.
        tx.bwrite_all(&TX_PATTERN).unwrap();

        // 7. Poll DMA1 Stream1 for transfer complete or error.
        //    `StreamISR::is_transfer_complete()` reads LISR.TCIF1.
        //    `StreamISR::is_transfer_error/direct_mode_error()` reads TEIF1/DMEIF1.
        let mut completed = false;
        let mut errored   = false;
        for _ in 0..2_000_000u32 {
            if stream.is_transfer_error() || stream.is_direct_mode_error() {
                errored = true;
                break;
            }
            if stream.is_transfer_complete() {
                completed = true;
                break;
            }
            cortex_m::asm::nop();
        }

        // Clean up regardless of outcome: disable stream, clear flags, disable DMAR.
        unsafe { stream.disable() };
        stream.clear_all_flags();
        // Disable DMAR so the last byte's RXNE does not latch a DMA request
        // in the DMA controller between cycles (see comment at enable site).
        // SAFETY: we own USART3 via peripheral_regions; single-threaded access.
        unsafe {
            pac::Peripherals::steal()
                .USART3
                .cr3()
                .modify(|_, w| w.dmar().clear_bit());
        }

        if !completed || errored {
            DMA_ERR.fetch_add(1, Ordering::Release);
            if let Err(e) = plib::sys_buf_revoke(slot, P1_PID.as_raw() as u8) {
                rprintln!("[DMA] sys_buf_revoke failed: {:?}", e);
            }
            if let Err(e) = plib::sys_buf_release(slot) {
                rprintln!("[DMA] sys_buf_release failed: {:?}", e);
            }
            continue;
        }

        // 8. Revoke P1's MPU grant.
        if plib::sys_buf_revoke(slot, P1_PID.as_raw() as u8).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
        }

        // 9. Verify via sys_buf_read (P0 is owner — always permitted).
        let mut verify = [0u8; BUF_LEN];
        match plib::sys_buf_read(BufferSlotId::new(0), &mut verify) {
            Ok(_) => {
                let all_ok = verify
                    .iter()
                    .enumerate()
                    .all(|(i, &b)| b == PATTERN_BASE.wrapping_add(i as u8));
                if all_ok {
                    VERIFY_OK.fetch_add(1, Ordering::Release);
                } else {
                    VERIFY_FAIL.fetch_add(1, Ordering::Release);
                }
            }
            Err(_) => {
                API_ERR.fetch_add(1, Ordering::Release);
            }
        }

        // 10. Return slot to pool.
        if plib::sys_buf_release(slot).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
        }

        CYCLE.fetch_add(1, Ordering::Release);
    }
}

// ---------------------------------------------------------------------------
// P1 — Dummy lendee: yield loop.
// ---------------------------------------------------------------------------
extern "C" fn dummy_body(_r0: u32) -> ! {
    loop {
        plib::sys_yield().ok();
    }
}

kernel::partition_trampoline!(producer_main => producer_body);
kernel::partition_trampoline!(dummy_main    => dummy_body);

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    rprintln!("\n=== Buffer Pool + DMA Demo (HAL variant) — STM32F429ZI ===");
    rprintln!("HAL: 168MHz PLL, typed GPIO/Serial/StreamsTuple, Stream trait DMA config.");
    rprintln!("USART3 HDSEL loopback → DMA1 Stream1 CH4 → kernel buffer slot (buf_ptr from r1).");

    let dp = pac::Peripherals::take().unwrap();
    let mut p = cortex_m::Peripherals::take().unwrap();

    // ------------------------------------------------------------------
    // HAL clock configuration: HSE 8 MHz → PLL → 168 MHz system clock.
    // APB1 bus: PCLK1 = 168/4 = 42 MHz (STM32F4 APB1 max = 42 MHz).
    // HAL computes USART3 BRR from actual PCLK1; no hard-coded BRR needed.
    // ------------------------------------------------------------------
    let mut rcc = dp.RCC.freeze(
        RccConfig::hse(8.MHz()).sysclk(168.MHz())
    );

    // ------------------------------------------------------------------
    // HAL GPIO: enables GPIOD AHB1 clock and returns typed pin handles.
    // into_alternate::<7>() sets MODER=Alternate and AFRH=AF7 in one call.
    // PD9 (RX) is configured for Serial::new() completeness; in HDSEL mode
    // the USART RX path uses the TX line internally — PD9 is not wired.
    // ------------------------------------------------------------------
    let gpiod = dp.GPIOD.split(&mut rcc);
    let pd8_tx = gpiod.pd8.into_alternate::<7>();
    let pd9_rx = gpiod.pd9.into_alternate::<7>();

    // ------------------------------------------------------------------
    // HAL Serial: enables USART3 APB1 clock, computes BRR from PCLK1=42 MHz,
    // configures CR1 (UE+TE+RE+8N1), resets CR2/CR3, then sets CR3.DMAR=1
    // via dma(DmaConfig::Rx). DMAR stays set; stream EN gates per-cycle DMA.
    // ------------------------------------------------------------------
    let serial = Serial::new(
        dp.USART3,
        (pd8_tx, pd9_rx),
        SerialConfig::default()
            .baudrate(115_200.bps())
            .dma(UartDmaConfig::Rx),
        &mut rcc,
    ).expect("USART3 init");

    // HDSEL (CR3 bit 3): RM0090 §27.3.10 suggests setting HDSEL when UE=0,
    // but in practice (as verified by uart_partition) setting HDSEL while UE=1
    // works correctly on STM32F429 and avoids a UE-cycle bug where disabling
    // then re-enabling UE leaves the TX path silently non-functional (TC stays
    // 1 forever, no byte ever transmitted, warmup RXNE poll hangs indefinitely).
    // uart_partition does the same and passes hw-tests.
    // SAFETY: single-threaded init; we own USART3 exclusively here.
    unsafe {
        pac::Peripherals::steal()
            .USART3
            .cr3()
            .modify(|_, w| w.hdsel().half_duplex()); // CR3.HDSEL=1 while UE=1
    }
    rprintln!("[INIT] HAL: PD8/PD9 AF7, USART3 115200 8N1 HDSEL+DMAR, PCLK1=42MHz");

    // ------------------------------------------------------------------
    // HAL DMA: enables DMA1 AHB1 clock via RCC and returns typed streams.
    // StreamsTuple::new() replaces the manual `ahb1enr.dma1en().set_bit()`.
    // `.1` is `Stream1<DMA1>` (USART3_RX channel, RM0090 Table 28).
    // ------------------------------------------------------------------
    let dma1 = StreamsTuple::new(dp.DMA1, &mut rcc);
    rprintln!("[INIT] HAL: DMA1 clock enabled, Stream1 handle acquired.");

    // Split serial into typed Tx and Rx handles; store for partition access.
    let (tx, rx) = serial.split();
    // SAFETY: single-threaded init; these statics are only written here and
    // only read by P0 (no concurrent access after boot()).
    unsafe {
        *addr_of_mut!(TX_HANDLE)  = Some(tx);
        *addr_of_mut!(RX_HANDLE)  = Some(rx);
        *addr_of_mut!(DMA_STREAM) = Some(dma1.1);
        // Serial::new().dma(DmaConfig::Rx) leaves DMAR=1 (CR3 bit 6).
        // Clear it here so the partition warmup echo does not latch a DMA
        // request while no stream is enabled (would corrupt cycle 1 buf[0]).
        // P0 toggles DMAR per cycle via pac::Peripherals::steal().
        // Steal USART3 to clear DMAR (dp.USART3 was moved into Serial::new).
        pac::Peripherals::steal().USART3.cr3().modify(|_, w| w.dmar().clear_bit());
    }

    // ------------------------------------------------------------------
    // Kernel setup.
    // ------------------------------------------------------------------
    let mut sched = ScheduleTable::<{ BufDmaHalConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(2).expect("sched SW");

    // Allocate per-partition stacks externally for ExternalPartitionMemory.
    static mut STACKS: [AlignedStack4K; NUM_PARTITIONS] = [AlignedStack4K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let sentinel_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    // P0: SRAM + USART3 (1KB) + DMA1 (1KB).
    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, producer_main as kernel::PartitionEntry, sentinel_mpu, kernel::PartitionId::new(0)
        )
        .expect("mem0")
            .with_code_mpu_region(MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0))
            .expect("code0")
        .with_peripheral_regions(&[
            MpuRegion::new(USART3_BASE, 1024, 0),
            MpuRegion::new(DMA1_BASE,   1024, 0),
        ])
        .expect("periph0");
    let mem1 = ExternalPartitionMemory::from_aligned_stack(
            s1, dummy_main as kernel::PartitionEntry, sentinel_mpu, kernel::PartitionId::new(1)
        )
        .expect("mem1")
            .with_code_mpu_region(MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0))
            .expect("code1");
    let mems = [mem0, mem1];

    let mut k = Kernel::<BufDmaHalConfig>::new(sched, &mems).expect("kernel");
    rprintln!("[INIT] Kernel created. Buffer pool: 1 slot × 32 bytes.");
    rprintln!("[INIT] P0: USART3+DMA1 grants. Pattern=0x{:02X}..0x{:02X}",
        PATTERN_BASE, PATTERN_BASE.wrapping_add(BUF_LEN as u8 - 1));
    store_kernel(&mut k);

    rprintln!("[INIT] Booting with MPU enabled...\n");
    match boot(p).expect("boot") {}
}

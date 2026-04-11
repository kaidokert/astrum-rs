//! UART DMA Transfer Demo — SAME51 Curiosity Nano
//!
//! Same as uart_dma_demo but uses `sys_buf_transfer` (permanent ownership
//! handoff) instead of lend/revoke.  P0 allocates, fills, and transfers
//! the buffer slot to P1.  P1 reads, verifies, releases, and signals P0.
//!
//! Protocol (one cycle):
//!   1. MCU sends "READY\n" via CPU TX
//!   2. Host sends 32-byte pattern (0x00..0x1F)
//!   3. DMAC receives into static buffer
//!   4. MCU copies DMA result into kernel buffer slot via sys_buf_write
//!   5. Transfer ownership to P1; P1 reads, verifies, releases
//!   6. MCU sends "OK\n" or "ERR:...\n" via CPU TX
//!
//! Build: cd same51_curiosity && cargo build --example uart_dma_transfer_demo \
//!            --features kernel-mpu --release
//! Flash: probe-rs download --chip ATSAME51J20A --probe 03eb:2175 <elf>
//! Host:  python3 same51_curiosity/uart_dma_host.py --port /dev/ttyACM4 --cycles 50
//!
//! ⚠ HOST TIMING: The MCU sends READY immediately on boot. If the host serial
//! port isn't open before reset, the first READY is lost and the MCU hangs in
//! xfer.wait() forever (looks like "TX not working" — it IS working, nobody's
//! listening). Fix: open the port BEFORE resetting the target (e.g. flash via
//! OpenOCD/GDB while the port stays open, or use probe-rs and add a delay/retry).

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicU32, Ordering};
use atsamd_hal as hal;
use cortex_m_rt::{entry, exception};
use hal::clock::GenericClockController;
use hal::dmac::{self, DmaController, PriorityLevel};
use hal::fugit::RateExtU32;
use hal::pac;
use hal::sercom::{uart, Sercom2};
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

// Peripheral bases for MPU grants.
const DMAC_BASE: u32 = 0x4100_A000;
const DMAC_SIZE: u32 = 0x100;
const SERCOM2_BASE: u32 = 0x4101_2000;
const SERCOM2_SIZE: u32 = 0x40;

const P0_PID: PartitionId = PartitionId::new(0);
const P1_PID: PartitionId = PartitionId::new(1);
const BUF_READY: EventMask = EventMask::new(0x1);
const BUF_DONE: EventMask = EventMask::new(0x2);

static CYCLE: AtomicU32 = AtomicU32::new(0);
static VERIFY_OK: AtomicU32 = AtomicU32::new(0);
static VERIFY_FAIL: AtomicU32 = AtomicU32::new(0);
static DMA_ERR: AtomicU32 = AtomicU32::new(0);
static API_ERR: AtomicU32 = AtomicU32::new(0);

// Full-duplex UART types.
type VcomPads = uart::PadsFromIds<Sercom2, hal::gpio::PA13, hal::gpio::PA12>;
type VcomConfig = uart::Config<VcomPads>;
type RxHalf = uart::Uart<VcomConfig, uart::RxDuplex>;
type TxHalf = uart::Uart<VcomConfig, uart::TxDuplex>;
type RxDmaChan = dmac::Channel<dmac::Ch0, dmac::Ready>;

// HAL objects passed from main() to partition.
static mut TX_HANDLE: Option<TxHalf> = None;
static mut RX_HANDLE: Option<RxHalf> = None;
static mut DMA_RX_CHAN: Option<RxDmaChan> = None;

kernel::kernel_config!(UartDmaConfig[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
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

kernel::bind_interrupts!(UartDmaConfig, IRQ_COUNT,
    DUMMY_IRQ => (0, 0x0000_0001, handler: dummy_irq),
);

kernel::define_kernel!(UartDmaConfig, |tick, _k| {
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
        if cycle > 5 && verify_fail == 0 && api_err == 0 {
            rprintln!("SUCCESS: SAME51 UART DMA transfer demo working! CYCLE={}", cycle);
        }
    }
});

// Static DMA receive buffer — must be 'static for Transfer API.
static mut DMA_RX_BUF: [u8; BUF_LEN] = [0u8; BUF_LEN];

// ---------------------------------------------------------------------------
// P0 — UART DMA receiver + buffer pool producer (transfers ownership to P1)
// ---------------------------------------------------------------------------
extern "C" fn producer_body(_r0: u32) -> ! {
    use embedded_hal_02::serial::Write as EhWrite;
    use hal::nb;

    let tx = unsafe { (*addr_of_mut!(TX_HANDLE)).take().expect("tx") };
    let rx = unsafe { (*addr_of_mut!(RX_HANDLE)).take().expect("rx") };
    let chan = unsafe { (*addr_of_mut!(DMA_RX_CHAN)).take().expect("dma chan") };

    let mut tx_slot: Option<TxHalf> = Some(tx);
    let mut rx_slot: Option<RxHalf> = Some(rx);
    let mut chan_slot: Option<RxDmaChan> = Some(chan);

    loop {
        // 1. Alloc buffer slot.
        let slot = loop {
            match plib::sys_buf_alloc(true, 0) {
                Ok(s) => break s,
                Err(_) => {
                    API_ERR.fetch_add(1, Ordering::Release);
                    plib::sys_yield().ok();
                }
            }
        };

        // 2. Signal host BEFORE arming DMA — ensures READY goes out.
        {
            let mut tx = tx_slot.take().expect("tx available");
            for &byte in b"READY\n" {
                nb::block!(EhWrite::write(&mut tx, byte)).ok();
            }
            // Flush: wait for TX complete before arming DMA RX.
            while !tx.read_flags().contains(uart::Flags::TXC) {}
            tx.clear_flags(uart::Flags::TXC);
            tx_slot = Some(tx);
        }

        // 3. Clear DMA buffer and arm DMA RX.
        let dma_buf = unsafe {
            let b = &mut *addr_of_mut!(DMA_RX_BUF);
            b.fill(0xFF);
            &mut *(b as *mut [u8; BUF_LEN])
        };

        let rx_uart = rx_slot.take().expect("rx available");
        let ch = chan_slot.take().expect("chan available");
        let xfer = rx_uart.receive_with_dma(dma_buf, ch);

        // 4. Wait for DMA completion (32 bytes from host).
        let (returned_chan, returned_rx, returned_buf) = xfer.wait();
        chan_slot = Some(returned_chan);
        rx_slot = Some(returned_rx);

        // 5. Write DMA result into kernel buffer slot.
        if plib::sys_buf_write(slot, returned_buf).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
            if let Err(e) = plib::sys_buf_release(slot) {
                rprintln!("[P0] sys_buf_release failed: {:?}", e);
            }
            continue;
        }

        // 6. Transfer ownership to P1 (permanent handoff — P0 loses access).
        if plib::sys_buf_transfer(slot, P1_PID.as_raw() as u8).is_err() {
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

        // 8. Wait for P1 to finish verifying.
        loop {
            match plib::sys_event_wait(BUF_DONE) {
                Err(_) => continue,
                Ok(bits) if bits.as_raw() == 0 => continue,
                Ok(_) => break,
            }
        }

        // 9. Send result to host.
        {
            let mut tx = tx_slot.take().expect("tx available");
            let msg = if VERIFY_FAIL.load(Ordering::Acquire) == 0 { b"OK\n" as &[u8] } else { b"ERR\n" };
            for &byte in msg {
                nb::block!(EhWrite::write(&mut tx, byte)).ok();
            }
            tx_slot = Some(tx);
        }

        // P0 no longer owns the slot — P1 released it.
        CYCLE.fetch_add(1, Ordering::Release);
    }
}

// ---------------------------------------------------------------------------
// P1 — Verifier (now receives ownership, reads, releases)
// ---------------------------------------------------------------------------
extern "C" fn verifier_body(_r0: u32) -> ! {
    let buffer_slot = BufferSlotId::new(0);
    loop {
        loop {
            match plib::sys_event_wait(BUF_READY) {
                Err(_) => continue,
                Ok(bits) if bits.as_raw() == 0 => continue,
                Ok(_) => break,
            }
        }

        let mut buf = [0u8; BUF_LEN];
        match plib::sys_buf_read(buffer_slot, &mut buf) {
            Ok(_) => {
                // Expected pattern: 0x00..0x1F
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

        // P1 owns the slot after transfer — release it back to the pool.
        if plib::sys_buf_release(buffer_slot).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
        }

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
    rprintln!("=== UART DMA Transfer Demo — SAME51 Curiosity Nano ===");

    let pac::Peripherals {
        gclk,
        mut mclk,
        mut osc32kctrl,
        mut oscctrl,
        mut nvmctrl,
        port,
        sercom2,
        dmac,
        mut pm,
        ..
    } = pac::Peripherals::take().unwrap();
    let p = cortex_m::Peripherals::take().unwrap();

    let mut clocks = GenericClockController::with_internal_32kosc(
        gclk,
        &mut mclk,
        &mut osc32kctrl,
        &mut oscctrl,
        &mut nvmctrl,
    );

    // UART: full-duplex on SERCOM2 (PA13 RX, PA12 TX) via nEDBG VCOM.
    let pins = hal::gpio::Pins::new(port);
    let gclk0 = clocks.gclk0();
    let pads = uart::Pads::<Sercom2>::default().rx(pins.pa13).tx(pins.pa12);
    let uart: uart::Uart<VcomConfig, uart::Duplex> = uart::Config::new(
        &mut mclk,
        sercom2,
        pads,
        clocks.sercom2_core(&gclk0).unwrap().freq(),
    )
    .baud(
        115_200.Hz(),
        uart::BaudMode::Fractional(uart::Oversampling::Bits16),
    )
    .enable();

    let (rx, tx) = uart.split();
    rprintln!("[INIT] SERCOM2 115200 8N1 duplex, split into RX + TX.");

    // DMAC channel 0 for RX.
    let mut dmac_ctrl = DmaController::init(dmac, &mut pm);
    let channels = dmac_ctrl.split();
    let chan0 = channels.0.init(PriorityLevel::Lvl0);
    rprintln!("[INIT] DMAC channel 0 ready for RX.");

    unsafe {
        *addr_of_mut!(TX_HANDLE) = Some(tx);
        *addr_of_mut!(RX_HANDLE) = Some(rx);
        *addr_of_mut!(DMA_RX_CHAN) = Some(chan0);
    }

    // Kernel setup.
    let mut sched = ScheduleTable::<{ UartDmaConfig::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(2).expect("sched SW");

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let data_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code_mpu = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    // P0: SRAM + flash + DMAC + SERCOM2 peripheral regions.
    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, producer_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0)
        )
        .expect("mem0")
        .with_code_mpu_region(code_mpu)
        .expect("code0")
        .with_peripheral_regions(&[
            MpuRegion::new(DMAC_BASE, DMAC_SIZE, 0),
            MpuRegion::new(SERCOM2_BASE, SERCOM2_SIZE, 0),
        ])
        .expect("periph0");

    let mem1 = ExternalPartitionMemory::from_aligned_stack(
            s1, verifier_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(1)
        )
        .expect("mem1")
        .with_code_mpu_region(code_mpu)
        .expect("code1");

    let mems = [mem0, mem1];
    let mut k = Kernel::<UartDmaConfig>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);
    rprintln!("[INIT] Kernel created. Buffer pool: 1 slot x 32 bytes.");
    rprintln!("[INIT] P0: DMAC+SERCOM2 grants. Transfer mode (P1 releases).");
    rprintln!("[INIT] Pattern: 0x00..0x{:02X}", BUF_LEN as u8 - 1);
    rprintln!("[INIT] Booting with MPU enabled...\n");

    match boot(p).expect("boot") {}
}

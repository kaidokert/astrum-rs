//! UART Full-Duplex DMA Echo — SAME51 Curiosity Nano
//!
//! DMA receive from host → kernel buffer pool → DMA transmit back to host.
//! Both RX and TX transfers use the DMAC (Ch0, reused sequentially).
//! The kernel buffer pool sits in the middle to prove data transits through
//! kernel-managed memory.
//!
//! Protocol (one cycle):
//!   1. MCU sends "READY\n" via CPU TX
//!   2. Host sends 32-byte pattern (0x00..0x1F)
//!   3. DMAC Ch0 receives 32 bytes into DMA_RX_BUF
//!   4. sys_buf_write copies into kernel buffer pool slot
//!   5. sys_buf_read copies out into DMA_TX_BUF
//!   6. DMAC Ch0 transmits 32 bytes from DMA_TX_BUF back to host
//!   7. Host verifies byte-for-byte match
//!   8. MCU sends "OK\n" via CPU TX
//!
//! Build: cd same51_curiosity && cargo build --example uart_dma_echo \
//!            --features kernel-mpu --release
//! Flash: probe-rs download --chip ATSAME51J20A --probe 03eb:2175 <elf>
//! Host:  python3 same51_curiosity/uart_dma_echo_host.py --port /dev/ttyACM4 --cycles 50
//!
//! ⚠ HOST TIMING: open the serial port BEFORE resetting the target.

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

const DMAC_BASE: u32 = 0x4100_A000;
const DMAC_SIZE: u32 = 0x100;
const SERCOM2_BASE: u32 = 0x4101_2000;
const SERCOM2_SIZE: u32 = 0x40;

const P1_PID: PartitionId = PartitionId::new(1);

static CYCLE: AtomicU32 = AtomicU32::new(0);
static DMA_ERR: AtomicU32 = AtomicU32::new(0);
static API_ERR: AtomicU32 = AtomicU32::new(0);

type VcomPads = uart::PadsFromIds<Sercom2, hal::gpio::PA13, hal::gpio::PA12>;
type VcomConfig = uart::Config<VcomPads>;
type RxHalf = uart::Uart<VcomConfig, uart::RxDuplex>;
type TxHalf = uart::Uart<VcomConfig, uart::TxDuplex>;
type DmaChan = dmac::Channel<dmac::Ch0, dmac::Ready>;

static mut TX_HANDLE: Option<TxHalf> = None;
static mut RX_HANDLE: Option<RxHalf> = None;
static mut DMA_CHAN: Option<DmaChan> = None;

kernel::kernel_config!(EchoCfg[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
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

kernel::bind_interrupts!(EchoCfg, IRQ_COUNT,
    DUMMY_IRQ => (0, 0x0000_0001, handler: dummy_irq),
);

kernel::define_kernel!(EchoCfg, |tick, _k| {
    if tick % 1000 == 0 {
        let cycle = CYCLE.load(Ordering::Acquire);
        let dma_err = DMA_ERR.load(Ordering::Acquire);
        let api_err = API_ERR.load(Ordering::Acquire);
        rprintln!(
            "[{:5}ms] CYCLE={} DMA_ERR={} API_ERR={}",
            tick, cycle, dma_err, api_err
        );
        if cycle >= 20 && api_err == 0 {
            rprintln!("SUCCESS: SAME51 full-duplex DMA echo working! CYCLE={}", cycle);
        }
    }
});

static mut DMA_RX_BUF: [u8; BUF_LEN] = [0u8; BUF_LEN];
static mut DMA_TX_BUF: [u8; BUF_LEN] = [0u8; BUF_LEN];

// ---------------------------------------------------------------------------
// P0 — full-duplex DMA echo via kernel buffer pool
// ---------------------------------------------------------------------------
extern "C" fn echo_body(_r0: u32) -> ! {
    use embedded_hal_02::serial::Write as EhWrite;
    use hal::nb;

    let tx = unsafe { (*addr_of_mut!(TX_HANDLE)).take().expect("tx") };
    let rx = unsafe { (*addr_of_mut!(RX_HANDLE)).take().expect("rx") };
    let chan = unsafe { (*addr_of_mut!(DMA_CHAN)).take().expect("dma chan") };

    let mut tx_slot: Option<TxHalf> = Some(tx);
    let mut rx_slot: Option<RxHalf> = Some(rx);
    let mut chan_slot: Option<DmaChan> = Some(chan);

    let buffer_slot = BufferSlotId::new(0);

    loop {
        // 1. Alloc buffer slot.
        let slot = match plib::sys_buf_alloc(true, 0) {
            Ok(s) => s,
            Err(_) => {
                API_ERR.fetch_add(1, Ordering::Release);
                plib::sys_yield().ok();
                continue;
            }
        };

        // 2. Send READY via CPU TX.
        {
            let mut tx = tx_slot.take().expect("tx available");
            for &byte in b"READY\n" {
                nb::block!(EhWrite::write(&mut tx, byte)).ok();
            }
            while !tx.read_flags().contains(uart::Flags::TXC) {}
            tx.clear_flags(uart::Flags::TXC);
            tx_slot = Some(tx);
        }

        // 3. RX DMA: receive 32 bytes from host into DMA_RX_BUF.
        let dma_buf = unsafe {
            let b = &mut *addr_of_mut!(DMA_RX_BUF);
            b.fill(0xFF);
            &mut *(b as *mut [u8; BUF_LEN])
        };
        let rx_uart = rx_slot.take().expect("rx available");
        let ch = chan_slot.take().expect("chan available");
        let xfer = rx_uart.receive_with_dma(dma_buf, ch);
        let (returned_chan, returned_rx, returned_buf) = xfer.wait();
        chan_slot = Some(returned_chan);
        rx_slot = Some(returned_rx);

        // 4. Copy into kernel buffer pool.
        if plib::sys_buf_write(slot, returned_buf).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
            if let Err(e) = plib::sys_buf_release(slot) {
                rprintln!("[P0] sys_buf_release failed: {:?}", e);
            }
            continue;
        }

        // 5. Read back from kernel buffer pool into TX buffer.
        //    Lend to self (writable) to get read access, then read.
        let mut tx_buf = unsafe {
            let b = &mut *addr_of_mut!(DMA_TX_BUF);
            b.fill(0x00);
            &mut *(b as *mut [u8; BUF_LEN])
        };
        if plib::sys_buf_read(slot, tx_buf).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
            if let Err(e) = plib::sys_buf_release(slot) {
                rprintln!("[P0] sys_buf_release failed: {:?}", e);
            }
            continue;
        }

        // 6. TX DMA: send 32 bytes back to host from DMA_TX_BUF.
        let tx_uart = tx_slot.take().expect("tx available");
        let ch = chan_slot.take().expect("chan available");
        let xfer = tx_uart.send_with_dma(tx_buf, ch);
        let (returned_chan, _returned_tx_buf, mut returned_tx) = xfer.wait();
        chan_slot = Some(returned_chan);

        // Wait for last byte to shift out before reusing TX.
        while !returned_tx.read_flags().contains(uart::Flags::TXC) {}
        returned_tx.clear_flags(uart::Flags::TXC);
        tx_slot = Some(returned_tx);

        // 7. Release buffer slot.
        if plib::sys_buf_release(slot).is_err() {
            API_ERR.fetch_add(1, Ordering::Release);
        }

        // 8. Send OK via CPU TX.
        {
            let mut tx = tx_slot.take().expect("tx available");
            for &byte in b"OK\n" {
                nb::block!(EhWrite::write(&mut tx, byte)).ok();
            }
            tx_slot = Some(tx);
        }

        CYCLE.fetch_add(1, Ordering::Release);
    }
}

// P1: dummy — kernel needs 2 partitions for the config.
extern "C" fn dummy_body(_r0: u32) -> ! {
    loop { plib::sys_yield().ok(); }
}

kernel::partition_trampoline!(echo_main => echo_body);
kernel::partition_trampoline!(dummy_main => dummy_body);

#[entry]
fn main() -> ! {
    rprintln!("=== UART Full-Duplex DMA Echo — SAME51 Curiosity Nano ===");

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
    rprintln!("[INIT] SERCOM2 115200 8N1 full-duplex.");

    let mut dmac_ctrl = DmaController::init(dmac, &mut pm);
    let channels = dmac_ctrl.split();
    let chan0 = channels.0.init(PriorityLevel::Lvl0);
    rprintln!("[INIT] DMAC channel 0 (RX + TX, sequential reuse).");

    unsafe {
        *addr_of_mut!(TX_HANDLE) = Some(tx);
        *addr_of_mut!(RX_HANDLE) = Some(rx);
        *addr_of_mut!(DMA_CHAN) = Some(chan0);
    }

    let mut sched = ScheduleTable::<{ EchoCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched P0");
    sched.add(ScheduleEntry::new(1, 2)).expect("sched P1");
    sched.add_system_window(2).expect("sched SW");

    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0, ref mut s1] = *stacks;

    let data_mpu = MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0);
    let code_mpu = MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0);

    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, echo_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(0)
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
            s1, dummy_main as kernel::PartitionEntry, data_mpu, kernel::PartitionId::new(1)
        )
        .expect("mem1")
        .with_code_mpu_region(code_mpu)
        .expect("code1");

    let mems = [mem0, mem1];
    let mut k = Kernel::<EchoCfg>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);
    rprintln!("[INIT] Kernel ready. P0=echo P1=dummy. Buffer pool: 1 slot x 32 bytes.");
    rprintln!("[INIT] Booting with MPU enabled...\n");

    match boot(p).expect("boot") {}
}

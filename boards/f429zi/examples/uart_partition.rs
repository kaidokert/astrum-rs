//! UART IRQ Partition Demo (HAL variant) — STM32F429ZI
//!
//! Same architecture as uart_irq_partition.rs (Approach D + Model B IRQ),
//! but hardware initialisation uses stm32f4xx-hal APIs instead of raw PAC
//! register accesses.  Only one PAC call remains: CR3.HDSEL.
//!
//! What HAL handles in main():
//! - `dp.RCC.freeze(Config::hsi())` — RCC clock object; APB1 = 16 MHz HSI.
//! - `dp.GPIOD.split(&mut rcc)` — enables GPIOD AHB1 clock (via Enable::enable()).
//! - `pd8.into_alternate::<7>()` — sets MODER8=0b10 + AFRH8=AF7 in one typed call.
//! - `dp.USART3.tx(pd8, 115_200.bps(), &mut rcc)` — enables USART3 APB1 clock,
//!   computes BRR from actual pclk_freq, writes CR1 with explicit 8N1 config
//!   (UE + TE + RE + over8=0 + 8-bit word + no parity), resets CR2 + CR3.
//!
//! What HAL cannot handle → one PAC call remains:
//! - `CR3.HDSEL` — HAL's `_new()` explicitly resets CR3 to 0 (line: `uart.cr3().reset()`).
//!   Half-duplex mode is not in the HAL's serial Config struct.  We modify
//!   CR3 via `pac::Peripherals::steal()` AFTER Serial::tx() completes.
//!   SAFETY: HDSEL is never touched by HAL after init.
//!
//! Architecture (Model B — partition clears hardware + ACK):
//! - USART3 in half-duplex loopback (HDSEL=1): TX is internally fed back
//!   to the RX input, so every transmitted byte is also received.
//!   No external wires, no serial terminal required.
//! - `bind_interrupts!(UartCfg, 91, 39 => (0, UART_RX_EVENT))` wires
//!   the USART3 RXNE IRQ (IRQ 39) through the kernel dispatch table.
//!   The patched `__irq_dispatch` MASKS the IRQ before signalling P0,
//!   preventing re-entry / livelock on the level-triggered RXNE line.
//! - P0 holds USART3 (0x4000_4800 / 1 KB) in peripheral_regions — under
//!   MPU enforcement this is the ONLY peripheral P0 can touch.
//! - P0 loop:
//!     1. enable RXNEIE (first iteration only)
//!     2. write byte to DR → TX shift reg → loopback → RX shift reg → RXNE
//!     3. SYS_EVT_WAIT(0, UART_RX_EVENT) — block until dispatch signals
//!     4. read DR → consumes byte + clears RXNE + de-asserts IRQ line
//!     5. SYS_IRQ_ACK(39) → kernel NVIC::unmask(IrqNr(39)) → IRQ re-armed
//!     6. TX_COUNT++, RX_COUNT++, goto 2
//!
//! Hardware: STM32F429ZI NUCLEO-144.
//!   USART3 APB1, 16 MHz HSI (no PLL). Half-duplex on PD8 (AF7).
//!   USART3 base: 0x4000_4800 — MPU-grantable at 1 KB granularity.
//!
//! Build:  cd f429zi && cargo build --example uart_partition \
//!             --features kernel-irq-mpu-hal --no-default-features
//! Flash:  (via GDB + OpenOCD — see CLAUDE.md)
//! Verify: RTT shows TX_COUNT == RX_COUNT advancing; MemManage=0.

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
    svc::Kernel,
    {DebugEnabled, MsgMinimal, Partitions2, PortsTiny, SyncMinimal},
};
use rtt_target::{rprintln, rtt_init_print};
use stm32f4xx_hal::{pac, prelude::*, rcc::Config, serial};
use f429zi::{FLASH_BASE, FLASH_SIZE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = 512;

// USART3 IRQ number in STM32F429ZI (confirmed from PAC: USART3 = 39).
const USART3_IRQ: u8 = 39;

// Kernel event bit set by the masked __irq_dispatch when USART3 RXNE fires.
const UART_RX_EVENT: u32 = 0x0000_0001;

// ---------------------------------------------------------------------------
// Diagnostic atomics — read by SysTick hook from SRAM (always accessible).
// ---------------------------------------------------------------------------
static TX_COUNT: AtomicU32 = AtomicU32::new(0);
static RX_COUNT: AtomicU32 = AtomicU32::new(0);

// ---------------------------------------------------------------------------
// Bind USART3 RXNE IRQ (IRQ 39) to partition 0, event bit 0x01.
//
// IRQ count = 91 (full STM32F429ZI table; from f429zi's device.x via build.rs).
// Our patched __irq_dispatch: masks IRQ 39 BEFORE signal_partition_from_isr,
// preventing level-triggered livelock when RXNE is asserted.
// enable_bound_irqs(): called after store_kernel() — see main().
// ---------------------------------------------------------------------------
kernel::bind_interrupts!(UartHalCfg, 91,
    39 => (0, UART_RX_EVENT),
);

kernel::kernel_config!(UartHalCfg[AlignedStack2K]<Partitions2, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::CORE_CLOCK_HZ;
    schedule_capacity = 8;
    mpu_enforce = true;
    SW = 2; MW = 2;
    QD = 2; QM = 4; QW = 2;
    SM = 2; BM = 2; BW = 2;
});

kernel::define_kernel!(UartHalCfg, |tick, _k| {
    if tick % 500 == 0 {
        let tx = TX_COUNT.load(Ordering::Acquire);
        let rx = RX_COUNT.load(Ordering::Acquire);
        rprintln!("[{:5}ms] TX={} RX={}", tick, tx, rx);
        if tx > 20 && tx == rx {
            rprintln!("SUCCESS: UART IRQ partition working! TX=RX={}", tx);
        }
    }
    // Dump active MPU regions on first tick to confirm USART3 wiring.
    if tick == 1 {
        let mpu = unsafe { &*cortex_m::peripheral::MPU::PTR };
        let ctrl = mpu.ctrl.read();
        rprintln!("--- MPU CTRL=0x{:x} ---", ctrl);
        for r in 0u32..8 {
            unsafe { mpu.rnr.write(r) };
            let rbar = mpu.rbar.read();
            let rasr = mpu.rasr.read();
            if rasr & 1 != 0 {
                rprintln!("  R{}: RBAR=0x{:08x} RASR=0x{:08x}", r, rbar, rasr);
            }
        }
        rprintln!("---");
    }
});

// ---------------------------------------------------------------------------
// Partition P0: UART driver (Approach D + IRQ Model B).
//
// Runs under MPU enforcement — only USART3 MMIO (0x4000_4800/1KB) is in
// peripheral_regions.  The NVIC (PPB 0xE000_E000) is NOT grantable, so
// IRQ unmask MUST go through the SYS_IRQ_ACK kernel syscall.
// ---------------------------------------------------------------------------
extern "C" fn uart_driver_body(uart_irq: u32) -> ! {
    // SAFETY: steal() returns raw-pointer struct — no hardware access yet.
    // All USART3 register dereferences are within the 1 KB MPU grant
    // (0x4000_4800 .. 0x4000_4BFF).
    let dp = unsafe { pac::Peripherals::steal() };

    // Enable RXNEIE: assert IRQ 39 when RDR contains a received byte.
    // Done here in the partition (not main) so the IRQ fires only after the
    // kernel event tables are initialised and bind_interrupts! is live.
    dp.USART3.cr1().modify(|_, w| w.rxneie().set_bit());

    loop {
        // Wait for TDR to be empty before writing — avoids data loss.
        // Under MPU enforcement, yielding is cheaper than spinning.
        while dp.USART3.sr().read().txe().bit_is_clear() {
            plib::sys_yield().ok();
        }

        // Write one byte to DR (→ TDR → TX shift reg).
        // In HDSEL loopback: TX output is internally fed to RX input.
        // The byte travels:  TDR → TX shift reg → TX pin → loopback
        //                   → RX shift reg → RDR → RXNE asserts → IRQ 39 fires.
        dp.USART3.dr().write(|w| unsafe { w.dr().bits(0xAA) });
        TX_COUNT.fetch_add(1, Ordering::Release);

        // Block until IrqClearModel::PartitionAcks signals UART_RX_EVENT.
        if plib::sys_event_wait(UART_RX_EVENT.into()).is_err() {
            plib::sys_yield().ok();
            continue;
        }

        // Read DR — two effects:
        //   1. Retrieves the looped-back byte (value verification optional).
        //   2. CLEARS the RXNE flag → de-asserts the USART3 IRQ line.
        // CRITICAL: RXNE must be clear BEFORE sys_irq_ack.  Unmasking while
        // RXNE is still asserted would immediately re-fire IRQ 39 — livelock.
        let _byte = dp.USART3.dr().read().dr().bits();
        RX_COUNT.fetch_add(1, Ordering::Release);

        // Re-arm the UART IRQ: kernel NVIC::unmask — partition cannot access NVIC directly.
        plib::sys_irq_ack(uart_irq as u8).ok();
    }
}

kernel::partition_trampoline!(uart_driver => uart_driver_body);

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    rprintln!("\n=== UART IRQ Partition Demo (HAL variant) — Approach D + Model B IRQ ===");
    rprintln!("USART3 half-duplex loopback: TX byte → internal RX → RXNE → IRQ 39");
    rprintln!("IrqClearModel::PartitionAcks (mask on dispatch) + plib::sys_irq_ack (unmask)");

    // -------------------------------------------------------------------------
    // Hardware initialisation — privileged Thread mode before boot() arms MPU.
    // -------------------------------------------------------------------------
    let dp = pac::Peripherals::take().unwrap();

    // HAL clock config: HSI 16 MHz, no PLL.  Gives us a typed Rcc object that
    // carries clock frequencies for BRR calculation.
    let mut rcc = dp.RCC.freeze(Config::hsi());

    // HAL GPIO: enables GPIOD AHB1 clock and returns typed pin handles.
    let gpiod = dp.GPIOD.split(&mut rcc);

    // PD8 → AF7 (USART3_TX): HAL sets MODER8 = Alternate and AFRH8 = AF7.
    let pd8_tx = gpiod.pd8.into_alternate::<7>();

    // HAL USART3 init (TX-only object — RE is still set in CR1 by the HAL):
    //   - Enables USART3 APB1 clock via Enable::enable().
    //   - Computes BRR from actual pclk1 freq (derived from RCC clock tree).
    //   - Resets CR2 and CR3 to 0 (clears any stale state).
    //   - Writes CR1 with full 8N1 config: UE + TE + RE + over8=0 (16×) +
    //     8-bit word + no parity.  More explicit than a hand-rolled CR1 write.
    //   - RXNEIE is NOT set here — P0 enables it after boot().
    let _tx: serial::Tx3 = dp.USART3
        .tx(pd8_tx, 115_200.bps(), &mut rcc)
        .expect("USART3 init");

    // HDSEL: HAL has no half-duplex config option and resets CR3 to 0 during
    // init.  Add HDSEL=1 via steal() after Serial::tx() completes.
    // SAFETY: _tx owns USART3 for typed access, but CR3.HDSEL is only written
    // here (once, before boot) and never read or written by HAL afterwards.
    unsafe {
        pac::Peripherals::steal()
            .USART3
            .cr3()
            .modify(|_, w| w.hdsel().half_duplex());
    }
    rprintln!("[INIT] HAL: PD8 AF7 + USART3 115200/8N1 + CR3.HDSEL=1 (loopback)");

    // -------------------------------------------------------------------------
    // Kernel setup.
    // -------------------------------------------------------------------------
    let usart3_base = pac::USART3::PTR as usize as u32;
    rprintln!("P0 peripheral_regions: USART3=0x{:08x}/1KB", usart3_base);

    let mut p = cortex_m::Peripherals::take().unwrap();

    // Schedule: P0 (4 ticks) + SystemWindow (2 ticks, required by dynamic-mpu).
    let mut sched = ScheduleTable::<{ UartHalCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched P0");
    sched.add_system_window(2).expect("sched SW");

    // Allocate partition stack externally for ExternalPartitionMemory.
    static mut STACKS: [AlignedStack2K; NUM_PARTITIONS] = [AlignedStack2K::ZERO; NUM_PARTITIONS];
    let stacks = unsafe { &mut *core::ptr::addr_of_mut!(STACKS) };
    let [ref mut s0] = *stacks;

    // P0: SRAM data region + USART3 peripheral grant (1 KB).
    let mem0 = ExternalPartitionMemory::from_aligned_stack(
            s0, uart_driver as kernel::PartitionEntry, MpuRegion::new(SRAM_BASE, SRAM_SIZE, 0), kernel::PartitionId::new(0),
        )
        .expect("mem0")
            .with_code_mpu_region(MpuRegion::new(FLASH_BASE, FLASH_SIZE, 0))
            .expect("code0")
        .with_peripheral_regions(&[MpuRegion::new(usart3_base, 1024, 0)])
        .expect("periph0")
        .with_r0_hint(USART3_IRQ as u32);

    let mems = [mem0];
    let mut k = Kernel::<UartHalCfg>::new(sched, &mems).expect("kernel");
    store_kernel(&mut k);

    // Unmask IRQ 39 (USART3) AFTER store_kernel — ISR must not fire on
    // uninitialised kernel state.  RXNEIE is still 0 (partition enables it).
    enable_bound_irqs(&mut p.NVIC, UartHalCfg::IRQ_DEFAULT_PRIORITY).unwrap();
    rprintln!("[INIT] IRQ 39 (USART3) enabled at priority 0x{:x} — kernel ready",
              UartHalCfg::IRQ_DEFAULT_PRIORITY);

    rprintln!("[INIT] Booting with MPU enforcement (deny-all + USART3 grant)\n");
    match boot(p).expect("boot") {}
}

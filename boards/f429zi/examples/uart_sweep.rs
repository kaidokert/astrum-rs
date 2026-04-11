//! UART IRQ Partition Echo — Full-Duplex End-to-End Test — STM32F429ZI
//!
//! End-to-end UART verification using a USB-to-UART adapter.
//! No loopback: device TX/RX wired to adapter RX/TX respectively.
//!
//! Wiring (NUCLEO-144):
//!   PG14 (USART6_TX, CN10 D1) → adapter RX
//!   PG9  (USART6_RX, CN10 D0) → adapter TX
//!   GND                        → adapter GND
//!
//! Architecture:
//!   - Full-duplex USART6 with HAL Serial::new() — no HDSEL, no steal().
//!   - Partition sends an incrementing byte counter, blocks on IRQ-driven RX,
//!     verifies the echoed byte matches what was sent.
//!   - After `ROUNDS_PER_RATE` successful echo round-trips the partition sends
//!     byte 0xFF ("rate-switch signal") and waits briefly, then changes BRR.
//!     The host script sees 0xFF and switches its own baud rate to match.
//!   - Baud rate table (84 MHz APB2 from HSE+PLL):
//!       9600, 115200, 230400, 460800, 921600, 1000000, 2000000
//!     The demo stops at the first rate where too many errors occur.
//!
//! Why USART6 instead of USART3?
//!   USART3 (PD8/PD9) is routed to the on-board ST-Link VCP (SB5/SB6 closed
//!   by default) — the external adapter wires connect to the Arduino CN10
//!   connector (D1/D0) which maps to PG14/PG9 = USART6.
//!
//! Why HSE+PLL?  16 MHz HSI gives poor BRR accuracy above ~460 800 bps.
//!   168 MHz sysclk → APB2 = 84 MHz → exact dividers for 1 Mbps and 2 Mbps.
//!
//! GPIO speed: OSPEEDR = VeryHigh on PG14/PG9 (required above ~1 Mbps).
//!   HAL's into_alternate() leaves OSPEEDR at reset (Low = 2 MHz);
//!   we call .speed(Speed::VeryHigh) to bump it.
//!
//! Host:  python3 f429zi/host_serial_echo.py --port /dev/ttyUSB0
//!
//! Build: cd f429zi && cargo build --example uart_sweep \
//!            --features kernel-irq-mpu-hal --no-default-features
//! Verify: RTT shows each baud rate PASS/FAIL; host prints received bytes.

#![no_std]
#![no_main]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m_rt::{entry, exception};
use plib::{define_partition_debug, dprint};
use kernel::{PartitionSpec, 
    partition_core::AlignedStack2K,
    scheduler::{ScheduleEntry, ScheduleTable},
    {DebugEnabled, MsgMinimal, Partitions1, PortsTiny, SyncMinimal},
};
use rtt_target::{rprintln, rtt_init};
use stm32f4xx_hal::{pac, prelude::*, rcc::Config, gpio::{Pull, Speed}};
use f429zi::{FLASH_BASE, SRAM_BASE, SRAM_SIZE};

const NUM_PARTITIONS: usize = 1;
const STACK_WORDS: usize = 1024;

const USART6_IRQ: u8 = 71;
const UART_RX_EVENT: u32 = 0x0000_0001;

// How many successful echo round-trips to run at each baud rate before
// advancing to the next one.  Enough to catch intermittent errors.
const ROUNDS_PER_RATE: u32 = 200;

// Special byte sent to the host before a baud-rate switch.
// The host script treats 0xFF as "switch to next rate in table".
const RATE_SWITCH_SIGNAL: u8 = 0xFF;

// ---------------------------------------------------------------------------
// Baud rate table.
// BRR = USARTDIV packed as (mantissa<<4)|fraction; USARTDIV = APB2/(16×baud).
// APB2 = 84 MHz (168 MHz sysclk, /2 prescaler).  Values are 2× the APB1 table.
//
// Rate      BRR    USARTDIV    Actual baud    Error
// 9600      8750   546.875     9600.0         0.00 %   exact
// 115200     729    45.5625    115226.3        0.02 %
// 230400     365    22.8125    230136.9       -0.11 %
// 460800     182    11.375     461538.5        0.16 %
// 921600      91     5.6875    923076.9        0.16 %
// 1000000     84     5.25      1000000.0       0.00 %   exact
// 2000000     42     2.625     2000000.0       0.00 %   exact
// ---------------------------------------------------------------------------
static BAUD_TABLE: &[(u32, u16, &str)] = &[
    (     9_600, 8750, "9600"),
    (   115_200,  729, "115200"),
    (   230_400,  365, "230400"),
    (   460_800,  182, "460800"),
    (   921_600,   91, "921600"),
    ( 1_000_000,   84, "1000000"),
    ( 2_000_000,   42, "2000000"),
];

// ---------------------------------------------------------------------------
// Diagnostics — read by SysTick harness hook.
// ---------------------------------------------------------------------------
static TX_COUNT: AtomicU32 = AtomicU32::new(0);
static RX_COUNT: AtomicU32 = AtomicU32::new(0);
static ERR_COUNT: AtomicU32 = AtomicU32::new(0);
static RATE_IDX: AtomicU32 = AtomicU32::new(0);

// Partition-side debug ring buffer.  Drained by the kernel via RTT during
// system windows — completely separate from the main 4KB RTT up-channel,
// so high-volume partition logging never overflows the RTT buffer.
define_partition_debug!(UART_P0_DEBUG, 1024);

kernel::bind_interrupts!(UartEchoCfg, 91,
    71 => (0, UART_RX_EVENT),
);

kernel::kernel_config!(UartEchoCfg[AlignedStack2K]<Partitions1, SyncMinimal, MsgMinimal, PortsTiny, DebugEnabled> {
    core_clock_hz = f429zi::USB_SYSCLK_HZ;  // 168 MHz HSE+PLL — corrects SysTick period
    schedule_capacity = 4;
    mpu_enforce = true;
    SW = 1; MW = 1;
    QD = 1; QM = 1; QW = 1;
    SM = 1; BM = 1; BW = 1;
});

kernel::define_kernel!(UartEchoCfg, |tick, _k| {
    if tick % 1000 == 0 {
        let tx  = TX_COUNT.load(Ordering::Acquire);
        let rx  = RX_COUNT.load(Ordering::Acquire);
        let err = ERR_COUNT.load(Ordering::Acquire);
        let ri  = RATE_IDX.load(Ordering::Acquire) as usize;
        let rate = if ri < BAUD_TABLE.len() { BAUD_TABLE[ri].2 } else { "done" };
        rprintln!("[{:6}ms] rate={} TX={} RX={} ERR={}", tick, rate, tx, rx, err);
    }
    if tick == 1 {
        let mpu = unsafe { &*cortex_m::peripheral::MPU::PTR };
        rprintln!("--- MPU CTRL=0x{:x} ---", mpu.ctrl.read());
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
// Partition P0: UART echo driver.
// ---------------------------------------------------------------------------
extern "C" fn uart_driver_body(uart_irq: u32) -> ! {
    let dp = unsafe { pac::Peripherals::steal() };

    // -----------------------------------------------------------------------
    // Helper: polling drain — yield and discard bytes until N consecutive
    // quiet yields.  Returns the number of bytes discarded.
    // RXNEIE must be CLEAR (no kernel events generated).
    // On STM32, reading SR then DR clears RXNE and ORE; the shift-register
    // byte then transfers to DR and sets RXNE again.  The loop handles this
    // correctly by continuing until both RXNE and ORE are 0.
    // -----------------------------------------------------------------------
    let polling_drain = |quiet_target: u32| -> u32 {
        let mut quiet = 0u32;
        let mut discarded = 0u32;
        loop {
            // Drain all bytes currently pending (handles ORE-shifted bytes).
            loop {
                let sr = dp.USART6.sr().read();
                if sr.rxne().bit_is_set() || sr.ore().bit_is_set() {
                    let b = dp.USART6.dr().read().dr().bits();
                    let _ = dprint!(&UART_P0_DEBUG, @size 64, "[UART] drain: 0x{:02x}", b);
                    discarded += 1;
                } else {
                    break;
                }
            }
            plib::sys_yield().ok();
            let sr = dp.USART6.sr().read();
            if sr.rxne().bit_is_set() || sr.ore().bit_is_set() {
                quiet = 0;
            } else {
                quiet += 1;
                if quiet >= quiet_target { break; }
            }
        }
        discarded
    };

    // Phase 1: Boot drain — wait until the line is silent after HAL init.
    // HAL USART6 init drives PG14 through GPIO-to-AF transitions that can
    // cause brief line glitches the host may echo back.  150 quiet yields
    // ≈ 900 ms of confirmed silence ensures the line is clean before sweep.
    let _ = dprint!(&UART_P0_DEBUG, @size 64, "[UART] Boot drain (150-quiet-yield)...");
    let n = polling_drain(150);
    let _ = dprint!(&UART_P0_DEBUG, @size 64, "[UART] boot drain done ({} bytes discarded)", n);

    // Phase 2: Handshake — enable RXNEIE and send one 0xA5 probe byte.
    // Block on SYS_EVT_WAIT until the host echoes it back.  This confirms
    // the end-to-end path (MCU→adapter→host→adapter→MCU) is live before
    // the baud sweep starts.
    dp.USART6.cr1().modify(|_, w| w.rxneie().set_bit());

    while dp.USART6.sr().read().txe().bit_is_clear() {
        plib::sys_yield().ok();
    }
    dp.USART6.dr().write(|w| unsafe { w.dr().bits(0xA5) });
    let _ = dprint!(&UART_P0_DEBUG, @size 64, "[UART] 0xA5 probe sent — waiting for host echo...");

    // Drain until we see the real 0xA5 echo.  Any non-0xA5 byte is a spurious
    // pre-probe echo (USART6 init glitch, stale OS buffer) — discard and keep
    // waiting.  The kernel masks IRQ71 on first fire; we re-arm via SYS_IRQ_ACK
    // so subsequent echoes can also fire.
    loop {
        if plib::sys_event_wait(UART_RX_EVENT.into()).is_err() {
            let _ = dprint!(&UART_P0_DEBUG, @size 64, "[UART] FATAL: sys_event_wait failed on handshake");
            loop { plib::sys_yield().ok(); }
        }
        let hs_echo = dp.USART6.dr().read().dr().bits() as u8;
        plib::sys_irq_ack(uart_irq as u8).ok();
        if hs_echo == 0xA5 {
            let _ = dprint!(&UART_P0_DEBUG, @size 64, "[UART] Handshake OK — echo=0xa5");
            break;
        }
        let _ = dprint!(&UART_P0_DEBUG, @size 64, "[UART] pre-probe echo=0x{:02x} discarded", hs_echo);
    }

    // Phase 3: Post-handshake silence drain.
    // Disable RXNEIE, drain any late 0xA5 echoes that may arrive in the
    // ~100 ms after the handshake (USB host latency jitter).
    dp.USART6.cr1().modify(|_, w| w.rxneie().clear_bit());
    let n2 = polling_drain(150);
    let _ = dprint!(&UART_P0_DEBUG, @size 64, "[UART] post-hs drain done ({} bytes discarded)", n2);
    // Clear any stale event_flags NOW — RXNEIE is disabled so __irq_dispatch cannot
    // fire and re-set them between this clear and the re-enable below.
    // We do it here (not inside "if hs_echo == 0xA5") because a second 0xA5 echo
    // can arrive AFTER the handshake break but BEFORE rxneie().clear_bit() above,
    // causing __irq_dispatch to set event_flags=UART_RX_EVENT again.  The drain
    // discards that byte but leaves event_flags set; clearing here is safe/race-free.
    plib::sys_event_clear(UART_RX_EVENT).ok();

    // Re-enable RXNEIE.  Event flag is 0.  Sweep's first SYS_EVT_WAIT blocks.
    dp.USART6.cr1().modify(|_, w| w.rxneie().set_bit());
    let _ = dprint!(&UART_P0_DEBUG, @size 64, "[UART] Host ready — starting sweep");

    // Start at 0x01 to avoid ambiguity with stale 0x00 glitch bytes.
    let mut tx_byte: u8 = 0x01;

    'rate_loop: for (rate_idx, &(_baud, brr, rate_str)) in BAUD_TABLE.iter().enumerate() {
        RATE_IDX.store(rate_idx as u32, Ordering::Release);
        let _ = dprint!(&UART_P0_DEBUG, @size 64, "[UART] Starting rate {} bps (BRR={})", rate_str, brr);

        let mut ok: u32 = 0;
        let mut fail: u32 = 0;
        let mut verbose: u32 = 0; // print first 8 echo values for diagnostics

        while ok < ROUNDS_PER_RATE {
            // Wait for TX shift register empty.
            while dp.USART6.sr().read().txe().bit_is_clear() {
                plib::sys_yield().ok();
            }

            let sent = tx_byte;
            dp.USART6.dr().write(|w| unsafe { w.dr().bits(sent as u16) });
            TX_COUNT.fetch_add(1, Ordering::Release);
            tx_byte = tx_byte.wrapping_add(1);
            // Skip reserved bytes in the payload counter.
            // 0xFF: rate-switch signal (host interprets it as baud switch).
            // 0xA5: handshake byte — any 0xA5 echo is a stale glitch echo,
            //       not a real sweep echo (see 'echo loop below).
            if tx_byte == RATE_SWITCH_SIGNAL { tx_byte = 0; }
            if tx_byte == 0xA5 { tx_byte = 0xA6; }

            // Wait for the echo of `sent`, discarding any stale 0xA5 bytes.
            // 0xA5 is the handshake probe byte; any 0xA5 arriving during the
            // sweep is a late echo of the probe (USB round-trip latency) and
            // can be safely discarded.  Since 0xA5 is excluded from the sweep
            // payload counter, any 0xA5 echo is definitively stale.
            'echo: loop {
                if plib::sys_event_wait(UART_RX_EVENT.into()).is_err() {
                    let _ = dprint!(&UART_P0_DEBUG, @size 64, "[UART] sys_event_wait err ok={} fail={}", ok, fail);
                    plib::sys_yield().ok();
                    fail += 1;
                    ERR_COUNT.fetch_add(1, Ordering::Release);
                    if fail > ROUNDS_PER_RATE / 4 {
                        let _ = dprint!(&UART_P0_DEBUG, @size 64, "[UART] rate {} FAIL: syscall errors (ok={} fail={})", rate_str, ok, fail);
                        break 'rate_loop;
                    }
                    break 'echo;
                }

                // Check SR first.  RXNE (bit 5) must be set to have a valid byte.
                // If RXNE=0, the wake is spurious: stale event_flags (from a previous
                // blocking wakeup that event_wait does not auto-clear) or a stale NVIC
                // ISPR pending bit caused the partition to wake with nothing in DR.
                // Log rc (0=from-blocking, non-0=from-immediate-path) and NVIC state to
                // diagnose the pattern, then clear event_flags and re-arm the IRQ.
                let sr = dp.USART6.sr().read();
                let sr_bits = sr.bits();
                if !sr.rxne().bit_is_set() && !sr.ore().bit_is_set() {
                    let _ = dprint!(&UART_P0_DEBUG, @size 80,
                        "[UART] spurious: SR={:x} ok={} fail={}",
                        sr_bits, ok, fail);
                    // Clear stale event_flags BEFORE re-arming: if this wake came from the
                    // blocking path (rc=0), event_flags still has UART_RX_EVENT; the next
                    // SYS_EVT_WAIT would return immediately again (immediate-path) with
                    // another spurious wake.  Clearing here avoids that second spurious.
                    plib::sys_event_clear(UART_RX_EVENT).ok();
                    plib::sys_irq_ack(uart_irq as u8).ok();
                    continue 'echo;
                }

                // Real byte: read DR (clears RXNE and any latched error flags).
                let echo = dp.USART6.dr().read().dr().bits() as u8;
                RX_COUNT.fetch_add(1, Ordering::Release);

                // Clear event_flags BEFORE re-arming the IRQ.
                // event_wait(blocking path) sets event_flags on wakeup but never clears
                // them; SYS_EVT_CLEAR here resets them to 0 while IRQ71 is still masked
                // (safe window: __irq_dispatch masked it on entry; no new IRQ can fire yet).
                // If we did ACK first, a new byte arriving between ACK and CLEAR would:
                //   fire IRQ71 → event_flags |= 1 → then CLEAR wipes it → IRQ71 masked
                //   → next SYS_EVT_WAIT blocks forever (IRQ71 never re-armed).
                plib::sys_event_clear(UART_RX_EVENT).ok();

                // Re-arm IRQ: unmask IRQ71 so the next byte fires __irq_dispatch again.
                // event_flags is now 0, so if a byte arrives here the IRQ fires and sets
                // event_flags=1; the next SYS_EVT_WAIT takes the fast immediate path.
                plib::sys_irq_ack(uart_irq as u8).ok();

                if echo == 0xA5 {
                    // Stale handshake probe echo arriving late — discard.
                    let _ = dprint!(&UART_P0_DEBUG, @size 64, "[UART] stale 0xa5 SR=0x{:x}", sr_bits);
                    continue 'echo;
                }

                // Verbose trace for first 8 non-stale echoes.
                if verbose < 8 {
                    let _ = dprint!(&UART_P0_DEBUG, @size 64, "[UART] echo[{}]: sent=0x{:02x} got=0x{:02x} SR=0x{:x}",
                        verbose, sent, echo, sr_bits);
                    verbose += 1;
                }

                if echo == sent {
                    ok += 1;
                } else {
                    fail += 1;
                    ERR_COUNT.fetch_add(1, Ordering::Release);
                    // Print first 10 mismatches with SR diagnostic.
                    if fail <= 10 {
                        let _ = dprint!(&UART_P0_DEBUG, @size 64, "[UART] MISMATCH #{}: sent=0x{:02x} echo=0x{:02x} SR=0x{:x}",
                            fail, sent, echo, sr_bits);
                    }
                    if fail > ROUNDS_PER_RATE / 4 {
                        let _ = dprint!(&UART_P0_DEBUG, @size 64, "[UART] rate {} FAIL: too many errors (ok={} fail={})", rate_str, ok, fail);
                        break 'rate_loop;
                    }
                }
                break 'echo;
            }
        }

        if ok >= ROUNDS_PER_RATE {
            let _ = dprint!(&UART_P0_DEBUG, @size 64, "[UART] rate {} PASS (ok={} fail={})", rate_str, ok, fail);
        }

        // Don't signal rate switch after the last entry.
        let next_idx = rate_idx + 1;
        if next_idx >= BAUD_TABLE.len() { break; }
        let next_brr = BAUD_TABLE[next_idx].1;

        // --- Baud rate switch ---
        // 1. Send 0xFF to signal host to switch.
        while dp.USART6.sr().read().txe().bit_is_clear() {
            plib::sys_yield().ok();
        }
        dp.USART6.dr().write(|w| unsafe { w.dr().bits(RATE_SWITCH_SIGNAL as u16) });

        // Disable RXNEIE NOW so no bytes arriving during the baud-switch delay
        // can fire IRQ 71 and leave a stale UART_RX_EVENT in P0's event register.
        // Without this, adapter TX-line glitches during tcsetattr(115200) on the
        // host set the event bit; P0's next SYS_EVT_WAIT returns immediately with
        // RXNE=0 → endless spurious-wake loop at every new rate.
        dp.USART6.cr1().modify(|_, w| w.rxneie().clear_bit());

        // Wait for TC (transmit complete) so 0xFF is fully shifted out before
        // we disable USART and change BRR.
        while dp.USART6.sr().read().tc().bit_is_clear() {
            plib::sys_yield().ok();
        }

        // Give host ≥200 ms to complete its baud-rate switch.
        // The host script does time.sleep(0.08) (80 ms) + pyserial SET_LINE_CODING
        // USB control transfer to the adapter (which can take 50-100 ms on some
        // USB-CDC adapters, especially CH340/CP210x under Linux).
        //
        // NOTE: SYS_YIELD is a no-op with a single partition (Bug 06 / no-op PendSV):
        // next_partition never changes for N=1, so PendSV restores the same partition
        // immediately — SYS_YIELD returns in ~10 CPU cycles, not ~1 ms.  Use a
        // hardware cycle-count delay instead (independent of the scheduler).
        // At 168 MHz: 250 ms = 42_000_000 cycles.
        cortex_m::asm::delay(42_000_000);

        // 2. Disable USART (required before changing BRR per RM0090 §30.3.2).
        dp.USART6.cr1().modify(|_, w| w.ue().clear_bit());

        // 3. Write new BRR.
        dp.USART6.brr().write(|w| unsafe { w.bits(next_brr) });

        // 4. Re-enable USART — RXNEIE stays OFF until we've drained stale bytes.
        // RM0090 §30.3.2: UE=0 clears error flags but the RXNE bit (data already
        // in DR) is RETAINED.  If a byte landed in DR before UE=0 (e.g. a glitch
        // during the host baud-rate switch), RXNE=1 on UE=1 would fire IRQ71
        // immediately when RXNEIE is set, before the new-rate sweep even starts.
        // That wake would have rc=0 (blocking path woken) with RXNE already 0
        // by the time the partition reads SR — spurious wake loop at every new rate.
        dp.USART6.cr1().modify(|_, w| w.ue().set_bit());

        // Drain stale bytes (RXNEIE still off → no kernel events generated).
        let n3 = polling_drain(10);
        let _ = dprint!(&UART_P0_DEBUG, @size 80,
            "[UART] post-switch drain: {} byte(s) discarded (next rate: {})",
            n3, BAUD_TABLE[next_idx].2);

        // Clear any stale event_flags — same race-free window as post-hs drain:
        // RXNEIE=0 means __irq_dispatch cannot fire, so this clear is not racy.
        plib::sys_event_clear(UART_RX_EVENT).ok();

        // 5. Re-enable RXNEIE. event_flags=0, first SYS_EVT_WAIT blocks cleanly.
        dp.USART6.cr1().modify(|_, w| w.rxneie().set_bit());
    }

    RATE_IDX.store(BAUD_TABLE.len() as u32, Ordering::Release);
    let _ = dprint!(&UART_P0_DEBUG, @size 64, "[UART] Sweep complete. Final TX={} RX={} ERR={}",
        TX_COUNT.load(Ordering::Acquire),
        RX_COUNT.load(Ordering::Acquire),
        ERR_COUNT.load(Ordering::Acquire));

    loop { plib::sys_yield().ok(); }
}

kernel::partition_trampoline!(uart_driver => uart_driver_body);

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
#[entry]
fn main() -> ! {
    // Use a 4 KB up-channel so diagnostic output doesn't overflow during the sweep.
    let channels = rtt_init! {
        up: {
            0: {
                size: 4096,
                mode: rtt_target::ChannelMode::NoBlockTrim,
                name: "Terminal",
            }
        }
    };
    rtt_target::set_print_channel(channels.up.0);
    rprintln!("\n=== UART IRQ Echo Demo — Full-Duplex End-to-End + Baud Rate Sweep ===");
    rprintln!("Wiring: PG14/CN10-D1(TX)→adapter_RX  PG9/CN10-D0(RX)←adapter_TX  GND");
    rprintln!("Host:   python3 f429zi/host_serial_echo.py --port /dev/ttyUSB0");

    // -------------------------------------------------------------------------
    // Hardware init — privileged, pre-MPU.
    // -------------------------------------------------------------------------
    let dp = pac::Peripherals::take().unwrap();

    // 168 MHz sysclk via HSE+PLL; APB2 = 84 MHz for clean UART BRR values.
    let mut rcc = dp.RCC.freeze(
        Config::hse(8.MHz())
            .sysclk(168.MHz()),
    );

    let gpiog = dp.GPIOG.split(&mut rcc);

    // PG14 = USART6_TX (AF8), PG9 = USART6_RX (AF8) — CN10 D1/D0.
    // PG9 (RX) gets an internal pull-up: keeps the line high (idle) when
    // the adapter is disconnected or powering up, preventing false start bits
    // from a floating line that would trigger spurious RXNE events.
    let pg14_tx = gpiog.pg14.into_alternate::<8>().speed(Speed::VeryHigh);
    let pg9_rx  = gpiog.pg9.into_alternate::<8>()
        .internal_resistor(Pull::Up)
        .speed(Speed::VeryHigh);

    // HAL Serial: clock enable + BRR (from 84 MHz pclk2) + full CR1 (8N1).
    // Start at the first rate in the table; partition handles subsequent changes.
    let initial_baud = BAUD_TABLE[0].0;
    let _serial = dp.USART6
        .serial::<u8>((pg14_tx, pg9_rx), initial_baud.bps(), &mut rcc)
        .expect("USART6 init");
    // No HDSEL! No steal() needed. Full-duplex: CR3 stays reset (= 0).

    rprintln!("[INIT] USART6 {}bps 8N1 full-duplex (HAL, no HDSEL)", initial_baud);
    rprintln!("[INIT] GPIO PG14/PG9 AF8 VeryHigh; APB2=84MHz");

    // -------------------------------------------------------------------------
    // Kernel setup.
    // -------------------------------------------------------------------------
    let usart6_base = pac::USART6::PTR as usize as u32;
    rprintln!("P0 peripheral_regions: USART6=0x{:08x}/1KB", usart6_base);

    // Check & clear stale NVIC pending bit for IRQ 71 BEFORE unmasking.
    // NVIC ICPR base = 0xE000E280; IRQ71 is bit7 of ICPR[2] (0xE000E288).
    let nvic_ispr2 = unsafe { (0xE000E200usize as *const u32).add(2).read_volatile() };
    let irq71_pending_before = (nvic_ispr2 >> 7) & 1;
    if irq71_pending_before != 0 {
        unsafe { (0xE000E280usize as *mut u32).add(2).write_volatile(1 << 7) };
        rprintln!("[INIT] Cleared stale NVIC pending bit for IRQ 71");
    }
    rprintln!("[INIT] IRQ71 pending before unmask: {}", irq71_pending_before);

    let mut p = cortex_m::Peripherals::take().unwrap();

    let mut sched = ScheduleTable::<{ UartEchoCfg::SCHED }>::new();
    sched.add(ScheduleEntry::new(0, 4)).expect("sched P0");
    sched.add_system_window(2).expect("sched SW");

    let parts: [PartitionSpec; NUM_PARTITIONS] = [(uart_driver, USART6_IRQ as u32)];

    store_kernel(&mut init_kernel(sched, &parts).expect("init_kernel"));

    // Register P0's debug ring buffer so dprint! output drains via RTT.
    kernel::state::with_kernel_mut::<UartEchoCfg, _, _>(|k| {
        k.partitions_mut()
            .get_mut(0)
            .unwrap()
            .set_debug_buffer(&UART_P0_DEBUG);
        Ok::<(), ()>(())
    }).expect("ipc setup");

    enable_bound_irqs(&mut p.NVIC, UartEchoCfg::IRQ_DEFAULT_PRIORITY).unwrap();
    rprintln!("[INIT] IRQ 71 unmasked, kernel ready — booting\n");

    match boot(p).expect("boot") {}
}

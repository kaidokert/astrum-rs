# QEMU UART0 Split-ISR Pipeline Demo

## 1. Overview

`kernel/examples/uart_irq_demo.rs` is an end-to-end integration test for the
split-ISR pipeline on the LM3S6965 QEMU target.  It exercises the full
four-step IRQ flow (see `notes/split-isr-interrupt-routing.md` §2) using
UART0 (IRQ 5) as the device under test:

1. **IRQ fires** — `NVIC::pend` simulates a UART0 RX event.
2. **Custom ISR** — `uart0_rx_isr` reads `UART0_DR`, pushes data into a
   `StaticIsrRing`, signals P0, and masks the IRQ.
3. **Partition bottom-half** — P0 wakes via `SYS_EVT_WAIT`, drains the
   ring, validates tag and payload length, increments `POP_COUNT`.
4. **IRQ ACK** — P0 calls `SYS_IRQ_ACK` to unmask UART0 for the next
   event.

The demo validates ISR dispatch, ring buffer transport, partition wake-up,
and the IRQ mask/unmask lifecycle in a single QEMU run.

## 2. Custom ISR — `uart0_rx_isr`

Bound via the `handler:` form of `bind_interrupts!`:

```rust
kernel::bind_interrupts!(Cfg, 70,
    5 => (0, 0x01, handler: uart0_rx_isr),
);
```

The ISR performs three operations:

1. **Read UART0_DR** — `read_volatile(0x4000_C000 as *const u32)`,
   extracting the low byte as the payload.
2. **Push to ring** — `RING.push_from_isr(UART0_IRQ, &[byte])` stores a
   1-byte tagged record in the `StaticIsrRing<4, 1>` (4 slots, 1-byte
   payload).
3. **Signal + mask** — `signal_partition_from_isr::<Cfg>(0, 0x01)` wakes
   P0, then `NVIC::mask(IrqNr(5))` prevents re-fire until the partition
   acknowledges.

## 3. Partition Bottom-Half — P0

P0 runs an infinite loop:

1. **Wait** — `SYS_EVT_WAIT(0x01)` blocks until the ISR signals.
2. **Drain** — `RING.pop_with(|tag, data| ...)` iterates until empty.
   Each record is validated:
   - `tag == UART0_IRQ (5)` — correct source.
   - `data.len() == 1` — expected payload size.
   - `POP_COUNT` incremented only after validation succeeds.
3. **ACK** — `SYS_IRQ_ACK(UART0_IRQ)` unmasks the IRQ via the kernel's
   ownership-checked unmask path.

Failure at any step prints a diagnostic and exits with `EXIT_FAILURE`.

## 4. Test Harness

The `define_unified_harness!` SysTick hook drives the test:

| Tick | Action |
|------|--------|
| 2    | `NVIC::pend(IRQ 5)` — first simulated RX event |
| 6    | `NVIC::pend(IRQ 5)` — second simulated RX event |
| ≥10  | Check `POP_COUNT >= 2` → **PASS** |
| ≥16  | Timeout → **FAIL** with pop count |

Two pend/pop cycles confirm the full round-trip works repeatedly, not
just once.

## 5. QEMU Limitation

QEMU's LM3S6965 UART model does not support injecting bytes into the RX
FIFO from software.  Reading `UART0_DR` after a software-pended IRQ
returns whatever QEMU's model yields (typically zero), not a controlled
test value.

**What is validated:** IRQ pend → ISR entry → ring push → partition
wake → ring pop → tag check → IRQ ACK → re-arm.

**What is not validated:** Payload *content* correctness.  The comment
in `uart_irq_demo.rs:112-114` documents this gap.  True payload
validation requires either real hardware or a QEMU model that supports
character injection (e.g. via a socket backend, which is outside the
scope of this demo).

## 6. References

- **Source:** `kernel/examples/uart_irq_demo.rs`
- **Parent spec:** `notes/split-isr-interrupt-routing.md` — §2 (four-step
  IRQ flow), §6 (IsrRingBuffer), §10.1 (custom-handler binding)
- **Ring buffer:** `kernel/src/split_isr.rs` — `StaticIsrRing<D, M>`
- **IRQ ACK:** `kernel/src/irq_ack.rs` — `SYS_IRQ_ACK` syscall

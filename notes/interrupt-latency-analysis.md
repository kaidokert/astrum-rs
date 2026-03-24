# Interrupt Latency Analysis — Split-ISR Model

## 1. Context

This document analyses bottom-half scheduling latency for the split-ISR
interrupt routing model described in `notes/split-isr-interrupt-routing.md`
§2 (Four-Step IRQ Flow). The top-half (hardware dispatch in Handler mode)
runs with near-zero latency via the NVIC; the latency discussed here is
the delay between the top-half signal and the partition's bottom-half
execution in Thread mode.

## 2. Definitions

| Symbol | Meaning |
|--------|---------|
| `N` | Number of time slots in the schedule |
| `S` | Slot duration (constant across all slots) |
| `T` | Major frame period = `N × S` |

The scheduler is a static cyclic executive: each partition owns one or
more fixed slots per major frame. A partition signalled by the top-half
becomes `Ready` and runs in its **next scheduled slot**.

## 3. Latency Bounds

### 3.1 Best Case — 0

The IRQ fires at the exact start of the target partition's own slot.
The top-half pends PendSV, the scheduler tail-chains into the partition
immediately, and the bottom-half runs with effectively zero additional
scheduling delay.

### 3.2 Worst Case — T (full major frame period)

The IRQ fires just **after** the partition's slot begins (e.g. one tick
into the slot). The partition is already running and has already called
`event_wait`; the new event bits are OR'd in but the partition will not
re-enter `event_wait` until its **next** slot one full major frame later.

    worst_case_latency = T = N × S

### 3.3 Average Case — T / 2

Assuming IRQs arrive uniformly over time, the expected delay to the
partition's next slot is half the major frame period:

    average_latency = T / 2 = (N × S) / 2

## 4. Numeric Example

Consider a schedule with **5 partitions**, each owning one **2 ms** slot:

    N = 5,  S = 2 ms,  T = 5 × 2 ms = 10 ms

| Case | Latency |
|------|---------|
| Best | 0 ms |
| Worst | 10 ms |
| Average | 5 ms |

If a UART RX interrupt fires 0.5 ms after Partition 2's slot starts,
Partition 2 must wait ≈ 9.5 ms (the remainder of the major frame) before
its next slot.

## 5. Model A (KernelClears) vs Model B (PartitionAcks)

The `IrqClearModel` enum (see `split-isr-interrupt-routing.md` §3)
determines **when the hardware interrupt source is silenced**, which
affects re-fire behaviour and effective latency.

### 5.1 Model A — `KernelClears`

The top-half writes an MMIO register to clear the hardware source
immediately during step 2 of the four-step flow. The IRQ line is
de-asserted before the handler returns.

- **Re-fire window:** None. The source is cleared in the top-half.
- **NVIC state:** IRQ stays unmasked — new occurrences of the same
  event can fire and be dispatched again before the partition runs.
- **Bottom-half scheduling latency:** Unchanged (§3 bounds apply).

### 5.2 Model B — `PartitionAcks`

The top-half **masks** the IRQ in the NVIC (step 2) but does **not**
clear the hardware source. The source remains asserted until the
partition's bottom-half runs in its next slot and calls `SYS_IRQ_ACK`,
which unmasks the IRQ after the partition has acknowledged the device.

- **Re-fire window:** The IRQ is masked from step 2 until the partition
  calls `SYS_IRQ_ACK`, so subsequent edges/levels from the same source
  are **not seen** during this interval.
- **NVIC state:** Masked between top-half and `SYS_IRQ_ACK`.
- **Effective deaf period:** Equal to the bottom-half scheduling latency
  (up to `T` worst case). During this window the system cannot detect
  new events from the same source.

### 5.3 Comparison Summary

| Property | Model A (KernelClears) | Model B (PartitionAcks) |
|----------|----------------------|------------------------|
| Source cleared by | Top-half (Handler mode) | Partition bottom-half (Thread mode) |
| IRQ masked between signal and ack | No | Yes |
| Can detect new events while waiting | Yes | No (deaf period up to `T`) |
| Partition trust required | Low — kernel handles HW | Higher — partition must call `SYS_IRQ_ACK` |
| Suitable for | Simple status-flag devices | Complex devices needing partition-side clear |

For the 5×2 ms example: under Model B, a worst-case deaf period of 10 ms
means a second UART byte arriving during that window is lost unless the
device has its own FIFO. Model A avoids this by clearing immediately,
but requires the kernel to know the device's clear protocol at build time
via `ClearStrategy`.

## 6. Use-Case Selection Guide

The table below maps common embedded use cases to their typical interrupt
frequencies and the recommended interrupt handling approach within this
RTOS. "Split-ISR" refers to the four-step flow described in §1, using
either Model A or Model B from §5. "Kernel top-half only" means the
kernel handles the entire interrupt in Handler mode with no bottom-half.
"Dedicated ISR" means the device is handled outside the Split-ISR
framework entirely, in a hard-real-time ISR with no kernel involvement.

| Use Case | Typical Frequency | Recommended Approach | Rationale |
|---|---|---|---|
| UART RX | < 1 kHz | Model B (PartitionAcks) | Partition reads FIFO and clears status; deaf period acceptable given UART FIFO depth and low rate. |
| GPIO debounce | 10–100 Hz | Model A (KernelClears) | Simple flag-clear in top-half; low rate means scheduling latency ≤ T is always adequate. |
| ADC sampling | 1–10 kHz | Kernel top-half only | DMA descriptor kick or buffer-swap must happen within microseconds; bottom-half latency up to T is too slow for continuous streaming. |
| Motor control (FOC) | 10–50 kHz | Dedicated ISR | Control loop deadline is 20–100 µs; any scheduling delay through PendSV would exceed the electrical time constant. |
| Power converter | > 50 kHz | Dedicated ISR | Sub-microsecond response required; the NVIC-to-handler path must be the only latency. No kernel involvement is viable. |

### When Split-ISR Is Inappropriate

The Split-ISR model routes interrupt processing through the cyclic
scheduler, which means the bottom-half cannot run more often than once
per major frame. This imposes a **fundamental frequency ceiling of
1/T Hz** — any interrupt source that fires faster than once per major
frame period `T` will experience event loss (Model B) or unbounded
event-bit accumulation (Model A) because the partition cannot drain
events as fast as they arrive.

For the 5×2 ms example schedule (T = 10 ms), this ceiling is 100 Hz.
Any source that must be serviced at or above 100 Hz should not use the
Split-ISR bottom-half path. Instead, use a kernel top-half-only handler
(for moderate rates up to ~10 kHz where Handler-mode processing time is
short) or a dedicated ISR outside the kernel's interrupt framework
(for rates above ~10 kHz where even top-half overhead is unacceptable).

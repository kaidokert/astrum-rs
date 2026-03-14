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

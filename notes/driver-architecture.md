# Driver Architecture

The core architectural insight: **the kernel is a resource mediator, not
a transaction mediator.** It provides resources — MPU regions, interrupt
routing, IPC channels — rather than intermediating every hardware
transaction on behalf of partitions. The kernel never touches a UART
data register or clocks an SPI byte. It grants access and gets out of
the way.

Traditional RTOS driver models place peripheral drivers inside the
kernel, making every `spi.write(&buf)` a syscall. This adds per-operation
overhead, forces the kernel to understand every peripheral protocol, and
creates ABI surface that must be maintained across releases. This RTOS
takes the opposite approach: for dedicated peripherals the kernel maps
the MMIO register block into the owning partition's MPU space, and the
partition runs standard PAC/HAL crate code directly.

---

## Table of Contents

1. [User-Space Drivers for Dedicated Peripherals](#1-user-space-drivers-for-dedicated-peripherals)
2. [Interrupt Routing Model](#2-interrupt-routing-model)
3. Shared Bus Arbitration (planned)
4. Kernel-Mediated Fallback (planned)

---

## 1. User-Space Drivers for Dedicated Peripherals

### 1.1 Mechanism: MPU Peripheral Pass-Through (Approach D)

When a peripheral is owned exclusively by one partition, the kernel
grants that partition direct hardware access through the MPU:

1. **Static configuration.** The partition's `PartitionConfig` declares
   peripheral register blocks via the `peripheral_regions` field.
2. **MPU grant.** The kernel programs an MPU region covering the
   peripheral's MMIO range with unprivileged RW access, using
   `DynamicStrategy::add_window` to allocate a dynamic slot (R5-R7).
3. **Direct register access.** The partition runs vendor PAC/HAL code
   against the granted range — plain memory-mapped loads and stores,
   no syscall, no context switch, no kernel involvement.
4. **Interrupt routing.** The kernel configures the NVIC to deliver
   the peripheral's interrupt to the owning partition.

The result is **zero syscall overhead** for normal peripheral I/O.

### 1.2 Kernel Data Structures

The `PartitionControlBlock` stores up to two peripheral regions per
partition (`kernel/src/partition.rs`):

```rust
peripheral_regions: Vec<MpuRegion, 2>,
```

Each `MpuRegion` carries a base address, size, and permissions word:

```rust
pub struct MpuRegion {
    base: u32,
    size: u32,
    permissions: u32,
}
```

The `heapless::Vec` capacity of 2 covers the common case without allocation.

### 1.3 MPU Window Allocation

`DynamicStrategy` manages MPU regions R4-R7 at runtime. Slot 0 (R4)
holds the partition's private RAM; slots 1-3 (R5-R7) are available
for peripheral windows and IPC grants. From `kernel/src/mpu_strategy.rs`,
`DynamicStrategy::add_window` validates alignment and power-of-two
constraints, then scans slots 1-3 inside a critical section:

```rust
fn add_window(&self, base: u32, size: u32, permissions: u32, owner: u8)
    -> Result<u8, MpuError>
{
    crate::mpu::validate_mpu_region(base, size)?;
    with_cs(|cs| {
        let mut slots = self.slots.borrow(cs).borrow_mut();
        for (idx, slot) in slots.iter_mut().enumerate().skip(1) {
            if slot.is_none() {
                *slot = Some(WindowDescriptor { base, size, permissions, owner });
                return Ok(DYNAMIC_REGION_BASE + idx as u8);
            }
        }
        Err(MpuError::SlotExhausted)
    })
}
```

### 1.4 Decision Rationale

The choice of Approach D over kernel-mediated alternatives rests on a
single observation: **interfaces are expensive, implementation is cheap.**

Every kernel-mediated peripheral class requires an operation enum
(`SpiOp`, `I2cOp`) defining the syscall ABI, a virtual device dispatch
path in the SVC handler, an error translation layer, and documentation
and versioning for all of the above. This is ABI surface the RTOS team
must own indefinitely.

Under Approach D the hardware register map *is* the interface, and the
community-maintained `embedded-hal` trait impls running in partition
space *are* the driver. The kernel introduces **zero new ABI** for
peripheral access — only MPU region configuration and interrupt routing,
mechanisms that already exist for memory isolation.

The trade-off: Approach D requires dedicated peripheral ownership (one
partition per peripheral). For shared buses, a server partition pattern
using existing IPC primitives handles arbitration without new kernel ABI.

---

## 2. Interrupt Routing Model

When a peripheral is passed through to a partition via MPU (Section 1),
its hardware IRQ must still be handled safely.  The partition cannot
install an ISR directly — the kernel owns the NVIC and the vector table.
The kernel provides a **split-ISR** model that separates interrupt
acknowledgement (privileged, deterministic) from interrupt processing
(unprivileged, partition-scheduled).

### 2.1 Four-Step IRQ Flow

1. **IRQ fires → kernel ISR top-half acknowledges minimally.**
   The kernel's ISR runs in Handler mode at privileged level.  It reads
   the peripheral's status register to identify the event, then clears
   the hardware condition that asserted the interrupt (status flag,
   pending bit, or FIFO threshold — whatever the peripheral requires to
   de-assert its IRQ line).  Clearing the source is **mandatory**; if
   the ISR returns without doing so, the NVIC re-triggers the vector
   immediately, causing an interrupt livelock that starves all
   Thread-mode execution.

2. **Kernel signals event via `event_set()` or pushes to
   `IsrRingBuffer`.**  Two notification mechanisms, both O(1) and
   allocation-free:

   - **Lightweight event flags** — `event_set(partition_id, mask)` sets
     bits in the target partition's event flags and wakes it if Waiting
     (`kernel/src/events.rs`).  Sufficient when the partition only needs
     to know *that* an interrupt fired, not carry per-event payload.

   - **Ring buffer delivery** —
     `IsrRingBuffer::push_from_isr(tag, data)` enqueues an
     `EventRecord` with a `DeviceNotification` tag and up to `M` bytes
     of payload (`kernel/src/split_isr.rs`).  Used when the ISR must
     carry data (UART RX bytes, ADC samples).

3. **Partition driver handles interrupt in its scheduling context.**
   The partition runs in its next time slot (Thread mode, unprivileged).
   It calls `event_wait(mask)` to consume event flags, or
   `isr_ring.pop_with(|tag, data| ..)` to drain buffered records.  The
   full vendor HAL runs here — the kernel never touches peripheral data
   registers.

4. **Partition may use async/await natively.**  Because `event_wait`
   blocks the partition (setting `PartitionState::Waiting`) and
   `event_set` wakes it (transitioning to `Ready`), the event flags API
   maps naturally onto Rust `async`/`await` — a future polls
   `event_wait`, yields when no flags match, and resumes when the kernel
   ISR sets flags via `event_set`.

### 2.2 IRQ Flow Diagram

```
Hardware IRQ
    │
    ▼
┌───────────────────────────────────────┐
│  Kernel ISR (Handler mode, privileged)│
│  1. Read peripheral status register   │
│  2. Clear IRQ source                  │
│  3a. event_set(owner, IRQ_MASK)       │
│      — OR —                           │
│  3b. isr_ring.push_from_isr(tag, data)│
└───────────────────────────────────────┘
    │  (deferred to partition time slot)
    ▼
┌───────────────────────────────────────┐
│  Partition bottom-half (Thread mode)  │
│  event_wait(IRQ_MASK)                 │
│  — OR —                               │
│  isr_ring.pop_with(|tag, payload| ..) │
│  Full vendor HAL processing           │
└───────────────────────────────────────┘
```

**Latency note:** The partition cannot respond until its next scheduled
time slot.  Worst-case latency is one full major frame (the sum of all
other partitions' slots).  For peripherals with sub-microsecond response
requirements, the kernel top-half must perform the time-critical work
directly; only the non-critical processing is deferred.

### 2.3 Contrast: Kernel-Internal Interrupts (SysTick, PendSV)

The split-ISR model above applies to **device interrupts** routed to
partitions.  The kernel's own exceptions follow a different pattern
where all work completes in the top-half:

- **SysTick** (`kernel/src/systick.rs`) runs entirely in Handler mode.
  It advances the schedule via `systick_handler()`, determines the next
  partition, and triggers PendSV.  There is no bottom-half and no
  partition notification — the kernel completes all tick processing
  before returning from the exception.

- **PendSV** (`kernel/src/pendsv.rs`) performs the context switch in
  hand-written assembly at the lowest exception priority (0xFF).  It
  saves the outgoing partition's r4-r11 and PSP, updates
  `current_partition`, restores the incoming partition's registers, and
  returns to unprivileged Thread mode.  This is pure kernel machinery.

The key distinction: kernel exceptions do all their work in the
top-half because they implement kernel-internal scheduling mechanics.
Device interrupts use the split model because the processing logic
belongs to the owning partition — the kernel only acknowledges and
relays.

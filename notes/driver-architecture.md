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
2. Shared Bus Arbitration (planned)
3. Interrupt Routing (planned)
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

# Buffer Pool Lend/Revoke Syscall Architecture

**Status:** Implemented
**Scope:** `kernel/src/buffer_pool.rs`, `kernel/src/mpu_strategy.rs`, `kernel/src/mpu.rs`, `kernel/src/svc.rs`, `kernel/src/syscall.rs`
**Feature gate:** `dynamic-mpu`

---

## 1. Problem Statement

The kernel provides a fixed-size buffer pool (`BufferPool<SLOTS, SIZE>`) for zero-copy inter-partition data sharing. Today the pool supports two lending models:

1. **Kernel-initiated lending** (`lend_to_partition` / `revoke_from_partition`) — privileged-only, used internally by the kernel (e.g. SysTick handler). Takes a `Free` slot, installs an MPU window, transitions to `BorrowedRead` or `BorrowedWrite`.

2. **Partition-initiated sharing** (`share_with_partition` / `unshare_from_partition`) — exposed as `SYS_BUF_LEND` (34) and `SYS_BUF_REVOKE` (35). Requires the caller to own the slot in `BorrowedWrite` state; grants the target a read-only MPU window while the owner retains write access.

### What's missing

The current `SYS_BUF_LEND` / `SYS_BUF_REVOKE` syscalls only support one pattern: **owner-writes, target-reads**. This is insufficient for several real-world scenarios:

| Scenario | Required access | Current support |
|----------|----------------|-----------------|
| Producer fills buffer, consumer reads | Owner=RW, Target=RO | Yes (share_with_partition) |
| DMA receive: peripheral writes into buffer, partition reads result | Owner=RO, Target=RW | **No** — share requires BorrowedWrite, and target always gets RO |
| Bidirectional IPC: both partitions read+write a shared region | Owner=RW, Target=RW | **No** — target always gets AP_RO_RO |
| Partition A lends a Read-borrowed buffer to Partition B | Owner=RO, Target=RO | **No** — share requires BorrowedWrite |
| DMA handoff: owner prepares buffer, yields ownership to DMA agent | Transfer of ownership | **No** — ownership is fixed at alloc time |

The USB high-speed DMA work item (`usb-highspeed-dma.md`) and the static DMA allocator (`static-dma-allocator.md`) both depend on a richer lending model that supports writable grants and safe ownership transfer for DMA-safe handoff.

---

## 2. Current Buffer Pool State Machine

### 2.1 States

```
BorrowState:
  Free                       — slot available for allocation
  BorrowedRead  { owner: u8 } — slot borrowed read-only
  BorrowedWrite { owner: u8 } — slot borrowed read-write
```

Auxiliary per-slot metadata:
- `mpu_region: Option<u8>` — MPU region ID from `lend_to_partition` (kernel-initiated)
- `lent_to: Option<LendRecord>` — cross-partition share record from `share_with_partition`
- `deadlines[slot]: Option<u64>` — auto-revoke tick

### 2.2 Transitions (current)

```
                    alloc(pid, Read)
           Free ─────────────────────> BorrowedRead{pid}
                    alloc(pid, Write)
           Free ─────────────────────> BorrowedWrite{pid}

     BorrowedRead{pid}  ──release(pid)──>  Free
     BorrowedWrite{pid} ──release(pid)──>  Free   (only if lent_to == None)

     BorrowedWrite{pid} ──share(pid,target,strategy)──> BorrowedWrite{pid}
                           + lent_to = Some(LendRecord{target, region_id})
                           + MPU window: AP_RO_RO for target

     BorrowedWrite{pid} ──unshare(pid,target,strategy)──> BorrowedWrite{pid}
            (with lent_to)   - lent_to = None
                              - MPU window removed

     Free ──lend_to_partition(pid, writable, strategy)──> Borrowed{Read|Write}{pid}
                           + mpu_region = Some(region_id)
                           + MPU window: AP_RO_RO or AP_FULL_ACCESS

     Borrowed*{pid} ──revoke_from_partition(strategy)──> Free
       (with mpu_region)   - mpu_region = None
                            - deadline = None
                            - MPU window removed
```

### 2.3 Invariants

1. A slot with `lent_to.is_some()` cannot be released (prevents dangling MPU windows).
2. A slot can have at most one active `lent_to` record (single target per share).
3. `share_with_partition` requires `BorrowedWrite` — `BorrowedRead` owners cannot share.
4. Self-lend (owner == target) is rejected.
5. `lend_to_partition` requires `Free` — only the kernel can initiate a cold-start lend.

---

## 3. Proposed Approach

### 3.1 Design goals

1. **Writable grants**: allow an owner to grant `AP_FULL_ACCESS` (RW) to a target partition, not just `AP_RO_RO`.
2. **Read-mode sharing**: allow `BorrowedRead` owners to share read-only access with a target.
3. **DMA-safe handoff protocol**: define a transfer-of-ownership sequence so one partition can prepare a buffer, hand it to a DMA-agent partition, and reclaim it after transfer completes.
4. **Backward compatibility**: existing `SYS_BUF_LEND` / `SYS_BUF_REVOKE` semantics remain valid (default = read-only grant from write-owner).
5. **Minimal new syscalls**: extend existing syscall encodings rather than adding new numbers.

### 3.2 Extended syscall encoding

#### SYS_BUF_LEND (34) — extended

Current encoding:
```
r0 = SYS_BUF_LEND (34)
r1 = slot_index
r2 = target_partition_id
```

Extended encoding (backward-compatible):
```
r0 = SYS_BUF_LEND (34)
r1 = slot_index
r2 = target_partition_id | (flags << 8)
       bits [7:0]  = target_partition_id (0-255)
       bits [15:8] = flags:
           bit 8: WRITABLE — grant AP_FULL_ACCESS instead of AP_RO_RO
           bits 9-15: reserved (must be 0)
```

When `flags == 0` (all bits [15:8] zero), behavior is identical to today. This preserves backward compatibility — existing callers passing a raw partition ID in r2 (which is always < 256 for valid partitions) get the same read-only grant.

Return value (unchanged):
```
r0 = MPU region ID (4-7) on success
r0 = SvcError code on failure
```

#### SYS_BUF_REVOKE (35) — unchanged

```
r0 = SYS_BUF_REVOKE (35)
r1 = slot_index
r2 = target_partition_id    (bits [7:0] only; upper bits ignored)
```

No changes needed. Revoke always removes the MPU window and clears `lent_to`.

#### New syscall: SYS_BUF_TRANSFER (36)

```
r0 = SYS_BUF_TRANSFER (36)
r1 = slot_index
r2 = new_owner_partition_id
```

Transfers ownership of a borrowed slot to a different partition. The caller must be the current owner and the slot must not have an active lend (`lent_to == None`). The buffer data is not touched — only the ownership metadata changes.

Return value:
```
r0 = 0 on success
r0 = SvcError::InvalidResource if not owner, slot free, or lent
r0 = SvcError::InvalidPartition if new_owner is out of range
r0 = SvcError::OperationFailed if new_owner == current_owner (no-op rejected)
```

#### New syscall: SYS_BUF_READ (37)

```
r0 = SYS_BUF_READ (37)
r1 = slot_index
r2 = length (bytes to read)
r3 = destination pointer (in caller's partition memory)
```

Symmetric counterpart to `SYS_BUF_WRITE` (26). Copies data from a kernel buffer slot into the caller's memory. The caller must own the slot in either `BorrowedRead` or `BorrowedWrite` state.

Return value:
```
r0 = bytes copied on success
r0 = SvcError::InvalidResource if not owner or slot free
r0 = SvcError::InvalidPointer if destination fails pointer validation
r0 = SvcError::OperationFailed if length exceeds buffer SIZE
```

### 3.3 Extended buffer pool operations

#### 3.3.1 `share_with_partition` — add `writable` parameter

```rust
pub fn share_with_partition(
    &mut self,
    slot: usize,
    owner: u8,
    target: u8,
    writable: bool,              // NEW: false=AP_RO_RO, true=AP_FULL_ACCESS
    strategy: &dyn MpuStrategy,
) -> Result<u8, BufferError> { ... }
```

Changes from current implementation:
- Accept `BorrowedRead` owners when `writable == false` (read-only grant from read-only owner is safe).
- When `writable == true`, require `BorrowedWrite` owner (cannot grant RW from RO borrow).
- Select `AP_RO_RO` or `AP_FULL_ACCESS` based on `writable` flag.

State machine addition:
```
BorrowedRead{pid}  ──share(pid, target, writable=false)──> BorrowedRead{pid}
                      + lent_to = Some(LendRecord{target, region_id})
                      + MPU window: AP_RO_RO for target

BorrowedWrite{pid} ──share(pid, target, writable=true)──> BorrowedWrite{pid}
                      + lent_to = Some(LendRecord{target, region_id})
                      + MPU window: AP_FULL_ACCESS for target
```

New validation rule:
- If `writable == true` and state is `BorrowedRead`, return `Err(BufferError::NotOwner)` (or a new `BufferError::PermissionDenied`).

#### 3.3.2 `transfer_ownership` — new method

```rust
pub fn transfer_ownership(
    &mut self,
    slot: usize,
    current_owner: u8,
    new_owner: u8,
) -> Result<(), BufferError> {
    let s = self.slots.get_mut(slot).ok_or(BufferError::InvalidSlot)?;
    match s.state {
        BorrowState::Free => return Err(BufferError::SlotNotBorrowed),
        BorrowState::BorrowedRead { owner } | BorrowState::BorrowedWrite { owner }
            if owner != current_owner =>
        {
            return Err(BufferError::NotOwner);
        }
        _ => {}
    }
    if s.lent_to.is_some() {
        return Err(BufferError::AlreadyLent);
    }
    if current_owner == new_owner {
        return Err(BufferError::SelfLend);  // reuse variant for self-transfer
    }
    // Transfer: keep borrow mode, change owner.
    s.state = match s.state {
        BorrowState::BorrowedRead { .. } => BorrowState::BorrowedRead { owner: new_owner },
        BorrowState::BorrowedWrite { .. } => BorrowState::BorrowedWrite { owner: new_owner },
        _ => unreachable!(),
    };
    Ok(())
}
```

Key properties:
- Borrow mode (Read/Write) is preserved across transfer.
- No MPU operations — only metadata changes.
- Cannot transfer while lent (must revoke first).

#### 3.3.3 `read_from_slot` — new method

```rust
pub fn read_from_slot(
    &self,
    slot: usize,
    owner: u8,
    dst: &mut [u8],
) -> Result<usize, BufferError> {
    let s = self.slots.get(slot).ok_or(BufferError::InvalidSlot)?;
    match s.state {
        BorrowState::BorrowedRead { owner: o } | BorrowState::BorrowedWrite { owner: o }
            if o == owner => {}
        BorrowState::Free => return Err(BufferError::SlotNotBorrowed),
        _ => return Err(BufferError::NotOwner),
    }
    let len = dst.len().min(s.data.len());
    dst[..len].copy_from_slice(&s.data[..len]);
    Ok(len)
}
```

### 3.4 LendRecord extension

The `LendRecord` needs to track the granted permission level so revocation can be validated:

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LendRecord {
    pub target: u8,
    pub region_id: u8,
    pub writable: bool,   // NEW: true if AP_FULL_ACCESS was granted
}
```

This enables future introspection (e.g. `SYS_BUF_STATUS` syscall) and ensures the kernel can log/audit the permission level of each grant.

---

## 4. MPU Region Grant/Revoke Flow

### 4.1 Dynamic region layout

```
R0-R3: Static partition regions (background, code, data, stack guard)
R4:    Partition private RAM (or peripheral MMIO slot 0 if peripheral_reserved=2)
R5:    Dynamic window slot 1 (peripheral MMIO slot 1 OR buffer window)
R6:    Dynamic window slot 2 (buffer window)
R7:    Dynamic window slot 3 (buffer window)
```

Available window slots for buffer lending: **R5-R7** (3 slots max, fewer if peripherals reserved).

### 4.2 Grant flow (SYS_BUF_LEND with WRITABLE flag)

```
Partition A (owner, pid=0)     Kernel SVC handler           DynamicStrategy
────────────────────────────────────────────────────────────────────────
SVC #34 (r1=slot, r2=1|0x100)
  ↓
                               1. Extract target=r2 & 0xFF → 1
                                  Extract flags=(r2>>8) & 0xFF → 0x01
                                  writable = flags & 1 → true
                               2. Validate target < partitions.len()
                               3. Call buffers.share_with_partition(
                                    slot, pid=0, target=1,
                                    writable=true, &strategy)
                               4. Inside share_with_partition:
                                  a. Validate state=BorrowedWrite{0} ✓
                                  b. Validate lent_to==None ✓
                                  c. Validate 0 != 1 (not self) ✓
                                  d. Compute RASR:
                                     ap = AP_FULL_ACCESS (writable=true)
                                     size_field = encode_size(SIZE)
                                     rasr = build_rasr(sf, ap, xn=true, scb=(0,0,0))
                                  e. base = &slots[slot].data as u32
                                  f. Call strategy.add_window(
                                       base, SIZE, rasr, owner=1)
                                                                      Find first free slot
                                                                      in slots[1..3] (R5-R7)
                                                                      → say slot 1 (R5)
                                                                      Store WindowDescriptor{
                                                                        base, size: SIZE,
                                                                        permissions: rasr,
                                                                        owner: 1
                                                                      }
                                                                      Return Ok(5)
                               5. Record lent_to = LendRecord{
                                    target=1, region_id=5, writable=true
                                  }
                               6. Return region_id=5 in r0
```

### 4.3 Context switch MPU programming

When PendSV switches to partition 1 (the target):

```
PendSV handler:
  1. Save outgoing partition context (r4-r11, PSP)
  2. Call STRATEGY.program_regions(&MPU):
     a. Disable MPU
     b. For each slot 0..3 (R4-R7):
        - Read WindowDescriptor from slots[]
        - If desc.owner == incoming_partition_id:
            Write RBAR/RASR to enable the region
        - Else if slot holds partition RAM:
            Write RBAR/RASR for incoming partition's RAM
        - Else:
            Write RASR=0 (disabled)
     c. Enable MPU with PRIVDEFENA
     d. DSB + ISB barriers
  3. Restore incoming partition context
  4. Return to thread mode (unprivileged)
```

The target partition now has an additional MPU region granting access to the buffer at its physical address in kernel SRAM. If `writable=true`, the region has `AP_FULL_ACCESS` — the target can read and write the buffer directly.

### 4.4 Revoke flow (SYS_BUF_REVOKE)

```
Partition A (owner, pid=0)     Kernel SVC handler           DynamicStrategy
────────────────────────────────────────────────────────────────────────
SVC #35 (r1=slot, r2=1)
  ↓
                               1. Validate target < partitions.len()
                               2. Call buffers.unshare_from_partition(
                                    slot, pid=0, target=1, &strategy)
                               3. Inside unshare_from_partition:
                                  a. Validate state=BorrowedWrite{0} ✓
                                  b. Validate lent_to==Some(LendRecord{1,5,_}) ✓
                                  c. Call strategy.remove_window(5)
                                                                      Clear slots[1] = None
                                  d. Clear lent_to = None
                               4. Return 0 in r0
```

After revoke, the descriptor slot is cleared. On the next context switch to partition 1, PendSV programs R5 as disabled (RASR=0). Any access by partition 1 to the buffer address will fault.

### 4.5 Revoke timing considerations

**Critical invariant**: Between `remove_window()` and the next PendSV that reprograms the MPU, the stale MPU configuration may still be active if the target partition is currently running. This is safe because:

1. `SYS_BUF_REVOKE` runs in SVC handler context (higher priority than PendSV).
2. The SVC handler runs on behalf of the **owner** partition (not the target).
3. The target partition is not running during the owner's SVC — the scheduler is single-threaded.
4. When PendSV next switches to the target, `program_regions()` reads the updated descriptor table (with the cleared slot) and programs R5 as disabled.

If the kernel needs to revoke a window from a **currently running** partition (e.g. during `revoke_expired` in SysTick), the same safety holds: SysTick preempts the target, clears the descriptor, and PendSV reprograms before the target resumes.

---

## 5. DMA-Safe Handoff Protocol

### 5.1 Problem

DMA transfers require a buffer to be:
1. At a fixed physical address for the duration of the transfer.
2. Not modified by any software while the DMA engine is reading/writing.
3. Accessible to the DMA controller (which reads physical addresses, ignoring the MPU).

The MPU-enforced buffer pool satisfies (1) — buffers are statically allocated in kernel SRAM at fixed addresses. The challenge is enforcing (2) and (3) through the ownership protocol.

### 5.2 Protocol: DMA transmit (partition → peripheral)

```
DMA-agent partition (D)          Owner partition (A)
──────────────────────────────────────────────────
                                 1. SYS_BUF_ALLOC(Write) → slot S
                                 2. SYS_BUF_WRITE(S, data, len)
                                    (fill buffer with transmit data)
                                 3. SYS_BUF_LEND(S, D, flags=WRITABLE)
                                    (grant RW access to DMA-agent)

4. Read buffer address from
   shared state / IPC message
5. Program DMA source = buffer addr
6. Start DMA transfer
7. Wait for DMA complete interrupt
   (split-ISR bottom half)
8. SYS_BUF_REVOKE equivalent or
   IPC signal to owner
                                 9. SYS_BUF_REVOKE(S, D)
                                    (revoke window, reclaim buffer)
                                10. SYS_BUF_RELEASE(S)
                                    (return to pool)
```

### 5.3 Protocol: DMA receive (peripheral → partition)

```
DMA-agent partition (D)          Owner partition (A)
──────────────────────────────────────────────────
                                 1. SYS_BUF_ALLOC(Write) → slot S
                                 2. SYS_BUF_LEND(S, D, flags=WRITABLE)
                                    (grant RW so DMA-agent can program
                                     the buffer as DMA destination)

3. Read buffer address from
   shared state / IPC message
4. Program DMA destination = buffer addr
5. Start DMA transfer
6. Wait for DMA complete interrupt
7. Signal owner via IPC (event/queuing port)

                                 8. SYS_BUF_REVOKE(S, D)
                                 9. SYS_BUF_READ(S, dst, len)
                                    (copy received data to local memory)
                                10. SYS_BUF_RELEASE(S)
```

### 5.4 Protocol: Ownership transfer (zero-copy pipeline)

For high-throughput pipelines where copying is unacceptable:

```
Producer partition (P)           Consumer partition (C)
──────────────────────────────────────────────────
1. SYS_BUF_ALLOC(Write) → slot S
2. SYS_BUF_WRITE(S, data, len)
3. SYS_BUF_TRANSFER(S, C)
   (ownership moves to C; P can
    no longer access S)
                                 4. SYS_BUF_READ(S, dst, len)
                                    or access via lend-to-self MPU window
                                 5. SYS_BUF_RELEASE(S)
```

Transfer is zero-copy: no data movement, only metadata update. The producer loses all access after transfer; the consumer gains full ownership.

### 5.5 Memory ordering and cache coherency

On Cortex-M (M0/M0+/M3/M4/M7):
- M0-M4 have no data cache — no cache maintenance needed.
- M7 has optional D-cache. Buffer pool buffers use Normal memory attributes (S=0, C=0, B=0 in current `build_rasr` calls), which maps to **Non-cacheable** on M7 when the default WBWA attribute is not set. This is DMA-safe by default.
- The `DSB` + `ISB` barrier pair in `program_regions()` ensures MPU configuration changes are visible before the target partition executes.

For M7 targets where the system integrator enables caching, the existing `build_rasr` SCB bits should be reviewed. The DMA handoff protocol should document that buffer pool buffers are intended to be non-cacheable. If caching is desired for performance, the protocol must include explicit cache maintenance (clean before DMA-read, invalidate before CPU-read after DMA-write). This is deferred to the `cache-maintenance-syscalls.md` work item.

---

## 6. Interface Definitions

### 6.1 Syscall constants (kernel/src/syscall.rs)

```rust
#[cfg(feature = "dynamic-mpu")]
pub const SYS_BUF_TRANSFER: u32 = 36;

#[cfg(feature = "dynamic-mpu")]
pub const SYS_BUF_READ: u32 = 37;
```

Add corresponding `SyscallId` variants:

```rust
#[cfg(feature = "dynamic-mpu")]
BufferTransfer,

#[cfg(feature = "dynamic-mpu")]
BufferRead,
```

### 6.2 Lend flags (kernel/src/buffer_pool.rs)

```rust
/// Flags for SYS_BUF_LEND, packed into upper bits of r2.
pub mod lend_flags {
    /// Grant AP_FULL_ACCESS instead of AP_RO_RO to the target.
    pub const WRITABLE: u32 = 1 << 8;
}
```

### 6.3 Updated LendRecord

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LendRecord {
    pub target: u8,
    pub region_id: u8,
    pub writable: bool,
}
```

### 6.4 Updated share_with_partition signature

```rust
pub fn share_with_partition(
    &mut self,
    slot: usize,
    owner: u8,
    target: u8,
    writable: bool,
    strategy: &dyn MpuStrategy,
) -> Result<u8, BufferError>
```

### 6.5 New BufferPool methods

```rust
/// Transfer ownership of a borrowed slot to a different partition.
/// Borrow mode is preserved. Slot must not be lent.
pub fn transfer_ownership(
    &mut self,
    slot: usize,
    current_owner: u8,
    new_owner: u8,
) -> Result<(), BufferError>

/// Copy data from a buffer slot into a destination slice.
/// Caller must own the slot (any borrow mode).
pub fn read_from_slot(
    &self,
    slot: usize,
    owner: u8,
    dst: &mut [u8],
) -> Result<usize, BufferError>
```

### 6.6 SVC dispatch handlers (kernel/src/svc.rs)

```rust
#[cfg(feature = "dynamic-mpu")]
Some(SyscallId::BufferLend) => {
    let slot = frame.r1 as usize;
    let target_raw = (frame.r2 & 0xFF) as usize;
    let flags = (frame.r2 >> 8) & 0xFF;
    let writable = flags & 1 != 0;
    if target_raw >= self.partitions().len() {
        SvcError::InvalidPartition.to_u32()
    } else {
        let target = target_raw as u8;
        match self.buffers.share_with_partition(
            slot,
            self.current_partition,
            target,
            writable,
            &self.dynamic_strategy,
        ) {
            Ok(region_id) => region_id as u32,
            Err(_) => SvcError::InvalidResource.to_u32(),
        }
    }
}

#[cfg(feature = "dynamic-mpu")]
Some(SyscallId::BufferTransfer) => {
    let slot = frame.r1 as usize;
    let new_owner_raw = frame.r2 as usize;
    if new_owner_raw >= self.partitions().len() {
        SvcError::InvalidPartition.to_u32()
    } else {
        match self.buffers.transfer_ownership(
            slot,
            self.current_partition,
            new_owner_raw as u8,
        ) {
            Ok(()) => 0,
            Err(BufferError::SelfLend) => SvcError::OperationFailed.to_u32(),
            Err(_) => SvcError::InvalidResource.to_u32(),
        }
    }
}

#[cfg(feature = "dynamic-mpu")]
Some(SyscallId::BufferRead) => {
    validated_ptr_dynamic!(self, frame.r3, frame.r2 as usize, {
        let slot_idx = frame.r1 as usize;
        let len = frame.r2 as usize;
        let dst_ptr = frame.r3 as *mut u8;
        let dst = unsafe { core::slice::from_raw_parts_mut(dst_ptr, len) };
        match self.buffers.read_from_slot(slot_idx, self.current_partition, dst) {
            Ok(n) => n as u32,
            Err(_) => SvcError::InvalidResource.to_u32(),
        }
    })
}
```

### 6.7 User-space helper (rtos-traits or partition support crate)

```rust
/// Lend a buffer slot to another partition.
///
/// `writable`: if true, target gets read-write access; if false, read-only.
/// Returns the MPU region ID on success.
pub fn buf_lend(slot: u32, target: u8, writable: bool) -> Result<u8, SvcError> {
    let r2 = (target as u32) | if writable { 1 << 8 } else { 0 };
    let result = unsafe { syscall2(SYS_BUF_LEND, slot, r2) };
    if result >= 0x8000_0000 {
        Err(SvcError::from_u32(result))
    } else {
        Ok(result as u8)
    }
}

/// Transfer buffer ownership to another partition (zero-copy).
pub fn buf_transfer(slot: u32, new_owner: u8) -> Result<(), SvcError> {
    let result = unsafe { syscall2(SYS_BUF_TRANSFER, slot, new_owner as u32) };
    if result == 0 { Ok(()) } else { Err(SvcError::from_u32(result)) }
}

/// Read data from a buffer slot into local memory.
pub fn buf_read(slot: u32, dst: &mut [u8]) -> Result<usize, SvcError> {
    let result = unsafe {
        syscall3(SYS_BUF_READ, slot, dst.len() as u32, dst.as_mut_ptr() as u32)
    };
    if result >= 0x8000_0000 {
        Err(SvcError::from_u32(result))
    } else {
        Ok(result as usize)
    }
}
```

---

## 7. Key Design Decisions and Trade-offs

### 7.1 Bit-packing flags in r2 vs. new syscall numbers

**Decision**: Pack flags into upper bits of the `target_partition_id` argument.

**Rationale**: ARM SVC provides only r0-r3 for arguments (4 registers). The current `SYS_BUF_LEND` uses r0 (syscall ID), r1 (slot), r2 (target). r3 is free but reserved for future use (e.g. deadline). Packing flags into r2 avoids consuming r3 and avoids adding a new syscall number for a single boolean flag.

**Trade-off**: Partition IDs are limited to 8 bits (0-255). This is already the effective limit since `current_partition` is `u8`. The flag packing is safe.

### 7.2 Read-mode sharing

**Decision**: Allow `BorrowedRead` owners to share read-only with a target.

**Rationale**: This enables the common pattern where a kernel service prepares a read-only buffer and multiple consumers read it. The safety argument: if the owner has read-only access, granting another partition read-only access does not introduce new hazards. No writer exists, so no data races.

**Constraint**: `BorrowedRead` owners cannot grant writable access (`writable=true` returns error). Only `BorrowedWrite` owners can grant writable access.

### 7.3 Single target per lend

**Decision**: Keep the current single-target limit (one `LendRecord` per slot).

**Rationale**: Multi-target lending would require a variable-size list of `LendRecord`s per slot, adding heap-like complexity. The single-target model is sufficient for point-to-point IPC and DMA handoff. For broadcast patterns, use sampling ports.

**Future escape hatch**: If multi-target is needed, the slot could hold a small fixed-size array (e.g. `[Option<LendRecord>; 2]`). This is a minor structural change, not an architectural one.

### 7.4 Transfer vs. release-then-realloc

**Decision**: Provide an explicit `transfer_ownership` operation.

**Rationale**: Without transfer, a zero-copy pipeline requires: owner releases slot → target allocates same slot. This has a TOCTOU race: another partition might allocate the slot between release and re-alloc. Transfer is atomic — no window for interleaving.

**Constraint**: Transfer cannot happen while the slot is lent (must revoke first). This prevents the new owner from being surprised by an active MPU window to a third party.

### 7.5 SYS_BUF_READ vs. MPU direct access

**Decision**: Provide `SYS_BUF_READ` as a copy-based read path.

**Rationale**: Direct access via MPU window (`lend_to_partition`) requires consuming a dynamic MPU slot for the duration of the read. For short reads, the syscall overhead is less costly than the MPU slot pressure. `SYS_BUF_READ` copies data into the caller's own memory without installing an MPU window.

**When to use which**:
- `SYS_BUF_READ`: small, infrequent reads (< 256 bytes). No MPU slot consumed.
- `SYS_BUF_LEND` + direct read: large, streaming reads. One MPU slot consumed but zero-copy.

---

## 8. MPU Region Pressure Mitigation

With only 3 dynamic window slots (R5-R7), buffer lending competes with peripheral MMIO regions. The `mpu-bundling-design.md` documents this constraint.

### 8.1 Slot accounting for common scenarios

| Scenario | R5 | R6 | R7 | Free |
|----------|----|----|----|----|
| No peripherals, 3 buffer grants | buf0 | buf1 | buf2 | 0 |
| 1 peripheral, 2 buffer grants | MMIO | buf0 | buf1 | 0 |
| 2 peripherals, 1 buffer grant | MMIO0 | MMIO1 | buf0 | 0 |
| 2 peripherals, 0 buffer grants | MMIO0 | MMIO1 | — | 1 |

### 8.2 Guidance for implementers

1. **Prefer copy-based syscalls** (`SYS_BUF_WRITE` / `SYS_BUF_READ`) over MPU grants when the transfer size is small (< 64 bytes). This avoids consuming a dynamic slot.
2. **Revoke promptly**: After a DMA transfer completes, immediately revoke the grant to free the MPU slot.
3. **Pipeline over one slot**: For streaming DMA, reuse a single buffer slot by cycling: write → lend → DMA → revoke → write → lend → ...
4. **Static DMA allocator**: For high-throughput DMA that needs dedicated buffers, the `static-dma-allocator.md` work item will provide compile-time allocation outside the dynamic buffer pool.

---

## 9. Risks and Open Questions

### 9.1 Writable grant safety

**Risk**: Granting `AP_FULL_ACCESS` to a target means both owner and target can write simultaneously, creating a data race.

**Mitigation**: This is by design — the protocol (section 5) requires the owner to stop writing before granting writable access. The kernel cannot enforce this in general (it would require tracking instruction-level access). The safety contract is: the owner must not access the buffer after granting writable access until it revokes the grant. This matches POSIX shared memory semantics.

**Open question**: Should the kernel downgrade the owner's access to `AP_RO_RO` when granting writable access to a target? This would require a second MPU window for the owner, consuming two dynamic slots per lend. Likely not worth the cost.

### 9.2 Revoke while target is accessing

**Risk**: If the owner revokes a writable grant while the target is in the middle of a DMA write, the DMA engine (which ignores the MPU) continues writing to the buffer. The buffer data may be partially written.

**Mitigation**: DMA controllers use physical addresses and are not subject to MPU enforcement. The revoke removes the software (MPU) access but does not abort the hardware DMA. The protocol requires the DMA-agent to signal completion before the owner revokes. For fault tolerance, the `revoke_expired` deadline mechanism provides a hard timeout — if the DMA-agent fails to signal, the kernel auto-revokes after the deadline.

**Open question**: Should auto-revoke (`revoke_expired`) also abort associated DMA transfers? This requires DMA controller awareness in the kernel, which is a significant scope increase. Recommend deferring to `health-monitor-fault-recovery.md`.

### 9.3 SYS_BUF_TRANSFER atomicity

**Risk**: `transfer_ownership` changes the `BorrowState` owner field. If the new owner is currently executing a syscall that checks ownership (unlikely, since only one partition runs at a time on single-core), the kernel might see inconsistent state.

**Mitigation**: On single-core Cortex-M, SVC handlers are not preemptible by other SVC calls. The transfer is atomic with respect to other syscalls. No lock needed.

### 9.4 Buffer address discovery

**Open question**: After `SYS_BUF_LEND`, how does the target partition know the buffer's address? The syscall returns the MPU region ID, not the base address.

**Options**:
1. **Out-of-band IPC**: Owner sends the address via a queuing port or sampling port message alongside the lend. This is the simplest approach and doesn't require new syscalls.
2. **Return address in r1**: Extend `SYS_BUF_LEND` to also return the buffer base address in r1 (r0 = region ID, r1 = base address). This requires modifying the SVC frame write to set two return registers.
3. **New introspection syscall**: `SYS_BUF_INFO(slot) → (base, size, state)`. Covered by `introspection-syscalls.md`.

**Recommendation**: Option (2) — return base address in r1. It's a one-line change to the SVC handler (`frame.r1 = base_addr`) and eliminates the need for a separate IPC message in the common case. Option (1) is the fallback if we want to keep the SVC return value ABI simple.

### 9.5 Multi-lend (one buffer to multiple targets)

**Open question**: Should a buffer be lendable to multiple targets simultaneously?

**Current answer**: No. Keep single-target per slot. Multi-target requires either an array of `LendRecord`s (memory overhead per slot) or multiple `add_window` calls consuming multiple MPU slots for one buffer. The single-target model matches the point-to-point DMA pattern.

### 9.6 Interaction with partition scheduling

**Open question**: Should lending block the owner's partition time window? If the owner lends a writable buffer and continues executing, it might modify the buffer while the target reads.

**Answer**: No scheduling interaction. The buffer pool is a voluntary-cooperation primitive, like shared memory in POSIX. The owner is responsible for not modifying the buffer after lending. If stronger guarantees are needed, use the transfer protocol (section 5.4) which removes owner access entirely.

---

## 10. Implementation Order

1. **Add `writable` parameter to `share_with_partition`** and update `LendRecord`. Update existing tests. (~50 lines changed in `buffer_pool.rs`)

2. **Update `SYS_BUF_LEND` handler** to extract flags from r2 and pass `writable` to `share_with_partition`. Update handler tests. (~10 lines changed in `svc.rs`)

3. **Allow `BorrowedRead` sharing** in `share_with_partition` when `writable=false`. Add tests for the new BorrowedRead→share path. (~15 lines changed in `buffer_pool.rs`)

4. **Add `transfer_ownership`** to `BufferPool`. Add `SYS_BUF_TRANSFER` constant, `SyscallId::BufferTransfer`, and SVC handler. (~40 lines new in `buffer_pool.rs`, ~20 lines new in `svc.rs`, ~5 lines new in `syscall.rs`)

5. **Add `read_from_slot`** to `BufferPool`. Add `SYS_BUF_READ` constant, `SyscallId::BufferRead`, and SVC handler with `validated_ptr_dynamic!`. (~30 lines new in `buffer_pool.rs`, ~20 lines new in `svc.rs`, ~5 lines new in `syscall.rs`)

6. **Add QEMU integration test** demonstrating the DMA-safe handoff protocol (transmit pattern from section 5.2). (~100 lines new example) — **Done**: `kernel/examples/buf_lend_rw_test.rs` exercises SYS_BUF_ALLOC → SYS_BUF_WRITE → SYS_BUF_LEND (writable) → write_volatile by P1 → SYS_BUF_REVOKE → SYS_BUF_READ.

7. **Update user-space helpers** in the traits crate with `buf_lend`, `buf_transfer`, `buf_read` wrappers. (~30 lines)

---

## 11. Relationship to Other Work Items

| Work item | Relationship |
|-----------|-------------|
| `usb-highspeed-dma.md` | **Consumer** — depends on writable grants and DMA handoff protocol |
| `static-dma-allocator.md` | **Parallel** — provides compile-time DMA buffers for dedicated peripherals; buffer pool serves dynamic/shared DMA |
| `split-isr-interrupt-routing.md` | **Dependency for DMA** — DMA completion interrupts routed via split-ISR |
| `external-partition-memory.md` | **Orthogonal** — external memory partitions may use buffer pool for kernel↔external data exchange |
| `cache-maintenance-syscalls.md` | **Future dependency** — M7 targets need cache clean/invalidate around DMA |
| `health-monitor-fault-recovery.md` | **Future consumer** — fault recovery may force-revoke buffer grants |
| `introspection-syscalls.md` | **Future consumer** — SYS_BUF_INFO for runtime buffer state queries |
| `ipc-middleware-rpc.md` | **Consumer** — zero-copy RPC layer built on buffer pool lend/transfer |

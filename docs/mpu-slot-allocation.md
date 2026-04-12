# MPU Slot Allocation

## 1. Slot Map

ARMv7-M provides 8 MPU regions (R0–R7). This kernel assigns them as follows:

| Region | Purpose                  | Reprogrammed on context switch? | Owner            |
|--------|--------------------------|---------------------------------|------------------|
| R0     | Background deny-all      | No (static at boot)             | Kernel           |
| R1     | Partition code (flash)   | Yes (base regions from cache)   | Current partition|
| R2     | Partition data (RAM)     | Yes (base regions from cache)   | Current partition|
| R3     | Stack guard (32 B)       | Yes (base regions from cache)   | Current partition|
| R4     | Dynamic slot 0           | Yes (dynamic strategy)          | Dynamic strategy |
| R5     | Dynamic slot 1           | Yes (dynamic strategy)          | Dynamic strategy |
| R6     | Dynamic slot 2           | Yes (dynamic strategy)          | Dynamic strategy |
| R7     | Dynamic slot 3           | Yes (dynamic strategy)          | Dynamic strategy |

**R0–R3** are the *static base regions*, precomputed at boot into each PCB's
`cached_base_regions` array and written to hardware by `write_base_regions()`.

**R4–R7** are the *dynamic slots* managed by `DynamicStrategy`
(`DYNAMIC_REGION_BASE = 4`, `DYNAMIC_SLOT_COUNT = 4` in `mpu_strategy.rs`).
These slots are reprogrammed on every context switch via
`partition_region_values()`.

**Why R2 (data) is duplicated into a dynamic slot:** The static base regions
always write R2 with the partition's data (RAM) region. The `DynamicStrategy`
*also* places the same data region into one of R4–R7 (via
`cached_dynamic_region()`, which returns `cached_base_regions[2]`). This
duplication is intentional: the dynamic strategy must account for the RAM
region in its slot arithmetic so peripherals and buffer windows compete for the
remaining dynamic slots. The higher-numbered dynamic copy takes priority per
ARMv7-M region-number precedence rules, but both regions cover the same
address range with the same permissions, so the overlap is benign.

## 2. Peripheral Ceiling

`MAX_PERIPHERAL_REGIONS` is defined as **3** in `partition.rs`. This is not an
arbitrary policy choice — it is the hardware ceiling derived from slot
arithmetic. Even though R2 already provides partition data (RAM) in the static
base regions, the `DynamicStrategy` duplicates that region into one dynamic
slot (see Section 1) so it can manage all competing allocations uniformly:

```
Total MPU regions:           8
Static base regions (R0–R3): 4
Dynamic slots (R4–R7):       4   (DYNAMIC_SLOT_COUNT)
RAM slot (duplicate of R2):  1   (RAM_SLOT_COUNT)
                             ─
Available for peripherals:   3   (DYNAMIC_SLOT_COUNT - RAM_SLOT_COUNT)
```

A compile-time assertion in `mpu_strategy.rs` enforces this relationship:

```rust
const _: () = assert!(
    MAX_PERIPHERAL_REGIONS == DYNAMIC_SLOT_COUNT - RAM_SLOT_COUNT,
);
```

### Headroom on other cores

- **Cortex-M7** (ARMv7-M): 16 MPU regions → 12 dynamic slots → up to 11
  peripheral regions (after 1 RAM slot). In practice the kernel would likely
  allocate extra slots for additional RAM windows or code overlays.

- **Cortex-M33** (ARMv8-M): 16 MPU regions + separate SAU (Security
  Attribution Unit) regions. The SAU handles TrustZone partitioning
  independently, leaving all 16 MPU regions available for the same
  static/dynamic split. Dynamic slot count could rise to 12.

Neither target is supported today (QEMU lm3s6965evb, Cortex-M3, 8 regions
only), but the constant-based design means porting requires changing
`DYNAMIC_SLOT_COUNT` and re-running the static assertions.

## 3. Dynamic Window Trade-off

Within the 4 dynamic slots, peripheral reservations and the mandatory RAM slot
compete with `sys_buf_lend` zero-copy buffer windows. The trade-off:

| Peripheral regions | RAM slot | Free for buf_lend windows | Notes                          |
|--------------------|----------|---------------------------|--------------------------------|
| 0                  | 1        | 3                         | Maximum zero-copy capacity     |
| 1                  | 1        | 2                         | e.g. UART-only partition       |
| 2                  | 1        | 1                         | e.g. UART + timer              |
| 3                  | 1        | 0                         | No zero-copy; all slots used   |

The `BOOT_WIRE_LIMIT` constant (= `DYNAMIC_SLOT_COUNT - RAM_SLOT_COUNT` = 3)
caps the number of peripherals `wire_boot_peripherals` may program.

### Combining adjacent peripherals

A partition that needs both 3 peripheral regions **and** zero-copy IPC can
recover a slot by combining adjacent peripherals into a single larger MPU
region. For example, if two peripherals occupy consecutive 4 KiB pages, a
single 8 KiB region covers both — reducing the peripheral count from 2 to 1
and freeing one slot for a buffer window. This requires the peripherals to be
naturally aligned to the combined power-of-two size (an ARMv7-M MPU
constraint).

## 4. Context Switch Behavior

On every context switch, the PendSV handler reprograms **all four dynamic
slots** (R4–R7) by calling `partition_region_values()` on the
`DynamicStrategy`. This function computes the complete `[(RBAR, RASR);
DYNAMIC_SLOT_COUNT]` output for the incoming partition, drawing from three
sources:

1. **Peripheral cache** — per-partition pre-computed `(RBAR, RASR)` pairs
   stored in `PeripheralCache`. Slots `0..reserved` are overridden from this
   cache so each partition sees exactly its own peripheral MMIO regions.

2. **RAM slot** — the partition's private data region is placed at slot index
   `reserved` (i.e. immediately after the peripheral slots).

3. **Shared windows** — any `buf_lend` windows occupying the remaining slots
   are carried forward from the global `slots[]` array, with visibility
   filtering so a partition cannot see another partition's peripheral-reserved
   slots.

### Stale-region zeroing requirement

When partition P0 has `reserved = 3` peripheral slots and partition P1 has
`reserved = 1`, a context switch from P0 to P1 leaves P0's peripheral data in
hardware regions R5 and R6 unless those slots are explicitly cleared. Stale
MMIO permissions from a previous partition are a **security violation** — the
incoming partition must never inherit access to peripherals it does not own.

The fix (Bug 40-koala, subtask 112) adds an explicit zeroing pass in
`partition_region_values()`:

```rust
// Disable stale peripheral slots between this partition's
// reserved count and the maximum reserved across all partitions.
let max_reserved = pr.iter().copied().max().unwrap_or(0);
for idx in reserved..=max_reserved {
    if let Some(dst) = out.get_mut(idx) {
        let region = DYNAMIC_REGION_BASE as u32 + idx as u32;
        *dst = disabled_pair(region as u8);
    }
}
```

Every slot from index `reserved` through `max_reserved` is set to
`disabled_pair()` — which writes `RASR = 0`, disabling the MPU region — before
the RAM slot overwrites index `reserved` with the partition's own data region.
This guarantees that no peripheral RASR bits survive a context switch to a
partition with fewer peripheral reservations.

### Debug verification

In debug builds (`cfg(debug_assertions)`), `debug_assert_no_stale_regions()`
runs after `partition_region_values()` computes its output. It checks two
invariants:

- **Peripheral-reserved slots** (`idx < reserved`) with a cached peripheral
  must have `RASR != 0` (the peripheral is actively mapped).
- **Slots beyond the reserved range** (`idx > reserved`, excluding the RAM
  slot) must have `RASR == 0` unless a shared window descriptor exists for
  that slot — otherwise the slot contains stale data from a previous partition.

## 5. Porting Guidance

### Checking MPU region count

Before porting to a new Cortex-M target, read the `MPU_TYPE` register's
`DREGION` field (bits 15:8) to determine how many MPU regions the hardware
provides:

```rust
let mpu_type = core::ptr::read_volatile(0xE000_ED90 as *const u32);
let dregion = (mpu_type >> 8) & 0xFF;
```

| Core        | Architecture | Typical DREGION | Total regions |
|-------------|-------------|-----------------|---------------|
| Cortex-M3   | ARMv7-M     | 8               | 8             |
| Cortex-M4   | ARMv7-M     | 8               | 8             |
| Cortex-M7   | ARMv7-M     | 8 or 16         | 8 or 16       |
| Cortex-M33  | ARMv8-M     | 8 or 16         | 8 or 16       |

### 8-region strategy (M3/M4)

The current kernel targets 8-region devices (QEMU lm3s6965evb, Cortex-M3).
With 4 static base regions (R0–R3) and `DYNAMIC_SLOT_COUNT = 4`, the ceiling
is 3 peripheral regions and 0 buffer windows, or 0 peripherals and 3 buffer
windows. This is the tightest configuration and the one all current tests
validate.

### 16-region strategy (M7/M33)

On a 16-region device, the static base regions remain at R0–R3 and
`DYNAMIC_SLOT_COUNT` rises to 12. To extend:

1. Change `DYNAMIC_SLOT_COUNT` in `mpu_strategy.rs` from 4 to 12.
2. Update `MAX_PERIPHERAL_REGIONS` in `partition.rs` to the new
   `DYNAMIC_SLOT_COUNT - RAM_SLOT_COUNT` (= 11). The compile-time assertion
   will enforce consistency.
3. Widen `PeripheralCache` and all `[(u32, u32); MAX_PERIPHERAL_REGIONS]`
   arrays — these are derived from the constant so they resize automatically.
4. Extend `BOOT_WIRE_LIMIT` — also derived from the constants.
5. Re-run the full test suite. The `debug_assert_no_stale_regions()` check
   will validate the wider slot range automatically.

The constant-driven design means most changes propagate from adjusting two
numbers. The primary porting risk is ensuring the target's MPU region alignment
and size constraints match ARMv7-M rules (power-of-two size >= 32, base
aligned to size). ARMv8-M relaxes this to arbitrary base/limit pairs, which
would require a different `build_rbar`/`build_rasr` implementation.

### Peripheral cache sizing

The peripheral cache was originally 2 entries per partition (matching an older
`PERIPHERAL_RESERVED_SLOTS = 2` constant). Bug 41-llama (subtask 111) expanded
it to 3 entries to match `MAX_PERIPHERAL_REGIONS`, enabling partitions to use
all 3 available peripheral slots. The old 2-entry cache silently dropped the
third peripheral, causing it to be unmapped after the first context switch. On
a 16-region device, the cache would grow to `MAX_PERIPHERAL_REGIONS` entries
(up to 11), but the per-partition memory cost remains small since each entry is
only 8 bytes (one `(u32, u32)` pair).

## References

- [MPU Bundling Design](../mpu-bundling-design.md) — combining adjacent
  peripherals into single MPU regions to recover dynamic slots.
- [Architecture Overview](../../docs/architecture.md) — system-level architecture
  including memory ownership model and partition lifecycle.

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

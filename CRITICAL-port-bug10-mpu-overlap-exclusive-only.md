# CRITICAL Port Bug #10: MPU Overlap Check Rejects Legitimate Shared-Memory IPC

Status: **Complete** (2026-03-01)

## Bug Summary

`assert_no_overlapping_mpu_regions` rejected legitimate Data-vs-Data and
Data-vs-Stack/Peripheral overlaps needed for shared-memory IPC, causing
false-positive panics during `Kernel::new()`. Any kernel configuration
with two partitions sharing an SRAM window for IPC would hit the overlap
assertion and panic at boot, even though the overlap was intentional and
correct.

### Symptoms

- Kernel panics at startup with `"invariant violation: partition N region
  [...) overlaps partition M region [...]"` when partitions share data
  regions for IPC.
- Shared-memory IPC configurations that are architecturally valid are
  rejected, preventing multi-partition designs that rely on overlapping
  SRAM windows.
- False positive: the panic fires on safe, intended overlaps — not on
  actual memory-safety violations.

## Root Cause

The original `assert_no_overlapping_mpu_regions` compared **every**
accessible region of partition *i* against **every** accessible region of
partition *j*, with no distinction between region types. The check used
`accessible_static_regions()` which returns data, stack, and peripheral
regions combined. This meant:

- **Data-vs-Data** overlaps were rejected (needed for shared-memory IPC).
- **Data-vs-Stack** overlaps were rejected (a partition's data window may
  legitimately cover another partition's stack address range at the MPU
  level).
- **Data-vs-Peripheral** overlaps were rejected (same reasoning).

The only overlaps that actually represent ownership conflicts are
**exclusive-vs-exclusive**: Stack-vs-Stack and Peripheral-vs-Peripheral
(and Stack-vs-Peripheral), since these regions must be owned by exactly
one partition.

## Fix (commit 2a65758)

Added an `exclusive_static_regions()` method on `PartitionControlBlock`
that returns only stack + peripheral regions (excluding the data/MPU
region). The overlap check now compares only exclusive-vs-exclusive
regions across partition pairs.

### What is now permitted

| Overlap type             | Verdict   | Rationale                          |
|--------------------------|-----------|------------------------------------|
| Data-vs-Data             | Allowed   | Shared-memory IPC windows          |
| Data-vs-Stack            | Allowed   | Data region is not exclusive       |
| Data-vs-Peripheral       | Allowed   | Data region is not exclusive       |

### What is still rejected

| Overlap type             | Verdict   | Rationale                          |
|--------------------------|-----------|------------------------------------|
| Stack-vs-Stack           | Rejected  | Stacks must be partition-exclusive |
| Peripheral-vs-Peripheral | Rejected  | Peripherals must be exclusive      |
| Stack-vs-Peripheral      | Rejected  | Both are exclusive regions         |

### Key changes

1. **`exclusive_static_regions()`** (`partition.rs:208–221`) — returns a
   `Vec<(u32, u32), 4>` containing only the stack region (if non-zero
   size) and peripheral regions (filtering zero-size entries). The data
   region (`mpu_region`) is excluded.

2. **`assert_no_overlapping_mpu_regions()`** (`invariants.rs:195–222`) —
   now calls `exclusive_static_regions()` on both partitions in each pair
   and only checks those regions for overlap. The previous two-pass
   approach (exclusive_i vs accessible_j, then accessible_i vs
   exclusive_j) was replaced with a single pass comparing exclusive_i vs
   exclusive_j.

## Affected Code Paths

- `kernel/src/invariants.rs` — `assert_no_overlapping_mpu_regions()`
  (lines 195–222): overlap detection logic.
- `kernel/src/partition.rs` — `exclusive_static_regions()` (lines
  208–221): new method providing the exclusive-region subset.

## Regression Tests

### `kernel/src/invariants.rs` (9 overlap tests)

| Test name                                  | Behavior                                         |
|--------------------------------------------|--------------------------------------------------|
| `no_overlap_empty_and_single`              | Passes with 0 or 1 partition                     |
| `no_overlap_disjoint_partitions`           | Passes when all regions are disjoint             |
| `no_overlap_adjacent_regions`              | Passes when regions are adjacent but not overlap  |
| `shared_data_regions_not_checked`          | Data-vs-Data overlap accepted                    |
| `data_overlapping_stack_allowed`           | Data-vs-Stack overlap accepted                   |
| `data_overlapping_peripheral_allowed`      | Data-vs-Peripheral overlap accepted              |
| `three_partitions_data_stack_overlap_allowed` | Data-vs-Stack across 3 partitions accepted    |
| `overlap_stack_stack`                      | `#[should_panic]` — Stack-vs-Stack rejected      |
| `overlap_peripheral`                       | `#[should_panic]` — Peripheral-vs-Peripheral rejected |

### `kernel/src/partition.rs` (6 `exclusive_static_regions` tests)

| Test name                                              | Behavior                                         |
|--------------------------------------------------------|--------------------------------------------------|
| `exclusive_static_regions_returns_stack_and_peripheral_not_data` | Returns stack + peripheral, excludes data |
| `exclusive_static_regions_zero_stack_returns_peripherals_only`   | Zero-size stack omitted                  |
| `exclusive_static_regions_no_peripherals_returns_stack_only`     | No peripherals — stack only              |
| `exclusive_static_regions_filters_zero_size_peripherals`         | Zero-size peripheral entries filtered    |
| `exclusive_static_regions_zero_stack_zero_peripherals_returns_empty` | Both zero — empty result            |
| `exclusive_static_regions_reflects_fix_stack_region`             | Reflects `fix_stack_region()` updates    |

## Resolution Checklist

- [x] Root cause identified (all-region comparison without type distinction)
- [x] Fix implemented (`exclusive_static_regions()` + exclusive-vs-exclusive check)
- [x] Commit landed (2a65758)
- [x] Overlap invariant tests updated (9 tests in `invariants.rs`)
- [x] `exclusive_static_regions` unit tests added (6 tests in `partition.rs`)
- [x] Data-vs-Data, Data-vs-Stack, and Data-vs-Peripheral overlaps verified as permitted
- [x] Stack-vs-Stack and Peripheral-vs-Peripheral overlaps verified as rejected

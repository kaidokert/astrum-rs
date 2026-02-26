# CRITICAL: Port Bug 07 — Shared MPU Data Region Triggers False Overlap Panic

**Status: RESOLVED**

**Severity:** CRITICAL — false-positive invariant panic blocks kernel boot when
two partitions share a data region for shared-memory IPC.

---

## Symptom

`assert_no_overlapping_mpu_regions` panicked during `Kernel::new()` when two
partitions shared a data region for shared-memory IPC.  The invariant treated
all region overlaps equally, so a legitimate Data-vs-Data overlap (required for
IPC) was indistinguishable from a dangerous Stack-vs-Stack or
Stack-vs-Peripheral overlap.

**Concrete scenario:** P0 and P1 each have a data region mapped at
`[0x2000_0000, 0x2000_1000)` for shared-memory communication.  At boot,
`Kernel::new()` calls `assert_no_overlapping_mpu_regions`, which compares every
accessible region of P0 against every accessible region of P1.  The shared data
region matches — panic fires, kernel refuses to start.

---

## Root Cause

The original invariant compared every accessible region of partition i against
every accessible region of partition j, making no distinction between region
categories.  Data regions may legitimately overlap across partitions (shared
memory for IPC), while stack and peripheral regions must be exclusive to each
partition.  The flat comparison had no way to express this asymmetry.

---

## Fix

Two changes implement an asymmetric overlap check that permits Data-vs-Data
overlaps while catching all other overlap combinations.

### 1. `exclusive_static_regions()` — partition.rs:208

Added a new method to `PartitionControlBlock` that returns only the regions
that must be exclusive to each partition: the stack region and peripheral
regions.  Data regions are excluded because they may be shared.

```rust
pub fn exclusive_static_regions(&self) -> Vec<(u32, u32), 4> {
    let mut regions = Vec::new();
    if self.stack_size > 0 {
        regions.push((self.stack_base, self.stack_size));
    }
    for pr in self.peripheral_regions.iter() {
        if pr.size() > 0 {
            regions.push((pr.base(), pr.size()));
        }
    }
    regions
}
```

### 2. Asymmetric invariant check — invariants.rs:195

Updated `assert_no_overlapping_mpu_regions` to perform two asymmetric checks
per partition pair instead of one symmetric check:

- **exclusive_i vs accessible_j** — catches stack/peripheral of partition i
  overlapping any region (data, stack, peripheral) of partition j.
- **accessible_i vs exclusive_j** — catches any region of partition i
  overlapping stack/peripheral of partition j.

The only overlap combination not checked is Data-vs-Data, which is the
legitimate shared-memory case.

---

## Affected Code Paths

| # | File:Line | Function / Method | Change |
|---|-----------|-------------------|--------|
| 1 | partition.rs:208 | `PartitionControlBlock::exclusive_static_regions()` | New method returning stack + peripheral regions only |
| 2 | invariants.rs:195 | `assert_no_overlapping_mpu_regions()` | Asymmetric check using exclusive vs accessible regions |
| 3 | svc.rs:1112 | `Kernel::new()` call site | No change — calls `assert_no_overlapping_mpu_regions` which now uses the updated logic |

---

## Regression Test Coverage

### invariants.rs — MPU overlap tests

| # | Test Function | Scenario |
|---|---------------|----------|
| 1 | `no_overlap_adjacent_regions` | Adjacent (non-overlapping) regions pass |
| 2 | `no_overlap_disjoint_partitions` | Fully disjoint partitions pass |
| 3 | `no_overlap_empty_and_single` | Empty partition list and single partition pass |
| 4 | `no_overlap_three_disjoint_partitions` | Three partitions with disjoint regions pass |
| 5 | `overlap_data_peripheral` | Data region overlapping peripheral region panics |
| 6 | `overlap_data_stack` | Data region overlapping stack region panics |
| 7 | `overlap_peripheral` | Peripheral-vs-peripheral overlap panics |
| 8 | `overlap_stack_stack` | Stack-vs-stack overlap panics |
| 9 | `kernel_invariants_allows_shared_data_regions` | Two partitions sharing a data region pass (Bug 07 core scenario) |
| 10 | `shared_data_regions_not_checked` | Data-vs-Data overlap explicitly permitted |
| 11 | `three_partitions_data_stack_overlap_rejected` | Three partitions: shared data allowed but data-stack overlap rejected |
| 12 | `three_partitions_shared_data_allowed` | Three partitions all sharing a data region pass |

### partition.rs — exclusive_static_regions tests

| # | Test Function | Scenario |
|---|---------------|----------|
| 1 | `exclusive_static_regions_returns_stack_and_peripheral_not_data` | Returns stack + peripherals, excludes data region |
| 2 | `exclusive_static_regions_no_peripherals_returns_stack_only` | No peripherals configured — returns stack only |
| 3 | `exclusive_static_regions_zero_stack_returns_peripherals_only` | Zero-size stack — returns peripherals only |
| 4 | `exclusive_static_regions_zero_stack_zero_peripherals_returns_empty` | No stack, no peripherals — returns empty |
| 5 | `exclusive_static_regions_filters_zero_size_peripherals` | Zero-size peripheral entries are filtered out |
| 6 | `exclusive_static_regions_reflects_fix_stack_region` | Results update after `fix_stack_region()` call |

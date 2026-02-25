# CRITICAL: PCB MPU Region Stale Address After Box Relocation

## Bug Summary

The PCB `mpu_region` field retained a stale pre-move stack address after the
Kernel struct was Box-allocated to the heap. This caused the MPU to be
programmed with a base address pointing to invalidated stack memory instead
of the actual heap-resident partition stack buffer.

**Symptom:** On 64-bit hosts (test harness), `mpu_region.base()` pointed to
the original `PartitionConfig` address rather than the relocated
`PartitionCore` stack buffer. On a real Cortex-M target, this would program
the MPU data region to protect the wrong memory, leaving the actual stack
unprotected.

## Root Cause

`Kernel::new()` in `svc.rs` copies `c.mpu_region` from the user-supplied
`PartitionConfig` directly into the PCB (line 1048). For sentinel partitions
(size==0), the base address is meaningless at config time. After the Kernel
is constructed on the stack and then moved (e.g., via `Box`), all internal
`PartitionCore` stack buffer addresses change. The PCB's `mpu_region.base()`
is never updated to reflect the new location, leaving a stale address that
no longer corresponds to any valid stack memory.

**Key location:** `svc.rs:1048` — `let mpu_region = c.mpu_region;` copies
the config value without post-move correction.

## Fix: Sentinel-Guarded Post-Move Fixup

The fix adds a post-move fixup in `boot()` that selectively updates sentinel
partitions while preserving user-configured MPU regions:

1. **`fix_mpu_data_region_if_sentinel()`** (`boot.rs:218-228`) — Guards the
   fixup: only applies when `mpu_region.size() == 0` (sentinel marker).
   User-configured partitions (size > 0) are left untouched.

2. **`fix_mpu_data_region()`** (`partition.rs:239-242`) — Patches the base
   address to the actual runtime stack buffer address. Preserves size and
   permissions.

3. **Boot loop** (`boot.rs:314-321`) — After stack relocation, iterates all
   partitions and calls `fix_mpu_data_region_if_sentinel()` with the real
   `core_stack` base for each.

4. **`Kernel::new()`** (`svc.rs:1048`) — Simplified to passthrough: copies
   `c.mpu_region` as-is, deferring fixup to boot time.

## Affected Code Paths

| File | Lines | Description |
|------|-------|-------------|
| `boot.rs` | 290-321 | Post-move fixup loop in `boot()` |
| `boot.rs` | 218-228 | `fix_mpu_data_region_if_sentinel()` helper |
| `partition.rs` | 239-242 | `fix_mpu_data_region()` base-patching method |
| `svc.rs` | 1048 | `Kernel::new()` config passthrough |
| `svc.rs` | 2184-2191 | `Kernel::fix_mpu_data_region()` public wrapper |

## Resolution Checklist

- [x] Identified root cause: pre-move base derivation in `Kernel::new()`
- [x] Implemented `fix_mpu_data_region()` on `PartitionControlBlock`
- [x] Extracted `fix_mpu_data_region_if_sentinel()` guard in `boot.rs`
- [x] Boot loop applies fixup only to sentinel partitions (size==0)
- [x] User-configured partitions (size > 0) preserve original base
- [x] Peripheral regions unaffected by fixup
- [x] All regression tests passing

## Regression Tests

### `test_harness.rs` (7 tests)

| Test | Line | Purpose |
|------|------|---------|
| `pcb_mpu_region_base_matches_core_stack_base` | 1393 | Post-move base equals core stack base |
| `fix_mpu_data_region_returns_false_for_out_of_bounds` | 1438 | Out-of-bounds index returns false |
| `fix_mpu_data_region_returns_true_and_updates_base` | 1456 | Valid index updates base correctly |
| `harness_build_applies_fix_mpu_data_region` | 1491 | Integration: build_kernel applies fixup |
| `mpu_base_stable_across_schedule_cycles` | 2555 | MPU base stable across 60 ticks |
| `pcb_mpu_region_not_stale_pre_move_address` | 2625 | Base != config-time address after move |
| `selective_sentinel_fixup_mixed_config` | 2721 | Sentinel updated, user-configured untouched |

### `partition.rs` (2 tests)

| Test | Line | Purpose |
|------|------|---------|
| `fix_mpu_data_region_preserves_peripheral_regions` | 920 | Peripheral regions untouched by fixup |
| `fix_mpu_data_region_user_configured_preserves_all_fields` | 956 | Non-sentinel fields fully preserved |

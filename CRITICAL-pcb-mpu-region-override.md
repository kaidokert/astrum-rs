# CRITICAL: PCB MPU Region Stale Address After Box Relocation

## Bug Summary

The PCB `mpu_region` field retained a stale pre-move stack address after the
Kernel struct was `Box`-allocated to the heap, causing the MPU to be programmed
with the wrong base. On a real Cortex-M target, this would leave the actual
partition stack unprotected while protecting invalidated memory, leading to
silent corruption or MemManage faults on context switch.

**Symptom:** `pcb.mpu_region().base()` pointed to the original stack-local
`PartitionConfig` address rather than the heap-relocated `PartitionCore`
stack buffer.

## Root Cause

`Kernel::new()` copies `c.mpu_region` from user-supplied `PartitionConfig`
directly into the PCB (`svc.rs:1127` -- `let mpu_region = c.mpu_region`).
For sentinel partitions (size==0), the base address is derived from the
internal stack pointer before struct placement. After `Kernel::new()` returns
and the struct is moved into `UNIFIED_KERNEL_STORAGE` (or heap via `Box`),
all internal `PartitionCore` stack buffer addresses change. The PCB's
`mpu_region.base()` is never updated, leaving a stale pre-move address.

## Fix: Sentinel-Guarded Post-Move Fixup

The boot sequence applies a post-move fixup that calls `fix_mpu_data_region()`
**only** for sentinel (size==0) partitions. User-configured partitions (size>0)
keep their original base untouched.

1. **`fix_mpu_data_region_if_sentinel()`** (`boot.rs:218-228`) -- Sentinel
   guard: applies fixup only when `mpu_region.size() == 0`.

2. **`fix_mpu_data_region()`** (`partition.rs:260-263`) -- Patches the base
   address to the actual runtime stack buffer address; preserves size and
   permissions.

3. **Boot fixup loop** (`boot.rs:314-321`) -- After stack relocation, calls
   `fix_mpu_data_region_if_sentinel()` with the real `core_stack` base.

4. **`Kernel::new()`** (`svc.rs:1127`) -- Simplified to passthrough copy;
   fixup deferred to boot time.

5. **Invariant check** (`boot.rs:325-336`) -- `assert_pcb_addresses_in_storage()`
   validates all PCB addresses fall within live storage post-fixup.

## Affected Code Paths

| File | Lines | Description |
|------|-------|-------------|
| `kernel/src/boot.rs` | 218-228 | `fix_mpu_data_region_if_sentinel()` helper |
| `kernel/src/boot.rs` | 290-303 | Stack relocation and AAPCS alignment |
| `kernel/src/boot.rs` | 314-321 | Post-move fixup loop in `boot()` |
| `kernel/src/boot.rs` | 325-336 | `assert_pcb_addresses_in_storage()` invariant |
| `kernel/src/partition.rs` | 260-263 | `fix_mpu_data_region()` base-patching method |
| `kernel/src/svc.rs` | 1127 | `Kernel::new()` passthrough copy |
| `kernel/src/svc.rs` | 2256-2263 | `Kernel::fix_mpu_data_region()` public wrapper |

## Resolution Checklist

- [x] Identified root cause: pre-move base derivation in `Kernel::new()`
- [x] Implemented `fix_mpu_data_region()` on `PartitionControlBlock`
- [x] Extracted `fix_mpu_data_region_if_sentinel()` guard in `boot.rs`
- [x] Boot loop applies fixup only to sentinel partitions (size==0)
- [x] User-configured partitions (size > 0) preserve original base
- [x] Peripheral regions unaffected by fixup
- [x] Post-move invariant check wired into boot sequence
- [x] `Kernel::new()` simplified to passthrough (deferred fixup)
- [x] All regression tests passing

## Regression Tests

### `test_harness.rs` (7 tests)

| Test | Line | Purpose |
|------|------|---------|
| `pcb_mpu_region_base_matches_core_stack_base` | 1381 | Post-move base equals core stack base |
| `fix_mpu_data_region_returns_false_for_out_of_bounds` | 1426 | Out-of-bounds index returns false |
| `fix_mpu_data_region_returns_true_and_updates_base` | 1444 | Valid index updates base correctly |
| `harness_build_applies_fix_mpu_data_region` | 1479 | Integration: build_kernel applies fixup |
| `mpu_base_stable_across_schedule_cycles` | 2543 | MPU base stable across 60 ticks |
| `pcb_mpu_region_not_stale_pre_move_address` | 2613 | Base != config-time address after move |
| `selective_sentinel_fixup_mixed_config` | 2709 | Sentinel updated, user-configured untouched |

### `partition.rs` (2 tests)

| Test | Line | Purpose |
|------|------|---------|
| `fix_mpu_data_region_preserves_peripheral_regions` | 979 | Peripheral regions untouched by fixup |
| `fix_mpu_data_region_user_configured_preserves_all_fields` | 1015 | Non-sentinel fields fully preserved |

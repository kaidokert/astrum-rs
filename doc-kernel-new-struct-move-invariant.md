# Kernel Struct-Move Invariant

## Overview

The `Kernel` struct is constructed on the stack inside `Kernel::new()` and then
moved into the static `UNIFIED_KERNEL_STORAGE` via `core::ptr::write()`.
Because the struct changes address during this move, any pointer or address
derived from a field *during* `new()` becomes stale once the struct lands in
its final location.

This document describes the invariant that prevents use of those stale
addresses and the post-move fixup sequence that restores correctness.

## The Problem

Several `PartitionControlBlock` fields store addresses that point back into
the struct itself (e.g. the base of a partition's stack buffer or MPU data
region).  During `Kernel::new()` the struct lives on the caller's stack, so
any address derived from a field at construction time reflects that *temporary*
stack location.  After `ptr::write()` copies the struct into
`UNIFIED_KERNEL_STORAGE`, the data now lives at a completely different address
and every self-referential value captured during construction is invalid.

Using a stale address would configure the MPU to protect the wrong memory
region or set a stack pointer into arbitrary RAM — both of which are
safety-critical failures in a hard-realtime RTOS.

## The Invariant

> **No address derived from a `Kernel` field during `Kernel::new()` may be
> used after placement without post-move patching.**

The correct pattern is:

1. Construct with sentinel / placeholder values (`PartitionConfig::sentinel()`).
2. Place into `UNIFIED_KERNEL_STORAGE` with `ptr::write()`.
3. Call the appropriate `fix_*()` methods **after** placement so they compute
   addresses from the struct's final location.
4. Verify with `invariants::assert_pcb_addresses_in_storage()`.

## Affected Fields

| Field | Fixup method | Applied in |
|---|---|---|
| `PartitionControlBlock.mpu_region.base` | `fix_mpu_data_region()` | `boot.rs` post-placement fixup loop |
| `PartitionControlBlock.stack_base` | `fix_stack_region()` | `boot.rs` post-placement fixup loop |

## In-Code Documentation

The invariant is documented inline in two locations within `kernel/src/svc.rs`:

- **Lines 14-37** — Primary block near the top of the file, covering the
  invariant statement, affected fields, the correct construction pattern, and
  cross-references to `boot.rs` and `invariants.rs`.
- **Lines 867-890** — Secondary block adjacent to the `Kernel` struct
  definition, restating the invariant with emphasis on the fixup methods and
  their calling sequence.

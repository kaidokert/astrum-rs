# CRITICAL: Port Bug 09 — validate_user_ptr Rejects Partition Stack Buffers

**Status: RESOLVED**

**Severity:** CRITICAL — IPC syscalls silently reject stack-allocated buffers
passed by user partitions, because the kernel-data guard fires before the
accessible-regions grant check.

---

## Symptom

IPC syscalls (`SYS_MSG_SEND`, `SYS_BB_READ`, etc.) silently reject
stack-allocated buffers with `SvcError::InvalidPointer`.  Partition stacks
reside inside the `.kernel_state` linker section in SRAM, so
`validate_user_ptr`'s monolithic `overlaps_kernel_memory` guard treated them
as kernel data and rejected them before checking whether the pointer fell
within a granted region.

**Concrete scenario:** Partition P0 has its stack at `0x2000_0000..0x2000_0800`
(inside `.kernel_state`), and its data region at `0x2000_2000..0x2000_3000`.
P0 issues `SYS_MSG_SEND` with a buffer on its stack (e.g. `0x2000_0400`).
The old `validate_user_ptr` checked `overlaps_kernel_memory` first — the
buffer address fell within `[0x2000_0000, __kernel_state_end)`, so the
pointer was rejected.  The syscall returned `InvalidPointer` and the IPC
message was silently dropped.

---

## Root Cause

Guard ordering in `validate_user_ptr`.  The original implementation ran the
kernel-data rejection check (SRAM guard) **before** the whitelist of granted
regions (data, stack, peripheral).  Partition stacks are allocated inside the
`.kernel_state` linker section, so their addresses overlap the kernel data
region `[0x2000_0000, __kernel_state_end)`.  Because the rejection fired
first, the grant check never had a chance to accept the pointer.

Original ordering: (1) reject kernel code, (2) reject kernel data, (3) accept
grant.  The stack is a legitimate grant, but step 2 rejected it before step 3.

---

## Fix

Three-guard ordering: flash reject, then grant accept, then SRAM reject.
Implemented in commit `a8043b9`.

### After (fixed ordering — svc.rs:82–118)

```rust
// Guard 1 — Flash: unconditionally reject pointers in kernel code region.
if ptr < KERNEL_CODE_END && end > 0 {
    return false;
}

// Guard 2 — Accessible-regions grant (runs before SRAM guard).
for (base, size) in pcb.accessible_static_regions() {
    let region_end = base + size;
    if ptr >= base && end <= region_end {
        return true;
    }
}

// Guard 3 — SRAM kernel-data: reject overlap with kernel data in SRAM.
if overlaps_kernel_data(ptr, end, kernel_data_end()) {
    return false;
}
```

**Key insight:** Guard 2 (grant) runs **before** Guard 3 (SRAM rejection).
A pointer to a partition's own stack is accepted by Guard 2 and never reaches
Guard 3.  Pointers that are genuinely inside kernel data (not granted to any
partition) fall through Guard 2 and are caught by Guard 3.

The same three-guard ordering is applied to `validate_user_ptr_dynamic`
(svc.rs:126–177), which additionally checks dynamic MPU windows in Guard 2a
before falling back to static regions in Guard 2b.

---

## Affected Code Paths

| # | File:Line | Function | Change |
|---|-----------|----------|--------|
| 1 | svc.rs:82–118 | `validate_user_ptr` | Three-guard ordering (flash → grant → SRAM) |
| 2 | svc.rs:126–177 | `validate_user_ptr_dynamic` | Same ordering with dynamic MPU windows |
| 3 | svc.rs:73–75 | `overlaps_kernel_data` | Helper: parameterized kernel-data overlap check |
| 4 | svc.rs:36–46 | `kernel_data_end()` | ARM: reads `__kernel_state_end` linker symbol |
| 5 | svc.rs:60–66 | `kernel_data_end()` | Host/test: compile-time constant with test override |

---

## Regression Test Coverage

All tests in `kernel/src/svc.rs`:

### Bug 09 core tests

| # | Test Function | Scenario |
|---|---------------|----------|
| 1 | `validate_ptr_own_stack_in_kernel_data_accepted` | Stack inside kernel data — Guard 2 grants before Guard 3 (core bug 09) |
| 2 | `validate_ptr_kernel_data_rejected` | Pointer in kernel data with no grant — Guard 3 rejects |
| 3 | `validate_ptr_in_kernel_code_fails` | Pointer in flash kernel code — Guard 1 rejects |
| 4 | `validate_ptr_kernel_code_boundary` | Exact KERNEL_CODE_END boundary |

### Region membership tests

`validate_ptr_valid_at_start`, `validate_ptr_valid_middle`,
`validate_ptr_valid_exact_end`, `validate_ptr_before_region`,
`validate_ptr_after_region`, `validate_ptr_spans_past_end`,
`validate_ptr_exact_full_region`, `validate_ptr_single_byte_at_region_edges`,
`validate_ptr_zero_length`, `validate_ptr_nonexistent_partition`,
`validate_ptr_in_stack_region_passes`, `validate_ptr_in_data_region_with_separate_stack`,
`validate_ptr_spanning_stack_and_data_fails`, `validate_ptr_outside_both_regions_fails`

### Edge case & overflow tests

`validate_ptr_overflow_wraps`, `validate_ptr_length_wraps_address_space`,
`validate_ptr_null_and_low_kernel_code_rejected`, `validate_ptr_null_pointer_rejected`,
`validate_ptr_at_u32_max_boundary`, `validate_ptr_misaligned_within_valid_region`

### Peripheral region tests

`validate_ptr_in_peripheral_region_passes`, `validate_ptr_outside_peripheral_region_fails`,
`validate_ptr_multiple_peripheral_regions`, `validate_ptr_no_peripheral_regions_configured`

### Dynamic-MPU variant tests (`#[cfg(feature = "dynamic-mpu")]`)

`validate_ptr_dynamic_own_stack_in_kernel_data_accepted` (mirrors core bug 09 test),
`validate_ptr_dynamic_comprehensive`, `validate_ptr_dynamic_in_peripheral_region_passes`,
`validate_ptr_dynamic_in_kernel_code_fails`, `validate_ptr_dynamic_kernel_data_empty_range`,
`validate_ptr_dynamic_window_revocation`

---

## Resolution Checklist

- [x] Three-guard ordering implemented: flash → grant → SRAM (svc.rs)
- [x] `overlaps_kernel_data` extracted as parameterized helper for testability
- [x] `kernel_data_end()` reads `__kernel_state_end` linker symbol on ARM
- [x] Same ordering applied to `validate_user_ptr_dynamic` (dynamic-mpu feature)
- [x] 34 regression tests covering all three guards, boundaries, and edge cases
- [x] No dynamic allocation — all checks are inline comparisons

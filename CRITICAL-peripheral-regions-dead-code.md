**Status: RESOLVED**

# CRITICAL: peripheral_regions — Dead-Code / Silent-Override Analysis

**Severity:** CRITICAL — user-specified peripheral permissions are silently
discarded at every MPU programming path. The `MpuRegion.permissions` field
(partition.rs:60) is dead code for peripheral regions.

---

## Root Cause 1 — `peripheral_region_pair` ignores `MpuRegion.permissions`

**File:** `kernel/src/mpu.rs:284-288`

`peripheral_region_pair` receives an `&MpuRegion` that carries a
caller-supplied `permissions` field, but the function never reads it.
Instead it hardcodes:

```rust
let rasr = build_rasr(size_field, AP_FULL_ACCESS, true, (true, false, true));
```

The RASR is built with fixed AP (full access), XN=true, and S/C/B = 1/0/1
regardless of what the partition configuration requested. Any
`MpuRegion::new(base, size, custom_permissions)` has its third argument
silently dropped.

---

## Root Cause 2 — `wire_boot_peripherals` duplicates the same override

**File:** `kernel/src/mpu_strategy.rs:305-309`

The dynamic-MPU path (`wire_boot_peripherals`) iterates each partition's
`peripheral_regions()` but builds its own RASR from scratch:

```rust
let rasr = crate::mpu::build_rasr(
    size.trailing_zeros() - 1,
    crate::mpu::AP_FULL_ACCESS,
    true,
    (true, false, true),
);
```

Again, `region.permissions()` is never called. This is the second
independent code path that silently overrides user-specified permissions
with the same hardcoded device-memory attributes.

---

## Root Cause 3 — Docstring / code mismatch on S/C/B bits

**File:** `kernel/src/mpu_strategy.rs:278`

The `wire_boot_peripherals` docstring states:

> device memory: **S/C/B=0**, XN, AP_FULL_ACCESS

But the code at line 309 passes `(true, false, true)` which encodes
**S=1, C=0, B=1** — Shareable Device memory, not Strongly-ordered
(S/C/B=0/0/0). The `peripheral_region_pair` docstring at mpu.rs:281
correctly documents S=1,C=0,B=1, confirming the code intent is Shareable
Device. The `wire_boot_peripherals` docstring is wrong.

---

## Root Cause 4 — Zero exercised configurations

**File:** `kernel/examples/*.rs` (all 23 examples)

Every example in the tree sets `peripheral_regions: heapless::Vec::new()`.
There is no example, test, or QEMU binary that configures a non-empty
peripheral region. This means:

- The override in Root Cause 1 has never been observed in practice.
- The `permissions` field has never been tested end-to-end.
- Any future user who sets `permissions` will get silently wrong MPU
  attributes with no compile-time or runtime warning.

---

## Resolution Checklist

| # | Subtask | Action | Status |
|---|---------|--------|--------|
| 1 | Fix wire_boot_peripherals docstring; add RASR attribute test | Correct S/C/B=0 docstring to S=1,C=0,B=1; add unit test asserting the RASR bits match Shareable Device attributes | done |
| 2 | Document peripheral RASR override; add permissions-independence test | Add doc-comments to both code paths explaining the intentional override; add test proving `permissions` field is not consulted | done |
| 3 | Create peripheral_passthrough QEMU example | Add a QEMU example that configures a non-empty `peripheral_regions` vec targeting a real MMIO range (e.g. UART0 on lm3s6965evb) | done |
| 4 | Update driver-architecture.md for static-mode and slot layout | Document the static-mode peripheral slot layout (R4/R5) and the intentional override rationale in design docs | done |
| 5 | **#286** — Close CRITICAL-peripheral-regions-dead-code backlog item | After resolution subtasks are merged, mark this item done in backlog.json | done |

---

## Decision Required

The `MpuRegion.permissions` field is currently meaningful only for the
primary data region (R3). For peripheral regions, the kernel enforces a
single policy: Shareable Device memory, full-access, execute-never. Two
paths forward:

- **Approach A — Keep the override, document it.** Peripheral regions are
  always Device memory. The `permissions` field is intentionally ignored.
  Add `#[allow(dead_code)]` or remove the field from peripheral MpuRegions.

- **Approach B — Honor `permissions`.** Thread the field through to
  `build_rasr`. This adds flexibility but requires validating that users
  don't accidentally map MMIO as Normal-cacheable or executable.

**Decision: Approach A was chosen and implemented.** The resolution
subtasks (see checklist above) implemented Approach A (document and test
the intentional override). Approach B is deferred unless a concrete use
case emerges.

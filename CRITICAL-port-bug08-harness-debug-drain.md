# CRITICAL: Port Bug 08 — Harness Yield Never Drains Partition Debug Buffers

**Status: RESOLVED**

**Severity:** CRITICAL — buffered debug output from partitions that yield
voluntarily is delayed indefinitely or lost, because the drain path only
existed in the SysTick handler.

---

## Symptom

Partition debug buffers are only drained at SysTick tick boundaries via
`systick_handler`.  Partitions that yield voluntarily (via `SYS_YIELD` or any
blocking syscall that calls `trigger_deschedule`) never have their debug
output flushed.  Buffered output is delayed until the next tick boundary — or
lost entirely if the buffer wraps before the tick fires.

---

## Root Cause

The `define_unified_harness!` yield closure called `yield_current_slot()` but
never invoked any debug drain logic.  The only drain path was inside the
SysTick handler (`tick.rs`), which calls `drain_debug_pending` at each tick
boundary.  Voluntary yields bypass the SysTick path entirely, so debug output
accumulated in partition ring buffers with no opportunity to flush until the
next tick — introducing unbounded latency between a partition writing debug
output and the host observing it.

---

## Fix

Three-part fix: a config constant, a new drain entry point, and wiring into
the harness yield closure.

### 1. Config constant `DEBUG_AUTO_DRAIN_BUDGET` — config.rs:258–261

New constant on `KernelConfig` controlling the maximum bytes
`drain_debug_auto` will drain per call.  Defaults to 256.  Not `cfg`-gated so
that `drain_debug_auto` compiles unconditionally.  Setting to 0 disables
automatic draining entirely.

```rust
/// Maximum bytes `drain_debug_auto` will drain per call.
/// Set to 0 to disable automatic draining entirely.
/// Not cfg-gated so that `drain_debug_auto` compiles unconditionally.
const DEBUG_AUTO_DRAIN_BUDGET: usize = 256;
```

### 2. `Kernel::drain_debug_auto()` — svc.rs:2593–2601

New public method that creates a stack-local `DrainContext` and forwards to
`drain_debug_pending` with the configured budget.  Compiles to zero-cost when
`partition-debug` is disabled or when `DEBUG_AUTO_DRAIN_BUDGET` is 0.

```rust
pub fn drain_debug_auto(&mut self) {
    #[cfg(feature = "partition-debug")]
    {
        if C::DEBUG_AUTO_DRAIN_BUDGET > 0 {
            let mut ctx = crate::partition_debug::DrainContext::new();
            self.drain_debug_pending(&mut ctx, C::DEBUG_AUTO_DRAIN_BUDGET);
        }
    }
}
```

### 3. Harness yield closure wiring — harness.rs:265

`drain_debug_auto()` is called immediately after `yield_current_slot()` in the
harness yield closure, ensuring every voluntary yield flushes pending debug
output.

```rust
// Flush partition debug buffers at yield boundaries.
// Zero-cost when partition-debug is disabled or budget is 0.
k.drain_debug_auto();
```

---

## Affected Code Paths

| # | File:Line | Function / Constant | Change |
|---|-----------|---------------------|--------|
| 1 | config.rs:258–261 | `DEBUG_AUTO_DRAIN_BUDGET` | New config constant (default 256) |
| 2 | svc.rs:2593–2601 | `Kernel::drain_debug_auto()` | New method — drain entry point for yield path |
| 3 | svc.rs:2574–2586 | `Kernel::drain_debug_pending()` | Existing method — called by `drain_debug_auto` |
| 4 | harness.rs:265 | yield closure | Wired `drain_debug_auto()` after `yield_current_slot()` |

---

## Regression Test Coverage

All tests in `kernel/src/svc.rs`, gated on `#[cfg(feature = "partition-debug")]`:

| # | Test Function | File:Line | Scenario |
|---|---------------|-----------|----------|
| 1 | `drain_debug_pending_iterates_and_clears` | svc.rs:9844 | Two partitions with pending output — drain iterates both, clears pending flags, returns total bytes drained |
| 2 | `drain_debug_pending_skips_non_pending` | svc.rs:9899 | Two partitions, only one pending — drain skips the non-pending partition |
| 3 | `drain_debug_auto_drains_pending_output` | svc.rs:9944 | Single partition with pending output — `drain_debug_auto` flushes buffer and clears pending flag |
| 4 | `drain_debug_auto_budget_zero_is_noop` | svc.rs:9971 | Custom config with `DEBUG_AUTO_DRAIN_BUDGET = 0` — `drain_debug_auto` is a no-op, pending flag remains set |
| 5 | `yield_then_drain_debug_auto_clears_pending` | svc.rs:10067 | Mirrors harness yield closure: `yield_current_slot` then `drain_debug_auto` clears pending output |
| 6 | `drain_debug_auto_multi_partition_clears_all` | svc.rs:10100 | Two partitions both pending — `drain_debug_auto` drains both in one call |
| 7 | `drain_debug_auto_idempotent_second_call_noop` | svc.rs:10152 | Second `drain_debug_auto` call after first already drained — no-op, returns zero |

---

## Resolution Checklist

- [x] `DEBUG_AUTO_DRAIN_BUDGET` constant added to `KernelConfig` (config.rs)
- [x] `drain_debug_auto()` method added to `Kernel` (svc.rs)
- [x] `drain_debug_auto()` compiles to zero-cost when feature disabled or budget is 0
- [x] Harness yield closure calls `drain_debug_auto()` after `yield_current_slot()` (harness.rs)
- [x] SysTick drain path unchanged — continues to use `drain_debug_pending` directly
- [x] All 7 regression tests passing under `partition-debug` feature gate
- [x] No dynamic allocation — `DrainContext` is stack-local
- [x] Budget bound ensures bounded loop iteration (deterministic)

---

## Known Optimization Opportunity

**SysTick ISR jitter:** The SysTick handler currently drains up to
`DEBUG_BUFFER_SIZE` bytes per tick, which adds variable latency to the ISR.
This is a known optimization opportunity — the drain could be deferred to a
lower-priority context (e.g., PendSV tail) to reduce tick jitter.  This is
**not a correctness issue**, only a latency optimization TODO.

# CRITICAL: Port Bug 05 — trigger_deschedule Leaves Partition Ready When No Partner

**Severity:** CRITICAL — violates the at-most-one-Running invariant, leaving a
partition in Ready state with no Running partition in the system.

---

## Symptom

When a blocking syscall (or `SYS_YIELD`) calls `trigger_deschedule()` and no
runnable partner exists in the next schedule slot, the outgoing partition is
left in **Ready** instead of **Running**.  No partition is Running, violating
the scheduler's fundamental invariant.

**Concrete scenario:** P0 is Running, P1 is Waiting (blocked on IPC).  P0
calls a blocking syscall.  `trigger_deschedule()` eagerly transitions P0
Running→Ready, then `yield_current_slot()` finds P1 is Waiting and returns
`None`.  P0 is now Ready with no Running partition.

---

## Root Cause

`trigger_deschedule()` called `transition_outgoing_ready()` **eagerly**, before
confirming that a runnable switch partner exists:

```rust
// BEFORE fix (broken):
pub fn trigger_deschedule(&mut self) -> u32 {
    self.transition_outgoing_ready();  // premature!
    self.yield_requested = true;
    handle_yield()
}
```

The Running→Ready transition fired unconditionally.  If
`yield_current_slot()` subsequently found no valid (non-Waiting) partner, no
context switch occurred — but the outgoing partition had already been demoted.

---

## Fix

Remove the premature `transition_outgoing_ready()` call from
`trigger_deschedule()`.  The transition is deferred to `yield_current_slot()`
(syscall path) and `advance_schedule_tick()` (tick-driven path), which only
fire the transition **after** confirming a runnable partner exists.

```rust
// AFTER fix (svc.rs:1286):
pub fn trigger_deschedule(&mut self) -> u32 {
    self.yield_requested = true;
    handle_yield()
}
```

In both `yield_current_slot()` and `advance_schedule_tick()`, the transition
is guarded: if the target partition is Waiting, the function returns early
without demoting the outgoing partition.

---

## Affected Syscall Paths

All blocking syscall paths that call `trigger_deschedule()`:

| # | Syscall | File:Line | Trigger Condition |
|---|---------|-----------|-------------------|
| 1 | `SYS_YIELD` | svc.rs:1406 | Always |
| 2 | `SYS_EVT_WAIT` | svc.rs:1410 | No events set (result == 0) |
| 3 | `SYS_SEM_WAIT` | svc.rs:1429 | Semaphore unavailable |
| 4 | `SYS_MTX_LOCK` | svc.rs:1451 | Mutex held by another partition |
| 5 | `SYS_MSG_SEND` | svc.rs:1496 | Sender blocked (queue full) |
| 6 | `SYS_MSG_RECV` | svc.rs:2619 | Receiver blocked (no message) |
| 7 | `SYS_BB_READ` | svc.rs:1708 | ReaderBlocked (empty blackboard) |
| 8 | `SYS_DEV_READ_TIMED` | svc.rs:1855 | No data, timeout > 0 |
| 9 | `SYS_QUEUING_RECV_TIMED` | svc.rs:1901 | ReceiverBlocked |
| 10 | `SYS_QUEUING_SEND_TIMED` | svc.rs:1939 | SenderBlocked |

---

## Regression Test Coverage

### Existing tests (kernel/src/svc.rs)

| Test | Syscall Path | Scenario |
|------|-------------|----------|
| `bug05_trigger_deschedule_then_yield_no_partner_preserves_running` | Core | trigger_deschedule + yield, no partner → P0 stays Running |
| `bug05_blocking_syscall_stays_waiting_after_yield_no_partner` | SYS_EVT_WAIT | Block → Waiting, yield finds no partner |
| `bug05_dispatch_yield_then_second_svc_no_invariant_panic` | SYS_YIELD | Yield then second SVC; invariant check passes |
| `bug05_advance_schedule_tick_skips_waiting_target` | Tick path | Tick-driven switch skips Waiting target |
| `bug05_trigger_deschedule_idempotent_double_call` | Core | Two consecutive calls are idempotent |
| `bug05_consecutive_block_no_partner_preserves_invariant` | SYS_EVT_WAIT | Block→yield→wake→block→yield cycle |
| `bug05_wakeup_reblock_cycle_full_lifecycle` | SYS_EVT_WAIT | Full block→switch→wake→run→reblock→yield lifecycle |
| `bug05_major_frame_wrap_waiting_partition_invariant` | Tick path | Major-frame wrap with Waiting partition |
| `bug05_mutex_lock_blocks_then_yield_no_partner` | SYS_MTX_LOCK | Mutex blocks P0, yield finds no partner |

### Remaining coverage gaps

| # | Syscall Path | Backlog Item |
|---|-------------|--------------|
| 1 | `SYS_MSG_SEND` | Add Bug 05 test for SYS_MSG_SEND blocking path |
| 2 | `SYS_MSG_RECV` | Add Bug 05 test for SYS_MSG_RECV blocking path |
| 3 | `SYS_BB_READ` | Add Bug 05 test for SYS_BB_READ blocking path |
| 4 | `SYS_DEV_READ_TIMED` | Add Bug 05 test for SYS_DEV_READ_TIMED blocking path |
| 5 | `SYS_QUEUING_SEND_TIMED` | Add Bug 05 test for SYS_QUEUING_SEND_TIMED blocking path |
| 6 | `SYS_QUEUING_RECV_TIMED` | Add Bug 05 test for SYS_QUEUING_RECV_TIMED blocking path |

---

## Invariant Guard

Commit `fc0a65b` added `assert_waiting_implies_yield_requested()` in
`kernel/src/invariants.rs`.  This fires after every `dispatch()` call,
ensuring that whenever a partition transitions Running→Waiting,
`yield_requested` is `true`.

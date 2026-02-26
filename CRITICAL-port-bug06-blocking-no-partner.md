# CRITICAL: Port Bug 06 — Blocking Syscall With No Runnable Partner Strands Partition

**Status: RESOLVED**

**Severity:** CRITICAL — violates the exactly-one-Running invariant, leaving
all partitions in Waiting state with no Running partition in the system.

---

## Symptom

When a blocking syscall (`SYS_EVT_WAIT`, `SYS_SEM_WAIT`, etc.) transitions the
active partition to Waiting and the nominated next partition is also Waiting, no
partition is Running.  The active partition is stranded in Waiting state,
violating the exactly-one-Running invariant.

**Concrete scenario (2 partitions):** P0 is Running, P1 is Waiting (blocked on
IPC).  P0 issues `SYS_EVT_WAIT` with no events set → P0 transitions
Running→Waiting, `trigger_deschedule()` calls `yield_current_slot()`.  The
schedule nominates P1, but P1 is Waiting.  Without the Bug 06 guard, the
function returns `None` — P0 stays Waiting, P1 stays Waiting, zero partitions
are Running.

---

## Root Cause

`yield_current_slot()` and `advance_schedule_tick()` did not check whether the
active partition needed to be restored to Running when the nominated next
partition was Waiting.  The Bug 05 fix correctly skipped context-switching to a
Waiting partition (returning early), but did not handle the case where the
active partition had *already* transitioned to Waiting before the yield.  When
both active and nominated are Waiting, the early return leaves no Running
partition.

---

## Fix

Two guards added — one in each scheduling path.  When the nominated partition
is Waiting, the guard checks whether the active partition is also Waiting.  If
so, it restores the active partition to Running via `try_transition` +
`set_next_partition` and returns `ScheduleEvent::None`.

### Before (Bug 05 only)

```rust
// advance_schedule_tick / yield_current_slot (svc.rs)
if is_waiting {
    return ScheduleEvent::None;
}
```

### After (Bug 06 guard added)

```rust
// advance_schedule_tick (svc.rs:2366-2377)
// yield_current_slot  (svc.rs:2406-2417)
if is_waiting {
    // Bug 06: restore active partition to Running if it is Waiting.
    if let Some(ap) = self.active_partition {
        if self
            .partitions()
            .get(ap as usize)
            .is_some_and(|p| p.state() == PartitionState::Waiting)
            && try_transition(self.partitions_mut(), ap, PartitionState::Ready)
        {
            self.set_next_partition(ap);
        }
    }
    return ScheduleEvent::None;
}
```

The `try_transition` moves the active partition Waiting→Ready, then
`set_next_partition` promotes it Ready→Running and re-establishes it as the
active partition — restoring the exactly-one-Running invariant.

---

## Relationship to Bug 05

Bug 05 removed the premature `transition_outgoing_ready()` from
`trigger_deschedule()`, preventing the active partition from being demoted to
Ready when no runnable partner exists.  Bug 06 adds the **complementary guard**
that handles the case where the active partition has already transitioned to
Waiting (via a blocking syscall) before the yield/tick path discovers that the
nominated partner is also Waiting.  Together, the two fixes ensure that no
scheduling path can leave the system without a Running partition.

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

All tests in `kernel/src/svc.rs`:

| # | Test Function | Scenario |
|---|---------------|----------|
| 1 | `bug06_advance_tick_restores_active_when_waiting` | Tick path: P0 active+Waiting, P1 Waiting; tick boundary nominates P1, guard restores P0 to Running |
| 2 | `bug06_yield_guard_noop_when_active_running` | Yield path: P1 Waiting but P0 is Running (not Waiting); guard is no-op, preserves Bug 05 behavior |
| 3 | `bug06_blocking_with_ready_partner_normal_switch` | Yield path: P0 blocks via SYS_EVT_WAIT, P1 is Ready; normal switch, guard never fires |
| 4 | `bug06_single_partition_blocking_no_panic` | Yield path: single-partition kernel, P0 blocks, yield wraps to self (Waiting), guard restores P0 |
| 5 | `bug06_yield_both_waiting_restores_active` | Yield path: both P0 and P1 Waiting; guard fires, restores P0 to Running |
| 6 | `bug06_tick_three_partitions_all_waiting_restores_active` | Tick path (3 partitions): P0+P1+P2 all Waiting; tick boundary nominates P1, guard restores P0 |
| 7 | `bug06_yield_three_partitions_nominee_waiting_restores_active` | Yield path (3 partitions): P0+P1 Waiting, P2 Ready; yield nominates P1 (Waiting), guard restores P0 |

---

## Invariant Guards

Three invariant assertions validate the fix at test time:

- **`assert_waiting_implies_yield_requested`** (`invariants.rs`): ensures that
  whenever a partition transitions Running→Waiting, `yield_requested` is true —
  blocking must always trigger deschedule.

- **`assert_running_matches_active`** (`invariants.rs`): ensures exactly one
  partition is Running and it matches `active_partition` — the primary invariant
  that Bug 06 protects.

- **`assert_partition_state_consistency`** (`invariants.rs`): ensures at most
  one partition is in Running state across the entire partition table.

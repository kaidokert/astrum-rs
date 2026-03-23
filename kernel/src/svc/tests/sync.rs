use super::*;

// -------------------------------------------------------------------------
// Sync primitive dispatch tests
// -------------------------------------------------------------------------
//
// # SAFETY — Test Dispatch Justification
//
// All `unsafe { k.dispatch(&mut ef) }` calls in this test module share
// the same safety justification:
//
// ## Test Isolation
//
// Each `#[test]` function creates its own `Kernel` and `ExceptionFrame`
// instances on the stack. No global state is shared between tests, so
// tests cannot interfere with each other's kernel or partition state.
//
// ## Single-Threaded Execution
//
// Rust's default test runner executes tests in a single-threaded manner
// (unless `--test-threads=N` is specified, which these tests do not
// rely on). Even with parallel test execution, each test has isolated
// stack-local state. The `dispatch()` function is not re-entrant, but
// since each test owns its own `Kernel` instance, concurrent test
// execution does not cause data races.
//
// ## Valid ExceptionFrame Construction
//
// The `frame()` test helper constructs `ExceptionFrame` instances with
// valid register values. Unlike hardware exception entry, these are not
// actual stacked registers, but the dispatch logic only reads/writes
// the r0-r3 fields which are always initialized. The remaining fields
// (r12, lr, pc, xpsr) are set to zero, which is safe because dispatch
// does not use them.
//
// ## Kernel Construction
//
// The `kernel()` and `kernel_with_registry()` helpers construct `Kernel`
// instances with properly initialized partition tables (via `tbl()`),
// schedule tables, and resource pools. All partitions have valid MPU
// regions and are transitioned to Running state before dispatch.
//
// ## Host-Mode Pointer Validation
//
// Tests run on the host (not target hardware) where `validate_user_ptr`
// checks pass for any pointer within the partition's configured MPU
// region. Tests that exercise pointer validation use `mmap` to allocate
// memory at addresses matching the partition's MPU region, ensuring
// the kernel's bounds checks succeed.

/// Confused-deputy regression: SemWait must use kernel-derived `caller`,
/// not user-supplied r2. If r2=1 but current_partition=0, sem blocks partition 0.
#[test]
fn sem_wait_uses_caller_not_r2() {
    // 1 semaphore with count=0 so wait will block.
    let mut k = kernel(1, 0, 0);
    // Drain the semaphore: count starts at 1, first wait acquires.
    let mut ef = frame(crate::syscall::SYS_SEM_WAIT, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 1, "first SemWait must acquire (count was 1)");

    // Now count=0. Attacker sets r2=1 to impersonate partition 1.
    let mut ef = frame(crate::syscall::SYS_SEM_WAIT, 0, 1);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0, "SemWait must block (count=0)");
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Waiting,
        "partition 0 must be Waiting (not partition 1)"
    );
    assert_eq!(
        k.partitions().get(1).unwrap().state(),
        PartitionState::Running,
        "partition 1 must be unaffected"
    );
}

/// Confused-deputy regression: MutexLock must use kernel-derived `caller`,
/// not user-supplied r2. If r2=1 but current_partition=0, mutex is owned by partition 0.
#[test]
fn mutex_lock_uses_caller_not_r2() {
    let mut k = kernel(0, 1, 0);
    // Attacker sets r2=1 to impersonate partition 1.
    let mut ef = frame(crate::syscall::SYS_MTX_LOCK, 0, 1);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 1, "MutexLock must acquire immediately");
    assert_eq!(
        k.mutexes().owner(0),
        Ok(Some(0)),
        "mutex must be owned by partition 0 (not partition 1)"
    );
}

/// Confused-deputy regression: MutexUnlock must use kernel-derived `caller`,
/// not user-supplied r2. If r2=1 but current_partition=0, unlock acts as partition 0.
#[test]
fn mutex_unlock_uses_caller_not_r2() {
    let mut k = kernel(0, 1, 0);
    // Partition 0 acquires the mutex.
    let mut ef = frame(crate::syscall::SYS_MTX_LOCK, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 1, "MutexLock must acquire");
    assert_eq!(k.mutexes().owner(0), Ok(Some(0)));

    // Attacker sets r2=1 to impersonate partition 1 during unlock.
    let mut ef = frame(crate::syscall::SYS_MTX_UNLOCK, 0, 1);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0, "MutexUnlock must succeed for partition 0");
    assert_eq!(
        k.mutexes().owner(0),
        Ok(None),
        "mutex must be unlocked by partition 0 (not rejected as not-owner)"
    );
}

#[test]
fn sem_wait_and_signal_dispatch() {
    let mut k = kernel(1, 0, 0);
    let mut ef = frame(crate::syscall::SYS_SEM_WAIT, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 1);
    let mut ef = frame(crate::syscall::SYS_SEM_SIGNAL, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0);
}

#[test]
fn dispatch_sem_wait_blocking_triggers_deschedule() {
    let mut k = kernel(1, 0, 0);
    // First wait: semaphore has count=1, so this acquires immediately (Ok(true)).
    let mut ef = frame(crate::syscall::SYS_SEM_WAIT, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 1);
    assert!(!k.yield_requested());

    // Second wait: count is now 0, so partition 0 blocks (Ok(false)).
    let mut ef = frame(crate::syscall::SYS_SEM_WAIT, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0);
    assert!(k.yield_requested());
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Waiting
    );
}

#[test]
fn dispatch_sem_wait_targets_correct_index_via_r1() {
    // Create 3 semaphores (indices 0, 1, 2) each with count=1, max=2.
    let mut k = kernel(3, 0, 0);
    // Dispatch SemWait with r1=2 to target semaphore index 2.
    let mut ef = frame(crate::syscall::SYS_SEM_WAIT, 2, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    // r0=1 means acquired immediately (Ok(true)).
    assert_eq!(ef.r0, 1, "SemWait must return 1 (acquired)");
    // Semaphore 2 should have count decremented from 1 to 0.
    assert_eq!(k.semaphores().get(2).unwrap().count(), 0);
    // Semaphores 0 and 1 must remain untouched at count=1.
    assert_eq!(k.semaphores().get(0).unwrap().count(), 1);
    assert_eq!(k.semaphores().get(1).unwrap().count(), 1);
}

#[test]
fn mutex_lock_unlock_dispatch() {
    let mut k = kernel(0, 1, 0);
    let mut ef = frame(crate::syscall::SYS_MTX_LOCK, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 1); // 1 = acquired immediately
    assert_eq!(k.mutexes().owner(0), Ok(Some(0)));
    let mut ef = frame(crate::syscall::SYS_MTX_UNLOCK, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0);
    assert_eq!(k.mutexes().owner(0), Ok(None));
}

#[test]
fn dispatch_mutex_lock_blocking_triggers_deschedule() {
    let mut k = kernel(0, 1, 0);
    // Partition 0 acquires mutex 0 immediately.
    let mut ef = frame(crate::syscall::SYS_MTX_LOCK, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 1, "uncontested MutexLock must return 1 (acquired)");
    assert!(!k.yield_requested());

    // Switch to partition 1 and attempt to lock the same mutex.
    k.set_current_partition(1);
    let mut ef = frame(crate::syscall::SYS_MTX_LOCK, 0, 1);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 0, "blocking MutexLock must return 0");
    assert!(
        k.yield_requested(),
        "blocking MutexLock must trigger deschedule"
    );
    assert_eq!(
        k.partitions().get(1).unwrap().state(),
        PartitionState::Waiting,
        "blocked partition must be in Waiting state"
    );
}

#[test]
fn dispatch_mutex_lock_immediate_no_deschedule() {
    let mut k = kernel(0, 1, 0);
    // Mutex 0 is free — MutexLock should acquire immediately without deschedule.
    let mut ef = frame(crate::syscall::SYS_MTX_LOCK, 0, 0);
    // SAFETY: See module-level SAFETY docs for test dispatch justification.
    unsafe { k.dispatch(&mut ef) };
    assert_eq!(ef.r0, 1, "immediate MutexLock must return 1 (acquired)");
    assert!(
        !k.yield_requested(),
        "immediate MutexLock must not trigger deschedule"
    );
    assert_eq!(
        k.partitions().get(0).unwrap().state(),
        PartitionState::Running,
        "acquiring partition must stay Running"
    );
    assert_eq!(
        k.mutexes().owner(0),
        Ok(Some(0)),
        "mutex must be owned by acquiring partition"
    );
}

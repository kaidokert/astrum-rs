use crate::config::KernelConfig;
use crate::context::ExceptionFrame;
use crate::kernel_config_types;
use crate::message::MessageQueue;
use crate::partition::{
    ConfigError, ExternalPartitionMemory, MpuRegion, PartitionState, TransitionError,
};
use crate::partition_core::{AlignedStack1K, StackStorage}; // StackStorage: trait for as_u32_slice()
use crate::scheduler::{ScheduleEntry, ScheduleTable};
use crate::semaphore::Semaphore;
use crate::svc::{Kernel, YieldResult};
use heapless::Vec;

pub struct HarnessConfig;

impl KernelConfig for HarnessConfig {
    const N: usize = 4;
    const SCHED: usize = 8;
    const S: usize = 4;
    const SW: usize = 4;
    const MS: usize = 4;
    const MW: usize = 4;
    const QS: usize = 4;
    const QD: usize = 2;
    const QM: usize = 4;
    const QW: usize = 4;
    const SP: usize = 4;
    const SM: usize = 4;
    const BS: usize = 4;
    const BM: usize = 4;
    const BW: usize = 4;
    #[cfg(feature = "dynamic-mpu")]
    const BP: usize = 4;
    kernel_config_types!();
}

/// Base address for partition flash (code) regions in the test memory map.
const FLASH_BASE: u32 = 0x0800_0000;
/// Base address for partition RAM (stack/data) regions in the test memory map.
const RAM_BASE: u32 = 0x2000_0000;
/// Address offset between consecutive partition regions.
const PARTITION_OFFSET: u32 = 0x1000;
/// Stack size in bytes for each test partition, derived from STACK_WORDS.
const STACK_SIZE_BYTES: u32 = HarnessConfig::STACK_WORDS as u32 * 4;

#[derive(Debug)]
pub enum HarnessError {
    InvalidPartitionCount,
    ScheduleFull,
    ConfigsFull,
    KernelInit(ConfigError),
    Transition(TransitionError),
    PartitionNotFound,
}

pub struct KernelTestHarness {
    kernel: Box<Kernel<HarnessConfig>>,
    _stacks: Box<[AlignedStack1K; HarnessConfig::N]>,
}

impl KernelTestHarness {
    /// Shared kernel construction: builds schedule, configs, kernel, and
    /// transitions all partitions to Running. The `peripheral_fn` callback
    /// supplies peripheral regions for each partition index.
    fn build_kernel(
        n: usize,
        mut peripheral_fn: impl FnMut(usize) -> Vec<MpuRegion, 2>,
    ) -> Result<Self, HarnessError> {
        if n == 0 || n > HarnessConfig::N {
            return Err(HarnessError::InvalidPartitionCount);
        }
        let mut schedule = ScheduleTable::new();
        for i in 0..n {
            schedule
                .add(ScheduleEntry::new(i as u8, 10))
                .map_err(|_| HarnessError::ScheduleFull)?;
        }
        #[cfg(feature = "dynamic-mpu")]
        schedule
            .add_system_window(10)
            .map_err(|_| HarnessError::ScheduleFull)?;
        let mut stacks = [AlignedStack1K::default(); HarnessConfig::N];
        let mut mems: Vec<ExternalPartitionMemory<'_>, { HarnessConfig::N }> = Vec::new();
        for (i, stack) in stacks.iter_mut().enumerate().take(n) {
            let o = (i as u32) * PARTITION_OFFSET;
            let mem = ExternalPartitionMemory::new(
                stack.as_u32_slice_mut(),
                FLASH_BASE + o,
                MpuRegion::new(RAM_BASE + o, STACK_SIZE_BYTES, 0),
                i as u8,
            )
            .map_err(HarnessError::KernelInit)?
            // TODO: reviewer false positive — with_peripheral_regions() copies MpuRegion values
            // into an owned Vec<MpuRegion, 2>; the temporary from peripheral_fn(i) only needs
            // to live for the duration of this statement, which it does.
            .with_peripheral_regions(&peripheral_fn(i));
            mems.push(mem).map_err(|_| HarnessError::ConfigsFull)?;
        }
        let mut kernel =
            Box::new(Kernel::new_external(schedule, &mems).map_err(HarnessError::KernelInit)?);
        // Only partition 0 starts Running; others remain Ready (at-most-one-Running invariant).
        kernel
            .partitions_mut()
            .get_mut(0)
            .ok_or(HarnessError::PartitionNotFound)?
            .transition(PartitionState::Running)
            .map_err(HarnessError::Transition)?;
        kernel.current_partition = 0;
        kernel.active_partition = Some(0);
        // Reconcile mpu_region bases with actual core stack addresses.
        // On 64-bit test hosts, addresses captured during Kernel::new become
        // stale when the struct moves. Boxing pins the kernel on the heap so
        // the stack addresses are stable for writing into the PCBs.
        drop(mems);
        let stacks = Box::new(stacks);
        for i in 0..n {
            let base = stacks[i].as_u32_slice().as_ptr() as u32;
            let sp = base.wrapping_add(STACK_SIZE_BYTES);
            kernel.set_sp(i, sp);
            assert!(
                kernel.fix_stack_region(i, base, STACK_SIZE_BYTES),
                "fix_stack_region failed for partition {i}"
            );
            assert!(
                kernel.fix_mpu_data_region(i, base),
                "fix_mpu_data_region failed for partition {i}"
            );
        }
        Ok(Self {
            kernel,
            _stacks: stacks,
        })
    }

    pub fn with_partitions(n: usize) -> Result<Self, HarnessError> {
        Self::build_kernel(n, |_| Vec::new())
    }

    /// Create a harness with two partitions and semaphores initialised at the
    /// given counts.  Each entry in `counts` adds one semaphore whose initial
    /// value is `count` and whose maximum is `count + 1`.
    pub fn with_semaphores(counts: &[u32]) -> Result<Self, HarnessError> {
        let mut h = Self::with_partitions(2)?;
        for &c in counts {
            h.kernel
                .semaphores_mut()
                .add(Semaphore::new(c, c + 1))
                .map_err(|_| HarnessError::ConfigsFull)?;
        }
        Ok(h)
    }

    /// Create a harness with two partitions suitable for mutex testing.
    ///
    /// Mutexes are pre-allocated at pool capacity in `SyncPools::default()`,
    /// so `count` documents how many the caller intends to use but does not
    /// change pool contents.
    pub fn with_mutexes(_count: usize) -> Result<Self, HarnessError> {
        Self::with_partitions(2)
    }

    /// Create a harness with two partitions suitable for event-flag testing.
    ///
    /// Event flags are per-partition fields, so no additional pool setup is
    /// needed.
    pub fn with_events() -> Result<Self, HarnessError> {
        Self::with_partitions(2)
    }

    /// Create a harness with two partitions where partition 0 has one
    /// peripheral region (0x4000_0000, size 256, permissions 0x03).
    ///
    /// This exercises the full peripheral-region pipeline: PartitionConfig →
    /// Kernel::new → PCB.with_peripheral_regions → accessor / MPU programming.
    pub fn with_peripheral_regions() -> Result<Self, HarnessError> {
        Self::build_kernel(2, |i| {
            let mut periph = Vec::new();
            if i == 0 {
                periph
                    .push(MpuRegion::new(0x4000_0000, 256, 0x03))
                    .expect("peripheral region vec has capacity");
            }
            periph
        })
    }

    /// Create a harness with two partitions where each has a distinct
    /// peripheral region: P0 has (0x4000_0000, 4096) and P1 has
    /// (0x4001_0000, 256).
    ///
    /// This exercises the full Kernel::new → PCB → MPU emission pipeline
    /// for multi-partition peripheral differentiation.
    pub fn with_two_peripheral_partitions() -> Result<Self, HarnessError> {
        Self::build_kernel(2, |i| {
            let mut periph = Vec::new();
            let (base, size) = match i {
                0 => (0x4000_0000, 4096),
                _ => (0x4001_0000, 256),
            };
            periph
                .push(MpuRegion::new(base, size, 0x03))
                .expect("peripheral region vec has capacity");
            periph
        })
    }

    /// Create a 2-partition harness with mixed MPU data regions:
    /// P0 uses the sentinel (size==0) and P1 uses a user-configured region.
    ///
    /// The post-move fixup replicates boot.rs's inline sentinel guard:
    /// only sentinel partitions (size==0) have their MPU base updated;
    /// user-configured partitions keep their original base address.
    pub fn with_mixed_mpu_regions() -> Result<Self, HarnessError> {
        let n = 2;
        let mut schedule = ScheduleTable::new();
        for i in 0..n {
            schedule
                .add(ScheduleEntry::new(i as u8, 10))
                .map_err(|_| HarnessError::ScheduleFull)?;
        }
        #[cfg(feature = "dynamic-mpu")]
        schedule
            .add_system_window(10)
            .map_err(|_| HarnessError::ScheduleFull)?;
        let mut stacks = Box::new([AlignedStack1K::default(); HarnessConfig::N]);
        let (head, tail) = stacks.split_at_mut(1);
        let mem0 = ExternalPartitionMemory::new(
            head[0].as_u32_slice_mut(),
            FLASH_BASE,
            MpuRegion::new(0, 0, 0),
            0,
        )
        .map_err(HarnessError::KernelInit)?;
        let mem1 = ExternalPartitionMemory::new(
            tail[0].as_u32_slice_mut(),
            FLASH_BASE + PARTITION_OFFSET,
            MpuRegion::new(0x2004_0000, 2048, 0x0306_0000),
            1,
        )
        .map_err(HarnessError::KernelInit)?;
        let mems = [mem0, mem1];
        let mut kernel =
            Box::new(Kernel::new_external(schedule, &mems).map_err(HarnessError::KernelInit)?);
        // Verify Kernel::new preserved P1's user-configured base before fixup.
        assert_eq!(
            kernel.partitions().get(1).map(|p| p.mpu_region().base()),
            Some(0x2004_0000),
            "Kernel::new must preserve user-configured mpu_region base for P1"
        );
        kernel
            .partitions_mut()
            .get_mut(0)
            .ok_or(HarnessError::PartitionNotFound)?
            .transition(PartitionState::Running)
            .map_err(HarnessError::Transition)?;
        kernel.current_partition = 0;
        kernel.active_partition = Some(0);
        // Post-move fixup: replicate boot.rs's inline sentinel guard.
        // Only sentinel partitions (size==0) get their MPU base updated;
        // user-configured partitions keep their original base.
        drop(mems);
        for i in 0..n {
            let base = stacks[i].as_u32_slice().as_ptr() as u32;
            let sp = base.wrapping_add(STACK_SIZE_BYTES);
            kernel.set_sp(i, sp);
            assert!(
                kernel.fix_stack_region(i, base, STACK_SIZE_BYTES),
                "fix_stack_region failed for partition {i}"
            );
            let is_sentinel = kernel
                .partitions()
                .get(i)
                .is_some_and(|p| p.mpu_region().size() == 0);
            if is_sentinel {
                assert!(
                    kernel.fix_mpu_data_region(i, base),
                    "fix_mpu_data_region failed for partition {i}"
                );
            }
        }
        Ok(Self {
            kernel,
            _stacks: stacks,
        })
    }

    /// Create a harness with two partitions and one message queue.
    ///
    /// The queue depth and message size come from `HarnessConfig` (QD=2, QM=4).
    pub fn with_messages() -> Result<Self, HarnessError> {
        let mut h = Self::with_partitions(2)?;
        h.kernel
            .messages_mut()
            .add(MessageQueue::new())
            .map_err(|_| HarnessError::ConfigsFull)?;
        Ok(h)
    }

    fn switch_to(&mut self, pid: usize) {
        let old = self.kernel.current_partition as usize;
        if old != pid {
            let parts = self.kernel.partitions();
            let deact = parts.get(old).map(|p| p.state()) == Some(PartitionState::Running);
            let act = parts.get(pid).map(|p| p.state()) == Some(PartitionState::Ready);
            if deact {
                let p = self.kernel.partitions_mut().get_mut(old).unwrap();
                p.transition(PartitionState::Ready).unwrap();
            }
            if act {
                let p = self.kernel.partitions_mut().get_mut(pid).unwrap();
                p.transition(PartitionState::Running).unwrap();
                self.kernel.active_partition = Some(pid as u8);
            } else if deact {
                self.kernel.active_partition = None;
            }
        }
        self.kernel.current_partition = pid as u8;
    }

    /// Assert all kernel invariants hold for the current kernel state.
    pub fn assert_invariants(&self) {
        let parts = self.kernel.partitions().as_slice();
        let active = self.kernel.active_partition;
        let mut sem_pairs = [(0u32, 0u32); HarnessConfig::S];
        let sem_len = (0..HarnessConfig::S)
            .map_while(|i| {
                self.kernel.semaphores().get(i).map(|s| {
                    sem_pairs[i] = (s.count(), s.max_count());
                })
            })
            .count();
        // When yield_requested is set, next_partition is stale (PendSV hasn't
        // advanced the schedule yet). Skip the next-partition check.
        let next = if self.kernel.yield_requested {
            None
        } else {
            Some(self.kernel.next_partition())
        };
        let sp = &self.kernel.partition_sp()[..parts.len()];
        crate::invariants::assert_kernel_invariants(parts, active, &sem_pairs[..sem_len], next, sp);
        crate::invariants::assert_storage_alignment(
            &*self.kernel as *const _ as usize,
            core::mem::align_of::<Kernel<HarnessConfig>>(),
        );
    }

    pub fn kernel(&self) -> &Kernel<HarnessConfig> {
        &self.kernel
    }

    pub fn kernel_mut(&mut self) -> &mut Kernel<HarnessConfig> {
        &mut self.kernel
    }

    /// Dispatch a syscall through the kernel and return the resulting frame.
    ///
    /// Creates an `ExceptionFrame` with `r0 = syscall`, `r1`, `r2`, `r3` set
    /// from the arguments, calls `kernel.dispatch()`, and returns the frame
    /// so callers can inspect the return value in `r0`.
    pub fn dispatch(&mut self, syscall: u32, r1: u32, r2: u32, r3: u32) -> ExceptionFrame {
        let mut frame = ExceptionFrame {
            r0: syscall,
            r1,
            r2,
            r3,
            r12: 0,
            lr: 0,
            pc: 0,
            xpsr: 0,
        };
        // SAFETY: stack-local frame, properly constructed kernel,
        // single-threaded test environment.
        unsafe { self.kernel.dispatch(&mut frame) };
        // Reconcile active_partition: blocking syscalls transition the current
        // partition to Waiting without updating active_partition (PendSV's job
        // in the real system). Since the harness has no PendSV, fix it here.
        let pid = self.kernel.current_partition as usize;
        if self.kernel.partitions().get(pid).map(|p| p.state()) != Some(PartitionState::Running) {
            self.kernel.active_partition = None;
        }
        self.assert_invariants();
        frame
    }

    /// Set `current_partition` to `pid` then dispatch a syscall.
    ///
    /// Useful for exercising syscalls as a specific partition without
    /// running the scheduler.
    pub fn dispatch_as(
        &mut self,
        pid: usize,
        syscall: u32,
        r1: u32,
        r2: u32,
        r3: u32,
    ) -> ExceptionFrame {
        self.switch_to(pid);
        self.dispatch(syscall, r1, r2, r3)
    }

    /// Assert that a blocking syscall correctly triggered a deschedule.
    ///
    /// Checks the invariant: if partition `pid` is in `PartitionState::Waiting`,
    /// then `yield_requested` must be `true`. Panics with a descriptive message
    /// if the invariant is violated.
    pub fn assert_blocking_triggered_deschedule(&self, pid: usize) {
        let partition = self
            .kernel
            .partitions()
            .get(pid)
            .unwrap_or_else(|| panic!("partition {pid} does not exist"));
        if partition.state() == PartitionState::Waiting {
            assert!(
                self.kernel.yield_requested,
                "invariant violation: partition {pid} is Waiting but yield_requested is false — \
                 the dispatch handler likely forgot to call trigger_deschedule()"
            );
        }
    }

    /// Assert that `next_partition` does not reference a `Waiting` partition.
    ///
    /// This invariant ensures the scheduler never selects a blocked partition
    /// for execution. If `next_partition` points to a partition in
    /// `PartitionState::Waiting`, it means the scheduler failed to skip
    /// a blocked partition and the system would switch to it on the next PendSV.
    pub fn assert_no_switch_to_waiting(&self) {
        let next = self.kernel.next_partition() as usize;
        let partition = self
            .kernel
            .partitions()
            .get(next)
            .unwrap_or_else(|| panic!("next_partition {next} does not exist"));
        assert_ne!(
            partition.state(),
            PartitionState::Waiting,
            "invariant violation: next_partition ({next}) is Waiting — \
             the scheduler failed to skip a blocked partition"
        );
    }

    /// Assert that every partition has a valid, unique stack base address.
    ///
    /// For each partition index `0..partition_count`:
    /// - `partitions().get(i)` returns `Some` (partition exists)
    /// - `pcb.stack_base()` is non-zero
    /// - The address is word-aligned (multiple of 4)
    /// - No two partitions share the same stack base address
    pub fn assert_stack_bases_valid(&self) {
        let n = self.kernel.partitions().len();
        let mut seen: std::vec::Vec<u32> = std::vec::Vec::with_capacity(n);
        const WORD_SIZE: usize = core::mem::size_of::<u32>();
        for i in 0..n {
            let pcb = self.kernel.partitions().get(i).unwrap_or_else(|| {
                panic!(
                    "invariant violation: partitions().get({i}) returned None \
                     but partition exists"
                )
            });
            let base = pcb.stack_base();
            assert_ne!(base, 0, "invariant violation: P{i} stack_base is zero");
            assert_eq!(
                base as usize % WORD_SIZE,
                0,
                "invariant violation: P{i} stack_base = {base:#x} is not word-aligned"
            );
            for (j, &prev) in seen.iter().enumerate() {
                assert_ne!(
                    base, prev,
                    "invariant violation: partitions {j} and {i} share \
                     stack base {base:#x}"
                );
            }
            seen.push(base);
        }
    }

    /// Assert that the syscall return value is consistent with blocking.
    ///
    /// If `blocked` is true, `frame.r0` must be 0 (the value returned by
    /// `trigger_deschedule()`). Panics with a descriptive message if a
    /// blocking path returned a non-zero value.
    pub fn assert_return_distinguishes_blocking(&self, frame: &ExceptionFrame, blocked: bool) {
        if blocked {
            assert_eq!(
                frame.r0, 0,
                "invariant violation: syscall blocked but frame.r0 is {} instead of 0 — \
                 the return value must come from trigger_deschedule() which always returns 0",
                frame.r0
            );
        }
    }

    /// Process a pending yield request, mirroring PendSV's role on real hardware.
    ///
    /// If `yield_requested` is true, clears it, advances the schedule via
    /// `yield_current_slot()`, and if the result selects a new partition,
    /// calls `set_next_partition()`.  Finishes by asserting invariants.
    /// If `yield_requested` is false, this is a no-op.
    pub fn process_pending_yield(&mut self) {
        if !self.kernel.yield_requested {
            return;
        }
        self.kernel.yield_requested = false;
        let result = self.kernel.yield_current_slot();
        if let Some(pid) = result.partition_id() {
            self.kernel.set_next_partition(pid);
        }
        self.assert_invariants();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scheduler::ScheduleEvent;
    use crate::svc::validate_user_ptr;
    use crate::syscall::{
        SYS_EVT_SET, SYS_EVT_WAIT, SYS_MSG_RECV, SYS_MSG_SEND, SYS_MTX_LOCK, SYS_MTX_UNLOCK,
        SYS_SEM_SIGNAL, SYS_SEM_WAIT, SYS_YIELD,
    };

    fn low32_buf(page: usize) -> *mut u8 {
        crate::test_mmap::low32_buf(page)
    }

    #[test]
    fn two_partitions_count_and_state() {
        let h = KernelTestHarness::with_partitions(2).expect("harness setup");
        let st = |i| h.kernel().partitions().get(i).unwrap().state();
        assert_eq!(h.kernel().partitions().len(), 2);
        assert_eq!(h.kernel().current_partition, 0);
        assert_eq!(st(0), PartitionState::Running);
        assert_eq!(st(1), PartitionState::Ready);
    }

    #[test]
    fn four_partitions_count_and_state() {
        let h = KernelTestHarness::with_partitions(4).expect("harness setup");
        let st = |i| h.kernel().partitions().get(i).unwrap().state();
        assert_eq!(h.kernel().partitions().len(), 4);
        assert_eq!(st(0), PartitionState::Running);
        for i in 1..4 {
            assert_eq!(st(i), PartitionState::Ready);
        }
    }

    #[test]
    fn kernel_mut_accessor() {
        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");
        h.kernel_mut().current_partition = 1;
        assert_eq!(h.kernel().current_partition, 1);
    }

    #[test]
    fn dispatch_yield_returns_zero_and_sets_yield_requested() {
        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");
        assert!(!h.kernel().yield_requested);

        let frame = h.dispatch(SYS_YIELD, 0, 0, 0);

        assert_eq!(frame.r0, 0, "SYS_YIELD must return 0");
        assert!(
            h.kernel().yield_requested,
            "yield_requested must be set after SYS_YIELD"
        );
    }

    #[test]
    fn dispatch_as_changes_partition_and_dispatches() {
        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");
        assert_eq!(h.kernel().current_partition, 0);

        let frame = h.dispatch_as(1, SYS_YIELD, 0, 0, 0);

        assert_eq!(
            h.kernel().current_partition,
            1,
            "dispatch_as must set current_partition to the given pid"
        );
        assert_eq!(frame.r0, 0, "SYS_YIELD must return 0");
        assert!(
            h.kernel().yield_requested,
            "yield_requested must be set after SYS_YIELD via dispatch_as"
        );
    }

    #[test]
    fn with_semaphores_creates_semaphore_at_given_count() {
        let mut h = KernelTestHarness::with_semaphores(&[3]).expect("harness setup");
        // SYS_SEM_WAIT: r1 = sem_id(0), r2 = caller_pid(0)
        // Semaphore has initial count 3, so wait should succeed (count → 2).
        let frame = h.dispatch(SYS_SEM_WAIT, 0, 0, 0);
        assert_eq!(
            frame.r0, 1,
            "SYS_SEM_WAIT on count-3 semaphore must return 1 (acquired)"
        );
        assert!(
            !h.kernel().yield_requested,
            "non-blocking wait must not trigger deschedule"
        );
        let sem = h.kernel().semaphores().get(0).expect("semaphore 0 exists");
        assert_eq!(
            sem.count(),
            2,
            "count must decrement from 3 to 2 after wait"
        );
    }

    #[test]
    fn dispatch_sem_wait_blocking_deschedules_and_returns_zero() {
        // Create a semaphore with initial count 0 so the first wait blocks.
        let mut h = KernelTestHarness::with_semaphores(&[0]).expect("harness setup");
        let frame = h.dispatch(SYS_SEM_WAIT, 0, 0, 0);
        assert_eq!(frame.r0, 0, "blocking SYS_SEM_WAIT must return 0");
        assert!(
            h.kernel().yield_requested,
            "blocking wait must trigger deschedule"
        );
        assert_eq!(
            h.kernel().partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "partition must transition to Waiting when blocked on semaphore"
        );
    }

    #[test]
    fn dispatch_sem_wait_nonblocking_acquires_and_returns_one() {
        // Signal the semaphore so count > 0, then wait should acquire immediately.
        let mut h = KernelTestHarness::with_semaphores(&[0]).expect("harness setup");
        // Signal to raise count from 0 → 1.
        let sig_frame = h.dispatch(SYS_SEM_SIGNAL, 0, 0, 0);
        assert_eq!(sig_frame.r0, 0, "SYS_SEM_SIGNAL must succeed");
        let sem = h.kernel().semaphores().get(0).expect("semaphore 0 exists");
        assert_eq!(sem.count(), 1, "count must be 1 after signal");

        // Non-blocking wait: count 1 → 0.
        let frame = h.dispatch(SYS_SEM_WAIT, 0, 0, 0);
        assert_eq!(
            frame.r0, 1,
            "non-blocking SYS_SEM_WAIT must return 1 (acquired)"
        );
        assert!(
            !h.kernel().yield_requested,
            "non-blocking wait must not trigger deschedule"
        );
        assert_eq!(
            h.kernel().partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "partition must remain Running after non-blocking wait"
        );
        let sem = h.kernel().semaphores().get(0).expect("semaphore 0 exists");
        assert_eq!(sem.count(), 0, "count must decrement to 0 after wait");
    }

    #[test]
    fn with_mutexes_allows_lock_unlock() {
        let mut h = KernelTestHarness::with_mutexes(1).expect("harness setup");
        // SYS_MTX_LOCK: r1 = mutex_id(0), r2 = caller_pid(0)
        let frame = h.dispatch(SYS_MTX_LOCK, 0, 0, 0);
        assert_eq!(frame.r0, 1, "MTX_LOCK must return 1 (acquired)");
        assert_eq!(
            h.kernel().mutexes().owner(0).unwrap(),
            Some(0),
            "partition 0 must own mutex 0 after lock"
        );
        // SYS_MTX_UNLOCK: r1 = mutex_id(0), r2 = caller_pid(0)
        let frame = h.dispatch(SYS_MTX_UNLOCK, 0, 0, 0);
        assert_eq!(frame.r0, 0, "MTX_UNLOCK must return 0 (success)");
        assert_eq!(
            h.kernel().mutexes().owner(0).unwrap(),
            None,
            "mutex 0 must be unowned after unlock"
        );
    }

    // ------------------------------------------------------------------
    // IPC return-value discipline: every dispatch result is asserted
    // ------------------------------------------------------------------

    #[test]
    fn sem_signal_invalid_id_returns_error() {
        let mut h = KernelTestHarness::with_semaphores(&[1]).expect("harness setup");
        // Signal semaphore ID 99, which does not exist.
        let frame = h.dispatch(SYS_SEM_SIGNAL, 99, 0, 0);
        assert_eq!(
            frame.r0, 0xFFFF_FFFE,
            "SEM_SIGNAL on invalid ID must return InvalidResource (0xFFFF_FFFE)"
        );
    }

    #[test]
    fn sem_signal_valid_id_returns_ok() {
        let mut h = KernelTestHarness::with_semaphores(&[0]).expect("harness setup");
        // Signal semaphore 0 — valid ID, must return 0 (Ok).
        let frame = h.dispatch(SYS_SEM_SIGNAL, 0, 0, 0);
        assert_eq!(frame.r0, 0, "SEM_SIGNAL on valid ID must return 0 (Ok)");
        let sem = h.kernel().semaphores().get(0).expect("semaphore 0 exists");
        assert_eq!(
            sem.count(),
            1,
            "count must increment from 0 to 1 after signal"
        );
    }

    #[test]
    fn sem_wait_available_returns_ok() {
        let mut h = KernelTestHarness::with_semaphores(&[1]).expect("harness setup");
        // Semaphore has count 1, so wait should acquire immediately.
        let frame = h.dispatch(SYS_SEM_WAIT, 0, 0, 0);
        assert_eq!(
            frame.r0, 1,
            "SEM_WAIT on available semaphore must return 1 (Ok/acquired)"
        );
        let sem = h.kernel().semaphores().get(0).expect("semaphore 0 exists");
        assert_eq!(
            sem.count(),
            0,
            "count must decrement from 1 to 0 after wait"
        );
    }

    #[test]
    fn sem_wait_unavailable_blocks() {
        let mut h = KernelTestHarness::with_semaphores(&[0]).expect("harness setup");
        // Semaphore has count 0, so wait must block (WouldBlock/0).
        let frame = h.dispatch(SYS_SEM_WAIT, 0, 0, 0);
        assert_eq!(
            frame.r0, 0,
            "SEM_WAIT on unavailable semaphore must return 0 (WouldBlock)"
        );
        assert_eq!(
            h.kernel().partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "partition must transition to Waiting when semaphore unavailable"
        );
    }

    #[test]
    fn mtx_lock_already_held_by_other_blocks() {
        let mut h = KernelTestHarness::with_mutexes(1).expect("harness setup");
        // P0 locks mutex 0.
        let lock_frame = h.dispatch_as(0, SYS_MTX_LOCK, 0, 0, 0);
        assert_eq!(lock_frame.r0, 1, "P0 MTX_LOCK must return 1 (acquired)");
        // Reset yield_requested before P1 dispatch.
        h.kernel_mut().yield_requested = false;
        // P1 tries to lock the same mutex — must block.
        let frame = h.dispatch_as(1, SYS_MTX_LOCK, 0, 1, 0);
        assert_eq!(
            frame.r0, 0,
            "MTX_LOCK by P1 on P0-held mutex must return 0 (blocked)"
        );
        assert_eq!(
            h.kernel().partitions().get(1).unwrap().state(),
            PartitionState::Waiting,
            "P1 must transition to Waiting when mutex is held by P0"
        );
    }

    #[test]
    fn mtx_lock_already_owned_returns_error() {
        let mut h = KernelTestHarness::with_mutexes(1).expect("harness setup");
        // P0 locks mutex 0.
        let lock_frame = h.dispatch(SYS_MTX_LOCK, 0, 0, 0);
        assert_eq!(lock_frame.r0, 1, "first MTX_LOCK must return 1 (acquired)");
        // P0 tries to lock it again — AlreadyOwned error.
        let frame = h.dispatch(SYS_MTX_LOCK, 0, 0, 0);
        assert_eq!(
            frame.r0, 0xFFFF_FFFA,
            "MTX_LOCK by owner must return OperationFailed (0xFFFF_FFFA)"
        );
    }

    #[test]
    fn mtx_unlock_non_owner_returns_error() {
        let mut h = KernelTestHarness::with_mutexes(1).expect("harness setup");
        // P0 locks mutex 0.
        let lock_frame = h.dispatch_as(0, SYS_MTX_LOCK, 0, 0, 0);
        assert_eq!(lock_frame.r0, 1, "P0 MTX_LOCK must return 1 (acquired)");
        // P1 tries to unlock it — NotOwner error.
        let frame = h.dispatch_as(1, SYS_MTX_UNLOCK, 0, 1, 0);
        assert_eq!(
            frame.r0, 0xFFFF_FFFA,
            "MTX_UNLOCK by non-owner must return OperationFailed (0xFFFF_FFFA)"
        );
        // Verify P0 still owns the mutex.
        assert_eq!(
            h.kernel().mutexes().owner(0).unwrap(),
            Some(0),
            "P0 must still own mutex 0 after non-owner unlock attempt"
        );
    }

    #[test]
    fn mtx_unlock_by_owner_returns_ok() {
        let mut h = KernelTestHarness::with_mutexes(1).expect("harness setup");
        // P0 locks mutex 0.
        let lock_frame = h.dispatch(SYS_MTX_LOCK, 0, 0, 0);
        assert_eq!(lock_frame.r0, 1, "MTX_LOCK must return 1 (acquired)");
        // P0 unlocks — owner unlock must return 0 (Ok).
        let frame = h.dispatch(SYS_MTX_UNLOCK, 0, 0, 0);
        assert_eq!(frame.r0, 0, "MTX_UNLOCK by owner must return 0 (Ok)");
        assert_eq!(
            h.kernel().mutexes().owner(0).unwrap(),
            None,
            "mutex must be unowned after owner unlock"
        );
    }

    #[test]
    fn with_events_allows_event_set() {
        let mut h = KernelTestHarness::with_events().expect("harness setup");
        // SYS_EVT_SET: r1 = target_pid(1), r2 = mask(0b0101)
        let frame = h.dispatch(SYS_EVT_SET, 1, 0b0101, 0);
        assert_eq!(frame.r0, 0, "EVT_SET must return 0 (success)");
        assert_eq!(
            h.kernel().partitions().get(1).unwrap().event_flags(),
            0b0101,
            "target partition must have event flags set"
        );
    }

    // ------------------------------------------------------------------
    // assert_blocking_triggered_deschedule
    // ------------------------------------------------------------------

    #[test]
    fn assert_blocking_deschedule_passes_when_waiting_and_yield_requested() {
        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");
        // Transition partition 0: Running → Waiting
        h.kernel_mut()
            .partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();
        h.kernel_mut().yield_requested = true;
        // Should not panic
        h.assert_blocking_triggered_deschedule(0);
    }

    #[test]
    fn assert_blocking_deschedule_passes_when_not_waiting() {
        let h = KernelTestHarness::with_partitions(2).expect("harness setup");
        // Partition 0 is Running, yield_requested is false — no check needed
        h.assert_blocking_triggered_deschedule(0);
    }

    #[test]
    #[should_panic(expected = "forgot to call trigger_deschedule()")]
    fn assert_blocking_deschedule_panics_when_waiting_without_yield() {
        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");
        // Transition partition 0: Running → Waiting
        h.kernel_mut()
            .partitions_mut()
            .get_mut(0)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();
        // yield_requested is still false — invariant violated
        h.assert_blocking_triggered_deschedule(0);
    }

    // ------------------------------------------------------------------
    // assert_return_distinguishes_blocking
    // ------------------------------------------------------------------

    #[test]
    fn assert_return_blocking_passes_when_r0_zero_and_blocked() {
        let h = KernelTestHarness::with_partitions(2).expect("harness setup");
        let frame = ExceptionFrame {
            r0: 0,
            r1: 0,
            r2: 0,
            r3: 0,
            r12: 0,
            lr: 0,
            pc: 0,
            xpsr: 0,
        };
        h.assert_return_distinguishes_blocking(&frame, true);
    }

    #[test]
    fn assert_return_blocking_passes_when_not_blocked() {
        let h = KernelTestHarness::with_partitions(2).expect("harness setup");
        let frame = ExceptionFrame {
            r0: 42,
            r1: 0,
            r2: 0,
            r3: 0,
            r12: 0,
            lr: 0,
            pc: 0,
            xpsr: 0,
        };
        // Non-blocking path — no constraint on r0
        h.assert_return_distinguishes_blocking(&frame, false);
    }

    #[test]
    #[should_panic(expected = "trigger_deschedule() which always returns 0")]
    fn assert_return_blocking_panics_when_r0_nonzero_and_blocked() {
        let h = KernelTestHarness::with_partitions(2).expect("harness setup");
        let frame = ExceptionFrame {
            r0: 1,
            r1: 0,
            r2: 0,
            r3: 0,
            r12: 0,
            lr: 0,
            pc: 0,
            xpsr: 0,
        };
        h.assert_return_distinguishes_blocking(&frame, true);
    }

    // ------------------------------------------------------------------
    // EventWait → EventSet full blocking/wake cycle
    // ------------------------------------------------------------------

    #[test]
    fn event_wait_blocks_then_event_set_wakes() {
        let mask: u32 = 0b1010;
        let mut h = KernelTestHarness::with_events().expect("harness setup");

        // Step 1: P0 dispatches SYS_EVT_WAIT — no flags set, so it blocks.
        // SYS_EVT_WAIT encoding: r1 = caller_pid, r2 = mask
        let wait_frame = h.dispatch_as(0, SYS_EVT_WAIT, 0, mask, 0);

        // P0 must now be Waiting.
        assert_eq!(
            h.kernel().partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must transition to Waiting when no event flags match"
        );

        // Validate deschedule invariant.
        h.assert_blocking_triggered_deschedule(0);

        // Validate return value is 0 for blocking path.
        h.assert_return_distinguishes_blocking(&wait_frame, true);

        // Step 2: P1 dispatches SYS_EVT_SET targeting P0 — wakes P0.
        // SYS_EVT_SET encoding: r1 = target_pid, r2 = mask
        let set_frame = h.dispatch_as(1, SYS_EVT_SET, 0, mask, 0);
        assert_eq!(set_frame.r0, 0, "EVT_SET must return 0 (success)");

        // Step 3: Verify P0 is now Ready and event flags contain the set bits.
        assert_eq!(
            h.kernel().partitions().get(0).unwrap().state(),
            PartitionState::Ready,
            "P0 must transition from Waiting to Ready after EVT_SET with matching mask"
        );
        assert_ne!(
            h.kernel().partitions().get(0).unwrap().event_flags() & mask,
            0,
            "P0 event flags must contain the bits set by P1"
        );
    }

    #[test]
    fn sem_wait_blocks_then_sem_signal_wakes() {
        // Semaphore with initial count 0 so the first wait blocks.
        let mut h = KernelTestHarness::with_semaphores(&[0]).expect("harness setup");

        // Step 1: P0 dispatches SYS_SEM_WAIT — count is 0, so it blocks.
        // SYS_SEM_WAIT encoding: r1 = sem_id, r2 = caller_pid
        let wait_frame = h.dispatch_as(0, SYS_SEM_WAIT, 0, 0, 0);

        // P0 must now be Waiting.
        assert_eq!(
            h.kernel().partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must transition to Waiting when semaphore count is 0"
        );

        // Validate deschedule invariant.
        h.assert_blocking_triggered_deschedule(0);

        // Validate return value is 0 for blocking path.
        h.assert_return_distinguishes_blocking(&wait_frame, true);

        // Step 2: P1 dispatches SYS_SEM_SIGNAL on the same semaphore — wakes P0.
        // SYS_SEM_SIGNAL encoding: r1 = sem_id
        let sig_frame = h.dispatch_as(1, SYS_SEM_SIGNAL, 0, 0, 0);
        assert_eq!(sig_frame.r0, 0, "SEM_SIGNAL must return 0 (success)");

        // Step 3: Verify P0 transitioned from Waiting to Ready.
        assert_eq!(
            h.kernel().partitions().get(0).unwrap().state(),
            PartitionState::Ready,
            "P0 must transition from Waiting to Ready after SEM_SIGNAL"
        );

        // Semaphore count must remain 0: the signal woke a waiter
        // rather than incrementing the count.
        let sem = h.kernel().semaphores().get(0).expect("semaphore 0 exists");
        assert_eq!(
            sem.count(),
            0,
            "semaphore count must stay 0 when signal wakes a blocked waiter"
        );
    }

    #[test]
    fn mtx_lock_blocks_then_mtx_unlock_wakes_with_ownership_transfer() {
        let mut h = KernelTestHarness::with_mutexes(1).expect("harness setup");

        // Step 1: P1 dispatches SYS_MTX_LOCK — mutex is free, so it acquires.
        // SYS_MTX_LOCK encoding: r1 = mutex_id, r2 = caller_pid
        let lock_frame = h.dispatch_as(1, SYS_MTX_LOCK, 0, 1, 0);
        assert_eq!(lock_frame.r0, 1, "MTX_LOCK must return 1 (acquired)");
        assert_eq!(
            h.kernel().mutexes().owner(0).unwrap(),
            Some(1),
            "P1 must own mutex 0 after successful lock"
        );

        // Step 2: P0 dispatches SYS_MTX_LOCK on same mutex — P1 holds it, so P0 blocks.
        let wait_frame = h.dispatch_as(0, SYS_MTX_LOCK, 0, 0, 0);

        // P0 must now be Waiting.
        assert_eq!(
            h.kernel().partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must transition to Waiting when mutex is held by P1"
        );

        // Validate deschedule invariant.
        h.assert_blocking_triggered_deschedule(0);

        // Validate return value is 0 for blocking path.
        h.assert_return_distinguishes_blocking(&wait_frame, true);

        // Mutex must still be owned by P1 while P0 waits.
        assert_eq!(
            h.kernel().mutexes().owner(0).unwrap(),
            Some(1),
            "P1 must still own mutex 0 while P0 is blocked"
        );

        // Step 3: P1 dispatches SYS_MTX_UNLOCK — wakes P0 and transfers ownership.
        let unlock_frame = h.dispatch_as(1, SYS_MTX_UNLOCK, 0, 1, 0);
        assert_eq!(unlock_frame.r0, 0, "MTX_UNLOCK must return 0 (success)");

        // Verify P0 transitioned from Waiting to Ready.
        assert_eq!(
            h.kernel().partitions().get(0).unwrap().state(),
            PartitionState::Ready,
            "P0 must transition from Waiting to Ready after MTX_UNLOCK"
        );

        // Verify ownership transferred to P0.
        assert_eq!(
            h.kernel().mutexes().owner(0).unwrap(),
            Some(0),
            "P0 must own mutex 0 after ownership transfer from P1 unlock"
        );
    }

    // ------------------------------------------------------------------
    // MsgRecv → MsgSend full blocking/wake cycle
    // ------------------------------------------------------------------

    #[test]
    fn with_messages_creates_queue() {
        let h = KernelTestHarness::with_messages().expect("harness setup");
        assert!(
            h.kernel().messages().get(0).is_some(),
            "message queue 0 must exist after with_messages()"
        );
    }

    #[test]
    fn msg_recv_blocks_then_msg_send_wakes() {
        let mut h = KernelTestHarness::with_messages().expect("harness setup");

        // Fix MPU data region bases for 64-bit test hosts: Kernel::new derives
        // mpu_region from internal stack bases, which are heap addresses on the host.
        // Reset them to the low-address range that validated_ptr! expects.
        let buf_ptr = low32_buf(0);
        for i in 0..2 {
            let base = 0x2000_0000 + (i as u32) * 0x1000;
            h.kernel_mut()
                .partitions_mut()
                .get_mut(i)
                .unwrap()
                .fix_mpu_data_region(base);
        }

        // Step 1: P0 dispatches SYS_MSG_RECV on empty queue — blocks.
        // SYS_MSG_RECV encoding: r1=queue_id, r2=caller_pid, r3=buf_ptr
        let recv_frame = h.dispatch_as(0, SYS_MSG_RECV, 0, 0, buf_ptr as u32);

        // P0 must now be Waiting.
        assert_eq!(
            h.kernel().partitions().get(0).unwrap().state(),
            PartitionState::Waiting,
            "P0 must transition to Waiting when queue is empty"
        );

        // Validate deschedule invariant (yield_requested == true).
        h.assert_blocking_triggered_deschedule(0);

        // Validate return value: kernel sets r0=0 on blocking path.
        h.assert_return_distinguishes_blocking(&recv_frame, true);

        // Reset yield_requested so the send dispatch starts clean.
        h.kernel_mut().yield_requested = false;
        // P0 is now in Waiting state after the blocking receive.  The
        // dispatch-exit invariant requires next_partition to reference a
        // non-Waiting partition, so advance it to P1 (the sender) which
        // is Running.
        h.kernel_mut().core.set_next_partition(1);

        // Step 2: P1 dispatches SYS_MSG_SEND on queue 0 — wakes P0.
        // SYS_MSG_SEND encoding: r1=queue_id, r2=sender_pid, r3=data_ptr
        // Use partition 1's MPU region (page 1) for the send pointer.
        let send_ptr = low32_buf(1);
        let data = [0xAB_u8; 4];
        // SAFETY: send_ptr is valid for 4096 bytes via low32_buf.
        unsafe { core::ptr::copy_nonoverlapping(data.as_ptr(), send_ptr, 4) };
        let send_frame = h.dispatch_as(1, SYS_MSG_SEND, 0, 1, send_ptr as u32);
        assert_eq!(send_frame.r0, 0, "SYS_MSG_SEND must return 0 (success)");

        // Step 3: Verify P0 transitioned Waiting → Ready.
        assert_eq!(
            h.kernel().partitions().get(0).unwrap().state(),
            PartitionState::Ready,
            "P0 must transition from Waiting to Ready after MsgSend wakes it"
        );
    }

    // ------------------------------------------------------------------
    // Cross-primitive: semaphore block + event set + semaphore wake
    // ------------------------------------------------------------------

    #[test]
    fn sem_block_then_event_set_does_not_wake_then_sem_signal_wakes() {
        // Semaphore with initial count 1 so P0 can acquire, then P1 blocks.
        let mut h = KernelTestHarness::with_semaphores(&[1]).expect("harness setup");
        let event_mask: u32 = 0b0110;

        // Step 1: P0 acquires the semaphore (count 1 → 0, non-blocking).
        let acq_frame = h.dispatch_as(0, SYS_SEM_WAIT, 0, 0, 0);
        assert_eq!(acq_frame.r0, 1, "P0 SEM_WAIT must return 1 (acquired)");
        assert_eq!(
            h.kernel().partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "P0 must remain Running after non-blocking acquire"
        );
        assert!(
            !h.kernel().yield_requested,
            "non-blocking acquire must not trigger deschedule"
        );
        let sem = h.kernel().semaphores().get(0).expect("sem 0");
        assert_eq!(sem.count(), 0, "semaphore count must be 0 after acquire");

        // Step 2: P1 waits on the same semaphore (count 0 → blocks).
        let wait_frame = h.dispatch_as(1, SYS_SEM_WAIT, 0, 1, 0);
        assert_eq!(wait_frame.r0, 0, "P1 SEM_WAIT must return 0 (blocked)");
        assert_eq!(
            h.kernel().partitions().get(1).unwrap().state(),
            PartitionState::Waiting,
            "P1 must transition to Waiting on empty semaphore"
        );
        h.assert_blocking_triggered_deschedule(1);
        h.assert_return_distinguishes_blocking(&wait_frame, true);

        // Reset yield_requested so subsequent steps start clean.
        h.kernel_mut().yield_requested = false;

        // Step 3: P0 sets event flags on P1 while P1 is blocked on semaphore.
        // P1 is NOT waiting on events, so this must NOT wake P1.
        let set_frame = h.dispatch_as(0, SYS_EVT_SET, 1, event_mask, 0);
        assert_eq!(set_frame.r0, 0, "EVT_SET must return 0 (success)");

        // Verify event flags are stored on P1.
        assert_eq!(
            h.kernel().partitions().get(1).unwrap().event_flags() & event_mask,
            event_mask,
            "P1 event flags must contain bits set by P0"
        );

        // P1 must STILL be Waiting (blocked on semaphore, not on events).
        assert_eq!(
            h.kernel().partitions().get(1).unwrap().state(),
            PartitionState::Waiting,
            "P1 must remain Waiting — event_set must not wake a semaphore-blocked partition"
        );

        // yield_requested must still be false (EVT_SET is non-blocking for caller).
        assert!(
            !h.kernel().yield_requested,
            "EVT_SET must not trigger deschedule on the caller"
        );

        // Step 4: P0 signals the semaphore — wakes P1.
        let sig_frame = h.dispatch_as(0, SYS_SEM_SIGNAL, 0, 0, 0);
        assert_eq!(sig_frame.r0, 0, "SEM_SIGNAL must return 0 (success)");

        // P1 must transition from Waiting to Ready.
        assert_eq!(
            h.kernel().partitions().get(1).unwrap().state(),
            PartitionState::Ready,
            "P1 must transition from Waiting to Ready after SEM_SIGNAL"
        );

        // Semaphore count must remain 0 (signal woke a waiter, didn't increment).
        let sem = h.kernel().semaphores().get(0).expect("sem 0");
        assert_eq!(
            sem.count(),
            0,
            "semaphore count must stay 0 when signal wakes a blocked waiter"
        );

        // Event flags must still be intact after the semaphore wake.
        assert_eq!(
            h.kernel().partitions().get(1).unwrap().event_flags() & event_mask,
            event_mask,
            "P1 event flags must survive the semaphore wake — primitives are independent"
        );
    }

    // Tick-path alignment check

    /// Verify the alignment check that `_unified_handle_tick!` performs at
    /// every tick does not panic on a properly-aligned kernel over 20 ticks.
    ///
    /// The macro itself cannot be invoked in host tests because it calls
    /// `cortex_m::peripheral::SCB::set_pendsv()`.  This test mirrors the
    /// macro's check sequence: call `assert_storage_alignment` before
    /// `advance_schedule_tick()` on every tick.
    ///
    /// Uses `align_of::<Kernel<HarnessConfig>>()` instead of `KERNEL_ALIGNMENT`
    /// because the test harness Box-allocates the kernel, which only
    /// guarantees the type's natural alignment (not linker-placed 4096).
    /// The production macro uses `KERNEL_ALIGNMENT` with linker-placed storage.
    #[test]
    fn tick_path_alignment_check_no_panic() {
        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");
        let align = core::mem::align_of::<Kernel<HarnessConfig>>();
        crate::svc::scheduler::start_schedule(h.kernel_mut());
        for tick in 0..20 {
            // Mirror _unified_handle_tick! macro body: check alignment
            // before advancing the schedule on every tick.
            let addr = h.kernel() as *const _ as usize;
            crate::invariants::assert_storage_alignment(addr, align);

            let event = crate::svc::scheduler::advance_schedule_tick(h.kernel_mut());
            if let ScheduleEvent::PartitionSwitch(pid) = event {
                assert!((pid as usize) < 2, "tick {tick}: invalid pid {pid}");
            }
        }
    }

    // ------------------------------------------------------------------
    // assert_no_switch_to_waiting — scheduler invariant tests
    // ------------------------------------------------------------------

    #[test]
    fn advance_schedule_skips_waiting() {
        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");
        crate::svc::scheduler::start_schedule(h.kernel_mut());

        // Transition P1 to Waiting (simulating a blocking syscall).
        h.switch_to(1);
        h.kernel_mut()
            .partitions_mut()
            .get_mut(1)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();

        // Advance 10 ticks to cross into P1's slot (each slot is 10 ticks).
        for i in 0..10 {
            let event = crate::svc::scheduler::advance_schedule_tick(h.kernel_mut());
            // The scheduler must not emit a PartitionSwitch to the Waiting P1.
            assert_ne!(
                event,
                ScheduleEvent::PartitionSwitch(1),
                "tick {i}: scheduler must not switch to Waiting P1"
            );
        }

        // The invariant must hold: next_partition must not point to Waiting P1.
        h.assert_no_switch_to_waiting();
    }

    #[test]
    fn yield_current_slot_skips_waiting() {
        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");
        crate::svc::scheduler::start_schedule(h.kernel_mut());

        // Transition P1 to Waiting.
        h.switch_to(1);
        h.kernel_mut()
            .partitions_mut()
            .get_mut(1)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();

        // Yield from P0's slot — scheduler should skip Waiting P1.
        let event = h.kernel_mut().yield_current_slot();
        assert_eq!(
            event.partition_id(),
            None,
            "yield must not produce a partition switch to Waiting P1"
        );

        // The invariant must hold.
        h.assert_no_switch_to_waiting();
    }

    #[test]
    #[should_panic(expected = "scheduler failed to skip a blocked partition")]
    fn assert_no_switch_to_waiting_fires_on_waiting_next() {
        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");

        // Transition P1 to Waiting.
        h.switch_to(1);
        h.kernel_mut()
            .partitions_mut()
            .get_mut(1)
            .unwrap()
            .transition(PartitionState::Waiting)
            .unwrap();

        // Manually set next_partition to the Waiting P1 (bypassing scheduler logic).
        h.kernel_mut().core.set_next_partition(1);

        // This must panic.
        h.assert_no_switch_to_waiting();
    }

    // ----- PCB stack_base accessor tests -----

    #[test]
    fn pcb_stack_base_returns_valid_addresses() {
        let h = KernelTestHarness::with_partitions(3).expect("harness setup");
        for i in 0..3 {
            let pcb = h.kernel().partitions().get(i);
            assert!(pcb.is_some(), "partition {i} should exist");
            let addr = pcb.unwrap().stack_base();
            assert_ne!(addr, 0, "partition {i} stack base must be non-zero");
            assert_eq!(addr % 4, 0, "partition {i} stack base must be word-aligned");
        }
    }

    #[test]
    fn partitions_get_returns_none_for_out_of_bounds() {
        let h = KernelTestHarness::with_partitions(2).expect("harness setup");
        assert!(h.kernel().partitions().get(HarnessConfig::N).is_none());
        assert!(h
            .kernel()
            .partitions()
            .get(HarnessConfig::N + 100)
            .is_none());
    }

    // ----- stack_bases_valid invariant tests -----

    #[test]
    fn stack_bases_valid_after_construction() {
        let h = KernelTestHarness::with_partitions(4).expect("harness setup");
        h.assert_stack_bases_valid();
    }

    // ------------------------------------------------------------------
    // PCB mpu_region matches stack base (regression for commit 2675853)
    // ------------------------------------------------------------------

    #[test]
    fn pcb_mpu_region_base_matches_stack_base() {
        for &n in &[1, 2, 4] {
            let h = KernelTestHarness::with_partitions(n).expect("harness setup");
            for i in 0..n {
                let pcb = h.kernel().partitions().get(i).expect("partition exists");
                assert_eq!(
                    pcb.mpu_region().base(),
                    pcb.stack_base(),
                    "n={n} partition {i}: mpu_region().base() must equal pcb.stack_base()"
                );
                assert_eq!(
                    pcb.mpu_region().size(),
                    STACK_SIZE_BYTES,
                    "n={n} partition {i}: mpu_region().size() must equal STACK_SIZE_BYTES"
                );
            }
        }
    }

    // ------------------------------------------------------------------
    // fix_stack_region out-of-bounds edge case
    // ------------------------------------------------------------------

    #[test]
    fn fix_stack_region_returns_false_for_out_of_bounds() {
        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");
        assert!(
            !h.kernel_mut().fix_stack_region(2, 0x2000_0000, 1024),
            "index == partition_count must return false"
        );
        assert!(
            !h.kernel_mut().fix_stack_region(100, 0x2000_0000, 1024),
            "large out-of-bounds index must return false"
        );
    }

    // ------------------------------------------------------------------
    // fix_mpu_data_region out-of-bounds edge case
    // ------------------------------------------------------------------

    #[test]
    fn fix_mpu_data_region_returns_false_for_out_of_bounds() {
        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");
        assert!(
            !h.kernel_mut().fix_mpu_data_region(2, 0x2000_0000),
            "index == partition_count must return false"
        );
        assert!(
            !h.kernel_mut().fix_mpu_data_region(100, 0x2000_0000),
            "large out-of-bounds index must return false"
        );
    }

    // ------------------------------------------------------------------
    // fix_mpu_data_region valid index: returns true, updates base,
    // preserves size and permissions
    // ------------------------------------------------------------------

    #[test]
    fn fix_mpu_data_region_returns_true_and_updates_base() {
        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");
        let pcb = h.kernel().partitions().get(0).expect("partition 0");
        let orig_size = pcb.mpu_region().size();
        let orig_perms = pcb.mpu_region().permissions();

        let new_base: u32 = 0x2008_0000;
        assert!(
            h.kernel_mut().fix_mpu_data_region(0, new_base),
            "valid index must return true"
        );

        let pcb = h.kernel().partitions().get(0).expect("partition 0");
        assert_eq!(
            pcb.mpu_region().base(),
            new_base,
            "base must be updated to new value"
        );
        assert_eq!(
            pcb.mpu_region().size(),
            orig_size,
            "size must be preserved after base update"
        );
        assert_eq!(
            pcb.mpu_region().permissions(),
            orig_perms,
            "permissions must be preserved after base update"
        );
    }

    // ------------------------------------------------------------------
    // Integration: fix_mpu_data_region is applied during harness build
    // ------------------------------------------------------------------

    #[test]
    fn harness_build_applies_fix_mpu_data_region() {
        // Verifies that build_kernel() calls fix_mpu_data_region for every
        // partition, reconciling MPU bases with pcb.stack_base() after build.
        //
        // On 64-bit test hosts the heap-allocated stack base differs
        // from the PartitionConfig value (RAM_BASE + i*PARTITION_OFFSET),
        // so a successful mismatch proves the fixup actually ran.
        for &n in &[1, 2, 4] {
            let h = KernelTestHarness::with_partitions(n).expect("harness setup");
            for i in 0..n {
                let pcb = h.kernel().partitions().get(i).expect("partition exists");
                assert_eq!(
                    pcb.mpu_region().base(),
                    pcb.stack_base(),
                    "n={n} partition {i}: mpu_region().base() must equal pcb.stack_base() \
                     after build (fix_mpu_data_region integration)"
                );
                let config_base = RAM_BASE + (i as u32) * PARTITION_OFFSET;
                assert_ne!(
                    pcb.mpu_region().base(),
                    config_base,
                    "n={n} partition {i}: mpu_region().base() must differ from the original \
                     config value ({config_base:#x}), proving fix_mpu_data_region overwrote it"
                );
            }
        }
    }

    // ------------------------------------------------------------------
    // PCB stack_base/stack_size valid after construction
    // ------------------------------------------------------------------

    #[test]
    fn pcb_stack_fields_valid_after_construction() {
        for &n in &[1, 2, 4] {
            let h = KernelTestHarness::with_partitions(n).expect("harness setup");
            for i in 0..n {
                let pcb = h.kernel().partitions().get(i).expect("partition exists");
                let base = pcb.stack_base();
                assert_ne!(base, 0, "n={n} P{i} stack_base must be non-zero");
                assert_eq!(base % 4, 0, "n={n} P{i} stack_base must be word-aligned");
                assert_eq!(
                    pcb.stack_size(),
                    STACK_SIZE_BYTES,
                    "n={n} P{i} stack_size mismatch"
                );
            }
        }
    }

    // ------------------------------------------------------------------
    // Peripheral region pipeline tests
    // ------------------------------------------------------------------

    #[test]
    fn peripheral_regions_stored_in_pcb_after_kernel_new() {
        let h = KernelTestHarness::with_peripheral_regions().expect("harness setup");
        let p0 = h.kernel().partitions().get(0).expect("partition 0");
        let regions = p0.peripheral_regions();
        assert_eq!(regions.len(), 1, "P0 must have exactly 1 peripheral region");
        assert_eq!(regions[0].base(), 0x4000_0000);
        assert_eq!(regions[0].size(), 256);
        assert_eq!(regions[0].permissions(), 0x03);

        // P1 must have no peripheral regions.
        let p1 = h.kernel().partitions().get(1).expect("partition 1");
        assert!(
            p1.peripheral_regions().is_empty(),
            "P1 must have no peripheral regions"
        );
    }

    #[test]
    fn accessible_static_regions_includes_peripheral_region() {
        let h = KernelTestHarness::with_peripheral_regions().expect("harness setup");
        let p0 = h.kernel().partitions().get(0).expect("partition 0");
        let regions = p0.accessible_static_regions();
        // Expected: data region, stack region, peripheral region = 3
        assert_eq!(
            regions.len(),
            3,
            "P0 accessible_static_regions must include data + stack + peripheral"
        );
        // The peripheral region must appear somewhere in the list.
        assert!(
            regions.iter().any(|&(b, s)| b == 0x4000_0000 && s == 256),
            "accessible_static_regions must contain the peripheral region (0x4000_0000, 256)"
        );

        // P1 must have only data + stack (no peripheral).
        let p1 = h.kernel().partitions().get(1).expect("partition 1");
        let p1_regions = p1.accessible_static_regions();
        assert_eq!(
            p1_regions.len(),
            2,
            "P1 accessible_static_regions must include only data + stack"
        );
    }

    // ------------------------------------------------------------------
    // Peripheral region pointer validation boundary tests
    // ------------------------------------------------------------------

    #[test]
    fn validate_ptr_peripheral_base_accepted() {
        let h = KernelTestHarness::with_peripheral_regions().expect("harness setup");
        assert!(
            validate_user_ptr(h.kernel().partitions(), 0, 0x4000_0000, 1),
            "pointer at peripheral base must be accepted"
        );
    }

    #[test]
    fn validate_ptr_peripheral_last_byte_accepted() {
        let h = KernelTestHarness::with_peripheral_regions().expect("harness setup");
        assert!(
            validate_user_ptr(h.kernel().partitions(), 0, 0x4000_00FF, 1),
            "pointer at base+size-1 with len=1 must be accepted"
        );
    }

    #[test]
    fn validate_ptr_peripheral_just_past_end_rejected() {
        let h = KernelTestHarness::with_peripheral_regions().expect("harness setup");
        assert!(
            !validate_user_ptr(h.kernel().partitions(), 0, 0x4000_0100, 1),
            "pointer at base+size must be rejected"
        );
    }

    #[test]
    fn validate_ptr_peripheral_just_before_base_rejected() {
        let h = KernelTestHarness::with_peripheral_regions().expect("harness setup");
        assert!(
            !validate_user_ptr(h.kernel().partitions(), 0, 0x3FFF_FFFF, 1),
            "pointer at base-1 must be rejected"
        );
    }

    #[test]
    fn validate_ptr_peripheral_spanning_end_rejected() {
        let h = KernelTestHarness::with_peripheral_regions().expect("harness setup");
        assert!(
            !validate_user_ptr(h.kernel().partitions(), 0, 0x4000_00FF, 2),
            "pointer spanning past peripheral region end must be rejected"
        );
    }

    #[test]
    fn validate_ptr_peripheral_wrong_partition_rejected() {
        let h = KernelTestHarness::with_peripheral_regions().expect("harness setup");
        assert!(
            !validate_user_ptr(h.kernel().partitions(), 1, 0x4000_0000, 1),
            "P1 (no peripheral) must not access P0 peripheral address"
        );
    }

    /// Verify that the two-peripheral harness produces distinct MPU R4
    /// encodings for each partition's peripheral region, and that unused
    /// R5 slots are disabled.
    #[test]
    fn two_peripheral_partitions_mpu_context_switch() {
        use crate::mpu::peripheral_mpu_regions_or_disabled;

        let h = KernelTestHarness::with_two_peripheral_partitions().expect("harness setup");

        let p0 = h.kernel().partitions().get(0).expect("partition 0");
        let p1 = h.kernel().partitions().get(1).expect("partition 1");

        let regions_p0 = peripheral_mpu_regions_or_disabled(p0);
        let regions_p1 = peripheral_mpu_regions_or_disabled(p1);

        // --- P0: R4 must encode 0x4000_0000 base, R5 disabled ---
        let p0_r4_rbar = regions_p0[0].0;
        let p0_r4_rasr = regions_p0[0].1;
        assert_eq!(
            p0_r4_rbar & 0xFFFF_FFE0,
            0x4000_0000,
            "P0 R4 RBAR base must be 0x4000_0000"
        );
        assert_ne!(p0_r4_rasr, 0, "P0 R4 RASR must be enabled");
        assert_eq!(
            regions_p0[1].1, 0,
            "P0 R5 RASR must be disabled (only one peripheral)"
        );

        // --- P1: R4 must encode 0x4001_0000 base, R5 disabled ---
        let p1_r4_rbar = regions_p1[0].0;
        let p1_r4_rasr = regions_p1[0].1;
        assert_eq!(
            p1_r4_rbar & 0xFFFF_FFE0,
            0x4001_0000,
            "P1 R4 RBAR base must be 0x4001_0000"
        );
        assert_ne!(p1_r4_rasr, 0, "P1 R4 RASR must be enabled");
        assert_eq!(
            regions_p1[1].1, 0,
            "P1 R5 RASR must be disabled (only one peripheral)"
        );

        // --- Isolation: P0 and P1 R4 RBAR values must differ ---
        assert_ne!(
            p0_r4_rbar, p1_r4_rbar,
            "P0 and P1 R4 RBAR must differ for peripheral isolation"
        );
    }

    /// Verify that `DynamicStrategy::cached_peripheral_regions()` produces
    /// correct R4/R5 register pairs after `wire_boot_peripherals()`.
    /// This tests the actual dynamic-mode runtime path executed during
    /// PendSV context switches, unlike the static-mode test above.
    #[test]
    #[cfg(feature = "dynamic-mpu")]
    fn two_peripheral_partitions_cached_peripheral_regions() {
        use crate::mpu::precompute_mpu_cache;
        use crate::mpu_strategy::{DynamicStrategy, MpuStrategy};

        let mut h = KernelTestHarness::with_two_peripheral_partitions().expect("harness setup");
        // Seal the MPU cache so cached_dynamic_region() is available.
        for pid in 0..2usize {
            let pcb = h
                .kernel_mut()
                .partitions_mut()
                .get_mut(pid)
                .expect("partition");
            precompute_mpu_cache(pcb).expect("precompute_mpu_cache");
        }
        let strategy = DynamicStrategy::new();

        // Configure each partition's dynamic region with 2 peripheral-reserved slots.
        for pid in 0..2u8 {
            let pcb = h
                .kernel()
                .partitions()
                .get(pid as usize)
                .expect("partition");
            let dyn_region = pcb.cached_dynamic_region();
            strategy
                .configure_partition(pid, &[dyn_region], 2)
                .expect("configure_partition");
        }

        // Wire boot peripherals into the strategy's cache.
        let wired = strategy.wire_boot_peripherals(h.kernel().partitions().as_slice());
        assert!(wired > 0, "at least one peripheral must be wired");

        let regions_p0 = strategy.cached_peripheral_regions(0);
        let regions_p1 = strategy.cached_peripheral_regions(1);

        // --- P0: R4 must encode 0x4000_0000 base, R5 disabled ---
        let p0_r4_rbar = regions_p0[0].0;
        let p0_r4_rasr = regions_p0[0].1;
        assert_eq!(
            p0_r4_rbar & 0xFFFF_FFE0,
            0x4000_0000,
            "P0 cached R4 RBAR base must be 0x4000_0000"
        );
        assert_ne!(p0_r4_rasr, 0, "P0 cached R4 RASR must be enabled");
        assert_eq!(
            regions_p0[1].1, 0,
            "P0 cached R5 RASR must be disabled (only one peripheral)"
        );

        // --- P1: R4 must encode 0x4001_0000 base, R5 disabled ---
        let p1_r4_rbar = regions_p1[0].0;
        let p1_r4_rasr = regions_p1[0].1;
        assert_eq!(
            p1_r4_rbar & 0xFFFF_FFE0,
            0x4001_0000,
            "P1 cached R4 RBAR base must be 0x4001_0000"
        );
        assert_ne!(p1_r4_rasr, 0, "P1 cached R4 RASR must be enabled");
        assert_eq!(
            regions_p1[1].1, 0,
            "P1 cached R5 RASR must be disabled (only one peripheral)"
        );

        // --- Isolation: P0 and P1 R4 RBAR values must differ ---
        assert_ne!(
            p0_r4_rbar, p1_r4_rbar,
            "P0 and P1 cached R4 RBAR must differ for peripheral isolation"
        );
    }

    #[test]
    fn process_pending_yield_advances_schedule_after_sys_yield() {
        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");
        crate::svc::scheduler::start_schedule(h.kernel_mut());

        // P0 is Running, P1 is Ready.
        assert_eq!(
            h.kernel().partitions().get(0).unwrap().state(),
            PartitionState::Running
        );
        assert_eq!(
            h.kernel().partitions().get(1).unwrap().state(),
            PartitionState::Ready
        );

        // Dispatch SYS_YIELD as P0.
        let frame = h.dispatch(SYS_YIELD, 0, 0, 0);
        assert_eq!(frame.r0, 0, "SYS_YIELD must return 0");
        assert!(h.kernel().yield_requested, "yield_requested must be set");

        // Process the pending yield — should advance to P1.
        h.process_pending_yield();

        assert!(
            !h.kernel().yield_requested,
            "yield_requested must be cleared"
        );
        assert_eq!(
            h.kernel().partitions().get(0).unwrap().state(),
            PartitionState::Ready,
            "P0 must transition to Ready after yield"
        );
        assert_eq!(
            h.kernel().partitions().get(1).unwrap().state(),
            PartitionState::Running,
            "P1 must transition to Running after yield"
        );
    }

    #[test]
    fn process_pending_yield_noop_when_not_requested() {
        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");
        crate::svc::scheduler::start_schedule(h.kernel_mut());

        assert!(!h.kernel().yield_requested);
        assert_eq!(
            h.kernel().partitions().get(0).unwrap().state(),
            PartitionState::Running
        );

        // Call process_pending_yield without a prior SYS_YIELD — should be a no-op.
        h.process_pending_yield();

        assert!(!h.kernel().yield_requested);
        assert_eq!(
            h.kernel().partitions().get(0).unwrap().state(),
            PartitionState::Running,
            "P0 must remain Running when no yield was requested"
        );
        assert_eq!(
            h.kernel().partitions().get(1).unwrap().state(),
            PartitionState::Ready,
            "P1 must remain Ready when no yield was requested"
        );
    }

    /// Full blocking IPC round-trip: P0 blocks on SemWait, schedule advances
    /// to P1, P1 signals the semaphore waking P0, schedule returns to P0.
    #[test]
    fn blocking_ipc_deschedule_round_trip() {
        let mut h = KernelTestHarness::with_semaphores(&[0]).expect("harness setup");
        crate::svc::scheduler::start_schedule(h.kernel_mut());

        let st = |h: &KernelTestHarness, i| h.kernel().partitions().get(i).unwrap().state();

        // -- Initial state: P0 Running, P1 Ready --
        assert_eq!(st(&h, 0), PartitionState::Running, "P0 must start Running");
        assert_eq!(st(&h, 1), PartitionState::Ready, "P1 must start Ready");

        // -- Step 1: P0 dispatches SemWait (count=0) → P0 blocks --
        let wait_frame = h.dispatch(SYS_SEM_WAIT, 0, 0, 0);
        assert_eq!(wait_frame.r0, 0, "blocking SemWait must return 0");
        assert_eq!(
            st(&h, 0),
            PartitionState::Waiting,
            "P0 must be Waiting after blocking SemWait"
        );
        h.assert_blocking_triggered_deschedule(0);
        h.assert_return_distinguishes_blocking(&wait_frame, true);

        // -- Step 2: Process pending yield → schedule advances to P1 --
        h.process_pending_yield();
        assert!(
            !h.kernel().yield_requested,
            "yield_requested must be cleared"
        );
        assert_eq!(
            st(&h, 0),
            PartitionState::Waiting,
            "P0 must remain Waiting after yield"
        );
        assert_eq!(
            st(&h, 1),
            PartitionState::Running,
            "P1 must be Running after yield"
        );

        // -- Step 3: P1 dispatches SemSignal → wakes P0 (Waiting→Ready) --
        let sig_frame = h.dispatch_as(1, SYS_SEM_SIGNAL, 0, 0, 0);
        assert_eq!(sig_frame.r0, 0, "SemSignal must succeed");
        assert_eq!(
            st(&h, 0),
            PartitionState::Ready,
            "P0 must be Ready after signal wakes it"
        );
        assert_eq!(
            st(&h, 1),
            PartitionState::Running,
            "P1 must remain Running after signal"
        );

        // -- Step 4: Advance schedule back to P0 via SYS_YIELD from P1 --
        let yield_frame = h.dispatch_as(1, SYS_YIELD, 0, 0, 0);
        assert_eq!(yield_frame.r0, 0, "SYS_YIELD must return 0");
        h.process_pending_yield();
        assert_eq!(
            st(&h, 0),
            PartitionState::Running,
            "P0 must be Running after schedule returns"
        );
        assert_eq!(
            st(&h, 1),
            PartitionState::Ready,
            "P1 must be Ready after yielding"
        );

        // Semaphore count must be 0: one signal consumed by the wakeup.
        let sem = h.kernel().semaphores().get(0).expect("semaphore 0 exists");
        assert_eq!(sem.count(), 0, "semaphore count must be 0 after round-trip");
    }

    /// Stress test: 4 partitions with mixed blocking (semaphore + events).
    /// Verifies the at-most-one-Running invariant across blocking operations
    /// and 8+ schedule transitions including major-frame wrap-around.
    #[test]
    fn mixed_blocking_four_partition_invariant_stress() {
        use crate::invariants::assert_partition_state_consistency;

        let mut h = KernelTestHarness::with_partitions(4).expect("harness setup");
        h.kernel_mut()
            .semaphores_mut()
            .add(Semaphore::new(0, 1))
            .expect("add semaphore");
        crate::svc::scheduler::start_schedule(h.kernel_mut());

        let st = |h: &KernelTestHarness, i: usize| h.kernel().partitions().get(i).unwrap().state();
        let check = |h: &KernelTestHarness| {
            assert_partition_state_consistency(h.kernel().partitions().as_slice());
        };

        // Initial: P0 Running, P1-P3 Ready.
        assert_eq!(st(&h, 0), PartitionState::Running);
        check(&h);

        // P0 blocks on SemWait (count=0).
        let f = h.dispatch(SYS_SEM_WAIT, 0, 0, 0);
        assert_eq!(f.r0, 0, "blocking SemWait must return 0");
        assert_eq!(st(&h, 0), PartitionState::Waiting);
        h.assert_blocking_triggered_deschedule(0);
        check(&h);

        // Yield → schedule advances to P1.
        h.process_pending_yield();
        assert_eq!(st(&h, 1), PartitionState::Running);
        check(&h);

        // P1 blocks on EventWait (mask=0x1, no flags set → blocks).
        // SYS_EVT_WAIT encoding: r1 = caller_pid, r2 = mask.
        let f = h.dispatch_as(1, SYS_EVT_WAIT, 1, 0x1, 0);
        assert_eq!(f.r0, 0, "blocking EventWait must return 0");
        assert_eq!(st(&h, 1), PartitionState::Waiting);
        h.assert_blocking_triggered_deschedule(1);
        check(&h);

        // Yield → schedule advances to P2.
        h.process_pending_yield();
        assert_eq!(st(&h, 2), PartitionState::Running);
        check(&h);

        // P2 signals semaphore → wakes P0 (Waiting → Ready).
        let f = h.dispatch_as(2, SYS_SEM_SIGNAL, 0, 0, 0);
        assert_eq!(f.r0, 0, "SemSignal must succeed");
        assert_eq!(st(&h, 0), PartitionState::Ready);
        check(&h);

        // Yield P2 → schedule advances to P3.
        let f = h.dispatch_as(2, SYS_YIELD, 0, 0, 0);
        assert_eq!(f.r0, 0);
        h.process_pending_yield();
        assert_eq!(st(&h, 3), PartitionState::Running);
        check(&h);

        // Cycle through 8+ schedule transitions via advance_schedule_tick.
        // Schedule: [P0:10, P1:10, P2:10, P3:10 (+SystemWindow with dynamic-mpu)].
        // P1 is Waiting → skipped. 200 ticks covers 4+ major frames.
        let mut transitions = 0u32;
        for _ in 0..200 {
            let event = crate::svc::scheduler::advance_schedule_tick(h.kernel_mut());
            check(&h);
            if let ScheduleEvent::PartitionSwitch(pid) = event {
                transitions += 1;
                assert_ne!(pid, 1, "scheduler must never switch to Waiting P1");
            }
        }
        assert!(
            transitions >= 8,
            "expected 8+ schedule transitions, got {transitions}"
        );

        // P1 must remain Waiting throughout (never woken).
        assert_eq!(st(&h, 1), PartitionState::Waiting);
        h.assert_no_switch_to_waiting();
    }

    /// Stress test: 4 partitions with two rounds of mixed blocking (sem + events).
    /// Round 1: P0 SemWait→block, P1 EventWait→block, P2 SemSignal→wake P0.
    /// Round 2: P3 EventSet→wake P1, P3 SemWait→block.
    /// Then 200 ticks verify at-most-one-Running across 8+ transitions and
    /// major-frame wrap-around while P3 stays Waiting.
    #[test]
    fn multi_partition_mixed_blocking_cycled_stress() {
        use crate::invariants::assert_partition_state_consistency;

        let mut h = KernelTestHarness::with_partitions(4).expect("harness setup");
        h.kernel_mut()
            .semaphores_mut()
            .add(Semaphore::new(0, 1))
            .expect("add semaphore");
        crate::svc::scheduler::start_schedule(h.kernel_mut());

        let st = |h: &KernelTestHarness, i: usize| h.kernel().partitions().get(i).unwrap().state();
        let check = |h: &KernelTestHarness| {
            assert_partition_state_consistency(h.kernel().partitions().as_slice());
        };

        // --- Round 1: P0 blocks on sem, P1 blocks on events ---
        assert_eq!(st(&h, 0), PartitionState::Running);
        check(&h);

        // P0 SemWait (blocks, count=0).
        let f = h.dispatch(SYS_SEM_WAIT, 0, 0, 0);
        assert_eq!(f.r0, 0, "blocking SemWait must return 0");
        assert_eq!(st(&h, 0), PartitionState::Waiting);
        check(&h);

        h.process_pending_yield(); // → P1 Running
        assert_eq!(st(&h, 1), PartitionState::Running);
        check(&h);

        // P1 EventWait (blocks, mask=0x1, no flags set).
        let f = h.dispatch_as(1, SYS_EVT_WAIT, 1, 0x1, 0);
        assert_eq!(f.r0, 0, "blocking EventWait must return 0");
        assert_eq!(st(&h, 1), PartitionState::Waiting);
        check(&h);

        h.process_pending_yield(); // → P2 Running
        assert_eq!(st(&h, 2), PartitionState::Running);
        check(&h);

        // P2 signals semaphore → wakes P0 (Waiting→Ready).
        let f = h.dispatch_as(2, SYS_SEM_SIGNAL, 0, 0, 0);
        assert_eq!(f.r0, 0, "SemSignal must succeed");
        assert_eq!(st(&h, 0), PartitionState::Ready);
        check(&h);

        // P2 yields → P3 Running.
        let f = h.dispatch_as(2, SYS_YIELD, 0, 0, 0);
        assert_eq!(f.r0, 0);
        h.process_pending_yield();
        assert_eq!(st(&h, 3), PartitionState::Running);
        check(&h);

        // --- Round 2: P3 wakes P1 via EventSet, then blocks on sem ---
        // P3 sets event on P1 (mask=0x1) → wakes P1 (Waiting→Ready).
        let f = h.dispatch_as(3, SYS_EVT_SET, 1, 0x1, 0);
        assert_eq!(f.r0, 0, "EventSet must succeed");
        assert_eq!(st(&h, 1), PartitionState::Ready);
        check(&h);

        // P3 SemWait (blocks, count=0 — consumed by P0 wake).
        let f = h.dispatch_as(3, SYS_SEM_WAIT, 0, 3, 0);
        assert_eq!(f.r0, 0, "blocking SemWait must return 0");
        assert_eq!(st(&h, 3), PartitionState::Waiting);
        check(&h);

        // --- Tick cycling: 8+ transitions across major-frame wrap-around ---
        // Schedule: [P0:10, P1:10, P2:10, P3:10].
        // P3 is Waiting → skipped. 200 ticks = 5+ major frames.
        h.process_pending_yield();
        check(&h);

        let mut transitions = 0u32;
        for _ in 0..200 {
            let event = crate::svc::scheduler::advance_schedule_tick(h.kernel_mut());
            check(&h);
            if let ScheduleEvent::PartitionSwitch(pid) = event {
                transitions += 1;
                assert_ne!(pid, 3, "scheduler must never switch to Waiting P3");
            }
        }
        assert!(
            transitions >= 8,
            "expected 8+ schedule transitions, got {transitions}"
        );

        // P0, P1, P2 schedulable; P3 still Waiting (never woken).
        assert_eq!(st(&h, 3), PartitionState::Waiting);
        assert_ne!(st(&h, 0), PartitionState::Waiting);
        assert_ne!(st(&h, 1), PartitionState::Waiting);
        assert_ne!(st(&h, 2), PartitionState::Waiting);
    }

    /// Stress test: 4 partitions exercising re-blocking across mixed
    /// primitives. P0 blocks on SemWait, gets woken, then re-blocks on
    /// EventWait — verifying Running→Waiting→Ready→Running→Waiting→Ready
    /// lifecycle. assert_partition_state_consistency called after every
    /// state-changing operation. 340 ticks drive 8+ major-frame
    /// wrap-arounds with P2 Waiting.
    #[test]
    fn mixed_blocking_reblock_wrap_around_stress() {
        use crate::invariants::assert_partition_state_consistency;

        let mut h = KernelTestHarness::with_partitions(4).expect("harness setup");
        h.kernel_mut()
            .semaphores_mut()
            .add(Semaphore::new(0, 1))
            .expect("add semaphore");
        crate::svc::scheduler::start_schedule(h.kernel_mut());

        let st = |h: &KernelTestHarness, i: usize| h.kernel().partitions().get(i).unwrap().state();
        let check = |h: &KernelTestHarness| {
            assert_partition_state_consistency(h.kernel().partitions().as_slice());
        };

        assert_eq!(st(&h, 0), PartitionState::Running);
        check(&h);

        // Phase 1: P0 blocks on SemWait (count=0).
        let f = h.dispatch(SYS_SEM_WAIT, 0, 0, 0);
        assert_eq!(f.r0, 0, "P0: blocking SemWait must return 0");
        assert_eq!(st(&h, 0), PartitionState::Waiting);
        h.assert_blocking_triggered_deschedule(0);
        check(&h);

        h.process_pending_yield();
        assert_eq!(st(&h, 1), PartitionState::Running);
        check(&h);

        // Phase 2: P1 blocks on EventWait (mask=0x1).
        let f = h.dispatch_as(1, SYS_EVT_WAIT, 1, 0x1, 0);
        assert_eq!(f.r0, 0, "P1: blocking EventWait must return 0");
        assert_eq!(st(&h, 1), PartitionState::Waiting);
        h.assert_blocking_triggered_deschedule(1);
        check(&h);

        h.process_pending_yield();
        assert_eq!(st(&h, 2), PartitionState::Running);
        check(&h);

        // Phase 3: P2 signals sem → wakes P0, sets event → wakes P1, yields.
        let f = h.dispatch_as(2, SYS_SEM_SIGNAL, 0, 0, 0);
        assert_eq!(f.r0, 0, "SemSignal must succeed");
        assert_eq!(st(&h, 0), PartitionState::Ready);
        check(&h);

        let f = h.dispatch_as(2, SYS_EVT_SET, 1, 0x1, 0);
        assert_eq!(f.r0, 0, "EventSet on P1 must succeed");
        assert_eq!(st(&h, 1), PartitionState::Ready);
        check(&h);

        let f = h.dispatch_as(2, SYS_YIELD, 0, 0, 0);
        assert_eq!(f.r0, 0);
        h.process_pending_yield();
        assert_eq!(st(&h, 3), PartitionState::Running);
        check(&h);

        // Phase 4: P3 yields → wraps to P0 Running.
        let f = h.dispatch_as(3, SYS_YIELD, 0, 0, 0);
        assert_eq!(f.r0, 0);
        h.process_pending_yield();
        assert_eq!(st(&h, 0), PartitionState::Running);
        check(&h);

        // Phase 5: P0 re-blocks on EventWait (mask=0x2).
        // Exercises Running→Waiting→Ready→Running→Waiting lifecycle on P0.
        let f = h.dispatch_as(0, SYS_EVT_WAIT, 0, 0x2, 0);
        assert_eq!(f.r0, 0, "P0: re-blocking EventWait must return 0");
        assert_eq!(st(&h, 0), PartitionState::Waiting);
        h.assert_blocking_triggered_deschedule(0);
        check(&h);

        // Yield → P1 (woken in phase 3) Running.
        h.process_pending_yield();
        assert_eq!(st(&h, 1), PartitionState::Running);
        check(&h);

        // Phase 6: P1 wakes P0 via EventSet, then yields to P2.
        let f = h.dispatch_as(1, SYS_EVT_SET, 0, 0x2, 0);
        assert_eq!(f.r0, 0, "EventSet on P0 must succeed");
        assert_eq!(st(&h, 0), PartitionState::Ready);
        check(&h);

        let f = h.dispatch_as(1, SYS_YIELD, 0, 0, 0);
        assert_eq!(f.r0, 0);
        h.process_pending_yield();
        assert_eq!(st(&h, 2), PartitionState::Running);
        check(&h);

        // Phase 7: P2 blocks on SemWait (count=0, consumed by P0 wake).
        let f = h.dispatch_as(2, SYS_SEM_WAIT, 0, 2, 0);
        assert_eq!(f.r0, 0, "P2: blocking SemWait must return 0");
        assert_eq!(st(&h, 2), PartitionState::Waiting);
        h.assert_blocking_triggered_deschedule(2);
        check(&h);

        h.process_pending_yield();
        assert_eq!(st(&h, 3), PartitionState::Running);
        check(&h);

        // Phase 8: 340 ticks — P2 Waiting, P0/P1/P3 schedulable.
        // 4 partitions × 10 ticks = 40 ticks/major frame.
        // 340 ticks = 8+ major frames with wrap-around.
        let mut transitions = 0u32;
        for _ in 0..340 {
            let event = crate::svc::scheduler::advance_schedule_tick(h.kernel_mut());
            check(&h);
            if let ScheduleEvent::PartitionSwitch(pid) = event {
                transitions += 1;
                assert_ne!(pid, 2, "scheduler must never switch to Waiting P2");
            }
        }
        assert!(
            transitions >= 8,
            "expected 8+ schedule transitions, got {transitions}"
        );

        // Final: P2 Waiting, others schedulable.
        assert_eq!(st(&h, 2), PartitionState::Waiting);
        assert_ne!(st(&h, 0), PartitionState::Waiting);
        assert_ne!(st(&h, 1), PartitionState::Waiting);
        assert_ne!(st(&h, 3), PartitionState::Waiting);
        h.assert_no_switch_to_waiting();
    }

    /// Stress test: 4 partitions, mixed blocking (sem + events), all woken
    /// before tick-driven schedule loop.  P0 SemWait→block, P1 EventWait→
    /// block, P2 SemSignal→wake P0, P3 EventSet→wake P1.  Then 340 ticks
    /// verify at-most-one-Running invariant across 8+ major-frame
    /// wrap-arounds with all 4 partitions schedulable.
    #[test]
    fn multi_partition_mixed_blocking_all_wake_stress() {
        use crate::invariants::assert_partition_state_consistency;

        let mut h = KernelTestHarness::with_partitions(4).expect("harness setup");
        h.kernel_mut()
            .semaphores_mut()
            .add(Semaphore::new(0, 1))
            .expect("add semaphore");
        crate::svc::scheduler::start_schedule(h.kernel_mut());

        let st = |h: &KernelTestHarness, i: usize| h.kernel().partitions().get(i).unwrap().state();
        let check = |h: &KernelTestHarness| {
            assert_partition_state_consistency(h.kernel().partitions().as_slice());
        };

        // Initial: P0 Running, P1-P3 Ready.
        assert_eq!(st(&h, 0), PartitionState::Running);
        check(&h);

        // Phase 1: P0 blocks on SemWait (count=0).
        let f = h.dispatch(SYS_SEM_WAIT, 0, 0, 0);
        assert_eq!(f.r0, 0, "P0: blocking SemWait must return 0");
        assert_eq!(st(&h, 0), PartitionState::Waiting);
        h.assert_blocking_triggered_deschedule(0);
        check(&h);

        // Yield → P1 Running.
        h.process_pending_yield();
        assert_eq!(st(&h, 1), PartitionState::Running);
        check(&h);

        // Phase 2: P1 blocks on EventWait (mask=0x1, no flags set).
        let f = h.dispatch_as(1, SYS_EVT_WAIT, 1, 0x1, 0);
        assert_eq!(f.r0, 0, "P1: blocking EventWait must return 0");
        assert_eq!(st(&h, 1), PartitionState::Waiting);
        h.assert_blocking_triggered_deschedule(1);
        check(&h);

        // Yield → P2 Running.
        h.process_pending_yield();
        assert_eq!(st(&h, 2), PartitionState::Running);
        check(&h);

        // Phase 3: P2 signals semaphore → wakes P0 (Waiting→Ready).
        let f = h.dispatch_as(2, SYS_SEM_SIGNAL, 0, 0, 0);
        assert_eq!(f.r0, 0, "SemSignal must succeed");
        assert_eq!(st(&h, 0), PartitionState::Ready);
        check(&h);

        // Yield P2 → P3 Running.
        let f = h.dispatch_as(2, SYS_YIELD, 0, 0, 0);
        assert_eq!(f.r0, 0);
        h.process_pending_yield();
        assert_eq!(st(&h, 3), PartitionState::Running);
        check(&h);

        // Phase 4: P3 sets event on P1 (mask=0x1) → wakes P1.
        let f = h.dispatch_as(3, SYS_EVT_SET, 1, 0x1, 0);
        assert_eq!(f.r0, 0, "EventSet on P1 must succeed");
        assert_eq!(st(&h, 1), PartitionState::Ready);
        check(&h);

        // All 4 partitions now schedulable: P3 Running, P0/P1/P2 Ready.
        assert_eq!(st(&h, 3), PartitionState::Running);
        assert_eq!(st(&h, 0), PartitionState::Ready);
        assert_eq!(st(&h, 1), PartitionState::Ready);
        assert_eq!(st(&h, 2), PartitionState::Ready);

        // Phase 5: 340 ticks — all 4 schedulable, 8+ major-frame wraps.
        // Schedule: [P0:10, P1:10, P2:10, P3:10] = 40 ticks/major frame.
        // 340 ticks = 8+ complete major frames.
        let mut transitions = 0u32;
        let mut seen = [false; 4];
        for _ in 0..340 {
            let event = crate::svc::scheduler::advance_schedule_tick(h.kernel_mut());
            check(&h);
            if let ScheduleEvent::PartitionSwitch(pid) = event {
                transitions += 1;
                seen[pid as usize] = true;
            }
        }
        assert!(
            transitions >= 8,
            "expected 8+ schedule transitions, got {transitions}"
        );

        // All 4 partitions must have been scheduled during tick loop.
        for (i, &was_seen) in seen.iter().enumerate() {
            assert!(was_seen, "P{i} must be scheduled during tick loop");
        }

        // No partition should be Waiting — all were woken before tick loop.
        for i in 0..4 {
            assert_ne!(
                st(&h, i),
                PartitionState::Waiting,
                "P{i} must not be Waiting after full-wake stress loop"
            );
        }
        h.assert_no_switch_to_waiting();
    }

    /// Timed semaphore wait expiry: SemWait with timeout on a zero-count
    /// semaphore blocks P0 (Running→Waiting), then expire_timed_waits at
    /// the expiry tick transitions P0 back (Waiting→Ready).
    #[test]
    fn sem_wait_timed_expiry_waiting_to_ready() {
        use crate::semaphore::Semaphore;
        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");
        *h.kernel_mut().semaphores_mut() = crate::semaphore::SemaphorePool::new();
        h.kernel_mut()
            .semaphores_mut()
            .add(Semaphore::new(0, 2))
            .unwrap();
        h.kernel_mut().sync_tick(100);
        let st = |h: &KernelTestHarness, i| h.kernel().partitions().get(i).unwrap().state();
        // -- Step 1: P0 starts Running --
        assert_eq!(st(&h, 0), PartitionState::Running, "P0 must start Running");
        // -- Step 2: Timed SemWait (count=0, timeout=50) → blocks --
        let k = h.kernel_mut();
        let acquired = k
            .sync
            .semaphores_mut()
            .wait_timed(k.core.partitions_mut(), 0, 0, 50, 100)
            .expect("wait_timed must not error");
        assert!(!acquired, "zero-count semaphore must block");
        // -- Step 3: Verify Running→Waiting --
        assert_eq!(
            st(&h, 0),
            PartitionState::Waiting,
            "P0 must be Waiting after block"
        );
        // -- Step 4: expire_timed_waits at tick 150 → Waiting→Ready --
        h.kernel_mut().expire_timed_waits::<8>(150);
        assert_eq!(
            st(&h, 0),
            PartitionState::Ready,
            "P0 must be Ready after expiry"
        );
        assert!(
            h.kernel().yield_requested,
            "yield_requested must be set after Waiting→Ready expiry to trigger reschedule"
        );
    }

    /// Timed IPC expiry: QueuingSendTimed on a full queue blocks P0
    /// (Running→Waiting), then expire_timed_waits at the expiry tick
    /// transitions P0 back (Waiting→Ready) and r0 reflects the IPC outcome.
    #[test]
    fn queuing_send_timed_expiry_waiting_to_ready() {
        use crate::sampling::PortDirection;
        use crate::syscall::SYS_QUEUING_SEND_TIMED;

        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");

        // Create source→destination route; fill destination to capacity (QD=2).
        let src = h
            .kernel_mut()
            .queuing_mut()
            .create_port(PortDirection::Source)
            .expect("create source port");
        let dst = h
            .kernel_mut()
            .queuing_mut()
            .create_port(PortDirection::Destination)
            .expect("create destination port");
        h.kernel_mut()
            .queuing_mut()
            .connect_ports(src, dst)
            .expect("connect ports");
        h.kernel_mut()
            .queuing_mut()
            .get_mut(dst)
            .unwrap()
            .inject_message(4, &[0xAA; 4]);
        h.kernel_mut()
            .queuing_mut()
            .get_mut(dst)
            .unwrap()
            .inject_message(4, &[0xBB; 4]);
        assert!(
            h.kernel().queuing().get(dst).unwrap().is_full(),
            "destination queue must be full before timed send"
        );

        h.kernel_mut().sync_tick(100);

        let st = |h: &KernelTestHarness, i| h.kernel().partitions().get(i).unwrap().state();

        // -- Step 1: P0 starts Running --
        assert_eq!(st(&h, 0), PartitionState::Running, "P0 must start Running");

        // -- Step 2: Dispatch QueuingSendTimed (full queue, timeout=50) → blocks --
        // r2 encoding: (timeout_hi16 << 16) | data_len_lo16
        let timeout: u32 = 50;
        let data_len: u32 = 4;
        let r2 = (timeout << 16) | data_len;
        let mpu_base = h.kernel().partitions().get(0).unwrap().mpu_region().base();
        let frame = h.dispatch(SYS_QUEUING_SEND_TIMED, src as u32, r2, mpu_base);

        // -- Step 3: Verify Running→Waiting transition and yield_requested --
        assert_eq!(
            st(&h, 0),
            PartitionState::Waiting,
            "P0 must be Waiting after blocking send"
        );
        assert!(
            h.kernel().yield_requested,
            "yield_requested must be set after blocking send triggers deschedule"
        );

        // -- Step 4: expire_timed_waits at expiry tick (100+50=150) → Waiting→Ready --
        h.kernel_mut().expire_timed_waits::<8>(150);
        assert_eq!(
            st(&h, 0),
            PartitionState::Ready,
            "P0 must be Ready after timeout expiry"
        );

        // Verify IPC outcome: r0=0 was set by trigger_deschedule() during the
        // blocking path. expire_timed_waits transitions state only and does not
        // modify the partition's saved frame.
        // TODO: The kernel should ideally write a TIMED_OUT error code into the
        // partition's saved frame on expiry so callers can distinguish timeout
        // from successful completion.
        assert_eq!(
            frame.r0, 0,
            "r0 must reflect IPC outcome after expiry (currently 0 from trigger_deschedule)"
        );

        // TODO: yield_requested after expiry — expire_timed_waits does not
        // re-assert yield_requested; it persists from the original blocking
        // deschedule. Update once the kernel sets it independently on wake-up.
        assert!(
            h.kernel().yield_requested,
            "yield_requested persists after expiry (not cleared by expire_timed_waits)"
        );
    }

    /// Timed IPC expiry: QueuingRecvTimed on an empty queue blocks P0
    /// (Running→Waiting), then expire_timed_waits at the expiry tick
    /// transitions P0 back (Waiting→Ready) and r0 reflects the IPC outcome.
    #[test]
    fn queuing_recv_timed_expiry_waiting_to_ready() {
        use crate::sampling::PortDirection;
        use crate::syscall::SYS_QUEUING_RECV_TIMED;

        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");

        // Create an empty destination queuing port.
        let dst = h
            .kernel_mut()
            .queuing_mut()
            .create_port(PortDirection::Destination)
            .expect("create destination port");
        assert!(
            h.kernel().queuing().get(dst).unwrap().is_empty(),
            "destination queue must be empty for blocking recv"
        );

        h.kernel_mut().sync_tick(100);

        let st = |h: &KernelTestHarness, i| h.kernel().partitions().get(i).unwrap().state();

        // -- Step 1: P0 starts Running --
        assert_eq!(st(&h, 0), PartitionState::Running, "P0 must start Running");

        // -- Step 2: Dispatch QueuingRecvTimed (empty queue, timeout=50) → blocks --
        // TODO: drive-by fix — r2 must be packed per unpack_packed_r2 convention
        let timeout: u32 = 50;
        let buf_len: u32 = 4;
        let r2 = (timeout << 16) | buf_len;
        let mpu_base = h.kernel().partitions().get(0).unwrap().mpu_region().base();
        let frame = h.dispatch(SYS_QUEUING_RECV_TIMED, dst as u32, r2, mpu_base);

        // -- Step 3: Verify Running→Waiting transition and yield_requested --
        assert_eq!(
            st(&h, 0),
            PartitionState::Waiting,
            "P0 must be Waiting after blocking recv"
        );
        assert!(
            h.kernel().yield_requested,
            "yield_requested must be set after blocking recv triggers deschedule"
        );

        // -- Step 4: expire_timed_waits at expiry tick (100+50=150) → Waiting→Ready --
        h.kernel_mut().expire_timed_waits::<8>(150);
        assert_eq!(
            st(&h, 0),
            PartitionState::Ready,
            "P0 must be Ready after timeout expiry"
        );

        // Verify IPC outcome: r0=0 was set by trigger_deschedule() during the
        // blocking path. expire_timed_waits transitions state only and does not
        // modify the partition's saved frame.
        // TODO: The kernel should ideally write a TIMED_OUT error code into the
        // partition's saved frame on expiry so callers can distinguish timeout
        // from successful completion.
        assert_eq!(
            frame.r0, 0,
            "r0 must reflect IPC outcome after expiry (currently 0 from trigger_deschedule)"
        );

        // TODO: yield_requested after expiry — expire_timed_waits does not
        // re-assert yield_requested; it persists from the original blocking
        // deschedule. Update once the kernel sets it independently on wake-up.
        assert!(
            h.kernel().yield_requested,
            "yield_requested persists after expiry (not cleared by expire_timed_waits)"
        );
    }

    /// Regression test: MPU data region bases must remain correct across
    /// multiple schedule cycles. Guards against schedule advancement,
    /// context switch logic, or tick handling corrupting PCB mpu_region.base.
    #[test]
    fn mpu_base_stable_across_schedule_cycles() {
        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");
        let n = h.kernel().partitions().len();

        // Capture initial MPU region state for each partition.
        let mut initial_base = [0u32; 2];
        let mut initial_size = [0u32; 2];
        let mut initial_perms = [0u32; 2];
        for i in 0..n {
            let pcb = h.kernel().partitions().get(i).unwrap();
            let region = pcb.mpu_region();
            initial_base[i] = region.base();
            initial_size[i] = region.size();
            initial_perms[i] = region.permissions();
            assert_eq!(
                initial_base[i],
                pcb.stack_base(),
                "partition {i}: initial mpu_region.base() != pcb.stack_base()"
            );
        }

        // Start the schedule and drive 60 ticks — 3 full major frames for
        // 2 partitions with 10-tick slots each (20 ticks per major frame).
        crate::svc::scheduler::start_schedule(h.kernel_mut());
        let total_ticks: u32 = 60;
        for tick in 1..=total_ticks {
            crate::svc::scheduler::advance_schedule_tick(h.kernel_mut());

            // After every tick, verify all partitions' MPU regions are intact.
            for i in 0..n {
                let pcb = h.kernel().partitions().get(i).unwrap();
                let region = pcb.mpu_region();
                assert_eq!(
                    region.base(),
                    pcb.stack_base(),
                    "tick {tick}: partition {i} mpu_region.base() diverged from pcb.stack_base()"
                );
                assert_eq!(
                    region.base(),
                    initial_base[i],
                    "tick {tick}: partition {i} mpu_region.base() changed"
                );
                assert_eq!(
                    region.size(),
                    initial_size[i],
                    "tick {tick}: partition {i} mpu_region.size() changed"
                );
                assert_eq!(
                    region.permissions(),
                    initial_perms[i],
                    "tick {tick}: partition {i} mpu_region.permissions() changed"
                );
            }
        }
    }

    // ------------------------------------------------------------------
    // Mixed MPU region config: sentinel + user-configured (bug04)
    // ------------------------------------------------------------------

    // ------------------------------------------------------------------
    // PCB mpu_region base is not stale pre-move config address
    // (regression for CRITICAL-pcb-mpu-region-override)
    // ------------------------------------------------------------------

    /// Regression: after Kernel::new + Box move + post-move fixup, every
    /// partition's mpu_region.base must point to the real pcb.stack_base()
    /// (the post-move heap address), NOT the PartitionConfig address that
    /// was computed before the move (RAM_BASE + offset).  This is the
    /// same root cause as bug01 (stale MPU base after kernel relocation).
    #[test]
    fn pcb_mpu_region_not_stale_pre_move_address() {
        for &n in &[2, 3, 4] {
            let h = KernelTestHarness::with_partitions(n).expect("harness setup");

            for i in 0..n {
                let config_base = RAM_BASE + (i as u32) * PARTITION_OFFSET;
                let pcb = h.kernel().partitions().get(i).expect("partition exists");
                let actual_base = pcb.mpu_region().base();

                // The post-move pcb.stack_base() must differ from the
                // config-time address (they live in different allocations).
                assert_ne!(
                    pcb.stack_base(),
                    config_base,
                    "n={n} partition {i}: pcb.stack_base() must differ from \
                     config-time RAM_BASE+offset (kernel was moved)"
                );

                // The PCB must hold the post-move address, not the stale one.
                assert_eq!(
                    actual_base,
                    pcb.stack_base(),
                    "n={n} partition {i}: mpu_region.base() must equal \
                     pcb.stack_base() (post-move address)"
                );
                assert_ne!(
                    actual_base, config_base,
                    "n={n} partition {i}: mpu_region.base() must NOT equal \
                     stale config-time address {config_base:#010x}"
                );
            }
        }
    }

    // ------------------------------------------------------------------
    // Mixed MPU region config: sentinel + user-configured (bug04)
    // ------------------------------------------------------------------

    /// Verifies Kernel::new sentinel handling and that the post-move fixup
    /// (inline sentinel guard, same pattern as boot.rs) preserves
    /// user-configured bases while updating sentinel bases.
    // TODO: boot.rs is #[cfg(not(test))]; an integration test on real hardware
    // would be needed to directly verify the boot path end-to-end.
    #[test]
    fn mixed_mpu_region_config_sentinel_and_user() {
        let h = KernelTestHarness::with_mixed_mpu_regions().expect("harness setup");
        let k = h.kernel();

        // P0: sentinel — Kernel::new derived base from internal stack,
        // then the sentinel-guarded fixup updated it to the pinned address.
        let p0 = k.partitions().get(0).expect("P0");
        assert_eq!(
            p0.mpu_region().base(),
            p0.stack_base(),
            "P0 sentinel: mpu_region.base() must equal pcb.stack_base()"
        );

        // P1: user-configured — Kernel::new preserved the original base,
        // and the sentinel-guarded fixup correctly skipped this partition.
        let p1 = k.partitions().get(1).expect("P1");
        assert_eq!(
            p1.mpu_region().base(),
            0x2004_0000,
            "P1 user-configured: base must be preserved by Kernel::new and sentinel-guarded fixup"
        );
        assert_eq!(
            p1.mpu_region().size(),
            2048,
            "P1 user-configured: size must be preserved"
        );
        assert_eq!(
            p1.mpu_region().permissions(),
            0x0306_0000,
            "P1 user-configured: permissions must be preserved"
        );

        // All partitions in valid state.
        assert_eq!(p0.state(), PartitionState::Running);
        assert_eq!(p1.state(), PartitionState::Ready);
        h.assert_stack_bases_valid();
    }

    // ------------------------------------------------------------------
    // Selective sentinel fixup on mixed-config kernel (boot.rs pattern)
    // ------------------------------------------------------------------

    /// Exercises the boot.rs selective sentinel fixup pattern directly:
    /// creates a 2-partition kernel via Kernel::new() where P0 has a
    /// sentinel mpu_region (size==0) and P1 has a user-configured region,
    /// boxes the kernel, then runs the size==0 guard loop mirroring
    /// boot.rs:296-302. Asserts that only the sentinel partition's base
    /// is updated while user-configured regions are untouched.
    #[test]
    fn selective_sentinel_fixup_mixed_config() {
        use crate::partition::SENTINEL_DATA_PERMISSIONS;

        let original_config_base: u32 = 0x2004_0000;
        let original_config_size: u32 = 1024;

        // Build schedule for 2 partitions.
        let mut schedule = ScheduleTable::new();
        schedule
            .add(ScheduleEntry::new(0, 10))
            .expect("schedule P0");
        schedule
            .add(ScheduleEntry::new(1, 10))
            .expect("schedule P1");
        #[cfg(feature = "dynamic-mpu")]
        schedule.add_system_window(10).expect("system window");

        // P0: sentinel mpu_region (size==0).
        // P1: user-configured mpu_region.
        let mut stacks = Box::new([AlignedStack1K::default(); 2]);
        let (head, tail) = stacks.split_at_mut(1);
        let mem0 = ExternalPartitionMemory::new(
            head[0].as_u32_slice_mut(),
            FLASH_BASE,
            MpuRegion::new(0, 0, 0),
            0,
        )
        .expect("mem0");
        let mem1 = ExternalPartitionMemory::new(
            tail[0].as_u32_slice_mut(),
            FLASH_BASE + PARTITION_OFFSET,
            MpuRegion::new(
                original_config_base,
                original_config_size,
                SENTINEL_DATA_PERMISSIONS,
            ),
            1,
        )
        .expect("mem1");
        let mems = [mem0, mem1];

        // Create kernel and box it to simulate boot placement.
        let mut kernel =
            Box::new(Kernel::<HarnessConfig>::new_external(schedule, &mems).expect("new_external"));
        drop(mems);

        // Use the external (boxed) stack address for P0 (the fixup target).
        let fixup_base = stacks[0].as_u32_slice().as_ptr() as u32;

        // Run selective fixup loop mirroring boot.rs:296-302.
        for i in 0..2usize {
            let is_sentinel = kernel
                .partitions()
                .get(i)
                .is_some_and(|p| p.mpu_region().size() == 0);
            if is_sentinel {
                assert!(
                    kernel.fix_mpu_data_region(i, fixup_base),
                    "fix_mpu_data_region must succeed for sentinel P{i}"
                );
            }
        }

        // Assert P0 (sentinel): base updated to the fixup address.
        let p0 = kernel.partitions().get(0).expect("P0");
        assert_eq!(
            p0.mpu_region().base(),
            fixup_base,
            "P0 sentinel: base must be updated to fixup_base"
        );
        assert_eq!(
            p0.mpu_region().size(),
            0,
            "P0 sentinel: size must be preserved (still 0)"
        );

        // Assert P1 (user-configured): completely untouched by selective fixup.
        let p1 = kernel.partitions().get(1).expect("P1");
        assert_eq!(
            p1.mpu_region().base(),
            original_config_base,
            "P1 user-configured: base must be untouched by selective fixup"
        );
        assert_eq!(
            p1.mpu_region().size(),
            original_config_size,
            "P1 user-configured: size must be preserved"
        );
        assert_eq!(
            p1.mpu_region().permissions(),
            SENTINEL_DATA_PERMISSIONS,
            "P1 user-configured: permissions must be preserved"
        );
    }

    /// Stress test: 4 partitions with a mid-stream wake during active schedule
    /// cycling.  P0 blocks on SemWait, P1 blocks on EventWait, P2 signals the
    /// semaphore (waking P0) and yields to P3.  After one major frame of ticks
    /// with P1 still Waiting, P1 is woken via EventSet mid-stream, then 300
    /// more ticks verify all 4 partitions rotate through Running across 8+
    /// major-frame wrap-arounds.  assert_partition_state_consistency is called
    /// after every state-changing operation.
    #[test]
    fn multi_partition_mixed_blocking_interleaved_tick_stress() {
        use crate::invariants::assert_partition_state_consistency;

        let mut h = KernelTestHarness::with_partitions(4).expect("harness setup");
        h.kernel_mut()
            .semaphores_mut()
            .add(Semaphore::new(0, 1))
            .expect("add semaphore");
        crate::svc::scheduler::start_schedule(h.kernel_mut());

        let st = |h: &KernelTestHarness, i: usize| h.kernel().partitions().get(i).unwrap().state();
        let check = |h: &KernelTestHarness| {
            assert_partition_state_consistency(h.kernel().partitions().as_slice());
        };

        // Initial: P0 Running, P1-P3 Ready.
        assert_eq!(st(&h, 0), PartitionState::Running);
        check(&h);

        // Phase 1: P0 blocks on SemWait (count=0).
        let f = h.dispatch(SYS_SEM_WAIT, 0, 0, 0);
        assert_eq!(f.r0, 0, "blocking SemWait must return 0");
        assert_eq!(st(&h, 0), PartitionState::Waiting);
        h.assert_blocking_triggered_deschedule(0);
        check(&h);

        h.process_pending_yield(); // → P1 Running
        assert_eq!(st(&h, 1), PartitionState::Running);
        check(&h);

        // Phase 2: P1 blocks on EventWait (mask=0x1, no flags set).
        let f = h.dispatch_as(1, SYS_EVT_WAIT, 1, 0x1, 0);
        assert_eq!(f.r0, 0, "blocking EventWait must return 0");
        assert_eq!(st(&h, 1), PartitionState::Waiting);
        h.assert_blocking_triggered_deschedule(1);
        check(&h);

        h.process_pending_yield(); // → P2 Running
        assert_eq!(st(&h, 2), PartitionState::Running);
        check(&h);

        // Phase 3: P2 signals semaphore → wakes P0 (Waiting→Ready), yields → P3.
        let f = h.dispatch_as(2, SYS_SEM_SIGNAL, 0, 0, 0);
        assert_eq!(f.r0, 0, "SemSignal must succeed");
        assert_eq!(st(&h, 0), PartitionState::Ready);
        check(&h);

        let f = h.dispatch_as(2, SYS_YIELD, 0, 0, 0);
        assert_eq!(f.r0, 0);
        h.process_pending_yield(); // → P3 Running
        assert_eq!(st(&h, 3), PartitionState::Running);
        check(&h);

        // State: P0=Ready, P1=Waiting, P2=Ready, P3=Running.
        // Phase 4: 40 ticks (1 major frame) with P1 still Waiting.
        let mut transitions = 0u32;
        for _ in 0..40 {
            let event = crate::svc::scheduler::advance_schedule_tick(h.kernel_mut());
            check(&h);
            if let ScheduleEvent::PartitionSwitch(pid) = event {
                transitions += 1;
                assert_ne!(pid, 1, "must not switch to Waiting P1 in phase 4");
            }
        }
        assert_eq!(
            st(&h, 1),
            PartitionState::Waiting,
            "P1 must still be Waiting"
        );

        // Phase 5: Wake P1 mid-stream via EventSet (mask=0x1).
        // Use the active (Running) partition as caller; after 40 ticks of
        // advance_schedule_tick, active_partition tracks the Running one
        // while current_partition is stale.
        let caller = h.kernel().active_partition().expect("active partition") as usize;
        let f = h.dispatch_as(caller, SYS_EVT_SET, 1, 0x1, 0);
        assert_eq!(f.r0, 0, "EventSet on P1 must succeed");
        assert_eq!(st(&h, 1), PartitionState::Ready);
        check(&h);

        // Phase 6: 300 ticks (7+ major frames) — all 4 now schedulable.
        let mut seen = [false; 4];
        for _ in 0..300 {
            let event = crate::svc::scheduler::advance_schedule_tick(h.kernel_mut());
            check(&h);
            if let ScheduleEvent::PartitionSwitch(pid) = event {
                transitions += 1;
                seen[pid as usize] = true;
            }
        }
        assert!(
            transitions >= 8,
            "expected 8+ total schedule transitions, got {transitions}"
        );

        // All 4 partitions must have been scheduled after P1 was woken.
        for (i, &was_seen) in seen.iter().enumerate() {
            assert!(was_seen, "P{i} must be scheduled after mid-stream wake");
        }

        // No partition should be Waiting at the end.
        for i in 0..4 {
            assert_ne!(
                st(&h, i),
                PartitionState::Waiting,
                "P{i} must not be Waiting at end of interleaved stress test"
            );
        }
        h.assert_no_switch_to_waiting();
    }

    /// Stress test: 4-partition mixed-blocking (semaphore + events) with all
    /// partitions eventually schedulable across 8+ major-frame wrap-arounds.
    ///
    /// Sequence: P0 SemWait→block, advance to P1, P1 EventWait→block, advance
    /// to P2, P2 SemSignal→wake P0, advance to P3, P3 EventSet→wake P1.
    /// All 4 are now schedulable. 340 ticks (8+ major frames of 40 ticks)
    /// verify at-most-one-Running at every tick boundary.
    #[test]
    fn multi_partition_mixed_blocking_invariant_wrap_stress() {
        use crate::invariants::assert_partition_state_consistency;

        let mut h = KernelTestHarness::with_partitions(4).expect("harness setup");
        h.kernel_mut()
            .semaphores_mut()
            .add(Semaphore::new(0, 1))
            .expect("add semaphore");
        crate::svc::scheduler::start_schedule(h.kernel_mut());

        let st = |h: &KernelTestHarness, i: usize| h.kernel().partitions().get(i).unwrap().state();
        let check = |h: &KernelTestHarness| {
            assert_partition_state_consistency(h.kernel().partitions().as_slice());
        };

        // Initial: P0 Running, P1-P3 Ready.
        assert_eq!(st(&h, 0), PartitionState::Running);
        check(&h);

        // P0 blocks on SemWait (count=0 → blocks).
        let f = h.dispatch(SYS_SEM_WAIT, 0, 0, 0);
        assert_eq!(f.r0, 0, "blocking SemWait must return 0");
        assert_eq!(st(&h, 0), PartitionState::Waiting);
        h.assert_blocking_triggered_deschedule(0);
        check(&h);

        // Schedule advances to P1.
        h.process_pending_yield();
        assert_eq!(st(&h, 1), PartitionState::Running);
        check(&h);

        // P1 blocks on EventWait (mask=0x1, no flags set → blocks).
        let f = h.dispatch_as(1, SYS_EVT_WAIT, 1, 0x1, 0);
        assert_eq!(f.r0, 0, "blocking EventWait must return 0");
        assert_eq!(st(&h, 1), PartitionState::Waiting);
        h.assert_blocking_triggered_deschedule(1);
        check(&h);

        // Schedule advances to P2.
        h.process_pending_yield();
        assert_eq!(st(&h, 2), PartitionState::Running);
        check(&h);

        // P2 signals the semaphore → wakes P0 (Waiting→Ready).
        let f = h.dispatch_as(2, SYS_SEM_SIGNAL, 0, 0, 0);
        assert_eq!(f.r0, 0, "SemSignal must succeed");
        assert_eq!(st(&h, 0), PartitionState::Ready);
        check(&h);

        // Schedule advances to P3.
        let f = h.dispatch_as(2, SYS_YIELD, 0, 0, 0);
        assert_eq!(f.r0, 0);
        h.process_pending_yield();
        assert_eq!(st(&h, 3), PartitionState::Running);
        check(&h);

        // P3 wakes P1 via EventSet (mask=0x1) → P1 Waiting→Ready.
        let f = h.dispatch_as(3, SYS_EVT_SET, 1, 0x1, 0);
        assert_eq!(f.r0, 0, "EventSet must succeed");
        assert_eq!(st(&h, 1), PartitionState::Ready);
        check(&h);

        // State: P0=Ready, P1=Ready, P2=Ready, P3=Running — all schedulable.
        // 340 ticks = 8.5 major frames (40 ticks each). Every tick checks
        // the at-most-one-Running invariant.
        let mut transitions = 0u32;
        let mut seen = [false; 4];
        for _ in 0..340 {
            let event = crate::svc::scheduler::advance_schedule_tick(h.kernel_mut());
            check(&h);
            if let ScheduleEvent::PartitionSwitch(pid) = event {
                transitions += 1;
                seen[pid as usize] = true;
            }
        }
        assert!(
            transitions >= 8,
            "expected 8+ schedule transitions, got {transitions}"
        );

        // All 4 partitions must have been scheduled during wrap-around.
        for (i, &was_seen) in seen.iter().enumerate() {
            assert!(was_seen, "P{i} must be scheduled during wrap-around");
        }

        // No partition should be Waiting at the end.
        for i in 0..4 {
            assert_ne!(
                st(&h, i),
                PartitionState::Waiting,
                "P{i} must not be Waiting at end"
            );
        }
        h.assert_no_switch_to_waiting();
    }

    // ------------------------------------------------------------------
    // IPC data-integrity round-trip tests
    // ------------------------------------------------------------------

    /// Fix MPU data regions for 2 partitions so validated_ptr accepts low32_buf.
    fn fix_mpu_for_ipc(h: &mut KernelTestHarness) {
        for i in 0..2 {
            let base = 0x2000_0000 + (i as u32) * 0x1000;
            h.kernel_mut()
                .partitions_mut()
                .get_mut(i)
                .unwrap()
                .fix_mpu_data_region(base);
        }
    }

    #[test]
    fn msg_queue_round_trip_data_integrity() {
        let mut h = KernelTestHarness::with_messages().expect("harness setup");
        fix_mpu_for_ipc(&mut h);
        let payload: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];

        // Byte offset within the shared mmap page, chosen to avoid data
        // races with parallel tests that also write to low32_buf at offset 0.
        const BUF_OFFSET: usize = 256;

        // P0 sends: r1=queue_id, r2=sender_pid, r3=data_ptr
        // SAFETY: low32_buf(0) is a 4096-byte mmap; BUF_OFFSET+4 is in bounds.
        let send_ptr = unsafe { low32_buf(0).add(BUF_OFFSET) };
        // SAFETY: send_ptr is within the mmap page; writing 4 bytes is in bounds.
        unsafe { core::ptr::copy_nonoverlapping(payload.as_ptr(), send_ptr, 4) };
        let sf = h.dispatch_as(0, SYS_MSG_SEND, 0, 0, send_ptr as u32);
        assert_eq!(sf.r0, 0, "SYS_MSG_SEND must return 0");

        // P1 receives: r1=queue_id, r2=caller_pid, r3=buf_ptr
        // SAFETY: low32_buf(1) is a 4096-byte mmap; BUF_OFFSET+4 is in bounds.
        let recv_ptr = unsafe { low32_buf(1).add(BUF_OFFSET) };
        // SAFETY: recv_ptr is within the mmap page; zeroing 4 bytes is in bounds.
        unsafe { core::ptr::write_bytes(recv_ptr, 0, 4) };
        let rf = h.dispatch_as(1, SYS_MSG_RECV, 0, 1, recv_ptr as u32);
        assert_eq!(rf.r0, 0, "SYS_MSG_RECV must return 0");
        assert_eq!(
            h.kernel().partitions().get(1).unwrap().state(),
            PartitionState::Running,
            "P1 must remain Running after non-blocking recv"
        );
        // SAFETY: recv_ptr is valid; dispatch wrote 4 bytes.
        let received = unsafe { core::slice::from_raw_parts(recv_ptr, 4) };
        assert_eq!(received, &payload, "recv payload mismatch");
    }

    #[test]
    fn sampling_port_round_trip_data_integrity() {
        use crate::sampling::PortDirection;
        use crate::syscall::{SYS_SAMPLING_READ, SYS_SAMPLING_WRITE};

        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");
        fix_mpu_for_ipc(&mut h);

        let src = h
            .kernel_mut()
            .sampling_mut()
            .create_port(PortDirection::Source, 1000)
            .unwrap();
        let dst = h
            .kernel_mut()
            .sampling_mut()
            .create_port(PortDirection::Destination, 1000)
            .unwrap();
        h.kernel_mut()
            .sampling_mut()
            .connect_ports(src, dst)
            .unwrap();
        let payload: [u8; 4] = [0xCA, 0xFE, 0xBA, 0xBE];

        // Byte offset within the shared mmap page, chosen to avoid data
        // races with parallel tests that also write to low32_buf at offset 0.
        const BUF_OFFSET: usize = 256;

        // P0 writes via SYS_SAMPLING_WRITE
        // SAFETY: low32_buf(0) is a 4096-byte mmap; BUF_OFFSET+4 is in bounds.
        let wr = unsafe { low32_buf(0).add(BUF_OFFSET) };
        // SAFETY: wr is within the mmap page; writing 4 bytes is in bounds.
        unsafe { core::ptr::copy_nonoverlapping(payload.as_ptr(), wr, 4) };
        let wf = h.dispatch_as(0, SYS_SAMPLING_WRITE, src as u32, 4, wr as u32);
        assert_eq!(wf.r0, 0, "SYS_SAMPLING_WRITE must return 0");

        // P1 reads via SYS_SAMPLING_READ
        // SAFETY: low32_buf(1) is a 4096-byte mmap; BUF_OFFSET+4 is in bounds.
        let rd = unsafe { low32_buf(1).add(BUF_OFFSET) };
        // SAFETY: rd is within the mmap page; zeroing 4 bytes is in bounds.
        unsafe { core::ptr::write_bytes(rd, 0, 4) };
        let rf = h.dispatch_as(1, SYS_SAMPLING_READ, dst as u32, 0, rd as u32);
        assert_eq!(rf.r0, 4, "SYS_SAMPLING_READ must return 4");
        // SAFETY: rd is valid; dispatch wrote 4 bytes.
        let received = unsafe { core::slice::from_raw_parts(rd, 4) };
        assert_eq!(received, &payload, "sampling read payload mismatch");
    }

    #[test]
    fn blackboard_round_trip_data_integrity() {
        use crate::syscall::{SYS_BB_DISPLAY, SYS_BB_READ};

        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");
        fix_mpu_for_ipc(&mut h);
        let bid = h.kernel_mut().blackboards_mut().create().unwrap();
        let payload: [u8; 4] = [0x12, 0x34, 0x56, 0x78];

        // Byte offset within the shared mmap page, chosen to avoid data
        // races with parallel tests that also write to low32_buf at offset 0.
        const BUF_OFFSET: usize = 256;

        // P0 writes via SYS_BB_DISPLAY
        // SAFETY: low32_buf(0) is a 4096-byte mmap; BUF_OFFSET+4 is in bounds.
        let wr = unsafe { low32_buf(0).add(BUF_OFFSET) };
        // SAFETY: wr is within the mmap page; writing 4 bytes is in bounds.
        unsafe { core::ptr::copy_nonoverlapping(payload.as_ptr(), wr, 4) };
        let wf = h.dispatch_as(0, SYS_BB_DISPLAY, bid as u32, 4, wr as u32);
        assert_eq!(wf.r0, 0, "SYS_BB_DISPLAY must return 0");

        // P1 reads via SYS_BB_READ (timeout=0, non-blocking)
        // SAFETY: low32_buf(1) is a 4096-byte mmap; BUF_OFFSET+4 is in bounds.
        let rd = unsafe { low32_buf(1).add(BUF_OFFSET) };
        // SAFETY: rd is within the mmap page; zeroing 4 bytes is in bounds.
        unsafe { core::ptr::write_bytes(rd, 0, 4) };
        let rf = h.dispatch_as(1, SYS_BB_READ, bid as u32, 0, rd as u32);
        assert_eq!(rf.r0, 4, "SYS_BB_READ must return 4");
        // SAFETY: rd is valid; dispatch wrote 4 bytes.
        let received = unsafe { core::slice::from_raw_parts(rd, 4) };
        assert_eq!(received, &payload, "blackboard read payload mismatch");
    }
}

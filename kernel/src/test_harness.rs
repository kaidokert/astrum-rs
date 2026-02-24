use crate::config::KernelConfig;
use crate::context::ExceptionFrame;
use crate::message::MessageQueue;
use crate::partition::{ConfigError, MpuRegion, PartitionConfig, PartitionState, TransitionError};
use crate::partition_core::{AlignedStack1K, PartitionCore};
use crate::scheduler::{ScheduleEntry, ScheduleTable};
use crate::semaphore::Semaphore;
use crate::svc::{Kernel, YieldResult};
use heapless::Vec;

pub struct HarnessConfig;

impl KernelConfig for HarnessConfig {
    const N: usize = 4;
    const SCHED: usize = 8;
    const STACK_WORDS: usize = 256;
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
    #[cfg(feature = "dynamic-mpu")]
    const BZ: usize = 32;
    type Core = PartitionCore<{ Self::N }, { Self::SCHED }, AlignedStack1K>;
    type Sync = crate::sync_pools::SyncPools<{ Self::S }, { Self::SW }, { Self::MS }, { Self::MW }>;
    type Msg = crate::msg_pools::MsgPools<{ Self::QS }, { Self::QD }, { Self::QM }, { Self::QW }>;
    type Ports = crate::port_pools::PortPools<
        { Self::SP },
        { Self::SM },
        { Self::BS },
        { Self::BM },
        { Self::BW },
    >;
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
        let mut configs: Vec<PartitionConfig, { HarnessConfig::N }> = Vec::new();
        for i in 0..n {
            let o = (i as u32) * PARTITION_OFFSET;
            configs
                .push(PartitionConfig {
                    id: i as u8,
                    entry_point: FLASH_BASE + o,
                    stack_base: RAM_BASE + o,
                    stack_size: STACK_SIZE_BYTES,
                    mpu_region: MpuRegion::new(RAM_BASE + o, STACK_SIZE_BYTES, 0),
                    peripheral_regions: peripheral_fn(i),
                })
                .map_err(|_| HarnessError::ConfigsFull)?;
        }
        let mut kernel = Box::new(
            Kernel::new(
                schedule,
                &configs,
                #[cfg(feature = "dynamic-mpu")]
                crate::virtual_device::DeviceRegistry::new(),
            )
            .map_err(HarnessError::KernelInit)?,
        );
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
        // core_stack_base() returns stable addresses we can write into the PCBs.
        // TODO: Kernel is not move-safe due to internal self-references to stack
        // buffers. Ideally Kernel::new should return a pinned/boxed type or use
        // relative offsets, eliminating this post-move fixup.
        for i in 0..n {
            let base = kernel
                .core_stack_base(i)
                .ok_or(HarnessError::PartitionNotFound)?;
            kernel.fix_mpu_data_region(i, base);
        }
        Ok(Self { kernel })
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
            &*self.kernel as *const _ as usize as u32,
            core::mem::align_of::<Kernel<HarnessConfig>>() as u32,
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

    /// Assert that every partition has a valid, unique core stack base address.
    ///
    /// For each partition index `0..partition_count`:
    /// - `core_stack_base(i)` returns `Some` (stack storage exists)
    /// - The address is non-zero
    /// - The address is word-aligned (multiple of 4)
    /// - No two partitions share the same core stack base address
    pub fn assert_stack_bases_valid(&self) {
        let n = self.kernel.partitions().len();
        let mut seen: std::vec::Vec<u32> = std::vec::Vec::with_capacity(n);
        const WORD_SIZE: usize = core::mem::size_of::<u32>();
        for i in 0..n {
            let base = self.kernel.core_stack_base(i).unwrap_or_else(|| {
                panic!(
                    "invariant violation: core_stack_base({i}) returned None \
                     but partition exists"
                )
            });
            assert_ne!(base, 0, "invariant violation: core_stack_base({i}) is zero");
            assert_eq!(
                base as usize % WORD_SIZE,
                0,
                "invariant violation: core_stack_base({i}) = {base:#x} is not word-aligned"
            );
            for (j, &prev) in seen.iter().enumerate() {
                assert_ne!(
                    base, prev,
                    "invariant violation: partitions {j} and {i} share \
                     core stack base {base:#x}"
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scheduler::ScheduleEvent;
    use crate::syscall::{
        SYS_EVT_SET, SYS_EVT_WAIT, SYS_MSG_RECV, SYS_MSG_SEND, SYS_MTX_LOCK, SYS_MTX_UNLOCK,
        SYS_SEM_SIGNAL, SYS_SEM_WAIT, SYS_YIELD,
    };

    /// Allocate a page at a fixed low address via `mmap` so that
    /// `ptr as u32` round-trips correctly on 64-bit test hosts.
    /// Each call site must use a distinct `page` offset. Leaked.
    fn low32_buf(page: usize) -> *mut u8 {
        extern "C" {
            fn mmap(a: *mut u8, l: usize, p: i32, f: i32, d: i32, o: i64) -> *mut u8;
        }
        let addr = 0x2000_0000 + page * 4096;
        // SAFETY: MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED at a known-free
        // low address. The mapping is intentionally leaked (test-only).
        let ptr = unsafe { mmap(addr as *mut u8, 4096, 0x3, 0x32, -1, 0) };
        assert_eq!(ptr as usize, addr, "mmap MAP_FIXED failed");
        ptr
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

        // TODO: reviewer false positive — fix_mpu_data_region calls are required
        // because Kernel::new now constructs mpu_region from internal stack bases
        // (commit 2675853), so the default harness MPU regions no longer cover the
        // mmap'd low addresses that validated_ptr! checks on 64-bit hosts.
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

    // ------------------------------------------------------------------
    // assert_no_switch_to_waiting — scheduler invariant tests
    // ------------------------------------------------------------------

    #[test]
    fn advance_schedule_skips_waiting() {
        let mut h = KernelTestHarness::with_partitions(2).expect("harness setup");
        h.kernel_mut().start_schedule();

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
            let event = h.kernel_mut().advance_schedule_tick();
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
        h.kernel_mut().start_schedule();

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

    // ----- core_stack_base accessor tests -----

    #[test]
    fn core_stack_base_returns_valid_addresses() {
        let h = KernelTestHarness::with_partitions(3).expect("harness setup");
        for i in 0..3 {
            let base = h.kernel().core_stack_base(i);
            assert!(base.is_some(), "partition {i} should have a stack base");
            let addr = base.unwrap();
            assert_ne!(addr, 0, "partition {i} stack base must be non-zero");
            assert_eq!(addr % 4, 0, "partition {i} stack base must be word-aligned");
        }
    }

    #[test]
    fn core_stack_base_returns_none_for_out_of_bounds() {
        let h = KernelTestHarness::with_partitions(2).expect("harness setup");
        // Indices >= N are truly out of bounds (N is the backing storage size).
        assert!(h.kernel().core_stack_base(HarnessConfig::N).is_none());
        assert!(h.kernel().core_stack_base(HarnessConfig::N + 100).is_none());
    }

    // ----- stack_bases_valid invariant tests -----

    #[test]
    fn stack_bases_valid_after_construction() {
        let h = KernelTestHarness::with_partitions(4).expect("harness setup");
        h.assert_stack_bases_valid();
    }

    // ------------------------------------------------------------------
    // PCB mpu_region matches core stack base (regression for commit 2675853)
    // ------------------------------------------------------------------

    #[test]
    fn pcb_mpu_region_base_matches_core_stack_base() {
        let h = KernelTestHarness::with_partitions(4).expect("harness setup");
        for i in 0..h.kernel().partitions().len() {
            let pcb = h.kernel().partitions().get(i).expect("partition exists");
            let core_base = h
                .kernel()
                .core_stack_base(i)
                .expect("core_stack_base must be Some");
            assert_eq!(
                pcb.mpu_region().base(),
                core_base,
                "partition {i}: mpu_region().base() must equal core_stack_base()"
            );
            assert_eq!(
                pcb.mpu_region().size(),
                STACK_SIZE_BYTES,
                "partition {i}: mpu_region().size() must equal STACK_SIZE_BYTES"
            );
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
}

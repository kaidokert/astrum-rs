use crate::config::KernelConfig;
use crate::context::ExceptionFrame;
use crate::partition::{ConfigError, MpuRegion, PartitionConfig, PartitionState, TransitionError};
use crate::partition_core::{AlignedStack1K, PartitionCore};
use crate::scheduler::{ScheduleEntry, ScheduleTable};
use crate::semaphore::Semaphore;
use crate::svc::Kernel;
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
    const QD: usize = 4;
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
    kernel: Kernel<HarnessConfig>,
}

impl KernelTestHarness {
    pub fn with_partitions(n: usize) -> Result<Self, HarnessError> {
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
                    peripheral_regions: Vec::new(),
                })
                .map_err(|_| HarnessError::ConfigsFull)?;
        }
        let mut kernel = Kernel::new(
            schedule,
            &configs,
            #[cfg(feature = "dynamic-mpu")]
            crate::virtual_device::DeviceRegistry::new(),
        )
        .map_err(HarnessError::KernelInit)?;
        for i in 0..n {
            kernel
                .partitions_mut()
                .get_mut(i)
                .ok_or(HarnessError::PartitionNotFound)?
                .transition(PartitionState::Running)
                .map_err(HarnessError::Transition)?;
        }
        kernel.current_partition = 0;
        Ok(Self { kernel })
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
        self.kernel.current_partition = pid as u8;
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
    use crate::syscall::{
        SYS_EVT_SET, SYS_MTX_LOCK, SYS_MTX_UNLOCK, SYS_SEM_SIGNAL, SYS_SEM_WAIT, SYS_YIELD,
    };

    #[test]
    fn two_partitions_count_and_state() {
        let h = KernelTestHarness::with_partitions(2).expect("harness setup");
        assert_eq!(h.kernel().partitions().len(), 2);
        assert_eq!(h.kernel().current_partition, 0);
        for i in 0..2 {
            assert_eq!(
                h.kernel().partitions().get(i).unwrap().state(),
                PartitionState::Running
            );
        }
    }

    #[test]
    fn four_partitions_count_and_state() {
        let h = KernelTestHarness::with_partitions(4).expect("harness setup");
        assert_eq!(h.kernel().partitions().len(), 4);
        for i in 0..4 {
            assert_eq!(
                h.kernel().partitions().get(i).unwrap().state(),
                PartitionState::Running
            );
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
}

//! Thread-related types for multi-threaded RTOS scheduling.

use crate::ids::ThreadId;

/// Execution state of a thread.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThreadState {
    /// Thread is eligible to run but not currently executing.
    Ready,
    /// Thread is the currently executing thread.
    Running,
    /// Thread is voluntarily suspended (e.g. waiting on a resource).
    Suspended,
    /// Thread has terminated and will not be scheduled again.
    Stopped,
}

/// Per-thread control block holding scheduling and stack metadata.
///
/// `stack_pointer` is at offset 0 so assembly context-switch handlers can
/// load/store the SP with a single `LDR/STR Rn, [Rx]` (no offset).
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ThreadControlBlock {
    /// Current stack pointer value (saved/restored on context switch).
    pub stack_pointer: u32,
    /// Unique thread identifier.
    pub id: ThreadId,
    /// Current execution state.
    pub state: ThreadState,
    /// Static priority (0 = lowest).
    pub priority: u8,
    /// Base address of the thread's stack allocation.
    pub stack_base: u32,
    /// Size of the thread's stack in bytes.
    pub stack_size: u32,
    /// Address of the thread's entry function.
    pub entry_point: u32,
    /// Argument passed in r0 at thread start.
    pub r0_arg: u32,
}

/// Scheduling algorithm used by the kernel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedulingPolicy {
    /// Equal-priority threads rotate in fixed time slices.
    RoundRobin,
    /// Highest-priority ready thread always runs.
    StaticPriority,
}

#[cfg(test)]
#[allow(clippy::clone_on_copy)]
mod tests {
    use super::*;

    #[test]
    fn thread_state_debug_clone_copy() {
        let s = ThreadState::Ready;
        let s2 = s;
        let s3 = s.clone();
        assert_eq!(s, s2);
        assert_eq!(s, s3);
        // Verify Debug output contains the variant name
        assert_eq!(format!("{:?}", ThreadState::Ready), "Ready");
        assert_eq!(format!("{:?}", ThreadState::Running), "Running");
        assert_eq!(format!("{:?}", ThreadState::Suspended), "Suspended");
        assert_eq!(format!("{:?}", ThreadState::Stopped), "Stopped");
    }

    #[test]
    fn thread_state_transitions() {
        // Verify that a thread's state can be mutated through a typical
        // lifecycle: Ready -> Running -> Suspended -> Ready -> Running -> Stopped.
        let mut state = ThreadState::Ready;
        assert_eq!(state, ThreadState::Ready);

        state = ThreadState::Running;
        assert_eq!(state, ThreadState::Running);

        state = ThreadState::Suspended;
        assert_eq!(state, ThreadState::Suspended);

        state = ThreadState::Ready;
        assert_eq!(state, ThreadState::Ready);

        state = ThreadState::Running;
        assert_eq!(state, ThreadState::Running);

        state = ThreadState::Stopped;
        assert_eq!(state, ThreadState::Stopped);
    }

    #[test]
    fn thread_control_block_construction() {
        let tcb = ThreadControlBlock {
            stack_pointer: 0x2000_1000,
            id: ThreadId::new(1),
            state: ThreadState::Ready,
            priority: 10,
            stack_base: 0x2000_0000,
            stack_size: 4096,
            entry_point: 0x0800_0100,
            r0_arg: 42,
        };
        assert_eq!(tcb.stack_pointer, 0x2000_1000);
        assert_eq!(tcb.id.as_raw(), 1);
        assert_eq!(tcb.state, ThreadState::Ready);
        assert_eq!(tcb.priority, 10);
        assert_eq!(tcb.stack_base, 0x2000_0000);
        assert_eq!(tcb.stack_size, 4096);
        assert_eq!(tcb.entry_point, 0x0800_0100);
        assert_eq!(tcb.r0_arg, 42);
    }

    #[test]
    fn thread_control_block_copy_clone() {
        let tcb = ThreadControlBlock {
            stack_pointer: 0x2000_0800,
            id: ThreadId::new(0),
            state: ThreadState::Running,
            priority: 5,
            stack_base: 0x2000_0000,
            stack_size: 2048,
            entry_point: 0x0800_0000,
            r0_arg: 0,
        };
        let tcb2 = tcb;
        let tcb3 = tcb.clone();
        assert_eq!(tcb, tcb2);
        assert_eq!(tcb, tcb3);
    }

    #[test]
    fn scheduling_policy_equality() {
        assert_eq!(SchedulingPolicy::RoundRobin, SchedulingPolicy::RoundRobin);
        assert_eq!(
            SchedulingPolicy::StaticPriority,
            SchedulingPolicy::StaticPriority
        );
        assert_ne!(
            SchedulingPolicy::RoundRobin,
            SchedulingPolicy::StaticPriority
        );
    }

    #[test]
    fn scheduling_policy_debug_clone_copy() {
        let p = SchedulingPolicy::RoundRobin;
        let p2 = p;
        let p3 = p.clone();
        assert_eq!(p, p2);
        assert_eq!(p, p3);
        assert_eq!(format!("{:?}", p), "RoundRobin");
        assert_eq!(
            format!("{:?}", SchedulingPolicy::StaticPriority),
            "StaticPriority"
        );
    }
}

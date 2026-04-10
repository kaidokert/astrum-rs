//! SVC call number constants and syscall dispatch types.

// Base (unconditional) syscall numbers — defined in rtos-traits, re-exported here.
pub use rtos_traits::syscall::SYS_GET_PARTITION_ID;
pub use rtos_traits::syscall::{
    SYS_BB_CLEAR, SYS_BB_DISPLAY, SYS_BB_READ, SYS_DEBUG_EXIT, SYS_DEBUG_PRINT, SYS_EVT_CLEAR,
    SYS_EVT_SET, SYS_EVT_WAIT, SYS_GET_ERROR_STATUS, SYS_GET_MAJOR_FRAME_COUNT,
    SYS_GET_PARTITION_RUN_COUNT, SYS_GET_PARTITION_STATUS, SYS_GET_SCHEDULE_INFO,
    SYS_GET_START_CONDITION, SYS_GET_TIME, SYS_IRQ_ACK, SYS_MSG_RECV, SYS_MSG_SEND, SYS_MTX_LOCK,
    SYS_MTX_UNLOCK, SYS_QUEUING_RECV, SYS_QUEUING_RECV_TIMED, SYS_QUEUING_SEND,
    SYS_QUEUING_SEND_TIMED, SYS_QUEUING_STATUS, SYS_REGISTER_ERROR_HANDLER, SYS_REQUEST_RESTART,
    SYS_REQUEST_STOP, SYS_SAMPLING_READ, SYS_SAMPLING_WRITE, SYS_SEM_SIGNAL, SYS_SEM_WAIT,
    SYS_SLEEP_TICKS, SYS_YIELD,
};

#[cfg(feature = "intra-threads")]
pub use rtos_traits::syscall::{
    SYS_THREAD_CREATE, SYS_THREAD_GET_ID, SYS_THREAD_RESUME, SYS_THREAD_START, SYS_THREAD_STOP,
    SYS_THREAD_SUSPEND,
};

// Buffer and device syscall numbers — defined in rtos-traits, re-exported here.
pub use rtos_traits::syscall::{
    SYS_BUF_ALLOC, SYS_BUF_LEND, SYS_BUF_READ, SYS_BUF_RELEASE, SYS_BUF_REVOKE, SYS_BUF_TRANSFER,
    SYS_BUF_WRITE, SYS_DEV_CLOSE, SYS_DEV_IOCTL, SYS_DEV_OPEN, SYS_DEV_READ, SYS_DEV_READ_TIMED,
    SYS_DEV_WRITE, SYS_QUERY_BOTTOM_HALF,
};

// Re-export SYS_DEBUG_NOTIFY and SYS_DEBUG_WRITE from shared traits crate for ABI isolation
#[cfg(feature = "partition-debug")]
pub use rtos_traits::syscall::{SYS_DEBUG_NOTIFY, SYS_DEBUG_WRITE};

/// Typed syscall identifier for use in the kernel dispatch path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyscallId {
    Yield,
    GetPartitionId,
    EventWait,
    EventSet,
    EventClear,
    SemWait,
    SemSignal,
    MutexLock,
    MutexUnlock,
    MsgSend,
    MsgRecv,
    GetTime,
    SamplingWrite,
    SamplingRead,
    QueuingSend,
    QueuingRecv,
    QueuingStatus,
    BbDisplay,
    BbRead,
    BbClear,
    QueuingSendTimed,
    QueuingRecvTimed,
    DebugPrint,
    DebugExit,
    IrqAck,
    SleepTicks,
    GetStartCondition,
    BufferAlloc,
    BufferRelease,
    DevOpen,
    DevRead,
    DevWrite,
    DevIoctl,
    BufferWrite,
    DevClose,
    DevReadTimed,
    QueryBottomHalf,
    BufferLend,
    BufferRevoke,
    BufferTransfer,
    BufferRead,
    RegisterErrorHandler,
    GetErrorStatus,
    RequestRestart,
    RequestStop,
    GetPartitionRunCount,
    GetMajorFrameCount,
    GetScheduleInfo,
    GetPartitionStatus,
    #[cfg(feature = "intra-threads")]
    ThreadCreate,
    #[cfg(feature = "intra-threads")]
    ThreadStart,
    #[cfg(feature = "intra-threads")]
    ThreadStop,
    #[cfg(feature = "intra-threads")]
    ThreadSuspend,
    #[cfg(feature = "intra-threads")]
    ThreadResume,
    #[cfg(feature = "intra-threads")]
    ThreadGetId,
    #[cfg(feature = "partition-debug")]
    DebugNotify,
    #[cfg(feature = "partition-debug")]
    DebugWrite,
}

impl SyscallId {
    /// Convert a raw `u32` syscall number to a typed [`SyscallId`].
    ///
    /// Returns `None` for any value that does not correspond to a defined
    /// syscall, including gaps in the numbering (e.g. 1).
    pub const fn from_u32(n: u32) -> Option<Self> {
        match n {
            SYS_YIELD => Some(Self::Yield),
            SYS_GET_PARTITION_ID => Some(Self::GetPartitionId),
            SYS_EVT_WAIT => Some(Self::EventWait),
            SYS_EVT_SET => Some(Self::EventSet),
            SYS_EVT_CLEAR => Some(Self::EventClear),
            SYS_SEM_WAIT => Some(Self::SemWait),
            SYS_SEM_SIGNAL => Some(Self::SemSignal),
            SYS_MTX_LOCK => Some(Self::MutexLock),
            SYS_MTX_UNLOCK => Some(Self::MutexUnlock),
            SYS_MSG_SEND => Some(Self::MsgSend),
            SYS_MSG_RECV => Some(Self::MsgRecv),
            SYS_GET_TIME => Some(Self::GetTime),
            SYS_SAMPLING_WRITE => Some(Self::SamplingWrite),
            SYS_SAMPLING_READ => Some(Self::SamplingRead),
            SYS_QUEUING_SEND => Some(Self::QueuingSend),
            SYS_QUEUING_RECV => Some(Self::QueuingRecv),
            SYS_QUEUING_STATUS => Some(Self::QueuingStatus),
            SYS_BB_DISPLAY => Some(Self::BbDisplay),
            SYS_BB_READ => Some(Self::BbRead),
            SYS_BB_CLEAR => Some(Self::BbClear),
            SYS_QUEUING_SEND_TIMED => Some(Self::QueuingSendTimed),
            SYS_QUEUING_RECV_TIMED => Some(Self::QueuingRecvTimed),
            SYS_DEBUG_PRINT => Some(Self::DebugPrint),
            SYS_DEBUG_EXIT => Some(Self::DebugExit),
            SYS_IRQ_ACK => Some(Self::IrqAck),
            SYS_SLEEP_TICKS => Some(Self::SleepTicks),
            SYS_GET_START_CONDITION => Some(Self::GetStartCondition),
            SYS_BUF_ALLOC => Some(Self::BufferAlloc),
            SYS_BUF_RELEASE => Some(Self::BufferRelease),
            SYS_DEV_OPEN => Some(Self::DevOpen),
            SYS_DEV_READ => Some(Self::DevRead),
            SYS_DEV_WRITE => Some(Self::DevWrite),
            SYS_DEV_IOCTL => Some(Self::DevIoctl),
            SYS_BUF_WRITE => Some(Self::BufferWrite),
            SYS_DEV_CLOSE => Some(Self::DevClose),
            SYS_DEV_READ_TIMED => Some(Self::DevReadTimed),
            SYS_QUERY_BOTTOM_HALF => Some(Self::QueryBottomHalf),
            SYS_BUF_LEND => Some(Self::BufferLend),
            SYS_BUF_REVOKE => Some(Self::BufferRevoke),
            SYS_BUF_TRANSFER => Some(Self::BufferTransfer),
            SYS_BUF_READ => Some(Self::BufferRead),
            SYS_REGISTER_ERROR_HANDLER => Some(Self::RegisterErrorHandler),
            SYS_GET_ERROR_STATUS => Some(Self::GetErrorStatus),
            SYS_REQUEST_RESTART => Some(Self::RequestRestart),
            SYS_REQUEST_STOP => Some(Self::RequestStop),
            SYS_GET_PARTITION_RUN_COUNT => Some(Self::GetPartitionRunCount),
            SYS_GET_MAJOR_FRAME_COUNT => Some(Self::GetMajorFrameCount),
            SYS_GET_SCHEDULE_INFO => Some(Self::GetScheduleInfo),
            SYS_GET_PARTITION_STATUS => Some(Self::GetPartitionStatus),
            #[cfg(feature = "intra-threads")]
            SYS_THREAD_CREATE => Some(Self::ThreadCreate),
            #[cfg(feature = "intra-threads")]
            SYS_THREAD_START => Some(Self::ThreadStart),
            #[cfg(feature = "intra-threads")]
            SYS_THREAD_STOP => Some(Self::ThreadStop),
            #[cfg(feature = "intra-threads")]
            SYS_THREAD_SUSPEND => Some(Self::ThreadSuspend),
            #[cfg(feature = "intra-threads")]
            SYS_THREAD_RESUME => Some(Self::ThreadResume),
            #[cfg(feature = "intra-threads")]
            SYS_THREAD_GET_ID => Some(Self::ThreadGetId),
            #[cfg(feature = "partition-debug")]
            SYS_DEBUG_NOTIFY => Some(Self::DebugNotify),
            #[cfg(feature = "partition-debug")]
            SYS_DEBUG_WRITE => Some(Self::DebugWrite),
            _ => None,
        }
    }

    /// Return a human-readable name for this syscall.
    pub const fn name(self) -> &'static str {
        match self {
            Self::Yield => "yield",
            Self::GetPartitionId => "get_partition_id",
            Self::EventWait => "event_wait",
            Self::EventSet => "event_set",
            Self::EventClear => "event_clear",
            Self::SemWait => "sem_wait",
            Self::SemSignal => "sem_signal",
            Self::MutexLock => "mutex_lock",
            Self::MutexUnlock => "mutex_unlock",
            Self::MsgSend => "msg_send",
            Self::MsgRecv => "msg_recv",
            Self::GetTime => "get_time",
            Self::SamplingWrite => "sampling_write",
            Self::SamplingRead => "sampling_read",
            Self::QueuingSend => "queuing_send",
            Self::QueuingRecv => "queuing_recv",
            Self::QueuingStatus => "queuing_status",
            Self::BbDisplay => "bb_display",
            Self::BbRead => "bb_read",
            Self::BbClear => "bb_clear",
            Self::QueuingSendTimed => "queuing_send_timed",
            Self::QueuingRecvTimed => "queuing_recv_timed",
            Self::DebugPrint => "debug_print",
            Self::DebugExit => "debug_exit",
            Self::IrqAck => "irq_ack",
            Self::SleepTicks => "sleep_ticks",
            Self::GetStartCondition => "get_start_condition",
            Self::BufferAlloc => "buf_alloc",
            Self::BufferRelease => "buf_release",
            Self::DevOpen => "dev_open",
            Self::DevRead => "dev_read",
            Self::DevWrite => "dev_write",
            Self::DevIoctl => "dev_ioctl",
            Self::BufferWrite => "buf_write",
            Self::DevClose => "dev_close",
            Self::DevReadTimed => "dev_read_timed",
            Self::QueryBottomHalf => "query_bottom_half",
            Self::BufferLend => "buf_lend",
            Self::BufferRevoke => "buf_revoke",
            Self::BufferTransfer => "buf_transfer",
            Self::BufferRead => "buf_read",
            Self::RegisterErrorHandler => "register_error_handler",
            Self::GetErrorStatus => "get_error_status",
            Self::RequestRestart => "request_restart",
            Self::RequestStop => "request_stop",
            Self::GetPartitionRunCount => "get_partition_run_count",
            Self::GetMajorFrameCount => "get_major_frame_count",
            Self::GetScheduleInfo => "get_schedule_info",
            Self::GetPartitionStatus => "get_partition_status",
            #[cfg(feature = "intra-threads")]
            Self::ThreadCreate => "thread_create",
            #[cfg(feature = "intra-threads")]
            Self::ThreadStart => "thread_start",
            #[cfg(feature = "intra-threads")]
            Self::ThreadStop => "thread_stop",
            #[cfg(feature = "intra-threads")]
            Self::ThreadSuspend => "thread_suspend",
            #[cfg(feature = "intra-threads")]
            Self::ThreadResume => "thread_resume",
            #[cfg(feature = "intra-threads")]
            Self::ThreadGetId => "thread_get_id",
            #[cfg(feature = "partition-debug")]
            Self::DebugNotify => "debug_notify",
            #[cfg(feature = "partition-debug")]
            Self::DebugWrite => "debug_write",
        }
    }

    /// Return the raw `u32` call number for this syscall.
    pub const fn as_u32(self) -> u32 {
        match self {
            Self::Yield => SYS_YIELD,
            Self::GetPartitionId => SYS_GET_PARTITION_ID,
            Self::EventWait => SYS_EVT_WAIT,
            Self::EventSet => SYS_EVT_SET,
            Self::EventClear => SYS_EVT_CLEAR,
            Self::SemWait => SYS_SEM_WAIT,
            Self::SemSignal => SYS_SEM_SIGNAL,
            Self::MutexLock => SYS_MTX_LOCK,
            Self::MutexUnlock => SYS_MTX_UNLOCK,
            Self::MsgSend => SYS_MSG_SEND,
            Self::MsgRecv => SYS_MSG_RECV,
            Self::GetTime => SYS_GET_TIME,
            Self::SamplingWrite => SYS_SAMPLING_WRITE,
            Self::SamplingRead => SYS_SAMPLING_READ,
            Self::QueuingSend => SYS_QUEUING_SEND,
            Self::QueuingRecv => SYS_QUEUING_RECV,
            Self::QueuingStatus => SYS_QUEUING_STATUS,
            Self::BbDisplay => SYS_BB_DISPLAY,
            Self::BbRead => SYS_BB_READ,
            Self::BbClear => SYS_BB_CLEAR,
            Self::QueuingSendTimed => SYS_QUEUING_SEND_TIMED,
            Self::QueuingRecvTimed => SYS_QUEUING_RECV_TIMED,
            Self::DebugPrint => SYS_DEBUG_PRINT,
            Self::DebugExit => SYS_DEBUG_EXIT,
            Self::IrqAck => SYS_IRQ_ACK,
            Self::SleepTicks => SYS_SLEEP_TICKS,
            Self::GetStartCondition => SYS_GET_START_CONDITION,
            Self::BufferAlloc => SYS_BUF_ALLOC,
            Self::BufferRelease => SYS_BUF_RELEASE,
            Self::DevOpen => SYS_DEV_OPEN,
            Self::DevRead => SYS_DEV_READ,
            Self::DevWrite => SYS_DEV_WRITE,
            Self::DevIoctl => SYS_DEV_IOCTL,
            Self::BufferWrite => SYS_BUF_WRITE,
            Self::DevClose => SYS_DEV_CLOSE,
            Self::DevReadTimed => SYS_DEV_READ_TIMED,
            Self::QueryBottomHalf => SYS_QUERY_BOTTOM_HALF,
            Self::BufferLend => SYS_BUF_LEND,
            Self::BufferRevoke => SYS_BUF_REVOKE,
            Self::BufferTransfer => SYS_BUF_TRANSFER,
            Self::BufferRead => SYS_BUF_READ,
            Self::RegisterErrorHandler => SYS_REGISTER_ERROR_HANDLER,
            Self::GetErrorStatus => SYS_GET_ERROR_STATUS,
            Self::RequestRestart => SYS_REQUEST_RESTART,
            Self::RequestStop => SYS_REQUEST_STOP,
            Self::GetPartitionRunCount => SYS_GET_PARTITION_RUN_COUNT,
            Self::GetMajorFrameCount => SYS_GET_MAJOR_FRAME_COUNT,
            Self::GetScheduleInfo => SYS_GET_SCHEDULE_INFO,
            Self::GetPartitionStatus => SYS_GET_PARTITION_STATUS,
            #[cfg(feature = "intra-threads")]
            Self::ThreadCreate => SYS_THREAD_CREATE,
            #[cfg(feature = "intra-threads")]
            Self::ThreadStart => SYS_THREAD_START,
            #[cfg(feature = "intra-threads")]
            Self::ThreadStop => SYS_THREAD_STOP,
            #[cfg(feature = "intra-threads")]
            Self::ThreadSuspend => SYS_THREAD_SUSPEND,
            #[cfg(feature = "intra-threads")]
            Self::ThreadResume => SYS_THREAD_RESUME,
            #[cfg(feature = "intra-threads")]
            Self::ThreadGetId => SYS_THREAD_GET_ID,
            #[cfg(feature = "partition-debug")]
            Self::DebugNotify => SYS_DEBUG_NOTIFY,
            #[cfg(feature = "partition-debug")]
            Self::DebugWrite => SYS_DEBUG_WRITE,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// All (constant, expected variant) pairs for exhaustive testing.
    const ALL_VARIANTS: &[(u32, SyscallId)] = &[
        (SYS_YIELD, SyscallId::Yield),
        (SYS_GET_PARTITION_ID, SyscallId::GetPartitionId),
        (SYS_EVT_WAIT, SyscallId::EventWait),
        (SYS_EVT_SET, SyscallId::EventSet),
        (SYS_EVT_CLEAR, SyscallId::EventClear),
        (SYS_SEM_WAIT, SyscallId::SemWait),
        (SYS_SEM_SIGNAL, SyscallId::SemSignal),
        (SYS_MTX_LOCK, SyscallId::MutexLock),
        (SYS_MTX_UNLOCK, SyscallId::MutexUnlock),
        (SYS_MSG_SEND, SyscallId::MsgSend),
        (SYS_MSG_RECV, SyscallId::MsgRecv),
        (SYS_GET_TIME, SyscallId::GetTime),
        (SYS_SAMPLING_WRITE, SyscallId::SamplingWrite),
        (SYS_SAMPLING_READ, SyscallId::SamplingRead),
        (SYS_QUEUING_SEND, SyscallId::QueuingSend),
        (SYS_QUEUING_RECV, SyscallId::QueuingRecv),
        (SYS_QUEUING_STATUS, SyscallId::QueuingStatus),
        (SYS_BB_DISPLAY, SyscallId::BbDisplay),
        (SYS_BB_READ, SyscallId::BbRead),
        (SYS_BB_CLEAR, SyscallId::BbClear),
        (SYS_QUEUING_SEND_TIMED, SyscallId::QueuingSendTimed),
        (SYS_QUEUING_RECV_TIMED, SyscallId::QueuingRecvTimed),
        (SYS_DEBUG_PRINT, SyscallId::DebugPrint),
        (SYS_DEBUG_EXIT, SyscallId::DebugExit),
        (SYS_IRQ_ACK, SyscallId::IrqAck),
        (SYS_SLEEP_TICKS, SyscallId::SleepTicks),
        (SYS_GET_START_CONDITION, SyscallId::GetStartCondition),
        (SYS_BUF_ALLOC, SyscallId::BufferAlloc),
        (SYS_BUF_RELEASE, SyscallId::BufferRelease),
        (SYS_DEV_OPEN, SyscallId::DevOpen),
        (SYS_DEV_READ, SyscallId::DevRead),
        (SYS_DEV_WRITE, SyscallId::DevWrite),
        (SYS_DEV_IOCTL, SyscallId::DevIoctl),
        (SYS_BUF_WRITE, SyscallId::BufferWrite),
        (SYS_DEV_CLOSE, SyscallId::DevClose),
        (SYS_DEV_READ_TIMED, SyscallId::DevReadTimed),
        (SYS_QUERY_BOTTOM_HALF, SyscallId::QueryBottomHalf),
        (SYS_BUF_LEND, SyscallId::BufferLend),
        (SYS_BUF_REVOKE, SyscallId::BufferRevoke),
        (SYS_BUF_TRANSFER, SyscallId::BufferTransfer),
        (SYS_BUF_READ, SyscallId::BufferRead),
        (SYS_REGISTER_ERROR_HANDLER, SyscallId::RegisterErrorHandler),
        (SYS_GET_ERROR_STATUS, SyscallId::GetErrorStatus),
        (SYS_REQUEST_RESTART, SyscallId::RequestRestart),
        (SYS_REQUEST_STOP, SyscallId::RequestStop),
        (SYS_GET_PARTITION_RUN_COUNT, SyscallId::GetPartitionRunCount),
        (SYS_GET_MAJOR_FRAME_COUNT, SyscallId::GetMajorFrameCount),
        (SYS_GET_SCHEDULE_INFO, SyscallId::GetScheduleInfo),
        (SYS_GET_PARTITION_STATUS, SyscallId::GetPartitionStatus),
        #[cfg(feature = "intra-threads")]
        (SYS_THREAD_CREATE, SyscallId::ThreadCreate),
        #[cfg(feature = "intra-threads")]
        (SYS_THREAD_START, SyscallId::ThreadStart),
        #[cfg(feature = "intra-threads")]
        (SYS_THREAD_STOP, SyscallId::ThreadStop),
        #[cfg(feature = "intra-threads")]
        (SYS_THREAD_SUSPEND, SyscallId::ThreadSuspend),
        #[cfg(feature = "intra-threads")]
        (SYS_THREAD_RESUME, SyscallId::ThreadResume),
        #[cfg(feature = "intra-threads")]
        (SYS_THREAD_GET_ID, SyscallId::ThreadGetId),
        #[cfg(feature = "partition-debug")]
        (SYS_DEBUG_NOTIFY, SyscallId::DebugNotify),
    ];

    #[test]
    fn from_u32_maps_all_valid() {
        for &(num, expected) in ALL_VARIANTS {
            assert_eq!(
                SyscallId::from_u32(num),
                Some(expected),
                "from_u32({num}) should return {expected:?}"
            );
        }
    }

    #[test]
    fn round_trip_all_variants() {
        for &(num, variant) in ALL_VARIANTS {
            assert_eq!(variant.as_u32(), num);
            assert_eq!(SyscallId::from_u32(variant.as_u32()), Some(variant));
        }
    }

    #[test]
    fn from_u32_rejects_invalid() {
        // 1 is now valid (GetPartitionId).
        assert_eq!(SyscallId::from_u32(1), Some(SyscallId::GetPartitionId));
        // 39 is now SYS_SLEEP_TICKS.
        assert_eq!(SyscallId::from_u32(39), Some(SyscallId::SleepTicks));
        // 40 is now SYS_GET_START_CONDITION.
        assert_eq!(SyscallId::from_u32(40), Some(SyscallId::GetStartCondition));
        assert_eq!(
            SyscallId::from_u32(45),
            Some(SyscallId::GetPartitionRunCount)
        );
        assert_eq!(SyscallId::from_u32(46), Some(SyscallId::GetMajorFrameCount));
        assert_eq!(SyscallId::from_u32(47), Some(SyscallId::GetScheduleInfo));
        assert_eq!(SyscallId::from_u32(48), Some(SyscallId::GetPartitionStatus));
        assert_eq!(SyscallId::from_u32(49), None);
        #[cfg(feature = "intra-threads")]
        {
            assert_eq!(
                SyscallId::from_u32(SYS_THREAD_CREATE),
                Some(SyscallId::ThreadCreate)
            );
            assert_eq!(
                SyscallId::from_u32(SYS_THREAD_GET_ID),
                Some(SyscallId::ThreadGetId)
            );
        }
        #[cfg(not(feature = "intra-threads"))]
        {
            use rtos_traits::syscall::{SYS_THREAD_CREATE, SYS_THREAD_GET_ID};
            assert_eq!(SyscallId::from_u32(SYS_THREAD_CREATE), None);
            assert_eq!(SyscallId::from_u32(SYS_THREAD_GET_ID), None);
        }
        assert_eq!(SyscallId::from_u32(56), None);
        assert_eq!(SyscallId::from_u32(100), None);
        assert_eq!(SyscallId::from_u32(u32::MAX), None);
    }

    #[test]
    fn constants_are_unique() {
        // round_trip_all_variants already proves each constant maps to a
        // distinct variant; here we just verify we have the expected count.
        // Base: 49, +6 for intra-threads, +1 for partition-debug
        let expected =
            49 + if cfg!(feature = "intra-threads") {
                6
            } else {
                0
            } + if cfg!(feature = "partition-debug") {
                1
            } else {
                0
            };
        assert_eq!(ALL_VARIANTS.len(), expected);
        // Spot-check boundary values.
        assert_eq!(SYS_YIELD, 0);
        assert_eq!(SYS_BB_CLEAR, 19);
        assert_eq!(SYS_QUEUING_SEND_TIMED, 27);
        assert_eq!(SYS_QUEUING_RECV_TIMED, 28);
        #[cfg(feature = "partition-debug")]
        assert_eq!(SYS_DEBUG_NOTIFY, 0x40);
    }

    #[test]
    fn enum_traits() {
        // Copy
        let a = SyscallId::Yield;
        let b = a;
        assert_eq!(a, b);
        // Debug
        assert!(!format!("{:?}", SyscallId::MsgRecv).is_empty());
    }

    #[test]
    fn name_returns_nonempty_for_all_variants() {
        for &(_num, variant) in ALL_VARIANTS {
            let n = variant.name();
            assert!(!n.is_empty(), "{variant:?} has empty name");
        }
    }

    #[test]
    fn name_spot_check() {
        assert_eq!(SyscallId::Yield.name(), "yield");
        assert_eq!(SyscallId::GetTime.name(), "get_time");
        assert_eq!(SyscallId::MsgSend.name(), "msg_send");
        assert_eq!(SyscallId::MsgRecv.name(), "msg_recv");
        assert_eq!(SyscallId::SleepTicks.name(), "sleep_ticks");
        assert_eq!(SyscallId::IrqAck.name(), "irq_ack");
    }

    #[test]
    fn name_unique_for_all_variants() {
        let names: Vec<&str> = ALL_VARIANTS.iter().map(|(_, v)| v.name()).collect();
        for (i, a) in names.iter().enumerate() {
            for b in &names[i + 1..] {
                assert_ne!(a, b, "duplicate syscall name: {a}");
            }
        }
    }

    /// Thread syscall numbers return None when intra-threads is off.
    #[test]
    #[cfg(not(feature = "intra-threads"))]
    fn thread_syscalls_return_none_without_feature() {
        use rtos_traits::syscall::{
            SYS_THREAD_CREATE, SYS_THREAD_GET_ID, SYS_THREAD_RESUME, SYS_THREAD_START,
            SYS_THREAD_STOP, SYS_THREAD_SUSPEND,
        };
        for num in [
            SYS_THREAD_CREATE,
            SYS_THREAD_START,
            SYS_THREAD_STOP,
            SYS_THREAD_SUSPEND,
            SYS_THREAD_RESUME,
            SYS_THREAD_GET_ID,
        ] {
            assert_eq!(
                SyscallId::from_u32(num),
                None,
                "thread syscall {num} should be None without intra-threads"
            );
        }
    }

    /// Thread syscall numbers return Some when intra-threads is on.
    #[test]
    #[cfg(feature = "intra-threads")]
    fn thread_syscalls_return_some_with_feature() {
        assert_eq!(
            SyscallId::from_u32(SYS_THREAD_CREATE),
            Some(SyscallId::ThreadCreate)
        );
        assert_eq!(
            SyscallId::from_u32(SYS_THREAD_START),
            Some(SyscallId::ThreadStart)
        );
        assert_eq!(
            SyscallId::from_u32(SYS_THREAD_STOP),
            Some(SyscallId::ThreadStop)
        );
        assert_eq!(
            SyscallId::from_u32(SYS_THREAD_SUSPEND),
            Some(SyscallId::ThreadSuspend)
        );
        assert_eq!(
            SyscallId::from_u32(SYS_THREAD_RESUME),
            Some(SyscallId::ThreadResume)
        );
        assert_eq!(
            SyscallId::from_u32(SYS_THREAD_GET_ID),
            Some(SyscallId::ThreadGetId)
        );
    }
}

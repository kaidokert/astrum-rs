//! SVC call number constants and syscall dispatch types.

// Base (unconditional) syscall numbers — defined in rtos-traits, re-exported here.
pub use rtos_traits::syscall::SYS_GET_PARTITION_ID;
pub use rtos_traits::syscall::{
    SYS_BB_CLEAR, SYS_BB_DISPLAY, SYS_BB_READ, SYS_DEBUG_EXIT, SYS_DEBUG_PRINT, SYS_EVT_CLEAR,
    SYS_EVT_SET, SYS_EVT_WAIT, SYS_GET_TIME, SYS_IRQ_ACK, SYS_MSG_RECV, SYS_MSG_SEND, SYS_MTX_LOCK,
    SYS_MTX_UNLOCK, SYS_QUEUING_RECV, SYS_QUEUING_RECV_TIMED, SYS_QUEUING_SEND,
    SYS_QUEUING_SEND_TIMED, SYS_QUEUING_STATUS, SYS_SAMPLING_READ, SYS_SAMPLING_WRITE,
    SYS_SEM_SIGNAL, SYS_SEM_WAIT, SYS_YIELD,
};

// Dynamic-MPU syscall numbers — defined in rtos-traits, re-exported here.
#[cfg(feature = "dynamic-mpu")]
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
    #[cfg(feature = "dynamic-mpu")]
    BufferAlloc,
    #[cfg(feature = "dynamic-mpu")]
    BufferRelease,
    #[cfg(feature = "dynamic-mpu")]
    DevOpen,
    #[cfg(feature = "dynamic-mpu")]
    DevRead,
    #[cfg(feature = "dynamic-mpu")]
    DevWrite,
    #[cfg(feature = "dynamic-mpu")]
    DevIoctl,
    #[cfg(feature = "dynamic-mpu")]
    BufferWrite,
    #[cfg(feature = "dynamic-mpu")]
    DevClose,
    #[cfg(feature = "dynamic-mpu")]
    DevReadTimed,
    // TODO: QueryBottomHalf gated behind dynamic-mpu; see SYS_QUERY_BOTTOM_HALF comment.
    #[cfg(feature = "dynamic-mpu")]
    QueryBottomHalf,
    #[cfg(feature = "dynamic-mpu")]
    BufferLend,
    #[cfg(feature = "dynamic-mpu")]
    BufferRevoke,
    #[cfg(feature = "dynamic-mpu")]
    BufferTransfer,
    #[cfg(feature = "dynamic-mpu")]
    BufferRead,
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
            #[cfg(feature = "dynamic-mpu")]
            SYS_BUF_ALLOC => Some(Self::BufferAlloc),
            #[cfg(feature = "dynamic-mpu")]
            SYS_BUF_RELEASE => Some(Self::BufferRelease),
            #[cfg(feature = "dynamic-mpu")]
            SYS_DEV_OPEN => Some(Self::DevOpen),
            #[cfg(feature = "dynamic-mpu")]
            SYS_DEV_READ => Some(Self::DevRead),
            #[cfg(feature = "dynamic-mpu")]
            SYS_DEV_WRITE => Some(Self::DevWrite),
            #[cfg(feature = "dynamic-mpu")]
            SYS_DEV_IOCTL => Some(Self::DevIoctl),
            #[cfg(feature = "dynamic-mpu")]
            SYS_BUF_WRITE => Some(Self::BufferWrite),
            #[cfg(feature = "dynamic-mpu")]
            SYS_DEV_CLOSE => Some(Self::DevClose),
            #[cfg(feature = "dynamic-mpu")]
            SYS_DEV_READ_TIMED => Some(Self::DevReadTimed),
            #[cfg(feature = "dynamic-mpu")]
            SYS_QUERY_BOTTOM_HALF => Some(Self::QueryBottomHalf),
            #[cfg(feature = "dynamic-mpu")]
            SYS_BUF_LEND => Some(Self::BufferLend),
            #[cfg(feature = "dynamic-mpu")]
            SYS_BUF_REVOKE => Some(Self::BufferRevoke),
            #[cfg(feature = "dynamic-mpu")]
            SYS_BUF_TRANSFER => Some(Self::BufferTransfer),
            #[cfg(feature = "dynamic-mpu")]
            SYS_BUF_READ => Some(Self::BufferRead),
            #[cfg(feature = "partition-debug")]
            SYS_DEBUG_NOTIFY => Some(Self::DebugNotify),
            #[cfg(feature = "partition-debug")]
            SYS_DEBUG_WRITE => Some(Self::DebugWrite),
            _ => None,
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
            #[cfg(feature = "dynamic-mpu")]
            Self::BufferAlloc => SYS_BUF_ALLOC,
            #[cfg(feature = "dynamic-mpu")]
            Self::BufferRelease => SYS_BUF_RELEASE,
            #[cfg(feature = "dynamic-mpu")]
            Self::DevOpen => SYS_DEV_OPEN,
            #[cfg(feature = "dynamic-mpu")]
            Self::DevRead => SYS_DEV_READ,
            #[cfg(feature = "dynamic-mpu")]
            Self::DevWrite => SYS_DEV_WRITE,
            #[cfg(feature = "dynamic-mpu")]
            Self::DevIoctl => SYS_DEV_IOCTL,
            #[cfg(feature = "dynamic-mpu")]
            Self::BufferWrite => SYS_BUF_WRITE,
            #[cfg(feature = "dynamic-mpu")]
            Self::DevClose => SYS_DEV_CLOSE,
            #[cfg(feature = "dynamic-mpu")]
            Self::DevReadTimed => SYS_DEV_READ_TIMED,
            #[cfg(feature = "dynamic-mpu")]
            Self::QueryBottomHalf => SYS_QUERY_BOTTOM_HALF,
            #[cfg(feature = "dynamic-mpu")]
            Self::BufferLend => SYS_BUF_LEND,
            #[cfg(feature = "dynamic-mpu")]
            Self::BufferRevoke => SYS_BUF_REVOKE,
            #[cfg(feature = "dynamic-mpu")]
            Self::BufferTransfer => SYS_BUF_TRANSFER,
            #[cfg(feature = "dynamic-mpu")]
            Self::BufferRead => SYS_BUF_READ,
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
        #[cfg(feature = "dynamic-mpu")]
        (SYS_BUF_ALLOC, SyscallId::BufferAlloc),
        #[cfg(feature = "dynamic-mpu")]
        (SYS_BUF_RELEASE, SyscallId::BufferRelease),
        #[cfg(feature = "dynamic-mpu")]
        (SYS_DEV_OPEN, SyscallId::DevOpen),
        #[cfg(feature = "dynamic-mpu")]
        (SYS_DEV_READ, SyscallId::DevRead),
        #[cfg(feature = "dynamic-mpu")]
        (SYS_DEV_WRITE, SyscallId::DevWrite),
        #[cfg(feature = "dynamic-mpu")]
        (SYS_DEV_IOCTL, SyscallId::DevIoctl),
        #[cfg(feature = "dynamic-mpu")]
        (SYS_BUF_WRITE, SyscallId::BufferWrite),
        #[cfg(feature = "dynamic-mpu")]
        (SYS_DEV_CLOSE, SyscallId::DevClose),
        #[cfg(feature = "dynamic-mpu")]
        (SYS_DEV_READ_TIMED, SyscallId::DevReadTimed),
        #[cfg(feature = "dynamic-mpu")]
        (SYS_QUERY_BOTTOM_HALF, SyscallId::QueryBottomHalf),
        #[cfg(feature = "dynamic-mpu")]
        (SYS_BUF_LEND, SyscallId::BufferLend),
        #[cfg(feature = "dynamic-mpu")]
        (SYS_BUF_REVOKE, SyscallId::BufferRevoke),
        #[cfg(feature = "dynamic-mpu")]
        (SYS_BUF_TRANSFER, SyscallId::BufferTransfer),
        #[cfg(feature = "dynamic-mpu")]
        (SYS_BUF_READ, SyscallId::BufferRead),
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
        // Just above the defined range depends on features:
        // - Without dynamic-mpu: 39 is invalid (after SYS_IRQ_ACK=38)
        // - With dynamic-mpu: 39 is invalid (after SYS_IRQ_ACK=38)
        assert_eq!(SyscallId::from_u32(39), None);
        assert_eq!(SyscallId::from_u32(100), None);
        assert_eq!(SyscallId::from_u32(u32::MAX), None);
    }

    #[test]
    fn constants_are_unique() {
        // round_trip_all_variants already proves each constant maps to a
        // distinct variant; here we just verify we have the expected count.
        // Base: 25, +14 for dynamic-mpu, +1 for partition-debug
        #[cfg(all(not(feature = "dynamic-mpu"), not(feature = "partition-debug")))]
        assert_eq!(ALL_VARIANTS.len(), 25);
        #[cfg(all(feature = "dynamic-mpu", not(feature = "partition-debug")))]
        assert_eq!(ALL_VARIANTS.len(), 39);
        #[cfg(all(not(feature = "dynamic-mpu"), feature = "partition-debug"))]
        assert_eq!(ALL_VARIANTS.len(), 26);
        #[cfg(all(feature = "dynamic-mpu", feature = "partition-debug"))]
        assert_eq!(ALL_VARIANTS.len(), 40);
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
}

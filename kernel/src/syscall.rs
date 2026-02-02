//! SVC call number constants and syscall dispatch types.

pub const SYS_YIELD: u32 = 0;
pub const SYS_EVT_WAIT: u32 = 2;
pub const SYS_EVT_SET: u32 = 3;
pub const SYS_EVT_CLEAR: u32 = 4;
pub const SYS_SEM_WAIT: u32 = 5;
pub const SYS_SEM_SIGNAL: u32 = 6;
pub const SYS_MTX_LOCK: u32 = 7;
pub const SYS_MTX_UNLOCK: u32 = 8;
pub const SYS_MSG_SEND: u32 = 9;
pub const SYS_MSG_RECV: u32 = 10;
pub const SYS_GET_TIME: u32 = 11;
pub const SYS_SAMPLING_WRITE: u32 = 12;
pub const SYS_SAMPLING_READ: u32 = 13;
pub const SYS_QUEUING_SEND: u32 = 14;
pub const SYS_QUEUING_RECV: u32 = 15;
pub const SYS_QUEUING_STATUS: u32 = 16;
pub const SYS_BB_DISPLAY: u32 = 17;
pub const SYS_BB_READ: u32 = 18;
pub const SYS_BB_CLEAR: u32 = 19;

/// Typed syscall identifier for use in the kernel dispatch path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyscallId {
    Yield,
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
}

impl SyscallId {
    /// Convert a raw `u32` syscall number to a typed [`SyscallId`].
    ///
    /// Returns `None` for any value that does not correspond to a defined
    /// syscall, including gaps in the numbering (e.g. 1).
    pub const fn from_u32(n: u32) -> Option<Self> {
        match n {
            SYS_YIELD => Some(Self::Yield),
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
            _ => None,
        }
    }

    /// Return the raw `u32` call number for this syscall.
    pub const fn as_u32(self) -> u32 {
        match self {
            Self::Yield => SYS_YIELD,
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
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// All (constant, expected variant) pairs for exhaustive testing.
    const ALL_VARIANTS: &[(u32, SyscallId)] = &[
        (SYS_YIELD, SyscallId::Yield),
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
        // Gap at 1 (reserved for SYS_GET_ID, not in this enum).
        assert_eq!(SyscallId::from_u32(1), None);
        // Just above the defined range.
        assert_eq!(SyscallId::from_u32(20), None);
        assert_eq!(SyscallId::from_u32(100), None);
        assert_eq!(SyscallId::from_u32(u32::MAX), None);
    }

    #[test]
    fn constants_are_unique() {
        // round_trip_all_variants already proves each constant maps to a
        // distinct variant; here we just verify we have the expected count.
        assert_eq!(ALL_VARIANTS.len(), 19);
        // Spot-check boundary values.
        assert_eq!(SYS_YIELD, 0);
        assert_eq!(SYS_BB_CLEAR, 19);
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

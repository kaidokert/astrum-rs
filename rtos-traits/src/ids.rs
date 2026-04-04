//! Newtype wrappers for resource identifiers used in syscall APIs.

macro_rules! define_id {
    ($name:ident, $inner:ty) => {
        #[repr(transparent)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
        pub struct $name($inner);

        impl $name {
            #[inline]
            pub const fn new(raw: $inner) -> Self {
                Self(raw)
            }
            #[inline]
            pub const fn as_raw(self) -> $inner {
                self.0
            }
        }

        impl From<$inner> for $name {
            #[inline]
            fn from(v: $inner) -> Self {
                Self(v)
            }
        }

        impl From<$name> for $inner {
            #[inline]
            fn from(id: $name) -> Self {
                id.0
            }
        }
    };
}

define_id!(SemaphoreId, u32);
define_id!(MutexId, u32);
define_id!(SamplingPortId, u32);
define_id!(QueuingPortId, u32);
define_id!(BlackboardId, u32);
define_id!(DeviceId, u8);
define_id!(PartitionId, u32);

impl core::fmt::Display for PartitionId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}", self.0)
    }
}

define_id!(BufferSlotId, u8);
define_id!(ThreadId, u8);
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct EventMask(u32);

impl EventMask {
    #[inline]
    pub const fn new(raw: u32) -> Self {
        Self(raw)
    }
    #[inline]
    pub const fn as_raw(self) -> u32 {
        self.0
    }
}

impl From<u32> for EventMask {
    #[inline]
    fn from(v: u32) -> Self {
        Self(v)
    }
}

impl From<EventMask> for u32 {
    #[inline]
    fn from(m: EventMask) -> Self {
        m.0
    }
}

impl core::ops::BitOr for EventMask {
    type Output = Self;
    #[inline]
    fn bitor(self, rhs: Self) -> Self {
        Self(self.0 | rhs.0)
    }
}

impl core::ops::BitAnd for EventMask {
    type Output = Self;
    #[inline]
    fn bitand(self, rhs: Self) -> Self {
        Self(self.0 & rhs.0)
    }
}

impl core::ops::BitOrAssign for EventMask {
    #[inline]
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

impl core::ops::BitAndAssign for EventMask {
    #[inline]
    fn bitand_assign(&mut self, rhs: Self) {
        self.0 &= rhs.0;
    }
}

impl core::ops::Not for EventMask {
    type Output = Self;
    #[inline]
    fn not(self) -> Self {
        Self(!self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    macro_rules! test_id {
        ($name:ident, $ty:ty, $inner:ty, $val:expr) => {
            mod $name {
                use super::*;

                #[test]
                fn round_trip() {
                    let id = <$ty>::new($val);
                    assert_eq!(id.as_raw(), $val);
                }

                #[test]
                fn from_raw() {
                    let id = <$ty>::from($val);
                    assert_eq!(id.as_raw(), $val);
                }

                #[test]
                fn into_raw() {
                    let id = <$ty>::new($val);
                    let raw: $inner = id.into();
                    assert_eq!(raw, $val);
                }

                #[test]
                fn equality() {
                    assert_eq!(<$ty>::new($val), <$ty>::new($val));
                    assert_ne!(<$ty>::new($val), <$ty>::new(0));
                }

                #[test]
                fn clone_copy() {
                    let a = <$ty>::new($val);
                    let b = a;
                    let c = a.clone();
                    assert_eq!(a, b);
                    assert_eq!(a, c);
                }
            }
        };
    }

    test_id!(semaphore, SemaphoreId, u32, 7u32);
    test_id!(mutex, MutexId, u32, 3u32);
    test_id!(sampling_port, SamplingPortId, u32, 5u32);
    test_id!(queuing_port, QueuingPortId, u32, 12u32);
    test_id!(blackboard, BlackboardId, u32, 2u32);
    test_id!(device, DeviceId, u8, 4u8);
    test_id!(partition, PartitionId, u32, 1u32);
    test_id!(buffer_slot, BufferSlotId, u8, 3u8);
    test_id!(thread, ThreadId, u8, 5u8);
    mod event_mask {
        use super::*;

        #[test]
        fn round_trip() {
            let m = EventMask::new(0xFF);
            assert_eq!(m.as_raw(), 0xFF);
        }

        #[test]
        fn from_into() {
            let m = EventMask::from(0xABu32);
            let raw: u32 = m.into();
            assert_eq!(raw, 0xAB);
        }

        #[test]
        fn bitor() {
            let a = EventMask::new(0x0F);
            let b = EventMask::new(0xF0);
            assert_eq!((a | b).as_raw(), 0xFF);
        }

        #[test]
        fn bitand() {
            let a = EventMask::new(0xFF);
            let b = EventMask::new(0x0F);
            assert_eq!((a & b).as_raw(), 0x0F);
        }

        #[test]
        fn bitor_assign() {
            let mut a = EventMask::new(0x0F);
            a |= EventMask::new(0xF0);
            assert_eq!(a.as_raw(), 0xFF);
        }

        #[test]
        fn bitand_assign() {
            let mut a = EventMask::new(0xFF);
            a &= EventMask::new(0x0F);
            assert_eq!(a.as_raw(), 0x0F);
        }

        #[test]
        fn not() {
            let a = EventMask::new(0x0000_00FF);
            assert_eq!((!a).as_raw(), 0xFFFF_FF00);
        }
    }
}

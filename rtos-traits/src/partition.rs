//! Partition-level type definitions shared between kernel and plib.

/// Signature for a partition body function that receives an argument in `r0`.
pub type PartitionBody = extern "C" fn(u32) -> !;
/// Signature for a partition entry point (no arguments, diverging).
pub type PartitionEntry = extern "C" fn() -> !;
/// Signature for an interrupt service routine handler.
pub type IsrHandler = unsafe extern "C" fn();

/// A partition descriptor: entry point address paired with its `r0` argument.
///
/// The entry point is stored as an [`EntryAddr`] (a `u32` address) rather than
/// a function pointer, so that both `PartitionEntry` and `PartitionBody`
/// signatures can be represented without `transmute`.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PartitionSpec {
    entry_point: EntryAddr,
    r0: u32,
}

impl PartitionSpec {
    pub fn new(entry_point: PartitionEntry, r0: u32) -> Self {
        Self {
            entry_point: EntryAddr(entry_point as *const () as usize as u32),
            r0,
        }
    }
    /// Create a spec from a body-style `extern "C" fn(u32) -> !`.
    pub fn from_body(body: PartitionBody, r0: u32) -> Self {
        Self {
            entry_point: EntryAddr(body as *const () as usize as u32),
            r0,
        }
    }
    pub const fn entry_point(&self) -> EntryAddr {
        self.entry_point
    }
    pub const fn r0(&self) -> u32 {
        self.r0
    }
}

impl From<(PartitionEntry, u32)> for PartitionSpec {
    fn from((entry_point, r0): (PartitionEntry, u32)) -> Self {
        Self::new(entry_point, r0)
    }
}
impl From<(PartitionBody, u32)> for PartitionSpec {
    fn from((body, r0): (PartitionBody, u32)) -> Self {
        Self::from_body(body, r0)
    }
}
impl From<PartitionEntry> for PartitionSpec {
    fn from(entry_point: PartitionEntry) -> Self {
        Self::new(entry_point, 0)
    }
}

/// Type-safe wrapper for a partition entry-point address.
///
/// Internally stores a `u32`, which is the native pointer width on Cortex-M.
/// The `usize`-to-`u32` cast in `from_fn`/`from_body` is lossless on 32-bit
/// targets; on wider hosts (e.g. 64-bit simulator builds) the cast will
/// truncate — callers must ensure addresses fit in 32 bits in that scenario.
// TODO: consider a compile_error!() or debug_assert for non-32-bit targets
//       if this crate is ever built for 64-bit hosted tests.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct EntryAddr(u32);

impl EntryAddr {
    #[inline]
    pub fn from_fn(f: PartitionEntry) -> Self {
        Self(f as *const () as usize as u32)
    }
    #[inline]
    pub fn from_body(f: PartitionBody) -> Self {
        Self(f as *const () as usize as u32)
    }
    #[inline]
    pub fn raw(self) -> u32 {
        self.0
    }
}
impl From<PartitionEntry> for EntryAddr {
    fn from(f: PartitionEntry) -> Self {
        Self::from_fn(f)
    }
}
impl From<PartitionBody> for EntryAddr {
    fn from(f: PartitionBody) -> Self {
        Self::from_body(f)
    }
}
impl From<u32> for EntryAddr {
    fn from(v: u32) -> Self {
        Self(v)
    }
}
impl From<EntryAddr> for u32 {
    fn from(v: EntryAddr) -> Self {
        v.0
    }
}

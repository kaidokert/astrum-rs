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
#[derive(Copy, Clone, PartialEq, Eq)]
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

impl core::fmt::Debug for EntryAddr {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "EntryAddr({:#010x})", self.0)
    }
}

impl PartialEq<u32> for EntryAddr {
    fn eq(&self, other: &u32) -> bool {
        self.0 == *other
    }
}

impl PartialEq<EntryAddr> for u32 {
    fn eq(&self, other: &EntryAddr) -> bool {
        other == self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[allow(clippy::empty_loop)]
    extern "C" fn _dummy_entry() -> ! {
        loop {}
    }

    #[allow(clippy::empty_loop)]
    extern "C" fn _dummy_body(_: u32) -> ! {
        loop {}
    }

    #[test]
    fn entry_addr_from_fn() {
        let addr = EntryAddr::from_fn(_dummy_entry);
        assert_eq!(addr.raw(), _dummy_entry as *const () as usize as u32);
    }

    #[test]
    fn entry_addr_from_body() {
        let addr = EntryAddr::from_body(_dummy_body);
        assert_eq!(addr.raw(), _dummy_body as *const () as usize as u32);
    }

    #[test]
    fn entry_addr_raw_round_trip() {
        let raw = 0x0800_1234u32;
        let addr = EntryAddr::from(raw);
        assert_eq!(addr.raw(), raw);
    }

    #[test]
    fn entry_addr_from_u32() {
        let addr = EntryAddr::from(0x0800_0000u32);
        assert_eq!(addr.raw(), 0x0800_0000);
    }

    #[test]
    fn entry_addr_into_u32() {
        let addr = EntryAddr::from(0x2000_0000u32);
        let v: u32 = addr.into();
        assert_eq!(v, 0x2000_0000);
    }

    #[test]
    fn entry_addr_from_partition_entry() {
        let ep: PartitionEntry = _dummy_entry;
        let addr = EntryAddr::from(ep);
        assert_eq!(addr.raw(), ep as *const () as usize as u32);
    }

    #[test]
    fn entry_addr_from_partition_body() {
        let body: PartitionBody = _dummy_body;
        let addr = EntryAddr::from(body);
        assert_eq!(addr.raw(), body as *const () as usize as u32);
    }

    #[test]
    fn entry_addr_debug_hex() {
        use core::fmt::Write;
        let addr = EntryAddr::from(0x0800_0000u32);
        let mut buf = [0u8; 64];
        let mut cursor = WriteBuf::new(&mut buf);
        write!(cursor, "{:?}", addr).unwrap();
        assert_eq!(cursor.as_str(), "EntryAddr(0x08000000)");
    }

    /// Minimal stack-allocated write buffer for no_std Debug formatting.
    struct WriteBuf<'a> {
        buf: &'a mut [u8],
        pos: usize,
    }
    impl<'a> WriteBuf<'a> {
        fn new(buf: &'a mut [u8]) -> Self {
            Self { buf, pos: 0 }
        }
        fn as_str(&self) -> &str {
            core::str::from_utf8(&self.buf[..self.pos]).unwrap()
        }
    }
    impl core::fmt::Write for WriteBuf<'_> {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let bytes = s.as_bytes();
            let end = self.pos + bytes.len();
            if end > self.buf.len() {
                return Err(core::fmt::Error);
            }
            self.buf[self.pos..end].copy_from_slice(bytes);
            self.pos = end;
            Ok(())
        }
    }

    #[test]
    fn entry_addr_partial_eq_u32_both_directions() {
        let addr = EntryAddr::from(0x0800_0100u32);
        assert_eq!(addr, 0x0800_0100u32);
        assert_eq!(0x0800_0100u32, addr);
        assert_ne!(addr, 0x0800_0001u32);
    }
}

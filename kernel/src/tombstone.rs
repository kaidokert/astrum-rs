//! Post-mortem panic diagnostics via a fixed-size tombstone in `.noinit` RAM.

use core::fmt::{self, Write as _};

/// Magic value distinguishing a valid tombstone from uninitialized SRAM.
pub const TOMBSTONE_MAGIC: u32 = 0xDEAD_C0DE;

/// Maximum length of the file-path field (bytes).
pub const FILE_LEN: usize = 48;
/// Maximum length of the message field (bytes).
pub const MSG_LEN: usize = 72;

/// 128-byte `#[repr(C)]` panic record for `.noinit` RAM.
/// Layout: magic(0) | line(4) | file(8..56) | message(56..128).
#[repr(C)]
pub struct PanicTombstone {
    /// Set to [`TOMBSTONE_MAGIC`] when the tombstone is valid.
    pub magic: u32,
    /// Source line number (0 if unavailable).
    pub line: u32,
    /// Truncated file path, zero-padded.
    pub file: [u8; FILE_LEN],
    /// Truncated panic message, zero-padded.
    pub message: [u8; MSG_LEN],
}

const _: () = assert!(
    core::mem::size_of::<PanicTombstone>() == 128,
    "PanicTombstone must be exactly 128 bytes",
);

/// Copy up to `dst.len()` bytes from `src` into `dst`, zero-filling
/// any remaining bytes.
fn copy_truncated(dst: &mut [u8], src: &[u8]) {
    let copy_len = src.len().min(dst.len());
    dst[..copy_len].copy_from_slice(&src[..copy_len]);
    dst[copy_len..].fill(0);
}

/// A no-alloc `fmt::Write` adapter that writes into a fixed-size byte buffer,
/// silently truncating if the output exceeds capacity.
struct BufWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> BufWriter<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }
}

impl fmt::Write for BufWriter<'_> {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let remaining = self.buf.len().saturating_sub(self.pos);
        let copy_len = s.len().min(remaining);
        self.buf[self.pos..self.pos + copy_len].copy_from_slice(&s.as_bytes()[..copy_len]);
        self.pos += copy_len;
        Ok(())
    }
}

impl PanicTombstone {
    /// An empty (invalid) tombstone with all fields zeroed.
    pub const EMPTY: Self = Self {
        magic: 0,
        line: 0,
        file: [0u8; FILE_LEN],
        message: [0u8; MSG_LEN],
    };

    /// Returns `true` if this tombstone carries valid panic data.
    pub fn is_valid(&self) -> bool {
        self.magic == TOMBSTONE_MAGIC
    }

    /// Populate the tombstone from decomposed panic components.
    ///
    /// This is the no-alloc core that both [`write_panic_info`](Self::write_panic_info)
    /// and test code call.  Strings that exceed their field capacity are
    /// silently truncated.
    pub fn write_from_parts(&mut self, file: &[u8], line: u32, message: &[u8]) {
        self.magic = TOMBSTONE_MAGIC;
        self.line = line;
        copy_truncated(&mut self.file, file);
        copy_truncated(&mut self.message, message);
    }

    /// Populate the tombstone from a [`core::panic::PanicInfo`].
    ///
    /// This is intended for use inside a `#[panic_handler]`.  Strings
    /// that exceed their field capacity are silently truncated.
    /// After this call, [`is_valid`](Self::is_valid) returns `true`.
    pub fn write_panic_info(&mut self, info: &core::panic::PanicInfo) {
        let (file, line) = if let Some(loc) = info.location() {
            (loc.file().as_bytes(), loc.line())
        } else {
            (&[] as &[u8], 0)
        };

        self.magic = TOMBSTONE_MAGIC;
        self.line = line;
        copy_truncated(&mut self.file, file);

        // Use a fmt::Write adapter to capture formatted panic messages
        // (e.g. `panic!("error: {}", code)`) into the fixed-size buffer.
        let mut writer = BufWriter::new(&mut self.message);
        let _ = write!(&mut writer, "{}", info.message());
        let written = writer.pos;
        self.message[written..].fill(0);
    }

    /// Return the file path as a `&str` (up to the first NUL or field end).
    pub fn file_str(&self) -> &str {
        str_from_zero_padded(&self.file)
    }

    /// Return the message as a `&str` (up to the first NUL or field end).
    pub fn message_str(&self) -> &str {
        str_from_zero_padded(&self.message)
    }
}

/// Interpret a zero-padded byte buffer as a UTF-8 `&str`, stopping at
/// the first NUL or the buffer end.
fn str_from_zero_padded(buf: &[u8]) -> &str {
    let len = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
    let slice = &buf[..len];
    match core::str::from_utf8(slice) {
        Ok(s) => s,
        Err(e) => {
            // Truncation may have split a multi-byte UTF-8 character.
            // Return the valid prefix rather than discarding everything.
            // SAFETY: valid_up_to is guaranteed to be a valid UTF-8 boundary.
            unsafe { core::str::from_utf8_unchecked(&slice[..e.valid_up_to()]) }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn size_is_128_bytes() {
        assert_eq!(core::mem::size_of::<PanicTombstone>(), 128);
    }

    #[test]
    fn empty_tombstone_is_invalid() {
        let t = PanicTombstone::EMPTY;
        assert!(!t.is_valid());
        assert_eq!(t.magic, 0);
        assert_eq!(t.line, 0);
        assert!(t.file.iter().all(|&b| b == 0));
        assert!(t.message.iter().all(|&b| b == 0));
    }

    #[test]
    fn magic_marks_valid() {
        let mut t = PanicTombstone::EMPTY;
        t.magic = TOMBSTONE_MAGIC;
        assert!(t.is_valid());
    }

    #[test]
    fn copy_truncated_variants() {
        // Short string: copies data, zeroes remainder.
        let mut buf = [0xFFu8; 10];
        copy_truncated(&mut buf, b"hi");
        assert_eq!(&buf[..2], b"hi");
        assert!(buf[2..].iter().all(|&b| b == 0));
        // Exact fit.
        let mut buf = [0xFFu8; 4];
        copy_truncated(&mut buf, b"abcd");
        assert_eq!(&buf, b"abcd");
        // Overflow truncates.
        copy_truncated(&mut buf, b"hello, world!");
        assert_eq!(&buf, b"hell");
        // Empty source zeroes all.
        buf.fill(0xFF);
        copy_truncated(&mut buf, b"");
        assert!(buf.iter().all(|&b| b == 0));
    }

    #[test]
    fn str_from_zero_padded_variants() {
        assert_eq!(str_from_zero_padded(b"abc\0\0\0"), "abc");
        assert_eq!(str_from_zero_padded(b"abcdef"), "abcdef");
        assert_eq!(str_from_zero_padded(&[0u8; 4]), "");
    }

    #[test]
    fn write_from_parts_roundtrip() {
        let mut t = PanicTombstone::EMPTY;
        t.write_from_parts(b"kernel/src/boot.rs", 42, b"stack overflow");
        assert!(t.is_valid());
        assert_eq!(t.line, 42);
        assert_eq!(t.file_str(), "kernel/src/boot.rs");
        assert_eq!(t.message_str(), "stack overflow");
    }

    #[test]
    fn write_from_parts_empty_fields() {
        let mut t = PanicTombstone::EMPTY;
        t.write_from_parts(b"", 0, b"");
        assert!(t.is_valid());
        assert_eq!(t.file_str(), "");
        assert_eq!(t.message_str(), "");
        assert!(t.file.iter().all(|&b| b == 0));
        assert!(t.message.iter().all(|&b| b == 0));
    }

    #[test]
    fn write_from_parts_truncates_fields() {
        // File truncation.
        let long_file = [b'F'; FILE_LEN + 20];
        let mut t = PanicTombstone::EMPTY;
        t.write_from_parts(&long_file, 1, b"msg");
        assert_eq!(t.file_str().len(), FILE_LEN);
        assert!(t.file.iter().all(|&b| b == b'F'));
        // Message truncation.
        let long_msg = [b'M'; MSG_LEN + 30];
        t.write_from_parts(b"f.rs", 1, &long_msg);
        assert_eq!(t.message_str().len(), MSG_LEN);
        assert!(t.message.iter().all(|&b| b == b'M'));
    }

    #[test]
    fn zero_fill_after_short_strings() {
        let mut t = PanicTombstone::EMPTY;
        t.file.fill(0xFF);
        t.message.fill(0xFF);
        t.write_from_parts(b"x.rs", 1, b"hi");
        assert_eq!(&t.file[..4], b"x.rs");
        assert!(t.file[4..].iter().all(|&b| b == 0));
        assert_eq!(&t.message[..2], b"hi");
        assert!(t.message[2..].iter().all(|&b| b == 0));
    }

    #[test]
    fn repr_c_field_offsets() {
        let t = PanicTombstone::EMPTY;
        let base = &t as *const _ as usize;
        assert_eq!(&t.magic as *const _ as usize - base, 0);
        assert_eq!(&t.line as *const _ as usize - base, 4);
        assert_eq!(&t.file as *const _ as usize - base, 8);
        assert_eq!(&t.message as *const _ as usize - base, 56);
    }

    /// Helper: install a panic hook that serializes into a shared tombstone,
    /// using `fmt::Write` to capture the message (matching `write_panic_info`).
    fn with_panic_tombstone(f: impl FnOnce() + std::panic::UnwindSafe) -> PanicTombstone {
        use std::panic;
        use std::sync::{Arc, Mutex};

        let tombstone = Arc::new(Mutex::new(PanicTombstone::EMPTY));
        let ts_clone = tombstone.clone();

        let prev_hook = panic::take_hook();
        panic::set_hook(Box::new(move |info| {
            let mut t = ts_clone.lock().unwrap();
            let (file, line) = if let Some(loc) = info.location() {
                (loc.file().as_bytes(), loc.line())
            } else {
                (&[] as &[u8], 0)
            };
            // PanicHookInfo (std) doesn't expose message() like core::panic::PanicInfo.
            // Use payload() downcasts to capture both literal and formatted panics.
            #[allow(deprecated)]
            let msg: &[u8] = if let Some(s) = info.payload().downcast_ref::<&str>() {
                s.as_bytes()
            } else if let Some(s) = info.payload().downcast_ref::<String>() {
                s.as_bytes()
            } else {
                b""
            };
            t.write_from_parts(file, line, msg);
        }));

        let _ = panic::catch_unwind(f);
        panic::set_hook(prev_hook);

        let guard = tombstone.lock().unwrap();
        PanicTombstone {
            magic: guard.magic,
            line: guard.line,
            file: guard.file,
            message: guard.message,
        }
    }

    #[test]
    fn panic_hook_captures_literal() {
        let t = with_panic_tombstone(|| panic!("tombstone test"));
        assert!(t.is_valid());
        assert!(t.line > 0);
        assert!(t.file_str().contains("tombstone.rs"));
        assert_eq!(t.message_str(), "tombstone test");
    }

    #[test]
    fn panic_hook_captures_formatted() {
        let code = 42u32;
        let t = with_panic_tombstone(|| panic!("error: {}", code));
        assert!(t.is_valid());
        assert_eq!(t.message_str(), "error: 42");
    }

    #[test]
    fn str_from_zero_padded_truncated_utf8() {
        // Simulate a 2-byte UTF-8 char (ñ = 0xC3 0xB1) truncated mid-character.
        let mut buf = [0u8; 8];
        buf[0] = b'a';
        buf[1] = b'b';
        buf[2] = 0xC3; // first byte of ñ, missing second byte
                       // Should return "ab", not "".
        assert_eq!(str_from_zero_padded(&buf), "ab");
    }

    #[test]
    fn panic_hook_truncates_long_literal() {
        let t = with_panic_tombstone(|| {
            panic!(
                "ABCDEFGHIJ0123456789ABCDEFGHIJabcdefghij0123456789abcdefghijABCDEFGHIJKLMNOP!!"
            );
        });
        assert!(t.is_valid());
        assert_eq!(t.message_str().len(), MSG_LEN);
        assert!(t.message_str().starts_with("ABCDEFGHIJ"));
    }
}

//! Formatting utilities for RTOS components.

/// Stack buffer for formatting debug messages (implements `core::fmt::Write`).
///
/// This is a shared utility used by both kernel and partition code for
/// formatting debug output without heap allocation.
pub struct FmtBuffer<const N: usize> {
    buf: [u8; N],
    pos: usize,
    truncated: bool,
}

impl<const N: usize> Default for FmtBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> FmtBuffer<N> {
    /// Create a new empty format buffer.
    pub const fn new() -> Self {
        Self {
            buf: [0u8; N],
            pos: 0,
            truncated: false,
        }
    }

    /// Returns the formatted bytes (panic-free via `.get()`).
    pub fn as_bytes(&self) -> &[u8] {
        self.buf.get(..self.pos).unwrap_or(&[])
    }

    /// Returns true if any write was truncated.
    pub fn was_truncated(&self) -> bool {
        self.truncated
    }
}

impl<const N: usize> core::fmt::Write for FmtBuffer<N> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remaining = N.saturating_sub(self.pos);
        if bytes.len() > remaining {
            // Truncation: copy what fits, mark truncated, return Err per trait contract
            if let Some(dest) = self.buf.get_mut(self.pos..self.pos + remaining) {
                if let Some(src) = bytes.get(..remaining) {
                    dest.copy_from_slice(src);
                }
            }
            self.pos = N;
            self.truncated = true;
            Err(core::fmt::Error)
        } else {
            // Full write: use panic-free indexing
            if let Some(dest) = self.buf.get_mut(self.pos..self.pos + bytes.len()) {
                dest.copy_from_slice(bytes);
            }
            self.pos += bytes.len();
            Ok(())
        }
    }
}

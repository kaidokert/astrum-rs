//! Partition debug output draining.

use crate::debug::{DebugRecordHeader, KIND_TEXT};
use crate::partition::PartitionControlBlock;

/// Context for draining partition debug output with a fixed-size scratch buffer.
pub struct DrainContext {
    buffer: [u8; 64],
}

impl DrainContext {
    pub const fn new() -> Self {
        Self { buffer: [0u8; 64] }
    }

    /// Drain up to `budget` bytes from the partition's debug buffer.
    /// Reads framed records and forwards TEXT records to klog!.
    /// Clears debug_pending after draining.
    pub fn drain_partition(&mut self, pcb: &mut PartitionControlBlock, budget: usize) -> usize {
        let Some(buf) = pcb.debug_buffer() else {
            pcb.clear_debug_pending();
            return 0;
        };
        let mut total = 0;
        let mut remaining = budget;
        let pid = pcb.id();

        while remaining >= DebugRecordHeader::SIZE {
            let hdr_bytes = buf.drain(&mut self.buffer[..4], 4);
            if hdr_bytes < 4 {
                break;
            }
            total += hdr_bytes;
            remaining = remaining.saturating_sub(hdr_bytes);

            let hdr = DebugRecordHeader::from_bytes([
                self.buffer[0],
                self.buffer[1],
                self.buffer[2],
                self.buffer[3],
            ]);
            let payload_len = hdr.len as usize;
            if payload_len == 0 {
                continue;
            }

            let to_read = payload_len.min(self.buffer.len()).min(remaining);
            let read = buf.drain(&mut self.buffer[..to_read], to_read);
            total += read;
            remaining = remaining.saturating_sub(read);

            if hdr.kind == KIND_TEXT && read > 0 {
                if let Ok(s) = core::str::from_utf8(&self.buffer[..read]) {
                    log_with_level(pid, hdr.level, s);
                }
            }
            if read < payload_len {
                break;
            }
        }
        pcb.clear_debug_pending();
        total
    }
}

impl Default for DrainContext {
    fn default() -> Self {
        Self::new()
    }
}

#[inline]
fn log_with_level(pid: u8, level: u8, msg: &str) {
    const TAGS: [&str; 5] = ["ERR", "WRN", "INF", "DBG", "TRC"];
    let tag = TAGS.get(level as usize).unwrap_or(&"???");
    let _ = (pid, tag, msg);
    crate::klog!("[P{}:{}] {}", pid, tag, msg);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::debug::{DebugRingBuffer, LOG_INFO, LOG_WARN};
    use crate::partition::MpuRegion;

    fn make_pcb(id: u8) -> PartitionControlBlock {
        PartitionControlBlock::new(id, 0x1000, 0x2000, 0x2100, MpuRegion::new(0, 0, 0))
    }

    #[test]
    fn drain_empty_and_no_buffer() {
        // Test empty buffer
        static BUF: DebugRingBuffer<64> = DebugRingBuffer::new();
        let mut pcb = make_pcb(1);
        pcb.set_debug_buffer(&BUF);
        pcb.signal_debug_pending();
        let mut ctx = DrainContext::new();
        assert_eq!(ctx.drain_partition(&mut pcb, 256), 0);
        assert!(!pcb.debug_pending());
        // Test no buffer set
        let mut pcb2 = make_pcb(2);
        pcb2.signal_debug_pending();
        assert_eq!(ctx.drain_partition(&mut pcb2, 256), 0);
        assert!(!pcb2.debug_pending());
    }

    #[test]
    fn drain_records_with_budget() {
        static BUF: DebugRingBuffer<128> = DebugRingBuffer::new();
        let mut pcb = make_pcb(3);
        pcb.set_debug_buffer(&BUF);
        BUF.write_record(LOG_INFO, KIND_TEXT, b"first");
        BUF.write_record(LOG_WARN, KIND_TEXT, b"second");
        pcb.signal_debug_pending();
        let mut ctx = DrainContext::new();
        // Budget of 10 only drains first record (4+5=9)
        assert_eq!(ctx.drain_partition(&mut pcb, 10), 9);
        assert!(!BUF.is_empty());
        // Full budget drains remaining
        pcb.signal_debug_pending();
        assert_eq!(ctx.drain_partition(&mut pcb, 256), 10);
        assert!(BUF.is_empty() && !pcb.debug_pending());
    }
}

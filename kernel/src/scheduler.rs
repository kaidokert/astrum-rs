//! Static schedule table for ARINC 653-style partition scheduling.

/// A single slot in the schedule table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScheduleEntry {
    pub partition_index: u8,
    pub duration_ticks: u32,
}

impl ScheduleEntry {
    pub const fn new(partition_index: u8, duration_ticks: u32) -> Self {
        Self {
            partition_index,
            duration_ticks,
        }
    }
}

/// Fixed-size schedule table with major frame wraparound.
pub struct ScheduleTable<const N: usize> {
    entries: heapless::Vec<ScheduleEntry, N>,
    pub major_frame_ticks: u32,
    current_slot: usize,
    ticks_remaining: u32,
    started: bool,
}

#[allow(clippy::new_without_default)]
impl<const N: usize> ScheduleTable<N> {
    pub const fn new() -> Self {
        Self {
            entries: heapless::Vec::new(),
            major_frame_ticks: 0,
            current_slot: 0,
            ticks_remaining: 0,
            started: false,
        }
    }

    /// Add a schedule entry. Returns `Err` if table is full or duration is zero.
    pub fn add(&mut self, entry: ScheduleEntry) -> Result<(), ScheduleEntry> {
        if entry.duration_ticks == 0 {
            return Err(entry);
        }
        self.entries.push(entry)?;
        self.major_frame_ticks = self.major_frame_ticks.saturating_add(entry.duration_ticks);
        Ok(())
    }

    /// Reset to the first slot. Call after adding all entries.
    pub fn start(&mut self) {
        self.current_slot = 0;
        self.ticks_remaining = self.entries.first().map_or(0, |e| e.duration_ticks);
        self.started = true;
    }

    pub fn current_partition(&self) -> Option<u8> {
        self.entries
            .get(self.current_slot)
            .map(|e| e.partition_index)
    }

    /// Advance by one tick. Returns `Some(partition_index)` on slot change.
    pub fn advance_tick(&mut self) -> Option<u8> {
        if !self.started || self.entries.is_empty() {
            return None;
        }
        self.ticks_remaining = self.ticks_remaining.saturating_sub(1);
        if self.ticks_remaining == 0 {
            self.current_slot += 1;
            if self.current_slot >= self.entries.len() {
                self.current_slot = 0;
            }
            self.ticks_remaining = self.entries[self.current_slot].duration_ticks;
            return Some(self.entries[self.current_slot].partition_index);
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn table(slots: &[(u8, u32)]) -> ScheduleTable<4> {
        let mut t = ScheduleTable::new();
        for &(p, d) in slots {
            t.add(ScheduleEntry::new(p, d)).unwrap();
        }
        t.start();
        t
    }

    #[test]
    fn empty_table_returns_none() {
        let mut t = table(&[]);
        assert!(t.current_partition().is_none());
        assert_eq!(t.advance_tick(), None);
    }

    #[test]
    fn single_partition_wraps() {
        let mut t = table(&[(0, 3)]);
        assert_eq!(t.current_partition(), Some(0));
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), Some(0));
    }

    #[test]
    fn two_partitions_alternate() {
        let mut t = table(&[(0, 2), (1, 3)]);
        assert_eq!(t.major_frame_ticks, 5);
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), Some(1));
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), Some(0));
    }

    #[test]
    fn major_frame_wraparound_sequence() {
        let mut t = table(&[(0, 1), (1, 1), (2, 1)]);
        for &exp in &[1u8, 2, 0, 1, 2, 0] {
            assert_eq!(t.advance_tick(), Some(exp));
        }
    }

    #[test]
    fn table_full_rejects() {
        let mut t: ScheduleTable<2> = ScheduleTable::new();
        assert!(t.add(ScheduleEntry::new(0, 10)).is_ok());
        assert!(t.add(ScheduleEntry::new(1, 10)).is_ok());
        assert!(t.add(ScheduleEntry::new(2, 10)).is_err());
        assert_eq!(t.major_frame_ticks, 20);
    }

    #[test]
    fn advance_tick_before_start_is_noop() {
        let mut t: ScheduleTable<4> = ScheduleTable::new();
        t.add(ScheduleEntry::new(0, 5)).unwrap();
        assert_eq!(t.advance_tick(), None);
        assert_eq!(t.advance_tick(), None);
        // After start, it should work normally
        t.start();
        assert_eq!(t.current_partition(), Some(0));
    }

    #[test]
    fn zero_duration_rejected() {
        let mut t: ScheduleTable<4> = ScheduleTable::new();
        assert!(t.add(ScheduleEntry::new(0, 0)).is_err());
        assert_eq!(t.major_frame_ticks, 0);
        assert!(t.entries.is_empty());
    }
}

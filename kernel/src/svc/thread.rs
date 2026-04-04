use crate::partition::PartitionTable;
use crate::svc::SvcError;
#[cfg(target_pointer_width = "32")]
use crate::thread::init_thread_stack_frame;
use crate::thread::{split_thread_stack, ThreadError};
use rtos_traits::ids::PartitionId;
use rtos_traits::thread::{ThreadControlBlock, ThreadState};

/// Handle the SYS_THREAD_CREATE syscall.
///
/// Arguments (from exception frame registers):
///   r1 = entry_point      — thread entry function address
///   r2 = priority          — thread priority (u8)
///   r3 = stack_size_hint   — requested stack size in bytes (0 = auto)
///
/// On success, returns the new `ThreadId` (as u32) in r0.
/// On failure, returns an `SvcError` code.
pub fn handle_thread_create<const N: usize>(
    partitions: &mut PartitionTable<N>,
    caller: PartitionId,
    entry_point: u32,
    priority: u8,
    _stack_size_hint: u32,
) -> u32 {
    // TODO: use stack_size_hint to support variable-size sub-stacks instead
    // of equal partitioning when split_thread_stack gains that capability.

    let pcb = match partitions.get_mut(caller.as_raw() as usize) {
        Some(pcb) => pcb,
        None => return SvcError::InvalidPartition.to_u32(),
    };

    let table = pcb.thread_table_mut();
    let thread_index = table.thread_count() as u32;
    let max_threads = table.capacity() as u32;

    // Check capacity before doing any work.
    if thread_index >= max_threads {
        return SvcError::OperationFailed.to_u32();
    }

    let (stack_base, stack_size) = pcb.stack_region();

    let (sub_base, sub_size) =
        match split_thread_stack(stack_base, stack_size, thread_index, max_threads) {
            Some(v) => v,
            None => return SvcError::OperationFailed.to_u32(),
        };

    // Initialize the context-switch frame on the sub-stack so the thread
    // can be scheduled (sets PC = entry_point, xPSR thumb bit, etc.).
    //
    // On 64-bit hosts (test-only), u32 addresses cannot be safely cast to
    // pointers, so we fall back to using the top-of-stack as SP. The real
    // Cortex-M target is always 32-bit.
    #[cfg(target_pointer_width = "32")]
    let sp = {
        let sub_words = (sub_size / 4) as usize;
        // SAFETY: sub_base..sub_base+sub_size lies within the partition's
        // stack region, which is owned exclusively by this partition and
        // backed by RAM. The slice is aligned to 4 bytes (split_thread_stack
        // guarantees 8-byte alignment) and does not alias — we hold &mut to
        // the partition table.
        let sub_stack = unsafe { core::slice::from_raw_parts_mut(sub_base as *mut u32, sub_words) };
        match init_thread_stack_frame(sub_stack, sub_base, entry_point, None) {
            Some(v) => v,
            None => return SvcError::OperationFailed.to_u32(),
        }
    };
    #[cfg(not(target_pointer_width = "32"))]
    let sp = match sub_base.checked_add(sub_size) {
        Some(v) => v,
        None => return SvcError::OperationFailed.to_u32(),
    };

    let tcb = ThreadControlBlock {
        stack_pointer: sp,
        id: rtos_traits::ids::ThreadId::new(0), // overwritten by add_thread
        state: ThreadState::Ready,
        priority,
        stack_base: sub_base,
        stack_size: sub_size,
        entry_point,
        r0_arg: 0,
    };

    match pcb.thread_table_mut().add_thread(tcb) {
        Ok(id) => id.as_raw() as u32,
        Err(ThreadError::TableFull) => SvcError::OperationFailed.to_u32(),
        Err(_) => SvcError::OperationFailed.to_u32(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::partition::{MpuRegion, PartitionControlBlock};

    fn pid(v: u32) -> PartitionId {
        PartitionId::new(v)
    }

    fn make_pt() -> PartitionTable<4> {
        let mut t = PartitionTable::new();
        let base = 0x2000_0000u32;
        let size = 4096u32;
        t.add(PartitionControlBlock::new(
            0,
            0x0800_0000u32,
            base,
            base + size,
            MpuRegion::new(base, size, 0),
        ))
        .unwrap();
        t.get_mut(0)
            .unwrap()
            .transition(crate::partition::PartitionState::Running)
            .unwrap();
        t
    }

    #[test]
    fn create_thread_returns_valid_id() {
        let mut pt = make_pt();
        let result = handle_thread_create(&mut pt, pid(0), 0x0800_1000, 5, 0);
        assert!(
            !SvcError::is_error(result),
            "expected success, got {result:#x}"
        );
        assert_eq!(result, 1);
    }

    #[test]
    fn created_thread_is_ready() {
        let mut pt = make_pt();
        let tid = handle_thread_create(&mut pt, pid(0), 0x0800_1000, 5, 0);
        assert!(!SvcError::is_error(tid));

        let tcb = pt
            .get(0)
            .unwrap()
            .thread_table()
            .get(rtos_traits::ids::ThreadId::new(tid as u8))
            .unwrap();
        assert_eq!(tcb.state, ThreadState::Ready);
        assert_eq!(tcb.priority, 5);
        assert_eq!(tcb.entry_point, 0x0800_1000);
    }

    #[test]
    fn create_fills_table_then_fails() {
        let mut pt = make_pt();
        for i in 0..3u32 {
            let r = handle_thread_create(&mut pt, pid(0), 0x0800_0000 + i * 4, i as u8, 0);
            assert!(!SvcError::is_error(r), "thread {i} creation failed: {r:#x}");
            assert_eq!(r, i + 1);
        }
        let r = handle_thread_create(&mut pt, pid(0), 0x0800_2000, 0, 0);
        assert!(SvcError::is_error(r), "expected error for full table");
    }

    #[test]
    fn invalid_partition_returns_error() {
        let mut pt = make_pt();
        let r = handle_thread_create(&mut pt, pid(99), 0x0800_0000, 0, 0);
        assert!(SvcError::is_error(r));
    }

    #[test]
    fn created_thread_has_valid_stack_region() {
        let mut pt = make_pt();
        let tid = handle_thread_create(&mut pt, pid(0), 0x0800_1000, 3, 0);
        assert!(!SvcError::is_error(tid));

        let pcb = pt.get(0).unwrap();
        let (part_base, part_size) = pcb.stack_region();
        let tcb = pcb
            .thread_table()
            .get(rtos_traits::ids::ThreadId::new(tid as u8))
            .unwrap();

        assert!(tcb.stack_base >= part_base);
        assert!(tcb.stack_base + tcb.stack_size <= part_base + part_size);
        assert_eq!(tcb.stack_base % 8, 0);
        assert_eq!(tcb.stack_size % 8, 0);
    }

    #[test]
    fn thread_count_increments() {
        let mut pt = make_pt();
        assert_eq!(pt.get(0).unwrap().thread_table().thread_count(), 1);

        handle_thread_create(&mut pt, pid(0), 0x0800_1000, 0, 0);
        assert_eq!(pt.get(0).unwrap().thread_table().thread_count(), 2);

        handle_thread_create(&mut pt, pid(0), 0x0800_2000, 0, 0);
        assert_eq!(pt.get(0).unwrap().thread_table().thread_count(), 3);
    }

    /// Verify that `init_thread_stack_frame` writes the entry point and xPSR
    /// to the correct offsets. Requires real backing memory, so gated to
    /// 32-bit targets (the actual Cortex-M target).
    #[cfg(target_pointer_width = "32")]
    #[cfg(not(feature = "fpu-context"))]
    #[test]
    fn stack_frame_is_initialized() {
        use crate::context::{SAVED_CONTEXT_WORDS, XPSR_THUMB};

        let mut stack = [0u32; 1024];
        let base = stack.as_mut_ptr() as u32;
        let size = (stack.len() * 4) as u32;
        let mut t = PartitionTable::new();
        t.add(PartitionControlBlock::new(
            0,
            0x0800_0000u32,
            base,
            base + size,
            MpuRegion::new(base, size, 0),
        ))
        .unwrap();
        t.get_mut(0)
            .unwrap()
            .transition(crate::partition::PartitionState::Running)
            .unwrap();

        let entry = 0x0800_1001u32;
        let tid = handle_thread_create(&mut t, pid(0), entry, 2, 0);
        assert!(!SvcError::is_error(tid));

        let tcb = t
            .get(0)
            .unwrap()
            .thread_table()
            .get(rtos_traits::ids::ThreadId::new(tid as u8))
            .unwrap();

        // SP must be decremented from the top of the sub-stack.
        let sub_top = tcb.stack_base + tcb.stack_size;
        assert!(
            tcb.stack_pointer < sub_top,
            "SP should be below sub-stack top after frame init"
        );

        // Read the initialized frame from the actual stack memory.
        let sp_offset = (tcb.stack_pointer - tcb.stack_base) as usize;
        let sub_stack = unsafe {
            core::slice::from_raw_parts(tcb.stack_base as *const u32, (tcb.stack_size / 4) as usize)
        };
        let frame_base = sp_offset / 4;
        let ef = frame_base + SAVED_CONTEXT_WORDS;
        assert_eq!(sub_stack[ef + 6], entry, "PC should be entry_point");
        assert_eq!(
            sub_stack[ef + 7],
            XPSR_THUMB,
            "xPSR should have thumb bit set"
        );
    }
}
